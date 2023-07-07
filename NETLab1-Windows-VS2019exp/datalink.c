#include <stdio.h>
#include <string.h>

#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER 4000                     //数据帧定时器时长
#define ACK_TIMER 400                       //ACK帧定时器时长
#define MAX_SEQ 31                          //窗口最大序列号
#define NR_BUFS ((MAX_SEQ + 1) / 2)         //窗口大小
#define inc(k) k = (k + 1) % (MAX_SEQ + 1)  //k的递增，模(MAX_SEQ + 1)

typedef enum { false, true } bool;

//定义包数据结构
typedef unsigned char Packet[PKT_LEN];

//定义帧数据结构
typedef struct {
    unsigned char kind;     //帧的类型
    unsigned char ack;      //ACK号
    unsigned char seq;      //帧序列号
    Packet data;            //数据段
    unsigned int  padding;  
}Frame;


static bool no_nak = true;      //是否发送过NAK帧
static int phl_ready = false;   //物理层是否准备好

//判断序列号b是否在接收窗口之中，窗口上下边界分别为c和a
static bool between(unsigned int a, unsigned int b, unsigned int c);

//将帧送往物理层
static void put_frame(unsigned char* frame, int len);

//发送帧函数，根据帧类型执行不同操作
static void send_data(unsigned char fk, unsigned char frame_nr, unsigned char frame_expected, Packet buffer[]);

int main(int argc, char** argv)
{
    unsigned char ack_expected = 0;         //发送窗口下界
    unsigned char next_frame_to_send = 0;   //发送窗口上界
    unsigned char too_far = NR_BUFS;        //接收窗口上界
    unsigned char frame_expected = 0;       //接收窗口下界
    unsigned char nbuffered = 0;            //计数，发送了几个帧
    Packet out_buf[NR_BUFS];                //输出流缓冲区
    Packet in_buf[NR_BUFS];                 //输入流缓冲区
    bool arrived[NR_BUFS] = { false };      //表示输入流缓冲区是否空闲

    int event;      //系统事件
    int arg;        //需要重传的帧的序列号
    Frame f;        //暂存帧
    int len = 0;    //接收的帧的长度

    //初始化
    enable_network_layer();
    protocol_init(argc, argv);
    lprintf("Designed by Zheng Yuheng, build: " __DATE__"  "__TIME__"\n");

    for (;;) {
        //获取系统事件。如果是数据帧超时需要重传，arg将存储需要重传的数据帧序列号。
        event = wait_for_event(&arg);   

        switch (event) {
        case NETWORK_LAYER_READY:   //网络层有数据发送
            nbuffered++;            //输出流缓冲区内的帧+1
            get_packet(out_buf[next_frame_to_send % NR_BUFS]);                  //从网络层获取帧
            send_data(FRAME_DATA, next_frame_to_send, frame_expected, out_buf); //发送帧
            inc(next_frame_to_send);    //更新发送窗口上界
            break;

        case PHYSICAL_LAYER_READY:  //物理层可传送数据
            phl_ready = true;
            break;

        case FRAME_RECEIVED:        //收到帧
            len = recv_frame((unsigned char*)&f, sizeof f);     //从物理层获取帧。帧长度存储在len。
            if (len < 5 || crc32((unsigned char*)&f, len) != 0) //检查帧长度是否正常，且能否通过CRC校验
            {
                dbg_event("**** Receiver Error, Bad CRC Checksum\n");
                if (no_nak) //之前没发送过NAK帧才能发送NAK帧
                    send_data(FRAME_NAK, 0, frame_expected, out_buf);   //发送NAK帧要求重传
                break;
            }

            if (f.kind == FRAME_ACK)        //收到ACK帧
                dbg_frame("Recv ACK  %d\n", f.ack);
            else if (f.kind == FRAME_DATA)  //收到数据帧
            {
                if (f.seq != frame_expected && no_nak)  //如果不等于接收窗口下界且没有发送过NAK帧，发送NAK帧要求重传
                    send_data(FRAME_NAK, 0, frame_expected, out_buf);
                else
                    start_ack_timer(ACK_TIMER);         //如没有反向流量，ACK无法捎带。定时结束后发送一个单独的ACK帧

                /*如果收到的帧序号在接收窗口内，且对应的输入缓冲区为空*/
                if (between(frame_expected, f.seq, too_far) && (arrived[f.seq % NR_BUFS] == false))
                {
                    dbg_frame("Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short*)&(f.data));
                    arrived[f.seq % NR_BUFS] = true;                    //标记缓冲区为满
                    memcpy(in_buf[f.seq % NR_BUFS], f.data, PKT_LEN);   //将数据写入缓冲区
                    while (arrived[frame_expected % NR_BUFS])           //将从接收窗口下界开始，所有不为空的缓冲区数据送往网络层
                    {
                        put_packet(in_buf[frame_expected % NR_BUFS], len - 7);  //将数据送往网络层
                        no_nak = true;
                        arrived[frame_expected % NR_BUFS] = false;      //标记缓冲区为空
                        inc(frame_expected);        //更新接收窗口下界
                        inc(too_far);               //更新接收窗口上界
                        start_ack_timer(ACK_TIMER); //如没有反向流量，ACK无法捎带。定时结束后发送一个单独的ACK帧
                    }
                }
            }
            else if (f.kind == FRAME_NAK && between(ack_expected, (f.ack + 1) % (MAX_SEQ + 1), next_frame_to_send)) //收到NAK帧，NAK帧的序号在发送窗口内
            {
                dbg_frame("Recv NAK DATA %d\n", f.ack);
                send_data(FRAME_DATA, (f.ack + 1) % (MAX_SEQ + 1), frame_expected, out_buf);    //重传数据帧
            }

            while (between(ack_expected, f.ack, next_frame_to_send))    //检查ACK序号是否在发送窗口内
            {
                nbuffered--;                        //输出流缓冲区内的帧-1
                stop_timer(ack_expected % NR_BUFS); //收到ACK，停止数据帧重传定时器
                inc(ack_expected);                  //更新发送窗口下界
            }
            break;

        case DATA_TIMEOUT:      //数据超时，选择重传
            dbg_event("---- DATA %d timeout\n", arg);
            if (!between(ack_expected, arg, next_frame_to_send))    //arg的取值范围在0-15，可能需要加NR_BUFS得到在发送窗口中正确的序号
                arg += NR_BUFS;
            send_data(FRAME_DATA, arg, frame_expected, out_buf);    //重新发送数据
            break;

        case ACK_TIMEOUT:       //ACK超时
            dbg_event("---- DATA %d timeout\n", arg);
            send_data(FRAME_ACK, 0, frame_expected, out_buf);       //单独发送一个ACK帧
            break;

        default:
            break;
        }

        //缓冲区没满，继续发送
        if (nbuffered < NR_BUFS && phl_ready)   
            enable_network_layer();
        //缓冲区满，停止发送
        else
            disable_network_layer();
    }
}

//判断序列号b是否在接收窗口之中，窗口上下边界分别为c和a
static bool between(unsigned int a, unsigned int b, unsigned int c)
{
    return ((a <= b) && (b < c)) || ((c < a) && (b >= a)) || ((c < a) && (b < c));
}

//将帧送往物理层
static void put_frame(unsigned char* frame, int len)
{
    *(unsigned int*)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = false;
}

//发送帧函数，根据帧类型执行不同操作
static void send_data(unsigned char fk, unsigned char frame_nr, unsigned char frame_expected, Packet buffer[])
{
    Frame s;

    s.kind = fk;        //帧类型
    s.seq = frame_nr;   //帧序号                                
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1); //累计ACK，序号表示最后一个正确收到的帧
    if (fk == FRAME_DATA)       //发送数据帧
    {
        memcpy(s.data, buffer[frame_nr % NR_BUFS], PKT_LEN);    //将缓冲区缓冲区复制到帧s中
        dbg_frame("Send DATA %d %d, ID %d\n", s.seq, s.ack, *(short*)&(s.data));    
        put_frame((unsigned char*)&s, 3 + PKT_LEN);             //交往物理层
        start_timer(frame_nr % NR_BUFS, DATA_TIMER);            //启动数据帧计时器
    }
    else if (fk == FRAME_ACK)   //发送ACK帧
    {
        dbg_frame("Send ACK  %d\n", s.ack);
        put_frame((unsigned char*)&s, 2);   //交往物理层
    }   
    else if (fk == FRAME_NAK)   //发送NAK帧
    {
        no_nak = false;
        put_frame((unsigned char*)&s, 2);   //交往物理层
    }

    stop_ack_timer();   //有反向流量，停止ACK计时器
}