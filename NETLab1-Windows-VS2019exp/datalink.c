#include <stdio.h>
#include <string.h>

#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER 4000                     //����֡��ʱ��ʱ��
#define ACK_TIMER 400                       //ACK֡��ʱ��ʱ��
#define MAX_SEQ 31                          //����������к�
#define NR_BUFS ((MAX_SEQ + 1) / 2)         //���ڴ�С
#define inc(k) k = (k + 1) % (MAX_SEQ + 1)  //k�ĵ�����ģ(MAX_SEQ + 1)

typedef enum { false, true } bool;

//��������ݽṹ
typedef unsigned char Packet[PKT_LEN];

//����֡���ݽṹ
typedef struct {
    unsigned char kind;     //֡������
    unsigned char ack;      //ACK��
    unsigned char seq;      //֡���к�
    Packet data;            //���ݶ�
    unsigned int  padding;  
}Frame;


static bool no_nak = true;      //�Ƿ��͹�NAK֡
static int phl_ready = false;   //������Ƿ�׼����

//�ж����к�b�Ƿ��ڽ��մ���֮�У��������±߽�ֱ�Ϊc��a
static bool between(unsigned int a, unsigned int b, unsigned int c);

//��֡���������
static void put_frame(unsigned char* frame, int len);

//����֡����������֡����ִ�в�ͬ����
static void send_data(unsigned char fk, unsigned char frame_nr, unsigned char frame_expected, Packet buffer[]);

int main(int argc, char** argv)
{
    unsigned char ack_expected = 0;         //���ʹ����½�
    unsigned char next_frame_to_send = 0;   //���ʹ����Ͻ�
    unsigned char too_far = NR_BUFS;        //���մ����Ͻ�
    unsigned char frame_expected = 0;       //���մ����½�
    unsigned char nbuffered = 0;            //�����������˼���֡
    Packet out_buf[NR_BUFS];                //�����������
    Packet in_buf[NR_BUFS];                 //������������
    bool arrived[NR_BUFS] = { false };      //��ʾ�������������Ƿ����

    int event;      //ϵͳ�¼�
    int arg;        //��Ҫ�ش���֡�����к�
    Frame f;        //�ݴ�֡
    int len = 0;    //���յ�֡�ĳ���

    //��ʼ��
    enable_network_layer();
    protocol_init(argc, argv);
    lprintf("Designed by Zheng Yuheng, build: " __DATE__"  "__TIME__"\n");

    for (;;) {
        //��ȡϵͳ�¼������������֡��ʱ��Ҫ�ش���arg���洢��Ҫ�ش�������֡���кš�
        event = wait_for_event(&arg);   

        switch (event) {
        case NETWORK_LAYER_READY:   //����������ݷ���
            nbuffered++;            //������������ڵ�֡+1
            get_packet(out_buf[next_frame_to_send % NR_BUFS]);                  //��������ȡ֡
            send_data(FRAME_DATA, next_frame_to_send, frame_expected, out_buf); //����֡
            inc(next_frame_to_send);    //���·��ʹ����Ͻ�
            break;

        case PHYSICAL_LAYER_READY:  //�����ɴ�������
            phl_ready = true;
            break;

        case FRAME_RECEIVED:        //�յ�֡
            len = recv_frame((unsigned char*)&f, sizeof f);     //��������ȡ֡��֡���ȴ洢��len��
            if (len < 5 || crc32((unsigned char*)&f, len) != 0) //���֡�����Ƿ����������ܷ�ͨ��CRCУ��
            {
                dbg_event("**** Receiver Error, Bad CRC Checksum\n");
                if (no_nak) //֮ǰû���͹�NAK֡���ܷ���NAK֡
                    send_data(FRAME_NAK, 0, frame_expected, out_buf);   //����NAK֡Ҫ���ش�
                break;
            }

            if (f.kind == FRAME_ACK)        //�յ�ACK֡
                dbg_frame("Recv ACK  %d\n", f.ack);
            else if (f.kind == FRAME_DATA)  //�յ�����֡
            {
                if (f.seq != frame_expected && no_nak)  //��������ڽ��մ����½���û�з��͹�NAK֡������NAK֡Ҫ���ش�
                    send_data(FRAME_NAK, 0, frame_expected, out_buf);
                else
                    start_ack_timer(ACK_TIMER);         //��û�з���������ACK�޷��Ӵ�����ʱ��������һ��������ACK֡

                /*����յ���֡����ڽ��մ����ڣ��Ҷ�Ӧ�����뻺����Ϊ��*/
                if (between(frame_expected, f.seq, too_far) && (arrived[f.seq % NR_BUFS] == false))
                {
                    dbg_frame("Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short*)&(f.data));
                    arrived[f.seq % NR_BUFS] = true;                    //��ǻ�����Ϊ��
                    memcpy(in_buf[f.seq % NR_BUFS], f.data, PKT_LEN);   //������д�뻺����
                    while (arrived[frame_expected % NR_BUFS])           //���ӽ��մ����½翪ʼ�����в�Ϊ�յĻ������������������
                    {
                        put_packet(in_buf[frame_expected % NR_BUFS], len - 7);  //���������������
                        no_nak = true;
                        arrived[frame_expected % NR_BUFS] = false;      //��ǻ�����Ϊ��
                        inc(frame_expected);        //���½��մ����½�
                        inc(too_far);               //���½��մ����Ͻ�
                        start_ack_timer(ACK_TIMER); //��û�з���������ACK�޷��Ӵ�����ʱ��������һ��������ACK֡
                    }
                }
            }
            else if (f.kind == FRAME_NAK && between(ack_expected, (f.ack + 1) % (MAX_SEQ + 1), next_frame_to_send)) //�յ�NAK֡��NAK֡������ڷ��ʹ�����
            {
                dbg_frame("Recv NAK DATA %d\n", f.ack);
                send_data(FRAME_DATA, (f.ack + 1) % (MAX_SEQ + 1), frame_expected, out_buf);    //�ش�����֡
            }

            while (between(ack_expected, f.ack, next_frame_to_send))    //���ACK����Ƿ��ڷ��ʹ�����
            {
                nbuffered--;                        //������������ڵ�֡-1
                stop_timer(ack_expected % NR_BUFS); //�յ�ACK��ֹͣ����֡�ش���ʱ��
                inc(ack_expected);                  //���·��ʹ����½�
            }
            break;

        case DATA_TIMEOUT:      //���ݳ�ʱ��ѡ���ش�
            dbg_event("---- DATA %d timeout\n", arg);
            if (!between(ack_expected, arg, next_frame_to_send))    //arg��ȡֵ��Χ��0-15��������Ҫ��NR_BUFS�õ��ڷ��ʹ�������ȷ�����
                arg += NR_BUFS;
            send_data(FRAME_DATA, arg, frame_expected, out_buf);    //���·�������
            break;

        case ACK_TIMEOUT:       //ACK��ʱ
            dbg_event("---- DATA %d timeout\n", arg);
            send_data(FRAME_ACK, 0, frame_expected, out_buf);       //��������һ��ACK֡
            break;

        default:
            break;
        }

        //������û������������
        if (nbuffered < NR_BUFS && phl_ready)   
            enable_network_layer();
        //����������ֹͣ����
        else
            disable_network_layer();
    }
}

//�ж����к�b�Ƿ��ڽ��մ���֮�У��������±߽�ֱ�Ϊc��a
static bool between(unsigned int a, unsigned int b, unsigned int c)
{
    return ((a <= b) && (b < c)) || ((c < a) && (b >= a)) || ((c < a) && (b < c));
}

//��֡���������
static void put_frame(unsigned char* frame, int len)
{
    *(unsigned int*)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = false;
}

//����֡����������֡����ִ�в�ͬ����
static void send_data(unsigned char fk, unsigned char frame_nr, unsigned char frame_expected, Packet buffer[])
{
    Frame s;

    s.kind = fk;        //֡����
    s.seq = frame_nr;   //֡���                                
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1); //�ۼ�ACK����ű�ʾ���һ����ȷ�յ���֡
    if (fk == FRAME_DATA)       //��������֡
    {
        memcpy(s.data, buffer[frame_nr % NR_BUFS], PKT_LEN);    //�����������������Ƶ�֡s��
        dbg_frame("Send DATA %d %d, ID %d\n", s.seq, s.ack, *(short*)&(s.data));    
        put_frame((unsigned char*)&s, 3 + PKT_LEN);             //���������
        start_timer(frame_nr % NR_BUFS, DATA_TIMER);            //��������֡��ʱ��
    }
    else if (fk == FRAME_ACK)   //����ACK֡
    {
        dbg_frame("Send ACK  %d\n", s.ack);
        put_frame((unsigned char*)&s, 2);   //���������
    }   
    else if (fk == FRAME_NAK)   //����NAK֡
    {
        no_nak = false;
        put_frame((unsigned char*)&s, 2);   //���������
    }

    stop_ack_timer();   //�з���������ֹͣACK��ʱ��
}