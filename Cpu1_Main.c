/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�TC264DA���İ�
����    д��chiusir
��E-mail��chiusir@163.com
������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
�������¡�2020��10��28��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://longqiu.taobao.com
------------------------------------------------
��dev.env.��AURIX Development Studio1.2.2�����ϰ汾
��Target �� TC264DA/TC264D
��Crystal�� 20.000Mhz
��SYS PLL�� 200MHz
________________________________________________________________
����iLLD_1_0_1_11_0�ײ����,

ʹ�����̵�ʱ�򣬽������û�пո��Ӣ��·����
����CIFΪTC264DA�����⣬�����Ĵ������TC264D
����Ĭ�ϳ�ʼ����EMEM��512K������û�ʹ��TC264D��ע�͵�EMEM_InitConfig()��ʼ��������
������\Libraries\iLLD\TC26B\Tricore\Cpu\CStart\IfxCpu_CStart0.c��164�����ҡ�
=================================================================
����������Ƶ��ַ��https://space.bilibili.com/95313236
=================================================================
����ͷ�ӿ�                  �������ۻ���OV7725ģ��
�� ���ݶ˿ڣ�P02.0-P02.7�ڣ���8λ��������ͷ�����ݶ˿ڣ�
�� ʱ�����أ��ⲿ�жϵ�0�飺P00_4��
�� ���źţ�    �ⲿ�жϵ�3�飺P15_1��
-----------------------------------------------------------------
�Ƽ�GPT12ģ�飬������ʵ��5·�����������������������ݴ�������������źŲɼ�������ѡ����·���ɣ�
P33_7, P33_6   ����TCĸ�������1
P02_8, P33_5   ����TCĸ�������2
P10_3, P10_1   ����TCĸ�������3
P20_3, P20_0   ����TCĸ�������4
-----------------------------------------------------------------
��е�ѹ�ɼ�ģ�������˷�ģ��
�Ƽ�ʹ��AN0-7������·ADC����������chirp�����źż���ų���е�ѹ�ɼ���
AN0-3          ����TC���ĸ���˷�ģ����ߵ��
-----------------------------------------------------------------
Ĭ�ϵ���ӿ�
ʹ��GTMģ�飬ATOM�ĸ�ͨ���ɲ���4*8��32·PWM�����Ҹ���Ƶ�ʺ�ռ�ձȿɵ����Ƽ�ʹ��ATOM0��0-7ͨ����
��һ��˫·ȫ������
P23_1         ����TCĸ��MOTOR1_P
P32_4         ����TCĸ��MOTOR1_N
P21_2         ����TCĸ��MOTOR2_P
P22_3         ����TCĸ��MOTOR2_N
�ڶ���˫·ȫ������
P21_4         ����TCĸ��MOTOR3_P
P21_3         ����TCĸ��MOTOR3_N
P20_8         ����TCĸ��MOTOR4_P
P21_5         ����TCĸ��MOTOR4_N
-----------------------------------------------------------------
Ĭ�϶���ӿ�
ʹ��GTMģ�飬ATOM�ĸ�ͨ���ɲ���4*8��32·PWM�����Ҹ���Ƶ�ʺ�ռ�ձȿɵ����Ƽ�ʹ��ATOM2��
P33_10        ����TCĸ����1
P33_13        ����TCĸ����2
-----------------------------------------------------------------
 Ĭ����Ļ��ʾ�ӿڣ��û�����ʹ��TFT����OLEDģ��
TFTSPI_CS     P20_14     ����TCĸ�� CS�ܽ� Ĭ�����ͣ����Բ���
TFTSPI_SCK    P20_11     ����TCĸ�� SPI SCK�ܽ�
TFTSPI_SDI    P20_10     ����TCĸ�� SPI MOSI�ܽ�
TFTSPI_DC     P20_12     ����TCĸ�� D/C�ܽ�
TFTSPI_RST    P20_13     ����TCĸ�� RESET�ܽ�
-----------------------------------------------------------------
OLED_CK       P20_14     ����TCĸ��OLED CK�ܽ�
OLED_DI       P20_11     ����TCĸ��OLED DI�ܽ�
OLED_RST      P20_10     ����TCĸ��OLED RST�ܽ�
OLED_DC       P20_12     ����TCĸ��OLED DC�ܽ�
OLED_CS       P20_13     ����TCĸ��OLED CS�ܽ� Ĭ�����ͣ����Բ���
----------------------------------------------------------------
Ĭ�ϰ����ӿ�
KEY0p         P22_0      ����TCĸ���ϰ���0
KEY1p         P22_1      ����TCĸ���ϰ���1
KEY2p         P22_2      ����TCĸ���ϰ���2
DSW0p         P33_9      ����TCĸ���ϲ��뿪��0
DSW1p         P33_11     ����TCĸ���ϲ��뿪��1
-----------------------------------------------------------------
Ĭ��LED�ӿ�
LED0p         P10_6      ����TCĸ����İ���LED0 ����
LED1p         P10_5      ����TCĸ����İ���LED1 ����
LED2p         P20_6      ����TCĸ����LED0
LED3p         P20_7      ����TCĸ����LED1
-----------------------------------------------------------------
��ʱ��
������CCU6ģ��  ÿ��ģ��������������ʱ��  ������ʱ���ж�
�Ƽ�ʹ��CCU6ģ�飬STM����ϵͳʱ�ӻ�����ʱ��
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <IfxCpu.h>
#include <IfxScuWdt.h>
#include <LQ_CAMERA.h>
#include <LQ_CCU6.h>
#include <LQ_GPIO_KEY.h>
#include <LQ_I2C_VL53.h>
#include <LQ_ImageProcess.h>
#include <LQ_MotorServo.h>
#include <LQ_SOFTI2C.h>
#include <LQ_TFT18.h>
#include <LQ_Inductor.h>
#include <Main.h>
#include <Platform_Types.h>
#include <stdint.h>
#include <stdio.h>
#include <LQ_ADC.h>

// ��ʱ�� 5ms��50ms��־λ
volatile uint8 cpu1Flage5ms = 0;
volatile uint8 cpu1Flage50ms = 0;

// �����ٶ�
volatile sint16 targetSpeed = 10;

struct pid_param_t
{
    float kp;    //P�ǵ÷�ֵ
    float ki;    //I
    float kd;    //D
    float i_max; //integrator_max
    float p_max; //integrator_max
    float d_max; //integrator_max

    float low_pass;

    float out_p;
    float out_i;
    float out_d;

    float error;
    float pre_error;
    float pre_pre_error;
} ;
struct CORNER
{
    int corner_flag;
    int corner_row;
    int corner_col;
    int corner_angle;
    int corner_icount;
};
extern struct CORNER Lc, Rc, uLc, uRc;
// ���ϱ�־λ
volatile uint8 evadibleFlage = 0;
extern  unsigned char output[LCDH][LCDW];
extern  unsigned char ooutput[LCDH][LCDW];
extern struct pid_param_t pama;
extern float KP_motor;extern float KI_motor;extern float KD_motor;
extern int t_state;
extern int RIGHT_EDGE_type;
extern int LEFT_EDGE_type;
extern float K_MD;
extern int time;
extern int inelement;
extern int ti;
int e1max=0;
int e2max=0;
int core1_main (void)
{
    delayms(300);

    // ����CPU���ж�
    IfxCpu_enableInterrupts();

    // �رտ��Ź�
    IfxScuWdt_disableCpuWatchdog (IfxScuWdt_getCpuWatchdogPassword ());

    // �ȴ�CPU0 ��ʼ�����
    while(!IfxCpu_acquireMutex(&mutexCpu0InitIsOk));
    //��м���ص�ѹ ADC�ɼ���ʼ��
       ADC_InitConfig(ADC7, 80000);//��ʼ��   ���ʹ������ĸ��  ����ѹ��ĵ�ص�ѹ��������Կ�ĸ��ԭ��ͼ
    // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
    mutexCpu0TFTIsOk=1;         // CPU1�� 0ռ��/1�ͷ� TFT
    /* ������������������ʼ�� */
       MotorInit();    // ���
       ServoInit();    // ���
       EncInit();  // ������


    // ��ʱ����ʼ��,ԭʼ�жϺ�����CCU6.C�� */
    CCU6_InitConfig(CCU61, CCU6_Channel0, 5000);// 5ms
    //Test_CAMERA();         //PASS,����������������ͷ������Ļ����ʾ  LQ_CAMERA.h ��ѡ����Ļ
    TFTSPI_Init(0);               // TFT1.8��ʼ��0:����  1������
    TFTSPI_CLS(u16BLACK);         // ����
    unsigned char oot[LCDH][LCDW];
    UART_InitConfig(UART0_RX_P14_1, UART0_TX_P14_0, 115200);
    //int lccr,rccr,lcco,rcco,la,ra;
    struct CORNER dLc,dRc,duLc,duRc;
    //while(1)

    while(1)//��ѭ��
    {
        // �м�CPU0,CPU1...������ͬʱ������Ļ��ʾ�������ͻ����ʾ
        char txt[30];
        int i,j;
        dLc=Lc;
        dRc=Rc;
        duLc=uLc;
        duRc=uRc;
        for (i=0;i<LCDH;i++)
        {
            for (j=0;j<LCDW;j++)
                oot[i][j]=output[i][j];
        }
        for(i=1;i<10;i++)
        {
            if(dLc.corner_flag==1&&dLc.corner_row+i<59&&dLc.corner_col+i<93&&dLc.corner_row-i>0&&dLc.corner_col-i>0)
            {
                oot[dLc.corner_row+i][dLc.corner_col-i]=1-oot[dLc.corner_row+i][dLc.corner_col-i];
                oot[dLc.corner_row+i][dLc.corner_col+i]=1-oot[dLc.corner_row+i][dLc.corner_col+i];
                oot[dLc.corner_row-i][dLc.corner_col-i]=1-oot[dLc.corner_row-i][dLc.corner_col-i];
                oot[dLc.corner_row-i][dLc.corner_col+i]=1-oot[dLc.corner_row-i][dLc.corner_col+i];
            }
            if(dRc.corner_flag==1&&dRc.corner_row+i<59&&dRc.corner_col+i<93&&dRc.corner_row-i>0&&dRc.corner_col-i>0)
            {
                oot[dRc.corner_row+i][dRc.corner_col-i]=1-oot[dRc.corner_row+i][dRc.corner_col-i];
                oot[dRc.corner_row+i][dRc.corner_col+i]=1-oot[dRc.corner_row+i][dRc.corner_col+i];
                oot[dRc.corner_row-i][dRc.corner_col-i]=1-oot[dRc.corner_row-i][dRc.corner_col-i];
                oot[dRc.corner_row-i][dRc.corner_col+i]=1-oot[dRc.corner_row-i][dRc.corner_col+i];
            }
            if(duLc.corner_flag==1&&duLc.corner_row+i<59&&duLc.corner_col+i<93&&duLc.corner_row-i>0&&duLc.corner_col-i>0)
            {
                oot[duLc.corner_row+i][duLc.corner_col-i]=1-oot[duLc.corner_row+i][duLc.corner_col-i];
                oot[duLc.corner_row+i][duLc.corner_col+i]=1-oot[duLc.corner_row+i][duLc.corner_col+i];
                oot[duLc.corner_row-i][duLc.corner_col-i]=1-oot[duLc.corner_row-i][duLc.corner_col-i];
                oot[duLc.corner_row-i][duLc.corner_col+i]=1-oot[duLc.corner_row-i][duLc.corner_col+i];
            }
            if(duRc.corner_flag==1&&duRc.corner_row+i<59&&duRc.corner_col+i<93&&duRc.corner_row-i>0&&duRc.corner_col-i>0)
            {
                oot[duRc.corner_row+i][duRc.corner_col-i]=1-oot[duRc.corner_row+i][duRc.corner_col-i];
                oot[duRc.corner_row+i][duRc.corner_col+i]=1-oot[duRc.corner_row+i][duRc.corner_col+i];
                oot[duRc.corner_row-i][duRc.corner_col-i]=1-oot[duRc.corner_row-i][duRc.corner_col-i];
                oot[duRc.corner_row-i][duRc.corner_col+i]=1-oot[duRc.corner_row-i][duRc.corner_col+i];
            }

        }
        //TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *)oot);




        for (i=0;i<LCDH;i++)
        {
            for (j=0;j<LCDW;j++)
                oot[i][j]=ooutput[i][j];
        }


        //TFTSPI_BinRoad(0, LCDH, LCDH, LCDW, (unsigned char *)oot);


        sprintf(txt, "state: %02d \n", t_state);
        //UART_PutStr(UART0,txt);
        TFTSPI_P8X16Str(0, 5, txt, u16WHITE, u16BLACK);

        sprintf(txt, "pid1:%.01f %.01f %.01f \n", pama.kp,pama.ki,pama.kd);
        UART_PutStr(UART0,txt);
        TFTSPI_P8X16Str(0, 6, txt, u16WHITE, u16BLACK);
        sprintf(txt, "pid2:%.01f %.01f %.01f \n", KP_motor,KI_motor,KD_motor);
        UART_PutStr(UART0,txt);
        TFTSPI_P8X16Str(0, 7, txt, u16WHITE, u16BLACK);


        //delayms(500);
        sprintf(txt, "E1=%05d ", ECPULSE1);
        TFTSPI_P8X16Str(0, 0, txt, u16WHITE, u16BLACK);
        if (ECPULSE1>e1max)
            e1max=ECPULSE1;
        sprintf(txt, "E1max=%05d ", e1max);
        TFTSPI_P8X16Str(0, 1, txt, u16WHITE, u16BLACK);
        UART_PutStr(UART0,txt);
        sprintf(txt, "E2=%05d ", ECPULSE2);
        TFTSPI_P8X16Str(0, 2, txt, u16WHITE, u16BLACK);
        if (ECPULSE2>e2max)
            e2max=ECPULSE2;
        sprintf(txt, "E2max=%05d ", e2max);
        TFTSPI_P8X16Str(0, 3, txt, u16WHITE, u16BLACK);
        //UART_PutStr(UART0,txt);
        //delayms(500);
        int lt=LEFT_EDGE_type;
        sprintf(txt, "Lt=%02d ", lt);
        TFTSPI_P8X16Str(12, 0, txt, u16WHITE, u16BLACK);
        UART_PutStr(UART0,txt);
        int rt=RIGHT_EDGE_type;
        sprintf(txt, "Rt=%02d ", rt);
        TFTSPI_P8X16Str(12, 1, txt, u16WHITE, u16BLACK);
        UART_PutStr(UART0,txt);
        int t=time/1000;
        sprintf(txt, "t=%03d ", t);

        TFTSPI_P8X16Str(12, 2, txt, u16WHITE, u16BLACK);
        //int t;
        t=ti;
        sprintf(txt, "ti=%03d ", t);
        TFTSPI_P8X16Str(12, 4, txt, u16WHITE, u16BLACK);
        int el=inelement;
        sprintf(txt, "el=%d ", el);
        TFTSPI_P8X16Str(12, 3, txt, u16WHITE, u16BLACK);
        //UART_PutStr(UART0,txt);
        //sprintf(txt, "Rp= %02d,%02d,%02d ", rccr,rcco,ra);
        //TFTSPI_P8X16Str(0, 7, txt, u16WHITE, u16BLACK);
        //UART_PutStr(UART0,txt);

        //delayms(500);
        if(t_state==0)
        {
            if(KEY_Read(KEY0)==0)//����KEY0����
            {
                t_state=10;//pid���
            }
            if(KEY_Read(KEY1)==0)//����KEY1����
            {
                t_state=20;//pid���
            }
            if(KEY_Read(KEY2)==0)//����KEY2����
            {
                //R=0;
                //R=0;
                ;
            }
        }
        else if(t_state==10)
        {
            if(KEY_Read(KEY0)==0)//����KEY0����
            {
                t_state=11;//p
            }
            if(KEY_Read(KEY1)==0)//����KEY1����
            {
                t_state=12;//i
            }
            if(KEY_Read(KEY2)==0)//����KEY2����
            {
                t_state=13;//d
            }
        }
        else if(t_state==20)
        {
            if(KEY_Read(KEY0)==0)//����KEY0����
            {
                t_state=21;//p
            }
            if(KEY_Read(KEY1)==0)//����KEY1����
            {
                t_state=22;//i
            }
            if(KEY_Read(KEY2)==0)//����KEY2����
            {
                t_state=23;//d
            }
        }
        else if(t_state==11)
                {
                    if(KEY_Read(KEY0)==0)//����KEY0����
                    {
                        pama.kp+=1;//p
                    }
                    if(KEY_Read(KEY1)==0)//����KEY1����
                    {
                        pama.kp-=1;//i
                    }
                    if(KEY_Read(KEY2)==0)//����KEY2����
                    {
                        t_state=0;//d
                        e1max=0;
                        e2max=0;
                    }
                }
        else if(t_state==12)
                {
                    if(KEY_Read(KEY0)==0)//����KEY0����
                    {
                        pama.ki+=0.1;//p
                    }
                    if(KEY_Read(KEY1)==0)//����KEY1����
                    {
                        pama.ki-=0.1;//i
                    }
                    if(KEY_Read(KEY2)==0)//����KEY2����
                    {
                        t_state=0;
                        e1max=0;//d
                        e2max=0;
                    }
                }
        else if(t_state==13)
                {
                    if(KEY_Read(KEY0)==0)//����KEY0����
                    {
                        pama.kd+=0.1;//p
                    }
                    if(KEY_Read(KEY1)==0)//����KEY1����
                    {
                        pama.kd-=0.1;//i
                    }
                    if(KEY_Read(KEY2)==0)//����KEY2����
                    {
                        t_state=0;//d
                        e1max=0;
                        e2max=0;
                    }
                }
        else if(t_state==21)
                {
                    if(KEY_Read(KEY0)==0)//����KEY0����
                    {
                        KP_motor+=0.1;//p
                    }
                    if(KEY_Read(KEY1)==0)//����KEY1����
                    {
                        KP_motor-=0.1;//i
                    }
                    if(KEY_Read(KEY2)==0)//����KEY2����
                    {
                        t_state=0;//d
                        e1max=0;
                        e2max=0;
                    }
                }
        else if(t_state==22)
                {
                    if(KEY_Read(KEY0)==0)//����KEY0����
                    {
                        KI_motor+=0.1;//i+
                    }
                    if(KEY_Read(KEY1)==0)//����KEY1����
                    {
                        KI_motor-=0.1;//i-
                    }
                    if(KEY_Read(KEY2)==0)//����KEY2����
                    {
                        t_state=0;
                        e1max=0;
                        e2max=0;
                    }
                }
        else if(t_state==23)
                {
                    if(KEY_Read(KEY0)==0)//����KEY0����
                    {
                        KD_motor+=0.1;//d+
                    }
                    if(KEY_Read(KEY1)==0)//����KEY1����
                    {
                        KD_motor-=0.1;//d-
                    }
                    if(KEY_Read(KEY2)==0)//����KEY2����
                    {
                        t_state=0;
                        e1max=0;
                        e2max=0;
                    }
                }
        if(KEY_Read(KEY0)& KEY_Read(KEY1)& KEY_Read(KEY2)==0)
            delayms(200);              //��ʱ�ȴ�

        // CameraCar(); // ����ͷ���ֳ�˫������ƣ��򵥵ķֶα��������㷨
    }
}
