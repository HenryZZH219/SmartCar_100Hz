/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技TC264DA核心板
【编    写】chiusir
【E-mail】chiusir@163.com
【软件版本】V1.1 版权所有，单位使用请先联系授权
【最后更新】2020年10月28日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【dev.env.】AURIX Development Studio1.2.2及以上版本
【Target 】 TC264DA/TC264D
【Crystal】 20.000Mhz
【SYS PLL】 200MHz
________________________________________________________________
基于iLLD_1_0_1_11_0底层程序,

使用例程的时候，建议采用没有空格的英文路径，
除了CIF为TC264DA独有外，其它的代码兼容TC264D
本库默认初始化了EMEM：512K，如果用户使用TC264D，注释掉EMEM_InitConfig()初始化函数。
工程下\Libraries\iLLD\TC26B\Tricore\Cpu\CStart\IfxCpu_CStart0.c第164行左右。
=================================================================
程序配套视频地址：https://space.bilibili.com/95313236
=================================================================
摄像头接口                  龙邱神眼或者OV7725模块
● 数据端口：P02.0-P02.7口，共8位，接摄像头的数据端口；
● 时钟像素：外部中断第0组：P00_4；
● 场信号：    外部中断第3组：P15_1；
-----------------------------------------------------------------
推荐GPT12模块，共可以实现5路正交解码增量编码器（兼容带方向编码器）信号采集，任意选择四路即可；
P33_7, P33_6   龙邱TC母板编码器1
P02_8, P33_5   龙邱TC母板编码器2
P10_3, P10_1   龙邱TC母板编码器3
P20_3, P20_0   龙邱TC母板编码器4
-----------------------------------------------------------------
电感电压采集模块或者麦克风模块
推荐使用AN0-7，共八路ADC，可以满足chirp声音信号及电磁车电感电压采集；
AN0-3          龙邱TC接四个麦克风模块或者电感
-----------------------------------------------------------------
默认电机接口
使用GTM模块，ATOM四个通道可产生4*8共32路PWM，而且各自频率和占空比可调，推荐使用ATOM0的0-7通道；
第一组双路全桥驱动
P23_1         龙邱TC母板MOTOR1_P
P32_4         龙邱TC母板MOTOR1_N
P21_2         龙邱TC母板MOTOR2_P
P22_3         龙邱TC母板MOTOR2_N
第二组双路全桥驱动
P21_4         龙邱TC母板MOTOR3_P
P21_3         龙邱TC母板MOTOR3_N
P20_8         龙邱TC母板MOTOR4_P
P21_5         龙邱TC母板MOTOR4_N
-----------------------------------------------------------------
默认舵机接口
使用GTM模块，ATOM四个通道可产生4*8共32路PWM，而且各自频率和占空比可调，推荐使用ATOM2；
P33_10        龙邱TC母板舵机1
P33_13        龙邱TC母板舵机2
-----------------------------------------------------------------
 默认屏幕显示接口，用户可以使用TFT或者OLED模块
TFTSPI_CS     P20_14     龙邱TC母板 CS管脚 默认拉低，可以不用
TFTSPI_SCK    P20_11     龙邱TC母板 SPI SCK管脚
TFTSPI_SDI    P20_10     龙邱TC母板 SPI MOSI管脚
TFTSPI_DC     P20_12     龙邱TC母板 D/C管脚
TFTSPI_RST    P20_13     龙邱TC母板 RESET管脚
-----------------------------------------------------------------
OLED_CK       P20_14     龙邱TC母板OLED CK管脚
OLED_DI       P20_11     龙邱TC母板OLED DI管脚
OLED_RST      P20_10     龙邱TC母板OLED RST管脚
OLED_DC       P20_12     龙邱TC母板OLED DC管脚
OLED_CS       P20_13     龙邱TC母板OLED CS管脚 默认拉低，可以不用
----------------------------------------------------------------
默认按键接口
KEY0p         P22_0      龙邱TC母板上按键0
KEY1p         P22_1      龙邱TC母板上按键1
KEY2p         P22_2      龙邱TC母板上按键2
DSW0p         P33_9      龙邱TC母板上拨码开关0
DSW1p         P33_11     龙邱TC母板上拨码开关1
-----------------------------------------------------------------
默认LED接口
LED0p         P10_6      龙邱TC母板核心板上LED0 翠绿
LED1p         P10_5      龙邱TC母板核心板上LED1 蓝灯
LED2p         P20_6      龙邱TC母板上LED0
LED3p         P20_7      龙邱TC母板上LED1
-----------------------------------------------------------------
定时器
有两个CCU6模块  每个模块有两个独立定时器  触发定时器中断
推荐使用CCU6模块，STM用作系统时钟或者延时；
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


// 定时器 5ms和50ms标志位
volatile uint8 cpu1Flage5ms = 0;
volatile uint8 cpu1Flage50ms = 0;

// 期望速度
volatile sint16 targetSpeed = 10;

struct pid_param_t
{
    float kp;    //P记得幅值
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
// 避障标志位
volatile uint8 evadibleFlage = 0;
extern  unsigned char output[LCDH][LCDW];
extern  unsigned char ooutput[LCDH][LCDW];
extern  unsigned char output2[LCDH][LCDW];
extern  unsigned char ooutput2[LCDH][LCDW];
extern int min_element[3];
extern struct pid_param_t pama;
extern float KP_motor;extern float KI_motor;extern float KD_motor;


extern int RIGHT_EDGE_type;
extern int LEFT_EDGE_type;



extern unsigned short L_Threshold;
extern float K_MD;
extern unsigned short distc;
extern int time;
extern int inelement;
extern int lb_ra;
extern int lb_la;
extern int ti;
extern float Yaw,Pitch,Roll;
int e1max=0;
int e2max=0;
extern int zdcd;
extern int duty1,duty2;

extern int tezheng[9];
struct running_pama
{
        int enable_csdjc;// chu sai dao jian ce
        int enable_jays;//jin ai yuan su
        int enable_jawd;//jin ai wan dao
        int speed_set;//sudushezhi
        int chasu_set;//chasushezhi
        int kmd_mod;//KMDmoshi
        int ckfx;//chukufangxiang
        int rucs;//dijicikandaobanmaxianruku
        int enable_lty;
        int pdspeed;

};

extern struct running_pama pama_run;

int core1_main (void)
{

    delayms(300);

    // ¿ªÆôCPU×ÜÖÐ¶Ï
    IfxCpu_enableInterrupts();

    // ¹Ø±Õ¿´ÃÅ¹·
    IfxScuWdt_disableCpuWatchdog (IfxScuWdt_getCpuWatchdogPassword ());

    // µÈ´ýCPU0 ³õÊ¼»¯Íê³É
    while(!IfxCpu_acquireMutex(&mutexCpu0InitIsOk));
    //µç¸Ð¼°µç³ØµçÑ¹ ADC²É¼¯³õÊ¼»¯
       ADC_InitConfig(ADC7, 80000);//³õÊ¼»¯   Èç¹ûÊ¹ÓÃÁúÇñÄ¸°å  Ôò²â·ÖÑ¹ºóµÄµç³ØµçÑ¹£¬¾ßÌå¿ÉÒÔ¿´Ä¸°åÔ­ÀíÍ¼
    // ÇÐ¼ÇCPU0,CPU1...²»¿ÉÒÔÍ¬Ê±¿ªÆôÆÁÄ»ÏÔÊ¾£¬·ñÔò³åÍ»²»ÏÔÊ¾
    mutexCpu0TFTIsOk=1;         // CPU1£º 0Õ¼ÓÃ/1ÊÍ·Å TFT
    /* µç»ú¡¢¶æ»ú£¬±àÂëÆ÷³õÊ¼»¯ */
       MotorInit();    // µç»ú
       ServoInit();    // ¶æ»ú
       EncInit();  // ±àÂëÆ÷
       //TestServo();
    // ¶¨Ê±Æ÷³õÊ¼»¯,Ô­Ê¼ÖÐ¶Ïº¯ÊýÔÚCCU6.CÖÐ */
       CCU6_InitConfig(CCU61, CCU6_Channel0, 5000);// 5ms
    //Test_CAMERA();         //PASS,²âÊÔÁúÇñÉñÑÛÉãÏñÍ·²¢ÔÚÆÁÄ»ÉÏÏÔÊ¾  LQ_CAMERA.h ÖÐÑ¡ÔñÆÁÄ»
    TFTSPI_Init(0);               // TFT1.8³õÊ¼»¯0:ºáÆÁ  1£ºÊúÆÁ
    TFTSPI_CLS(u16BLACK);         // ÇåÆÁ


    unsigned char oot[LCDH][LCDW];
    unsigned char oott[60][94];
    UART_InitConfig(UART0_RX_P14_1, UART0_TX_P14_0, 115200);
    //int lccr,rccr,lcco,rcco,la,ra;
    struct CORNER dLc,dRc,duLc,duRc;
    //while(1)


    //menu
    char menu[5][30]={
            "pama_run_set",
            "basic_pid",
            "show_carema",
            "run_gogog!!",
            "quick_gogogo"
    };
    char page_0[12][30]={
            "enable_csdjc",
            "enable_jays",
            "enable_jawd",
            "speed_set",
            "chasu_set",
            "kmd_mod",
            "ckfx",
            "rucs",
            "enable_lty",
            "pdspeed"
    };
    char page_1[12][30]={
            "kp mt",
            "ki mt",
            "kd mt",
            "kp smt",
            "ki smt",
            "kd smt"
    };
    int page_cj=0;//yemiancengji
    int select_xx=0;//dijigexuanxiang
    int sfxz=0;//shifouxuanzhong

    int page_num[3]={5,10,6};

    int pama_run_tidai[15];
    pama_run_tidai[0]=pama_run.enable_csdjc;
    pama_run_tidai[1]=pama_run.enable_jays;
    pama_run_tidai[2]=pama_run.enable_jawd;
    pama_run_tidai[3]=pama_run.speed_set;
    pama_run_tidai[4]=pama_run.chasu_set;//fanleyibaibei
    pama_run_tidai[5]=pama_run.kmd_mod;
    pama_run_tidai[6]=pama_run.ckfx;
    pama_run_tidai[7]=pama_run.rucs;
    pama_run_tidai[8]=pama_run.enable_lty;
    pama_run_tidai[9]=pama_run.pdspeed;

    float pid_tidai[6];
    pid_tidai[0] = KP_motor;
    pid_tidai[1] = KI_motor;
    pid_tidai[2] = KD_motor;
    pid_tidai[3] = pama.kp;      // P记得幅值
    pid_tidai[4] = pama.ki;       // I
    pid_tidai[5] = pama.kd;


    int pama_run_zengjian[15]={1,1,1,50,1,1,1,1,1,50};
    int pama_run_yuzhi[15]={2,2,2,-1,-1,-1,3,-1,2,-1};

    float pid_zengjian[6]={1,0.1,0.1,10,10,10};
    extern int waiting_cpu1;
    while(waiting_cpu1==1)
    {
        TFTSPI_P6X8Str(1, 0, "to my dear son -- ftl", u16WHITE, u16BLACK);
        if(page_cj==0)
        {
            int i;
            for(i=0;i<page_num[0];i++)
            {
                if(i==select_xx)
                    TFTSPI_P6X8Str(1, 1+i, menu[i], u16BLACK, u16WHITE);
                else
                    TFTSPI_P6X8Str(1, 1+i, menu[i], u16WHITE, u16BLACK);
            }



            if(KEY_Read(KEY0)==0)//按下KEY0键，
            {
                select_xx--;
                delayms(200);
            }
            if(KEY_Read(KEY1)==0)//按下KEY1键，
            {
                select_xx++;
                delayms(200);
            }
            if(KEY_Read(KEY2)==0)//按下KEY2键，
            {
                page_cj=select_xx+1;
                select_xx=0;
                TFTSPI_CLS(u16BLACK);
                delayms(200);
            }
            select_xx=(select_xx+page_num[0])%page_num[0];




            //if(KEY_Read(KEY0)==0|| KEY_Read(KEY1)==0||KEY_Read(KEY2)==0)
            //    delayms(200);


        }

        if(page_cj==1)
        {
            int i;
            for(i=0;i<page_num[1];i++)
            {
                if(i==select_xx&&sfxz==0)
                    TFTSPI_P6X8Str(1, 1+i, page_0[i], u16BLACK, u16WHITE);
                else if(i==select_xx&&sfxz==1)
                    TFTSPI_P6X8Str(1, 1+i, page_0[i], u16WHITE, u16BLUE);
                else
                    TFTSPI_P6X8Str(1, 1+i, page_0[i], u16WHITE, u16BLACK);
            }
            for(i=0;i<page_num[1];i++)
            {
                char txt[20];
                sprintf(txt,"%4d", pama_run_tidai[i]);
                if(i==select_xx&&sfxz==0)
                    TFTSPI_P6X8Str(16, 1+i, txt, u16BLACK, u16WHITE);
                else if(i==select_xx&&sfxz==1)
                    TFTSPI_P6X8Str(16, 1+i, txt, u16WHITE, u16BLUE);
                else
                    TFTSPI_P6X8Str(16, 1+i, txt, u16WHITE, u16BLACK);
            }



            if(sfxz==1)
            {
                if(KEY_Read(KEY0)==0)//按下KEY0键，
                {
                    pama_run_tidai[select_xx]+=pama_run_zengjian[select_xx];
                    delayms(200);
                }
                if(KEY_Read(KEY1)==0)//按下KEY1键，
                {
                    pama_run_tidai[select_xx]-=pama_run_zengjian[select_xx];
                    delayms(200);
                }
                if(KEY_Read(KEY2)==0)//按下KEY2键，
                {
                    sfxz=1-sfxz;
                    delayms(200);
                }
                if(pama_run_yuzhi[select_xx]!=-1)
                    pama_run_tidai[select_xx]=(pama_run_tidai[select_xx]+pama_run_yuzhi[select_xx])%pama_run_yuzhi[select_xx];
            }
            else if(sfxz==0)
            {
                if(KEY_Read(KEY0)==0)//按下KEY0键，
                {
                    //select_xx--;
                    page_cj=0;
                    select_xx=0;
                    TFTSPI_CLS(u16BLACK);
                    delayms(200);
                }
                if(KEY_Read(KEY1)==0)//按下KEY1键，
                {
                    select_xx++;

                    delayms(200);
                }
                if(KEY_Read(KEY2)==0)//按下KEY2键，
                {
                    sfxz=1-sfxz;
                    delayms(200);
                }
                select_xx=(select_xx+page_num[1])%page_num[1];
            }


        }

        if(page_cj==2)
        {
            int i;
            for(i=0;i<page_num[2];i++)
            {
                if(i==select_xx&&sfxz==0)
                    TFTSPI_P6X8Str(1, 1+i, page_1[i], u16BLACK, u16WHITE);
                else if(i==select_xx&&sfxz==1)
                    TFTSPI_P6X8Str(1, 1+i, page_1[i], u16WHITE, u16BLUE);
                else
                    TFTSPI_P6X8Str(1, 1+i, page_1[i], u16WHITE, u16BLACK);
            }
            for(i=0;i<page_num[2];i++)
            {
                char txt[20];
                sprintf(txt,"%2.1f", pid_tidai[i]);
                if(i==select_xx&&sfxz==0)
                    TFTSPI_P6X8Str(16, 1+i, txt, u16BLACK, u16WHITE);
                else if(i==select_xx&&sfxz==1)
                    TFTSPI_P6X8Str(16, 1+i, txt, u16WHITE, u16BLUE);
                else
                    TFTSPI_P6X8Str(16, 1+i, txt, u16WHITE, u16BLACK);
            }



            if(sfxz==1)
            {
                if(KEY_Read(KEY0)==0)//按下KEY0键，
                {
                    pid_tidai[select_xx]+=pid_zengjian[select_xx];
                    delayms(200);
                }
                if(KEY_Read(KEY1)==0)//按下KEY1键，
                {
                    pid_tidai[select_xx]-=pid_zengjian[select_xx];
                    delayms(200);
                }
                if(KEY_Read(KEY2)==0)//按下KEY2键，
                {
                    sfxz=1-sfxz;
                    delayms(200);
                }

            }
            else if(sfxz==0)
            {
                if(KEY_Read(KEY0)==0)//按下KEY0键，
                {
                    //select_xx--;
                    page_cj=0;
                    select_xx=0;
                    TFTSPI_CLS(u16BLACK);
                    delayms(200);
                }
                if(KEY_Read(KEY1)==0)//按下KEY1键，
                {
                    select_xx++;

                    delayms(200);
                }
                if(KEY_Read(KEY2)==0)//按下KEY2键，
                {
                    sfxz=1-sfxz;
                    delayms(200);
                }
                select_xx=(select_xx+page_num[2])%page_num[2];
            }


        }

        if(page_cj==3)
        {
            int i,j;

            for (i=0;i<LCDH;i++)
            {
                 for (j=0;j<LCDW;j++)
                     oot[i][j]=ooutput[i][j];
                     //oot[i][j]=output2[i][j];
            }
            TFTSPI_BinRoad(0, 10, LCDH, LCDW, (unsigned char *)oot);
            for (i=0;i<LCDH;i++)
            {
                 for (j=0;j<LCDW;j++)
                     //oot[i][j]=ooutput[i][j];
                     oot[i][j]=output2[i][j];
            }
     /*
             for (i=0;i<60;i++)
             {
                 for (j=0;j<94;j++)
                 {
                     oott[i][j]=ooutput2[i][j];
                 }
                     //oot[i][j]=ooutput[i][j];


             }*/

             TFTSPI_BinRoad(0, LCDH+10, LCDH, LCDW, (unsigned char *)oot);

             char txt[20];
             sprintf(txt,"%2d %2d %2d", tezheng[0],tezheng[1],tezheng[2]);
             TFTSPI_P6X8Str(16, 1, txt, u16WHITE, u16BLACK);
             sprintf(txt,"%2d %2d %2d", tezheng[3],tezheng[4],tezheng[5]);
             TFTSPI_P6X8Str(16, 2, txt, u16WHITE, u16BLACK);
             sprintf(txt,"%2d %2d %2d", tezheng[6],tezheng[7],tezheng[8]);
             TFTSPI_P6X8Str(16, 3, txt, u16WHITE, u16BLACK);

             sprintf(txt,"%10d", min_element[0]);
             TFTSPI_P6X8Str(16, 4, txt, u16WHITE, u16BLACK);
             sprintf(txt,"%10d", min_element[1]);
             TFTSPI_P6X8Str(16, 5, txt, u16WHITE, u16BLACK);
             sprintf(txt,"%10d", min_element[2]);
             TFTSPI_P6X8Str(16, 6, txt, u16WHITE, u16BLACK);

             sprintf(txt, "E1=%05d ", ECPULSE1);
             TFTSPI_P6X8Str(16, 7, txt, u16WHITE, u16BLACK);
             sprintf(txt, "E2=%05d ", ECPULSE2);
             TFTSPI_P6X8Str(16, 8, txt, u16WHITE, u16BLACK);

             sprintf(txt, "duty1=%05d ", duty1);
             TFTSPI_P6X8Str(16, 9, txt, u16WHITE, u16BLACK);
             sprintf(txt, "duty2=%05d ", duty2);
             TFTSPI_P6X8Str(16, 10, txt, u16WHITE, u16BLACK);

             sprintf(txt,"di=%5d", distc);
             TFTSPI_P6X8Str(16, 11, txt, u16WHITE, u16BLACK);

             if(KEY_Read(KEY0)==0|| KEY_Read(KEY1)==0||KEY_Read(KEY2)==0)
             {
                 TFTSPI_CLS(u16BLACK);
                 page_cj=0;
                 delayms(200);

             }
        }


        if(page_cj==4)
        {
            pama_run.enable_csdjc=pama_run_tidai[0];
            pama_run.enable_jays=pama_run_tidai[1];
            pama_run.enable_jawd=pama_run_tidai[2];
            pama_run.speed_set=pama_run_tidai[3];
            pama_run.chasu_set=pama_run_tidai[4];//fanleyibaibei
            pama_run.kmd_mod=pama_run_tidai[5];
            pama_run.ckfx=pama_run_tidai[6];
            pama_run.rucs=pama_run_tidai[7];
            pama_run.enable_lty=pama_run_tidai[8];
            pama_run.pdspeed=pama_run_tidai[9];


            KP_motor = pid_tidai[0];
            KI_motor = pid_tidai[1];
            KD_motor = pid_tidai[2];
            pama.kp = pid_tidai[3];      // P记得幅值
            pama.ki = pid_tidai[4];       // I
            pama.kd = pid_tidai[5];

            TFTSPI_CLS(u16BLACK);
            delayms(1000);
            waiting_cpu1=0;
        }
        if(page_cj==5)
        {
            pama_run.enable_csdjc=pama_run_tidai[0];
            pama_run.enable_jays=pama_run_tidai[1];
            pama_run.enable_jawd=pama_run_tidai[2];
            pama_run.speed_set=pama_run_tidai[3];
            pama_run.chasu_set=pama_run_tidai[4];//fanleyibaibei
            pama_run.kmd_mod=pama_run_tidai[5];
            pama_run.ckfx=2;
            pama_run.rucs=pama_run_tidai[7];
            pama_run.enable_lty=pama_run_tidai[8];
            pama_run.pdspeed=pama_run_tidai[9];


            KP_motor = pid_tidai[0];
            KI_motor = pid_tidai[1];
            KD_motor = pid_tidai[2];
            pama.kp = pid_tidai[3];      // P记得幅值
            pama.ki = pid_tidai[4];       // I
            pama.kd = pid_tidai[5];

            TFTSPI_CLS(u16BLACK);
            delayms(500);
            waiting_cpu1=0;
        }






              //延时等待

    }


    while(1)//主循环
    {
        // 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
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
        //TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *)Bin_Image);
        TFTSPI_BinRoad(0, 0, LCDH, LCDW, (unsigned char *)oot);




       for (i=0;i<LCDH;i++)
        {
            for (j=0;j<LCDW;j++)
                oot[i][j]=ooutput[i][j];
                //oot[i][j]=output2[i][j];
        }
/*
        for (i=0;i<60;i++)
        {
            for (j=0;j<94;j++)
            {
                oott[i][j]=ooutput2[i][j];
            }
                //oot[i][j]=ooutput[i][j];


        }*/

        TFTSPI_BinRoad(0, LCDH, LCDH, LCDW, (unsigned char *)oot);

/*
        sprintf(txt, "state: %02d \n", t_state);
        //UART_PutStr(UART0,txt);
        TFTSPI_P6X8Str(0, 5, txt, u16WHITE, u16BLACK);

        sprintf(txt, "pid1:%.01f %.01f %.01f \n", pama.kp,pama.ki,pama.kd);
        UART_PutStr(UART0,txt);
        TFTSPI_P6X8Str(0, 6, txt, u16WHITE, u16BLACK);
        sprintf(txt, "pid2:%.02f %.02f %.01f \n", KP_motor,KI_motor,KD_motor);
        UART_PutStr(UART0,txt);
        TFTSPI_P6X8Str(0, 7, txt, u16WHITE, u16BLACK);
*/
/*
        //delayms(500);
        sprintf(txt, "E1=%05d ", ECPULSE1);
        TFTSPI_P6X8Str(0, 0, txt, u16WHITE, u16BLACK);
        if (ECPULSE1>e1max)
            e1max=ECPULSE1;
        sprintf(txt, "E1max=%05d ", e1max);
        TFTSPI_P6X8Str(0, 1, txt, u16WHITE, u16BLACK);
        UART_PutStr(UART0,txt);
        sprintf(txt, "E2=%05d ", ECPULSE2);
        TFTSPI_P6X8Str(0, 2, txt, u16WHITE, u16BLACK);
        if (ECPULSE2>e2max)
            e2max=ECPULSE2;
        sprintf(txt, "E2max=%05d ", e2max);
        TFTSPI_P6X8Str(0, 3, txt, u16WHITE, u16BLACK);
        //UART_PutStr(UART0,txt);
        //delayms(500);
         */

        int lt=LEFT_EDGE_type;
        sprintf(txt, "Lt=%02d ", lt);
        TFTSPI_P6X8Str(16, 0, txt, u16WHITE, u16BLACK);
        //UART_PutStr(UART0,txt);
        int rt=RIGHT_EDGE_type;
        sprintf(txt, "Rt=%02d ", rt);
        TFTSPI_P6X8Str(16, 1, txt, u16WHITE, u16BLACK);
        //UART_PutStr(UART0,txt);

        int c1=0;
        int temp_i=1;
        int scan_row=40;
        while(temp_i<40)
        {
            if(white_(ooutput[scan_row][47-temp_i])||white_(ooutput[scan_row][47-temp_i-1])||white_(ooutput[scan_row][47-temp_i-2]))
            {
                temp_i++;
                c1++;
            }
            else
                break;

        }
        temp_i=1;
        while(temp_i<40)
        {
            if(white_(ooutput[scan_row][47+temp_i])||white_(ooutput[scan_row][47+temp_i+1])||white_(ooutput[scan_row][47+temp_i+2]))
            {
                temp_i++;
                c1++;
            }
            else
                break;

        }

        int t=time/1000;
        //t=c1;
        sprintf(txt, "t=%03d ", t);

        TFTSPI_P6X8Str(16, 2, txt, u16WHITE, u16BLACK);
        //int t;
        t=ti;
        sprintf(txt, "ti=%03d ", t);
        TFTSPI_P6X8Str(16, 4, txt, u16WHITE, u16BLACK);
        int el=inelement;
        sprintf(txt, "el=%d ", el);
        TFTSPI_P6X8Str(16, 3, txt, u16WHITE, u16BLACK);
        extern int det_error_mohu;
        extern int error_mohu;
        extern int xieru_flag;
        sprintf(txt, "xr=%d ", xieru_flag);
        TFTSPI_P6X8Str(16, 5, txt, u16WHITE, u16BLACK);
        sprintf(txt, "ra=%3d", lb_ra);
        TFTSPI_P6X8Str(16, 6, txt, u16WHITE, u16BLACK);

        sprintf(txt, "la=%3d ", lb_la);
        TFTSPI_P6X8Str(16, 7, txt, u16WHITE, u16BLACK);
        sprintf(txt, "yaw=%3.2f ", Yaw);
        TFTSPI_P6X8Str(16, 8, txt, u16WHITE, u16BLACK);
        sprintf(txt, "pitch=%3.2f ", Pitch);
        TFTSPI_P6X8Str(16, 9, txt, u16WHITE, u16BLACK);
        sprintf(txt, "roll=%3.2f ", Roll);
        TFTSPI_P6X8Str(16, 10, txt, u16WHITE, u16BLACK);
        sprintf(txt, "zdcd=%3d ", zdcd);
        TFTSPI_P6X8Str(16, 11, txt, u16WHITE, u16BLACK);

        sprintf(txt,"yyh=%6d", min_element[0]);
        TFTSPI_P6X8Str(16, 12, txt, u16WHITE, u16BLACK);
        sprintf(txt,"zyh=%6d", min_element[1]);
        TFTSPI_P6X8Str(16, 13, txt, u16WHITE, u16BLACK);
        sprintf(txt,"cd=%6d", min_element[2]);
        TFTSPI_P6X8Str(16, 14, txt, u16WHITE, u16BLACK);
        sprintf(txt,"di=%4d", distc);
        TFTSPI_P6X8Str(16, 15, txt, u16WHITE, u16BLACK);
        //Yaw,Pitch,Roll

        //delayms(500);



        // CameraCar(); // 摄像头四轮车双电机控制，简单的分段比例控制算法
    }
}
