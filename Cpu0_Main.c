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
● 场信号：外部中断第3组：P15_1；
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
默认蜂鸣器接口
BEEPp         P33_8      龙邱TC母板上蜂鸣器接口
-----------------------------------------------------------------
定时器
有两个CCU6模块  每个模块有两个独立定时器  触发定时器中断
推荐使用CCU6模块，STM用作系统时钟或者延时；
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <include.h> //各个模块的头文件
#include <IfxCpu.h>
#include <IfxScuCcu.h>
#include <IfxScuWdt.h>
#include <IfxStm.h>
#include <IfxStm_reg.h>
#include <LQ_CAMERA.h>
#include <LQ_CCU6.h>
#include <LQ_GPIO_KEY.h>
#include <LQ_GPIO_LED.h>
#include <LQ_MotorServo.h>
#include <LQ_SOFTI2C.h>
#include <LQ_TFT18.h>
#include <LQ_UART.h>
#include <LQ_Inductor.h>
#include <Main.h>
#include "LQ_ImageProcess.h"

App_Cpu0 g_AppCpu0;                     // brief CPU 0 global data
IfxCpu_mutexLock mutexCpu0InitIsOk = 1; // CPU0 初始化完成标志位
volatile char mutexCpu0TFTIsOk = 0;     // CPU1 0占用/1释放 TFT

/*************************************************************************
 *  函数名称：int core0_main (void)
 *  功能说明：CPU0主函数
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年3月10日
 *  备    注：
 *************************************************************************/

#define IMGH 60
#define IMGW 94
#define length 150
int K_MD = 30;
int t_state = 0; // ceshiyong shan
unsigned char output[LCDH][LCDW];
unsigned char ooutput[LCDH][LCDW];
unsigned char output2[LCDH][LCDW];
unsigned char ooutput2[LCDH][LCDW];
int time=0;
int ti=0;

struct pid_param_t
{
    float kp;    // P记得幅值
    float ki;    // I
    float kd;    // D
    float i_max; // integrator_max
    float p_max; // integrator_max
    float d_max; // integrator_max

    float low_pass;

    float out_p;
    float out_i;
    float out_d;

    float error;
    float pre_error;
    float pre_pre_error;
};

//进入元素 0为无 1为十字 2为圆环
int inelement=0;
void event_check(); //事件判断
int check_yhj();
int motor_PID(int ECPULSE_SET, int ECPULSE_REAL, int *error_last_1, int *error_last_2, float KP, float KI, float KD, int *duty);
int duty_max = 7000;
struct pid_param_t pama;
struct pid_param_t *p_pama = &pama;
int error_last_11 = 0;
int error_last_12 = 0;
int error_last_21 = 0;
int error_last_22 = 0;
int duty1 = 0, duty2 = 0;
int SPEED_SET = 550;
float KP_motor = 2.0;
float KI_motor = 6.0;
float KD_motor = 4.0;
float pid_solve(struct pid_param_t *pid, float error);
float get_pure_bias();
void pid_init();
void Image_process(unsigned char image[][IMGW]);
void clear_point();
int edge_point_ornot(unsigned char image[][IMGW], int row, int side);
int white_(int x); //ÅÐ¶Ï°×
int black_(int x); //ÅÐ¶ÏºÚ
int Get_angle(int ax, int ay, int bx, int by, int cx, int cy);
void get_mid(unsigned char image[][IMGW]);
float akm(int duty);
int Road_Width = 36; //Â·¿í
//      Ê¹ÄÜÏà¹Ø±äÁ¿
int enable_balinyu = 1;             //°ËÁÚÓòÅÀÏßÊ¹ÄÜ±êÖ¾£¬Ä¬ÈÏÊ¹ÄÜ
int enable_midline = 0;             //Ê¹ÄÜÖÐÏßÄâºÏ£¬Ä¬ÈÏ²»¿ªÆô
int enable_midpross = 1;            //Ê¹ÄÜÖÐÏß´¦Àí£¬Ä¬ÈÏ¿ªÆô
int enable_check_l_r_edge_same = 0; //Ê¹ÄÜ×ó¡¢ÓÒ±ßÏßÊÇ·ñÖØºÏ£¬Ä¬ÈÏ²»¿ªÆô
//      ËÑÏßÏÞÖÆÏà¹Ø
int L_edge_end_row = 5;              //×ó±ß½çÐÐ½áÊøµã
int R_edge_end_row = 5;              //ÓÒ
int M_edge_end_row = 5;              //ÖÐ
int min_col = 3, max_col = IMGW - 3; //×îÐ¡ÁÐÖµ¡¢×î´óÁÐÖµ
//      ËÑÏß¼ÆÊýÏà¹Ø

int left_findflag, right_findflag;                    //ÊÇ·ñÕÒµ½×óÓÒ±ß½ç
int left_findflag2, right_findflag2;                  //ÊÇ·ñÕÒµ½×óÓÒ±ß½ç
int L_basic_row_start = IMGH - 1;                     //×ó±ßÏßËÑÏß¿ªÊ¼µã
int R_basic_row_start = IMGH - 1;                     //ÓÒ±ßÏßËÑÏß¿ªÊ¼µã
int L_search_edge_count = 0, R_search_edge_count = 0; //ËÑË÷µ½±ß½ç
int line_point_count_left, line_point_count_right;    //×óÓÒÏßÓÐÐ§µã¼ÆÊý
int L_edge_count = 0, R_edge_count = 0;               //×óÓÒ±ßµãµÄ¸öÊý
int Mid_count = 0;                                    //ÖÐÏßµãµÄ¸öÊý
int center_arry_count;
int line_lose_center_left;
int line_lose_center_right;
int dire_left;        //¼ÇÂ¼ÉÏÒ»¸öµãµÄÏà¶ÔÎ»ÖÃ
int dire_right;       //¼ÇÂ¼ÉÏÒ»¸öµãµÄÏà¶ÔÎ»ÖÃ
int center_turn_flag; // 0 ×óÇ÷ÊÆ£¬ 1 ÓÒÇ÷ÊÆ
int center_biaoxiang_arry[5];
//      ±ß½çÔ¤´¦Àí
int edge_process_flag = 0;
int pre_L_edge_count = 0;
int pre_R_edge_count = 0;
int num_cnt = 0; //¼ÇÂ¼Á¬ÐøË®Æ½µãµÄ¸öÊý
int L_count = 0;
int R_count = 0;
//          ¹Õµã´¦Àí
int enable_L_corner = 1; //×ó¹ÕµãËÑË÷Ê¹ÄÜ±êÖ¾ Ä¬ÈÏÊ¹ÄÜ
int enable_R_corner = 1; //ÓÒ¹ÕµãËÑË÷ÊÇÄÜ±êÖ¾ Ä¬ÈÏÊ¹ÄÜ
struct CORNER
{
    int corner_flag;
    int corner_row;
    int corner_col;
    int corner_angle;
    int corner_icount;
};
struct CORNER Lc, Rc, uLc, uRc;


/*
int L_corner_flag = 0;   //×ó¹Õµã´æÔÚ±êÖ¾
int L_corner_row = 0;    //×ó¹ÕµãËùÔÚÐÐ
int L_corner_col = 0;    //×ó¹ÕµãËùÔÚÁÐ
int L_corner_angle = 0;  //×ó¹Õµã½Ç¶È
int R_corner_flag = 0;   //ÓÒ¹Õµã´æÔÚ±êÖ¾
int R_corner_row = 0;    //ÓÒ¹ÕµãËùÔÚÐÐ
int R_corner_col = 0;    //ÓÒ¹ÕµãËùÔÚÁÐ
int R_corner_angle = 0;  //ÓÒ¹Õµã½Ç¶È*/
//×Ô¶¨Òå½á¹¹Ìå¼°Ïà¹Ø±äÁ¿
struct EDGE
{
    int row;  //ÐÐ×ø±ê
    int col;  //ÁÐ×ø±ê
    int flag; //´æÔÚ±ß½çµÄ±êÖ¾
};            /*
            struct RIGHT_EDGE
            {
                int row;  //ÐÐ×ø±ê
                int col;  //ÁÐ×ø±ê
                int flag; //´æÔÚ±ß½çµÄ±êÖ¾
            };*/
int min_count = 20;
unsigned char image_zero[IMGH][IMGW]; //Ê¹ÓÃ¶þÎ¬Ö¸Õëimage´úÌæÐÅÏ¢Í¼Ïñ£¬±ãÓÚºóÆÚ¶þÖµ»¯Í¼ÏñºÍÉî¶ÈÍ¼ÏñµÄ×ª»»
int row_known_geti(int row,struct EDGE line[], int count);


struct EDGE L_edge[150]; //×ó±ß½ç½á¹¹Ìå
struct EDGE R_edge[150]; //ÓÒ±ß½ç½á¹¹Ìå
int LEFT_EDGE_type;
/*0 丢线      1 短水平线       2 弯道    3 角点  4 异常角点
 * -1 未知（无拐点——大概率直线
 *
 * */
int RIGHT_EDGE_type;
int LEFT_EDGE_losscount,RIGHT_EDGE_losscount;

struct EDGE M_Line[150];      //ÖÐÏß½á¹¹Ìå£¬±¸ÓÃÑ¡Ôñ2
struct EDGE Last_M_Line[150]; //ÉÏ´ÎÖÐÏß½á¹¹Ìå
struct EDGE MID_LINE[150];    //ÖÐÏß½á¹¹Ìå£¬±¸ÓÃÑ¡Ôñ1
struct EDGE *Mid_Line;        //ÖÐÏß½á¹¹Ìå¡¾Êµ¼ÊÊ¹ÓÃ¡¿
                              //      ÓÃÓÚÖÐÏß´¦Àí
int searchpoint = 1;          //Õý³£Çó³öÀ´µÄÖÐÏßµã±êÖ¾£¬ÓÃÓÚ½á¹¹Ìå.flag£¬±ãÓÚºóÆÚ´¦Àí
int nihepoint = 2;            //ÄâºÏÇó³öÀ´µÄÖÐÏßµã±êÖ¾£¬ÓÃÓÚ½á¹¹Ìå.flag£¬±ãÓÚºóÆÚ´¦Àí
float Kakm;
int ercinh(struct EDGE *Line, int count);
void corner_find(struct EDGE line[], int dir, struct CORNER *cor, int count); //线的性质判断
void line_check();


void process_sz();
void process_element();
void process_yh();
void process_cd();
int szorcd_check();
int checkcorner(struct EDGE line[],int i);
void spot2spot(unsigned char immage[][IMGW], int x1,int y1, int x2,int y2 );
/*************************************************************************
函数声明用

*************************************************************************/
int core0_main(void)
{

    // 关闭CPU总中断
    IfxCpu_disableInterrupts();

    // 关闭看门狗，如果不设置看门狗喂狗需要关闭
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    // 读取总线频率
    g_AppCpu0.info.pllFreq = IfxScuCcu_getPllFrequency();
    g_AppCpu0.info.cpuFreq = IfxScuCcu_getCpuFrequency(IfxCpu_getCoreIndex());
    g_AppCpu0.info.sysFreq = IfxScuCcu_getSpbFrequency();
    g_AppCpu0.info.stmFreq = IfxStm_getFrequency(&MODULE_STM0);

    // TFTSPI_Init(0);               // TFT1.8初始化0:横屏  1：竖屏
    // TFTSPI_CLS(u16BLACK);         // 清屏
    // TFTSPI_P16x16Str(0,0,(unsigned char*)"冯腾龙",u16RED,u16BLUE);// 字符串显示

    // 按键初始化
    GPIO_KEY_Init();
    // LED灯所用P10.6和P10.5初始化
    GPIO_LED_Init();

    // 串口P14.0管脚输出,P14.1输入，波特率115200
    UART_InitConfig(UART0_RX_P14_1, UART0_TX_P14_0, 115200);

    // 开启CPU总中断
    IfxCpu_enableInterrupts();

    // 通知CPU1，CPU0初始化完成
    IfxCpu_releaseMutex(&mutexCpu0InitIsOk);
    // 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
    mutexCpu0TFTIsOk = 1; // CPU1： 0占用/1释放 TFT

    // 程序配套视频地址：https://space.bilibili.com/95313236
    // 以下测试函数，内建死循环，用户可调用所用模块的初始化及读写函数来实现自己的任务
    //________________________________________________________________________________
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //    Test_ADC();            //PASS,测试ADC采样时间  OLED上显示 ADC采样10K次时间
    //    Test_ADC_7mic();       //PASS,测试ADC\UART0、STM延时和闪灯，通过UART0打印 AN0--AN7共8个通道ADC转换数据
    //    LQ_Atom_Motor_8chPWM();//PASS,测试GTM_ATOM生成不同频率下的8路PWM
    //    LQ_ATom_Servo_2chPWM();//PASS,测试GTM_ATOM、STM延时和闪灯，P33.10和P33.13作为ATOM输出口控制舵机
    //    LQ_Tom_Servo_2chPWM(); //PASS,测试GTM_TOM、STM延时和闪灯，P33.10和P33.13作为TOM输出口控制舵机
    //    Test_GPIO_Extern_Int();//PASS,测试外部第1组中断P15.8，P10.6和P10.5闪灯
    //    Test_GPIO_LED();       //PASS,测试GPIO，P10.6和P10.5闪灯
    //    Test_GPIO_KEY();       //PASS,测试外部按键输入，P22.0--2   按下按键 LED亮
    //    Test_ComKEY_Tft(  );     //PASS,测试外部组合按键输入并TFT1.8显示，P22.0--2
    //    LQ_GPT_4mini512();     //PASS,测试编码器正交解码,OLED和UART输出
    //  LQ_GPT_4mini512TFT();  //PASS,测试编码器正交解码,TFT1.8和UART输出
    //  Test_OLED();           //PASS,测试OLED0.96屏使用P20.14--10，显示字符串及动态数据
    //    LQ_STM_Timer();        //PASS,测试定时中断、闪灯
    // Test_TFT18();          //PASS,测试TFT1.8彩屏使用P20.14--10，显示字符串及动态数据
    //    LQ_TIM_InputCature();  //PASS,测试GTM_TOM_TIM、P20_9作为PWM输出口，P15_0作为TIM输入口，两者短接后，串口P14.0发送到上位机
    //   Test_Bluetooth();      //PASS,测试UART0(P14.0RX/P14.1TX)，
    //    Test_EEPROM();         //PASS,测试内部EEPROM擦写功能  OLED提示是否写入成功
    //    Test_Vl53();           //PASS,测试VL53  IIC接线   P13_1接SCL  P13_2接SDA OLED显示原始数据
    //    Test_9AX();            //PASS,测试龙邱九轴 IIC接线   P13_1接SCL  P13_2接SDA OLED显示原始数据
    //    Test_MPU6050();        //PASS,测试MPU6050或者ICM20602 IIC接线   P13_1接SCL  P13_2接SDA OLED显示原始数据
    //    Test_ICM20602();       //PASS,测试ICM20602 SPI接线   P15_8接SCL  P15_5接SDA  P15_7接SA  P15_2接CS OLED显示原始数据
    // Test_CAMERA();         //PASS,测试龙邱神眼摄像头并在屏幕上显示  LQ_CAMERA.h 中选择屏幕
    //    Test_SoftFft();        //PASS,测试ILLD库的软件FFT函数
    //    Test_FFT();            //PASS,测试硬件FFT  注意需要芯片后缀带DA的才有硬件FFT功能
    //    Test_RDA5807();        //PASS,测试RDA5807立体声收音机SCL 接 P00_1   SDA 接 P00_2
    //    TestMotor();           //PASS,测试电机，TFT1.8屏幕显示PWM，通过宏定义选择不通的驱动板。BTN7971或DRV8701
    //________________________________________________________________________________
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // 电感及电池电压 ADC采集初始化
    // InductorInit();
    // ADC_InitConfig(ADC7, 80000); // 初始化   如果使用龙邱母板  则测分压后的电池电压，具体可以看母板原理图

    /* 电机、舵机，编码器初始化 */
    MotorInit(); // 电机
    ServoInit(); // 舵机
    EncInit();   // 编码器

    // 以下三个测试函数为死循环，标定舵机、电机和编码器用的，开启后后面不会运行！
    // TestServo();  // 测试及标定舵机，TFT1.8输出
    //  TestMotor();  // 测试及标定电机，TFT1.8输f出
    // TestEncoder();  // 测试编码器正交解码,TFT1.8和UART输出
    /* 摄像头初始化 */
    CAMERA_Init(100);
    pid_init();
    RIGHT_EDGE_type=LEFT_EDGE_type=-5;
    int cccc=0;
    //unsigned long lastTime = STM_GetNowUs(STM0);

    int delttime;
    while (1) //主循环
    {

        if (Camera_Flag == 2)
        {

            unsigned long nowTime = STM_GetNowUs(STM0);

            Mid_count=0;
            Get_Use_Image();
            Camera_Flag = 0;
            Get_Bin_Image(0);

            int X, Y, x, y;
            uint32 duty_smotor;
            float s;
            char txt[20];

            float Hd[3][3] = {{2.659134, -0.004177, -27.589874}, {1.114640, 1.033582, -18.780196}, {0.024500, -0.000071, 0.746029}};
            for (x = 0; x < LCDH; x++)
            {
                for (y = 0; y < LCDW; y++)
                {

                    X = (int)(((Hd[0][1] - Hd[2][1] * y) * (Hd[1][2] - Hd[2][2] * y) - (Hd[0][2] - Hd[2][2] * x) * (Hd[1][1] - Hd[2][1] * y)) / ((Hd[0][0] - Hd[2][0] * x) * (Hd[1][1] - Hd[2][1] * y) - (Hd[0][1] - Hd[2][1] * x) * (Hd[1][0] - Hd[2][0] * y)) + 0.5);
                    Y = -(int)(((Hd[0][0] - Hd[2][0] * x) * (Hd[1][2] - Hd[2][2] * y) - (Hd[0][2] - Hd[2][2] * x) * (Hd[1][0] - Hd[2][0] * y)) / ((Hd[0][0] - Hd[2][0] * x) * (Hd[1][1] - Hd[2][1] * y) - (Hd[0][1] - Hd[2][1] * x) * (Hd[1][0] - Hd[2][0] * y)) + 0.5);
                    if (X >= 0 && LCDH > X && Y >= 0 && LCDW > Y)
                    {
                        ooutput[x][y] = Bin_Image[X][Y];
                    }
                    else
                    {
                        ooutput[x][y] = 0;
                    }
                }
            }


            Image_process((unsigned char *)ooutput);
            line_check();
            // event_check();
            if (enable_midline)
                get_mid((unsigned char *)ooutput);

            s = (int)(get_pure_bias() + 0.5);


            duty_smotor = 625 - 5 * s;
            if (duty_smotor >= Servo_Left_Max)                  //限制幅值
                duty_smotor = Servo_Left_Max;
            else if (duty_smotor <= Servo_Right_Min)            //限制幅值
                duty_smotor = Servo_Right_Min;


            cccc++;
            if(cccc==2)
            {
                ServoCtrl(duty_smotor);
                cccc=0;
            }




            Kakm= akm(duty_smotor);

            //lastTime = nowTime;
            //nowTime = STM_GetNowUs(STM0); //采样时间开始
            //ECPULSE1 = ENC_GetCounter(ENC2_InPut_P33_7);  //左电机 母板上编码器1，小车前进为负值
            //ECPULSE2 = -ENC_GetCounter(ENC4_InPut_P02_8); //右电机 母板上编码器2，小车前进为正值


            int E1=ECPULSE1*20/5;
            int E2=-ECPULSE2*20/5;

            motor_PID(SPEED_SET-(int)(Kakm*SPEED_SET), E1, &error_last_11, &error_last_12, KP_motor, KI_motor, KD_motor, &duty1);
            motor_PID(SPEED_SET+(int)(Kakm*SPEED_SET), E2, &error_last_21, &error_last_22, KP_motor, KI_motor, KD_motor, &duty2);

            int delt=1000;
            int bangbangduty=7000;
            if(E1-(SPEED_SET-(int)(Kakm*SPEED_SET))>delt)
                duty1=-bangbangduty+6000;
            else if((SPEED_SET-(int)(Kakm*SPEED_SET))-E1>delt)
                duty1=bangbangduty;
            if(E2-(SPEED_SET+(int)(Kakm*SPEED_SET))>delt)
                duty2=-bangbangduty+6000;
            else if((SPEED_SET+(int)(Kakm*SPEED_SET))-E2>delt)
                duty2=bangbangduty;
            //duty1=duty2=1000;
            MotorCtrl4w(duty1, duty2, duty1, duty2);




            LED_Ctrl(LEDALL, RVS);
            time = STM_GetNowUs(STM0) - nowTime; //采样结束
        }

    }
}

/*--------------------------------------------------------------------------
 * ¡¾º¯Êý¹¦ÄÜ¡¿£ºÍ¼Ïñ´¦Àí£¨ÇãÐ±Ð£Õý£¬ÖÐÏß¡¢ÔªËØÊ¶±ð£©_°ËÁÚÓòÈÝÒ×Ô½½ç
 * ¡¾²Î    Êý¡¿£ºÎÞ
 * ¡¾·µ »Ø Öµ¡¿£ºÎÞ
 *--------------------------------------------------------------------------*/
int x1, x2, x3, x4;
void Image_process(unsigned char image[][IMGW])
{

    //ÄæÍ¸ÊÓ¿ªÊ¼

    //ÄæÍ¸ÊÓÍê³É

    // int Bottom2 = IMGH - 40;                                         //µ¹ÊýµÚ¶þÐÐ
    int max_col = IMGW - 1, min_col = 1;                                //×î´ó/Ð¡ÁÐ×ø±ê£¬¼´±ß½çÖµ
    int L_search_amount = 150, R_search_amount = 150;                   //×óÓÒ±ß½çËÑµãÊ±×î¶àÔÊÐíµÄµã
    int jilu_row_l = 0, jilu_col_l = 0, jilu_row_r = 0, jilu_col_r = 0; //¼ÇÂ¼ËÑË÷µ½µÄ»ù´¡±ß½çµãÐÐÁÐÖµ
                                                                        //Í¼Ïñ´¦Àí-----------------------
                                                                        //  Ò»°ãÍ¼Ïñ´¦Àí
                                                                        //      ³õÊ¼»¯Ïà¹Ø±äÁ¿
    // enable_check_l_r_edge_same = 0;                                       //Ê¹ÄÜ¼ì²é×öÓÒ±ßÏßÊÇ·ñÅÀÖØºÏ£¬Ä¬ÈÏ²»¿ªÆô£¬µ±ÅÀÏßÆðÊ¼µã¹ý¸ßÊ±¿ªÆô
    left_findflag = 0;            //×ó±ß½ç´æÔÚ±êÖ¾£¬1ÕÒµ½×ó±ß½ç£¬0Ã»ÕÒµ½×ó±ß½ç        Ä¬ÈÏÃ»ÓÐÕÒµ½×ó±ß½ç
    right_findflag = 0;           //ÓÒ±ß½ç´æÔÚ±êÖ¾£¬1ÕÒµ½ÓÐ±ß½ç£¬0Ã»ÕÒµ½ÓÒ±ß½ç        Ä¬ÈÏÃ»ÓÐÕÒµ½ÓÒ±ß½ç
    L_basic_row_start = IMGH - 2; //×ó¿ªÊ¼ËÑÏßµã
    R_basic_row_start = IMGH - 2; //ÓÒ¿ªÊ¼ËÑÏßµã
    //±ßÏßÑ°ÕÒ¿ªÊ¼
    line_lose_center_left = 0;
    line_lose_center_right = 0;
    line_point_count_left = 0;
    line_point_count_right = 0;
    L_edge_count = 0;        //×ó±ßµã¸öÊýÇå0
    R_edge_count = 0;        //ÓÒ±ßµã¸öÊýÇå0
    LEFT_EDGE_losscount=0;
    RIGHT_EDGE_losscount=0;
    int exist_edge_size = 0; //ÅÐ¶ÏÊÇ·ñ´æÔÚ×ó/ÓÒ±ß½ç

    //Ñ°ÕÒ×ó\ÓÒÏß¿ªÊ¼µã£¬²¢ÅÐ¶ÏÊÇ·ñ´æÔÚµ±Ç°±ß
    clear_point();
    exist_edge_size = edge_point_ornot(image, L_basic_row_start, 0);
    if (exist_edge_size >= 0)
    {
        jilu_row_l = L_basic_row_start;
        jilu_col_l = exist_edge_size;
        left_findflag = 1;
    }
    exist_edge_size = edge_point_ornot(image, R_basic_row_start, 1);
    if (exist_edge_size >= 0)
    {
        jilu_row_r = R_basic_row_start;
        jilu_col_r = exist_edge_size;
        right_findflag = 1;
    }
    //°ËÁÚÓòËÑÏß
    if (left_findflag) //Èç¹û×ó±ß½çµã´æÔÚ²¢ÕÒµ½,Ôò¿ªÊ¼ÅÀÏß
    {
        //±äÁ¿ÉùÃ÷
        L_edge[0].row = jilu_row_l;
        L_edge[0].col = jilu_col_l;
        L_edge[0].flag = 1;
        int curr_row = jilu_row_l; //³õÊ¼»¯ÐÐ×ø±ê
        int curr_col = jilu_col_l; //³õÊ¼»¯ÁÐ×ø±ê
        dire_left = 0;             //³õÊ¼»¯ÉÏ¸ö±ß½çµãµÄÀ´Ïò
        center_turn_flag = 1;      //³õÊ¼»¯ÔÚÎ´±ê¶¨×´Ì¬
        //¿ªÊ¼ËÑÏß£¬×î¶àÈ¡150¸öµã£¬²»»áÍùÏÂËÑ£¬¹²7¸ö·½Î»
        for (int i = 1; i < L_search_amount; i++) //×î¶àËÑË÷70¸öµã
        {
            ////Ô½½çÍË³ö ÐÐÔ½½çºÍÁÐÔ½½ç£¨ÏòÉÏÏòÏÂÏò×óÏòÓÒ£©
            if (curr_row < L_edge_end_row || curr_row > IMGH - 1 || curr_row + 1 < L_edge_end_row)
                break;

            if (curr_col > max_col-15 || curr_col < min_col)
            // if (curr_col - 23.0 / 47 * curr_row + 276.0 / 47 < 0 && curr_row < 59 - min_count)
            {
                // if (++L_search_edge_count == 3)
                //Á¬Ðø3´ÎËÑË÷µ½±ß½ç£¬ÍË³ö

                //L_search_edge_count = 1;
                break;
            }
            else if(curr_col - 23.0 / 47 * curr_row + 276.0 / 47 < 0 && curr_row < 59 - min_count&&LEFT_EDGE_losscount==0)
                LEFT_EDGE_losscount=L_edge_count;
                //L_search_edge_count = 0;
            //ËÑÏß¹ý³Ì
            if (dire_left != 2 && black_(image[curr_row - 1][curr_col - 1]) && white_(image[curr_row - 1][curr_col])) //×óÉÏºÚ£¬2£¬ÓÒ±ß°×
            {
                curr_row = curr_row - 1;
                curr_col = curr_col - 1;
                L_edge_count = L_edge_count + 1;
                dire_left = 7;
                L_edge[i].row = curr_row;
                L_edge[i].col = curr_col;
                L_edge[i].flag = 1;
            }
            else if (dire_left != 3 && black_(image[curr_row - 1][curr_col + 1]) && white_(image[curr_row][curr_col + 1])) //ÓÒÉÏºÚ£¬3£¬ÏÂ±ß°×
            {
                curr_row = curr_row - 1;
                curr_col = curr_col + 1;
                L_edge_count = L_edge_count + 1;
                dire_left = 6;
                L_edge[i].row = curr_row;
                L_edge[i].col = curr_col;
                L_edge[i].flag = 1;
            }
            else if (dire_left != 1 && black_(image[curr_row - 1][curr_col]) && white_(image[curr_row - 1][curr_col + 1])) //ÕýÉÏºÚ£¬1£¬ÓÒ°×
            {
                curr_row = curr_row - 1;
                L_edge_count = L_edge_count + 1;
                dire_left = 0;
                L_edge[i].row = curr_row;
                L_edge[i].col = curr_col;
                L_edge[i].flag = 1;
            }
            else if (dire_left != 5 && black_(image[curr_row][curr_col - 1]) && white_(image[curr_row - 1][curr_col - 1])) //Õý×óºÚ£¬5£¬ÉÏ°×
            {
                curr_col = curr_col - 1;
                L_edge_count = L_edge_count + 1;
                dire_left = 4;
                L_edge[i].row = curr_row;
                L_edge[i].col = curr_col;
                L_edge[i].flag = 1;
            }
            else if (dire_left != 4 && black_(image[curr_row][curr_col + 1]) && white_(image[curr_row + 1][curr_col + 1])) //ÕýÓÒºÚ£¬4£¬ÏÂ°×
            {
                curr_col = curr_col + 1;
                L_edge_count = L_edge_count + 1;
                dire_left = 5;
                L_edge[i].row = curr_row;
                L_edge[i].col = curr_col;
                L_edge[i].flag = 1;
            }
            else if (dire_left != 6 && black_(image[curr_row + 1][curr_col - 1]) && white_(image[curr_row][curr_col - 1])) //×óÏÂºÚ£¬6£¬ÉÏ°×
            {
                curr_row = curr_row + 1;
                curr_col = curr_col - 1;
                L_edge_count = L_edge_count + 1;
                dire_left = 3;
                L_edge[i].row = curr_row;
                L_edge[i].col = curr_col;
                L_edge[i].flag = 1;
            }
            else if (dire_left != 7 && black_(image[curr_row + 1][curr_col + 1]) && white_(image[curr_row + 1][curr_col])) //ÓÒÏÂºÚ£¬7£¬×ó°×
            {
                curr_row = curr_row + 1;
                curr_col = curr_col + 1;
                L_edge_count = L_edge_count + 1;
                dire_left = 2;
                L_edge[i].row = curr_row;
                L_edge[i].col = curr_col;
                L_edge[i].flag = 1;
            }
            else if (dire_left != 0 && black_(image[curr_row + 1][curr_col]) && white_(image[curr_row + 1][curr_col - 1])) //ÕýÉÏºÚ£¬1£¬ÓÒ°×
            {
                curr_row = curr_row + 1;
                L_edge_count = L_edge_count + 1;
                dire_left = 1;
                L_edge[i].row = curr_row;
                L_edge[i].col = curr_col;
                L_edge[i].flag = 1;
            }
            else
                break;
        }
    }
    if (right_findflag) //Èç¹ûÓÒ±ß½ç´æÔÚ²¢ËÑµ½
    {
        R_edge[0].row = jilu_row_r;
        R_edge[0].col = jilu_col_r;
        R_edge[0].flag = 1;
        int curr_row = jilu_row_r;
        int curr_col = jilu_col_r;
        dire_right = 0;
        for (int i = 1; i < R_search_amount; i++)
        {
            //Ô½½çÍË³ö ÐÐÔ½½çºÍÁÐÔ½½ç£¨ÏòÉÏÏòÏÂÏò×óÏòÓÒ£©
            if (curr_row < L_edge_end_row || curr_row > IMGH - 1 || curr_row + 1 < L_edge_end_row)
                break;
            if (curr_col > max_col || curr_col < min_col+15) //Á¬ÐøÈý´ÎËÑË÷µ½±ß½ç£¬ÍË³ö
            // if(curr_col+24.0/59*curr_row-88>0&&curr_row<59 - min_count)
            {
                // if (++R_search_edge_count == 3)

                //R_search_edge_count = 1;
                break;
            }
            else if(curr_col+24.0/59*curr_row-88>0&&curr_row<59 - min_count&&RIGHT_EDGE_losscount==0)
                RIGHT_EDGE_losscount=R_edge_count;
                //R_search_edge_count = 0;
            //ÅÀÏß¹ý³Ì
            if (curr_col < IMGW && dire_right != 3 && black_(image[curr_row - 1][curr_col + 1]) && white_(image[curr_row - 1][curr_col])) //ÓÒÉÏºÚ£¬3£¬×ó°×
            {
                curr_row = curr_row - 1;
                curr_col = curr_col + 1;
                R_edge_count = R_edge_count + 1;
                dire_right = 6;
                R_edge[i].row = curr_row;
                R_edge[i].col = curr_col;
                R_edge[i].flag = 1;
            }
            else if (dire_right != 2 && black_(image[curr_row - 1][curr_col - 1]) && white_(image[curr_row][curr_col - 1])) //×óÉÏºÚ£¬2£¬ÏÂ°×
            {
                curr_row = curr_row - 1;
                curr_col = curr_col - 1;
                R_edge_count = R_edge_count + 1;
                dire_right = 7;
                R_edge[i].row = curr_row;
                R_edge[i].col = curr_col;
                R_edge[i].flag = 1;
            }
            else if (dire_right != 1 && black_(image[curr_row - 1][curr_col]) && white_(image[curr_row - 1][curr_col - 1])) //ÕýÉÏºÚ£¬1£¬×ó°×
            {
                curr_row = curr_row - 1;
                R_edge_count = R_edge_count + 1;
                dire_right = 0;
                R_edge[i].row = curr_row;
                R_edge[i].col = curr_col;
                R_edge[i].flag = 1;
            }
            else if (dire_right != 4 && black_(image[curr_row][curr_col + 1]) && white_(image[curr_row - 1][curr_col + 1])) //ÕýÓÒºÚ£¬4£¬ÉÏ°×
            {
                curr_col = curr_col + 1;
                R_edge_count = R_edge_count + 1;
                dire_right = 5;
                R_edge[i].row = curr_row;
                R_edge[i].col = curr_col;
                R_edge[i].flag = 1;
            }
            else if (dire_right != 5 && black_(image[curr_row][curr_col - 1]) && white_(image[curr_row + 1][curr_col - 1])) //Õý×óºÚ£¬5£¬ÏÂ°×
            {
                curr_col = curr_col - 1;
                R_edge_count = R_edge_count + 1;
                dire_right = 4;
                R_edge[i].row = curr_row;
                R_edge[i].col = curr_col;
                R_edge[i].flag = 1;
            }

            else if (dire_right != 7 && black_(image[curr_row + 1][curr_col + 1]) && white_(image[curr_row][curr_col + 1])) //ÓÒÏÂºÚ£¬7£¬ÉÏ°×
            {
                curr_row = curr_row + 1;
                curr_col = curr_col + 1;
                R_edge_count = R_edge_count + 1;
                dire_right = 2;
                R_edge[i].row = curr_row;
                R_edge[i].col = curr_col;
                R_edge[i].flag = 1;
            }
            else if (dire_right != 6 && black_(image[curr_row + 1][curr_col - 1]) && white_(image[curr_row + 1][curr_col])) //×óÏÂºÚ£¬6£¬ÓÒ°×
            {
                curr_row = curr_row + 1;
                curr_col = curr_col - 1;
                R_edge_count = R_edge_count + 1;
                dire_right = 3;
                R_edge[i].row = curr_row;
                R_edge[i].col = curr_col;
                R_edge[i].flag = 1;
            }

            else if (dire_right != 0 && black_(image[curr_row + 1][curr_col]) && white_(image[curr_row + 1][curr_col + 1])) //ÕýÉÏºÚ£¬1£¬ÓÒ°×
            {
                curr_row = curr_row + 1;
                R_edge_count = R_edge_count + 1;
                dire_right = 1;
                R_edge[i].row = curr_row;
                R_edge[i].col = curr_col;
                R_edge[i].flag = 1;
            }
            else
                break;
        }
    }
/*
    //±ß½çÔ¤´¦Àí
    // Ïà¹Ø±äÁ¿³õÊ¼»¯
    pre_L_edge_count = 0;
    pre_R_edge_count = 0;
    // ×ó±ß½ç´¦Àí
    if (left_findflag) //Èç¹û×ó±ß½ç´æÔÚ²¢ÕÒµ½
    {
        //Æ½»¬±ß½ç
        for (int i = 1; i < L_edge_count - 1; i++)
        {
            L_edge[i].row = (L_edge[i].row + L_edge[i + 1].row) / 2;
            L_edge[i].col = (L_edge[i].col + L_edge[i + 1].col) / 2;
        }
        for (int i = L_edge_count - 1; i > 1; i--)
        {
            L_edge[i].row = (L_edge[i].row + L_edge[i - 1].row) / 2;
            L_edge[i].col = (L_edge[i].col + L_edge[i - 1].col) / 2;
        }
        edge_process_flag = 0; //ÇåÁã±êÖ¾
        //½Ø¶ÏÁ¬ÐøË®Æ½µÄ±ß½ç
        /*
        if (L_edge_count >= 70)
        {
            num_cnt = 0; //¼ÇÂ¼Á¬ÐøË®Æ½µãµÄ¸öÊý
            L_count = L_edge_count / 2;
            while (L_count < L_edge_count)
            {
                if (L_edge[L_count].row == L_edge[L_count + 1].row)
                    num_cnt = num_cnt + 1;
                else
                    num_cnt = 0;
                if (num_cnt > 5) //Á¬Ðø5¸öµãË®Æ½
                    break;
                L_count = L_count + 1;
            }
            L_edge_count = L_count; //½Ø¶ÏÔÚ5¸öË®Æ½µã´¦
        }*//*
    }
    //ÓÒ±ß½ç´¦Àí
    if (right_findflag) //ÓÒ±ß½ç´æÔÚÇÒÕÒµ½Ê±¿ªÊ¼´¦Àí
    {
        //Æ½»¬±ß½ç
        for (int i = 1; i < R_edge_count - 1; i++)
        {
            R_edge[i].row = (R_edge[i].row + R_edge[i + 1].row) / 2;
            R_edge[i].col = (R_edge[i].col + R_edge[i - 1].col) / 2;
        }
        for (int i = R_edge_count - 1; i > 1; i--)
        {
            R_edge[i].row = (R_edge[i].row + R_edge[i + 1].row) / 2;
            R_edge[i].col = (R_edge[i].col + R_edge[i - 1].col) / 2;
        }
        //½Ø¶ÏÁ¬ÐøË®Æ½µÄ±ß½ç
        /*
        if (R_edge_count >= 70) //±ß½ç¹»³¤Ê±²Å¿ªÊ¼´¦Àí
        {
            num_cnt = 0;
            R_count = R_edge_count / 2;
            while (R_count < R_edge_count)
            {
                if (R_edge[R_count].row == R_edge[R_count + 1].row)
                    num_cnt = num_cnt + 1;
                else
                    num_cnt = 0;
                if (num_cnt > 5)
                    break;
                R_count = R_count + 1;
            }
            R_edge_count = R_count;
        }*//*
    }

    //¼ì²âÒì³£±ß½ç
    // Çé¿ö1£º×ó¡¢ÓÒ±ßÏßºá¿çÍ¼Ïñ¾ù³¬¹ýWidth/2£¬¿¼ÂÇ×öÓÒ±ßÏßÅÀÏßÖØºÏµÄÎÊÌâ£¬Ê¹ÄÜÖØºÏ¼ìË÷
    if (fabs(L_edge[0].col - L_edge[L_edge_count].col) > IMGW / 2 && fabs(L_edge[0].col - L_edge[L_edge_count].col) > IMGW / 2)
        enable_check_l_r_edge_same = 1;
    if (enable_check_l_r_edge_same)
    {
        int i_left = L_edge_count - 1;
        int chongdie_cnt = 0;
        for (int i = 0; i < R_edge_count; i++)
        {
            if (fabs(R_edge[i].row - L_edge[i_left].row) < 5 && fabs(R_edge[i].col - L_edge[i_left].col) < 5)
            {
                chongdie_cnt = chongdie_cnt + 1;
                i_left = i_left - 1;
                if (chongdie_cnt > 3)
                    break;
                if (i_left < 0)
                    break;
            }
        }
        if (chongdie_cnt >= 2)
        {
            x2 = fabs(L_edge[0].row - R_edge[0].row);
            //Çé¿ö1£º´óÍäµÀ-ÌØÕ÷£º×óÓÒ±ß½çËÑÏßÆðÊ¼µãÐÐÖµ²î´ó
            if (fabs(L_edge[0].row - R_edge[0].row) >= 15)
            {
                if (L_edge[0].row > R_edge[0].row) //×óËÑÏßµã¸ß£¬¿¼ÂÇ×ó±ßÏß¶ªÏß£¬×óÏòÍäµÀ
                    left_findflag = 0;
                else if (R_edge[0].row > L_edge[0].row) //ÓÒËÑÏßµã¸ß£¬¿¼ÂÇÓÒ±ßÏß¶ªÏß£¬ÓÒÏòÍäµÀ
                    right_findflag = 0;
            }
            //Çé¿ö2£º¿¼ÂÇÈý²æ/±Õ»·Ê®×Ö¡ªÌØÕ÷£º¿¼ÂÇÓÐÎÞ¹Õµã£¨Èý²æÂ·¿Ú±äÏÖÖØºÏÓÐ¹Õµã¡¢±Õ»·Ê®×ÖÎÞ¹Õµã£©
            else if (fabs(L_edge[0].row - R_edge[0].row) < 15)
            {
            }
        }
    }
    //±ß½çµãÌ«ÉÙ£¬È¥µô

    //×ó±ßÏßÆðÊ¼µã´óÓÚÓÒ±ßÏß
    if (left_findflag && right_findflag)
    {
        if (jilu_col_l > jilu_col_r)
        {
            if (jilu_row_l > jilu_row_r)
                left_findflag = 0;
            else if (jilu_row_l < jilu_row_r)
                right_findflag = 0;
        }
    }
    /*
    if (L_edge_count - R_edge_count > 30 && right_findflag) //×óÓÒ±ß½çÊýÁ¿²î¾àÌ«´ó£¬×ó>>ÓÒ
    {
        right_findflag = 0;
        R_edge_count = 0;
    }
    if (R_edge_count - L_edge_count > 30 && left_findflag) //ÓÒ>>×ó
    {
        left_findflag = 0;
        L_edge_count = 0;
    }*/
    //Èç¹ûÓÒ/ÓÒ±ß½çÌ«ÍùÉÏÇÒµãÊý²î¾à½Ï´ó£¬É¾³ý±ßÏß
/*
    if (left_findflag && right_findflag)
    {
        if (jilu_row_r - jilu_row_l > IMGH / 2 && L_edge_count - R_edge_count > 10)
        {
            right_findflag = 0;
            R_edge_count = 0;
        }
        else if (jilu_row_l - jilu_row_r > IMGH / 2 && R_edge_count - L_edge_count > 10)
        {
            left_findflag = 0;
            L_edge_count = 0;
        }
    }
    if (L_edge[L_edge_count - 1].row - L_edge[1].row > -10) //±ßÏß¼¸ºõË®Æ½ ×ó
        left_findflag = 0;
    if (R_edge[R_edge_count - 1].row - R_edge[1].row > -10) //ÓÒ
        right_findflag = 0;
    if (L_edge_count < min_count + 3)
        left_findflag = 0;
    if (R_edge_count < min_count + 3)
        right_findflag = 0;
    //´æÔÚÄ³Ò»±ß½ç£¬¿ªÆôÖÐÏßÄâºÏ*/
    if (left_findflag == 0)
        L_edge_count = 0;
    if (right_findflag == 0)
        R_edge_count = 0;
    if (left_findflag || right_findflag)
        enable_midline = 1;
    else
        enable_midline = 0;



}

/*--------------------------------------------------------------------------
 * ¡¾º¯Êý¹¦ÄÜ¡¿£º³õÊ¼»¯×ó±ß½çÓÒ±ß½ç½á¹¹Ìå
 * ¡¾²Î    Êý¡¿£ºÎÞ
 * ¡¾·µ »Ø Öµ¡¿£ºÎÞ
 *--------------------------------------------------------------------------*/
void clear_point()
{
    for (int i = 0; i < L_edge_count; i++)
    {
        L_edge[i].row = 0;
        L_edge[i].col = 0;
        L_edge[i].flag = 0;
    }
    for (int i = 0; i < R_edge_count; i++)
    {
        R_edge[i].row = 0;
        R_edge[i].col = 0;
        R_edge[i].flag = 0;
    }
}

/*--------------------------------------------------------------------------
 * ¡¾º¯Êý¹¦ÄÜ¡¿£ºÅÐ¶ÏÊÇ·ñ´æÔÚ×ó/ÓÒ±ß½çµã
 * ¡¾²Î    Êý¡¿£ºËÑÏß»ù±¾ÐÐÊýÖµ      ×ó/ÓÒ±ß½çÑ¡Ôñ±êÖ¾(0:×ó±ß£¬1:ÓÒ±ß)
 * ¡¾·µ »Ø Öµ¡¿£º-1 »ò ±ß½çµãÁÐÖµ£¬-1±íÊ¾Ã»ÓÐÕÒµ½±ß½çµã
 * ¡¾±¸    ×¢¡¿£ºÎÞ
 *--------------------------------------------------------------------------*/
int edge_point_ornot(unsigned char image[][IMGW], int row, int side)
{
    //×ó±ßÏß
    if (side == 0)
    {
        int find_edge = 0;
        //´Ó¿ªÊ¼ËÑÏßÐÐÏòÉÏÉú³¤
        /*for (int rowi = row; rowi > 0; rowi--)
        {
            //Èç¹ûÍ¼Ïñ×î×ó²àÎªÈüµÀÍâ£¬Ôò¿ªÊ¼ºáÏòÉú³¤
            if (black_(image[rowi][3]))
            {
                find_edge = find_edge + 1;
                //ºáÏòÉú³¤
                for (int col = 2; col < IMGW - 5; col++)
                {
                    //Èç¹û³öÏÖºÚºÚ°×°×£¬ÔòÅÐ¶ÏÎª±ß½çÏß£¬ÍË³öÑ­»·
                    if (black_(image[rowi][col]) && black_(image[rowi][col + 1]) && white_(image[rowi][col + 2]) && white_(image[rowi][col + 3]))
                    {
                        L_basic_row_start = rowi; //¸³Öµ¿ªÊ¼ËÑÏßÐÐ£¨×ó£©
                        if (rowi < IMGH / 2 && col > IMGW / 2)
                            return -1;
                        return col + 1; //·µ»ØÁÐÖµ
                    }
                }
            }
            if (find_edge == 2)
                return -1;
        }*/
        // return -1;
                for (int rowi = row; rowi > 0; rowi--)
                {

                        for (int col = IMGW/2; col >  1; col--)
                        {
                            //Èç¹û³öÏÖºÚºÚ°×°×£¬ÔòÅÐ¶ÏÎª±ß½çÏß£¬ÍË³öÑ­»·
                            if (black_(image[rowi][col]) && black_(image[rowi][col + 1]) && white_(image[rowi][col + 2]) && white_(image[rowi][col + 3]))
                            {
                                L_basic_row_start = rowi; //¸³Öµ¿ªÊ¼ËÑÏßÐÐ£¨×ó£©
                                if (rowi < IMGH / 2 && col > IMGW / 2)
                                    return -1;
                                return col + 1; //·µ»ØÁÐÖµ
                            }
                        }
                }


                return -1;
    }
    //ÓÒ±ßÏß
    if (side == 1)
    {
        int find_edge = 0;
        /*for (int rowi = row; rowi > 0; rowi--)
        {
            if (black_(image[rowi][IMGW - 4]))
            {
                find_edge = find_edge + 1;
                for (int col = IMGW - 4; col > 5; col--)
                {
                    if (white_(image[rowi][col - 3]) && white_(image[rowi][col - 2]) && black_(image[rowi][col - 1]) && black_(image[rowi][col]))
                    {
                        R_basic_row_start = rowi;
                        if (rowi < IMGH / 2 && col < IMGW / 2)
                            return -1;
                        return col - 1;
                    }
                }
            }
            if (find_edge == 2)
                return -1;
        }*/
        for (int rowi = row; rowi > 0; rowi--)
                {

                        for (int col = IMGW/2 ; col < IMGW-1; col++)
                        {
                            if (white_(image[rowi][col - 3]) && white_(image[rowi][col - 2]) && black_(image[rowi][col - 1]) && black_(image[rowi][col]))
                            {
                                R_basic_row_start = rowi;
                                if (rowi < IMGH / 2 && col < IMGW / 2)
                                    return -1;
                                return col - 1;
                            }
                        }

                }
        // return -1;
    }
    return -1;
}

/*--------------------------------------------------------------------------
 * ¡¾º¯Êý¹¦ÄÜ¡¿£ºÅÐ¶ÏÊÇ·ñÊÇºÚÏñËØ/°×ÏñËØµã£¬ÎªºóÆÚ²»ÊÊÓÃ¶þÖµ»¯Í¼Ïñ×ö×¼±¸
 * ¡¾²Î    Êý¡¿£º¸ÃµãµÄÏñËØÖµ
 * ¡¾·µ »Ø Öµ¡¿£ºÎÞ
 * ¡¾±¸    ×¢¡¿£ºÎÞ
 *--------------------------------------------------------------------------*/
int black_(int x) //ÅÐ¶ÏºÚ
{

    if (x == 0)
        return 1;
    else
        return 0;
}

int white_(int x) //ÅÐ¶Ï°×
{

    if (x == 1)
        return 1;
    else
        return 0;
}

/*--------------------------------------------------------------------------
 * ¡¾º¯Êý¹¦ÄÜ¡¿£ºÇó¹ÕµãµÄ½Ç¶ÈÖµ
 * ¡¾²Î    Êý¡¿£ºÈý¸öµãµÄÐÐÁÐ×ø±ê
 * ¡¾·µ »Ø Öµ¡¿£º½Ç¶ÈÖµ
 * ¡¾±¸    ×¢¡¿£ºÎÞ
 *--------------------------------------------------------------------------*/
int Get_angle(int ax, int ay, int bx, int by, int cx, int cy)
{
    int abx = -ax + bx;
    int aby = -ay + by;
    int cbx = cx - bx;
    int cby = cy - by;
    int ab_muti_cb = abx * cbx + aby * cby;
    float dist_ab = sqrt(abx * abx + aby * aby);
    float dist_cb = sqrt(cbx * cbx + cby * cby);
    float cosvalue = ab_muti_cb / (dist_ab * dist_cb);
    return (int)(acos(cosvalue) * 180 / 3.14);
}

/*--------------------------------------------------------------------------
 * ¡¾º¯Êý¹¦ÄÜ¡¿£º»ñÈ¡ÖÐÏß
 * ¡¾²Î    Êý¡¿£ºÎÞ
 * ¡¾·µ »Ø Öµ¡¿£ºÎÞ
 * ¡¾±¸    ×¢¡¿£ºÎÞ
 *--------------------------------------------------------------------------*/
int size1 = 0;
void get_mid(unsigned char image[][IMGW])
{
    size1 = 1;
    //Ïà¹Ø±äÁ¿
    int mid_cnt = 0;        //ÓÃÓÚÏòÏÂÄâºÏ£¬Èç¹û¿ªÆôÄâºÏ£¬ÔòÓÃÓÚ¼ÇÂ¼ÐÂµÄÖÐÏßµã¸öÊý
    int last_mid_count = 0; //ÓÃÓÚ¼ÇÂ¼ÉÏÒ»³¡ÖÐÏßµÄÖÐÏß¸öÊý£¬ÓÃÓÚÖÐÏßÆ½»º
    int up_cnt = 15;        //ÏòÉÏÉú³¤µÄÏÞÖÆ¸öÊý
    //±£´æÉÏÒ»³¡ÖÐÏßÊý¾Ý
    for (int i = 0; i < Mid_count; i++)
    {
        Last_M_Line[i].row = Mid_Line[i].row;
        Last_M_Line[i].col = Mid_Line[i].col;
    }
    last_mid_count = Mid_count;
    //Çé¿ö1£º×óÓÒ¾ùÕÒµ½
    if (left_findflag && right_findflag)
    {
        size1 = 2;
        //³õÊ¼»¯ÖÐÏß¸öÊý
        Mid_count = 0;
        //·ÀÖ¹ÖÐÏß³ö´í
        L_edge[0].row = L_edge[1].row;
        L_edge[0].col = L_edge[1].col;
        R_edge[0].row = R_edge[1].row;
        R_edge[0].col = R_edge[1].col;
        //×ó±ßÏß±ÈÓÒ±ßÏß³¤
        if (L_edge_count >= R_edge_count)
        {
            float k = 1.0 * L_edge_count / R_edge_count;
            for (int i = 0; i < R_edge_count; i++)
            {
                Mid_count = Mid_count + 1;
                M_Line[i].row = (int)((L_edge[(int)(k * i)].row + R_edge[i].row) / 2);
                M_Line[i].col = (int)((L_edge[(int)(k * i)].col + R_edge[i].col) / 2);
                M_Line[i].flag = searchpoint;
            }
        }
        //ÓÒ±ßÏß±È×ó±ßÏß³¤
        else if (L_edge_count < R_edge_count)
        {
            float k = 1.0 * R_edge_count / L_edge_count;
            for (int i = 0; i < L_edge_count; i++)
            {
                Mid_count = Mid_count + 1;
                M_Line[i].row = (int)((L_edge[i].row + R_edge[(int)(k * i)].row) / 2);
                M_Line[i].col = (int)((L_edge[i].col + R_edge[(int)(k * i)].col) / 2);
                M_Line[i].flag = searchpoint;
            }
        }
    }
    //Çé¿ö2£ºµ¥±ßÈ«¶ªÏß
    //      ÓÒ±ß½ç¶ªÏß£¬×ó±ß½ç²»¶ªÏß||±ßÏßÊýÁ¿²î¾à´ó
    else if ((left_findflag == 1 && right_findflag == 0) || (left_findflag && L_edge_count - R_edge_count > 70))
    {
        //·ÀÖ¹ÖÐÏß³ö´í
        L_edge[0].row = L_edge[1].row;
        L_edge[0].col = L_edge[1].col;
        //³õÊ¼»¯ÖÐÏß¸öÊý
        Mid_count = 0;
        for (int i = 0; i < L_edge_count; i++)
        {
            int col = ((2 * (int)L_edge[i].col + Road_Width) / 2);
            if (col > IMGW - 1)
                col = IMGW - 1;
            else if (col < 0)
                col = 0;
            Mid_count = Mid_count + 1;
            M_Line[i].row = L_edge[i].row;
            M_Line[i].col = (int)col;
            M_Line[i].flag = searchpoint;
        }
    }
    //      ×ó±ßÏß¶ªÏß£¬ÓÒ±ßÏß²»¶ªÏß
    else if ((left_findflag == 0 && right_findflag == 1) || (right_findflag && R_edge_count - L_edge_count > 70))
    {
        //·ÀÖ¹ÖÐÏß³ö´í
        R_edge[0].row = R_edge[1].row;
        R_edge[0].col = R_edge[1].col;
        //³õÊ¼»¯ÖÐÏß¸öÊý
        Mid_count = 0;
        for (int i = 0; i < R_edge_count; i++)
        {
            int col = ((2 * (int)R_edge[i].col - Road_Width) / 2);
            if (col > IMGW - 1)
                col = IMGW - 1;
            else if (col < 0)
                col = 0;
            Mid_count = Mid_count + 1;
            M_Line[i].row = R_edge[i].row;
            M_Line[i].col = (int)col;
            M_Line[i].flag = searchpoint;
        }
    }
    //Çé¿ö3£ºÁ½±ßÈ«¶ªÏß
    //Çé¿ö4£º×óÓÒ±ß¾ù´æÔÚ£¬µ«ÖÐÏßÌ«ÉÙ
    if (Mid_count > 0 && 0)
    {
        int down_cnt = 15;
        //³µÍ·¸½½üÈüµÀÃ»ÕÒµ½£¬ÔòÏòÏÂÄâºÏ
        if (M_Line[0].row < IMGH - 2 && M_Line[0].row > IMGH / 4 && Mid_count > 2)
        {
            int num = Mid_count / 4;
            int sumX = 0, sumY = 0;
            float sumUP = 0, sumDown = 0, avrX = 0, avrY = 0, K, B;
            for (int i = 0; i < num; i++)
            {
                sumX = sumX + M_Line[i].row;
                sumY = sumY + M_Line[i].col;
            }
            avrX = 1.0 * sumX / num;
            avrY = 1.0 * sumY / num;
            for (int i = 0; i < num; i++)
            {
                sumUP = sumUP + (M_Line[i].col - avrY) * (M_Line[i].row - avrX);
                sumDown = sumDown + (M_Line[i].row - avrX) * (M_Line[i].row - avrX);
            }
            if (sumDown == 0)
            {
                K = 0;
            }
            else
            {
                K = 1.0 * sumUP / sumDown;
                B = 1.0 * (sumY - K * sumX) / num;
            }
            for (int i = M_Line[0].row; i < IMGH; i++) //¿ªÊ¼ÏòÏÂÄâºÏ
            {
                int col = K * i + B;
                if (col > IMGW - 1)
                    col = IMGW - 1;
                else if (col < 0)
                    col = 0;
                MID_LINE[mid_cnt].row = (int)i;
                MID_LINE[mid_cnt].col = (int)col;
                mid_cnt = mid_cnt + 1;
                if (--down_cnt == 0)
                    break;
            }
        }
        //ÖÐÏßµã¸öÊýÌ«ÉÙ£¬¿ªÆôÏòÉÏÄâºÏ
        if (Mid_count + mid_cnt < 60 && Mid_count > 2)
        {
            int num = Mid_count / 4;
            int sumX = 0, sumY = 0;
            float sumUP = 0, sumDown = 0, avrX = 0, avrY = 0, K, B;
            for (int i = Mid_count - num - 1; i < Mid_count; i++)
            {
                sumX = sumX + M_Line[i].row;
                sumY = sumY + M_Line[i].col;
            }
            avrX = 1.0 * sumX / num;
            avrY = 1.0 * sumY / num;
            for (int i = Mid_count - num - 1; i < Mid_count; i++)
            {
                sumUP = sumUP + (M_Line[i].col - avrY) * (M_Line[i].row - avrX);
                sumDown = sumDown + (M_Line[i].row - avrX) * (M_Line[i].row - avrX);
            }
            if (sumDown == 0)
            {
                K = 0;
            }
            else
            {
                K = 1.0 * sumUP / sumDown;
                B = 1.0 * (sumY - K * sumX) / num;
            }
            for (int i = M_Line[Mid_count - 1].row; i > 0; i--) //¿ªÊ¼ÏòÉÏÄâºÏ
            {
                int col = K * i + B;
                if (col > IMGW - 1)
                    col = IMGW - 1;
                else if (col < 0)
                    col = 0;
                M_Line[Mid_count].row = (int)i;
                M_Line[Mid_count].col = (int)col;
                Mid_count = Mid_count + 1;
                if (--up_cnt == 0)
                    break;
            }
        }
        //Èç¹ûÏòÏÂÄâºÏÁË£¬Ôò°áÔËÖÐÏß´úÂëÖÁ
        if (mid_cnt > 0)
        {
            for (int i = 0; i < Mid_count; i++)
            {
                MID_LINE[mid_cnt].row = M_Line[i].row;
                MID_LINE[mid_cnt].col = M_Line[i].col;
                mid_cnt = mid_cnt + 1;
            }
        }
    }
    if (mid_cnt > 0)
    {
        Mid_count = mid_cnt;
        Mid_Line = MID_LINE;
    }
    else
        Mid_Line = M_Line;
    //ÖÐÏßÆ½»º
    float k = 0.8;
    float k2;
    for (int i = 0; i < Mid_count - 1; i++)
    {
        Mid_Line[i].row = (Mid_Line[i].row + Mid_Line[i + 1].row) / 2;
        Mid_Line[i].col = (Mid_Line[i].col + Mid_Line[i + 1].col) / 2;
    }
    for (int i = Mid_count - 1; i > 1; i--)
    {
        Mid_Line[i].row = (Mid_Line[i].row + Mid_Line[i - 1].row) / 2;
        Mid_Line[i].col = (Mid_Line[i].col + Mid_Line[i - 1].col) / 2;
    }
    if (last_mid_count >= Mid_count)
    {
        k2 = 1.0 * last_mid_count / Mid_count;
        for (int i = 0; i < Mid_count; i++)
        {
            Mid_Line[i].row = (1 - k) * Last_M_Line[(int)(k2 * i)].row + k * Mid_Line[i].row;
            Mid_Line[i].col = (1 - k) * Last_M_Line[(int)(k2 * i)].col + k * Mid_Line[i].col;
        }
    }
    else if (last_mid_count < Mid_count)
    {
        k2 = 1.0 * Mid_count / last_mid_count;
        for (int i = 0; i < last_mid_count; i++)
        {
            Mid_Line[i].row = (1 - k) * Last_M_Line[i].row + k * Mid_Line[(int)(k2 * i)].row;
            Mid_Line[i].col = (1 - k) * Last_M_Line[i].col + k * Mid_Line[(int)(k2 * i)].col;
        }
    }

    // L_edge_count=ercinh(L_edge,L_edge_count);

    int i, j;
    for (i = 0; i < IMGH; i++)
        for (j = 0; j < IMGW; j++)
            output[i][j] = 0;
    for (i = 0; i <= L_edge_count; i++)
        output[L_edge[i].row][L_edge[i].col] = 1;
    for (i = 0; i <= R_edge_count; i++)
        output[R_edge[i].row][R_edge[i].col] = 1;
    for (i = 0; i <= Mid_count; i++)
        output[Mid_Line[i].row][Mid_Line[i].col] = 1;
}

int ercinh(struct EDGE *Line, int count)
{
    int i = 0;

    double N1 = 1e-13;
    double al, bl, cl, m1, m2, m3, z1, z2, z3;
    al = bl = cl = 0;
    double sumx = 0, sumx2 = 0, sumx3 = 0, sumx4 = 0, sumy = 0, sumxy = 0, sumx2y = 0;

    for (i = 0; i <= count; i++)
    {
        sumx += Line[i].row;
        sumy += Line[i].col;
        sumx2 += Line[i].row * Line[i].row;
        sumxy += Line[i].row * Line[i].col;
        sumx3 += Line[i].row * Line[i].row * Line[i].row;
        sumx2y += Line[i].row * Line[i].row * Line[i].col;
        sumx4 += Line[i].row * Line[i].row * Line[i].row * Line[i].row;
    }
    do
    {
        m1 = al;
        al = (sumx2y - sumx3 * bl - sumx2 * cl) / sumx4;
        z1 = (al - m1) * (al - m1);
        m2 = bl;
        bl = (sumxy - sumx * cl - sumx3 * al) / sumx2;
        z2 = (bl - m2) * (bl - m2);
        m3 = cl;
        cl = (sumy - sumx2 * al - sumx * bl) / i;
        z3 = (cl - m3) * (cl - m3);
    } while ((z1 > N1) || (z2 > N1) || (z3 > N1));
    //printf("a=%9.6f,\nb=%9.6f,\nc=%9.6f\n", al, bl, cl);
    //printf("ÄâºÏ·½³ÌÎª   y=%9.6fx*x+%9.6fx+%9.6f\n", al, bl, cl);
    int y;
    int end = Line[count].row;
    int c = 0;
    for (i = Line[0].row; i >= end; i--)
    {
        y = (int)(al * i * i + bl * i + cl + 0.5);
        Line[c].row = i;
        Line[c].col = y;
        c++;
    }
    return c;
}

float get_pure_bias()
{
    float L = 15.0;             //车身轴距
    float dir_bias_angle = 0.0; //方向差角度
    int row = 0;
    int col = 0;

    float s;
    int k1=row_known_geti(K_MD,Mid_Line,Mid_count);
    int k2=row_known_geti(K_MD-10,Mid_Line,Mid_count);
    int k3=row_known_geti(K_MD+10,Mid_Line,Mid_count);
    int k4=row_known_geti(K_MD-15,Mid_Line,Mid_count);
    int k5=row_known_geti(K_MD-10,Mid_Line,Mid_count);
    int k6=row_known_geti(K_MD,Mid_Line,Mid_count);
    //int k = (int)(K_MD * Mid_count);
    if (k2 == 0)
        return 0;
    row = 0.2*Mid_Line[k1].row+0.3*Mid_Line[k2].row+0.2*Mid_Line[k3].row+0.1*Mid_Line[k4].row+0.1*Mid_Line[k5].row+0.1*Mid_Line[k6].row;
    col = 0.2*Mid_Line[k1].col+0.3*Mid_Line[k2].col+0.2*Mid_Line[k3].col+0.1*Mid_Line[k4].col+0.1*Mid_Line[k5].col+0.1*Mid_Line[k6].col;

    dir_bias_angle = atan((col - IMGW / 2) / (row + L));
    s = pid_solve(p_pama, dir_bias_angle);
    // printf("%.5f",dir_bias_angle);

    return s;
}
// 增量式PID
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MINMAX(input, low, upper) MIN(MAX(input, low), upper)

float pid_solve(struct pid_param_t *pid, float error)
{
    pid->pre_pre_error = pid->pre_pre_error + error;
    //pid->out_p = MINMAX(pid->kp * error, -pid->d_max, pid->d_max);
    pid->out_p = pid->kp * error;
    pid->out_d = MINMAX(pid->kd * (error - pid->pre_error), -pid->p_max, pid->p_max);
    pid->out_i = MINMAX(pid->ki * pid->pre_pre_error, -pid->i_max, pid->i_max);
    pid->pre_error = error;
    // printf("%.5f",pid->out_p + pid->out_i + pid->out_d);
    return pid->out_p+pid->out_d;
}
void pid_init()
{
    pama.kp = 35;      // P记得幅值
    pama.ki = 0;       // I
    pama.kd = 30;      // D
    pama.i_max = 1000; // integrator_max
    pama.p_max = 1000; // integrator_max
    pama.d_max = 1000; // integrator_max
    pama.pre_error = 0;
    pama.pre_pre_error = 0;
}
int motor_PID(int ECPULSE_SET, int ECPULSE_REAL, int *error_last_1, int *error_last_2, float KP, float KI, float KD, int *duty)
{
    int error;
    error = ECPULSE_SET - ECPULSE_REAL;
    *duty += (int)(KP * (error - (*error_last_1)) + KI * error + KD * (error - 2 * (*error_last_1) + (*error_last_2)));
    *error_last_2 = *error_last_1;
    *error_last_1 = error;
    if (*duty > duty_max)
    {
        *duty = duty_max;
    }
    if (*duty < 0)
    {
        *duty = 0;
    }
    return *duty;
}

int checkcorner(struct EDGE line[],int i)
{
    int delt=10;
    int dx=2*line[i].row-line[i-delt].row-line[i+delt].row;
    int dy=2*line[i].col-line[i-delt].col-line[i+delt].col;

    if(dx>0&&dy>0)
        return 1;//ZS
    if(dx>0&&dy<0)
        return 2;//YS
    if(dx<0&&dy>0)
        return 3;//ZX
    if(dx<0&&dy<0)
        return 4;//YX
}

void line_check()
{

    //int temp;
    //int delt = 10;
    Lc.corner_flag = 0; // 初始化变量
    Rc.corner_flag = 0; //初始化变量
    uLc.corner_flag = 0; // 初始化变量
    uRc.corner_flag = 0;
    //检测拐点

    Lc.corner_row = 0;
    Lc.corner_col = 0;
    Lc.corner_angle = 0;
    Rc.corner_row = 0;
    Rc.corner_col = 0;
    Rc.corner_angle = 0;
    //左线；

    if (enable_L_corner) //如果使能搜索左拐点
    {
        corner_find(L_edge, 1, &Lc, L_edge_count);
        if(Lc.corner_flag&&checkcorner(L_edge,Lc.corner_icount)!=3)
            Lc.corner_flag=0;

        corner_find(L_edge, 2, &uLc, L_edge_count);
        if(uLc.corner_flag&&checkcorner(L_edge,uLc.corner_icount)!=1)
            uLc.corner_flag=0;
        //if(L_edge[uLc.corner_icount+6].row<L_edge[uLc.corner_icount].row&&L_edge[uLc.corner_icount-6].col<L_edge[uLc.corner_icount].col)
        //    uLc.corner_flag=1;
        //else
        //    uLc.corner_flag=0;

    }
    if (enable_R_corner) //如果使能搜索右拐点
    {
        corner_find(R_edge, 2, &uRc, R_edge_count);
        if(uRc.corner_flag&&checkcorner(R_edge,uRc.corner_icount)!=2)
            uRc.corner_flag=0;



        corner_find(R_edge, 1, &Rc, R_edge_count);

        if(Rc.corner_flag&&checkcorner(R_edge,Rc.corner_icount)!=4)
            Rc.corner_flag=0;
    }
    if (LEFT_EDGE_losscount < min_count + 3 && LEFT_EDGE_losscount > min_count-3)
        LEFT_EDGE_type = 0;
    else if (L_edge[L_edge_count].row > 50)
        LEFT_EDGE_type = 1;
    else if (L_edge[L_edge_count].col - L_edge[0].col > IMGW / 4)
        LEFT_EDGE_type = 2;
    else
    {

        if (Lc.corner_flag == 0)
            LEFT_EDGE_type = -1;
        else
        {
            if(L_edge[Lc.corner_icount+6].col<L_edge[Lc.corner_icount-6].col)
                LEFT_EDGE_type = 3;
            else
                LEFT_EDGE_type = 4;

            /*if (Lc.corner_angle < 75)
                LEFT_EDGE_type = 3;
            else
                LEFT_EDGE_type = 4;*/
        }
    }

    if (RIGHT_EDGE_losscount < min_count + 3 && RIGHT_EDGE_losscount > min_count-3)
        RIGHT_EDGE_type = 0;
    else if (R_edge[R_edge_count].row > 50)
        RIGHT_EDGE_type = 1;
    else if (R_edge[R_edge_count].col - R_edge[0].col < -IMGW / 4)
        RIGHT_EDGE_type = 2;
    else
    {

        if (Rc.corner_flag == 0)
            RIGHT_EDGE_type = -1;
        else
        {

            if(R_edge[Rc.corner_icount+6].col>R_edge[Rc.corner_icount-6].col)
                RIGHT_EDGE_type = 3;
            else
                RIGHT_EDGE_type = 4;
            /*if (Rc.corner_angle < 75)
                RIGHT_EDGE_type = 3;
            else
                RIGHT_EDGE_type = 4;*/
        }

    }

    //if(RIGHT_EDGE_type==3||RIGHT_EDGE_type==0||LEFT_EDGE_type==3||LEFT_EDGE_type==0)
    if(inelement==0)
    {




        event_check();
        if(inelement==0)
        {
            if (LEFT_EDGE_losscount < min_count + 3 && LEFT_EDGE_losscount > min_count - 3||RIGHT_EDGE_type==2)
            {
                left_findflag = 0;
                LEFT_EDGE_type = 0;
            }

            if (RIGHT_EDGE_losscount < min_count + 3 && RIGHT_EDGE_losscount > min_count - 3||LEFT_EDGE_type==2)
            {
                right_findflag = 0;
                RIGHT_EDGE_type = 0;
            }

            //´æÔÚÄ³Ò»±ß½ç£¬¿ªÆôÖÐÏßÄâºÏ*/
            if (LEFT_EDGE_type == 0)
                L_edge_count = 0;
            if (RIGHT_EDGE_type == 0)
                R_edge_count = 0;
        }
    }
    else
    {
        process_element();
    }
}

void corner_find(struct EDGE line[], int dir, struct CORNER *cor, int count)
{
    int delt = 6;
    int temp = 0;
    if (count > 2 * delt + 1)
    {
        if (dir == 1)
        {
            for (int i = 0; i < count - 2 * delt - 1; i++)
            {
                if (line[i + delt].row > 1)
                {

                    // temp = (R_edge[i+delt].col - R_edge[i ].col) * (R_edge[i + delt*2].col - R_edge[i + delt].col) +
                    //        (R_edge[i+delt].row - R_edge[i ].row) * (R_edge[i + delt*2].row - R_edge[i + delt].row);
                    // if (1) //初步确认为锐角或者直角 向量法
                    //{
                    temp = Get_angle(line[i].row, line[i].col, line[i + delt].row, line[i + delt].col, line[i + delt * 2].row, line[i + delt * 2].col); //求角度
                    if (temp > 45)                                                                                                                      // Rc.corner_angle > 40 && Rc.corner_angle < 110
                    {
                        if (cor->corner_flag == 0) //确定拐角朝向，左拐点没有朝向做的
                        {

                            cor->corner_flag = 1; //异常拐点
                            cor->corner_row = line[i + delt].row;
                            cor->corner_col = line[i + delt].col;
                            cor->corner_angle = temp;
                            cor->corner_icount = i + delt;
                            // break;
                        }
                        else if (cor->corner_flag == 1 && temp > cor->corner_angle)
                        {
                            cor->corner_flag = 1; //异常拐点
                            cor->corner_row = line[i + delt].row;
                            cor->corner_col = line[i + delt].col;
                            cor->corner_angle = temp;
                            cor->corner_icount = i + delt;
                        }
                        // if (Get_angle(R_edge[i + 1].row, R_edge[i + 1].col, R_edge[i + delt + 1].row, R_edge[i + delt + 1].col, R_edge[i + delt*2 + 1].row, R_edge[i + delt*2 + 1].col) < Rc.corner_angle && Get_angle(R_edge[i + 2].row, R_edge[i + 2].col, R_edge[i + delt + 2].row, R_edge[i + delt + 2].col, R_edge[i + delt*2 + 2].row, R_edge[i + delt*2 + 2].col) < Rc.corner_angle)
                        // break;
                    }
                    if (temp < 30 && cor->corner_flag == 1)
                        break;
                    //}
                }
            }
        }
        else if (dir == 2)
        {
            for (int i = count - 1; i > 2 * delt; i--)
            {
                if (line[i - delt].row > 1)
                {

                    // temp = (R_edge[i+delt].col - R_edge[i ].col) * (R_edge[i + delt*2].col - R_edge[i + delt].col) +
                    //        (R_edge[i+delt].row - R_edge[i ].row) * (R_edge[i + delt*2].row - R_edge[i + delt].row);
                    // if (1) //初步确认为锐角或者直角 向量法
                    //{
                    temp = Get_angle(line[i].row, line[i].col, line[i - delt].row, line[i - delt].col, line[i - delt * 2].row, line[i - delt * 2].col); //求角度
                    if (temp > 45)                                                                                                                      // Rc.corner_angle > 40 && Rc.corner_angle < 110
                    {
                        if (cor->corner_flag == 0) //确定拐角朝向，左拐点没有朝向做的
                        {

                            cor->corner_flag = 1; //异常拐点
                            cor->corner_row = line[i - delt].row;
                            cor->corner_col = line[i - delt].col;
                            cor->corner_angle = temp;
                            cor->corner_icount = i - delt;
                            // break;
                        }
                        else if (cor->corner_flag == 1 && temp > cor->corner_angle)
                        {
                            cor->corner_flag = 1; //异常拐点
                            cor->corner_row = line[i - delt].row;
                            cor->corner_col = line[i - delt].col;
                            cor->corner_angle = temp;
                            cor->corner_icount = i - delt;
                        }
                        // if (Get_angle(R_edge[i + 1].row, R_edge[i + 1].col, R_edge[i + delt + 1].row, R_edge[i + delt + 1].col, R_edge[i + delt*2 + 1].row, R_edge[i + delt*2 + 1].col) < Rc.corner_angle && Get_angle(R_edge[i + 2].row, R_edge[i + 2].col, R_edge[i + delt + 2].row, R_edge[i + delt + 2].col, R_edge[i + delt*2 + 2].row, R_edge[i + delt*2 + 2].col) < Rc.corner_angle)
                        // break;
                    }
                    if (temp < 30 && cor->corner_flag == 1)
                        break;
                    //}
                }
            }
        }

    }
}

void event_check() //事件判断
{

    if((RIGHT_EDGE_type==3)&&(LEFT_EDGE_type==3))//双拐 三岔或十字
    {
        /*if(LEFT_EDGE_type==3)
        {
            int r=L_edge[Lc.corner_icount].row;
            int c=L_edge[Lc.corner_icount].col;
            if(r>5&&c>5&&ooutput[r-5][c-5]==1&&ooutput[r-4][c-4]==1)
                inelement=1;//后续添加判断
        }*/

        if(RIGHT_EDGE_type==3&&R_edge[Rc.corner_icount].row<10)
            return;
        if(LEFT_EDGE_type==3&&L_edge[Lc.corner_icount].row<10)
            return;
        int flag=szorcd_check();
        if(flag==3&&45<Lc.corner_angle&&Lc.corner_angle<90&&45<Rc.corner_angle&&Lc.corner_angle<90)
        inelement=3;
        if(flag==1&&60<Lc.corner_angle&&Lc.corner_angle<120&&60<Rc.corner_angle&&Rc.corner_angle<120)
        inelement=1;
        return;
    }
    if((LEFT_EDGE_type==-1||LEFT_EDGE_type==0)&&RIGHT_EDGE_type==3)//车库或圆环
    {
        if(Rc.corner_row>35)
            if(Lc.corner_flag==0&&uLc.corner_flag==0)
                inelement=2;
        return;
    }

}
int row_known_geti(int row,struct EDGE line[], int count)
{
    int i=0;
    while(i<count&&line[i].row>row)
    {
        i++;
    }
    if(i>=count)
        i=count-1;
    if(i<0)
        i=0;
    return i;
}

void process_element()
{
    switch(inelement)
    {
        case 1:process_sz();break;

        case 2:process_yh();break;

        case 3:process_cd();break;
    }

}
void process_sz()
{
    static int szstate;
    if(szstate==0)
    {   //SPEED_SET=550;
        if(LEFT_EDGE_type==0&&RIGHT_EDGE_type==0)
            {szstate=1;}
    }

    if(szstate==1)
    {
        if(LEFT_EDGE_type==2)
        {
            szstate=2;
        }
    }
    if(szstate==2)
    {
        if(Lc.corner_flag&&Lc.corner_angle>60&&Lc.corner_angle<120&&Lc.corner_row>20)
        {
            szstate=3;
        }
    }
    if(szstate==3)
    {
        if(LEFT_EDGE_type==0&&LEFT_EDGE_type==0)
            {szstate=4;}
    }
    if(szstate==4)
    {
        if(LEFT_EDGE_type==-1&&LEFT_EDGE_type==-1)
            {szstate=5;}
    }

    if(szstate==0||szstate==1)
    {
        if(Lc.corner_flag&&uLc.corner_flag)
        {
            spot2spot(ooutput,Lc.corner_row,Lc.corner_col,uLc.corner_row,uLc.corner_col);
        }
        else if(Lc.corner_flag&&!uLc.corner_flag)
        {
            spot2spot(ooutput,Lc.corner_row,Lc.corner_col,2,Lc.corner_col);
        }
        else if(!Lc.corner_flag&&uLc.corner_flag)
        {
            spot2spot(ooutput,uLc.corner_row,uLc.corner_col,57,21);
        }
        else
        {
            spot2spot(ooutput,2,25,58,25);
        }
        if(Rc.corner_flag&&uRc.corner_flag)
        {
            spot2spot(ooutput,Rc.corner_row,Rc.corner_col,uRc.corner_row,uRc.corner_col);
        }
        else if(Rc.corner_flag&&!uRc.corner_flag)
        {
            spot2spot(ooutput,Rc.corner_row,Rc.corner_col,2,Rc.corner_col);
        }
        else if(!Rc.corner_flag&&uRc.corner_flag)
        {
            spot2spot(ooutput,uRc.corner_row,uRc.corner_col,57,66);
        }
        else
        {
            spot2spot(ooutput,2,60,58,60);
        }
        Image_process(ooutput);
    }
    if(szstate==2)
    {
        right_findflag = 0;
        RIGHT_EDGE_type = 0;
        R_edge_count = 0;
    }
    if(szstate==3)
    {
        if(Lc.corner_flag&&uLc.corner_flag)
        {
            spot2spot(ooutput,Lc.corner_row,Lc.corner_col,uLc.corner_row,uLc.corner_col);
        }
        else if(Lc.corner_flag&&!uLc.corner_flag)
        {
            spot2spot(ooutput,Lc.corner_row,Lc.corner_col,3,80);
        }
        else if(!Lc.corner_flag&&uLc.corner_flag)
        {
            spot2spot(ooutput,uLc.corner_row,uLc.corner_col,59,21);
        }
        else
        {
            spot2spot(ooutput,2,21,58,21);
        }
        if(Rc.corner_flag&&uRc.corner_flag)
        {
            spot2spot(ooutput,Rc.corner_row,Rc.corner_col,uRc.corner_row,uRc.corner_col);
        }
        else if(Rc.corner_flag&&!uRc.corner_flag)
        {
            spot2spot(ooutput,Rc.corner_row,Rc.corner_col,1,Rc.corner_col);
        }
        else if(!Rc.corner_flag&&uRc.corner_flag)
        {
            spot2spot(ooutput,uRc.corner_row,uRc.corner_col,58,66);
        }
        else
        {
            spot2spot(ooutput,2,66,58,66);
        }
        Image_process(ooutput);
    }
    if(szstate==4)
    {
        if(Lc.corner_flag&&uLc.corner_flag)
        {
            spot2spot(ooutput,Lc.corner_row,Lc.corner_col,uLc.corner_row,uLc.corner_col);
        }
        else if(Lc.corner_flag&&!uLc.corner_flag)
        {
            spot2spot(ooutput,Lc.corner_row,Lc.corner_col,3,80);
        }
        else if(!Lc.corner_flag&&uLc.corner_flag)
        {
            spot2spot(ooutput,uLc.corner_row,uLc.corner_col,59,21);
        }
        else
        {
            spot2spot(ooutput,2,21,58,21);
        }
        if(Rc.corner_flag&&uRc.corner_flag)
        {
            spot2spot(ooutput,Rc.corner_row,Rc.corner_col,uRc.corner_row,uRc.corner_col);
        }
        else if(Rc.corner_flag&&!uRc.corner_flag)
        {
            spot2spot(ooutput,Rc.corner_row,Rc.corner_col,1,Rc.corner_col);
        }
        else if(!Rc.corner_flag&&uRc.corner_flag)
        {
            spot2spot(ooutput,uRc.corner_row,uRc.corner_col,58,66);
        }
        else
        {
            spot2spot(ooutput,2,66,58,66);
        }
        Image_process(ooutput);
    }
    if(szstate==5)
    {
        szstate=0;
        inelement=0;

    }
    ti=szstate;
}

void process_cd(){
    static int cdstate;
    if(cdstate==0)
    {
        if(Lc.corner_row>45||Rc.corner_row>45)
            cdstate=1;
    }

    if(cdstate==1)
    {
        if(RIGHT_EDGE_type==-1&&LEFT_EDGE_type==-1||LEFT_EDGE_type==2)
            cdstate=2;
    }




    if(cdstate==2)
    {
        if((Lc.corner_flag&&Lc.corner_row>25)||(Rc.corner_flag&&Rc.corner_row>25))
            cdstate=3;
    }
    if(cdstate==3)
    {
        if(Lc.corner_row>45||Rc.corner_row>45)
            cdstate=4;
        //SPEED_SET=400;
    }
    if(cdstate==4){
        if(LEFT_EDGE_type==-1&&RIGHT_EDGE_type==2)
        {
            cdstate=0;
            inelement=0;

        }
    }


    ti=cdstate;

    if(cdstate==0)
    {

        spot2spot(ooutput,58,66,0,60);
        spot2spot(ooutput,58,21,0,21);
        Image_process(ooutput);
    }

    if(cdstate==1)
    {

        if(Rc.corner_flag==1)
        {
            spot2spot(ooutput,0,Rc.corner_col,Rc.corner_row,Rc.corner_col);
        }

        else
        {
                spot2spot(ooutput,58,66,0,60);
        }
        Image_process(ooutput);
    }
    if(cdstate==2)
    {
        right_findflag = 0;
        RIGHT_EDGE_type = 0;
        R_edge_count = 0;
    }
    if(cdstate==3)
    {

        spot2spot(ooutput,58,66,0,70);
        spot2spot(ooutput,58,21,0,21);
        Image_process(ooutput);
    }
    if(cdstate==4)
    {
        if(Rc.corner_flag==1)
        {
            spot2spot(ooutput,0,Rc.corner_col-10,Rc.corner_row,Rc.corner_col);
        }

        else
        {
                spot2spot(ooutput,58,66,0,60);
        }
        Image_process(ooutput);

    }
}




void process_yh()
{
    static int yhstate;
    //zhuangtaiqiehuan
    if(yhstate==0)//发现拐点
    {
        if(RIGHT_EDGE_type==0 )
            yhstate=1;
    }
    if(yhstate==1)//
    {
        //if(RIGHT_EDGE_type==-1 )
            yhstate=2;
    }
    if(yhstate==2)//进入圆环
    {
        if(uRc.corner_flag==1&&RIGHT_EDGE_type==0&&uRc.corner_angle>90 )
            yhstate=3;
        if(LEFT_EDGE_type==-1&&RIGHT_EDGE_type==0&&check_yhj()!=-1 )
            yhstate=3;
    }
    if(yhstate==3)//贴线走
    {
        if(LEFT_EDGE_type==2)
           yhstate=4;
        //if(Lc.corner_row)
    }
    if(yhstate==4)//准备出圆环
    {
        if(Lc.corner_flag&&L_edge[Lc.corner_icount].row>30&&Lc.corner_angle>90&&L_edge[L_edge_count].col<30)
            yhstate=5;
    }
    if(yhstate==5)//双白
    {
        if(RIGHT_EDGE_type==0&&LEFT_EDGE_type==0)
            yhstate=6;
    }
    if(yhstate==6)//双白
    {
        if(LEFT_EDGE_type==-1)
            yhstate=7;
    }
    if(yhstate==7)
    {
        //if(uRc.corner_flag==1&&(RIGHT_EDGE_type==0||RIGHT_EDGE_type==-1)&&LEFT_EDGE_type==-1)
        if(uRc.corner_flag==1&&LEFT_EDGE_type==-1)
            yhstate=8;
    }
    if(yhstate==8)
    {
        if(uRc.corner_flag==0&&RIGHT_EDGE_type==-1)
        {
            yhstate=0;
            inelement=0;
        }

    }

    ti=yhstate;



    //zhuangtaichuli
    if(yhstate==0||yhstate==1||yhstate==2)
    {

        //ECPULSE_SET=300;
        right_findflag = 0;
        RIGHT_EDGE_type = 0;

        R_edge_count = 0;
    }
    if(yhstate==3)
    {

        //int tr=0,tc=0;
        pama.kp = 45;
        if(uRc.corner_flag==1)
        {
            //spot2spot(ooutput,58,20,uRc.corner_row-5,uRc.corner_col+5);
            //tr=uRc.corner_row;
            //tc=uRc.corner_col;
            spot2spot(ooutput,58,20,20,IMGW-5);
        }

        else
        {
            //int sw=check_yhj();
            //if(sw!=-1)
            //    spot2spot(ooutput,58,20,0,sw);
            //else
                spot2spot(ooutput,58,20,20,IMGW/2);

        }

        Image_process(ooutput);
        right_findflag = 0;
        RIGHT_EDGE_type = 0;
        R_edge_count = 0;
    }
    if(yhstate==4)
    {
        pama.kp = 40;
        right_findflag = 0;
        RIGHT_EDGE_type = 0;
        R_edge_count = 0;
    }
    if(yhstate==5)
    {
        pama.kp = 45;
        spot2spot(ooutput,L_edge[0].row,L_edge[0].col,5,IMGW-20);

        Image_process(ooutput);
        right_findflag = 0;
        RIGHT_EDGE_type = 0;
        R_edge_count = 0;
    }
    if(yhstate==6)
    {
        spot2spot(ooutput,L_edge[0].row,L_edge[0].col,5,IMGW-20);

        Image_process(ooutput);
        right_findflag = 0;
        RIGHT_EDGE_type = 0;
        R_edge_count = 0;
    }
    if(yhstate==7)
    {

        right_findflag = 0;
        RIGHT_EDGE_type = 0;
        R_edge_count = 0;
    }
    if(yhstate==8)
    {
        //ECPULSE_SET=450;
        right_findflag = 0;
        RIGHT_EDGE_type = 0;
        R_edge_count = 0;
        pama.kp = 35;
    }
}


void spot2spot(unsigned char immage[][IMGW], int x1,int y1, int x2,int y2 )
{
    if (x1>=IMGH)x1=IMGH-1;if (x1<0)x1=0;
    if(x2>=IMGH)x2=IMGH-1;if(x2<0)x2=0;
    if(y1>=IMGW)y1=IMGW-1;if(y1<0)y1=0;
    if(y2>=IMGW)y2=IMGW-1;if(y2<0)y2=0;


    int b;
    int x,y,i,dx,dy;
    if (x1==x2)
    {
        if(y1>y2)
        {
        b=y2;
        y2=y1;
        y1=b;
        }
        for (i=y1;i<=y2;i++)
            {
            x=x1;
            y=i;
            immage[x][y]=1;
            if(x-1>=0&&x+1<IMGH&&y-1>=0&&y+1<IMGW)
                {
                immage[x][y-1]=0; immage[x][y+1]=0; immage[x-1][y-1]=0; immage[x-1][y]=0;immage[x-1][y+1]=0;immage[x+1][y-1]=0;immage[x+1][y]=0;immage[x+1][y+1]=0;
                }
            }

        return;
    }
    else
    {

        if(x1<x2)
        {
            b=x2;
            x2=x1;
            x1=b;
            b=y2;
            y2=y1;
            y1=b;
        }
        dx=x2-x1;
        dy=y2-y1;
        if (-dx>dy&&-dx>-dy)
        {

            for (i=x2;i<=x1;i++){
                x=i;
                y=(int)(dy*1.0/dx*(i-x2)+y2+0.5);
                immage[x][y]=0;
                if(x-1>=0&&x+1<IMGH&&y-1>=0&&y+1<IMGW)
                    {
                    immage[x][y-1]=0; immage[x][y+1]=0; immage[x-1][y-1]=0; immage[x-1][y]=0;immage[x-1][y+1]=0;immage[x+1][y-1]=0;immage[x+1][y]=0;immage[x+1][y+1]=0;
                    }
            }
        }

        else
        {
            if (y2>y1)
            {
                b=x2;
                x2=x1;
                x1=b;
                b=y2;
                y2=y1;
                y1=b;
            }
            for (i=y2;i<=y1;i++)
            {
                x=(int)(dx*1.0/dy*(i-y2)+x2+0.5);
                y=i;
                immage[x][y]=0;
                if(x-1>=0&&x+1<IMGH&&y-1>=0&&y+1<IMGW)
                    {
                    immage[x][y-1]=0; immage[x][y+1]=0; immage[x-1][y-1]=0; immage[x-1][y]=0;immage[x-1][y+1]=0;immage[x+1][y-1]=0;immage[x+1][y]=0;immage[x+1][y+1]=0;
                    }
            }
        }
        return;
    }
}

int szorcd_check(){
    int count_sz=0;
    int count_cd=0;
    int i;
    for (i=0;i<5;i++){
        if (Bin_Image[i][IMGW/2]==0)
        count_cd++;
        else
        count_sz++;
    }
    if (count_cd>=4)
        return 3;
    if(count_sz>=4)
        return 1;
    return 0;
}

float akm(int duty){
    float x=15.5;
    float L=20;
    float k=3.8;
    float V= tan((duty-625)/10*k*3.14/180)*x/(L*2);
    return 0.65*V;
}

int check_yhj()
{
    int fw,fb,sw,sb;
    fw=-1;
    fb=-1;
    sw=-1;
    sb=-1;
    int i=0;
    int r;
    r=0;
    while(i<IMGW)
    {
        if(ooutput[r][i]==1)
        {
            fw=i;
            break;
        }
        i++;
    }
    while(i<IMGW)
    {
        if(ooutput[r][i]==0)
        {
            fb=i;
            break;
        }
        i++;
    }
    while(i<IMGW)
    {
        if(ooutput[r][i]==1)
        {
            sw=i;
            break;
        }
        i++;
    }
    while(i<IMGW)
    {
        if(ooutput[r][i]==0)
        {
            sb=i;
            break;
        }
        i++;
    }
    if(fw>22&&sb!=-1&&(sw-fb)>2)
    {
        return sw;
    }
    else
    {
        r=6;
        i=0;
        while(i<IMGW)
        {
            if(ooutput[r][i]==1)
            {
                fw=i;
                break;
            }
            i++;
        }
        while(i<IMGW)
        {
            if(ooutput[r][i]==0)
            {
                fb=i;
                break;
            }
            i++;
        }
        while(i<IMGW)
        {
            if(ooutput[r][i]==1)
            {
                sw=i;
                break;
            }
            i++;
        }
        while(i<IMGW)
        {
            if(ooutput[r][i]==0)
            {
                sb=i;
                break;
            }
            i++;
        }
        if(fw>22&&sb!=-1&&(sw-fb)>2)
        {
            return sw;
        }
        else
        {
            r=3;
            i=0;
            while(i<IMGW)
            {
                if(ooutput[r][i]==1)
                {
                    fw=i;
                    break;
                }
                i++;
            }
            while(i<IMGW)
            {
                if(ooutput[r][i]==0)
                {
                    fb=i;
                    break;
                }
                i++;
            }
            while(i<IMGW)
            {
                if(ooutput[r][i]==1)
                {
                    sw=i;
                    break;
                }
                i++;
            }
            while(i<IMGW)
            {
                if(ooutput[r][i]==0)
                {
                    sb=i;
                    break;
                }
                i++;
            }
            if(fw>22&&sb!=-1&&(sw-fb)>2)
            {
                return sw;
            }
            else
            {
                r=9;
                i=0;
                while(i<IMGW)
                {
                    if(ooutput[r][i]==1)
                    {
                        fw=i;
                        break;
                    }
                    i++;
                }
                while(i<IMGW)
                {
                    if(ooutput[r][i]==0)
                    {
                        fb=i;
                        break;
                    }
                    i++;
                }
                while(i<IMGW)
                {
                    if(ooutput[r][i]==1)
                    {
                        sw=i;
                        break;
                    }
                    i++;
                }
                while(i<IMGW)
                {
                    if(ooutput[r][i]==0)
                    {
                        sb=i;
                        break;
                    }
                    i++;
                }
                if(fw>22&&sb!=-1&&(sw-fb)>2)
                {
                    return sw;
                }
                else
                    return -1;
            }
        }
    }
        //return -1;
}
