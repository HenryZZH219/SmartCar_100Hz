/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 【平    台】北京龙邱智能科技TC264DA核心板
 【编    写】chiusir
 【E-mail  】chiusir@163.com
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
 使用例程的时候，建议采用没有中文及空格的英文路径，
 除了CIF为TC264DA独有外，其它的代码兼容TC264D
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include <ANO_DT.h>
#include <IfxGtm_PinMap.h>
#include <LQ_GPT12_ENC.h>
#include <LQ_GTM.h>
#include <LQ_PID.h>
#include <stdint.h>
#include <IfxGtm_PinMap.h>
#include <LQ_GPIO_KEY.h>
#include <LQ_GPIO_LED.h>
#include <LQ_GTM.h>
#include <LQ_STM.h>
#include <LQ_TFT18.h>
#include <LQ_UART.h>
#include <stdio.h>
#include <LQ_Inductor.h>
#include <LQ_MotorServo.h>
#include <LQ_CCU6.h>

#include <LQ_CAMERA.h>


volatile uint8 Game_Over = 0; // 小车完成全部任务，停车
sint16 ServoDuty = Servo_Center_Mid;
sint16 MotorDuty1 = 500;      // 电机驱动占空比数值
sint16 MotorDuty2 = 500;      // 电机驱动占空比数值
sint32 NowTime = 0;
uint16 BatVolt = 0;           // 电池电压采集
//电机频率
#define MOTOR_FREQUENCY    12500
#define IMGW  94
//电机PWM 宏定义
#define MOTOR1_P          IfxGtm_ATOM0_6_TOUT42_P23_1_OUT
#define MOTOR1_N          IfxGtm_ATOM0_5_TOUT40_P32_4_OUT

#define MOTOR2_P          IfxGtm_ATOM0_0_TOUT53_P21_2_OUT
#define MOTOR2_N          IfxGtm_ATOM0_4_TOUT50_P22_3_OUT

#define MOTOR3_P          IfxGtm_ATOM0_7_TOUT64_P20_8_OUT
#define MOTOR3_N          IfxGtm_ATOM0_3_TOUT56_P21_5_OUT

#define MOTOR4_P          IfxGtm_ATOM0_2_TOUT55_P21_4_OUT
#define MOTOR4_N          IfxGtm_ATOM0_1_TOUT54_P21_3_OUT

#define ATOMSERVO1       IfxGtm_ATOM2_0_TOUT32_P33_10_OUT
#define ATOMSERVO2       IfxGtm_ATOM2_5_TOUT35_P33_13_OUT

//通过宏定义选择不同的驱动板
//#define USE7843or7971   //USEDRV8701 使用龙邱不同的驱动模块，选择对应的宏定义
#define USEDRV8701
/*************************************************************************
 *  函数名称：void MotorInit(void)
 *  功能说明：电机PWM初始化
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年4月1日
 *  备    注：驱动2个电机
 *************************************************************************/
void MotorInit (void)
{
    ATOM_PWM_InitConfig(MOTOR1_P, 0, MOTOR_FREQUENCY);
    ATOM_PWM_InitConfig(MOTOR1_N, 0, MOTOR_FREQUENCY);
    ATOM_PWM_InitConfig(MOTOR2_P, 0, MOTOR_FREQUENCY);
    ATOM_PWM_InitConfig(MOTOR2_N, 0, MOTOR_FREQUENCY);
    ATOM_PWM_InitConfig(MOTOR3_P, 0, MOTOR_FREQUENCY);
    ATOM_PWM_InitConfig(MOTOR3_N, 0, MOTOR_FREQUENCY);
    ATOM_PWM_InitConfig(MOTOR4_P, 0, MOTOR_FREQUENCY);
    ATOM_PWM_InitConfig(MOTOR4_N, 0, MOTOR_FREQUENCY);
}

/*************************************************************************
 *  函数名称：void EncInit (void)
 *  功能说明：编码器初始化函数
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年4月1日
 *  备    注：驱动2个编码器
 *************************************************************************/
void EncInit (void)
{
    ENC_InitConfig(ENC5_InPut_P10_3, ENC5_Dir_P10_1);
    ENC_InitConfig(ENC6_InPut_P20_3, ENC6_Dir_P20_0);
    ENC_InitConfig(ENC2_InPut_P33_7, ENC2_Dir_P33_6);
    ENC_InitConfig(ENC4_InPut_P02_8, ENC4_Dir_P33_5);
}

/*************************************************************************
 *  函数名称：void MotorCtrl(float motor1, float motor2)
 *  功能说明：舵机转角函数，由于小车拉杆范围限制，较小
 *  参数说明：   @param    motor1   ： 电机1占空比
 @param    motor2   ： 电机2占空比
 *  函数返回：无
 *  修改时间：2020年4月1日
 *  备    注：驱动2个舵机，普通四轮只需要一个舵机即可
 *************************************************************************/
/*************************************************************************
 *  函数名称：void MotorCtrl(float motor1, float motor2)
 *  功能说明：舵机转角函数，由于小车拉杆范围限制，较小
 *  参数说明：   @param    motor1   ： 电机1占空比
 @param    motor2   ： 电机2占空比
 *  函数返回：无
 *  修改时间：2020年4月1日
 *  备    注：驱动2个舵机，普通四轮只需要一个舵机即可
 *************************************************************************/
#ifdef USE7843or7971
void MotorCtrl (sint32 motor1, sint32 motor2)
{
    if (motor1 > 0)
    {
        ATOM_PWM_SetDuty(MOTOR3_P, motor1, MOTOR_FREQUENCY);
        ATOM_PWM_SetDuty(MOTOR3_N, 0, MOTOR_FREQUENCY);
    }
    else
    {
        ATOM_PWM_SetDuty(MOTOR3_P, 0, MOTOR_FREQUENCY);
        ATOM_PWM_SetDuty(MOTOR3_N, (-motor1), MOTOR_FREQUENCY);
    }

    if (motor2 > 0)
    {
        ATOM_PWM_SetDuty(MOTOR4_P, motor2, MOTOR_FREQUENCY);
        ATOM_PWM_SetDuty(MOTOR4_N, 0, MOTOR_FREQUENCY);
    }
    else
    {
        ATOM_PWM_SetDuty(MOTOR4_P, 0, MOTOR_FREQUENCY);
        ATOM_PWM_SetDuty(MOTOR4_N, (-motor2), MOTOR_FREQUENCY);
    }
}
void MotorCtrl4w(sint32 motor1, sint32 motor2,sint32 motor3, sint32 motor4)
{
    if (motor1 > 0)
       {
           ATOM_PWM_SetDuty(MOTOR1_P, motor1, MOTOR_FREQUENCY);
           ATOM_PWM_SetDuty(MOTOR1_N, 0, MOTOR_FREQUENCY);
       }
       else
       {
           ATOM_PWM_SetDuty(MOTOR1_P, 0, MOTOR_FREQUENCY);
           ATOM_PWM_SetDuty(MOTOR1_N, (-motor1), MOTOR_FREQUENCY);
       }

    if (motor2 > 0)
       {
           ATOM_PWM_SetDuty(MOTOR2_P, motor2, MOTOR_FREQUENCY);
           ATOM_PWM_SetDuty(MOTOR2_N, 0, MOTOR_FREQUENCY);
       }
       else
       {
           ATOM_PWM_SetDuty(MOTOR2_P, 0, MOTOR_FREQUENCY);
           ATOM_PWM_SetDuty(MOTOR2_N, (-motor2), MOTOR_FREQUENCY);
       }
    if (motor3 > 0)
       {
           ATOM_PWM_SetDuty(MOTOR3_P, motor3, MOTOR_FREQUENCY);
           ATOM_PWM_SetDuty(MOTOR3_N, 0, MOTOR_FREQUENCY);
       }
       else
       {
           ATOM_PWM_SetDuty(MOTOR3_P, 0, MOTOR_FREQUENCY);
           ATOM_PWM_SetDuty(MOTOR3_N, (-motor3), MOTOR_FREQUENCY);
       }

       if (motor4 > 0)
       {
           ATOM_PWM_SetDuty(MOTOR4_P, motor4, MOTOR_FREQUENCY);
           ATOM_PWM_SetDuty(MOTOR4_N, 0, MOTOR_FREQUENCY);
       }
       else
       {
           ATOM_PWM_SetDuty(MOTOR4_P, 0, MOTOR_FREQUENCY);
           ATOM_PWM_SetDuty(MOTOR4_N, (-motor4), MOTOR_FREQUENCY);
       }
}
#else   //USEDRV8701
void MotorCtrl (sint32 motor1, sint32 motor2)
{
    if (motor1 > 0)
    {
        ATOM_PWM_SetDuty(MOTOR3_P, motor1, MOTOR_FREQUENCY);
        ATOM_PWM_SetDuty(MOTOR3_N, 0, MOTOR_FREQUENCY);
    }
    else
    {
        ATOM_PWM_SetDuty(MOTOR3_P, (0-motor1),  MOTOR_FREQUENCY);
        ATOM_PWM_SetDuty(MOTOR3_N, (motor1), MOTOR_FREQUENCY);
    }

    if (motor2 > 0)
    {
        ATOM_PWM_SetDuty(MOTOR4_P, motor2, MOTOR_FREQUENCY);
        ATOM_PWM_SetDuty(MOTOR4_N, 0,MOTOR_FREQUENCY);
    }
    else
    {
        ATOM_PWM_SetDuty(MOTOR4_P, (0-motor2), MOTOR_FREQUENCY);
        ATOM_PWM_SetDuty(MOTOR4_N, motor2,MOTOR_FREQUENCY);
    }
}

void MotorCtrl4w(sint32 motor1, sint32 motor2,sint32 motor3, sint32 motor4)
{
    /*
    if(motor1==4500)
        motor1=7000;
    else if(motor1==-4500)
        motor1=-7000;
    if(motor2==4500)
        motor2=7000;
    else if(motor2==-4500)
        motor2=-7000;
    if(motor3==4500)
        motor3=7000;
    else if(motor3==-4500)
        motor3=-7000;
    if(motor4==4500)
        motor4=7000;
    else if(motor4==-4500)
        motor4=-7000;*/


    if (motor1 > 0)
       {
           ATOM_PWM_SetDuty(MOTOR1_P, motor1, MOTOR_FREQUENCY);
           ATOM_PWM_SetDuty(MOTOR1_N, 0, MOTOR_FREQUENCY);
       }
       else
       {
           ATOM_PWM_SetDuty(MOTOR1_P, (0-motor1),  MOTOR_FREQUENCY);
           ATOM_PWM_SetDuty(MOTOR1_N, (motor1), MOTOR_FREQUENCY);
       }

       if (motor2 > 0)
       {
           ATOM_PWM_SetDuty(MOTOR2_P, motor2, MOTOR_FREQUENCY);
           ATOM_PWM_SetDuty(MOTOR2_N, 0,MOTOR_FREQUENCY);
       }
       else
       {
           ATOM_PWM_SetDuty(MOTOR2_P, (0-motor2), MOTOR_FREQUENCY);
           ATOM_PWM_SetDuty(MOTOR2_N, motor2,MOTOR_FREQUENCY);
       }
    if (motor3 > 0)
       {
           ATOM_PWM_SetDuty(MOTOR3_P, motor3, MOTOR_FREQUENCY);
           ATOM_PWM_SetDuty(MOTOR3_N, 0, MOTOR_FREQUENCY);
       }
       else
       {
           ATOM_PWM_SetDuty(MOTOR3_P, (0-motor3),  MOTOR_FREQUENCY);
           ATOM_PWM_SetDuty(MOTOR3_N, motor3,MOTOR_FREQUENCY);
       }

       if (motor4 > 0)
       {
           ATOM_PWM_SetDuty(MOTOR4_P, motor4, MOTOR_FREQUENCY);
           ATOM_PWM_SetDuty(MOTOR4_N, 0, MOTOR_FREQUENCY);
       }
       else
       {
           ATOM_PWM_SetDuty(MOTOR4_P, (0-motor4), MOTOR_FREQUENCY);
           ATOM_PWM_SetDuty(MOTOR4_N, motor4, MOTOR_FREQUENCY);
       }
}


#endif

/*************************************************************************
 *  函数名称：TestMotor(void)
 *  功能说明：测试标定输出PWM控制电机
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年4月1日
 *  备    注：驱动2个电机
 【注意事项】注意，一定要对舵机打角进行限制
 使用龙邱母板测试流程：
 1.先使用万用表测量电池电压，务必保证电池电压在7V以上，否则无力不反应！
 2.接好母板到驱动板的信号线及电源线；
 3.接好驱动板到电机的导线；
 4.烧写程序并运行，确定电机能正常转动后，开启驱动板电源开关；
 5.按键K0/K1确定电机转动速度及方向；
 6.如果出现疯转，按下K2键返回低速模式，或者直接关闭驱动板电源！
 *************************************************************************/
void TestMotor (void)
{
    short duty = 2000;
    char txt[32];
    volatile sint16 ECPULSE3 = 0;          // 速度全局变量
    volatile sint16 ECPULSE4 = 0;          // 速度全局变量
    MotorInit();
    EncInit();
    ENC_InitConfig(ENC5_InPut_P10_3, ENC5_Dir_P10_1);
    ENC_InitConfig(ENC6_InPut_P20_3, ENC6_Dir_P20_0);
    ENC_InitConfig(ENC2_InPut_P33_7, ENC2_Dir_P33_6);
    //ENC_InitConfig(ENC3_InPut_P02_6, ENC3_Dir_P02_7);//摄像头冲突，不建议用
    ENC_InitConfig(ENC4_InPut_P02_8, ENC4_Dir_P33_5);
    TFTSPI_Init(0);        //LCD初始化  0:横屏  1：竖屏
    TFTSPI_CLS(u16BLUE);   //蓝色屏幕
    TFTSPI_P16x16Str(0,0,(unsigned char*)"北京龙邱智能科技",u16RED,u16BLUE);      //字符串显示
    TFTSPI_P8X16Str(0,1,"Long Qiu i.s.t.",u16WHITE,u16BLACK);     //字符串显示


    while (1)
    {
        if (KEY_Read(KEY0) == 0)      //按下KEY0键，占空比减小
        {
            //if (duty > 9000)
                duty -= 100;
        }
        if (KEY_Read(KEY2) == 0)      //按下KEY2键，占空比加大
        {
           // if (duty < 11000)      //满占空比为12500
                duty += 100;
        }
        if (KEY_Read(KEY1) == 0)      //按下KEY1键，占空比中值
        {
            duty =2000;
        }

       // MotorCtrl(duty, duty);
       MotorCtrl4w(duty,duty,duty,duty);
        sprintf(txt, "PWM: %05d;", duty);
       // UART_PutStr(UART0, txt);
        //TFTSPI_CLS(u16BLUE);          //清屏
        //TFTSPI_P8X16Str(2, 0, txt, u16RED, u16BLUE);
        TFTSPI_P8X16Str(0, 8, txt, u16RED, u16BLUE);
        /* 获取编码器值 */
       ECPULSE1 = ENC_GetCounter(ENC5_InPut_P10_3); //左电机 母板上编码器1，小车前进为负值
        ECPULSE2 = ENC_GetCounter(ENC6_InPut_P20_3); //右电机 母板上编码器2，小车前进为正值
        ECPULSE3 = ENC_GetCounter(ENC2_InPut_P33_7); //左电机 母板上编码器1，小车前进为负值
        ECPULSE4 = ENC_GetCounter(ENC4_InPut_P02_8); //右电机 母板上编码器2，小车前进为正值
        sprintf(txt, "Enc1: %05d;", ECPULSE1);
        TFTSPI_P8X16Str(0, 3, txt, u16WHITE, u16BLACK);       //字符串显示
        sprintf(txt, "Enc2: %05d;", ECPULSE2);
        TFTSPI_P8X16Str(0, 4, txt, u16WHITE, u16BLACK);
        sprintf(txt, "Enc3: %05d;", ECPULSE3);
        TFTSPI_P8X16Str(0, 5, txt, u16WHITE, u16BLACK);       //字符串显示
        sprintf(txt, "Enc4: %05d;", ECPULSE4);
        TFTSPI_P8X16Str(0, 6, txt, u16WHITE, u16BLACK);
        LED_Ctrl(LED0, RVS);       //电平翻转,LED闪烁
        delayms(200);              //延时等待
    }
}
/*************************************************************************
 *  函数名称：void ServoInit(void)
 *  功能说明：舵机PWM初始化
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年4月1日
 *  备    注：驱动2个舵机
 *************************************************************************/
void ServoInit (void)
{
    ATOM_PWM_InitConfig(ATOMSERVO1, Servo_Center_Mid, 50);  //舵机频率为100HZ，初始值为1.5ms中值
    ATOM_PWM_InitConfig(ATOMSERVO2, Servo_Center_Mid, 50);  //舵机理论范围为：0.5ms--2.5ms，大多舵机实际比这个范围小
}

/*************************************************************************
 *  函数名称：void ServoCtrl(uint32 duty)
 *  功能说明：舵机转角函数，由于小车拉杆范围限制，较小
 *  参数说明：short duty，占空比
 *  函数返回：无
 *  修改时间：2020年4月1日
 *  备    注：驱动2个舵机，普通四轮只需要一个舵机即可
 *************************************************************************/
void ServoCtrl (uint32 duty)
{
    if (duty >= Servo_Left_Max)                  //限制幅值
        duty = Servo_Left_Max;
    else if (duty <= Servo_Right_Min)            //限制幅值
        duty = Servo_Right_Min;

    ATOM_PWM_InitConfig(ATOMSERVO1, duty, 50);  //舵机频率为100HZ，初始值为1.5ms中值
    ATOM_PWM_InitConfig(ATOMSERVO2, duty, 50);  //舵机理论范围为：0.5ms--2.5ms，大多舵机实际比这个范围小
}

/*************************************************************************
 *  函数名称：Test_Servo(void)
 *  功能说明：舵机PWM初始化，测试标定输出PWM控制SD5/S3010舵机
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年4月1日
 *  备    注：驱动2个舵机
 【注意事项】注意，一定要对舵机打角进行限制
 使用龙邱母板测试流程：
 1.先使用万用表测量电池电压，务必保证电池电压在7V以上，否则无力不反应！
 2.然后确定舵机供电电压，SD5舵机用5V供电，S3010用6-7V供电！！！
 3.把舵机的舵盘去掉，让舵机可以自由转动；
 4.烧写程序并运行，让舵机转动到中值附近；如果没反应重复1-2步，或者调整舵机的PWM频率计占空比，能受控为准；
 5.舵机受控后用手轻转，舵机会吱吱响，对抗转动，此时可以装上舵盘；
 6.按键K0/K1确定舵机的左右转动极限，并记下来，作为后续限幅防止舵机堵转烧毁！
 *************************************************************************/
void TestServo (void)
{
    char txt[16] = "X:";
    signed short duty = Servo_Center_Mid;

    TFTSPI_CLS(u16BLUE);          //清屏
   // TFTSPI_P8X16Str(2, 0, "LQ Servo Test", u16RED, u16BLUE);
    ServoInit();
    ServoCtrl(Servo_Center_Mid);      //中值
    while (1)
    {
        if (!KEY_Read(KEY0))
        {
            if (duty > 10)  //防止duty超
            {
                duty -= 10;

            }

        }
        if (!KEY_Read(KEY1))
        {
            duty = Servo_Center_Mid;

        }
        if (!KEY_Read(KEY2))
        {
            duty += 10;

        }

        ServoCtrl(duty);
        sprintf(txt, "Servo duty:%04d ", duty);
        TFTSPI_P8X16Str(0, 3, txt, u16WHITE, u16BLACK);       //字符串显示
        //UART_PutStr(UART0, txt);

        LED_Ctrl(LEDALL, RVS);        //四个LED同时闪烁;
        delayms(100);

    }
}

/*************************************************************************
 *  函数名称：void TestEncoder(void)
 *  功能说明：测试程序，TFT1.8显示
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年4月10日
 *  备    注：
 *************************************************************************/
void TestEncoder(void)
{
    char txt[32];
    TFTSPI_Init(0);        //LCD初始化  0:横屏  1：竖屏
    TFTSPI_CLS(u16BLUE);   //蓝色屏幕
    TFTSPI_P16x16Str(0,0,(unsigned char*)"北京龙邱智能科技",u16RED,u16BLUE);      //字符串显示
    TFTSPI_P8X16Str(0,1,"Long Qiu i.s.t.",u16WHITE,u16BLACK);     //字符串显示
    //TFTSPI_CLS(u16BLUE);   //蓝色屏幕
    //TFTSPI_P8X16Str(0, 0, "Test Encoder", u16WHITE, u16BLACK);      //字符串显示
    EncInit();
    while (1)
    {
        /* 获取编码器值 */
        ECPULSE1 = ENC_GetCounter(ENC2_InPut_P33_7); //左电机 母板上编码器1，小车前进为负值
        ECPULSE2 = ENC_GetCounter(ENC4_InPut_P02_8); //右电机 母板上编码器2，小车前进为正值

        sprintf(txt, "Enc1: %05d;", ECPULSE1);
        TFTSPI_P8X16Str(0, 3, txt, u16WHITE, u16BLACK);       //字符串显示
        sprintf(txt, "Enc2: %05d;", ECPULSE2);
        TFTSPI_P8X16Str(0, 5, txt, u16WHITE, u16BLACK);       //字符串显示

        LED_Ctrl(LED0, RVS);        //电平翻转,LED闪烁
        delayms(200);              //延时等待
    }
}

/*************************************************************************
 *  函数名称：uint8 SetCircleNum (void)
 *  功能说明：设置需要进入圆环的个数；
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年11月18日
 *  备    注：
 *************************************************************************/
uint8 SetCircleNum (void)
{
    char txt[16] = " ";
    uint8 num = 1;

    TFTSPI_CLS(u16BLACK);            // 清屏
    TFTSPI_P8X16Str(2, 1, "LQ Smart Car", u16RED, u16BLUE);
    TFTSPI_P8X16Str(2, 3, "K2 num +", u16RED, u16BLUE);
    TFTSPI_P8X16Str(2, 4, "K1 set OK", u16RED, u16BLUE);
    TFTSPI_P8X16Str(2, 5, "K0 num -", u16RED, u16BLUE);
    TFTSPI_P8X16Str(2, 7, "Ring num:  ", u16RED, u16BLUE);

    while (KEY_Read(KEY1))
    {
        if (KEY_Read(KEY2) == 0)
        {
            if (num < 10)
                num++;
        }
        else if (KEY_Read(KEY0) == 0)
        {
            if (num > 0)
                num--;
        }
        sprintf(txt, "Ring num: %d ", num);
        TFTSPI_P8X16Str(2, 7, txt, u16WHITE, u16BLUE);

        delayms(100);
    }
    return num;
}

/*************************************************************************
 *  函数名称：void OutInGarage(uint8 inout, uint8 lr)
 *  功能说明：出入库
 *  参数说明：uint8 inout:0出库，1入库；
 *          uint8 lr：0左出入库；1右出入库
 *  函数返回：无
 *  修改时间：2020年11月18日
 *  备    注：   OutInGarage(1,0); // 右侧出库
 *************************************************************************/
void OutInGarageTft (uint8 inout, uint8 lr)// 测试用，有屏幕显示，不要冲突
{
    char txt[16];
    sint32 ps = 0;
    // 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
    mutexCpu0TFTIsOk = 0;         // CPU1： 0占用/1释放 TFT
    delayms(10);
    TFTSPI_CLS(u16BLACK);         // 清屏
    if (lr)           // 1右出入库
    {
        if (inout)    // 1右入库
        {
            NowTime = (STM_GetNowUs(STM0) - NowTime) / 1000;  // 获取STM0 当前时间，得到毫秒
            // 2020年新加出库元素，此处为盲走入库
            ps = RAllPulse;
            ServoCtrl(Servo_Center_Mid);  // 回中倒车
            MotorCtrl(-2500, -2500);      // 左后倒车
            while (RAllPulse > ps - 1500) //
            {
                // 右电机 母板上编码器2，小车前进为正值，并累加
                sprintf(txt, "Enc1:%04d ", (sint16)(RAllPulse - ps));
                TFTSPI_P8X16Str(2, 4, txt, u16BLACK, u16BLUE); // 显示出库实际脉冲数，以便灵活调整
                delayms(10);
            }

            ps = RAllPulse;
            ServoCtrl(Servo_Right_Min);    // 舵机向右打死为出库做准备
            MotorCtrl(-3000, -2000);       // 右后倒车，左轮快，右轮慢，
            while (RAllPulse > ps - 2000)  // 从停车位出库，大约要512编码器2000个脉冲，龙邱512带方向编码器1米5790个脉冲
            {
                // 右电机 母板上编码器1，小车后退为正值，并累加到出库为止
                sprintf(txt, "Enc1:%04d ", (sint16)(RAllPulse - ps));
                TFTSPI_P8X16Str(2, 4, txt, u16BLACK, u16BLUE); // 显示出库实际脉冲数，以便灵活调整
                delayms(10);
            }
            ps = RAllPulse;
            ServoCtrl(Servo_Center_Mid);  // 回中倒车
            MotorCtrl(-2500, -2500);      // 左后倒车，右轮快，左轮慢
            while (RAllPulse > ps - 800) // 右电机 母板上编码器1，小车后退为正值，并累加到出库为止
            {
                // 右电机 母板上编码器1，小车后退为正值，并累加到出库为止
                sprintf(txt, "Enc1:%04d ", (sint16)(RAllPulse - ps));
                TFTSPI_P8X16Str(2, 4, txt, u16BLACK, u16BLUE); // 显示出库实际脉冲数，以便灵活调整
                delayms(10);
            }
            sprintf(txt, "Enc1:%04d ", (sint16)(RAllPulse - ps));
            TFTSPI_P8X16Str(2, 4, txt, u16BLACK, u16BLUE); // 显示出库实际脉冲数，以便灵活调整

            IfxCpu_disableInterrupts(); // 关闭所有中断
            MotorCtrl(3000, 3000);
            delayms(300);               // 电机反转刹车，防止滑出赛道，时间根据速度调整
            MotorCtrl(0, 0);            // 停车

            TFTSPI_CLS(u16RED);        // 清屏
            sprintf(txt, "Time:%d.%03ds", (sint16) (NowTime / 1000), (sint16) (NowTime % 1000));
            TFTSPI_P8X16Str(2, 4, txt, u16BLACK, u16BLUE); // 显示出库实际脉冲数，以便灵活调整
            TFTSPI_P8X16Str(2, 2, "!!WellDone!!", u16WHITE, u16RED); // 入库完毕，永久停车
            while (1);                 // 入库完毕，永久停车
        }
        else  // 0右出库
        {
            // 2020年新加出库元素，此处为盲走出库
            ps = RAllPulse;
            ServoCtrl(Servo_Center_Mid); // 直行大约10cm
            MotorCtrl(2500, 2500);       //
            while (RAllPulse < ps + 600)
            {
                // 右电机 母板上编码器2，小车前进为正值，并累加到出库为止
                sprintf(txt, "Enc2:%04d ", (sint16)(RAllPulse - ps));
                TFTSPI_P8X16Str(2, 4, txt, u16BLACK, u16BLUE); // 显示出库实际脉冲数，以便灵活调整
                delayms(10);
            }

            ps = RAllPulse;
            ServoCtrl(Servo_Right_Min); // 舵机向右打死为出库做准备
            MotorCtrl(3500, 3000);       // 右转，左轮快，右轮慢，
            while (RAllPulse < ps + 1200)
            {
                // 右电机 母板上编码器2，小车前进为正值，并累加到出库为止
                sprintf(txt, "Enc2:%04d ", (sint16)(RAllPulse - ps));
                TFTSPI_P8X16Str(2, 4, txt, u16BLACK, u16BLUE); // 显示出库实际脉冲数，以便灵活调整
                delayms(10);
            }
            LED_Ctrl(LED3, OFF);        // LED闪烁 指示程序运行状态
            sprintf(txt, "Enc2:%04d ", (sint16)(RAllPulse - ps));
            TFTSPI_P8X16Str(2, 4, txt, u16BLACK, u16BLUE); // 显示出库实际脉冲数，以便灵活调整

            // MotorCtrl(-2000, -2000);
            // delayms(200);               // 电机反转刹车，防止滑出赛道，时间根据速度调整
            // ServoCtrl(Servo_Center_Mid);// 回中
            // MotorCtrl(0, 0);            // 停车
            // while(1);                   // 测试用，正常跑车需要删除该句
        }
    }
    else // 0：左出入库；
    {
        if (inout) // 1左入库
        {
            NowTime = (STM_GetNowUs(STM0) - NowTime) / 1000;  // 获取STM0 当前时间，得到毫秒
            // 2020年新加出库元素，此处为盲走入库
            ps = RAllPulse;
            ServoCtrl(Servo_Center_Mid);  // 回中倒车
            MotorCtrl(-2000, -2000);      // 左后倒车，右轮快，左轮慢
            while (RAllPulse > ps - 600)  // 右电机 母板上编码器1，小车后退为正值，并累加到为止
            {
                delayms(10);
            }

            ps = RAllPulse;
            ServoCtrl(Servo_Left_Max);    // 舵机向右打死为出库做准备
            MotorCtrl(-2500, -2500);      // 左后倒车，右轮快，左轮慢
            while (RAllPulse > ps - 2800)
            {
                // 右电机 母板上编码器1，小车后退为正值，并累加到出库为止
                sprintf(txt, "Enc1:%04d ", (sint16)(RAllPulse - ps));
                TFTSPI_P8X16Str(2, 4, txt, u16BLACK, u16BLUE); // 显示出库实际脉冲数，以便灵活调整
                delayms(10);
            }

            ps = RAllPulse;
            ServoCtrl(Servo_Center_Mid);  // 回中倒车
            MotorCtrl(-2500, -2500);      // 左后倒车，右轮快，左轮慢
            while (RAllPulse > ps - 1500)
            {
                // 右电机 母板上编码器1，小车后退为正值，并累加到出库为止
                sprintf(txt, "Enc1:%04d ", (sint16)(RAllPulse - ps));
                TFTSPI_P8X16Str(2, 4, txt, u16BLACK, u16BLUE); // 显示出库实际脉冲数，以便灵活调整
                delayms(10);
            }
            sprintf(txt, "Enc1:%04d ", (sint16)(RAllPulse - ps));
            TFTSPI_P8X16Str(2, 4, txt, u16BLACK, u16BLUE); // 显示出库实际脉冲数，以便灵活调整

            IfxCpu_disableInterrupts(); // 关闭所有中断
            MotorCtrl(3000, 3000);
            delayms(300);               // 电机反转刹车，防止滑出赛道，时间根据速度调整
            MotorCtrl(0, 0);            // 停车

            TFTSPI_CLS(u16RED);        // 清屏
            sprintf(txt, "Time:%d.%03ds", (sint16)(NowTime/1000),(sint16)(NowTime%1000));
            TFTSPI_P8X16Str(2, 4, txt, u16BLACK, u16BLUE); // 显示出库实际脉冲数，以便灵活调整
            TFTSPI_P8X16Str(2, 2, "!!WellDone!!", u16WHITE, u16RED); // 入库完毕，永久停车
            while (1) ;                 // 入库完毕，永久停车
        }
        else  // 0左出库
        {
            // 2020年新加出库元素，此处为盲走出库
            ps = RAllPulse;
            ServoCtrl(Servo_Center_Mid); // 直行大约10cm
            MotorCtrl(2500, 2500);       //
            while (RAllPulse < ps + 600)
            {
                // 右电机 母板上编码器2，小车前进为正值，并累加到出库为止
                sprintf(txt, "Enc2:%04d ", (sint16)(RAllPulse - ps));
                TFTSPI_P8X16Str(2, 4, txt, u16BLACK, u16BLUE); // 显示出库实际脉冲数，以便灵活调整
                delayms(10);
            }

            ps = RAllPulse;
            ServoCtrl(Servo_Left_Max); // 舵机向左打死为出库做准备
            MotorCtrl(3000, 3500);     // 左转，右轮快，左轮慢，
            while (RAllPulse < ps + 1800)
            {
                // 右电机 母板上编码器2，小车前进为正值，并累加到出库为止
                sprintf(txt, "Enc2:%04d ", (sint16)(RAllPulse - ps));
                TFTSPI_P8X16Str(2, 4, txt, u16BLACK, u16BLUE); // 显示出库实际脉冲数，以便灵活调整
                delayms(10);
            }
            LED_Ctrl(LED3, OFF);        // LED闪烁 指示程序运行状态
            sprintf(txt, "Enc2:%04d ", (sint16)(RAllPulse - ps));
            TFTSPI_P8X16Str(2, 4, txt, u16BLACK, u16BLUE); // 显示出库实际脉冲数，以便灵活调整
            // MotorCtrl(-2000, -2000);
            // delayms(100);               // 电机反转刹车，防止滑出赛道，时间根据速度调整
            // ServoCtrl(Servo_Center_Mid);// 回中
            // MotorCtrl(0, 0);            // 停车
            // while(1);                   // 测试用，正常跑车需要删除该句
        }
    }
    // 切记CPU0,CPU1...不可以同时开启屏幕显示，否则冲突不显示
    mutexCpu0TFTIsOk = 1;         // CPU1： 0占用/1释放 TFT
}
void OutInGarage (uint8 inout, uint8 lr)
{
    sint32 ps = 0;

    if (lr)           // 1右出入库
    {
        if (inout)    // 1右入库
        {
            NowTime = (STM_GetNowUs(STM0) - NowTime) / 1000;  // 获取STM0 当前时间，得到毫秒
            // 2020年新加出库元素，此处为盲走入库
            ps = RAllPulse;
            ServoCtrl(Servo_Center_Mid);  // 回中倒车
            MotorCtrl(2000, 2000);        // 左后倒车
            while (RAllPulse < ps + 2000) // 继续前进大约35cm
            {
                delayms(10);
            }
            MotorCtrl(-2500, -2500);      // 刹车
            delayms(300);

            ps = RAllPulse;
            ServoCtrl(Servo_Right_Min);   // 舵机向右打死为出库做准备
            MotorCtrl(-3000, -2000);      // 右后倒车，左轮快，右轮慢，
            while (RAllPulse > ps - 2000) // 从停车位出库，大约要512编码器2000个脉冲，龙邱512带方向编码器1米5790个脉冲
            {   // 右电机 母板上编码器1，小车后退为正值，并累加到出库为止
                delayms(10);
            }
            ps = RAllPulse;
            ServoCtrl(Servo_Center_Mid);  // 回中倒车
            MotorCtrl(-2500, -2500);      // 左后倒车，右轮快，左轮慢
            while (RAllPulse > ps - 800)  // 右电机 母板上编码器1，小车后退为正值，并累加到出库为止
            {
                delayms(10);
            }
            IfxCpu_disableInterrupts();  // 关闭所有中断
            MotorCtrl(3000, 3000);
            delayms(300);                // 电机反转刹车，防止滑出赛道，时间根据速度调整
            MotorCtrl(0, 0);             // 停车
            while (1);                   // 入库完毕，永久停车
        }
        else  // 0右出库
        {
            // 2020年新加出库元素，此处为盲走出库
            ps = RAllPulse;
            ServoCtrl(Servo_Center_Mid); // 直行大约10cm
            MotorCtrl(2500, 2500);       //
            while (RAllPulse < ps + 600)
            {
                delayms(10);
            }

            ps = RAllPulse;
            ServoCtrl(Servo_Right_Min); // 舵机向右打死为出库做准备
            MotorCtrl(3500, 3000);       // 右转，左轮快，右轮慢，
            while (RAllPulse < ps + 1200)
            {
                delayms(10);
            }
        }
    }
    else // 0：左出入库；
    {
        if (inout) // 1左入库
        {
            NowTime = (STM_GetNowUs(STM0) - NowTime) / 1000;  // 获取STM0 当前时间，得到毫秒
            // 2020年新加出库元素，此处为盲走入库
            ps = RAllPulse;
            ServoCtrl(Servo_Center_Mid);  // 回中倒车
            MotorCtrl(2000, 2000);        // 左后倒车
            while (RAllPulse < ps + 2500) // 继续前进大约35cm
            {
                delayms(10);
            }
            MotorCtrl(-2500, -2500);      // 刹车
            delayms(300);

            ps = RAllPulse;
            ServoCtrl(Servo_Left_Max);    // 舵机向右打死为出库做准备
            MotorCtrl(-2000, -3000);      // 左后倒车，右轮快，左轮慢
            while (RAllPulse > ps - 3000)
            {
                delayms(10);
            }

            ps = RAllPulse;
            ServoCtrl(Servo_Center_Mid);  // 回中倒车
            MotorCtrl(-2500, -2500);      // 左后倒车，右轮快，左轮慢
            while (RAllPulse > ps - 1500)
            {
                delayms(10);
            }
            IfxCpu_disableInterrupts(); // 关闭所有中断
            MotorCtrl(3000, 3000);
            delayms(300);               // 电机反转刹车，防止滑出赛道，时间根据速度调整
            MotorCtrl(0, 0);            // 停车
            while (1) ;                 // 入库完毕，永久停车
        }
        else  // 0左出库
        {
            // 2020年新加出库元素，此处为盲走出库
            ps = RAllPulse;
            ServoCtrl(Servo_Center_Mid); // 直行大约10cm
            MotorCtrl(2500, 2500);       //
            while (RAllPulse < ps + 600)
            {
                delayms(10);
            }

            ps = RAllPulse;
            ServoCtrl(Servo_Left_Max); // 舵机向左打死为出库做准备
            MotorCtrl(3000, 3500);     // 左转，右轮快，左轮慢，
            while (RAllPulse < ps + 1800)
            {
                delayms(10);
            }
        }
    }
}

/*************************************************************************
 *  函数名称：uint8 ReadOutInGarageMode(void)
 *  功能说明：读取拨码开关设置出入库模式
 *  参数说明：无
 *  函数返回：出入库模式,0左出入库；默认1右出入库
 *  修改时间：2020年11月18日
 *  备    注：
 *************************************************************************/
uint8 ReadOutInGarageMode (void)
{
    return (KEY_Read(DSW0));
}

