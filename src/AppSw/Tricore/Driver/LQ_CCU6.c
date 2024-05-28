/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 【平    台】北京龙邱智能科技TC264DA核心板
 【编    写】ZYF/chiusir
 【E-mail  】chiusir@163.com
 【软件版本】V1.1 版权所有，单位使用请先联系授权
 【最后更新】2020年10月28日
 【相关信息参考下列地址】
 【网    站】http:// www.lqist.cn
 【淘宝店铺】http:// longqiu.taobao.com
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
 ________________________________________________________________

 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ
 *  备    注：TC264 有两个CCU6模块  每个模块有两个独立定时器  触发定时器中断
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

#include <ANO_DT.h>

int Angle_X_Final;
int Angle_Y_Final;
float ax,ay,az,Gyro_x,Gyro_y,Gyro_z;


extern int duty1;
extern int duty2;
extern int SPEED_SET;
extern float Kakm;
extern float Pitch;
extern float Yaw;
extern int error_last_11;
extern int error_last_12;
extern int error_last_21;
extern int error_last_22;
extern int KP_motor;
extern int KI_motor;
extern int KD_motor;
extern int xieru_flag;
extern int lb_la;
extern int flag_cs;
extern int K_MD;
extern int duty_max;
extern int flag_pd;
volatile sint16 ECPULSE1 = 0;          // 速度全局变量
volatile sint16 ECPULSE2 = 0;          // 速度全局变量
volatile sint32 RAllPulse = 0;          // 速度全局变量
extern unsigned short distc;
extern float speed_delta;
short aacx,aacy,aacz;            //加速度传感器原始数据
short gyrox,gyroy,gyroz;        //陀螺仪原始数据
IFX_INTERRUPT(CCU60_CH0_IRQHandler, CCU60_VECTABNUM, CCU60_CH0_PRIORITY);
IFX_INTERRUPT(CCU60_CH1_IRQHandler, CCU60_VECTABNUM, CCU60_CH1_PRIORITY);
IFX_INTERRUPT(CCU61_CH0_IRQHandler, CCU61_VECTABNUM, CCU61_CH0_PRIORITY);
IFX_INTERRUPT(CCU61_CH1_IRQHandler, CCU61_VECTABNUM, CCU61_CH1_PRIORITY);

/** CCU6中断CPU标号 */
const uint8 Ccu6IrqVectabNum[2] = {CCU60_VECTABNUM, CCU61_VECTABNUM};

/** CCU6中断优先级 */
const uint8 Ccu6IrqPriority[4] = {CCU60_CH0_PRIORITY, CCU60_CH1_PRIORITY, CCU61_CH0_PRIORITY, CCU61_CH1_PRIORITY};

/** CCU6中断服务函数地址 */
const void *Ccu6IrqFuncPointer[4] = {&CCU60_CH0_IRQHandler, &CCU60_CH1_IRQHandler, &CCU61_CH0_IRQHandler,
        &CCU61_CH1_IRQHandler};
void Imu_Update(float gx,float gy,float gz,float ax,float ay,float az);
void MPU6050(void);
static float invSqrt(float x);
/***********************************************************************************************/
/********************************CCU6外部中断  服务函数******************************************/
/***********************************************************************************************/

/*************************************************************************
 *  函数名称：void CCU60_CH0_IRQHandler(void)
 *  功能说明：
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年3月30日
 *  备    注：CCU60_CH0使用的中断服务函数
 *************************************************************************/
void CCU60_CH0_IRQHandler (void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
//   IfxCpu_enableInterrupts();
// 清除中断标志
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU60, IfxCcu6_InterruptSource_t12PeriodMatch);

    /* 用户代码 */

    // InductorGetSample();
}

/*************************************************************************
 *  函数名称：void CCU60_CH1_IRQHandler(void)
 *  功能说明：
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年3月30日
 *  备    注：CCU60_CH1使用的中断服务函数
 *************************************************************************/
void CCU60_CH1_IRQHandler (void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
    IfxCpu_enableInterrupts();

    // 清除中断标志
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU60, IfxCcu6_InterruptSource_t13PeriodMatch);

    /* 用户代码 */
    LED_Ctrl(LED1, RVS);        // 电平翻转,LED闪烁

}

/*************************************************************************
 *  函数名称：void CCU61_CH0_IRQHandler(void)
 *  功能说明：
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年3月30日
 *  备    注：CCU61_CH0使用的中断服务函数
 *************************************************************************/
void CCU61_CH0_IRQHandlerXXXXXXXXXXXXXXX (void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
    IfxCpu_enableInterrupts();

    // 清除中断标志
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t12PeriodMatch);

    /* 用户代码 */
    /* 获取编码器值 */
}



void CCU61_CH0_IRQHandler (void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
    IfxCpu_enableInterrupts();

    // 清除中断标志
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t12PeriodMatch);

    /* 用户代码 */
    /* 获取编码器值 */
    ECPULSE1 = ENC_GetCounter(ENC2_InPut_P33_7); // 左电机 母板上编码器1，小车前进为负值
    ECPULSE2 = -ENC_GetCounter(ENC4_InPut_P02_8); // 右电机 母板上编码器2，小车前进为正值
    int E1=ECPULSE1*20/5;
    int E2=ECPULSE2*20/5;
    static int count_fs=0;
    count_fs++;
    int SPEED=(int)(1.0*SPEED_SET*(1+speed_delta));
    float k_ftl=1.2;
    if(Kakm>0)
    {
        motor_PID(SPEED-(int)(k_ftl*Kakm*SPEED), E1, &error_last_11, &error_last_12, KP_motor, KI_motor, KD_motor, &duty1);
        motor_PID(SPEED, E2, &error_last_21, &error_last_22, KP_motor, KI_motor, KD_motor, &duty2);
    }
    else
    {
        motor_PID(SPEED, E1, &error_last_11, &error_last_12, KP_motor, KI_motor, KD_motor, &duty1);
        motor_PID(SPEED+(int)(k_ftl*Kakm*SPEED), E2, &error_last_21, &error_last_22, KP_motor, KI_motor, KD_motor, &duty2);
    }




    int delt=150;
    int bangbangduty=duty_max;
    int duty11,duty22;
    extern uint32 duty_smotor;
    static uint32 last_duty_smotor;
    if(last_duty_smotor!=duty_smotor)
        ServoCtrl(duty_smotor);
    last_duty_smotor=duty_smotor;
    if(Kakm>0)
    {
        if(E1-(SPEED-(int)(k_ftl*Kakm*SPEED))>delt)
        {
            duty11=- bangbangduty/2;
        }
        else if((SPEED-(int)(k_ftl*Kakm*SPEED))-E1>delt)
        {
            duty11=bangbangduty;
        }
        else
        {
            duty11=duty1;
        }
        if(E2-(SPEED)>delt)
        {
            duty22=- bangbangduty/2;
        }
        else if((SPEED)-E2>delt)
        {
            duty22=bangbangduty;
        }
        else
        {
            duty22=duty2;
        }
    }

    else if(Kakm<=0)
    {
        if(E1-(SPEED)>delt)
        {
            duty11=- bangbangduty/2;
        }
        else if((SPEED)-E1>delt)
        {
            duty11=bangbangduty;
        }
        else
        {
            duty11=duty1;
        }
        if(E2-(SPEED+(int)(k_ftl*Kakm*SPEED))>delt)
        {
            duty22=- bangbangduty/2;
        }
        else if((SPEED+(int)(k_ftl*Kakm*SPEED))-E2>delt)
        {
            duty22=bangbangduty;
        }
        else
        {
            duty22=duty2;
        }
    }

    //duty11=duty1;
    //duty22=duty2;
    //duty1=duty2=1000;
    extern uint32 smotor_pd, ck_left, ck_right;
    if(flag_pd==1&&smotor_pd==ck_left)
        duty11=0;
    if(flag_pd==1&&smotor_pd==ck_right)
        duty22=0;
    MotorCtrl4w(duty11, duty22, duty11, duty22);
    extern int inelement,ti;

    extern int det_error_mohu;
    extern int error_mohu;
    int er1=det_error_mohu;
    int er2=error_mohu;
    extern int xieru_flag;
    MPU6050();
    //ICM_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);

    if (count_fs>4)
    {
        count_fs=0;
        ANO_DT_send_int16byte16((int)(ti),(int)(inelement),(int)(Yaw),(int)(Pitch),(int)(gyroy),(int)(gyroz),(int)(duty11),(int)(duty22));
    }



     distc=Ad_Value();

    //RAllPulse += ECPULSE2;                       //
}
/*************************************************************************
 *  函数名称：void CCU61_CH1_IRQHandler(void)
 *  功能说明：
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2020年3月30日
 *  备    注：CCU61_CH1使用的中断服务函数
 *************************************************************************/
void CCU61_CH1_IRQHandler (void)
{
    /* 开启CPU中断  否则中断不可嵌套 */
    IfxCpu_enableInterrupts();

    // 清除中断标志
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t13PeriodMatch);

    /* 用户代码 */
    LED_Ctrl(LED0, RVS);        // 电平翻转,LED闪烁
}

/*************************************************************************
 *  函数名称：CCU6_InitConfig CCU6
 *  功能说明：定时器周期中断初始化
 *  参数说明：ccu6    ： ccu6模块            CCU60 、 CCU61
 *  参数说明：channel ： ccu6模块通道  CCU6_Channel0 、 CCU6_Channel1
 *  参数说明：us      ： ccu6模块  中断周期时间  单位us
 *  函数返回：无
 *  修改时间：2020年3月30日
 *  备    注：    CCU6_InitConfig(CCU60, CCU6_Channel0, 100);  // 100us进入一次中断
 *************************************************************************/
void CCU6_InitConfig (CCU6_t ccu6, CCU6_Channel_t channel, uint32 us)
{
    IfxCcu6_Timer_Config timerConfig;

    Ifx_CCU6 * module = IfxCcu6_getAddress((IfxCcu6_Index) ccu6);

    uint8 Index = ccu6 * 2 + channel;

    uint32 period = 0;

    uint64 clk = 0;

    /* 关闭中断 */
    boolean interrupt_state = disableInterrupts();

    IfxCcu6_Timer_initModuleConfig(&timerConfig, module);

    clk = IfxScuCcu_getSpbFrequency();

    /* 设置时钟频率  */
    uint8 i = 0;
    while (i++ < 16)
    {
        period = (uint32) (clk * us / 1000000);
        if (period < 0xffff)
        {
            break;
        }
        else
        {
            clk = clk / 2;
        }
    }
    switch (channel)
    {
        case CCU6_Channel0 :
            timerConfig.timer = IfxCcu6_TimerId_t12;
            timerConfig.interrupt1.source = IfxCcu6_InterruptSource_t12PeriodMatch;
            timerConfig.interrupt1.serviceRequest = IfxCcu6_ServiceRequest_1;
            timerConfig.base.t12Frequency = (float) clk;
            timerConfig.base.t12Period = period;                                  // 设置定时中断
            timerConfig.clock.t12countingInputMode = IfxCcu6_CountingInputMode_internal;
            timerConfig.timer12.counterValue = 0;
            timerConfig.interrupt1.typeOfService = Ccu6IrqVectabNum[ccu6];
            timerConfig.interrupt1.priority = Ccu6IrqPriority[Index];
            break;

        case CCU6_Channel1 :
            timerConfig.timer = IfxCcu6_TimerId_t13;
            timerConfig.interrupt2.source = IfxCcu6_InterruptSource_t13PeriodMatch;
            timerConfig.interrupt2.serviceRequest = IfxCcu6_ServiceRequest_2;
            timerConfig.base.t13Frequency = (float) clk;
            timerConfig.base.t13Period = period;
            timerConfig.clock.t13countingInputMode = IfxCcu6_CountingInputMode_internal;
            timerConfig.timer13.counterValue = 0;
            timerConfig.interrupt2.typeOfService = Ccu6IrqVectabNum[ccu6];
            timerConfig.interrupt2.priority = Ccu6IrqPriority[Index];
            break;
    }

    timerConfig.trigger.t13InSyncWithT12 = FALSE;

    IfxCcu6_Timer Ccu6Timer;

    IfxCcu6_Timer_initModule(&Ccu6Timer, &timerConfig);

    IfxCpu_Irq_installInterruptHandler((void*) Ccu6IrqFuncPointer[Index], Ccu6IrqPriority[Index]);          // 配置中断函数和中断号

    restoreInterrupts(interrupt_state);

    IfxCcu6_Timer_start(&Ccu6Timer);
}

/*************************************************************************
 *  函数名称：CCU6_DisableInterrupt(CCU6_t ccu6, CCU6_Channel_t channel)
 *  功能说明：停止CCU6通道中断
 *  参数说明：ccu6    ： ccu6模块            CCU60 、 CCU61
 *  参数说明：channel ： ccu6模块通道  CCU6_Channel0 、 CCU6_Channel1
 *  函数返回：无
 *  修改时间：2020年5月6日
 *  备    注：
 *************************************************************************/
void CCU6_DisableInterrupt (CCU6_t ccu6, CCU6_Channel_t channel)
{
    Ifx_CCU6 * module = IfxCcu6_getAddress((IfxCcu6_Index) ccu6);
    IfxCcu6_clearInterruptStatusFlag(module, (IfxCcu6_InterruptSource) (7 + channel * 2));
    IfxCcu6_disableInterrupt(module, (IfxCcu6_InterruptSource) (7 + channel * 2));

}

/*************************************************************************
 *  函数名称：CCU6_DisableInterrupt(CCU6_t ccu6, CCU6_Channel_t channel)
 *  功能说明：使能CCU6通道中断
 *  参数说明：ccu6    ： ccu6模块            CCU60 、 CCU61
 *  参数说明：channel ： ccu6模块通道  CCU6_Channel0 、 CCU6_Channel1
 *  函数返回：无
 *  修改时间：2020年5月6日
 *  备    注：
 *************************************************************************/
void CCU6_EnableInterrupt (CCU6_t ccu6, CCU6_Channel_t channel)
{
    Ifx_CCU6 * module = IfxCcu6_getAddress((IfxCcu6_Index) ccu6);
    IfxCcu6_clearInterruptStatusFlag(module, (IfxCcu6_InterruptSource) (7 + channel * 2));
    IfxCcu6_enableInterrupt(module, (IfxCcu6_InterruptSource) (7 + channel * 2));

}

void MPU6050(void)
{
        ICM_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);
        //MPU_Get_Raw_data(&Accel_x,&Accel_y,&Accel_z,&Gyro_x,&Gyro_y,&Gyro_z);   //得到加速度传感器数据
        ax = (9.8*aacx)/8192;
        ay = (9.8*aacy)/8192;
        az = (9.8*aacz)/8192;
        Gyro_x = (gyrox)/16.4*3.14159/180;
        Gyro_y = (gyroy)/16.4*3.14159/180;
        Gyro_z = (gyroz)/16.4*3.14159/180;

        Imu_Update(Gyro_x,Gyro_y,Gyro_z,ax,ay,az);
}




#define Acc_Gain 0.0001220f         //加速度转换单位(初始化加速度计量程+-4g，由于mpu6050的数据寄存器是16位的，LSBa = 2*4 / 65535.0)
#define Gyro_Gain 0.0609756f        //角速度转换为角度(LSBg = 2000*2 / 65535)
#define Gyro_Gr 0.0010641f          //角速度转换成弧度(3.1415 / 180 * LSBg)
#define G 9.7936f                  // m/s^2



static float invSqrt(float x)       //快速计算 1/Sqrt(x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}





#define Kp 1.50f
#define Ki 0.005f
#define halfT 0.0025f                       //计算周期的一半，单位s

extern float  last_Yaw,Yaw,Pitch,Roll;                //我要给其他文件用所以定义了extern，不用管
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;       //四元数
float exInt = 0, eyInt = 0, ezInt = 0;      //叉积计算误差的累计积分


void Imu_Update(float gx,float gy,float gz,float ax,float ay,float az)
{
    //int i;
    float vx,vy,vz;                         //实际重力加速度
    float ex,ey,ez;                         //叉积计算的误差
    float norm;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    if(ax*ay*az == 0)
        return;

    //加速度计测量的重力方向(机体坐标系)
    norm = invSqrt(ax*ax + ay*ay + az*az);          //之前这里写成invSqrt(ax*ax + ay+ay + az*az)是错误的，现在修改过来了
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    //四元数推出的实际重力方向(机体坐标系)
    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    //叉积误差
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    //叉积误差积分为角速度
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    //角速度补偿
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;

    //更新四元数
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    //单位化四元数
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    //四元数反解欧拉角
    Yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)* 57.3f;
    Pitch = -asin(2.f * (q1q3 - q0q2))* 57.3f;
    Roll = atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f;

}

/*

#define delta_T     0.001f  // 采样周期1ms 即频率1KHZ
#define PI          3.1415926f

float I_ex, I_ey, I_ez;  // 误差积分
quater_param_t Q_info = {1, 0, 0, 0};  // 四元数初始化
euler_param_t eulerAngle;              // 欧拉角
icm_param_t icm_data;                  // ICM20602采集的六轴数值
float icm_kp= 0.17;    // 加速度计的收敛速率比例增益
float icm_ki= 0.004;   // 陀螺仪收敛速率的积分增益
/**
 * @brief 用互补滤波算法解算陀螺仪姿态(即利用加速度计修正陀螺仪的积分误差)
 * 加速度计对振动之类的噪声比较敏感，长期数据计算出的姿态可信；陀螺仪对振动噪声不敏感，短期数据可信，但长期使用积分误差严重(内部积分算法放大静态误差)。
 * 因此使用姿态互补滤波，短期相信陀螺仪，长期相信加速度计。
 * @tips: n - 导航坐标系； b - 载体坐标系
 *//*
void icmAHRSupdate(icm_param_t* icm)
{
    float halfT = 0.5 * delta_T;    // 采样周期一半
    float vx, vy, vz;               // 当前姿态计算得来的重力在三轴上的分量
    float ex, ey, ez;               // 当前加速计测得的重力加速度在三轴上的分量与用当前姿态计算得来的重力在三轴上的分量的误差

    float q0 = Q_info.q0;  //四元数
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;

    float q0q0 = q0 * q0;  //先相乘，方便后续计算
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // 正常静止状态为-g 反作用力。
    if(icm->acc_x * icm->acc_y * icm->acc_z == 0) // 加计处于自由落体状态时(此时g = 0)不进行姿态解算，因为会产生分母无穷大的情况
        return;

    // 对加速度数据进行归一化 得到单位加速度 (a^b -> 载体坐标系下的加速度)
    float norm = myRsqrt(icm->acc_x * icm->acc_x + icm->acc_y * icm->acc_y + icm->acc_z * icm->acc_z);
    icm->acc_x = icm->acc_x * norm;
    icm->acc_y = icm->acc_y * norm;
    icm->acc_z = icm->acc_z * norm;

    // 载体坐标系下重力在三个轴上的分量
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // g^b 与 a^b 做向量叉乘，得到陀螺仪的校正补偿向量e的系数
    ex = icm->acc_y * vz - icm->acc_z * vy;
    ey = icm->acc_z * vx - icm->acc_x * vz;
    ez = icm->acc_x * vy - icm->acc_y * vx;

    // 误差累加
    I_ex += halfT * ex;
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    // 使用PI控制器消除向量积误差(陀螺仪漂移误差)
    icm->gyro_x = icm->gyro_x + icm_kp* ex + icm_ki* I_ex;
    icm->gyro_y = icm->gyro_y + icm_kp* ey + icm_ki* I_ey;
    icm->gyro_z = icm->gyro_z + icm_kp* ez + icm_ki* I_ez;

    // 一阶龙格库塔法求解四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为b系陀螺仪角速度。
    q0 = q0 + (-q1 * icm->gyro_x - q2 * icm->gyro_y - q3 * icm->gyro_z) * halfT;
    q1 = q1 + (q0 * icm->gyro_x + q2 * icm->gyro_z - q3 * icm->gyro_y) * halfT;
    q2 = q2 + (q0 * icm->gyro_y - q1 * icm->gyro_z + q3 * icm->gyro_x) * halfT;
    q3 = q3 + (q0 * icm->gyro_z + q1 * icm->gyro_y - q2 * icm->gyro_x) * halfT;

    // 单位化四元数在空间旋转时不会拉伸，仅有旋转角度，下面算法类似线性代数里的正交变换
    norm = myRsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;  // 用全局变量记录上一次计算的四元数值
}
*/
