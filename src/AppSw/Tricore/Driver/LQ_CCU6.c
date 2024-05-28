/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
 ��ƽ    ̨�������������ܿƼ�TC264DA���İ�
 ����    д��ZYF/chiusir
 ��E-mail  ��chiusir@163.com
 ������汾��V1.1 ��Ȩ���У���λʹ��������ϵ��Ȩ
 �������¡�2020��10��28��
 �������Ϣ�ο����е�ַ��
 ����    վ��http:// www.lqist.cn
 ���Ա����̡�http:// longqiu.taobao.com
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
 ________________________________________________________________

 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ
 *  ��    ע��TC264 ������CCU6ģ��  ÿ��ģ��������������ʱ��  ������ʱ���ж�
 QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include <include.h> //����ģ���ͷ�ļ�
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
volatile sint16 ECPULSE1 = 0;          // �ٶ�ȫ�ֱ���
volatile sint16 ECPULSE2 = 0;          // �ٶ�ȫ�ֱ���
volatile sint32 RAllPulse = 0;          // �ٶ�ȫ�ֱ���
extern unsigned short distc;
extern float speed_delta;
short aacx,aacy,aacz;            //���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;        //������ԭʼ����
IFX_INTERRUPT(CCU60_CH0_IRQHandler, CCU60_VECTABNUM, CCU60_CH0_PRIORITY);
IFX_INTERRUPT(CCU60_CH1_IRQHandler, CCU60_VECTABNUM, CCU60_CH1_PRIORITY);
IFX_INTERRUPT(CCU61_CH0_IRQHandler, CCU61_VECTABNUM, CCU61_CH0_PRIORITY);
IFX_INTERRUPT(CCU61_CH1_IRQHandler, CCU61_VECTABNUM, CCU61_CH1_PRIORITY);

/** CCU6�ж�CPU��� */
const uint8 Ccu6IrqVectabNum[2] = {CCU60_VECTABNUM, CCU61_VECTABNUM};

/** CCU6�ж����ȼ� */
const uint8 Ccu6IrqPriority[4] = {CCU60_CH0_PRIORITY, CCU60_CH1_PRIORITY, CCU61_CH0_PRIORITY, CCU61_CH1_PRIORITY};

/** CCU6�жϷ�������ַ */
const void *Ccu6IrqFuncPointer[4] = {&CCU60_CH0_IRQHandler, &CCU60_CH1_IRQHandler, &CCU61_CH0_IRQHandler,
        &CCU61_CH1_IRQHandler};
void Imu_Update(float gx,float gy,float gz,float ax,float ay,float az);
void MPU6050(void);
static float invSqrt(float x);
/***********************************************************************************************/
/********************************CCU6�ⲿ�ж�  ������******************************************/
/***********************************************************************************************/

/*************************************************************************
 *  �������ƣ�void CCU60_CH0_IRQHandler(void)
 *  ����˵����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��30��
 *  ��    ע��CCU60_CH0ʹ�õ��жϷ�����
 *************************************************************************/
void CCU60_CH0_IRQHandler (void)
{
    /* ����CPU�ж�  �����жϲ���Ƕ�� */
//   IfxCpu_enableInterrupts();
// ����жϱ�־
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU60, IfxCcu6_InterruptSource_t12PeriodMatch);

    /* �û����� */

    // InductorGetSample();
}

/*************************************************************************
 *  �������ƣ�void CCU60_CH1_IRQHandler(void)
 *  ����˵����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��30��
 *  ��    ע��CCU60_CH1ʹ�õ��жϷ�����
 *************************************************************************/
void CCU60_CH1_IRQHandler (void)
{
    /* ����CPU�ж�  �����жϲ���Ƕ�� */
    IfxCpu_enableInterrupts();

    // ����жϱ�־
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU60, IfxCcu6_InterruptSource_t13PeriodMatch);

    /* �û����� */
    LED_Ctrl(LED1, RVS);        // ��ƽ��ת,LED��˸

}

/*************************************************************************
 *  �������ƣ�void CCU61_CH0_IRQHandler(void)
 *  ����˵����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��30��
 *  ��    ע��CCU61_CH0ʹ�õ��жϷ�����
 *************************************************************************/
void CCU61_CH0_IRQHandlerXXXXXXXXXXXXXXX (void)
{
    /* ����CPU�ж�  �����жϲ���Ƕ�� */
    IfxCpu_enableInterrupts();

    // ����жϱ�־
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t12PeriodMatch);

    /* �û����� */
    /* ��ȡ������ֵ */
}



void CCU61_CH0_IRQHandler (void)
{
    /* ����CPU�ж�  �����жϲ���Ƕ�� */
    IfxCpu_enableInterrupts();

    // ����жϱ�־
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t12PeriodMatch);

    /* �û����� */
    /* ��ȡ������ֵ */
    ECPULSE1 = ENC_GetCounter(ENC2_InPut_P33_7); // ���� ĸ���ϱ�����1��С��ǰ��Ϊ��ֵ
    ECPULSE2 = -ENC_GetCounter(ENC4_InPut_P02_8); // �ҵ�� ĸ���ϱ�����2��С��ǰ��Ϊ��ֵ
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
 *  �������ƣ�void CCU61_CH1_IRQHandler(void)
 *  ����˵����
 *  ����˵������
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��30��
 *  ��    ע��CCU61_CH1ʹ�õ��жϷ�����
 *************************************************************************/
void CCU61_CH1_IRQHandler (void)
{
    /* ����CPU�ж�  �����жϲ���Ƕ�� */
    IfxCpu_enableInterrupts();

    // ����жϱ�־
    IfxCcu6_clearInterruptStatusFlag(&MODULE_CCU61, IfxCcu6_InterruptSource_t13PeriodMatch);

    /* �û����� */
    LED_Ctrl(LED0, RVS);        // ��ƽ��ת,LED��˸
}

/*************************************************************************
 *  �������ƣ�CCU6_InitConfig CCU6
 *  ����˵������ʱ�������жϳ�ʼ��
 *  ����˵����ccu6    �� ccu6ģ��            CCU60 �� CCU61
 *  ����˵����channel �� ccu6ģ��ͨ��  CCU6_Channel0 �� CCU6_Channel1
 *  ����˵����us      �� ccu6ģ��  �ж�����ʱ��  ��λus
 *  �������أ���
 *  �޸�ʱ�䣺2020��3��30��
 *  ��    ע��    CCU6_InitConfig(CCU60, CCU6_Channel0, 100);  // 100us����һ���ж�
 *************************************************************************/
void CCU6_InitConfig (CCU6_t ccu6, CCU6_Channel_t channel, uint32 us)
{
    IfxCcu6_Timer_Config timerConfig;

    Ifx_CCU6 * module = IfxCcu6_getAddress((IfxCcu6_Index) ccu6);

    uint8 Index = ccu6 * 2 + channel;

    uint32 period = 0;

    uint64 clk = 0;

    /* �ر��ж� */
    boolean interrupt_state = disableInterrupts();

    IfxCcu6_Timer_initModuleConfig(&timerConfig, module);

    clk = IfxScuCcu_getSpbFrequency();

    /* ����ʱ��Ƶ��  */
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
            timerConfig.base.t12Period = period;                                  // ���ö�ʱ�ж�
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

    IfxCpu_Irq_installInterruptHandler((void*) Ccu6IrqFuncPointer[Index], Ccu6IrqPriority[Index]);          // �����жϺ������жϺ�

    restoreInterrupts(interrupt_state);

    IfxCcu6_Timer_start(&Ccu6Timer);
}

/*************************************************************************
 *  �������ƣ�CCU6_DisableInterrupt(CCU6_t ccu6, CCU6_Channel_t channel)
 *  ����˵����ֹͣCCU6ͨ���ж�
 *  ����˵����ccu6    �� ccu6ģ��            CCU60 �� CCU61
 *  ����˵����channel �� ccu6ģ��ͨ��  CCU6_Channel0 �� CCU6_Channel1
 *  �������أ���
 *  �޸�ʱ�䣺2020��5��6��
 *  ��    ע��
 *************************************************************************/
void CCU6_DisableInterrupt (CCU6_t ccu6, CCU6_Channel_t channel)
{
    Ifx_CCU6 * module = IfxCcu6_getAddress((IfxCcu6_Index) ccu6);
    IfxCcu6_clearInterruptStatusFlag(module, (IfxCcu6_InterruptSource) (7 + channel * 2));
    IfxCcu6_disableInterrupt(module, (IfxCcu6_InterruptSource) (7 + channel * 2));

}

/*************************************************************************
 *  �������ƣ�CCU6_DisableInterrupt(CCU6_t ccu6, CCU6_Channel_t channel)
 *  ����˵����ʹ��CCU6ͨ���ж�
 *  ����˵����ccu6    �� ccu6ģ��            CCU60 �� CCU61
 *  ����˵����channel �� ccu6ģ��ͨ��  CCU6_Channel0 �� CCU6_Channel1
 *  �������أ���
 *  �޸�ʱ�䣺2020��5��6��
 *  ��    ע��
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
        //MPU_Get_Raw_data(&Accel_x,&Accel_y,&Accel_z,&Gyro_x,&Gyro_y,&Gyro_z);   //�õ����ٶȴ���������
        ax = (9.8*aacx)/8192;
        ay = (9.8*aacy)/8192;
        az = (9.8*aacz)/8192;
        Gyro_x = (gyrox)/16.4*3.14159/180;
        Gyro_y = (gyroy)/16.4*3.14159/180;
        Gyro_z = (gyroz)/16.4*3.14159/180;

        Imu_Update(Gyro_x,Gyro_y,Gyro_z,ax,ay,az);
}




#define Acc_Gain 0.0001220f         //���ٶ�ת����λ(��ʼ�����ٶȼ�����+-4g������mpu6050�����ݼĴ�����16λ�ģ�LSBa = 2*4 / 65535.0)
#define Gyro_Gain 0.0609756f        //���ٶ�ת��Ϊ�Ƕ�(LSBg = 2000*2 / 65535)
#define Gyro_Gr 0.0010641f          //���ٶ�ת���ɻ���(3.1415 / 180 * LSBg)
#define G 9.7936f                  // m/s^2



static float invSqrt(float x)       //���ټ��� 1/Sqrt(x)
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
#define halfT 0.0025f                       //�������ڵ�һ�룬��λs

extern float  last_Yaw,Yaw,Pitch,Roll;                //��Ҫ�������ļ������Զ�����extern�����ù�
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;       //��Ԫ��
float exInt = 0, eyInt = 0, ezInt = 0;      //������������ۼƻ���


void Imu_Update(float gx,float gy,float gz,float ax,float ay,float az)
{
    //int i;
    float vx,vy,vz;                         //ʵ���������ٶ�
    float ex,ey,ez;                         //�����������
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

    //���ٶȼƲ�������������(��������ϵ)
    norm = invSqrt(ax*ax + ay*ay + az*az);          //֮ǰ����д��invSqrt(ax*ax + ay+ay + az*az)�Ǵ���ģ������޸Ĺ�����
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    //��Ԫ���Ƴ���ʵ����������(��������ϵ)
    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    //������
    ex = (ay*vz - az*vy);
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    //���������Ϊ���ٶ�
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;

    //���ٶȲ���
    gx = gx + Kp*ex + exInt;
    gy = gy + Kp*ey + eyInt;
    gz = gz + Kp*ez + ezInt;

    //������Ԫ��
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    //��λ����Ԫ��
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    //��Ԫ������ŷ����
    Yaw = atan2(2.f * (q1q2 + q0q3), q0q0 + q1q1 - q2q2 - q3q3)* 57.3f;
    Pitch = -asin(2.f * (q1q3 - q0q2))* 57.3f;
    Roll = atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f;

}

/*

#define delta_T     0.001f  // ��������1ms ��Ƶ��1KHZ
#define PI          3.1415926f

float I_ex, I_ey, I_ez;  // ������
quater_param_t Q_info = {1, 0, 0, 0};  // ��Ԫ����ʼ��
euler_param_t eulerAngle;              // ŷ����
icm_param_t icm_data;                  // ICM20602�ɼ���������ֵ
float icm_kp= 0.17;    // ���ٶȼƵ��������ʱ�������
float icm_ki= 0.004;   // �������������ʵĻ�������
/**
 * @brief �û����˲��㷨������������̬(�����ü��ٶȼ����������ǵĻ������)
 * ���ٶȼƶ���֮��������Ƚ����У��������ݼ��������̬���ţ������Ƕ������������У��������ݿ��ţ�������ʹ�û����������(�ڲ������㷨�Ŵ�̬���)��
 * ���ʹ����̬�����˲����������������ǣ��������ż��ٶȼơ�
 * @tips: n - ��������ϵ�� b - ��������ϵ
 *//*
void icmAHRSupdate(icm_param_t* icm)
{
    float halfT = 0.5 * delta_T;    // ��������һ��
    float vx, vy, vz;               // ��ǰ��̬��������������������ϵķ���
    float ex, ey, ez;               // ��ǰ���ټƲ�õ��������ٶ��������ϵķ������õ�ǰ��̬��������������������ϵķ��������

    float q0 = Q_info.q0;  //��Ԫ��
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;

    float q0q0 = q0 * q0;  //����ˣ������������
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // ������ֹ״̬Ϊ-g ����������
    if(icm->acc_x * icm->acc_y * icm->acc_z == 0) // �Ӽƴ�����������״̬ʱ(��ʱg = 0)��������̬���㣬��Ϊ�������ĸ���������
        return;

    // �Լ��ٶ����ݽ��й�һ�� �õ���λ���ٶ� (a^b -> ��������ϵ�µļ��ٶ�)
    float norm = myRsqrt(icm->acc_x * icm->acc_x + icm->acc_y * icm->acc_y + icm->acc_z * icm->acc_z);
    icm->acc_x = icm->acc_x * norm;
    icm->acc_y = icm->acc_y * norm;
    icm->acc_z = icm->acc_z * norm;

    // ��������ϵ���������������ϵķ���
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // g^b �� a^b ��������ˣ��õ������ǵ�У����������e��ϵ��
    ex = icm->acc_y * vz - icm->acc_z * vy;
    ey = icm->acc_z * vx - icm->acc_x * vz;
    ez = icm->acc_x * vy - icm->acc_y * vx;

    // ����ۼ�
    I_ex += halfT * ex;
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    // ʹ��PI�������������������(������Ư�����)
    icm->gyro_x = icm->gyro_x + icm_kp* ex + icm_ki* I_ex;
    icm->gyro_y = icm->gyro_y + icm_kp* ey + icm_ki* I_ey;
    icm->gyro_z = icm->gyro_z + icm_kp* ez + icm_ki* I_ez;

    // һ����������������Ԫ��΢�ַ��̣�����halfTΪ�������ڵ�1/2��gx gy gzΪbϵ�����ǽ��ٶȡ�
    q0 = q0 + (-q1 * icm->gyro_x - q2 * icm->gyro_y - q3 * icm->gyro_z) * halfT;
    q1 = q1 + (q0 * icm->gyro_x + q2 * icm->gyro_z - q3 * icm->gyro_y) * halfT;
    q2 = q2 + (q0 * icm->gyro_y - q1 * icm->gyro_z + q3 * icm->gyro_x) * halfT;
    q3 = q3 + (q0 * icm->gyro_z + q1 * icm->gyro_y - q2 * icm->gyro_x) * halfT;

    // ��λ����Ԫ���ڿռ���תʱ�������죬������ת�Ƕȣ������㷨�������Դ�����������任
    norm = myRsqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;  // ��ȫ�ֱ�����¼��һ�μ������Ԫ��ֵ
}
*/
