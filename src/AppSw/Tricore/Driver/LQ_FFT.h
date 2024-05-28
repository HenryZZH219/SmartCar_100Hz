/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技TC264DA核心板
【编    写】ZYF/chiusir
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

使用例程的时候，建议采用没有空格的英文路径，
除了CIF为TC264DA独有外，其它的代码兼容TC264D
本库默认初始化了EMEM：512K，如果用户使用TC264D，注释掉EMEM_InitConfig()初始化函数。
工程下\Libraries\iLLD\TC26B\Tricore\Cpu\CStart\IfxCpu_CStart0.c第164行左右。__________________________________________________________
注意  硬件FFT需要TC264 DA的芯片才有此功能
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef _LQ_FFT_H_
#define _LQ_FFT_H_

#include "Cpu/Std/Platform_Types.h"
#include "Fft/Std/IfxFft.h"
#include "stdint.h"
#include <Ifx_Types.h>
#include <IfxCpu.h>
#include <IfxCpu_Irq.h>
#include <IfxDma_cfg.h>
#include <IfxFft.h>
#include <IfxFft_Fft.h>
#include <IfxFft_reg.h>
#include <IfxLmu_reg.h>
#include <math.h>
#include <Platform_Types.h>
#include <stdint.h>

/** FFT中断服务函数优先级   范围：1-255   数字越大 优先级越高  注意优先级不要重复 */
#define  FFT_INPUT_PRIORITY    250

/** FFT中断服务函数优先级   范围：1-255   数字越大 优先级越高  注意优先级不要重复 */
#define  FFT_OUTPUT_PRIORITY   251

/** FFT中断服务函数优先级   范围：1-255   数字越大 优先级越高  注意优先级不要重复 */
#define  FFT_INTRA_PRIORITY    252

/** FFT中断归哪个内核管理？ 范围：0：CPU0   1：CPU1   3：DMA*/
#define  FFT_VECTABNUM         1




void FFT_InitConfig(uint32 InDataAddr, uint32 OutDataAddr, IfxFft_Length len);
void IFFT_InitConfig(uint32 InDataAddr, uint32 OutDataAddr, IfxFft_Length len);
void FFT_Start(void);

#endif /* 0_APPSW_TRICORE_APP_LQ_FFT_H_ */
