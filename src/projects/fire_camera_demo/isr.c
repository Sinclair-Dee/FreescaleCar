/******************** (C) COPYRIGHT 2011 野火嵌入式开发工作室 ********************
 * 文件名       ：isr.c
 * 描述         ：中断处理例程
 *
 * 实验平台     ：野火kinetis开发板
 * 库版本       ：
 * 嵌入系统     ：
 *
 * 作者         ：野火嵌入式开发工作室
 * 淘宝店       ：http://firestm32.taobao.com
 * 技术支持论坛 ：http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
**********************************************************************************/



#include "common.h"
#include "include.h"
/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：PORTE_IRQHandler
*  功能说明：PORTE端口中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-9-17    已测试
*  备    注：引脚号需要自己初始化来清除
*************************************************************************/
void PORTE_IRQHandler()
{
    u8  n;    //引脚号
   // DEBUG_OUT("intoPORTE");

    n = 27;
    if(PORTE_ISFR & (1 << n))           //PTA26触发中断
    {
        PORTE_ISFR  |= (1 << n);        //写1清中断标志位
        disable_irq(PIT1 + 68);         //关闭PIT1中断，避免影响无线模块收发

        NRF_Handler();                  //无线模块中断处理函数
        enable_irq(PIT1 + 68);         //关闭PIT1中断，避免影响无线模块收发
    }
}

volatile u16 Vnum = 0;

/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：PORTA_IRQHandler
*  功能说明：PORTA端口中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-1-25    已测试
*  备    注：引脚号需要自己初始化来清除
*************************************************************************/
void PORTB_IRQHandler()
{
    u8  n = 0;    //引脚号
    //Site_t site={10,90};
    //   volatile u16 i;
    /*==============================================================================
    注意：
    	场中断放在行中断前面，即优先级高于行中断。
    	场中断用户任务里，全部都return，确保不进入行中断的用户任务。

    ==============================================================================*/
    n = 3;	//场中断
    
    if(PORTB_ISFR & (1 << n))           			//PTA29触发中断
    {

        /*  以下为用户任务  */

        //场中断需要判断是场结束还是场开始
        if(img_flag == IMG_START)					//需要开始采集图像
        {   //DEBUG_OUT("intoPORTB");
            img_flag = IMG_GATHER;					//标记图像采集中
            disable_irq(88);

            DMA_EN(CAMERA_DMA_CH);            		//使能通道CHn 硬件请求

            DMA_DADDR(CAMERA_DMA_CH) = (u32)IMG_BUFF;    //恢复地址
            //DEBUG_OUT("OUT PORTB");
        }
#ifdef DEBUG
        else if(img_flag == IMG_GATHER)				//图像采集中进入场中断，即图像采集完毕
        {   
            DEBUG_OUT("警告:DMA采集异常");
            while(1); //DMA采集异常
        }
#endif
        else										//图像采集错误
        {
            disable_irq(88); 						//关闭PTA的中断
            //DMA_IRQ_DIS(CAMERA_DMA_CH);	                //关闭通道CHn 中断请求
            //LCD_Str(site,"ERROR",BLUE,RED);
            img_flag = IMG_FAIL;					//标记图像采集失败
        }
        PORTB_ISFR  = ~0;        			        //场中断里，全部都要清中断标志位
        return;										//场中断触发了，就不需要处理行中断

        /*  以上为用户任务  */
    }


    PORTB_ISFR  = ~0; //写1清中断标志位
    DEBUG_OUT("OUT PORTB");
}
volatile u32 LPT_INT_count=0;
volatile u8 pit_flag=0;
void LPT_Handler(void)
{
  LPTMR0_CSR|=LPTMR_CSR_TCF_MASK;
  LPT_INT_count++;
}
/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：PIT0_IRQHandler
*  功能说明：PIT0 定时中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-18    已测试
*  备    注：
*************************************************************************/
extern int cesu;
extern u16 count;

void PIT0_IRQHandler(void)
{    
    							//允许PTA的中断
    PIT_Flag_Clear(PIT0); //清中断标志位
    count=LPTMR0_CNR;
    lptmr_counter_clean();
    LPTMR_CSR_REG(LPTMR0_BASE_PTR)&=~(LPTMR_CSR_TEN_MASK|LPTMR_CSR_TIE_MASK);
    disable_irq(85u);
    PIT_TCTRL(PIT0)&=~(PIT_TCTRL_TEN_MASK|PIT_TCTRL_TIE_MASK);
    disable_irq(PIT0+68);
    cesu=LPT_INT_count*100+count;
    uart_putchar(UART1,cesu);
    LPT_INT_count=0;
    
}

/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：PIT1_IRQHandler
*  功能说明：PIT1 定时中断服务函数
*  参数说明：无
*  函数返回：无
*  修改时间：2012-11-18    已测试
*  备    注：按键处理服务函数
*************************************************************************/
void PIT1_IRQHandler(void)
{ // DEBUG_OUT("into  PIT1");
    key_IRQHandler();
    PIT_Flag_Clear(PIT1);       				//清中断标志位
}



void DMA0_IRQHandler()
{   //DEBUG_OUT("into DMA");
    volatile u8 i;
    img_flag = IMG_FINISH ;
    //disable_irq(87);                      //场中断开始采集图片时，就关闭了中断，这里就不需要再关闭
    //DMA_DIS(CAMERA_DMA_CH);            	//关闭通道CHn 硬件请求
    DMA_IRQ_CLEAN(CAMERA_DMA_CH);
    //清除通道传输中断标志位

   // Vnum++;
   // i++;
   // i++;
   // i++;
    //i++;                        //延时，避免超频时，DMA不稳定而上访
}

