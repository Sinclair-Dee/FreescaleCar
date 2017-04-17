/******************** (C) COPYRIGHT 2011 Ұ��Ƕ��ʽ���������� ********************
 * �ļ���       ��isr.c
 * ����         ���жϴ�������
 *
 * ʵ��ƽ̨     ��Ұ��kinetis������
 * ��汾       ��
 * Ƕ��ϵͳ     ��
 *
 * ����         ��Ұ��Ƕ��ʽ����������
 * �Ա���       ��http://firestm32.taobao.com
 * ����֧����̳ ��http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
**********************************************************************************/



#include "common.h"
#include "include.h"
/*************************************************************************
*                             Ұ��Ƕ��ʽ����������
*
*  �������ƣ�PORTE_IRQHandler
*  ����˵����PORTE�˿��жϷ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-9-17    �Ѳ���
*  ��    ע�����ź���Ҫ�Լ���ʼ�������
*************************************************************************/
void PORTE_IRQHandler()
{
    u8  n;    //���ź�
   // DEBUG_OUT("intoPORTE");

    n = 27;
    if(PORTE_ISFR & (1 << n))           //PTA26�����ж�
    {
        PORTE_ISFR  |= (1 << n);        //д1���жϱ�־λ
        disable_irq(PIT1 + 68);         //�ر�PIT1�жϣ�����Ӱ������ģ���շ�

        NRF_Handler();                  //����ģ���жϴ�����
        enable_irq(PIT1 + 68);         //�ر�PIT1�жϣ�����Ӱ������ģ���շ�
    }
}

volatile u16 Vnum = 0;

/*************************************************************************
*                             Ұ��Ƕ��ʽ����������
*
*  �������ƣ�PORTA_IRQHandler
*  ����˵����PORTA�˿��жϷ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-1-25    �Ѳ���
*  ��    ע�����ź���Ҫ�Լ���ʼ�������
*************************************************************************/
void PORTB_IRQHandler()
{
    u8  n = 0;    //���ź�
    //Site_t site={10,90};
    //   volatile u16 i;
    /*==============================================================================
    ע�⣺
    	���жϷ������ж�ǰ�棬�����ȼ��������жϡ�
    	���ж��û������ȫ����return��ȷ�����������жϵ��û�����

    ==============================================================================*/
    n = 3;	//���ж�
    
    if(PORTB_ISFR & (1 << n))           			//PTA29�����ж�
    {

        /*  ����Ϊ�û�����  */

        //���ж���Ҫ�ж��ǳ��������ǳ���ʼ
        if(img_flag == IMG_START)					//��Ҫ��ʼ�ɼ�ͼ��
        {   //DEBUG_OUT("intoPORTB");
            img_flag = IMG_GATHER;					//���ͼ��ɼ���
            disable_irq(88);

            DMA_EN(CAMERA_DMA_CH);            		//ʹ��ͨ��CHn Ӳ������

            DMA_DADDR(CAMERA_DMA_CH) = (u32)IMG_BUFF;    //�ָ���ַ
            //DEBUG_OUT("OUT PORTB");
        }
#ifdef DEBUG
        else if(img_flag == IMG_GATHER)				//ͼ��ɼ��н��볡�жϣ���ͼ��ɼ����
        {   
            DEBUG_OUT("����:DMA�ɼ��쳣");
            while(1); //DMA�ɼ��쳣
        }
#endif
        else										//ͼ��ɼ�����
        {
            disable_irq(88); 						//�ر�PTA���ж�
            //DMA_IRQ_DIS(CAMERA_DMA_CH);	                //�ر�ͨ��CHn �ж�����
            //LCD_Str(site,"ERROR",BLUE,RED);
            img_flag = IMG_FAIL;					//���ͼ��ɼ�ʧ��
        }
        PORTB_ISFR  = ~0;        			        //���ж��ȫ����Ҫ���жϱ�־λ
        return;										//���жϴ����ˣ��Ͳ���Ҫ�������ж�

        /*  ����Ϊ�û�����  */
    }


    PORTB_ISFR  = ~0; //д1���жϱ�־λ
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
*                             Ұ��Ƕ��ʽ����������
*
*  �������ƣ�PIT0_IRQHandler
*  ����˵����PIT0 ��ʱ�жϷ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-2-18    �Ѳ���
*  ��    ע��
*************************************************************************/
extern int cesu;
extern u16 count;

void PIT0_IRQHandler(void)
{    
    							//����PTA���ж�
    PIT_Flag_Clear(PIT0); //���жϱ�־λ
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
*                             Ұ��Ƕ��ʽ����������
*
*  �������ƣ�PIT1_IRQHandler
*  ����˵����PIT1 ��ʱ�жϷ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-11-18    �Ѳ���
*  ��    ע���������������
*************************************************************************/
void PIT1_IRQHandler(void)
{ // DEBUG_OUT("into  PIT1");
    key_IRQHandler();
    PIT_Flag_Clear(PIT1);       				//���жϱ�־λ
}



void DMA0_IRQHandler()
{   //DEBUG_OUT("into DMA");
    volatile u8 i;
    img_flag = IMG_FINISH ;
    //disable_irq(87);                      //���жϿ�ʼ�ɼ�ͼƬʱ���͹ر����жϣ�����Ͳ���Ҫ�ٹر�
    //DMA_DIS(CAMERA_DMA_CH);            	//�ر�ͨ��CHn Ӳ������
    DMA_IRQ_CLEAN(CAMERA_DMA_CH);
    //���ͨ�������жϱ�־λ

   // Vnum++;
   // i++;
   // i++;
   // i++;
    //i++;                        //��ʱ�����ⳬƵʱ��DMA���ȶ����Ϸ�
}

