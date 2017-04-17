#ifndef	_BL14002_H_
#define _BL14002_H_

#define	LCD_H	132		//��
#define LCD_W	132		//��

#define	wr		PTC9_OUT
#define	rd		PTC10_OUT
#define	cs		PTC11_OUT
#define	rs		PTC12_OUT
#define	reset	PTC19_OUT

#define P0			PTD_BYTE0_OUT
#define	PIN			PTD_BYTE0_IN
#define PDDR_OUT()	(DDRD_BYTE0 = 0xff)
#define PDDR_IN()	(DDRD_BYTE0 = 0)

#define	BL14002_DELAY()		//do{asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");asm("nop");}while(0)

typedef	struct
{
    u16 x;
    u16 y;
} Site_t;

typedef	struct
{
    u16 W;		//��
    u16 H;		//��
} Size_t;

#define LCD_RAMWR()		LCD_WR_CMD(0x2C)				        //���� Memory Write 
#define LCD_RAMRD()     do{LCD_WR_CMD(0x2E);rd_data();}while(0) //���� Memory Read 

//void 	wr_cmd(u8 command_data_8);
//void 	LCD_WR_Data(u8 low_8_data);
u8  	rd_data(void);
void 	lcd_initialize();
void 	LCD_PTLON(Site_t site, Size_t size);		//����

/*******************����Ϊд���д����*********************/
/*****���ں����϶̣�ִ�д���Ƶ��������д�ɺ궨�壬�ӿ��ٶ�*****/
#define LCD_WR_CMD(command_data_8)		do\
										{\
										rd=1;\
										BL14002_DELAY();\
										rs=0;\
										BL14002_DELAY();\
										cs=0;\
										BL14002_DELAY();\
										P0=(u8)(command_data_8);\
										wr=0;\
										BL14002_DELAY();\
										wr=1;\
										BL14002_DELAY();\
										cs=1;\
									}while(0)	//wr=0;wr=1;����һ��������

#define LCD_WR_Data(low_8_data)		do\
								{\
									rd=1;\
									rs=1;\
									cs=0;\
									BL14002_DELAY();\
									P0=(u8)(low_8_data);\
									wr=0;\
									wr=1;\
									cs=1;\
								}while(0) 	//wr=0;������д�����ݵ�RAM 

#define LCD_WR_16Data(DATA)        LCD_WR_Data((u8)((DATA)>>8) );LCD_WR_Data( (u8)(DATA) ) //д16λ����

#define LCD_WR_16Data_BIG(DATA)    LCD_WR_Data((u8)(DATA) );LCD_WR_Data( (u8)((DATA)>>8) ) //д16λ���ݣ����

#endif	//_BL14002_H_