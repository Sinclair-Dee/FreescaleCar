#ifndef __SCCB_H
#define __SCCB_H


#define SCL_H()         PTC10_OUT = 1
#define SCL_L()         PTC10_OUT = 0
#define	SCL_DDR_OUT() 	DDRC10 = 1
#define	SCL_DDR_IN() 	DDRC10 = 0

#define SDA_H()         PTC11_OUT = 1
#define SDA_L()         PTC11_OUT = 0
#define SDA_IN()      	PTC11_IN
#define SDA_DDR_OUT()	DDRC11 = 1
#define SDA_DDR_IN()	DDRC11 = 0

#define ADDR_OV7725   0x42

#define SCCB_DELAY()	SCCB_delay(100)


void SCCB_GPIO_init(void);
int SCCB_WriteByte( u16 WriteAddress , u8 SendByte);
int SCCB_ReadByte(u8 *pBuffer,   u16 length,   u8 ReadAddress);

static void SCCB_delay(u16 i);
#endif
