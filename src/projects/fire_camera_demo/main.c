///********************************智能车程序**************************************///
//                                                                                  //
//                                                                                  //
//                                                                                  //
//                                                                                  //
//                                                                                  //
//                                                                                  // 
//                                                                                  //
///********************************************************************************///

/****************************************START***************************************/

#include "common.h"
#include "include.h"
/**************************************舵机*******************************************/
#define steer_left    1000	  //左极限
#define steer_right   1550	  //右极限
#define steer_center  1265        //舵机中点
#define Middle_point 73
#define Select_line  60         //选定前瞻
#define FAZHI_shizhi 15
#define FAZHI_zhidao 3
#define zhidao 1
#define left  2
#define right  3
u32 angle;                      //角度
u8 Cnt_shizhi=0;
u8 Cnt_tend=10;
u8 flag_shizhi=0;
u8 flag_jiwan =0;
u8 flag_zhipao0=0;
u8 flag_zhipao1=0;
u8 flag_zhipao2=0;
u8 flag_zhipao3=0;
u8 flag_nochange=0;
u8 flag_loselineL=0;
u8 flag_loselineR=0;

float errorPRE;
float errorLAST;
float errorNOW;
float errorAVE;
float Kp=11.0;
float Kd=2.5;
u8 fangxiang=0;
u8 tend=0;
u8 end_rowC=0;
u8 end_rowL=45;
u8 end_rowR=45;
s8 aa1=0;
s8 aa2=0;
s8 aa3=0;
      
u8  tiaodian_z=130;
u8  tiaodian_y=130;
u8  guaidian=130;
/***********************************end***************************************/



/**********************************电机****************************************/
#define  Speed_FAZHI   600

#define wandao_OTHERS 1           //其他的弯道
#define On_straight   2           //直道
#define	In_straight   3           //弯内直道 
#define wandao_BAG    4           //大弯 
#define zhidao_wandao 5           //直入弯
#define wandao_zhidao 6           //弯入直道
#define wandao_xiaoS  7           //小S
#define Speedmax0     4000       
u8  flag_yunsu=0;
u8  Con_road;
u8  flag_stop=0;
u8  flog_zhipao=0;
u8  cnt_stop=0;
int Speed_zhidao ;
int Speed_wandao;
int Speed_jizhuanwan;
int Speed_stop=0; 
float KI=2.5;
float KP=1.0;
float KD=0.5;
int Speedmax=3000;
int Speedmin=100;
int Set_speed=2300;
int Sur_speed;
int Set_speed0=2000;
int PREerror;
int LASTerror;
int NOWerror;
int Speed_out;
int V_add;
u32 k;
int cesu;
u16 count;
u32 PWM_out;

/***********************************end****************************************/



/*************************************摄像头******************************************/

u8  nrf_buff[CAMERA_SIZE + MAX_ONCE_TX_NUM];     //预多
u8 *img_bin_buff = (u8 *)(((u8 *)&nrf_buff) + COM_LEN);  //二值化图像的buf指针，由于开头有 COM_LEN 个字节是留给校验，所以需要加 COM_LEN
u8 img_buff[120][160];
u8 i=0,j=0;
u8 mid[121],row_left[121],row_right[121];
u8 loseline[121];
u8 a,b;
u32 sum=1000;

/**************************************end********************************************/

u8 zhipao[240]={ 
 0x23, 0x71, 
 0x23, 0x71, 
 0x23, 0x71,
 0x24, 0x71,
 0x24, 0x70,
 0x24, 0x70,
 0x24, 0x70,
 0x25, 0x70,
 0x25, 0x6F,
 0x25, 0x6F,
 0x25, 0x6F,
 0x26, 0x6E,
 0x26, 0x6E,
 0x26, 0x6E, 
 0x27, 0x6E, 
 0x27, 0x6D, 
 0x27, 0x6D, 
 0x27, 0x6D, 
 0x28, 0x6D, 
 0x28, 0x6C, 
 0x28, 0x6C, 
 0x28, 0x6C, 
 0x29, 0x6B, 
 0x29, 0x6B, 
 0x29, 0x6B, 
 0x29, 0x6B, 
 0x2A, 0x6A, 
 0x2A, 0x6A, 
 0x2A, 0x6A, 
 0x2B, 0x69, 
 0x2B, 0x69, 
 0x2B, 0x69, 
 0x2B, 0x69,
 0x2C, 0x68, 
 0x2C, 0x68, 
 0x2C, 0x68, 
 0x2D, 0x67, 
 0x2D, 0x67, 
 0x2D, 0x67, 
 0x2D, 0x67, 
 0x2E, 0x66, 
 0x2E, 0x66, 
 0x2E, 0x66, 
 0x2F, 0x65, 
 0x2F, 0x65, 
 0x2F, 0x65, 
 0x30, 0x65, 
 0x30, 0x64, 
 0x30, 0x64, 
 0x31, 0x64, 
 0x31, 0x63, 
 0x31, 0x63, 
 0x31, 0x63, 
 0x32, 0x62, 
 0x32, 0x62, 
 0x32, 0x62, 
 0x33, 0x61, 
 0x33, 0x61, 
 0x33, 0x61, 
 0x34, 0x61, 
 0x34, 0x60, 
 0x34, 0x60, 
 0x34, 0x60, 
 0x35, 0x5F, 
 0x35, 0x5F, 
 0x35, 0x5F, 
 0x36, 0x5E, 
 0x36, 0x5E, 
 0x36, 0x5E, 
 0x37, 0x5D, 
 0x37, 0x5D, 
 0x37, 0x5D, 
 0x38, 0x5C, 
 0x38, 0x5C, 
 0x39, 0x5C, 
 0x39, 0x5B, 
 0x39, 0x5B, 
 0x3A, 0x5B, 
 0x3A, 0x5A, 
 0x3A, 0x5A, 
 0x3B, 0x5A, 
 0x3B, 0x59, 
 0x3B, 0x59, 
 0x3C, 0x59, 
 0x3C, 0x58, 
 0x3C, 0x58, 
 0x3D, 0x58, 
 0x3D, 0x57, 
 0x3D, 0x57, 
 0x3E, 0x57, 
 0x3E, 0x56, 
 0x3E, 0x56, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55,
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55,
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55,
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55, 
 0x3F, 0x55,
};
/*********************************最小二乘法拟合*************************************/
#define nihestart 35
u8 Cnt_nihe=10;
u8 nihe_i=0;
u8 chazhi0=0;
u8 chazhi1=0;
u8  nihe_fenjie;
//整场 
s32     X;
s32     Y;
s32     XY;
s32     XX;
float   X_AVE=0.0;
float   Y_AVE=0.0;
float   XY_AVE=0.0;
float   XX_AVE=0.0;
float   CK=0.0;
float   CB=0.0;
float   Nihe_panduan=0.0;

//近处
s32     X1;
s32     Y1;
s32     XY1;
s32     XX1;
float   X_AVE1=0.0;
float   Y_AVE1=0.0;
float   XY_AVE1=0.0;
float   XX_AVE1=0.0;
float   CK1=0.0;
float   CB1=0.0;

//远处
s32     X2;
s32     Y2;
s32     XY2;
s32     XX2;
float   X_AVE2=0.0;
float   Y_AVE2=0.0;
float   XY_AVE2=0.0;
float   XX_AVE2=0.0;
float   CK2=0.0;
float   CB2=0.0;
void switch_init(void)
{
    u8 k1,k2,k3,k4;u8 a;
    gpio_init (PORTC, 0, GPI_UP, 0);
    gpio_init (PORTC, 1 , GPI_UP, 0);
    gpio_init (PORTC, 2, GPI_UP, 0);
    gpio_init (PORTC, 3, GPI_UP, 0);
    
    k1=gpio_get(PORTC,1);
    k2=gpio_get(PORTC,3); 
    k3=gpio_get(PORTC,2); 
    k4=gpio_get(PORTC,0);
    a=k1*8+k2*4+k3*2+k4*1;
    flag_yunsu=0;
    switch(a)
    {
        case 0:  Speedmax=2000;break; 
        case 1:  Speedmax=2100;break; 
        case 2:  Speedmax=2200;break;
        case 3:  Speedmax=2300;break;
        case 4:  Speedmax=2400;break;
        case 5:  Speedmax=2500;break;
        case 6:  Speedmax=2600;break;
        case 7:  Speedmax=2700;break;
        case 8:  Speedmax=2800;break;
        case 9:  Speedmax=2900;break;
        case 10: Speedmax=3000;break;
        case 11: Speedmax=3100;break;
        case 12: Speedmax=3200;break;
        case 13: Speedmax=3300;break;
        case 14: Speedmax=3500;break;
        case 15: flag_yunsu=1; break;
    }
}



//残差
float   D=0.0;
float   D_AVE=0.0; //全场残差
/**************************************end********************************************/
extern volatile u32 LPT_INT_COUNT; int cesu; u16 count;
extern volatile u8 pit_flag;

void main(void)
{   switch_init();
    time_delay_ms(1000);
    time_delay_ms(1000);
  //全局变量初始化
    mid[120]=74;
    row_left[120]=0;
    row_right[120]=148;
    errorPRE= 0.0;
    errorLAST=0.0;
    errorAVE= 0.0;
    errorNOW= 0.0;
    angle=steer_center;
    PREerror=0.0;  
    LASTerror=0.0;
    NOWerror=0.0;
    V_add=0.0;
    Speed_out=Speed_zhidao;
  //end  
    uart_init (UART1, 128000);
    Ov7725_Init(img_bin_buff);          	    //摄像头初始化
    
    gpio_init(PORTD,5,GPO,0);             //配置电机
    FTM_PWM_init(FTM0,CH4,10000,2500);    
    FTM_PWM_init(FTM1,CH0,100,1265);      //配置舵机
    time_delay_ms(500);
    
    while(1)
    {

/************************图像处理**********************************************/    
        
        ov7725_get_img();                           //采集
        lptmr_counter_init(LPT0_ALT2,100,0,LPT_Rising);    //测速
        pit_init_ms(PIT0,10);
        img_extract((u8 *)img_buff,(u8 *)img_bin_buff,2400);      
         for(i=119;i>0;i--)                         // 图像处理，提取中心线
        {
          for(j=mid[i+1];j>0;j--)
          {
            if(img_buff[i][j]==0&&img_buff[i][j-1]==0&&img_buff[i][j+1]==255)
            {    row_left[i]=j;
                 break;
            }
          }
          if(j==0) 
              {
              row_left[i]=row_left[i+1];
              flag_loselineL=1;
              }
          else
              flag_loselineL=0;
                  
          for(j=mid[i+1];j<159;j++)
          {
            if(img_buff[i][j]==0&&img_buff[i][j+1]==0&&img_buff[i][j-1]==255)
            {
              row_right[i]=j;
              break;
            }
          }
          if(j==159) 
               {
                 row_right[i]=row_right[i+1];
                 flag_loselineR=1;
               }
          else
                 flag_loselineR=0;
          
          if(flag_loselineR==1&&flag_loselineL==1)
              loseline[i]=1;
          else
              loseline[i]=0;
          
      
            mid[i]=(row_left[i]+row_right[i])/2;          //中心线
   
         
            

        }  

      
        
        
     /************************结束行*************************/ 
        
       end_rowL=0; 
       end_rowC=0;
       end_rowR=0;
       for(i=119;i>0;i--)                        //左
           {
           if(img_buff[i][zhipao[238-2*i]]==0)
               { end_rowL=i;
                  break;
               }
           }  
       
        for(i=119;i>0;i--)                        //中
           {
           if(img_buff[i][74]==0)
               { end_rowC=i;
                  break;
               }
           } 
       
       for(i=119;i>0;i--)                        //右
           {
           if(img_buff[i][zhipao[239-2*i]]==0)
               { end_rowR=i;
                  break;
               }
           } 
       
      flag_zhipao0=0;
      flag_zhipao1=0;
      flag_zhipao2=0;
      flag_zhipao3=0;
      
      if(end_rowL<=50&&end_rowR<=50&&end_rowC<=50)
        flag_zhipao0=1;
      else
        flag_zhipao0=0;
      
      if(end_rowL<=44&&end_rowR<=44&&end_rowC<=44)
        flag_zhipao1=1;
      else
        flag_zhipao1=0;
      
      if(end_rowL<=35&&end_rowR<=35&&end_rowC<=35)
        flag_zhipao2=1;
      else
        flag_zhipao2=0;
      
      if(end_rowL<=30&&end_rowR<=30&&end_rowC<=30)
        flag_zhipao3=1;
      else
        flag_zhipao3=0;
      

   
      
/********************************end**********************************/
     errorAVE=mid[Select_line]+mid[Select_line-1]+mid[Select_line-2]+mid[Select_line+1]+mid[Select_line+2];     //计算偏差
     errorAVE/=5;
     
     errorPRE= errorLAST;
     errorLAST=errorNOW;
     errorNOW =(float)(Middle_point-errorAVE) ;  
     
 /************************end**********************************************/  

/******************************************最小二乘法拟合***********************************************/
     /***********整场拟合***********/
     X=0;
     Y=0;
     XY=0;
     XX=0;
     Cnt_nihe=10;
     flag_shizhi=0;
     Cnt_shizhi=0;
     for(i=119;i>=nihestart;i--)
          {
          if(mid[i]-mid[i-1]>0)    //滤除噪点
            chazhi0=mid[i]-mid[i-1];
          else
            chazhi0=mid[i-1]-mid[i];
          if(mid[i]-mid[i+1]>0)
            chazhi1=mid[i]-mid[i+1];
          else
            chazhi1=mid[i+1]-mid[i];  
          if(chazhi0>=5&&chazhi1>=5)
             continue;
          if(img_buff[i-1][mid[i]]==0||row_left[i]>=155||row_right[i]<=5)
             break;
          X+=(119-i);
          Y+=mid[i];
          XY+=mid[i]*(119-i);
          XX+=(119-i)*(119-i);
          Cnt_nihe++;
          }
       nihe_i=i+1;    //结束点

       nihe_fenjie=(u8)((119-nihe_i)/3)+nihe_i; //远处和近处的分界线
       Cnt_nihe-=10;
       X_AVE=(float)2*X/Cnt_nihe;
       Y_AVE=(float)Y/Cnt_nihe;
       XY_AVE=(float)XY/Cnt_nihe;
       XX_AVE=(float)XX/Cnt_nihe;
       CK=128*(XY_AVE - X_AVE*Y_AVE/2)/(XX_AVE-( X_AVE)*( X_AVE )/4); //斜率
       CB=(float)(128*Y_AVE - CK*X_AVE/2);                            //截距
       Nihe_panduan=(CK*(119-Select_line)+CB)/128;                    //拟合线
       
       
     /***********近处拟合***********/
     X1=0;
     Y1=0;
     XY1=0;
     XX1=0;
     Cnt_nihe=10;
       for(i=119;i>=nihe_fenjie;i--)
          {
          if(mid[i]-mid[i-1]>0)    //过滤噪点
            chazhi0=mid[i]-mid[i-1];
          else
            chazhi0=mid[i-1]-mid[i];
          if(mid[i]-mid[i+1]>0)
            chazhi1=mid[i]-mid[i+1];
          else
            chazhi1=mid[i+1]-mid[i];  
          if(chazhi0>=5&&chazhi1>=5)
             continue;
          
          X1+=(119-i);
          Y1+=mid[i];
          XY1+=mid[i]*(119-i);
          XX1+=(119-i)*(119-i);
          Cnt_nihe++;
          }
       Cnt_nihe-=10;
       X_AVE1=(float)2*X1/Cnt_nihe;
       Y_AVE1=(float)Y1/Cnt_nihe;
       XY_AVE1=(float)XY1/Cnt_nihe;
       XX_AVE1=(float)XX1/Cnt_nihe;
       CK1=128*(XY_AVE1 - X_AVE1*Y_AVE1/2)/(XX_AVE1-( X_AVE1)*( X_AVE1)/4); //斜率
       CB1=(float)(128*Y_AVE1 - CK1*X_AVE1/2);                            //截距
     /***********远处拟合***********/
     X2=0;
     Y2=0;
     XY2=0;
     XX2=0;
     Cnt_nihe=10;
         for(i=nihe_fenjie;i>=nihe_i;i--)
          {
          if(mid[i]-mid[i-1]>0)    //过滤噪点
            chazhi0=mid[i]-mid[i-1];
          else
            chazhi0=mid[i-1]-mid[i];
          if(mid[i]-mid[i+1]>0)
            chazhi1=mid[i]-mid[i+1];
          else
            chazhi1=mid[i+1]-mid[i];  
          if(chazhi0>=5&&chazhi1>=5)
             continue;
          
          X2+=(119-i);
          Y2+=mid[i];
          XY2+=mid[i]*(119-i);
          XX2+=(119-i)*(119-i);
          Cnt_nihe++;
          }
       Cnt_nihe-=10;
       X_AVE2=(float)2*X2/Cnt_nihe;
       Y_AVE2=(float)Y2/Cnt_nihe;
       XY_AVE2=(float)XY2/Cnt_nihe;
       XX_AVE2=(float)XX2/Cnt_nihe;
       CK2=128*(XY_AVE2 - X_AVE2*Y_AVE2/2)/(XX_AVE2-( X_AVE2)*( X_AVE2)/4); //斜率
       CB2=(float)(128*Y_AVE2 - CK2*X_AVE2/2);                            //截距
       
       
      /*******************拟合结束**************************/
       D=0.0;
       for(i=119;i>=nihe_i;i--)
            D+=(mid[i]-(CB+CK*(119-i))/128)*(mid[i]-(CB+CK*(119-i))/128);
  
       D_AVE=(float)(D /(119-i));                 //全场残差，仅作参考
       
      
       
  
 /*******************************最小二乘法拟合end******************************************/ 


       
 /************************************判断路况************************************************/ 
  //根据拟合情况判断路况调节PD参数及设定速度
  Con_road=wandao_OTHERS;
  if((Nihe_panduan>=118.0||Nihe_panduan<=30.0)&&!(D_AVE<=11.0))                //其他道路的自适应调节
      Kp=13.5;
  
  
  if((Nihe_panduan>30.0&&Nihe_panduan<=44.0)&&!(D_AVE<=11.0))
    //Kp=11.0;
     Kp=12.0+(44.0-Nihe_panduan)*0.15;
  if((Nihe_panduan>44.0&&Nihe_panduan<=54.0)&&!(D_AVE<=11.0))
    //Kp=10.0;
     Kp=10.5+(54.0-Nihe_panduan)*0.15;
  if((Nihe_panduan>54.0&&Nihe_panduan<=64.0)&&!(D_AVE<=11.0))
   // Kp=9.5;
   Kp=9.0+(64.0-Nihe_panduan)*0.15;
  if((Nihe_panduan>64.0&&Nihe_panduan<84.0)&&!(D_AVE<=11.0))
   Kp=9.0;
  if((Nihe_panduan>=84.0&&Nihe_panduan<94.0)&&!(D_AVE<=11.0))    
     // Kp=9.5;
     Kp=9.0+(Nihe_panduan-84.0)*0.15; 
  if((Nihe_panduan>=94.0&&Nihe_panduan<104.0)&&!(D_AVE<=11.0))
     //  Kp=10.0;
     Kp=10.5+(Nihe_panduan-94.0)*0.15;
  if((Nihe_panduan>=104.0&&Nihe_panduan<118.0)&&!(D_AVE<=11.0)) 
     // Kp=11.0;
     Kp=12.0+(Nihe_panduan-104.0)*0.15;
  
  
  if((nihe_i<41)&&(end_rowC>30)&& ((Nihe_panduan>78.5)||(Nihe_panduan<69.5)) && (D_AVE<=118.0&&D_AVE>11.0)&&(nihe_i<=47)) //180度大弯,可调大速度。
     {Kp=6.0; Con_road=wandao_BAG;}
  if((Nihe_panduan>50.0&&Nihe_panduan<96.0)&&end_rowC<=30&&D_AVE>=40.56)   //除了残差外满足直道的其他条件，则为小S
     {Kp=7.5;  Con_road=wandao_xiaoS;}
  if((CK1<=12.0&&CK1>=-12.0)&&(!(D_AVE<=11.0))&&(CK2>=98.0||CK2<=-98.0))   //非长直道，近处为直道，远处斜率很大，则为直入弯                                                                  //(CK1<=12.0&&CK1>=-12.0)则前方35cm为直道
     {Kp=10.0; Con_road=zhidao_wandao;}
  if(((Nihe_panduan>50.0&&Nihe_panduan<96.0)&&end_rowC<=30)&&(D_AVE<=11.0))     //直道的绝对条件
      {Kp=4.5; Con_road=On_straight;}

    
/**************************************end**********************************************/ 
  for(i=40;i<=80;i++)               //判定十字弯
       if(loseline[i]==1)
         Cnt_shizhi++;
    
      
    if(Cnt_shizhi>=FAZHI_shizhi&&nihe_i<=60)                   
          flag_shizhi=1;
       else
          flag_shizhi=0;

       
       

/**************************************设定速度*****************************************/
  Set_speed0=Set_speed;     //记录上一次的设定值，可能用不到
  switch(Con_road)
    {
    case wandao_OTHERS: Set_speed=(int)(0.65*Speedmax);break;
    case On_straight  : Set_speed= Speedmax;           break;
    case wandao_BAG   : Set_speed=(int)(0.65*Speedmax); break;
    case wandao_xiaoS : Set_speed=(int)(0.60*Speedmax);break; 
    case zhidao_wandao: Set_speed=(int)(0.25*Speedmax);break;     
    
    }
  //if(flag_zhipao0)
    //Set_speed=(int)(0.70*Speedmax);
  if(flag_zhipao0&&flag_zhipao1)
    Set_speed=(int)(0.80*Speedmax);
  if(flag_zhipao0&&flag_zhipao1&&flag_zhipao2)
    Set_speed=(int)(0.92*Speedmax);
  if(flag_zhipao0&&flag_zhipao1&&flag_zhipao2&&flag_zhipao3)
    Set_speed=(int)(Speedmax);
  
/**************************************设定速度end*****************************************/

/**************************************起跑线及减速********************************************/
  cnt_stop=0;
  if(end_rowC>=96||end_rowR>=96||end_rowL>=96)
       Set_speed=(int)(0.45*Speedmax);
  if(end_rowC>=100&&end_rowR>=100&&end_rowL>=100)
       Set_speed=Speedmin+150;

   for(i=119;i>110;i--)
       if(mid[i]-row_left[i]<=25&&row_right[i]-mid[i]<=25)
	 cnt_stop++;
   if(cnt_stop>=1)
        break;
   
    
/****************************************end*********************************************/ 
/**************************************速度PID********************************************/
    Sur_speed=cesu;                          //实际速度  
    Sur_speed=(int)(26*Sur_speed+220) ;      //单位转换，b可能为零
    PREerror=LASTerror;                      //计算速度偏差
    LASTerror=NOWerror;
    NOWerror=Set_speed-Sur_speed;         
    
    if(Set_speed-Sur_speed>=Speed_FAZHI)
	 Speed_out=Speedmax0-250;
    else
         if(Sur_speed-Set_speed>=Speed_FAZHI)
	      Speed_out=Speedmin;
         else
	     {
	      V_add=(int)(KP*(NOWerror-LASTerror)+KI*NOWerror+KD*(NOWerror+PREerror-2*LASTerror));

                          Speed_out=Speed_out+V_add;
	      }
    
    if(Speed_out<Speedmin)   Speed_out=Speedmin; //电机限幅
    if(Speed_out>Speedmax0)   Speed_out=Speedmax0;
    PWM_out=(u32)Speed_out;
/**************************************速度PIDend****************************************/  
 
    
    
/**************************************转角PID************************************************/ 
       if(flag_shizhi||flag_zhipao3)
          angle=steer_center;    //或用angle,待调整.
       else
            if( (Middle_point-errorAVE)>FAZHI_zhidao||(errorAVE-Middle_point>FAZHI_zhidao) )               //舵机控制
                  //angle= (u32 )( angle  +Kp*(errorNOW-errorLAST)+Kd*(errorNOW-2*errorLAST+ errorPRE) );  //增量式
                  angle=(u32 )(steer_center+Kp*errorNOW+Kd*(errorNOW - errorLAST));                        //位置式
            else
                  angle=steer_center;
       if(angle>1550)  angle=1550;           //舵机限幅
       if(angle<1000)  angle=1000;  
/**************************************转角PID end**********************************************/                       

    FTM_PWM_Duty(FTM1,CH0,angle);
    
    if(flag_yunsu)
      FTM_PWM_Duty(FTM0,CH4,2300);
    else
      FTM_PWM_Duty(FTM0,CH4,PWM_out);
   
    

  } 


/******************************************************/  

FTM_PWM_init(FTM1,CH0,100,1265);         //配置舵机


gpio_init(PORTD,4,GPO,0);             //反配置电机
FTM_PWM_init(FTM0,CH5,10000,5000); 
time_delay_ms(800);

gpio_init(PORTD,4,GPO,0);                //置0电机
FTM_PWM_init(FTM0,CH5,10000,0); 
/*****************************************************/
}


/****************************************END************************************************/
