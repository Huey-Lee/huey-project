//#include "decode.h"
//#include "common.h"

//_Bool old_bit;                   									//保存上一次 查询到的电平状态
//_Bool syn_ok;                   								  //同步码标志位 
//_Bool rf_ok=0;                     								//接收完整命令标志位，通知解码程序可以解码了
//_Bool rf_ok_first,rf_ok_second;  									//接收成功临时标志

//u8 key_value= 0;                     							//按键编码值
//u16 high_width_cnt,low_width_cnt;   						  //高，低电平脉宽
//u8  rec_bit_cnt;                     							//接收到第几位编码了
//u8  temp_code1,temp_code2,temp_code3,temp_code4;  //用于接收过程存放遥控编码
//u8  first_code1,first_code2,first_code3;          //第一次接收到的编码
//u8  second_code1,second_code2,second_code3;       //第二次接收到的编码 ，用于解码过程
//u8  match_first_code1 = 0,match_first_code2 = 0; 	//遥控器坏了，其他遥控器对码第一次接收的编码
//u8  match_key_value = 0; 													//遥控器坏了，其他遥控器对码按键编码值

//u16 s_tim;                												//用于定时


////RF433M解码：解码出的值存放在key_value
//void RF433M_Init(void)
//{
//  rf_ok =0;  
//}


//void RF433M_RecevieDecode(void)  
//{          
//  if(!RF_REC)  //P00
//  { 
//    low_width_cnt++;
//    old_bit=0; 
//  }  
//  else        
//  {
//    if(!old_bit)   //检测到从低到高的跳变，已检测到一个完整的（高-低）电平周期
//    {
//      if(((high_width_cnt >= RF_SYN_H_MIN)&&(high_width_cnt <= RF_SYN_H_MAX ))&&((low_width_cnt>= RF_SYN_L_MIN)&&(low_width_cnt<= RF_SYN_L_MAX)))  //判同步码        
//      {                                                    
//        rec_bit_cnt=0;
//        syn_ok=1;
//        temp_code1=0;
//        temp_code2=0;
//        temp_code3=0; 
//        temp_code4=0; 
//      }
//      else if((syn_ok)&&((high_width_cnt >= RF_DATA0_MIN )&&(high_width_cnt <= RF_DATA0_MAX)))  //已经接收到同步码，根据高电平判断数据0
//      {        
//        rec_bit_cnt++;                     //已经接收到同步码，判断数据为bit0,
//        if(rec_bit_cnt >= RF_REC_BIT_LEN)  //接收完32位数据
//        {
//          if(!rf_ok_first)
//          { 
//						first_code1=temp_code1;
//						first_code2=temp_code2;						
//            first_code3=temp_code3;    //将接收到的编码赋值到存放接收的第1次数据的变量中                           
//       
//            rf_ok_first=1;
//            rec_bit_cnt=0;                           
//            syn_ok=0;
//            s_tim=1200;     //2500晚点调整大小         （1200*31+6300）*4=57300*4=229200                                      
//          }
//          else
//          {
//            second_code1=temp_code1;
//            second_code2=temp_code2;
//            second_code3=temp_code3;   //将接收到的编码赋值到存放接收第2次数据的变量中                       

//            rf_ok_second=1;
//            rec_bit_cnt=0;                              
//            syn_ok=0;                                                                                                                                                     
//          }
//        }
//      }                  
//      else if((syn_ok)&&((high_width_cnt >= RF_DATA1_MIN)&&(high_width_cnt <= RF_DATA1_MAX))) //数据bit1    
//      {
//        rec_bit_cnt++;

//        switch (rec_bit_cnt)
//        {
//          case 1 : { temp_code1=temp_code1 | 0x80; break; }//遥控地址编码第1位
//          case 2 : { temp_code1=temp_code1 | 0x40; break; }
//          case 3 : { temp_code1=temp_code1 | 0x20; break; }
//          case 4 : { temp_code1=temp_code1 | 0x10; break; }
//          case 5 : { temp_code1=temp_code1 | 0x08; break; }
//          case 6 : { temp_code1=temp_code1 | 0x04; break; }
//          case 7 : { temp_code1=temp_code1 | 0x02; break; }
//          case 8 : { temp_code1=temp_code1 | 0x01; break; }
//          case 9 : { temp_code2=temp_code2 | 0x80; break; }
//          case 10: { temp_code2=temp_code2 | 0x40; break; }
//          case 11: { temp_code2=temp_code2 | 0x20; break; }
//          case 12: { temp_code2=temp_code2 | 0x10; break; }
//          case 13: { temp_code2=temp_code2 | 0x08; break; }
//          case 14: { temp_code2=temp_code2 | 0x04; break; }
//          case 15: { temp_code2=temp_code2 | 0x02; break; }
//          case 16: { temp_code2=temp_code2 | 0x01; break; }
//          case 17: { temp_code3=temp_code3 | 0x80; break; }//按键码第1位
//          case 18: { temp_code3=temp_code3 | 0x40; break; }
//          case 19: { temp_code3=temp_code3 | 0x20; break; }
//          case 20: { temp_code3=temp_code3 | 0x10; break; }
//          case 21: { temp_code3=temp_code3 | 0x08; break; }
//          case 22: { temp_code3=temp_code3 | 0x04; break; }
//          case 23: { temp_code3=temp_code3 | 0x02; break; }
//          case 24: { temp_code3=temp_code3 | 0x01; break; }
//          case 25: { temp_code4=temp_code4 | 0x80; break; }//校验码第1位
//          case 26: { temp_code4=temp_code4 | 0x40; break; }
//          case 27: { temp_code4=temp_code4 | 0x20; break; }
//          case 28: { temp_code4=temp_code4 | 0x10; break; }
//          case 29: { temp_code4=temp_code4 | 0x08; break; }
//          case 30: { temp_code4=temp_code4 | 0x04; break; }
//          case 31: { temp_code4=temp_code4 | 0x02; break; }
//          case 32: { temp_code4=temp_code4 | 0x01; //break; }
//          // 校验码结束 
//                    if(!rf_ok_first)
//                    {
//											first_code1=temp_code1;
//											first_code2=temp_code2;
//                      first_code3=temp_code3;   //将接收到的编码赋值到存放接收的第1次数据的解码寄存器(变量中)                      

//                      rf_ok_first=1;                                
//                      syn_ok=0;
//                      rec_bit_cnt=0;
//                      s_tim=1200;             //2500; //晚点调整大小   2500*20=50ms                    
//                      break;
//                    }
//                    else
//                    {
//                      second_code1=temp_code1;
//                      second_code2=temp_code2;
//                      second_code3=temp_code3; //将接收到的编码赋值到存放接收的第2次数据的解码寄存器(变量中)                         

//                      rf_ok_second=1;                                
//                      syn_ok=0;              
//                      rec_bit_cnt=0;  
//                      break;                              
//                    }             
//					}                        
//          default : break;                    
//				}
//      }
//      else  //接收到的为无效电平，杂波   
//      {
//        rec_bit_cnt=0;
//        syn_ok=0; 
//        temp_code1=0; 
//        temp_code2=0; 
//        temp_code3=0; 
//        
//        high_width_cnt=0;  
//        low_width_cnt=0;
//        key_value = 0;   
//      }                  
//      low_width_cnt=0;
//      high_width_cnt=0;
//    }          
//    high_width_cnt++;  //记录高电平时间
//    old_bit=1;      
//  }

//  if(rf_ok_first)      //在规定的时间内接收到2次相同的编码数据才有效
//  {       
//    s_tim--;
//    if(!s_tim) rf_ok_first=0;

//    if(rf_ok_second)
//    {
//      if((first_code1==second_code1)&&(first_code2==second_code2)&&(first_code3==second_code3))//比较两次收到的数据是否一致
//      {
//        rf_ok=1;
//        rf_ok_first=0;
//        rf_ok_second=0;
//      }    
//      else if((match_first_code1==second_code1)&&(match_first_code2==second_code2)&&(first_code3==second_code3))//收到其他遥控器的数据
//      {
//        rf_ok=1;
//        rf_ok_first=0;
//        rf_ok_second=0;                                         
//      }
//      else 
//      {
//        rf_ok=0;
//        rf_ok_first=0;
//        rf_ok_second=0;                                          
//      }                              
//    }                                       
//  }    
//  if(rf_ok)                 
//  {         
//    rf_ok=0;
//		key_value = first_code3; 
//    temp_code3 = 0;
//    first_code3 = 0;   //按键码取出后清0，没有键码是0，所以可清0                                       
//  }
//}

#include "decode.h"
#include "common.h"

_Bool old_bit;                   //保存上一次 查询到的电平状态
_Bool syn_ok;                    //同步码标志位 
_Bool rf_ok=0;                     //接收完整命令标志位，通知解码程序可以解码了
_Bool rf_ok_first,rf_ok_second;  //接收成功临时标志

u8 key_value= 0,key_value_2=0;                     //按键编码值
u16 key_value_3=0;
u16 high_width_cnt,low_width_cnt;    //高，低电平脉宽
u8  rec_bit_cnt;                     //接收到第几位编码了
u8  temp_code1,temp_code2,temp_code3,temp_code4;  //用于接收过程存放遥控编码
u8  first_code1,first_code2,first_code3;          //第一次接收到的编码
u8  second_code1,second_code2,second_code3;       //第二次接收到的编码 ，用于解码过程

u16 s_tim;                //用于定时


//RF433M解码：解码出的值存放在key_value
void RF433M_Init(void)
{
//  P00_Input_Mode;  
  rf_ok =0;  
}


void RF433M_RecevieDecode(void)  
{          
  if(!RF_REC)  //P00
  { 
    low_width_cnt++;
    old_bit=0; 
  }  
  else        
  {
    if(!old_bit)   //检测到从低到高的跳变，已检测到一个完整的（高-低）电平周期
    {
      if(((high_width_cnt >= RF_SYN_H_MIN)&&(high_width_cnt <= RF_SYN_H_MAX ))&&((low_width_cnt>= RF_SYN_L_MIN)&&(low_width_cnt<= RF_SYN_L_MAX)))  //判同步码        
      {                                                    
        rec_bit_cnt=0;
        syn_ok=1;
        temp_code1=0;
        temp_code2=0;
        temp_code3=0; 
      }
      else if((syn_ok)&&((high_width_cnt >= RF_DATA0_MIN )&&(high_width_cnt <= RF_DATA0_MAX)))  //已经接收到同步码，根据高电平判断数据0
      {        
        rec_bit_cnt++;                     //已经接收到同步码，判断数据为bit0,
        if(rec_bit_cnt >= RF_REC_BIT_LEN)  //接收完24位数据
        {
          if(!rf_ok_first)
          {
            first_code1=temp_code1;
            first_code2=temp_code2;
            first_code3=temp_code3;    //将接收到的编码赋值到存放接收的第1次数据的变量中                           

            rf_ok_first=1;
            rec_bit_cnt=0;                           
            syn_ok=0;
            s_tim=900;     //2500晚点调整大小                                               
          }
          else
          {
            second_code1=temp_code1;
            second_code2=temp_code2;
            second_code3=temp_code3;   //将接收到的编码赋值到存放接收第2次数据的变量中                       

            rf_ok_second=1;
            rec_bit_cnt=0;                              
            syn_ok=0;                                                                                                                                                     
          }
        }
      }                  
      else if((syn_ok)&&((high_width_cnt >= RF_DATA1_MIN)&&(high_width_cnt <= RF_DATA1_MAX))) //数据bit1    
      {
        rec_bit_cnt++;

        switch (rec_bit_cnt)
        {
          case 1 : { temp_code1=temp_code1 | 0x80; break; }//遥控地址编码第1位
          case 2 : { temp_code1=temp_code1 | 0x40; break; }
          case 3 : { temp_code1=temp_code1 | 0x20; break; }
          case 4 : { temp_code1=temp_code1 | 0x10; break; }
          case 5 : { temp_code1=temp_code1 | 0x08; break; }
          case 6 : { temp_code1=temp_code1 | 0x04; break; }
          case 7 : { temp_code1=temp_code1 | 0x02; break; }
          case 8 : { temp_code1=temp_code1 | 0x01; break; }
          case 9 : { temp_code2=temp_code2 | 0x80; break; }
          case 10: { temp_code2=temp_code2 | 0x40; break; }
          case 11: { temp_code2=temp_code2 | 0x20; break; }
          case 12: { temp_code2=temp_code2 | 0x10; break; }
          case 13: { temp_code2=temp_code2 | 0x08; break; }
          case 14: { temp_code2=temp_code2 | 0x04; break; }
          case 15: { temp_code2=temp_code2 | 0x02; break; }
          case 16: { temp_code2=temp_code2 | 0x01; break; }
          case 17: { temp_code3=temp_code3 | 0x80; break; }//1527按键码第1位
          case 18: { temp_code3=temp_code3 | 0x40; break; }
          case 19: { temp_code3=temp_code3 | 0x20; break; }
          case 20: { temp_code3=temp_code3 | 0x10; break; }
          case 21: { temp_code3=temp_code3 | 0x08; break; }
          case 22: { temp_code3=temp_code3 | 0x04; break; }
          case 23: { temp_code3=temp_code3 | 0x02; break; }
          case 24: { temp_code3=temp_code3 | 0x01; 
           
                    if(!rf_ok_first)
                    {
                      first_code1=temp_code1;
                      first_code2=temp_code2;
                      first_code3=temp_code3;   //将接收到的编码赋值到存放接收的第1次数据的解码寄存器(变量中)                      

                      rf_ok_first=1;                                
                      syn_ok=0;
                      rec_bit_cnt=0;
                      s_tim=900;             //2500; //晚点调整大小   2500*20=50ms                    
                      break;
                    }
                    else
                    {
                      second_code1=temp_code1;
                      second_code2=temp_code2;
                      second_code3=temp_code3; //将接收到的编码赋值到存放接收的第2次数据的解码寄存器(变量中)                         

                      rf_ok_second=1;                                
                      syn_ok=0;              
                      rec_bit_cnt=0;  
                      break;                              
                    }             
                  }                        
          default : break;                    
        }
      }
      else  //接收到的为无效电平，杂波   
      {
        rec_bit_cnt=0;
        syn_ok=0; 
        temp_code1=0; 
        temp_code2=0; 
        temp_code3=0; 
        
        high_width_cnt=0;  
        low_width_cnt=0;
        key_value = 0;   //2023.2.27
      }                  
      low_width_cnt=0;
      high_width_cnt=0;
    }          
    high_width_cnt++;  //记录高电平时间
    old_bit=1;      
  }

  if(rf_ok_first)      //在规定的时间内接收到2次相同的编码数据才有效
  {       
    s_tim--;
    if(!s_tim) rf_ok_first=0;

    if(rf_ok_second)
    {
      if((first_code1==second_code1)&&(first_code2==second_code2)&&(first_code3==second_code3))//比较两次收到的数据是否一致
      {
        rf_ok=1;
        rf_ok_first=0;
        rf_ok_second=0;                                         
      }
      else 
      {
        rf_ok=0;
        rf_ok_first=0;
        rf_ok_second=0;                                          
      }                              
    }                                       
  }    
  if(rf_ok)                 
  {         
//    EA = 0; //关闭中断
    rf_ok=0;

    key_value = first_code3;
    
    temp_code3 = 0;
    first_code3 = 0;   //按键码取出后清0，没有键码是0，所以可清0
                                       
//    EA = 1; //打开中断
  }
}

