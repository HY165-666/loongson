#include "ls1x.h"
#include "Config.h"
#include "ls1x_gpio.h"
#include "ls1x_latimer.h"
#include "ls1c102_ptimer.h"
#include "ls1x_common.h"
#include "ls1x_gpio.h"
#include "ls1x_exti.h"
#include "ls1x_latimer.h"
#include "ls1c102_touch.h"
#include "ls1x_string.h"
#include "ls1x_uart.h"
#include "ls1x_uart.h"
#include "ls1x_clock.h"
#include "UserGpio.h"
#include "Config.h"
#include "oled.h"
#include "queue.h"
#include "ls1c102_adc.h"
#include "ls1c102_interrupt.h"

char str[50];
uint8_t received_data = 0;
uint8_t Read_Buffer[DATA_LEN]; // 设置接收缓冲数组
uint8_t Read_length;


int Sec = 0;
int Min = 0;
int Hour = 18;


int duty1 = 0, status1 = 2;           //0静息，1正常，2故障
int duty2 = 0, status2 = 1;
int duty3 = 0, status3 = 1;
uint16_t temperature = 29; 
uint16_t humidity = 45; 
uint16_t light = 0; 
uint8_t react = 0;

uint8_t temp_str[5];
uint8_t humi_str[5];
uint8_t Light_str[5]; 
uint8_t duty1_str[5];
uint8_t duty2_str[5];
uint8_t duty3_str[5]; 
uint8_t status1_str[5];
uint8_t status2_str[5];
uint8_t status3_str[5]; 


int m_day1=6, m_day2=18;          //白天策略时间段
int m_dusk1=18, m_dusk2=20;       //傍晚
int m_eve1=20, m_eve2=5;         //晚上
int m_night1=22, m_night2=5;      //深夜
int m_sunrise1=5, m_sunrise2=6;   //凌晨

int light1 = 0;
uint8_t byte;
uint8_t byte_str[5]; 
int flag1 = 0;
int flag2 = 0;
int flag3 = 0;

uint8_t react1 = 1;
uint8_t react2 = 1;
uint8_t react3 = 1;


uint8_t light1_str[5];

int main(int arg, char *args[])
{
    SystemClockInit(); // 时钟等系统配置
    GPIOInit();        // io配置
    IIC_Init();
    OLED_Init();
    EnableInt(); // 开总中断
    
    timer_init(1); // 初始化定时器
    Queue_Init(&Circular_queue);

    Uart0_init(115200); // 串口0初始化，io06 io07   串口初始化需要在开启EnableInt之后
    Uart1_init(115200);

    //DHT11_Init();
    AFIO_RemapConfig(AFIOA, GPIO_Pin_16, 0);        
    AFIO_RemapConfig(AFIOA, GPIO_Pin_17, 0);
    Adc_powerOn();                                  // 打开ADC电源
    Adc_open(ADC_CHANNEL_I6);
    Adc_open(ADC_CHANNEL_I7);

    gpio_write_pin(GPIO_PIN_40, 1);
    gpio_set_direction(GPIO_PIN_40,GPIO_Mode_In);
    gpio_set_direction(GPIO_PIN_18,GPIO_Mode_Out);
     gpio_set_direction(GPIO_PIN_19,GPIO_Mode_Out);
    gpio_write_pin(GPIO_PIN_37, 1);
    gpio_set_direction(GPIO_PIN_37,GPIO_Mode_In);
    
    while (1)
    {
        //DHT11_Read_Data(&temperature,&humidity);   //温湿度采集
        light = Adc_Measure(ADC_CHANNEL_I6);
        react1 = gpio_get_pin(GPIO_PIN_40);
        
        react3 = gpio_get_pin(GPIO_PIN_37);
        
        
        Clock(Hour, Min, Sec);

        OLED_ShowInt32Num(0,6,Hour,2,16);
        OLED_Show_Str(16, 6, ":", 16);
        OLED_ShowInt32Num(24,6,Min,2,16);
        OLED_Show_Str(40, 6, ":", 16);
        OLED_ShowInt32Num(48,6,Sec,2,16);

        LED_PWMC1(duty1);
        LED_PWMC2(duty2);
        LED_PWMC3(duty3);
        if(Hour >= m_day1 && Hour <= m_day2)            //白天熄灭
        {
            duty1 = 0;
            duty2 = 0;
            duty3 = 0;
            
            
        }
        if(Hour >= m_dusk1 && Hour <= m_dusk2)          //傍晚渐亮 0~30
        {
            status1 = 2;
            status2 = 1;
            status3 = 1;
            light1 = (light-2000)/66;
            duty1 = 0;
            LED_PWMC1(30);
            duty2 = (light-2000)/66;
            duty3 = (light-2000)/66;
            
        }
        if((Hour >= 20 && Hour <= 23) || (Hour >= 0 && Hour <= 5))            //晚上常亮
        {
            duty1 = 0;
            duty2 = 30;
            duty3 = 30;
            status1 = 1;
            status2 = 1;
            status3 = 1;
            
            if (react1 == 0 && flag1 == 0) {
                UART_SendData(UART0, 'A');
                flag1 = 1;
            }
            if (react1 == 1 && flag1 == 1) {
                flag1 = 0;
            }
            
            if (Queue_isEmpty(&Circular_queue) == 0) // 判断队列是否为空，即判断是否收到数据
            {
            Read_length = Queue_HadUse(&Circular_queue); // 返回队列中数据的长度
            Queue_Read(&Circular_queue, Read_Buffer, Read_length); // 读取队列缓冲区的值到接收缓冲区
            
            Queue_Read(&Circular_queue, &byte, 1);
            //OLED_Show_Str(96, 6, Read_Buffer, 16);
            sprintf(byte_str, "%d" , byte);
            OLED_Show_Str(64, 6, byte_str, 16);
            }
            else
            {
                memset(Read_Buffer, 0, DATA_LEN); // 填充接收缓冲区为0
            }
            

            if (Read_length > 0 && Read_Buffer[0] == 'B' ) {
                flag2 = 1;

            }
            if(flag2 == 1)
            {
                duty1 = 100;
                duty2 = 100;
                duty3 = 100;
            }
            if(react3 == 0 )
            {
                duty1 = 0;
                duty2 = 30;
                duty3 = 30;
                flag2 = 0;
            }
            LED_PWMC1(duty1);
            LED_PWMC2(duty2);
            LED_PWMC3(duty3);
            

        }
        sprintf(light1_str, "%3d", light);
        OLED_Show_Str(50, 2, light1_str,16);

        
        if(Hour >= m_sunrise1 && Hour <= m_sunrise2)    //凌晨渐灭
        {
            light1 = (light-2000)/66;
            duty1 = 0;
            duty2 = (light-2000)/66;
            duty3 = (light-2000)/66;
            status1 = 1;
            status2 = 1;
            status3 = 1;
            
        }
        OLED_Show_Str(0, 0, "light:", 16);
        OLED_Show_Str(52, 0, Light_str,16);
        OLED_ShowInt32Num(0,4,react1,1,16);
        OLED_ShowInt32Num(12,4,react2,1,16);
        OLED_ShowInt32Num(28,4,react3,1,16);

        sprintf(Light_str, "%4d", light);                
        sprintf(temp_str, "%2d" ,temperature);
        sprintf(humi_str, "%2d" , humidity);
        sprintf(duty1_str, "%d", duty1);                
        sprintf(duty2_str, "%d" ,duty2);
        sprintf(duty3_str, "%d" , duty3);
        sprintf(status1_str, "%d", status1);                
        sprintf(status2_str, "%d" ,status2);
        sprintf(status3_str, "%d" , status3);



        // if (Queue_isEmpty(&Circular_queue) == 0) // 判断队列是否为空，即判断是否收到数据
        // {
        //     Read_length = Queue_HadUse(&Circular_queue); // 返回队列中数据的长度
        //     Queue_Read(&Circular_queue, Read_Buffer, Read_length); // 读取队列缓冲区的值到接收缓冲区
        //     Queue_Read(&Circular_queue, flag, 1);
        //     Queue_Read(&Circular_queue, &byte, 1);
        //     // UART_SendDataALL(UART0, Read_Buffer, 1);
        //     sprintf(byte_str, "%d" , byte);
        //     OLED_Show_Str(64, 6, byte_str, 16);
        // }
        // else
        // {
        //     memset(Read_Buffer, 0, DATA_LEN); // 填充接收缓冲区为0
        // }
        

        // if (Read_length > 0 && Read_Buffer[0] == 'B') {
        //     UART_SendData(UART0, 'A');
        //     gpio_write_pin(GPIO_PIN_34, 1);
        // }
        
        


        UART_SendData(UART0, '#');


        UART_SendDataALL(UART0, duty1_str , strlen(duty1_str));
        UART_SendDataALL(UART0, "," , 1);


        UART_SendDataALL(UART0, status1_str , strlen(status1_str));
        UART_SendDataALL(UART0, "," , 1);


        UART_SendDataALL(UART0, duty2_str , strlen(duty2_str));
        UART_SendDataALL(UART0, "," , 1);


        UART_SendDataALL(UART0, status2_str , strlen(status2_str));
        UART_SendDataALL(UART0, "," , 1);


        UART_SendDataALL(UART0, duty3_str , strlen(duty3_str));
        UART_SendDataALL(UART0, "," , 1);


        UART_SendDataALL(UART0, status3_str , strlen(status3_str));
        UART_SendDataALL(UART0, "," , 1);


        UART_SendDataALL(UART0, Light_str , strlen(Light_str));
        UART_SendDataALL(UART0, "," , 1);


        UART_SendDataALL(UART0, temp_str , strlen(temp_str));
        UART_SendDataALL(UART0, "," , 1);
          
        
        UART_SendDataALL(UART0, humi_str , strlen(humi_str));

        
        UART_SendData(UART0, '$');


        UART_SendData(UART0, '\n');
        //delay_ms(1000);


        
    }
    return 0;
}
