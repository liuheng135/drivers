#include "hal_device.h"
#include "hal_data_define.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include "string.h"
#include "font.h"

#define OLED_DC_PORT   GPIOB
#define OLED_DC_PIN    LL_GPIO_PIN_12
#define OLED_SCK_PORT  GPIOB
#define OLED_SCK_PIN   LL_GPIO_PIN_13
#define OLED_SDI_PORT  GPIOB
#define OLED_SDI_PIN   LL_GPIO_PIN_15
#define OLED_RST_PORT  GPIOB
#define OLED_RST_PIN   LL_GPIO_PIN_14

#define OLED_DC_SET    LL_GPIO_SetOutputPin(OLED_DC_PORT,OLED_DC_PIN)
#define OLED_DC_CLR    LL_GPIO_ResetOutputPin(OLED_DC_PORT,OLED_DC_PIN)

#define OLED_SCK_SET   LL_GPIO_SetOutputPin(OLED_SCK_PORT,OLED_SCK_PIN)
#define OLED_SCK_CLR   LL_GPIO_ResetOutputPin(OLED_SCK_PORT,OLED_SCK_PIN)

#define OLED_SDI_SET   LL_GPIO_SetOutputPin(OLED_SDI_PORT,OLED_SDI_PIN)
#define OLED_SDI_CLR   LL_GPIO_ResetOutputPin(OLED_SDI_PORT,OLED_SDI_PIN)

#define OLED_RST_SET   LL_GPIO_SetOutputPin(OLED_RST_PORT,OLED_RST_PIN)
#define OLED_RST_CLR   LL_GPIO_ResetOutputPin(OLED_RST_PORT,OLED_RST_PIN)



#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

#define OLED_FONT_SIZE 16
#define OLED_XLevelL		0x02
#define OLED_XLevelH		0x10
#define	OLED_Brightness	0xFF 
#define OLED_WIDTH 	        128
#define OLED_HEIGHT 	    64	  

void oled_delay_ms(unsigned int ms)
{                         
	volatile unsigned int a;
	while(ms)
	{
		a=1800;
		while(a--);
		ms--;
	}
	return;
}


//向SSD1306写入一个字节。
//dat:要写入的数据/命令
//cmd:数据/命令标志 0,表示命令;1,表示数据;
void oled_write_byte(uint8_t dat,uint8_t type)
{	
	uint8_t i;			  
	if(type){
	  OLED_DC_SET;
    }else{ 
	  OLED_DC_CLR;
    }        
    
	for(i=0;i<8;i++){			  
		OLED_SCK_CLR;
		if(dat&0x80){
		   OLED_SDI_SET;
		}else{
		   OLED_SDI_CLR;
		}
        OLED_SCK_SET;
		dat<<=1;   
	}				 		  
	OLED_DC_SET;   	  
} 

void oled_set_pos(unsigned char x, unsigned char y) 
{ 
	oled_write_byte(0xb0+y,OLED_CMD);
	oled_write_byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	oled_write_byte((x&0x0f)|0x01,OLED_CMD); 
}   	  
//开启OLED显示    
void oled_display_on(void)
{
	oled_write_byte(0X8D,OLED_CMD);  //SET DCDC命令
	oled_write_byte(0X14,OLED_CMD);  //DCDC ON
	oled_write_byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//关闭OLED显示     
void oled_display_off(void)
{
	oled_write_byte(0X8D,OLED_CMD);  //SET DCDC命令
	oled_write_byte(0X10,OLED_CMD);  //DCDC OFF
	oled_write_byte(0XAE,OLED_CMD);  //DISPLAY OFF
}	

//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
void oled_clear(void)  
{  
	uint8_t i,n;		    
	for(i=0;i<8;i++)  
	{  
		oled_write_byte (0xb0+i,OLED_CMD);    //设置页地址（0~7）
		oled_write_byte (0x00,OLED_CMD);      //设置显示位置—列低地址
		oled_write_byte (0x10,OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<128;n++){
            oled_write_byte(0,OLED_DATA);
        }            
	} //更新显示
}

void oled_show_char(uint8_t x,uint8_t y,uint8_t chr)
{      	
    unsigned char c=0,i=0;	
    c=chr-' ';//得到偏移后的值			
    if(x>OLED_WIDTH-1){x=0;y=y+2;}
    if(OLED_FONT_SIZE ==16){
        oled_set_pos(x,y);	
        for(i=0;i<8;i++){
            oled_write_byte(F8X16[c*16+i],OLED_DATA);
        }
        oled_set_pos(x,y+1);
        for(i=0;i<8;i++){
            oled_write_byte(F8X16[c*16+i+8],OLED_DATA);
        }    
    }else{	
        oled_set_pos(x,y+1);
        for(i=0;i<6;i++){
            oled_write_byte(F6x8[c][i],OLED_DATA);
        }
    }
}


void oled_gpio_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    
    GPIO_InitStruct.Pin = LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

int oled_init(struct hal_dev_s *dev)
{
    oled_gpio_init();
    OLED_RST_SET;
    oled_delay_ms(100);
    OLED_RST_CLR;
    oled_delay_ms(100);
    OLED_RST_SET;
    
    oled_write_byte(0xAE,OLED_CMD);//--turn off oled panel
	oled_write_byte(0x00,OLED_CMD);//---set low column address
	oled_write_byte(0x10,OLED_CMD);//---set high column address
	oled_write_byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	oled_write_byte(0x81,OLED_CMD);//--set contrast control register
	oled_write_byte(0xCF,OLED_CMD); // Set SEG Output Current Brightness
	oled_write_byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	oled_write_byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
	oled_write_byte(0xA6,OLED_CMD);//--set normal display
	oled_write_byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	oled_write_byte(0x3f,OLED_CMD);//--1/64 duty
	oled_write_byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	oled_write_byte(0x00,OLED_CMD);//-not offset
	oled_write_byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
	oled_write_byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
	oled_write_byte(0xD9,OLED_CMD);//--set pre-charge period
	oled_write_byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	oled_write_byte(0xDA,OLED_CMD);//--set com pins hardware configuration
	oled_write_byte(0x12,OLED_CMD);
	oled_write_byte(0xDB,OLED_CMD);//--set vcomh
	oled_write_byte(0x40,OLED_CMD);//Set VCOM Deselect Level
	oled_write_byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
	oled_write_byte(0x02,OLED_CMD);//
	oled_write_byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
	oled_write_byte(0x14,OLED_CMD);//--set(0x10) disable
	oled_write_byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
	oled_write_byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
	oled_write_byte(0xAF,OLED_CMD);//--turn on oled panel
	
	oled_write_byte(0xAF,OLED_CMD); /*display ON*/ 
	oled_clear();
	oled_set_pos(0,0); 	
    
    return 0;
}

int oled_open(struct hal_dev_s *dev, uint16_t oflag)
{
    return 0;
}

int oled_write(struct hal_dev_s *dev, const void *buffer, int size,int pos)
{
    
}

int oled_ioctl(struct hal_dev_s *dev, uint8_t cmd, void *args)
{
    int i;
    struct lcd_string_s *str;
    int pos_x,pos_y;
    int str_len;
    if(cmd == IO_CMD_USR1){
        str = (struct lcd_string_s *)args;
        str_len = strlen(str->str);
        pos_x = str->x;
        pos_y = str->y;
        for(i = 0;i < str_len;i++){
            oled_show_char(pos_x,pos_y,str->str[i]);
            pos_x+=8;
            if(pos_x > (OLED_WIDTH - 8)){
                break;
            }
        }
    }
    return 0;
}

struct hal_dev_s oled_dev;

int oled_register(void)
{
	oled_dev.init  = oled_init;
	oled_dev.open  = oled_open;
	oled_dev.write = oled_write;
    oled_dev.ioctl = oled_ioctl;

	hal_dev_register(&oled_dev,"oled",HAL_O_RDWR | HAL_DEV_STANDALONE);
	return 0;
}
