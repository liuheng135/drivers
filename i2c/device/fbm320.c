#include "hal_i2c.h"
#include <stdbool.h>

typedef struct
{
	uint16_t C0, C1, C2, C3, C6, C8, C9, C10, C11, C12; 
	uint32_t C4, C5, C7;
	uint8_t ver;
} fbm320_calib_data;

static void fbm320_delay_us(uint32_t ms)
{
    volatile int i,j;
    for(i = ms;i > 0;i--){
        for(j = 20;j > 0;j--);
    }
}

struct fbm320_report_s{
	uint8_t mode;
	int32_t pressure;
	int32_t temp;
	fbm320_calib_data calib;
};

static fbm320_calib_data fbm320_calib; 

int fbm320_read8(struct hal_dev_s *dev,uint8_t reg,uint8_t * data)
{
  	return hal_i2c_read_from_addr(dev,reg,data,1);
}

int fbm320_read32_data(struct hal_dev_s *dev,uint32_t * data)
{
	uint8_t value[3];
	int ret;

	ret = hal_i2c_read_from_addr(dev,0xF6,value,3);
	*data = ((uint32_t)value[0] << 16) | ((uint32_t)value[1] << 8) | (uint32_t)value[2];
	return ret;
}

int fbm320_write8(struct hal_dev_s *dev,uint8_t reg,uint8_t val)
{
	int ret;

	ret = hal_i2c_write_to_addr(dev,reg,&val,1);

	return ret;
}

static bool fbm320_i2c_test(struct hal_dev_s *dev)
{
	uint8_t id;
	int ret;
	
	ret = fbm320_read8(dev,0x6B,&id);
	if(ret < 0){
		printf("==%s== get id failed\r\n",__func__);
		return false;
	}
	
	printf("==%s== %x\r\n",__func__,id);

	return true;
}


int fbm320_readCoefficients(struct hal_dev_s *dev)
{
	int ret;
	uint8_t i;
	uint8_t tmp[2];
	uint16_t R[10]={0};

	fbm320_write8(dev,0xE0,0xB6);	

	fbm320_delay_us(10000);

	fbm320_read8(dev,0xA5,&tmp[0]);
	fbm320_read8(dev,0xF4,&tmp[1]);

	fbm320_calib.ver = ((tmp[0] & 0x70) >> 2) | ((tmp[1] & 0xC0) >> 6);
	
	for(i=0; i<9; i++){
		ret = fbm320_read8(dev,0xAA+(i*2),&tmp[0]);
		if(ret < 0){
			return -1;
		}
		ret = fbm320_read8(dev,0xAB+(i*2),&tmp[1]);
		if(ret < 0){
			return -1;
		}
		R[i] = ((uint8_t)tmp[0] << 8 | tmp[1]);
	}

	ret = fbm320_read8(dev,0xA4,&tmp[0]);
	if(ret < 0){
		return -1;
	}
	ret = fbm320_read8(dev,0xF1,&tmp[0]);
	if(ret < 0){
		return -1;
	}
	R[9] = ((uint8_t)tmp[0] << 8) | tmp[1];
	
	
	/*	Use R0~R9 calculate C0~C12 of FBM320-02	*/
	fbm320_calib.C0 = R[0] >> 4;
	fbm320_calib.C1 = ((R[1] & 0xFF00) >> 5) | (R[2] & 7);
	fbm320_calib.C2 = ((R[1] & 0xFF) << 1) | (R[4] & 1);
	fbm320_calib.C3 = R[2] >> 3;
	fbm320_calib.C4 = ((uint32_t)R[3] << 2) | (R[0] & 3);
	fbm320_calib.C5 = R[4] >> 1;
	fbm320_calib.C6 = R[5] >> 3;
	fbm320_calib.C7 = ((uint32_t)R[6] << 3) | (R[5] & 7);
	fbm320_calib.C8 = R[7] >> 3;
	fbm320_calib.C9 = R[8] >> 2;
	fbm320_calib.C10 = ((R[9] & 0xFF00) >> 6) | (R[8] & 3);
	fbm320_calib.C11 = R[9] & 0xFF;
	fbm320_calib.C12 = ((R[0] & 0x0C) << 1) | (R[7] & 7);

	printf("==%s== ver:0x%x\r\n",__func__,fbm320_calib.ver);

#if 0
	printf("C0:%x(%d) \r\n" , fbm320_calib.C0,fbm320_calib.C0);
	printf("C1:%x(%d) \r\n" , fbm320_calib.C1,fbm320_calib.C1);
	printf("C2:%x(%d) \r\n" , fbm320_calib.C2,fbm320_calib.C2);
	printf("C3:%x(%d) \r\n" , fbm320_calib.C3,fbm320_calib.C3);
	printf("C4:%x(%d) \r\n" , fbm320_calib.C4,fbm320_calib.C4);
	printf("C5:%x(%d) \r\n" , fbm320_calib.C5,fbm320_calib.C5);
	printf("C6:%x(%d) \r\n" , fbm320_calib.C6,fbm320_calib.C6);
	printf("C7:%x(%d) \r\n" , fbm320_calib.C7,fbm320_calib.C7);
	printf("C8:%x(%d) \r\n" , fbm320_calib.C8,fbm320_calib.C8);
	printf("C9:%x(%d) \r\n" , fbm320_calib.C9,fbm320_calib.C9);
	printf("C10:%x(%d) \r\n" , fbm320_calib.C10,fbm320_calib.C10);
	printf("C11:%x(%d) \r\n" , fbm320_calib.C11,fbm320_calib.C11);
	printf("C12:%x(%d) \r\n" , fbm320_calib.C12,fbm320_calib.C12);
#endif

	return 0;
}

static int fbm320_init(struct hal_dev_s *dev)
{
	int ret;
	ret = fbm320_readCoefficients(dev);
	if(ret < 0){
		return -1;
	}
	
	ret = fbm320_write8(dev,0xF4,0x2E);	
	if(ret < 0){
		return -1;
	}
	
	printf("==%s== done\r\n",__func__);

	return 0;
}

static int fbm320_measure(struct hal_dev_s *dev,struct fbm320_report_s *pf)
{
	int ret;
	
	pf->calib.C0 = fbm320_calib.C0;
	pf->calib.C1 = fbm320_calib.C1;
	pf->calib.C2 = fbm320_calib.C2;
	pf->calib.C3 = fbm320_calib.C3;
	pf->calib.C4 = fbm320_calib.C4;
	pf->calib.C5 = fbm320_calib.C5;
	pf->calib.C6 = fbm320_calib.C6;
	pf->calib.C7 = fbm320_calib.C7;
	pf->calib.C8 = fbm320_calib.C8;
	pf->calib.C9 = fbm320_calib.C9;
	pf->calib.C10 = fbm320_calib.C10;
	pf->calib.C11 = fbm320_calib.C11;
	pf->calib.C12 = fbm320_calib.C12;

	if(pf->mode){
		ret = fbm320_read32_data(dev,(uint32_t *)&pf->pressure);
		if(ret < 0){
			printf("fbm320_read32_data failed\r\n");
			return -1;
		}
		
		ret = fbm320_write8(dev,0xF4,0x2E);	
		if(ret < 0){
			printf("fbm320_write8 failed\r\n");
			return -1;
		}

	}else{
		ret = fbm320_read32_data(dev,(uint32_t *)&pf->temp);
		if(ret < 0){
			printf("fbm320_read32_data failed\r\n");
			return -1;
		}

		ret = fbm320_write8(dev,0xF4,0xF4);	
		if(ret < 0){
			printf("fbm320_write8 failed\r\n");
			return -1;
		}

	}

	return 0;
}

int fbm320_open(struct hal_dev_s *dev,uint16_t oflag)
{
	if(fbm320_i2c_test(dev) == false){
		return -1;
	}
		
	return 0; 
}

static int fbm320_read(struct hal_dev_s *dev, void *buffer, int size,int pos)
{
	int ret;
	struct fbm320_report_s *data = (struct fbm320_report_s *)buffer;

	ret = fbm320_measure(dev,data);
	if(ret < 0){
		return -1;
	}
	
	return 1;
}

struct hal_i2c_dev_s fbm320_0;

int fbm320_register(void)
{
	fbm320_0.port	   = 0;
	fbm320_0.address   = 0x6C;
	fbm320_0.cfg.flags = HAL_I2C_WR | HAL_I2C_RD;
	fbm320_0.cfg.speed = 1200000;
	fbm320_0.cfg.width = 8;

	fbm320_0.dev.init  = fbm320_init;
	fbm320_0.dev.open  = fbm320_open;
	fbm320_0.dev.close = NULL;
	fbm320_0.dev.read  = fbm320_read;
	fbm320_0.dev.write = NULL;
	fbm320_0.dev.ioctl = NULL;

	hal_i2c_device_register(&fbm320_0,"fbm320_0",HAL_O_RDWR | HAL_DEV_STANDALONE);
	return 0;
}

