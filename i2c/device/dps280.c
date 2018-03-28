#include "hal_i2c.h"
#include <stdbool.h>
#include <stdint.h>
#include <hal_sensor.h>

/* Attributes: Product identification and version */

#define     VENDOR_NAME                                 "Infineon"
#define     DRIVER_NAME                                 "IFXDD"
#define     DEVICE_NAME                                 "Digital Barometric Pressure Sensor"
#define     DEVICE_MODEL_NAME                           "DPS280"
#define     DEVICE_HW_VERSION                           1.0
#define     DRIVER_VERSION                              1.0
#define     DEVICE_PROD_REV_ID                          0x10

/* Attributes: Device performance :Pressure Sensing */
#define     IFX_DPS280_PROD_REV_ID_REG_ADDR             0x0D
#define     IFX_DPS280_PROD_REV_ID_LEN                  1
#define     IFX_DSPS280_PROD_REV_ID_VAL                 DEVICE_PROD_REV_ID

#define     IFX_DPS280_SOFT_RESET_REG_ADDR              0x0C
#define     IFX_DPS280_SOFT_RESET_REG_DATA              0x09
#define     IFX_DPS280_SOFT_RESET_REG_LEN               1
#define     IFX_DPS280_SOFT_RESET_VERIFY_REG_ADDR       0x06

#define     IFX_DPS280_COEF_REG_ADDR                    0x10
#define     IFX_DPS280_COEF_LEN                         18    // Length in bytes

#define     IFX_DPS280_TMP_COEF_SRCE_REG_ADDR           0x28
#define     IFX_DPS280_TMP_COEF_SRCE_REG_LEN            1    // Length in bytes
#define     IFX_DPS280_TMP_COEF_SRCE_REG_POS_MASK       7    // Length in bytes

#define     IFX_DPS280_PSR_TMP_READ_REG_ADDR            0x00
#define     IFX_DPS280_PSR_TMP_READ_LEN                 6

#define     IFX_DPS280_PRS_CFG_REG_ADDR                 0x06
#define     IFX_DPS280_PRS_CFG_REG_LEN                  1

#define     IFX_DPS280_TMP_CFG_REG_ADDR                 0x07
#define     IFX_DPS280_TMP_CFG_REG_LEN                  1

#define     IFX_DPS280_MEAS_CFG_REG_ADDR                0x08
#define     IFX_DPS280_MEAS_CFG_REG_LEN                 1

#define     IFX_DPS280_CFG_REG_ADDR                     0x09
#define     IFX_DPS280_CFG_REG_LEN                      1

#define     IFX_DPS280_CFG_TMP_SHIFT_EN_SET_VAL         0x08
#define     IFX_DPS280_CFG_PRS_SHIFT_EN_SET_VAL         0x04


#define     IFX_DPS280_FIFO_READ_REG_ADDR               0x00
#define     IFX_DPS280_FIFO_REG_READ_LEN                3
#define     IFX_DPS280_FIFO_BYTES_PER_ENTRY             3

#define     IFX_DPS280_FIFO_FLUSH_REG_ADDR              0x0C
#define     IFX_DPS280_FIFO_FLUSH_REG_VAL               0b1000000U

#define     IFX_DPS280_CFG_SPI_MODE_POS                 0
#define     IFX_DPS280_CFG_SPI_MODE_3_WIRE_VAL          1
#define     IFX_DPS280_CFG_SPI_MODE_4_WIRE_VAL          0

#define     IFX_DPS280_CFG_FIFO_ENABLE_POS              1
#define     IFX_DPS280_CFG_FIFO_ENABLE_VAL              1
#define     IFX_DPS280_CFG_FIFO_DISABLE_VAL             0

#define     IFX_DPS280_CFG_INTR_PRS_ENABLE_POS          4
#define     IFX_DPS280_CFG_INTR_PRS_ENABLE_VAL          1U
#define     IFX_DPS280_CFG_INTR_PRS_DISABLE_VAL         0U

#define     IFX_DPS280_CFG_INTR_TEMP_ENABLE_POS         5
#define     IFX_DPS280_CFG_INTR_TEMP_ENABLE_VAL         1U
#define     IFX_DPS280_CFG_INTR_TEMP_DISABLE_VAL        0U

#define     IFX_DPS280_CFG_INTR_FIFO_FULL_ENABLE_POS    6
#define     IFX_DPS280_CFG_INTR_FIFO_FULL_ENABLE_VAL    1U
#define     IFX_DPS280_CFG_INTR_FIFO_FULL_DISABLE_VAL   0U

#define     IFX_DPS280_CFG_INTR_LEVEL_TYP_SEL_POS       7
#define     IFX_DPS280_CFG_INTR_LEVEL_TYP_ACTIVE_H      1U
#define     IFX_DPS280_CFG_INTR_LEVEL_TYP_ACTIVE_L      0U

#define     IFX_DPS280_INTR_SOURCE_PRESSURE             0
#define     IFX_DPS280_INTR_SOURCE_TEMPERATURE          1
#define     IFX_DPS280_INTR_SOURCE_BOTH                 2

#define     IFX_DPS280_INTR_STATUS_REG_ADDR             0x0A
#define     IFX_DPS280_INTR_STATUS_REG_LEN              1
#define     IFX_DPS280_INTR_DISABLE_ALL                (uint8_t)0b10001111

/* _______________________________________________________ */

#define POW_2_23_MINUS_1	0x7FFFFF   //implies 2^23-1
#define POW_2_24			0x1000000
#define POW_2_15_MINUS_1	0x7FFF
#define POW_2_16			0x10000
#define POW_2_11_MINUS_1	0x7FF
#define POW_2_12			0x1000
#define POW_2_20			0x100000
#define POW_2_19_MINUS_1	524287

/* _______________________________________________________ */

/* Struct to hold calibration coefficients read from device*/
typedef struct
{
  /* calibration registers */

  int16_t 	C0;	// 12bit
  int16_t 	C1;	// 12bit
  int32_t	C00;	// 20bit
  int32_t  C10;	// 20bit
  int16_t 	C01;	// 16bit
  int16_t	C11;	// 16bit
  int16_t	C20;	// 16bit
  int16_t	C21;	// 16bit
  int16_t	C30;	// 16bit

}dps280_cal_coeff_regs_s;

/* enum for seeting/getting device operating mode*/

typedef enum
{
  DPS280_MODE_IDLE                   =  0x00,
  DPS280_MODE_COMMAND_PRESSURE       =  0x01,
  DPS280_MODE_COMMAND_TEMPERATURE    =  0x02,
  DPS280_MODE_BACKGROUND_PRESSURE    =  0x04,
  DPS280_MODE_BACKGROUND_TEMPERATURE =  0x08,
  DPS280_MODE_BACKGROUND_ALL         =  0x10,

}dps280_operating_modes_e;



/* enum of scaling coefficients either Kp or Kt*/
typedef enum
{
    OSR_SF_1   = 524288,
    OSR_SF_2   = 1572864,
    OSR_SF_4   = 3670016,
    OSR_SF_8   = 7864320,
    OSR_SF_16  = 253952,
    OSR_SF_32  = 516096,
    OSR_SF_64  = 1040384,
    OSR_SF_128 = 2088960,

} dps280_scaling_coeffs_e;



/* enum of oversampling rates for pressure and temperature*/
typedef enum
{
    OSR_1   = 0x00,
    OSR_2   = 0x01,
    OSR_4   = 0x02,
    OSR_8   = 0x04,
    OSR_16  = 0x08,
    OSR_32  = 0x10,
    OSR_64  = 0x20,
    OSR_128 = 0x40,

} dps280_osr_e;



/* enum of measurement rates for pressure*/

typedef enum
{
    PM_MR_1   = 0x00,
    PM_MR_2   = 0x01,
    PM_MR_4   = 0x02,
    PM_MR_8   = 0x04,
    PM_MR_16  = 0x08,
    PM_MR_32  = 0x10,
    PM_MR_64  = 0x20,
    PM_MR_128 = 0x40,

} dps280_pm_rate_e;



/* enum of measurement rates for temperature*/

typedef enum
{
    TMP_MR_1   = 0x00,
    TMP_MR_2   = 0x01,
    TMP_MR_4   = 0x02,
    TMP_MR_8   = 0x04,
    TMP_MR_16  = 0x08,
    TMP_MR_32  = 0x10,
    TMP_MR_64  = 0x20,
    TMP_MR_128 = 0x40,

} dps280_tmp_rate_e;


/* enum of oversampling and measurement rates*/

typedef enum

{
    TMP_EXT_ASIC = 0x00,
    TMP_EXT_MEMS = 0x80,

}dps280_temperature_src_e;

/* Meaningful Defauhal Configuration */
#define     IFX_DPS280_TEMPERATURE_OSR                  OSR_16
#define     IFX_DPS280_PRESSURE_OSR                     OSR_16
#define     IFX_DPS280_TEMPERATURE_MR                   TMP_MR_16
#define     IFX_DPS280_PRESSURE_MR                      PM_MR_16


struct dps280_state {

        dps280_scaling_coeffs_e   tmp_osr_scale_coeff;                    /* Temperature scaling coefficient*/
        dps280_scaling_coeffs_e   prs_osr_scale_coeff;                    /* Pressure scaling coefficient*/
        dps280_cal_coeff_regs_s   calib_coeffs;                           /* Calibration coefficients index */
        dps280_operating_modes_e  dev_mode;                               /* Current operating mode of device */
        dps280_pm_rate_e	      press_mr;				  /* Current measurement readout rate (ODR) for pressure */
        dps280_tmp_rate_e         temp_mr;				  /* Current measurement readout rate (ODR) for temperature */
        dps280_osr_e		      temp_osr;				  /* Current oversampling rate (OSR) for temperature */
        dps280_osr_e		      press_osr;				  /* Current oversampling rate (OSR) for pressure */
        dps280_temperature_src_e  tmp_ext;                                /* Temperature ASIC or MEMS. Should always be set MEMS*/
        uint8_t                       cfg_word;                               /* Keep the contents of CFG register as it gets configured
                                                                            to avoid excessive bus transactions */
		bool 			          enable;
		struct hal_dev_s *dev;
};




struct dps280_report_s{
	uint8_t pressure[3];
	uint8_t temperature[3];
};


static void dps280_delay_us(uint32_t ms)
{
    volatile int i,j;
    for(i = ms;i > 0;i--){
        for(j = 20;j > 0;j--);
    }
}

static dps280_scaling_coeffs_e dps280_get_scaling_coef (dps280_osr_e osr)
{
        dps280_scaling_coeffs_e scaling_coeff;

        switch (osr){

              case OSR_1:
                    scaling_coeff = OSR_SF_1;
                    break;
              case OSR_2:
                    scaling_coeff = OSR_SF_2;
                    break;
              case OSR_4:
                    scaling_coeff = OSR_SF_4;
                    break;
              case OSR_8:
                    scaling_coeff = OSR_SF_8;
                    break;
              case OSR_16:
                    scaling_coeff = OSR_SF_16;
                    break;
              case OSR_32:
                    scaling_coeff = OSR_SF_32;
                    break;
              case OSR_64:
                    scaling_coeff = OSR_SF_64;
                    break;
              case OSR_128:
                    scaling_coeff = OSR_SF_128;
                    break;
              default:
                     scaling_coeff = OSR_SF_1;
                     break;
        }
        return scaling_coeff;
}


static int dps280_read_calib_coeffs(struct dps280_state *drv_state)

{
        int32_t ret;
        uint8_t read_buffer[IFX_DPS280_COEF_LEN] = {0};

        if (drv_state == NULL)
            return -1;
		if (drv_state == NULL)
            return -1;

		ret =  hal_i2c_read_from_addr(drv_state->dev, IFX_DPS280_COEF_REG_ADDR, read_buffer, IFX_DPS280_COEF_LEN);

        if ( ret <= 0)
            return ret;
		
        drv_state->calib_coeffs.C0 = (read_buffer[0] << 4) + ((read_buffer[1] >>4) & 0x0F);

        if(drv_state->calib_coeffs.C0 > POW_2_11_MINUS_1)
            drv_state->calib_coeffs.C0 = drv_state->calib_coeffs.C0 - POW_2_12;

        drv_state->calib_coeffs.C1 = (read_buffer[2] + ((read_buffer[1] & 0x0F)<<8));

        if(drv_state->calib_coeffs.C1 > POW_2_11_MINUS_1)
            drv_state->calib_coeffs.C1 = drv_state->calib_coeffs.C1 - POW_2_12;

        drv_state->calib_coeffs.C00 = ((read_buffer[4]<<4) + (read_buffer[3]<<12)) + ((read_buffer[5]>>4) & 0x0F);

        if(drv_state->calib_coeffs.C00 > POW_2_19_MINUS_1)
            drv_state->calib_coeffs.C00 = drv_state->calib_coeffs.C00 -POW_2_20;

        drv_state->calib_coeffs.C10 = ((read_buffer[5] & 0x0F)<<16) + read_buffer[7] + (read_buffer[6]<<8);

        if(drv_state->calib_coeffs.C10 > POW_2_19_MINUS_1)
            drv_state->calib_coeffs.C10 = drv_state->calib_coeffs.C10 - POW_2_20;

        drv_state->calib_coeffs.C01 = (read_buffer[9] + (read_buffer[8]<<8));

        if(drv_state->calib_coeffs.C01 > POW_2_15_MINUS_1)
            drv_state->calib_coeffs.C01 = drv_state->calib_coeffs.C01 - POW_2_16;

        drv_state->calib_coeffs.C11 = (read_buffer[11] + (read_buffer[10]<<8));

        if(drv_state->calib_coeffs.C11 > POW_2_15_MINUS_1)
            drv_state->calib_coeffs.C11 = drv_state->calib_coeffs.C11 - POW_2_16;

        drv_state->calib_coeffs.C20 = (read_buffer[13] + (read_buffer[12]<<8));

        if(drv_state->calib_coeffs.C20 > POW_2_15_MINUS_1)
            drv_state->calib_coeffs.C20 = drv_state->calib_coeffs.C20 - POW_2_16;

        drv_state->calib_coeffs.C21 = (read_buffer[15] + (read_buffer[14]<<8));

        if(drv_state->calib_coeffs.C21 > POW_2_15_MINUS_1)
            drv_state->calib_coeffs.C21 = drv_state->calib_coeffs.C21 - POW_2_16;

        drv_state->calib_coeffs.C30 = (read_buffer[17] + (read_buffer[16]<<8));

        if(drv_state->calib_coeffs.C30 > POW_2_15_MINUS_1)
            drv_state->calib_coeffs.C30 = drv_state->calib_coeffs.C30 - POW_2_16;

        /* lets see which temperature diode is used for calibration and update state accordingly*/
        if (hal_i2c_read_from_addr(drv_state->dev, IFX_DPS280_TMP_COEF_SRCE_REG_ADDR, read_buffer, 1) < 0){
            return -1;
        }
        if ((read_buffer[0] >> IFX_DPS280_TMP_COEF_SRCE_REG_POS_MASK) & 1 ){
			drv_state->tmp_ext = TMP_EXT_MEMS;
		}
		else{
			drv_state->tmp_ext = TMP_EXT_ASIC;
		}
		
        return 0;
}


int dps280_resume(struct dps280_state *drv_state)
{
        int32_t ret;
        if (drv_state == NULL)
            return -1;
		uint8_t reg_val = 0;

		reg_val = DPS280_MODE_BACKGROUND_ALL;
		ret = hal_i2c_write_to_addr(drv_state->dev,IFX_DPS280_MEAS_CFG_REG_ADDR, &reg_val,1); 
		if (ret < 0)
			return -1;

        drv_state->dev_mode = DPS280_MODE_BACKGROUND_ALL;

        return 0;

}


int dps280_standby(struct dps280_state *drv_state)
{
        int32_t ret;
		uint8_t reg_val = 0;

		if (drv_state == NULL)
          return -1;

		reg_val = DPS280_MODE_IDLE;
		ret = hal_i2c_write_to_addr(drv_state->dev,IFX_DPS280_MEAS_CFG_REG_ADDR, &reg_val,1); 
		if (ret < 0)
		    return -1;

        drv_state->dev_mode = DPS280_MODE_IDLE;

        return 0;
}


int dps280_config(struct dps280_state *drv_state,
                            dps280_osr_e osr_temp,
                            dps280_tmp_rate_e mr_temp,
                            dps280_osr_e osr_press,
                            dps280_pm_rate_e mr_press,
                            dps280_temperature_src_e temp_src)
{
	uint8_t config;

	if (drv_state == NULL){
		return -1;
	}

	/* configure temperature measurements first*/
	/*Prepare a configuration word for TMP_CFG register*/
	config = (uint8_t) temp_src;

	/*First Set the TMP_RATE[2:0] -> 6:4 */
	config |= ((uint8_t)mr_temp);

	/*Set the TMP_PRC[3:0] -> 2:0 */
	config |= ((uint8_t)osr_temp);

	if (hal_i2c_write_to_addr(drv_state->dev,IFX_DPS280_TMP_CFG_REG_ADDR, &config,1) < 0){
	    return -1;
	}
	/*Prepare a configuration word for PRS_CFG register*/
	/*First Set the PM_RATE[2:0] -> 6:4 */
	config = (uint8_t) ( 0x00 ) | ((uint8_t)mr_press);

	/*Set the PM_PRC[3:0] -> 3:0 */
	config |= ((uint8_t)osr_press);
	dps280_delay_us(2000);
	if (hal_i2c_write_to_addr(drv_state->dev,IFX_DPS280_PRS_CFG_REG_ADDR, &config,1) < 0){
	    return -1;
	}
	/* always take configuration word from state*/
	config = drv_state->cfg_word;

	/*If oversampling rate for temperature is greater than 8 times, then set TMP_SHIFT bit in CFG_REG */
	if ((uint8_t)osr_temp > (uint8_t) OSR_8){
		config |= (uint8_t) IFX_DPS280_CFG_TMP_SHIFT_EN_SET_VAL;
	}

	/*If oversampling rate for pressure is greater than 8 times, then set P_SHIFT bit in CFG_REG */
	if ((uint8_t)osr_press > (uint8_t) OSR_8){
		config |= (uint8_t) IFX_DPS280_CFG_PRS_SHIFT_EN_SET_VAL;
	}
	dps280_delay_us(2000);
	/* write CFG_REG */
	if (hal_i2c_write_to_addr(drv_state->dev,IFX_DPS280_CFG_REG_ADDR, &config,1) < 0){
	    return -1;
	}
	dps280_delay_us(2000);
	/*Update state accordingly with proper scaling factors based on oversampling rates*/
	drv_state->tmp_osr_scale_coeff = dps280_get_scaling_coef(osr_temp);
	drv_state->prs_osr_scale_coeff = dps280_get_scaling_coef(osr_press);
	drv_state->press_mr = mr_press;
	drv_state->temp_mr  = mr_temp;
	drv_state->temp_osr = osr_temp;
	drv_state->press_osr = osr_press;
	drv_state->tmp_ext  = temp_src;

	return 0;
}

int dps280_sensor_init(struct hal_dev_s *dev, struct dps280_state *drv_state)
{
	int i;
	int32_t ret;
	uint8_t read_value;
	uint8_t config;

	if (!drv_state){
	    return -1;
	}

	if (!dev){
	    return -1;
	}
	drv_state->dev = dev;

	drv_state->cfg_word = 0;
	drv_state->enable = 0;

	if (hal_i2c_read_from_addr(drv_state->dev, IFX_DPS280_PROD_REV_ID_REG_ADDR, &read_value, 1) < 0){
		ret = -1;
		goto err_handler_iio;
	}

	if (read_value != IFX_DSPS280_PROD_REV_ID_VAL){
	    ret = -1;
		goto err_handler_iio;
	}

	/* attach bus connection instance to state*/
	

	/* from here wait for about 40ms till calibration coefficients become available*/
	dps280_delay_us(60000);

	for(i = 0;i < 50;i++)
	{
		dps280_delay_us(20000);
		if (hal_i2c_read_from_addr(drv_state->dev, IFX_DPS280_MEAS_CFG_REG_ADDR, &read_value, 1) < 0){
			ret = -1;
			goto err_handler_iio;
		}
		if((read_value&0xC0)!=0){
			break;
		}
	}
	if((read_value&0xC0)==0){
		printf("\n\nsig check failed,Reg(0x%x):0x%x\n\n\n",IFX_DPS280_MEAS_CFG_REG_ADDR,read_value);
		return -1;
	}

	////////////////////////////////////
    /* read now the calibration coeffs, temperature coef source and store in driver state*/
    ret = dps280_read_calib_coeffs(drv_state);

    if (ret < 0){
			goto err_handler_iio;
    }
	dps280_delay_us(20000);
    /* Now apply ADC Temperature gain settings*/
    /* First write valid signature on 0x0e and 0x0f
     * to unlock address 0x62 */

	config = 0xa5;
	hal_i2c_write_to_addr(drv_state->dev,0x0e, &config,1);
	dps280_delay_us(2000);
	config = 0x96;
	hal_i2c_write_to_addr(drv_state->dev,0x0f, &config,1);
	dps280_delay_us(2000);
	config = 0x02;
	hal_i2c_write_to_addr(drv_state->dev,0x62, &config,1);
	dps280_delay_us(2000);
	config = 0x00;
	hal_i2c_write_to_addr(drv_state->dev,0x0e, &config,1);
	dps280_delay_us(2000);
	config = 0x00;
	hal_i2c_write_to_addr(drv_state->dev,0x0f, &config,1);

    /* configure sensor for defauhal ODR settings*/
    ret = dps280_config(drv_state,
                        IFX_DPS280_TEMPERATURE_OSR,
                        IFX_DPS280_TEMPERATURE_MR,
                        IFX_DPS280_PRESSURE_OSR,
                        IFX_DPS280_PRESSURE_MR,
                        drv_state->tmp_ext);
    if (ret < 0){
		printf("[dps280 driver] config dps280 failed\n");
        goto err_handler_iio;
    }

	hal_i2c_read_from_addr(drv_state->dev, IFX_DPS280_TMP_CFG_REG_ADDR, &read_value, 1);
    /* activate sensor*/
    ret = dps280_resume(drv_state);
    if (ret < 0){
        goto err_handler_iio;
    }
	printf("[dps280 driver] config dps280 successful\n");
    return 0;

err_handler_iio:
	return ret;

}

void dps280_convert(struct dps280_report_s *rp,struct dps280_state *state,float * pressure,float *temp)
{
   
   double	press_raw;
   double	temp_raw;

   double 	temp_scaled;
   double 	temp_final;
   double 	press_scaled;
   double 	press_final;
   

	press_raw = (rp->pressure[2]) + (rp->pressure[1]<<8) + (rp->pressure[0] <<16);
    temp_raw  = (rp->temperature[2]) + (rp->temperature[1]<<8) + (rp->temperature[0] <<16);

	if(temp_raw > POW_2_23_MINUS_1){
		temp_raw = temp_raw - POW_2_24;
	}

	if(press_raw > POW_2_23_MINUS_1){
		press_raw = press_raw - POW_2_24;
	}

	temp_scaled = (double)temp_raw / (double) (state->tmp_osr_scale_coeff);

	temp_final =  (state->calib_coeffs.C0 /2.0f) + state->calib_coeffs.C1 * temp_scaled ;
    
	press_scaled = (double) press_raw / state->prs_osr_scale_coeff;

	press_final = state->calib_coeffs.C00 +
                      press_scaled *  (  state->calib_coeffs.C10 + press_scaled *
                      ( state->calib_coeffs.C20 + press_scaled * state->calib_coeffs.C30 )  ) +
                      temp_scaled * state->calib_coeffs.C01 +
                      temp_scaled * press_scaled * ( state->calib_coeffs.C11 +
                                                      press_scaled * state->calib_coeffs.C21 );


	press_final = press_final * 0.01f;	//to convert it into mBar

	*temp = temp_final;
    *pressure    = press_final;  //press_final;

}

int dps280_init(struct hal_dev_s *dev)
{
	struct hal_i2c_dev_s *client = (struct hal_i2c_dev_s *)dev;
	struct dps280_state *state = (struct dps280_state *)client->user_data;

	if(dps280_sensor_init(dev,state)){
		return -1;
	} else {
		return 0;
	}
}
int dps280_open(struct hal_dev_s *dev, uint16_t oflag)
{

    uint8_t read_value;
	if (hal_i2c_read_from_addr(dev, IFX_DPS280_PROD_REV_ID_REG_ADDR, &read_value, 1) < 0){
		return -1;
	}

	if (read_value != IFX_DSPS280_PROD_REV_ID_VAL){
		return -1;
	}

	return 0;
}
int dps280_close(struct hal_dev_s *dev)
{
	return 0;
}

int dps280_read(struct hal_dev_s *dev, void *buffer, int size,int pos)
{
	struct hal_i2c_dev_s *client = (struct hal_i2c_dev_s *)dev;
	struct dps280_state *state = (struct dps280_state *)client->user_data;
	struct baro_report_s *report = (struct baro_report_s *)buffer;
	struct dps280_report_s data;
	uint8_t	   read_buffer[IFX_DPS280_PSR_TMP_READ_LEN] = {0};
	int ret = 0;

	ret = hal_i2c_read_from_addr(dev,IFX_DPS280_PSR_TMP_READ_REG_ADDR,read_buffer,IFX_DPS280_PSR_TMP_READ_LEN);
	
	memcpy(data.pressure,&(read_buffer[0]),3);
	memcpy(data.temperature,&(read_buffer[3]),3);

	dps280_convert(&data,state,&report->pressure,&report->temperature);
	
	return ret;
}

struct dps280_state state_0;
struct hal_i2c_dev_s dps280_0;

int dps280_register(void)
{

	dps280_0.port	   = 0;
	dps280_0.address   = 0x77;
	dps280_0.cfg.flags = HAL_I2C_WR | HAL_I2C_RD;
	dps280_0.cfg.speed = 1200000;
	dps280_0.cfg.width = 8;
	dps280_0.user_data = &state_0;

	dps280_0.dev.init  = dps280_init;
	dps280_0.dev.open  = dps280_open;
	dps280_0.dev.close = dps280_close;
	dps280_0.dev.read  = dps280_read;
	dps280_0.dev.write = NULL;
	dps280_0.dev.ioctl = NULL;
	
	hal_i2c_device_register(&dps280_0,"dps280-0",HAL_O_RDWR | HAL_DEV_STANDALONE);
	return 0;
}

