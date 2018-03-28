#ifndef _HAL_SENSOR_DEFINE_H_
#define _HAL_SENSOR_DEFINE_H_

#ifdef __cplusplus
extern "C" {
#endif

enum sensor_cmd_e{
	SENSOR_CMD_SET_RANGE1 = 0,
	SENSOR_CMD_SET_RANGE2,
	SENSOR_CMD_SET_RANGE3,
};

struct baro_report_s{
	float pressure;      /* unit: mbar  */ 
	float temperature;   /* unit: degree  */
};

struct acc_report_s{
	float accel[3];       /* unit: m/s^2  */ 
	float temperature;    /* unit: degree  */
};

struct gyro_report_s{
	float gyro[3];        /* unit: rad/s  */ 
	float temperature;    /* unit: degree  */
};

struct mag_report_s{
	float mag[3];        /* unit: mguess  */ 
	float temperature;   /* unit: degree  */
};

struct imu_report_s{
	struct acc_report_s  acc;
	struct gyro_report_s gyro;
};


#ifdef __cplusplus
}
#endif

#endif
