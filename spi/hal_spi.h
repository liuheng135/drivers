#ifndef _SPI_H_
#define _SPI_H_

#define SPI_ADAPTER_NUM_MAX  5

#ifndef NULL
#define NULL 0
#endif
#include "hal_device.h"

#define HAL_SPI_CPHA     (1<<0)                             /* bit[0]:CPHA, clock phase */
#define HAL_SPI_CPOL     (1<<1)                             /* bit[1]:CPOL, clock polarity */
/**
 * At CPOL=0 the base value of the clock is zero
 *  - For CPHA=0, data are captured on the clock's rising edge (low¡úhigh transition)
 *    and data are propagated on a falling edge (high¡úlow clock transition).
 *  - For CPHA=1, data are captured on the clock's falling edge and data are
 *    propagated on a rising edge.
 * At CPOL=1 the base value of the clock is one (inversion of CPOL=0)
 *  - For CPHA=0, data are captured on clock's falling edge and data are propagated
 *    on a rising edge.
 *  - For CPHA=1, data are captured on clock's rising edge and data are propagated
 *    on a falling edge.
 */
#define HAL_SPI_LSB      (0<<2)                             /* bit[2]: 0-LSB */
#define HAL_SPI_MSB      (1<<2)                             /* bit[2]: 1-MSB */

#define HAL_SPI_MASTER   (0<<3)                             /* SPI master device */
#define HAL_SPI_SLAVE    (1<<3)                             /* SPI slave device */

#define HAL_SPI_MODE_0       (0 | 0)                        /* CPOL = 0, CPHA = 0 */
#define HAL_SPI_MODE_1       (0 | HAL_SPI_CPHA)              /* CPOL = 0, CPHA = 1 */
#define HAL_SPI_MODE_2       (HAL_SPI_CPOL | 0)              /* CPOL = 1, CPHA = 0 */
#define HAL_SPI_MODE_3       (HAL_SPI_CPOL | HAL_SPI_CPHA)    /* CPOL = 1, CPHA = 1 */

#define HAL_SPI_MODE_MASK    (HAL_SPI_CPHA | HAL_SPI_CPOL | HAL_SPI_MSB)

#define HAL_SPI_CS_HIGH  (1<<4)                             /* Chipselect active high */
#define HAL_SPI_NO_CS    (1<<5)                             /* No chipselect */
#define HAL_SPI_3WIRE    (1<<6)                             /* SI/SO pin shared */
#define HAL_SPI_READY    (1<<7)                             /* Slave pulls low to pause */



/**
 * SPI configuration structure
 */
struct hal_spi_cfg_s{
	int speed;
	int mode;
	int width;
};

/**
 * SPI message
 */
struct hal_spi_msg_s{
	unsigned char *send_buffer;
	unsigned char *recv_buffer;
	int length;
	unsigned cs_take:1;
	unsigned cs_release:1;
};

struct hal_spi_dev_s{
	struct hal_dev_s dev;
	int  port;
	int  cs_id;
	struct hal_spi_cfg_s cfg;
	void *user_data;
};

struct hal_spi_adapter_s{
	int  (*init)(struct hal_spi_adapter_s *adpt);
	void (*configure)(struct hal_spi_adapter_s *adpt,struct hal_spi_cfg_s *cfg);
	int  (*transfer)(struct hal_spi_adapter_s *adpt,struct hal_spi_msg_s *msg);
	void (*cs_take)(int id);
	void (*cs_release)(int id);
	struct hal_spi_dev_s *owner;
	void *user_data;
	struct hal_lock_s lock;
	int   max_speed;
};

/*  for spi adapter drivers */
int hal_spi_init(void);
int hal_spi_adapter_register(struct hal_spi_adapter_s *adpt,int port);

/*  for spi slave address drivers */
int hal_spi_device_register(struct hal_spi_dev_s *spi_dev,char *name,uint16_t flags);
int hal_spi_transfer(struct hal_dev_s *dev,struct hal_spi_msg_s *msg);
int hal_spi_configure(struct hal_dev_s *dev,struct hal_spi_cfg_s *cfg);
int hal_spi_write_then_read(struct hal_dev_s *dev,void *txbuf, unsigned n_tx,
		void *rxbuf, unsigned n_rx);
int hal_spi_write_then_write(struct hal_dev_s *dev,void *txbuf1, unsigned n_tx1,
		void *txbuf2, unsigned n_tx2);
int hal_spi_write(struct hal_dev_s *dev,void *buf, unsigned n_tx);
int hal_spi_read(struct hal_dev_s *dev,void *buf, unsigned n_rx);

#endif
