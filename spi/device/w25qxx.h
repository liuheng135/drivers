/*
 * File      : spi_flash_w25qxx.h
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2011, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2011-12-16     aozima      the first version
 * 2012-08-23     aozima       add flash lock.
 */

#ifndef _LITOS_W25QXX_H_
#define _LITOS_W25QXX_H_

#include "stm32f4xx.h"

#include <litos.h>


#define FLASH_CS_PORT              	GPIOB
#define FLASH_CS_CLK               	RCC_AHB1Periph_GPIOB
#define FLASH_CS_PIN               	GPIO_Pin_12
#define FLASH_CS_SET  			   	FLASH_CS_PORT->BSRRL = FLASH_CS_PIN 
#define FLASH_CS_CLR				FLASH_CS_PORT->BSRRH = FLASH_CS_PIN 

struct w25qxx_dev_s
{
	struct lt_dev_block_geometry_s geometry;
    mutex_t            lock;
};

int w25qxx_init(void);


#endif 
