/*
 * File      : spi_flash_w25qxx.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2011, RT-Thread Development Team
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rt-thread.org/license/LICENSE
 *
 * Change Logs:
 * Date           Author       Notes
 * 2011-12-16     aozima       the first version
 * 2012-05-06     aozima       can page write.
 * 2012-08-23     aozima       add flash lock.
 * 2012-08-24     aozima       fixed write status register BUG.
 */

#include <stdint.h>
#include "w25qxx.h"
#include "lsh.h"

#define FLASH_TRACE(format,...)\
	do {\
			lsh_printf(format,##__VA_ARGS__ );\
	} while (0)

#define PAGE_SIZE           4096

/* JEDEC Manufacturer¡¯s ID */
#define MF_ID           (0xEF)
/* JEDEC Device ID: Memory type and Capacity */
#define MTC_W25Q16_BV_CL_CV   (0x4015) /* W25Q16BV W25Q16CL W25Q16CV  */
#define MTC_W25Q16_DW         (0x6015) /* W25Q16DW  */
#define MTC_W25Q32_BV         (0x4016) /* W25Q32BV */
#define MTC_W25Q32_DW         (0x6016) /* W25Q32DW */
#define MTC_W25Q64_BV_CV      (0x4017) /* W25Q64BV W25Q64CV */
#define MTC_W25Q64_DW         (0x4017) /* W25Q64DW */
#define MTC_W25Q128_BV        (0x4018) /* W25Q128BV */
#define MTC_W25Q256_FV        (TBD)    /* W25Q256FV */

#define MX25L16               (0x3015) /* MX25L16  */

/* command list */
#define CMD_WRSR                    (0x01)  /* Write Status Register */
#define CMD_PP                      (0x02)  /* Page Program */
#define CMD_READ                    (0x03)  /* Read Data */
#define CMD_WRDI                    (0x04)  /* Write Disable */
#define CMD_RDSR1                   (0x05)  /* Read Status Register-1 */
#define CMD_WREN                    (0x06)  /* Write Enable */
#define CMD_FAST_READ               (0x0B)  /* Fast Read */
#define CMD_ERASE_4K                (0x20)  /* Sector Erase:4K */
#define CMD_RDSR2                   (0x35)  /* Read Status Register-2 */
#define CMD_ERASE_32K               (0x52)  /* 32KB Block Erase */
#define CMD_JEDEC_ID                (0x9F)  /* Read JEDEC ID */
#define CMD_ERASE_full              (0xC7)  /* Chip Erase */
#define CMD_ERASE_64K               (0xD8)  /* 64KB Block Erase */

#define DUMMY                       (0xFF)



static lt_status flash_lock(struct w25qxx_dev_s * flash_device)
{
   return lt_mutex_take(flash_device->lock, 10);
}

static void flash_unlock(struct w25qxx_dev_s * flash_device)
{
    lt_mutex_release(flash_device->lock);
}

static uint8_t w25qxx_read_status(struct lt_dev_s *dev)
{
	uint8_t recv;
	uint8_t send;
	send = CMD_RDSR1;
    spi_write_then_read(dev, &send,1,&recv,1);
	return recv;
}

static void w25qxx_wait_busy(struct lt_dev_s *dev)
{
    while( w25qxx_read_status(dev) & (0x01));
}

/** \brief read [size] byte from [offset] to [buffer]
 *
 * \param offset uint32_t unit : byte
 * \param buffer uint8_t*
 * \param size uint32_t   unit : byte
 * \return uint32_t byte for read
 *
 */
static uint32_t w25qxx_read(struct lt_dev_s *dev, uint8_t * buffer, uint32_t size,uint32_t offset)
{
    uint8_t send_buffer[4];

    send_buffer[0] = CMD_WRDI;
    spi_write(dev, send_buffer, 1);

    send_buffer[0] = CMD_READ;
    send_buffer[1] = (uint8_t)(offset>>16);
    send_buffer[2] = (uint8_t)(offset>>8);
    send_buffer[3] = (uint8_t)(offset);

    spi_write_then_read(dev,
                          send_buffer, 4,
                          buffer, size);

    return size;
}

/** \brief write N page on [page]
 *
 * \param page_addr uint32_t unit : byte (4096 * N,1 page = 4096byte)
 * \param buffer const uint8_t*
 * \return uint32_t
 *
 */
uint32_t w25qxx_page_write(struct lt_dev_s *dev,uint32_t page_addr, uint8_t* buffer)
{
    uint32_t index;
    uint8_t send_buffer[4];

    //RT_ASSERT((page_addr&0xFF) == 0); /* page addr must align to 256byte. */

    send_buffer[0] = CMD_WREN;
    spi_write(dev, send_buffer, 1);

    send_buffer[0] = CMD_ERASE_4K;
    send_buffer[1] = (page_addr >> 16);
    send_buffer[2] = (page_addr >> 8);
    send_buffer[3] = (page_addr);
    spi_write(dev, send_buffer, 4);

    w25qxx_wait_busy(dev); // wait erase done.

    for(index=0; index < (PAGE_SIZE / 256); index++)
    {
        send_buffer[0] = CMD_WREN;
        spi_write(dev, send_buffer, 1);

        send_buffer[0] = CMD_PP;
        send_buffer[1] = (uint8_t)(page_addr >> 16);
        send_buffer[2] = (uint8_t)(page_addr >> 8);
        send_buffer[3] = (uint8_t)(page_addr);

        spi_write_then_write(dev,
                              send_buffer,
                              4,
                              buffer,
                              256);

        buffer += 256;
        page_addr += 256;
        w25qxx_wait_busy(dev);
    }

    send_buffer[0] = CMD_WRDI;
    spi_write(dev, send_buffer, 1);

    return PAGE_SIZE;
}


static void w25qxx_pin_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(FLASH_CS_CLK , ENABLE);

	GPIO_InitStructure.GPIO_Pin = FLASH_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT  ;   
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_Init(FLASH_CS_PORT, &GPIO_InitStructure);
	GPIO_SetBits(FLASH_CS_PORT,FLASH_CS_PIN);
}	

static int w25qxx_flash_init(struct lt_dev_s *dev)
{
	struct lt_spi_dev_s *spi_dev = (struct lt_spi_dev_s *)dev;
	struct w25qxx_dev_s *flash = (struct w25qxx_dev_s *)spi_dev->user_data;
	
	w25qxx_pin_init();
	
	
	
	/* init flash */
    {
        uint8_t cmd;
        uint8_t id_recv[3];
        uint16_t memory_type_capacity;


        cmd = 0xFF; /* reset SPI FLASH, cancel all cmd in processing. */
        spi_write(dev, &cmd, 1);

        cmd = CMD_WRDI;
        spi_write(dev, &cmd, 1);

        /* read flash id */
        cmd = CMD_JEDEC_ID;
        spi_write_then_read(dev, &cmd, 1, id_recv, 3);

        if(id_recv[0] != MF_ID)
        {
            FLASH_TRACE("Manufacturers ID error!\r\n");
            FLASH_TRACE("JEDEC Read-ID Data : %02X %02X %02X\r\n", id_recv[0], id_recv[1], id_recv[2]);
            return -1;
        }
		flash->lock = lt_mutex_create();
		
		if(flash->lock == NULL){
			FLASH_TRACE("Can not create mutex for flash!\r\n");
			return -1;
		}
		
        flash->geometry.bytes_per_sector = 4096;
        flash->geometry.block_size = 4096; /* block erase: 4k */

        /* get memory type and capacity */
        memory_type_capacity = id_recv[1];
        memory_type_capacity = (memory_type_capacity << 8) | id_recv[2];

        if(memory_type_capacity == MTC_W25Q128_BV)
        {
            FLASH_TRACE("W25Q128BV detection\r\n");
            flash->geometry.sector_count = 4096;
        }
        else if(memory_type_capacity == MTC_W25Q64_BV_CV)
        {
            FLASH_TRACE("W25Q64BV or W25Q64CV detection\r\n");
            flash->geometry.sector_count = 2048;
        }
        else if(memory_type_capacity == MTC_W25Q64_DW)
        {
            FLASH_TRACE("W25Q64DW detection\r\n");
            flash->geometry.sector_count = 2048;
        }
        else if(memory_type_capacity == MTC_W25Q32_BV)
        {
            FLASH_TRACE("W25Q32BV detection\r\n");
            flash->geometry.sector_count = 1024;
        }
        else if(memory_type_capacity == MTC_W25Q32_DW)
        {
            FLASH_TRACE("W25Q32DW detection\r\n");
            flash->geometry.sector_count = 1024;
        }
        else if(memory_type_capacity == MTC_W25Q16_BV_CL_CV)
        {
            FLASH_TRACE("W25Q16BV or W25Q16CL or W25Q16CV detection\r\n");
            flash->geometry.sector_count = 512;
        }
        else if(memory_type_capacity == MTC_W25Q16_DW)
        {
            FLASH_TRACE("W25Q16DW detection\r\n");
            flash->geometry.sector_count = 512;
        }
		else if(memory_type_capacity == MX25L16)
        {
            FLASH_TRACE("MX25L16 detection\r\n");
            flash->geometry.sector_count = 512;
        }
        else
        {
            FLASH_TRACE("Memory Capacity error!\r\n");
            return -1;
        }
    }
	
    return 0;
}

static int w25qxx_flash_open(struct lt_dev_s *dev, uint16_t oflag)
{
    uint8_t send_buffer[3];
	
	struct lt_spi_dev_s *spi_dev = (struct lt_spi_dev_s *)dev;
	struct w25qxx_dev_s *flash = (struct w25qxx_dev_s *)spi_dev->user_data;

    if(flash_lock(flash) == LT_OK){

		send_buffer[0] = CMD_WREN;
		spi_write(dev, send_buffer, 1);

		send_buffer[0] = CMD_WRSR;
		send_buffer[1] = 0;
		send_buffer[2] = 0;
		spi_write(dev, send_buffer, 3);

		w25qxx_wait_busy(dev);

		flash_unlock(flash);
	}else{
		return -1;
	}

    return 0;
}

static int w25qxx_flash_close(struct lt_dev_s *dev)
{
    return 0;
}

static int w25qxx_flash_control(struct lt_dev_s *dev, uint8_t cmd, void *args)
{
	struct lt_spi_dev_s *spi_dev = (struct lt_spi_dev_s *)dev;
	struct w25qxx_dev_s *flash = (struct w25qxx_dev_s *)spi_dev->user_data;
	
    if (cmd == DEVICE_CTRL_BLK_GETGEOME)
    {
        struct lt_dev_block_geometry_s *geometry;

        geometry = (struct lt_dev_block_geometry_s *)args;
        if (geometry == NULL) return -1;

        geometry->bytes_per_sector = flash->geometry.bytes_per_sector;
        geometry->sector_count = flash->geometry.sector_count;
        geometry->block_size = flash->geometry.block_size;
    }

    return 0;
}

static void w25qxx_cs_take(void)
{
	GPIO_ResetBits(FLASH_CS_PORT,FLASH_CS_PIN);
}

static void w25qxx_cs_release(void)
{
	GPIO_SetBits(FLASH_CS_PORT,FLASH_CS_PIN);
}

static int w25qxx_flash_read(struct lt_dev_s *dev,
                                   void* buffer,
                                   int size,
								   int pos)
{
	struct lt_spi_dev_s *spi_dev = (struct lt_spi_dev_s *)dev;
	struct w25qxx_dev_s *flash = (struct w25qxx_dev_s *)spi_dev->user_data;
	
    if(flash_lock(flash) == LT_OK){
		w25qxx_read(dev, buffer,size*flash->geometry.bytes_per_sector,
					pos*flash->geometry.bytes_per_sector);

		flash_unlock(flash);
	}else{
		return -1;
	}

    return size;
}

static int w25qxx_flash_write(struct lt_dev_s *dev,
                                    const void* buffer,
                                    int size,
									int pos)
{
    int i = 0;
    int block = size;
    uint8_t * ptr = ( uint8_t *)buffer;
	struct lt_spi_dev_s *spi_dev = (struct lt_spi_dev_s *)dev;
	struct w25qxx_dev_s *flash = (struct w25qxx_dev_s *)spi_dev->user_data;
	
    if(flash_lock(flash) == LT_OK){
		while(block--){
			w25qxx_page_write(dev,(pos + i)*flash->geometry.bytes_per_sector,
							  ptr);
			ptr += PAGE_SIZE;
			i++;
		}

		flash_unlock(flash);
	}else{
		return -1;
	}
    return size;
}

struct lt_spi_dev_s  w25qxx_spi_dev;
struct w25qxx_dev_s  w25qxx_data;

int w25qxx_init(void)
{
    w25qxx_spi_dev.port       = 1;
	w25qxx_spi_dev.cfg.speed  = 10000000; /*  50MHz */
	w25qxx_spi_dev.cfg.mode   = LT_SPI_MODE_0 | LT_SPI_MSB;
	w25qxx_spi_dev.cfg.width  = 8;
	w25qxx_spi_dev.cs_take    = w25qxx_cs_take;
	w25qxx_spi_dev.cs_release = w25qxx_cs_release;
	
    /* register device */
    w25qxx_spi_dev.dev.type    = LT_Device_Class_Block;
    w25qxx_spi_dev.dev.init    = w25qxx_flash_init;
    w25qxx_spi_dev.dev.open    = w25qxx_flash_open;
    w25qxx_spi_dev.dev.close   = w25qxx_flash_close;
    w25qxx_spi_dev.dev.read    = w25qxx_flash_read;
    w25qxx_spi_dev.dev.write   = w25qxx_flash_write;
    w25qxx_spi_dev.dev.ioctl   = w25qxx_flash_control;

    w25qxx_spi_dev.user_data = &w25qxx_data;

    return spi_device_register(&w25qxx_spi_dev, "w25qxx-0",
                       DEVICE_FLAG_RDWR | DEVICE_FLAG_STANDALONE);
}
