#include "hal_device.h"
#include "stm32f4xx.h"

struct flash_sector_s{
	uint32_t addr;
	uint32_t size;
};

#define SECTOR_MASK               ((uint32_t)0xFFFFFF07)

#define RDP_KEY                  ((uint16_t)0x00A5)
#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)
#define FLASH_OPT_KEY1           ((uint32_t)0x08192A3B)
#define FLASH_OPT_KEY2           ((uint32_t)0x4C5D6E7F)

#define ADDR_FLASH_SECTOR_0    ((uint32_t)0x08000000)
#define ADDR_FLASH_SECTOR_1    ((uint32_t)0x08004000)
#define ADDR_FLASH_SECTOR_2    ((uint32_t)0x08008000)
#define ADDR_FLASH_SECTOR_3    ((uint32_t)0x0800C000)
#define ADDR_FLASH_SECTOR_4    ((uint32_t)0x08010000)
#define ADDR_FLASH_SECTOR_5    ((uint32_t)0x08020000)
#define ADDR_FLASH_SECTOR_6    ((uint32_t)0x08040000)
#define ADDR_FLASH_SECTOR_7    ((uint32_t)0x08060000)
#define ADDR_FLASH_SECTOR_8    ((uint32_t)0x08080000)
#define ADDR_FLASH_SECTOR_9    ((uint32_t)0x080A0000)
#define ADDR_FLASH_SECTOR_10   ((uint32_t)0x080C0000)
#define ADDR_FLASH_SECTOR_11   ((uint32_t)0x080E0000)

#define FLASH_Sector_0     ((uint16_t)0x0000) /*!< Sector Number 0 */
#define FLASH_Sector_1     ((uint16_t)0x0008) /*!< Sector Number 1 */
#define FLASH_Sector_2     ((uint16_t)0x0010) /*!< Sector Number 2 */
#define FLASH_Sector_3     ((uint16_t)0x0018) /*!< Sector Number 3 */
#define FLASH_Sector_4     ((uint16_t)0x0020) /*!< Sector Number 4 */
#define FLASH_Sector_5     ((uint16_t)0x0028) /*!< Sector Number 5 */
#define FLASH_Sector_6     ((uint16_t)0x0030) /*!< Sector Number 6 */
#define FLASH_Sector_7     ((uint16_t)0x0038) /*!< Sector Number 7 */
#define FLASH_Sector_8     ((uint16_t)0x0040) /*!< Sector Number 8 */
#define FLASH_Sector_9     ((uint16_t)0x0048) /*!< Sector Number 9 */
#define FLASH_Sector_10    ((uint16_t)0x0050) /*!< Sector Number 10 */
#define FLASH_Sector_11    ((uint16_t)0x0058) /*!< Sector Number 11 */
#define IS_FLASH_SECTOR(SECTOR) (((SECTOR) == FLASH_Sector_0) || ((SECTOR) == FLASH_Sector_1) ||\
                                 ((SECTOR) == FLASH_Sector_2) || ((SECTOR) == FLASH_Sector_3) ||\
                                 ((SECTOR) == FLASH_Sector_4) || ((SECTOR) == FLASH_Sector_5) ||\
                                 ((SECTOR) == FLASH_Sector_6) || ((SECTOR) == FLASH_Sector_7) ||\
                                 ((SECTOR) == FLASH_Sector_8) || ((SECTOR) == FLASH_Sector_9) ||\
                                 ((SECTOR) == FLASH_Sector_10) || ((SECTOR) == FLASH_Sector_11))
#define IS_FLASH_ADDRESS(ADDRESS) ((((ADDRESS) >= 0x08000000) && ((ADDRESS) < 0x080FFFFF)) ||\
                                   (((ADDRESS) >= 0x1FFF7800) && ((ADDRESS) < 0x1FFF7A0F)))  


#define FLASH_FLAG_EOP                 ((uint32_t)0x00000001)  /*!< FLASH End of Operation flag */
#define FLASH_FLAG_OPERR               ((uint32_t)0x00000002)  /*!< FLASH operation Error flag */
#define FLASH_FLAG_WRPERR              ((uint32_t)0x00000010)  /*!< FLASH Write protected error flag */
#define FLASH_FLAG_PGAERR              ((uint32_t)0x00000020)  /*!< FLASH Programming Alignment error flag */
#define FLASH_FLAG_PGPERR              ((uint32_t)0x00000040)  /*!< FLASH Programming Parallelism error flag  */
#define FLASH_FLAG_PGSERR              ((uint32_t)0x00000080)  /*!< FLASH Programming Sequence error flag  */
#define FLASH_FLAG_BSY                 ((uint32_t)0x00010000)  /*!< FLASH Busy flag */ 
#define IS_FLASH_CLEAR_FLAG(FLAG) ((((FLAG) & (uint32_t)0xFFFFFF0C) == 0x00000000) && ((FLAG) != 0x00000000))
#define IS_FLASH_GET_FLAG(FLAG)  (((FLAG) == FLASH_FLAG_EOP) || ((FLAG) == FLASH_FLAG_OPERR) || \
                                  ((FLAG) == FLASH_FLAG_WRPERR) || ((FLAG) == FLASH_FLAG_PGAERR) || \
                                  ((FLAG) == FLASH_FLAG_PGPERR) || ((FLAG) == FLASH_FLAG_PGSERR) || \
                                  ((FLAG) == FLASH_FLAG_BSY))

#define VoltageRange_1        ((uint8_t)0x00)  /*!< Device operating range: 1.8V to 2.1V */
#define VoltageRange_2        ((uint8_t)0x01)  /*!<Device operating range: 2.1V to 2.7V */
#define VoltageRange_3        ((uint8_t)0x02)  /*!<Device operating range: 2.7V to 3.6V */
#define VoltageRange_4        ((uint8_t)0x03)  /*!<Device operating range: 2.7V to 3.6V + External Vpp */


#define FLASH_PSIZE_BYTE           ((uint32_t)0x00000000)
#define FLASH_PSIZE_HALF_WORD      ((uint32_t)0x00000100)
#define FLASH_PSIZE_WORD           ((uint32_t)0x00000200)
#define FLASH_PSIZE_DOUBLE_WORD    ((uint32_t)0x00000300)
#define CR_PSIZE_MASK              ((uint32_t)0xFFFFFCFF)

typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_PGS,
  FLASH_ERROR_PGP,
  FLASH_ERROR_PGA,
  FLASH_ERROR_WRP,
  FLASH_ERROR_PROGRAM,
  FLASH_ERROR_OPERATION,
  FLASH_COMPLETE
}FLASH_Status;


void FLASH_Unlock(void)
{
  if((FLASH->CR & FLASH_CR_LOCK) != RESET){
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }  
}


void FLASH_Lock(void)
{
  FLASH->CR |= FLASH_CR_LOCK;
}

FLASH_Status FLASH_GetStatus(void)
{
  FLASH_Status flashstatus = FLASH_COMPLETE;
  
  if((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY) 
  {
    flashstatus = FLASH_BUSY;
  }
  else 
  {  
    if((FLASH->SR & FLASH_FLAG_WRPERR) != (uint32_t)0x00)
    { 
      flashstatus = FLASH_ERROR_WRP;
    }
    else 
    {
      if((FLASH->SR & (uint32_t)0xEF) != (uint32_t)0x00)
      {
        flashstatus = FLASH_ERROR_PROGRAM; 
      }
      else
      {
        if((FLASH->SR & FLASH_FLAG_OPERR) != (uint32_t)0x00)
        {
          flashstatus = FLASH_ERROR_OPERATION;
        }
        else
        {
          flashstatus = FLASH_COMPLETE;
        }
      }
    }
  }
  /* Return the FLASH Status */
  return flashstatus;
}

FLASH_Status FLASH_WaitForLastOperation(void)
{ 
  __IO FLASH_Status status = FLASH_COMPLETE;
   
  /* Check for the FLASH Status */
  status = FLASH_GetStatus();

  /* Wait for the FLASH operation to complete by polling on BUSY flag to be reset.
     Even if the FLASH operation fails, the BUSY flag will be reset and an error
     flag will be set */
  while(status == FLASH_BUSY)
  {
    status = FLASH_GetStatus();
  }
  /* Return the operation status */
  return status;
}


uint32_t FLASH_GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_Sector_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_Sector_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_Sector_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_Sector_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_Sector_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_Sector_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_Sector_6;  
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_Sector_7;  
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_Sector_8;  
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_Sector_9;  
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_Sector_10;  
  }
  else
  {
    sector = FLASH_Sector_11;  
  }

  return sector;
}


FLASH_Status FLASH_EraseSector(uint32_t FLASH_Sector, uint8_t VoltageRange)
{
  uint32_t tmp_psize = 0x0;
  FLASH_Status status = FLASH_COMPLETE;

  if(VoltageRange == VoltageRange_1)
  {
     tmp_psize = FLASH_PSIZE_BYTE;
  }
  else if(VoltageRange == VoltageRange_2)
  {
    tmp_psize = FLASH_PSIZE_HALF_WORD;
  }
  else if(VoltageRange == VoltageRange_3)
  {
    tmp_psize = FLASH_PSIZE_WORD;
  }
  else
  {
    tmp_psize = FLASH_PSIZE_DOUBLE_WORD;
  }
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation();
  
  if(status == FLASH_COMPLETE)
  { 
    /* if the previous operation is completed, proceed to erase the sector */
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= tmp_psize;
    FLASH->CR &= SECTOR_MASK;
    FLASH->CR |= FLASH_CR_SER | FLASH_Sector;
    FLASH->CR |= FLASH_CR_STRT;
    
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();
    
    /* if the erase operation is completed, disable the SER Bit */
    FLASH->CR &= (~FLASH_CR_SER);
    FLASH->CR &= SECTOR_MASK; 
  }
  /* Return the Erase Status */
  return status;
}

FLASH_Status FLASH_ProgramByte(uint32_t Address, uint8_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation();
  
  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to program the new data */
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= FLASH_PSIZE_BYTE;
    FLASH->CR |= FLASH_CR_PG;
  
    *(__IO uint8_t*)Address = Data;
        
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();

    /* if the program operation is completed, disable the PG Bit */
    FLASH->CR &= (~FLASH_CR_PG);
  } 

  /* Return the Program Status */
  return status;
}

FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data)
{
  FLASH_Status status = FLASH_COMPLETE;

  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation();
  
  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to program the new data */
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= FLASH_PSIZE_WORD;
    FLASH->CR |= FLASH_CR_PG;
  
    *(__IO uint32_t*)Address = Data;
        
    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation();

    /* if the program operation is completed, disable the PG Bit */
    FLASH->CR &= (~FLASH_CR_PG);
  } 
  /* Return the Program Status */
  return status;
}

static int param_read(struct hal_dev_s *dev, void *buffer, int buflen,int pos)
{
	int i;
	uint32_t addr;
	uint32_t *data = (uint32_t *)buffer;
	struct flash_sector_s *sct = (struct flash_sector_s *)dev->priv_data;
	
	addr = sct->addr;
	
	for(i = 0;i < buflen / 4;i++){
		*data = *((volatile uint32_t *)addr);
		data++;
		addr+=4;
	}
}

static int param_init(struct hal_dev_s *dev)
{
	return 0;
}

static int param_open(struct hal_dev_s *dev,uint16_t flag)
{
	return 0;
}

static int param_write(struct hal_dev_s *dev, const void *buffer, int buflen,int pos)
{
	uint32_t sector_number;
	uint32_t *data = (uint32_t *)buffer;
	uint32_t  addr;

	int i;
	
	struct flash_sector_s *sct = (struct flash_sector_s *)dev->priv_data;
	
	if(buflen > sct->size){
		return -1;
	}
	
	if(VDD_VALUE < 2700){
		return -1;
	}
	
	FLASH_Unlock();
	sector_number = FLASH_GetSector(sct->addr);
	FLASH_EraseSector(sector_number,VoltageRange_3);
	
	addr = sct->addr;
	for(i = 0;i < buflen / 4;i++){
		FLASH_ProgramWord(addr,*data);
		data++;
		addr+=4;
	}
	FLASH_Lock();
	return buflen;
}

struct flash_sector_s sector7 = {
	.addr = 0x08060000,
	.size = 128 * 1024,
};

struct hal_dev_s param_dev;

int param_dev_register(void)
{
	param_dev.open = param_open;
	param_dev.read = param_read;
	param_dev.write = param_write;
	param_dev.priv_data = &sector7;
	param_dev.init  = param_init;
	
	hal_dev_register(&param_dev,"param",HAL_O_RDWR);
}