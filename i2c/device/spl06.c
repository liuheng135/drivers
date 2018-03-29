#include <linux/i2c.h>
#include <linux/i2c/uav_sensor.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/types.h>

#define SENSOR_NAME "spl06"
#define SENSOR_I2C_SLAVE_ADDRESS 0x76
#define SENSOR_I2C_SPEED		(1200 * 1000)

static struct i2c_client *spl06_i2c_client = NULL;

#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3
#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1

#define SPL06_REG_PRE		0x00
#define SPL06_REG_TEMP		0x03
#define SPL06_REG_PRS_CFG	0x06
#define SPL06_REG_TMP_CFG	0x07
#define SPL06_REG_MEAS_CFG	0x08
#define SPL06_REG_CFG		0x09
#define SPL06_REG_RST		0x0C
#define SPL06_REG_ID		0x0D
#define SPL06_REG_CONF		0x10

typedef struct
{
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;       
    uint8_t id;
    int32_t kP;    
    int32_t kT;
} spl06_calib_data;

struct spl06_report_s{
	spl06_calib_data calib;
	int32_t rawPre;
	int32_t rawTemp;
};

static spl06_calib_data spl06_calib; 

static const unsigned short normal_i2c[2] = {SENSOR_I2C_SLAVE_ADDRESS, I2C_CLIENT_END};

static dev_t devno;
static struct cdev * cdev; 
static struct device *device;
static struct class *cls;

int spl06_read8(struct i2c_client *client,uint8_t reg,uint8_t * data)
{
	int ret;
  
	ret = i2c_smbus_read_i2c_block_data(client,reg,1,data);

	return ret;
}

int spl06_write8(struct i2c_client *client,uint8_t reg,uint8_t val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client,reg,val);

	return ret;
}

static bool spl06_get_id(struct i2c_client * client)
{
	uint8_t id;
	int ret;
	
	ret = spl06_read8(client,SPL06_REG_ID,&id);
	if(ret < 0){
		printk("==%s== get id failed:%x\r\n",__func__,id);
		return false;
	}

	spl06_calib.id = id;
	printk("==%s== %x\r\n",__func__,id);

	return true;
}

static int spl06_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	pr_info("%s: addr=0x%x\n",__func__,client->addr);
	strlcpy(info->type, SENSOR_NAME, I2C_NAME_SIZE);
	return 0;
}


int spl06_readCoefficients(struct i2c_client *client)
{
	uint8_t buf[18];
    uint32_t h;
    uint32_t m;
    uint32_t l;

	i2c_smbus_read_i2c_block_data(client,SPL06_REG_CONF,18,buf);

    h = buf[0];
    l = buf[1];
    spl06_calib.c0 = (int16_t)h<<4 | l>>4;
    spl06_calib.c0 = (spl06_calib.c0&0x0800)?(0xF000|spl06_calib.c0):spl06_calib.c0;
    h = buf[1];
    l = buf[2];
    spl06_calib.c1 = (int16_t)(h&0x0F)<<8 | l;
    spl06_calib.c1 = (spl06_calib.c1&0x0800)?(0xF000|spl06_calib.c1):spl06_calib.c1;
    h = buf[3];
    m = buf[4];
    l = buf[5];
    spl06_calib.c00 = (int32_t)h<<12 | (int32_t)m<<4 | (int32_t)l>>4;
    spl06_calib.c00 = (spl06_calib.c00&0x080000)?(0xFFF00000|spl06_calib.c00):spl06_calib.c00;
    h = buf[5];
    m = buf[6];
    l = buf[7];
    spl06_calib.c10 = (int32_t)(h&0x0F)<<16 | (int32_t)m<<8 | l;
    spl06_calib.c10 = (spl06_calib.c10&0x080000)?(0xFFF00000|spl06_calib.c10):spl06_calib.c10;
    h = buf[8];
    l = buf[9];
	spl06_calib.c01 = (int16_t)h<<8 | l;
    h = buf[10];
    l = buf[11];
	spl06_calib.c11 = (int16_t)h<<8 | l;
    h = buf[12];
    l = buf[13];
	spl06_calib.c20 = (int16_t)h<<8 | l;
    h = buf[14];
    l = buf[15];
	spl06_calib.c21 = (int16_t)h<<8 | l;
    h = buf[16];
    l = buf[17];
	spl06_calib.c30 = (int16_t)h<<8 | l;

#if 0
	{
		int i = 0;
		for(i=0;i<18;i++){
			printk("%d(%02x)\r\n",i,buf[i]);
		}
	}

	printk("C0:%d\r\n",spl06_calib.c0);
	printk("C1:%d\r\n",spl06_calib.c1);
	printk("C00:%d\r\n",spl06_calib.c00);
	printk("C10:%d\r\n",spl06_calib.c10);
	printk("C01:%d\r\n",spl06_calib.c01);
	printk("C11:%d\r\n",spl06_calib.c11);
	printk("C20:%d\r\n",spl06_calib.c20);
	printk("C21:%d\r\n",spl06_calib.c21);
	printk("C30:%d\r\n",spl06_calib.c30);
#endif

	return 0;
}

void spl06_rateset(struct i2c_client *client,uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
    uint8_t reg = 0;
    int32_t i32kPkT = 0;
	
    switch(u8SmplRate){
        case 2:
            reg |= (1<<4);
            break;
        case 4:
            reg |= (2<<4);
            break;
        case 8:
            reg |= (3<<4);
            break;
        case 16:
            reg |= (4<<4);
            break;
        case 32:
            reg |= (5<<4);
            break;
        case 64:
            reg |= (6<<4);
            break;
        case 128:
            reg |= (7<<4);
            break;
        case 1:
        default:
            break;
    }

	switch(u8OverSmpl){
        case 2:
            reg |= 1;
            i32kPkT = 1572864;
            break;
        case 4:
            reg |= 2;
            i32kPkT = 3670016;
            break;
        case 8:
            reg |= 3;
            i32kPkT = 7864320;
            break;
        case 16:
            i32kPkT = 253952;
            reg |= 4;
            break;
        case 32:
            i32kPkT = 516096;
            reg |= 5;
            break;
        case 64:
            i32kPkT = 1040384;
            reg |= 6;
            break;
        case 128:
            i32kPkT = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            i32kPkT = 524288;
            break;
    }

    if(iSensor == PRESSURE_SENSOR){
        spl06_calib.kP = i32kPkT;
        spl06_write8(client,SPL06_REG_PRS_CFG,reg);
        if(u8OverSmpl > 8){
            spl06_read8(client,SPL06_REG_CFG,&reg);
            spl06_write8(client,SPL06_REG_CFG,reg | 0x04);
        }else{
            spl06_read8(client,SPL06_REG_CFG,&reg);
            spl06_write8(client,SPL06_REG_CFG,reg & (~0x04));
        }
    }
	
    if(iSensor == TEMPERATURE_SENSOR){
        spl06_calib.kT = i32kPkT;
        spl06_write8(client,SPL06_REG_TMP_CFG,reg|0x80);  //Using mems temperature
        if(u8OverSmpl > 8){
            spl06_read8(client,SPL06_REG_CFG,&reg);
            spl06_write8(client,SPL06_REG_CFG,reg | 0x08);
        }else{
            spl06_read8(client,SPL06_REG_CFG,&reg);
            spl06_write8(client,SPL06_REG_CFG,reg & (~0x08));
        }
    }

}

static int spl06_init_client(struct i2c_client *client)
{
	int ret;

	//spl06_write8(client,SPL06_REG_RST,0x09);
	//mdelay(10);
	
	ret = spl06_readCoefficients(client);
	if(ret < 0){
		return -1;
	}

    spl06_rateset(client,PRESSURE_SENSOR,32,16);
    spl06_rateset(client,TEMPERATURE_SENSOR,4,4);

	
    spl06_write8(client,SPL06_REG_MEAS_CFG,CONTINUOUS_P_AND_T + 4);

	printk("==%s== done\r\n",__func__);

	return 0;
}

static int spl06_measure(struct i2c_client *client,struct spl06_report_s *pf)
{
	int ret;
	uint8_t buf[6];
	uint8_t h,m,l;
	
	ret = i2c_smbus_read_i2c_block_data(client,SPL06_REG_PRE,6,buf);
	if(ret < 0){
		return -1;
	}

	h = buf[3];
	m = buf[4];
	l = buf[5];
	pf->rawTemp = (int32_t)h<<16 | (int32_t)m<<8 | (int32_t)l;
	pf->rawTemp = (pf->rawTemp&0x800000) ? (0xFF000000|pf->rawTemp) : pf->rawTemp;

	h = buf[0];
	m = buf[1];
	l = buf[2];
	pf->rawPre = (int32_t)h<<16 | (int32_t)m<<8 | (int32_t)l;
	pf->rawPre = (pf->rawPre&0x800000) ? (0xFF000000|pf->rawPre) : pf->rawPre;
	
	memcpy(&(pf->calib),&spl06_calib,sizeof(spl06_calib_data));
	
	return 0;
}

int spl06_open(struct inode *inode, struct file *filp)
{
	int ret;
	
	printk("==%s==\r\n",__func__);

	if(spl06_get_id(spl06_i2c_client) == false){
		return -1;
	}
	
	ret = spl06_init_client(spl06_i2c_client);
	if(ret < 0){
		return -1;
	}
	
	return 0; 
}

int spl06_release(struct inode *inode, struct file *filp)
{
	printk("==%s==\r\n",__func__);
	return 0;
}

static ssize_t spl06_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	int ret;
	struct spl06_report_s data;
	
	ret = spl06_measure(spl06_i2c_client,&data);
	if(ret < 0){
		return -1;
	}
	
	copy_to_user(buf,(void *)&data,sizeof(data));
	
	return 1;
}

static const struct file_operations spl06_fops =
{
	.owner = THIS_MODULE,
	.read = spl06_read,
	.open = spl06_open,
	.release = spl06_release,
};

static int spl06_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	printk("==%s==\r\n",__func__);
	if(spl06_i2c_client == NULL && client != NULL){
		client->max_speed = SENSOR_I2C_SPEED;
		spl06_i2c_client = client;
	}

	if(spl06_i2c_client == NULL){
		printk("==%s== failed\r\n",__func__);
	}

	return 0;
}

static int spl06_remove(struct i2c_client *client)
{
	return 0;
}


static const struct i2c_device_id spl06_id[] = {
	{ SENSOR_NAME, 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, spl06_id);

static struct i2c_driver spl06_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name	= SENSOR_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= spl06_probe,
	.remove	= spl06_remove,
	.id_table = spl06_id,
	.detect = spl06_detect,
	.address_list	= normal_i2c,
};

static int __init spl06_init(void)
{
	int ret = -1;
	int result;

	printk("==%s==\r\n",__func__);

	ret = i2c_add_driver(&spl06_driver);
	if (ret < 0) {
		printk(KERN_INFO "add spl06 i2c driver failed\n");
		return -ENODEV;
	}

	result = alloc_chrdev_region(&devno, 0, 1, "spl06");
	if (result < 0)
		return result;

	cdev = cdev_alloc();  
	cdev_init(cdev, &spl06_fops);
	cdev->owner = THIS_MODULE;
	result = cdev_add(cdev,devno,1);

    cls = class_create(THIS_MODULE, "spl06");
    if(IS_ERR(cls)){
		ret = PTR_ERR(cls);
		printk("==%s== class_create failed:%d\r\n",__func__,ret);
		
    }
	
	device = device_create(cls,NULL,devno,NULL,"spl06");
	if(IS_ERR(device)){
		ret = PTR_ERR(device);
		printk("==%s== device_create failed:%d\r\n",__func__,ret);
	}

	printk("==%s== done:%d\r\n",__func__,result);

	return ret;
}

static void __exit spl06_exit(void)
{
	printk(KERN_INFO "remove spl06 i2c driver.\n");
	device_destroy(cls,devno);
	class_destroy(cls);
	cdev_del(cdev);
	unregister_chrdev_region(devno,1);
	i2c_del_driver(&spl06_driver);
}

module_init(spl06_init);
module_exit(spl06_exit);

MODULE_DESCRIPTION("spl06 barometer Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");

