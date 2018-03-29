#include <linux/i2c/uav_sensor.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>

#define SENSOR_NAME "mmc5883"
#define SENSOR_I2C_SLAVE_ADDRESS    SENSOR_MMC5883_ADDR

#define MMC5883MA_CHIP_ID_VAL		0x0C

#define MMC5883MA_REG_DATA			0x00
#define MMC5883MA_REG_TEMP			0x06
#define MMC5883MA_REG_STATUS		0x07
#define MMC5883MA_REG_CTRL0			0x08
#define MMC5883MA_REG_CTRL1			0x09
#define MMC5883MA_REG_CTRL2			0x0A
#define MMC5883MA_REG_SELFTEST		0x0E
#define MMC5883MA_REG_PASSWORD		0x0F
#define MMC5883MA_REG_OTPMODE		0x12
#define MMC5883MA_REG_TESTMODE		0x13
#define MMC5883MA_REG_OTP			0x2A
#define MMC5883MA_REG_PRODUCTID		0x2F
 
#define MMC5883MA_CMD_REFILL		0x20
#define MMC5883MA_CMD_RESET         0x10
#define MMC5883MA_CMD_SET			0x08
#define MMC5883MA_CMD_TM_M			0x01
#define MMC5883MA_CMD_TM_T			0x02
#define MMC5883MA_CMD_PASSWORD		0xE1
#define MMC5883MA_CMD_OTP_OPER		0x11
#define MMC5883MA_CMD_OTP_MR		0x80
#define MMC5883MA_CMD_OTP_ACT		0x80

#define MMC5883MA_CMD_100HZ			0x00
#define MMC5883MA_CMD_200HZ			0x01
#define MMC5883MA_CMD_400HZ			0x02
#define MMC5883MA_CMD_600HZ			0x03

// 16-bit mode, null field output (32768)
#define MMC5883MA_OFFSET			32768
#define MMC5883MA_SENSITIVITY		4096
#define MMC5883MA_T_OFFSET			128
#define MMC5883MA_READ_RATE         100


uint8_t OtpMatrix_Reg[2];
uint16_t set_timer = 0;



static struct i2c_client *mmc5883_i2c_client = NULL;

struct mmc5883_report_s{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t otp_reg[2];
};

static const unsigned short normal_i2c[2] = {SENSOR_I2C_SLAVE_ADDRESS, I2C_CLIENT_END};

static dev_t devno;
static struct cdev * cdev; 
static struct device *device;
static struct class *cls;

static int mmc5883_i2c_test(struct i2c_client * client)
{
	unsigned char id;

	if(i2c_smbus_read_i2c_block_data(client,MMC5883MA_REG_PRODUCTID,1,&id) < 0){
		return -1;
	}
	if(id != MMC5883MA_CHIP_ID_VAL){
		return -1;
	}

	return 0;
}

static int mmc5883_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	pr_info("%s: addr=0x%x\n",__func__,client->addr);
	strlcpy(info->type, SENSOR_NAME, I2C_NAME_SIZE);

	//if(mmc5883_i2c_test(client) != 0){
	//	return -1;
	//}
	
	return 0;
}

/*********************************************************************************
* decription: get the factory compensation parameters
*********************************************************************************/
int MEMSIC_Comp_Matrix(struct i2c_client *client)
{
	int result = 1;
	
	/* write the password and set the otp register read mode */
	i2c_smbus_write_byte_data(client,MMC5883MA_REG_PASSWORD,MMC5883MA_CMD_PASSWORD);
	i2c_smbus_write_byte_data(client,MMC5883MA_REG_OTPMODE,MMC5883MA_CMD_OTP_OPER);
	i2c_smbus_write_byte_data(client,MMC5883MA_REG_TESTMODE,MMC5883MA_CMD_OTP_MR);
	i2c_smbus_write_byte_data(client,MMC5883MA_REG_CTRL2,MMC5883MA_CMD_OTP_ACT);
	
	/* read 2 bytes data from MMC5883MA sensor registers 0x2A~0x2B */
	if(i2c_smbus_read_i2c_block_data(client,MMC5883MA_REG_OTP,2,OtpMatrix_Reg) < 0){
		return 0;
	}
	
	return result;	
}


static int mmc5883_init_client(struct i2c_client *client)
{
	uint8_t data = 0;

	i2c_smbus_write_byte_data(client,MMC5883MA_REG_CTRL0,MMC5883MA_CMD_SET);
	mdelay(2);
	i2c_smbus_write_byte_data(client,MMC5883MA_REG_CTRL1,MMC5883MA_CMD_100HZ);
	mdelay(2);
	i2c_smbus_write_byte_data(client,MMC5883MA_REG_CTRL0,MMC5883MA_CMD_TM_M);
	mdelay(10);
	
	if(i2c_smbus_read_i2c_block_data(client,MMC5883MA_REG_PRODUCTID,1,&data) < 0){
		printk("%s can not read client\n",__func__);
		return -1;
	}
	if(data != MMC5883MA_CHIP_ID_VAL){
		printk("%s unvalid id:%d\n",__func__,data);
		return -1;
	}
		
	printk("==%s==\r\n",__func__);

	MEMSIC_Comp_Matrix(client);

	return 0;
}

static int mmc5883_measure(struct i2c_client *client,struct mmc5883_report_s *pf)
{
	int ret = -1;
	char buf[7];
	int i;
	uint16_t data_temp[3] = {0};
	
	ret = i2c_smbus_read_i2c_block_data(client,MMC5883MA_REG_DATA,7,buf);
	if(ret < 0){
		printk("MMC5883MA_REG_DATA failed \r\n");
		return -1;
	}else{
		/* the output raw data unit is "count or LSB" */
		for(i = 0; i < 3; i++) { 
			data_temp[i]=(uint16_t)(buf[2*i+1]<< 8 | buf[2*i]);
		}
		
		pf->x = (int16_t)(data_temp[0] - MMC5883MA_OFFSET);
		pf->y = (int16_t)(data_temp[1] - MMC5883MA_OFFSET); 
		pf->z = (int16_t)(data_temp[2] - MMC5883MA_OFFSET);
		pf->otp_reg[0] = OtpMatrix_Reg[0];
		pf->otp_reg[1] = OtpMatrix_Reg[1];
	}
	ret = i2c_smbus_write_byte_data(client, MMC5883MA_REG_CTRL0, MMC5883MA_CMD_TM_M);
	if(ret < 0){
		printk("MMC5883MA_REG_CTRL0 failed \r\n");
	}
	set_timer ++;
	if(set_timer > MMC5883MA_READ_RATE * 3){
		set_timer = 0;
		ret = i2c_smbus_write_byte_data(client, MMC5883MA_REG_CTRL0, MMC5883MA_CMD_SET);
		if(ret < 0){
			printk("MMC5883MA_REG_CTRL0 failed \r\n");
		}
	}
	return 0;
}

int mmc5883_open(struct inode *inode, struct file *filp)
{
	
	if(mmc5883_i2c_test(mmc5883_i2c_client) < 0){
		return -1;
	}
	
	if(mmc5883_init_client(mmc5883_i2c_client) < 0){
		return -1;
	}

	i2c_smbus_write_byte_data(mmc5883_i2c_client, MMC5883MA_REG_CTRL0, MMC5883MA_CMD_TM_M);
	mdelay(10);
	return 0; 
}

int mmc5883_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t mmc5883_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
	struct mmc5883_report_s data;
	int ret;
	
	ret = mmc5883_measure(mmc5883_i2c_client,&data);
	if(ret < 0){
		return -1;
	}
	
	copy_to_user(buf,(void *)&data,sizeof(data));
	
	return 1;
}

static const struct file_operations mmc5883_fops =
{
	.owner = THIS_MODULE,
	.read = mmc5883_read,
	.open = mmc5883_open,
	.release = mmc5883_release,
};

static int mmc5883_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	if(mmc5883_i2c_client == NULL){
		mmc5883_i2c_client = client;
	}

	if(mmc5883_i2c_client == NULL){
		printk("==%s== failed\r\n",__func__);
	}

	return 0;
}

static int mmc5883_remove(struct i2c_client *client)
{
	return 0;
}


static const struct i2c_device_id mmc5883_id[] = {
	{ SENSOR_NAME, 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mmc5883_id);

static struct i2c_driver mmc5883_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name	= SENSOR_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= mmc5883_probe,
	.remove	= mmc5883_remove,
	.id_table = mmc5883_id,
	.detect = mmc5883_detect,
	.address_list	= normal_i2c,
};

static int __init mmc5883_init(void)
{
	int ret = -1;
	int result;

	ret = i2c_add_driver(&mmc5883_driver);
	if (ret < 0) {
		printk(KERN_INFO "add mmc5883 i2c driver failed\n");
		return -ENODEV;
	}

	result = alloc_chrdev_region(&devno, 0, 1, "mmc5883");
	if (result < 0)
		return result;

	cdev = cdev_alloc();  
	cdev_init(cdev, &mmc5883_fops);
	cdev->owner = THIS_MODULE;
	result = cdev_add(cdev,devno,1);

    cls = class_create(THIS_MODULE, "mmc5883");
    if(IS_ERR(cls)){
		ret = PTR_ERR(cls);
		printk("==%s== class_create failed:%d\r\n",__func__,ret);
		
    }
	
	device = device_create(cls,NULL,devno,NULL,"mmc5883");
	if(IS_ERR(device)){
		ret = PTR_ERR(device);
		printk("==%s== device_create failed:%d\r\n",__func__,ret);
	}

	printk("==%s== done:%d\r\n",__func__,result);

	return ret;
}

static void __exit mmc5883_exit(void)
{
	printk(KERN_INFO "remove mmc5883 i2c driver.\n");
	device_destroy(cls,devno);
	class_destroy(cls);
	cdev_del(cdev);
	unregister_chrdev_region(devno,1);
	i2c_del_driver(&mmc5883_driver);
}

module_init(mmc5883_init);
module_exit(mmc5883_exit);

MODULE_DESCRIPTION("mmc5883 mag Sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1");

