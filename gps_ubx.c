#include "hal_device.h"
#include "hal_sensor.h"
#include "ubx.h"


struct ubx_dev_data_s{
    ubx_decoder_t    ubx_decoder;
    int uart_fd;
};

int ubx_config_gps(struct hal_dev_s *dev,uint32_t buadrate,uint8_t retry)
{
    uint8_t buffer[256];
    int msg_length;
	ubx_buf_t		t_buf;
    
    struct ubx_dev_data_s *ubx_dev = (struct ubx_dev_data_s *)dev->priv_data;
	
    memset(&t_buf.payload_tx_cfg_prt, 0, sizeof(t_buf.payload_tx_cfg_prt));
    t_buf.payload_tx_cfg_prt.portID		= UBX_TX_CFG_PRT_PORTID;
    t_buf.payload_tx_cfg_prt.mode		= UBX_TX_CFG_PRT_MODE;
    t_buf.payload_tx_cfg_prt.baudRate	= 115200;
    t_buf.payload_tx_cfg_prt.inProtoMask	= UBX_TX_CFG_PRT_INPROTOMASK;
    t_buf.payload_tx_cfg_prt.outProtoMask	= UBX_TX_CFG_PRT_OUTPROTOMASK;
    msg_length = ubx_pack_message(buffer,UBX_MSG_CFG_PRT, t_buf.raw, sizeof(t_buf.payload_tx_cfg_prt));
    hal_dev_write(ubx_dev->uart_fd,buffer,msg_length,0);
    /* wait ack to continue */
	
    memset(&t_buf.payload_tx_cfg_msg, 0, sizeof(t_buf.payload_tx_cfg_msg));
	t_buf.payload_tx_cfg_msg.msgClass = UBX_CLASS_NAV;
	t_buf.payload_tx_cfg_msg.msgID    = UBX_ID_NAV_PVT;
	t_buf.payload_tx_cfg_msg.rate     = 1;
	msg_length = ubx_pack_message(buffer,UBX_MSG_CFG_MSG, t_buf.raw, sizeof(t_buf.payload_tx_cfg_msg));
	hal_dev_write(ubx_dev->uart_fd,buffer,msg_length,0);
    /* wait ack to continue */
    
	memset(&t_buf.payload_tx_cfg_nav5, 0, sizeof(t_buf.payload_tx_cfg_nav5));
	t_buf.payload_tx_cfg_nav5.mask		= UBX_TX_CFG_NAV5_MASK;
	t_buf.payload_tx_cfg_nav5.dynModel	= UBX_TX_CFG_NAV5_DYNMODEL;
	t_buf.payload_tx_cfg_nav5.fixMode	= UBX_TX_CFG_NAV5_FIXMODE;
	msg_length = ubx_pack_message(buffer,UBX_MSG_CFG_NAV5, t_buf.raw, sizeof(t_buf.payload_tx_cfg_nav5));
	hal_dev_write(ubx_dev->uart_fd,buffer,msg_length,0);
    /* wait ack to continue */

	memset(&t_buf.payload_tx_cfg_rate, 0, sizeof(t_buf.payload_tx_cfg_rate));
	t_buf.payload_tx_cfg_rate.measRate       = 100;    //100ms for 10Hz
	t_buf.payload_tx_cfg_rate.navRate        = UBX_TX_CFG_RATE_NAVRATE;
	t_buf.payload_tx_cfg_rate.timeRef        = UBX_TX_CFG_RATE_TIMEREF;
    /* wait ack to continue */
	msg_length = ubx_pack_message(buffer,UBX_MSG_CFG_RATE, t_buf.raw, sizeof(t_buf.payload_tx_cfg_rate));
    hal_dev_write(ubx_dev->uart_fd,buffer,msg_length,0);

	return 1;
}

int ubx_init(struct hal_dev_s *dev)
{
    struct ubx_dev_data_s *ubx_dev = (struct ubx_dev_data_s *)dev->priv_data;
    
    if(ubx_dev == NULL){
        return -1;
    }

    ubx_dev->uart_fd = hal_dev_open("ttyS3", HAL_O_RDWR);

	if(ubx_dev->uart_fd < 0){
		//printf("[GPS] Can not open uart %s!\r\n",GPS_UART_PATH);
		return -1;
	}else{
		ubx_decoder_init(&ubx_dev->ubx_decoder);
		return 0;
	}
}

int ubx_open(struct hal_dev_s *dev, uint16_t oflag)
{
    return 0;
}

int ubx_read(struct hal_dev_s *dev, void *buffer, int size,int pos)
{
	int count = 0;
	char buf[256];
	int flag = 0;
	int ret = -1;
	int i;
	struct ubx_dev_data_s *ubx_dev = (struct ubx_dev_data_s *)dev->priv_data;
	struct gps_info_s *report = (struct gps_info_s *)buffer;

    if(size < sizeof(struct gps_info_s)){
        return -1;
    }

    if(ubx_dev == NULL){
        return -1;
    }
    
	count = hal_dev_read(ubx_dev->uart_fd,buf,256,0);
	
	if(count > 0){
		for(i = 0 ;i < count;i++){
			flag =  ubx_parse_char(&ubx_dev->ubx_decoder,buf[i]);
			if(flag == 1){
				ubx_copy(&ubx_dev->ubx_decoder,report);
				ret = 1;
			}
		}
	}
	return ret;
}

struct ubx_dev_data_s ubx_dev_data;
struct hal_dev_s ubx_dev;

int ubx_register(void)
{
    ubx_dev.init  = ubx_init;
    ubx_dev.read  = ubx_read;
    ubx_dev.write = NULL;
    ubx_dev.priv_data = &ubx_dev_data;

    hal_dev_register(&ubx_dev,"gps-ubx",HAL_O_RDWR | HAL_DEV_STANDALONE);
    return 0;
}





