#include <stdlib.h>
#include <stdio.h> 
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "ubx.h"


void ubx_decoder_init(ubx_decoder_t *dec)
{
	dec->_decode_state = UBX_DECODE_SYNC1;
	dec->_rx_ck_a = 0;
	dec->_rx_ck_b = 0;
	dec->_rx_payload_length = 0;
	dec->_rx_payload_index = 0;
}

void ubx_add_byte_to_checksum(ubx_decoder_t *dec,const uint8_t b)
{
	dec->_rx_ck_a = dec->_rx_ck_a + b;
	dec->_rx_ck_b = dec->_rx_ck_b + dec->_rx_ck_a;
}

/**
   * Add payload rx byte
    */
// -1 = error, 0 = ok, 1 = payload completed
int	payload_rx_add(ubx_decoder_t *dec,const uint8_t b)
{
	int ret = 0;
	dec->_buf.raw[dec->_rx_payload_index] = b;
	if (++(dec->_rx_payload_index) >= dec->_rx_payload_length) {
		ret = 1;	// payload received completely
	}
	if(dec->_rx_payload_index > 200){
		return -1;
	}
	return ret;
}

int	payload_rx_done(ubx_decoder_t *dec)
{
	int ret = 0;
	switch(dec->_rx_msg){

		case UBX_MSG_NAV_PVT:
			dec->_gps_info.fix_type = dec->_buf.payload_rx_nav_pvt.fixType;				
			dec->_gps_info.satellites_used = dec->_buf.payload_rx_nav_pvt.numSV;			
			dec->_gps_info.lat = (double)(dec->_buf.payload_rx_nav_pvt.lat) * (double)1.0e-7;
			dec->_gps_info.lon = (double)(dec->_buf.payload_rx_nav_pvt.lon) * (double)1.0e-7;
			dec->_gps_info.alt = (float)(dec->_buf.payload_rx_nav_pvt.hMSL* 1e-3f);
			dec->_gps_info.eph = (float)(dec->_buf.payload_rx_nav_pvt.hAcc * 1e-3f);
			dec->_gps_info.epv = (float)(dec->_buf.payload_rx_nav_pvt.vAcc * 1e-3f);
			dec->_gps_info.s_variance_m_s	= (float)(dec->_buf.payload_rx_nav_pvt.sAcc * 1e-3f);
			
			dec->_gps_info.vel_m_s = (float)(dec->_buf.payload_rx_nav_pvt.gSpeed * 1e-3f);
			
			dec->_gps_info.vel_n_m_s = (float)(dec->_buf.payload_rx_nav_pvt.velN * 1e-3f);
			dec->_gps_info.vel_e_m_s = (float)(dec->_buf.payload_rx_nav_pvt.velE * 1e-3f);
			dec->_gps_info.vel_d_m_s = (float)(dec->_buf.payload_rx_nav_pvt.velD * 1e-3f);
			
			dec->_gps_info.cog_rad = (float)(dec->_buf.payload_rx_nav_pvt.headMot * M_DEG_TO_RAD_F * 1e-5f);
			dec->_gps_info.c_variance_rad	= (float)(dec->_buf.payload_rx_nav_pvt.headAcc * M_DEG_TO_RAD_F * 1e-5f);
			if(dec->_gps_info.fix_type == 3){
				dec->_gps_info.vel_ned_valid = true;
			}else{
				dec->_gps_info.vel_ned_valid = false;	
			}
            
            dec->_module_existed = true;
            
			ret = true;
			break;
		case UBX_MSG_NAV_POSLLH:	
			break;
		case UBX_MSG_NAV_SOL:
			break;
		case UBX_MSG_NAV_VELNED:
			break;
		case UBX_MSG_ACK_ACK:
            if (dec->_ack_status == UBX_ACK_WAITING)  {
                    dec->_ack_status = UBX_ACK_GOT_ACK;
            }
            ret = true;
            break;

        case UBX_MSG_ACK_NAK:
            if (dec->_ack_status == UBX_ACK_WAITING) {
                    dec->_ack_status = UBX_ACK_GOT_NAK;
            }
            ret = true;
            break;
		default:
			break;
	}
	return ret;
}


int ubx_parse_char(ubx_decoder_t *dec,const uint8_t b)
{
	int ret = 0;
	switch(dec->_decode_state){
		case UBX_DECODE_SYNC1:
			if (b == UBX_SYNC1) {	// Sync1 found --> expecting Sync2
				dec->_decode_state = UBX_DECODE_SYNC2;
			}
			break;
		case UBX_DECODE_SYNC2:
			if (b == UBX_SYNC2) {	// Sync2 ;found --> expecting Class
				dec->_decode_state = UBX_DECODE_CLASS;
                dec->_rx_ck_a = 0;
                dec->_rx_ck_b = 0;
                dec->_rx_payload_length = 0;
                dec->_rx_payload_index = 0;
            }else{
				dec->_decode_state = UBX_DECODE_SYNC1;
			}
			break;
		case UBX_DECODE_CLASS:
			ubx_add_byte_to_checksum(dec,b);  // checksum is calculated for everything except Sync and Checksum bytes
			dec->_rx_msg = b;
			dec->_decode_state = UBX_DECODE_ID;
			break;
			/* Expecting ID */
		case UBX_DECODE_ID:
			ubx_add_byte_to_checksum(dec,b);
			dec->_rx_msg |= b << 8;
			dec->_decode_state = UBX_DECODE_LENGTH1;
			break;

			/* Expecting first length byte */
		case UBX_DECODE_LENGTH1:
			ubx_add_byte_to_checksum(dec,b);
			dec->_rx_payload_length = b;
			dec->_decode_state = UBX_DECODE_LENGTH2;
			break;
			/* Expecting second length byte */
		case UBX_DECODE_LENGTH2:
			ubx_add_byte_to_checksum(dec,b);
			dec->_rx_payload_length |= b << 8;	// calculate payload size
			dec->_decode_state = (dec->_rx_payload_length > 0) ? UBX_DECODE_PAYLOAD : UBX_DECODE_CHKSUM1;
			break;
			/* Expecting payload */
		case UBX_DECODE_PAYLOAD:
			ubx_add_byte_to_checksum(dec,b);
			switch (dec->_rx_msg) {
				default:
					ret = payload_rx_add(dec,b);		// add a payload byte
					break;
			}
			if (ret < 0) {
				// payload not handled, discard message
				dec->_decode_state = UBX_DECODE_SYNC1;
			} else if (ret > 0) {
				dec->_decode_state = UBX_DECODE_CHKSUM1;
			} else {
				// expecting more payload, stay in state UBX_DECODE_PAYLOAD
			}
			ret = 0;
			break;
		case UBX_DECODE_CHKSUM1:
			if (dec->_rx_ck_a != b) {
				dec->_decode_state = UBX_DECODE_SYNC1;
			} else {
				dec->_decode_state = UBX_DECODE_CHKSUM2;
			}
			break;

			/* Expecting second checksum byte */
		case UBX_DECODE_CHKSUM2:
			if (dec->_rx_ck_b == b) {
				ret = payload_rx_done(dec);
			}
            dec->_decode_state = UBX_DECODE_SYNC1;
			
			break;
		default:
			dec->_decode_state = UBX_DECODE_SYNC1;
			break;
	}
	return ret;
}

void ubx_calc_checksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum)
{
	uint16_t i;

	for ( i = 0; i < length; i++) {
		checksum->ck_a = checksum->ck_a + buffer[i];
		checksum->ck_b = checksum->ck_b + checksum->ck_a;
	}
}

int ubx_pack_message(uint8_t *buf,const uint16_t msg, const uint8_t *payload, const uint16_t length)
{
    int i;
    int buf_ptr;
    uint8_t      *ptr;
	ubx_header_t   header = {UBX_SYNC1, UBX_SYNC2};
	ubx_checksum_t checksum = {0, 0};

	// Populate header
	header.msg	= msg;
	header.length	= length;

	// Calculate checksum
	ubx_calc_checksum(((uint8_t *)&header) + 2, sizeof(header) - 2, &checksum); // skip 2 sync bytes

	if (payload != NULL) {
		ubx_calc_checksum(payload, length, &checksum);
	}
       
    buf_ptr = 0;
    ptr = (uint8_t *)&header;
    for(i = 0;i < sizeof(header);i++){
        buf[buf_ptr++] = ptr[i];
    }

	if (payload != NULL) {
        for(i = 0;i < length;i++){
            buf[buf_ptr++] = payload[i];
        }
	}
    
    ptr = (uint8_t *)&checksum;
    for(i = 0;i < sizeof(checksum);i++){
        buf[buf_ptr++] = payload[i];
    }
    return buf_ptr;
}

ubx_ack_state_t ubx_get_ack_state(ubx_decoder_t *dec)
{
	return dec->_ack_status;
}

void ubx_set_ack_state(ubx_decoder_t *dec,ubx_ack_state_t status)
{
	 dec->_ack_status = status;
}

void ubx_copy(ubx_decoder_t *dec,gps_info_s *dat)
{
	memcpy(dat,&(dec->_gps_info),sizeof(gps_info_s));
}


