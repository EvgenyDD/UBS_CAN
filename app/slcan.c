#include "slcan.h"
#include "fw_header.h"
#include "platform.h"
#include "usbd_core_cdc.h"
#include <stdbool.h>
#include <string.h>

#define RET_ACK "\a"
#define RET_NACK "\r"

#define SLCAN_BUFFER_SIZE 200
static uint8_t buf_[SLCAN_BUFFER_SIZE + 1];
static int32_t pos_ = 0;

static struct
{
	bool silent;
	bool loopback;
	bool timestamp;
} cfg = {false, false, true};

static uint8_t ret_buf[128];

static uint8_t nibble2hex(uint8_t x) { return (x & 0x0F) > 9 ? (x & 0x0F) - 10 + 'A' : (x & 0x0F) + '0'; }
static uint8_t hex2nibble(char c, bool *e)
{
	if(c >= '0' && c <= '9') return c - '0';
	if(c >= 'a' && c <= 'f') return c - 'a' + 10;
	if(c >= 'A' && c <= 'F') return c - 'A' + 10;
	*e = true;
	return 0;
}

static int cb_frm_data_ext(CAN_TypeDef *dev, const char *cmd, uint32_t size)
{
	can_msg_t msg;
	bool e = false;
	msg.id.ext = (hex2nibble(cmd[1], &e) << 28) | (hex2nibble(cmd[2], &e) << 24) | (hex2nibble(cmd[3], &e) << 20) | (hex2nibble(cmd[4], &e) << 16) |
				 (hex2nibble(cmd[5], &e) << 12) | (hex2nibble(cmd[6], &e) << 8) | (hex2nibble(cmd[7], &e) << 4) | (hex2nibble(cmd[8], &e) << 0);
	msg.DLC = hex2nibble(cmd[9], &e);
	if(e || msg.DLC > 8) return -1;
	if(size < 10U + 2 * msg.DLC) return -2;

	const char *p = &cmd[10];
	for(unsigned i = 0; i < msg.DLC; i++)
	{
		msg.data[i] = (hex2nibble(*p, &e) << 4) | hex2nibble(*(p + 1), &e);
		p += 2;
	}

	if(e) return -3;
	return can_drv_tx_ex(dev, msg.id.ext, msg.DLC, msg.data, true, false);
}

static int cb_frm_data_std(CAN_TypeDef *dev, const char *cmd, uint32_t size)
{
	can_msg_t msg;
	bool e = false;
	msg.id.std = (hex2nibble(cmd[1], &e) << 8) | (hex2nibble(cmd[2], &e) << 4) | (hex2nibble(cmd[3], &e) << 0);
	if(cmd[4] < '0' || cmd[4] > '0' + 8) return -1;
	msg.DLC = cmd[4] - '0';
	if(msg.DLC > 8) return -2;
	if(size < 5U + 2 * msg.DLC) return -3;

	const char *p = &cmd[5];
	for(unsigned i = 0; i < msg.DLC; i++)
	{
		msg.data[i] = (hex2nibble(*p, &e) << 4) | hex2nibble(*(p + 1), &e);
		p += 2;
	}

	if(e) return -4;
	return can_drv_tx_ex(dev, msg.id.std, msg.DLC, msg.data, false, false);
}

static int cb_frm_rtr_ext(CAN_TypeDef *dev, const char *cmd, uint32_t size)
{
	can_msg_t msg;
	bool e = false;
	msg.id.ext = (hex2nibble(cmd[1], &e) << 28) | (hex2nibble(cmd[2], &e) << 24) | (hex2nibble(cmd[3], &e) << 20) | (hex2nibble(cmd[4], &e) << 16) |
				 (hex2nibble(cmd[5], &e) << 12) | (hex2nibble(cmd[6], &e) << 8) | (hex2nibble(cmd[7], &e) << 4) | (hex2nibble(cmd[8], &e) << 0);
	if(cmd[9] < '0' || cmd[9] > '0' + 8) return -1;
	msg.DLC = cmd[9] - '0';
	if(msg.DLC > 8) return -2;
	if(size < 10) return -3;
	if(e) return -4;
	return can_drv_tx_ex(dev, msg.id.ext, msg.DLC, msg.data, true, true);
}

static int cb_frm_rtr_std(CAN_TypeDef *dev, const char *cmd, uint32_t size)
{
	can_msg_t msg;
	bool e = false;
	msg.id.std = (hex2nibble(cmd[1], &e) << 8) | (hex2nibble(cmd[2], &e) << 4) | (hex2nibble(cmd[3], &e) << 0);
	if(cmd[4] < '0' || cmd[4] > '0' + 8) return -1;
	msg.DLC = cmd[4] - '0';
	if(msg.DLC > 8) return -2;
	if(size < 5) return -3;
	if(e) return -3;
	return can_drv_tx_ex(dev, msg.id.std, msg.DLC, msg.data, false, true);
}

static const char *process(CAN_TypeDef *dev, const uint8_t *data, uint32_t size)
{
	switch(data[0])
	{
	case 'T': return cb_frm_data_ext(dev, &data[0], size) ? RET_NACK : RET_ACK;
	case 't': return cb_frm_data_std(dev, &data[0], size) ? RET_NACK : RET_ACK;
	case 'R': return cb_frm_rtr_ext(dev, &data[0], size) ? RET_NACK : RET_ACK;
	case 'r': return cb_frm_rtr_std(dev, &data[0], size) ? RET_NACK : RET_ACK;

	case 's': // Set CAN bitrate
	case 'S':
	{
		if(size != 2) return NULL;
		bool e = false;
		uint8_t spd = hex2nibble(data[1], &e);
		if(spd > 8 || e) return RET_NACK;
		int32_t br = (uint32_t[]){10, 20, 50, 100, 125, 250, 500, 800, 1000}[spd] * 1000;
		can_drv_enter_init_mode(dev);
		int match = can_drv_check_set_bitrate(dev, br, true) == br;
		can_drv_leave_init_mode(dev, cfg.silent, cfg.loopback);
		return match ? RET_ACK : RET_NACK;
	}
	break;

	case 'O': // Open CAN in normal mode
		cfg.loopback = cfg.silent = 0;
		can_drv_enter_init_mode(dev);
		can_drv_leave_init_mode(dev, cfg.silent, cfg.loopback);
		return RET_ACK;

	case 'C': // Close CAN
		cfg.loopback = cfg.silent = 0;
		can_drv_enter_init_mode(dev);
		return RET_ACK;

	case 'L': // Open CAN in silent mode
		cfg.silent = 1;
		can_drv_enter_init_mode(dev);
		can_drv_leave_init_mode(dev, cfg.silent, cfg.loopback);
		return RET_ACK;

	case 'l': // Open CAN with loopback enabled
		cfg.loopback = 1;
		can_drv_enter_init_mode(dev);
		can_drv_leave_init_mode(dev, cfg.silent, cfg.loopback);
		return RET_ACK;

	case 'Z': // Enable/disable RX and loopback timestamping
		if(size != 2) return NULL;
		bool e = false;
		uint8_t v = hex2nibble(data[1], &e);
		if(v > 1 || e) return RET_NACK;
		cfg.timestamp = v;
		return RET_ACK;

	case 'F': // Get status flags
	{
		uint32_t r = dev->ESR;
		uint8_t *p = ret_buf;
		for(uint32_t j = 0; j < 8; j++)
			*p++ = nibble2hex((r >> (4 * (8 - j))) & 0xF);
		*p++ = '\r';
		*p++ = 0;
		return ret_buf;
	}

	case 'V':
	{
		uint8_t *p = ret_buf;
		for(uint32_t j = 0; j < 4; j++)
			*p++ = nibble2hex((g_fw_info[FW_APP].ver_major >> (4 * (3 - j))) & 0xF);
		*p++ = '.';
		for(uint32_t j = 0; j < 4; j++)
			*p++ = nibble2hex((g_fw_info[FW_APP].ver_minor >> (4 * (3 - j))) & 0xF);
		*p++ = '.';
		for(uint32_t j = 0; j < 4; j++)
			*p++ = nibble2hex((g_fw_info[FW_APP].ver_patch >> (4 * (3 - j))) & 0xF);
		*p++ = '\a';
		return ret_buf;
	}

	case 'N':
	{
		uint8_t *p = ret_buf;
		for(uint32_t i = 0; i < 3; i++)
		{
			for(uint32_t j = 0; j < 8; j++)
				*p++ = nibble2hex((g_uid[i] >> (4 * (7 - j))) & 0xF);
			if(i < 2) *p++ = '.';
		}
		*p++ = '\a';
		return ret_buf;
	}

	case 'M': // Set CAN acceptance filter ID
	case 'm': // Set CAN acceptance filter mask
	case 'D': // Tx CAN FD
		return RET_NACK;

	default: return 0;
	}
}

const char *slcan_parse(CAN_TypeDef *dev, const uint8_t *data, uint32_t size)
{
	if(size == 0) return NULL;
	const char *resp = NULL;

	for(uint32_t i = 0; i < size; i++)
	{
		if(data[i] >= 32 && data[i] <= 126)
		{
			if(pos_ < SLCAN_BUFFER_SIZE)
			{
				buf_[pos_] = data[i];
				pos_++;
			}
			else
			{
				pos_ = 0; // overrun; silently drop the data
			}
		}
		else if(data[i] == '\r') // End of command (SLCAN)
		{
			buf_[pos_] = '\0';
			resp = process(dev, buf_, pos_);
			pos_ = 0;
		}
		else if(data[i] == 8 || data[i] == 127) // DEL or BS
		{
			if(pos_ > 0) pos_ -= 1;
		}
		else
		{
			pos_ = 0; // invalid byte - drop the current command, this also includes Ctrl+C, Ctrl+D
		}
	}

	return resp;
}

int slcan_frame2buf(uint8_t buf[32], const can_msg_t *msg)
{
	uint8_t *p = buf;
	if(msg->RTR)
	{
		*p++ = msg->IDE ? 'R' : 'r';
	}
	else
	{
		*p++ = msg->IDE ? 'T' : 't';
	}

	if(msg->IDE)
	{
		*p++ = nibble2hex(msg->id.ext >> 28);
		*p++ = nibble2hex(msg->id.ext >> 24);
		*p++ = nibble2hex(msg->id.ext >> 20);
		*p++ = nibble2hex(msg->id.ext >> 16);
		*p++ = nibble2hex(msg->id.ext >> 12);
		*p++ = nibble2hex(msg->id.ext >> 8);
		*p++ = nibble2hex(msg->id.ext >> 4);
		*p++ = nibble2hex(msg->id.ext >> 0);
	}
	else
	{
		*p++ = nibble2hex(msg->id.std >> 8);
		*p++ = nibble2hex(msg->id.std >> 4);
		*p++ = nibble2hex(msg->id.std >> 0);
	}
	*p++ = nibble2hex(msg->DLC);
	for(uint32_t i = 0; i < msg->DLC; i++)
	{
		const uint8_t byte = msg->data[i];
		*p++ = nibble2hex(byte >> 4);
		*p++ = nibble2hex(byte);
	}

	if(cfg.timestamp)
	{
		*p++ = nibble2hex(msg->ts >> 12);
		*p++ = nibble2hex(msg->ts >> 8);
		*p++ = nibble2hex(msg->ts >> 4);
		*p++ = nibble2hex(msg->ts >> 0);
	}
	*p++ = '\r';

	return p - buf;
}
