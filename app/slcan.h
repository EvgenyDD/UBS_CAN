#ifndef SLCAN_H__
#define SLCAN_H__

#include "can_driver.h"
#include <stdint.h>

const uint8_t *slcan_parse(CAN_TypeDef *dev, const uint8_t *data, uint32_t size);
int slcan_frame2buf(uint8_t buf[32], const can_msg_t *msg);

#endif // SLCAN_H__