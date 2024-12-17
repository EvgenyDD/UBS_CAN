#ifndef PLATFORM_H
#define PLATFORM_H

#include "stm32f4xx.h"
#include <stdbool.h>
#include <stdint.h>

#define _BV(x) (1ULL << (x))

#define PAGE_SIZE (2 * 1024)

#define FLASH_LEN (0x00040000U) // 256kB
#define FLASH_START FLASH_BASE
#define FLASH_ORIGIN FLASH_BASE
#define FLASH_FINISH (FLASH_BASE + FLASH_LEN)
#define FLASH_SIZE FLASH_LEN

#define PIN_SET_(x, y) (x)->BSRR = (y)
#define PIN_RST_(x, y) (x)->BSRR = ((uint32_t)(y)) << 16
#define PIN_WR_(x, y, v) (x)->BSRR = ((uint32_t)(y)) << ((!(v)) * 16)
#define PIN_GET_(x, y) !!((x)->IDR & (y))
#define PIN_GET_ODR_(x, y) !!((x)->ODR & (y))

#define PIN_SET(x) (x##_Port->BSRRL = x##_Pin)
#define PIN_RST(x) (x##_Port->BSRRH = x##_Pin)
#define PIN_WR(x, v) (v) ? PIN_SET(x) : PIN_RST(x)
#define PIN_GET(x) !!(x##_Port->IDR & x##_Pin)
#define PIN_GET_ODR(x) !!(x##_Port->ODR & x##_Pin)

#define LED_TX_Port GPIOA
#define LED_TX_Pin GPIO_Pin_0
#define LED_RX_Port GPIOA
#define LED_RX_Pin GPIO_Pin_1
#define LED_ERR_Port GPIOA
#define LED_ERR_Pin GPIO_Pin_2

#define UNIQUE_ID 0x1FFF7A10

void platform_flash_erase_flag_reset(void);
void platform_flash_erase_flag_reset_sect_cfg(void);

void platform_flash_unlock(void);
void platform_flash_lock(void);
int platform_flash_read(uint32_t addr, uint8_t *src, uint32_t sz);
int platform_flash_write(uint32_t dest, const uint8_t *src, uint32_t sz);

void platform_init(void);
void platform_deinit(void);
void platform_reset(void);
void platform_run_address(uint32_t address);

void platform_get_uid(uint32_t *id);

void delay_ms(volatile uint32_t delay_ms);

void platform_watchdog_init(void);
static inline void platform_watchdog_reset(void) { IWDG_ReloadCounter(); }

const char *platform_reset_cause_get(void);

extern int __preldr_start, __preldr_end;
extern int __ldr_start, __ldr_end;
extern int __cfg_start, __cfg_end;
extern int __app_start, __app_end;
extern int __header_offset;

extern uint32_t g_uid[3];
extern volatile uint32_t system_time_ms;

#endif // PLATFORM_H
