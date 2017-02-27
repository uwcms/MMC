/*
 * gpio_init.h
 *
 *  Created on: Oct 5, 2010
 *      Author: tgorski
 */

#ifndef GPIO_INIT_H_
#define GPIO_INIT_H_

#define GPIO	AVR32_GPIO

void gpio_scanslv_mode(void);

void gpio_spi1_mode(void);

void gpio_spi1_bitbang(void);

inline int gpio_get_spi1_mode(void);

void gpio_init();

void gpio_set_gpio_pin(unsigned int pin);

void gpio_clr_gpio_pin(unsigned int pin);

void gpio_tgl_gpio_pin(unsigned int pin);

int gpio_get_pin_value(unsigned int pin);

void gpio_disable_gpio_pin_output(unsigned int pin);

int gpio_get_pin_oe_setting(unsigned int pin);

#endif /* GPIO_INIT_H_ */
