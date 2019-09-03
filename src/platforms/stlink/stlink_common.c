/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2017 Uwe Bonnes (bon@elektron.ikp.physik.tu-darmstadt.de)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/* returns one of the following values to indicate hardware:
 * 0: ST-Link V1, e.g. on VL Discovery
 * 1: ST-Link V2, e.g. on F4 Discovery or ST-Link "baite" clones
 * 2: ST-Link V2.1
 * 255: not recognized (alternatively mapped to a default!)
 *
 * Detection is done reading gpio values.
 *
 * WARNING:
 * Performs hw specific setup
 */

uint32_t detect_rev(void)
{
	uint32_t rev;

	/* Test for HSI Clock, switch to HSI if necessary */
	while (RCC_CFGR & 0xf) {
		RCC_CFGR &= ~3;
	}

	/* Enable all Peripheral Clocks */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_USB);
	rcc_periph_reset_pulse(RST_USB);
	rcc_periph_clock_enable(RCC_AFIO);
	rcc_periph_clock_enable(RCC_CRC);
	AFIO_MAPR |= 0x02000000; /* Release from TDI such that PA15 can be read */

#define READ_WITH_PULLUP(MVAR,MPORT,MPIN) \
	do {\
		gpio_set_mode(MPORT,GPIO_MODE_INPUT,GPIO_CNF_INPUT_PULL_UPDOWN,MPIN);\
		gpio_set(MPORT,MPIN);\
		for (int i=0; i<100; i++) {\
			MVAR = gpio_get(MPORT,MPIN);\
		}\
	} while (0)

#define READ_WITH_PULLDOWN(MVAR,MPORT,MPIN) \
	do {\
		gpio_set_mode(MPORT,GPIO_MODE_INPUT,GPIO_CNF_INPUT_PULL_UPDOWN,MPIN);\
		gpio_clear(MPORT,MPIN);\
		for (int i=0; i<100; i++) {\
			MVAR = gpio_get(MPORT,MPIN);\
		}\
	} while (0)

	/*
		The code has the following pattern:
			read_input_to_variable();
			if (is_floating) { ... }
			else if (is_pulled low) { ... }
			else { ... }
	*/

	/* read PC13 */
	int pc_13_pup, pc_13_pud;
	READ_WITH_PULLUP(pc_13_pup,GPIOC,GPIO13);
	READ_WITH_PULLDOWN(pc_13_pud,GPIOC,GPIO13);

	if (pc_13_pup != pc_13_pud) {
		/* PC13 floating */

		/* check for PB11 which is pulled high by SWIM circutry
			in baite Variant and not connected on VL Discovery */

		/* read PB11 */
		int pb_11_pup, pb_11_pud;
		READ_WITH_PULLUP(pb_11_pup,GPIOB,GPIO11);
		READ_WITH_PULLDOWN(pb_11_pud,GPIOB,GPIO11);

		if (pb_11_pup != pb_11_pud) {
			/* PB11 floating */
			rev = 0; /* ST-Link V1 on VL Discovery */
		} else if (pb_11_pup == 0) {
			/* PB11 not floating and pulled low externally */
			rev = 0xFF; /* unknown variant */
		} else {
			/* PB11 not floating and pulled high externally */
			rev = 1; /* ST-Link V2 "baite" */
		}

	} else if (pc_13_pup == 0) {
		/* PC13 not floating and pulled low externally */

		/* read PC14 */
		int pc_14_pup, pc_14_pud;
		READ_WITH_PULLUP(pc_14_pup,GPIOC,GPIO14);
		READ_WITH_PULLDOWN(pc_14_pud,GPIOC,GPIO14);

		if (pc_14_pup != pc_14_pud) {
			/* PC14 floating */

			/* Check for V2.1 boards. PA15/TDI is USE_RENUM,
				pulled with 10 k to U5V on V2.1, otherwise unconnected. */

			/* read PA15 */
			int pa_15_pup, pa_15_pud;
			READ_WITH_PULLUP(pa_15_pup,GPIOA,GPIO15);
			READ_WITH_PULLDOWN(pa_15_pud,GPIOA,GPIO15);

		 	if (pa_15_pup != pa_15_pud) {
				/* PA15 floating */
				rev = 1; /* ST-Link V2 */
			} else if (pa_15_pup == 0) {
				/* PA15 not floating and pulled low externally */
				rev = 0xFF; /* unknown variant */
			} else {
				/* PA15 not floating and pulled high externally */
				rev = 2; /* ST-Link V2.1 */
			}

		} else if (pc_14_pup == 0) {
			/* PC14 not floating and pulled low externally */
			rev = 1; /* ST-Link V2 on Legacy Discovery F4xx */
		} else {
			/* PC14 not floating and pulled high externally */
			rev = 0xFF; /* unknown variant */
		}

	} else {
		/* PC13 not floating and pulled high externally */
		rev = 0xFF; /* unknown variant */
	}

	#define STLINK_DEFAULT_ON_UNKNOWN (1)
	#ifdef STLINK_DEFAULT_ON_UNKNOWN
	{
		/* check if one of the known variants is selected, otherewise default */
		if (rev == 0xFF) {
			rev = STLINK_DEFAULT_ON_UNKNOWN;
		}
	}
	#endif

	/* apply revision specific configuration */
	/* TODO: is there a more appropriate place for this? */

	if (rev == 0 || rev == 1) {
		/* PA12: open-drain, pulled low (USB-Data-Plus) */
		gpio_clear(GPIOA, GPIO12);
		gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, GPIO12);
	}

	if (rev == 2) {
		/* PB15: Pull PWR_ENn low. */
		gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, GPIO15);
		gpio_clear(GPIOB, GPIO15);
		/* PA15: Pull USB_RENUM low. */
		gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_OPENDRAIN, GPIO15);
		gpio_clear(GPIOA, GPIO15);
	}

	if (rev == 1 || rev == 2) {
		/* On Rev > 0 unconditionally activate MCO on PORTA8 with HSE! */
		RCC_CFGR &= ~(0xf << 24);
		RCC_CFGR |= (RCC_CFGR_MCO_HSE << 24);
		gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO8);
	}

	#define STLINK_DEBUG_HW_DETECTION
	#ifdef STLINK_DEBUG_HW_DETECTION
	{
		/* read all pins anyways */
		int pa_15_pup, pa_15_pud;
		int pb_11_pup, pb_11_pud;
		int pc_14_pup, pc_14_pud;
		int pc_13_pup, pc_13_pud;
		READ_WITH_PULLUP(pa_15_pup,GPIOA,GPIO15);
		READ_WITH_PULLDOWN(pa_15_pud,GPIOA,GPIO15);
		READ_WITH_PULLUP(pb_11_pup,GPIOB,GPIO11);
		READ_WITH_PULLDOWN(pb_11_pud,GPIOB,GPIO11);
		READ_WITH_PULLUP(pc_14_pup,GPIOC,GPIO14);
		READ_WITH_PULLDOWN(pc_14_pud,GPIOC,GPIO14);
		READ_WITH_PULLUP(pc_13_pup,GPIOC,GPIO13);
		READ_WITH_PULLDOWN(pc_13_pud,GPIOC,GPIO13);
		/*
			encode gpio readings into revision
			lower 16 bit are still revision
			upper 16 bit are 1 nibble per pin, interpration is: as follows:
				0x3 -> externally pulled high
				0x2 -> basically impossible (pin inverts internal pull)
				0x1 -> floating
				0x0 -> externally
		*/
		rev = (rev&0xFF)
			| ((pa_15_pup!=0)?(1<<16):0)
			| ((pa_15_pud!=0)?(1<<17):0)
			| ((pb_11_pup!=0)?(1<<20):0)
			| ((pb_11_pud!=0)?(1<<21):0)
			| ((pc_14_pup!=0)?(1<<24):0)
			| ((pc_14_pud!=0)?(1<<25):0)
			| ((pc_13_pup!=0)?(1<<28):0)
			| ((pc_13_pud!=0)?(1<<29):0)
			;
	}
	#endif

	return rev;
}

void platform_request_boot(void)
{
	uint32_t crl = GPIOA_CRL;
	/* Assert bootloader marker.
	 * Enable Pull on GPIOA1. We don't rely on the external pin
	 * really pulled, but only on the value of the CNF register
	 * changed from the reset value
	 */
	crl &= 0xffffff0f;
	crl |= 0x80;
	GPIOA_CRL = crl;
	SCB_VTOR = 0;
}
