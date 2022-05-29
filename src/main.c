
#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <string.h>
#include <math.h>

#include "csd_usb.h"
#include "csd_gpio.h"


#define UPBOT GPIO2
#define DNORG GPIO4
#define LTBRN GPIO6
#define RTYEL GPIO8
#define PSHTP GPIO9

#define SDFWD GPIO7
#define SDAFT GPIO5

#define SIDEGND 14  // ground for two side buttons
#define DIRGND 13  // ground for five-way switch
#define CLRGND 12    // Ground pin for color and top & bottom black buttons

/*
// colors
// gnd 12
// bottom 3
// orange 4
// brown 6
// yellow 8
// top 9

// directional
// gnd 13
// up 3
// down 4
// left 6
// right 8
// push 9

// side
// gnd 14
// fwd 7
// aft 5
*/


bool csdbg[8];
float csflt[8];
int csint[8];

// Output buffer (must match HID report descriptor)
int8_t outbuf[5];

int8_t swbnk[16] = {0};

double mapOneRangeToAnother(double sourceNumber, double fromA, double fromB, double toA, double toB, int decimalPrecision ) {
    double deltaA = fromB - fromA;
    double deltaB = toB - toA;
    double scale  = deltaB / deltaA;
    double negA   = -1 * fromA;
    double offset = (negA * scale) + toA;
    double finalNumber = (sourceNumber * scale) + offset;
    int calcScale = (int) pow(10, decimalPrecision);
    return (double) round(finalNumber * calcScale) / calcScale;
}

void set_ground(int gnd_pin) 
{
	switch(gnd_pin)
	{
		case 12:
			gpio_clear(GPIOB, GPIO12);
			gpio_set(GPIOB, GPIO13 | GPIO14);
		break;
		case 13:
			gpio_clear(GPIOB, GPIO13);
			gpio_set(GPIOB, GPIO12 | GPIO14);
		break;
		case 14:
			gpio_clear(GPIOB, GPIO14);
			gpio_set(GPIOB, GPIO12 | GPIO13);
		break;
		default:
			//gpio_clear(GPIOB, GPIO12);
			gpio_set(GPIOB, GPIO12 | GPIO13 | GPIO14);
		break;
	}
}

int main(void)
{
	// Initialized explicitly for clarity and debugging
	outbuf[0] = 0; // slider wheel
	outbuf[1] = 0; // buttons
	outbuf[2] = 0; // buttons
	outbuf[3] = 0; // switch bank 2
	outbuf[4] = 0; // switch bank 2

	edtc_report.slider1 = 0;
	edtc_report.buttons = 0;

	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	gpio_setup();
	adc_setup();

    /* ADC channels for conversion*/
	uint8_t channel_array[2] = {0};
	/*
	 * This is a somewhat common cheap hack to trigger device re-enumeration
	 * on startup.  Assuming a fixed external pullup on D+, (For USB-FS)
	 * setting the pin to output, and driving it explicitly low effectively
	 * "removes" the pullup.  The subsequent USB init will "take over" the
	 * pin, and it will appear as a proper pullup to the host.
	 * The magic delay is somewhat arbitrary, no guarantees on USBIF
	 * compliance here, but "it works" in most places.
	 */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_clear(GPIOA, GPIO12);
	for (unsigned i = 0; i < 800000; i++) {
		__asm__("nop");
	}

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, hid_set_config);

    // turn off the LED
	gpio_clear(GPIOC, GPIO13);


	while (true)
	{
		channel_array[0] = 0;
		adc_set_regular_sequence(ADC1, 1, channel_array);
		adc_start_conversion_regular(ADC1);
		while (!(adc_eoc(ADC1)))
			continue;
		int16_t tk0 = adc_read_regular(ADC1); 
		int tmp_mix = (int)mapOneRangeToAnother(tk0, 0 , 4095, -127, 127, 0);
		if(tmp_mix > 127) tmp_mix = 127;
		if(tmp_mix < -127) tmp_mix = -127;
		edtc_report.slider1 = tmp_mix;

		// directional switch
		set_ground(13);
		swbnk[0] = !gpio_get(GPIOB, PSHTP); // idx push
		swbnk[1] = !gpio_get(GPIOA, UPBOT); // idx up 
		swbnk[2] = !gpio_get(GPIOB, DNORG); // idx down
		swbnk[3] = !gpio_get(GPIOB, LTBRN); // idx left
		swbnk[4] = !gpio_get(GPIOB, RTYEL); // idx right

	    // chromatic
		set_ground(14);
		swbnk[5] = !gpio_get(GPIOB, PSHTP);  // black top
		swbnk[6] = !gpio_get(GPIOA, UPBOT);  // black bottom
		swbnk[7] = !gpio_get(GPIOB, DNORG);  // orange
		swbnk[8] = !gpio_get(GPIOB, LTBRN);  // brown
		swbnk[9] = !gpio_get(GPIOB, RTYEL);  // yellow

		// side mount
		set_ground(12);
		swbnk[10] = !gpio_get(GPIOB, SDFWD); // side fwd
		swbnk[11] = !gpio_get(GPIOB, SDAFT); // side aft
		swbnk[12] = 0; // !gpio_get(GPIOB, GPIO7);
		swbnk[13] = 0; // !gpio_get(GPIOB, GPIO8);
		swbnk[14] = 0; // !gpio_get(GPIOB, GPIO6);
		swbnk[15] = 0; // !gpio_get(GPIOB, GPIO4);

		edtc_report.buttons = 0;

		for(int i=0;i<16;i++) {
			edtc_report.buttons |= (swbnk[i] << i);
		}

		usbd_poll(usbd_dev);
	}
}

void sys_tick_handler(void)
{
	memcpy(&outbuf[0], &edtc_report.slider1, 1);
	memcpy(&outbuf[1], &edtc_report.buttons, 2);

	usbd_ep_write_packet(usbd_dev, 0x81, &outbuf, 3);
}