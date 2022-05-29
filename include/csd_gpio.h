#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>



static void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOA); 
	rcc_periph_clock_enable(RCC_GPIOB); 
	rcc_periph_clock_enable(RCC_GPIOC);
    // Initialize LED
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    gpio_set(GPIOC, GPIO13);
	// turn off LED until in active mode
	//gpio_clear(GPIOC, GPIO13);

    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 | GPIO8 | GPIO9 );
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12 | GPIO13 | GPIO14);
	gpio_set(GPIOB, GPIO3 | GPIO4 | GPIO5 | GPIO6 | GPIO7 | GPIO8 | GPIO9 );

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO2 );
    //gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8 | GPIO9);
	gpio_set(GPIOA, GPIO2);

    // Configure A1 & A2 as analog inputs
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);
    //gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO2);
}

static void adc_setup(void)
{
	int i;
	rcc_periph_clock_enable(RCC_ADC1);
	/*****    ADC1   *********/

	/* Make sure the ADC doesn't run during config. */
	adc_power_off(ADC1);

	adc_disable_scan_mode(ADC1);
	//adc_enable_scan_mode(ADC1);
	//adc_set_continuous_conversion_mode(ADC1);
	//adc_enable_discontinuous_mode_regular(ADC1, 2);
	adc_set_single_conversion_mode(ADC1);
	adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);
	adc_set_right_aligned(ADC1);
	
	adc_disable_temperature_sensor();

	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_55DOT5CYC);

	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	for (i = 0; i < 800000; i++)    /* Wait a bit. */
		__asm__("nop");

	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);
}