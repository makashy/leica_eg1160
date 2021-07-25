/*
 * leica_eg1160 Software.cpp
 *
 * Created: 2021-06-19 12:47:55 PM
 * Author : mamin
 */ 

#include "asf.h"
#include <stdio.h>
#include <math.h>
#include "libs/temp_sensor.h"

#define ADC_MOLD_WARMER ADC_CH2
#define ADC_REFRIGERATOR ADC_CH0
float adc_offset = 0;
float adc_gain = 0;


void adc_calibration (void)
{
	float sum_for_gain = 0, sum_for_offset = 0;
	uint8_t count = 100;
	
	for (int i = 0; i < count ; i ++)
	{
		delay_ms(2); //# Time needed for good result (tested)
		adc_start_conversion(&ADCA, ADC_CH0);
		adc_wait_for_interrupt_flag(&ADCA, ADC_CH0);
		sum_for_offset           += adc_get_unsigned_result(&ADCA, ADC_CH0)/float(count);
		
		adc_start_conversion(&ADCA, ADC_CH1);
		adc_wait_for_interrupt_flag(&ADCA, ADC_CH1);
		sum_for_gain             += adc_get_unsigned_result(&ADCA, ADC_CH1)/float(count);
		
		adc_clear_interrupt_flag(&ADCA, ADC_CH0 | ADC_CH1 | ADC_CH2 | ADC_CH3);
	}
	adc_offset = sum_for_offset;
	// ratio between R2 and R1 is 0.49954 and measured by multimeter on ADCCH_POS_PIN4
	adc_gain = 0.499/(sum_for_gain - sum_for_offset); 
}

void adcInit(uint8_t gain)
{
	struct adc_config adc_conf;
	struct adc_channel_config adcch_conf;
	adc_read_configuration(&ADCA, &adc_conf);
	adc_set_conversion_parameters(&adc_conf, ADC_SIGN_OFF, ADC_RES_12, ADC_REF_VCC);
	adc_set_conversion_trigger(&adc_conf, ADC_TRIG_MANUAL, 1, 0);
	adc_set_clock_rate(&adc_conf, 200000UL);
	adc_write_configuration(&ADCA, &adc_conf);
	
	adcch_read_configuration(&ADCA, ADC_CH0, &adcch_conf);
	
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN7, ADCCH_NEG_NONE, gain);
	adcch_write_configuration(&ADCA, ADC_CH0, &adcch_conf);
	
	adcch_set_input(&adcch_conf, ADCCH_POS_PIN4, ADCCH_NEG_NONE, gain);
	adcch_write_configuration(&ADCA, ADC_CH1, &adcch_conf);
	
	adc_enable(&ADCA);
	delay_ms(10);
	adc_calibration();
}

void display_data(PT1000 *sensor, int idx){
	//int x = idx%2;
	int y = idx;
	
	gfx_color_t bg_color = (sensor->state == sensor->WITHOUT_ERROR)? ILI9341_COLOR(0, 255, 145):ILI9341_COLOR(255, 106, 31);
	
	if (sensor->state_change)
	{
		sensor->state_change = false;
		gfx_draw_filled_rect(0,y*ILI9341_DEFAULT_HEIGHT/4,ILI9341_DEFAULT_WIDTH, ILI9341_DEFAULT_HEIGHT/4, bg_color);
		gfx_draw_rect(0,y*ILI9341_DEFAULT_HEIGHT/4,ILI9341_DEFAULT_WIDTH, ILI9341_DEFAULT_HEIGHT/4, ILI9341_COLOR(29,29,31));
		gfx_draw_string_aligned(sensor->label, 10,  y*ILI9341_DEFAULT_HEIGHT/4 + ILI9341_DEFAULT_HEIGHT/8, &sysfont, bg_color, ILI9341_COLOR(150,150,150),
		TEXT_POS_CENTER_LEFT, TEXT_ALIGN_CENTER);
	}
	
	//int count = sprintf(data, "offset: %c%d.%d, gain: %d", sign,
	//int(fabs(adc_offset)),
	//int(fabs(adc_offset*100))%100,
	//int(adc_gain*100000));
	
	char data[10];
	sprintf(data, " %d.%dC", int(sensor->temperature), abs(int(sensor->temperature*10)%10));
	gfx_draw_string_aligned(data, ILI9341_DEFAULT_WIDTH-10,  y*ILI9341_DEFAULT_HEIGHT/4 + ILI9341_DEFAULT_HEIGHT/8, &sysfont, bg_color, ILI9341_COLOR(150,150,150),
	TEXT_POS_CENTER_RIGHT, TEXT_ALIGN_CENTER);
}

int main(void)
{
	sysclk_init();
	ioport_init();
	
	ioport_configure_pin(DC_CONTROL2, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);//Fan:(fuse) - Initially active.
	ioport_configure_pin(DC_CONTROL1, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);//Cold area: (fuse )
	
	ioport_configure_pin(AC_CONTROL2, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);//mold warmer: 1.2A (fuse 2A)
	ioport_configure_pin(AC_CONTROL4, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);// work area : 1.8 A (fuse 3A)
	ioport_configure_pin(AC_CONTROL5, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);// CassetteBath : 1.187 (Fuse 3A)
	ioport_configure_pin(AC_CONTROL3, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);//pances: (fuse 2A) - Initially active.
	
	adcInit(1);

	AdcSettings cold_plate_adc   (ADC_CH0, ADCCH_POS_PIN6, ADCCH_NEG_NONE, 1, adc_offset, adc_gain, 1098);
	AdcSettings work_area_adc    (ADC_CH1, ADCCH_POS_PIN5, ADCCH_NEG_NONE, 1, adc_offset, adc_gain, 1098);
	AdcSettings cassette_bath_adc(ADC_CH2, ADCCH_POS_PIN2, ADCCH_NEG_NONE, 1, adc_offset, adc_gain, 1097);
	AdcSettings mold_warmer_adc  (ADC_CH3, ADCCH_POS_PIN1, ADCCH_NEG_NONE, 1, adc_offset, adc_gain, 1098);
	
	ControlSettings cold_plate_control   (-5, 10,  5, DC_CONTROL1, true, false);
	ControlSettings work_area_control    (65, 0.1, 2, AC_CONTROL4, false, true);
	ControlSettings cassette_bath_control(65, 0.1, 2, AC_CONTROL5, false, true);
	ControlSettings mold_warmer_control  (65, 0.1, 2, AC_CONTROL2, false, true);
	
	PT1000 cold_plate   ("Cold Plate"   , cold_plate_adc   , cold_plate_control   );
	PT1000 work_area    ("Work Area"    , work_area_adc    , work_area_control    );
	PT1000 cassette_bath("Cassette Bath", cassette_bath_adc, cassette_bath_control);
	PT1000 mold_warmer  ("Mold Warmer"  , mold_warmer_adc  , mold_warmer_control  );
	
	
	ioport_configure_pin(SCK, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(MOSI, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(MISO, IOPORT_DIR_INPUT);
	ioport_configure_pin(CONF_ILI9341_CS_PIN, IOPORT_DIR_OUTPUT	| IOPORT_INIT_HIGH);
	ioport_configure_pin(CONF_ILI9341_DC_PIN, IOPORT_DIR_OUTPUT);
	ioport_configure_pin(CONF_ILI9341_BACKLIGHT_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(CONF_ILI9341_RESET_PIN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	
	gfx_init();
	delay_ms(50);
	ili9341_backlight_off();

	uint32_t counter = 0;
	display_data(&cold_plate, 0);
	display_data(&work_area, 1);
	display_data(&cassette_bath, 2);
	display_data(&mold_warmer, 3);
	
	wdt_set_timeout_period(WDT_TIMEOUT_PERIOD_500CLK);
	wdt_enable();
	
	while (1)
	{
		wdt_reset();
		counter++;
		if (counter == 500)
		{
			counter =0;
			
			display_data(&cold_plate, 0);
			display_data(&work_area, 1);
			display_data(&cassette_bath, 2);
			display_data(&mold_warmer, 3);
			
			//char data[40];
			//int line_counter = 0;
			//
			//char sign = '+';
			//if (adc_offset < 0) {
				//sign = '-';
			//}
			//int count = sprintf(data, " offset: %c%d.%d, gain: %d", sign,
			//int(fabs(adc_offset)),
			//int(fabs(adc_offset*100))%100,
			//int(adc_gain*100000));
			//gfx_draw_string(data, 0, SYSFONT_HEIGHT*line_counter, &sysfont, GFX_COLOR_WHITE, GFX_COLOR_RED );
			//line_counter++;
		//
			//
			//count = sprintf(data, " ColdPlate   T:%d V:%d R:%d ",
			//int(cold_plate.temperature*10),
			//int(cold_plate.voltage*10) ,
			//int(cold_plate.resistance));
			//gfx_draw_string(data, 0, SYSFONT_HEIGHT*line_counter, &sysfont, GFX_COLOR_WHITE, GFX_COLOR_RED );
			//line_counter++;
			//
			//count = sprintf(data, " WorkArea    T:%d V:%d R:%d ",
			//int(work_area.temperature*10),
			//int(work_area.voltage*10) ,
			//int(work_area.resistance));
			//gfx_draw_string(data, 0, SYSFONT_HEIGHT*line_counter, &sysfont, GFX_COLOR_WHITE, GFX_COLOR_RED );
			//line_counter++;
			//
			//count = sprintf(data, " CassetteBathT:%d V:%d R:%d ",
			//int(cassette_bath.temperature*10),
			//int(cassette_bath.voltage*10) ,
			//int(cassette_bath.resistance));
			//gfx_draw_string(data, 0, SYSFONT_HEIGHT*line_counter, &sysfont, GFX_COLOR_WHITE, GFX_COLOR_RED );
			//line_counter++;
			//
			//count = sprintf(data, " MoldWarmer  T:%d V:%d R:%d ",
			//int(mold_warmer.temperature*10),
			//int(mold_warmer.voltage*10) ,
			//int(mold_warmer.resistance));
			//gfx_draw_string(data, 0, SYSFONT_HEIGHT*line_counter, &sysfont, GFX_COLOR_WHITE, GFX_COLOR_RED );
		}
		
		cold_plate   .get_temperature();
		work_area    .get_temperature();
		cassette_bath.get_temperature();
		mold_warmer  .get_temperature();
		
		cold_plate   .control_temp();
		work_area    .control_temp();
		cassette_bath.control_temp();
		mold_warmer  .control_temp();
		delay_ms(1);
		
	}
}

