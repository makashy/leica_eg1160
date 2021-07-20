/*
* temp_sensor.h
*
* Created: 2021-07-07 11:48:48 AM
*  Author: mamin
*/


#ifndef TEMP_SENSOR_H_
#define TEMP_SENSOR_H_
#include "asf.h"
#include <math.h>

class AdcSettings{
	
	public:
	AdcSettings(uint8_t adc_channel,
				enum    adcch_positive_input pos,
				enum    adcch_negative_input neg,
				uint8_t opamp_gain,
				float   adc_offset,
				float   sreies_resistor,
				float   composite_coefficient,
				float   R1R2_ratio);

	uint8_t adc_channel;
	enum adcch_positive_input pos;
	enum adcch_negative_input neg;
	uint8_t opamp_gain;
	float adc_offset;
	float sreies_resistor;
	float composite_coefficient;
	float R1R2_ratio;
	};

class ControlSettings{
	
	public:
	float set_point;
	float threshold;
	float display_threshold;
	port_pin_t actuator_pin;
	bool is_active_high;
	bool positive_control;
	
	ControlSettings(float set_point,
					float threshold,
					float display_threshold,
					port_pin_t actuator_pin,
					bool is_active_high,
					bool positive_control);
	};

class PT1000{
	
	public:
	
	typedef enum
	{
		WITH_ERROR,
		WITHOUT_ERROR,
	} State_t;
	
	
	const char * label;
	AdcSettings adc_settings;
	ControlSettings control_settings;
	
	
	float result;
	float voltage;
	float resistance;
	float temperature;
	State_t state;
	bool state_change;
	
	PT1000(const char *label,
		   AdcSettings adc_settings,
		   ControlSettings control_settings);
	
	float get_temperature();
	
	void control_temp();
};

#endif /* TEMP_SENSOR_H_ */