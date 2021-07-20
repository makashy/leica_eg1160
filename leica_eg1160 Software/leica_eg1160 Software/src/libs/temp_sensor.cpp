/*
 * temp_sensor.cpp
 *
 * Created: 2021-07-07 11:48:35 AM
 *  Author: mamin
 */ 

#include "temp_sensor.h"


AdcSettings::AdcSettings(uint8_t adc_channel,
						 enum    adcch_positive_input pos,
						 enum    adcch_negative_input neg,
						 uint8_t opamp_gain,
						 float   adc_offset,
						 float   sreies_resistor,
						 float   composite_coefficient,
						 float   R1R2_ratio)
						 :adc_channel(adc_channel), pos(pos), neg(neg), opamp_gain(opamp_gain),
						 adc_offset(adc_offset), sreies_resistor(sreies_resistor),
						 composite_coefficient(composite_coefficient), R1R2_ratio(R1R2_ratio){
							 
						 }
						 
ControlSettings::ControlSettings(float set_point,
								 float threshold,
								 float display_threshold,
								 port_pin_t actuator_pin,
								 bool is_active_high,
								 bool positive_control)
								 :set_point(set_point), threshold(threshold),
								 display_threshold(display_threshold),
								 actuator_pin(actuator_pin), is_active_high(is_active_high),
								 positive_control(positive_control){
									 
								 }

PT1000::PT1000(const char *label, 
			   AdcSettings adc_settings, 
			   ControlSettings control_settings)
			   :label(label), adc_settings(adc_settings), control_settings(control_settings){
	
	result = 0;
	resistance = 0;
	temperature = 25;
	state = WITH_ERROR;
	state_change = true;
	
	struct adc_channel_config adcch_conf;
	adcch_read_configuration(&ADCA, ADC_CH0, &adcch_conf);
	adcch_set_input(&adcch_conf, adc_settings.pos, adc_settings.neg, adc_settings.opamp_gain);
	adcch_write_configuration(&ADCA, adc_settings.adc_channel, &adcch_conf);
}

float PT1000::get_temperature(){
	//for(int i = 0;i<3;i++){
		//adc_start_conversion(&ADCA, adc_settings.adc_channel);
		//adc_wait_for_interrupt_flag(&ADCA, adc_settings.adc_channel);
		//result = (-static_cast<float>(adc_get_signed_result(&ADCA, adc_settings.adc_channel) - adc_settings.adc_offset)- result)*0.001 + result;
	//}
	adc_start_conversion(&ADCA, adc_settings.adc_channel);
	adc_wait_for_interrupt_flag(&ADCA, adc_settings.adc_channel);
	result = (adc_get_signed_result(&ADCA, adc_settings.adc_channel) - result)*0.001 + result;
	
	//voltage = (result - adc_settings.adc_offset) * adc_settings.R1R2_ratio;
	resistance = adc_settings.sreies_resistor * (1/(result - adc_settings.adc_offset)/adc_settings.R1R2_ratio - 1);
	
	voltage =(resistance - 1000)/static_cast<float>(4);
	float a = 3.90802e-03;
	float b = -5.80195e-07;
	float R0 = 1000;
	temperature = (-R0 * a + sqrt(pow(R0 * a, 2) - 4 * R0 * b * (R0 - resistance)))/ (2 * R0 * b);
	
	State_t new_state = (abs(control_settings.set_point - temperature) > control_settings.display_threshold)? WITH_ERROR : WITHOUT_ERROR;
	state_change = (new_state != state)? true:state_change;
	state = new_state;
	return temperature;
}

void PT1000::control_temp(){
	float error = control_settings.set_point - temperature;
	
	if (error > control_settings.threshold){
		if(control_settings.positive_control){
			ioport_set_pin_level(control_settings.actuator_pin,  control_settings.is_active_high);
		}else{
			ioport_set_pin_level(control_settings.actuator_pin, !control_settings.is_active_high);
		}
	}
	
	if (error < -control_settings.threshold){
		if (!control_settings.positive_control){
			ioport_set_pin_level(control_settings.actuator_pin,  control_settings.is_active_high);
		}else{
			ioport_set_pin_level(control_settings.actuator_pin, !control_settings.is_active_high);
		}
	}
	
}