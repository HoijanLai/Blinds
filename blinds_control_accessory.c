/*
 * simple_led_accessory.c
 * Define the accessory in pure C language using the Macro in characteristics.h
 *
 *  Created on: 03/09 2021
 *      Author: Peter Lai
 */

#include <Arduino.h>
#include <homekit/types.h>
#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <stdio.h>
#include <port.h>

//const char * buildTime = __DATE__ " " __TIME__ " GMT";

#define ACCESSORY_NAME  ("Peter's Shades")
#define ACCESSORY_SN  ("SN_03092021")  //SERIAL_NUMBER
#define ACCESSORY_MANUFACTURER ("PeterLai")
#define ACCESSORY_MODEL  ("ESP8266")



homekit_characteristic_t accessory_name = HOMEKIT_CHARACTERISTIC_(NAME, ACCESSORY_NAME);
homekit_characteristic_t serial_number = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, ACCESSORY_SN);

homekit_characteristic_t position_state = HOMEKIT_CHARACTERISTIC_(POSITION_STATE, 2);
homekit_characteristic_t current_position = HOMEKIT_CHARACTERISTIC_(CURRENT_POSITION, 100);
homekit_characteristic_t target_position = HOMEKIT_CHARACTERISTIC_(TARGET_POSITION, 100); 



/* 
 * [ ] TODO@ Peter : blinking to something else
 */
void accessory_identify(homekit_value_t _value) {
	printf("accessory identify\n");
}


/* 
 * [x] TODO#2 @Peter : change led registering to blinds registering
 */
homekit_accessory_t *accessories[] =
		{
				HOMEKIT_ACCESSORY(
						.id = 1,
						.category = homekit_accessory_category_window_covering,
						.services=(homekit_service_t*[]){
						HOMEKIT_SERVICE(ACCESSORY_INFORMATION,
						.characteristics=(homekit_characteristic_t*[]){
						&accessory_name,
						HOMEKIT_CHARACTERISTIC(MANUFACTURER, ACCESSORY_MANUFACTURER),
						&serial_number,
						HOMEKIT_CHARACTERISTIC(MODEL, ACCESSORY_MODEL),
						HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "v1.0.0"),
						HOMEKIT_CHARACTERISTIC(IDENTIFY, accessory_identify), // [x] TODO: lightbulb to blinds
						NULL
						}),
						HOMEKIT_SERVICE(WINDOW_COVERING, .primary=true, 
						.characteristics=(homekit_characteristic_t*[]){
						HOMEKIT_CHARACTERISTIC(NAME, "Shades Control"),
            &current_position,
            &target_position,
            &position_state,
						NULL
						}),
						NULL
						}),
				NULL
		};


homekit_server_config_t accessory_config = {
		.accessories = accessories,
		.password = "101-11-101",
		//.on_event = on_homekit_event,
		.setupId = "ABCD"
};
