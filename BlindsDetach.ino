/*
 * simple_led.ino
 *
 * Setup code: 111-11-111
 *
 *  Created on: 03/09 2021
 *      Author: Peter Lai
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>        // Include the mDNS library
#include "wifi_info.h"

#include <arduino_homekit_server.h>
#define PIN_VO 14 // main pin
#define PIN_EN 12 // enable LOW: connect HIGH: no state

#define X  0
#define UP 1 // Direction ID
#define HL 2
#define DN 3

// states
#define TO_MIN 0
#define TO_MAX 1
#define STOP   2


#define PIN_LED 2//D0




#define SIMPLE_INFO(fmt, ...)   printf_P(PSTR(fmt "\n") , ##__VA_ARGS__);


// homekit characteristics
int bri_now = 0;
int bri_target = 100; //[0, 100], 0: full cover, 100: no cover
int current_bri_target = 100;
uint8_t state = STOP; // stopped: 2; to min: 0, to max: 1


// action logics
uint32_t hl_millis = 0;  // the time when the hold button should be push


const uint16_t push_ms = 60;  // for single click simulation 60mm

const uint16_t percent_ms = 180;  //it takes about 9500mm for the blinds to fully cover
const uint16_t sink_ms = 3800; 

const uint8_t SPin[4] = {5, 4, 0, 2}; 


// only 0~3 is enabled
const uint8_t STable[16][4] = {
  // s0, s1, s2, s3
  {0,  0,  0,  0}, // X
  {1,  0,  0,  0}, // UP
  {0,  1,  0,  0}, // HL
  {1,  1,  0,  0}, // DN 
  {0,  0,  1,  0}, // NA
  {1,  0,  1,  0}, // NA
  {0,  1,  1,  0}, // NA
  {1,  1,  1,  0}, // NA
  {0,  0,  0,  1}, // NA
  {1,  0,  0,  1}, // NA
  {0,  1,  0,  1}, // NA
  {1,  1,  0,  1}, // NA
  {0,  0,  1,  1}, // NA
  {1,  0,  1,  1}, // NA
  {0,  1,  1,  1}, // NA
  {1,  1,  1,  1}  // NA
};



extern "C" homekit_server_config_t accessory_config;
extern "C" homekit_characteristic_t accessory_name;
extern "C" homekit_characteristic_t position_state;
extern "C" homekit_characteristic_t current_position;
extern "C" homekit_characteristic_t target_position; 


/* ==============================
 * Misc.
 * ==============================
 */
void builtinledSetStatus(bool on) {
  digitalWrite(PIN_LED, on ? LOW : HIGH);
}

void blink_led(int interval, int count) {
  for (int i = 0; i < count; i++) {
    builtinledSetStatus(true);
    delay(interval);
    builtinledSetStatus(false);
    delay(interval);
  }
}

void info_overview() {
  SIMPLE_INFO("");
  SIMPLE_INFO("SketchSize: %d", ESP.getSketchSize());
  SIMPLE_INFO("FreeSketchSpace: %d", ESP.getFreeSketchSpace());
  SIMPLE_INFO("FlashChipSize: %d", ESP.getFlashChipSize());
  SIMPLE_INFO("FlashChipRealSize: %d", ESP.getFlashChipRealSize());
  SIMPLE_INFO("FlashChipSpeed: %d", ESP.getFlashChipSpeed());
  SIMPLE_INFO("SdkVersion: %s", ESP.getSdkVersion());
  SIMPLE_INFO("FullVersion: %s", ESP.getFullVersion().c_str());
  SIMPLE_INFO("CpuFreq: %dMHz", ESP.getCpuFreqMHz());
  SIMPLE_INFO("FreeHeap: %d", ESP.getFreeHeap());
  SIMPLE_INFO("ResetInfo: %s", ESP.getResetInfo().c_str());
  SIMPLE_INFO("ResetReason: %s", ESP.getResetReason().c_str());
}


void accessory_init() {
  pinMode(PIN_EN, OUTPUT);
  toggle_select(false);
  
  for (int i = 0; i < 4; i++) {
    pinMode(SPin[i], OUTPUT);
  }
  pinMode(PIN_VO, OUTPUT);
  
  bri_reset();

  hl_millis = millis();
}


/* =============================
 * Simulate Button Select & Push
 * =============================
 */

void select_dir(int dir){
  for (int i = 0; i < 4; i++) {
    digitalWrite(SPin[i], STable[dir][i]);
  }
}

void toggle_select(bool select_on) {
  digitalWrite(PIN_EN, select_on? LOW : HIGH);
}

uint8_t dir_to_state(int dir) {
  switch (dir) {
    case UP: return TO_MAX;
    case DN: return TO_MIN;
    case HL: return STOP;
    default: ;
  }
}

/* 
 * Simulate Push Button
 */
void push_btn(int dir) {
  // 1. select direction and then enable selection
  select_dir(dir);
  toggle_select(true); 
  digitalWrite(PIN_VO, HIGH);

  // 2. simulate the duration of a button push
  delay(push_ms);

  // 3. disconnect, clear selection
  digitalWrite(PIN_VO, LOW); 
  toggle_select(false);
  select_dir(X); 

  // 4. update the state (non-homekit
  state = dir_to_state(dir);
}



/* =============================
 * Loop Updates
 * =============================
 */

void update_hold_schedule(int offset = 0) {
  uint32_t t_millis = millis();
  /* uncorrected schedule */
  uint16_t duration = abs(bri_target - bri_now) * percent_ms;
  if (bri_target > 0 && bri_now == 0) duration += sink_ms;
  printf("moving from %d to %d and it will take %d ms \n", bri_now, bri_target, duration); 

  /* correct schedule by miusing click time */
  hl_millis = t_millis + duration + offset;
  current_bri_target = bri_target;
}

void update_current_position() {
  uint32_t t_millis = millis();
  uint32_t percent_diff = (hl_millis - t_millis) / (uint32_t)percent_ms; 
  
  switch (state) {
    case TO_MAX: bri_now = bri_target - percent_diff; break;
    case TO_MIN: bri_now = bri_target + percent_diff; break;
    case STOP:   bri_now = bri_target; break;
    default: ;
  }
}

/* 
 * reset the coverage to 0% (bri = 100)
 */
void bri_reset() {
  bri_target = 100;
  homekit_characteristic_notify(&target_position, HOMEKIT_UINT8(bri_target));
}

void perform_hold() {
   printf("done moving\n");
   if (bri_target % 100 == 0) state = STOP;
   else {                       
     push_btn(HL);
     delay(60);
     push_btn(HL);
   }
   update_current_position();
}

void perform_up( ) {
  push_btn(UP); 
  update_hold_schedule( -push_ms );
}

void perform_down() {
  push_btn(DN); 
  update_hold_schedule( -push_ms );
}


void to_max_handler() {
  uint32_t t_millis = millis();
  if ( bri_target < bri_now ) perform_down();

  /* state change to STOP if it is scheduled time */
  else if ( t_millis >= hl_millis ) perform_hold();

  else {
    /* state remains the same, update bri_now regularily */
    if ( (hl_millis - t_millis) % (uint32_t)percent_ms == 0 ) update_current_position();
    /* state remains the same, check if target change */
    if (current_bri_target != bri_target)                     update_hold_schedule(); 
  }
}

void to_min_handler() {
  uint32_t t_millis = millis();
  if ( bri_target > bri_now ) perform_up();

  /* state change to STOP if it is scheduled time */
  else if ( t_millis >= hl_millis ) perform_hold();

  else {
    /* state remains the same, update bri_now regularily */
    if ( (hl_millis - t_millis) % (uint32_t)percent_ms == 0 ) update_current_position();
    /* state remains the same, check if target change */
    if (current_bri_target != bri_target)                     update_hold_schedule(); 
  }
}


void stop_handler() {
  if (bri_now == bri_target) return;
  else if (bri_target > bri_now) perform_up();
  else                           perform_down();
}


void user_loop() {
  switch (state) {
    case TO_MAX: to_max_handler(); break;
    case TO_MIN: to_min_handler(); break;
    case STOP:   stop_handler();   break;
    default: break;
  }
}


/* =============================
 * Characteristics get/set
 * =============================
 */
 
/*
 *  [x] TODO#1 @Peter change brightness to target position (RW)
 */
homekit_value_t position_state_getter() {
  printf("homekit is trying to get %s\n", "state");
  return HOMEKIT_UINT8(state);
}

homekit_value_t current_position_getter() {
  printf("homekit is trying to get %s\n", "position");
  return HOMEKIT_UINT8((uint8_t)bri_now);
}

homekit_value_t target_position_getter() {
  printf("homekit is trying to get %s\n", "target");
  return HOMEKIT_UINT8((uint8_t)bri_target);
}

void target_position_setter(homekit_value_t value) {
  if (value.format != homekit_format_uint8) {
    return;
  }
  printf("homekit is trying to set target to %d%\n", value.uint8_value);
  bri_target = (int) (value.uint8_value); 
  printf("target position : %d\n", bri_target);
}



//==============================
// Homekit setup and loop
//==============================


void homekit_setup() {
	accessory_init();
  homekit_storage_reset();
	uint8_t mac[WL_MAC_ADDR_LENGTH];
	WiFi.macAddress(mac);
	int name_len = snprintf(NULL, 0, "%s_%02X%02X%02X",
			accessory_name.value.string_value, mac[3], mac[4], mac[5]);
	char *name_value = (char*) malloc(name_len + 1);
	snprintf(name_value, name_len + 1, "%s_%02X%02X%02X",
			accessory_name.value.string_value, mac[3], mac[4], mac[5]);
	accessory_name.value = HOMEKIT_STRING_CPP(name_value);


  position_state.getter = position_state_getter;
  current_position.getter = current_position_getter;
  target_position.getter = target_position_getter;
  target_position.setter = target_position_setter;

	arduino_homekit_setup(&accessory_config);

}



void homekit_loop() {
	arduino_homekit_loop();
	static uint32_t next_heap_millis = 0;
  
	uint32_t t_millis = millis();
 
	if (t_millis > next_heap_millis) {
		SIMPLE_INFO("heap: %d, sockets: %d",
				ESP.getFreeHeap(), arduino_homekit_connected_clients_count());
		next_heap_millis = t_millis + 60000;
	}

  static uint32_t next_mdns_millis = 0;
  if (t_millis > next_mdns_millis) {
    MDNS.announce();
    next_mdns_millis = t_millis + 5000;
  }
   
}




//==============================
// Arduino setup and loop
//==============================

void setup() {
  // Serial Port
  Serial.begin(115200);
  Serial.setRxBufferSize(32);
  Serial.setDebugOutput(false);

  // WIFI
  pinMode(PIN_LED, OUTPUT);
  wifi_connect();

  // info 1
  info_overview();
  INFO_HEAP();


  // setup 
  blink_led(200, 3);
  homekit_setup();

  // info 2 
  INFO_HEAP();
  
}

void loop() {
  homekit_loop();
  user_loop();
}
