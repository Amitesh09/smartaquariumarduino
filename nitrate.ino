/*
 * This is the main firmware to be run on the Arduino Mega
 */
 
#include <stdlib.h>
#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Arduino_JSON.h>
#include "Adafruit_TCS34725.h"

#define MUX_Address 0x77 // TCA9548A Encoders address

struct Color {
  float red;
  float green;
  float blue;
} color;

#define firstPin 22
#define firstMuxPin 2

#define secondPin 24
#define secondMuxPin 3

#define thirdPin 26
#define thirdMuxPin 0

#define commonAnode true
byte gammatable[256];
Adafruit_TCS34725 tcs;// = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 tcs2;// = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 tcs3;// = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


#define pumpPin1 30
#define pumpPin2 32
#define pumpPin3 34
#define pumpPin4 36
#define pumpPin5 38
#define pumpPin6 40

#define servo1_pin 7
#define servo1_power_pin 27
#define servo2_pin 8
#define servo2_power_pin 29
#define servo3_pin 9
#define servo3_power_pin 25
//#define servo4_pin A7
#define fishServo A2


void setup(void)
{
  // Serial setup
  // Serial is for debugging and Serial3 is for communication with the wifi module
  Serial.begin(115200);
  Serial3.begin(38400);

  pinMode(firstPin, OUTPUT);
  pinMode(secondPin, OUTPUT);
  pinMode(thirdPin, OUTPUT);

  pinMode(pumpPin1, OUTPUT);
  digitalWrite(pumpPin1, LOW);
  pinMode(pumpPin2, OUTPUT);
  digitalWrite(pumpPin2, LOW);
  pinMode(pumpPin3, OUTPUT);
  digitalWrite(pumpPin3, LOW);
  pinMode(pumpPin4, OUTPUT);
  digitalWrite(pumpPin4, LOW);
  pinMode(pumpPin5, OUTPUT);
  digitalWrite(pumpPin5, LOW);
  pinMode(pumpPin6, OUTPUT);
  digitalWrite(pumpPin6, LOW);

  // Start up the library
  //initialize_temp_sensor();

  // Delay to allow wifi to connect
  delay(10000);
  Serial.println("Setup complete");
}

void loop(void){ 
  // Color sensor code
  float red, green, blue;
 
  digitalWrite(secondPin, HIGH);
  delay(1000);
  
  initialize_color_sensors();
  delay(1000);
  select_mux_bus(3);

  
  tcs2.setInterrupt(false);  // turn on LED
  delay(60);  // takes 50ms to read
  tcs2.getRGB(&red, &green, &blue);
  tcs2.setInterrupt(true);  // turn off LED
  delay(1000);
  digitalWrite(secondPin, LOW);

  
  Serial.println("sensor:");
  Serial.print("R:\t"); Serial.print(int(red));
  Serial.print("\tG:\t"); Serial.print(int(green));
  Serial.print("\tB:\t"); Serial.print(int(blue));
  Serial.print("\n");
  if (red > 102){
    send_nitrate_to_server(100);
  }
  else{
    send_nitrate_to_server(20);
  }
  delay(500);
}

boolean send_nitrate_to_server(float nitrate)
{
  char nitrateBuffer[8];
  dtostrf(nitrate, 4, 2, nitrateBuffer);

  return send_data_to_server(3, nitrateBuffer, "nitrate");
}

boolean send_data_to_server(int id, char * value, String endpoint)
{  
  JSONVar dataValues;
  dataValues["message_id"] = id;
  dataValues["message"][endpoint] = value;

  String transferString = JSON.stringify(dataValues);
  const char * sensor_json = transferString.c_str();
  Serial.println(transferString);
  Serial3.write(sensor_json);
  Serial3.flush();
  delay(1000);
}

boolean get_tasks_from_server()
{
  String TASKS[4] = {"feed", "runChemicalTest", "lightToggle", "heaterToggle"};
  int TASK_OFFSET = 7;
  bool TASK_STATUS[4] = {false, false, false, false};
  bool TASK_SET_VALUES[4] = {true, true, true, true};

  JSONVar sendJSON, resetValueJSON;

  for (int i = 0 ; i < 4; i++) {
    sendJSON["message_id"] = TASK_OFFSET + i;
    String transferString = JSON.stringify(sendJSON);
    const char * get_task_json = transferString.c_str();
    Serial.println(get_task_json);
    Serial3.write(get_task_json);

    //delay(3000);
    int timeout = 0;
    while (Serial3.available() < 10 && timeout <= 400) {
      delay(25);
      timeout++;
    }

    if (Serial3.available() > 0) {
      while (true && Serial3.available() > 0) {
        String json_task_string = Serial3.readString();
        Serial.println(json_task_string);
        JSONVar task = JSON.parse(json_task_string);
        TASK_STATUS[i] = (bool) task[TASKS[i]];
        TASK_SET_VALUES[i] = (bool) task[TASKS[i]];
        if (json_task_string.indexOf(TASKS[i]) > -1) {
          break;
        }
      }
    } else {
      Serial.println("Not reading values");
    }
    delay(4000);
    Serial3.read();
  }

  if (TASK_STATUS[0]) {
    Serial.println("Feeding fish");
    //feed_fish();
    send_reset_signal(11);
  }

  if (TASK_STATUS[1]) {
    Serial.println("Running chemical tests");
   // run_chemical_test();
    send_reset_signal(12);
  }

  if (TASK_STATUS[2]) {
    Serial.println("Toggling lights");
    //toggle_lights();
    send_reset_signal(13);
  }

  if (TASK_STATUS[3]) {
    Serial.println("Toggling heater");
    //toggle_lights();
    send_reset_signal(14);
  }

  return true;
}

void select_mux_bus(uint8_t bus)
{
  Wire.beginTransmission(MUX_Address);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  delay(50);
}

void send_reset_signal(int id)
{
  JSONVar resetValueJSON;
  resetValueJSON["message_id"] = id;
  String transferString = JSON.stringify(resetValueJSON);
  const char * return_task_json = transferString.c_str();
  Serial3.write(return_task_json);
  delay(2000);
}

void initialize_color_sensors()
{
  Wire.begin();
  select_mux_bus(secondMuxPin);
  tcs2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
  if (tcs2.begin()) {
    Serial.println("Sensor found");
  } else {
    Serial.println("Sensor not found");
  }
}

Color get_rgb_values_from_color_sensor(int sensor_number)
{
  
}
    
