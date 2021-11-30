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

#define pHPin 53
#define pHValPin 8

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


#define servo1_pin A4
#define servo2_pin A5
#define servo3_pin A6
#define servo4_pin A7
#define fishServo A2

void set_low_on_startup() {
  digitalWrite(pumpPin1, LOW);
  digitalWrite(pumpPin2, LOW);
  digitalWrite(pumpPin3, LOW);
  digitalWrite(firstPin, LOW);
  digitalWrite(secondPin, LOW);
  digitalWrite(thirdPin, LOW);
  
  analogWrite(servo1_pin, LOW);
  analogWrite(servo2_pin, LOW);
  analogWrite(servo3_pin, LOW);
  analogWrite(servo4_pin, LOW);
  analogWrite(fishServo, LOW);

}
// Data wire is conntec to the Arduino digital pin 2
#define ONE_WIRE_BUS A3

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

Servo Servo1, Servo2, Servo3, Servo4;
int pos = 0;

void setup(void)
{
  //Servo setup
  Servo1.attach(servo1_pin);
  turn_servo_to_angle(Servo1, 0);
  delay(200);
  Servo2.attach(servo2_pin);
  turn_servo_to_angle(Servo2, 0);
  delay(200);

  // Serial setup
  // Serial is for debugging and Serial3 is for communication with the wifi module
  Serial.begin(115200);
  Serial3.begin(38400);

  pinMode(firstPin, OUTPUT);
  pinMode(secondPin, OUTPUT);
  pinMode(thirdPin, OUTPUT);
  
  pinMode(pHValPin,OUTPUT);
  pinMode(pHPin,OUTPUT);


  pinMode(pumpPin1, OUTPUT);
  digitalWrite(pumpPin1, LOW);

  // Start up the library
  // sensors.begin();
  initialize_temp_sensor();

  // Delay to allow wifi to connect
  delay(10000);
  Serial.println("Setup complete");
}

void loop(void){ 
  // Color sensor code
  //run_chemical_test();


  // Temperature Code
  float tempReading = get_current_temp();
  Serial.println(tempReading);

  // Servo code
  //  turn_servo_to_angle(Servo1, 0);
  //  turn_servo_to_angle(Servo1, 45);

  //  turn_servo_to_angle(Servo2, 0);
  //  delay(3000);
  //  turn_servo_to_angle(Servo2, 90);
  

  // Code to communicate with server
  send_temp_to_server(tempReading);
  get_tasks_from_server();


  digitalWrite(pHPin, HIGH);       
  delay(200);
  float phValue = run_pH_sensor();
  delay(8000);
  digitalWrite(pHPin, LOW); 
  Serial.println("pH:");  
  Serial.print(phValue,2);
  
  send_ph_to_server(phValue);
  
  // Main loop delay
  delay(500);
}

boolean send_temp_to_server(float temp)
{
  char tempBuffer[8];
  dtostrf(temp, 4, 2, tempBuffer);

  return send_data_to_server(0, tempBuffer, "temp");
}

boolean send_ph_to_server(float ph)
{
  char phBuffer[8];
  dtostrf(ph, 4, 2, phBuffer);

  return send_data_to_server(1, phBuffer, "ph");
}

boolean send_ammonia_to_server(float ammonia)
{
  char ammoniaBuffer[8];
  dtostrf(ammonia, 4, 2, ammoniaBuffer);

  return send_data_to_server(2, ammoniaBuffer, "ammonia");
}

boolean send_nitrate_to_server(float nitrate)
{
  char nitrateBuffer[8];
  dtostrf(nitrate, 4, 2, nitrateBuffer);

  return send_data_to_server(3, nitrateBuffer, "nitrate");
}

boolean send_nitrite_to_server(float nitrite)
{
  char nitriteBuffer[8];
  dtostrf(nitrite, 4, 2, nitriteBuffer);

  return send_data_to_server(4, nitriteBuffer, "nitrite");
}

boolean send_light_status_to_server(boolean light_illuminated)
{
  char* light_status;
  if (light_illuminated) {
    light_status = "true";
  } else {
    light_status = "false";
  }
  return send_data_to_server(5, light_status, "light_illuminated");
}

boolean send_heater_status_to_server(boolean heater_on)
{
  char* heater_status;
  if (heater_on) {
    heater_status = "true";
  } else {
    heater_status = "false";
  }
  return send_data_to_server(6, heater_status, "heater_on");
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
    run_chemical_test();
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

void initialize_temp_sensor()
{
  sensors.begin();
}

float get_current_temp()
{
  sensors.requestTemperatures();
  
  float tempReading = sensors.getTempFByIndex(0);
  return tempReading;
}

void select_mux_bus(uint8_t bus)
{
  Wire.beginTransmission(MUX_Address);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  delay(50);
}

//void initialize_color_sensor(int sensorNumber)
//{
//  select_mux_bus(firstMuxPin);
//  tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
//  if (tcs.begin()) {
//    Serial.println("Sensor 1 found");
//  } else {
//    Serial.println("Sensor 1 not found");
//  }
//}

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
  select_mux_bus(firstMuxPin);
  tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
  if (tcs.begin()) {
    Serial.println("Sensor 1 found");
  } else {
    Serial.println("Sensor 1 not found");
  }

  
  select_mux_bus(secondMuxPin);
  tcs2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
  if (tcs2.begin()) {
    Serial.println("Sensor 2 found");
  } else {
    Serial.println("Sensor 2 not found");
  }

  select_mux_bus(thirdMuxPin);
  tcs3 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
  if (tcs3.begin()) {
    Serial.println("Sensor 3 found");
  } else {
    Serial.println("Sensor 3 not found");
  }
}

void run_chemical_test()
{
  float red, green, blue;
  float red2, green2, blue2;
  float red3, green3, blue3;

  digitalWrite(pumpPin1, HIGH);
  delay(4000);
  digitalWrite(pumpPin1, LOW);
  digitalWrite(firstPin, HIGH);
  delay(1000);
  
  initialize_color_sensors();
  delay(1000);
  select_mux_bus(2);
  
  tcs.setInterrupt(false);  // turn on LED
  delay(60);  // takes 50ms to read
  tcs.getRGB(&red, &green, &blue);
  tcs.setInterrupt(true);  // turn off LED
  delay(1000);
  digitalWrite(firstPin, LOW);
  
  Serial.println("First sensor:");
  Serial.print("R:\t"); Serial.print(int(red));
  Serial.print("\tG:\t"); Serial.print(int(green));
  Serial.print("\tB:\t"); Serial.print(int(blue));
  Serial.print("\n");


  digitalWrite(pumpPin2, HIGH);
  delay(100);
  digitalWrite(pumpPin2, LOW);

  digitalWrite(secondPin, HIGH);
  delay(1000);
  initialize_color_sensors();
  delay(1000);
  select_mux_bus(3);
  tcs2.setInterrupt(false);  // turn on LED
  delay(60);  // takes 50ms to read
  tcs2.getRGB(&red2, &green2, &blue2);
  tcs2.setInterrupt(true);  // turn off LED
  delay(1000);
  digitalWrite(secondPin, LOW);

  Serial.println("Second sensor:");
  Serial.print("R:\t"); Serial.print(int(red2));
  Serial.print("\tG:\t"); Serial.print(int(green2));
  Serial.print("\tB:\t"); Serial.print(int(blue2));
  Serial.print("\n");


  digitalWrite(pumpPin3, HIGH);
  delay(100);
  digitalWrite(pumpPin3, LOW);

  digitalWrite(thirdPin, HIGH);
  delay(1000);
  initialize_color_sensors();
  delay(1000);
  select_mux_bus(0);
  tcs3.setInterrupt(false);  // turn on LED
  delay(60);  // takes 50ms to read
  tcs3.getRGB(&red3, &green3, &blue3);
  tcs3.setInterrupt(true);  // turn off LED
  delay(1000);
  digitalWrite(thirdPin, LOW);

  Serial.println("Third sensor:");
  Serial.print("R:\t"); Serial.print(int(red3));
  Serial.print("\tG:\t"); Serial.print(int(green3));
  Serial.print("\tB:\t"); Serial.print(int(blue3));
  Serial.print("\n");
}

Color get_rgb_values_from_color_sensor(int sensor_number)
{
  
}

void turn_servo_to_angle(Servo servo, int angle)
{
  servo.write(angle);
  //delay(2000);

  //feedback = analogRead(A0);//Connect Analog pin to A0
  //observedAngle = (double) feedback * 0.47-33.4;
  //Serial.println(observedAngle);
}
        
float run_pH_sensor(){
  
  unsigned long int avgValue;
  float b;
  int buf[10],temp;

  for(int i=0;i<10;i++)       
  { 
    buf[i]=analogRead(pHValPin);
    delay(10);
  }
  for(int i=0;i<9;i++)       
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                      
    avgValue+=buf[i];
  float phValue=(float)avgValue*5.0/1024/6; 
  phValue=3.5*phValue;  
                   
  return phValue;
}
