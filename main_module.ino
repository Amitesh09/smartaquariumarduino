/*
   This is the main firmware to be run on the Arduino Mega
*/
#include <stdlib.h>
#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
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
Adafruit_TCS34725 tcs; // = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 tcs2; // = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Adafruit_TCS34725 tcs3; // = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#define pumpPin1 30
#define pumpPin2 32
#define pumpPin3 34
#define pumpPin4 36
#define pumpPin5 38
#define pumpPin6 40

#define servo1 A4
#define servo2 A5
#define servo3 A6
#define servo4 A7
#define fishServo A2

// Data wire is conntec to the Arduino digital pin 2
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

Servo myservo;
int pos = 0;

void setup(void)
{
  // Start serial communication for debugging purposes
  myservo.attach(9);
  Serial.begin(115200);
  Serial3.begin(9600);
  pinMode(firstPin, OUTPUT);
  pinMode(secondPin, OUTPUT);
  pinMode(thirdPin, OUTPUT);
  // Start up the library
  //  sensors.begin();
  
  initialize_temp_sensor();

}

void loop(void) {
  // Call sensors.requestTemperatures() to issue a global temperature and Requests to all devices on the bus

  // Color sensor code
  float red, green, blue;
  float red2, green2, blue2;
  float red3, green3, blue3;

  digitalWrite(pumpPin1, HIGH);
  delay(100);
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
  
  // if (val1 == reqdVal1) {
  // Serial.println("Test was completed. Results are positive.");
  // }
  // else (val1 == reqdVal1) {
  // Serial.println("Test was completed. Results are negative.");
  // }
  // analogWrite(redpin, gammatable[(int)red]);
  // analogWrite(greenpin, gammatable[(int)green]);
  // analogWrite(bluepin, gammatable[(int)blue]);

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
  //if (val2 == reqdVa2) {
  //  Serial.println("Test was completed. Results are positive.");
  // }
  // else (val2 == reqdVal2) {
  //  Serial.println("Test was completed. Results are negative.");
  // }

  // analogWrite(redpin2, gammatable[(int)red2]);
  // analogWrite(greenpin2, gammatable[(int)green2]);
  // analogWrite(bluepin2, gammatable[(int)blue2]);

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
  // if (val3 == reqdVal3) {
  //  Serial.println("Test was completed. Results are positive.");
  // }
  // else (val3 == reqdVal3) {
  //  Serial.println("Test was completed. Results are negative.");
  // }

 // analogWrite(redpin3, gammatable[(int)red3]);
 // analogWrite(greenpin3, gammatable[(int)green3]);
 // analogWrite(bluepin3, gammatable[(int)blue3]);


  uint16_t val;
  double dat;
  for (pos = 0; pos < 180; pos += 1) {        //servo turns from 0 to 180 in stepsof 1 degree
    myservo.write(pos);                       // tell servo to go to position in variable 'pos'
    delay(15);                                // wts 15ms for the servo to reach the position
    val = analogRead(A0); //Connect Analog pin to A0
    dat = (double) val * 0.47 - 33.4;
  }
  for (pos = 180; pos >= 1; pos -= 1) {      // servo turns from 180 to 0 in steps of 1 degree
    myservo.write(pos);                      //tell servo to go to position in variable 'pos'
    delay(15);                               //waits 15ms for the servo to reach the position
    val = analogRead(A0); //Connect Analog pin to A0
    dat = (double) val * 0.47 - 33.4;
 }

  // Temperature Code
  //  sensors.requestTemperatures();
  char tempBuffer[8];

  float tempReading = get_current_temp();
  dtostrf(tempReading, 4, 2, tempBuffer);
  Serial3.write(tempBuffer);
  delay(1000);
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
