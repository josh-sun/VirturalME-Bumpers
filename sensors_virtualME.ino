/* 
   Sensors -1 and 0 are ultrasonics
   1 - 6 are IR sensors
*/

#include <SoftwareSerial.h>

//Ultrasonic sensors
#define trig_SensorOne A0
#define echo_SensorOne A1
#define trig_SensorTwo A2
#define echo_SensorTwo A3

//IR sensors
#define irPin_One 4
#define irPin_Two 5
#define irPin_Three 6
#define irPin_Four 7
#define irPin_Five 8
#define irPin_Six 9

float distance_all_sensors[8];

float distance_U;
float distance_irPin;

//SoftwareSerial blue (0,1); //add this line if using bluetooth shield

void setup() {
  //blue.begin (9600);
  pinMode(trig_SensorOne, OUTPUT);
  pinMode(echo_SensorOne, INPUT);
  pinMode(trig_SensorTwo, OUTPUT);
  pinMode(echo_SensorTwo, INPUT);
  
  Serial.begin(9600);
  //while (!Serial) { ;} add this line if using leonardo
}

float ultrasonic_detect (char trig, char echo) {

    long duration_U;
    
    digitalWrite(trig, LOW); 
    delayMicroseconds(2); 
    digitalWrite(trig, HIGH);
    delayMicroseconds(10); 
    digitalWrite(trig, LOW);
    
    duration_U = pulseIn(echo, HIGH);
    distance_U = (float(duration_U)/2.0) / 29.1; //convert to cm
    delay(10);
}

void ir_detect (int irPin) {
    float volts_irPin = analogRead(irPin)*0.0048828125;   // value from sensor * (5/1024) - if running 3.3.volts then change 5 to 3.3
    distance_irPin = 62.21*pow((volts_irPin),-1.0902);      // convert to cm with the power function derived from graph
    delay(10);                                     // arbitary wait time.
}

void loop() {
  if (Serial.available()) {
    
    ultrasonic_detect(A0,A1); //detect distance with ultrasonic sensors
    distance_all_sensors[0] = distance_U;
    ultrasonic_detect(A2,A3);
    distance_all_sensors[1] = distance_U;
    
    for (int x = 4; x < 9; x++) {
        ir_detect(x); //detect distance with ir sensors
        distance_all_sensors[x] = distance_irPin;
    }
    
    for (int x = 0; x < 8; x++) {
        Serial.print("Sensor: ");
        Serial.print(x-2);
        Serial.print(" ");
        Serial.println(distance_all_sensors[x]);
        Serial.println("");
    }
    
    delay(100);
  }
}

