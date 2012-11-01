//--------------------------------
// @file
// Tacit Glove
// Tactile feedback for the blind.
//--------------------------------

#include <NewPing.h>
#include <Servo.h>

#define SENSOR_NUM     2 // Number or sensors.
#define MAX_DISTANCE 150 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 50 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SENSOR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SENSOR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SENSOR_NUM] = {     // Sensor object array.
  NewPing(6, 7, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(8, 9, MAX_DISTANCE)
  };

Servo ServoList[SENSOR_NUM];
const int ServoPins[SENSOR_NUM] = {
  3,12}; //Servo motor pins on Arduino
const int ServoMaxAngle[SENSOR_NUM] = {
  0,180}; //Where to turns the servos when something is close
const int ServoMinAngle[SENSOR_NUM] = {
  80,110}; //Where to turns the servos when something is far

int val = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Tacit Glove Project");

  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SENSOR_NUM; i++) {
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;  // Set the starting time for each sensor.
  }
  for (uint8_t i = 0; i < SENSOR_NUM; i++) {  
    ServoList[i].attach(ServoPins[i]);  //Attach Servos
    //do a full sweep
    delay(10);
    ServoList[i].write(ServoMaxAngle[i]);
    delay(200);
    ServoList[i].write(ServoMinAngle[i]);
    delay(200);
    ServoList[i].write(90);
  }

}

void loop() {
  for (uint8_t i = 0; i < SENSOR_NUM; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SENSOR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SENSOR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  // The rest of the code
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  for (uint8_t i = 0; i < SENSOR_NUM; i++) {
    val = cm[i];
    val = map(val, 1, MAX_DISTANCE, ServoMinAngle[i], ServoMaxAngle[i]);
    ServoList[i].write(val);
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();
}


