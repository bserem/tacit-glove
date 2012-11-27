//--------------------------------
// @file
// Sonar Glove
// Tactile feedback for the blind
// for HC-SR04 Ultrasonic Rangefinders
// adapted to work with the NewPing library which allows very fast scanning
// by Bill Seremetis (bill@seremetis.net)
// 2012 November
//--------------------------------

#define _DEBUG_MODE 1
#define _DEBUG_SENSOR 0 //either 0 or 1

#include <NewPing.h>
#include <Servo.h>

#define SENSOR_NUM     2 // Number or sensors.
#define MAX_DISTANCE 350 // Maximum distance (in cm) to ping. Our sensors go to 400cm.
#define PING_INTERVAL 50 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SENSOR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SENSOR_NUM];         // Where the ping distances are stored.
byte currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SENSOR_NUM] = {     // Sensor object array.
  NewPing(6, 7, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(8, 9, MAX_DISTANCE)
};

Servo ServoList[SENSOR_NUM];
const int ServoPins[SENSOR_NUM] = {3,7};       //Servo motor pins on Arduino
const int ServoMaxAngle[SENSOR_NUM] = {90,90}; //Where to turns the servos when something is close
const int ServoMinAngle[SENSOR_NUM] = {0,180}; //Where to turns the servos when something is far
const int SensorClose = 10;                    // Closest value we detect with the PING sensor. (Soundwave travel time in milliseconds.)
const int SensorFar = 14000;                   // Furthest distance we register on the PING sensor. (Soundwave travel time in milliseconds.)
const int ReadingsPerSensor = 3;               // The number of historic readings to consider when determining position.
const int TimePerDegree = 30;                  // ms per degree rotation on the servo to prevent servo motor electrical noise from interfering with the ultrasonic sensor readings
const int MinimumTurnDistance = 3;             // Minimum number of degrees that the servo will turn. Keeps the servos from being too twitchy.

int sensorReadings[SENSOR_NUM][ReadingsPerSensor];   // Hold past readings for each sensor.
int calculatedSensorReadings[SENSOR_NUM];             // The calculated distance for each sensor.
int latestReading = 0;                               // Current position in the array for the most recent reading.
int servoLocations[SENSOR_NUM];                      // The current position of each sensor.

void setup() {
  Serial.begin(9600);
  Serial.println("Tacit Glove Project\r\nBOOT");

  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SENSOR_NUM; i++) {
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;  // Set the starting time for each sensor.
  }
  for (uint8_t i = 0; i < SENSOR_NUM; i++) {  
    ServoList[i].attach(ServoPins[i]);  //Attach Servos
    delay(10);
    //do a full sweep at boot
    ServoList[i].write(ServoMaxAngle[i]);
    delay(500);
    ServoList[i].write(ServoMinAngle[i]);
    delay(500);
    ServoList[i].write(90);
  }
}

void loop() {
  int i, j, oldLocation;
  unsigned long delayTime;

  // Loop through each range sensor
  for (i = 0; i < SENSOR_NUM; i++){
    // Get the current sensor's range.
    sensorReadings[i][latestReading] = getDistance(i);
    // Figure out an averaged/smoothed readings based on this and past data.
    calculatedSensorReadings[i] = calculateNewDistace(i);

    // Set the servo to the correct angle.
    oldLocation = servoLocations[i];
    servoLocations[i] = map(calculatedSensorReadings[i], 0, 100, ServoMinAngle[i], ServoMaxAngle[i]);

    if (latestReading >= ReadingsPerSensor-1){                          // Don't do anything until we have enough data to trend.
      if (abs(servoLocations[i]-oldLocation) >= MinimumTurnDistance){   // Only try to turn it if we have somewhere to go.
        ServoList[i].attach(ServoPins[i]);
        delay(10);
        ServoList[i].write(servoLocations[i]);
        delayTime = (TimePerDegree * (abs(servoLocations[i]-oldLocation))+20);      // Set a delay for the next reading so motor noise doesn't interfere with senor readings.
        if (abs(delayTime)>500){ // If it can't do it in this amount of time       // It's based on how far it has to turn to keep the delay to a minimum, response  me at a maximum.
            delayTime=500;         // we'll get it next time. Keep it responsive.
        }
        delay(delayTime);
        ServoList[i].detach();
      } 
      else {                                          // Otherwise if the reading hasn't changed enough write the old value to
        ServoList[i].attach(ServoPins[i]);            // the servo so that it will hold in place if it's applying pressure.
        delay(10);
        ServoList[i].write(oldLocation);
        delay(50);         
        ServoList[i].detach();   
        servoLocations[i]=oldLocation;
      }
    }
    delay(20); // Added to fix left sensor misbehavior reported by Rob.
  }

  latestReading++; // Increment the reading counter so we know where we're at.
  if (latestReading >= ReadingsPerSensor){  // Make sure we don't record more readings than we have space to hold.
    latestReading = ReadingsPerSensor-1;
    // Pop the oldest reading off the list.
    for (i = 0; i < SENSOR_NUM; i++){
      for (j=0; j < ReadingsPerSensor-1; j++){
        sensorReadings[i][j] = sensorReadings[i][j+1];
      }
    }
  }

}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

int calculateNewDistace(int sensorNumber){
  int output = SensorFar;                      // Default value is the furthest distance.

  float weightingFactor = 0.5;                 // How fast the reading's importance tapers off in time. (1= no taper, 0 = divide by zero error.)
  float flickerFactor = 30;                    // When the change is greater than this, ignore it unless its two in a row. (It's probably noise.)

  if (latestReading >= ReadingsPerSensor-1) {  // Only do this if we have a full set of readings to sample.
    int total = 0;                             // Average them with a weighting.
    float currentWeight = 1;                   // New readings count more than older readings.
    float percentagePossible = 0;
    boolean flickered = false;
    for (int i=ReadingsPerSensor-1; i >=0; i--){   // Check for flicker (This reduces jitter with something right on the threshold.)
      flickered = false;
      if (i==ReadingsPerSensor-1){
        if ((abs(sensorReadings[sensorNumber][i])-abs(sensorReadings[sensorNumber][i-1]) > flickerFactor) &&
          (abs(sensorReadings[sensorNumber][i-1])-abs(sensorReadings[sensorNumber][i-2]) > flickerFactor)){
          flickered = true;
        }
      }
      if (flickered==false){
        total += (sensorReadings[sensorNumber][i] * currentWeight);
        percentagePossible += currentWeight;
        currentWeight *= weightingFactor;
      }
    }
    output = total / percentagePossible;
  }
  return output;
}

int getDistance(int sensorNumber){
  long duration;   // How long it takes a sonic pulse to reflect back.
  int out;         // The value we send back from the function

  duration = sonar[sensorNumber].ping();

  // Trim the data into minimums and maximums and map it to the 0-100 output range.
  duration = constrain(duration, SensorClose, SensorFar);
  out = map(duration,  SensorClose, SensorFar, 0, 100);
#if _DEBUG_MODE
  if (sensorNumber == _DEBUG_SENSOR) {  //for debug reasons we track only one sensor on the debug terminal
    Serial.print("Time required by pulse to come back: ");
    Serial.println(duration);
    Serial.print("What will be send back: ");
    Serial.println(out);
  }
#endif
  
  //HC-SR04 Sensors return 10 when they scan nothing. We fix this by code and return the longest possible distance
  if (duration != 10) {
    return out;
  } else {
    return map(SensorFar, SensorClose, SensorFar, 0, 100);
  }
}

