//--------------------------------
// @file
// Sonar Glove
// Tactile feedback for the blind
// for HC-SR04 Ultrasonic Rangefinders
// adapted to work with the NewPing library which allows very fast scanning
// and use Vibration Motors for smaller and quiter footprint.
// by Bill Seremetis (bill@seremetis.net)
// 2012 November
// BRANCH: Two Vibration Motors (two-vibras)
//--------------------------------

#define _DEBUG_MODE 1
#define _DEBUG_SENSOR 1 //either 0 or 1

#define VIBRATOR_MAX_PWM 195 //set PWM output to match max voltage of motor. 195 equals 3.8V. CAREFULL NOT TO BURN THINGS!!!
#define VIBRATOR_MIN_PWM 0  //45 equals 0.8V

#include <NewPing.h>

#define SENSOR_NUM     2 // Number or sensors.
#define MAX_DISTANCE 150 // Maximum distance (in cm) to ping. Our sensors go to 400cm, but without much accuracy.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SENSOR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SENSOR_NUM];         // Where the ping distances are stored.
byte currentSensor = 0;          // Keeps track of which sensor is active.

NewPing sonar[SENSOR_NUM] = {     // Sensor object array.
  NewPing(6, 7, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(8, 9, MAX_DISTANCE)
};

const int vibrators[2] = { 11, 5 }; //Vibrator PIN, must be a PWM pin.

const int SensorClose = 10;                    // Closest value we detect with the PING sensor. (Soundwave travel time in milliseconds.)
const int SensorFar = 14000;                   // Furthest distance we register on the PING sensor. (Soundwave travel time in milliseconds.)
const int ReadingsPerSensor = 3;               // The number of historic readings to consider when determining position.

int sensorReadings[SENSOR_NUM][ReadingsPerSensor];   // Hold past readings for each sensor.
int calculatedSensorReadings[SENSOR_NUM];             // The calculated distance for each sensor.
int latestReading = 0;                               // Current position in the array for the most recent reading.

void setup() {
  Serial.begin(9600);
  Serial.println("Tacit Glove Project\r\nBOOT");
  
  for (int i = 0; i < SENSOR_NUM; i++) {
    pinMode(vibrators[i], OUTPUT);
  }

  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SENSOR_NUM; i++) {
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;  // Set the starting time for each sensor.
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


    Serial.println(calculatedSensorReadings[i]);
    if (calculatedSensorReadings[i] <= VIBRATOR_MAX_PWM) {
      analogWrite(vibrators[i], calculatedSensorReadings[i]);
    }

    //delay(20); // Added to fix left sensor misbehavior
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

  // Trim the data into minimums and maximums and map it to the output range of our vibrator voltage (PWM).
  duration = constrain(duration, SensorClose, SensorFar);
  out = map(duration,  SensorClose, SensorFar, VIBRATOR_MAX_PWM, VIBRATOR_MIN_PWM);
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
    return map(SensorFar, SensorClose, SensorFar, VIBRATOR_MAX_PWM, VIBRATOR_MIN_PWM);
  }
}

