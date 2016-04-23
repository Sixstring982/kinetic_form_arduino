 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/**
   Protocol:
   SERVO ADDRESS: 1b
   DURATION: 1b
   ... (many of these)
   STOP: 1b == 0xff
*/

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  325 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOSTOP 350
#define SERVOMAX  375 // this is the 'maximum' pulse length count (out of 4096)

#define LEFT_TIME 500
#define RIGHT_TIME 560

#define SERVO_COUNT 16
#define MAX_DISTANCE 5

enum SerialState {
  READ_INDEX,
  READ_DISTANCE
};

SerialState state = READ_INDEX;
uint8_t distances[SERVO_COUNT];
bool moving_left[SERVO_COUNT];
uint8_t queued_servos[SERVO_COUNT];
uint8_t queued_distances[SERVO_COUNT];
uint8_t queue_size = 0;

// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);

  pwm.begin();

  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  yield();

  for (int i = 0; i < SERVO_COUNT; i++) {
    distances[i] = 0;
    moving_left[i] = false;
    stop_servo(i);
  }
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period");
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit");
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void drive_left(uint8_t servoNum) {
  pwm.setPWM(servoNum, 0, SERVOMAX);
}

void drive_right(uint8_t servoNum) {
  pwm.setPWM(servoNum, 0, SERVOMIN);
}

void stop_servo(uint8_t servoNum) {
  pwm.setPWM(servoNum, 0, 350);
}

void stopAllServos() {
  /* Wait until all are ready to stop */
  delay(LEFT_TIME);
  /* Stop all servos moving left */
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    if (moving_left[i]) {
      stop_servo(i);
    }
  }
  /* Sleep the difference */
  delay(RIGHT_TIME - LEFT_TIME);
  /* Stop all right moving servos. */
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    if (!moving_left[i]) {
      stop_servo(i);
    }
  }
}

void flushServos() {
  bool done = false;
  uint8_t i;
  while (!done) {
    for (i = 0; i < queue_size; i++) {
      if (queued_distances[i] > 0) {
        /* Get this servo moving in the right direction */
        if (moving_left[i] && distances[i] == 0) {
          moving_left[i] = false;
        } else if (!moving_left[i] && distances[i] == MAX_DISTANCE) {
          moving_left[i] = true;
        }
        
        if (moving_left[i]) {
          drive_left(i);
          distances[i]--;
        } else {
          drive_right(i);
          distances[i]++;
        }
        queued_distances[i]--;
      }
    }
    
    /* Sync everything up at the next step */
    stopAllServos();
    
    /* If there aren't any servos to move, we're done. */
    done = true;
    for (i = 0; i < queue_size; i++) {
      if (queued_distances[i] > 0) {
        done = false;
        break;
      }
    }
  }
  queue_size = 0;
}

void loop() {
  serialLoop();
}

/*
void fauxLoop() {
  uint8_t i;
  for (i = 0; i < 
}
*/

void serialLoop() {
  if (Serial.available() > 0) {
    byte r = Serial.read();
    switch (state) {
      case READ_INDEX:
        if (r != 0xff) {
          queued_servos[queue_size] = r;
          Serial.print("Servo index: ");
          Serial.println(r);
          state = READ_DISTANCE;
        } else {
          Serial.println("Flusing...");
          flushServos();
        }
        break;
      case READ_DISTANCE:
        queued_distances[queue_size++] = r;
        Serial.print("Servo distance: ");
        Serial.println(r);
        state = READ_INDEX;
        break;
    }
  }
}

