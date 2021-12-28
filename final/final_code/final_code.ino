/***************************************************************/
/* Sample code for CSL final project                           */
/* Demonstrating 1) IR sensor, 2) Servo motor, and 3) DC Motor */
/***************************************************************/

// Power your board by connecting 9V battery to Vin pin on Arduino
// Connect the gounds (GND pin) of all of the parts & Arduino

/***** IR Sensor *****/
// #define ir_sensor A2 // A0 pin
// Vcc -> 5V
// GND -> GND
#define ir_left A2
#define ir_center A3
#define ir_right A4


/***** Servo Motor *****/
#define servo_pin 11 // Orange pin (pwm)
// Red pin -> 5V
// Brown pin -> GND

#include <Servo.h>
Servo myservo;
int servo_output = 0;


/***** DC Motor (L298N) *****/
#define ENA 5 // (pwm)
#define IN1 6
#define IN2 7
// +12V -> (+) of 9V battery
// GND -> GND
// OUT1 & OUT2 -> DC Motor

int dc_dir = 1;
int dc_output = 180;

/***** Custom global variables *****/
int DELAY = 500;
enum Mode {
  STRAIGHT,
  S_SHAPED,
  SENSOR
};
int timer = 0;
// Mode mode = STRAIGHT;
Mode mode = S_SHAPED;


void setup() {
  /****** IR Sensor ******/
  //pinMode(ir_sensor, INPUT);
  pinMode(ir_left, INPUT);
  pinMode(ir_center, INPUT);
  pinMode(ir_right, INPUT);

  /***** Servo Motor *****/
  pinMode(servo_pin, OUTPUT);
  myservo.attach(servo_pin);

  /** DC Motor (L298N) ***/
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  setDirection(dc_dir);
  analogWrite(ENA, dc_output);

  Serial.begin(38400);
}

void loop() {
  int ir_left_val, ir_center_val, ir_right_val;
  /* Read IR Sensor value */
  ir_left_val = analogRead(ir_left);
  ir_center_val = analogRead(ir_center);
  ir_right_val = analogRead(ir_right);

  switch (mode) {
    case STRAIGHT:
      forward();
      break;
    case S_SHAPED:
      int interval = 2000;
      if (timer >= 4 * interval) {
        timer = 0;
      }
      if (timer < interval) {
        forward();
      } else if (timer < 2 * interval) {
        turn_left();
      } else if (timer < 3 * interval) {
        forward();
      } else {
        turn_right();
      }
      break;
    case SENSOR:
      Serial.println(ir_left_val);
      Serial.println(ir_center_val);
      Serial.println(ir_right_val);
      Serial.println();
      break;
    default:
      break;
  }

  /* Rotate servo motor according to the input (degree) */
  myservo.write(servo_output);

  /* Set DC motor direction and power */
  setDirection(dc_dir);
  analogWrite(ENA,  dc_output);

  timer += DELAY;
  delay(DELAY);
}

void forward() {
  servo_output = 91;
  dc_output = 180;
}

void turn_left() {
  servo_output = 70;
  dc_output = 200;
}

void turn_right() {
  servo_output = 115;
  dc_output = 200;
}

void setDirection(int dir) {
  if (dir == 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (dir == 1) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}
