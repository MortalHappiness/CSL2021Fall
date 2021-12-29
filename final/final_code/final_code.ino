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
#define SERVO_LEFT 81
#define SERVO_CENTER 91
#define SERVO_RIGHT 101
int servo_output = SERVO_CENTER;


/***** DC Motor (L298N) *****/
#define ENA 5 // (pwm)
#define IN1 6
#define IN2 7
// +12V -> (+) of 9V battery
// GND -> GND
// OUT1 & OUT2 -> DC Motor

#define DC_NORMAL 170
#define DC_HIGH 190
#define DIR_FORWARD 1
#define DIR_BACKWARD 0
int dc_dir = DIR_FORWARD;
int dc_output = DC_NORMAL;

/***** Custom global variables *****/
enum Mode {
  STRAIGHT,
  S_SHAPED,
  SENSOR,
};
enum Color {
  WHITE,
  BLACK,
  GRAY,
};

// Print debug message
#define DEBUG
// Do not stop for debuging
#define NO_STOP
#define STOP_TIME 3000

int DELAY = 100;
int timer = 0;

// Mode mode = STRAIGHT;
// Mode mode = S_SHAPED;
Mode mode = SENSOR;


/* Record number of flips to pause */
int color_flips = 0;
int pause_timer = STOP_TIME;
Color last_center_color = GRAY;


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
    case SENSOR:
#ifdef DEBUG
      Serial.print("ir_left_val = ");
      Serial.println(ir_left_val);
      Serial.print("ir_center_val = ");
      Serial.println(ir_center_val);
      Serial.print("ir_right_val = ");
      Serial.println(ir_right_val);
      Serial.print("color_flips = ");
      Serial.println(color_flips);
      Serial.print("pause_timer = ");
      Serial.println(pause_timer);
      Serial.println();
#endif

      Color left_color, center_color, right_color;
      left_color = value_to_color(ir_left_val);
      center_color = value_to_color(ir_center_val);
      right_color = value_to_color(ir_right_val);

      if (center_color != last_center_color) color_flips++;
      last_center_color = center_color;

      if (color_flips >= 20){
        color_flips = 0;
#ifndef NO_STOP
        stop();
#endif
        pause_timer = DELAY;
      } else if (pause_timer < STOP_TIME){
#ifndef NO_STOP
        stop();
#endif
        pause_timer += DELAY;
      } else if (center_color != GRAY) {
        forward();
      } else if (left_color != GRAY) {
        turn_left();
      } else if (right_color != GRAY) {
        turn_right();
      } else {
        stop();
      }

      break;
    case STRAIGHT:
      forward();
      break;
    case S_SHAPED:
      Serial.println(timer);
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
  servo_output = SERVO_CENTER;
  dc_output = DC_NORMAL;
}

void turn_left() {
  servo_output = SERVO_LEFT;
  dc_output = DC_HIGH;
}

void turn_right() {
  servo_output = SERVO_RIGHT;
  dc_output = DC_HIGH;
}

void stop() {
  dc_output = 0;
}

void kickstart() {
}

void setDirection(int dir) {
  if (dir == DIR_BACKWARD) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (dir == DIR_FORWARD) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}

Color value_to_color(int val) {
  if (val < 100) return WHITE;
  if (val < 600) return GRAY;
  return BLACK;
}
