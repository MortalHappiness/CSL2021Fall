/***************************************************************/
/* Sample code for CSL final project                           */
/* Demonstrating 1) IR sensor, 2) Servo motor, and 3) DC Motor */
/***************************************************************/

enum Mode {
  STRAIGHT,
  S_SHAPED,
  SENSOR,
  DEBUG,
};
enum Color {
  WHITE,
  BLACK,
  GRAY,
};
enum DIRECTION {
  FORWARD,
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT,
  STOP,
};

// Power your board by connecting 9V battery to Vin pin on Arduino
// Connect the gounds (GND pin) of all of the parts & Arduino

/***** IR Sensor *****/
// #define ir_sensor A2 // A0 pin
// Vcc -> 5V
// GND -> GND
#define ir_front_left A2
#define ir_front_center A3
#define ir_front_right A4
#define ir_back_left A7
#define ir_back_center A6
#define ir_back_right A5

Color front_left_color = GRAY;
Color front_center_color = GRAY;
Color front_right_color = GRAY;
Color back_left_color = GRAY;
Color back_center_color = GRAY;
Color back_right_color = GRAY;


/***** Servo Motor *****/
#define servo_pin 11 // Orange pin (pwm)
// Red pin -> 5V
// Brown pin -> GND

#include <Servo.h>
Servo myservo;
// #define SERVO_LEFT 70
#define SERVO_LEFT 60
#define SERVO_CENTER 87
// #define SERVO_RIGHT 104
#define SERVO_RIGHT 114
int servo_output = SERVO_CENTER;


/***** DC Motor (L298N) *****/
#define ENA 5 // (pwm)
#define IN1 6
#define IN2 7
// +12V -> (+) of 9V battery
// GND -> GND
// OUT1 & OUT2 -> DC Motor

#define DC_LOW 100
#define DC_NORMAL 190
// #define DC_HIGH 210
#define DC_HIGH 225
#define DC_MAX 255
#define DIR_FORWARD 1
#define DIR_BACKWARD 0
int dc_dir = DIR_FORWARD;
int dc_output = DC_HIGH;

/***** Custom global variables *****/
#define DELAY 50
#define STOP_TIME 3000
#define WHITE_THRESHOLD 150
#define BACK_THRESHOLD 650

// Mode mode = STRAIGHT;
// Mode mode = S_SHAPED;
Mode mode = SENSOR;
// Mode mode = DEBUG;

int timer = 0;
DIRECTION prev_direction = FORWARD;
#define BACKWARD_COUNT_THRESHOLD 5
int backward_counter = 0;
#define CENTER_GRAY_COUNT_THRESHOLD 5
int center_gray_counter = 0;
#define DC_MAX_COUNTER 5
int dc_max_counter = 0;

/* Record number of flips to pause */
int color_flips = 0;
int pause_timer = STOP_TIME;
Color last_center_color = GRAY;


void setup() {
  /****** IR Sensor ******/
  //pinMode(ir_sensor, INPUT);
  pinMode(ir_front_left, INPUT);
  pinMode(ir_front_center, INPUT);
  pinMode(ir_front_right, INPUT);
  pinMode(ir_back_left, INPUT);
  pinMode(ir_back_center, INPUT);
  pinMode(ir_back_right, INPUT);

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
  /* Read IR Sensor values */
  int ir_front_left_val = analogRead(ir_front_left);
  int ir_front_center_val = analogRead(ir_front_center);
  int ir_front_right_val = analogRead(ir_front_right);
  int ir_back_left_val = analogRead(ir_back_left);
  int ir_back_center_val = analogRead(ir_back_center);
  int ir_back_right_val = analogRead(ir_back_right);

  front_left_color = value_to_color(ir_front_left_val);
  front_center_color = value_to_color(ir_front_center_val);
  front_right_color = value_to_color(ir_front_right_val);
  back_left_color = value_to_color(ir_back_left_val);
  back_center_color = value_to_color(ir_back_center_val);
  back_right_color = value_to_color(ir_back_right_val);

  switch (mode) {
    case DEBUG:
      Serial.print("ir_front_left_val = ");
      Serial.println(ir_front_left_val);
      Serial.print("ir_front_center_val = ");
      Serial.println(ir_front_center_val);
      Serial.print("ir_front_right_val = ");
      Serial.println(ir_front_right_val);
      Serial.print("ir_back_left_val = ");
      Serial.println(ir_back_left_val);
      Serial.print("ir_back_center_val = ");
      Serial.println(ir_back_center_val);
      Serial.print("ir_back_right_val = ");
      Serial.println(ir_back_right_val);
      Serial.print("color_flips = ");
      Serial.println(color_flips);
      Serial.print("pause_timer = ");
      Serial.println(pause_timer);
      Serial.println();
      forward();
      delay(500);
      break;
    case SENSOR:
      make_decision();
      break;
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

void make_decision() {
  if (back_center_color != GRAY) {
    if (prev_direction != STOP && center_gray_counter < CENTER_GRAY_COUNT_THRESHOLD && (back_center_color == WHITE) && (last_center_color != WHITE)) {
      if (prev_direction == BACKWARD) color_flips--;
      else color_flips++;
    }
    center_gray_counter = 0;
  } else if (prev_direction != STOP) {
    center_gray_counter++;
  }
  last_center_color = back_center_color;

  if (color_flips >= 8){
    color_flips = 0;
    stop();
    pause_timer = DELAY;
    return;
  }
  if (pause_timer < STOP_TIME){
    stop();
    pause_timer += DELAY;
    return;
  }

  if (prev_direction == BACKWARD) {
    if (front_center_color != GRAY) {
      forward();
    } else {
      backward();
    }
    return;
  }
  
  if (front_left_color != GRAY) {
    turn_left();
  } else if (front_right_color != GRAY) {
    turn_right();
  } else if (front_center_color != GRAY) {
    forward();
  } else { // All 3 sensors in front are gray
    if (back_left_color != GRAY) {
      turn_left();
    } else if (back_right_color != GRAY) {
      turn_right();
    } else if (front_center_color != GRAY){
      forward();
    } else {
      if (backward_counter < BACKWARD_COUNT_THRESHOLD ) {
        backward_counter++;
      } else {
        backward_counter = 0;
        backward();
      }
    }
  }
}

void forward() {
  servo_output = SERVO_CENTER;
  dc_output = DC_NORMAL;
  dc_dir = DIR_FORWARD;
  prev_direction = FORWARD;
}

void turn_left() {
  servo_output = SERVO_LEFT;
  if (prev_direction == STOP) {
    dc_max_counter = DC_MAX_COUNTER;
  }
  if (dc_max_counter > 0) {
    dc_output = DC_MAX;
    dc_max_counter--;
  } else {
    dc_output = DC_HIGH;
  }
  dc_dir = DIR_FORWARD;
  prev_direction = TURN_LEFT;
}

void turn_right() {
  servo_output = SERVO_RIGHT;
  if (prev_direction == STOP) {
    dc_max_counter = DC_MAX_COUNTER;
  }
  if (dc_max_counter > 0) {
    dc_output = DC_MAX;
    dc_max_counter--;
  } else {
    dc_output = DC_HIGH;
  }
  dc_dir = DIR_FORWARD;
  prev_direction = TURN_RIGHT;
}

void backward() {
  servo_output = SERVO_CENTER;
  dc_output = DC_HIGH;
  dc_dir = DIR_BACKWARD;
  prev_direction = BACKWARD;
}

void stop() {
  if (dc_output > DC_NORMAL) {
    dc_output = 140;
  } else {
    dc_output = 100;
  }
  dc_dir = DIR_BACKWARD;
  prev_direction = STOP;
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
  if (val < WHITE_THRESHOLD) return WHITE;
  if (val < BACK_THRESHOLD) return GRAY;
  return BLACK;
}
