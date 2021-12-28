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
int left_state, center_state, right_state;


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
int dc_output = 0;

int timer = 0;


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
  /* Read IR Sensor value */
  //Serial.println(analogRead(ir_sensor));
  /*
  Serial.println(left_state = analogRead(ir_left));
  Serial.println(center_state = analogRead(ir_center));
  Serial.println(right_state = analogRead(ir_right));
  Serial.println();
  */
  /*
  if((left_state > 800 || left_state < 100) && (right_state < 800 && right_state > 100)){
    // turn right
    if(servo_output <= 130) servo_output += 5;
    myservo.write(servo_output);
  }

  if((left_state < 800 && left_state > 100) && (right_state > 800 || right_state << 100)){
    // turn left
    if(servo_output >= 50) servo_output -= 5;
    myservo.write(servo_output);
  }
  */

  /* Rotate servo motor according to the input (degree) */
  //myservo.write((servo_output += 10) % 180);
  //myservo.write(((servo_output += 10) % 120) + 30);
  if(timer > 2000 && timer <= 5000){
    myservo.write(servo_output = 70);
  }
  else{
    myservo.write(servo_output = 91);
  }
  if(timer > 5000) timer = 0;
  /* Set DC motor direction and power */
  setDirection(dc_dir);
  //analogWrite(ENA, (dc_output += 63) % 255);
  analogWrite(ENA,  150);

  timer += 500;
  delay(500);
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
