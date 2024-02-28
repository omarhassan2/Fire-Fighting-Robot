/**
 * @brief   Firefighting Robot 🔥🤖 operates in two modes (Manual and Autonomous)
 * @author  Group C | IEEE-RAS-ZSC 24 ❤️
 * @date    February 24, 2024 
*/

/*************** Includes Section ***************/
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <Servo.h>
/*************** Macro Defentions Section ***************/
#define LCD_RS                    (13)
#define LCD_EN                    (12)
#define LCD_D4                    (11)
#define LCD_D5                    (10)
#define LCD_D6                    (9)
#define LCD_D7                    (8)

#define ALARM_LED                 (7)
#define ALARM_BUZ                 (6)

#define MOTOR_A_IN1               (5)
#define MOTOR_A_IN2               (4)
#define MOTOR_B_IN1               (3)
#define MOTOR_B_IN2               (2)

#define BLUETOOTH_RX              (1)
#define BLUETOOTH_TX              (0)

#define FLAME_SENSOR_FORWARD      (A0)
#define FLAME_SENSOR_RIGHT        (A1)
#define FLAME_SENSOR_LEFT         (A2)

#define NO_FLAME                  (0)
#define LEFT_FLAME                (1)
#define RIGHT_FLAME               (2)
#define FORWARD_FLAME             (3)

#define PUMP_RELAY_PIN            (A3)
#define SERVO_PIPE_PIN            (A4)


/*************** Function Decleration Section ***************/
void moveForward();
void moveBack();
void turnLeft();
void turnRight();
void stop();;
uint8_t detectFireDirection();
void movePipe();
void firefighting();


/*************** Global Decleration Section ***************/
LiquidCrystal LCD(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

SoftwareSerial Bluetooth_Serial(BLUETOOTH_TX, BLUETOOTH_RX);

Servo Pipe;


/*************** Setup Application Section ***************/
void setup() {
  LCD.begin(16, 2);

  Bluetooth_Serial.begin(9600);

  Pipe.attach(SERVO_PIPE_PIN);
  Pipe.write(90);
}


/*************** Start Application Section ***************/
void loop() {
  
}


/*************** Function Defentions Section ***************/
void PrintLCD(bool STATE, String DIRECTION){
  LCD.setCursor(0, 0);
  LCD.print("Pump State: ");
  LCD.print(STATE ? "ON" : "OFF");

  LCD.setCursor(0, 1);
  LCD.print("Fire: ");
  LCD.print(DIRECTION);
}


void moveForward(){
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
}


void moveBack(){
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
}


void turnLeft(){
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
}


void turnRight(){
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
}


void stop(){
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
}


uint8_t detectFireDirection(){
  int left = digitalRead(FLAME_SENSOR_LEFT);
  int right = digitalRead(FLAME_SENSOR_RIGHT);
  int forward = digitalRead(FLAME_SENSOR_FORWARD);

  /* Priority : Forward > Right > Left */
  if (forward == HIGH){
    return FORWARD_FLAME;
  }
  else if (right == HIGH){
    return RIGHT_FLAME;
  } 
  else if (left == HIGH){
    return LEFT_FLAME;
  } 
  else{
    return NO_FLAME;
  }
}


void movePipe(){
  for (int pos = 50; pos <= 130; pos += 1){
    Pipe.write(pos);
    delay(10);
  }
  for (int pos = 130; pos >= 50; pos -= 1){
    Pipe.write(pos);
    delay(10);
  }
}


void firefighting(){
  digitalWrite(PUMP_RELAY_PIN, HIGH);
  movePipe();  
  digitalWrite(PUMP_RELAY_PIN, LOW);
}