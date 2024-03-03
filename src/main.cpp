/**
 * @brief   Firefighting Robot 🔥🤖 operates in two modes (Manual and Autonomous)
 * @author  Group C | IEEE-RAS-ZSC 24 ❤️
 * @date    February 24, 2024 
*/


/*************** Includes Section ***************/
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
/************************************************************************/


/*************** Macro Defentions Section ***************/
/* Alaram System */
#define ALARM_RED_LED             (2)
#define ALARM_BUZZER              (3)

/* Active Water Pump with control pipe direction */
#define PUMP_RELAY_PIN            (4)
#define SERVO_PIPE_PIN            (5)

/* Flame Sensors */
#define FLAME_SENSOR_RIGHT        (7)
#define FLAME_SENSOR_FORWARD      (8)
#define FLAME_SENSOR_LEFT         (9)

/* Motors Driver */
#define MOTOR_A_IN1               (10)
#define MOTOR_A_IN2               (11)
#define MOTOR_B_IN1               (13)
#define MOTOR_B_IN2               (12)

/* LCD for Monitoring System */
#define LCD_SDA                   (A4)
#define LCD_SCL                   (A5)

/* Flags to detect fire direction */
#define NO_FLAME                  (0)
#define LEFT_FLAME                (1)
#define RIGHT_FLAME               (2)
#define FORWARD_FLAME             (3)
/************************************************************************/


/*************** Function Decleration Section ***************/
/* Motor Driver Functions */
void moveForward();
void moveBack();
void turnLeft();
void turnRight();
void stop();

/* LCD Functions */
void PrintLCD(bool STATE, String DIRECTION);

/* Autonomous mode functions */
uint8_t detectFireDirection();
void movePipe();
void firefighting();
void AutonomousFirefightingMode();

void RemoteControlMode();
void ActivePump();
/************************************************************************/


/*************** Global Decleration Section ***************/
Servo Pipe;
// LiquidCrystal_I2C lcd(0x27, 16, 2);
char controlSignal;
bool autonomousModeFlag = true;
bool activePumpFlag = true;
/************************************************************************/


/*************** Setup Application Section ***************/
void setup(){
  // lcd.begin(16, 2);         
  // lcd.backlight(); 

  pinMode(ALARM_RED_LED, OUTPUT);
  pinMode(ALARM_BUZZER, OUTPUT);

  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  pinMode(FLAME_SENSOR_FORWARD, INPUT);
  pinMode(FLAME_SENSOR_RIGHT, INPUT);
  pinMode(FLAME_SENSOR_LEFT, INPUT);

  pinMode(PUMP_RELAY_PIN, OUTPUT);

  Serial.begin(9600);

  Pipe.attach(SERVO_PIPE_PIN);
  Pipe.write(45);  /* Forward */ 
}
/************************************************************************/


/*************** Start Application Section ***************/
void loop(){
  firefighting();
}


/*************** Function Defentions Section ***************/
void PrintLCD(bool STATE, String DIRECTION) {
  // lcd.clear();
  // lcd.setCursor(0, 0);
  // lcd.print("Pump State: ");
  // lcd.print(STATE ? "ON" : "OFF");

  // lcd.setCursor(0, 1);
  // lcd.print("Fire: ");
  // lcd.print(DIRECTION);
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
  digitalWrite(MOTOR_A_IN1, HIGH);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, HIGH);
}


void turnRight(){
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN1, HIGH);
  digitalWrite(MOTOR_B_IN2, LOW);
}


void stop(){
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
}


uint8_t detectFireDirection(){
  
}


void movePipe(){
  for(int i = 0; i < 3; i++){
    Pipe.write(90);
    delay(200);
    Pipe.write(0);
  }
}


void firefighting(){
  digitalWrite(PUMP_RELAY_PIN, HIGH);
  movePipe();
  digitalWrite(PUMP_RELAY_PIN, LOW);
}


void RemoteControlMode(){
  while (Serial.available()) {
    controlSignal = Serial.read();
    Serial.println(controlSignal);
    delay(5);
    if (controlSignal == 'F') {
      // PrintLCD(false, "Forward");
      moveForward();
      delay(10);
    }
    if (controlSignal == 'R') {
      // PrintLCD(false, "RIGHT");
      turnRight();
      delay(10);
    }
    if (controlSignal == 'L') {
      // PrintLCD(false, "LEFT");
      turnLeft();
      delay(10);
    }
    if (controlSignal == 'B') {
      // PrintLCD(false, "BACK");
      moveBack();
      delay(5);
    }
    if (controlSignal == 'S') {
      // PrintLCD(false, "STOP");
      stop();
      delay(10);
    }
    if (controlSignal == 'M') {
      if(activePumpFlag){
        ActivePump();
      }
      activePumpFlag = !activePumpFlag;
    }
    if (controlSignal == 'N') {
      autonomousModeFlag = !autonomousModeFlag;
    }
  }
}


void AutonomousFirefightingMode(){
  uint8_t fireDirection = detectFireDirection();
  switch (fireDirection) {
    case FORWARD_FLAME:
      firefighting();
      PrintLCD(true, "Forward");
      break;
    case RIGHT_FLAME:
      turnRight();
      delay(500);
      firefighting();
      PrintLCD(true, "Right");
      break;
    case LEFT_FLAME:
      turnLeft();
      delay(500);
      firefighting();
      PrintLCD(true, "Left");
      break;
    case NO_FLAME:
      stop();
      PrintLCD(false, "No Fire");
      break;
  }
}

void ActivePump(){
  digitalWrite(PUMP_RELAY_PIN, HIGH);
  delay(2000);
  digitalWrite(PUMP_RELAY_PIN, LOW);
}