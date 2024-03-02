/**
 * @brief   Firefighting Robot 🔥🤖 operates in two modes (Manual and Autonomous)
 * @author  Group C | IEEE-RAS-ZSC 24 ❤️
 * @date    February 24, 2024 
*/

/*************** Includes Section ***************/
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
// #include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
/*************** Macro Defentions Section ***************/
#define LCD_SDA                   (A4)
#define LCD_SCL                   (A5)

#define ALARM_RED_LED             (2)
#define ALARM_BUZZER              (3)

#define MOTOR_A_IN1               (10)
#define MOTOR_A_IN2               (11)
#define MOTOR_B_IN1               (13)
#define MOTOR_B_IN2               (12)

// #define BLUETOOTH_RX              (4)
// #define BLUETOOTH_TX              (5)

#define FLAME_SENSOR_FORWARD      (8)
#define FLAME_SENSOR_RIGHT        (7)
#define FLAME_SENSOR_LEFT         (9)

#define FLAME_READ_FORWARD        (A0)
#define FLAME_READ_RIGHT          (A1)
#define FLAME_READ_LEFT           (A2)

#define NO_FLAME                  (0)
#define LEFT_FLAME                (1)
#define RIGHT_FLAME               (2)
#define FORWARD_FLAME             (3)

#define PUMP_RELAY_PIN            (4)
#define SERVO_PIPE_PIN            (5)


/*************** Function Decleration Section ***************/
void moveForward();
void moveBack();
void turnLeft();
void turnRight();
void stop();
void PrintLCD(bool STATE, String DIRECTION);
// void startAlarmSystem();
// void stopAlarmSystem();
uint8_t detectFireDirection();
void movePipe();
void firefighting();
void RemoteControlMode();
void AutonomousFirefightingMode();


/*************** Global Decleration Section ***************/
// LiquidCrystal LCD(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
// SoftwareSerial Bluetooth_Serial(BLUETOOTH_TX, BLUETOOTH_RX);
Servo Pipe;

LiquidCrystal_I2C lcd(0x27, 16, 2);

char controlSignal;

bool autonomousMode = false;

/*************** Setup Application Section ***************/
void setup(){
  // LCD.begin(16, 2);
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
  // Bluetooth_Serial.begin(9600);
  Serial.begin(9600);
  Pipe.attach(SERVO_PIPE_PIN);
  Pipe.write(90);

  lcd.init();         
  lcd.backlight();    
}


/*************** Start Application Section ***************/
void loop(){
  RemoteControlMode();
}


/*************** Function Defentions Section ***************/
void PrintLCD(bool STATE, String DIRECTION) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pump State: ");
  lcd.print(STATE ? "ON" : "OFF");

  lcd.setCursor(0, 1);
  lcd.print("Fire: ");
  lcd.print(DIRECTION);
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


// void startAlarmSystem(){
//   digitalWrite(ALARM_RED_LED, HIGH); 
//   digitalWrite(ALARM_BUZZER, HIGH); 
//   delay(100);
    
//   digitalWrite(ALARM_RED_LED, LOW); 
//   digitalWrite(ALARM_BUZZER, LOW); 
//   delay(100);  
// }


// void stopAlarmSystem(){
//   digitalWrite(ALARM_RED_LED, LOW); 
//   digitalWrite(ALARM_BUZZER, LOW); 
// }


void RemoteControlMode() {
  while (Serial.available()) {
    controlSignal = Serial.read();
    Serial.println(controlSignal);
    delay(5);
    if (controlSignal == 'F') {
      PrintLCD(false, "Forward");
      moveForward();
      delay(10);
    }
    if (controlSignal == 'R') {
      PrintLCD(false, "RIGHT");
      turnRight();
      delay(10);
    }
    if (controlSignal == 'L') {
      PrintLCD(false, "LEFT");
      turnLeft();
      delay(10);
    }
    if (controlSignal == 'G') {
      PrintLCD(false, "BACK");
      moveBack();
      delay(5);
    }
    if (controlSignal == 'S') {
      PrintLCD(false, "STOP");
      stop();
      delay(10);
    }
    if (controlSignal == 'X') {
      firefighting();
    }
    if (controlSignal == 'M') {
      autonomousMode = false;
      PrintLCD(false, "Manual Mode");
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