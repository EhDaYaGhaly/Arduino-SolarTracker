#include <IRremote.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
// print hello world
//define IR receiver pin
#define ir_pin 3  //Analog

//define motor pins
#define MotorH_pin1 2
#define MotorH_pin2 4
#define MotorH_enable 5  //Analog
#define MotorV_pin1 7
#define MotorV_pin2 8
#define MotorV_enable 6  //Analog
#define LDRV A2         //Analog
#define LDRH A3           //Analog

//define remote buttons
#define button1 0xBA45FF00 //auto
#define button2 0xB946FF00 //manual
#define buttonUp 0xE718FF00
#define buttonDown 0xAD52FF00
#define buttonRight 0xA55AFF00
#define buttonLeft 0xF708FF00


LiquidCrystal_I2C lcd(0x27, 16, 2);
IRrecv receiver(ir_pin);


void MotorUp();
void MotorDown();
void MotorRight();
void MotorLeft();
void printVolts();
void IRreceiver();
void SpeedControl(int value);
void printVolts();
void lcdupdate(String msg);
int speed = 170;
int delayy = 50;
unsigned long key_value = 0;
bool auto_mode = false;

void setup() {
  Serial.begin(9600);
  receiver.enableIRIn();  // Enable IR reception
  pinMode(ir_pin, INPUT);
  lcd.init();
  lcd.backlight();
  digitalWrite(MotorV_pin1, LOW);
  digitalWrite(MotorV_pin2, LOW);
  digitalWrite(MotorH_pin1, LOW);
  digitalWrite(MotorH_pin2, LOW);
}


void loop() {

  IRreceiver();
  // printVolts();
}

void IRreceiver() {
  if (receiver.decode()) {
    long code = receiver.decodedIRData.decodedRawData;
    //Serial.println("Code");
    Serial.println(code, HEX);
    if (!code){
      code = key_value;
      speed += 10;
      delayy += 20;
      Serial.println(speed);
      }
      else{
        speed = 170;
        delayy = 20;
      }
    //Serial.println("KeyValue");
    //Serial.println(code, HEX);
    if (code == button1) {
      Serial.println("AutoMode On");
      auto_mode = true;
    } else if (code == button2) {
      Serial.println("ManualMode On");
      speed = 170;
      delayy = 50;
      auto_mode = false;
    }
    if (!auto_mode)
      Manual_mode(code);
    receiver.resume();
  }
  if (auto_mode) {
    Auto_mode();
  }
}
//////////////////
void MotorUp() {
  analogWrite(MotorV_enable, speed);
  digitalWrite(MotorV_pin1, HIGH);
  digitalWrite(MotorV_pin2, LOW);
  delay(delayy);
  digitalWrite(MotorV_pin1, LOW);
}
void MotorDown() {
  analogWrite(MotorV_enable, speed);
  digitalWrite(MotorV_pin1, LOW);
  digitalWrite(MotorV_pin2, HIGH);
  delay(delayy);
  digitalWrite(MotorV_pin2, LOW);
}
void MotorRight() {
  analogWrite(MotorH_enable, speed);
  digitalWrite(MotorH_pin1, HIGH);
  digitalWrite(MotorH_pin2, LOW);
  delay(delayy);
  digitalWrite(MotorH_pin1, LOW);
}
void MotorLeft() {
  analogWrite(MotorH_enable, speed);
  digitalWrite(MotorH_pin1, LOW);
  digitalWrite(MotorH_pin2, HIGH);

  delay(delayy);
  digitalWrite(MotorH_pin2, LOW);
}

void printVolts() {
  int sensorValue = analogRead(A0);
  float Voltage = sensorValue * (12.0 / 1023.0);  // Convert the sensor value to true voltage
  lcd.setCursor(0, 0);
  lcd.print("Voltage =");
  lcd.print(Voltage);  // Print the Voltage on LCD
}
void lcdupdate(String msg){
    lcd.clear();
    lcd.print(msg);
    delay(1500);
    lcd.clear();
    printVolts();
}

void Manual_mode(long code) {
  switch (code) {
    case buttonUp:
      MotorUp();
      Serial.println("Up");
      lcdupdate("Up");
      break;
    case buttonDown:
      MotorDown();
      Serial.println("Down");
      lcdupdate("Up");
      break;
    case buttonLeft:
      MotorLeft();
      Serial.println("Left");
      lcdupdate("Up");
      break;
    case buttonRight:
      MotorRight();
      Serial.println("Right");
      lcdupdate("Up");
      break;
  }
  key_value = code;
}

void Auto_mode() {

  int v_value = analogRead(LDRV);
  int h_value = analogRead(LDRH);

  
  Serial.print("v_value: ");
  Serial.println(v_value);
  SpeedControl(v_value); 
  Serial.print("speed: ");
  Serial.println(delayy);
  if (v_value < 250) {
    MotorUp();
  } else if (v_value > 750) {
    MotorDown();
  }
  Serial.print("h_value: ");
  Serial.println(h_value);
  SpeedControl(h_value);

 
  if (h_value < 250) {
    MotorLeft();
    
  } else if (h_value > 750) {
    MotorRight();
     
  }
}

void SpeedControl(int value) {
  int x = abs(value - 511);
  if (x < 150)
    speed = 0;
  else if ( x < 250)
    speed = 50;
  else if (x < 350)
    speed = 130;
  else
    speed = 170;
}