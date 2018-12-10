//#include <characters.h>
//#include <phi_super_font.h>
//#include <LCD.h>
//#include <LiquidCrystal_I2C.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// defines for LCD_I2C
#define I2C_ADDR 0x27
#define BACKLIGHT_PIN 3
#define En_pin 2
#define Rw_pin 1
#define Rs_pin 0
#define D4_pin 4
#define D5_pin 5
#define D6_pin 6
#define D7_pin 7

LiquidCrystal_I2C lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);

int M1dirpin = 7;  //Motor X direction pin
int M1steppin = 6; //Motor X step pin
int M1en = 8; //Motor X enable pin
int M2dirpin = 4;  //Motor Y direction pin
int M2steppin = 5; //Motor Y step pin
int M2en = 12; //Motor Y enable pin

const int potPin = A0; // Change this if pot is connected to another pin
const int robotDO14 = A3; // This is the voltage regualted DO_14 signal from the robot. Hardwire to 3.3V if not connected to the robot.
const int robotDO15 = A2; // This is the voltage regualted DO_15 signal from the robot. Hardwire to 3.3V if not connected to the robot.

int toggle14 = 0;
int toggle15 = 0;
int upLim = 1700; // 2500 is top start speed without stalling
int lowLim = 2;

int rate = 2000;


long tellTime = 500;
long timeLast = 0;
String state14 = "OFF";
String state15 = "OFF";
String rateString;

void setup()
{
  pinMode(M1dirpin, OUTPUT);
  pinMode(M1steppin, OUTPUT);
  pinMode(M1en, OUTPUT);
  pinMode(M2dirpin, OUTPUT);
  pinMode(M2steppin, OUTPUT);
  pinMode(M2en, OUTPUT);


  digitalWrite(M1en, LOW); // Low Level Enable
  digitalWrite(M2en, LOW); // Low Level Enable

  digitalWrite(M1dirpin, LOW);
  digitalWrite(M2dirpin, LOW);

  pinMode(13, OUTPUT);

  Serial.begin(115200);

  lcd.begin (20, 4, LCD_5x8DOTS);
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE);
  //init_super_font(&lcd);
  lcd.setBacklight(HIGH);
  lcd.setCursor ( 0, 1 );
  lcd.print("    HELLO WORLD  ");
  delay(5000);
  lcd.clear();



}
void loop()
{
 rate = getPot();
 
  toggle14 = analogRead(robotDO14);
  //toggle15 = analogRead(robotDO15);

 

  if (toggle14 < 800) {

    //digitalWrite(M1steppin,HIGH); //stop Motor
    digitalWrite(M2steppin, HIGH);
    state14 = "OFF";
    digitalWrite(13, LOW);



  } else {

    digitalWrite(13, HIGH);
    delayMicroseconds(2);
    //digitalWrite(M1steppin,LOW);
    digitalWrite(M2steppin, LOW);
    delayMicroseconds(rate);
    //digitalWrite(M1steppin,HIGH); //Rising step
    digitalWrite(M2steppin, HIGH);
    delay(1);
    state14 = " ON";

  }

  //{
  toggle15 = analogRead(robotDO15);
  
  //rate = getPot();

  if (toggle15 < 800) {

    digitalWrite(M1steppin, HIGH); //stop Motor
    //digitalWrite(M2steppin,HIGH);
    state15 = "OFF";
    digitalWrite(13, LOW);



  } else {

    digitalWrite(13, HIGH);
    delayMicroseconds(2);
    digitalWrite(M1steppin, LOW);
    //digitalWrite(M2steppin,LOW);
    delayMicroseconds(rate);
    digitalWrite(M1steppin, HIGH); //Rising step
    //digitalWrite(M2steppin,HIGH);
    delay(1);
    state15 = " ON";

  }


  if (millis() - timeLast >= tellTime) {
    Serial.print("rate : ");
    Serial.println(rate);
    Serial.print("state14 : ");
    Serial.println(state14);
    Serial.print("state15 : ");
    Serial.println(state15);


    rateString = buff(rate);
    lcd.setBacklight(HIGH);
    //lcd.clear();
    lcd.setCursor ( 0, 0 );
    lcd.print("Frequency : ");
    lcd.print(rateString);
    lcd.setCursor ( 0, 1 );
    lcd.print("DO 14 Status : ");
    lcd.print(state14);
    lcd.setCursor ( 0, 2 );
    lcd.print("DO 15 Status : ");
    lcd.print(state15);
    timeLast = millis();
  }
}


int getPot() {
  int potVal = analogRead(potPin); // Reads the potentiometer
  int newCustom = map(potVal, 0, 1022, lowLim, upLim); // Convrests the read values of the potentiometer from 0 to 1023 into desireded delay values (300 to 4000)
  return newCustom;

}

String buff(int n) {
  String out;
  String nS = String(n);
  if (n < 10) {
    out = "000" + nS;
  } else if (n < 100) {
    out = "00" + nS;
  } else if (n < 1000) {
    out = "0" + nS;
  } else {
    out = nS;
  }
  return out;
}




