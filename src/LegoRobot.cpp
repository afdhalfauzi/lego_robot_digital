#include <Arduino.h>
#include <LegoRobot.h>
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LiquidCrystal_I2C.h>


Servo servoGrip, servoArm;
RF24 radio(48, 49); //CE, CSN
LiquidCrystal_I2C lcd(0x27, 16, 2);

LegoRobot::LegoRobot() {
    // init();
    // const byte address[6] = {8,4,5,2,1};
}

void LegoRobot::init() {
  servoGrip.attach(6);
  servoArm.attach(7);

  pinMode(rm1, OUTPUT);
  pinMode(lm1, OUTPUT);
  pinMode(rm2, OUTPUT);
  pinMode(lm2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  radio.begin();
  // radio.openReadingPipe(0, 84521);
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  lcd.begin();
  lcd.backlight();
}

void LegoRobot::bukaanGripper(int derajat){
  servoGripPos = map(derajat, 0, 100, 0, 180);
  servoGrip.write(servoGripPos);
  Serial.println("mamamam");
}

void LegoRobot::ketinggianGripper(int derajat){
  servoArmPos = map(derajat, 0, 100, 0, 180);
  servoArm.write(servoArmPos);
}

void LegoRobot::maju(int time, int speed) {
  Serial.println("Maju");
  digitalWrite(rm1, 1);
  digitalWrite(rm2, 0);
  digitalWrite(lm1, 1);
  digitalWrite(lm2, 0);
  analogWrite(enA, map(speed, 0, 100, 0, 255));
  analogWrite(enB, map(speed, 0, 100, 0, 255));
  delay(time); 

  stopMotor();
}

void LegoRobot::mundur(int time, int speed) {
  Serial.println("Mundur");
  digitalWrite(rm1, 0);
  digitalWrite(rm2, 1);
  digitalWrite(lm1, 0);
  digitalWrite(lm2, 1);
  analogWrite(enA, map(speed, 0, 100, 0, 255));
  analogWrite(enB, map(speed, 0, 100, 0, 255));
  delay(time); 

  stopMotor();
}

void LegoRobot::jeda(int time) {
  Serial.println("Jeda");
  delay(time);
}

void LegoRobot::putarKanan(int time) {
  Serial.println("Putar Kanan");
  digitalWrite(rm1, 1);
  digitalWrite(rm2, 0);
  digitalWrite(lm1, 0);
  digitalWrite(lm2, 1);
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  delay(time);

  stopMotor();
}

void LegoRobot::putarKiri(int time) {
  Serial.println("Putar Kanan");
  digitalWrite(rm1, 0);
  digitalWrite(rm2, 1);
  digitalWrite(lm1, 1);
  digitalWrite(lm2, 0);
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  delay(time);
  
  stopMotor();
}

void LegoRobot::stopMotor() {
  digitalWrite(rm1, 0);
  digitalWrite(rm2, 0);
  digitalWrite(lm1, 0);
  digitalWrite(lm2, 0);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

bool LegoRobot::sambungRemote() {
  if (radio.available()) {
    delay(15);
    int dataPackage[6] = {};
    radio.read(&dataPackage, sizeof(dataPackage));
    lcd.setCursor(0,0);
    lcd.print("CONNECTED       ");

    joystickX = dataPackage[0];
    joystickY = dataPackage[1];
    action1 = dataPackage[2];
    action2 = dataPackage[3];
    action3 = dataPackage[4];
    action4 = dataPackage[5];

    Serial.print("{");
    for(int i=0; i<6; i++){
      Serial.print(dataPackage[i]);
      Serial.print(", ");
    }
    Serial.println("}");

    //JOYSTICK ACTION
    if(joystickY < 470) {
      Serial.print("Mundur");
      digitalWrite(rm1, 1);
      digitalWrite(rm2, 0);
      digitalWrite(lm1, 1);
      digitalWrite(lm2, 0);
      speedRight = map(joystickY, 470, 0, 0, 255);
      speedLeft = map(joystickY, 470, 0, 0, 255);
    }
    else if(joystickY > 550) {
      Serial.print("Maju");
      digitalWrite(rm1, 0);
      digitalWrite(rm2, 1);
      digitalWrite(lm1, 0);
      digitalWrite(lm2, 1);
      speedRight = map(joystickY, 550, 1023, 0, 255);
      speedLeft = map(joystickY, 550, 1023, 0, 255);
    }
    else{
      speedRight = 0;
      speedLeft = 0;
    }
    
    if(joystickX < 470) {
      x = map(joystickX, 470, 0, 0, 255);
      Serial.print("kiri");
      speedRight = speedRight - x;
      speedLeft = speedLeft + x;
    
      if (speedRight < 0){
        speedRight = 0;
      }
      if (speedLeft > 255){
        speedLeft = 255;
      }
    }
    
    if(joystickX > 550) {
      x = map(joystickX, 550, 1023, 0, 255);
      Serial.print("kanan");
      speedRight = speedRight + x;
      speedLeft = speedLeft - x;
    
      if (speedRight > 255){
        speedRight = 255;
      }
      if (speedLeft < 0){
        speedLeft = 0;
      }
    }
    
    //prevent buzzing
    if(speedRight < 70){
      speedRight = 0;
    }
    if(speedLeft < 70){
      speedLeft = 0;
    }

    analogWrite(enA, speedRight);
    analogWrite(enB, speedLeft);
    
    //GRIPPER ACTION
    if(action1 == 0){
      Serial.print("Buka Gripper");
      if (servoArmPos < 180){
        servoArmPos+=3;
        servoArm.write(servoArmPos);
      }
    }
    else if(action2 == 0){
      Serial.print("Tutup Gripper");
      if (servoArmPos > 0){
        servoArmPos-=3;
        servoArm.write(servoArmPos);
      }
    }
    else if(action3 == 0){
      Serial.print("Gripper Turun");
      if (servoGripPos < 180){
        servoGripPos+=3;
        servoGrip.write(servoGripPos);
      }
    }
    else if(action4 == 0){
      Serial.print("Gripper Naik");
      if (servoGripPos > 0){
        servoGripPos-=3;
        servoGrip.write(servoGripPos);
      }
    }
    return true;
  }
  else {
    Serial.println("CONNECTION LOST");
    lcd.setCursor(0,0);
    lcd.print("CONNECTION LOST ");
    return false;
  }
}

void LegoRobot::belokKanan(int time, int speed) {
  digitalWrite(rm1, 0);
  digitalWrite(rm2, 0);
  digitalWrite(lm1, 1);
  digitalWrite(lm2, 0);
  analogWrite(enA, map(speed, 0, 100, 0, 255));
  analogWrite(enB, map(speed, 0, 100, 0, 255));
  delay(time); 

  stopMotor();
}

void LegoRobot::belokKiri(int time, int speed) {
  digitalWrite(rm1, 1);
  digitalWrite(rm2, 0);
  digitalWrite(lm1, 0);
  digitalWrite(lm2, 0); 
  analogWrite(enA, map(speed, 0, 100, 0, 255));
  analogWrite(enB, map(speed, 0, 100, 0, 255));
  delay(time); 

  stopMotor();
}

int LegoRobot::bacaGaris(String posisi){
  int sensor[4];
  if (posisi == "kiri"){
    sensor[0] = analogRead(A0);
    Serial.print("bacakiri : ");
    Serial.println(sensor[0]);
    int bacakiri = sensor[0] ;
    return bacakiri;
  }
  else if (posisi == "tengahkiri"){
    sensor[1] = analogRead(A2);
    Serial.print("bacatengahkiri : "); 
    Serial.println(sensor[1]);
    int bacatengahkiri = sensor[1] ;
    return bacatengahkiri;
  }
  else if (posisi == "tengahkanan"){
    sensor[2] = analogRead(A4); 
    Serial.print("bacatengahkanan : ");
    Serial.println(sensor[2]);
    int bacatengahkanan = sensor[2] ;
    return bacatengahkanan;
  }
  else if (posisi == "kanan"){
    sensor[3] = analogRead(A6);
    Serial.print("bacakanan : ");
    Serial.println(sensor[3]);
    int bacakanan= sensor[3] ;
    return bacakanan;
  }
  else{
    Serial.println("Parameter tidak valid");
  }
}

int LegoRobot::printGaris(){
  int sensor[4] ;
  sensor[0] = analogRead(A0); //kiri
  sensor[1] = analogRead(A2); //tengah
  sensor[2] = analogRead(A4); //tengah
  sensor[3] = analogRead(A6); //kanan
  int bacaKiri = sensor[0] ;
  int bacaTengahKiri = sensor[1] ;
  int bacaTengahKanan = sensor[2] ;
  int bacaKanan = sensor[3] ;
  Serial.print("bacakiri : ");
  Serial.println(sensor[0]);
  Serial.print("bacatengahkiri : "); 
  Serial.println(sensor[1]);
  Serial.print("bacatengahkanan : ");
  Serial.println(sensor[2]);
  Serial.print("bacakanan : ");
  Serial.println(sensor[3]);
  return sensor;
}

void LegoRobot::stop() {
  digitalWrite(rm1, 0);
  digitalWrite(rm2, 0);
  digitalWrite(lm1, 0);
  digitalWrite(lm2, 0);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

int Legorobot::bacaSensorAvoider(String arah){
  if (arah == "depan"){
    readPing();
  }
  else if (arah == "kiri"){
    myservo.write(180);
    delay(500);
    readPing();
    delay(100);
    myservo.write(115);
  }
  else if (arah == "kanan"){
    myservo.write(50);
    delay(500);
    readPing();
    delay(100);
    myservo.write(115);
  }
  else{
    Serial.println("Parameter tidak valid");
  }
}

int LegoRobot::readPing(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distanceCm = duration * 0.034 / 2;
  //Serial.println(distanceCm);
  return distanceCm;
}
