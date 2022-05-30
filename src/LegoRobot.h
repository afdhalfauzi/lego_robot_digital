#ifndef LEGO_ROBOT_H
#define LEGO_ROBOT_H

#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// class Servo;
class LegoRobot {
  
  public:
    LegoRobot();
  
    void init();  
    void on(int time);
    void off();

    void maju(int time, int speed);
    void mundur(int time, int speed);
    void jeda(int time);
    void putarKanan(int time);
    void putarKiri(int time);
    void belokKanan(int time, int speed);
    void belokKiri(int time, int speed);
    void bukaanGripper(int derajat);
    void ketinggianGripper(int derajat);
    int bacaGaris(String posisi);
    int printGaris();
    int bacaSensorAvoider(String arah);
    void stop();
    bool sambungRemote();


    void stopMotor();

    private:
      // RF24 radio(48, 49); //CE, CSN
      Servo myservo ,servoGrip, servoArm;

      //MOTOR
      int rm1 = 30;//right motor
      int rm2 = 27;
      int lm1 = 29;//left motor
      int lm2 = 31;
      int enA = 10;//right motor speed
      int enB = 12;//left motor speed

      // const byte address[6] = "84521";
      // byte address[6] = {"8","4","5","2","1"};
      const byte address[6] = {8,4,5,2,1};
      int joystickX;
      int joystickY;
      int x;
      int y;
      int speedRight;
      int speedLeft;
      int action1, action2, action3, action4;

      int sensor[6];

      int servoGripPos = 0;
      int servoArmPos = 0;
      // int time;
      // int speed;
      // extern Servo* grip;

      int trigPin = 44;
      int echoPin = 46;
      long duration;
      int distanceCm;

};

#endif
