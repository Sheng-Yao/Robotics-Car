#include <Arduino.h>
//Initialize MPU6050 and include libraries
#include "Wire.h"
#include <MPU6050_light.h>
MPU6050 mpu(Wire);
unsigned long timer = 0;

//Initialize servo motor and include libraries
#include <Servo.h>
Servo myServo;

//Initialize LCD
#include <LiquidCrystal.h>
#define pin_RS 8
#define pin_EN 9
#define pin_d4 4
#define pin_d5 5
#define pin_d6 6
#define pin_d7 7
#define pin_BL 10
LiquidCrystal lcd( pin_RS,  pin_EN,  pin_d4,  pin_d5,  pin_d6,  pin_d7);

//Encoder1
int encoder_pin1 = 19;  // The pin the encoder is connected           
unsigned int rpm1;     // first rpm reading
volatile int pulses1;  // number of pulses from encoder 1

//Encoder2
int encoder_pin2 = 2;  // The pin the encoder is connected           
unsigned int rpm2;     // second rpm reading
volatile int pulses2;  // number of pulses from encoder 2

unsigned long timeold; 
// The number of pulses per revolution
const int pulsesperturn = 20;

//Define variable name for each pin
#define enableMotor1 3
#define enableMotor2 11
#define motorInput1 24
#define motorInput2 25
#define motorInput3 26
#define motorInput4 27
#define leftSensor 36
#define rightSensor 33
#define trigPin 50
#define echoPin 51
#define detectingMode1AndMode2AndMode3 28
#define detectingMode2AndMode3 16
#define yellowLED 30
#define redLED 32
#define blueLED 34

//Initialize variable
long totalCount = 0; // the time taken for robot car to remain on
int lSensor; //value from left sensor
int rSensor; //value from right sensor
int distance=0; //current distance
int distanceTravelled = 0; // total distance travelled
unsigned long duration; // time taken for the wave to return to ultrasonic sensor
int distanceResult; // distance being calculated by ultrasonic sensor
int mode1;//line following
int mode2;//ultrasonic sensor
int mode3;//bluetooth
int mode; //current mode
int modeChanging;
int rps;// rps of average from rpm1 and rpm2
int pos = 90; //initial position

char Incoming_value;//value received by the Bluetooth module
int maxAngle = 0; // max angle being detected
int currentAngle; // current angle being detected
int flag=0; // is on the ramp or under the ramp
int turnAxis; // the turning degree
unsigned long startingTime = 0; // starting time for the line following mode
int stopped = 0; //is stopped at 200cm or not

void counter1()
{
    //Update count
    pulses1++;    
}
void counter2()
{
    //Update count
    pulses2++;    
}

void fullSpeed()//255
{
  analogWrite(enableMotor1,255);
  analogWrite(enableMotor2,255);
}

void halfSpeed()//90
{
  analogWrite(enableMotor1,90);
  analogWrite(enableMotor2,90);
}

void slowSpeed()//60
{
  analogWrite(enableMotor1,60);
  analogWrite(enableMotor2,60);
}

void stop()
{
  digitalWrite(motorInput1,LOW);
  digitalWrite(motorInput2,LOW);
  digitalWrite(motorInput3,LOW);
  digitalWrite(motorInput4,LOW);
}

void setup() {
  // put your setup code here, to run once:

  //Initialize LCD
  lcd.begin(16, 2);
  lcd.setCursor(2,0);                                                                                                                                                                                                                                                                                                                   
  lcd.print("Welcome!!!!!");
  lcd.setCursor(2, 1);
  lcd.print("Initializing");

  //Initialize Serial monitor  
  Serial.begin(9600);

  //Initialize the accerelometer
  Wire.begin();
  byte status = mpu.begin();
  while(status!=0){ } // stop everything if could not connect to MPU6050
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero

  //Initialize the starting z axis
  turnAxis = mpu.getAngleZ();

  //Initialize servo motor
  myServo.attach(53);
  for(int i=0;i<=180;i++)
  {
    myServo.write(i);
    delay(10);
  }
  for(int i=180;i>=0;i--)
  {
    myServo.write(i);
    delay(10);
  }
  myServo.write(90);  

  //Initialize encoder1
  pulses1 = 0;
  pinMode(encoder_pin1, INPUT);
  attachInterrupt(4, counter1, RISING);

  //Initialie encoder2
  pulses2 = 0;
  pinMode(encoder_pin2, INPUT);
  attachInterrupt(0, counter2, RISING);

  timeold = millis();
  rpm1 = 0;
  rpm2 = 0;

  //reset the lcd
  lcd.clear();
  lcd.setCursor(0, 0);

  //Define the mode of the pin
  pinMode(enableMotor1,OUTPUT);
  pinMode(enableMotor2,OUTPUT);
  pinMode(motorInput1,OUTPUT);
  pinMode(motorInput2,OUTPUT);
  pinMode(motorInput3,OUTPUT);
  pinMode(motorInput4,OUTPUT);
  pinMode(leftSensor,INPUT);
  pinMode(rightSensor,INPUT);
  pinMode(echoPin,INPUT);
  pinMode(trigPin,OUTPUT);
  pinMode(14,OUTPUT);
  digitalWrite(14,HIGH);
  pinMode(detectingMode2AndMode3,INPUT);
  pinMode(18,OUTPUT);
  digitalWrite(18,LOW);
  pinMode(yellowLED,OUTPUT);
  pinMode(redLED,OUTPUT);
  pinMode(blueLED,OUTPUT);

  //Initialize the value of the variable
  mode1 = 0;
  mode2 = 0;
  mode3 = 0; 
  startingTime=0;

  //stopping for 1s before everythings started for safety
  delay(1000);
}

void goForwardhalf()
{
  halfSpeed();
  digitalWrite(motorInput1,HIGH);
  digitalWrite(motorInput2,LOW);
  digitalWrite(motorInput3,HIGH);
  digitalWrite(motorInput4,LOW);
}

void Left()
{
  fullSpeed();
  digitalWrite(motorInput1,HIGH);
  digitalWrite(motorInput2,LOW);
  digitalWrite(motorInput3,LOW);
  digitalWrite(motorInput4,HIGH);
}

void Right()
{
  fullSpeed();
  digitalWrite(motorInput1,LOW);
  digitalWrite(motorInput2,HIGH);
  digitalWrite(motorInput3,HIGH);
  digitalWrite(motorInput4,LOW);
}

void pureLeft()
{
  digitalWrite(motorInput1,HIGH);
  digitalWrite(motorInput2,LOW);
  digitalWrite(motorInput3,LOW);
  digitalWrite(motorInput4,HIGH);
}

void pureRight()
{
  digitalWrite(motorInput1,LOW);
  digitalWrite(motorInput2,HIGH);
  digitalWrite(motorInput3,HIGH);
  digitalWrite(motorInput4,LOW);
}

void goRight()
{
  pureRight();
  delay(50);
  goForwardhalf();
  delay(15);
  stop();
}

void goLeft()
{
  pureLeft();
  delay(50);
  goForwardhalf();
  delay(15);
  stop();
}

void goForwardfull()
{
  fullSpeed();
  digitalWrite(motorInput1,HIGH);
  digitalWrite(motorInput2,LOW);
  digitalWrite(motorInput3,HIGH);
  digitalWrite(motorInput4,LOW);
}

void goForwardslow()
{
  slowSpeed();
  digitalWrite(motorInput1,HIGH);
  digitalWrite(motorInput2,LOW);
  digitalWrite(motorInput3,HIGH);
  digitalWrite(motorInput4,LOW);
}

void goBackward1()
{
  fullSpeed();
  digitalWrite(motorInput1,LOW);
  digitalWrite(motorInput2,HIGH);
  digitalWrite(motorInput3,LOW);
  digitalWrite(motorInput4,HIGH);
}

void goRightstop()
{
  pureRight();
  delay(600);
  stop();
}

void uTurn()
{
  pureRight();
  delay(1300);
  stop();
}

void goLeftstop()
{
  pureLeft();
  delay(600);
  stop();
}

void checkDistance()
{
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  duration = pulseIn(echoPin,HIGH);
  distanceResult = (duration / 2) * (0.34);
}

void loop() {
  mode = digitalRead(detectingMode1AndMode2AndMode3);
  modeChanging = digitalRead(detectingMode2AndMode3);
  if(mode == 0)
  {
    if(mode1 == 0)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      digitalWrite(yellowLED,HIGH);
      digitalWrite(redLED,LOW);
      digitalWrite(blueLED,LOW);
    }
    mode1 = 1;
    mode2 = 0;
    mode3 = 0; 

    if(totalCount <= 60)
    {
      mpu.update();//update the data of mpu
      if((millis()-timer)>100)
      { // print data every 100ms
        currentAngle = mpu.getAngleY(); // get the value of y axis
        if(currentAngle >= maxAngle)// get the max value of y-axis
        {
          maxAngle = currentAngle;
        }
        timer = millis();  
      }
      
      if (millis() - timeold >= 1000)
      {  /*Uptade every one second, this will be equal to reading frecuency (Hz).*/
        //Don't process interrupts during calculations
        detachInterrupt(4);
        detachInterrupt(0);
        rpm1 = ((60 * 1000 / pulsesperturn )/ (millis() - timeold)* pulses1) * 0.01667;             
        rpm2 = ((60 * 1000 / pulsesperturn )/ (millis() - timeold)* pulses2) * 0.01667;  
        rps = (rpm1+rpm2)/2;         
        timeold = millis();
        pulses1 = 0;
        pulses2 = 0;
        distance = 2 * 3.142 * 3.25 * rps;
        distanceTravelled+=distance;
        //Restart the interrupt processing
        attachInterrupt(4, counter1, RISING);
        attachInterrupt(0, counter2, RISING);
      }

      if(startingTime > 0)// starting the calculation of time after the ramp
      {
        lcd.setCursor(0, 1);
        lcd.print("Distance : ");
        lcd.setCursor(11, 1);
        lcd.print(distanceTravelled);
        lcd.setCursor(0,0);    
        totalCount=(millis()-startingTime)/1000;
        lcd.print(totalCount);
      }
      lSensor = digitalRead(leftSensor);
      rSensor = digitalRead(rightSensor);
      lcd.setCursor(8, 0);
      lcd.print(maxAngle);

      if(lSensor == 1 && rSensor == 1)
      {
        if(distanceTravelled >= 200 && startingTime !=0 && stopped == 0)
        {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("STOP!!!!!!!!!!");
          stop();
          delay(3000);
          lcd.clear();
          stopped = 1;
        }
        else
        {
          if(currentAngle > 10 && flag == 0)
          {
            goForwardfull();
            delay(1500);
            stop();
            delay(4000);
            flag=1;
          }
          else
          {
            goForwardhalf();
          }
          if(flag == 1)
          {
            while(turnAxis <= 350)
            {
              pureRight();
              delay(10);    
              mpu.update();
              turnAxis = mpu.getAngleZ();   
            }
            goForwardslow();
            delay(400);
            distanceTravelled =0;
            flag = 0;
          }
          if(currentAngle <= -5)
          {
            distanceTravelled= 0;
            startingTime = millis();
          }
        }
      }
      else if(lSensor == 1 && rSensor == 0)
      {
        goLeft();
      }
      else if(lSensor == 0 && rSensor == 1)
      {
        goRight();
      }
      else if(lSensor == 0 && rSensor == 0)
      {
        goForwardslow();
        delay(200);
        stop();
      }
    }
    else
    {
      stop();
    }
  }else if(mode == 1 && modeChanging == 1)
  {
    if(mode2 == 0)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      digitalWrite(yellowLED,LOW);
      digitalWrite(redLED,HIGH);
      digitalWrite(blueLED,LOW);
    }
    mode1 = 0;
    mode2 = 1;
    mode3 = 0; 

    if(totalCount <= 60)
    {
      lcd.setCursor(0, 0);
      totalCount=millis()/1000;
      lcd.print(totalCount-6);
      checkDistance();
      if(distanceResult > 400)
      {
        goForwardfull();
        delay(20);
      }
      else
      {
        stop();
        delay(400);
        checkDistance();
        if(distanceResult > 400)
        {
          goForwardfull();
          delay(20);
        }
        else
        {
          myServo.write(180);
          delay(300);
          checkDistance();

          if(distanceResult > 400)
          {
            goRightstop();
          }
          else
          {
            checkDistance();
            if(distanceResult > 400)
            {
              goRightstop();
            }
            else
            {
              myServo.write(0);
              delay(300);
              checkDistance();
              if(distanceResult > 400)
              {
                goLeftstop();
              }
              else
              {
                  goBackward1();
                  delay(500);
                  uTurn();
              }
            }
          }
        }
      }
      myServo.write(90);
    }   
  }else if(mode == 1 && modeChanging == 0)
  {
    if(mode3 == 0)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      digitalWrite(yellowLED,LOW);
      digitalWrite(redLED,LOW);
      digitalWrite(blueLED,HIGH);
    }
    mode1 = 0;
    mode2 = 0;
    mode3 = 1; 
    
    lcd.setCursor(0,0);    
    totalCount=millis()/1000;
    lcd.print(totalCount-6);

    //default password 1234 OR 0000
    if(Serial.available() > 0)
    {
      Incoming_value = Serial.read();
      Serial.print(Incoming_value);
      if(Incoming_value == 'l')
      {
        pureLeft();
      }
      else if(Incoming_value == 'f')
      {
        goForwardfull();
      }
      else if(Incoming_value == 'r')
      {
        Right();
      }
      else if(Incoming_value == 'b')
      {
        goBackward1();
      }
      else if(Incoming_value == 's')//the cross shape
      {
        stop();
      }
      else if(Incoming_value == 'x')//the triangle shape
      {
        goForwardfull();
        delay(100);
        goRightstop();            
      }
      else if(Incoming_value == 'y')//the rectangular shape
      {
        goForwardfull();
        delay(100);
        goLeftstop();
      }
      else if(Incoming_value == 't')
      {
        uTurn();
      }
    }
  }
}