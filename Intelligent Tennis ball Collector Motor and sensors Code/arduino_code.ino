#include <PID_v1.h>
#include <Wire.h> // Library for I2C communication
#include <LiquidCrystal_I2C.h> // Library for LCD

//define pins for 2 encoders

#define ENCA 2 
#define ENCB 13
#define ENCC 3
#define ENCD 12

//define pins for communication between arduino and rasbperry PI
#define recieve A3
#define face_rec A2

//define pins for the ultrasonic obstacle detection
#define echoPin 10 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 11 //attach pin D3 Arduino to pin Trig of HC-SR04

long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

//defines for the LCD
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);

//define motor and H bridge PIns
#define enA 9
#define in1 7
#define in2 8
#define enB 6
#define in3 4
#define in4 5

//define the IR sensor
#define IR A1


//various data used for coding
String data;
int start=0;
char my_balls[10];
int no_balls=0;
int wel=0;
int key=0;
int key2=0;
int stopp=0;

//angular speed variables using encoders for motor 1
volatile long pos=0;
long newposition;
long oldposition = 0;
unsigned long newtime;
unsigned long oldtime = 0;
double vel;

//angular speed variables using encoders for motor 2
volatile long pos2=0;
long newposition2;
long oldposition2 = 0;
unsigned long newtime2;
unsigned long oldtime2 = 0;
double vel2;
double deltaT,deltaT2;

//2 PID controllers parameters
PID myPID(&vel, &Output, &Setpoint, kp, ki, kd, DIRECT); 
PID myPID2(&vel2, &Output2, &Setpoint2, kp, ki, kd, DIRECT);  

//setpoints of 2 motors
double Setpoint ; // will be the desired speed value of motor 1
double Setpoint2 ; // will be the desired speed value of motor 2

//PID controller Outputs for 2 motors
double Output ;  
double Output2 ;

void calculate_speed_1()                //calculation of motor 1 speed
{
   newposition = pos;
 newtime = millis();
 vel = (newposition-oldposition) * 1000 *0.16/(newtime-oldtime);
 deltaT=(newtime-oldtime)/1000;
 Serial.print ("speed = ");
 Serial.println (vel);
 oldposition = newposition;
 oldtime = newtime;
 
 Serial.print("                   vel= ");
 Serial.print(vel);
}
void calculate_speed_2()                //calculation of motor 2 speed
{
  newposition2 = pos2;
 newtime2 = millis();
 vel2 = (newposition2-oldposition2) * 1000 *0.16/(newtime2-oldtime2);
 deltaT2=(newtime2-oldtime2)/1000;
 Serial.print ("speed2 = ");
 Serial.println (vel2);
 oldposition2 = newposition2;
 oldtime2 = newtime2;
 
 Serial.print("                   vel2= ");
 Serial.print(vel2);
}

void display_balls()                    //display of number of collected balls
{
  lcd.setCursor(1, 0); // Set the cursor on the third column and first row.
  lcd.print("Balls Collected:"); 
  lcd.setCursor(8, 1); // Set the cursor on the third column and first row.
  sprintf(my_balls,"%d",no_balls);
  lcd.print(my_balls);
}
void approach()                         //approaching the balls when in the center position
{
  Setpoint=50;                          //set motor 1 to rotate 
  Setpoint2=50;                         //set motor 2 to rotate
  calculate_speed_1();
  calculate_speed_2();
  myPID.Compute();                      //calculate the outputs from the error signal and closed loop control
  myPID2.Compute();                     //calculate the outputs from the error signal and closed loop control                      
  analogWrite(enA,Output);              // Send PWM signal to L298N Enable pin         
  analogWrite(enB,Output2);             // Send PWM signal to L298N Enable pin
  digitalWrite(in2,LOW);                //Set motor 1 direction
  digitalWrite(in1,HIGH);
  digitalWrite(in3,LOW);                //Set motor 2 direction
  digitalWrite(in4,HIGH);
  
   while(1)
   {
    avoid_obstacle();                   //keep approaching the ball in straight line till obstacle detected or IR pn detects ball collected
    if(distance<=40)                    //if obstacle is detected 
      {
        analogWrite(enA,0);             // Send LOW signal to L298N Enable pin to stop motor 1 
        analogWrite(enB,0);             // Send LOW signal to L298N Enable pin to stop motor 2 
        break;
      }
    if(digitalRead(IR)==LOW)            //if ball is collected
      {
        no_balls++;                     //increase no of collected balls
        display_balls();                //display the no of balls
        delay(2000);
        break;  
      }
    }
}
void stop_motor()                       // stop the motor from rotating and approach the ball
{
  stopp=0;
  analogWrite(enA,0);                   // Send LOW signal to L298N Enable pin to stop motor 1 
  analogWrite(enB,0);                   // Send LOW signal to L298N Enable pin to stop motor 1 
  delay(500);
  approach();                           //approach the ball
}
void rotate()                           //rotate in one direction and search for balls
{
  calculate_speed_1();
  Setpoint=20;
  myPID.Compute();                      //calculate the outputs from the error signal and closed loop control
  analogWrite(enA,Output);              // Send PWM signal to L298N Enable pin
  analogWrite(enB,0);                   // Send LOW signal to L298N Enable pin to stop motor 1 
  digitalWrite(in2,LOW);                //set motor 1 rotating direction
  digitalWrite(in1,HIGH);
  
  if (digitalRead(recieve)==HIGH)       //if the communication from PI detects ball in the center of the frame
    {
       stop_motor();                    //stop the robot and approach the ball
       return;
    }
   display_balls();                     //display the number of collected balls

}

void display_welcome()                  //display the welcome screen when Preset Face is recognized
{
   lcd.setCursor(1, 0); // Set the cursor on the third column and first row. 
   lcd.print("Welcome User      "); 
   delay(3000);
}

void avoid_obstacle()                   //calculate distance of objected and robot using echo and trigger
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;      // Speed of sound wave divided by 2 (go and back)
}

void readEncoder(){                     //function to update the position of the first encoder
  int b = digitalRead(ENCB);
  if(b > 0){
    pos++;  
  }
  else{
    pos--;
  }
}


void readEncoder2(){                    //function to update the position of the second encoder
  int b = digitalRead(ENCD);
  if(b > 0){
    pos2++;  
  }
  else{
    pos2--;
  }
}


void setup() 
{
  //LCD initialize
  start=0;
  no_balls=0;
  lcd.init();
  lcd.backlight();

  //Rasbperry PI and Arduino Pins communication
  pinMode(recieve, INPUT);              //communication pin from PI to be high when the ball is detected in the center of the frame
  pinMode(face_rec, INPUT);             //communication pin from PI to be high when a Preset face is detected 

  //IR pins
  pinMode(IR, INPUT);

  //PID parameters using Zigler Method 
  kp=1.25;//1.25
  ki=3.24;//3.24
  kd=0;                                   //set as 0 to keep the system steady
  myPID.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  
  //Adjust PID values
  myPID.SetTunings(kp, ki, kd);
  myPID2.SetTunings(kp, ki, kd);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCC),readEncoder2,RISING);
  
  //Ultrasonic
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT


  //motor
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA,OUTPUT);  
  pinMode(enB,OUTPUT);

  
  Serial.begin(9600);

  //Encoders
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(ENCC,INPUT);
  pinMode(ENCD,INPUT);

} 


void loop() 
{
  if( digitalRead(face_rec)==1)             //dont start code when until the Preset face is detected
    {
      if(wel==0)
        {
          display_welcome();                //if face is detected Display welcome screen
        }
  wel=1;
  start=1; 
    }
  if(start==1)
  {
    wel=1;
    rotate();                               //start searching for the balls 
  }
  else                                      //keep displaying look to the camera until   a preset face is recognized
  {
     lcd.setCursor(1, 0);                   // Set the cursor on the third column and first row.
     lcd.print("Look to camera"); 
  }
}
