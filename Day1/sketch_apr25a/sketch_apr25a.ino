#include <QTRSensors.h>
#include <AFMotor.h>

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2


AF_DCMotor motor1(1), motor2(2);

int crit = 500;
int SpeedMax = 120;
int sensorLeft, sensorRight;

// PID gain for line follow. initially set zero.
float Kp =0;
float Kd =0;
float Ki =0;

// PID calculation for wall follow. 
int preverror = 0;
int error = 0;
int derror = 0;
int ierror = 0;
int U;

int Uled= 35;
int rightLed = 34;
int leftLed = 36;
//int sLed = 42;
  
int follow=0;
int notfollow=1;
float avg[10];
float total;



// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrc((unsigned char[]) {53, 52, 51, 22, 49, 48, 47, 46}, //53 left most sensor and 46 rightmost sensorValues[0]= LeftMost sensorValues[7]= rightMost
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];


void setup()
{ 
  
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  
  
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
  
  Serial.println("Motor test!");

  // turn on motor
  motor1.setSpeed(SpeedMax);
  motor1.run(RELEASE);
  motor2.setSpeed(SpeedMax);
  motor2.run(RELEASE);
  
  
  for (int i=0; i<10; i++)
  {
  sensorRead();
  if (sensorValues[0]==1 && sensorValues[7]==1)
  {
    avg[i] = 0;
  }
  else avg[i] =1;
  }
  
  total = (avg[0]+avg[1]+avg[2]+avg[3]+avg[4]+avg[5]+avg[6]+avg[7]+avg[8]+avg[9])/10;
 if (total>0.5)
 {
   follow =1;
   notfollow=0;
 }
 else 
 {
   follow =0;
   notfollow=1;
 }
 
   pinMode(Uled, OUTPUT);
   pinMode(rightLed, OUTPUT);
   pinMode(leftLed, OUTPUT);
   //pinMode(sLed, OUTPUT);
}


void loop()
{
  sensorRead();
  digitalWrite (Uled,LOW);
  digitalWrite (rightLed,LOW);
  digitalWrite (leftLed,LOW);
  if (sensorValues[0]!=follow && sensorValues[1] != follow && sensorValues[2] !=follow && sensorValues[3] != follow && sensorValues[4]!= follow && sensorValues[5] != follow && sensorValues[6] != follow && sensorValues[7] != follow  && sensorRight!=follow && sensorLeft!=follow)
  {
   
   digitalWrite (Uled,HIGH);
   //delay(300);
   while(sensorValues[2] !=follow && sensorValues[3] !=follow && sensorValues[4] !=follow && sensorValues[5] !=follow)
   {
    turnU();
    sensorRead();
   }
  }
  
  else if(sensorRight==follow && sensorLeft==follow)
    {
      digitalWrite (rightLed,HIGH);
      
      while(sensorRight==follow)
    {  
      
      turnRight();
      delay(400); // last editied 
      sensorRead();
    }
  }
  
  
  else if (sensorRight==follow  && (sensorValues[3] == follow || sensorValues[4] == follow))
  {
      digitalWrite (rightLed,HIGH);
      while(sensorRight==follow)
    {  
      
      turnRight();
      delay(400); // to be adjustable. 
      sensorRead();
    }
 }
 //Turn Left while straight is available
 else if(sensorLeft ==follow  && sensorRight!=follow && (sensorValues[3] == follow || sensorValues[4] == follow))
  {  
    digitalWrite (leftLed,HIGH);
    while (sensorLeft==follow)
    {
     turnLeft();
     delay(400);
     sensorRead();
    }
  }
  
  // Turn left condition without intersection. 
  else if (sensorLeft==follow && sensorRight!=follow && sensorValues[3]!=follow && sensorValues[4]!=follow)
  {  
    digitalWrite (leftLed,HIGH);
    while(sensorValues[2]!=follow && sensorValues[3]!=follow && sensorValues[4]!=follow && sensorValues[5]!=follow)
    { 
      turnLeft();
      //delay(300);
      sensorRead();
    }
  }
  
  
  //Turn Right Condition
  else if (sensorRight==follow && sensorLeft!=follow && sensorValues[3]!=follow && sensorValues[4]!=follow)
  { 
    digitalWrite (rightLed,HIGH);
     while (sensorValues[2]!=follow && sensorValues[3]!=follow && sensorValues[4]!=follow && sensorValues[5]!=follow)
    {
      turnRight();
      sensorRead();
    } 
    
  }
  
  
    
  
  
  else if(sensorValues[1]== follow || sensorValues[2]== follow || sensorValues[3]== follow || sensorValues[4]== follow || sensorValues[5]== follow || sensorValues[6]==follow)
  {
    SpeedMax=130;
    Kp = 6; //50
    Kd = 70; //100
    Ki = 0;
  }
  
  
  
  else 
  {
    Kp = 100;
    Kd = 0;
    Ki = 0;
  }
  
  
  
  
  error =  (-4*sensorValues[0])  +(-3*sensorValues[1])  + (-2*sensorValues[2]) + (-1*sensorValues[3]) + (1*sensorValues[4]) + (2*sensorValues[5]) + (3*sensorValues[6])  +(4*sensorValues[7]);
  derror = error - preverror;
  ierror += error; 
  U = Kp * error + Kd * derror + Ki * ierror;  
  preverror = error;
  
  

  
  
  if(U<-SpeedMax) U=-SpeedMax;
  if(U>SpeedMax) U=SpeedMax;
 
  
  
  if(U>0)
  {
    motor1.run(FORWARD);
    motor1.setSpeed(SpeedMax);
  
    motor2.run(FORWARD);
    motor2.setSpeed(SpeedMax-U);
  
  }
  else if(U<0)
  {
    motor1.run(FORWARD);
    motor1.setSpeed(SpeedMax+U);
  
    motor2.run(FORWARD);
    motor2.setSpeed(SpeedMax);
    
  }
  else if (error==0) 
  {
    motor1.run(FORWARD);
    motor1.setSpeed(SpeedMax);
  
    motor2.run(FORWARD);
    motor2.setSpeed(SpeedMax);
 
  }
  else if (U==0) 
  {
    motor1.run(FORWARD);
    motor1.setSpeed(SpeedMax);
  
    motor2.run(FORWARD);
    motor2.setSpeed(SpeedMax);
 
  }   
  
  
}


void sensorRead()
{
  // read calibrated sensor values and obtain a measure of the line position from 0 to 5000
  // To get raw sensor values, call:
  //  qtrrc.read(sensorValues); instead of unsigned int position = qtrrc.readLine(sensorValues);
  unsigned int position = qtrrc.readLine(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum reflectance and
  // 1000 means minimum reflectance, followed by the line position
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(); // uncomment this line if you are using raw values
  //Serial.println(position); // comment this line out if you are using raw values
  
  //delay(250);
  
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    if(sensorValues[i]<crit)
     {
       sensorValues[i]=1;
     }
     else sensorValues[i]=0;
  }
  
  //for black line 0 1 for white line 1 0
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(); // uncomment this line if you are using raw values
  
   sensorLeft = digitalRead(33);
   if (sensorLeft == 1)
   {
     sensorLeft=0;
   }
   else sensorLeft =1;
   Serial.print(sensorLeft);
   Serial.print(' ');
   
   sensorRight = digitalRead(32);
   if (sensorRight == 1)
   {
     sensorRight=0;
   }
   else sensorRight =1;
   
   Serial.print(sensorRight);
   Serial.println(' ');
   
   Serial.println(); // uncomment this line if you are using raw values
   //delay(1000);
}


void Stop()
{  
  
    motor1.run(FORWARD);
    motor1.setSpeed(0);
  
    motor2.run(FORWARD);
    motor2.setSpeed(0);

}

void turnLeft()
{ 
    motor1.run(BACKWARD);
    motor1.setSpeed(100);
  
    motor2.run(FORWARD);
    motor2.setSpeed(100);
}


void turnRight()
{ 
    motor1.run(FORWARD);
    motor1.setSpeed(100);
  
    motor2.run(BACKWARD);
    motor2.setSpeed(100);
}

void goStraight()
{ 
    motor1.run(FORWARD);
    motor1.setSpeed(90);
  
    motor2.run(FORWARD);
    motor2.setSpeed(90);
}

void turnU()
{ 
    motor1.run(FORWARD);
    motor1.setSpeed(70);
  
    motor2.run(BACKWARD);
    motor2.setSpeed(70);
}

