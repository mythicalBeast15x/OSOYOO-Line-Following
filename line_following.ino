
/*Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.*/


//Define L298N Dual H-Bridge Motor Controller Pins
 

/*From left to right, connect to D3,A1-A3 ,D10*/
#define LFSensor_0 A0  //OLD D3
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 A4  //OLD D10

#define FAST_SPEED 160
#define MID_SPEED 140
#define SLOW_SPEED  130     //back speed
#define speedPinR 5    //  RIGHT PWM pin connect MODEL-X ENA
#define RightMotorDirPin1  12    //Right Motor direction pin 1 to MODEL-X IN1 
#define RightMotorDirPin2  11    //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6    // Left PWM pin connect MODEL-X ENB
#define LeftMotorDirPin1  7    //Left Motor direction pin 1 to MODEL-X IN3 
#define LeftMotorDirPin2  8   //Left Motor direction pin 1 to MODEL-X IN4 

//sensor pin definitions
#define echo_pin  4//2
#define trig_pin  10
#define buzzer  13


void disableBuzzer(){
  digitalWrite(buzzer, HIGH);
}
void soundAlarm(){
  digitalWrite(buzzer, LOW);
  delay(500);
  disableBuzzer();
}
int testNum = 40;
int getDistance(){
  long avgDist = 0;
  for(int x = 0; x < testNum; x++){
    long elapsed_time;
    digitalWrite(trig_pin, LOW);
    //Serial.println("Sensed");
    delayMicroseconds(5);
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(15);
    digitalWrite(trig_pin, LOW);
    
    elapsed_time = pulseIn(echo_pin, HIGH);
    long distance = (elapsed_time * 0.0135039)/2;
    avgDist += distance;
    //Serial.println(avgDist);
  }
  //Serial.println(avgDist);
  avgDist /= testNum;
  return avgDist;

}

/*motor control*/
void go_Advance(void)  //Forward
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
}
void go_Left(int t=0)  //Turn left
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}
void go_Right(int t=0)  //Turn right
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}
void go_Back(int t=0)  //Reverse
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}
void stop_Stop()    //Stop
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,LOW);
}
/*set motor speed */
void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L); 
  analogWrite(speedPinR,speed_R);   
}


void setup()
{
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, HIGH); 

  pinMode(trig_pin, OUTPUT); 
  pinMode(echo_pin,INPUT); 

  pinMode(RightMotorDirPin1, OUTPUT); 
  pinMode(RightMotorDirPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
 
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop_Stop();//stop move  

  Serial.begin(9600);   // initialize serial for debugging

}

boolean flag=false;
void loop()
{ 
 auto_tracking();
} //end of loop
 
char sensor[5];
 /*read sensor value string, 1 stands for black, 0 starnds for white, i.e 10000 means the first sensor(from left) detect black line, other 4 sensors detected white ground */
String read_sensor_values()
{   int sensorvalue=32;
  sensor[0]= digitalRead(LFSensor_0);
 
  sensor[1]=digitalRead(LFSensor_1);
 
  sensor[2]=digitalRead(LFSensor_2);
 
  sensor[3]=digitalRead(LFSensor_3);
 
  sensor[4]=digitalRead(LFSensor_4);
  sensorvalue +=sensor[0]*16+sensor[1]*8+sensor[2]*4+sensor[3]*2+sensor[4];
  
  String senstr= String(sensorvalue,BIN);
  senstr=senstr.substring(1,6);


  return senstr;
}

void auto_tracking(){
  String sensorval= read_sensor_values();
  Serial.println(sensorval);
  int dist = getDistance();
  Serial.println(dist);
  
  //if(sensorval == "00100" /*|| sensorval == "01110"*/){

    //go_Advance();  //Go straight
    //set_Motorspeed( 110,110);
  //}
  if(sensorval=="10000")
  { 
    //The black line is in the left of the car, need  left turn 
    go_Left();  //Turn left
    set_Motorspeed(FAST_SPEED,FAST_SPEED);
  }
  if (sensorval=="10100"  || sensorval=="01000" || sensorval=="01100" || sensorval=="11100"  || sensorval=="10010" || sensorval=="11010")
  {
    go_Advance();  //Turn slight left
    set_Motorspeed(0,FAST_SPEED);
  }
  if (    sensorval=="00001"  ){ //The black line is  on the right of the car, need  right turn 
    go_Right();  //Turn right
    set_Motorspeed(FAST_SPEED,FAST_SPEED);
  }
  if (sensorval=="00011" || sensorval=="00010"  || sensorval=="00101" || sensorval=="00110" || sensorval=="00111" || sensorval=="01101" || sensorval=="01111"   || sensorval=="01011"  || sensorval=="01001")
  {
    go_Advance();  //Turn slight right
    set_Motorspeed( FAST_SPEED,0);
  }
  
  
  if(sensorval == "00100" || sensorval == "01110"){

    go_Advance();  //Go straight
    set_Motorspeed( SLOW_SPEED,SLOW_SPEED);
  }

  if (sensorval=="01111"){
    stop_Stop();   //The car front touch stop line, need stop
    set_Motorspeed(0,0);
  }
    
  if(getDistance() < 6){
    stop_Stop();   
    set_Motorspeed(0,0);
    soundAlarm();
    while(getDistance <= 6){

      stop_Stop();
    }
  }

}
