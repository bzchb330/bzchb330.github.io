#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define ENCODER_A 2                         //编码器引脚
#define ENCODER_B 4

//Gyro Variables
float elapsedTime, time_1, timePrev;        //Variables for time control
int gyro_error=0;                           //We use this variable to only calculate once the gyro data error
float Gyr_rawX;                             //Here we store the raw data read 
float Gyro_angle_x;                         //Here we store the angle value obtained with Gyro data
float Gyro_raw_error_x;                     //Here we store the initial gyro data error

//Acc Variables
int   acc_error=0;                         //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180/3.141592654;        //This value is for pasing from radians to degrees values
float Acc_rawX, Acc_rawY, Acc_rawZ;        //Here we store the raw data read 
float Acc_angle_x;                         //Here we store the angle value obtained with Acc data
float Acc_angle_error_x;                   //Here we store the initial Acc data error
float Total_angle_x;

// PID parameters
double desired_Position=56.5;              // target position
double motor_PWM;
double elapsedTime_2, time_2, timePrev_2, error, previous_Error;
double Balance_KP =27, Balance_KI =0, Balance_KD =7.5;
double velocity_KP=0.0005, velocity_KI=0, velocity_KD=0;
int flag;
// Motor parameter
float oldtimes, newtimes, time_d;   //时间变量
float vel;
float Position_new = 10000, Position_old = 10000;
float speed_feedback, speed_feedback_derror, speed_feedback_pre,speed_integral,vel_PWM;
void setup() {
  
  Wire.begin();                           //begin the wire comunication
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);             //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68);           //Start communication with the address found during search.
  Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true); 

  Serial.begin(9600);                     
  pinMode(ENCODER_A, INPUT);                  // 编码器引脚A
  pinMode(ENCODER_B, INPUT);                  // 编码器引脚B
  pinMode(A0 ,INPUT);                         // potentio meter
  pinMode(7  ,OUTPUT);                        // IN1 驱动
  pinMode(8  ,OUTPUT);                        // IN2 驱动
  pinMode(11 ,OUTPUT);                        // PWM 驱动
  attachInterrupt(0, READ_ENCODER_A, CHANGE); // 中断
  
  if(acc_error==0)
  {
    for(int a=0; a<200; a++)
    {
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);                              //Ask for the 0x3B register- correspond to AcX
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,6,true); 
      
      Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; //each value needs two registres
      Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
      Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ;
      
      Acc_angle_error_x = Acc_angle_error_x + ((atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg));      
      if(a==199)
      {
        Acc_angle_error_x = Acc_angle_error_x/200;
        acc_error=1;
      }
    }
  }//end of acc error calculation   

  if(gyro_error==0)
  {
    for(int i=0; i<200; i++)
    {
      Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
      Wire.write(0x43);                        //First adress of the Gyro data
      Wire.endTransmission(false);
      Wire.requestFrom(0x68,4,true);           //We ask for just 4 registers 
         
      Gyr_rawX=Wire.read()<<8|Wire.read();     //Once again we shif and sum   
      /*---X---*/
      Gyro_raw_error_x = Gyro_raw_error_x + (Gyr_rawX/32.8); 
      if(i==199)
      {
        Gyro_raw_error_x = Gyro_raw_error_x/200;
        gyro_error=1;
      }
    }
  }//end of gyro error calculation   
}

void loop() {
  newtimes = millis();
  time_d   = newtimes-oldtimes;
  vel      = (Position_new-Position_old)*1000/(time_d);
  float Actual_position = MPU_READ();
  vel = (Position_new-Position_old)*1000/(newtimes-oldtimes);
  error     = Actual_position - desired_Position;
  if(abs(error)<=25){
    flag = 1;
    motor_PWM = Balance_Control(abs(error))-5;
    if(motor_PWM<60){
      motor_PWM=60;
    }
    if(motor_PWM>250){
      motor_PWM=250;
    }
    if(error>0){
      Negative(motor_PWM);
    }
    else{
      Positive(motor_PWM);
    }
    Serial.print("Xº: ");
    Serial.print(Actual_position);
    Serial.print(" ");  
    Serial.print(motor_PWM);
    Serial.print(" ");  
    Serial.print(vel);
    Serial.println(" ");      
  }
  else{
    flag = 0;
    motor_stop();
    Serial.print("Xº: ");
    Serial.print(Actual_position);
    Serial.print(' ');
    Serial.println("motor is shut done");    
  }
//  delay(10);
}
//end loop

// PID balance main start
float Balance_Control(float Error)
{  
   int balance;
   float pid_d = Balance_KD*((Error - previous_Error)/elapsedTime);
   balance = Balance_KP*(Error+speed_control()) + pid_d; 
   previous_Error = Error;  
   return balance;
}
// PID balance main end

//PID velocity control start
float speed_control()
{
  speed_feedback = 0 - abs(vel);                 //vel是飞轮转速
  speed_feedback_derror = speed_feedback - speed_feedback_pre;
  speed_feedback_pre = speed_feedback;
  speed_integral += speed_feedback * velocity_KI;
  vel_PWM = speed_feedback * velocity_KP + speed_integral + velocity_KD * speed_feedback_derror; //得到飞轮转速的补偿量，在后续计算过程中会用到。
  return vel_PWM;
}
//PID velocity control end

//motor turning  start
void Positive(double PID_PWM){            //Error <0
//  Serial.print("Positive");
//  Serial.print(PID_PWM);
  digitalWrite(7,HIGH);  // IN1 high
  digitalWrite(8,LOW);   // IN2 low motor positive
  analogWrite(11,PID_PWM); // PWM
//  delay(5);
}
void Negative(double PID_PWM){            //Error >0 
//  Serial.print("Negative");
//  Serial.print(PID_PWM);
  digitalWrite(7,LOW);    // IN1 LOW
  digitalWrite(8,HIGH);   // IN2 HIGH motor negative
  analogWrite(11,PID_PWM);  // PWM
//  delay(5);
}
void motor_stop(){               // abs(Error) >10 
  digitalWrite(7,HIGH);    // IN1 HIGH
  digitalWrite(8,HIGH);    // IN2 HIGH motor stop
}
//motor turning  end

//Encoder value start
void READ_ENCODER_A() {
    if (digitalRead(ENCODER_A) == HIGH) {     
    if (digitalRead(ENCODER_B) == LOW)      Position_new++;  //根据另外一相电平判定方向
    else      Position_new--;
  }
    else {    
    if (digitalRead(ENCODER_B) == LOW)      Position_new--; //根据另外一相电平判定方向
    else     Position_new++;
  }
}
// encoder value end

// MPU read start
float MPU_READ(){
  timePrev = time_1;                         // the previous time is stored before the actual time read
  time_1 = millis();                         // actual time read
  elapsedTime = (time_1 - timePrev) / 1000;  //divide by 1000 in order to obtain seconds
    Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
    Wire.write(0x43);                        //First adress of the Gyro data
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,4,true);           //We ask for just 4 registers
    Gyr_rawX=Wire.read()<<8|Wire.read();     //Once again we shif and sum
    Gyr_rawX = (Gyr_rawX/32.8) - Gyro_raw_error_x; 
    
    /*---X---*/
    Gyro_angle_x = Gyr_rawX*elapsedTime;
  //////////////////////////////////////Acc read/////////////////////////////////////

  Wire.beginTransmission(0x68);     //begin, Send the slave adress (in this case 68) 
  Wire.write(0x3B);                 //Ask for the 0x3B register- correspond to AcX
  Wire.endTransmission(false);      //keep the transmission and next
  Wire.requestFrom(0x68,6,true);    //We ask for next 6 registers starting withj the 3B  
   Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; //each value needs two registres
  Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
 /*---X---*/
 Acc_angle_x = (atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg) - Acc_angle_error_x;
 //////////////////////////////////////Total angle and filter/////////////////////////////////////
 /*---X axis angle---*/
 Total_angle_x = 0.98 *(Total_angle_x + Gyro_angle_x) + 0.02*Acc_angle_x;
 return Total_angle_x;
}
// MPU read end
