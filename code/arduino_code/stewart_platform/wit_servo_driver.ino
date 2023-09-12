#include <Wire.h>
//#include <DMREG.h>

//3v3----3v3
//RX------1
//TX------0
//GND----GND

unsigned char test0_Speed1_Position_45[15]={0xff,0x01,0x00,0x0a,0x00,0xff,0x02,0x00,0xe8,0x03};
unsigned char test0_Speed1_Position_90[15]={0xff,0x01,0x00,0x0a,0x00,0xff,0x02,0x00,0xdc,0x05}; 
unsigned char test1_Speed1_Position_45[15]={0xff,0x01,0x01,0x0a,0x00,0xff,0x02,0x01,0xe8,0x03};  
unsigned char test1_Speed1_Position_90[15]={0xff,0x01,0x01,0x0a,0x00,0xff,0x02,0x01,0xdc,0x05}; 
unsigned char test2_Speed1_Position_45[15]={0xff,0x01,0x02,0x0a,0x00,0xff,0x02,0x02,0xe8,0x03};  
unsigned char test2_Speed1_Position_90[15]={0xff,0x01,0x02,0x0a,0x00,0xff,0x02,0x02,0xdc,0x05};    

void setup() 
{
  Serial.begin(9600);
  pinMode(A0,INPUT);
}

void loop() 
{
  int output = average_filter(analogRead(A0));
  int Value = map(output,0,1024,0,255);
  
  if (Value>100){
    Serial.write(test0_Speed1_Position_45, 10);
    Serial.write(test1_Speed1_Position_45, 10);
    Serial.write(test2_Speed1_Position_45, 10);
    Serial.println(Value);
  }else{
    Serial.write(test0_Speed1_Position_90, 10);
    Serial.write(test1_Speed1_Position_90, 10);
    Serial.write(test2_Speed1_Position_90, 10);
    Serial.println(Value);
  }
}

int average_filter(int input){
  int total = 0;
  for (int i=0;i<5;i++){
    total = total + input;
  }
  int value = total / 5;
  return value;
}
