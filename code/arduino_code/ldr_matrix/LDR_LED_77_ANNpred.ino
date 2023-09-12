#include "Neurona.h"
#include "model.h"
MLP mlp(NET_INPUTS, NET_OUTPUTS, layerSizes, MLP::LOGISTIC, initW, true);

int led_p[7] = {2,3,4,5,6,7,8};        // LED positive
int led_n[7] = {40,42,44,46,48,50,52}; // LED negative
int ldr_r[7] = {A6,A5,A4,A3,A2,A1,A0}; // LDR read-in pin
int ldr_p[7] = {22,24,26,28,30,32,34}; // LDR row selection pin
int iter = 0;    // iteration times
int itSet = 100; // read 100 times and take the average

// store realTime value
double reading [7][7] ={{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
// store mean of real time value
double readMean [7][7] ={{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
// store summed realTime value
double temp_val [7][7] ={{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
void setup() {
  pinMode(10,OUTPUT); // reference voltage pin
  for( int i = 0;i<7;i++){
    pinMode(led_p[i],OUTPUT);
    pinMode(led_n[i],OUTPUT);
    pinMode(ldr_r[i],INPUT);
    pinMode(ldr_p[i],OUTPUT);
  }
  // switch off all LEDs first
  for( int i = 0;i<7;i++){
    digitalWrite(led_p[i],0);
    digitalWrite(led_n[i],HIGH);
  }
   Serial.begin(9600);
}
void loop() {
  iter = iter + 1; //update the iteration times
  //////////////////////////////
  //first readin all LDR sensor
  //////////////////////////////
  // set reference voltage
  int num = 255;
  digitalWrite(10,num);

  // first set all row selection pins to V_ref
  for( int i = 0;i<7;i++){
    digitalWrite(ldr_p[i],num);
  }

  if(iter<itSet){ // smaller than 50 iterations
    // readin
    for( int i = 0;i<7;i++){
      digitalWrite(ldr_p[i],LOW);  // set selected row to GND
      for ( int j = 0; j<7 ; j++){
        reading[i][j]=analogRead(ldr_r[j]); // store read in value
        temp_val[i][j] = temp_val[i][j] + analogRead(ldr_r[j]); 
      }
      digitalWrite(ldr_p[i],num); // set row back to V_ref
    }
    
  }else{      // larger than 50 iterations
    //////////////////////////
    // after taking average, subtract the realtime value by mean
    /////////////////////////
    for( int i = 0;i<7;i++){
      for ( int j = 0; j<7 ; j++){
        readMean[i][j] = temp_val[i][j]/itSet; 
      }
    }

     // readin - mean
     int max_val = 0;
    for( int i = 0;i<7;i++){
      digitalWrite(ldr_p[i],LOW);  // set selected row to GND
      for ( int j = 0; j<7 ; j++){
        int val = (analogRead(ldr_r[j])-readMean[i][j])*2;
        if(val<60){
          val = 0;
        }
        reading[i][j]=val;          // store read in value
        
        max_val = max(max_val,val); // obtain the max value
      }
      digitalWrite(ldr_p[i],num);   // set row back to V_ref
    }

  //////////////
  // LED display
  /////////////

  for( int i = 0; i<7 ; i++){
    for ( int j = 0; j<7 ; j++){
      int val_led = map(reading[i][j],1,max_val,0,50);
      if(max_val<60){
        val_led = 0;
      }
      
      digitalWrite(led_p[i],val_led);
      digitalWrite(led_n[j],LOW);
      delay(1);
      digitalWrite(led_p[i],0);
      digitalWrite(led_n[j],HIGH);
    }
  }
//  Serial.println(max_val);
int k = 0;
  for( int i = 0;i<7;i++){
    for ( int j = 0; j<7 ; j++){
      if(max_val<1){
        max_val = 1;
      }
      k=k+1;
      netInput[k] = float(reading[i][j])/max_val;
//      Serial.print(float(reading[i][j])/max_val); 
//      Serial.print(' ');
    }
  }
  int index = mlp.getActivation(netInput);
  mlpClass = Class[index];
  Serial.println(mlpClass);
    
  }// end else

}
