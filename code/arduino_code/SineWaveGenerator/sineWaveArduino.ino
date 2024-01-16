float f = 50.0;    //signal frequency
float fs = 1000.0; //sampling frequency
float sig;
float t;
float val = 0.0;


void setup(){
  pinMode(10,OUTPUT);
  pinMode(A0,INPUT);
  Serial.begin(9600);
}

void loop(){
  
  for(int i=0;i<500;i++){
    val = analogRead(A0);
    float scale = map(0.1*analogRead(A0)+0.9*val,0,350,1,60);
    t = (float)i/fs;
    sig = (scale*(sin(2*3.14*f*t))+127);
    Serial.println(sig);
    analogWrite(10,sig);
    // delay(2);
  }
}