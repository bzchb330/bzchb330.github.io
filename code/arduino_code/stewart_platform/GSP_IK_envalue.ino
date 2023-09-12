// fully functional Arduino code for GSP
// A0 for potentio meter
#include <Wire.h>
#include <DMREG2.h> 
#include <math.h>

// offsets
float x_t   = 0.0; // x axis (mm)
float y_t   = 0.0; // y axis (mm)
float z_t   = 0.0; // z axis (mm)
// rotations
float psi   = 0.0; // x axis
float theta = 0.0; // y axis
float phi   = 0.0; // z axis 
float arm = 10.5;  // servo arm length (mm) (a)
float rod = 80.0;  // rod length       (mm) (s)
//coordinates of servo_0 rotation center in base frame 
float x_b = 15.0;  
float y_b = 50.0;
float z_b = 17.85;
//coordinates of platform rotation center in platform frame
float x_p = 16.0;
float y_p = 38.6865;
float z_p = 0.0 ;
// coordinates b
float b_i[3][6]={{-x_b, x_b, 0.0, 0.0, 0.0, 0.0},
                 { y_b, y_b, 0.0, 0.0, 0.0, 0.0},
                 { z_b, z_b, 0.0, 0.0, 0.0, 0.0}};
// rotation 120
float deg120         = 120/180.0*3.1415926; // in radians
float R_z_n_120[3][3]={{ cos(deg120), sin(deg120), 0.0},
                       {-sin(deg120), cos(deg120), 0.0},
                       { 0.0,         0.0,         1.0}};
double R_p_to_b [3][3]={{ 0, 0, 0.0},
                        { 0, 0, 0.0},
                        { 0, 0, 0.0}};
// coordinates p
float p_i[3][6] ={{-41.5035, -25.5035, 0.0, 0.0, 0.0, 0.0},
                 {  5.4846,   33.1997, 0.0, 0.0, 0.0, 0.0},
                 {  0.0,       0.0,    0.0, 0.0, 0.0, 0.0}};
// coordinates p2 p=[p_2 p_3 p_4 p_5 p_6 p_1];
float p_i2[3][6]={{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                 { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                 { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
// translational matrix
double T     [3]  ={0,0,0};
//coordinates q_i
float q_i[3][6] ={{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
//length of ith leg l_i
float l_i[3][6] ={{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
//l_i square
float l_i_pow2[6]={ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//calculation of L = l_i^2-(s^2-a^2)
float L[6]       ={ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//calculation of M = 2*a*(Z_p-z_b)
float M[6]       ={ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//calculation of N = 2*a*(cos(beta)*(x_p-x_b)+sin(beta)(y_p-y_b));
float N[6]       ={ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//calculation of alpha = asind(L./sqrt(M.^2+N.^2))-atand(N./M) servo angle
float alpha_deg    [6] ={ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; //alpha in degrees
float alpha_pulse_1[3] ={ 0.0, 0.0, 0.0}; //alpha in pulse for servo driver(odd  servo)
float alpha_pulse_2[3] ={ 0.0, 0.0, 0.0}; //alpha in pulse for servo driver(even servo)
//beta value : angle of servo arm plane relative to x axis
float beta[6]    ={ 0, 3.141592653, 4.188790204, 1.04719755, 2.094395102, 5.23598775};      
float ArcSin     =0.0;
float deg_min    =0;
float deg_max    =0;
          
void setup() {
  Serial.begin(9600);
  pinMode(A0,INPUT);
}

void loop() {
  float input_1024 = average_filter(analogRead(A0)); //range 0~1024
  // home position psi=0, theta=0, phi=0
  // home position x_t=0, y_t  =0, z_t=95
  psi   = 0 /180.0*3.1415926; // x axis rotation (radian)
  theta = 0 /180.0*3.1415926; // y axis rotation (radian)
  phi   = map(input_1024,0,1024,-10,10) /180.0*3.1415926; // z axis rotation (radian)
  
  x_t   = 0.0;  // x axis (mm)
  y_t   = 0.0;  // y axis (mm)
  z_t   = 95; // z axis (mm)
  
  // memory pre-allocation for rotation matrix 
  float **R_p_to_b= new float *[3];
  for (int i = 0;i<3;i++){
    R_p_to_b[i] = new float [3];
  }
  float *T      = new float [3]; //in base frame
  
  R_p_to_b_matrix();              // obtain R_p_to_b
  b_coordinates();                // obtain b_i
  p_coordinates();                // obtain p_i
  q_coordinates();                // obtain q_i
  inverse_kinematics();           // obtain l_i L M N alpha
  
////  for (int i=0;i<3;i++){
//    for (int j=0;j<6;j++){
//      Serial.print(alpha_deg[j]);
//      Serial.print(' ');
////      Serial.print(alpha_pulse_2[j]);
////      Serial.print(' ');
//    }
//    Serial.print(x_t);
//    Serial.print(' ');
//     Serial.print(ArcSin);
//    Serial.print(' ');
//   Serial.println(); 
////  }
//  Serial.println(' ');
//  delay(100);
  
  CH_0_RealTime[9]=data_high(alpha_pulse_2[0]);
  CH_0_RealTime[8]=data_low (alpha_pulse_2[0]);
  
  CH_1_RealTime[9]=data_high(alpha_pulse_1[0]);
  CH_1_RealTime[8]=data_low (alpha_pulse_1[0]);
  
  CH_2_RealTime[9]=data_high(alpha_pulse_2[1]);
  CH_2_RealTime[8]=data_low (alpha_pulse_2[1]);
  
  CH_3_RealTime[9]=data_high(alpha_pulse_1[1]);
  CH_3_RealTime[8]=data_low (alpha_pulse_1[1]);
  
  CH_4_RealTime[9]=data_high(alpha_pulse_2[2]);
  CH_4_RealTime[8]=data_low (alpha_pulse_2[2]);

  CH_5_RealTime[9]=data_high(alpha_pulse_1[2]);
  CH_5_RealTime[8]=data_low (alpha_pulse_1[2]);

//  Serial.print(alpha_pulse_2[0]);
//  Serial.print(' ');
//  for (int i=0;i<10;i++){
//    Serial.print(CH_5_RealTime[i]);
//    Serial.print(' ');
//  }
//  Serial.println();
  
  Serial.write(CH_0_RealTime, 10);
  Serial.write(CH_1_RealTime, 10);
  Serial.write(CH_2_RealTime, 10);
  Serial.write(CH_3_RealTime, 10);
  Serial.write(CH_4_RealTime, 10);
  Serial.write(CH_5_RealTime, 10);
  Serial.println(x_t);
}//main loop

void R_p_to_b_matrix (){ //rotation matrix from platform to base frame
  double Rotation[3][3]={{cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)},
                        {cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi)},
                        {        -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta)}};
  for (int i=0;i<3;i++){
    for (int j=0;j<3;j++){
      R_p_to_b[i][j]=Rotation[i][j];
    }
  }
  T[0]=x_t;
  T[1]=y_t;
  T[2]=z_t;
  return;
}// rotation matrix

void b_coordinates(){
  for (int i=0;i<4;i++){
    for (int j=0;j<3;j++){
     b_i[j][i+2]=R_z_n_120[j][0]*b_i[0][i] + R_z_n_120[j][1]*b_i[1][i]+ R_z_n_120[j][2]*b_i[2][i];
    }
  } 
  return;
}//b coordinates

void p_coordinates(){
  for (int i=0;i<4;i++){
    for (int j=0;j<3;j++){
     p_i[j][i+2]=R_z_n_120[j][0]*p_i[0][i] + R_z_n_120[j][1]*p_i[1][i]+ R_z_n_120[j][2]*p_i[2][i];
    }
  } 
  for (int i=0;i<5;i++){
    for (int j=0;j<3;j++){
      p_i2[j][i]=p_i[j][i+1];
    }
  }
  p_i2[0][5]=p_i[0][0];
  p_i2[1][5]=p_i[1][0];
  p_i2[2][5]=p_i[2][0];
  return;
}//p coordinates

void q_coordinates(){
  for (int i=0;i<6;i++){
    for (int j=0;j<3;j++){
     q_i[j][i]=R_p_to_b[j][0]*p_i2[0][i] + R_p_to_b[j][1]*p_i2[1][i] + R_p_to_b[j][2]*p_i2[2][i] + T[j];
    }
  } 
  return;
}//q coordinates

void inverse_kinematics(){
  for (int i=0;i<6;i++){
    for (int j=0;j<3;j++){
     l_i[j][i]=q_i[j][i]-b_i[j][i];
    }
    l_i_pow2[i] = pow(l_i[0][i],2)+pow(l_i[1][i],2)+pow(l_i[2][i],2);
    L[i]        = l_i_pow2[i]-(pow(rod,2)-pow(arm,2));
    M[i]        = 2*arm*(q_i[2][i]-b_i[2][i]);
    N[i]        = 2*arm*(cos(beta[i])*(q_i[0][i]-b_i[0][i])+sin(beta[i])*(q_i[1][i]-b_i[1][i]));
    ArcSin=asin(L[i]/sqrt(pow(M[i],2)+pow(N[i],2)));
    if (abs(ArcSin)<1){ //arcsin实数解
      //-9.675: round to zero
      //+90: servo angle according to home position
      alpha_deg  [i] = (asin(L[i]/sqrt(pow(M[i],2)+pow(N[i],2)))-atan(N[i]/M[i]))*57.29577951-9.675; 
    }
  } 
  alpha_pulse_2[0] = map(alpha_deg[0]+90,0,180,500,2500)-100; //CH_0
  alpha_pulse_2[1] = map(alpha_deg[2]+90,0,180,500,2500)+67;  //CH_2
  alpha_pulse_2[2] = map(alpha_deg[4]+90,0,180,500,2500)-88;  //CH_4

  alpha_pulse_1[0] = map(alpha_deg[1]+90,180,0,500,2500)-111; //CH_1
  alpha_pulse_1[1] = map(alpha_deg[3]+90,180,0,500,2500)-189; //CH_3
  alpha_pulse_1[2] = map(alpha_deg[5]+90,180,0,500,2500)-178; //CH_5
  return;
}//li calculation

int average_filter(int input){//average filter for potentio meter
  int total = 0;
  for (int i=0;i<5;i++){
    total = total + input;
  }
  int value = total / 5;
  return value;
}//filter

int data_high(int pulse){
  char s1[3]={0}; 
  sprintf(s1, "%x", pulse); 
  char hex_H[1]={s1[0]};
  int data_H=0;
  sscanf(hex_H, "%x", &data_H); 
  return data_H;
}
int data_low(int pulse){
  char s1[3]={0}; 
  sprintf(s1, "%x", pulse); 
  char hex_L[3]={s1[1],s1[2]};
  int data_L=0;
  sscanf(hex_L, "%x", &data_L);
  return data_L;
}
