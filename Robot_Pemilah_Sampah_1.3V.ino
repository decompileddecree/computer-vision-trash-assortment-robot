#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "PCA9685.h"

unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long startMillis2;
unsigned long currentMillis;
const unsigned long period = 200;  //the value is a number of milliseconds

unsigned long lastCommandTime = 0;
const unsigned long timeoutMs = 100;

// jarak robot ke meja
int jarak = 28;

// Serial Variable
int val[2];  // Store data from packet
int prev_val[2];

// Motor variables
int Xpos;
int Ypos;
int Zpos;

// Display variables
int lcdX = 0;
int lcdY = 0;
int lcdZ = 0;
int lcdLF = 0;
int lcdRF = 0;
int lcdLR = 0;
int lcdRR = 0;
int lcdSD = 0;
int lcdSM = 0;
int lcdSU = 0;
int lcdP = 0;


// PID parameters (adjust as needed)
float KpY = 3.0, KiY = 0.07, KdY = 0.3;
float KpZ = 5.0, KiZ = 0.07, KdZ = 0.3;

struct PIDState {
  float lastError;
  float integral;
  unsigned long lastTime;
};

PIDState pidY = {0, 0, 0};
PIDState pidZ = {0, 0, 0};


int targetDistance = 28;  // cm



//Ultravalue
float ultraValueL = 0;
float ultraValueR = 0;

// UltraSonic Pin Config
int TrigL = 51;
int EchoL = 50;
int TrigR = 53;
int EchoR = 52;

// Movement Speed settings
int kecepatan = 5;
int servoSpeed = 100;

// Motor A connections
int enA = 7;
int inA1 = 31;
int inA2 = 33;
// Motor B connections
int enB = 8;
int inB3 = 35;
int inB4 = 37;
// Motor C connections
int enC = 9;
int inC1 = 39;
int inC2 = 41;
// Motor D connections
int enD = 10;
int inD3 = 43;
int inD4 = 45;

// i2c LCD
LiquidCrystal_I2C lcd(0x26,20,4);

//PCA9685 Servo PWM Synth
PCA9685 pwmController1(B000000);
PCA9685 pwmController2(B000001);
PCA9685_ServoEval pwmServo1;
PCA9685_ServoEval pwmServo2;
PCA9685_ServoEval pwmServo3;
PCA9685_ServoEval pwmServo4;

int computePID(
  float setpoint, 
  float input, 
  float Kp, 
  float Ki, 
  float Kd, 
  PIDState &state, 
  float integralClamp
) {
  float error = setpoint - input;

  unsigned long now = millis();
  float dT = (now - state.lastTime) / 1000.0f;
  if (dT <= 0) dT = 0.001f;

  float pTerm = error * Kp;

  state.integral += error * dT;
  if (state.integral > integralClamp) state.integral = integralClamp;
  if (state.integral < -integralClamp) state.integral = -integralClamp;
  float iTerm = state.integral * Ki;

  float derivative = (error - state.lastError) / dT;
  float dTerm = derivative * Kd;

  state.lastError = error;
  state.lastTime = now;

  float output = pTerm + iTerm + dTerm;

  if (output > 35) output = 35;
  if (output < -35) output = -35;

  return (int)output;
}



void serialRead(){
  if (Serial.available() >= 4){
    uint8_t data [4];
    Serial.readBytes(data, 4);
    for (int i = 0; i < 2; i++){
      val[i] = (int16_t)((data[i*2+1]<<8)|data[i*2]);
    }
    lastCommandTime = millis();  // Update timeout
    //lcd.clear();
  }
}


void serialWrite(int value){
  byte* data = (byte*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    Serial.write(data[i]);
  }
}

int clamp(int value) {
  return constrain(value, 0, 200);
}

void servoatas(int derajatA) {
  pwmController1.setChannelPWM(0, pwmServo1.pwmForAngle((180 - derajatA)-90));
  pwmController1.setChannelPWM(1, pwmServo2.pwmForAngle(derajatA-90));
}

void servotengah(int derajatB) {
  pwmController1.setChannelPWM(3, pwmServo4.pwmForAngle((180 - derajatB)-90));
  pwmController1.setChannelPWM(2, pwmServo3.pwmForAngle(derajatB-90));
}

void servobawah(int derajatC) {
  // Batasi nilai agar selalu dalam rentang 10–1500
  if (derajatC < 0) derajatC = 0;
  else if (derajatC > 2100) derajatC = 2100;

  // Ubah derajat ke nilai PWM (dikalikan 16)
  int pwmValue = derajatC;
  
  // Set nilai PWM ke channel 0
  pwmController2.setChannelPWM(0, pwmValue);
}


void motorStop() {
  digitalWrite(enA, LOW);
  digitalWrite(enB, LOW);
  digitalWrite(enC, LOW);
  digitalWrite(enD, LOW);
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, LOW);
  digitalWrite(inB3, LOW);
  digitalWrite(inB4, LOW);
  digitalWrite(inC1, LOW);
  digitalWrite(inC2, LOW);
  digitalWrite(inD3, LOW);
  digitalWrite(inD4, LOW);
}

void motorMove(int Y, int X, int Z) {
  /*
   * Y = Maju Mundur
   * X = Geser Kiri Kanan
   * Z = Rotasi Kiri Kanan
   * 
   * Y = value+ = Forward
   * Y = value- = Reverse
   * 
   * X = value+ = Move to Right POV From Back of The Robot
   * X = value- = Move to Left POV From Back of The Robot
   * 
   * Z = value+ = Rotating Clockwise POV From Above a Robot
   * Z = value- = Rotating CounterClockwise POV From Above a Robot
   * 
   * LF = Left Front (Kiri Depan)
   * RF = Right Front (Kanan Depan)
   * LR = Left Rear (Kiri Belakang)
   * RR = Right Rear (Kanan Belakang)
   * 
   * Formula:
   * 
   * LF = Y + X + Z
   * RF = Y - X - Z
   * LR = Y + X - Z
   * RR = Y - X + Z
   * 
   * enA = LF
   * enB = RF
   * enC = LR
   * enD = RR
   * 
   * A1 = + LF
   * A2 = - LF
   * B3 = + RF
   * B4 = - RF
   * C1 = + LR
   * C2 = - LR
   * D3 = + RR
   * D4 = - RR
   * 
   */

  Y = -Y;
  
  int LF = Y + X + Z;
  int RF = Y - X - Z;
  int LR = Y + X - Z;
  int RR = Y - X + Z;

  //lcdLF = LF * kecepatan;
  //lcdRF = RF * kecepatan;
  //lcdLR = LR * kecepatan;
  //lcdRR = RR * kecepatan;

  lcd.setCursor (13,0);
  lcd.print("LF");lcd.print(LF * kecepatan);
  lcd.setCursor (13,1);
  lcd.print("RF");lcd.print(RF * kecepatan);
  lcd.setCursor (13,2);
  lcd.print("LR");lcd.print(LR * kecepatan);
  lcd.setCursor (13,3);
  lcd.print("RR");lcd.print(RR * kecepatan);

  if (LF > 0) {
    analogWrite(enA, clamp(int(LF * kecepatan)));
    digitalWrite(inA1, HIGH);
    digitalWrite(inA2, LOW);
  } else if (LF < 0) {
    analogWrite(enA, clamp(int((-LF) * kecepatan)));
    digitalWrite(inA1, LOW);
    digitalWrite(inA2, HIGH);
  } else {
    analogWrite(enA, 0); // or digitalWrite(enA, LOW);
    digitalWrite(inA1, LOW);
    digitalWrite(inA2, LOW);
  }

  if (RF > 0) {
    analogWrite(enB, clamp(int(RF * kecepatan)));
    digitalWrite(inB3, HIGH);
    digitalWrite(inB4, LOW);
  } else if (RF < 0) {
    analogWrite(enB, clamp(int((-RF) * kecepatan)));
    digitalWrite(inB3, LOW);
    digitalWrite(inB4, HIGH);
  } else {
    analogWrite(enB, 0); // or digitalWrite(enB, LOW);
    digitalWrite(inB3, LOW);
    digitalWrite(inB4, LOW);
  }

  if (LR > 0) {
    analogWrite(enC, clamp(int(LR * kecepatan)));
    digitalWrite(inC1, HIGH);
    digitalWrite(inC2, LOW);
  } else if (LR < 0) {
    analogWrite(enC, clamp(int((-LR) * kecepatan)));
    digitalWrite(inC1, LOW);
    digitalWrite(inC2, HIGH);
  } else {
    analogWrite(enC, 0); // or digitalWrite(enC, LOW);
    digitalWrite(inC1, LOW);
    digitalWrite(inC2, LOW);
  }

  if (RR > 0) {
    analogWrite(enD, clamp(int(RR * kecepatan)));
    digitalWrite(inD3, HIGH);
    digitalWrite(inD4, LOW);
  } else if (RR < 0) {
    analogWrite(enD, clamp(int((-RR) * kecepatan)));
    digitalWrite(inD3, LOW);
    digitalWrite(inD4, HIGH);
  } else {
    analogWrite(enD, 0); // or digitalWrite(enD, LOW);
    digitalWrite(inD3, LOW);
    digitalWrite(inD4, LOW);
  }
}

void pump(int State){
  /*
   * State = 0 == OFF
   * State = 1 == Suck
   * State = 2 == Blow
   * 
   */

  if (State == 0){
      digitalWrite(22, LOW);
      digitalWrite(24, LOW);
      digitalWrite(A15, HIGH); //selenoid gede mati
      digitalWrite(A14, HIGH); //relay modul mati
  }
  if (State == 1){
      digitalWrite(A14, LOW); //relay modul hidup
      digitalWrite(A15, LOW); 
      digitalWrite(22, HIGH);
      digitalWrite(24, LOW);
  }
  if (State == 2){
      digitalWrite(A14, LOW);
      digitalWrite(A15, LOW);   
      digitalWrite(22, LOW);
      digitalWrite(24, HIGH);
  }
}

float ultraSonicL() {
  float pulse;
  float cm;
  digitalWrite(TrigL,LOW);
  delayMicroseconds(2);
  digitalWrite(TrigL,HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigL,LOW);
  pulse = pulseIn(EchoL,HIGH);
  cm = pulse * 0.034 / 2;
  return cm;
  //ultraValueL
}

float ultraSonicR() {
  float pulse;
  float cm;
  digitalWrite(TrigR,LOW);
  delayMicroseconds(2);
  digitalWrite(TrigR,HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigR,LOW);
  pulse = pulseIn(EchoR,HIGH);
  cm = pulse * 0.034 / 2;
  return cm;
  //ultraValueR
}



void maju() {
  // PID Constants (tune as needed)
 // double KpY = 2.0, KiY = 0.1, KdY = 0.5;
 // double KpZ = 3.0, KiZ = 0.1, KdZ = 0.6;

  // PID States
 // double prevErrorY = 0, integralY = 0;
 // double prevErrorZ = 0, integralZ = 0;

  // Time tracking
 // unsigned long lastPIDTime = millis();
    pidY.integral = 0;
    pidY.lastError = 0;
    pidY.lastTime = millis();

    pidZ.integral = 0;
    pidZ.lastError = 0;
    pidZ.lastTime = millis();

  while (true) {
    currentMillis = millis();

    if (currentMillis - startMillis >= 20) {
        static bool readLeft = true;
        if (readLeft) {
            ultraValueL = ultraSonicL();
        } else {
            ultraValueR = ultraSonicR();
        }
        readLeft = !readLeft;
        
        startMillis = currentMillis;
    }

    // Compute timing
    double avgDist = (ultraValueL + ultraValueR) / 2.0;
    double diff = ultraValueL - ultraValueR;

    int Y = computePID(targetDistance, avgDist, KpY, KiY, KdY, pidY, 1000);
    int Z = computePID(0, diff, KpZ, KiZ, KdZ, pidZ, 150);

    // Optional deadband for Z to reduce jitter
    //if (abs(Z) < 2) Z = 0;

    // Display
    lcd.setCursor(0,0); lcd.print("Maju        ");
    lcd.setCursor(0,1); lcd.print("Dist: "); lcd.print(avgDist); lcd.print("   ");
    lcd.setCursor(0,2); lcd.print("Y:"); lcd.print((int)-Y); lcd.print(" Z:"); lcd.print((int)-Z); lcd.print("   ");

    // Move
    motorMove((int)-Y, 0, (int)-Z);

    // Stop condition
    if (avgDist <= jarak) {
      lcd.setCursor(0,0); lcd.print("Stop        ");
      motorStop();  // or motorMove(0,0,0);

      delay(100);

      serialWrite(1);  // Send back to Python

      val[0] = 0;
      val[1] = 0;
  
     // pidY.integral = 0;
     // pidY.lastError = 0;
     // pidY.lastTime = millis();
  
     // pidZ.integral = 0;
     // pidZ.lastError = 0;
     // pidZ.lastTime = millis();

      break;
    }
  }
}


void geserRobot(int y,int x,int z){

  motorMove((int)-y,x,(int)-z);
  //lcdX = x;
  //lcdY = -y;
  //lcdZ = -z;
  lcd.setCursor(0,2);
  lcd.print("Geser");
  lcd.setCursor(6,0);
  lcd.print("Y :");lcd.print(-y);
  lcd.setCursor(6,1);
  lcd.print("X :");lcd.print(x);
  lcd.setCursor(6,2);
  lcd.print("Z :");lcd.print(-z);
  
}

void setup() {
  pinMode(A15, OUTPUT); // Relay Ch2
  pinMode(A14, OUTPUT); // Relay Ch2
  digitalWrite(A15, HIGH);
  digitalWrite(A14, HIGH);
  startMillis = millis();
  Serial.begin(9600);
  Wire.begin();
  lcd.init();
  lcd.backlight();

  pidY.lastTime = millis();
  pidZ.lastTime = millis();


  pwmController1.resetDevices();
  pwmController2.resetDevices();
  pwmController1.init();
  pwmController2.init();
  pwmController1.setPWMFreqServo();
  pwmController2.setPWMFrequency(200);
  
  // Motor pin setup
  pinMode(enA, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(inB3, OUTPUT);
  pinMode(inB4, OUTPUT);
  pinMode(enC, OUTPUT);
  pinMode(inC1, OUTPUT);
  pinMode(inC2, OUTPUT);
  pinMode(enD, OUTPUT);
  pinMode(inD3, OUTPUT);
  pinMode(inD4, OUTPUT);

  //DC Pump
  pinMode(22, OUTPUT);
  pinMode(24, OUTPUT);

  //UltraSonic
  pinMode(TrigL, OUTPUT);
  pinMode(EchoL, INPUT);
  pinMode(TrigR, OUTPUT);
  pinMode(EchoR, INPUT);
  ultraValueL = ultraSonicL();
  ultraValueR = ultraSonicR();

  motorStop();
  servoatas(20);
  servotengah(2);
  servobawah(2100);
  pump(0);
  



}

void loop() {
  
  // List address
  // addres = data
  // 0 = stop = 0
  // 1 = Y axis = int
  // 2 = X axis = int
  // 3 = Z axis = int
  // 4 = Low Servo = int
  // 5 = Mid Servo = int
  // 6 = Up Servo = int
  // 7 = Pump = int = (0 = off) (1 = Suck) (2 = Blow)
  // 9 = maju = 0
  // 10 = geser = int
  // 11 = sampah up
  // 12 = sampah down
  // 13 = put sampah
  currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
if (currentMillis - startMillis >= 20) {
    static bool readLeft = true;
    if (readLeft) {
        ultraValueL = ultraSonicL();
    } else {
        ultraValueR = ultraSonicR();
    }
    readLeft = !readLeft;
    lcd.setCursor(0, 0); lcd.print(ultraValueL);
    lcd.setCursor(0, 1); lcd.print(ultraValueR);
    startMillis = currentMillis;
}

  if (currentMillis - startMillis2 >= 200){
     lcd.clear();
    if (val[0] == 0){
     


      lcd.setCursor(0, 0); lcd.print(ultraValueL);
      lcd.setCursor(0, 1); lcd.print(ultraValueR);
      lcd.setCursor(0, 2); lcd.print("STOP ");

        lcd.setCursor(4,0);
        lcd.print("L");
        lcd.setCursor(4,1);
        lcd.print("R");
    }
  //  Display Pump
      
      if (lcdP == 0){
        lcd.setCursor(19,0);
        lcd.print("P");
        lcd.setCursor(19,1);
        lcd.print("O");
        lcd.setCursor(19,2);
        lcd.print("F");
        lcd.setCursor(19,3);
        lcd.print("F");
      }
      if (lcdP == 1){
        lcd.setCursor(19,0);
        lcd.print("S");
        lcd.setCursor(19,1);
        lcd.print("U");
        lcd.setCursor(19,2);
        lcd.print("C");
        lcd.setCursor(19,3);
        lcd.print("K");
      }
      if (lcdP == 2){
        lcd.setCursor(19,0);
        lcd.print("B");
        lcd.setCursor(19,1);
        lcd.print("L");
        lcd.setCursor(19,2);
        lcd.print("O");
        lcd.setCursor(19,3);
        lcd.print("W");
      }
  //  Display Servo Atas
      lcd.setCursor(9,3);
      lcd.print("U");
      lcd.setCursor(10,3);
      lcd.print(lcdSU);
      if (lcdSU < 100){
      lcd.setCursor(12,3);
      lcd.print(" ");
      }
      if (lcdSU < 10){
      lcd.setCursor(11,3);
      lcd.print(" ");
      }
  //  Display Servo Tengah 
      lcd.setCursor(5,3);
      lcd.print("M");
      lcd.setCursor(6,3);
      lcd.print(lcdSM);
      if (lcdSM < 100){
      lcd.setCursor(8,3);
      lcd.print(" ");
      }
      if (lcdSM < 10){
      lcd.setCursor(7,3);
      lcd.print(" ");
      } 
  //  Display Servo Bawah
      lcd.setCursor(0,3);
      lcd.print("D");
      lcd.setCursor(1,3);
      lcd.print(lcdSD);
      if (lcdSD < 1000){
      lcd.setCursor(4,3);
      lcd.print(" ");
      }
      if (lcdSD < 100){
      lcd.setCursor(3,3);
      lcd.print(" ");
      }
      if (lcdSD < 10){
      lcd.setCursor(2,3);
      lcd.print(" ");
      }
 //   Display XYZ Movement
      
      lcd.setCursor(6,0);
      lcd.print("Y :");lcd.print(lcdY);
      lcd.setCursor(6,1);
      lcd.print("X :");lcd.print(lcdX);
      lcd.setCursor(6,2);
      lcd.print("Z :");lcd.print(lcdZ);
 //   Display Motor Variable
      lcd.setCursor (13,0);
      lcd.print("LF");lcd.print(lcdLF);
      lcd.setCursor (13,1);
      lcd.print("RF");lcd.print(lcdRF);
      lcd.setCursor (13,2);
      lcd.print("LR");lcd.print(lcdLR);
      lcd.setCursor (13,3);
      lcd.print("RR");lcd.print(lcdRR); 
    
   startMillis2 = currentMillis; 
  }
  
  serialRead();
  
  if (millis() - lastCommandTime > timeoutMs) {
    val[0] = 0; // stop command
    val[1] = 0;
  }
  
  
switch (val[0]) {
  case 0:
    motorStop();
    break;

  case 1:
    motorMove(val[1], 0, 0);
    break;

  case 2:
    motorMove(0, val[1], 0);
    break;

  case 3:
    motorMove(0, 0, val[1]);
    break;

  case 4:
    servobawah(val[1]);
    lcd.setCursor(0, 2); lcd.print("SERVO");
    lcdSD = val[1];
   // lastPIDTime = millis();
    break;

  case 5:
    servotengah(val[1]);
    lcd.setCursor(0, 2); lcd.print("SERVO");
    lcdSM = val[1];
    break;

  case 6:
    servoatas(val[1]);
    lcd.setCursor(0, 2); lcd.print("SERVO");
    lcdSU = val[1];
    break;

  case 7:
    pump(val[1]);
    lcdP = val[1];
    break;

  case 9:
    maju();
    break;

  case 10:
    break;

  case 11:
    break;

  default:
    motorStop();
    val[0] = 0;
    // Handle unknown command if needed
    break;
}

  if (val[0] == 10){
        // Compute Y and Z from ultrasonic
    double avgDist = (ultraValueL + ultraValueR) / 2.0;
    double diff = ultraValueL - ultraValueR;

    int Y = computePID(targetDistance, avgDist, KpY, KiY, KdY, pidY, 150);
    int Z = computePID(0, diff, KpZ, KiZ, KdZ, pidZ, 150);

    geserRobot(Y, val[1], Z);
  }

}
