/***************************************************************
* Desarrollado por Garikoitz Martínez [garikoitz.info] [06/2020]
* https://garikoitz.info/blog/?p=890
***************************************************************/
/***************************************************************
* Librerías
***************************************************************/
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Wire.h>
#include <PID_v1.h>
/***************************************************************
* Variables
***************************************************************/
  //Hélice
  const int INA = 10; //Motor Forward
  const int INB = 11; //Motor Reverse
  const int Interruptor = 12;
  const int pot = 0;                                                                                //Potenciómetro en pin A0
  unsigned long previousMillis = 0;
  int Ts = 100; //Sample time 100ms
  //Giroscópio
  const int mpuAddress = 0x68;  //Puede ser 0x68 o 0x69
  MPU6050 mpu(mpuAddress);
  //Offsets de calibración
  #define MPU6050_ACCEL_OFFSET_X -2936
  #define MPU6050_ACCEL_OFFSET_Y 1412
  #define MPU6050_ACCEL_OFFSET_Z 962
  #define MPU6050_GYRO_OFFSET_X  58
  #define MPU6050_GYRO_OFFSET_Y  85
  #define MPU6050_GYRO_OFFSET_Z  15
  int ax, ay, az;
  int gx, gy, gz;
  float ang_x, ang_y;
  float ang_x_prev, ang_y_prev;
  long tiempo_prev;
  float dt;
  //Control PID
  boolean modo = false; //false=auto true=manual
  float tmp = 0;
  float tmp_ant = 0;
  float OP = 0;
  double SetpointP, InputP, OutputP;
  double Kp=6, Ki=3, Kd=2;
  PID myPID(&InputP, &OutputP, &SetpointP, Kp, Ki, Kd,P_ON_E, DIRECT);    //PI-D
  //PID myPID(&InputP, &OutputP, &SetpointP, Kp, Ki, Kd,P_ON_M, DIRECT);  //I-PD
/***************************************************************
* SETUP
***************************************************************/
void setup() 
{ 
  //Hélice
  pinMode(INA,OUTPUT); 
  pinMode(INB,OUTPUT); 
  pinMode(Interruptor, INPUT);
  //PID 
  myPID.SetSampleTime(50);        //PID calcula cada 50ms. Por defecto es cada 100ms.
  myPID.SetOutputLimits(0, 100);
  myPID.SetMode(MANUAL);
  //Giroscópio
  mpu.initialize();
  //Calibración
  mpu.setXAccelOffset(MPU6050_ACCEL_OFFSET_X);
  mpu.setYAccelOffset(MPU6050_ACCEL_OFFSET_Y);
  mpu.setZAccelOffset(MPU6050_ACCEL_OFFSET_Z);
  mpu.setXGyroOffset(MPU6050_GYRO_OFFSET_X);
  mpu.setYGyroOffset(MPU6050_GYRO_OFFSET_Y);
  mpu.setZGyroOffset(MPU6050_GYRO_OFFSET_Z);
  Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));
  Serial.begin(9600);
}
/***************************************************************
* BUCLE PRINCIPAL
***************************************************************/
void loop() 
{
  if (millis() - previousMillis > Ts) 
  {
    previousMillis = millis();
    if (digitalRead(12) == HIGH){
      modo=true;//Manual
      myPID.SetMode(MANUAL);
    }else{
      digitalWrite(INB ,LOW);
      analogWrite(INA,OP);   //Forward
      modo=false;//Automático
      myPID.SetMode(AUTOMATIC);
    }
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);
    //Calcular los ángulos con acelerometro
    double accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
    double accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
    dt = (millis()-tiempo_prev)/1000.0;
    tiempo_prev=millis();
    //Ángulo de rotación y filtro complementario 
    ang_x = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x;
    ang_x_prev=ang_x;
    //
    /***************************************************************
    * Modo MANUAL - Potenciómetro ---> PWM Motor CC
    ***************************************************************/
    if (modo==true){
      SetpointP=0;
      //Leo el sensor
      InputP = mapf(ang_x, -30, 30, 0, 100);          //Filtro complementario
      //Leo el potenciómetro
      //Evitar saltos del 0.85% al leer el potenciómetro
      tmp = analogRead(pot);
      tmp = mapf(tmp, 0, 1023, 0, 100);
      if(abs(tmp-tmp_ant)>0.85){
        OP = tmp;
      }
      tmp_ant=OP;
      digitalWrite(INB ,LOW);
      analogWrite(INA,OP/0.392156);   //FORWARD 0-100 a 0-255 para PWM
      /***************************************************************
      * DEBUG PUERTO SERIE (Para Arduino COM Plotter)
      ***************************************************************/
      Serial.print("#");             //Char inicio
      Serial.print(SetpointP,0);     //SP
      Serial.write(" ");             //Char separador
      Serial.print(InputP,3);        //PV
      Serial.write(" ");             //Char separador
      Serial.print(OP,0);            //OP
      Serial.println();
    }
    /***************************************************************
    * Modo AUTOMÁTICO - PID
    ***************************************************************/
    if (modo==false){
      tmp = analogRead(pot);
      tmp = mapf(tmp, 0, 1023, 0, 100);
      //Evitar saltos del 0.85% al leer el potenciómetro
      if(abs(tmp-tmp_ant)>0.85){
        SetpointP = tmp;
      }
      tmp_ant=SetpointP;
      InputP = mapf(ang_x, -30, 30, 0, 100);
      myPID.Compute();
      digitalWrite(INB ,LOW);
      analogWrite(INA,OutputP/0.392156); //FORWARD 0-100 a 0-255 para PWM
      /***************************************************************
      * DEBUG PUERTO SERIE (Para Arduino COM Plotter)
      ***************************************************************/
      Serial.print("#");                //Char inicio
      Serial.print(SetpointP,0);        //SP
      Serial.write(" ");                //Char separador
      Serial.print(InputP,3);           //PV
      Serial.write(" ");                //Char separador
      Serial.print(OutputP,0);          //OP
      Serial.println();
    }
  }
}
/***************************************************************
* FUNCIONES
***************************************************************/
//Función MAP adaptada a punto flotante.
double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
