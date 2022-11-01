// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;
#define echoPin 13 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 12 //attach pin D3 Arduino to pin Trig of HC-SR04

// Valores RAW (sin procesar) del acelerometro  en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;
//Variables Globales
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
int velocidad; 
int pwmCarrito;
int pwmPendulo;
int angulo;
float Mkc=0;
float Mkp;


float Ts=0.015; //Calculamos tiempo de muestreo

//Valores PID Discreto Pendulo
float kpp=-54.8683679286121;
float kip=2.04775747078051;
float kdp=0.0904146272305055;
float Np=83.9576991418546;
//Valores PID Discreto Carrito
float kpc=0.0857737044561583;
float kic=0.0832220773397635;
float kdc=1.96746042698113;
float Nc=7.47498988089675;
//Valores para p0-p4 Topologia Ideal Pendulo
float p0p=kpp;
float p1p=(kpp*kip)*(Ts/2);
float p2p=(2*Np*kpp*kdp);
float p3p=(2+(Np*Ts));
float p4p=(Np*Ts)-2;
//Valores b0-b2 y a0-a2 para funcion antes de inversa Z Pendulo
float b0p=(p0p*p3p)+(p1p*p3p)+(p2p);
float b1p=(-p0p*p3p)+(p0p*p4p)+(p1p*p3p)+(p1p*p4p)-(2*p2p);
float b2p=(-p0p*p4p)+(p1p*p4p)+(p2p);
float a0p=p3p;
float a1p=(-p3p+p4p);
float a2p=-p4p;
//Valores para p0-p4 Topologia Ideal Carrito
float p0c=kpc;
float p1c=(kpc*kic)*(Ts/2);
float p2c=(2*Nc*kpc*kdc);
float p3c=(2+(Nc*Ts));
float p4c=(Nc*Ts)-2;
//Valores b0-b2 y a0-a2 para funcion antes de inversa Z Carrito
float b0c=(p0c*p3c)+(p1c*p3c)+(p2c);
float b1c=(-p0c*p3c)+(p0c*p4c)+(p1c*p3c)+(p1c*p4c)-(2*p2c);
float b2c=(-p0c*p4c)+(p1c*p4c)+(p2c);
float a0c=p3c;
float a1c=(-p3c+p4c);
float a2c=-p4c;

// Motor Derecha
int ENA = 5;
int IN1 = 9;
int IN2 = 10;

// Motor Izquierda
int ENB = 6;
int IN3 = 7;
int IN4 = 8;
 
//Initial conditions Pendulo
float Mk1p=0,Mk2p=0, Ek1p=0, Ek2p=0;
//Initial conditions Carrito
float Mk1c=0,Mk2c=0, Ek1c=0, Ek2c=0;




void setup ()
{
  Serial.begin(9600);    //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el acelerometro
  // Declaramos los pines como salidas
  pinMode (ENA, OUTPUT);
  pinMode (ENB, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode(trigPin, OUTPUT); // Trigger pin como output
  pinMode(echoPin, INPUT); // Echo pin como input
  
  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
}
void loop ()
{
  LecturaSensores();
  //Suponemos el valor inicial del angulo en 96 para simular que el pendulo se encuentra derecho y la distancia a la que queremos el carrito
  //int setpointAngulo=5;
  float setPointcarrito=40;
  //Calculamos los valores de los errores para cada uno de los PID
  float Ekp=(Mkc+5)-angulo;//float Ekp=setpointAngulo-angulo;float Ekp=Mkc-angulo; //Ya que funcione el pendulo cambiamos la ecuacion para la implementacion de ambos PID
  float Ekc=setPointcarrito-distance;

//  Serial.println("Valor Error Pendulo");
//  Serial.println(Ekp);
//  Serial.println("Valor Error Carrito");
//  Serial.println(Ekc);
//  delay(500);
  

  //PID pendulo
  Mkp=((-a1p*Mk1p)-(a2p*Mk2p)+(b0p*Ekp)+(b1p*Ek1p)+(b2p*Ek2p))/a0p;
  //PID carrito
  Mkc=((-a1c*Mk1c)-(a2c*Mk2c)+(b0c*Ekc)+(b1c*Ek1c)+(b2c*Ek2c))/a0c;
  
  
  //si x es 1 calculamos pwm carrito, si x es 2 calculamos pwm pendulo
  CalcularPWM(1,Mkc);
  CalcularPWM(2,Mkp);
  //Llamamos a la funcion Mover Carrito para mover el carro a la posicion deseada
  MoverCarrito(Ekc,pwmCarrito);
  //MoverPendulo(Ekp,pwmCarrito);
  //Llamamos a la funcion MoverPendulo para mover el pendulo a la posicion deseada
  if(-2>Ekc<2){
     MoverPendulo(angulo,pwmCarrito);
  }


  //Retroalimentacion de valores de Mk y Ek Pendulo
  Ek2p=Ek1p;Ek1p=Ekp;Mk2p=Mk1p;Mk1p=Mkp;
  //Retroalimentacion de valores de Mk y Ek Carrito
  Ek2c=Ek1c;Ek1c=Ekc;Mk2c=Mk1c;Mk1c=Mkc;

}
void Adelante (int pwm)
{

  //Direccion de motor 1
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  analogWrite (ENA, pwm); //Velocidad de motor 1
  //Direccion de motor 2
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
  analogWrite (ENB, pwm); //Velocidad de motor 2
  
}

void Atras (int pwm)
{
  //Direccion de motor 1
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  analogWrite (ENA, pwm);  //Velocidad de motor 1
  //Direccion de motor 2
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
  analogWrite (ENB, pwm); //Velocidad de motor 2
}

void Parar ()
{
  //Direccion de motor 1
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);
  analogWrite (ENA, 0); //Velocidad de motor 1
  //Direccion de motor 2
  digitalWrite (IN3, LOW);
  digitalWrite (IN4, LOW);
  analogWrite (ENB, 0); //Velocidad de motor 2
  
}

void LecturaSensores(){
  
  // Leer las aceleraciones del acelerometro
  sensor.getAcceleration(&ax, &ay, &az);
  angulo=map(az,-32768,32768,-90,90);
  // Ponemos Trigger en low
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Ponemos trigger en HIGH para poder iniciar la medicion del ultrasonico
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Leemos el valor de echo
  duration = pulseIn(echoPin, HIGH);
  //Calculamos el valor de distancia del ultrasonico en CM
  distance = duration * 0.034 / 2; 
  }

void CalcularPWM(int x,float PID){
    
    if(x==1){
      pwmCarrito=map(PID,-10,50,120,255);
    }
   else if(x==2){

      pwmPendulo=map(PID,-1000,1000,140,180);

    }
}

void MoverCarrito(float error,int pwm){

    
    if(error<0){
        Atras(pwm);
        delay(30);
        Parar();  
      }
    else if(error>0){
        Adelante(pwm);
        delay(30);
        Parar();
      }
    else if(-2>error<2){
        Parar();
        delay(50);
      }
    
  }
void MoverPendulo(float error,int pwm){
  
//    if(pwm>160){
//      pwm=160;
//      }
//     else if(pwm<130){
//      pwm=130;
//      }
//      Serial.println("Valor error");
//      Serial.println(error);
//      delay(500);    
    delay(30);

    if(2<=error>=12){
        Parar();
        delay(30);
      }
    else if(error<2){
       Adelante(pwm);
       delay(50);
       Parar();
       //delay(10);
      }
    else if(error>12){

       Atras(pwm);
       delay(50);
       Parar();
       //delay(10);
      }
      
  }
  
