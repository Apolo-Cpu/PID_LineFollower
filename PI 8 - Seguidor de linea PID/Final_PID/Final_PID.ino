#include <analogWrite.h>
//Asegurate de instalar todas las librerias correspondientes
#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// ----------------------- blynk ----------------------------------------
#define BLYNK_PRINT Serial


#define BLYNK_TEMPLATE_ID "TMPL2dblfNcJr"
#define BLYNK_TEMPLATE_NAME "Proyecto i"
#define BLYNK_AUTH_TOKEN "T62fD1NIZgp5Z9hdkSsxlhUb6xInWlge"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "prueba";
char pass[] = "987654321Leidy";
// ----------------------- blynk ----------------------------------------

float Kp = 8;
float Ki = 0.01;
float Kd = 3;
int P, I, D;

int lastError = 0;
boolean onoff = false;

// ----------------------- Velocidad Motores -----------------------

const uint8_t maxspeeda = 250;
const uint8_t maxspeedb = 250;
const uint8_t basespeeda = 50;  // Motor A - Izquierdo
const uint8_t basespeedb = 100;  // Motor B - Derecho

//Pines IN del 1 al 4
int aphase = 5;
int aenbl = 18;
int bphase = 19;
int benbl = 21;
int wifi=22;

// Leds - check
int LED_BUILTIN = 23;
int LED_CALIBRACION = 2;

//Botones Calibracion e Inicio
int buttoncalibrate = 15;
int buttonstart = 4;  // 15 INICIO 



void setup() {

  Serial.begin(9600);

  pinMode(aphase, OUTPUT);
  pinMode(aenbl, OUTPUT);
  pinMode(bphase, OUTPUT);
  pinMode(benbl, OUTPUT);
  //Configuracion de los sensores qtr
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 13, 12, 14, 27, 26, 25, 33, 32 }, SensorCount);  //check
  qtr.setEmitterPin(35);                                                                //LEDON PIN

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_CALIBRACION, OUTPUT);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  Serial.println("wifi conectado");
  pinMode(wifi, OUTPUT);
  digitalWrite(wifi, HIGH),delay(100),digitalWrite(wifi,LOW),delay(50),digitalWrite(wifi, HIGH),delay(100),digitalWrite(wifi,LOW),delay(50);


  boolean Ok = false;
  while (Ok == false) {  //El ciclo no comenzará hasta que el robot esté calibrado
    
    Blynk.run();

    if (digitalRead(buttoncalibrate) == HIGH) {
      calibration();  //Calibra el Vehiculo robot unos 10 segundos
      Ok = true;
    }
  }
  forward_brake(0, 0);
}


//Calibracion de los Sensores
void calibration() {
 digitalWrite(LED_CALIBRACION, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
   digitalWrite(LED_CALIBRACION, LOW); 
  Serial.println(" CALIBRACION FINALIZADA ");
}

//Boton Inicio del Sistema
void loop() {
   Blynk.run();
  BLYNK_WRITE(V0);

  if (digitalRead(buttonstart) == HIGH) {
    digitalWrite(LED_BUILTIN, HIGH);
    onoff = !onoff;
    delay(onoff ? 1000 : 50);  // Si onoff es true, espera 1000 ms; si es false, espera 50 ms
  }

  if (onoff) {
    PID_control();
  } else {
    forward_brake(0, 0);
  }
    digitalWrite(LED_BUILTIN, LOW); 
}

void forward_brake(int posa, int posb) {
  // Establecer los valores apropiados para aphase y bphase para que el robot vaya recto
  digitalWrite(aphase, LOW);  // IN1
  digitalWrite(bphase, LOW);  // IN2
  analogWrite(aenbl, posa);   // IN3 - Posa Velocidad
  analogWrite(benbl, posb);   // IN4 - Posb Velocidad

}

void PID_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 3500 - position;

  P = error;
  I += error;
  D = error - lastError;
  lastError = error;

  int motorspeed = P * Kp + I * Ki + D * Kd;

  int motorspeeda = basespeeda + motorspeed;
  int motorspeedb = basespeedb - motorspeed;

if (motorspeeda > maxspeeda) {

  motorspeeda = maxspeeda;

 }



 if (motorspeeda < 0) {

  motorspeeda = 0;

 }

 if (motorspeedb < 0) {

  motorspeedb = 0;

 } 

  Serial.print(motorspeeda);
  Serial.print(" ");
  Serial.println(motorspeedb);

 Blynk.virtualWrite(V1,motorspeed);

  forward_brake(motorspeeda, motorspeedb);
}

BLYNK_WRITE(V0)
{
  buttoncalibrate = param.asInt();

}