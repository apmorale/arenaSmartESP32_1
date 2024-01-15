#include <Arduino.h>
#include <Stepper.h>
#define PIN_PIR 27 
#define DIR_PIN 12
#define STEP_PIN 14
#define DELAY_US 25000
#define LED_inter 26
#define button_0 25

#define IN1 16
#define IN2 17
#define IN3 5
#define IN4 18

volatile int value_boton;
volatile int val;
bool prevSensorState = HIGH;
bool prevInterruption=LOW;

int actividad;
const int stepsPerRevolution = 400;
Stepper myStepper(stepsPerRevolution, 16, 17, 5, 18);

void IRAM_ATTR isr2() {
    Serial.println("Se ha detectado movimiento");
    digitalWrite(LED_inter, HIGH);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}


void giroHorario(int grados) {
  int steps = map(grados, 0, 360, 0, stepsPerRevolution);
  myStepper.setSpeed(2);
  myStepper.step(steps);
}

void giroAntiHorario(int grados) {
  int steps = map(grados, 0, 360, 0, -stepsPerRevolution);
  myStepper.setSpeed(2);
  myStepper.step(steps);
}


void activacionLimpieza(){
  Serial.println("Limpieza en camino...");
  giroAntiHorario(60);
  delayMicroseconds(5000);
  giroHorario(60);
  
}

void writeToSerial2Buffer(char str) {
  Serial2.print(str);
}

void setup() {
  // Inicio comunicacion
  Serial.begin(115200);
  Serial2.begin(115200);

  myStepper.setSpeed(60);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //pinMode(DIR_PIN, OUTPUT);
  //pinMode(STEP_PIN, OUTPUT);
  pinMode(PIN_PIR, INPUT);
  pinMode(LED_inter, OUTPUT);
  pinMode(button_0, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_PIR), isr2, FALLING);
  delay(500);
}

void loop() {
  value_boton = digitalRead(button_0);

  if (digitalRead(PIN_PIR) == LOW && prevSensorState == HIGH) {
    // Si hay presencia y el estado anterior fue HIGH, incrementar la variable actividad
    actividad += 1;
    Serial.println("Presencia CORRECTA. Actividad: " + String(actividad));
    delay(1000);  // Esperar 1 segundo para evitar múltiples detecciones en un corto período
  }

  prevSensorState = digitalRead(PIN_PIR);  // Actualizar el estado anterior del sensor


  if (actividad == 2 || value_boton == LOW) {
    delayMicroseconds(1000000);
    Serial.println("ACTIVIDAD O BOTOOOON");
    // Si la actividad acumulada es suficiente o se presiona el botón, activar la limpieza
    activacionLimpieza();
    actividad=0;
  }
}