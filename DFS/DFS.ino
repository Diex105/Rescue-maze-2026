#include "Navegacion.h"           // ← Algoritmo DFS + Dijkstra

#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Encoder.h>
#include <Adafruit_VL53L0X.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_APDS9960.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>

#define DEBOUNCE_TICKS 5
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ===============================
// NEOPIXEL
// ===============================
#define NEO_PIN 33
#define NUMPIXELS 8
Adafruit_NeoPixel pixels(NUMPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

// ===============================
// COMUNICACION OpenMV
// ===============================
#define RX_PIN 17
#define TX_PIN 16
SoftwareSerial openmv(RX_PIN, TX_PIN);
int n = 20;
volatile int colores[4] = {0, 0, 0, 0};
Adafruit_APDS9960 apds;
int contadorAzul = 0;
bool pausaAzul = false;
unsigned long tiempoAzul = 0;
bool ignorarAzul = false;
bool interrumpidoPorAzul = false;

// ===============================
// VELOCIDADES Y DISTANCIA
// ===============================
#define BASE_SPEED 180
#define TURN_SPEED_MAX 200
#define TURN_SPEED_MIN 150
#define TICKS_POR_CM 225.0

// ===============================
// ENCODERS
// ===============================
Encoder encLeft(15, 14);
Encoder encRight(11, 12);
int modoLateral = 0;
int contadorFrentes = 0;

// ===============================
// IMU
// ===============================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
bool bnoActivo = false;
float headingFiltrado = 0;
float headingObjetivo = 0;

// ===============================
// SENSORES TOF
// ===============================
#define NUM_SENSORES 11
int xshutPins[NUM_SENSORES] = { 1,3,0,5,6,2,8,9,10,7,4 };
uint8_t direcciones[NUM_SENSORES] = {
  0x30,0x31,0x32,0x33,0x34,
  0x35,0x36,0x37,0x38,0x39,0x3A
};
Adafruit_VL53L0X sensores[NUM_SENSORES];
bool sensorActivo[NUM_SENSORES];
float Dist[11] = {0,0,0,0,0,0,0,0,0,0,0};

// ===============================
// MOTORES
// ===============================
#define L_F1 34
#define L_F2 35
#define L_F_PWM 36
#define L_B1 39
#define L_B2 38
#define L_B_PWM 37
#define R_F1 31
#define R_F2 30
#define R_F_PWM 29
#define R_B1 26
#define R_B2 27
#define R_B_PWM 28

// Variables globales control 30cm
volatile long lastLeftTicks = 0;
volatile long lastRightTicks = 0;
unsigned long lastControlTime = 0;
float integralLeft = 0, integralRight = 0;
float lastErrorLeft = 0, lastErrorRight = 0;

// Variables globales PID GIRO
float pidIntegral = 0;
float pidErrorAnterior = 0;
unsigned long pidTiempoAnterior = 0;

// Limit switch
#define LLS 41
#define RLS 40

// Bumper / stall
const int STALL_PWM_UMBRAL = 120;
const unsigned long STALL_MS = 120;
const float RETROCEDER_CM = 4.0;
static long lastLeftTicksLocal = 0;
static long lastRightTicksLocal = 0;
static unsigned long stallStartMs = 0;
static bool enContacto = false;

// =============================== MOTORES

void setupMotor(int in1, int in2, int pwm) {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(pwm, OUTPUT);
}

void stopMotors() {
  analogWrite(L_F_PWM, 0);
  analogWrite(L_B_PWM, 0);
  analogWrite(R_F_PWM, 0);
  analogWrite(R_B_PWM, 0);
  delayMicroseconds(20);
}

void forwardRaw(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  static int lastLeftSpeed  = 0;
  static int lastRightSpeed = 0;
  int maxAccel = 30;
  leftSpeed  = constrain(leftSpeed,  lastLeftSpeed  - maxAccel, lastLeftSpeed  + maxAccel);
  rightSpeed = constrain(rightSpeed, lastRightSpeed - maxAccel, lastRightSpeed + maxAccel);
  lastLeftSpeed  = leftSpeed;
  lastRightSpeed = rightSpeed;

  digitalWrite(L_F1, LOW);  digitalWrite(L_F2, HIGH);
  digitalWrite(L_B1, HIGH); digitalWrite(L_B2, LOW);
  digitalWrite(R_F1, HIGH); digitalWrite(R_F2, LOW);
  digitalWrite(R_B1, LOW);  digitalWrite(R_B2, HIGH);

  analogWrite(L_F_PWM, leftSpeed);
  analogWrite(L_B_PWM, leftSpeed);
  analogWrite(R_F_PWM, rightSpeed);
  analogWrite(R_B_PWM, rightSpeed);
}

void backwardRaw(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  static int lastLeftSpeed  = 0;
  static int lastRightSpeed = 0;
  int maxAccel = 30;
  leftSpeed  = constrain(leftSpeed,  lastLeftSpeed  - maxAccel, lastLeftSpeed  + maxAccel);
  rightSpeed = constrain(rightSpeed, lastRightSpeed - maxAccel, lastRightSpeed + maxAccel);
  lastLeftSpeed  = leftSpeed;
  lastRightSpeed = rightSpeed;

  digitalWrite(L_F1, HIGH); digitalWrite(L_F2, LOW);
  digitalWrite(L_B1, LOW);  digitalWrite(L_B2, HIGH);
  digitalWrite(R_F1, LOW);  digitalWrite(R_F2, HIGH);
  digitalWrite(R_B1, HIGH); digitalWrite(R_B2, LOW);

  analogWrite(L_F_PWM, leftSpeed);
  analogWrite(L_B_PWM, leftSpeed);
  analogWrite(R_F_PWM, rightSpeed);
  analogWrite(R_B_PWM, rightSpeed);
}

void turnLeftMotor(int velocidad) {
  digitalWrite(L_F1, HIGH); digitalWrite(L_F2, LOW);
  digitalWrite(L_B1, LOW);  digitalWrite(L_B2, HIGH);
  digitalWrite(R_F1, HIGH); digitalWrite(R_F2, LOW);
  digitalWrite(R_B1, LOW);  digitalWrite(R_B2, HIGH);
  analogWrite(L_F_PWM, velocidad);
  analogWrite(L_B_PWM, velocidad);
  analogWrite(R_F_PWM, velocidad);
  analogWrite(R_B_PWM, velocidad);
}

void turnRightMotor(int velocidad) {
  digitalWrite(L_F1, LOW);  digitalWrite(L_F2, HIGH);
  digitalWrite(L_B1, HIGH); digitalWrite(L_B2, LOW);
  digitalWrite(R_F1, LOW);  digitalWrite(R_F2, HIGH);
  digitalWrite(R_B1, HIGH); digitalWrite(R_B2, LOW);
  analogWrite(L_F_PWM, velocidad);
  analogWrite(L_B_PWM, velocidad);
  analogWrite(R_F_PWM, velocidad);
  analogWrite(R_B_PWM, velocidad);
}

// =============================== IMU

float getHeading() {
  if (!bnoActivo) return 0;
  return bno.getVector(Adafruit_BNO055::VECTOR_EULER).x();
}

float readHeadingRaw() {
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

float readHeadingAvg(uint8_t samples) {
  float sumSin = 0.0, sumCos = 0.0;
  for (uint8_t i = 0; i < samples; i++) {
    float h = readHeadingRaw() * DEG_TO_RAD;
    sumSin += sin(h);
    sumCos += cos(h);
    delay(2);
  }
  float avg = atan2(sumSin, sumCos) * RAD_TO_DEG;
  if (avg < 0) avg += 360.0;
  return avg;
}

float angleDiff(float target, float current) {
  float diff = target - current;
  while (diff >  180.0) diff -= 360.0;
  while (diff < -180.0) diff += 360.0;
  return diff;
}

float normalize360(float angle) {
  while (angle >= 360.0) angle -= 360.0;
  while (angle <    0.0) angle += 360.0;
  return angle;
}

void turnToHeading(float objetivo) {
  if (!bnoActivo) return;

  const float KP = 2.2, KI = 0.2, KD = 0.1;
  const int VEL_MAX = 240, VEL_MIN = 100;
  const float ANGULO_TOL = 0.6;
  const int SETTLE_MS = 180;
  const float FASE_FINE = 3.0;

  float integral = 0.0, lastError = 0.0;
  unsigned long lastUpdate = micros();
  unsigned long inTolStart = 0;
  bool lastTurnLeft = true;

  while (true) {
    float actual = readHeadingAvg(3);
    float error  = angleDiff(objetivo, actual);

    unsigned long ahora = micros();
    float dt = (ahora - lastUpdate) / 1000000.0;
    if (dt < 0.008) continue;

    if (abs(error) < 12.0) {
      integral += error * dt;
      integral  = constrain(integral, -25.0, 25.0);
    } else {
      integral = 0.0;
    }

    float dError    = (error - lastError) / dt;
    lastError       = error;
    float velocidad = KP * abs(error) + KI * abs(integral) + KD * abs(dError);

    if (abs(error) < 10.0) velocidad *= (0.5 + abs(error) * 0.035);

    velocidad = constrain(velocidad, (float)VEL_MIN, (float)VEL_MAX);

    if (abs(error) <= ANGULO_TOL) {
      if (inTolStart == 0) inTolStart = millis();
      if (millis() - inTolStart >= (unsigned long)SETTLE_MS) break;
      velocidad = max(35, (int)(velocidad * 0.6));
    } else {
      inTolStart = 0;
    }

    if (abs(error) > FASE_FINE) {
      if (error < 0) { turnLeftMotor((int)velocidad);  lastTurnLeft = true;  }
      else           { turnRightMotor((int)velocidad); lastTurnLeft = false; }
    } else {
      int pulseVel = max(30, (int)(velocidad * 0.5));
      if (error < 0) { turnLeftMotor(pulseVel);  lastTurnLeft = true;  }
      else           { turnRightMotor(pulseVel); lastTurnLeft = false; }
      delay(10);
      stopMotors();
      delay(8);
    }
    lastUpdate = ahora;
  }

  for (int i = VEL_MIN; i > 20; i -= 15) {
    if (lastTurnLeft) turnLeftMotor(i);
    else              turnRightMotor(i);
    delay(5);
  }
  stopMotors();
  delay(120);
}

float getHeadingFiltrado() {
  float nuevo = getHeading();
  headingFiltrado = 0.90 * headingFiltrado + 0.10 * nuevo;
  return headingFiltrado;
}

// =============================== LECTURAS TOF

void leerTOFS() {
  VL53L0X_RangingMeasurementData_t measure;
  for (int i = 0; i < NUM_SENSORES; i++) {
    sensores[i].rangingTest(&measure, false);
    if (measure.RangeStatus != 4) Dist[i] = measure.RangeMilliMeter;
    if (i == 6) { Dist[6] -= 15; delayMicroseconds(50); }
    if (i == 5) { Dist[5] -= 28; delayMicroseconds(50); }
    if (i == 2) { Dist[2] +=  8; delayMicroseconds(50); }
    delayMicroseconds(50);
  }
}

// =============================== LOGICA DFS (requerida por Navegacion.h)

#define UMBRAL_PARED 160.0

bool libreFrente() {
  return Dist[0] > UMBRAL_PARED;
}

bool libreIzquierda() {
  return Dist[1] > UMBRAL_PARED;
}

bool libreDerecha() {
  return Dist[4] > UMBRAL_PARED;
}

void giro90Der() {
  if (!bnoActivo) return;
  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("Girando 90...");
  display.display();

  float inicio   = readHeadingAvg(5);
  float objetivo = normalize360(inicio + 90.0);
  turnToHeading(objetivo);

  float fin    = readHeadingAvg(5);
  float girado = angleDiff(fin, inicio);

  display.clearDisplay();
  display.setCursor(5, 5);
  display.print("Giro: "); display.print(abs(girado), 1); display.println(" deg");
  display.setCursor(5, 25);
  display.print("Error: "); display.print(girado + 90.0, 1);
  display.display();
  delay(200);
}

void giro90Izq() {
  if (!bnoActivo) return;
  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("Girando 90...");
  display.display();

  float inicio   = readHeadingAvg(5);
  float objetivo = normalize360(inicio - 90.0);
  turnToHeading(objetivo);

  float fin    = readHeadingAvg(5);
  float girado = angleDiff(fin, inicio);

  display.clearDisplay();
  display.setCursor(5, 5);
  display.print("Giro: "); display.print(abs(girado), 1); display.println(" deg");
  display.setCursor(5, 25);
  display.print("Error: "); display.print(girado + 90.0, 1);
  display.display();
  delay(200);
}

void giro180() {
  if (!bnoActivo) return;
  float inicio   = readHeadingAvg(5);
  float objetivo = normalize360(inicio + 180.0);
  turnToHeading(objetivo);
  delay(300);
}

// =============================== AVANZAR OPTIMIZADO

void avanzar_optimizado(int DISTANCIA_CM) {
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  const float KP = 0.6, KI = 0.02, KD = 0.05;
  const int COMPENSACION_INERCIA = 50;
  long ticksObjetivo = (DISTANCIA_CM * TICKS_POR_CM) - COMPENSACION_INERCIA;

  float anguloObjetivo = readHeadingAvg(5);
  encLeft.write(0);
  encRight.write(0);

  float lastGyroError = 0;
  unsigned long lastTime = micros();
  int currentLeftSpeed = BASE_SPEED, currentRightSpeed = BASE_SPEED;

  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("Recto IMU...");
  display.display();

  while (true) {
    if (revisarUART(anguloObjetivo)) stopMotors();

    long leftTicks  = abs(encLeft.read());
    long rightTicks = abs(encRight.read());
    long promedio   = (leftTicks + rightTicks) / 2;

    unsigned long ahora_ms = millis();

    if (BASE_SPEED > STALL_PWM_UMBRAL) {
      if (leftTicks == lastLeftTicksLocal && rightTicks == lastRightTicksLocal) {
        if (stallStartMs == 0) stallStartMs = ahora_ms;
        else if (ahora_ms - stallStartMs > STALL_MS) enContacto = true;
      } else {
        stallStartMs = 0;
        enContacto   = false;
      }
    } else {
      stallStartMs = 0;
      enContacto   = false;
    }

    lastLeftTicksLocal  = leftTicks;
    lastRightTicksLocal = rightTicks;

    if (enContacto) {
      Serial.println("Contacto detectado: stall");
      stopMotors();
      delay(40);
      retroceder(RETROCEDER_CM);
      turnToHeading(anguloObjetivo);
      encLeft.write(0);
      encRight.write(0);
      enContacto          = false;
      lastLeftTicksLocal  = 0;
      lastRightTicksLocal = 0;
      stallStartMs        = 0;
      continue;
    }

    float distanciaRecorrida = promedio / TICKS_POR_CM;
    String color = detectarColor();

    if (color != "NONE") {
      reaccionColor(color, distanciaRecorrida);
      if (color == "NEGRO") return;
    }

    if (promedio >= ticksObjetivo) break;

    unsigned long currentTime = micros();
    float deltaTime = (currentTime - lastTime) / 1000000.0;

    if (deltaTime >= 0.01) {
      float anguloActual  = readHeadingRaw();
      float errorGyro     = angleDiff(anguloObjetivo, anguloActual);
      float derivativoGyro = (errorGyro - lastGyroError) / deltaTime;
      float correccion    = (KP * errorGyro) + (KD * derivativoGyro);
      lastGyroError = errorGyro;
      lastTime      = currentTime;

      float correccionLat   = controlLateralContinuo();
      float correccionTotal = correccion + correccionLat;

      currentLeftSpeed  = constrain(BASE_SPEED + (int)correccionTotal, BASE_SPEED - 60, BASE_SPEED + 60);
      currentRightSpeed = constrain(BASE_SPEED - (int)correccionTotal, BASE_SPEED - 60, BASE_SPEED + 60);
    }

    forwardRaw(currentLeftSpeed, currentRightSpeed);
    delayMicroseconds(200);
  }

  stopMotors();
  delay(80);

  // Detección azul al final del avance
  int azulCount = 0;
  for (int i = 0; i < 5; i++) {
    if (detectarColor() == "AZUL") azulCount++;
    delay(15);
  }
  if (azulCount >= 3) {
    navSetCheckpoint();   // ← Informa al navegador que es checkpoint
    Serial.println("AZUL → esperar 5s");
    for (int i = 5; i > 0; i--) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.println("AZUL");
      display.print("Tiempo: "); display.println(i);
      display.display();
      delay(1000);
    }
  }

  delay(100);
  turnToHeading(anguloObjetivo);

  leerTOFS();
  if (Dist[0] <= 180.0 && Dist[5] <= 180.0) {
    contadorFrentes++;
    if (contadorFrentes >= 3) {
      calibrar_limit();
      contadorFrentes = 0;
    }
  }
  delay(200);
}

// =============================== CALIBRACIONES

void calibrar_limit() {
  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("Calibrando...");
  display.display();

  int   cal_vel = 75;
  const float KP_CAL = 2.0;
  float promedio = 0;
  float anguloObjetivo = readHeadingAvg(10);

  while (true) {
    float actual    = readHeadingRaw();
    float error     = angleDiff(anguloObjetivo, actual);
    float correccion = error * KP_CAL;
    forwardRaw(cal_vel - (int)correccion, cal_vel + (int)correccion);
    if (digitalRead(LLS) == 1 && digitalRead(RLS) == 1) break;
    delay(5);
  }
  stopMotors();

  while (true) {
    float actual    = readHeadingRaw();
    float error     = angleDiff(anguloObjetivo, actual);
    float correccion = error * KP_CAL;
    backwardRaw(cal_vel + (int)correccion, cal_vel - (int)correccion);

    float d0 = 1000, d5 = 1000;
    VL53L0X_RangingMeasurementData_t measure;
    if (sensorActivo[0]) { sensores[0].rangingTest(&measure, false); if (measure.RangeStatus != 4) d0 = measure.RangeMilliMeter; }
    if (sensorActivo[5]) { sensores[5].rangingTest(&measure, false); if (measure.RangeStatus != 4) d5 = measure.RangeMilliMeter; }
    promedio = (d0 + d5) / 2.0;
    if (promedio >= 120.0) break;
  }
  stopMotors();
  turnToHeading(anguloObjetivo);

  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("OK! Dist:");
  display.setCursor(5, 25);
  display.print(promedio, 1); display.print(" mm");
  display.display();
  delay(500);
}

float controlLateralContinuo() {
  const float KP_LAT = 0.6;
  const float MIN_PARED = 40.0, MAX_PARED = 200.0, UMBRAL = 5.0;

  float s1 = 1000, s4 = 1000;
  VL53L0X_RangingMeasurementData_t measure;

  if (sensorActivo[1]) { sensores[1].rangingTest(&measure, false); if (measure.RangeStatus != 4) s1 = measure.RangeMilliMeter; }
  if (sensorActivo[4]) { sensores[4].rangingTest(&measure, false); if (measure.RangeStatus != 4) s4 = measure.RangeMilliMeter; }

  bool paredIzq = (s1 > MIN_PARED && s1 < MAX_PARED);
  bool paredDer = (s4 > MIN_PARED && s4 < MAX_PARED);
  float correccion = 0;

  if      (paredIzq && paredDer) { if (abs(s4 - s1) > UMBRAL) correccion = (s4 - s1) * KP_LAT; }
  else if (paredIzq)             { correccion = (120.0 - s1) * KP_LAT; }
  else if (paredDer)             { correccion = (s4 - 120.0) * KP_LAT; }

  return correccion;
}

void calibrar_lateral() {
  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("Alineando...");
  display.display();

  const float KP_LAT = 0.6;
  const int   VEL_BASE = 55;
  const float MIN_PARED = 40.0, MAX_PARED = 400.0;

  unsigned long startTime = millis();
  const unsigned long TIMEOUT = 600;
  float anguloObjetivo = readHeadingAvg(5);

  while (true) {
    modoLateral = 0;
    if (millis() - startTime > TIMEOUT) break;

    float s1 = 1000, s3 = 1000, s4 = 1000, s6 = 1000;
    VL53L0X_RangingMeasurementData_t measure;

    if (sensorActivo[1]) { sensores[1].rangingTest(&measure, false); if (measure.RangeStatus != 4) s1 = measure.RangeMilliMeter; }
    if (sensorActivo[3]) { sensores[3].rangingTest(&measure, false); if (measure.RangeStatus != 4) s3 = measure.RangeMilliMeter; }
    if (sensorActivo[4]) { sensores[4].rangingTest(&measure, false); if (measure.RangeStatus != 4) s4 = measure.RangeMilliMeter; }
    if (sensorActivo[6]) { sensores[6].rangingTest(&measure, false); if (measure.RangeStatus != 4) s6 = measure.RangeMilliMeter; }

    float izquierda = (s1 + s3) / 2.0;
    float derecha   = (s4 + s6) / 2.0;
    bool  paredIzq  = (izquierda > MIN_PARED && izquierda < MAX_PARED);
    bool  paredDer  = (derecha   > MIN_PARED && derecha   < MAX_PARED);
    const float UMBRAL_CENTRADO = 5.0;

    if      (paredIzq && paredDer)  modoLateral = 1;
    else if (paredIzq && !paredDer) modoLateral = 2;
    else if (!paredIzq && paredDer) modoLateral = 3;

    float correccion = 0;
    if (modoLateral == 1) {
      float diferencia = derecha - izquierda;
      if (abs(diferencia) <= UMBRAL_CENTRADO) break;
      correccion = diferencia * KP_LAT;
    } else if (modoLateral == 2) {
      correccion = (118.0 - izquierda) * KP_LAT;
    } else if (modoLateral == 3) {
      correccion = (derecha - 128.0) * KP_LAT;
    }

    float actual       = readHeadingRaw();
    float errorAng     = angleDiff(anguloObjetivo, actual);
    float correccionAng = errorAng * 1.2;

    int velIzq = constrain(VEL_BASE + (int)correccion + (int)correccionAng, 30, 100);
    int velDer = constrain(VEL_BASE - (int)correccion - (int)correccionAng, 30, 100);

    forwardRaw(velIzq, velDer);
    delay(15);
  }

  stopMotors();
  turnToHeading(anguloObjetivo);

  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("Centrado OK");
  display.display();
  delay(200);
}

void calibrar_atras() {
  const float DIST_OBJ = 65.0, UMBRAL = 4.0;

  leerTOFS();

  bool paredIzq   = Dist[1] < 200 || Dist[3] < 250;
  bool paredDer   = Dist[4] < 200 || Dist[6] < 250;
  bool paredAtras = Dist[7] < 150 || Dist[8] < 150;
  bool tresParedes = paredIzq && paredDer && paredAtras;

  if (!paredAtras || tresParedes) return;

  float atras = (Dist[7] + Dist[8]) / 2.0;
  if (abs(atras - DIST_OBJ) <= UMBRAL) return;

  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("Alineando atras");
  display.display();

  const float KP_BACK = 0.6;
  const int   VEL_BASE = 50;
  unsigned long startTime = millis();
  const unsigned long TIMEOUT = 500;
  float anguloObjetivo = readHeadingAvg(5);

  while (true) {
    if (millis() - startTime > TIMEOUT) break;
    leerTOFS();
    float atrasAct = (Dist[7] + Dist[8]) / 2.0;
    float error    = DIST_OBJ - atrasAct;
    if (abs(error) <= UMBRAL) break;

    int vel = constrain(VEL_BASE + abs(error * KP_BACK), 30, 80);
    if (error > 0) backwardRaw(vel, vel);
    else           forwardRaw(vel, vel);
    delay(15);
  }

  stopMotors();
  turnToHeading(anguloObjetivo);

  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("Atras OK");
  display.display();
  delay(150);
}

void retroceder(float distanciaCM) {
  long ticksObjetivo = distanciaCM * TICKS_POR_CM;
  encLeft.write(0);
  encRight.write(0);

  while (true) {
    long leftTicks  = abs(encLeft.read());
    long rightTicks = abs(encRight.read());
    long promedio   = (leftTicks + rightTicks) / 2;
    if (promedio >= ticksObjetivo) break;
    backwardRaw(BASE_SPEED, BASE_SPEED);
    delayMicroseconds(200);
  }
  stopMotors();
  delay(100);
}

// =============================== COLOR

String detectarColor() {
  uint16_t r, g, b, c;
  if (!apds.colorDataReady()) return "NONE";
  apds.getColorData(&r, &g, &b, &c);

  int diffBR = abs(b - r), diffBG = abs(b - g), diffRG = abs(r - g);
  int totales = diffBR + diffBG + diffRG;

  if (c == 0) return "NONE";

  if (r < (colores[0] * 0.2) &&
      g < (colores[1] * 0.15) &&
      b < (colores[2] * 0.15) &&
      c < (colores[3] * 0.2)) {
    return "NEGRO";
  } else if (r < (colores[0] * 0.2) &&
             g < (colores[1] * 0.45) &&
             b < (colores[2] * 0.45) &&
             c > (colores[3] * 0.2)) {
    return "AZUL";
  } else if (r < (colores[0] * 0.9) &&
             g < (colores[1] * 0.3) &&
             b > (colores[2] * 0.08) &&
             c > (colores[3] * 0.2)) {
    return "ROJO";
  } else if (c >= 430 && c <= 500 && totales < 35) {
    return "PLATEADO";
  } else if (c > 500) {
    return "PLATEADO";
  } else {
    return "BLANCO";
  }
}

void reaccionColor(String color, float distanciaRecorrida) {
  if (color == "NEGRO") {
    stopMotors();
    Serial.println("NEGRO → retrocediendo");
    retroceder(distanciaRecorrida);
    delay(100);
    giro180();
    return;
  }

  if (color == "AZUL") {
    if (ignorarAzul) return;
    contadorAzul++;
    if (contadorAzul >= 7 && !pausaAzul) {
      stopMotors();
      Serial.println("AZUL");
      pausaAzul = true;
      tiempoAzul = millis();
      contadorAzul = 0;
      ignorarAzul = true;
      interrumpidoPorAzul = true;
    }
  } else {
    contadorAzul = 0;
    ignorarAzul  = false;
  }

  if (color == "ROJO") {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("ROJO");
    display.display();
  }

  if (color == "PLATEADO") {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("PLATEADO");
    display.display();
  }
}

// =============================== UART OpenMV

bool revisarUART(float anguloObjetivo) {
  static unsigned long ultimaEjecucion = 0;
  const unsigned long tiempoEspera = 10000;

  if (millis() - ultimaEjecucion < tiempoEspera) {
    while (openmv.available()) openmv.read();
    return false;
  }

  if (!openmv.available()) return false;

  String dato = openmv.readStringUntil('\n');
  dato.trim();

  int kits = -1;
  if      (dato == "L:omega") kits = 0;
  else if (dato == "L:phi")   kits = 2;
  else if (dato == "L:psi")   kits = 1;

  if (kits != -1) {
    stopMotors();
    navSetVictima();   // ← Informa al navegador que hay víctima aquí

    for (int i = 0; i < NUMPIXELS; i++) pixels.setPixelColor(i, pixels.Color(0, 0, 255));
    pixels.show();
    delay(1000);
    turnToHeading(anguloObjetivo);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.println(dato);
    display.display();

    for (int k = 0; k < kits; k++) {
      for (int i = 0; i < NUMPIXELS; i++) pixels.setPixelColor(i, pixels.Color(255, 255, 255));
      pixels.show();
      delay(400);
      for (int i = 0; i < NUMPIXELS; i++) pixels.setPixelColor(i, pixels.Color(0, 0, 255));
      pixels.show();
      delay(400);
    }

    for (int i = 0; i < NUMPIXELS; i++) pixels.setPixelColor(i, pixels.Color(255, 255, 255));
    pixels.show();

    ultimaEjecucion = millis();
    return true;
  }
  return false;
}

// =============================== INICIAR SENSORES

void iniciarTOFS() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
    sensorActivo[i] = false;
  }
  delay(50);
  for (int i = 0; i < NUM_SENSORES; i++) {
    digitalWrite(xshutPins[i], HIGH);
    delay(20);
    if (!sensores[i].begin(0x29)) {
      sensorActivo[i] = false;
    } else {
      sensores[i].setAddress(direcciones[i]);
      sensorActivo[i] = true;
    }
  }
}

// =============================== SETUP

void setup() {
  Serial.begin(9600);

  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  openmv.begin(230400);

  delayMicroseconds(50);
  Wire.begin();
  Wire.setClock(400000);

  setupMotor(L_F1, L_F2, L_F_PWM);
  setupMotor(L_B1, L_B2, L_B_PWM);
  setupMotor(R_F1, R_F2, R_F_PWM);
  setupMotor(R_B1, R_B2, R_B_PWM);

  pinMode(LLS, INPUT);
  pinMode(RLS, INPUT);

  iniciarTOFS();

  if (bno.begin()) {
    bnoActivo = true;
    bno.setExtCrystalUse(true);
    headingFiltrado = getHeading();
    Serial.println("BNO OK");
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.display();
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  delayMicroseconds(50);

  pixels.begin();
  pixels.setBrightness(100);
  for (int i = 0; i < 8; i++) pixels.setPixelColor(i, pixels.Color(255, 255, 230));
  pixels.show();

  if (!apds.begin()) {
    Serial.println("Error APDS");
    while (1);
  }
  apds.enableColor(true);

  // --- Calibración de color ---
  Serial.println("=== CALIBRANDO COLOR ===");
  Serial.println("Coloca el robot sobre el PISO (blanco)");
  delay(3000);

  for (int i = 0; i < n; i++) {
    uint16_t r, g, b, c;
    while (!apds.colorDataReady()) delay(5);
    apds.getColorData(&r, &g, &b, &c);
    colores[0] += r; colores[1] += g;
    colores[2] += b; colores[3] += c;
    delay(20);
  }
  colores[0] /= n; colores[1] /= n;
  colores[2] /= n; colores[3] /= n;
  Serial.println("Calibración lista");

  // ── INICIAR NAVEGADOR ───────────────────────────────────────
  // El robot parte desde la casilla (5,5) del mapa mirando al NORTE.
  // Ajusta los valores según donde coloques al robot en la grilla.
  navInit(5, 5, 0, NAV_NORTE);
}

// =============================== LOOP

void loop() {
  // Una sola llamada gestiona todo:
  //   • Lee paredes con leerTOFS()
  //   • Decide hacia dónde ir (DFS)
  //   • Gira y avanza con tus funciones
  //   • Cuando termina la exploración, regresa al origen por Dijkstra
  navLoop();

  // Debug opcional: imprime mapa cada 10 s
  static unsigned long ultimoMapa = 0;
  if (millis() - ultimoMapa > 10000) {
    navImprimirMapa(navGetZ());
    ultimoMapa = millis();
  }
}