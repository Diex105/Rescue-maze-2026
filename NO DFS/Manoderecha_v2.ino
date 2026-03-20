//Version de los 90° que funcionan y los 30cm con encoders, se le agrega el limit switch si avanza 30 cm proximo a una pared ... 
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Encoder.h>
#include <Adafruit_VL53L0X.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define DEBOUNCE_TICKS 5  // Ignorar cambios menores a 5 ticks
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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
// ===============================
// SENSORES TOF
// ===============================
#define NUM_SENSORES 11
// Pines XSHUT (puedes cambiarlos si quieres)
int xshutPins[NUM_SENSORES] = {
  1,3,0,5,6,2,8,9,10,7,4
};
// Direcciones nuevas para cada sensor
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

// Variables globales para control de 30cm
volatile long lastLeftTicks = 0;
volatile long lastRightTicks = 0;
unsigned long lastControlTime = 0;
float integralLeft = 0, integralRight = 0;
float lastErrorLeft = 0, lastErrorRight = 0;
// Variables globales para PID GIRO
float pidIntegral = 0;
float pidErrorAnterior = 0;
unsigned long pidTiempoAnterior = 0;
//////////// Limit switch
#define LLS 41
#define RLS 40

// =============================== MOVIMIENTOS DE MOTORES
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

  // Asegurar que las velocidades estén en rango válido
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Aplicar rampa de aceleración para movimientos suaves
  static int lastLeftSpeed = 0;
  static int lastRightSpeed = 0;
  
  int maxAccel = 30; // Máximo cambio de velocidad por ciclo
  
  leftSpeed = constrain(leftSpeed, lastLeftSpeed - maxAccel, lastLeftSpeed + maxAccel);
  rightSpeed = constrain(rightSpeed, lastRightSpeed - maxAccel, lastRightSpeed + maxAccel);
  
  lastLeftSpeed = leftSpeed;
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

  // Asegurar que las velocidades estén en rango válido
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  
  // Aplicar rampa de aceleración para movimientos suaves
  static int lastLeftSpeed = 0;
  static int lastRightSpeed = 0;
  
  int maxAccel = 30; // Máximo cambio de velocidad por ciclo
  
  leftSpeed = constrain(leftSpeed, lastLeftSpeed - maxAccel, lastLeftSpeed + maxAccel);
  rightSpeed = constrain(rightSpeed, lastRightSpeed - maxAccel, lastRightSpeed + maxAccel);
  
  lastLeftSpeed = leftSpeed;
  lastRightSpeed = rightSpeed;

  digitalWrite(L_F1, HIGH);  digitalWrite(L_F2, LOW);
  digitalWrite(L_B1, LOW); digitalWrite(L_B2, HIGH);

  digitalWrite(R_F1, LOW); digitalWrite(R_F2, HIGH);
  digitalWrite(R_B1, HIGH);  digitalWrite(R_B2, LOW);

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
  float sumSin = 0.0;
  float sumCos = 0.0;
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
  while (diff > 180.0) diff -= 360.0;
  while (diff < -180.0) diff += 360.0;
  return diff;
}
float normalize360(float angle) {
  while (angle >= 360.0) angle -= 360.0;
  while (angle < 0.0) angle += 360.0;
  return angle;
}
void turnToHeading(float objetivo) {
  if (!bnoActivo) return;

  const float KP = 2.2;
  const float KI = 0.2;
  const float KD = 0.1;  // Más freno
  const int VEL_MAX = 240;
  const int VEL_MIN = 100;
  const float ANGULO_TOL = 0.6;
  const int SETTLE_MS = 180;
  const float FASE_FINE = 3.0;

  float integral = 0.0;
  float lastError = 0.0;
  unsigned long lastUpdate = micros();
  unsigned long inTolStart = 0;
  bool lastTurnLeft = true;

  while (true) {
    float actual = readHeadingAvg(3);
    float error = angleDiff(objetivo, actual);

    unsigned long ahora = micros();
    float dt = (ahora - lastUpdate) / 1000000.0;
    if (dt < 0.008) {
      continue;
    }

    if (abs(error) < 12.0) {
      integral += error * dt;
      integral = constrain(integral, -25.0, 25.0);
    } else {
      integral = 0.0;
    }

    float dError = (error - lastError) / dt;
    lastError = error;

    float velocidad = KP * abs(error) + KI * abs(integral) + KD * abs(dError);

    if (abs(error) < 10.0) {
      float factor = 0.5 + (abs(error) * 0.035);
      velocidad *= factor;
    }

    velocidad = constrain(velocidad, (float)VEL_MIN, (float)VEL_MAX);

    if (abs(error) <= ANGULO_TOL) {
      if (inTolStart == 0) inTolStart = millis();
      if (millis() - inTolStart >= (unsigned long)SETTLE_MS) {
        break;
      }
      velocidad = max(35, (int)(velocidad * 0.6));
    } else {
      inTolStart = 0;
    }

    if (abs(error) > FASE_FINE) {
      if (error < 0) {
        turnLeftMotor((int)velocidad);
        lastTurnLeft = true;
      } else {
        turnRightMotor((int)velocidad);
        lastTurnLeft = false;
      }
    } else {
      int pulseVel = max(30, (int)(velocidad * 0.5));
      if (error < 0) {
        turnLeftMotor(pulseVel);
        lastTurnLeft = true;
      } else {
        turnRightMotor(pulseVel);
        lastTurnLeft = false;
      }
      delay(10);
      stopMotors();
      delay(8);
    }

    static unsigned long lastDisp = 0;
    /*if (millis() - lastDisp > 80) {
      lastDisp = millis();
      display.clearDisplay();
      display.setCursor(5, 5);
      display.print("A:");
      display.print(actual, 1);
      display.setCursor(5, 25);
      display.print("E:");
      display.print(error, 1);
      display.setCursor(5, 45);
      display.print("V:");
      display.print((int)velocidad);
      display.display();
    }*/

    lastUpdate = ahora;
  }

  for (int i = VEL_MIN; i > 0; i -= 15) {
    if (i < 20) break;
    if (lastTurnLeft) {
      turnLeftMotor(i);
    } else {
      turnRightMotor(i);
    }
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
// =============================== LECTURAS DE TOFS
void leerTOFS() {
  for (int i = 0; i < NUM_SENSORES; i++) {
    int distancia = 0;
    if (sensorActivo[i]) {
      VL53L0X_RangingMeasurementData_t measure;
      sensores[i].rangingTest(&measure, false);
      if (measure.RangeStatus != 4) {
        Dist[i] = measure.RangeMilliMeter;
      }
    }
    /*Serial.print("S");
    Serial.print(i);
    Serial.print(":");
    Serial.print(distancia);
    Serial.print("mm ");*/
  }
}
// =============================== SECUENCIAS DE MOVIMIENTOS
void avanzar_optimizado(int DISTANCIA_CM) {
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  

  const float KP = 0.6;      // Reducido para movimiento más suave
  const float KI = 0.02;     // Integral más baja
  const float KD = 0.05;     // Derivativo más bajo
  const int COMPENSACION_INERCIA = 50;

  long ticksObjetivo = (DISTANCIA_CM * TICKS_POR_CM) - COMPENSACION_INERCIA;
  
  // 1. Capturar el ángulo objetivo justo antes de empezar a movernos
  // Esto asegura que el robot mantenga la dirección que tiene al inicio
  float anguloObjetivo = readHeadingAvg(5); 

  encLeft.write(0);
  encRight.write(0);
  
  float lastGyroError = 0;
  unsigned long lastTime = micros();
  
  int currentLeftSpeed = BASE_SPEED;
  int currentRightSpeed = BASE_SPEED;
  const int ACELERACION_MAX = 20; 

  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("Recto IMU...");
  display.display();
  
  while (true) {
    long leftTicks = abs(encLeft.read());
    long rightTicks = abs(encRight.read());
    long promedio = (leftTicks + rightTicks) / 2;
    
    // Condición de salida por distancia
    if (promedio >= ticksObjetivo) break;
    
    unsigned long currentTime = micros();
    float deltaTime = (currentTime - lastTime) / 1000000.0;
    
    if (deltaTime >= 0.01) { // Control cada 10ms
      // 2. Calcular error de orientación con el IMU
      float anguloActual = readHeadingRaw(); // Lectura rápida
      float errorGyro = angleDiff(anguloObjetivo, anguloActual); // Diferencia angular real

      // 3. PID de Orientación
      float derivativoGyro = (errorGyro - lastGyroError) / deltaTime;
      float correccion = (KP * errorGyro) + (KD * derivativoGyro);
      
      lastGyroError = errorGyro;
      lastTime = currentTime;

      float correccionLat = controlLateralContinuo();
      float correccionTotal = correccion + correccionLat;
      int targetLeft = BASE_SPEED + (int)correccionTotal;
      int targetRight = BASE_SPEED - (int)correccionTotal;

      
      // Rampa de aceleración suave
      currentLeftSpeed = constrain(targetLeft, BASE_SPEED - 60, BASE_SPEED + 60);
      currentRightSpeed = constrain(targetRight, BASE_SPEED - 60, BASE_SPEED + 60);
    }
    
    // 5. Mover motores
    forwardRaw(currentLeftSpeed, currentRightSpeed);
    
    // Seguridad extra: Si detecta colisión frontal muy cerca, para
    // (Opcional: leerTOFS() es lento, mejor leer solo el sensor central si es crítico)
    
    delayMicroseconds(200);
  }
  
  stopMotors();
  
  // --- ACOMODO FINAL ---
  // Una vez que llega a la distancia, hacemos una corrección final de ángulo 
  // para asegurar que quedó perfectamente alineado a 0, 90, 180 o 270.
  delay(100);
  turnToHeading(anguloObjetivo); 
  
  // Lógica de los Limit Switch que ya tenías
  leerTOFS();
  if(Dist[0] <= 180.0 && Dist[5] <= 180.0){

    contadorFrentes++;
    if (contadorFrentes >= 5){
      calibrar_limit();
      contadorFrentes = 0;

    }
    
  
  }
  
  delay(200);
}
// ===============================
// LOGICA DFS
// ===============================
#define UMBRAL_PARED 160.0   // mm (ajústalo según tu laberinto)
bool libreFrente() {
  float d = Dist[0];
  if (d == 0 || d > 1200) d = 2000;  // Si no mide, asumimos libre
  return d > UMBRAL_PARED;
}

bool libreIzquierda() {
  float d = Dist[1];
  if (d == 0 || d > 1200) d = 2000;
  return d > UMBRAL_PARED;
}

bool libreDerecha() {
  float d = Dist[4];
  if (d == 0 || d > 1200) d = 2000;
  return d > UMBRAL_PARED;
}

void giro90Der() {
  if (!bnoActivo) return;
  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("Girando 90...");
  display.display();
  float inicio = readHeadingAvg(5);
  float objetivo = normalize360(inicio + 90.0);
  turnToHeading(objetivo);

  float final = readHeadingAvg(5);
  float girado = angleDiff(final, inicio);

  display.clearDisplay();
  display.setCursor(5, 5);
  display.print("Giro: ");
  display.print(abs(girado), 1);
  display.println(" deg");
  display.setCursor(5, 25);
  display.print("Error: ");
  display.print(girado + 90.0, 1);
  display.display();

  delay(200);
}
void giro180() {
  if (!bnoActivo) return;

  float inicio = readHeadingAvg(5);
  float objetivo = normalize360(inicio + 180.0);
  turnToHeading(objetivo);

  delay(300);
}
void decisionDFS() {

  leerTOFS();
  delay(50);

  bool frente = libreFrente();
  bool izquierda = libreIzquierda();
  bool derecha = libreDerecha();

  Serial.print("F: "); Serial.print(frente);
  Serial.print("  I: "); Serial.print(izquierda);
  Serial.print("  D: "); Serial.println(derecha);
  display.clearDisplay();
  display.setCursor(5, 5);
  display.print("F: ");
  display.print(frente);
  display.setCursor(5, 25);
  display.print("I: ");
  display.print(izquierda);
  display.setCursor(5, 45);
  display.print("D: ");
  display.print(derecha);
  display.display();
  delay(2000);
  // PRIORIDAD DFS → IZQUIERDA
  if (derecha) {
    Serial.println("DFS: Giro Derecha");
    giro90Der();
  }
  else if (frente) {
    Serial.println("DFS: Sigo Frente");
  }
  else if (izquierda) {
    Serial.println("DFS: Giro Derecha");
    giro90Izq();
  }
  else {
    Serial.println("DFS: Dead End → Giro 180");
    giro180();
  }
}
void giro90Izq() {
  if (!bnoActivo) return;
  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("Girando 90...");
  display.display();
  float inicio = readHeadingAvg(5);
  float objetivo = normalize360(inicio - 90.0);
  turnToHeading(objetivo);

  float final = readHeadingAvg(5);
  float girado = angleDiff(final, inicio);

  display.clearDisplay();
  display.setCursor(5, 5);
  display.print("Giro: ");
  display.print(abs(girado), 1);
  display.println(" deg");
  display.setCursor(5, 25);
  display.print("Error: ");
  display.print(girado + 90.0, 1);
  display.display();

  delay(200);
}
void calibrar_limit() {
  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("Calibrando...");
  display.display();

  // --- CONFIGURACIÓN DE CONTROL ---
  int cal_vel = 75;            // Velocidad constante para calibración
  const float KP_CAL = 2.0;    // Fuerza de corrección (ajusta si serpentea)
  float promedio = 0;          // Declarada aquí para evitar scope error
  
  // Capturamos el ángulo actual para mantenerlo durante toda la maniobra
  float anguloObjetivo = readHeadingAvg(10); 

  // --- FASE 1: AVANCE RECTO HACIA LA PARED ---
  while (true) {
    float actual = readHeadingRaw();
    float error = angleDiff(anguloObjetivo, actual);
    float correccion = error * KP_CAL;

    // Aplicamos corrección al avance
    forwardRaw(cal_vel - (int)correccion, cal_vel + (int)correccion);

    // Salir cuando ambos limit switches detecten la pared
    if (digitalRead(LLS) == 1 || digitalRead(RLS) == 1) {
      break;
    }
    delay(5); 
  }

  stopMotors();

  // --- FASE 2: RETROCESO RECTO HASTA 120mm ---
  while (true) {
    float actual = readHeadingRaw();
    float error = angleDiff(anguloObjetivo, actual);
    float correccion = error * KP_CAL;

    // Al retroceder, la lógica de corrección se invierte para mantener el rumbo
    backwardRaw(cal_vel + (int)correccion, cal_vel - (int)correccion);

    // Lectura de los sensores ToF frontales (0 y 5)
    float d0 = 1000, d5 = 1000;
    VL53L0X_RangingMeasurementData_t measure;

    if (sensorActivo[0]) {
      sensores[0].rangingTest(&measure, false);
      if (measure.RangeStatus != 4) d0 = measure.RangeMilliMeter;
    }
    if (sensorActivo[5]) {
      sensores[5].rangingTest(&measure, false);
      if (measure.RangeStatus != 4) d5 = measure.RangeMilliMeter;
    }

    promedio = (d0 + d5) / 2.0;

    // Detenerse cuando alcancemos la distancia deseada
    if (promedio >= 120.0) {
      break;
    }
  }

  stopMotors();
  
  // Ajuste final: asegurar que quedamos exactamente en el ángulo objetivo
  turnToHeading(anguloObjetivo);


  // Mostrar resultado final en OLED
  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("OK! Dist:");
  display.setCursor(5, 25);
  display.print(promedio, 1);
  display.print(" mm");
  display.display();
  
  delay(500);
}

float controlLateralContinuo() {

  const float KP_LAT = 0.6;
  const float MIN_PARED = 40.0;
  const float MAX_PARED = 200.0;
  const float UMBRAL = 5.0;

  float s1 = 1000;
  float s4 = 1000;

  VL53L0X_RangingMeasurementData_t measure;

  // 🔵 Sensor izquierdo (S1)
  if (sensorActivo[1]) {
    sensores[1].rangingTest(&measure, false);
    if (measure.RangeStatus != 4) 
      s1 = measure.RangeMilliMeter;
  }

  // 🔴 Sensor derecho (S4)
  if (sensorActivo[4]) {
    sensores[4].rangingTest(&measure, false);
    if (measure.RangeStatus != 4) 
      s4 = measure.RangeMilliMeter;
  }

  bool paredIzq = (s1 > MIN_PARED && s1 < MAX_PARED);
  bool paredDer = (s4 > MIN_PARED && s4 < MAX_PARED);

  float correccion = 0;

  // 🔥 Si hay pared en ambos lados → centrar
  if (paredIzq && paredDer) {
    float diferencia = s4 - s1;

    if (abs(diferencia) > UMBRAL)
      correccion = diferencia * KP_LAT;
  }

  else if (paredIzq) {
    const float DISTANCIA_OBJ = 120.0;  // Ajustable
    correccion = (DISTANCIA_OBJ - s1) * KP_LAT;
  }

 
  else if (paredDer) {
    const float DISTANCIA_OBJ = 120.0;  // Ajustable
    correccion = (s4 - DISTANCIA_OBJ) * KP_LAT;
  }

  return correccion;
}


void calibrar_lateral() {

  display.clearDisplay();
  display.setCursor(5,5);
  display.println("Alineando...");
  display.display();

  const float KP_LAT = 0.6;
  const int VEL_BASE = 55;

  const float MIN_PARED = 40.0;   // mm
  const float MAX_PARED = 400.0;  // mm
  


  unsigned long startTime = millis();
  const unsigned long TIMEOUT = 600; // ms seguridad

  float anguloObjetivo = readHeadingAvg(5);
  

  while (true) {

    modoLateral =0;

    if (millis() - startTime > TIMEOUT) break;  // seguridad anti-loop

    float s1=1000, s3=1000, s4=1000, s6=1000;
    VL53L0X_RangingMeasurementData_t measure;

    if (sensorActivo[1]) {
      sensores[1].rangingTest(&measure,false);
      if (measure.RangeStatus != 4) s1 = measure.RangeMilliMeter;
    }

    if (sensorActivo[3]) {
      sensores[3].rangingTest(&measure,false);
      if (measure.RangeStatus != 4) s3 = measure.RangeMilliMeter;
    }

    if (sensorActivo[4]) {
      sensores[4].rangingTest(&measure,false);
      if (measure.RangeStatus != 4) s4 = measure.RangeMilliMeter;
    }

    if (sensorActivo[6]) {
      sensores[6].rangingTest(&measure,false);
      if (measure.RangeStatus != 4) s6 = measure.RangeMilliMeter;
    }

    float izquierda = (s1 + s3) / 2.0;
    float derecha   = (s4 + s6) / 2.0;

    bool paredIzq = (izquierda > MIN_PARED && izquierda < MAX_PARED);
    bool paredDer = (derecha   > MIN_PARED && derecha   < MAX_PARED);
    
    const float UMBRAL_CENTRADO = 5.0;

    if (paredIzq && paredDer) {
      modoLateral= 1;          
    }
    else if (paredIzq && !paredDer){
      modoLateral = 2;
    }

    else if (!paredIzq && paredDer){
      modoLateral = 3;
    }

    float correccion = 0;

    if (modoLateral == 1){

      float diferencia = derecha - izquierda;
      
      if (abs(diferencia) <= UMBRAL_CENTRADO){
      break;
      }
      correccion = diferencia * KP_LAT;
    }

    else if (modoLateral == 2){
      const float DIST_OBJ_IZQ = 118.0;
      float error = DIST_OBJ_IZQ - izquierda;
      correccion = error * KP_LAT;
    }

    else if (modoLateral == 3){
      const float DIST_OBJ_DER = 128.0;
      float error = derecha - DIST_OBJ_DER;
      correccion = error * KP_LAT;
    }

    else {
      correccion = 0;
    }

    float actual = readHeadingRaw();



    float errorAng = angleDiff(anguloObjetivo, actual);
    float correccionAng = errorAng * 1.2;

    int velIzq = VEL_BASE + (int)correccion + (int)correccionAng;
    int velDer = VEL_BASE - (int)correccion - (int)correccionAng;

    velIzq = constrain(velIzq, 30, 100);
    velDer = constrain(velDer, 30, 100);

    forwardRaw(velIzq, velDer);

    delay(15);
  }

  stopMotors();
  turnToHeading(anguloObjetivo);

  display.clearDisplay();
  display.setCursor(5,5);
  display.println("Centrado OK");
  display.display();

  delay(200);
}


// =============================== INICIAR SENSORES
void iniciarTOFS() {
  // -------- APAGAR TODOS LOS TOF --------
  for (int i = 0; i < NUM_SENSORES; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
    sensorActivo[i] = false;
  }
  delay(50);
  // -------- INICIALIZAR UNO POR UNO --------
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
// ===============================





//_----------------------------------------------
void setup() {
  Serial.begin(115200);
  delayMicroseconds(50);
  Wire.begin();
  Wire.setClock(400000);
  setupMotor(L_F1, L_F2, L_F_PWM);
  setupMotor(L_B1, L_B2, L_B_PWM);
  setupMotor(R_F1, R_F2, R_F_PWM);
  setupMotor(R_B1, R_B2, R_B_PWM);
  pinMode(LLS, INPUT); //INicia los pines de los limit switch
  pinMode(RLS, INPUT);
  iniciarTOFS(); //Inicia todos los tofs
  if (bno.begin()) { //Inicia el bno
    bnoActivo = true;
    bno.setExtCrystalUse(true);
    headingFiltrado = getHeading();
    Serial.println("BNO OK");
  }
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { //Inicia la oled
    Serial.println(F("SSD1306 allocation failed"));
    //for(;;); //Esta linea es para que se cicle si no se inicia la oled
  }
  display.display(); //Hace display de la imagen de adafruit
  display.clearDisplay(); // Limpia el display
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  delayMicroseconds(50);

 
}


// ===============================
void loop() {
  /*leerTOFS();
  Serial.print("IF: ");
  Serial.println(Dist[1]);
  Serial.print("IA ");
  Serial.println(Dist[3]);
  Serial.println("//////////////////////////////");
  Serial.println("==== DFS STEP ====");*/
  decisionDFS();          // Decide hacia dónde ir
  delay(50);
  avanzar_optimizado(30);  // Avanza una celda
  delay(200);
}
