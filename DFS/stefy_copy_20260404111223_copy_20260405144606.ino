#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Encoder.h>
#include <Adafruit_VL53L0X.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_APDS9960.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <vector>
#include <queue>
#include <algorithm>

using namespace std;

vector<int> distancias;
vector<int> padre;

enum Direccion {
  NORTE = 0,
  ESTE  = 1,
  SUR   = 2,
  OESTE = 3
};

enum ResultadoMovimiento {
  MOV_OK,
  MOV_NEGRO,
  MOV_AZUL,
  MOV_ABORTADO,
  MOV_ROJO
};

struct Conexion {
  int destino;           // id del nodo vecino
  bool existe;           // hay camino en esa dirección
  bool explorada;        // ya recorrí esa salida
  bool esRojo;
};

struct Nodo {
  int id;
  int x;
  int y;
  bool visitado;
  bool tieneAzul;
  bool tienePlateado;

  Conexion vecinos[4];   // NORTE, ESTE, SUR, OESTE
};

vector<Nodo> mapa;
int nodoActual = -1;
int nodoInicio = -1;
Direccion dirActual = NORTE;
bool listoParaDecidir = true;

bool ultimoTileAzul = false;
bool ultimoTilePlateado = false;
bool ignorarPlateado = false;
bool interrumpidoPorAzul = false;

// Ultimo plateado detectado


// Reposición / restore

bool modoReposicion = false;
bool restauracionHecha = false;


//calibracion
bool modoCalibracion = true;
int pasoCalibracion = 0;
bool switchPrevio = false;
unsigned long ultimoFlanco = 0;

int moverANodoEnDireccion(Direccion d);
void debugEstado(const char* evento);
const char* nombreDireccion(Direccion d);


#define DEBOUNCE_TICKS 5  // Ignorar cambios menores a 5 ticks
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// ===============================
//NEOPIXEL
#define NEO_PIN 33
#define NUMPIXELS 8
Adafruit_NeoPixel pixels(NUMPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

#define PIN_REPOSICION 22

// servos
Servo SR;
Servo SL;

int contadorKitsIzq   = 0;
int contadorKitsDer   = 0;
int contadorKitsTotal = 0;
const int MAX_KITS_TOTAL     = 8;
const int MAX_KITS_POR_SERVO = 4;


const int SERVO_IZQ_CERRADO = 180;
const int SERVO_IZQ_EMPUJE  = 30;

const int SERVO_DER_CERRADO = 0;
const int SERVO_DER_EMPUJE  = 100;
//comunicación

volatile bool leerLetra = false;
volatile char letraPendiente = '\0';


const int pinVoltaje = A2;   // pin 16
const int pinPulso   = 17;   // pulso desde OpenMV
/////////

int n = 20;
volatile int colorBlanco[5] = {0, 0, 0, 0, 0};
const int NUM_PLATEADOS = 4;
volatile int colorPlateado[NUM_PLATEADOS][5];
const int MARGEN_PLATEADO = 60;
Adafruit_APDS9960 apds;
int contadorAzul = 0;
bool pausaAzul = false;
unsigned long tiempoAzul = 0;
bool ignorarAzul = false;
int contadorRojo = 0;
bool ultimoTileRojo = false;
bool bloquearOpenMV = false;
bool enLackOfProgress = false;

const unsigned long TIEMPO_SUBIDA = 3000;
const unsigned long TIEMPO_BAJADA = 1500;
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
float pitchFiltrado = 0.0;
bool pitchInit = false;
bool enRampa = false;
bool saliendoDeRampa = false;
const int BNO_RST_PIN = 32;
bool resetPendientePostRampa = false;
unsigned long tPlanoEstable = 0;
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
bool iniciarBNO(adafruit_bno055_opmode_t modo = OPERATION_MODE_NDOF) {
  if (!bno.begin(modo)) {
    bnoActivo = false;
    Serial.println("Error al iniciar BNO");
    return false;
  }

  bnoActivo = true;


  bno.setExtCrystalUse(true);

  delay(650);  // tiempo de estabilización

  pitchInit = false;
  float pTmp;
  leerPitchFiltrado(pTmp);

  headingFiltrado = readHeadingRaw();

  Serial.print("BNO iniciado en modo: 0x");
  Serial.println((int)modo, HEX);

  return true;
}

void resetBNO_Hardware(adafruit_bno055_opmode_t modo = OPERATION_MODE_NDOF) {
  Serial.println("Reset hardware BNO055...");

  bnoActivo = false;

  // nRESET es activo en LOW
  digitalWrite(BNO_RST_PIN, LOW);
  delay(15);
  digitalWrite(BNO_RST_PIN, HIGH);

  // Bosch indica ~650 ms desde reset a config mode
  delay(700);

  if (iniciarBNO(modo)) {
    Serial.println("BNO reiniciado correctamente");
  } else {
    Serial.println("Fallo al reiniciar BNO");
  }
}
void resincronizarHeading() {
  stopMotors();
  delay(120);
  esperarEstable();
  
  headingObjetivo = readHeadingAvg(10);

  Serial.print("Heading resincronizado: ");
  Serial.println(headingObjetivo);
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

  const float KP = 2.1;
  const float KI = 0.05;
  const float KD = 0.12;

  const int VEL_MAX = 220;
  const int VEL_MIN = 140;        // antes 150, demasiado alta
  const int VEL_FINE = 105;       // velocidad fija para corrección fina

  const float ANGULO_TOL = 1.5;  // antes 0.6, muy estricto para robot real
  const int SETTLE_MS = 120;     // antes 180
  const float FASE_FINE = 6.0;   // antes 3.0, entrar antes a modo fino
  const float FASE_BRAKE = 18.0; // zona de frenado progresivo

  float integral = 0.0;
  float lastError = 0.0;
  unsigned long lastUpdate = micros();
  unsigned long inTolStart = 0;

  bool lastTurnLeft = true;
  int ciclosSeguidosIgual = 0;
  float ultimoErrorSigno = 0.0;

  while (true) {

    if (atenderDeteccionOpenMV()) {
      lastUpdate = micros();
      inTolStart = 0;
      continue;
    }

    float actual = readHeadingRaw();   // lectura más rápida
    float error = angleDiff(objetivo, actual);

    unsigned long ahora = micros();
    float dt = (ahora - lastUpdate) / 1000000.0f;

    if (dt < 0.01f) continue;   // 10 ms
    if (dt > 0.08f) dt = 0.08f; // evitar picos raros en derivativo

    if (abs(error) < 15.0f) {
      integral += error * dt;
      integral = constrain(integral, -15.0f, 15.0f);
    } else {
      integral = 0.0f;
    }

    float dError = (error - lastError) / dt;
    dError = constrain(dError, -120.0f, 120.0f);

    float velocidad = KP * abs(error) + KI * abs(integral) + KD * abs(dError);

    // Freno progresivo al acercarse
    if (abs(error) < FASE_BRAKE) {
      float factor = abs(error) / FASE_BRAKE;   // 0..1
      factor = constrain(factor, 0.35f, 1.0f);
      velocidad *= factor;
    }

    velocidad = constrain(velocidad, (float)VEL_MIN, (float)VEL_MAX);

    // Verificación de tolerancia
    if (abs(error) <= ANGULO_TOL) {
      if (inTolStart == 0) inTolStart = millis();

      stopMotors();

      if (millis() - inTolStart >= (unsigned long)SETTLE_MS) {
        break;
      }

      lastError = error;
      lastUpdate = ahora;
      delay(8);
      continue;
    } else {
      inTolStart = 0;
    }

    // Detectar si ya empezó a oscilar cerca del objetivo
    if ((error > 0 && ultimoErrorSigno < 0) || (error < 0 && ultimoErrorSigno > 0)) {
      ciclosSeguidosIgual++;
    }
    ultimoErrorSigno = error;

    // Giro principal / giro fino
    if (abs(error) > FASE_FINE) {
      if (error < 0) {
        turnLeftMotor((int)velocidad);
        lastTurnLeft = true;
      } else {
        turnRightMotor((int)velocidad);
        lastTurnLeft = false;
      }
    } else {
      // Corrección fina por pulsos cortos
      if (error < 0) {
        turnLeftMotor(VEL_FINE);
        lastTurnLeft = true;
      } else {
        turnRightMotor(VEL_FINE);
        lastTurnLeft = false;
      }

      delay(6);
      stopMotors();
      delay(12);
    }

    lastError = error;
    lastUpdate = ahora;
  }

  stopMotors();
  delay(60);

  // Verificación final
  float final = readHeadingAvg(10);
  float errorFinal = angleDiff(objetivo, final);

  Serial.print("Objetivo: ");
  Serial.print(objetivo);
  Serial.print(" | Final: ");
  Serial.print(final);
  Serial.print(" | Error final: ");
  Serial.println(errorFinal);
}

bool modoPostReinicio = false;

float clampFloat(float v, float minV, float maxV) {
  if (v < minV) return minV;
  if (v > maxV) return maxV;
  return v;
}

void leerPitchFiltrado(float &pitch) {
  sensors_event_t event;
  bno.getEvent(&event);

  float rawPitch = event.orientation.z;

  const float alpha = 0.35;  // respuesta rápida pero algo filtrada

  if (!pitchInit) {
    pitchFiltrado = rawPitch;
    pitchInit = true;
  } else {
    pitchFiltrado = alpha * rawPitch + (1.0 - alpha) * pitchFiltrado;
  }

  pitch = pitchFiltrado;
}

// =============================== LECTURAS DE TOFS
void leerTOFS() {
  VL53L0X_RangingMeasurementData_t measure;

  for (int i = 0; i < NUM_SENSORES; i++) {
    
    sensores[i].rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {
      Dist[i] = measure.RangeMilliMeter;
    }

    if(i == 6){
      Dist[6] = Dist[6] - 15;
      delayMicroseconds(10);
    }
    if(i == 5){
      Dist[5]  = Dist[5]  - 28;
      delayMicroseconds(10);
    }
    if(i == 2){
      Dist[2]  = Dist[2]  + 8;
      delayMicroseconds(10);
    }

    delayMicroseconds(10);

  }
}
// =============================== SECUENCIAS DE MOVIMIENTOS
ResultadoMovimiento avanzar_optimizado(int DISTANCIA_CM) {
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  ultimoTileAzul = false;
  ultimoTilePlateado = false;
  ignorarPlateado = false;

  
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

    if (digitalRead(PIN_REPOSICION) == LOW) {
      stopMotors();
      return MOV_ABORTADO;
    }
    if (atenderDeteccionOpenMV()) {
      continue;
    }
    

    long leftTicks = abs(encLeft.read());
    long rightTicks = abs(encRight.read());
    long promedio = (leftTicks + rightTicks) / 2;
    
    int pwmEstimado = BASE_SPEED;
    #ifdef USE_CURRENT_SPEED_VARS
     pwmEstimado = max(abs(currentLeftSpeed), abs(currentRightSpeed));
    #endif

    float distanciaRecorrida = promedio / TICKS_POR_CM; 
    

    String color = "NONE";
    if (!enRampa) {
      color = detectarColor();
    }

    if (color != "NONE"){

      if (color == "NEGRO"){
        reaccionColor(color, distanciaRecorrida);
        return MOV_NEGRO;
      }
      if (color == "ROJO") {
        bool permitirRojoAhora = !hayPendientesNormalesEnMapa() || modoPostReinicio;

        if (!permitirRojoAhora) {
          stopMotors();
          Serial.println("ROJO -> aun faltan nodos normales, retrocediendo");
          retroceder(distanciaRecorrida);
          delay(80);
          return MOV_ROJO;
        } else {
          Serial.println("ROJO -> ya no quedan nodos normales, se permite avanzar");
        }
      }
      reaccionColor(color, distanciaRecorrida);
    }

    // Condición de salida por distanci
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
      if (enRampa) {
        correccionLat *= 0.3;
      }
      float correccionTotal = correccion + correccionLat;
      int targetLeft = BASE_SPEED + (int)correccionTotal;
      int targetRight = BASE_SPEED - (int)correccionTotal;

      
      // Rampa de aceleración suave
      currentLeftSpeed = constrain(targetLeft, BASE_SPEED - 60, BASE_SPEED + 60);
      currentRightSpeed = constrain(targetRight, BASE_SPEED - 60, BASE_SPEED + 60);
    }
    
    // 5. Mover motores
    forwardRaw(currentLeftSpeed, currentRightSpeed);
    
    bool limitIzq = digitalRead(LLS);
    bool limitDer = digitalRead(RLS);
    if (limitIzq || limitDer) {
      stopMotors();
      delay(50);
      // calcular distancia recorrida hasta el choque
      long leftTicks = abs(encLeft.read());
      long rightTicks = abs(encRight.read());
      long promedio = (leftTicks + rightTicks) / 2;
      float distanciaRecorrida = promedio / TICKS_POR_CM;
      // ángulo objetivo inicial
      float anguloObjetivo = readHeadingAvg(5);
      // Lectura ToFs laterales y central
      float centro = Dist[0];   // frontal central
      float izq = Dist[2];
      float der = Dist[5];
      if (limitDer) {
        bool brusco = der <= (centro*0.7) && der <= (izq*0.7); 
        Serial.println(brusco ? "Choque DER + ToF → acomodo brusco" : "Choque DER → acomodo leve");
        acomodarObstaculoPID("DER", distanciaRecorrida, anguloObjetivo, brusco);
        return MOV_OK;
      }
      if (limitIzq) {
        bool brusco = izq <= (centro*0.7) && izq <= (der*0.7); 
        Serial.println(brusco ? "Choque IZQ + ToF → acomodo brusco" : "Choque IZQ → acomodo leve");
        acomodarObstaculoPID("IZQ", distanciaRecorrida, anguloObjetivo, brusco);
        return MOV_OK;
      }
    }


    // --- DETECCIÓN DE RAMPA/ESCALERA ---
    float pitch;
    leerPitchFiltrado(pitch);
    static unsigned long lastPitchPrint = 0;
    if (millis() - lastPitchPrint > 80) {
      lastPitchPrint = millis();
      Serial.print("Pitch filtrado: ");
      Serial.println(pitch);
    }

    if (pitch <= -14.0) { //-25 -14
      Serial.println("Subida Rampa");
      avanzarInclinacion(anguloObjetivo);
      return MOV_OK; // salir de avanzar_optimizado
    }else if (pitch >= 12.0) { //20 12
      Serial.println("Bajada Rampa");
      delay(100);
      avanzarInclinacion(anguloObjetivo);
      return MOV_OK; // salir de avanzar_optimizado
    }
    delayMicroseconds(50);
  }

  //Fuera del while
  stopMotors();
  delay(30);
  // --- ACOMODO FINAL ---
  // Una vez que llega a la distancia, hacemos una corrección final de ángulo 
  // para asegurar que quedó perfectamente alineado a 0, 90, 180 o 270.
  turnToHeading(anguloObjetivo); 
  delay(30);
  headingObjetivo = readHeadingAvg(5);


  int plateadoCount = 0;

  for (int i = 0; i < 5; i++) {
    if (detectarColor() == "PLATEADO") {
      plateadoCount++;
    }
    delay(15);
  }
  if (plateadoCount >= 3) {

    ultimoTilePlateado = true;

    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.println("PLATEADO");
    display.display();

    ignorarPlateado = true;
  }
  int azulCount = 0;
  for (int i = 0; i < 5; i++) {

    if (detectarColor() == "AZUL") {
      azulCount++;
    }

    delay(15);
  }
  if (azulCount >= 3) {
    ultimoTileAzul = true;
    Serial.println("AZUL → esperar 5s");

    for (int i = 5; i > 0; i--) {

      display.clearDisplay();
      display.setCursor(0,0);
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);

      display.println("AZUL");
      display.print("Tiempo: ");
      display.println(i);

      display.display();

      delay(1000);
    }
  }
  // Lógica de los Limit Switch que ya tenías
  leerTOFS();
  if(Dist[0] <= 180.0 && Dist[5] <= 180.0){
    contadorFrentes++;
    if (contadorFrentes >= 3){
      calibrar_limit();
      contadorFrentes = 0;
    }
  }
  delay(200);
  if (ultimoTileAzul) return MOV_AZUL;
  return MOV_OK;
}




/*void avanzarInclinacion(float anguloObjetivo) {
  // Avanzar recto ignorando encoders hasta que vuelva a plano
  digitalWrite(L_F1, LOW);  digitalWrite(L_F2, HIGH);
  digitalWrite(L_B1, HIGH); digitalWrite(L_B2, LOW);

  digitalWrite(R_F1, HIGH); digitalWrite(R_F2, LOW);
  digitalWrite(R_B1, LOW);  digitalWrite(R_B2, HIGH);
  

  while (true) {
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    float pitch = orientationData.orientation.z;
    Serial.println(pitch);
    int vel_subida = 130;
    if (pitch <= -35){
      analogWrite(L_F_PWM, vel_subida-15);
      analogWrite(L_B_PWM, vel_subida+15);
      analogWrite(R_F_PWM, vel_subida-15);
      analogWrite(R_B_PWM, vel_subida+15);
    }else if (pitch < 0) {
      analogWrite(L_F_PWM, vel_subida);
      analogWrite(L_B_PWM, vel_subida);
      analogWrite(R_F_PWM, vel_subida);
      analogWrite(R_B_PWM, vel_subida);
    }
    if (pitch > 20){
      analogWrite(L_F_PWM, vel_subida-45); //frente llantas
      analogWrite(L_B_PWM, vel_subida-70); //atras llantas
      analogWrite(R_F_PWM, vel_subida-45);
      analogWrite(R_B_PWM, vel_subida-70);
    }else if (pitch > 0) {
      analogWrite(L_F_PWM, vel_subida-45);
      analogWrite(L_B_PWM, vel_subida-45);
      analogWrite(R_F_PWM, vel_subida-45);
      analogWrite(R_B_PWM, vel_subida-45);
    }
    // condición de salida: plano de nuevo
    if (abs(pitch) < 8.0) {
      stopMotors();
      break;
    }
    delay(20);
  }
  // Realinearse con IMU
  turnToHeading(anguloObjetivo);
  // Avanzar 10 cm extra ya en terreno plano
  avanzar_optimizado(10);
}*/

void avanzarRectoConHeading(float distanciaCM, float anguloObjetivo, int velocidadBase = BASE_SPEED) {
  long ticksObjetivo = distanciaCM * TICKS_POR_CM;
  encLeft.write(0);
  encRight.write(0);

  float lastError = 0;
  unsigned long lastTime = micros();

  while (true) {
    long leftTicks = abs(encLeft.read());
    long rightTicks = abs(encRight.read());
    long promedio = (leftTicks + rightTicks) / 2;
    if (promedio >= ticksObjetivo) break;

    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    if (dt < 0.01) continue;

    float actual = readHeadingRaw();
    float error = angleDiff(anguloObjetivo, actual);
    float dError = (error - lastError) / dt;
    float correccion = 0.7 * error + 0.03 * dError;

    int leftSpeed = constrain(velocidadBase + correccion, 120, 220);
    int rightSpeed = constrain(velocidadBase - correccion, 120, 220);

    forwardRaw(leftSpeed, rightSpeed);

    lastError = error;
    lastTime = now;
    delayMicroseconds(100);
  }

  stopMotors();
  delay(80);
}

void avanzarInclinacion(float anguloObjetivo) {

  enRampa = true;

  // Direccion motores (subida)
  digitalWrite(L_F1, LOW);
  digitalWrite(L_F2, HIGH);
  digitalWrite(L_B1, HIGH);
  digitalWrite(L_B2, LOW);
  digitalWrite(R_F1, HIGH);
  digitalWrite(R_F2, LOW);
  digitalWrite(R_B1, LOW);
  digitalWrite(R_B2, HIGH);

  const float KP = 2.2;
  const float KD = 0.08;
  const int MAX_CORRECCION = 25;

  const int TRIM_IZQ = -35;
  const int TRIM_DER = 0;

  float lastErrorAng = 0;
  unsigned long lastTime = micros();

  int contadorPlano = 0;
  unsigned long ultimoConteoNodoRampa = millis();
  int modoRampa = 0;

  while (true) {

    // Leer IMU
    sensors_event_t event;
    bno.getEvent(&event);
    float pitch = event.orientation.z;

    // Detectar si sube o baja
    if (pitch <= -14.0) {
      modoRampa = 0; // SUBIDA
    } 
    else if (pitch >= 12.0) {
      modoRampa = 1; // BAJADA
    }

    unsigned long tiempoNodoActual = (modoRampa == 0) ? TIEMPO_SUBIDA : TIEMPO_BAJADA;

    // =========================
    // PID DE ANGULO
    // =========================
    float actual = readHeadingRaw();
    float errorAng = angleDiff(anguloObjetivo, actual);

    unsigned long currentTime = micros();
    float deltaTime = (currentTime - lastTime) / 1000000.0;

    float derivativoAng = 0;
    if (deltaTime >= 0.001) {
      derivativoAng = (errorAng - lastErrorAng) / deltaTime;
      derivativoAng = constrain(derivativoAng, -100.0, 100.0);
    }

    float correccionAng = (KP * errorAng) + (KD * derivativoAng);
    correccionAng = constrain(correccionAng, -MAX_CORRECCION, MAX_CORRECCION);

    float correccionLat = controlLateralContinuo();
    if (enRampa) {
      correccionLat *= 0.25;
    }

    float correccionTotal = correccionAng + correccionLat;
    correccionTotal = constrain(correccionTotal, -5, 5);

    lastErrorAng = errorAng;
    lastTime = currentTime;

    // =========================
    // VELOCIDADES BASE
    // =========================
    int vel_subida = 185;

    int pwmLF = vel_subida;
    int pwmLB = vel_subida;
    int pwmRF = vel_subida;
    int pwmRB = vel_subida;

    // =========================
    // SUBIDA
    // =========================
    if (pitch <= -35) {
      pwmLF = vel_subida - 30;
      pwmLB = vel_subida + 25;
      pwmRF = vel_subida - 30;
      pwmRB = vel_subida + 25;
    } 
    else if (pitch <= -20) {
      pwmLF = vel_subida - 15;
      pwmLB = vel_subida + 10;
      pwmRF = vel_subida - 15;
      pwmRB = vel_subida + 10;
    } 
    else if (pitch < 0) {
      pwmLF = vel_subida;
      pwmLB = vel_subida;
      pwmRF = vel_subida;
      pwmRB = vel_subida;
    }

    // =========================
    // BAJADA
    // =========================
    if (pitch > 20) {
      pwmLF = vel_subida - 45;
      pwmLB = vel_subida - 70;
      pwmRF = vel_subida - 45;
      pwmRB = vel_subida - 70;
    } 
    else if (pitch > 0) {
      pwmLF = vel_subida - 30;
      pwmLB = vel_subida - 30;
      pwmRF = vel_subida - 30;
      pwmRB = vel_subida - 30;
    }

    // =========================
    // APLICAR PID
    // =========================
    pwmLF = pwmLF + (int)correccionTotal;
    pwmLB = pwmLB + (int)correccionTotal;
    pwmRF = pwmRF - (int)correccionTotal;
    pwmRB = pwmRB - (int)correccionTotal;

    // TRIM
    pwmLF += TRIM_IZQ;
    pwmLB += TRIM_IZQ;
    pwmRF += TRIM_DER;
    pwmRB += TRIM_DER;

    // LIMITAR PWM
    pwmLF = constrain(pwmLF, 70, 190);
    pwmLB = constrain(pwmLB, 70, 190);
    pwmRF = constrain(pwmRF, 70, 190);
    pwmRB = constrain(pwmRB, 70, 190);

    // ESCRIBIR PWM
    analogWrite(L_F_PWM, pwmLF);
    analogWrite(L_B_PWM, pwmLB);
    analogWrite(R_F_PWM, pwmRF);
    analogWrite(R_B_PWM, pwmRB);

    // =========================
    // CONTEO DE NODOS EN RAMPA
    // =========================
    while (millis() - ultimoConteoNodoRampa >= tiempoNodoActual) {
      int idRampa = moverANodoEnDireccion(dirActual);
      debugEstado("NODO_RAMPA");
      ultimoConteoNodoRampa += tiempoNodoActual;
    }

    // =========================
    // DETECTAR SALIDA DE RAMPA
    // =========================
    if (abs(pitch) < 8.0) {
      contadorPlano++;
    } else {
      contadorPlano = 0;
    }

    if (contadorPlano >= 4) {
      stopMotors();
      break;
    }

    delay(8);
  }

  enRampa = false;
  stopMotors();
  delay(180);

  // Realinearse con IMU
  turnToHeading(anguloObjetivo);
  delay(60);

  avanzarRectoConHeading(5, anguloObjetivo, BASE_SPEED - 25);

  saliendoDeRampa = true;
}

// ===============================
// LOGICA DFS
// ===============================
#define UMBRAL_PARED 160.0   // mm (ajústalo según tu laberinto)

bool libreFrente() {
  //return (Dist[0] == 0 || Dist[0] > UMBRAL_PARED || Dist[0] <= 85);
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

  float inicio = readHeadingAvg(5);
  float objetivo = normalize360(inicio + 90.0);
  turnToHeading(objetivo);

  turnToHeading(objetivo);
  delay(80);
  esperarEstable();

  float actual = readHeadingRaw();
  float error = angleDiff(objetivo, actual);

  if (abs(error) > 1.5) {
    turnToHeading(objetivo);
    delay(60);
    esperarEstable();
  }

  float final = readHeadingAvg(5);
  float girado = angleDiff(final, inicio);
  headingObjetivo = final;

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
  if (!bnoActivo) return;   //return

  float inicio = readHeadingAvg(5);
  float objetivo = normalize360(inicio + 180.0);
  turnToHeading(objetivo);

  delay(300);
}

bool tirarUnKitDisponible() {
  if (contadorKitsTotal >= MAX_KITS_TOTAL) {
    Serial.println("No quedan kits totales");
    return false;
  }

  if (contadorKitsDer < MAX_KITS_POR_SERVO) {
    accionarServo(SR, SERVO_DER_CERRADO, SERVO_DER_EMPUJE);

    contadorKitsDer++;
    contadorKitsTotal++;

    Serial.print("Kit lanzado desde DERECHA | Izq: ");
    Serial.print(contadorKitsIzq);
    Serial.print(" | Der: ");
    Serial.print(contadorKitsDer);
    Serial.print(" | Total: ");
    Serial.println(contadorKitsTotal);

    return true;
  }
  if (contadorKitsIzq < MAX_KITS_POR_SERVO) {
    accionarServo(SL, SERVO_IZQ_CERRADO, SERVO_IZQ_EMPUJE);

    contadorKitsIzq++;
    contadorKitsTotal++;

    Serial.print("Kit lanzado desde IZQUIERDA | Izq: ");
    Serial.print(contadorKitsIzq);
    Serial.print(" | Der: ");
    Serial.print(contadorKitsDer);
    Serial.print(" | Total: ");
    Serial.println(contadorKitsTotal);

    return true;
  }

  Serial.println("No quedan kits en ninguna torre");
  return false;
}

int tirarNKits(int cantidad) {
  int lanzados = 0;
  bool giradoParaIzquierda = false;

  for (int i = 0; i < cantidad; i++) {

    if (contadorKitsTotal >= MAX_KITS_TOTAL) {
      Serial.println("No quedan kits totales");
      break;
    }

    // ========= CASO 1: aun hay kits en derecha =========
    if (contadorKitsDer < MAX_KITS_POR_SERVO) {
      accionarServo(SR, SERVO_DER_CERRADO, SERVO_DER_EMPUJE);

      contadorKitsDer++;
      contadorKitsTotal++;
      lanzados++;

      Serial.print("Kit lanzado desde DERECHA | Izq: ");
      Serial.print(contadorKitsIzq);
      Serial.print(" | Der: ");
      Serial.print(contadorKitsDer);
      Serial.print(" | Total: ");
      Serial.println(contadorKitsTotal);

      delay(250);
      continue;
    }

    // ========= CASO 2: ya toca usar izquierda =========
    if (contadorKitsIzq < MAX_KITS_POR_SERVO) {

      if (!giradoParaIzquierda) {
        Serial.println("Girando 180 para lanzar desde IZQUIERDA");
        giro90Izq();
        delay(500);
        giro90Izq();
        delay(150);
        giradoParaIzquierda = true;
      }

      accionarServo(SL, SERVO_IZQ_CERRADO, SERVO_IZQ_EMPUJE);

      contadorKitsIzq++;
      contadorKitsTotal++;
      lanzados++;

      Serial.print("Kit lanzado desde IZQUIERDA | Izq: ");
      Serial.print(contadorKitsIzq);
      Serial.print(" | Der: ");
      Serial.print(contadorKitsDer);
      Serial.print(" | Total: ");
      Serial.println(contadorKitsTotal);

      delay(250);
      continue;
    }

    Serial.println("No quedan kits en ninguna torre");
    break;
  }


  if (giradoParaIzquierda) {
    Serial.println("Regresando 180 a orientacion original");
    giro90Izq();
    delay(500);
    giro90Izq();
    delay(150);
  }

  return lanzados;
}

void ISR_pulsoOpenMV() {
  if (modoCalibracion) return;
  if (enLackOfProgress) return;
  if (bloquearOpenMV) return;
  if (enRampa) return;
  if (modoReposicion) return;

  leerLetra = true;
}

float leerVoltajeOpenMV(int muestras = 8) {
  long suma = 0;

  for (int i = 0; i < muestras; i++) {
    suma += analogRead(pinVoltaje);
  }

  float raw = suma / (float)muestras;
  return (raw * 3.3f) / 4095.0f;   // Teensy a 12 bits
}

char decodificarLetraOpenMV(float v) {
  // phi = 0.00V
  // psi = 1.33V
  // omega = 3.00V

  if (v < 0.35f) {
    return 'P';   // phi
  }
  else if (v > 1.00f && v < 1.60f) {
    return 'S';   // psi
  }
  else if (v > 2.60f) {
    return 'O';   // omega
  }

  return '\0';
}

bool procesarSenalOpenMV() {

  if (!leerLetra) return false;

  leerLetra = false;

  delayMicroseconds(300);   // deja estabilizar el DAC

  float v = leerVoltajeOpenMV(8);
  char letra = decodificarLetraOpenMV(v);

  Serial.print("Voltaje OpenMV: ");
  Serial.println(v, 3);

  if (letra == '\0') {
    Serial.println("Senal invalida");
    return false;
  }

  letraPendiente = letra;

  if (letra == 'P') {
    Serial.println("OPENMV -> PHI");
  } else if (letra == 'S') {
    Serial.println("OPENMV -> PSI");
  } else if (letra == 'O') {
    Serial.println("OPENMV -> OMEGA");
  }

  return true;
}


bool atenderDeteccionOpenMV() {
  if (bloquearOpenMV) return false;
  if (enLackOfProgress) return false;
  
  // Tiene que existir una peticion pendiente desde OpenMV
  if (!leerLetra) return false;

  // ============================
  // 1) DETENER INMEDIATAMENTE
  // ============================
  stopMotors();
  delay(80);   // darle tiempo real al robot para asentarse

  // ============================
  // 2) MIENTRAS ESTA DETENIDO,
  //    LEER LOS TOF
  // ============================
  leerTOFS();

  const float UMBRAL_PARED_DERECHA = 180.0;

  // Si no hay pared derecha cercana, descartar
  if (Dist[4] == 0 || Dist[4] > UMBRAL_PARED_DERECHA) {
    Serial.print("Victima descartada por distancia. Dist[4]: ");
    Serial.println(Dist[4]);

    leerLetra = false;
    letraPendiente = '\0';

    // sigue detenido solo este instante; al salir, la navegacion continua
    return false;
  }

  // ============================
  // 3) TODAVIA DETENIDO,
  //    AHORA SI LEER LA LETRA
  // ============================
  if (!procesarSenalOpenMV()) {
    leerLetra = false;
    letraPendiente = '\0';
    return false;
  }

  // ============================
  // 4) SI YA HAY LETRA VALIDA,
  //    EJECUTAR ACCION DE VICTIMA
  // ============================
  stopMotors();

  int kitsSolicitados = 0;

  if (letraPendiente == 'P') {
    Serial.println("Detenido por PHI");
    kitsSolicitados = 2;

  }
  else if (letraPendiente == 'S') {
    Serial.println("Detenido por PSI");
    kitsSolicitados = 1;

  }
  else if (letraPendiente == 'O') {
    Serial.println("Detenido por OMEGA");
    kitsSolicitados = 0;

    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(255, 0, 0));   // rojo
    }
    pixels.show();
  }
  else {
    leerLetra = false;
    letraPendiente = '\0';
    return false;
  }

  // ============================
  // 5) PAUSA DE ATENCION
  // ============================
  unsigned long t0 = millis();
  bool kitsYaLanzados = false;

  while (millis() - t0 < 5000) {
    stopMotors();

    if (!kitsYaLanzados) {
      int lanzados = tirarNKits(kitsSolicitados);

      Serial.print("Kits solicitados: ");
      Serial.print(kitsSolicitados);
      Serial.print(" | Kits lanzados: ");
      Serial.println(lanzados);

      kitsYaLanzados = true;
    }

    delay(10);
  }

  // ============================
  // 6) RESTAURAR ESTADO VISUAL
  // ============================
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(255, 255, 230));
  }
  pixels.show();

  leerLetra = false;
  letraPendiente = '\0';

  return true;
}

void accionarServo(Servo &servo, int anguloCerrado, int anguloEmpuje) {
  servo.write(anguloCerrado);
  delay(800);
  servo.write(anguloEmpuje);
  delay(250);
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
  delay(80);
  esperarEstable();

  float actual = readHeadingRaw();
  float error = angleDiff(objetivo, actual);

  if (abs(error) > 1.5) {
    turnToHeading(objetivo);
    delay(60);
    esperarEstable();
  }

  float final = readHeadingAvg(5);
  float girado = angleDiff(final, inicio);
  headingObjetivo = final;

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
  int cal_vel = 180;            // Velocidad constante para calibración
  const float KP_CAL = 2.0;   // Fuerza de corrección (ajusta si serpentea)
  float promedio = 0;          // Declarada aquí para evitar scope error
  
  // Capturamos el ángulo actual para mantenerlo durante toda la maniobra
  float anguloObjetivo = readHeadingAvg(10); 

  // --- FASE 1: AVANCE RECTO HACIA LA PARED ---
  while (true) {
    float actual = readHeadingAvg(5); 
    float error = angleDiff(anguloObjetivo, actual);
    float correccion = error * KP_CAL;

    // Aplicamos corrección al avance
    forwardRaw(cal_vel - (int)correccion, cal_vel + (int)correccion);

    // Salir cuando ambos limit switches detecten la pared
    if (digitalRead(LLS) == 1 && digitalRead(RLS) == 1) {
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
  delay(80);
  esperarEstable();
  resincronizarHeading();


  // Mostrar resultado final en OLED
  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("OK! Dist:");
  display.setCursor(5, 25);
  display.print(promedio, 1);
  display.print(" mm");
  display.display();
  Serial.println("Calibracion");
  
  delay(500);
}

void acomodarObstaculoPID(String lado, float distanciaAvanzada, float anguloObjetivo, bool brusco) {

  float retrocederCMparaAcomodar = 10.0;
  long ticksObjetivo = retrocederCMparaAcomodar * TICKS_POR_CM;
  long fase1 = ticksObjetivo * 0.89;
  long fase2 = ticksObjetivo * 0.375;

  int velBase = 100;
  int delta = 125;

  // =========================
  // DETECTAR CONTACTO INICIAL
  // =========================
  bool izq = digitalRead(LLS);
  bool der = digitalRead(RLS);

  if (izq || der) {

    Serial.println("CONTACTO DETECTADO");

    stopMotors();
    delay(80);

    while (digitalRead(LLS) == 1 || digitalRead(RLS) == 1) {
      retroceder(1);

    }
    stopMotors();
    delay(80);

    // Guardar lado de impacto
    String ladoImpacto;
    if (der) ladoImpacto = "DER";
    else ladoImpacto = "IZQ";

    // =========================
    // FASE 1 (DIAGONAL)
    // =========================
    encLeft.write(0);
    encRight.write(0);

    while (true) {

      bool izqNow = digitalRead(LLS);
      bool derNow = digitalRead(RLS);

  
      if (izqNow && derNow) {
        Serial.println("PARED DETECTADA (FASE 1)");

        stopMotors();
        delay(120);

        float nuevoObjetivo;
        if (ladoImpacto == "DER")
          nuevoObjetivo = normalize360(anguloObjetivo + 90.0);
        else
          nuevoObjetivo = normalize360(anguloObjetivo - 90.0);

        turnToHeading(nuevoObjetivo);
        delay(100);
        esperarEstable();

        return;
      }

      long promedio = (abs(encLeft.read()) + abs(encRight.read())) / 2;
      if (promedio >= fase1) break;

      if (ladoImpacto == "DER")
        backwardRaw(velBase - (delta * 0.3), velBase + delta);
      else
        backwardRaw(velBase + delta, velBase - (delta * 0.3));
    }

    stopMotors();
    delay(120);

    // =========================
    // FASE 2 (GIRO LOCAL)
    // =========================
    encLeft.write(0);
    encRight.write(0);

    while (true) {

      bool izqNow = digitalRead(LLS);
      bool derNow = digitalRead(RLS);

      if (izqNow && derNow) {
        Serial.println("PARED DETECTADA (FASE 2)");

        stopMotors();
        delay(120);

        float nuevoObjetivo;
        if (ladoImpacto == "DER")
          nuevoObjetivo = normalize360(anguloObjetivo + 90.0);
        else
          nuevoObjetivo = normalize360(anguloObjetivo - 90.0);

        turnToHeading(nuevoObjetivo);
        delay(100);
        esperarEstable();

        return;
      }

      long promedio = (abs(encLeft.read()) + abs(encRight.read())) / 2;
      if (promedio >= fase2) break;

      if (ladoImpacto == "DER")
        backwardRaw(velBase + delta, 0);
      else
        backwardRaw(0, velBase + delta);
    }

    stopMotors();
    delay(120);
  }



  // =========================
  // AVANCE FINAL
  // =========================
  #define FACTOR_AVANCE 1.08

  float compensacionRetroceso = retrocederCMparaAcomodar * 1.1;
  float distanciaPendiente = (30.0 - distanciaAvanzada + compensacionRetroceso) * FACTOR_AVANCE;

  if (distanciaPendiente < 0) distanciaPendiente = 0;

  avanzar_optimizado(distanciaPendiente);

  stopMotors();
  delay(120);
  esperarEstable();
}



void esperarEstable() {
  float prev = readHeadingAvg(5);
  int estables = 0;

  const float UMBRAL = 0.5;  
  const int NECESARIOS = 3;   // menos ciclos necesarios

  while (estables < NECESARIOS) {
    delay(30);

    float actual = readHeadingAvg(5);

    if (abs(angleDiff(actual, prev)) < UMBRAL) {
      estables++;
    } else {
      estables = 0;
    }

    prev = actual;
  }
}



float controlLateralContinuo() {

  const float KP_LAT = 0.6;
  const float MIN_PARED = 40.0;
  const float MAX_PARED = 200.0;
  const float UMBRAL = 5.0;

  float s1 = 1000;
  float s4 = 1000;

  VL53L0X_RangingMeasurementData_t measure;


  if (sensorActivo[1]) {
    sensores[1].rangingTest(&measure, false);
    if (measure.RangeStatus != 4) 
      s1 = measure.RangeMilliMeter;
  }

  if (sensorActivo[4]) {
    sensores[4].rangingTest(&measure, false);
    if (measure.RangeStatus != 4) 
      s4 = measure.RangeMilliMeter;
  }

  bool paredIzq = (s1 > MIN_PARED && s1 < MAX_PARED);
  bool paredDer = (s4 > MIN_PARED && s4 < MAX_PARED);

  float correccion = 0;

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



void retroceder(float distanciaCM) {

  long ticksObjetivo = distanciaCM * TICKS_POR_CM;

  encLeft.write(0);
  encRight.write(0);

  float anguloObjetivoLocal = headingObjetivo;

  while (true) {

    long leftTicks = abs(encLeft.read());
    long rightTicks = abs(encRight.read());
    long promedio = (leftTicks + rightTicks) / 2;

    if (promedio >= ticksObjetivo) break;

    float actual = readHeadingRaw();
    float error = angleDiff(anguloObjetivoLocal, actual);

    float kP = 2.0;  // AJUSTAR si vibra o corrige poco
    int correccion = error * kP;

    correccion = constrain(correccion, -60, 60);

    int velIzq = BASE_SPEED - correccion;
    int velDer = BASE_SPEED + correccion;

    velIzq = constrain(velIzq, 0, 255);
    velDer = constrain(velDer, 0, 255);

    backwardRaw(velIzq, velDer);

    delayMicroseconds(200);
  }

  stopMotors();
  delay(80);
  esperarEstable();


  float errorFinal = angleDiff(anguloObjetivoLocal, readHeadingRaw());

  if (abs(errorFinal) > 1.5) {
    turnToHeading(anguloObjetivoLocal);
    delay(60);
    esperarEstable();
  }

  headingObjetivo = readHeadingAvg(5);
}
//////////////////////////////////////////////

long distanciaColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c, uint16_t p, volatile int ref[5]) {

  
  long distRGB =
    abs((int)r - ref[0]) +
    abs((int)g - ref[1]) +
    abs((int)b - ref[2]);

  long distC = abs((int)c - ref[3]);
  long distProx = abs((int)p - ref[4]);

  return (distRGB * 1) + (distC * 3) + (distProx * 3);

}            

String detectarColor() {

  uint16_t r, g, b, c;

  if (!apds.colorDataReady()) return "NONE";

  apds.getColorData(&r, &g, &b, &c);
  uint16_t p = apds.readProximity();

  int diffBR = abs((int)b - (int)r);
  int diffBG = abs((int)b - (int)g);
  int diffRG = abs((int)r - (int)g);
  int totales = diffBR + diffBG + diffRG;

  if (c == 0) return "NONE";

  // ===============================
  // LÓGICA DE DETECCIÓN
  // ===============================
  if (r < (colorBlanco[0] * 0.2) &&
      g < (colorBlanco[1] * 0.15) &&
      b < (colorBlanco[2] * 0.15) &&
      c < (colorBlanco[3] * 0.2)) {

    return "NEGRO";

  } else if (r < (colorBlanco[0] * 0.2) &&
             g < (colorBlanco[1] * 0.45) &&
             b < (colorBlanco[2] * 0.45) &&
             c > (colorBlanco[3] * 0.2)) {

    return "AZUL";

  } else if (r < (colorBlanco[0] * 0.9) &&
             g < (colorBlanco[1] * 0.3) &&
             b > (colorBlanco[2] * 0.08) &&
             c > (colorBlanco[3] * 0.2)) {

    return "ROJO";
  }

  if (p >= 32) {
    if (r > 290) {
      if (ignorarPlateado) return "BLANCO";
      return "PLATEADO";
    }
  }

  // ===== 2. RESPALDO CON DISTANCIA =====
  long distBlanco = distanciaColor(r, g, b, c, p, colorBlanco);
  long distPlateado = distanciaMinPlateado(r, g, b, c, p);
  long diferencia = distBlanco - distPlateado;

  // DEBUG
  static unsigned long lastColorPrint = 0;
  if (millis() - lastColorPrint > 200) {
    lastColorPrint = millis();
    Serial.print("R: "); Serial.print(r);
    Serial.print(" G: "); Serial.print(g);
    Serial.print(" B: "); Serial.print(b);
    Serial.print(" C: "); Serial.print(c);
    Serial.print(" P: "); Serial.print(p);
    Serial.print(" | Blanco: "); Serial.print(distBlanco);
    Serial.print(" Plateado: "); Serial.print(distPlateado);
    Serial.print(" Diff: "); Serial.println(diferencia);
  }

  // ===== DECISIÓN FINAL =====
  if (diferencia > MARGEN_PLATEADO) {
    if (ignorarPlateado) return "BLANCO";
    return "PLATEADO";
  }

  return "BLANCO";
}

void calibrarReferenciaColor(volatile int destino[5], const char* nombreSuperficie) {
  destino[0] = destino[1] = destino[2] = destino[3] = destino[4] = 0;

  const int muestras = 10;

  for (int i = 0; i < muestras; i++) {
    uint16_t r, g, b, c;

    while (!apds.colorDataReady()) delay(5);
    apds.getColorData(&r, &g, &b, &c);
    uint16_t p = apds.readProximity();

    destino[0] += r;
    destino[1] += g;
    destino[2] += b;
    destino[3] += c;
    destino[4] += p;

    delay(20);
  }
  
  for(int i=0; i<5; i++) destino[i] /= muestras;  
  Serial.print(nombreSuperficie);
  Serial.print(" -> R:");
  Serial.print(destino[0]);
  Serial.print(" G:");
  Serial.print(destino[1]);
  Serial.print(" B:");
  Serial.print(destino[2]);
  Serial.print(" C:");
  Serial.print(destino[3]);
  Serial.print(" P:");
  Serial.println(destino[4]);
}


void reaccionColor(String color, float distanciaRecorrida) {
  
  
  if (color == "NEGRO") {
    stopMotors();

    Serial.println("NEGRO → retrocediendo");

    retroceder(distanciaRecorrida); 

    delay(100);

    giro90Izq();
    delay(500);
    giro90Izq();

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
    ignorarAzul = false;
  }

  if (color == "ROJO") {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("ROJO");
    display.display();
    return;
  }

  if (color == "PLATEADO") {
    return;
  }
}

long distanciaMinPlateado(uint16_t r, uint16_t g, uint16_t b, uint16_t c, uint16_t p) {
  long minDist = 999999;

  for (int i = 0; i < NUM_PLATEADOS; i++) {
    long d = distanciaColor(r, g, b, c,p, colorPlateado[i]);
    if (d < minDist) {
      minDist = d;
    }
  }

  return minDist;
}



// =============================== INICIAR SENSORES
void iniciarTOFS() {
  // -------- APAGAR TODOS LOS TOF --------
  for (int i = 0; i < NUM_SENSORES; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
    sensorActivo[i] = false;
  }
  delay(20);
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

void mostrarOLED(String linea1, String linea2 = "", String linea3 = "") {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.println(linea1);

  if (linea2 != "") display.println(linea2);
  if (linea3 != "") display.println(linea3);

  display.display();
}







// ===============================DFS Y DIJKSTRA

int buscarNodoXY(int x, int y);
Direccion girarIzquierdaDir(Direccion d);
Direccion girarDerechaDir(Direccion d);
Direccion opuestaDir(Direccion d);
void siguienteXY(int x, int y, Direccion d, int &nx, int &ny);
void actualizarSalidasNodoActual();
int obtenerOCrearNodo(int x, int y);
int moverANodoEnDireccion(Direccion d);
void orientarRobotA(Direccion objetivo);
ResultadoMovimiento avanzarANodo(Direccion d);
bool haySalidaNueva(int idNodo);
bool obtenerDireccionNueva(Direccion &dElegida);
bool nodoPendiente(int idNodo);
void dijkstraNodos(int origen);
int buscarNodoPendienteMasCercano();
vector<int> reconstruirRutaNodos(int destino);
Direccion direccionHaciaVecino(int desde, int hacia);
void seguirRutaNodos(vector<int> ruta);
void explorarDFS_Dijkstra();
void iniciarMapa();
bool enNodoListoParaDecidir();
int penalizacionPrimerMovimiento(const vector<int>& ruta);



int buscarNodoXY(int x, int y) {
  for (int i = 0; i < (int)mapa.size(); i++) {
    if (mapa[i].x == x && mapa[i].y == y) return i;
  }
  return -1;
}



Direccion girarIzquierdaDir(Direccion d) {
  return (Direccion)((d + 3) % 4);
}

Direccion girarDerechaDir(Direccion d) {
  return (Direccion)((d + 1) % 4);
}

Direccion opuestaDir(Direccion d) {
  return (Direccion)((d + 2) % 4);
}

void manejarCalibracionManual() {
  stopMotors();

  bool switchActual = (digitalRead(PIN_REPOSICION) == LOW);
  bool flancoBajada = false;

  if (switchActual && !switchPrevio) {
    if (millis() - ultimoFlanco > 250) {
      flancoBajada = true;
      ultimoFlanco = millis();
    }
  }

  switchPrevio = switchActual;

  if (!flancoBajada) return;


  if (pasoCalibracion == 0) {
    mostrarOLED("CALIBRANDO", "BLANCO");
    Serial.println("Calibrando BLANCO...");
    calibrarReferenciaColor(colorBlanco, "BLANCO");

    pasoCalibracion = 1;
    mostrarOLED("PLATEADO", "MUESTRA 1");
    Serial.println("Coloca el robot en PLATEADO muestra 1 y baja la palanca");
    return;
  }

  if (pasoCalibracion >= 1 && pasoCalibracion <= 4) {
    int idx = pasoCalibracion - 1;

    mostrarOLED("CALIBRANDO", "PLATEADO", String(idx + 1));
    Serial.print("Calibrando PLATEADO muestra ");
    Serial.println(idx + 1);

    calibrarReferenciaColor(colorPlateado[idx], "PLATEADO");

    pasoCalibracion++;

    if (pasoCalibracion <= 4) {
      mostrarOLED("PLATEADO", "MUESTRA " + String(pasoCalibracion));
      Serial.print("Coloca el robot en PLATEADO muestra ");
      Serial.print(pasoCalibracion);
      Serial.println(" y baja la palanca");
    } else {
      modoCalibracion = false;
      leerLetra = false;
      letraPendiente = '\0';

      mostrarOLED("LISTO");
      Serial.println("Calibracion completa. Iniciando mapa...");

      delay(6000);

      iniciarMapa();
    }
  }
}

void siguienteXY(int x, int y, Direccion d, int &nx, int &ny) {
  nx = x;
  ny = y;

  if (d == NORTE) ny++;
  else if (d == ESTE) nx++;
  else if (d == SUR) ny--;
  else if (d == OESTE) nx--;
}

void actualizarSalidasNodoActual() {
  bool frente = libreFrente();
  bool izquierda = libreIzquierda();
  bool derecha = libreDerecha();

  Nodo &n = mapa[nodoActual];

  Direccion dFrente = dirActual;
  Direccion dIzq = girarIzquierdaDir(dirActual);
  Direccion dDer = girarDerechaDir(dirActual);

  n.vecinos[dFrente].existe = frente;
  n.vecinos[dIzq].existe = izquierda;
  n.vecinos[dDer].existe = derecha;
}

int moverANodoEnDireccion(Direccion d) {
  Nodo &actual = mapa[nodoActual];

  int nx, ny;
  siguienteXY(actual.x, actual.y, d, nx, ny);
  
  Serial.println("-----------");
  Serial.print("MOVIMIENTO: ");
  Serial.print(nombreDireccion(d));
  Serial.print(" | Desde (");
  Serial.print(actual.x);
  Serial.print(",");
  Serial.print(actual.y);
  Serial.print(") -> (");
  Serial.print(nx);
  Serial.print(",");
  Serial.print(ny);
  Serial.println(")");

  int nuevoId = obtenerOCrearNodo(nx, ny);

  mapa[nodoActual].vecinos[d].existe = true;
  mapa[nodoActual].vecinos[d].destino = nuevoId;
  mapa[nodoActual].vecinos[d].explorada = true;
  mapa[nodoActual].vecinos[d].esRojo = false;

  mapa[nuevoId].vecinos[opuestaDir(d)].existe = true;
  mapa[nuevoId].vecinos[opuestaDir(d)].destino = nodoActual;
  mapa[nuevoId].vecinos[opuestaDir(d)].explorada = true;
  mapa[nuevoId].vecinos[opuestaDir(d)].esRojo = false;


  nodoActual = nuevoId;
  mapa[nodoActual].visitado = true;

  return nuevoId;
}

void orientarRobotA(Direccion objetivo) {

  Serial.print("ORIENTAR: ");
  Serial.print(nombreDireccion(dirActual));
  Serial.print(" -> ");
  Serial.println(nombreDireccion(objetivo));

  if (dirActual == objetivo) return;

  if (girarDerechaDir(dirActual) == objetivo) {
    Serial.println("Accion: Giro DERECHA");
    giro90Der();
  }
  else if (girarIzquierdaDir(dirActual) == objetivo) {
    Serial.println("Accion: Giro IZQUIERDA");
    giro90Izq();
  }
  else {
    Serial.println("Accion: Giro 180");
    giro90Izq();
    delay(500);
    giro90Izq();
  }

  dirActual = objetivo;
  debugEstado("DESPUES_GIRO");
}

ResultadoMovimiento avanzarANodo(Direccion d) {

  orientarRobotA(d);


  ResultadoMovimiento r = avanzar_optimizado(30);

  if (r == MOV_ABORTADO) {
    return r;
  }

  if (r == MOV_OK) {

    int idLlegada = moverANodoEnDireccion(d);

    if (ultimoTileAzul) {
      mapa[idLlegada].tieneAzul = true;
    }

    if (ultimoTilePlateado) {
      mapa[idLlegada].tienePlateado = true;
    }
  }
  else if (r == MOV_AZUL) {
    int idLlegada = moverANodoEnDireccion(d);
    mapa[idLlegada].tieneAzul = true;

    if (ultimoTilePlateado) {
      mapa[idLlegada].tienePlateado = true;
    }
  }
  else if (r == MOV_ROJO){
    mapa[nodoActual].vecinos[d].existe = true;
    mapa[nodoActual].vecinos[d].destino = -1;     // aun no sabemos a qué nodo lleva
    mapa[nodoActual].vecinos[d].explorada = false;
    mapa[nodoActual].vecinos[d].esRojo = true;
  }
  else if (r == MOV_NEGRO) {
    mapa[nodoActual].vecinos[d].existe = false;
    mapa[nodoActual].vecinos[d].destino = -1;
    mapa[nodoActual].vecinos[d].explorada = true;
    dirActual = opuestaDir(d);
  }

  return r;
}

bool haySalidaNueva(int idNodo) {
  for (int d = 0; d < 4; d++) {
    if (mapa[idNodo].vecinos[d].existe && !mapa[idNodo].vecinos[d].explorada) {
      return true;
    }
  }
  return false;
}

const Direccion PRIORIDAD_ABSOLUTA[4] = {NORTE, ESTE, SUR, OESTE};

bool obtenerDireccionNueva(Direccion &dElegida, bool permitirRojo) {
  for (int i = 0; i < 4; i++) {
    Direccion d = PRIORIDAD_ABSOLUTA[i];

    if (mapa[nodoActual].vecinos[d].existe &&
        !mapa[nodoActual].vecinos[d].explorada) {

      if (!permitirRojo && mapa[nodoActual].vecinos[d].esRojo) {
        continue;
      }

      dElegida = d;
      return true;
    }
  }
  return false;
}

bool exploracionCompleta() {
  for (int i = 0; i < (int)mapa.size(); i++) {
    for (int d = 0; d < 4; d++) {
      if (mapa[i].vecinos[d].existe && !mapa[i].vecinos[d].explorada) {
        return false;
      }
    }
  }
  return true;
}
bool enInicio() {
  return nodoActual == nodoInicio;
}


bool nodoPendienteNormal(int idNodo) {
  for (int d = 0; d < 4; d++) {
    if (mapa[idNodo].vecinos[d].existe &&
        !mapa[idNodo].vecinos[d].explorada &&
        !mapa[idNodo].vecinos[d].esRojo) {
      return true;
    }
  }
  return false;
}

bool hayPendientesNormalesEnMapa() {
  for (int i = 0; i < (int)mapa.size(); i++) {
    for (int d = 0; d < 4; d++) {
      Conexion &c = mapa[i].vecinos[d];

      if (c.existe && !c.explorada && !c.esRojo) {
        return true;
      }
    }
  }
  return false;
}

void dijkstraNodos(int origen) {
  int n = mapa.size();
  distancias.assign(n, 2147483647);
  padre.assign(n, -1);

  priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> pq;

  distancias[origen] = 0;
  pq.push({0, origen});

  while (!pq.empty()) {
    int d = pq.top().first;
    int u = pq.top().second;
    pq.pop();

    if (d > distancias[u]) continue;

    for (int dir = 0; dir < 4; dir++) {
      if (!mapa[u].vecinos[dir].existe) continue;

      int v = mapa[u].vecinos[dir].destino;
      if (v == -1) continue;

      int costo = 1;

      if (mapa[v].tieneAzul){
        costo +=20;
      }

      if (distancias[v] > distancias[u] + costo) {
        distancias[v] = distancias[u] + costo;
        padre[v] = u;
        pq.push({distancias[v], v});
      }
    }
  }
}

int buscarNodoPendienteMasCercano(bool permitirRojo) {
  dijkstraNodos(nodoActual);

  int mejorNodo = -1;
  int mejorDist = 2147483647;
  int mejorPenalizacion = 2147483647;

  for (int i = 0; i < (int)mapa.size(); i++) {
    bool pendiente = false;

    for (int d = 0; d < 4; d++) {
      if (mapa[i].vecinos[d].existe &&
          !mapa[i].vecinos[d].explorada) {

        if (!permitirRojo && mapa[i].vecinos[d].esRojo) continue;

        pendiente = true;
        break;
      }
    }

    if (!pendiente) continue;
    if (distancias[i] == 2147483647) continue;

    vector<int> ruta = reconstruirRutaNodos(i);
    int penal = penalizacionPrimerMovimiento(ruta);

    if (distancias[i] < mejorDist ||
       (distancias[i] == mejorDist && penal < mejorPenalizacion)) {
      mejorDist = distancias[i];
      mejorPenalizacion = penal;
      mejorNodo = i;
    }
  }

  return mejorNodo;
}

vector<int> reconstruirRutaNodos(int destino) {
  vector<int> ruta;
  int actual = destino;

  while (actual != -1) {
    ruta.push_back(actual);
    actual = padre[actual];
  }

  reverse(ruta.begin(), ruta.end());
  return ruta;
}


Direccion direccionHaciaVecino(int desde, int hacia) {
  for (int d = 0; d < 4; d++) {
    if (mapa[desde].vecinos[d].existe &&
        mapa[desde].vecinos[d].destino == hacia) {
      return (Direccion)d;
    }
  }

  return NORTE; // fallback
}

void seguirRutaNodos(vector<int> ruta) {
  for (int i = 1; i < ruta.size(); i++) {
    int desde = ruta[i - 1];
    int hacia = ruta[i];

    Direccion d = direccionHaciaVecino(desde, hacia);
    ResultadoMovimiento r = avanzarANodo(d);

    if (r == MOV_NEGRO) {
      break;
    }
  }
}

void explorarDFS_Dijkstra() {
  leerTOFS();
  debugEstado("ANTES_DECISION");

  mapa[nodoActual].visitado = true;
  actualizarSalidasNodoActual();

  bool permitirRojo = !hayPendientesNormalesEnMapa() || modoPostReinicio;
  bool permitirRojoAhora = !hayPendientesNormalesEnMapa() || modoPostReinicio;

  if (enInicio() && exploracionCompleta()) {
    Serial.println("Regrese al inicio y exploracion completa");
    indicarFinal();
    while (true) { delay(1000); }
  }

  Direccion dNueva;
  if (obtenerDireccionNueva(dNueva, permitirRojo)) {
    Serial.print("DECISION DFS -> ");
    Serial.println(nombreDireccion(dNueva));
    avanzarANodo(dNueva);
    return;
  }

  int pendiente = buscarNodoPendienteMasCercano(permitirRojo);

  if (pendiente == -1) {
    Serial.println("Exploracion completa");

    if (!enInicio()) {
      Serial.println("Regresando al nodo inicial...");
      dijkstraNodos(nodoActual);
      vector<int> rutaInicio = reconstruirRutaNodos(nodoInicio);
      seguirRutaNodos(rutaInicio);
      return;
    }

    indicarFinal();
    while (true) { delay(1000); }
  }

  dijkstraNodos(nodoActual);
  vector<int> ruta = reconstruirRutaNodos(pendiente);
  seguirRutaNodos(ruta);
}

int obtenerOCrearNodo(int x, int y) {
  int id = buscarNodoXY(x, y);
  if (id != -1) return id;

  Nodo nuevo;
  nuevo.id = mapa.size();
  nuevo.x = x;
  nuevo.y = y;
  nuevo.visitado = false;
  nuevo.tieneAzul = false;
  nuevo.tienePlateado = false;

  for (int i = 0; i < 4; i++) {
    nuevo.vecinos[i].destino = -1;
    nuevo.vecinos[i].existe = false;
    nuevo.vecinos[i].explorada = false;
    nuevo.vecinos[i].esRojo = false;
  }

  mapa.push_back(nuevo);
  return nuevo.id;
}


void reiniciarTodoDFS() {
  stopMotors();

  mapa.clear();
  distancias.clear();
  padre.clear();

  nodoActual = -1;
  nodoInicio = -1;
  dirActual = NORTE;
  listoParaDecidir = true;

  ultimoTileAzul = false;
  ultimoTilePlateado = false;
  ignorarPlateado = false;
  interrumpidoPorAzul = false;
  
  contadorAzul = 0;
  pausaAzul = false;
  tiempoAzul = 0;
  ignorarAzul = false;
  contadorRojo = 0;

  // 👇 ESTA ES LA LINEA IMPORTANTE
  modoPostReinicio = true;

  iniciarMapa();

  esperarEstable();
  resincronizarHeading();

  Serial.println("DFS reiniciado por completo");
  Serial.println("MODO POST-REINICIO ACTIVADO -> ROJO PERMITIDO");
  Serial.print("Nodo actual: ");
  Serial.println(nodoActual);
  Serial.print("Direccion logica: ");
  Serial.println(nombreDireccion(dirActual));
}


void iniciarMapa() {
  mapa.clear();
  nodoActual = obtenerOCrearNodo(0, 0);
  nodoInicio = nodoActual;
  mapa[nodoActual].visitado = true;
  mapa[nodoActual].tieneAzul = false;
  mapa[nodoActual].tienePlateado = false;
  dirActual = NORTE;
}

bool enNodoListoParaDecidir() {
  return listoParaDecidir;
}

int penalizacionPrimerMovimiento(const vector<int>& ruta) {
  if (ruta.size() < 2) return 0;

  Direccion primeraDir = direccionHaciaVecino(ruta[0], ruta[1]);

  if (primeraDir == dirActual) return 0;                         // seguir recto
  if (primeraDir == girarDerechaDir(dirActual)) return 1;        // giro leve
  if (primeraDir == girarIzquierdaDir(dirActual)) return 1;      // giro leve
  if (primeraDir == opuestaDir(dirActual)) return 3;             // 180 es peor

  return 2;
}

void indicarFinal() {
  stopMotors();

  Serial.println("FINAL: regreso al inicio");

  // OLED
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 10);
  display.println("FIN");
  display.setCursor(0, 38);
  display.println("EN INICIO");
  display.display();

  // NeoPixel: destellos verdes
  for (int k = 0; k < 6; k++) {
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 255, 0));
    }
    pixels.show();
    delay(250);

    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    }
    pixels.show();
    delay(250);
  }

  // Dejar fijo en verde
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(0, 255, 0));
  }
  pixels.show();
}


const char* nombreDireccion(Direccion d) {
  switch (d) {
    case NORTE: return "NORTE";
    case ESTE:  return "ESTE";
    case SUR:   return "SUR";
    case OESTE: return "OESTE";
  }
  return "?";
}

void debugEstado(const char* evento) {
  Serial.print("[");
  Serial.print(evento);
  Serial.print("] ");

  Serial.print("NodoID=");
  Serial.print(nodoActual);

  Serial.print(" | XY=(");
  Serial.print(mapa[nodoActual].x);
  Serial.print(",");
  Serial.print(mapa[nodoActual].y);
  Serial.print(")");

  Serial.print(" | Dir=");
  Serial.print(nombreDireccion(dirActual));

  Serial.println();
}

void setup() {
  Serial.begin(9600);
  SL.attach(25);
  SR.attach(24);

  SL.write(SERVO_IZQ_EMPUJE);
  SR.write(SERVO_DER_EMPUJE);

  analogReadResolution(12);
  pinMode(pinVoltaje, INPUT);
  pinMode(pinPulso, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(pinPulso), ISR_pulsoOpenMV, RISING);

  Wire.begin();
  Wire.setClock(400000);

  setupMotor(L_F1, L_F2, L_F_PWM);
  setupMotor(L_B1, L_B2, L_B_PWM);
  setupMotor(R_F1, R_F2, R_F_PWM);
  setupMotor(R_B1, R_B2, R_B_PWM);

  pinMode(LLS, INPUT);
  pinMode(RLS, INPUT);
  pinMode(PIN_REPOSICION, INPUT_PULLUP);

  iniciarTOFS();

  pinMode(BNO_RST_PIN, OUTPUT);
  digitalWrite(BNO_RST_PIN, HIGH);

  if (iniciarBNO(OPERATION_MODE_NDOF)) {
    Serial.println("BNO OK");
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }

  display.display();
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  pixels.begin();
  pixels.setBrightness(100);

  for (int i = 0; i < 8; i++) {
    pixels.setPixelColor(i, pixels.Color(255, 255, 230));
  }
  pixels.show();

  if (!apds.begin()) {
    Serial.println("Error APDS");
    while (1);
  }

  apds.enableColor(true);
  apds.enableProximity(true);
  apds.setADCGain(APDS9960_AGAIN_4X);

  mostrarOLED("PONER", "EN BLANCO");
  Serial.println("Modo calibracion manual activo");
  Serial.println("Paso 0: coloca el robot sobre BLANCO y baja la palanca");
}
// ===============================
void loop() {

  if (modoCalibracion) {
    manejarCalibracionManual();
    return;
  }

  bool switchActivo = (digitalRead(PIN_REPOSICION) == LOW);

  // =========================

  if (switchActivo && !modoReposicion) {
    modoReposicion = true;
    restauracionHecha = false;

    stopMotors();

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.println("PAUSA");
    display.println("CHECKPOINT");
    display.display();

    Serial.println("Switch en 0 -> pausa/reposicion");
  }

  // =========================
  // EN 0
  // =========================
  if (modoReposicion) {
    stopMotors();

    if (!restauracionHecha) {
      reiniciarTodoDFS();
      Serial.println("Mapa restaurado");
      restauracionHecha = true;
    }

    // mientras siga en 0, no navegar
    if (switchActivo) {
      return;
    }
  }

  // =========================
  // VOLVER A 1
  // =========================
  if (!switchActivo && modoReposicion) {
    modoReposicion = false;


    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.println("REANUDAR");
    display.display();

    Serial.println("Switch en 1 -> reanudar navegacion");
  }
  if (saliendoDeRampa && !enRampa) {
  Serial.println("Esperando estabilizacion post-rampa...");

  stopMotors();
  delay(150);

  esperarEstable();   // tu estabilidad por heading
  delay(120);         // pequeña espera extra

  // Reset una sola vez al terminar la rampa
  resetBNO_Hardware(OPERATION_MODE_NDOF);

  // vuelve a esperar a que se estabilice ya reiniciado
  delay(150);
  esperarEstable();

  saliendoDeRampa = false;

  Serial.println("IMU reiniciada y estabilizada, continuar DFS");
  return;
  }

  if (enNodoListoParaDecidir() && !enRampa) {
    listoParaDecidir = false;
    explorarDFS_Dijkstra();
    listoParaDecidir = true;
  }
}