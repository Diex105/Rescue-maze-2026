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
  MOV_ROJO,
  MOV_CHOQUE
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

#define PIN_NEOPIXEL 23
#define NUM_PIXELS   8
#define NEO_PIN 33
#define NUMPIXELS 8
Adafruit_NeoPixel pixels(NUMPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels1(NUM_PIXELS, PIN_NEOPIXEL , NEO_GRB + NEO_KHZ800);

#define PIN_REPOSICION 22
// servos
Servo SR;
Servo SL;

int contadorKitsIzq   = 0;
int contadorKitsDer   = 0;
int contadorKitsTotal = 0;
const int MAX_KITS_TOTAL     = 8;
const int MAX_KITS_POR_SERVO = 4;


const int SERVO_IZQ_CERRADO = 160;
const int SERVO_IZQ_EMPUJE  = 60;

const int SERVO_DER_CERRADO = 0;
const int SERVO_DER_EMPUJE  = 100;
//comunicación
#define OPENMV_IZQ_SERIAL Serial5
#define OPENMV_DER_SERIAL Serial2

const uint32_t BAUD_OPENMV = 115200;

bool bloquearOpenMV = false;
unsigned long bloquearOpenMV_hasta = 0;

volatile bool leerValor = false;
volatile int valorPendiente = -1;
char ladoDeteccion = 'N';

int proxBlancoMin = 999;
int proxBlancoMax = -999;


struct DeteccionVictima {
  int  kits;   // 0,1,2
  char lado;   // 'I' o 'D'
};

#define COLA_DET_MAX 6
struct ColaDet {
  DeteccionVictima buf[COLA_DET_MAX];
  uint8_t head = 0;   // indice de salida
  uint8_t tail = 0;   // indice de entrada
  uint8_t count = 0;  // elementos validos
};

void colaClear(ColaDet &c) { c.head = 0; c.tail = 0; c.count = 0; }

bool colaPush(ColaDet &c, int kits, char lado) {
  if (c.count >= COLA_DET_MAX) {           // llena -> descarta la mas vieja
    c.head = (c.head + 1) % COLA_DET_MAX;
    c.count--;
  }
  c.buf[c.tail].kits = kits;
  c.buf[c.tail].lado = lado;
  c.tail = (c.tail + 1) % COLA_DET_MAX;
  c.count++;
  return true;
}

bool colaPop(ColaDet &c, DeteccionVictima &out) {
  if (c.count == 0) return false;
  out = c.buf[c.head];
  c.head = (c.head + 1) % COLA_DET_MAX;
  c.count--;
  return true;
}

ColaDet colaDetecciones;

char tramaIzq[24]; int idxIzq = 0; bool enTramaIzq = false;
char tramaDer[24]; int idxDer = 0; bool enTramaDer = false;

unsigned long cooldownIzq_hasta = 0;
unsigned long cooldownDer_hasta = 0;
const unsigned long COOLDOWN_VICTIMA_MS = 3000;

bool arcoirisActivo = false;
unsigned long arcoirisInicio = 0;
unsigned long arcoirisDuracion = 4000;
int arcoirisOffset = 0;
unsigned long ultimoUpdateArcoiris = 0;

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
bool enLackOfProgress = false;

const unsigned long TIEMPO_SUBIDA = 2000;
const unsigned long TIEMPO_BAJADA = 1500;
const unsigned long TIEMPO_MOVIMIENTO = 6000;

// VELOCIDADES Y DISTANCIA
// ===============================
#define BASE_SPEED 180
#define TURN_SPEED_MAX 200
#define TURN_SPEED_MIN 150
#define TICKS_POR_CM 225.0

// ENCODERS

Encoder encLeft(15, 14);
Encoder encRight(11, 12);
int modoLateral = 0;
int contadorFrentes = 0;

// IMU

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
#define NUM_SENSORES 9
#define NUM_TOF_FIJOS 9

int xshutPins[NUM_SENSORES] = {
  1, 3, 0, 5, 6, 2, 4, 9, 10
};

// Direcciones SOLO para S0 a S6
uint8_t direcciones[NUM_TOF_FIJOS] = {
  0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
};

Adafruit_VL53L0X sensores[NUM_SENSORES];
bool sensorActivo[NUM_SENSORES];


float Dist[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };


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

bool limitIzqPresionado() {
  return digitalRead(LLS) == LOW;
}

bool limitDerPresionado() {
  return digitalRead(RLS) == LOW;
}
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
  
  
  static int lastLeftSpeed = 0;
  static int lastRightSpeed = 0;
  
  int maxAccel = 30; 
  
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

  digitalWrite(BNO_RST_PIN, LOW);
  delay(15);
  digitalWrite(BNO_RST_PIN, HIGH);

  
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

  const int VEL_MAX = 250;
  const int VEL_MIN = 170;       
  const int VEL_FINE = 155;      

  const float ANGULO_TOL = 1.5;  
  const int SETTLE_MS = 120;     
  const float FASE_FINE = 6.0;   
  const float FASE_BRAKE = 18.0; 

  float integral = 0.0;
  float lastError = 0.0;
  unsigned long lastUpdate = micros();
  unsigned long inTolStart = 0;

  bool lastTurnLeft = true;
  int ciclosSeguidosIgual = 0;
  float ultimoErrorSigno = 0.0;
  unsigned long tInicio = millis();
  bool tiempo = false;


  while (true) {
    
    if (millis() - tInicio > TIEMPO_MOVIMIENTO) {
      stopMotors();
      tiempo = true;
      break;
    }

    if (servicioOpenMV()){ 
      lastUpdate = micros();
      inTolStart = 0;
      lastError = 0;
      
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
      float factor = abs(error) / FASE_BRAKE;   
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
  delay(30);

 
  float final = readHeadingAvg(10);
  float errorFinal = angleDiff(objetivo, final);

  headingObjetivo = objetivo;

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

  const float alpha = 0.35; 

  if (!pitchInit) {
    pitchFiltrado = rawPitch;
    pitchInit = true;
  } else {
    pitchFiltrado = alpha * rawPitch + (1.0 - alpha) * pitchFiltrado;
  }

  pitch = pitchFiltrado;
}
float readPitch() {
  float pitch;
  leerPitchFiltrado(pitch);
  return pitch;
}

float readHeading() {
  return readHeadingRaw();
}
// =============================== LECTURAS DE TOFS
const uint16_t TOF_LIBRE = 8190;
const uint16_t TOF_MIN_UTIL = 12;
const uint16_t TOF_MAX_UTIL = 1000;

// Por ahora usamos solo S0 a S6 para evitar saturar I2C.
// Si quieres activar S7 y S8 luego, cambia este valor a 9.
const int NUM_TOF_LEER = 7;

// Correcciones por sensor
int offsetTOF[NUM_SENSORES] = {
 -15,    // S0
 -30,   // S1
 -24,   // S2
  0,    // S3
 -5,    // S4
 -23,   // S5
 -7,    // S6
  0,    // S7
  0     // S8
};

void leerTOFS() {
  VL53L0X_RangingMeasurementData_t measure;

  for (int i = 0; i < NUM_TOF_LEER; i++) {

    if (!sensorActivo[i]) {
      Dist[i] = TOF_LIBRE;
      continue;
    }

    sensores[i].rangingTest(&measure, false);

    uint16_t lectura = measure.RangeMilliMeter;
    uint8_t status = measure.RangeStatus;

    bool invalida = false;

    if (status == 4) invalida = true;
    if (lectura == 0) invalida = true;
    if (lectura < TOF_MIN_UTIL) invalida = true;
    if (lectura > TOF_MAX_UTIL) invalida = true;

    if (invalida) {
      // Punto clave:
      // No conservar el valor anterior.
      // Si no hay lectura clara, se toma como "sin pared".
      Dist[i] = TOF_LIBRE;
    } 
    else {
      int d = lectura + offsetTOF[i];

      if (d < 0) d = 0;

      Dist[i] = d;
    }

    delayMicroseconds(20);
  }

  // Los sensores que no se leen se dejan como libres,
  // para que nunca se queden con basura anterior.
  for (int i = NUM_TOF_LEER; i < NUM_SENSORES; i++) {
    Dist[i] = TOF_LIBRE;
  }
}

// =============================== SECUENCIAS DE MOVIMIENTOS
ResultadoMovimiento avanzar_optimizado(int DISTANCIA_CM) {
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  ultimoTileAzul = false;
  ultimoTilePlateado = false;
  ignorarPlateado = false;

  
  const float KP = 0.6;      
  const float KI = 0.02;     
  const float KD = 0.05;    
  const int COMPENSACION_INERCIA = 50;

  long ticksObjetivo = (DISTANCIA_CM * TICKS_POR_CM) - COMPENSACION_INERCIA;

  float anguloObjetivo = headingObjetivo;

  unsigned long tInicio = millis();
  bool tiempo = false;  

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
    if (servicioOpenMV()) {

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
          delay(30);
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
      // Calcular error de orientación con el IMU
      float anguloActual = readHeadingRaw(); // Lectura rápida
      float errorGyro = angleDiff(anguloObjetivo, anguloActual); // Diferencia angular real
      
      // PID de Orientación
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
    
   
    forwardRaw(currentLeftSpeed, currentRightSpeed);

    bool L = limitIzqPresionado();
    bool R = limitDerPresionado();
    
    if (R || L) {
      stopMotors();
      delay(50);
      
      Serial.println("Choque detectado");

      long leftTicks2 = abs(encLeft.read());
      long rightTicks2 = abs(encRight.read());
      long promedio2 = (leftTicks2 + rightTicks2) / 2;
      float distanciaChoque = promedio2 / TICKS_POR_CM;

      if (R && !L) acomodarObstaculoPID("DER", distanciaChoque);
      else if (L && !R) acomodarObstaculoPID("IZQ", distanciaChoque);
      else acomodarObstaculoPID("DER", distanciaChoque);
      
      return MOV_OK;
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

  if (!tiempo){
    turnToHeading(anguloObjetivo); 
    delay(30);

  }

  headingObjetivo = anguloObjetivo;


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
  if(Dist[0] <= 180.0){
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

    float actual = readHeadingAvg(5);
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
  delay(30);
}
float leerPitchRampa() {
  if (!bnoActivo) return 0.0;

  sensors_event_t event;
  bno.getEvent(&event);

  return event.orientation.z;  // pitch
}

float leerRollRampa() {
  if (!bnoActivo) return 0.0;

  sensors_event_t event;
  bno.getEvent(&event);

  return event.orientation.y;  // roll lateral
}

float controlLateralRampaSuave(float pitch, bool posibleEscalon) {
  float corrLat = controlLateralContinuo();

  // En rampa, la correccion lateral debe ser mucho mas suave
  corrLat *= 0.38;

  // En escalon, todavia mas suave para no voltearse
  if (posibleEscalon) {
    corrLat *= 0.45;
  }

  // En subida o bajada pronunciada, limitar mas
  if (fabs(pitch) > 22.0) {
    corrLat *= 0.75;
  }

  corrLat = constrain(corrLat, -18.0, 18.0);

  return corrLat;
}
void direccionAdelanteRampa() {
  // Misma dirección que usabas en tu función original de subida

  digitalWrite(L_F1, LOW);
  digitalWrite(L_F2, HIGH);

  digitalWrite(L_B1, HIGH);
  digitalWrite(L_B2, LOW);

  digitalWrite(R_F1, HIGH);
  digitalWrite(R_F2, LOW);

  digitalWrite(R_B1, LOW);
  digitalWrite(R_B2, HIGH);
}

void direccionFrenoTotalBajada() {
  /*
    Freno activo para bajada:
    manda los motores en sentido contrario al avance.
    No es para avanzar hacia atrás mucho tiempo,
    solo para pulsos cortos de freno.
  */

  digitalWrite(L_F1, HIGH);
  digitalWrite(L_F2, LOW);

  digitalWrite(L_B1, LOW);
  digitalWrite(L_B2, HIGH);

  digitalWrite(R_F1, LOW);
  digitalWrite(R_F2, HIGH);

  digitalWrite(R_B1, HIGH);
  digitalWrite(R_B2, LOW);
}

void aplicarPWMRampa(int pwmLF, int pwmLB, int pwmRF, int pwmRB) {
  pwmLF = constrain(pwmLF, 0, 255);
  pwmLB = constrain(pwmLB, 0, 255);
  pwmRF = constrain(pwmRF, 0, 255);
  pwmRB = constrain(pwmRB, 0, 255);

  analogWrite(L_F_PWM, pwmLF);
  analogWrite(L_B_PWM, pwmLB);
  analogWrite(R_F_PWM, pwmRF);
  analogWrite(R_B_PWM, pwmRB);
}
void avanzarInclinacion(float anguloObjetivo) {

  enRampa = true;

  Serial.println("=== RAMPA VERSION ESTABLE ===");

  const float KP_ANG = 2.1;
  const float KD_ANG = 0.05;

  const int MAX_CORRECCION_ANG = 25;

  const unsigned long TIEMPO_MAX_RAMPA = 12000;
  const unsigned long TIEMPO_PLANO_SALIDA = 550;
  const unsigned long TIEMPO_MIN_RAMPA = 900;

  unsigned long tInicio = millis();
  unsigned long tPlano = 0;

  float lastErrorAng = 0;
  unsigned long lastTime = micros();

  float pitchAnterior = readPitch();

  while (true) {

    float pitch = readPitch();
    float heading = readHeading();

    float errorAng = angleDiff(anguloObjetivo, heading);

    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    if (dt <= 0) dt = 0.001;

    float derivada = (errorAng - lastErrorAng) / dt;
    derivada = constrain(derivada, -100.0, 100.0);

    lastErrorAng = errorAng;
    lastTime = now;

    float correccionAng = KP_ANG * errorAng + KD_ANG * derivada;
    correccionAng = constrain(correccionAng, -MAX_CORRECCION_ANG, MAX_CORRECCION_ANG);

    int pwmLF = 0;
    int pwmLB = 0;
    int pwmRF = 0;
    int pwmRB = 0;

    float cambioPitch = abs(pitch - pitchAnterior);
    pitchAnterior = pitch;

    bool posibleEscalon = cambioPitch > 5.0;

    // =========================
    // SUBIDA / TRANSICION
    // =========================
    if (pitch <= 8.0) {

      direccionAdelanteRampa();

      int velBase = 180;

      if (pitch < -30.0) {
        velBase = 220;
      }
      else if (pitch < -20.0) {
        velBase = 205;
      }
      else if (pitch < -12.0) {
        velBase = 190;
      }
      else {
        velBase = 170;
      }

      // Si parece escalon, meter torque corto
      if (posibleEscalon || pitch < -24.0) {
        unsigned long fasePulso = (millis() - tInicio) % 650;

        if (fasePulso < 150) {
          velBase += 20;
        }
      }

      velBase = constrain(velBase, 150, 230);

      // Lateral suave usando tu calibracion lateral real
      float correccionLat = controlLateralContinuo();

      // En rampa no queremos que sea tan agresiva como en piso
      correccionLat *= 0.40;
      correccionLat = constrain(correccionLat, -18.0, 18.0);

      float correccionTotal = correccionAng + correccionLat;
      correccionTotal = constrain(correccionTotal, -28.0, 28.0);

      /*
        IMPORTANTE:
        Signo corregido.
        Igual que en avanzar_optimizado:
        izquierda = base + correccion
        derecha   = base - correccion
      */
      pwmLF = velBase + (int)correccionTotal;
      pwmLB = velBase + (int)correccionTotal;
      pwmRF = velBase - (int)correccionTotal;
      pwmRB = velBase - (int)correccionTotal;

      pwmLF = constrain(pwmLF, 80, 240);
      pwmLB = constrain(pwmLB, 80, 240);
      pwmRF = constrain(pwmRF, 80, 240);
      pwmRB = constrain(pwmRB, 80, 240);

      aplicarPWMRampa(pwmLF, pwmLB, pwmRF, pwmRB);
    }

    // =========================
    // BAJADA
    // =========================
    else {

      /*
        Como viste que el peso lo baja bien:
        no lo empujamos hacia adelante.
        Solo dejamos motores en 0 y metemos freno corto si la bajada es fuerte.
      */

      bool frenar = false;

      if (pitch > 22.0) {
        frenar = ((millis() - tInicio) % 150) < 25;
      }
      else if (pitch > 13.0) {
        frenar = ((millis() - tInicio) % 190) < 18;
      }

      if (frenar) {
        direccionFrenoTotalBajada();

        int pwmFreno = 70;

        if (pitch > 22.0) {
          pwmFreno = 85;
        }

        aplicarPWMRampa(pwmFreno, pwmFreno, pwmFreno, pwmFreno);
      }
      else {
        // Baja por gravedad
        aplicarPWMRampa(0, 0, 0, 0);
      }
    }

    // =========================
    // SALIDA DE RAMPA
    // =========================
    // Esta parte la dejamos parecida a la que dices que funcionaba bien.
    if ((millis() - tInicio) > TIEMPO_MIN_RAMPA && abs(pitch) < 7.0) {
      if (tPlano == 0) {
        tPlano = millis();
      }

      if (millis() - tPlano >= TIEMPO_PLANO_SALIDA) {
        Serial.println("Rampa: piso detectado");
        break;
      }
    }
    else {
      tPlano = 0;
    }

    if (millis() - tInicio > TIEMPO_MAX_RAMPA) {
      Serial.println("Rampa: salida por timeout");
      break;
    }

    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 250) {
      lastDebug = millis();

      Serial.print("RAMPA | pitch:");
      Serial.print(pitch);

      Serial.print(" heading:");
      Serial.print(heading);

      Serial.print(" err:");
      Serial.print(errorAng);

      Serial.print(" corrAng:");
      Serial.print(correccionAng);

      Serial.print(" pwmLF:");
      Serial.print(pwmLF);

      Serial.print(" pwmRF:");
      Serial.println(pwmRF);
    }

    delay(10);
  }

  stopMotors();
  delay(150);

  enRampa = false;

  headingObjetivo = anguloObjetivo;
  turnToHeading(headingObjetivo);

  Serial.println("=== FIN RAMPA VERSION ESTABLE ===");
}

/*
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
  const float KD = 0.04;
  const int MAX_CORRECCION = 25;

  const int TRIM_IZQ = -30;
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
      correccionLat *= 0.5;
    }

    float correccionTotal = correccionAng + correccionLat;
    correccionTotal = constrain(correccionTotal, -18, 18);

    lastErrorAng = errorAng;
    lastTime = currentTime;

    // =========================
    // VELOCIDADES BASE
    // =========================
    int vel_subida = 205; //185

    int pwmLF = vel_subida;
    int pwmLB = vel_subida;
    int pwmRF = vel_subida;
    int pwmRB = vel_subida;

    // =========================
    // SUBIDA
    // =========================
    //invertir signos
    if (pitch <= -35) {
      pwmLF = vel_subida + 30;
      pwmLB = vel_subida + 35;
      pwmRF = vel_subida + 30;
      pwmRB = vel_subida - 25;
    } 
    else if (pitch <= -20) {
      pwmLF = vel_subida + 15;
      pwmLB = vel_subida + 20;
      pwmRF = vel_subida + 15;
      pwmRB = vel_subida - 10;
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
    if (pitch > 13) {
      pwmLF = vel_subida - 110; //45
      pwmLB = vel_subida - 60;
      pwmRF = vel_subida - 110;
      pwmRB = vel_subida - 60;
    } 
    else if (pitch > 0) {
      pwmLF = vel_subida - 90; //30
      pwmLB = vel_subida - 40;
      pwmRF = vel_subida - 90;
      pwmRB = vel_subida - 40;
    }
        else if (pitch > 25) {
      pwmLF = vel_subida - 120; //30
      pwmLB = vel_subida - 40;
      pwmRF = vel_subida - 120;
      pwmRB = vel_subida - 40;
    }

    // =========================
    // APLICAR PID
    // =========================
    pwmLF += (int)(correccionTotal);
    pwmLB += (int)(correccionTotal);
    pwmRF -= (int)(correccionTotal);
    pwmRB -= (int)(correccionTotal);

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
  saliendoDeRampa = true;

  avanzar_optimizado(3);
  //avanzarRectoConHeading(5, anguloObjetivo, BASE_SPEED - 25);

 
}
*/
// ===============================
// LOGICA DFS
// ===============================
#define UMBRAL_PARED 160.0   // mm (ajústalo según tu laberinto)

const int TOF_FRONT_IZQ = 2;
const int TOF_FRONT_CEN = 0;
const int TOF_FRONT_DER = 5;

const float UMBRAL_PARED_FRENTE  = 150.0;
const float UMBRAL_PARED_LATERAL = 160.0;

const int TOF_LAT_IZQ = 1;
const int TOF_LAT_DER = 4;

const int MUESTRAS_PARED = 3;
const int HITS_PARED = 2;

bool libreFrente() {
  bool paredCen = (Dist[TOF_FRONT_CEN] >= TOF_MIN_UTIL &&
                   Dist[TOF_FRONT_CEN] <= UMBRAL_PARED_FRENTE);

  bool paredIzq = (Dist[TOF_FRONT_IZQ] >= TOF_MIN_UTIL &&
                   Dist[TOF_FRONT_IZQ] <= UMBRAL_PARED_FRENTE);

  bool paredDer = (Dist[TOF_FRONT_DER] >= TOF_MIN_UTIL &&
                   Dist[TOF_FRONT_DER] <= UMBRAL_PARED_FRENTE);

  // Si el centro NO ve pared, el frente se considera libre.
  // S2 y S5 no pueden bloquear solos porque pueden ver paredes laterales.
  if (!paredCen) {
    return true;
  }

  // Si el centro sí ve pared, entonces sí usamos S2/S5 para confirmar.
  if (paredIzq || paredDer) {
    return false;
  }

  // Si solo S0 ve pared, probablemente es lectura aislada.
  return true;
}

bool libreIzquierda() {
  bool paredIzq = (Dist[TOF_LAT_IZQ] >= TOF_MIN_UTIL && Dist[TOF_LAT_IZQ] <= UMBRAL_PARED_LATERAL);
  return !paredIzq;
}

bool libreDerecha() {
  bool paredDer = (Dist[TOF_LAT_DER] >= TOF_MIN_UTIL && Dist[TOF_LAT_DER] <= UMBRAL_PARED_LATERAL);
  return !paredDer;
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
  headingObjetivo = objetivo;

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
  headingObjetivo = objetivo;

  display.clearDisplay();
  display.setCursor(5, 5);
  display.print("Giro: ");
  display.print(abs(girado), 1);
  display.println(" deg");
  display.setCursor(5, 25);
  display.print("Error: ");
  display.print(girado + 90.0, 1);
  display.display();

  delay(200); //200
}

int tirarNKits(int cantidad, char ladoCamara) {
  if (cantidad <= 0) {
    Serial.println("Victima de 0 kits, no se lanza nada");
    return 0;
  }

  if (ladoCamara != 'I' && ladoCamara != 'D') {
    Serial.println("Lado de camara invalido");
    return 0;
  }

  leerValor = false;
  valorPendiente = -1;

  int lanzados = 0;
  bool giro180 = false;

  Serial.print("Kits solicitados: ");
  Serial.print(cantidad);
  Serial.print(" | Camara: ");
  Serial.println(ladoCamara);

  for (int i = 0; i < cantidad; i++) {

    if (contadorKitsTotal >= MAX_KITS_TOTAL) {
      Serial.println("No quedan kits totales");
      break;
    }

    // ==================================================
    // CAMARA DERECHA -> TORRE DERECHA PRIMERO
    // ==================================================
    if (ladoCamara == 'D') {

      if (!giro180 && contadorKitsDer < MAX_KITS_POR_SERVO) {
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

        delayConArcoiris(250);
        continue;
      }

      if (contadorKitsIzq < MAX_KITS_POR_SERVO) {
        if (!giro180) {
          Serial.println("Derecha sin kits suficientes. Girando 180 para usar IZQUIERDA");

          giro90Izq();
          delayConArcoiris(500);

          giro90Izq();
          delayConArcoiris(150);

          giro180 = true;
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

        delayConArcoiris(250);
        continue;
      }

      Serial.println("No quedan kits en ninguna torre");
      break;
    }

    // ==================================================
    // CAMARA IZQUIERDA -> TORRE IZQUIERDA PRIMERO
    // ==================================================
    if (ladoCamara == 'I') {

      if (!giro180 && contadorKitsIzq < MAX_KITS_POR_SERVO) {
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

        delayConArcoiris(250);
        continue;
      }

      if (contadorKitsDer < MAX_KITS_POR_SERVO) {
        if (!giro180) {
          Serial.println("Izquierda sin kits suficientes. Girando 180 para usar DERECHA");

          giro90Izq();
          delayConArcoiris(500);

          giro90Izq();
          delayConArcoiris(150);

          giro180 = true;
        }

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

        delayConArcoiris(250);
        continue;
      }

      Serial.println("No quedan kits en ninguna torre");
      break;
    }
  }

  if (giro180) {
    Serial.println("Regresando 180 a orientacion original");

    giro90Izq();
    delayConArcoiris(500);

    giro90Izq();
    delayConArcoiris(150);
  }

  leerValor = false;
  valorPendiente = -1;


  Serial.print("Kits solicitados: ");
  Serial.print(cantidad);
  Serial.print(" | Kits lanzados: ");
  Serial.println(lanzados);

  return lanzados;
}

void encolarDeteccion(int kits, char lado) {
  if (kits < 0 || kits > 2) return;

  if (modoCalibracion) return;
  if (enLackOfProgress) return;
  if (enRampa) return;
  if (modoReposicion) return;
  if (bloquearOpenMV) return;

  unsigned long ahora = millis();

  if (lado == 'I') {
    if (ahora < cooldownIzq_hasta) return;

    colaPush(colaDetecciones, kits, 'I');
  } 
  else if (lado == 'D') {
    if (ahora < cooldownDer_hasta) return;

    colaPush(colaDetecciones, kits, 'D');
  } 
  else {
    return;
  }

  Serial.print("UART ");
  Serial.print(lado == 'I' ? "IZQ" : "DER");
  Serial.print(": encolada victima con ");
  Serial.print(kits);
  Serial.println(" kits");
}

void procesarContenidoTrama(const char *s, char ladoPorDefecto) {
  if (s == NULL || s[0] == '\0') return;

  const char *numStr = s;

  while (*numStr == ' ') numStr++;

  if (numStr[0] < '0' || numStr[0] > '2') {
    Serial.print("UART: kit invalido -> ");
    Serial.println(s);
    return;
  }

  const char *resto = numStr + 1;

  while (*resto == ' ') resto++;

  if (*resto != '\0') {
    Serial.print("UART: trama con basura -> ");
    Serial.println(s);
    return;
  }

  int kits = numStr[0] - '0';

  encolarDeteccion(kits, ladoPorDefecto);
}

void alimentarParser(char c, char *trama, int &idx, bool &enTrama, char ladoPorDefecto) {
  if (c == '<') {                
    enTrama = true; idx = 0; trama[0] = '\0';
    return;
  }
  if (c == '>') {                 
    if (enTrama) {
      trama[idx] = '\0';
      procesarContenidoTrama(trama, ladoPorDefecto);
    }
    enTrama = false; idx = 0; trama[0] = '\0';
    return;
  }
  if (c == '\n' || c == '\r') {   // fin de linea plana (modo compat)
    if (!enTrama && idx > 0) {
      trama[idx] = '\0';
      procesarContenidoTrama(trama, ladoPorDefecto);
    }
    if (!enTrama) { idx = 0; trama[0] = '\0'; }
    return;
  }
  // caracter normal -> acumular
  if (idx < 22) {
    trama[idx++] = c;
  } else {                        // overflow -> resetear parser
    idx = 0; enTrama = false; trama[0] = '\0';
  }
}

void leerUARTOpenMV(Stream &puerto, char *trama, int &idx, bool &enTrama, char lado) {
  while (puerto.available()) {
    char c = puerto.read();
    alimentarParser(c, trama, idx, enTrama, lado);
  }
}

void pollUARTOpenMV() {
  if (bloquearOpenMV) return;
  if (millis() < bloquearOpenMV_hasta) return;

  leerUARTOpenMV(OPENMV_DER_SERIAL, tramaDer, idxDer, enTramaDer, 'D');

  leerUARTOpenMV(OPENMV_IZQ_SERIAL, tramaIzq, idxIzq, enTramaIzq, 'I');
}

uint32_t colorWheel(uint8_t posicion) {
  posicion = 255 - posicion;
  if (posicion < 85) return pixels1.Color(255 - posicion * 3, 0, posicion * 3);
  if (posicion < 170) {
    posicion -= 85;
    return pixels1.Color(0, posicion * 3, 255 - posicion * 3);
  }
  posicion -= 170;
  return pixels1.Color(posicion * 3, 255 - posicion * 3, 0);
}

void apagarNeoPixels() {
  pixels1.clear();
  pixels1.show();
}

void iniciarArcoiris(unsigned long duracionMs = 4000) {
  arcoirisActivo       = true;
  arcoirisInicio       = millis();
  arcoirisDuracion     = duracionMs;
  arcoirisOffset       = 0;
  ultimoUpdateArcoiris = 0;

  actualizarArcoiris();  // prende inmediatamente
}

void actualizarArcoiris() {
  if (!arcoirisActivo) return;
  if (millis() - arcoirisInicio >= arcoirisDuracion) {
    arcoirisActivo = false;
    apagarNeoPixels();
    return;
  }
  if (millis() - ultimoUpdateArcoiris < 20) return;
  ultimoUpdateArcoiris = millis();
  for (int i = 0; i < NUM_PIXELS; i++) {
    int colorIndex = ((i * 256 / NUM_PIXELS) + arcoirisOffset) & 255;
    pixels1.setPixelColor(i, colorWheel(colorIndex));
  }
  pixels1.show();
  arcoirisOffset++;
}

bool atenderDeteccionOpenMV() {
  if (bloquearOpenMV) return false;
  if (enLackOfProgress) return false;
  if (enRampa) return false;
  if (modoReposicion) return false;
  if (modoCalibracion) return false;

  // Si no hay victimas en cola, no hacemos nada
  if (colaDetecciones.count == 0) {
    return false;
  }

  // Desde aqui no aceptamos mas lecturas UART
  bloquearOpenMV = true;

  bool atendioAlgo = false;
  DeteccionVictima det;

  // Atiende TODAS las detecciones que ya estaban en cola
  while (colaPop(colaDetecciones, det)) {

    atendioAlgo = true;  // aunque se descarte por distancia, ya atendimos una deteccion

    ladoDeteccion  = det.lado;
    valorPendiente = det.kits;
    leerValor      = false;

    stopMotors();
    delayConArcoiris(100);

    leerTOFS();

    float distPared = (ladoDeteccion == 'I') ? Dist[3] : Dist[4];

    // Si no hay pared valida en ese lado, descartamos deteccion
    if (distPared <= 20 || distPared > UMBRAL_PARED || distPared == 8190) {
      Serial.print("Victima descartada por distancia. Lado: ");
      Serial.print(ladoDeteccion);
      Serial.print(" | Dist: ");
      Serial.println(distPared);

      valorPendiente = -1;
      leerValor = false;

      continue;
    }

    if (valorPendiente < 0 || valorPendiente > 2) {
      Serial.println("Valor pendiente invalido");

      valorPendiente = -1;
      leerValor = false;

      continue;
    }

    display.clearDisplay();
    display.setCursor(5, 5);
    display.println("VICTIM");
    display.println("DETECTED");
    display.display();

    int kitsSolicitados = valorPendiente;

    Serial.print("Victima detectada | Lado: ");
    Serial.print(ladoDeteccion);
    Serial.print(" | Kits solicitados: ");
    Serial.println(kitsSolicitados);

    iniciarArcoiris(4000);

    int lanzados = 0;

    if (kitsSolicitados > 0) {
      lanzados = tirarNKits(kitsSolicitados, ladoDeteccion);

      // Si pidio kits pero ya no habia, dejamos indicador visual
      if (lanzados == 0) {
        stopMotors();
        delayConArcoiris(4000);
      }
    } 
    else {
      Serial.println("Victima de 0 kits, solo indicador visual");

      stopMotors();
      delayConArcoiris(4000);
    }

    Serial.print("Kits solicitados: ");
    Serial.print(kitsSolicitados);
    Serial.print(" | Kits lanzados: ");
    Serial.println(lanzados);

    // Cooldown por lado para no repetir la misma victima
    unsigned long hasta = millis() + COOLDOWN_VICTIMA_MS;

    if (ladoDeteccion == 'I') {
      cooldownIzq_hasta = hasta;
    } 
    else if (ladoDeteccion == 'D') {
      cooldownDer_hasta = hasta;
    }

    display.clearDisplay();
    display.display();

    valorPendiente = -1;
    leerValor = false;
  }

  // ===============================
  // LIMPIEZA FINAL
  // ===============================

  while (OPENMV_DER_SERIAL.available()) {
    OPENMV_DER_SERIAL.read();
  }

  while (OPENMV_IZQ_SERIAL.available()) {
    OPENMV_IZQ_SERIAL.read();
  }

  // Reiniciar parser UART
  idxDer = 0;
  idxIzq = 0;
  enTramaDer = false;
  enTramaIzq = false;

  // Asegurar que la cola quede vacia
  colaDetecciones.head = 0;
  colaDetecciones.tail = 0;
  colaDetecciones.count = 0;

  // Cooldown general para no volver a entrar por la misma victima
  bloquearOpenMV_hasta = millis() + COOLDOWN_VICTIMA_MS;

  // Ya puede volver a aceptar UART despues del cooldown
  bloquearOpenMV = false;

  return atendioAlgo;
}

bool servicioOpenMV() {
  // Si NO estamos bloqueados, leer UART
  if (!bloquearOpenMV) {
    pollUARTOpenMV();
  }

  actualizarArcoiris();

  if (millis() < bloquearOpenMV_hasta) return false;

  // Si hay detecciones pendientes, se atienden
  if (atenderDeteccionOpenMV()) {
    return true;
  }

  return false;
}

void delayConArcoiris(unsigned long tiempoMs) {
  unsigned long inicio = millis();
  while (millis() - inicio < tiempoMs) {
    actualizarArcoiris();
    delay(1);
  }
}

void accionarServo(Servo &servo, int anguloCerrado, int anguloEmpuje) {
  servo.write(anguloCerrado);
  delayConArcoiris(250);
  servo.write(anguloEmpuje);
  delayConArcoiris(1000);
}

void calibrar_limit() {
  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("Calibrating...");
  display.display();

  // --- CONFIGURACIÓN DE CONTROL ---
  int cal_vel = 150;            // Velocidad constante para calibración
  const float KP_CAL = 3.0;   // Fuerza de corrección (ajusta si serpentea)
  
  const unsigned long TIMEOUT_FASE1 = 5000; 
  const unsigned long TIMEOUT_FASE2 = 5000; 
  float promedio = 0;
  bool timeoutFase1 = false;
  bool timeoutFase2 = false;
  
  // Capturamos el ángulo actual para mantenerlo durante toda la maniobra
  stopMotors();
  delay(50); //100
  esperarEstable();
  float anguloObjetivo = readHeadingAvg(10); 

  // --- FASE 1: AVANCE RECTO HACIA LA PARED ---
  unsigned long tInicioFase1 = millis();
  while (true) {
    
    if (millis() - tInicioFase1 > TIMEOUT_FASE1) {
      Serial.println("Timeout FASE 1 calibrar_limit");
      timeoutFase1 = true;
      break;
    }
    float actual = readHeadingRaw(); 
    float error = angleDiff(anguloObjetivo, actual);
    float correccion = error * KP_CAL;

    // Aplicamos corrección al avance
    forwardRaw(cal_vel - (int)correccion, cal_vel + (int)correccion);

    // Salir cuando ambos limit switches detecten la pared
    if (limitIzqPresionado() && limitDerPresionado()) {
      break;
    }
    delay(5); 
  }

  stopMotors();
  if (timeoutFase1) {
    turnToHeading(anguloObjetivo);
    delay(80);
    esperarEstable();
    resincronizarHeading();
    return;
  }
  esperarEstable();
  float anguloChoque = readHeadingAvg(5);
  unsigned long tInicioFase2 = millis();

  while (true) {

    if (millis() - tInicioFase2 > TIMEOUT_FASE2) {
      Serial.println("Timeout FASE 2 calibrar_limit");
      timeoutFase2 = true;
      break;
    }
    float actual = readHeadingRaw();
    float error = angleDiff(anguloChoque, actual);
    float correccion = error * KP_CAL;

    backwardRaw(cal_vel + (int)correccion, cal_vel - (int)correccion);

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
    if (promedio >= 100.0) {
      break;
    }
  }

  stopMotors();
  
  // Ajuste final: asegurar que quedamos exactamente en el ángulo objetivo
  
  float finalHeading = readHeadingAvg(8);
  float errorFinal = angleDiff(anguloChoque, finalHeading);
  
  if (abs(errorFinal) > 1.5) {
    turnToHeading(anguloChoque);
    delay(80);
    esperarEstable();
  }

  headingObjetivo = anguloChoque;
  resincronizarHeading();

  // Mostrar resultado final en OLED
  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("OK! Dist:");
  display.setCursor(5, 25);
  display.print(promedio, 1);
  display.print(" mm");
  display.display();
  Serial.println("Calibration");
  delay(500);
}

void acomodarObstaculoPID(String lado, float distanciaAvanzada) { 
  const float DISTANCIA_TILE = 30.0;
  const float ANGULO_DESVIO_DER = 30.0;
  const float ANGULO_DESVIO_IZQ = 30.0;
  const float AVANCE_DIAGONAL_DER = 9.0;
  const float AVANCE_DIAGONAL_IZQ = 9.0;

  stopMotors();
  delay(100); //100
  esperarEstable();
  float anguloOriginal = readHeadingAvg(5);

  retroceder(4);
  delay(30);
  stopMotors();
  delay(30);
  esperarEstable();
  
  leerTOFS();

  const float UMBRAL_FRENTE = 120.0;
  if (Dist[0] == 0 || Dist[0] < UMBRAL_FRENTE) {
    Serial.print("Muy cerca al frente, cancelar evasión. Dist[0]: ");
    Serial.println(Dist[0]);
    return;  
  }

  float objetivoDesvio;
  float avanceDiagonal;
  float anguloDesvio;

  if (lado == "DER") {
    anguloDesvio = ANGULO_DESVIO_DER;
    objetivoDesvio = normalize360(anguloOriginal - anguloDesvio);
    avanceDiagonal = AVANCE_DIAGONAL_DER;
  } else {
    anguloDesvio = ANGULO_DESVIO_IZQ;
    objetivoDesvio = normalize360(anguloOriginal + anguloDesvio);
    avanceDiagonal = AVANCE_DIAGONAL_IZQ;
  }

  turnToHeading(objetivoDesvio);
  delay(80);
  avanzarRectoConHeading(avanceDiagonal, objetivoDesvio, BASE_SPEED - 15);
  stopMotors();
  delay(30);
  turnToHeading(anguloOriginal - 5);
  esperarEstable();
  delay(60);
 


  float avanceFrontalDiagonal = avanceDiagonal * cos(anguloDesvio * DEG_TO_RAD);
  float distanciaPendiente = DISTANCIA_TILE - avanceFrontalDiagonal;

  if (distanciaPendiente < 0) distanciaPendiente = 0;
  if (distanciaPendiente > DISTANCIA_TILE) distanciaPendiente = DISTANCIA_TILE;


  avanzarRectoConHeading(distanciaPendiente, anguloOriginal - 5, BASE_SPEED);
  
  stopMotors();
  delay(100); //100
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



/*float controlLateralContinuo() {

  const float KP_LAT = 1.0;
  const float MIN_PARED = 40.0;
  const float MAX_PARED = 200.0;
  const float UMBRAL = 3.0;

  float s1 = 1000;
  float s4 = 1100;

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
}*/
/*
float controlLateralContinuo() {

  const float KP_LAT      = 0.6;
  const float MIN_PARED   = 40.0;
  const float MAX_PARED   = 200.0;
  const float UMBRAL      = 5.0;
  const float DISTANCIA_OBJ = 120.0;

  // Lee Dist[] ya corregido en leerTOFS (con offsets aplicados)
  leerTOFS();

  float s1 = Dist[1];   // pared izquierda (ya con -17)
  float s4 = Dist[4];   // pared derecha

  bool paredIzq = (s1 > MIN_PARED && s1 < MAX_PARED);
  bool paredDer = (s4 > MIN_PARED && s4 < MAX_PARED);

  float correccion = 0;

  if (paredIzq && paredDer) {
    float diferencia = s4 - s1;
    if (abs(diferencia) > UMBRAL) {
      correccion = diferencia * KP_LAT;
    }
  }
  else if (paredIzq) {
    correccion = (DISTANCIA_OBJ - s1) * KP_LAT;
  }
  else if (paredDer) {
    correccion = (s4 - DISTANCIA_OBJ) * KP_LAT;
  }

  return correccion;
}*/

float controlLateralContinuo() {

  const float KP_LAT        = 0.85;   // parecido al viejo, pero no tan brusco como 1.0
  const float MIN_PARED     = 40.0;
  const float MAX_PARED     = 220.0;
  const float UMBRAL        = 4.0;
  const float DISTANCIA_OBJ = 120.0;
  const float TOF_LIBRE     = 8190.0;
  const float MAX_CORR_LAT  = 55.0;

  float s1 = TOF_LIBRE;   // izquierda
  float s4 = TOF_LIBRE;   // derecha

  VL53L0X_RangingMeasurementData_t measure;

  // =========================
  // Leer sensor lateral izquierdo S1
  // =========================
  if (sensorActivo[1]) {
    sensores[1].rangingTest(&measure, false);

    uint16_t lectura = measure.RangeMilliMeter;
    uint8_t status   = measure.RangeStatus;

    if (status != 4 && lectura > 12 && lectura < 1000) {
      s1 = lectura;

      // Offset del sensor izquierdo
      s1 -= 17;

      if (s1 < 0) s1 = 0;
    } else {
      s1 = TOF_LIBRE;
    }
  }

  delayMicroseconds(20);

  // =========================
  // Leer sensor lateral derecho S4
  // =========================
  if (sensorActivo[4]) {
    sensores[4].rangingTest(&measure, false);

    uint16_t lectura = measure.RangeMilliMeter;
    uint8_t status   = measure.RangeStatus;

    if (status != 4 && lectura > 12 && lectura < 1000) {
      s4 = lectura;

      // Si S4 tiene offset, ponlo aqui.
      // Por ahora lo dejamos sin correccion.
      // s4 += 0;

      if (s4 < 0) s4 = 0;
    } else {
      s4 = TOF_LIBRE;
    }
  }

  bool paredIzq = (s1 > MIN_PARED && s1 < MAX_PARED);
  bool paredDer = (s4 > MIN_PARED && s4 < MAX_PARED);

  float correccion = 0;

  // =========================
  // Caso 1: hay dos paredes
  // =========================
  if (paredIzq && paredDer) {
    float diferencia = s4 - s1;

    if (abs(diferencia) > UMBRAL) {
      correccion = diferencia * KP_LAT;
    }
  }

  // =========================
  // Caso 2: solo pared izquierda
  // =========================
  else if (paredIzq) {
    float error = DISTANCIA_OBJ - s1;

    if (abs(error) > UMBRAL) {
      correccion = error * KP_LAT;
    }
  }

  // =========================
  // Caso 3: solo pared derecha
  // =========================
  else if (paredDer) {
    float error = s4 - DISTANCIA_OBJ;

    if (abs(error) > UMBRAL) {
      correccion = error * KP_LAT;
    }
  }

  // =========================
  // Limite de seguridad
  // =========================
  correccion = constrain(correccion, -MAX_CORR_LAT, MAX_CORR_LAT);

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

    float kP = 1.5;  
    int correccion = error * kP;

    correccion = constrain(correccion, -60, 60);

    int velIzq = BASE_SPEED + correccion;
    int velDer = BASE_SPEED - correccion;

    velIzq = constrain(velIzq, 0, 255);
    velDer = constrain(velDer, 0, 255);

    backwardRaw(velIzq, velDer);

    delayMicroseconds(200);
  }

  stopMotors();
  delay(30);
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

long distanciaColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c, volatile int ref[5]) {
  long distRGB = abs((int)r - ref[0]) + abs((int)g - ref[1]) + abs((int)b - ref[2]);
  long distC = abs((int)c - ref[3]);
  return (distRGB * 1) + (distC * 3);
}          

String detectarColor() {
  uint16_t r, g, b, c;
  if (!apds.colorDataReady()) return "NONE";
  apds.getColorData(&r, &g, &b, &c);
  if (c == 0) return "NONE";
  
  if (r < (colorBlanco[0] * 0.2) && g < (colorBlanco[1] * 0.15) && b < (colorBlanco[2] * 0.15) && c < (colorBlanco[3] * 0.2)) {
    return "NEGRO";
  } else if (r < (colorBlanco[0] * 0.20) && g < (colorBlanco[1] * 0.45) && b < (colorBlanco[2] * 0.45) && c > (colorBlanco[3] * 0.20)) {
    return "AZUL";
  } else if (r < (colorBlanco[0] * 0.9) && g < (colorBlanco[1] * 0.3) && b > (colorBlanco[2] * 0.08) && c > (colorBlanco[3] * 0.2)) {
    return "ROJO";
  }

  long sumaP = 0;
  for (int i = 0; i < 5; i++) {
    sumaP += apds.readProximity();
    delay(2);
  }
  uint16_t pAvg = sumaP / 5;

  long distBlanco   = distanciaColor(r, g, b, c, colorBlanco);
  long distPlateado = distanciaMinPlateado(r, g, b, c);

  bool proxAnomalo = false;
  const int margenProx = 3;
  if (pAvg < (proxBlancoMin - margenProx) || pAvg > (proxBlancoMax + margenProx)) {
    proxAnomalo = true;
  }


  if (distPlateado < distBlanco && proxAnomalo) {
    if (ignorarPlateado) return "BLANCO";
    return "PLATEADO";
  }

  return "BLANCO";
}

void calibrarReferenciaColor(volatile int destino[5], const char* nombreSuperficie, bool usarRangoProx) {
  
  destino[0] = destino[1] = destino[2] = destino[3] = destino[4] = 0;
  const int muestras = 20;

  if (usarRangoProx) {
    proxBlancoMin = 999;
    proxBlancoMax = -999;
  }

  for (int i = 0; i < muestras; i++) {
    uint16_t r, g, b, c;
    while (!apds.colorDataReady()) delay(2);
    apds.getColorData(&r, &g, &b, &c);
    uint16_t p = apds.readProximity();

    destino[0] += r;
    destino[1] += g;
    destino[2] += b;
    destino[3] += c;
    destino[4] += p;

    if (usarRangoProx) {
      if (p < proxBlancoMin) proxBlancoMin = p;
      if (p > proxBlancoMax) proxBlancoMax = p;
    }
    delay(30);
  }

  for (int i = 0; i < 5; i++) destino[i] /= muestras;

  Serial.print(nombreSuperficie); Serial.print(" -> R:"); Serial.print(destino[0]);
  Serial.print(" G:"); Serial.print(destino[1]); Serial.print(" B:"); Serial.print(destino[2]);
  Serial.print(" C:"); Serial.print(destino[3]); Serial.print(" P:"); Serial.println(destino[4]);
  if (usarRangoProx) {
    Serial.print("PROX BLANCO MIN: "); Serial.println(proxBlancoMin);
    Serial.print("PROX BLANCO MAX: "); Serial.println(proxBlancoMax);
  }
}

void reaccionColor(String color, float distanciaRecorrida) {
  if (color == "NEGRO") {
    stopMotors();
    Serial.println("NEGRO -> retrocediendo");
    retroceder(distanciaRecorrida); 
    delay(200); 

    giro90Izq();
    delay(500); 
    esperarEstable();
    headingObjetivo = readHeadingAvg(8);

    giro90Izq();
    delay(500); 
    esperarEstable();
    headingObjetivo = readHeadingAvg(8);
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
    display.println("RED");
    display.display();
    return;
  }

  if (color == "PLATEADO") {
    return;
  }
}

long distanciaMinPlateado(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  long minDist = 999999;
  for (int i = 0; i < NUM_PLATEADOS; i++) {
    long d = distanciaColor(r, g, b, c, colorPlateado[i]);
    if (d < minDist) minDist = d;
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

  delay(100);

  // -------- INICIALIZAR SOLO S0 A S6 CON DIRECCIÓN FIJA --------
  for (int i = 0; i < NUM_TOF_FIJOS; i++) {

    digitalWrite(xshutPins[i], HIGH);
    delay(100);

    if (!sensores[i].begin(0x29)) {
      sensorActivo[i] = false;

      Serial.print("TOF ");
      Serial.print(i);
      Serial.println(" NO detectado");
    } else {
      sensores[i].setAddress(direcciones[i]);
      delay(50);

      sensorActivo[i] = true;

      Serial.print("TOF ");
      Serial.print(i);
      Serial.print(" OK -> 0x");
      Serial.println(direcciones[i], HEX);
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

  static bool palancaYaProcesada = false;

  bool palancaActiva = (digitalRead(PIN_REPOSICION) == LOW);

  // Si la palanca esta en medio, se rearma para la siguiente muestra
  if (!palancaActiva) {
    palancaYaProcesada = false;
    return;
  }

  // Si ya se proceso esta bajada de palanca, no repetir muestra
  if (palancaYaProcesada) {
    return;
  }

  // Debounce
  delay(80);

  if (digitalRead(PIN_REPOSICION) != LOW) {
    return;
  }

  // Marcamos que esta bajada ya fue usada
  palancaYaProcesada = true;

  // =========================
  // MUESTRA 0: BLANCO
  // =========================
  if (pasoCalibracion == 0) {
    mostrarOLED("CALIBRATING", "WHITE");
    Serial.println("Calibrating WHITE...");

    // Toma la muestra AHORITA, mientras la palanca esta activa
    calibrarReferenciaColor(colorBlanco, "BLANCO", true);

    pasoCalibracion = 1;

    mostrarOLED("SILVER", "SAMPLE 1");
    Serial.println("Coloca el robot en PLATEADO muestra 1, regresa la palanca al medio y vuelve a bajarla");

    return;
  }

  // =========================
  // MUESTRAS 1 A 4: PLATEADO
  // =========================
  if (pasoCalibracion >= 1 && pasoCalibracion <= 4) {
    int idx = pasoCalibracion - 1;

    mostrarOLED("CALIBRATING", "SILVER", String(idx + 1));

    Serial.print("Calibrando PLATEADO muestra ");
    Serial.println(idx + 1);

    calibrarReferenciaColor(colorPlateado[idx], "PLATEADO", false);

    pasoCalibracion++;

    if (pasoCalibracion <= 4) {
      mostrarOLED("SILVER", "SAMPLE " + String(pasoCalibracion));

      Serial.print("Coloca el robot en PLATEADO muestra ");
      Serial.print(pasoCalibracion);
      Serial.println(", regresa la palanca al medio y vuelve a bajarla");

      return;
    }

    // =========================
    // CALIBRACION COMPLETA
    // =========================
    modoCalibracion = false;

    mostrarOLED("READY :D", "RETURN MIDDLE");
    Serial.println("Calibracion completa.");
    Serial.println("Regresa la palanca al medio para iniciar mapa...");

    while (digitalRead(PIN_REPOSICION) == LOW) {
      stopMotors();
      delay(5);
    }

    delay(200);

    mostrarOLED("STARTING", "MAP");
    Serial.println("Palanca en medio. Iniciando mapa...");

    iniciarMapa();

    return;
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
  int hitFrente = 0;
  int hitIzq = 0;
  int hitDer = 0;

  for (int m = 0; m < MUESTRAS_PARED; m++) {
    leerTOFS();

    if (!libreFrente()) hitFrente++;
    if (!libreIzquierda()) hitIzq++;
    if (!libreDerecha()) hitDer++;

    delay(15);
  }

  bool frente = !(hitFrente >= HITS_PARED);
  bool izquierda = !(hitIzq >= HITS_PARED);
  bool derecha = !(hitDer >= HITS_PARED);

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
  Serial.println(Dist[0]);


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

  if (r == MOV_ABORTADO || r == MOV_CHOQUE) {
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
    mapa[nodoActual].vecinos[d].destino = -1;     
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

/*void explorarDFS_Dijkstra() {
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
    indicarFinal();
    while (true) { delay(1000); }
  }

  dijkstraNodos(nodoActual);
  vector<int> ruta = reconstruirRutaNodos(pendiente);
  seguirRutaNodos(ruta);
  
}*/
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
  delay(100);

  Serial.println("=== REINICIO DFS + BNO ===");

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println("Reset DFS");
  display.println("Reset BNO...");
  display.display();

  // ==================================================
  // 1. REINICIAR BNO COMO PARTE DEL REINICIO GENERAL
  // ==================================================
  resetBNO_Hardware(OPERATION_MODE_NDOF);

  delay(300);
  esperarEstable();

  // Reiniciar filtros internos del BNO
  pitchInit = false;

  float pitchTmp;
  leerPitchFiltrado(pitchTmp);

  // Tomar heading actual como nueva referencia limpia
  headingObjetivo = readHeadingAvg(12);
  headingFiltrado = headingObjetivo;

  Serial.print("Nuevo heading base despues de reset BNO: ");
  Serial.println(headingObjetivo);

  // ==================================================
  // 2. REINICIAR MAPA / DFS
  // ==================================================
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

  modoPostReinicio = true;

  iniciarMapa();

  // Volvemos a asegurar la referencia de heading ya con el mapa nuevo
  headingObjetivo = readHeadingAvg(12);
  headingFiltrado = headingObjetivo;

  Serial.print("Heading final DFS/BNO: ");
  Serial.println(headingObjetivo);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println("DFS RESET OK");
  display.print("H:");
  display.println(headingObjetivo, 1);
  display.display();

  delay(300);
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

  if (primeraDir == dirActual) return 0;                         
  if (primeraDir == girarDerechaDir(dirActual)) return 1;        
  if (primeraDir == girarIzquierdaDir(dirActual)) return 1;      
  if (primeraDir == opuestaDir(dirActual)) return 3;             

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

  OPENMV_IZQ_SERIAL.begin(BAUD_OPENMV);
  OPENMV_DER_SERIAL.begin(BAUD_OPENMV);

  Wire.begin();
  Wire.setClock(400000);

  setupMotor(L_F1, L_F2, L_F_PWM);
  setupMotor(L_B1, L_B2, L_B_PWM);
  setupMotor(R_F1, R_F2, R_F_PWM);
  setupMotor(R_B1, R_B2, R_B_PWM);
  
  pinMode(LLS, INPUT_PULLUP); 
  pinMode(RLS, INPUT_PULLUP);
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
  pixels1.begin();
  pixels1.setBrightness(100);
  pixels1.clear();
  pixels1.show();


  if (!apds.begin()) {
    Serial.println("Error APDS");
    while (1);
  }

  apds.enableColor(true);
  apds.enableProximity(true);
  apds.setADCGain(APDS9960_AGAIN_4X);

  mostrarOLED("PONER", "EN WHITE");
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
  delay(150); //150

  esperarEstable();   // tu estabilidad por heading
  delay(80);  
  float h1 = readHeadingAvg(8);
  delay(80);
  float h2 = readHeadingAvg(8);

  float drift = abs(angleDiff(h2, h1));
  
  if (drift < 2.0) {
    Serial.println("IMU estable -> resincronizando heading");
    headingObjetivo = readHeadingAvg(8);
    resincronizarHeading();
  } 

 
  //resetBNO_Hardware(OPERATION_MODE_NDOF);

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