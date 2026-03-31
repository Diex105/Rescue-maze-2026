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
  MOV_ABORTADO
};

struct Conexion {
  int destino;           // id del nodo vecino
  bool existe;           // hay camino en esa dirección
  bool explorada;        // ya recorrí esa salida
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
int ultimoNodoPlateado = -1;
Direccion direccionEnUltimoPlateado = NORTE;

// Reposición / restore
bool switchReposicionActivo = false;
bool modoReposicion = false;
bool restauracionHecha = false;


// Plateado pendiente de salida
int nodoPlateadoPendiente = -1;
bool plateadoPendienteSalida = false;


// Snapshot del plateado
vector<Nodo> mapaSnapshotPlateado;
int nodoSnapshotPlateado = -1;
Direccion dirSnapshotPlateado = NORTE;
bool snapshotPlateadoValido = false;

// Reanudar exactamente saliendo del plateado
bool reanudarDesdePlateado = false;
Direccion direccionReanudacionPlateado = NORTE;

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
const int SERVO_IZQ_EMPUJE  = 70;

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
  const int VEL_MIN = 150;
  const float ANGULO_TOL = 0.6; //0.8
  const int SETTLE_MS = 180; //200
  const float FASE_FINE = 3.0;

  float integral = 0.0;
  float lastError = 0.0;
  unsigned long lastUpdate = micros();
  unsigned long inTolStart = 0;
  bool lastTurnLeft = true;

  while (true) {

    if (atenderDeteccionOpenMV()) {
      lastUpdate = micros();
      inTolStart = 0;
      continue;
    }
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

float clampFloat(float v, float minV, float maxV) {
  if (v < minV) return minV;
  if (v > maxV) return maxV;
  return v;
}

void leerPitchFiltrado(float &pitch) {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  float rawPitch = orientationData.orientation.z;

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
      delayMicroseconds(50);
    }
    if(i == 5){
      Dist[5]  = Dist[5]  - 28;
      delayMicroseconds(50);
    }
    if(i == 2){
      Dist[2]  = Dist[2]  + 8;
      delayMicroseconds(50);
    }

    delayMicroseconds(50);

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

      reaccionColor(color, distanciaRecorrida);
      if (color == "NEGRO") return MOV_NEGRO;
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
    
    bool L = digitalRead(LLS);
    bool R = digitalRead(RLS);
    
    if (R || L) {
      stopMotors();
      delay(50);

      long leftTicks2 = abs(encLeft.read());
      long rightTicks2 = abs(encRight.read());
      long promedio2 = (leftTicks2 + rightTicks2) / 2;
      float distanciaChoque = promedio2 / TICKS_POR_CM;

      Serial.println("Choque detectado");

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
  // Una vez que llega a la distancia, hacemos una corrección final de ángulo 
  // para asegurar que quedó perfectamente alineado a 0, 90, 180 o 270.
  turnToHeading(anguloObjetivo); 
  delay(30);


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

  digitalWrite(L_F1, LOW);  digitalWrite(L_F2, HIGH);
  digitalWrite(L_B1, HIGH); digitalWrite(L_B2, LOW);

  digitalWrite(R_F1, HIGH); digitalWrite(R_F2, LOW);
  digitalWrite(R_B1, LOW);  digitalWrite(R_B2, HIGH);

  const float KP = 2.2;  
  const float KD = 0.08;   
  const int MAX_CORRECCION = 25;
  const int TRIM_IZQ = -12;  // o -10
  const int TRIM_DER = 0;
  
  float lastErrorAng = 0;
  unsigned long lastTime = micros();
  int contadorPlano = 0;
  
  unsigned long ultimoConteoNodoRampa = millis();
  int modoRampa = 0;

  while (true) {
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    float pitch = orientationData.orientation.z;
    if (pitch <= -14.0) {
      modoRampa = 0;
    }
    else if (pitch >= 12.0) {
      modoRampa = 1;
    }

    unsigned long tiempoNodoActual = (modoRampa == 0) ? TIEMPO_SUBIDA : TIEMPO_BAJADA;

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
      correccionLat *= 0.3;
    }

    
    float correccionTotal = correccionAng + correccionLat;
    correccionTotal = constrain(correccionTotal, -35, 35);


    lastErrorAng = errorAng;
    lastTime = currentTime;

    int vel_subida = 130;

    int pwmLF = vel_subida;
    int pwmLB = vel_subida;
    int pwmRF = vel_subida;
    int pwmRB = vel_subida;

    // =========================
    // SUBIDA
    // =========================
    if (pitch <= -35) {
      pwmLF = vel_subida - 15; // -15
      pwmLB = vel_subida + 20; //  + 20
      pwmRF = vel_subida - 15; // -15
      pwmRB = vel_subida + 40; // +40
    }
    else if (pitch <= -20) {
      pwmLF = vel_subida - 30;
      pwmLB = vel_subida + 40;
      pwmRF = vel_subida - 30;
      pwmRB = vel_subida + 40;
    }

    else if (pitch < 0) {
      pwmLF = vel_subida;
      pwmLB = vel_subida;
      pwmRF = vel_subida;
      pwmRB = vel_subida;
    }


    // BAJADA

    if (pitch > 20) {
      pwmLF = vel_subida - 45; // frente
      pwmLB = vel_subida - 70; // atras
      pwmRF = vel_subida - 45;
      pwmRB = vel_subida - 70;
    }
    else if (pitch > 0) {
      pwmLF = vel_subida - 45;
      pwmLB = vel_subida - 45;
      pwmRF = vel_subida - 45;
      pwmRB = vel_subida - 45;
    }



    // =========================
    pwmLF = pwmLF + (int)correccionTotal;
    pwmLB = pwmLB + (int)correccionTotal;
    pwmRF = pwmRF - (int)correccionTotal;
    pwmRB = pwmRB - (int)correccionTotal;

    pwmLF += TRIM_IZQ;
    pwmLB += TRIM_IZQ;
    pwmRF += TRIM_DER;
    pwmRB += TRIM_DER;

    pwmLF = constrain(pwmLF, 70, 190);
    pwmLB = constrain(pwmLB, 70, 190);
    pwmRF = constrain(pwmRF, 70, 190);
    pwmRB = constrain(pwmRB, 70, 190);

    analogWrite(L_F_PWM, pwmLF);
    analogWrite(L_B_PWM, pwmLB);
    analogWrite(R_F_PWM, pwmRF);
    analogWrite(R_B_PWM, pwmRB);
    
    while (millis() - ultimoConteoNodoRampa >= tiempoNodoActual) {
      int idRampa = moverANodoEnDireccion(dirActual);

      debugEstado("NODO_RAMPA");

      ultimoConteoNodoRampa += tiempoNodoActual;
    }


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
  esperarEstable();
  delay(80);

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
  return (Dist[0] == 0 || Dist[0] > UMBRAL_PARED);
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

  for (int i = 0; i < cantidad; i++) {
    if (tirarUnKitDisponible()) {
      lanzados++;
      delay(250);
    } else {
      break;
    }
  }

  return lanzados;
}


void ISR_pulsoOpenMV() {
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
  if (!procesarSenalOpenMV()) return false;


  stopMotors();
  int kitsSolicitados = 0;

  if (letraPendiente == 'P') {
    Serial.println("Detenido por PHI");
    kitsSolicitados = 2;

    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 0, 255));   // azul
    }
    pixels.show();
  }
  else if (letraPendiente == 'S') {
    Serial.println("Detenido por PSI");
    kitsSolicitados = 1;
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 255, 0));   // verde
    }
    pixels.show();
  }
  else if (letraPendiente == 'O') {
    Serial.println("Detenido por OMEGA");
    kitsSolicitados = 0;
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i, pixels.Color(255, 0, 0));   // rojo
    }
    pixels.show();
  }

  // pausa total 5 segundos
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


  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(255, 255, 230));
  }
  pixels.show();
  letraPendiente = '\0';

  return true;
}


/*void decisionDFS() {

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
}*/ //no la usaré

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
  int cal_vel = 145;            // Velocidad constante para calibración
  const float KP_CAL = 2.0;   // Fuerza de corrección (ajusta si serpentea)
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



void acomodarObstaculoPID(String lado, float distanciaAvanzada) {
  const float DISTANCIA_TILE = 30.0;
  const float ANGULO_DESVIO_DER = 30.0;
  const float ANGULO_DESVIO_IZQ = 30.0;
  const float AVANCE_DIAGONAL_DER = 9.0;
  const float AVANCE_DIAGONAL_IZQ = 9.0;
  const float AVANCE_RECTO_POST_DIAGONAL = 2.0;

  float anguloOriginal = readHeadingAvg(3);

  stopMotors();
  delay(60);

  // 1) regresar exactamente lo que ya había avanzado
  retroceder(distanciaAvanzada);
  delay(60);

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
  delay(60);

  avanzarRectoConHeading(avanceDiagonal, objetivoDesvio, BASE_SPEED - 15);
  stopMotors();
  delay(80);
  esperarEstable();

  turnToHeading(anguloOriginal);
  delay(60);


  avanzarRectoConHeading(AVANCE_RECTO_POST_DIAGONAL, anguloOriginal, BASE_SPEED - 20);

  float avanceFrontalDiagonal = avanceDiagonal * cos(anguloDesvio * DEG_TO_RAD);

  float avanceYaGanado = avanceFrontalDiagonal + AVANCE_RECTO_POST_DIAGONAL;

  float distanciaPendiente = DISTANCIA_TILE - avanceYaGanado;

  if (distanciaPendiente < 0) distanciaPendiente = 0;
  if (distanciaPendiente > DISTANCIA_TILE) distanciaPendiente = DISTANCIA_TILE;


  avanzarRectoConHeading(distanciaPendiente, anguloOriginal, BASE_SPEED);

  calibrar_lateral();
}

/*void acomodarObstaculoPID(String lado, float distanciaAvanzada) {
  const float DISTANCIA_TILE = 30.0;
  const float RETROCESO_CM_LOCAL = 10.0;
  const float ANGULO_DESVIO_DER = 30.0;
  const float ANGULO_DESVIO_IZQ = 30.0;
  const float AVANCE_DIAGONAL_DER = 9.0;
  const float AVANCE_DIAGONAL_IZQ = 9.0;
  const float AVANCE_RECTO_POST_DIAGONAL = 2.0;

  float anguloOriginal = readHeadingAvg(3);

  stopMotors();
  delay(60);
  retroceder(distanciaAvanzada);

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
  delay(60);

  avanzarRectoConHeading(avanceDiagonal, objetivoDesvio, BASE_SPEED - 15);
  stopMotors();
  delay(80);
  esperarEstable();

  turnToHeading(anguloOriginal);
  delay(60);

  avanzarRectoConHeading(AVANCE_RECTO_POST_DIAGONAL, anguloOriginal, BASE_SPEED - 20);
  
  float avanceFrontalDiagonal = avanceDiagonal * cos(anguloDesvio * DEG_TO_RAD);
  
  float avanceYaGanado = avanceFrontalDiagonal + AVANCE_RECTO_POST_DIAGONAL;
  
  float distanciaPendiente = DISTANCIA_TILE - avanceYaGanado;
  
  if (distanciaPendiente < 0) distanciaPendiente = 0;
  if (distanciaPendiente > DISTANCIA_TILE) distanciaPendiente = DISTANCIA_TILE;

  avanzarRectoConHeading(distanciaPendiente, anguloOriginal, BASE_SPEED);

  calibrar_lateral();
}*/

void calibrar_lateral() {
  display.clearDisplay();
  display.setCursor(5, 5);
  display.println("Alineando...");
  display.display();

  const float KP_LAT = 0.7;
  const int VEL_BASE = 165;
  const float MIN_PARED = 40.0;
  const float MAX_PARED = 400.0;
  const float UMBRAL_CENTRADO = 5.0;

  unsigned long startTime = millis();
  const unsigned long TIMEOUT = 600;
  float anguloObjetivo = readHeadingAvg(5);

  while (true) {
    modoLateral = 0;
    if (millis() - startTime > TIMEOUT) break;

    float s1 = 1000, s3 = 1000, s4 = 1000, s6 = 1000;
    VL53L0X_RangingMeasurementData_t measure;

    if (sensorActivo[1]) {
      sensores[1].rangingTest(&measure, false);
      if (measure.RangeStatus != 4) s1 = measure.RangeMilliMeter;
    }
    if (sensorActivo[3]) {
      sensores[3].rangingTest(&measure, false);
      if (measure.RangeStatus != 4) s3 = measure.RangeMilliMeter;
    }
    if (sensorActivo[4]) {
      sensores[4].rangingTest(&measure, false);
      if (measure.RangeStatus != 4) s4 = measure.RangeMilliMeter;
    }
    if (sensorActivo[6]) {
      sensores[6].rangingTest(&measure, false);
      if (measure.RangeStatus != 4) s6 = measure.RangeMilliMeter;
    }

    float izquierda = (s1 + s3) / 2.0;
    float derecha = (s4 + s6) / 2.0;

    bool paredIzq = (izquierda > MIN_PARED && izquierda < MAX_PARED);
    bool paredDer = (derecha > MIN_PARED && derecha < MAX_PARED);

    if (paredIzq && paredDer) modoLateral = 1;
    else if (paredIzq && !paredDer) modoLateral = 2;
    else if (!paredIzq && paredDer) modoLateral = 3;

    float correccion = 0;

    if (modoLateral == 1) {
      float diferencia = derecha - izquierda;
      if (abs(diferencia) <= UMBRAL_CENTRADO) break;
      correccion = diferencia * KP_LAT;
    } else if (modoLateral == 2) {
      const float DIST_OBJ_IZQ = 118.0;
      float error = DIST_OBJ_IZQ - izquierda;
      correccion = error * KP_LAT;
    } else if (modoLateral == 3) {
      const float DIST_OBJ_DER = 128.0;
      float error = derecha - DIST_OBJ_DER;
      correccion = error * KP_LAT;
    }

    float actual = readHeadingRaw();
    float errorAng = angleDiff(anguloObjetivo, actual);
    float correccionAng = errorAng * 1.2;

    int velIzq = VEL_BASE + (int)correccion + (int)correccionAng;
    int velDer = VEL_BASE - (int)correccion - (int)correccionAng;

    velIzq = constrain(velIzq, 55, 220);
    velDer = constrain(velDer, 55, 220);

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

void esperarEstable() {
  float prev = readHeadingAvg(5);
  int estables = 0;
  while (estables < 4) {
    delay(25);
    float actual = readHeadingAvg(5);
    if (abs(angleDiff(actual, prev)) < 0.15) estables++;
    else estables = 0;
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

void accionarServo(Servo &servo, int anguloCerrado, int anguloEmpuje) {
  servo.write(anguloCerrado);
  delay(800);
  servo.write(anguloEmpuje);
  delay(800);
}


void retroceder(float distanciaCM) {

  long ticksObjetivo = distanciaCM * TICKS_POR_CM;

  encLeft.write(0);
  encRight.write(0);

  while (true) {

    long leftTicks = abs(encLeft.read());
    long rightTicks = abs(encRight.read());
    long promedio = (leftTicks + rightTicks) / 2;

    if (promedio >= ticksObjetivo) break;

  
    backwardRaw(BASE_SPEED, BASE_SPEED);

    delayMicroseconds(200);
  }

  stopMotors();
  delay(20);
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
    ignorarAzul = false;
  }

  if (color == "ROJO") {
    display.clearDisplay();
    display.setCursor(0,0);
    display.println("ROJO");
    display.display();
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

  mapa[nuevoId].vecinos[opuestaDir(d)].existe = true;
  mapa[nuevoId].vecinos[opuestaDir(d)].destino = nodoActual;
  mapa[nuevoId].vecinos[opuestaDir(d)].explorada = true;


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
    giro180();
  }

  dirActual = objetivo;
  debugEstado("DESPUES_GIRO");
}

ResultadoMovimiento avanzarANodo(Direccion d) {

  orientarRobotA(d);

  if (plateadoPendienteSalida && nodoActual == nodoPlateadoPendiente) {
    mapaSnapshotPlateado = mapa;
    nodoSnapshotPlateado = nodoActual;   // la casilla plateada
    dirSnapshotPlateado = d;             // direccion en la que voy a salir
    snapshotPlateadoValido = true;

    plateadoPendienteSalida = false;     // ya consumi la salida pendiente

    Serial.println("SNAPSHOT PLATEADO GUARDADO");
    Serial.print("Nodo snapshot: ");
    Serial.println(nodoSnapshotPlateado);
    Serial.print("Direccion snapshot: ");
    Serial.println((int)dirSnapshotPlateado);
  }

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

      ultimoNodoPlateado = idLlegada;
      nodoPlateadoPendiente = idLlegada;
      plateadoPendienteSalida = true;
    }
  }
  else if (r == MOV_AZUL) {
    int idLlegada = moverANodoEnDireccion(d);
    mapa[idLlegada].tieneAzul = true;

    if (ultimoTilePlateado) {
      mapa[idLlegada].tienePlateado = true;

      ultimoNodoPlateado = idLlegada;
      nodoPlateadoPendiente = idLlegada;
      plateadoPendienteSalida = true;
    }
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

bool obtenerDireccionNueva(Direccion &dElegida) {
  for (int i = 0; i < 4; i++) {
    Direccion d = PRIORIDAD_ABSOLUTA[i];

    if (mapa[nodoActual].vecinos[d].existe &&
        !mapa[nodoActual].vecinos[d].explorada) {
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




bool nodoPendiente(int idNodo) {
  for (int d = 0; d < 4; d++) {
    if (mapa[idNodo].vecinos[d].existe && !mapa[idNodo].vecinos[d].explorada) {
      return true;
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




int buscarNodoPendienteMasCercano() {
  dijkstraNodos(nodoActual);

  int mejorNodo = -1;
  int mejorDist = 2147483647;
  int mejorPenalizacion = 2147483647;

  for (int i = 0; i < (int)mapa.size(); i++) {
    if (!nodoPendiente(i)) continue;
    if (distancias[i] == 2147483647) continue;

    vector<int> ruta = reconstruirRutaNodos(i);
    int penal = penalizacionPrimerMovimiento(ruta);

    if (distancias[i] < mejorDist) {
      mejorDist = distancias[i];
      mejorPenalizacion = penal;
      mejorNodo = i;
    }
    else if (distancias[i] == mejorDist) {
      if (penal < mejorPenalizacion) {
        mejorPenalizacion = penal;
        mejorNodo = i;
      }
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
  
  if (enInicio() && exploracionCompleta()) {
    Serial.println("Regrese al inicio y exploracion completa");
    indicarFinal();
    while (true) { delay(1000); }
  }

  Direccion dNueva;
  if (obtenerDireccionNueva(dNueva)) {
    Serial.print("DECISION DFS -> ");

    avanzarANodo(dNueva);
    return;
  }
  Serial.print("DECISION DFS -> ");
  int pendiente = buscarNodoPendienteMasCercano();

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

  // Caso 1: ya estoy en inicio y ya no falta nada
  if (enInicio() && exploracionCompleta()) {
    Serial.println("Regrese al inicio y exploracion completa");
    indicarFinal();
    while (true) { delay(1000); }
  }

  Direccion dNueva;
  if (obtenerDireccionNueva(dNueva)) {
    Serial.print("DECISION DFS -> ");
    Serial.println(nombreDireccion(dNueva));
    avanzarANodo(dNueva);
    return;
  }

  int pendiente = buscarNodoPendienteMasCercano();

  // Caso 2: ya no hay nodos pendientes
  if (pendiente == -1) {
    Serial.println("Exploracion completa");

    // Si no estoy en el inicio, regreso al inicio
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
  }

  mapa.push_back(nuevo);
  return nuevo.id;
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

void restaurarDesdeUltimoPlateado() {
  if (!snapshotPlateadoValido) {
    Serial.println("No hay snapshot plateado valido");
    return;
  }

  mapa = mapaSnapshotPlateado;
  nodoActual = nodoSnapshotPlateado;   // volver al plateado
  dirActual = dirSnapshotPlateado;     // dejar la orientación guardada

  // IMPORTANTE:
  // no reanudar automáticamente
  reanudarDesdePlateado = false;
  direccionReanudacionPlateado = dirSnapshotPlateado;

  Serial.println("Estado restaurado desde plateado");
  Serial.print("Nodo restaurado: ");
  Serial.println(nodoActual);
  Serial.print("Direccion restaurada: ");
  Serial.println(nombreDireccion(dirActual));
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


  delayMicroseconds(50);
  Wire.begin();
  Wire.setClock(400000);
  setupMotor(L_F1, L_F2, L_F_PWM);
  setupMotor(L_B1, L_B2, L_B_PWM);
  setupMotor(R_F1, R_F2, R_F_PWM);
  setupMotor(R_B1, R_B2, R_B_PWM);
  pinMode(LLS, INPUT); //INicia los pines de los limit switch
  pinMode(RLS, INPUT);
  pinMode(PIN_REPOSICION, INPUT_PULLUP);
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
      if (snapshotPlateadoValido) {
        restaurarDesdeUltimoPlateado();
        Serial.println("Mapa restaurado al ultimo plateado");
      } else {
        Serial.println("No hay checkpoint plateado valido");
      }

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
    switchReposicionActivo = false;

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

    esperarEstable();
    delay(80);

    saliendoDeRampa = false;

    Serial.println("IMU estabilizada, continuar DFS");
    return;   // este loop solo estabiliza; decidir en la siguiente vuelta
  }


  if (enNodoListoParaDecidir() && !enRampa) {
    listoParaDecidir = false;
    explorarDFS_Dijkstra();
    listoParaDecidir = true;
  }
}