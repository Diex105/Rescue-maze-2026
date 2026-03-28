volatile bool leerLetra = false;

const int pinVoltaje = A2;   // OpenMV P6 (DAC)
const int pinPulso   = 17;   // OpenMV P4 (pulso)

void ISR_pulso() {
  leerLetra = true;
}

// Promedio para estabilidad
float leerVoltajePromedio(int n = 8) {
  long suma = 0;

  for (int i = 0; i < n; i++) {
    suma += analogRead(pinVoltaje);
  }

  float raw = suma / (float)n;

  // 12 bits -> 0 a 4095
  return (raw * 3.3f) / 4095.0f;
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);  // MUY IMPORTANTE

  pinMode(pinVoltaje, INPUT);
  pinMode(pinPulso, INPUT_PULLDOWN);

  attachInterrupt(digitalPinToInterrupt(pinPulso), ISR_pulso, RISING);

  Serial.println("Sistema listo");
}

void loop() {
  if (leerLetra) {

    // sección crítica
    noInterrupts();
    leerLetra = false;
    interrupts();

    // pequeño delay para estabilizar lectura
    delayMicroseconds(300);

    float v = leerVoltajePromedio(8);

    Serial.print("Voltaje: ");
    Serial.println(v, 3);

    // ============================
    // DECODIFICACION
    // ============================
    if (v < 0.40f) {
      Serial.println("PHI");
      accionPhi();
    }
    else if (v > 0.90f && v < 1.80f) {
      Serial.println("PSI");
      accionPsi();
    }
    else if (v > 2.50f) {
      Serial.println("OMEGA");
      accionOmega();
    }
    else {
      Serial.println("Voltaje invalido");
    }
  }
}

// ============================
// ACCIONES
// ============================
void accionPhi() {
  Serial.println("-> accion PHI");
}

void accionPsi() {
  Serial.println("-> accion PSI");
}

void accionOmega() {
  Serial.println("-> accion OMEGA");
}