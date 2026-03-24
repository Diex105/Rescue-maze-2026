#include <Wire.h>
#include <Adafruit_APDS9960.h>

Adafruit_APDS9960 apds;

void setup() {
  Serial.begin(115200);

  if (!apds.begin()) {
    Serial.println("Error inicializando APDS9960");
    while (1);
  }

  apds.enableColor(true);

  Serial.println("Lectura directa RGB + Clear (sin porcentajes)");
}

void loop() {
  uint16_t r, g, b, c;

  // Esperar datos listos
  while (!apds.colorDataReady()) {
    delay(5);
  }

  apds.getColorData(&r, &g, &b, &c);

  Serial.print("R: ");
  Serial.print(r);

  Serial.print("  G: ");
  Serial.print(g);

  Serial.print("  B: ");
  Serial.print(b);

  Serial.print("  C: ");
  Serial.println(c);

  delay(200);
}