// ============================================================
//  Navegacion.h 
//  Algoritmo DFS (exploración) + Dijkstra (regreso al origen)
//
//  USO EN TU .ino PRINCIPAL:
//    1. Coloca este archivo en la misma carpeta que tu .ino
//    2. Al inicio del .ino agrega:   #include "Navegacion.h"
//    3. En setup():                  navInit();
//    4. En loop():                   navLoop();
//
//  REQUISITO: Tu .ino debe tener definidas estas funciones:
//    bool libreFrente()
//    bool libreIzquierda()
//    bool libreDerecha()
//    void avanzar_optimizado(int cm)
//    void giro90Der()
//    void giro90Izq()
//    void giro180()
//    void leerTOFS()
//    void calibrar_atras()       <- opcional, se puede deshabilitar
//    void calibrar_lateral()     <- opcional, se puede deshabilitar
// ============================================================

#ifndef NAVEGACION_H
#define NAVEGACION_H

#include <Arduino.h>

// ============================================================
//  CONFIGURACION  (ajusta según tu laberinto)
// ============================================================
#define NAV_MAX_X       10        // Columnas máximas del mapa
#define NAV_MAX_Y       10        // Filas máximas del mapa
#define NAV_MAX_Z       3         // Pisos máximos
#define NAV_TILE_CM     30        // Tamaño de casilla en cm
#define NAV_MAX_TILES   (NAV_MAX_X * NAV_MAX_Y * NAV_MAX_Z)

// Descomenta la siguiente línea para deshabilitar calibraciones
// #define NAV_SIN_CALIBRACION

// ============================================================
//  DECLARACION DE FUNCIONES EXTERNAS
//  (definidas en tu .ino principal)
// ============================================================
extern bool libreFrente();
extern bool libreIzquierda();
extern bool libreDerecha();
extern void avanzar_optimizado(int cm);
extern void giro90Der();
extern void giro90Izq();
extern void giro180();
extern void leerTOFS();

#ifndef NAV_SIN_CALIBRACION
extern void calibrar_atras();
extern void calibrar_lateral();
#endif

// ============================================================
//  DIRECCIONES ABSOLUTAS
// ============================================================
#define NAV_NORTE  0
#define NAV_ESTE   1
#define NAV_SUR    2
#define NAV_OESTE  3

// Bits de pared (dentro del campo 'paredes' de cada Tile)
#define PARED_N  (1 << NAV_NORTE)   // 0x01
#define PARED_E  (1 << NAV_ESTE)    // 0x02
#define PARED_S  (1 << NAV_SUR)     // 0x04
#define PARED_O  (1 << NAV_OESTE)   // 0x08

// ============================================================
//  ESTRUCTURA DE CADA CASILLA DEL MAPA
// ============================================================
struct NavTile {
  bool     visitado;      // ¿Ya se exploró esta casilla?
  uint8_t  paredes;       // Bitmap de paredes (bits NESW)
  bool     victima;       // ¿Se detectó víctima aquí?
  bool     checkpoint;    // ¿Es casilla azul?
  bool     esRampa;       // ¿Hay rampa en esta casilla?
  // --- campos auxiliares para Dijkstra ---
  int      costoG;        // Costo acumulado desde origen
  int8_t   prevX;         // Predecesor en el camino óptimo
  int8_t   prevY;
  int8_t   prevZ;
};

// ============================================================
//  MAPA Y ESTADO DE NAVEGACION
// ============================================================
static NavTile _mapa[NAV_MAX_X][NAV_MAX_Y][NAV_MAX_Z];

static int _posX, _posY, _posZ;      // Posición actual del robot
static int _dir;                      // Dirección actual (0-3)
static int _origenX, _origenY, _origenZ;

// ---- Stack DFS ----
struct _NavNodo { int8_t x, y, z; };
static _NavNodo _stackDFS[NAV_MAX_TILES];
static int _topStack = -1;

static bool _exploracionCompleta = false;

// ---- Min-Heap para Dijkstra ----
struct _NodoDijk {
  int8_t x, y, z;
  int    costo;
};
static _NodoDijk _heap[NAV_MAX_TILES];
static int _heapSize = 0;

// ============================================================
//  UTILIDADES INTERNAS (prefijo _ = privado)
// ============================================================

static bool _enRango(int x, int y, int z) {
  return x >= 0 && x < NAV_MAX_X &&
         y >= 0 && y < NAV_MAX_Y &&
         z >= 0 && z < NAV_MAX_Z;
}

// Posición vecina en dirección d
static void _vecino(int x, int y, int z, int d, int &nx, int &ny, int &nz) {
  nx = x; ny = y; nz = z;
  if      (d == NAV_NORTE) ny++;
  else if (d == NAV_SUR)   ny--;
  else if (d == NAV_ESTE)  nx++;
  else if (d == NAV_OESTE) nx--;
}

// Dirección opuesta
static int _opuesto(int d) { return (d + 2) % 4; }

// Dirección relativa → dirección absoluta
static int _dirFrente()     { return _dir; }
static int _dirIzquierda()  { return (_dir + 3) % 4; }
static int _dirDerecha()    { return (_dir + 1) % 4; }
static int _dirAtras()      { return (_dir + 2) % 4; }

// ---- Min-Heap operaciones ----
static void _heapPush(_NodoDijk n) {
  _heap[_heapSize++] = n;
  int i = _heapSize - 1;
  while (i > 0) {
    int p = (i - 1) / 2;
    if (_heap[p].costo > _heap[i].costo) {
      _NodoDijk tmp = _heap[p]; _heap[p] = _heap[i]; _heap[i] = tmp;
      i = p;
    } else break;
  }
}

static _NodoDijk _heapPop() {
  _NodoDijk top = _heap[0];
  _heap[0] = _heap[--_heapSize];
  int i = 0;
  while (true) {
    int l = 2*i+1, r = 2*i+2, m = i;
    if (l < _heapSize && _heap[l].costo < _heap[m].costo) m = l;
    if (r < _heapSize && _heap[r].costo < _heap[m].costo) m = r;
    if (m == i) break;
    _NodoDijk tmp = _heap[i]; _heap[i] = _heap[m]; _heap[m] = tmp;
    i = m;
  }
  return top;
}

// ============================================================
//  GIRAR HACIA UNA DIRECCION ABSOLUTA
// ============================================================
static void _girarHacia(int dirObjetivo) {
  if (dirObjetivo == _dir) return;

  int diff = ((dirObjetivo - _dir) + 4) % 4;

  if (diff == 1) {           // 90° derecha
    giro90Der();
    _dir = (_dir + 1) % 4;
  } else if (diff == 3) {    // 90° izquierda
    giro90Izq();
    _dir = (_dir + 3) % 4;
  } else if (diff == 2) {    // 180°
    giro180();
    _dir = (_dir + 2) % 4;
  }
}

// ============================================================
//  LEER PAREDES DE LA CASILLA ACTUAL
//  Actualiza el mapa con lo que detectan los sensores
// ============================================================
static void _leerParedes() {
  leerTOFS();

  NavTile &t = _mapa[_posX][_posY][_posZ];

  // Array de [dirección relativa → función de sensor]
  // Usamos lambdas compactas como arreglo de punteros
  struct { int dir; bool libre; } lados[3] = {
    { _dirFrente(),    libreFrente()    },
    { _dirIzquierda(), libreIzquierda() },
    { _dirDerecha(),   libreDerecha()   }
  };

  for (int i = 0; i < 3; i++) {
    int d    = lados[i].dir;
    bool ok  = lados[i].libre;

    if (!ok) {
      // Marcar pared en casilla actual
      t.paredes |= (1 << d);

      // Marcar pared simétrica en el vecino (si existe)
      int nx, ny, nz;
      _vecino(_posX, _posY, _posZ, d, nx, ny, nz);
      if (_enRango(nx, ny, nz)) {
        _mapa[nx][ny][nz].paredes |= (1 << _opuesto(d));
      }
    }
  }
}

// ============================================================
//  MOVER EL ROBOT UNA CASILLA EN DIRECCION ABSOLUTA d
// ============================================================
static void _moverUna(int d) {
  _girarHacia(d);

#ifndef NAV_SIN_CALIBRACION
  calibrar_lateral();
#endif

  avanzar_optimizado(NAV_TILE_CM);

  // Actualizar posición
  int nx, ny, nz;
  _vecino(_posX, _posY, _posZ, d, nx, ny, nz);
  _posX = nx;
  _posY = ny;
  _posZ = nz;

#ifndef NAV_SIN_CALIBRACION
  calibrar_atras();
#endif
}

// ============================================================
//  DIJKSTRA: calcula camino más corto entre dos puntos del mapa
//  Retorna false si no existe camino
// ============================================================
static bool _dijkstra(int sx, int sy, int sz,
                      int tx, int ty, int tz) {
  // Inicializar costos
  for (int x = 0; x < NAV_MAX_X; x++)
    for (int y = 0; y < NAV_MAX_Y; y++)
      for (int z = 0; z < NAV_MAX_Z; z++) {
        _mapa[x][y][z].costoG = 32767;
        _mapa[x][y][z].prevX  = -1;
        _mapa[x][y][z].prevY  = -1;
        _mapa[x][y][z].prevZ  = -1;
      }

  _mapa[sx][sy][sz].costoG = 0;
  _heapSize = 0;
  _heapPush({(int8_t)sx, (int8_t)sy, (int8_t)sz, 0});

  while (_heapSize > 0) {
    _NodoDijk cur = _heapPop();
    int ax = cur.x, ay = cur.y, az = cur.z;

    if (ax == tx && ay == ty && az == tz) break;
    if (cur.costo > _mapa[ax][ay][az].costoG) continue;  // entrada obsoleta

    for (int d = 0; d < 4; d++) {
      // No cruzar pared
      if (_mapa[ax][ay][az].paredes & (1 << d)) continue;

      int nx, ny, nz;
      _vecino(ax, ay, az, d, nx, ny, nz);
      if (!_enRango(nx, ny, nz)) continue;
      if (!_mapa[nx][ny][nz].visitado) continue; // Solo tiles conocidos

      // Costo: 1 por casilla normal, +2 extra si hay rampa
      int costo = _mapa[ax][ay][az].costoG + 1 +
                  (_mapa[ax][ay][az].esRampa ? 2 : 0);

      if (costo < _mapa[nx][ny][nz].costoG) {
        _mapa[nx][ny][nz].costoG = costo;
        _mapa[nx][ny][nz].prevX  = (int8_t)ax;
        _mapa[nx][ny][nz].prevY  = (int8_t)ay;
        _mapa[nx][ny][nz].prevZ  = (int8_t)az;
        _heapPush({(int8_t)nx, (int8_t)ny, (int8_t)nz, costo});
      }
    }
  }

  return (_mapa[tx][ty][tz].prevX != -1 ||
          (tx == sx && ty == sy && tz == sz));
}

// ============================================================
//  EJECUTAR CAMINO DIJKSTRA: navega físicamente desde posición
//  actual hasta (tx, ty, tz)
// ============================================================
static bool _navegarHasta(int tx, int ty, int tz) {
  if (_posX == tx && _posY == ty && _posZ == tz) return true;

  bool ok = _dijkstra(_posX, _posY, _posZ, tx, ty, tz);
  if (!ok) {
    Serial.println("[NAV] Dijkstra: sin camino");
    return false;
  }

  // Reconstruir camino en buffer
  _NavNodo camino[NAV_MAX_TILES];
  int len = 0;
  int cx = tx, cy = ty, cz = tz;

  while (!(cx == _posX && cy == _posY && cz == _posZ)) {
    camino[len++] = {(int8_t)cx, (int8_t)cy, (int8_t)cz};
    int px = _mapa[cx][cy][cz].prevX;
    int py = _mapa[cx][cy][cz].prevY;
    int pz = _mapa[cx][cy][cz].prevZ;
    if (px == -1) { Serial.println("[NAV] Dijkstra: camino roto"); return false; }
    cx = px; cy = py; cz = pz;
  }

  // Invertir para tener orden origen→destino
  for (int i = 0; i < len / 2; i++) {
    _NavNodo tmp = camino[i];
    camino[i] = camino[len - 1 - i];
    camino[len - 1 - i] = tmp;
  }

  // Mover paso a paso
  for (int i = 0; i < len; i++) {
    int nx = camino[i].x;
    int ny = camino[i].y;
    int nz = camino[i].z;

    // Calcular dirección hacia la siguiente casilla
    int dx = nx - _posX;
    int dy = ny - _posY;

    int d;
    if      (dy > 0) d = NAV_NORTE;
    else if (dy < 0) d = NAV_SUR;
    else if (dx > 0) d = NAV_ESTE;
    else             d = NAV_OESTE;

    _moverUna(d);
  }

  return true;
}

// ============================================================
//  PUSH DE VECINOS NO VISITADOS AL STACK DFS
//  Prioridad: izquierda → frente → derecha → atrás
//  (se pushean en orden inverso para que izquierda salga primero)
// ============================================================
static void _pushVecinos() {
  // Orden inverso de prioridad (el último pusheado es el primero en salir)
  int orden[4] = {
    _dirAtras(),      // menor prioridad → push primero
    _dirDerecha(),
    _dirFrente(),
    _dirIzquierda()   // mayor prioridad → push último (sale primero)
  };

  for (int i = 0; i < 4; i++) {
    int d = orden[i];

    // ¿Hay pared en esa dirección?
    if (_mapa[_posX][_posY][_posZ].paredes & (1 << d)) continue;

    int nx, ny, nz;
    _vecino(_posX, _posY, _posZ, d, nx, ny, nz);
    if (!_enRango(nx, ny, nz))           continue;
    if (_mapa[nx][ny][nz].visitado)      continue;

    // Verificar que no esté ya en el stack (evitar duplicados)
    bool yaEnStack = false;
    for (int s = 0; s <= _topStack; s++) {
      if (_stackDFS[s].x == nx &&
          _stackDFS[s].y == ny &&
          _stackDFS[s].z == nz) {
        yaEnStack = true;
        break;
      }
    }
    if (yaEnStack) continue;

    if (_topStack < NAV_MAX_TILES - 1) {
      _topStack++;
      _stackDFS[_topStack] = {(int8_t)nx, (int8_t)ny, (int8_t)nz};
    }
  }
}

// ============================================================
//  UN PASO DFS
//  Llamar desde navLoop(). Retorna true si sigue explorando,
//  false si terminó y ya regresó al origen.
// ============================================================
static bool _navPaso() {
  // 1. Marcar casilla actual como visitada
  _mapa[_posX][_posY][_posZ].visitado = true;

  // 2. Leer paredes desde sensores
  _leerParedes();

  // 3. Empujar vecinos no visitados al stack
  _pushVecinos();

  // Debug por serial
  Serial.print("[NAV] Pos:(");
  Serial.print(_posX); Serial.print(",");
  Serial.print(_posY); Serial.print(",");
  Serial.print(_posZ); Serial.print(") Dir:");
  Serial.print(_dir);  Serial.print(" Stack:");
  Serial.println(_topStack + 1);

  // 4. Si el stack está vacío → exploración completa
  if (_topStack < 0) {
    Serial.println("[NAV] Exploración completa. Regresando al origen...");
    _exploracionCompleta = true;
    _navegarHasta(_origenX, _origenY, _origenZ);
    return false;   // fin
  }

  // 5. Obtener próxima casilla del stack (puede estar ya visitada)
  _NavNodo siguiente;
  do {
    if (_topStack < 0) {
      // Stack se vació buscando no visitado → terminar
      _exploracionCompleta = true;
      _navegarHasta(_origenX, _origenY, _origenZ);
      return false;
    }
    siguiente = _stackDFS[_topStack--];
  } while (_mapa[siguiente.x][siguiente.y][siguiente.z].visitado);

  int sx = siguiente.x;
  int sy = siguiente.y;
  int sz = siguiente.z;

  // 6. ¿Es casilla adyacente? → moverse directamente
  int dx = sx - _posX;
  int dy = sy - _posY;
  bool adyacente = (abs(dx) + abs(dy) == 1 && sz == _posZ);

  if (adyacente) {
    int d;
    if      (dy > 0) d = NAV_NORTE;
    else if (dy < 0) d = NAV_SUR;
    else if (dx > 0) d = NAV_ESTE;
    else             d = NAV_OESTE;

    // Verificar que no haya pared (puede haber sido descubierta después del push)
    if (!(_mapa[_posX][_posY][_posZ].paredes & (1 << d))) {
      _moverUna(d);
    } else {
      // Había pared → el vecino era inaccesible, intentar Dijkstra
      _navegarHasta(sx, sy, sz);
    }
  } else {
    // No adyacente → usar Dijkstra para llegar
    _navegarHasta(sx, sy, sz);
  }

  return true;   // Aún explorando
}

// ============================================================
//  API PÚBLICA
// ============================================================

// ---- navInit() ----
// Llama en setup() DESPUÉS de inicializar sensores y motores.
// Parámetros opcionales: posición de inicio en la grilla y
// dirección inicial del robot (por defecto centro del mapa, mirando al NORTE).
void navInit(int inicioX = NAV_MAX_X / 2,
             int inicioY = NAV_MAX_Y / 2,
             int inicioZ = 0,
             int dirInicial = NAV_NORTE) {

  memset(_mapa, 0, sizeof(_mapa));

  _posX = inicioX;
  _posY = inicioY;
  _posZ = inicioZ;
  _dir  = dirInicial;

  _origenX = inicioX;
  _origenY = inicioY;
  _origenZ = inicioZ;

  _topStack           = -1;
  _heapSize           = 0;
  _exploracionCompleta = false;

  // Poner posición inicial en el stack
  _topStack++;
  _stackDFS[_topStack] = {(int8_t)inicioX,
                           (int8_t)inicioY,
                           (int8_t)inicioZ};

  Serial.println("[NAV] Inicializado.");
  Serial.print("[NAV] Origen: (");
  Serial.print(inicioX); Serial.print(",");
  Serial.print(inicioY); Serial.print(",");
  Serial.print(inicioZ); Serial.println(")");
}

// ---- navLoop() ----
// Llama en loop(). Ejecuta un paso de DFS por iteración.
// Cuando la exploración termina, el robot regresa al origen
// automáticamente y la función ya no hace nada.
void navLoop() {
  if (!_exploracionCompleta) {
    _navPaso();
  }
}

// ---- Funciones de estado (útiles para el .ino principal) ----

// Posición actual del robot en la grilla
int navGetX()     { return _posX; }
int navGetY()     { return _posY; }
int navGetZ()     { return _posZ; }
int navGetDir()   { return _dir; }

// ¿Terminó la exploración?
bool navTerminado() { return _exploracionCompleta; }

// Marcar casilla actual como checkpoint o víctima desde el .ino principal
void navSetCheckpoint() { _mapa[_posX][_posY][_posZ].checkpoint = true; }
void navSetVictima()    { _mapa[_posX][_posY][_posZ].victima    = true; }
void navSetRampa()      { _mapa[_posX][_posY][_posZ].esRampa    = true; }

// Navegar manualmente a una casilla conocida (llama a Dijkstra directo)
bool navIrA(int x, int y, int z) {
  return _navegarHasta(x, y, z);
}

// Imprime el mapa 2D del piso z por Serial (para debug)
void navImprimirMapa(int z = 0) {
  Serial.println("[NAV] ---- MAPA ----");
  for (int y = NAV_MAX_Y - 1; y >= 0; y--) {
    for (int x = 0; x < NAV_MAX_X; x++) {
      if (x == _posX && y == _posY && z == _posZ)
        Serial.print("R");
      else if (_mapa[x][y][z].visitado)
        Serial.print(".");
      else
        Serial.print("?");
    }
    Serial.println();
  }
  Serial.println("[NAV] ------------------");
}

#endif // NAVEGACION_H