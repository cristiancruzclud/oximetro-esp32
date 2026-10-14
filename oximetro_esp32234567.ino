/* =============================================================================
   Oxímetro ESP32 + MAX30102 + TFT ST7789  —  v3
   BPM, SpO2 y onda de pulsaciones (pantalla + Serial Plotter de Arduino)
   =============================================================================

   CONEXIONES (placa de expansión)
   -----------------------------------------------------------
   PANTALLA (lado derecho)      PLACA
     SCL                        D18
     SDA                        D23
     RES                        D4
     DC                         D2
     CS                         D5
     VCC y BLK                  VCC (3.3V)
     GND                        GND
   -----------------------------------------------------------
   MAX30102 (lado izquierdo)    PLACA
     SDA                        D25
     SCL                        D26
     GND                        GND
     VIN                        3.3V
   -----------------------------------------------------------

   CÓMO VER LAS PULSACIONES EN ARDUINO IDE
     1. Sube el programa.
     2. Menú Herramientas > Serial Plotter (Ctrl+Shift+L).
     3. Velocidad: 115200 baudios.
     4. Verás 3 líneas:
          Pulso -> la onda de cada latido (escala 0 a 100)
          BPM   -> pulsaciones por minuto
          SpO2  -> oxígeno en sangre (%)
     Si prefieres ver texto, pon MODO_PLOTTER = false y abre el
     Monitor Serie (no uses los dos a la vez).

   LIBRERÍAS: Adafruit GFX, Adafruit ST7789, SparkFun MAX3010x
   ============================================================================= */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"


// =============================================================================
//                              CONFIGURACIÓN
// =============================================================================

// ---- Pantalla TFT (SPI por hardware: SCL = GPIO18, SDA = GPIO23) ----
constexpr int TFT_CS  = 5;
constexpr int TFT_DC  = 2;
constexpr int TFT_RST = 4;
constexpr int TFT_SCREEN_W = 240;
constexpr int TFT_SCREEN_H = 280;

// ---- Sensor MAX30102 (I2C en pines nuevos) ----
constexpr int  SDA_PIN        = 25;
constexpr int  SCL_PIN        = 26;
constexpr byte LED_BRIGHTNESS = 60;
constexpr byte SAMPLE_AVERAGE = 8;
constexpr byte LED_MODE       = 2;
constexpr byte SAMPLE_RATE    = 100;
constexpr int  PULSE_WIDTH    = 411;
constexpr int  ADC_RANGE      = 4096;

// ---- Buffers del algoritmo ----
constexpr int32_t BUFFER_LENGTH     = 100;
constexpr byte    SAMPLES_PER_CYCLE = 10;

// ---- Filtros de BPM ----
constexpr int   BPM_MIN      = 40;
constexpr int   BPM_MAX      = 180;
constexpr int   BPM_MAX_JUMP = 25;
constexpr float EMA_ALPHA_BPM         = 0.30f;
constexpr float EMA_ALPHA_BPM_INICIAL = 0.65f;

// ---- Filtros de SpO2 ----
constexpr int   SPO2_MIN      = 70;
constexpr int   SPO2_MAX      = 100;
constexpr int   SPO2_MAX_JUMP = 6;
constexpr float EMA_ALPHA_SPO2         = 0.25f;
constexpr float EMA_ALPHA_SPO2_INICIAL = 0.55f;

constexpr byte LECTURAS_CALIBRACION_RAPIDA = 4;

// ---- Detección de dedo (histéresis) ----
constexpr long IR_DEDO_PUESTO  = 55000;
constexpr long IR_DEDO_QUITADO = 45000;

// ---- Tiempos ----
constexpr unsigned long INTERVALO_PANTALLA = 800;
constexpr unsigned long TIMEOUT_SENSOR_MS  = 1200;

// ---- Gráfica en pantalla ----
constexpr int GRAPH_X      = 10;
constexpr int GRAPH_Y      = 160;
constexpr int GRAPH_WIDTH  = 220;
constexpr int GRAPH_HEIGHT = 90;
constexpr uint16_t GRAPH_BG = ST77XX_BLACK;
constexpr float GRAPH_SUAVIZADO    = 0.35f;
constexpr float ADAPT_DECAY        = 0.0015f;
constexpr float ADAPT_DECAY_RAPIDO = 0.02f;
constexpr int   MUESTRAS_CALIBRACION_RAPIDA = 120;

// ---- Salida por puerto serie ----
// true  = formato para Serial Plotter (onda + BPM + SpO2)
// false = texto legible para Monitor Serie
constexpr bool MODO_PLOTTER = true;


// =============================================================================
//                           OBJETOS Y ESTADO GLOBAL
// =============================================================================

// ---- Dispositivos ----
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
MAX30105 particleSensor;

// ---- Buffers y resultados del algoritmo ----
uint32_t irBuffer[BUFFER_LENGTH];
uint32_t redBuffer[BUFFER_LENGTH];

int32_t spo2;
int8_t  validSPO2;
int32_t heartRate;
int8_t  validHeartRate;

// ---- Valores mostrados y filtrados ----
int   lastBPM  = -1;
int   lastSPO2 = -1;
float bpmFiltrado  = -1;
float spo2Filtrado = -1;

// ---- Estado general ----
bool primeraVez = true;
bool dedoPuesto = false;
unsigned long ultimaActualizacionPantalla = 0;

byte lecturasBPMDesdeDedoPuesto  = 0;
byte lecturasSPO2DesdeDedoPuesto = 0;

// ---- Estado de la gráfica ----
int   graphBuffer[GRAPH_WIDTH];
int   graphIndex = 0;
float irSuavizado = 0;
bool  irSuavizadoInicializado = false;
float irMinAdapt = 30000;
float irMaxAdapt = 120000;
int   muestrasDesdeDedoPuesto = 0;


// ---- Prototipos ----
bool inicializarSensor();
bool leerNuevasMuestras();
void imprimirDebug();
void enviarAlPlotter(float onda);
void actualizarDeteccionDedo(long irValue);
void dibujarInterfaz();
void mostrarErrorSensor();
void mostrarResultados();
void mostrarEstadoDedo();
void actualizarGrafica(uint32_t irValueRaw);


// =============================================================================
//                                  SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);

  tft.init(TFT_SCREEN_W, TFT_SCREEN_H);
  tft.setRotation(0);
  tft.fillScreen(ST77XX_BLACK);
  dibujarInterfaz();

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!inicializarSensor()) {
    mostrarErrorSensor();
    while (true) { delay(1000); }
  }

  for (int i = 0; i < GRAPH_WIDTH; i++) {
    graphBuffer[i] = GRAPH_Y + GRAPH_HEIGHT / 2;
  }
}


// =============================================================================
//                                   LOOP
// =============================================================================

void loop() {
  if (!leerNuevasMuestras()) {
    mostrarErrorSensor();
    delay(500);
    return;
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_LENGTH, redBuffer,
                                         &spo2, &validSPO2, &heartRate, &validHeartRate);

  if (!MODO_PLOTTER) imprimirDebug();

  actualizarDeteccionDedo(irBuffer[BUFFER_LENGTH - 1]);

  unsigned long ahora = millis();
  if (ahora - ultimaActualizacionPantalla >= INTERVALO_PANTALLA) {
    ultimaActualizacionPantalla = ahora;
    mostrarResultados();
    mostrarEstadoDedo();
  }
}


// =============================================================================
//                                  SENSOR
// =============================================================================

bool inicializarSensor() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    return false;
  }
  particleSensor.setup(LED_BRIGHTNESS, SAMPLE_AVERAGE, LED_MODE, SAMPLE_RATE, PULSE_WIDTH, ADC_RANGE);
  return true;
}

bool leerNuevasMuestras() {
  auto esperarMuestra = [&]() -> bool {
    unsigned long inicioEspera = millis();
    while (particleSensor.available() == false) {
      particleSensor.check();
      if (millis() - inicioEspera > TIMEOUT_SENSOR_MS) return false;
    }
    return true;
  };

  if (primeraVez) {
    // Primera lectura: se llena el buffer completo
    for (byte i = 0; i < BUFFER_LENGTH; i++) {
      if (!esperarMuestra()) return false;
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i]  = particleSensor.getIR();
      particleSensor.nextSample();
      actualizarGrafica(irBuffer[i]);
    }
    primeraVez = false;
  } else {
    // Lecturas siguientes: se desplazan las muestras y se añaden las nuevas
    for (byte i = SAMPLES_PER_CYCLE; i < BUFFER_LENGTH; i++) {
      redBuffer[i - SAMPLES_PER_CYCLE] = redBuffer[i];
      irBuffer[i - SAMPLES_PER_CYCLE]  = irBuffer[i];
    }
    for (byte i = BUFFER_LENGTH - SAMPLES_PER_CYCLE; i < BUFFER_LENGTH; i++) {
      if (!esperarMuestra()) return false;
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i]  = particleSensor.getIR();
      particleSensor.nextSample();
      actualizarGrafica(irBuffer[i]);
    }
  }
  return true;
}

void actualizarDeteccionDedo(long irValue) {
  if (dedoPuesto && irValue < IR_DEDO_QUITADO) {
    // Se quitó el dedo: se reinicia todo
    dedoPuesto = false;
    lastBPM = -1;
    lastSPO2 = -1;
    bpmFiltrado = -1;
    spo2Filtrado = -1;
    lecturasBPMDesdeDedoPuesto = 0;
    lecturasSPO2DesdeDedoPuesto = 0;
    tft.fillRect(90, 60, 130, 30, ST77XX_BLACK);
    tft.fillRect(110, 110, 110, 30, ST77XX_BLACK);
  } else if (!dedoPuesto && irValue > IR_DEDO_PUESTO) {
    // Se puso el dedo: se recalibra la gráfica
    dedoPuesto = true;
    irMinAdapt = irValue - 2000;
    irMaxAdapt = irValue + 2000;
    muestrasDesdeDedoPuesto = 0;
    lecturasBPMDesdeDedoPuesto = 0;
    lecturasSPO2DesdeDedoPuesto = 0;
  }
}


// =============================================================================
//                              PUERTO SERIE
// =============================================================================

// Texto para el Monitor Serie (solo si MODO_PLOTTER = false)
void imprimirDebug() {
  Serial.print("IR=");        Serial.print(irBuffer[BUFFER_LENGTH - 1]);
  Serial.print(" BPM=");      Serial.print(heartRate);
  Serial.print(" validBPM="); Serial.print(validHeartRate);
  Serial.print(" SpO2=");     Serial.print(spo2);
  Serial.print(" validSpO2="); Serial.println(validSPO2);
}

// Formato "etiqueta:valor,etiqueta:valor" que entiende el Serial Plotter.
// Se envía una línea por cada muestra del sensor, así la onda se ve fluida.
void enviarAlPlotter(float onda) {
  if (!dedoPuesto) onda = 0;                        // sin dedo: línea plana
  int bpm = (dedoPuesto && lastBPM  > 0) ? lastBPM  : 0;
  int sp  = (dedoPuesto && lastSPO2 > 0) ? lastSPO2 : 0;

  Serial.print("Pulso:"); Serial.print(onda, 1);
  Serial.print(",BPM:");  Serial.print(bpm);
  Serial.print(",SpO2:"); Serial.println(sp);
}


// =============================================================================
//                           INTERFAZ (PANTALLA)
// =============================================================================

void dibujarInterfaz() {
  tft.fillScreen(ST77XX_BLACK);

  // Título
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 5);
  tft.println("MONITOR CARDIACO");

  tft.drawFastHLine(0, 30, tft.width(), ST77XX_WHITE);

  // Etiquetas
  tft.setTextSize(2);
  tft.setCursor(10, 60);
  tft.setTextColor(ST77XX_CYAN);
  tft.print("BPM:");

  tft.setCursor(10, 110);
  tft.setTextColor(ST77XX_GREEN);
  tft.print("SpO2:");

  // Marco de la gráfica
  tft.drawRect(GRAPH_X - 2, GRAPH_Y - 2, GRAPH_WIDTH + 4, GRAPH_HEIGHT + 4, ST77XX_WHITE);
}

void mostrarErrorSensor() {
  tft.fillRect(0, 0, tft.width(), 40, ST77XX_BLACK);
  tft.setCursor(10, 10);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(2);
  tft.println("Sensor no");
  tft.println("responde!");
  if (!MODO_PLOTTER) Serial.println("MAX30102: sin respuesta. Revisa cableado/alimentación.");
}

void mostrarEstadoDedo() {
  tft.fillRect(0, 140, tft.width(), 15, ST77XX_BLACK);
  if (!dedoPuesto) {
    tft.setCursor(10, 140);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_RED);
    tft.print("Coloca el dedo sobre el sensor");
  }
}

void mostrarResultados() {
  if (!dedoPuesto) return;

  // -------- BPM --------
  bool bpmFisiologico = (heartRate >= BPM_MIN && heartRate <= BPM_MAX);
  bool bpmEstable = (lastBPM == -1) || (abs(heartRate - lastBPM) <= BPM_MAX_JUMP);

  if (validHeartRate && bpmFisiologico && bpmEstable) {
    float alphaBPM = (lecturasBPMDesdeDedoPuesto < LECTURAS_CALIBRACION_RAPIDA)
                       ? EMA_ALPHA_BPM_INICIAL
                       : EMA_ALPHA_BPM;
    bpmFiltrado = (bpmFiltrado < 0)
                    ? heartRate
                    : (alphaBPM * heartRate + (1 - alphaBPM) * bpmFiltrado);
    if (lecturasBPMDesdeDedoPuesto < 255) lecturasBPMDesdeDedoPuesto++;

    int bpmMostrado = round(bpmFiltrado);
    if (bpmMostrado != lastBPM) {
      tft.fillRect(90, 60, 130, 30, ST77XX_BLACK);
      tft.setCursor(90, 60);
      tft.setTextSize(3);
      tft.setTextColor(ST77XX_CYAN);
      tft.print(bpmMostrado);
      lastBPM = bpmMostrado;
    }
  }

  // -------- SpO2 --------
  bool spo2Fisiologico = (spo2 >= SPO2_MIN && spo2 <= SPO2_MAX);
  bool spo2Estable = (lastSPO2 == -1) || (abs(spo2 - lastSPO2) <= SPO2_MAX_JUMP);

  if (validSPO2 && spo2Fisiologico && spo2Estable) {
    float alphaSPO2 = (lecturasSPO2DesdeDedoPuesto < LECTURAS_CALIBRACION_RAPIDA)
                        ? EMA_ALPHA_SPO2_INICIAL
                        : EMA_ALPHA_SPO2;
    spo2Filtrado = (spo2Filtrado < 0)
                     ? spo2
                     : (alphaSPO2 * spo2 + (1 - alphaSPO2) * spo2Filtrado);
    if (lecturasSPO2DesdeDedoPuesto < 255) lecturasSPO2DesdeDedoPuesto++;

    int spo2Mostrado = round(spo2Filtrado);
    if (spo2Mostrado != lastSPO2) {
      tft.fillRect(110, 110, 110, 30, ST77XX_BLACK);
      tft.setCursor(110, 110);
      tft.setTextSize(3);
      tft.setTextColor(ST77XX_GREEN);
      tft.print(spo2Mostrado);
      tft.print("%");
      lastSPO2 = spo2Mostrado;
    }
  }
}


// =============================================================================
//                                 GRÁFICA
// =============================================================================

void actualizarGrafica(uint32_t irValueRaw) {
  // ---- Suavizado de la señal IR ----
  if (!irSuavizadoInicializado) {
    irSuavizado = irValueRaw;
    irSuavizadoInicializado = true;
  } else {
    irSuavizado = GRAPH_SUAVIZADO * irValueRaw + (1 - GRAPH_SUAVIZADO) * irSuavizado;
  }

  // ---- Escala adaptativa (mínimo y máximo) ----
  float decayActual = (muestrasDesdeDedoPuesto < MUESTRAS_CALIBRACION_RAPIDA)
                        ? ADAPT_DECAY_RAPIDO
                        : ADAPT_DECAY;
  if (muestrasDesdeDedoPuesto < MUESTRAS_CALIBRACION_RAPIDA) muestrasDesdeDedoPuesto++;

  if (irSuavizado < irMinAdapt) irMinAdapt = irSuavizado;
  if (irSuavizado > irMaxAdapt) irMaxAdapt = irSuavizado;
  irMinAdapt += (irSuavizado - irMinAdapt) * decayActual;
  irMaxAdapt -= (irMaxAdapt - irSuavizado) * decayActual;
  if (irMaxAdapt - irMinAdapt < 1000) irMaxAdapt = irMinAdapt + 1000;

  // ---- Onda normalizada 0..100 para el Serial Plotter ----
  if (MODO_PLOTTER) {
    float onda = 100.0f * (irSuavizado - irMinAdapt) / (irMaxAdapt - irMinAdapt);
    onda = constrain(onda, 0.0f, 100.0f);
    enviarAlPlotter(onda);
  }

  // ---- Dibujo en pantalla ----
  int y = map((long)irSuavizado, (long)irMinAdapt, (long)irMaxAdapt,
              GRAPH_Y + GRAPH_HEIGHT, GRAPH_Y);
  y = constrain(y, GRAPH_Y, GRAPH_Y + GRAPH_HEIGHT);

  int prevIndex = (graphIndex == 0) ? GRAPH_WIDTH - 1 : graphIndex - 1;

  tft.drawFastVLine(GRAPH_X + graphIndex, GRAPH_Y, GRAPH_HEIGHT, GRAPH_BG);

  tft.drawLine(GRAPH_X + prevIndex, graphBuffer[prevIndex],
               GRAPH_X + graphIndex, y, ST77XX_YELLOW);

  graphBuffer[graphIndex] = y;
  graphIndex = (graphIndex + 1) % GRAPH_WIDTH;

  // Borra unas columnas por delante para el efecto de barrido
  int barridoIndex = (graphIndex + 2) % GRAPH_WIDTH;
  tft.drawFastVLine(GRAPH_X + barridoIndex, GRAPH_Y, GRAPH_HEIGHT, GRAPH_BG);
}
