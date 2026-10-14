/*

  OXIMETRO ESP32 + MAX30102 + ST7789 240x280 - v4

  Basado en el codigo proporcionado en este chat.

  CONEXIONES (ESP32 clasico, por ejemplo ESP32 Dev Module):

  TFT: SCL=18, SDA=23, RES=4, DC=2, CS=5, VCC/BLK=3.3V, GND=GND.

  MAX30102: SDA=25, SCL=26, VIN=3.3V, GND=GND.

  SDA de la TFT es MOSI de SPI; SDA del sensor pertenece a I2C.

  Instalar desde el gestor de bibliotecas de Arduino:

    Adafruit GFX Library, Adafruit ST7735 and ST7789 Library,

    SparkFun MAX3010x Pulse and Proximity Sensor Library.

  Aceptar sus dependencias (incluida Adafruit BusIO).

  Guardar en carpeta Oximetro_MAX30102_v4 y abrir este .ino.

  Seleccionar la placa ESP32 y su puerto. Serial: 115200 baudios.

  MODO_PLOTTER=true: Herramientas > Serial Plotter; false: Monitor Serie.

  En el plotter, BPM/SpO2=0 significa dato NO disponible.

  En pantalla se muestra -- cuando no hay una estimacion reciente valida.

  CAMBIOS:

  - 100 Hz / promedio 4 = 25 muestras/s, como requiere spo2_algorithm.

  - FIFO secuencial, sin esperar a llenar 100 muestras en un bucle bloqueante.

  - Ventana de 4 s; recalculo cada 10 muestras (0.4 s).

  - Deteccion del dedo en cada muestra; reinicio de la ventana al retirarlo.

  - Grafica PPG real normalizada (no es un electrocardiograma).

  - Valores caducan tras 3 s sin resultado valido; reconexion automatica.

  - Sin linea diagonal al volver al principio de la grafica.

  Mantener el dedo inmovil, apoyado suavemente y cubriendo ambos LED.

  El primer resultado requiere AL MENOS 4 s; una senal mala tarda mas.

  Los umbrales IR se conservan del montaje original; son ajustables.

  Prototipo educativo: no tiene calibracion ni validacion clinica.

  Referencia: SparkFun_MAX3010x_Sensor_Library/examples/Example8_SPO2.

*/

#include <Arduino.h>

#include <Wire.h>

#include <SPI.h>

#include <Adafruit_GFX.h>

#include <Adafruit_ST7789.h>

#include "MAX30105.h"

#include "spo2_algorithm.h"

#include <math.h>

#include <stdio.h>

#include <string.h>

constexpr int TFT_CS=5, TFT_DC=2, TFT_RST=4;

constexpr int SDA_PIN=25, SCL_PIN=26;

constexpr bool MODO_PLOTTER=true;

constexpr uint8_t LED_BRIGHTNESS=60;

constexpr int SAMPLE_RATE=100, SAMPLE_AVERAGE=4;

constexpr int N=100, PASO_CALCULO=10;

static_assert(FreqS == 25, "Esta version requiere el algoritmo SparkFun de 25 Hz");

static_assert(SAMPLE_RATE / SAMPLE_AVERAGE == FreqS, "Muestreo incompatible");

constexpr uint32_t IR_ENTRADA=55000, IR_SALIDA=45000;

constexpr uint32_t CADUCIDAD_MS=3000, TIMEOUT_MS=1200;

constexpr int GX=10, GY=164, GW=220, GH=88;

constexpr uint16_t FONDO=ST77XX_BLACK, REJILLA=0x18E3;

Adafruit_ST7789 tft(TFT_CS,TFT_DC,TFT_RST);

MAX30105 sensor;

uint32_t irRing[N], redRing[N], irTrabajo[N], redTrabajo[N];

int cabeza=0, cantidad=0, nuevas=0;

bool conectado=false, dedo=false, calculoPendiente=false;

uint32_t ultimaMuestra=0, ultimoIntento=0, ultimaUI=0, ultimoPoll=0;

uint32_t ultimoTexto=0, ultimoBPM=0, ultimoSPO2=0;

uint32_t irActual=0;

float bpm=-1, oxigeno=-1;

bool bpmValido=false, spo2Valido=false;

int xGrafica=0, yAnterior=GY+GH/2;

bool hayPunto=false, filtroListo=false;

float irSuave=0, minimoIR=0, maximoIR=0, ondaActual=0;

int bpmPintado=-999, spo2Pintado=-999;

char estadoPintado[40]="";

void reiniciarMedicion();

void dibujarBase();

void limpiarGrafica();

void dibujarColumna(int x);

void graficar(uint32_t ir);

void procesarMuestra(uint32_t rojo,uint32_t ir);

void calcular();

bool iniciarSensor();

void caducar(uint32_t ahora);

void actualizarUI();

void salidaSerial();

void setup() {

  Serial.begin(115200);

  SPI.begin(18,-1,23,TFT_CS);

  tft.init(240,280);

  tft.setRotation(0);

  tft.setTextWrap(false);

  dibujarBase();

  Wire.begin(SDA_PIN,SCL_PIN);

  Wire.setClock(400000);

  Wire.setTimeOut(25); // Limita la espera si el bus I2C falla.

  conectado=iniciarSensor();

  ultimoIntento=millis();

  actualizarUI();

}

bool iniciarSensor() {

  if (!sensor.begin(Wire,I2C_SPEED_FAST)) return false;

  sensor.setup(LED_BRIGHTNESS,SAMPLE_AVERAGE,2,SAMPLE_RATE,411,4096);

  while (sensor.available()) sensor.nextSample();

  sensor.clearFIFO();

  dedo=false;

  reiniciarMedicion();

  ultimaMuestra=ultimoPoll=millis();

  return true;

}

void reiniciarMedicion() {

  cabeza=cantidad=nuevas=0;

  calculoPendiente=false;

  bpm=oxigeno=-1;

  bpmValido=spo2Valido=false;

  ultimoBPM=ultimoSPO2=0;

  filtroListo=false;

  ondaActual=0;

  limpiarGrafica();

}

void loop() {

  uint32_t ahora=millis();

  if (!conectado) {

    if (ahora-ultimoIntento>=2000) {

      conectado=iniciarSensor();

      ultimoIntento=millis();

    }

  } else if (ahora-ultimoPoll>=2) {

    // SparkFun tiene un buffer software pequeno. Si hubo una pausa larga,

    // descartar la ventana para no calcular con muestras discontinuas.

    if (ahora-ultimoPoll>=100) {

      while (sensor.available()) sensor.nextSample();

      sensor.clearFIFO();

      dedo=false;

      reiniciarMedicion();

    }

    ultimoPoll=millis();

    uint16_t recibidas=sensor.check();

    if (recibidas>=4) {

      while (sensor.available()) sensor.nextSample();

      sensor.clearFIFO();

      dedo=false;

      reiniciarMedicion();

    } else {

      while (sensor.available()) {

        // Leer la misma posicion FIFO de los dos canales.

        uint32_t rojo=sensor.getFIFORed();

        uint32_t ir=sensor.getFIFOIR();

        sensor.nextSample();

        ultimaMuestra=millis();

        procesarMuestra(rojo,ir);

      }

    }

    if (millis()-ultimaMuestra>TIMEOUT_MS) {

      conectado=false;

      dedo=false;

      reiniciarMedicion();

      ultimoIntento=millis();

    }

  }

  if (conectado && calculoPendiente) calcular();

  ahora=millis();

  caducar(ahora);

  if (ahora-ultimaUI>=100) {

    ultimaUI=ahora;

    actualizarUI();

  }

  if (!MODO_PLOTTER && ahora-ultimoTexto>=500) {

    ultimoTexto=ahora;

    salidaSerial();

  }

  delay(1); // Cede CPU; no espera la siguiente muestra.

}

void procesarMuestra(uint32_t rojo,uint32_t ir) {

  irActual=ir;

  if (!dedo && ir>IR_ENTRADA) {

    dedo=true;

    reiniciarMedicion();

  } else if (dedo && ir<IR_SALIDA) {

    dedo=false;

    reiniciarMedicion();

  }

  if (dedo) {

    irRing[cabeza]=ir;

    redRing[cabeza]=rojo;

    cabeza=(cabeza+1)%N;

    if (cantidad<N) {

      cantidad++;

      if (cantidad==N) { calculoPendiente=true; nuevas=0; }

    } else if (++nuevas>=PASO_CALCULO) {

      nuevas=0;

      calculoPendiente=true;

    }

  }

  graficar(ir);

  caducar(millis());

  if (MODO_PLOTTER) salidaSerial();

}

void calcular() {

  calculoPendiente=false;

  if (!dedo || cantidad<N) return;

  uint32_t minIR=0xFFFFFFFFUL, maxIR=0, minRed=0xFFFFFFFFUL, maxRed=0;

  for (int i=0;i<N;i++) {

    int j=(cabeza+i)%N; // Orden cronologico de la ventana completa.

    irTrabajo[i]=irRing[j];

    redTrabajo[i]=redRing[j];

    if (irTrabajo[i]<minIR) minIR=irTrabajo[i];

    if (irTrabajo[i]>maxIR) maxIR=irTrabajo[i];

    if (redTrabajo[i]<minRed) minRed=redTrabajo[i];

    if (redTrabajo[i]>maxRed) maxRed=redTrabajo[i];

  }

  // Rechazo basico de saturacion/canal casi plano; NO mide calidad clinica.

  if (maxIR>=260000 || maxRed>=260000 || maxIR-minIR<50 || maxRed-minRed<50) return;

  int32_t hr=0, sp=0;

  int8_t okHR=0, okSP=0;

  maxim_heart_rate_and_oxygen_saturation(irTrabajo,N,redTrabajo,&sp,&okSP,&hr,&okHR);

  uint32_t ahora=millis();

  caducar(ahora);

  // EMA sin bloqueo permanente por comparar con un valor antiguo.

  // Rangos de visualizacion del prototipo; no equivalen a diagnostico.

  if (okHR && hr>=40 && hr<=180) {

    bpm=bpmValido ? 0.45f*hr+0.55f*bpm : float(hr);

    bpmValido=true;

    ultimoBPM=ahora;

  }

  if (okSP && sp>=70 && sp<=100) {

    oxigeno=spo2Valido ? 0.35f*sp+0.65f*oxigeno : float(sp);

    spo2Valido=true;

    ultimoSPO2=ahora;

  }

}

void caducar(uint32_t ahora) {

  if (!dedo || ahora-ultimoBPM>CADUCIDAD_MS) { bpmValido=false; bpm=-1; }

  if (!dedo || ahora-ultimoSPO2>CADUCIDAD_MS) { spo2Valido=false; oxigeno=-1; }

}

void dibujarBase() {

  tft.fillScreen(FONDO);

  tft.setTextColor(ST77XX_WHITE);

  tft.setTextSize(2);

  tft.setCursor(12,8); tft.print("OXIMETRO ESP32");

  tft.drawFastHLine(10,32,220,REJILLA);

  tft.setTextColor(ST77XX_CYAN);

  tft.setCursor(12,45); tft.print("BPM");

  tft.setTextColor(ST77XX_GREEN);

  tft.setCursor(133,45); tft.print("SpO2 %");

  tft.setTextSize(1);

  tft.setTextColor(ST77XX_WHITE);

  tft.setCursor(10,151); tft.print("ONDA DE PULSO (PPG)");

  tft.drawRect(GX-1,GY-1,GW+2,GH+2,ST77XX_WHITE);

  tft.setTextColor(0x8410);

  tft.setCursor(10,264); tft.print("MAX30102 | Prototipo educativo");

  limpiarGrafica();

}

void dibujarColumna(int x) {

  tft.drawFastVLine(GX+x,GY,GH,(x%25==0)?REJILLA:FONDO);

  for (int y=22;y<GH;y+=22) tft.drawPixel(GX+x,GY+y,REJILLA);

}

void limpiarGrafica() {

  tft.fillRect(GX,GY,GW,GH,FONDO);

  for (int x=0;x<GW;x+=25) tft.drawFastVLine(GX+x,GY,GH,REJILLA);

  for (int y=22;y<GH;y+=22) tft.drawFastHLine(GX,GY+y,GW,REJILLA);

  xGrafica=0; yAnterior=GY+GH/2; hayPunto=false;

}

void graficar(uint32_t ir) {

  float normalizada=0.5f;

  if (dedo) {

    if (!filtroListo) {

      irSuave=float(ir); minimoIR=irSuave-500; maximoIR=irSuave+500;

      filtroListo=true;

    } else {

      irSuave+=0.6f*(float(ir)-irSuave);

    }

    minimoIR+=0.015f*(irSuave-minimoIR);

    maximoIR+=0.015f*(irSuave-maximoIR);

    if (irSuave<minimoIR) minimoIR=irSuave;

    if (irSuave>maximoIR) maximoIR=irSuave;

    float amplitud=fmaxf(maximoIR-minimoIR,200.0f);

    float centro=(minimoIR+maximoIR)*0.5f;

    normalizada=constrain(0.5f+(irSuave-centro)/amplitud,0.0f,1.0f);

  }

  ondaActual=dedo ? normalizada*100.0f : 0.0f;

  int y=GY+GH-3-int(normalizada*(GH-6));

  dibujarColumna(xGrafica);

  // Limpiar dos columnas por delante da un barrido visible.

  dibujarColumna((xGrafica+1)%GW);

  dibujarColumna((xGrafica+2)%GW);

  uint16_t color=dedo?ST77XX_YELLOW:REJILLA;

  if (hayPunto && xGrafica>0)

    tft.drawLine(GX+xGrafica-1,yAnterior,GX+xGrafica,y,color);

  else tft.drawPixel(GX+xGrafica,y,color);

  yAnterior=y; hayPunto=true;

  xGrafica=(xGrafica+1)%GW;

}

void actualizarUI() {

  int b=bpmValido?int(lroundf(bpm)):-1;

  int s=spo2Valido?int(lroundf(oxigeno)):-1;

  char texto[12];

  tft.setTextSize(4);

  if (b!=bpmPintado) {

    if (b<0) strcpy(texto,"-- "); else snprintf(texto,sizeof(texto),"%-3d",b);

    tft.setTextColor(ST77XX_CYAN,FONDO);

    tft.setCursor(12,73); tft.print(texto); bpmPintado=b;

  }

  if (s!=spo2Pintado) {

    if (s<0) strcpy(texto,"-- "); else snprintf(texto,sizeof(texto),"%-3d",s);

    tft.setTextColor(ST77XX_GREEN,FONDO);

    tft.setCursor(133,73); tft.print(texto); spo2Pintado=s;

  }

  char estado[40];

  uint16_t color=ST77XX_WHITE;

  if (!conectado) { strcpy(estado,"Sensor sin respuesta. Reintentando"); color=ST77XX_RED; }

  else if (!dedo) strcpy(estado,"Coloca el dedo sobre el sensor");

  else if (cantidad<N) snprintf(estado,sizeof(estado),"Adquiriendo senal: %d%%",cantidad);

  else if (!bpmValido || !spo2Valido) { strcpy(estado,"Ajusta el dedo y mantenlo quieto"); color=ST77XX_YELLOW; }

  else strcpy(estado,"Leyendo. Manten el dedo quieto");

  if (strcmp(estado,estadoPintado)!=0) {

    tft.fillRect(10,119,220,17,FONDO);

    tft.setTextSize(1); tft.setTextColor(color);

    tft.setCursor(10,123); tft.print(estado);

    strcpy(estadoPintado,estado);

  }

}

void salidaSerial() {

  char linea[120];

  int b=bpmValido?int(lroundf(bpm)):0;

  int s=spo2Valido?int(lroundf(oxigeno)):0;

  int n;

  if (MODO_PLOTTER) {

    n=snprintf(linea,sizeof(linea),"Pulso:%.1f,BPM:%d,SpO2:%d\n",ondaActual,b,s);

  } else {

    n=snprintf(linea,sizeof(linea),"IR:%lu Dedo:%d BPM:%d SpO2:%d Validos:%d/%d Sensor:%d\n",

       (unsigned long)irActual,dedo,b,s,bpmValido,spo2Valido,conectado);

  }

  // Si el puerto esta ocupado, omitir una linea antes que frenar el sensor.

  if (n>0 && n<int(sizeof(linea)) && Serial.availableForWrite()>=n)

    Serial.write((const uint8_t*)linea,n);

}
