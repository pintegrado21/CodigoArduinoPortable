
//Arduino Nano 33 IoT
#include <SPI.h>
#include <WiFiNINA.h>
//Referencia: https://platformio.org/lib/show/798/ArduinoHttpClient/examples
#include <ArduinoHttpClient.h>
#include <Arduino.h>
#include "wiring_private.h"

#include <TinyGPS++.h>
TinyGPSPlus gps;


char ssid[] = "NEW-AGORA";      // Wifi SSID
char password[] = "6f78g512j716r";       // Wifi password 

int status = WL_IDLE_STATUS;

//Initialize the Wifi client
WiFiSSLClient client;
Uart mySerial (&sercom0, 5, 6, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// Sensor NO2
#define PRE_PIN          8
#define VNOX_PIN         A1
#define VRED_PIN         A2

//MiCS-4514 NO2 y CO2

String data() {
  //NO2 variables
  const float R0_NO2=8; //Valor dentro del límite (0.8-8)
  const float RL_NO2=0.82; //Valor mínimo recomendado datashet 
  int vnox_value = 0;
  float vnox_v;
  float RS_NO2;
  float ratio_no2;
  float no2ppb=0;
  float ugNO2=0;

  //CO variables
  int vred_value = 0;
  const float R0_CO=750; //Valor obtenido en  http://openaccess.uoc.edu/webapps/o2/bitstream/10609/42991/7/jgmansillaTFG0615memoria.pdf  (100-1000)
  const float RL_CO=100; //Valor que mejor se ajusta a CO
  float vred_v;
  float RS_CO;
  float ratio_co;
  float coppm=0;
  float mgCO=0;


  //Ozono variables.
  const int MQ_PIN = A0;      // Pin del sensor
  int valorA0=0;
  float  valorA0_v=0;
  const float R0_O3=40; //Valor obtenido con la función de calibración a 10ppb.
  const int RL_O3 = 10;    // Resistencia RL del modulo en Kilo ohms
  float RS_O3=0;
  float ratio=0;
  float ugO3=0;
  float o3ppb=0;
  float o3ppm=0;

  //Cálculos

  //Ozono
      valorA0= analogRead (MQ_PIN);
      valorA0_v = (float)(valorA0*5 )/1023;
      RS_O3=((5*RL_O3) /valorA0_v) - RL_O3;  //Obtener Rs
      ratio= RS_O3/R0_O3;
      o3ppb = 9.4783 * pow(ratio, 2.3348);  // Obtener la concentración.
      o3ppm=o3ppb/1000;
      ugO3= 1000*(o3ppm*47.99820)/(24.45); //en microgramos/m3;
      Serial.println(ugO3);

  //NO2
      vnox_value = analogRead(VNOX_PIN);
      vnox_v=(float)(vnox_value*5 )/1023; //Conversión a voltaje
      RS_NO2=((5*RL_NO2) /vnox_v) - RL_NO2;  //Obtener Rs
      ratio_no2= RS_NO2/R0_NO2;
      no2ppb = 12.429 * pow(ratio_no2, 0.5328);  // Obtener la concentración.
      ugNO2=no2ppb*46.0055/24.45;
      Serial.println(ugNO2);

  //CO
    vred_value = analogRead(VRED_PIN);
    vred_v=(float)(vred_value*5 )/1023; //Conversión a voltaje
    RS_CO=((5*RL_CO) /vred_v) - RL_CO;  //Obtener Rs
    ratio_co= RS_CO/R0_CO;
    coppm = 1.9366 * pow(ratio_co, -2.557);  // Obtener la concentración.
    mgCO=coppm*28.0101/(24.45);
    Serial.println(mgCO);
    String datos =  "O3=" + String(ugO3) + "&NO2=" + String(ugNO2)+ "&CO=" + String(mgCO);
    Serial.println(datos);
 return datos;

}
void SERCOM0_Handler()
{
    mySerial.IrqHandler();
}

//Conexión al aparato y a la wifi.
void setup() {
  Serial.begin(9600);
  pinPeripheral(5, PIO_SERCOM_ALT);
  pinPeripheral(6, PIO_SERCOM_ALT);
  mySerial.begin(9600);
// Attempt to connect to WiFi
while (WiFi.status() != WL_CONNECTED) {
  Serial.println();
  Serial.print("Connecting to WiFi network: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  //delay(1000);
   Serial.print(".");
  }
}

//Proceso que se repite cada 5 minutos
//Si la conexión wifi es correcta envía los datos a un Heroku que se debe encargar de guardar los datos en la B.de Datos.


void loop() {
  Serial.println("Entrando");
  String gpsdatos;

  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status
    Serial.println("Conectado a la Wifi");
    Serial.println("gps");
    Serial.println(gps.location.lat());
    Serial.println(gps.location.lng());
    Serial.println("************************");
    if(mySerial.available()){
      Serial.println("Conectado al GPS");
      gps.encode(mySerial.read());
      Serial.println(gps.encode(mySerial.read()));
    }
    if(gps.location.isUpdated()){
        Serial.println(gps.location.lat());
        double lat = gps.location.lat();
        double lng = gps.location.lng();
        gpsdatos =  "&LA=" + String(lat) + "&LO=" + String(lng);
         Serial.println(gpsdatos);
    }

   
    WiFiClient wifi;
    HttpClient http = HttpClient(wifi, "portableaire.herokuapp.com");  // Poner la app vuestra 
   int httpCode = http.get("/inserts/b?" + data()+ gpsdatos);   // poner la ruta vuestra 
   
   
    //Send the request
    Serial.println("Enviando Datos!");
    if (httpCode == 0) { //Check the returning code
      Serial.println("CORRECTO");
      Serial.println(httpCode);
      //String payload = http.getString();
      //Serial.println(payload);
    } else {
      Serial.println("Error:");
      Serial.println(httpCode);
    }
    http.stop(); //Desconexión
  }
  delay(300000); //Cada 5 minutos //
}
