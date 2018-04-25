
/*
sketch boitiers capteurs plantoid, 8 analog/digital in , 2 utrasonic sensors , 1 dht 11 , 1 ws2812b dediagnostic
   comportement de la led de diag : blue = boot , red = wifi connection error , green =connecteé au routeur
*/
//librairies
#include <ESP8266WiFi.h>                       //esp WiFi
#include <DNSServer.h>                         //serveur DNS
#include <ESP8266WebServer.h>                  //serveur WEB
#include <ESP8266mDNS.h>                       //DNS
#include <WiFiUdp.h>                           //serveur UDP
#include <WiFiManager.h>                       //WiFiManager
#include <OSCMessage.h>                        //gestion des signaux OSC
#include "NewPing.h"                           //Capteurs à ultrasons
#include "DHTesp.h"                            //Capteur dht11
#include "FastLED.h"                           //gestion de la ws2812b
#include <EEPROM.h>
//defines
#define NUM_LEDS       1                       // nombre de leds de diagnostique
#define LED_DATA_PIN  15                       // data pin des leds de diag.                                            wemos D8
#define TRIGGER_PIN_1  4                       // Arduino pin tied to trigger pin on the ultrasonic sensor1.            wemos D2
#define ECHO_PIN_1     5                       // Arduino pin tied to echo pin on the ultrasonic sensor1.               wemos D1
#define TRIGGER_PIN_2  2                       // Arduino pin tied to trigger pin on the ultrasonic sensor2.            wemos D4
#define ECHO_PIN_2     0                       // Arduino pin tied to echo pin on the ultrasonic sensor2.               wemos D3
#define pin4051_1     14                       // Select bus du 4051                                                    wemos D5
#define pin4051_2     12                       // Select bus du 4051                                                    wemos D6
#define pin4051_3     13                       // Select bus du 4051                                                    wemos D7
#define dhtPin        16                                                                  // Connect DHT sensor to                                                 wemos D0 
#define MAX_DISTANCE  200                                                                 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define aref_voltage  5                                                                   // tension analog_ref = 5 volts
ESP8266WebServer server(80);
NewPing sonar_1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);                                 // NewPing setup of pins and maximum distance.
NewPing sonar_2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);                                 // NewPing setup of pins and maximum distance.
CRGB leds[NUM_LEDS];                                                                      // Define the array of leds
//variables d'identification du neud de capteurs
char* plantoide = "2";                                                                    //numero de la plantoide
char* numeroBoitier = "1";                                                                //numero du boitier           
char* base = "plantoid/";                                                                 //base de l'adresse OSC
char oscAddr[80] = "";                                                                    // tableau contenant l'adresse OSC complette pour les adresse osc
char addr[80] = "";                                                                       // tableau contenant l'adresse OSC pour le serveur web        
WiFiUDP Udp;  
const IPAddress outIp(192,168,1,17);                                                      // remote IP of the receptor
const unsigned int outPort = 8000;                                                        // remote port to send OSC
const unsigned int localPort = 8888;                                                      // local port to listen for OSC packets (actually not used for sending)
DHTesp dht;
// variables d'état des lecture analog/digital 
float a1State = 0;
float a1PreviousState = 0;
float a2State = 0;
float a2PreviousState = 0;
float a3State = 0;
float a3PreviousState = 0;
float a4State = 0;
float a4PreviousState = 0;
float a5State = 0;
float a5PreviousState = 0;
float a6State = 0;
float a6PreviousState = 0;
float a7State = 0;
float a7PreviousState = 0;
float a8State = 0;
float a8PreviousState = 0;
float distance1, distance2;
float previousDistance1, previousDistance2;
float humidity = 0; 
float temperature = 0;
float previousTemperature = 0; 
float previousHumidity= 0;
//variables de calibration des capteurs analog/digital
int aMinimumTrueValue = 17;                                                                 //filtre les valeurs de moins de x, pour les lectures analogiques
int aReadDelay = 5;                                                                         //delais entre commutation du4051 et la lecture analogique
int aMinNoise = 60;                                                                         //bruit tolléré en mode analogique


//voids
void handleRoot() {
  sprintf(addr, "%s%s/%s/", base, plantoide, numeroBoitier);
//  digitalWrite(led, 1);
  String page = "";
   page +=  "<html><bosdy><H1>configuration du noeud plantoide : ";
   page += addr;
   page +=" </H1></body></html>";
  server.send(200,"text/plain", page);
 // digitalWrite(led, 0);
}

void handleNotFound(){
  //digitalWrite(led, 1);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  //digitalWrite(led, 0);
}


void setup() {
                                                                                            // initialisation de la led de diag.
   FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS);                     
   leds[0] = CRGB::Black;                                                                     
   FastLED.show();
                                                                                            //initialisation du port d'adresse du 4051
  pinMode(pin4051_1, OUTPUT); 
  pinMode(pin4051_2, OUTPUT); 
  pinMode(pin4051_3, OUTPUT); 
                                                                                             // initialisation du port serie
  Serial.begin(115200);
                                                                                             //message de courtoisie
  Serial.println();
  Serial.println();
  Serial.print("vous etes en communication avec le neud de capteur numero: ");
  Serial.print(numeroBoitier);
  Serial.print(" installé sur le plantoide: ");
  Serial.print(plantoide);
  Serial.println();
  Serial.println();
  Serial.print("Les messages OSC. seronts envoyés à la cible: ");
  Serial.print(outIp);
  Serial.println();
  Serial.println();
  dht.setup(dhtPin);        
  int P = plantoide[0];
  leds[0] = CRGB::Blue;                                                                    //led de diag en bleu = boot 
  FastLED.show();
  WiFiManager wifiManager;
//wifiManager.setSTAStaticIPConfig(IPAddress(10,0,int(plantoide[]),int(numeroBoitier[])), IPAddress(10,0,1,1), IPAddress(255,255,0,0));
//wifiManager.setSTAStaticIPConfig(IPAddress(192,168,P,1), IPAddress(192,168,1,255), IPAddress(255,255,0,0));                                                                   
  wifiManager.autoConnect("unconfigured_node");

Serial.println("Server started"); 
Serial.println(P);
  Serial.println("connecté au routeur avec succes");
  Serial.println("UDP OK");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());                                                            //led diag. verte = bien booté 
  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder OK");
  }
  
  server.on("/", handleRoot);

  server.on("/inline", [](){
    server.send(200, "text/plain", "this works as well");
  });

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("serveur HTTP OK");
  leds[0] = CRGB::Green;
  FastLED.show();
  delay(20);
}

void loop() {
    leds[0] = CRGB::Red;
  FastLED.show();
server.handleClient();
                                                                                               //cycle du sonar 1
distance1 = sonar_1.ping_cm();
if (distance1 != previousDistance1){
   sprintf(oscAddr, "%s%s/%s/sonar1/", base, plantoide, numeroBoitier);
   OSCMessage msg(oscAddr);
    msg.add(distance1);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    previousDistance1 = distance1;
} 
                                                                                                 //cycle du sonar 2
distance2 = sonar_2.ping_cm();
if (distance2 != previousDistance2){
   sprintf(oscAddr, "%s%s/%s/sonar2/", base, plantoide, numeroBoitier);
   OSCMessage msg(oscAddr);
    msg.add(distance2);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    previousDistance2 = distance2;
} 
                                                                                                  // cycle d'humidité
humidity = dht.getHumidity();
if (humidity != previousHumidity){
 sprintf(oscAddr, "%s%s/%s/hum/", base, plantoide, numeroBoitier);
   OSCMessage msg(oscAddr);
    msg.add(humidity);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    previousHumidity = humidity;
}


                                                                                                      // cycle analog in 1
digitalWrite(pin4051_1, HIGH);      
digitalWrite(pin4051_2, HIGH);      
digitalWrite(pin4051_3, LOW);   
delay(aReadDelay); 
a1State = analogRead(0);  
if(a1State < aMinimumTrueValue){
  a1State = 0;
} 
if ((a1State - a1PreviousState) > aMinNoise || ((a1PreviousState - a1State) > aMinNoise))
{
if (a1State != a1PreviousState){
 sprintf(oscAddr, "%s%s/%s/analog1/", base, plantoide, numeroBoitier);
   OSCMessage msg(oscAddr);
    msg.add(a1State);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    a1PreviousState = a1State;
}  
}



                                                                                                   // cycle analog in 2
digitalWrite(pin4051_1, LOW);      
digitalWrite(pin4051_2, LOW);      
digitalWrite(pin4051_3, LOW);   
delay(aReadDelay); //not sure if this delay is strictly necessary 
a2State = analogRead(0);  // read the input pin 


if(a2State < aMinimumTrueValue){
  a2State = 0;
} 
if ((a2State - a2PreviousState) > aMinNoise || ((a2PreviousState - a2State) > aMinNoise))
{
if (a2State != a2PreviousState){
 sprintf(oscAddr, "%s%s/%s/analog2/", base, plantoide, numeroBoitier);
   OSCMessage msg(oscAddr);
    msg.add(a2State);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    a2PreviousState = a2State;
}   
}
                                                                                                   // cycle analog in 3
digitalWrite(pin4051_1, HIGH);      
digitalWrite(pin4051_2, LOW);      
digitalWrite(pin4051_3, LOW);   
delay(aReadDelay); 
a3State = analogRead(0);  
if(a3State < aMinimumTrueValue){
  a3State = 0;
} 
if ((a3State - a3PreviousState) > aMinNoise || ((a3PreviousState - a3State) > aMinNoise))
{
if (a3State != a3PreviousState){
 sprintf(oscAddr, "%s%s/%s/analog3/", base, plantoide, numeroBoitier);
   OSCMessage msg(oscAddr);
    msg.add(a3State);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    a3PreviousState = a3State;
} 
} 
                                                                                                   // cycle analog in 4
digitalWrite(pin4051_1, LOW);      
digitalWrite(pin4051_2, HIGH);      
digitalWrite(pin4051_3, LOW);   
delay(aReadDelay); 
a4State = analogRead(0);  
if(a4State < aMinimumTrueValue){
  a4State = 0;
} 
if ((a4State - a4PreviousState) > aMinNoise || ((a4PreviousState - a4State) > aMinNoise))
{
if (a4State != a4PreviousState){
 sprintf(oscAddr, "%s%s/%s/analog4/", base, plantoide, numeroBoitier);
   OSCMessage msg(oscAddr);
    msg.add(a4State);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    a4PreviousState = a4State;
}  
}
                                                                                                     // cycle analog in 5
digitalWrite(pin4051_1, LOW);      
digitalWrite(pin4051_2, LOW);      
digitalWrite(pin4051_3, HIGH);   
delay(aReadDelay); 
a5State = analogRead(0);  
if(a5State < aMinimumTrueValue){
  a5State = 0;
} 
if ((a5State - a5PreviousState) > aMinNoise || ((a5PreviousState - a5State) > aMinNoise))
{
if (a5State != a5PreviousState){
 sprintf(oscAddr, "%s%s/%s/analog5/", base, plantoide, numeroBoitier);
   OSCMessage msg(oscAddr);
    msg.add(a5State);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    a5PreviousState = a5State;
}  
}
                                                                                                           // cycle analog in 6
digitalWrite(pin4051_1, LOW);      
digitalWrite(pin4051_2, HIGH);      
digitalWrite(pin4051_3, HIGH);   
delay(aReadDelay); 
a6State = analogRead(0); 
if(a6State < aMinimumTrueValue){
  a6State = 0;
} 
if ((a6State - a6PreviousState) > aMinNoise || ((a6PreviousState - a6State) > aMinNoise))
{
if (a6State != a6PreviousState){
 sprintf(oscAddr, "%s%s/%s/analog6/", base, plantoide, numeroBoitier);
   OSCMessage msg(oscAddr);
    msg.add(a6State);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    a6PreviousState = a6State;
}  
}
                                                                                                               // cycle analog in 7
digitalWrite(pin4051_1, HIGH);      
digitalWrite(pin4051_2, HIGH);      
digitalWrite(pin4051_3, HIGH);   
delay(aReadDelay);
a7State = analogRead(0);
if(a7State < aMinimumTrueValue){
  a7State = 0;
}   
if ((a7State - a7PreviousState) > aMinNoise || ((a7PreviousState - a7State) > aMinNoise))
{
if (a7State != a7PreviousState){
 sprintf(oscAddr, "%s%s/%s/analog7/", base, plantoide, numeroBoitier);
   OSCMessage msg(oscAddr);
    msg.add(a8State);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    a7PreviousState = a7State;
}  
}

                                                                                                        // cycle analog in 8
digitalWrite(pin4051_1, HIGH);      
digitalWrite(pin4051_2, LOW);      
digitalWrite(pin4051_3, HIGH);   
delay(aReadDelay); 
a6State = analogRead(0);   
if(a8State < aMinimumTrueValue){
  a8State = 0;
} 
if ((a8State - a8PreviousState) > aMinNoise || ((a8PreviousState - a8State) > aMinNoise))
{
if (a8State != a8PreviousState){
 sprintf(oscAddr, "%s%s/%s/analog8/", base, plantoide, numeroBoitier);
   OSCMessage msg(oscAddr);
    msg.add(a6State);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    a8PreviousState = a8State;
}  
}

                                                                                                                    // cycle de temperature
temperature = dht.getTemperature();
if (temperature != previousTemperature){
   sprintf(oscAddr, "%s%s/%s/temp/", base, plantoide, numeroBoitier);
   OSCMessage msg(oscAddr);
    msg.add(temperature);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    previousTemperature = temperature;
} 
  leds[0] = CRGB::Black;
  FastLED.show();
}


