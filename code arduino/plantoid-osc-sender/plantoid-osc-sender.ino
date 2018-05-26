
/*
sketch boitiers capteurs plantoid, 8 analog/digital in , 2 utrasonic sensors , 1 dht 11 , 1 ws2812b dediagnostic
   comportement de la led de diag : blue = boot , red = wifi connection error , green = connecteé au routeur, black = fonctionnement normal
*/
//librairies
#include <ESP8266WiFi.h>                                                                  // librairie esp WiFi
#include <DNSServer.h>                                                                    // serveur DNS
#include <ESP8266WebServer.h>                                                             // serveur WEB
#include <ESP8266mDNS.h>                                                                  // service mDNS
#include <WiFiUdp.h>                                                                      // serveur UDP
#include <WiFiManager.h>                                                                  // librairie WiFiManager
#include <OSCMessage.h>                                                                   // librairie de gestion des signaux OSC
#include "NewPing.h"                                                                      // librairie de gestion des capteurs à ultrasons
#include "DHTesp.h"                                                                       // librairie de gestion du capteur dht11
#include "FastLED.h"                                                                      // librairie de gestion des ws2812b
#include <EEPROM.h>                                                                       // librairie de gestion de l'eeprom
#define NUM_LEDS       1                                                                  // nombre de leds de diagnostique
#define LED_DATA_PIN  15                                                                  // data pin des leds de diag.                                            wemos D8
#define TRIGGER_PIN_1  4                                                                  // Arduino pin tied to trigger pin on the ultrasonic sensor1.            wemos D2
#define ECHO_PIN_1     5                                                                  // Arduino pin tied to echo pin on the ultrasonic sensor1.               wemos D1
#define TRIGGER_PIN_2  2                                                                  // Arduino pin tied to trigger pin on the ultrasonic sensor2.            wemos D4
#define ECHO_PIN_2     0                                                                  // Arduino pin tied to echo pin on the ultrasonic sensor2.               wemos D3
#define pin4051_1     14                                                                  // Select bus du 4051                                                    wemos D5
#define pin4051_2     12                                                                  // Select bus du 4051                                                    wemos D6
#define pin4051_3     13                                                                  // Select bus du 4051                                                    wemos D7
#define dhtPin        16                                                                  // Connect DHT sensor to                                                 wemos D0 
#define MAX_DISTANCE  200                                                                 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define aref_voltage  5                                                                   // tension analog_ref = 5 volts
ESP8266WebServer server(80);                                                              // serveur web sur port 80
NewPing sonar_1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);                                 // NewPing setup of pins and maximum distance.
NewPing sonar_2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);                                 // NewPing setup of pins and maximum distance.
CRGB leds[NUM_LEDS];                                                                      // Define the array of leds      
char* base = "plantoid/";                                                                 // base de l'adresse OSC
char oscAddr[80] = "";                                                                    // tableau contenant l'adresse OSC complette pour les adresse osc
char addr[80] = "";                                                                       // tableau contenant l'adresse OSC pour le serveur web        
WiFiUDP Udp;  
//const IPAddress outIp(192,168,1,8);                                                     // remote IP of the receptor
const unsigned int outPort = 8000;                                                        // remote port to send OSC
const unsigned int localPort = 8888;                                                      // local port to listen for OSC packets (actually not used for sending)
DHTesp dht;
float a1State = 0;                                                                        // variables d'état des lectures analog/digital
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
float duration1, duration2, distance1, distance2;
float previousDistance1, previousDistance2;
int sonar_iterations = 5;
float humidity ; 
float temperature ;
float previousTemperature ; 
float previousHumidity;
int aMinimumTrueValue = 17;                                                                 //filtre les valeurs de moins de x, pour les lectures analogiques
int aReadDelay = 5;                                                                         //delais entre commutation du4051 et la lecture analogique
int aMinNoise = 60;                                                                         //bruit tolléré en mode analogique
int initiation = EEPROM.read(9);
int IP1 = EEPROM.read(10)? EEPROM.read(10): 192;
int IP2 = EEPROM.read(11)? EEPROM.read(11): 168;
int IP3 = EEPROM.read(12)? EEPROM.read(12): 1;
int IP4 = EEPROM.read(13)? EEPROM.read(13): 8;
IPAddress outIp;
int plantoide = EEPROM.read(14);                                                             //numero de la plantoide    byte 14 de l'eeprom
int numeroBoitier = EEPROM.read(15);                                                         //numero du boitier         byte 15 de l'eeprom
void setup() {                                                                                         
   if(IP1) {  outIp = IPAddress(IP1,IP2,IP3,IP4);
   } else {   outIp = IPAddress(192,168,1,8);
   }   
   plantoide = plantoide ? plantoide : 9;
   numeroBoitier = numeroBoitier ? numeroBoitier : 9;                                                                                                                                                                      
   FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS);                                  // initialisation de la led de diag.                
   leds[0] = CRGB::Black;                                                                     
   FastLED.show();
   temperature = dht.getTemperature();                                                       //prise de temperature
   delay(dht.getMinimumSamplingPeriod());
   humidity = dht.getHumidity();                                                             //prise d'humidité
   delay(dht.getMinimumSamplingPeriod());                                                                                       
   pinMode(pin4051_1, OUTPUT);                                                               //initialisation du port d'adressage du 4051
   pinMode(pin4051_2, OUTPUT); 
   pinMode(pin4051_3, OUTPUT);                                        
   Serial.begin(115200);                                                                     // initialisation du port serie                                                                       
   Serial.println();                                                                         //message de courtoisie
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
   leds[0] = CRGB::Blue;                                                                     //led de diag en bleu = boot 
   FastLED.show();
   WiFiManager wifiManager;
   byte mac[6]; WiFi.macAddress(mac);
   if(initiation) {
      sprintf(addr, "%s%d/%d/", base, plantoide, numeroBoitier);
                  } 
   else { sprintf(addr, "%s-%d:%d:%d:%d", base, mac[5],mac[4],mac[3],mac[2]);
        }
   wifiManager.autoConnect(addr);
   Serial.println("Plantoid Sensor Node Pret"); 
   Serial.println("connecté au routeur avec succes");
   Serial.println("UDP OK");
   Udp.begin(localPort);
   Serial.print("Port: ");
   Serial.println(outPort);                                                             
   if (MDNS.begin("esp8266")) {
      Serial.println("MDNS OK");
                              }
   server.on("/", handleRoot);
   server.on("/restart", [](){
      server.send(200, "text/plain", "OK! Lets restart this plantoid node. Wait a second.");
      delay(600);
      ESP.restart();
                             });
   server.on("/lit", [](){
      server.send(200, "text/plain", "OK! Lets lit the diag. LED in red!.");
      leds[0] = CRGB::Red; 
      FastLED.show();
                             });                             
   server.on("/unlit", [](){
      server.send(200, "text/plain", "OK! Lets unlit the diag. LED");
      leds[0] = CRGB::Black; 
      FastLED.show();
                             });                             
   server.onNotFound(handleNotFound);
   server.on("/submit", handleSubmit);
   server.begin();
   Serial.println("serveur HTTP OK");
   leds[0] = CRGB::Green;                                                                  //led diag. verte = bien booté
   FastLED.show();
   delay(800);
   leds[0] = CRGB::Black;                                                                  //led diag. verte = bien booté
   FastLED.show();
}
void loop() {
server.handleClient();                                                                               
temperature = dht.getTemperature();                                                        // cycle de temperature
if (temperature != previousTemperature){
   sprintf(oscAddr, "%s%d/%d/temp ", base, plantoide, numeroBoitier);
   OSCMessage msg(oscAddr);
   msg.add(temperature);
   Udp.beginPacket(outIp, outPort);
   msg.send(Udp);
   Udp.endPacket();
   msg.empty();
   previousTemperature = temperature;  }                                                                                              
duration1 = sonar_1.ping_median(sonar_iterations);                                        //cycle du sonar 1
distance1 = (duration1 / 2) * ((331.4 + (0.606 * temperature) +(0.0124 * humidity) )/1000);
if (distance1 != previousDistance1){
   sprintf(oscAddr, "%s%d/%d/sonar 1 ", base, plantoide, numeroBoitier);
   OSCMessage msg(oscAddr);
   msg.add(distance1);
   Udp.beginPacket(outIp, outPort);
   msg.send(Udp);
   Udp.endPacket();
   msg.empty();
   previousDistance1 = distance1;  }                                                                                                
duration2 = sonar_2.ping_median(sonar_iterations);                                        //cycle du sonar 2
distance2 = (duration2 / 2) * ((331.4 + (0.606 * temperature) +(0.0124 * humidity) )/1000);
if (distance2 != previousDistance2){
   sprintf(oscAddr, "%s%d/%d/sonar 2 ", base, plantoide, numeroBoitier);
   OSCMessage msg(oscAddr);
   msg.add(distance2);
   Udp.beginPacket(outIp, outPort);
   msg.send(Udp);
   Udp.endPacket();
   msg.empty();
   previousDistance2 = distance2;  }                                                                                                  
humidity = dht.getHumidity();                                                              // cycle d'humidité
if (humidity != previousHumidity){
 sprintf(oscAddr, "%s%d/%d/hum ", base, plantoide, numeroBoitier);
   OSCMessage msg(oscAddr);
    msg.add(humidity);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    previousHumidity = humidity; }                                                                                    
digitalWrite(pin4051_1, HIGH);                                                              // cycle analog in 1
digitalWrite(pin4051_2, HIGH);      
digitalWrite(pin4051_3, LOW);   
delay(aReadDelay); 
a1State = analogRead(0);  
if(a1State < aMinimumTrueValue){
  a1State = 0;
                               } 
if ((a1State - a1PreviousState) > aMinNoise || ((a1PreviousState - a1State) > aMinNoise))
{
if (a1State != a1PreviousState)
{
 sprintf(oscAddr, "%s%d/%d/analog 1 ", base, plantoide, numeroBoitier);
 OSCMessage msg(oscAddr);
 msg.add(a1State);
 Udp.beginPacket(outIp, outPort);
 msg.send(Udp);
 Udp.endPacket();
 msg.empty();
 a1PreviousState = a1State;
}  
}                                                                
digitalWrite(pin4051_1, LOW);                                                              // cycle analog in 2
digitalWrite(pin4051_2, LOW);      
digitalWrite(pin4051_3, LOW);   
delay(aReadDelay); //not sure if this delay is strictly necessary 
a2State = analogRead(0);  // read the input pin 
if(a2State < aMinimumTrueValue){
  a2State = 0;
                               } 
if ((a2State - a2PreviousState) > aMinNoise || ((a2PreviousState - a2State) > aMinNoise))
{
if (a2State != a2PreviousState)
{
 sprintf(oscAddr, "%s%d/%d/analog 2 ", base, plantoide, numeroBoitier);
 OSCMessage msg(oscAddr);
 msg.add(a2State);
 Udp.beginPacket(outIp, outPort);
 msg.send(Udp);
 Udp.endPacket();
 msg.empty();
 a2PreviousState = a2State;
}   
}
digitalWrite(pin4051_1, HIGH);                                                             // cycle analog in 3
digitalWrite(pin4051_2, LOW);      
digitalWrite(pin4051_3, LOW);   
delay(aReadDelay); 
a3State = analogRead(0);  
if(a3State < aMinimumTrueValue){
  a3State = 0;
                               } 
if ((a3State - a3PreviousState) > aMinNoise || ((a3PreviousState - a3State) > aMinNoise))
{
if (a3State != a3PreviousState)
{
 sprintf(oscAddr, "%s%d/%d/analog 3 ", base, plantoide, numeroBoitier);
 OSCMessage msg(oscAddr);
 msg.add(a3State);
 Udp.beginPacket(outIp, outPort);
 msg.send(Udp);
 Udp.endPacket();
 msg.empty();
 a3PreviousState = a3State;
} 
}                                                                                                   
digitalWrite(pin4051_1, LOW);                                                              // cycle analog in 4
digitalWrite(pin4051_2, HIGH);      
digitalWrite(pin4051_3, LOW);   
delay(aReadDelay); 
a4State = analogRead(0);  
if(a4State < aMinimumTrueValue){
  a4State = 0;
                               } 
if ((a4State - a4PreviousState) > aMinNoise || ((a4PreviousState - a4State) > aMinNoise))
{
if (a4State != a4PreviousState)
{
 sprintf(oscAddr, "%s%d/%d/analog 4 ", base, plantoide, numeroBoitier);
 OSCMessage msg(oscAddr);
 msg.add(a4State);
 Udp.beginPacket(outIp, outPort);
 msg.send(Udp);
 Udp.endPacket();
 msg.empty();
 a4PreviousState = a4State;
}  
}                                                                                                     
digitalWrite(pin4051_1, LOW);                                                           // cycle analog in 5
digitalWrite(pin4051_2, LOW);      
digitalWrite(pin4051_3, HIGH);   
delay(aReadDelay); 
a5State = analogRead(0);  
if(a5State < aMinimumTrueValue){
  a5State = 0;
                               } 
if ((a5State - a5PreviousState) > aMinNoise || ((a5PreviousState - a5State) > aMinNoise))
{
if (a5State != a5PreviousState)
{
 sprintf(oscAddr, "%s%d/%d/analog 5 ", base, plantoide, numeroBoitier);
 OSCMessage msg(oscAddr);
 msg.add(a5State);
 Udp.beginPacket(outIp, outPort);
 msg.send(Udp);
 Udp.endPacket();
 msg.empty();
 a5PreviousState = a5State;
}  
}                                                                                                           
digitalWrite(pin4051_1, LOW);                                                            // cycle analog in 6
digitalWrite(pin4051_2, HIGH);      
digitalWrite(pin4051_3, HIGH);   
delay(aReadDelay); 
a6State = analogRead(0); 
if(a6State < aMinimumTrueValue){
  a6State = 0;
} 
if ((a6State - a6PreviousState) > aMinNoise || ((a6PreviousState - a6State) > aMinNoise))
{
if (a6State != a6PreviousState)
{
 sprintf(oscAddr, "%s%d/%d/analog 6 ", base, plantoide, numeroBoitier);
 OSCMessage msg(oscAddr);
 msg.add(a6State);
 Udp.beginPacket(outIp, outPort);
 msg.send(Udp);
 Udp.endPacket();
 msg.empty();
 a6PreviousState = a6State;
}  
}                                                                                                              
digitalWrite(pin4051_1, HIGH);                                                            // cycle analog in 7
digitalWrite(pin4051_2, HIGH);      
digitalWrite(pin4051_3, HIGH);   
delay(aReadDelay);
a7State = analogRead(0);
if(a7State < aMinimumTrueValue){
  a7State = 0;
                               }   
if ((a7State - a7PreviousState) > aMinNoise || ((a7PreviousState - a7State) > aMinNoise))
{
if (a7State != a7PreviousState)
{
 sprintf(oscAddr, "%s%d/%d/analog 7 ", base, plantoide, numeroBoitier);
 OSCMessage msg(oscAddr);
 msg.add(a8State);
 Udp.beginPacket(outIp, outPort);
 msg.send(Udp);
 Udp.endPacket();
 msg.empty();
 a7PreviousState = a7State;
}  
}
digitalWrite(pin4051_1, HIGH);                                                             // cycle analog in 8   
digitalWrite(pin4051_2, LOW);      
digitalWrite(pin4051_3, HIGH);   
delay(aReadDelay); 
a6State = analogRead(0);   
if(a8State < aMinimumTrueValue){
  a8State = 0;
                               } 
if ((a8State - a8PreviousState) > aMinNoise || ((a8PreviousState - a8State) > aMinNoise))
{
if (a8State != a8PreviousState)
{
 sprintf(oscAddr, "%s%d/%d/analog 8 ", base, plantoide, numeroBoitier);
 OSCMessage msg(oscAddr);
 msg.add(a6State);
 Udp.beginPacket(outIp, outPort);
 msg.send(Udp);
 Udp.endPacket();
 msg.empty();
 a8PreviousState = a8State;
} 
}
}
void handleRoot() {                                                                        //formulaire html de configuration du sensor node
  String page = "";
   page +=  "<html><body><H1>Configuration du noeud de capteurs plantoide : </h1> ";
   page +=" <form action='/submit' method='get'>";
   page +=" Adresse IP. du récépteur: ";
   String pp = (String)" <input type='number' min=0 max=255 name='IP1' value='" + IP1 + (String)"'></input> . <input type='number' min=0 max=255 name='IP2' value='" + IP2 +  (String)"'></input> . <input type='number' min=0 max=255 name='IP3' value='" + IP3 +  (String)"'></input> . <input type='number' min=0 max=255 name='IP4' value='" + IP4 + (String)"'></input>";
   page += pp;
   page += "<br> Plantoide #" + (String)" <input type='number' min=0 max=255 name='NPlantoid' value='" + plantoide + (String)"'></input>/<input type='number' min=0 max=255 name='NBoitier' value='" + numeroBoitier + (String)"'></input>";
   page +=" <input type='submit' value='save values'></form>";
   page +=" </body></html>";
   server.send(200,"text/html", page);
}
void handleSubmit(){                                                                      //encaissement du formulaire dans l'eeprom
  Serial.println("reading arguments: " + (String)server.method() + "   ..." + server.arg(0) + server.arg(1) + server.arg(2) + server.arg(3));
  IP1 = atoi(server.arg(0).c_str());  EEPROM.write(10, byte(IP1));
  IP2 = atoi(server.arg(1).c_str());  EEPROM.write(11, byte(IP2));
  IP3 = atoi(server.arg(2).c_str());  EEPROM.write(12, byte(IP2));
  IP4 = atoi(server.arg(3).c_str());  EEPROM.write(13, byte(IP3));
  plantoide = atoi(server.arg(4).c_str()); EEPROM.write(14, byte(plantoide));
  numeroBoitier = atoi(server.arg(5).c_str()); EEPROM.write(15, byte(numeroBoitier));
  EEPROM.write(9, byte(1));
  EEPROM.commit();
  outIp = IPAddress(IP1, IP2, IP3, IP4);
  String page = "";
  page +=  "<html><body><H1>Nouveau récépteur : ";
  page += outIp;
  page +=" </h1> Nom du Boitier = ";
  page += plantoide;
  page += "/";
  page += numeroBoitier;
  page +="</body></html>";
  server.send(200, "text/html", page);
}
void handleNotFound(){
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
 }
