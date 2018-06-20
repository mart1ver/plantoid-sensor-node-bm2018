/*
sketch boitiers capteurs plantoid, 8 analog/digital in , 2 utrasonic sensors , 1 dht 11 , 1 ws2812b dediagnostic
   comportement de la led de diag : blue = boot , red = s'allume en http via /lit s'etteind via /unlit , green = connecteé au routeur (dure une seconde), black = fonctionnement normal
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
#define dhtPin        16                                                                  // Connect DHT sensor to                                                 wemos D0 
#define ECHO_PIN_1     5                                                                  // Arduino pin tied to echo pin on the ultrasonic sensor1.               wemos D1
#define TRIGGER_PIN_1  4                                                                  // Arduino pin tied to trigger pin on the ultrasonic sensor1.            wemos D2
#define ECHO_PIN_2     0                                                                  // Arduino pin tied to echo pin on the ultrasonic sensor2.               wemos D3
#define TRIGGER_PIN_2  2                                                                  // Arduino pin tied to trigger pin on the ultrasonic sensor2.            wemos D4
#define pin4051_1     14                                                                  // Select bus du 4051                                                    wemos D5
#define pin4051_2     12                                                                  // Select bus du 4051                                                    wemos D6
#define pin4051_3     13                                                                  // Select bus du 4051                                                    wemos D7
#define LED_DATA_PIN  15                                                                  // data pin des leds de diag.                                            wemos D8
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
int sonar_iterations = 1;
float humidity ; 
float temperature ;
float previousTemperature ; 
float previousHumidity;
int defaultAMinimumTrueValue = 17;
int defaultAMinNoise = 60;
int aReadDelay =  1;                                                                       // delais entre commutation du4051 et la lecture analogique
int a1MinimumTrueValue = EEPROM.read(17)? EEPROM.read(17): defaultAMinimumTrueValue;       // filtre les valeurs de moins de x, pour les lectures analogiques sur analog 1
int a1MinNoise = EEPROM.read(18)? EEPROM.read(18): defaultAMinNoise;                       // bruit tolléré en mode analogique sur analog 1
int a2MinimumTrueValue = EEPROM.read(19)? EEPROM.read(19): defaultAMinimumTrueValue;       // filtre les valeurs de moins de x, pour les lectures analogiques sur analog 2
int a2MinNoise = EEPROM.read(20)? EEPROM.read(20): defaultAMinNoise;                       // bruit tolléré en mode analogique sur analog 2
int a3MinimumTrueValue = EEPROM.read(21)? EEPROM.read(21): defaultAMinimumTrueValue;       // filtre les valeurs de moins de x, pour les lectures analogiques sur analog 3
int a3MinNoise = EEPROM.read(22)? EEPROM.read(22): defaultAMinNoise;                       // bruit tolléré en mode analogique sur analog 3
int a4MinimumTrueValue = EEPROM.read(23)? EEPROM.read(23): defaultAMinimumTrueValue;       // filtre les valeurs de moins de x, pour les lectures analogiques sur analog 4
int a4MinNoise = EEPROM.read(24)? EEPROM.read(24): defaultAMinNoise;                       // bruit tolléré en mode analogique sur analog 4
int a5MinimumTrueValue = EEPROM.read(25)? EEPROM.read(25): defaultAMinimumTrueValue;       // filtre les valeurs de moins de x, pour les lectures analogiques sur analog 5
int a5MinNoise = EEPROM.read(26)? EEPROM.read(26): defaultAMinNoise;                       // bruit tolléré en mode analogique sur analog 5
int a6MinimumTrueValue = EEPROM.read(27)? EEPROM.read(27): defaultAMinimumTrueValue;       // filtre les valeurs de moins de x, pour les lectures analogiques sur analog 6
int a6MinNoise = EEPROM.read(28)? EEPROM.read(28): defaultAMinNoise;                       // bruit tolléré en mode analogique sur analog 6
int a7MinimumTrueValue = EEPROM.read(29)? EEPROM.read(29): defaultAMinimumTrueValue;       // filtre les valeurs de moins de x, pour les lectures analogiques sur analog 7
int a7MinNoise = EEPROM.read(30)? EEPROM.read(30): defaultAMinNoise;                       // bruit tolléré en mode analogique sur analog 7
int a8MinimumTrueValue = EEPROM.read(31)? EEPROM.read(31): defaultAMinimumTrueValue;       // filtre les valeurs de moins de x, pour les lectures analogiques sur analog 8
int a8MinNoise = EEPROM.read(32)? EEPROM.read(32): defaultAMinNoise;                       // bruit tolléré en mode analogique sur analog 8
int initiation = EEPROM.read(9);                                                           // byte 9 de l'eeprom , censcé etre à 1 si le boitier à déja été configuré via l'interface html 
int IP1 = EEPROM.read(10)? EEPROM.read(10): 192;                                           // lecture de l'ip dans l'eeprom , valeurs par défaut = 192.168.1.8,  bytes 10, 11, 12, 13 de l'eeprom
int IP2 = EEPROM.read(11)? EEPROM.read(11): 168;
int IP3 = EEPROM.read(12)? EEPROM.read(12): 1;
int IP4 = EEPROM.read(13)? EEPROM.read(13): 8;
IPAddress outIp;
int plantoide = EEPROM.read(14);                                                           // numero de la plantoide    byte 14 de l'eeprom
int numeroBoitier = EEPROM.read(15);                                                       // numero du boitier         byte 15 de l'eeprom
void setup() {                                                                                         
   if(IP1) {  outIp = IPAddress(IP1,IP2,IP3,IP4);
   } else {   outIp = IPAddress(192,168,1,8);
   }   
   plantoide = plantoide ? plantoide : 9;                                                  // si pas de numero de plantoide valeur par defaut = 9
   numeroBoitier = numeroBoitier ? numeroBoitier : 9;                                      // si pas de numero de plantoide valeur par defaut = 9                                                                                                                              
   FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS);                                // initialisation de la led de diag.                
   leds[0] = CRGB::Black;                                                                     
   FastLED.show();
   temperature = dht.getTemperature();                                                     // prise de temperature
   delay(dht.getMinimumSamplingPeriod());                                                  // delais mini entre deux sollicitations du dht11
   humidity = dht.getHumidity();                                                           // prise d'humidité                      
   pinMode(pin4051_1, OUTPUT);                                                             // initialisation du port d'adressage du 4051
   pinMode(pin4051_2, OUTPUT); 
   pinMode(pin4051_3, OUTPUT);                                        
   Serial.begin(115200);                                                                   // initialisation du port serie                                                                       
   Serial.println();                                                                       // message de courtoisie
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
   leds[0] = CRGB::Blue;                                                                   // led de diag en bleu = boot 
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
   leds[0] = CRGB::Green;                                                                // led diag. verte = bien booté
   FastLED.show();
   delay(1000);
   leds[0] = CRGB::Black;                                                                // led diag. black = en fonctionnement normal
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
duration1 = sonar_1.ping_median(sonar_iterations);                                        // cycle du sonar 1
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
duration2 = sonar_2.ping_median(sonar_iterations);                                        // cycle du sonar 2
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
if(a1State < a1MinimumTrueValue){
  a1State = 0;
                               } 
if ((a1State - a1PreviousState) > a1MinNoise || ((a1PreviousState - a1State) > a1MinNoise))
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
if(a2State < a2MinimumTrueValue){
  a2State = 0;
                               } 
if ((a2State - a2PreviousState) > a2MinNoise || ((a2PreviousState - a2State) > a2MinNoise))
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
if(a3State < a3MinimumTrueValue){
  a3State = 0;
                               } 
if ((a3State - a3PreviousState) > a3MinNoise || ((a3PreviousState - a3State) > a3MinNoise))
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
if(a4State < a4MinimumTrueValue){
  a4State = 0;
                               } 
if ((a4State - a4PreviousState) > a4MinNoise || ((a4PreviousState - a4State) > a4MinNoise))
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
digitalWrite(pin4051_1, LOW);                                                            // cycle analog in 5
digitalWrite(pin4051_2, LOW);      
digitalWrite(pin4051_3, HIGH);   
delay(aReadDelay); 
a5State = analogRead(0);  
if(a5State < a5MinimumTrueValue){
  a5State = 0;
                               } 
if ((a5State - a5PreviousState) > a5MinNoise || ((a5PreviousState - a5State) > a5MinNoise))
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
if(a6State < a6MinimumTrueValue){
  a6State = 0;
} 
if ((a6State - a6PreviousState) > a6MinNoise || ((a6PreviousState - a6State) > a6MinNoise))
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
if(a7State < a7MinimumTrueValue){
  a7State = 0;
                               }   
if ((a7State - a7PreviousState) > a7MinNoise || ((a7PreviousState - a7State) > a7MinNoise))
{
if (a7State != a7PreviousState)
{
 sprintf(oscAddr, "%s%d/%d/analog 7 ", base, plantoide, numeroBoitier);
 OSCMessage msg(oscAddr);
 msg.add(a7State);
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
a8State = analogRead(0);   
if(a8State < a8MinimumTrueValue){
  a8State = 0;
                               } 
if ((a8State - a8PreviousState) > a8MinNoise || ((a8PreviousState - a8State) > a8MinNoise))
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
void handleRoot() {                                                                        // formulaire html de configuration du sensor node
  String page = "";
  page +=  "<html lang='fr' ><head><meta charset='UTF-8'></head><body>";
  page +=  "<H1>Configuration du noeud de capteurs plantoide : </h1> ";
  page +=" <form action='/submit' method='get'>";
  page +=" Adresse IP. du récépteur: ";
  String pp = (String)" <input type='number' min=0 max=255 name='IP1' value='" + IP1 + (String)"'></input> . <input type='number' min=0 max=255 name='IP2' value='" + IP2 +  (String)"'></input> . <input type='number' min=0 max=255 name='IP3' value='" + IP3 +  (String)"'></input> . <input type='number' min=0 max=255 name='IP4' value='" + IP4 + (String)"'></input>";
  page += pp;
  page += "<br> Plantoide #" + (String)" <input type='number' min=0 max=255 name='NPlantoid' value='" + plantoide + (String)"'></input>/<input type='number' min=0 max=255 name='NBoitier' value='" + numeroBoitier + (String)"'></input>";
   page +="<br><br> <h2>Configuration des analog in:</h2>";
   page += "<br> a1MinimumTrueValue:" + (String)" <input type='number' min=0 max=255 name='a1MinimumTrueValue' value='" + a1MinimumTrueValue + (String)"'></input>a1MinNoise:<input type='number' min=0 max=255 name='a1MinNoise' value='" + a1MinNoise + (String)"'></input>";
   page += "<br> a2MinimumTrueValue:" + (String)" <input type='number' min=0 max=255 name='a2MinimumTrueValue' value='" + a2MinimumTrueValue + (String)"'></input>a2MinNoise:<input type='number' min=0 max=255 name='a2MinNoise' value='" + a2MinNoise + (String)"'></input>";
   page += "<br> a3MinimumTrueValue:" + (String)" <input type='number' min=0 max=255 name='a3MinimumTrueValue' value='" + a3MinimumTrueValue + (String)"'></input>a3MinNoise:<input type='number' min=0 max=255 name='a3MinNoise' value='" + a3MinNoise + (String)"'></input>";
   page += "<br> a4MinimumTrueValue:" + (String)" <input type='number' min=0 max=255 name='a4MinimumTrueValue' value='" + a4MinimumTrueValue + (String)"'></input>a4MinNoise:<input type='number' min=0 max=255 name='a4MinNoise' value='" + a4MinNoise + (String)"'></input>";
   page += "<br> a5MinimumTrueValue:" + (String)" <input type='number' min=0 max=255 name='a5MinimumTrueValue' value='" + a5MinimumTrueValue + (String)"'></input>a5MinNoise:<input type='number' min=0 max=255 name='a5MinNoise' value='" + a5MinNoise + (String)"'></input>";
   page += "<br> a6MinimumTrueValue:" + (String)" <input type='number' min=0 max=255 name='a6MinimumTrueValue' value='" + a6MinimumTrueValue + (String)"'></input>a6MinNoise:<input type='number' min=0 max=255 name='a6MinNoise' value='" + a6MinNoise + (String)"'></input>";   
   page += "<br> a7MinimumTrueValue:" + (String)" <input type='number' min=0 max=255 name='a7MinimumTrueValue' value='" + a7MinimumTrueValue + (String)"'></input>a7MinNoise:<input type='number' min=0 max=255 name='a7MinNoise' value='" + a7MinNoise + (String)"'></input>";
   page += "<br> a8MinimumTrueValue:" + (String)" <input type='number' min=0 max=255 name='a8MinimumTrueValue' value='" + a8MinimumTrueValue + (String)"'></input>a8MinNoise:<input type='number' min=0 max=255 name='a8MinNoise' value='" + a8MinNoise + (String)"'></input>";   
  page +=" <br><input type='submit' value='enregistrer'></form>";
  page +="  <br><a href='lit'>Allumer la LED en rouge</a> ";
  page +="  <br><a href='unlit'>Eteindre la LED</a> ";
  page +=" </body></html>";
  server.send(200,"text/html", page);
}
void handleSubmit(){                                                                      // encaissement du formulaire dans l'eeprom
  Serial.println("reading arguments: " + (String)server.method() + "   ..." + server.arg(0) + server.arg(1) + server.arg(2) + server.arg(3));
  IP1 = atoi(server.arg(0).c_str());  EEPROM.write(10, byte(IP1));
  IP2 = atoi(server.arg(1).c_str());  EEPROM.write(11, byte(IP2));
  IP3 = atoi(server.arg(2).c_str());  EEPROM.write(12, byte(IP2));
  IP4 = atoi(server.arg(3).c_str());  EEPROM.write(13, byte(IP3));
  plantoide = atoi(server.arg(4).c_str()); EEPROM.write(14, byte(plantoide));
  numeroBoitier = atoi(server.arg(5).c_str()); EEPROM.write(15, byte(numeroBoitier));

a1MinimumTrueValue = atoi(server.arg(6).c_str()); EEPROM.write(17, byte(a1MinimumTrueValue));
a1MinNoise = atoi(server.arg(7).c_str()); EEPROM.write(18, byte(a1MinNoise));

a2MinimumTrueValue = atoi(server.arg(8).c_str()); EEPROM.write(19, byte(a1MinimumTrueValue));
a2MinNoise = atoi(server.arg(9).c_str()); EEPROM.write(20, byte(a1MinNoise));

a3MinimumTrueValue = atoi(server.arg(10).c_str()); EEPROM.write(21, byte(a1MinimumTrueValue));
a3MinNoise = atoi(server.arg(11).c_str()); EEPROM.write(22, byte(a1MinNoise));

a4MinimumTrueValue = atoi(server.arg(12).c_str()); EEPROM.write(23, byte(a1MinimumTrueValue));
a4MinNoise = atoi(server.arg(13).c_str()); EEPROM.write(24, byte(a1MinNoise));

a5MinimumTrueValue = atoi(server.arg(14).c_str()); EEPROM.write(25, byte(a1MinimumTrueValue));
a5MinNoise = atoi(server.arg(15).c_str()); EEPROM.write(26, byte(a1MinNoise));

a6MinimumTrueValue = atoi(server.arg(16).c_str()); EEPROM.write(27, byte(a1MinimumTrueValue));
a6MinNoise = atoi(server.arg(17).c_str()); EEPROM.write(28, byte(a1MinNoise));

a7MinimumTrueValue = atoi(server.arg(18).c_str()); EEPROM.write(29, byte(a1MinimumTrueValue));
a7MinNoise = atoi(server.arg(19).c_str()); EEPROM.write(30, byte(a1MinNoise));

a8MinimumTrueValue = atoi(server.arg(20).c_str()); EEPROM.write(31, byte(a1MinimumTrueValue));
a8MinNoise = atoi(server.arg(21).c_str()); EEPROM.write(32, byte(a1MinNoise));
  
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
