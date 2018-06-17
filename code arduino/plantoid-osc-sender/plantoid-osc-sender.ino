/*
sketch boitiers capteurs plantoid:
	8 analog/digital in
	2 utrasonic sensors
	1 dht 11
	1 ws2812b de diagnostic

	comportement de la led de diag:
		blue = boot,
		red = s'allume en http via /lit s'etteind via /unlit,
		green = connecteé au routeur (dure une seconde),
		black = fonctionnement normal
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
NewPing	sonar_1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);                                 // NewPing setup of pins and maximum distance.
NewPing	sonar_2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);                                 // NewPing setup of pins and maximum distance.
WiFiUDP	Udp;
DHTesp dht;

CRGB	leds[NUM_LEDS];                                                                      // Define the array of leds
char*	base = "plantoid/";                                                                 // base de l'adresse OSC
char	oscAddr[80] = "";                                                                    // tableau contenant l'adresse OSC complette pour les adresse osc
char	addr[80] = "";                                                                       // tableau contenant l'adresse OSC pour le serveur web

//const IPAddress outIp(192,168,1,8);                                                     // remote IP of the receptor
const unsigned int outPort = 8000;                                                        // remote port to send OSC
const unsigned int localPort = 8888;                                                      // local port to listen for OSC packets (actually not used for sending)

float aState[8];                                                                        // variables d'état des lectures analog/digital
float aPreviousState[8];
float distance1, distance2;
float previousDistance1, previousDistance2;
int sonar_iterations = 5;
float humidity ;
float temperature ;
float previousTemperature ;
float previousHumidity;
int aReadDelay = 5;                                                                          // delais entre commutation du4051 et la lecture analogique
int aMinimumTrueValue[8] = { 17, 17, 17, 17, 17, 17, 17, 17 };
int aMinNoise[8] = { 60, 60, 60, 60, 60, 60, 60, 60 };

int initiation = EEPROM.read(9);                                                             // byte 9 de l'eeprom , censcé etre à 1 si le boitier à déja été configuré via l'interface html
int IP1 = EEPROM.read(10)? EEPROM.read(10): 192;                                             // lecture de l'ip dans l'eeprom , valeurs par défaut = 192.168.1.8,  bytes 10, 11, 12, 13 de l'eeprom
int IP2 = EEPROM.read(11)? EEPROM.read(11): 168;
int IP3 = EEPROM.read(12)? EEPROM.read(12): 1;
int IP4 = EEPROM.read(13)? EEPROM.read(13): 8;

IPAddress outIp;

int plantoide = EEPROM.read(14);                                                             // numero de la plantoide    byte 14 de l'eeprom
int numeroBoitier = EEPROM.read(15);                                                         // numero du boitier         byte 15 de l'eeprom

void setup() {
	if(IP1)
		outIp = IPAddress(IP1,IP2,IP3,IP4);
	else
		outIp = IPAddress(192,168,1,8);

	plantoide = plantoide ? plantoide : 9;                                                    // si pas de numero de plantoide valeur par defaut = 9
	numeroBoitier = numeroBoitier ? numeroBoitier : 9;                                        // si pas de numero de plantoide valeur par defaut = 9

	FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS);                                  // initialisation de la led de diag.
	leds[0] = CRGB::Black;
	FastLED.show();

	temperature = dht.getTemperature();                                                       // prise de temperature
	delay(dht.getMinimumSamplingPeriod());                                                    // delais mini entre deux sollicitations du dht11

	humidity = dht.getHumidity();                                                             // prise d'humidité

	pinMode(pin4051_1, OUTPUT);                                                               // initialisation du port d'adressage du 4051
	pinMode(pin4051_2, OUTPUT);
	pinMode(pin4051_3, OUTPUT);

	Serial.begin(115200);
	Serial.print("\n\nvous etes en communication avec le neud de capteur numero: ");
	Serial.print(numeroBoitier);
	Serial.print(" installé sur le plantoide: ");
	Serial.print(plantoide);
	Serial.print("\n\nLes messages OSC. seronts envoyés à la cible: ");
	Serial.println(outIp);
	Serial.println();

	dht.setup(dhtPin);

	leds[0] = CRGB::Blue;                                                                     // led de diag en bleu = boot
	FastLED.show();

	WiFiManager wifiManager;
	byte mac[6]; WiFi.macAddress(mac);
	if(initiation)
		sprintf(addr, "%s%d/%d/", base, plantoide, numeroBoitier);
	else
		sprintf(addr, "%s-%d:%d:%d:%d", base, mac[5],mac[4],mac[3],mac[2]);

	wifiManager.autoConnect(addr);
	Udp.begin(localPort);

	Serial.println("Plantoid Sensor Node Pret");
	Serial.println("connecté au routeur avec succes");
	Serial.println("UDP OK");
	Serial.print("Port: ");
	Serial.println(outPort);

	if (MDNS.begin("esp8266"))
		Serial.println("MDNS OK");

	server.on("/", handleRoot);
	server.on("/submit", handleSubmit);
	server.onNotFound(handleNotFound);

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

	server.begin();

	Serial.println("serveur HTTP OK");
	leds[0] = CRGB::Green;                                                                  // led diag. verte = bien booté
	FastLED.show();
	delay(1000);
	leds[0] = CRGB::Black;                                                                  // led diag. black = en fonctionnement normal
	FastLED.show();
}

void loop() {
	server.handleClient();

	temperature = dht.getTemperature();                                                        // cycle de temperature
	if (temperature != previousTemperature) {
		sprintf(oscAddr, "%s%d/%d/temp ", base, plantoide, numeroBoitier);
		OSCMessage msg(oscAddr);
		msg.add(temperature);
		Udp.beginPacket(outIp, outPort);
			msg.send(Udp);
		Udp.endPacket();
		msg.empty();
		previousTemperature = temperature;
	}

	float duration = sonar_1.ping_median(sonar_iterations);                                        // cycle du sonar 1
	distance1 = (duration / 2) * ((331.4 + (0.606 * temperature) +(0.0124 * humidity) )/1000);
	if (distance1 != previousDistance1){
		sprintf(oscAddr, "%s%d/%d/sonar 1 ", base, plantoide, numeroBoitier);
		OSCMessage msg(oscAddr);
		msg.add(distance1);
		Udp.beginPacket(outIp, outPort);
		msg.send(Udp);
			Udp.endPacket();
		msg.empty();
		previousDistance1 = distance1;
	}

	duration = sonar_2.ping_median(sonar_iterations);                                        // cycle du sonar 2
	distance2 = (duration / 2) * ((331.4 + (0.606 * temperature) +(0.0124 * humidity) )/1000);
	if (distance2 != previousDistance2){
		sprintf(oscAddr, "%s%d/%d/sonar 2 ", base, plantoide, numeroBoitier);
		OSCMessage msg(oscAddr);
		msg.add(distance2);
		Udp.beginPacket(outIp, outPort);
		msg.send(Udp);
			Udp.endPacket();
		msg.empty();
		previousDistance2 = distance2;
	}

	humidity = dht.getHumidity();                                                              // cycle d'humidité
	if (humidity != previousHumidity){
		sprintf(oscAddr, "%s%d/%d/hum ", base, plantoide, numeroBoitier);
		OSCMessage msg(oscAddr);
		msg.add(humidity);
		Udp.beginPacket(outIp, outPort);
			msg.send(Udp);
		Udp.endPacket();
		msg.empty();
		previousHumidity = humidity;
	}

	for (uint i=0;i<8;i++) {
		digitalWrite(pin4051_1, i&1);
		digitalWrite(pin4051_2, (i>>1)&1);
		digitalWrite(pin4051_3, (i>>2)&1);
		delay(aReadDelay);

		aState[i] = analogRead(0);
		if (aState[i] < aMinimumTrueValue[i])
			aState[i] = 0;

		if ((aState[i] - aPreviousState[i]) > aMinNoise[i] || ((aPreviousState[i] - aState[i]) > aMinNoise[i]))
		{
			if (aState[i] != aPreviousState[i])
			{
				sprintf(oscAddr, "%s%d/%d/analog %d ", base, plantoide, numeroBoitier, i);
				OSCMessage msg(oscAddr);
				msg.add(aState[i]);
				Udp.beginPacket(outIp, outPort);
					msg.send(Udp);
				Udp.endPacket();
				msg.empty();
				aPreviousState[i] = aState[i];
			}
		}
	}
}

void handleRoot() {                                                                        // formulaire html de configuration du sensor node
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

void handleSubmit() {   // encaissement du formulaire dans l'eeprom
	Serial.println("reading arguments: "+ (String)server.method() + "   ..." + server.arg(0) + server.arg(1) + server.arg(2) + server.arg(3));
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
