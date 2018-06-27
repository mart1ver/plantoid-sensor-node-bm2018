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
#define LOOP_DELAY    100
#define defaultAMinimumTrueValue 17
#define defaultAMinNoise 60
#define aReadDelay 1                                                                      // delais entre commutation du4051 et la lecture analogique
#define sonar_iterations 1

typedef struct s_value {
	float	value = 0;
	float	previous_value = 0;
	uint8_t	minimum_true_value;
	uint8_t	min_noise;
} t_value;


ESP8266WebServer server(80);                                                              // serveur web sur port 80

NewPing sonar_1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE);                                 // NewPing setup of pins and maximum distance.
NewPing sonar_2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);                                 // NewPing setup of pins and maximum distance.

DHTesp	dht;

CRGB	leds[NUM_LEDS];                                                                   // Define the array of leds
char*	base		= "plantoid";                                                        // base de l'adresse OSC
char	addr[80]	= "";                                                                 // tableau contenant l'adresse OSC pour le serveur web
WiFiUDP	Udp;

const unsigned int outPort = 8000;                                                        // remote port to send OSC
const unsigned int localPort = 8888;                                                      // local port to listen for OSC packets (actually not used for sending)

t_value		adc[8];

t_value humidity;
t_value temperature;
t_value distance1;
t_value distance2;

uint8_t adc_order[8] = {
	0b110, // 6
	0b000, // 0
	0b100, // 4
	0b010, // 2
	0b001, // 1
	0b011, // 3
	0b111, // 7
	0b101  // 5
};

uint8_t initiation;
uint8_t IP1;
uint8_t IP2;
uint8_t IP3;
uint8_t IP4;

IPAddress outIp;

uint8_t plantoide;
uint8_t numeroBoitier;

void init_eeprom() {
	for (int i=0; i<16; i+=2) {
		uint8_t minimum_true_value = EEPROM.read(17+i);
		uint8_t min_noise = EEPROM.read(18+i);

		adc[i/2].minimum_true_value = minimum_true_value ? minimum_true_value : defaultAMinimumTrueValue;
		adc[i/2].min_noise = min_noise ? min_noise : defaultAMinNoise;
	}

	int initiation = EEPROM.read(9);                                            // byte 9 de l'eeprom , censcé etre à 1 si le boitier à déja été configuré via l'interface html
	IP1 = EEPROM.read(10);                                                      // lecture de l'ip dans l'eeprom , valeurs par défaut = 192.168.1.8,  bytes 10, 11, 12, 13 de l'eeprom
	IP2 = EEPROM.read(11);
	IP3 = EEPROM.read(12);
	IP4 = EEPROM.read(13);

	if (IP1 == 0) {
		IP1 = 192;
		IP2 = 168;
		IP3 = 1;
		IP4 = 8;
	}

	plantoide = EEPROM.read(14);                                                // numero de la plantoide    byte 14 de l'eeprom
	numeroBoitier = EEPROM.read(15);                                            // numero du boitier         byte 15 de l'eeprom
}

void setup() {

	EEPROM.begin(512);
	init_eeprom();

	if(IP1)
		outIp = IPAddress(IP1,IP2,IP3,IP4);
	else
		outIp = IPAddress(192,168,1,8);

	plantoide = plantoide ? plantoide : 9;                                                  // si pas de numero de plantoide valeur par defaut = 9
	numeroBoitier = numeroBoitier ? numeroBoitier : 9;                                      // si pas de numero de plantoide valeur par defaut = 9

	FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS);                                // initialisation de la led de diag.
	leds[0] = CRGB::Black;
	FastLED.show();

	dht.setup(dhtPin);
	temperature.value = dht.getTemperature();                                                     // prise de temperature
	delay(dht.getMinimumSamplingPeriod());                                                  // delais mini entre deux sollicitations du dht11

	humidity.value = dht.getHumidity();                                                     // prise d'humidité

	pinMode(pin4051_1, OUTPUT);                                                             // initialisation du port d'adressage du 4051
	pinMode(pin4051_2, OUTPUT);
	pinMode(pin4051_3, OUTPUT);

	Serial.begin(115200);                                                                   // initialisation du port serie
	Serial.print("\n\nvous etes en communication avec le neud de capteur numero: ");
	Serial.print(numeroBoitier);
	Serial.print(" installé sur le plantoide: ");
	Serial.println(plantoide);
	Serial.print("\nLes messages OSC. seronts envoyés à la cible: ");
	Serial.println(outIp);
	Serial.println();

	leds[0] = CRGB::Blue;                                                                   // led de diag en bleu = boot
	FastLED.show();
	WiFiManager wifiManager;

	byte mac[6];
	WiFi.macAddress(mac);

	if(initiation)
		sprintf(addr, "%s/%d/%d/", base, plantoide, numeroBoitier);
	else
		sprintf(addr, "%s-%d:%d:%d:%d", base, mac[5],mac[4],mac[3],mac[2]);

	wifiManager.autoConnect(addr);
	Serial.println("Plantoid Sensor Node Pret");
	Serial.println("connecté au routeur avec succes");

	Udp.begin(localPort);
	Serial.println("UDP OK");
	Serial.print("Port: ");
	Serial.println(outPort);

	if (MDNS.begin("esp8266"))
		Serial.println("MDNS OK");

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

void send_osc(const char *sensor, uint8_t index, float value) {
	char oscAddr[80];

	sprintf(oscAddr, "/%s/%d/%d/%s", base, plantoide, numeroBoitier, sensor);
	OSCMessage msg(oscAddr);
	msg.add(index);
	msg.add(value);
	Udp.beginPacket(outIp, outPort);
		msg.send(Udp);
	Udp.endPacket();
	msg.empty();
}

void loop() {
	server.handleClient();

	temperature.value = dht.getTemperature();                                                        // cycle de temperature
	if (temperature.value != temperature.previous_value) {
		send_osc("temp", 0, temperature.value);
		temperature.previous_value = temperature.value;
	}

	int duration = sonar_1.ping_median(sonar_iterations);                                        // cycle du sonar 1
	distance1.value = (duration / 2) * ((331.4 + (0.606 * temperature.value) +(0.0124 * humidity.value) )/1000);
	if (distance1.value != distance1.previous_value) {
		send_osc("sonar", 0, distance1.value);
		distance1.previous_value = distance1.value;
	}

	duration = sonar_2.ping_median(sonar_iterations);                                        // cycle du sonar 2
	distance2.value = (duration / 2) * ((331.4 + (0.606 * temperature.value) +(0.0124 * humidity.value) )/1000);
	if (distance2.value != distance2.previous_value){
		send_osc("sonar", 1, distance2.value);
		distance2.previous_value = distance2.value;
	}

	humidity.value = dht.getHumidity();                                                              // cycle d'humidité
	if (humidity.value != humidity.previous_value){
		send_osc("hum", 0, humidity.value);
		humidity.previous_value = humidity.value;
	}

	for (int i=0; i < 8; i++) {
		digitalWrite(pin4051_1, (adc_order[i]>>2)&1);                                                              // cycle analog in 1
		digitalWrite(pin4051_2, (adc_order[i]>>1)&1);
		digitalWrite(pin4051_3, (adc_order[i]>>0)&1);
		delay(aReadDelay);

		adc[i].value = analogRead(0);

		if(adc[i].value < adc[i].minimum_true_value)
			adc[i].value = 0;

		if ((adc[i].value - adc[i].previous_value) > adc[i].min_noise || ((adc[i].previous_value - adc[i].value) > adc[i].min_noise)) {
			if (adc[i].value != adc[i].previous_value) {
				send_osc("analog", i, adc[i].value);
				adc[i].previous_value = adc[i].value;
			}
		}
	}
	delay(LOOP_DELAY);
}

void handleRoot() {                                                                        // formulaire html de configuration du sensor node
	String page = "";
	page +=  "<html lang='fr' ><head><meta charset='UTF-8'></head><body>";
	page +=  "<H1>Configuration du noeud de capteurs plantoide : </h1> ";
	page +=" <form action='/submit' method='get'>";
	page +=" Adresse IP. du récépteur: ";
	String pp =   (String)" <input type='number' min=0 max=255 name='IP1' value='" + IP1
				+ (String)"'></input> . <input type='number' min=0 max=255 name='IP2' value='" + IP2
				+ (String)"'></input> . <input type='number' min=0 max=255 name='IP3' value='" + IP3
				+ (String)"'></input> . <input type='number' min=0 max=255 name='IP4' value='" + IP4
				+ (String)"'></input>";
	page += pp;
	page += "<br> Plantoide #" + (String)" <input type='number' min=0 max=255 name='NPlantoid' value='" + plantoide + (String)"'></input>/<input type='number' min=0 max=255 name='NBoitier' value='" + numeroBoitier + (String)"'></input>";
	page +="<br><br> <h2>Configuration des analog in:</h2>";
	page += "<br> a1MinimumTrueValue:" + (String)" <input type='number' min=0 max=255 name='a1MinimumTrueValue' value='" + adc[0].minimum_true_value + (String)"'></input>a1MinNoise:<input type='number' min=0 max=255 name='a1MinNoise' value='" + adc[0].min_noise + (String)"'></input>";
	page += "<br> a2MinimumTrueValue:" + (String)" <input type='number' min=0 max=255 name='a2MinimumTrueValue' value='" + adc[1].minimum_true_value + (String)"'></input>a2MinNoise:<input type='number' min=0 max=255 name='a2MinNoise' value='" + adc[1].min_noise + (String)"'></input>";
	page += "<br> a3MinimumTrueValue:" + (String)" <input type='number' min=0 max=255 name='a3MinimumTrueValue' value='" + adc[2].minimum_true_value + (String)"'></input>a3MinNoise:<input type='number' min=0 max=255 name='a3MinNoise' value='" + adc[2].min_noise + (String)"'></input>";
	page += "<br> a4MinimumTrueValue:" + (String)" <input type='number' min=0 max=255 name='a4MinimumTrueValue' value='" + adc[3].minimum_true_value + (String)"'></input>a4MinNoise:<input type='number' min=0 max=255 name='a4MinNoise' value='" + adc[3].min_noise + (String)"'></input>";
	page += "<br> a5MinimumTrueValue:" + (String)" <input type='number' min=0 max=255 name='a5MinimumTrueValue' value='" + adc[4].minimum_true_value + (String)"'></input>a5MinNoise:<input type='number' min=0 max=255 name='a5MinNoise' value='" + adc[4].min_noise + (String)"'></input>";
	page += "<br> a6MinimumTrueValue:" + (String)" <input type='number' min=0 max=255 name='a6MinimumTrueValue' value='" + adc[5].minimum_true_value + (String)"'></input>a6MinNoise:<input type='number' min=0 max=255 name='a6MinNoise' value='" + adc[5].min_noise + (String)"'></input>";
	page += "<br> a7MinimumTrueValue:" + (String)" <input type='number' min=0 max=255 name='a7MinimumTrueValue' value='" + adc[6].minimum_true_value + (String)"'></input>a7MinNoise:<input type='number' min=0 max=255 name='a7MinNoise' value='" + adc[6].min_noise + (String)"'></input>";
	page += "<br> a8MinimumTrueValue:" + (String)" <input type='number' min=0 max=255 name='a8MinimumTrueValue' value='" + adc[7].minimum_true_value + (String)"'></input>a8MinNoise:<input type='number' min=0 max=255 name='a8MinNoise' value='" + adc[7].min_noise + (String)"'></input>";
	page +=" <br><input type='submit' value='enregistrer'></form>";
	page +="  <br><a href='lit'>Allumer la LED en rouge</a> ";
	page +="  <br><a href='unlit'>Eteindre la LED</a> ";
	page +=" </body></html>";
	server.send(200,"text/html", page);
}

void handleSubmit() {                                                                      // encaissement du formulaire dans l'eeprom
	Serial.println("reading arguments: " + (String)server.method() + "   ..." + server.arg(0) + server.arg(1) + server.arg(2) + server.arg(3));
	IP1 = atoi(server.arg(0).c_str());  EEPROM.write(10, byte(IP1));
	IP2 = atoi(server.arg(1).c_str());  EEPROM.write(11, byte(IP2));
	IP3 = atoi(server.arg(2).c_str());  EEPROM.write(12, byte(IP3));
	IP4 = atoi(server.arg(3).c_str());  EEPROM.write(13, byte(IP4));
	plantoide = atoi(server.arg(4).c_str()); EEPROM.write(14, byte(plantoide));
	numeroBoitier = atoi(server.arg(5).c_str()); EEPROM.write(15, byte(numeroBoitier));

	for (int i=0;i<16; i+=2) {
		adc[i].minimum_true_value = atoi(server.arg(6+i).c_str());
		EEPROM.write(17 + i, adc[i].minimum_true_value);

		adc[i].min_noise = atoi(server.arg(7+i).c_str());
		EEPROM.write(18 + i, adc[i].min_noise);
	}

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
	for (uint8_t i=0; i<server.args(); i++)
		message += " " + server.argName(i) + ": " + server.arg(i) + "\n";

	server.send(404, "text/plain", message);
}
