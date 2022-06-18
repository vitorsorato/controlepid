#include <SPI.h>
#include <Ethernet.h>
#include "PubSubClient.h"

#define CLIENT_ID       "VSRA"
#define INTERVAL        2000

int lichtstatus;

uint8_t mac[6] = {0x00,0x01,0x02,0x03,0x04,0x05};

EthernetClient ethClient;
PubSubClient mqttClient;

long previousMillis;
const byte heater =3, cooler = 5, sensorPin = A0;
byte valorADTemp;
float tempAtual, millisAtual;

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  String valorPayload = "";
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    valorPayload = valorPayload + (char)payload[i];
  }
  Serial.println("");
  Serial.println(topic);
  int intValor = valorPayload.toInt();
  
  if(String(topic) == String("changeTemperatura")){
    int intTemp = (intValor*(0.01*1024)/5);
    Serial.println(String(intTemp));
    digitalWrite(heater, 20);
  }
  if(String(topic) == String("changeCooler")){
    int intCooler = ((intValor*255)/100);
    Serial.println(String(intCooler));
    digitalWrite(cooler, intCooler);
  }
}

void setup() {
  Serial.begin(9600);
  if(Ethernet.begin(mac) == 0) {
    for(;;);
  }
  mqttClient.setClient(ethClient);
  mqttClient.setServer("broker.hivemq.com",1883);
  mqttClient.setCallback(callback);
  previousMillis = millis();
  pinMode(heater,OUTPUT);
  pinMode(cooler,OUTPUT);
  pinMode(2,INPUT);
  pinMode(4,INPUT);
  digitalWrite(heater, 0);

  tempAtual = 0;
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect(CLIENT_ID)) {
      Serial.println("connected");
      
      mqttClient.publish("arduinouno","conectado");
      mqttClient.subscribe("changeTemperatura");
      mqttClient.subscribe("changeCooler");
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  delay(2000);
  if(!mqttClient.connected()){
    reconnect();
  }
  if(millis() - previousMillis > INTERVAL) {

    valorADTemp = analogRead(sensorPin);

    if(tempAtual != valorADTemp*5/(0.01*1024)){
      tempAtual = valorADTemp*5/(0.01*1024);
      sendData(tempAtual);
    }

    previousMillis = millis();
  }


  
  mqttClient.loop();
}

void sendData(float temp) {
  char msgBuffer[20];
  float temperatura = temp;
  if(mqttClient.connect(CLIENT_ID)) {
   mqttClient.publish("temperatura", dtostrf(temperatura, 6, 2, msgBuffer));
   Serial.println("Pacote enviado");
 }
}
