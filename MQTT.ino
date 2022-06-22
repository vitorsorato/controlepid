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
const byte heater = 3, cooler = 5, sensorPin = A0;
byte valorADTemp;
float tempAtual, millisAtual;
long int lastDeltaTime;


double error = 0;
double temperature;
double lastTemperature;

double kP = 5;
double kI = 3;
double kD = 6;

double Prop = 0;
double Integ = 0;
double Deriv = 0;

double PID = 0;
double setPoint = 40;
double setColer = 10;

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensagem recebida [");
  Serial.print(topic);
  Serial.print("]: ");
  String valorPayload = "";
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
    valorPayload = valorPayload + (char)payload[i];
  }
  int intValor = valorPayload.toInt();
  
  if(String(topic) == String("changePSTemperatura")){
    Serial.println("changePSTemperatura");
    setPoint = intValor;
  }
  if(String(topic) == String("changePSCooler")){
    Serial.println("changePSCooler");
    setColer = intValor;
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

  pinMode(heater,OUTPUT);
  pinMode(cooler,OUTPUT);
  pinMode(2,INPUT);
  pinMode(4,INPUT);
  digitalWrite(heater, 0);
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect(CLIENT_ID)) {
      Serial.println("connected");
      
      mqttClient.publish("arduinouno","conectado");
      mqttClient.subscribe("changePSTemperatura");
      mqttClient.subscribe("changePSCooler");
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  delay(500);
  if(!mqttClient.connected()){
    reconnect();
  }

  valorADTemp = analogRead(sensorPin);
  if(tempAtual != valorADTemp*5/(0.01*1024)){
    tempAtual = valorADTemp*5/(0.01*1024);
    sendData(tempAtual);
  }

  error  = setPoint - tempAtual;
  long int deltaTime = (millis() - lastDeltaTime);  
    
  Prop = error * kP;
  Integ += (error *kI) * deltaTime/1000.0;
  
  Deriv = ((lastTemperature - tempAtual) * kD)/deltaTime/1000.0;

  Serial.println("NOVA TEMPERATURA: " + padLeft(String(setPoint), 6) + "COOLER: " + padLeft(String(setColer), 6) + "Propositional: " + padLeft(String(Prop), 6) + "Integral: " + padLeft(String(Integ), 6) + "Derivative: " + padLeft(String(Deriv), 6));
    
  PID = Prop + Integ + Deriv;

  if(PID>255) PID = 255;
  if(PID<0)   PID = 0;
  analogWrite(heater, PID);

  int intCooler = ((setColer*1022)/100);
  analogWrite(cooler, intCooler);

  mqttClient.loop();
  lastDeltaTime = deltaTime;
  lastTemperature = tempAtual;
}

void sendData(float temp) {
  char msgBuffer[20];
  float temperatura = temp;
  if(mqttClient.connect(CLIENT_ID)) {
   mqttClient.publish("PStemperatura", dtostrf(temperatura, 6, 2, msgBuffer));
 }
}

String padLeft(String variable, int space) {
  String strSpaces = "";
   for(int i=0;i<space;i++){
    strSpaces += " ";
   }
   return variable + strSpaces;
}
