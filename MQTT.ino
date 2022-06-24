#include <SPI.h>
#include <Ethernet.h>
#include "PubSubClient.h"

#define CLIENT_ID       "PSVSRA"
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
long int PIDTemp = 0;


double error = 0;
double temperature;
double lastTemperature;

double kP = 10;
double kI = 5;
double kD = 5;

double Prop = 0;
double Integ = 0;
double Deriv = 0;

double PID = 0;
double setPoint = 27.5;
double setColer = 0;

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
    setPoint = intValor;
  }
  if(String(topic) == String("changePSCooler")){
    setColer = intValor;
  }
  if(String(topic) == String("changePSKp")){
    kP = intValor;
  }
  if(String(topic) == String("changePSKi")){
    kI = intValor;
  }
  if(String(topic) == String("changePSKd")){
    kD = intValor;
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
      
      mqttClient.subscribe("changePSTemperatura");
      mqttClient.subscribe("changePSCooler");
      mqttClient.subscribe("changePSKp");
      mqttClient.subscribe("changePSKi");
      mqttClient.subscribe("changePSKd");
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  delay(1000);
  if(!mqttClient.connected()){
    reconnect();
  }

  valorADTemp = analogRead(sensorPin);
  if(tempAtual != valorADTemp*5/(0.01*1024)){
    tempAtual = valorADTemp*5/(0.01*1024);
    sendTemperatura(tempAtual);
  }

  error  = setPoint - tempAtual;
  long int deltaTime = (millis() - lastDeltaTime);  
    
  Prop = error * kP;
  Integ += (error *kI) * deltaTime/1000.0;
  Deriv = ((lastTemperature - tempAtual) * kD)/deltaTime/1000.0;
    
  PID = Prop + Integ + Deriv;

  if(PID>255) PID = 255;
  if(PID<0)   PID = 0;
  analogWrite(heater, PID);

  if(PID != PIDTemp){
    sendPID(PID);
  }

  int intCooler = round((setColer*1022)/100);
  analogWrite(cooler, intCooler);

  String newTemp = "Nova Temperatura: " + padLeft(String(setPoint), 10);
  String newCool = "Cooler: " + padLeft(String(setColer), 10);
    
  String knsProp = "kP: " + padLeft(String(kP), 10);
  String knsIntg = "kI: " + padLeft(String(kI), 10);
  String knsDerv = "kD: " + padLeft(String(kD), 10);
  
  String pidProp = "PROP: " + padLeft(String(Prop), 10);
  String pidIntg = "INTG: " + padLeft(String(Integ), 10);
  String pidDerv = "DERV: " + padLeft(String(Deriv), 10);

  Serial.println(newTemp + newCool + knsProp + knsIntg + knsDerv + pidProp + pidIntg + pidDerv);

  mqttClient.loop();
  lastDeltaTime = deltaTime;
  lastTemperature = tempAtual;
  PIDTemp = PID;
}

void sendTemperatura(float temp) {
  char msgBuffer[20];
  if(mqttClient.connect(CLIENT_ID)) {
   mqttClient.publish("PStemperatura", dtostrf(temp, 6, 2, msgBuffer));
 }
}


void sendPID(int PID) {
  char msgBuffer[20];
  if(mqttClient.connect(CLIENT_ID)) {
   mqttClient.publish("PSPID", dtostrf(PID, 6, 2, msgBuffer));
 }
}

String padLeft(String variable, int space) {
  String strSpaces = "";
  int lenVar = variable.length(); 
   for(int i=0;i<(space - lenVar);i++){
    strSpaces += " ";
   }
   return variable + strSpaces;
}
