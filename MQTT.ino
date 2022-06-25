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
float tempAtual, tempAtualFilt, millisAtual, xn, yn;
float xn1 = 0;
float yn1 = 0;

long int lastDeltaTime = 0;
long int PIDTemp = 0;


double error = 0;
double error_ant = 0;
double temperature;
double lastTemperature;

double kP = 10;
double kI = 5;
double kD = 5;

double Prop = 0;
double Integ = 0;
double Integ_ant = 0;
double Deriv = 0;

double PID = 0;
double setPoint = 27.5;
double setColer = 0;

void callback(char* topic, byte* payload, unsigned int length) {
  String valorPayload = "";
  for (int i=0;i<length;i++) {
    valorPayload = valorPayload + (char)payload[i];
  }
  Serial.println("Mensagem recebida [" + String(topic) + "]: " + valorPayload);
  
  double doubleValor = valorPayload.toDouble();
  
  if(String(topic) == String("changePSTemperatura")){
    setPoint = doubleValor;
  }
  if(String(topic) == String("changePSCooler")){
    setColer = doubleValor;
  }
  if(String(topic) == String("changePSKp")){
    kP = doubleValor;
  }
  if(String(topic) == String("changePSKi")){
    kI = doubleValor;
  }
  if(String(topic) == String("changePSKd")){
    kD = doubleValor;
  }

  zeraVariaveis();
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
    Serial.println("Conectando com MQTT...");
    if (mqttClient.connect(CLIENT_ID)) {
      Serial.println("CONECTADO!!");
      
      mqttClient.subscribe("changePSTemperatura");
      mqttClient.subscribe("changePSCooler");
      mqttClient.subscribe("changePSKp");
      mqttClient.subscribe("changePSKi");
      mqttClient.subscribe("changePSKd");
      
    } else {
      Serial.println("falha, rc=");
      Serial.println(mqttClient.state());
      Serial.println(" tentando novamente em 5 seg.");
      delay(5000);
    }
  }
}

void loop() {
  delay(1000);
  if(!mqttClient.connected()){
    reconnect();
  }


  tempAtual = analogRead(sensorPin)*5/(0.01*1024);
  Serial.println(tempAtual);
  tempAtualFilt = filtroPassaBaixa(tempAtual);
  Serial.println(tempAtualFilt);
  
  
  if(tempAtual<27.5) tempAtual = 27.5;
  
  sendTemperatura(tempAtual);

  error = setPoint - tempAtual;
  long int deltaTime = (millis() - lastDeltaTime)/1000;  

    
  Prop = error * kP;
  Integ = Integ_ant + ((error * kI * (deltaTime)));
  Deriv = ((error - error_ant) * kD)/deltaTime;

  if (isnan(Deriv)) Deriv = 0;
    
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
  String pidIntg = "INTG: " + padLeft(String(Integ), 14);
  String pidDerv = "DERV: " + padLeft(String(Deriv), 10);

  Serial.println(newTemp + newCool + knsProp + knsIntg + knsDerv + pidProp + pidIntg + pidDerv);

  lastDeltaTime = millis();
  lastTemperature = tempAtual;
  PIDTemp = PID;
  Integ_ant = Integ;
  error_ant = error;
  mqttClient.loop();
}

float filtroPassaBaixa(float tempAtual){

  xn = tempAtual + random(-50, 50)*0.01;
  if(yn1 == 0) yn1 = xn;
  yn = 0.969*yn1 + 0.0155*xn+ 0.0155*xn1;
  yn1 = yn;
  xn1 = xn;
  
  return yn;
}

void zeraVariaveis(){
  Integ_ant = 0;
  error_ant = 0;
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
