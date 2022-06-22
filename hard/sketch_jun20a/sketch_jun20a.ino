long previousMillis;
const byte heater = 3, cooler = 5, sensorPin = A0;
byte valorADTemp;
float tempAtual, millisAtual;
long int lastDeltaTime;

double error = 0;
double temperature;
double lastTemperature;

double kP = 10;
double kI = 10;
double kD = 10;

double Prop = 0;
double Integ = 0;
double Deriv = 0;

double PID = 0;
double setPoint = 40;
double setColer = 10;


void setup() {
  Serial.begin(9600);
  pinMode(heater,OUTPUT);
  pinMode(cooler,OUTPUT);
  pinMode(2,INPUT);
  pinMode(4,INPUT);
  digitalWrite(heater, 0);

  pinMode(6,INPUT);
  pinMode(7,INPUT);
  pinMode(8,INPUT);
  pinMode(9,INPUT);
  pinMode(10,INPUT);
  pinMode(11,INPUT);
  pinMode(12,INPUT);
  pinMode(13,INPUT);
}


void loop() {
  
  valorADTemp = analogRead(sensorPin);
  tempAtual = valorADTemp*5/(0.01*1024);

  error  = setPoint - tempAtual;
    
  long int deltaTime = (millis() - lastDeltaTime);  
    
  Prop = error * kP;
  Integ += (error *kI) * deltaTime/1000.0;
  
  Deriv = ((lastTemperature - tempAtual) * kD)/deltaTime/1000.0;

  Serial.println(Prop);
  Serial.println(Integ);
  Serial.println(Deriv);
  

  PID = Prop + Integ + Deriv;

  if(PID>255) PID = 255;
  if(PID<0)   PID = 0;
  analogWrite(heater, PID);
  Serial.println(PID);
  
  int intCooler = ((setColer*1024)/100);
  analogWrite(cooler, intCooler);

  Serial.println("...............");
  Serial.println("Valor ADTemp   : "+String(valorADTemp));
  Serial.println("Valor tempAtual: "+String(tempAtual));
  Serial.println("Valor setPoint : "+String(setPoint));
  Serial.println("Valor kP       : "+String(kP));
  Serial.println("Valor kI       : "+String(kI));
  Serial.println("Valor kD       : "+String(kD));
  Serial.println("Valor error: "+String(error));
  Serial.println("______");


  if (digitalRead(6) == HIGH){
    setPoint = setPoint + 10;
  }
  if (digitalRead(7) == HIGH){
    kP = kP + 10;
  }
  if (digitalRead(8) == HIGH){
    kI = kI + 0.1;
  }
  if (digitalRead(9) == HIGH){
    kD = kD + 0.1;
  }
  if (digitalRead(13) == HIGH){
    digitalWrite(cooler, HIGH);
  }   
  
  lastDeltaTime = deltaTime;
  lastTemperature = tempAtual;
}
