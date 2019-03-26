#include <SoftwareSerial.h>

SoftwareSerial BTSerial(6, 7); // RX | TX

void setup()
{
  pinMode(13, OUTPUT);    // Vamos usar LED onboard como sinalizador de comunicação
  pinMode(9, OUTPUT);     // Pino para acionar o modo de configuracao (pino KEY do módulo)
  digitalWrite(9, HIGH);  // Ativar modo de configuracao para aceitar comandos Hayes
  Serial.begin(38400);
  BTSerial.begin(38400);  // HC-05 default speed in AT command more
  delay(1000);
  Serial.println("Entre com Comandos:");
}


void loop()
{

  // Leia a saída do HC-05 and envie ao Monitor Serial do Arduino 
  if (BTSerial.available()){
    digitalWrite(13, HIGH);
    Serial.write(BTSerial.read());
    digitalWrite(13, LOW);
  }

  // Leia o que foi digitado no Monitor Serial do Arduino e envie ao HC-05
  if (Serial.available()){
    digitalWrite(13, HIGH);
    BTSerial.write(Serial.read());
    digitalWrite(13, LOW);
  }
}
