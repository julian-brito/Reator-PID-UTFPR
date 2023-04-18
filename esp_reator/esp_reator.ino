#include <Q2HX711.h> //biblioteca do sensor de nivel
#include <OneWire.h> // biblioteca sensor de temperatura
#include <DallasTemperature.h> // biblioteca sensor de temperatura


//PINAGEM DO SISTEMA////////////////////////////////////////////////////////////////////
//Pinagem para o sensor de pressão
const int DOUT = 19;  // sinal DT do amplificador do sensor de pressao
const int PD_SCK = 18; // sinal SCK do amplificador do sensor de pressao

// Sensor Temperatura
#define ONE_WIRE_BUS 13 // recebera o sinal do sensor de temperatura 

// Instanciando objetos
// Sensor de pressão
Q2HX711 preson(DOUT, PD_SCK);
//comunicacao um fio do sensor de temperatura
OneWire oneWire(ONE_WIRE_BUS);
//sensores de temperatura
DallasTemperature sensors(&oneWire);

typedef struct __attribute__((packed)) {
  float inputnivel; 
  float temperatura;
} SensorData;

SensorData sensorData; // Crie uma instância da estrutura SensorData

void setup() {
  
  Serial.begin(115200);
  
  Serial1.begin(115200, SERIAL_8N1, 17, 16); // Configura a porta serial 1 (TX: pino 17, RX: pino 16)

  sensors.begin();
}

void loop() {
  
  sensorData.inputnivel = preson.read(); // Atribua o valor do sensor ao campo correto da estrutura

  sensors.requestTemperatures();
  sensorData.temperatura = sensors.getTempCByIndex(0);;

  // Enviando os dados via porta serial
  Serial1.write((uint8_t*)&sensorData, sizeof(sensorData));

  Serial.print("Leitura do sensor: ");
  Serial.println(sensorData.inputnivel);
  Serial.print("Leitura do sensor temperatura: ");
  Serial.println(sensorData.temperatura);

  delay(100);
}
