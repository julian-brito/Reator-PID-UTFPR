//Controle de PID de temperatura arduino versao 1.8
#include <PID_v2.h>  // biblioteca PID
#include <WiFi.h> // Recursos de WiFi
#include <ThingSpeak.h> // Recursos e comunicação com nuvem ThingSpeak
#include <driver/mcpwm.h>
#include <HardwareSerial.h>
#include <PubSubClient.h>


// MQTT Configuração

// Definições do Broker MQTT
/* URL do broker MQTT que deseja utilizar */
const char* BROKER_MQTT = "broker.hivemq.com";
/* Porta do Broker MQTT */
int BROKER_PORT = 1883;
#define ID_MQTT  "ESP32_JJB"

HardwareSerial SerialPort(2); // use UART2

typedef struct __attribute__((packed)) {
  float inputnivel; // Adicione os campos necessários para os dados do sensor
  float temperatura;
  float nivel_min;
  float nivel_max;
} SensorData;

SensorData sensorData; // Crie uma instância da estrutura SensorData


// setting PWM properties
const int freq = 50;
const int ledChannel_b5 = 1; // valor fixo
const int ledChannel_b6 = 2; // valor fixo
const int ledChannel_b9 = 3; // valor fixo
const int ledChannel_motor = 15; // valor fixo
const int resolution = 8; // valor fixo
int freq_motor_passo = 65;

float temperatura_wifi = 0;
float nivel_wifi = 0;
float pwmtemperatura_wifi = 0;
float pwmnivel_wifi = 0;


//PINAGEM DO SISTEMA////////////////////////////////////////////////////////////////////
//Pinagem para as bombas
const int pinoTemp = 32; // pinagem 2 do esp - bomba que ira controlar a temperatura
const int PinoSaida = 4; // pinagem 4 do esp - bomba que ira controlar o nível
const int PinoEntrada = 14; // pinagem 14 do esp - bomba que ira alimentar constatemente
const int PinoRele = 15; // pinagem 12 do esp - relé de segurança que irá acionar a resistência a um determinado nível
const int pinoNivel = 35;
const int pinoNivelMax = 33;
const int pinoMotorPasso = 5;

//DEFININDO VARIAVEIS GLOBAIS NECESSARIAS////////////////////////////////////////////
//primeira letra minuscula, se houver segunda palavra colocar maiuscula a 1ª letra

double T_temp = 25;
double temp = 0;

// Controle nível //
double nivel_sp, inputnivel, outputnivel;
double kpn = 200, kin = 15, kdn = 2; //constantes kp ki kd de nível

//Controle Temperatura
double temperatura_sp, inputtemperatura, outputtemperatura;
double kpt = 150, kit = 45, kdt = 4;

double offsetMedio = 9218170; //8788978; //8517054; //8816715; //8388608; //9178247;

// Sensor nível
int nivelPontual = 0;
boolean TemNivel = false;

int nivelPontualMax = 0;
boolean nivelMax = false;

//valor da vazao de entrada
double PWM_entrada = 11; //Essa se mantém constante o tempo todo

//INSTANCIANDO OBJETOS NECESSARIOS////////////////////////////////////////////////////

//Objetos PID para o nivel e para a temperatura
PID controlenivel(&inputnivel, &outputnivel, &nivel_sp, kpn, kin, kdn, REVERSE);
PID controletempeatura(&inputtemperatura, &outputtemperatura, &temperatura_sp, kpt, kit, kdt, REVERSE);
//Instanciando objeto HX710 do sensor de pressao
//HX710 ps;
//Q2HX711 preson(DOUT,PD_SCK);


// Definindo variáveis e constantes globais ===================================
//
const char* ssid = "REATOR-UTFPR"; //"UTFPR-VISITANTE"; // Nome e senha para conexão
const char* password = ""; // com rede WiFi
const long CHANNEL = 2068593; // Parâmetros de acesso ao canal
const char *WRITE_API = "VJ4JWA7AGQXO1S30"; // específico na nuvem ThingSpeak:
const char *READ_API = "8SITM1BT0PXT2BI1"; // Canal e chaves escrita/leitura
int intervalSensor = 1500; // Parâmetros para definição de intervalo
long prevMillisThingSpeak = 0; // mínimo entre duas escritas no ThingSpeak,
int intervalThingSpeak = 15000; // que deve ser de 15s para contas free
int erro; // Guarda valor de erro de escrita no ThingSpeak
WiFiClient client; // Objeto para operações via WiFi
PubSubClient MQTT(client);
//
void ConectaWiFi() {
  // Caso ainda não conectado à rede WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(); Serial.println();
    Serial.print("(Re)Conectando à rede WiFi "); Serial.print(ssid);
    Serial.print(". Aguarde");
    // Tentando iniciar a conexão
    WiFi.begin(ssid, password);
    // Aguardando a conexão ser efetivada
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000); Serial.print(".");
    }
    Serial.println(", WiFi Conectado!"); Serial.println();
  }
  // Após conectado à rede WiFi
  //  Serial.println();
  //  Serial.print("Conectado à "); Serial.print(ssid);
  //  Serial.print(", IP:"); Serial.print(WiFi.localIP()); Serial.println(".");
  //  Serial.println();
}
//

void configura_velocidade_agitador(int rpm)
{
  // configuração do pino de saída PWM
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, pinoMotorPasso);

  // configuração do canal MCPWM 0 em modo UP
  mcpwm_config_t pwm_config;
  pwm_config.frequency = int(rpm * 200 / 60); // frequência de 1 kHz
  pwm_config.cmpr_a = 50.0;       // duty cycle de 50%
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void init_mqtt(void)
{
  /* informa a qual broker e porta deve ser conectado */
  MQTT.setServer(BROKER_MQTT, BROKER_PORT);
  /* atribui função de callback (função chamada quando qualquer informação do
    tópico subescrito chega) */
  MQTT.setCallback(mqtt_callback);
}


void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
  String msg;

  //obtem a string do payload recebido
  for (int i = 0; i < length; i++)
  {

    char c = (char)payload[i];
    if (!(isdigit(c) or c == '.'))
    {
      // se não for um número, retorna sem fazer nada
      return;
    }
    msg += c;
  }
  Serial.print("[MQTT] - "); Serial.print(topic); Serial.print(" Mensagem recebida: "); // add a space before "Mensagem"
  Serial.println(msg);

  if (strcmp(topic, "UTFPR/REATOR_PID/SP_ALTURA") == 0) {
    //fazer algo com a mensagem recebida em SP_ALTURA
    // Atualiza o PID de nível//
    nivel_sp = atof(msg.c_str()); //cm  // Setpoint nível
    controlenivel.SetMode(AUTOMATIC);
    controlenivel.SetTunings(kpn, kin, kdn);
  }
  else if (strcmp(topic, "UTFPR/REATOR_PID/OFFSET_ALTURA") == 0) {
    //atualiza OFFSET_ALTURA
    offsetMedio = atof(msg.c_str());
  }
  else if (strcmp(topic, "UTFPR/REATOR_PID/PWM_BOMBA_06_NIVEL") == 0) {
    //fazer algo com a mensagem recebida em PWM_BOMBA_06_NIVEL
  }
  else if (strcmp(topic, "UTFPR/REATOR_PID/SP_TEMPERATURA") == 0) {
    //fazer algo com a mensagem recebida em SP_TEMPERATURA
    temperatura_sp = atof(msg.c_str()); //cm  // Setpoint temperatura
    controletempeatura.SetMode(AUTOMATIC);
    controletempeatura.SetTunings(kpt, kit, kdt);
  }
  else if (strcmp(topic, "UTFPR/REATOR_PID/PWM_BOMBA_09_TEMPERATURA") == 0) {
    //fazer algo com a mensagem recebida em PWM_BOMBA_09_TEMPERATURA
  }
  else if (strcmp(topic, "UTFPR/REATOR_PID/RPM_MOTOR_PASSO") == 0) {
    freq_motor_passo = atoi(msg.c_str());
    configura_velocidade_agitador(freq_motor_passo);
  }
  else if (strcmp(topic, "UTFPR/REATOR_PID/KP_NIVEL") == 0) {
    //fazer algo com a mensagem recebida em KP_NIVEL
    kpn = atof(msg.c_str());
    controlenivel.SetTunings(kpn, kin, kdn);
  }
  else if (strcmp(topic, "UTFPR/REATOR_PID/KI_NIVEL") == 0) {
    //fazer algo com a mensagem recebida em KI_NIVEL
    kin = atof(msg.c_str());
    controlenivel.SetTunings(kpn, kin, kdn);
  }
  else if (strcmp(topic, "UTFPR/REATOR_PID/KD_NIVEL") == 0) {
    //fazer algo com a mensagem recebida em KI_NIVEL
    kdn = atof(msg.c_str());
    controlenivel.SetTunings(kpn, kin, kdn);
  }
  else if (strcmp(topic, "UTFPR/REATOR_PID/KP_TEMPERATURA") == 0) {
    //fazer algo com a mensagem recebida em KP_NIVEL
    kpt = atof(msg.c_str());
    controletempeatura.SetTunings(kpt, kit, kdt);
  }
  else if (strcmp(topic, "UTFPR/REATOR_PID/KI_TEMPERATURA") == 0) {
    //fazer algo com a mensagem recebida em KI_NIVEL
    kit = atof(msg.c_str());
    controletempeatura.SetTunings(kpt, kit, kdt);
  }
  else if (strcmp(topic, "UTFPR/REATOR_PID/KD_TEMPERATURA") == 0) {
    //fazer algo com a mensagem recebida em KI_NIVEL
    kdt = atof(msg.c_str());
    controletempeatura.SetTunings(kpt, kit, kdt);
  }
}


void reconnect_mqtt(void)
{
  while (!MQTT.connected())
  {
    Serial.print("* Tentando se conectar ao Broker MQTT: ");
    Serial.println(BROKER_MQTT);
    if (MQTT.connect(ID_MQTT))
    {
      Serial.println("Conectado com sucesso ao broker MQTT!");
      MQTT.subscribe("UTFPR/REATOR_PID/SP_ALTURA");
      MQTT.subscribe("UTFPR/REATOR_PID/OFFSET_ALTURA");
      MQTT.subscribe("UTFPR/REATOR_PID/PWM_BOMBA_06_NIVEL");
      MQTT.subscribe("UTFPR/REATOR_PID/SP_TEMPERATURA");
      MQTT.subscribe("UTFPR/REATOR_PID/PWM_BOMBA_09_TEMPERATURA");
      MQTT.subscribe("UTFPR/REATOR_PID/RPM_MOTOR_PASSO");
      MQTT.subscribe("UTFPR/REATOR_PID/KP_NIVEL");
      MQTT.subscribe("UTFPR/REATOR_PID/KI_NIVEL");
      MQTT.subscribe("UTFPR/REATOR_PID/KD_NIVEL");
      MQTT.subscribe("UTFPR/REATOR_PID/KP_TEMPERATURA");
      MQTT.subscribe("UTFPR/REATOR_PID/KI_TEMPERATURA");
      MQTT.subscribe("UTFPR/REATOR_PID/KD_TEMPERATURA");
      MQTT.subscribe("UTFPR/REATOR_PID/MODO_CONTROLE");
    }
    else
    {
      Serial.println("Falha ao reconectar no broker.");
      Serial.println("Havera nova tentatica de conexao em 2s");
      delay(2000);
    }
  }
}

void verifica_conexoes_mqtt(void)
{
  /* se não há conexão com o Broker, a conexão é refeita */
  if (!MQTT.connected())
    reconnect_mqtt();
}

void publica_informacoes()
{
  MQTT.publish("UTFPR/REATOR_PID/ALTURA", String(inputnivel).c_str());
  MQTT.publish("UTFPR/REATOR_PID/SP_ALTURA_VA", String(nivel_sp).c_str());
  MQTT.publish("UTFPR/REATOR_PID/PWM_BOMBA_06_NIVEL_VA", String(outputnivel).c_str());
  MQTT.publish("UTFPR/REATOR_PID/TEMPERATURA", String(inputtemperatura).c_str());
  MQTT.publish("UTFPR/REATOR_PID/SP_TEMPERATURA_VA", String(temperatura_sp).c_str());
  MQTT.publish("UTFPR/REATOR_PID/PWM_BOMBA_09_TEMPERATURA_VA", String(outputtemperatura).c_str());
  MQTT.publish("UTFPR/REATOR_PID/NIVEL_MIN", String(TemNivel).c_str());
  MQTT.publish("UTFPR/REATOR_PID/NIVEL_MAX", String(nivelMax).c_str());
  MQTT.publish("UTFPR/REATOR_PID/RPM_MOTOR_PASSO_VA", String(freq_motor_passo).c_str());
  MQTT.publish("UTFPR/REATOR_PID/KP_NIVEL_VA", String(kpn).c_str());
  MQTT.publish("UTFPR/REATOR_PID/KI_NIVEL_VA", String(kin).c_str());
  MQTT.publish("UTFPR/REATOR_PID/KD_NIVEL_VA", String(kdn).c_str());
  MQTT.publish("UTFPR/REATOR_PID/KP_TEMPERATURA_VA", String(kpt).c_str());
  MQTT.publish("UTFPR/REATOR_PID/KI_TEMPERATURA_VA", String(kit).c_str());
  MQTT.publish("UTFPR/REATOR_PID/KD_TEMPERATURA_VA", String(kdt).c_str());
}

void task1(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // chamar a função aqui
    String variaveis[] = {"ALTURA", "SP_ALTURA", "OFFSET_ALTURA", "PWM_BOMBA_06_NIVEL",
                          "TEMPERATURA", "SP_TEMPERATURA", "PWM_BOMBA_09_TEMPERATURA", "NIVEL_MIN",
                          "NIVEL_MAX", "RPM_MOTOR_PASSO", "KP NIVEL", "KI NIVEL", "KD NIVEL", "KP TEMPERATURA", "KI TEMPERATURA", "KD TEMPERATURA"
                         };
    double valores[] = {inputnivel, nivel_sp, offsetMedio, outputnivel, inputtemperatura, temperatura_sp,
                        outputtemperatura, TemNivel, nivelMax, freq_motor_passo, kpn, kin, kdn, kpt, kit, kdt
                       };
    printInformacoes(variaveis, valores);
    publica_informacoes();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 17, 16); // Configura a porta serial 1 (TX: pino 17, RX: pino 16)

  pinMode(PinoRele, OUTPUT);
  digitalWrite(PinoRele, LOW);

  ConectaWiFi();
  Serial.println();
  Serial.print("Conectado à "); Serial.print(ssid);
  Serial.print(", IP:"); Serial.print(WiFi.localIP()); Serial.println(".");
  Serial.println();
  delay(2000);

  ThingSpeak.begin(client);
  init_mqtt();

  analogReadResolution(11); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
  analogSetAttenuation(ADC_6db); // Default is 11db which is very noisy. Recommended to use 2.5 or 6.

  // Setup PWM configurações do acionamento
  // pinoTemp - bomba que ira controlar a temperatura
  ledcSetup(ledChannel_b9, freq, resolution);
  ledcAttachPin(pinoTemp, ledChannel_b9);

  // PinoSaida - bomba que ira controlar o nível
  ledcSetup(ledChannel_b6, freq, resolution);
  ledcAttachPin(PinoSaida, ledChannel_b6);

  // PinoSaida - bomba que ira bomba que ira alimentar constatemente
  ledcSetup(ledChannel_b5, freq, resolution);
  ledcAttachPin(PinoEntrada, ledChannel_b5);


  // inicia o PID de nível//
  nivel_sp = 5; //cm  // Setpoint nível
  controlenivel.SetMode(AUTOMATIC);
  controlenivel.SetTunings(kpn, kin, kdn);

  temperatura_sp = 30; // Setpoint temperatura
  controletempeatura.SetMode(AUTOMATIC);
  controletempeatura.SetTunings(kpt, kit, kdt);

  configura_velocidade_agitador(60);

  // ativa task que imprimi as informações a cada 1 segundo
  xTaskCreate(task1, "Task 1", 10000, NULL, 1, NULL);


}

void loop() {
  ConectaWiFi();  //para tirar o modo wifi só comentar a linha inteira
  verifica_conexoes_mqtt();
  MQTT.loop();

  if (Serial1.available() >= sizeof(sensorData))
  { // Verifica se há dados disponíveis na porta serial 1
    Serial1.readBytes((uint8_t*)&sensorData, sizeof(sensorData)); // Lê os dados disponíveis na porta serial 1 e armazena na estrutura sensorData
  }

  inputnivel = 0.0000156 * (sensorData.inputnivel - offsetMedio); // + 0.562;


  controlenivel.Compute(); // executa o controle de nivel

  //controle temperatura//
  temp = sensorData.temperatura;
  //FILTRO PROS VALORES DE TEMPERATURA
  if (temp == -127) {
    inputtemperatura = T_temp;
  }
  else {
    inputtemperatura = temp;
  }

  T_temp = inputtemperatura;

  if (inputnivel > 4 and TemNivel and temp < 55)
  { //Verifica  se o nivel de agua ta cobrindo o rabo quente
    digitalWrite(PinoRele, HIGH);    // Se sim, liga o relé que liga o rabo quente
  }
  else
  {
    digitalWrite(PinoRele, LOW);
  }

  controletempeatura.Compute();  // executa o controle de temperatura

  // Verifica nível máximo antes de manter a entrada ligada
  if (nivelMax)
  {
    //Desliga bomba 05
    ledcWrite(ledChannel_b5, 0);

    //Desliga bomba 09 - controle temperatura
    ledcWrite(ledChannel_b9, 0);

    //Desliga bomba 06 - retirada de água, pode ser que a mangueira esteja invertida.
    ledcWrite(ledChannel_b9, 0);
  }
  else
  {
    //Liga bomba 05
    ledcWrite(ledChannel_b5, PWM_entrada);
    //Envia ação de controle para controlar o TEMPERATURA pino pinagem  12
    ledcWrite(ledChannel_b9, outputtemperatura);
  }

  ledcWrite(ledChannel_b6, outputnivel);      // envia ação de controle para controlar o nivel pino saida= pinagem 14

  // lê nível pontual
  nivelPontual = analogReadMilliVolts(pinoNivel);
  if (nivelPontual > 800)
  {
    TemNivel = false; // Se sim, liga o relé que liga o rabo quente
  }
  else
  {
    TemNivel = true;
  }

  // lê nível pontual maximo
  nivelPontualMax = analogReadMilliVolts(pinoNivelMax);
  if (nivelPontualMax > 800)
  {
    nivelMax = false; // Se sim, liga o relé que liga o rabo quente
  }
  else
  {
    nivelMax = true;
  }
  // declara valores para enviar info à nuvem Thingspeak
  temperatura_wifi = inputtemperatura;
  nivel_wifi = inputnivel;
  pwmtemperatura_wifi = outputtemperatura;
  pwmnivel_wifi = outputnivel;
  //
  //printa dados

  String variaveis[] = {"ALTURA", "SP_ALTURA", "OFFSET_ALTURA", "PWM_BOMBA_06_NIVEL",
                        "TEMPERATURA", "SP_TEMPERATURA", "PWM_BOMBA_09_TEMPERATURA", "NIVEL_MIN",
                        "NIVEL_MAX", "RPM_MOTOR_PASSO", "KP NIVEL", "KI NIVEL", "KD NIVEL", "KP TEMPERATURA", "KI TEMPERATURA", "KD TEMPERATURA"
                       };
  double valores[] = {inputnivel, nivel_sp, offsetMedio, outputnivel, inputtemperatura, temperatura_sp,
                      outputtemperatura, TemNivel, nivelMax, freq_motor_passo, kpn, kin, kdn, kpt, kit, kdt
                     };



  //printInformacoes(variaveis, valores);
  //  print_wifi();

}

void printInformacoes(const String* varNames, const double* varValues)
{
  // Identifica o número de variáveis
  int numVars = 16;

  // LIMPA A TELA
  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
  Serial.write(27);
  Serial.print("[H");     // cursor to home command

  // Imprime as informações
  for (int i = 0; i < numVars; i++) {
    Serial.print(varNames[i]);
    Serial.print(": ");
    Serial.println(varValues[i]);
  }
}



void print_wifi()
{

  if (millis() - prevMillisThingSpeak > intervalThingSpeak)
  {
    ThingSpeak.setField(1, temperatura_wifi);
    ThingSpeak.setField(2, nivel_wifi);
    ThingSpeak.setField(3, pwmtemperatura_wifi);
    ThingSpeak.setField(4, pwmnivel_wifi);

    // Configurando campo 1 com valor lido
    erro = ThingSpeak.writeFields(CHANNEL, WRITE_API); // Escrevendo no canal
    // Informando se ocorreu escrita com sucesso ou algum erro
    if (erro == 200)
    {
      Serial.println(" >> Update realizado com sucesso!");
    }
    else
    {
      Serial.println(" >> Problema no canal - erro HTTP " + String(erro));
    }
    prevMillisThingSpeak = millis();
  }
}
