
// /univap/proj/pulse

#include <WiFi.h>
#include <PubSubClient.h>

const int PulseSensorPurplePin = 34; // Pulse Sensor WIRE connected to Pin 34
const int TemperaturePin = A0;
int LED = 2;   //  The on-board ESP32 LED
unsigned int loop_main = 0;

unsigned long time_anterior, time_atual, time_total;
char toggle = 0;

const char* SSID = "";
const char* PASSWORD = "";
const char* BROKER_MQTT = "broker.hivemq.com";
const int BROKER_PORT = 1883;
const char* ID_MQTT = "pulse";

char msg[32];

WiFiClient espClient;
PubSubClient mqtt(espClient);
 
int Signal; // holds the incoming raw data. Signal value can range from 0-1024
int Threshold = 2000; // Determine which Signal to "count as a beat", and which to ingore.
int RawValue;
double Voltage = 0;
double tempC = 0;
double tempF = 0;
double BPM;

void wifi_connect(void)
{
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(SSID);

  WiFi.begin(SSID, PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  if(WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi not connected");
  }else
  {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
  }
}

void initMQTT(void) 
{
    mqtt.setServer(BROKER_MQTT, BROKER_PORT);   //informa qual broker e porta deve ser conectado
    mqtt.setCallback(mqttCallback);            //atribui função de callback (função chamada quando qualquer informação de um dos tópicos subescritos chega)
}

void connectMQTT(void) 
{
    while (!mqtt.connected()) 
    {
        Serial.print("* Tentando se conectar ao Broker MQTT: ");
        Serial.println(BROKER_MQTT);
        if (mqtt.connect(ID_MQTT)) 
        {
            Serial.println("Conectado com sucesso ao broker MQTT!");
        } 
        else 
        {
            Serial.println("Falha ao reconectar no broker.");
            Serial.println("Havera nova tentatica de conexao em 2s");
            delay(2000);
        }
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) 
{
    char c;
    String msg;
    
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("]: ");
    Serial.write(payload, length);
    Serial.println();

    //obtem a string do payload recebido
    for(int i = 0; i < length; i++) 
    {
       c = (char)payload[i];
       msg += c;
    }
    
    //avalia se a mensagem é para este NodeMCU
    if (msg.equals("1"))
    {
      Serial.println("");   
    }

    if (msg.equals("0"))
    {
      Serial.println("");   
    } 
}

void reconectWiFi() 
{
    //se já está conectado a rede WI-FI, nada é feito. 
    //Caso contrário, são efetuadas tentativas de conexão
    if (WiFi.status() == WL_CONNECTED)
        return;
        
    WiFi.begin(SSID, PASSWORD); // Conecta na rede WI-FI
    
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(100);
        Serial.print(".");
    }
  
    Serial.println();
    Serial.print("Reconectado com sucesso na rede ");
    Serial.println(SSID);
    Serial.print("IP obtido: ");
    Serial.println(WiFi.localIP());
}

void checkConnections(void)
{
  if (!(WiFi.status() == WL_CONNECTED))
  {

    Serial.print("Lost WiFi!");
    
    reconectWiFi();
  }
  
  if(mqtt.connected())
  {
    Serial.print("            ");
    return;
  }
    else
        {
          Serial.println("Lost Server!");
      
          connectMQTT();
        }
}
 
// The SetUp Function:
void setup() {
  pinMode(LED,OUTPUT); // pin that will blink to your heartbeat
  Serial.begin(115200); // Set's up Serial Communication at certain speed.
  wifi_connect();
  initMQTT();
  connectMQTT();
}

// The Main Loop Function
void loop() {
checkConnections();
  Signal = analogRead(PulseSensorPurplePin);  // Read the PulseSensor's value.
                                              // Assign this value to the "Signal" variable.
 
   Serial.println(Signal);                    // Send the Signal value to Serial Plotter.
 
 
 
   if(Signal > Threshold){   // If the signal is above "2400", then "turn-on" ESP32's on-Board LED.
     digitalWrite(LED,HIGH);
     if(toggle)
     {
      time_anterior = time_atual;
      time_atual = millis();
     }
     toggle = 0;
     
   } else {
     digitalWrite(LED,LOW); //  Else, the sigal must be below "2400", so "turn-off" this LED.
     toggle = 1;
   }
 
loop_main++;
if(loop_main > 200)
{
  RawValue = analogRead(TemperaturePin);
  Voltage = (RawValue / 2048.0) * 3300; // 5000 to get millivots.
  tempC = Voltage * 0.1;
  tempF = (tempC * 1.8) + 32; // conver to F
  loop_main = 0;
  //Serial.print("mili volt: ");
  //Serial.println(Voltage);
  //Serial.print("Temp: ");
  //Serial.println(tempC);
  //snprintf(msg, 8, "%3d", (int)tempC);
  //mqtt.publish("/univap/proj/pulse", msg);

  //if(tempC > 37) mqtt.publish("/univap/cti/relogio/alarme/", "1");else
  //mqtt.publish("/univap/cti/relogio/alarme/", "0");
  
  if(time_atual > time_anterior) time_total = time_atual - time_anterior;
  if(time_anterior > time_atual) time_total = time_anterior - time_atual;

  BPM = (double)60000/time_total;
  if(BPM < 50.0 || BPM > 150) BPM = 0;
  //Serial.print("BPM: ");
  //Serial.println((int)BPM);
  snprintf(msg, 8, "%3d", (int)BPM);
  mqtt.publish("/univap/proj/pulse", msg);
  //Serial.print("Tempo1: ");
  //Serial.println(time_anterior);
  //Serial.print("Tempo2: ");
  //Serial.println(time_atual);
  //Serial.print("Tempo: ");
  //Serial.println(time_total);
}
delay(10); // too higher delay will 

mqtt.loop();
}
