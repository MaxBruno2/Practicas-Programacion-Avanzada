#include <WiFi.h>

TaskHandle_t Task1;
TaskHandle_t Task2;

WiFiClient localClient;

const char* ssid = "IZZI-B204";
const char* password = "eacyPzrxHG74Zrkdbb";

const uint port = 4062;
const char* ip = "3.149.222.108";

// Alias de pines
const int BotonPin      = 13;
const int AtrasPin      = 25;
const int PausaPin      = 26;
const int AdelantePin   = 27;

const int EjeXPin       = 36;
const int EjeYPin       = 39;
const int PotPin        = 34;

void setup() {
  Serial.begin(115200);

  pinMode(2, OUTPUT);
  
  pinMode(13, INPUT_PULLUP);
  pinMode(33, INPUT_PULLUP);
  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);

  Serial.println("Conectando a Internet");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.localIP());


  // Tarea a ejecutarse en el core 0. Enviar mensajes al servidor.
  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Task1",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task1,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */

  delay(500);

  //c Tarea a ejecutarse en el core 1. Recibir mensajes del servidor.
  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Task2",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */

  delay(500);
}

void loop() {
  if (!localClient.connected()) {
    Serial.println("Conectando al servidor");
    while (!localClient.connect(ip, port)) {
      Serial.print(".");
      delay(500);
    }
    localClient.print("<name>ESP32");
  }
}



// Tarea 1. Mandar mensajes al server
void Task1code(void* pvParameters) {
  Serial.print("Tarea 1 corriendo en el core ");
  Serial.println(xPortGetCoreID());
  
  while (1) {
    if (localClient.connected()) {
      // Lecturas digitales
      int boton    = digitalRead(BotonPin);
      int atras    = digitalRead(AtrasPin);
      int pausa    = digitalRead(PausaPin);
      int adelante = digitalRead(AdelantePin);

      // Lecturas analógicas
      int ejeX        = analogRead(EjeXPin);
      int ejeY        = analogRead(EjeYPin);
      int potenciometro = analogRead(PotPin);

      // Enviar al servidor
      localClient.println("<boton>" + String(boton));
      localClient.println("<atras>" + String(atras));
      localClient.println("<pausa>" + String(pausa));
      localClient.println("<adelante>" + String(adelante));

      localClient.println("<ejeX>" + String(ejeX));
      localClient.println("<ejeY>" + String(ejeY));
      localClient.println("<pot>" + String(potenciometro));
    }
    delay(500);
  }
}

// Tarea 2. Esperar mensajes del server
void Task2code(void* pvParameters) {
  Serial.print("Tarea 2 corriendo en el core ");
  Serial.println(xPortGetCoreID());

  while (1) {
    if (localClient.connected()) {
      while (!localClient.available());
      String str = localClient.readStringUntil('\n');
      if(str.startsWith("<digital_write>on")) {
        pinMode(2, OUTPUT);
        digitalWrite(2, HIGH);
      } else if(str.startsWith("<digital_write>off")) {
        pinMode(2, OUTPUT);
        digitalWrite(2, LOW);
      } else if(str.startsWith("<analog_write>")) { // <analog_write>50
        int valor = str.substring(14).toInt();
        analogWrite(2, valor);
      } else {
        Serial.println(str);
      }
    }
    //delay(100);
  }
}

