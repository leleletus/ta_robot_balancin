#include <Arduino.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <XSpaceV21.h>
#include <XSpaceIoT.h>
//#include <XSControl.h> // Required for motor control functions
#include "WiFi.h"
#include "wifi_credentials.h" // Incluir el archivo de credenciales
#include <BluetoothSerial.h>

bool recording = true; // Variable de control para la grabación


// Configuración de la placa y variables
XSpaceV21Board OBJ;
//XSThing IOT;

int Ts = 10; // Tiempo de muestreo en milisegundos
double u = 0; // Voltaje de entrada
double setpoint=0; // Valor recibido para sp
int pin1 = 0; // Pin 1 para acelerómetro
int pin2 = 5; // Pin 2 para giroscopio
double VM = 5.0; // Voltaje máximo del driver
float ax, ay, az; // Variables para aceleración
float gx, gy, gz; // Variables para giroscopio
double inclinacion; // Ángulo de inclinación
double vel_M1, vel_M2, pos_M1, pos_M2; // Velocidad y posición angular
int k = 0; // Inicialización global


BluetoothSerial SerialBT;
uint16_t  joystickY = 512;
String nombreDispositivo;

float remap(int x, int in_min, int in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Función para generar un nombre único basado en la dirección MAC
String genera_nombre_unico() {
  // Obtén la dirección MAC del ESP32
  String macAddress = WiFi.macAddress();

  // Extraer los últimos 2 bytes de la dirección MAC como sufijo
  String suffix = macAddress.substring(macAddress.length() - 5);  // Últimos 2 bytes de la MAC
  suffix.replace(":", "");  // Elimina los dos puntos (:) de la dirección MAC

  // Generar el nombre único para el dispositivo
  return "Balancin_" + suffix;
}

// Función para inicializar el Bluetooth con el nombre generado
void configurarBluetooth() {
  nombreDispositivo = genera_nombre_unico();
  SerialBT.begin(nombreDispositivo);  
  Serial.println("Bluetooth Iniciado con el nombre: " + nombreDispositivo);
}

// Función para recibir la posición del joystick
void recibirdatos() { // al enviar 0x01F401F4 recibo F4 01 F4 01 little-endian
  if (SerialBT.available()) { // Verificar si hay disponibles
    // uint8_t datos[2]; // para guardar los datos de 8 bits o sea 2 serian para x y 2 para y
    // SerialBT.readBytes(datos, 2); // Leer los 2 bytes
    // joystickY = (datos[1] << 8) | datos[0]; // Little-endian
    SerialBT.readBytes((char*)&joystickY, sizeof(joystickY)); // Lee 2 bytes y los convierte en un número
    //Serial.print(" Y: ");
    //Serial.println(joystickY);
  }
}


// Función para conexión WiFi
void conectar_wifi() {
  Serial.println("Escaneando redes WiFi...");
  int n = WiFi.scanNetworks();
  if (n == 0) {
    Serial.println("No se encontraron redes.");
  } else {
    Serial.print(n);
    Serial.println(" redes encontradas:");
  }

  bool connected = false;
  for (int i = 0; i < numNetworks; ++i) {
    for (int j = 0; j < n; ++j) {
      if (WiFi.SSID(j) == ssidList[i]) {
        Serial.print("Conectando a ");
        Serial.println(ssidList[i]);
        WiFi.begin(ssidList[i], passwordList[i]);
        int attempts = 10;
        while (WiFi.status() != WL_CONNECTED && attempts > 0) {
          delay(1000);
          Serial.print(".");
          attempts--;
        }
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("\nConectado.");
          connected = true;
          break;
        }
      }
    }
    if (connected) break;
  }

  if (!connected) {
    Serial.println("No se pudo conectar a ninguna red específica.");
  }

  Serial.print("IP del servidor local: ");
  Serial.println(WiFi.localIP());
}

// Función para recibir datos desde MQTT
void ref_sp(char* topicx, byte* Data, unsigned int DataLen) {
  String RecivedData = String((char*)Data, DataLen);
  String Topic = String((char*)topicx);

  if (Topic == "control/ref") {
    setpoint = RecivedData.toDouble(); // Pasamos de string a double
  }
}

// Tarea combinada para captura y envío de datos de aceleración, giroscopio, velocidad y posición angular
void tarea_1(void *pvParameters) {
  while (1) {
    // **IMU** - Captura de datos de aceleración y giroscopio
    OBJ.BMI088_GetAccelData(&ax, &ay, &az);
    OBJ.BMI088_GetGyroData(&gx, &gy, &gz);
    inclinacion = OBJ.BMI088_GetPitch_Accel();

    // Enviar datos de IMU a MQTT
    //IOT.Mqtt_Publish("sensor/inclinacion", inclinacion);
    //IOT.Mqtt_Publish("sensor/acceleration", String(ax) + "," + String(ay) + "," + String(az));
    //IOT.Mqtt_Publish("sensor/gyroscope", String(gx) + "," + String(gy) + "," + String(gz));

    //recibirdatos();
    // **Encoders y Control de Motores** - Captura de datos de velocidad y posición
    //setpoint = remap(joystickY, 0, 1024, -1*VM, VM);  // Actualizar el voltaje de entrada según el setpoint recibido
    u=setpoint;
    OBJ.DRV8837_Voltage(DRVx1, u);
    OBJ.DRV8837_Voltage(DRVx2, u);
    //IOT.Mqtt_Publish("sensor/u", u);

    vel_M1 = OBJ.GetEncoderSpeed(E1, DEGREES_PER_SECOND);
    pos_M1 = OBJ.GetEncoderPosition(E1, DEGREES);
    vel_M2 = OBJ.GetEncoderSpeed(E2, DEGREES_PER_SECOND);
    pos_M2 = OBJ.GetEncoderPosition(E2, DEGREES);


    // Serial.print("Voltaje : ");
    // Serial.println(u);
    // // Mostrar datos de IMU en Serial
    // Serial.print("Inclinación: "); 
    // Serial.println(inclinacion);
    

    // // Mostrar datos de velocidad y posición en Serial
    // Serial.print("Velocidad_M1:  Posición_M1:");
    // Serial.print(vel_M1); Serial.print("\t"); Serial.println(pos_M1);
    
    // // Mostrar datos de velocidad y posición en Serial
    // Serial.print("Velocidad_M2:  Posición_M2:");
    // Serial.print(vel_M2); Serial.print("\t"); Serial.println(pos_M2);

    // Serial.print("Aceleración [x, y, z]:");
    // Serial.print("\t"); Serial.print(ax); Serial.print("\t"); Serial.print(ay); Serial.print("\t"); Serial.println(az);
    // Serial.print("Giroscopio [x, y, z]:");
    // Serial.print("\t"); Serial.print(gx); Serial.print("\t"); Serial.print(gy); Serial.print("\t"); Serial.println(gz);

    // Enviar datos de velocidad y posición a MQTT
    // IOT.Mqtt_Publish("motor/velocidad_M1", vel_M1);
    // IOT.Mqtt_Publish("motor/posicion_M1", pos_M1);
    // IOT.Mqtt_Publish("motor/velocidad_M2", vel_M2);
    // IOT.Mqtt_Publish("motor/posicion_M2", pos_M2);

    // Verificar si hay nueva información publicada en el buffer MQTT
    //IOT.Mqtt_CheckBuffer();

    

    // Delay según el tiempo de muestreo
    vTaskDelay(Ts);
  }

  // Eliminar tarea en caso de salida del bucle (opcional, usualmente innecesario en FreeRTOS)
  vTaskDelete(NULL);
}


void log_datos() {
  // Leer comandos del puerto serie
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'n') {
      recording = false;
      Serial.println("Grabación detenida");
    } else if (command == 'r') {
      Serial.println("Mostrando datos:");
      File file = SPIFFS.open("/datalog.txt", FILE_READ);
      if (file) {
        while (file.available()) {
          Serial.write(file.read());
        }
        file.close();
      } else {
        Serial.println("Error al abrir el archivo para lectura");
      }
    }
  }

  // Grabar datos si la grabación está habilitada
  if (recording) {
    File file = SPIFFS.open("/datalog.txt", FILE_APPEND);
    if (file) {
      file.print("Voltaje : ");
      file.println(u);
      // Mostrar datos de IMU en Serial
      file.print("Inclinación: "); 
      file.println(inclinacion);

      // Mostrar datos de velocidad y posición en Serial
      file.print("Velocidad_M1:  Posición_M1:");
      file.print(vel_M1); Serial.print("\t"); Serial.println(pos_M1);
      file.print("Velocidad_M2:  Posición_M2:");
      file.print(vel_M2); Serial.print("\t"); Serial.println(pos_M2);
      
      file.close();
      Serial.println("Dato actualizado");
    } else {
      Serial.println("Error al abrir el archivo");
    }
  }


}


void setup() {
  Serial.begin(115200);


  // Configurar el Bluetooth con el nombre único
  //configurarBluetooth();

  //conectar_wifi();


  // Montar SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("Error al montar SPIFFS");
    return;
  }
  Serial.println("SPIFFS montado correctamente");

  // Crear o abrir el archivo y escribir datos
  File file = SPIFFS.open("/datalog.txt", FILE_APPEND);
  if (!file) {
    Serial.println("Error al abrir el archivo");
    return;
  }

  // Escribir datos en el archivo
  file.println("Reiniciado datos nuevos:");
  file.close();
  Serial.println("empezando a guardar datos");

  int pwm_hz = 20000;
  int encoder_res = 1280;

  OBJ.init(pwm_hz, encoder_res, VM);
  OBJ.BMI088_init(pin1, pin2);
  OBJ.DRV8837_Wake();

  //para imprimir valores
  Serial.begin(115200);


  // IOT.Mqtt_SerialInfo(true);
  // IOT.Mqtt_init("www.xspace.pe", 1883, ref_sp);
  //IOT.Mqtt_Connect(WiFi.SSID().c_str(), WiFi.psk().c_str(), "djvemo_xspace");
  // // me suscribo para enviar datos del esp al server para verlo en el pc suscribiendome
  // IOT.Mqtt_Suscribe("control/ref"); //se susbribe a un topic, lo yo le mando un valor desde pc en publish


  xTaskCreatePinnedToCore(tarea_1, "Tarea1", 4000, NULL, 1, NULL, 0); //el ultimo indica el mucleo
  //xTaskCreatePinnedToCore(tarea_encoder, "Tarea2", 4000, NULL, 2, NULL, 0);
}

void loop() {
  // Vacío ya que las tareas manejan la lógica
  void log_datos();
}
