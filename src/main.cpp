#include <Arduino.h>
#include <XSpaceV21.h>
#include <XSpaceIoT.h>
//#include <XSControl.h> // Required for motor control functions
#include "WiFi.h"

// Configuración de la placa y variables
XSpaceV21Board OBJ;
XSThing IOT;

int Ts = 10; // Tiempo de muestreo en milisegundos
double u = 5; // Voltaje de entrada
double setpoint=5; // Valor recibido para sp
int pin1 = 0; // Pin 1 para acelerómetro
int pin2 = 5; // Pin 2 para giroscopio
double VM = 5; // Voltaje máximo del driver
float ax, ay, az; // Variables para aceleración
float gx, gy, gz; // Variables para giroscopio
double inclinacion; // Ángulo de inclinación
double vel_M1, vel_M2, pos_M1, pos_M2; // Velocidad y posición angular
int k = 0; // Inicialización global

// Configuración de redes WiFi
const char* ssidList[] = {"redpucp", "Machado Ferrer", "djvemo's S24U"};
const char* passwordList[] = {"C9AA28BA93", "0932Admin0", "0611000019"};
int numNetworks = sizeof(ssidList) / sizeof(ssidList[0]);

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

    // Mostrar datos de IMU en Serial
    Serial.print("Inclinación: "); 
    Serial.println(inclinacion);
    Serial.print("Aceleración [x, y, z]: ");
    Serial.print(ax); Serial.print("\t"); Serial.print(ay); Serial.print("\t"); Serial.println(az);
    Serial.print("Giroscopio [x, y, z]: ");
    Serial.print(gx); Serial.print("\t"); Serial.print(gy); Serial.print("\t"); Serial.println(gz);

    // Enviar datos de IMU a MQTT
    IOT.Mqtt_Publish("sensor/inclinacion", inclinacion);
    //IOT.Mqtt_Publish("sensor/acceleration", String(ax) + "," + String(ay) + "," + String(az));
    //IOT.Mqtt_Publish("sensor/gyroscope", String(gx) + "," + String(gy) + "," + String(gz));

    // **Encoders y Control de Motores** - Captura de datos de velocidad y posición
    u = setpoint;  // Actualizar el voltaje de entrada según el setpoint recibido
    OBJ.DRV8837_Voltage(DRVx1, u);
    OBJ.DRV8837_Voltage(DRVx2, u);

    vel_M1 = OBJ.GetEncoderSpeed(E1, DEGREES_PER_SECOND);
    pos_M1 = OBJ.GetEncoderPosition(E1, DEGREES);
    vel_M2 = OBJ.GetEncoderSpeed(E2, DEGREES_PER_SECOND);
    pos_M2 = OBJ.GetEncoderPosition(E2, DEGREES);

    // Mostrar datos de velocidad y posición en Serial
    Serial.print("Velocidad_M1:  Posición_M1:");
    Serial.print(vel_M1); Serial.print("\t"); Serial.println(pos_M1);
    
    // Mostrar datos de velocidad y posición en Serial
    Serial.print("Velocidad_M2:  Posición_M2:");
    Serial.print(vel_M2); Serial.print("\t"); Serial.println(pos_M2);

    // Enviar datos de velocidad y posición a MQTT
    IOT.Mqtt_Publish("motor/velocidad_M1", vel_M1);
    IOT.Mqtt_Publish("motor/posicion_M1", pos_M1);
    IOT.Mqtt_Publish("motor/velocidad_M2", vel_M2);
    IOT.Mqtt_Publish("motor/posicion_M2", pos_M2);

    // Verificar si hay nueva información publicada en el buffer MQTT
    IOT.Mqtt_CheckBuffer();

    // Delay según el tiempo de muestreo
    vTaskDelay(Ts);
  }

  // Eliminar tarea en caso de salida del bucle (opcional, usualmente innecesario en FreeRTOS)
  vTaskDelete(NULL);
}


void setup() {
  Serial.begin(115200);
  conectar_wifi();
  int pwm_hz = 20000;
  int encoder_res = 1280;

  OBJ.init(pwm_hz, encoder_res, VM);
  OBJ.BMI088_init(pin1, pin2);
  OBJ.DRV8837_Wake();

  //para imprimir valores
  Serial.begin(115200);


  IOT.Mqtt_SerialInfo(true);
  IOT.Mqtt_init("www.xspace.pe", 1883, ref_sp);
  IOT.Mqtt_Connect(WiFi.SSID().c_str(), WiFi.psk().c_str(), "djvemo_xspace");
  // me suscribo para enviar datos del esp al server para verlo en el pc suscribiendome
  IOT.Mqtt_Suscribe("control/ref"); //se susbribe a un topic, lo yo le mando un valor desde pc en publish


  xTaskCreatePinnedToCore(tarea_1, "Tarea1", 4000, NULL, 1, NULL, 0);
  //xTaskCreatePinnedToCore(tarea_encoder, "Tarea2", 4000, NULL, 2, NULL, 0);
}

void loop() {
  // Vacío ya que las tareas manejan la lógica
}
