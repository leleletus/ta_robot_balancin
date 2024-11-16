

#include <Arduino.h>
#include <XSpaceV21.h>
#include <XSpaceIoT.h>
#include <XSControl.h>
#include "WiFi.h"
#include "wifi_credentials.h" // Incluir el archivo de credenciales

// Definición de los pines para controlar los LEDs de estado
#define RED 21
#define GREEN 22
#define BLUE 17
#define OFF 0

// Prototipos de funciones para el estado del LED y la obtención del voltaje de la batería
void XSQC_2S_Shield_LedSTATUS(int color);
double XSQC_2S_Shield_GetBatteryVoltage();

// Instanciación de objetos para la interfaz con el hardware
XSpaceV21Board XSBoard;
XSFilter Filter_a, Filter_g;
XSController Controller1, Controller2, Controller3;
XSThing IOT;

// Definición de constantes para la configuración del control del motor
#define PWM_FREQUENCY 20000 // Frecuencia PWM para control de motor a 20 kHz para evitar ruido audible.
#define ENCODER_RESOLUTION 960 // Resolución del codificador, típicamente el número de pasos por revolución.

double u1,u2;

// Variables para almacenar datos brutos del sensor
float gx, gy, gz; // Datos del giroscopio para el ángulo de inclinación
float pitchAccel; // Ángulo de inclinación calculado a partir del acelerómetro
float pitchAccel_filtered; // Ángulo de inclinación del acelerómetro filtrado
float pitchGyro; // Ángulo de inclinación calculado a partir del giroscopio
float pitchGyro_filtered; // Ángulo de inclinación del giroscopio filtrado

// Variables para el filtro complementario
float pitch = 0.0; // Ángulo de inclinación combinado
const float alpha = 0.65; // Coeficiente del filtro complementario
double equilibrium_angle = 0; // Ángulo de equilibrio del robot
double setpoint=0; // Valor recibido para sp

// Variables para almacenar la velocidad y el punto de consigna de los motores
double speed_m1;
double speed_m2;
double position_m1;
double position_m2;
double speed_m1_sp = 0; // Setpoint (punto de consigna) para el motor 1
double speed_m2_sp = 0; // Setpoint (punto de consigna) para el motor 2

// Función para recibir datos desde MQTT
void ref_sp(char* topicx, byte* Data, unsigned int DataLen) {
  String ReceivedData = String((char*)Data, DataLen);
  String Topic = String((char*)topicx);

  if (Topic == "control/ref") {
    setpoint = ReceivedData.toDouble(); // Pasamos de string a double
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


// Tarea de filtrado: combina datos de acelerómetro y giroscopio para calcular el ángulo de inclinación
void FilterTask(void *pv) {
    pitch = XSBoard.BMI088_GetPitch_Accel(); // Inicializa la inclinación a partir del acelerómetro

    while (1) {
        // Recupera los últimos datos del giroscopio
        XSBoard.BMI088_GetGyroData(&gx, &gy, &gz);
        // Calcula el ángulo de inclinación del acelerómetro
        float pitchAccel = XSBoard.BMI088_GetPitch_Accel();

        // Integra los datos del giroscopio para obtener el cambio de inclinación
        float pitchGyro = pitch + gy * 0.001; // gy en grados por segundo

        // Aplica filtros de paso bajo de segundo orden a los datos
        pitchAccel_filtered = Filter_a.SecondOrderLPF(pitchAccel, 35, 0.001);
        pitchGyro_filtered  = Filter_g.SecondOrderLPF(pitchGyro, 35, 0.001);

        // Filtro complementario para combinar datos del acelerómetro y giroscopio
        pitch = alpha * pitchGyro_filtered + (1 - alpha) * pitchAccel_filtered;

        vTaskDelay(1); // Retraso de 1 milisegundo (ajustar según la aplicación)
    }

    // Nunca se alcanza, pero se coloca por seguridad
    vTaskDelete(NULL);
}

// Tarea del controlador de velocidad: ajusta el voltaje de los motores para seguir las velocidades deseadas
void SpeedController(void *pv){
    while(1){
        // Obtiene la velocidad actual de los motores a partir de los codificadores
        speed_m1 = XSBoard.GetEncoderSpeed(E1, DEGREES_PER_SECOND);
        speed_m2 = XSBoard.GetEncoderSpeed(E2, DEGREES_PER_SECOND);
        position_m1 = XSBoard.GetEncoderPosition(E1, DEGREES);
        position_m2 = XSBoard.GetEncoderPosition(E2, DEGREES);

        u1=Controller1.PI_ControlLaw(speed_m1, speed_m1_sp, 0.0241, 0.4820, FORWARD_EULER, 0.01);
        u2=Controller2.PI_ControlLaw(speed_m2, speed_m2_sp, 0.0222, 0.4440, FORWARD_EULER, 0.01);

        // Aplica el controlador PI para ajustar el voltaje del motor
        XSBoard.DRV8837_Voltage(DRVx1, u1);
        XSBoard.DRV8837_Voltage(DRVx2, u2);

        vTaskDelay(10); // Pausa de 10 milisegundos entre mediciones de velocidad
    }

    // Nunca se alcanza, pero se coloca por seguridad
    vTaskDelete(NULL);
}

// Tarea del controlador de ángulo: ajusta la velocidad de los motores para mantener el ángulo deseado
void AngleController(void *pv){
    float velx; // Velocidad calculada para los motores

    double Kp = 50; // Ganancia proporcional
    double Kd = 0; // Ganancia derivativa
    double Ki = 120; // Ganancia integrativa

    while(1){
        // Calcula la velocidad deseada basada en el control PID
        velx = Controller3.PID_ControlLaw(pitch, equilibrium_angle, Kp, Ki, FORWARD_EULER, Kd, FORWARD_EULER, 9.73920144223375, 0.02);
        
        // Ajusta los puntos de consigna de velocidad para los motores
        speed_m1_sp = -velx;
        speed_m2_sp = velx;

        vTaskDelay(20); // Pausa de 20 milisegundos entre ajustes
    }

    // Nunca se alcanza, pero se coloca por seguridad
    vTaskDelete(NULL);
}

// Configuración inicial
void setup() {
    //Serial.begin(1000000); // Inicializa la comunicación serial a 1 Mbps para transmisión rápida de datos
    Serial.begin(115200);

    conectar_wifi(); // Conexión a la red WiFi
    
    // Obtiene el voltaje de la batería conectada al escudo XSQC-2S
    double VM = XSQC_2S_Shield_GetBatteryVoltage();

    // Inicializa la placa XSpace con la frecuencia PWM y resolución del codificador
    XSBoard.init(PWM_FREQUENCY, ENCODER_RESOLUTION, VM);
    XSBoard.DRV8837_Wake(DRVx1); // Despierta el controlador del motor 1
    XSBoard.DRV8837_Wake(DRVx2); // Despierta el controlador del motor 2

    // Inicializa WiFi y conexión UDP para monitoreo
    IOT.Mqtt_SerialInfo(true);
    IOT.Mqtt_init("www.xspace.pe", 1883, ref_sp);
    IOT.Mqtt_Connect(WiFi.SSID().c_str(), WiFi.psk().c_str(), "emf xspace");
    // me suscribo para enviar datos del esp al server para verlo en el pc suscribiendome
    IOT.Mqtt_Suscribe("control/ref"); //se susbribe a un topic, lo yo le mando un valor desde pc en publish
    // Crea la tarea para el filtrado y cálculo del ángulo de inclinación
    xTaskCreate(FilterTask, "FilterTask", 5000, NULL, 1, NULL);

    // Encuentra el ángulo de equilibrio inicial
    XSQC_2S_Shield_LedSTATUS(BLUE); // LED azul indica calibración
    for(int i = 0; i < 200; i++){
        equilibrium_angle = pitch; // El ángulo de equilibrio es el valor promedio de inclinación
        //XSnet.println(equilibrium_angle); // Envía el ángulo de equilibrio a través de la red
        delay(10);
    }
    XSQC_2S_Shield_LedSTATUS(OFF); // Apaga el LED después de la calibración

    delay(1000); // Pausa para asegurar que todo esté listo
    XSQC_2S_Shield_LedSTATUS(GREEN); // LED verde indica que el robot está listo

    // Crea las tareas para el control de velocidad y ángulo
    xTaskCreate(SpeedController, "SpeedController", 5000, NULL, 1, NULL);
    xTaskCreate(AngleController, "AngleController", 5000, NULL, 1, NULL);
}

// Bucle principal: transmite datos de inclinación y velocidad
void loop() {
    //XSnet.println(pitch); // Envia el ángulo de inclinación y la velocidad a través de la red

    // Seguridad: si la inclinación supera 40 grados, detiene los motores
    if(abs(pitch) > 40){
        XSQC_2S_Shield_LedSTATUS(RED); // LED rojo indica un error
        XSBoard.DRV8837_Sleep(DRVx1); // Pone a dormir el motor 1
        XSBoard.DRV8837_Sleep(DRVx2); // Pone a dormir el motor 2
        //delay(1000000); // Espera indefinidamente para seguridad
    }

    // Enviar por MQTT la inclinación del robot, la velocidad y la posición de los motores

    // Creamos primero un string con todos los datos a enviar
    String data = "v1: " + String(u1) + ", v2: " + String(u2) + ", pitch: " + String(pitch) + ", speed_m1: " + String(speed_m1) + ", speed_m2: " + String(speed_m2) + ", position_m1: " + String(position_m1) + ", position_m2: " + String(position_m2);
    // Convertimos el string a un array de caracteres
    char data_array[data.length() + 1];
    data.toCharArray(data_array, data.length() + 1);
    // Enviamos el array de caracteres por MQTT
    IOT.Mqtt_Publish("robot/data", data_array);

    // Verificar si hay nueva información publicada en el buffer MQTT
    IOT.Mqtt_CheckBuffer();

    delay(10); // Pausa de 10ms entre envíos de datos
}

double XSQC_2S_Shield_GetBatteryVoltage(){
    return (double)analogRead(36)/4096.0*3.3*4.0;
}

void XSQC_2S_Shield_LedSTATUS(int color){
    pinMode(22,OUTPUT); //STATUS GREEN LED
    pinMode(21,OUTPUT); //STATUS RED LED
    pinMode(17,OUTPUT); //STATUS BLUE LED

        if(color == RED){
            digitalWrite(RED,HIGH);
            digitalWrite(GREEN,LOW);
            digitalWrite(BLUE,LOW);
        } 
        if(color == GREEN){
            digitalWrite(RED,LOW);
            digitalWrite(GREEN,HIGH);
            digitalWrite(BLUE,LOW);
        }
        if(color == BLUE){
            digitalWrite(RED,LOW);
            digitalWrite(GREEN,LOW);
            digitalWrite(BLUE,HIGH);
        } 
        if(color == OFF){
            digitalWrite(RED,LOW);
            digitalWrite(GREEN,LOW);
            digitalWrite(BLUE,LOW);
        } 
}