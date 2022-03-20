/*
 Name:		    Marcador_maestro.ino
 Description:   MARCADOR POSESION KAYAK POLO
 Created:	    19/03/2022 13:01:25
 Author:	    Juan Carlos Rodriguez Lara
 Mail:          jrodrila@gmail.com
 Version:       0.1 - Slave
*/

// Librerias
#include <dummy.h>
#include <Wire.h>  //Libreria para que funcione el I2C de la pantalla OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Arduino.h>
#include <iostream>

// Definiciones Pantalla_OLED
#define SCREEN_WIDTH 128  // OLED width,  in pixels
#define SCREEN_HEIGHT 32  // OLED height, in pixels

// Definiciones pines ESP32C3
#define LIBRE_PIN_0     0    //(IO0 / ADC1_CH0 / XTAL_32K_N)
#define LIBRE_PIN_1     1    //(IO1 / ADC1_CH1 / XTAL_32K_N)
#define LIBRE_PIN_2     2    //(IO2 / ADC1_CH2 / FSPIQ)
#define PIN_RED         3    // ADC1_CH3 / (IO03 / ADC1_CH3)
#define PIN_GREEN       4    // ADC1_CH4 / (IO04 / ADC1_CH4 / FSPIHD / MTMS)
#define PIN_BLUE        5    // ADC2_CH0 / (IO05 / ADC2_CH0 / FSPIWP / MTDI)
#define I2C_SDA         6    //I2C_SDA (IO6 / FSPICLK / MTCK)  pin 7 de J3 _0x3C
#define I2C_SCL         7    //I2C_SCL (IO7 / FSPID / MTDO)    pn 8 de J3
#define LIBRE_PIN_8     8    //(IO8)
#define BUTTON_1        9    // IO09 / Boton integrado en PCB
#define BOT_CH          10   //(IO10 / FSPICSO)
#define BOT_UP          12   //(IO12 / SPIHD)
#define LIBRE_PIN_14    14   //(IO14 / SPICS0)
#define LIBRE_PIN_15    15   //(IO15 / SPICLK)
#define LIBRE_PIN_16    16   //(IO16 / SPID)
#define LIBRE_PIN_17    17   //(IO17 / SPIQ)
#define LED_PIN_1       18   // IO18 Led Naranja Integrado
#define LED_PIN_2       19   // IO19 Led Blanco Integrado
#define U0RX            20   // RX0 (IO20 / RX0)
#define U0TX            21   // TX0 (IO21 / TX0)

// Definiciones
#define MAX_DISPLAY     11          // Valor máximo por defecto
#define AUTORESET       true        // Autoreset por defecto activado
#define AVISO           5           //Tiempo de aviso de posesión
#define uS_TO_S_FACTOR  1000000     /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP   0.5         /* Time ESP32 will go to sleep (in seconds) */



//Variables timer
unsigned long intervalo = 1000;  //milisegundos a contar
unsigned long tiempo_previo_boton = 0;
unsigned long tiempo_actual = 0;
unsigned long power_saving_time = 10000 ; //tiempo en ms en el cual activamos power saving 
bool power_saving = 0;  //power saving activado

//Variables microprocesador
uint32_t frecuencia = 0;
uint32_t cpu_freq_mhz = 160;     //Menos de 80 MHz no va bien la WIFI ni el puerto serie
RTC_DATA_ATTR int bootCount = 0; //


//Variables marcador
const int timeThreshold = 500;  //Tiempo de filtro de rebote para los botones
long contador_max = MAX_DISPLAY;
long contador_max_temp = MAX_DISPLAY;
long contador = contador_max;
bool autoreset = AUTORESET;
int decenas;
int unidades;
int segundos_aviso = AVISO;
bool estadoLED = 0;  //SOLO Esclavo
bool estadoBOT_CH = 0;
bool estadoBOT_UP = 0;
bool actualizar = 0;      //Flag para actualizar datos

//Variables menu
int numScreen = 0;  //Número de pantalla


//Variables ESP-NOW
int id_pcb = 111;         //ID de MCU
bool resetear = 0;        //Para mandar un reset
bool auto_setting = 0;    //Para cambiar parámetro de autoreset
int tiempo_setting = 0;   //Para cambiar el parámetro de tiempo
int aviso_setting = 0;    //Para cambiar el parámetro de aviso
int contador_master = 0;  //Para guardar el valor del contador del maestro
String success;           //Varible para saber que el mensaje se ha entregado
uint8_t broadcastAddress1[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };  //Direccion MAC donde queremos mandar los datos

//Estructura para enviar datos
typedef struct struct_message {
    int id;          //ID de MCU
    bool rst;        //resetear
    bool aut;        //autoreset
    int cnt;         //contador
    int set_tiempo;  //ajustar tiempo
    int set_aviso;   //ajustar aviso
} struct_message;

struct_message datos_slave;   //creamos estructura para MANDAR datos del esclavo
struct_message datos_master;  //creamos estructura para RECIBIR los datos del maestro

esp_now_peer_info_t peerInfo;


//Objetos
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);  //Pantalla OLED 0.91

//FUNCIONES
// ESP-NOW Callback when data is sent
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    //Serial.print("\r\nLast:\t");
    //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery MS OK" : "Fallo Entrega en MAESTRO");
    if (status == 0) {
        success = "Envio a Maestro OK :)";
    }
    else {
        success = "Envio a Maestro NOK :(";
    }
}
// ESP-NOW Callback when data is received
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    memcpy(&datos_master, incomingData, sizeof(datos_master));
    //Serial.print("Bytes on SLAVE: ");
    //Serial.println(len);
    //resetear = datos_master.rst;
    actualizar = 1;
    auto_setting = datos_master.aut;
    tiempo_setting = datos_master.set_tiempo;
    aviso_setting = datos_master.set_aviso;
    contador_master = datos_master.cnt;
    contador = contador_master;
    autoreset = auto_setting;
    contador_max = tiempo_setting;
    segundos_aviso = aviso_setting;
    Serial.print(datos_master.id);
    Serial.print("--> rst: ");
    Serial.print(datos_master.rst);
    Serial.print(" - aut: ");
    Serial.print(auto_setting);
    Serial.print(" - set_tiempo: ");
    Serial.print(tiempo_setting);
    Serial.print(" - set_aviso: ");
    Serial.print(aviso_setting);
    Serial.print(" - cnt: ");
    Serial.println(contador_master);
}

//Funcion para actualizar el OLED
void actualizarOLED(int menu)
{
    oled.clearDisplay();  // clear display
    if (menu == 0) {
        //contador_max = contador_max_temp;
        oled.setTextSize(4);
        oled.setCursor(35, 5);           // set position to display
        oled.println(String(decenas));   // set text
        oled.setCursor(60, 5);           // set position to display
        oled.println(String(unidades));  // set text
        oled.setTextSize(1);
        oled.setCursor(90, 0);  // set position to display
        oled.println("T: ");    // set text
        oled.setTextSize(2);
        oled.setCursor(100, 0);                  // set position to display
        oled.println(String(contador_max - 1));  // set text
        oled.setTextSize(1);
        oled.setCursor(90, 18);  // set position to display
        oled.println("A: ");     // set text
        oled.setTextSize(2);
        oled.setCursor(100, 18);               // set position to display
        oled.println(String(segundos_aviso));  // set text
        if (autoreset) {
            oled.setTextSize(1);
            oled.setCursor(0, 25);  // set position to display
            oled.println("AUTO");   // set text
        }
    }
    if (menu == 1) {
        if (estadoBOT_UP) {
            contador_max_temp += 10;
            if (contador_max_temp > 99) {
                contador_max_temp = 11;
            }
            estadoBOT_UP = 0;
        }
        oled.setTextSize(1);
        oled.setCursor(90, 0);  // set position to display
        oled.println("T: ");    // set text
        oled.setTextSize(2);
        oled.setCursor(100, 0);                       // set position to display
        oled.println(String(contador_max_temp - 1));  // set text
    }
    if (menu == 2) {
        if (estadoBOT_UP) {
            segundos_aviso += 5;
            if (segundos_aviso > contador_max) {
                segundos_aviso = 0;
            }
            estadoBOT_UP = 0;
        }
        oled.setTextSize(1);
        oled.setCursor(90, 18);  // set position to display
        oled.println("A: ");     // set text
        oled.setTextSize(2);
        oled.setCursor(100, 18);               // set position to display
        oled.println(String(segundos_aviso));  // set text
    }
    if (menu == 3) {
        if (estadoBOT_UP) {
            autoreset = !autoreset;
            estadoBOT_UP = 0;
        }
        oled.setTextSize(2);
        oled.setCursor(50, 18);  // set position to display
        if (autoreset) {
            oled.println("ON");  // set text
            oled.setTextSize(1);
            oled.setCursor(0, 25);  // set position to display
            oled.println("AUTO");   // set text
        }
        else {
            oled.println("OFF");  // set text
        }
    }
    if (menu == 4) {
        oled.setTextSize(1);
        oled.setCursor(6, 0);
        oled.print("-IP:");
        //oled.println(WiFi.localIP());
        oled.setCursor(6, 8);
        oled.print(frecuencia);
        oled.println(" MHz");
    }
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print(String((menu + 1)));
    oled.println("-SL");
    oled.display();  // display on OLED
}

//Función RGB
void eval_RGB() {
    if (contador < 1) {
        analogWrite(PIN_RED, 255);
        analogWrite(PIN_GREEN, 0);
        analogWrite(PIN_BLUE, 0);
    }
    if (contador >= 1 && contador <= 5) {
        analogWrite(PIN_RED, 255);
        analogWrite(PIN_GREEN, 128);
        analogWrite(PIN_BLUE, 0);
    }
    if (contador > 5 && contador <= contador_max) {
        analogWrite(PIN_RED, 0);
        analogWrite(PIN_GREEN, 255);
        analogWrite(PIN_BLUE, 0);
    }
}




//Interrupciones
void IRAM_ATTR ISR_button1()  //Resetear el contador#####NO FUNCIONA PRINTLN (resetea el micro)
{
    estadoLED = !estadoLED;
    digitalWrite(LED_PIN_1, estadoLED);  // turn on LED
    //Serial.println("Estadoboton: ON " + String(estadoLED));
    resetear = 1;
    actualizar = 1;
    tiempo_actual = millis();
    power_saving = 0; //desactivamos power saving
}
void IRAM_ATTR ISR_BOT_CH()  //Boton cambio de funcion #####NO FUNCIONA PRINTLN (resetea el micro)
{
    if (millis() - tiempo_previo_boton > timeThreshold) {
        estadoBOT_CH = 1;
        if (numScreen < 4) {
            numScreen++;
        }
        else {
            numScreen = 0;
        }
        tiempo_previo_boton = millis();
        power_saving = 0; //desactivamos power saving
    }
}
void IRAM_ATTR ISR_BOT_UP()  //Boton cambio de opcion/Resetear el contador#####NO FUNCIONA PRINTLN (resetea el micro)
{
    if (millis() - tiempo_previo_boton > timeThreshold) {
        estadoBOT_UP = 1;
        tiempo_previo_boton = millis();
        power_saving = 0; //desactivamos power saving
    }
}


void setup()
{
    /*Iniciamos monitor serie*/
    Serial.begin(115200);
    /*Definimos frecuencia de trabajo del micro*/
    setCpuFrequencyMhz(cpu_freq_mhz);

    /*I2C Inicializacion*/
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(100);//500

    /*Pines RGB*/
    pinMode(PIN_RED, OUTPUT);
    pinMode(PIN_GREEN, OUTPUT);
    pinMode(PIN_BLUE, OUTPUT);
    /*Pines botones*/
    // HIGH interruptor abierto - LOW interruptor cerrado.
    pinMode(BUTTON_1, INPUT_PULLUP);  //Boton interno
    //pinMode(BOT_CH, INPUT_PULLUP);//Boton cambio menu
    //pinMode(BOT_UP, INPUT_PULLUP);//Botion cambio opcion
    pinMode(LED_PIN_1, OUTPUT);  //LED interno


    /*Conectamos a WIFI*/
    WiFi.mode(WIFI_STA);
    // WiFi.begin(ssid, password);
    // if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    //   Serial.printf("WiFi Failed!\n");
    //   //return;
    // }
    // Serial.print("IP: ");
    // Serial.println(WiFi.localIP());

    /*******************Init ESP-NOW***********************/
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializando ESP-NOW");
        //return;
    }
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    // Preparamos info para registrar esclavo
    memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    // Añadimos esclavo
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Fallo añadiendo peer");
        return;
    }
    // Registramos funcion callback que sera llamada cuandop recibimos datos
    esp_now_register_recv_cb(OnDataRecv);
    /*******************FIN Init ESP-NOW********************/

    /*Inicializamos interrupciones*/
    attachInterrupt(BUTTON_1, ISR_button1, FALLING);  //Boton para resetear el tiempo
    //attachInterrupt(BOT_CH, ISR_BOT_CH, FALLING);//Boton para cambio de funcion
    //attachInterrupt(BOT_UP, ISR_BOT_UP, FALLING);////Boton para resetear el tiempo / cambio de opcion
    Serial.print(bootCount);
    Serial.println(" inicios");
   
    /*********************Inicializamos OLED*****************/
    if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("failed to start SSD1306 OLED"));
        while (1);
    }
    oled.clearDisplay();          // clear display
    oled.setTextSize(2);         // set text size
    oled.setTextColor(WHITE);    // set text color

     if (bootCount == 0) {
        oled.clearDisplay();          // clear display
        oled.setTextSize(2);         // set text size
        oled.setTextColor(WHITE);    // set text color
        oled.setCursor(0, 0);         // set position to display
        oled.println(" Marcador ");   // set text
        oled.setCursor(0, 17);        // set position to display
        oled.println(" v0.1 SL");       // set text
        oled.display();               // display on OLED
        delay(1000); 
        }
    /*********************FIN Inicializamos OLED*****************/
    /* DEBUGGER */
    frecuencia = getCpuFrequencyMhz();  // In MHz
    Serial.print(frecuencia);
    Serial.println(" MHz");

    bootCount++;
  



}

/*####################### BUCLE PRINCIPAL ######################*/
void loop() {
    

   
    if (actualizar == 1)
    {
        decenas = (contador - (contador % 10)) / 10;
        unidades = contador - decenas * 10;
        if (millis() - tiempo_actual > power_saving_time) {
            power_saving = 1;
            oled.clearDisplay();
            oled.display();               
        }

        if (power_saving == 0) {
            actualizarOLED(numScreen);

        }

        /*Enviamos info ESP-NOW*/
        //Actualizar datos de envio
        aviso_setting = segundos_aviso;
        tiempo_setting = contador_max;
        auto_setting = autoreset;
        datos_slave.id = id_pcb;
        datos_slave.cnt = contador;
        datos_slave.rst = resetear;
        datos_slave.aut = auto_setting;
        datos_slave.set_tiempo = tiempo_setting;
        datos_slave.set_aviso = aviso_setting;
        // Enviamos mensaje ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t*)&datos_slave, sizeof(datos_slave));
        if (result == ESP_OK) {
            //Serial.println("Envio a maestro OK");00
        }
        else {
            Serial.println("Envio a maestro NOK");
        }
        resetear = 0;
        actualizar = 0;
        /*FIN Enviamos info ESP-NOW*/

        //eval_RGB();
       
        esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); // ESP32 wakes up every 0.8 seconds
        Serial.println("Going to light-sleep now");
        Serial.flush();
        //esp_deep_sleep_start();

    }
}

