/*
 Name:		    Marcador_maestro.ino
 Description:   MARCADOR POSESION KAYAK POLO
 Created:	    19/03/2022 13:01:25
 Author:	    Juan Carlos Rodriguez Lara
 Mail:          jrodrila@gmail.com
 Version:       0.1 - Master
*/

// Librerias
#include <dummy.h>
#include <Wire.h> //Libreria para que funcione el I2C de la pantalla OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Arduino.h>
#include <iostream>

// Definiciones Pantalla_OLED
#define SCREEN_WIDTH  128   // OLED width,  in pixels
#define SCREEN_HEIGHT 32    // OLED height, in pixels

// Definiciones pines ESP32-DevKit v4
#define BOT_CH        16      //36 IO36 / I   /   GPIO36, ADC1_CH0, S_VP                    -> J1 - Pin3 (no pull-up resistors)
#define BOT_UP        17      //39 IO39 / I   /   GPIO39, ADC1_CH3, S_VN                    -> J1 - Pin4 (no pull-up resistors)   
#define LIBRE_IO34    34      // IO34 / I   /   GPIO34, ADC1_CH6, VDET_1                  -> J1 - Pin5 (no pull-up resistors)
#define LIBRE_IO35    35      // IO35 / I   /   GPIO35, ADC1_CH7, VDET_2                  -> J1 - Pin6 (no pull-up resistors)  
#define SEG_UN_a      32      // IO32 / IO  /   GPIO32, ADC1_CH4, TOUCH_CH9, XTAL_32K_P   -> J1 - Pin7 
#define SEG_UN_b      33      // IO33 / IO  /   GPIO33, ADC1_CH5, TOUCH_CH8, XTAL_32K_N   -> J1 - Pin8 
#define SEG_UN_c      25      // IO25 / IO  /   GPIO25, ADC1_CH8, DAC_1                   -> J1 - Pin9
#define SEG_UN_d      26      // IO26 / IO  /   GPIO26, ADC2_CH9, DAC_2                   -> J1 - Pin10
#define SEG_UN_e      27      // IO27 / IO  /   GPIO27, ADC2_CH7, TOUCH_CH7               -> J1 - Pin11
#define SEG_UN_f      14      // IO14 / IO  /   GPIO14, ADC2_CH6, TOUCH_CH6, MTMS         -> J1 - Pin12
#define SEG_UN_g      12      // IO12 / IO  /   GPIO12, ADC2_CH5, TOUCH_CH5, MTDI         -> J1 - Pin13 
#define SEG_UN_dot    13      // IO13 / IO  /   GPIO13, ADC2_CH4, TOUCH_CH4, MTCK         -> J1 - Pin15 
#define D2            9       // IO09 / IO  /   GPIO9, D2                                 -> J1 - Pin16 (no usar: flash)
#define D3            10      // IO10 / IO  /   GPIO10, D3                                -> J1 - Pin17 (no usar: flash)
#define CMD           11      // IO11 / IO  /   GPIO11, CMD                               -> J1 - Pin18 (no usar: flash)
#define SEG_DC_a      23      // IO23 / IO  /   GPIO23                                    -> J3 - Pin2
#define I2C_SCL       22      // IO22 / IO  /   GPIO22                                    -> J3 - Pin3
#define U0TXD         1       // IO01 / IO  /   GPIO1, U0TXD                              -> J3 - Pin4
#define U0RXD         3       // IO03 / IO  /   GPIO3, U0RXD                              -> J3 - Pin5
#define I2C_SDA       21      // IO21 / IO  /   GPIO21                                    -> J3 - Pin6
#define SEG_DC_b      19      // IO19 / IO  /   GPIO19                                    -> J3 - Pin8
#define SEG_DC_c      18      // IO18 / IO  /   GPIO18                                    -> J3 - Pin9
#define SEG_DC_d      5       // IO05 / IO  /   GPIO5                                     -> J3 - Pin10
#define SEG_DC_e      17      // IO17 / IO  /   GPIO17                                    -> J3 - Pin11
#define SEG_DC_f      16      // IO16 / IO  /   GPIO16                                    -> J3 - Pin12
#define SEG_DC_g      4       // IO04 / IO  /   GPIO4, ADC2_CH0, TOUCH_CH0                -> J3 - Pin13
#define BUTTON_1      0       // IO00 / IO  /   GPIO0, ADC2_CH1, TOUCH_CH1, Boot          -> J3 - Pin14 (boton integrado en PCB)
#define SEG_DC_dot    2       // IO02 / IO  /   GPIO2, ADC2_CH2, TOUCH_CH2                -> J3 - Pin15
#define BOCINA        15      // IO15 / IO  /   GPIO15, ADC2_CH3, TOUCH_CH3, MTDO         -> J3 - Pin16
#define D1            8       // IO08 / IO  /   GPIO8, D1                                 -> J3 - Pin17 (no usar: flash)
#define D0            7       // IO07 / IO  /   GPIO7, D0                                 -> J3 - Pin18 (no usar: flash)
#define SCK           6       // IO06 / IO  /   GPIO6, SCK                                -> J3 - Pin19 (no usar: flash)


// Definiciones
#define MAX_DISPLAY   11      // Valor máximo por defecto
#define AUTORESET     true    // Autoreset por defecto activado
#define AVISO         5       //Tiempo de aviso de posesión


//Variables timer
unsigned long intervalo = 1000; //milisegundos a contar
unsigned long tiempo_previo = 0;
unsigned long tiempo_actual = 0;
unsigned long tiempo_previo_boton = 0;

//Variables microprocesador
uint32_t frecuencia = 0;
uint32_t cpu_freq_mhz = 240;     //Menos de 80 MHz no va bien la WIFI ni el puerto serie

//Variables marcador
const int timeThreshold = 500;  //Tiempo de filtro de rebote para los botones
long contador_max = MAX_DISPLAY;
long contador_max_temp = MAX_DISPLAY;
long contador = contador_max;
long tiempo_bocina;
bool autoreset = AUTORESET;
int decenas;
int unidades;
int segundos_aviso = AVISO;
int activar_aviso = 0;
int activar_final = 0;
bool estadoBOT_CH = 0;
bool estadoBOT_UP = 0;
bool numero_lcd[10][8] = {
    {0,1,0,0,0,0,0,0}, //0
    {0,1,1,1,1,0,0,1}, //1
    {0,0,1,0,0,1,0,0}, //2
    {0,0,1,1,0,0,0,0}, //3
    {0,0,0,1,1,0,0,1}, //4
    {0,0,0,1,0,0,1,0}, //5
    {0,0,0,0,0,0,1,0}, //6
    {0,1,1,1,1,0,0,0}, //7
    {0,0,0,0,0,0,0,0}, //8
    {0,0,0,1,0,0,0,0}, //9
};


//Variables menu
int numScreen = 0; //Número de pantalla


//Variables ESP-NOW
int id_pcb = 100;
bool resetear = 0; //Para mandar un reset
bool auto_setting = 0; //Para cambiar parámetro de autoreset
int tiempo_setting = 0; //Para cambiar el parámetro de tiempo
int aviso_setting = 0; //Para cambiar el parámetro de aviso
int contador_slave = 0; //Para guardar el valor del contador del esclavo
String success;//Varible para saber que el mensaje se ha entregado
uint8_t broadcastAddress1[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };//Direccion MAC donde queremos mandar los datos

//Estructura para enviar datos
typedef struct struct_message {
    int id;
    bool rst;//resetear
    bool aut;//autoreset
    int cnt;//contador
    int set_tiempo;//ajustar tiempo
    int set_aviso;//ajustar aviso
} struct_message;

struct_message datos_slave;//creamos estructura para RECIBIR datos del esclavo
struct_message datos_master;//creamos estructura para MANDAR los datos del maestro

esp_now_peer_info_t peerInfo;

//Objetos
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);//Pantalla OLED 0.91


//FUNCIONES
// ESP-NOW Funcion Callback cuando mandamos datos
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    //Serial.print("\r\nLast:\t");
    //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery SL OK" : "Fallo Entrega en ESCLAVO");
    if (status == 0) {
        success = "Envio a Esclavo OK :)";
    }
    else {
        success = "Envio a Esclavo NOK :(";
    }
}
// ESP-NOW Funcion Callback cuando recibimos datos
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    memcpy(&datos_slave, incomingData, sizeof(datos_slave));
    //Serial.print("Bytes on MASTER: ");
    //Serial.println(len);
    resetear = datos_slave.rst;
    auto_setting = datos_slave.aut;
    tiempo_setting = datos_slave.set_tiempo;
    aviso_setting = datos_slave.set_aviso;
    contador_slave = datos_slave.cnt;
    //DEBUGGER
    Serial.print(datos_slave.id);
    Serial.print("--> rst: ");
    Serial.print(resetear);
    Serial.print(" - aut: ");
    Serial.print(auto_setting);
    Serial.print(" - set_tiempo: ");
    Serial.print(tiempo_setting);
    Serial.print(" - set_aviso: ");
    Serial.print(aviso_setting);
    Serial.print(" - cnt: ");
    Serial.println(contador_slave);
}

//Funcion para actualizar el OLED
void actualizarOLED(int menu)
{
    oled.clearDisplay(); // clear display
    if (menu == 0)
    {
        contador_max = contador_max_temp;
        oled.setTextSize(4);
        oled.setCursor(35, 5);       // set position to display
        oled.println(String(decenas)); // set text
        oled.setCursor(60, 5);       // set position to display
        oled.println(String(unidades)); // set text 
        oled.setTextSize(1);
        oled.setCursor(90, 0);       // set position to display
        oled.println("T: "); // set text
        oled.setTextSize(2);
        oled.setCursor(100, 0);       // set position to display
        oled.println(String(contador_max - 1)); // set text
        oled.setTextSize(1);
        oled.setCursor(90, 18);       // set position to display
        oled.println("A: "); // set text
        oled.setTextSize(2);
        oled.setCursor(100, 18);       // set position to display
        oled.println(String(segundos_aviso)); // set text
        if (autoreset) {
            oled.setTextSize(1);
            oled.setCursor(0, 25);       // set position to display
            oled.println("AUTO"); // set text
        }
    }
    if (menu == 1)
    {
        if (estadoBOT_UP) {
            contador_max_temp += 10;
            if (contador_max_temp > 99) {
                contador_max_temp = 11;
            }
            estadoBOT_UP = 0;
        }
        oled.setTextSize(1);
        oled.setCursor(90, 0);       // set position to display
        oled.println("T: "); // set text
        oled.setTextSize(2);
        oled.setCursor(100, 0);       // set position to display
        oled.println(String(contador_max_temp - 1)); // set text
    }
    if (menu == 2)
    {
        if (estadoBOT_UP) {
            segundos_aviso += 5;
            if (segundos_aviso > contador_max) {
                segundos_aviso = 0;
            }
            estadoBOT_UP = 0;
        }
        oled.setTextSize(1);
        oled.setCursor(90, 18);       // set position to display
        oled.println("A: "); // set text
        oled.setTextSize(2);
        oled.setCursor(100, 18);       // set position to display
        oled.println(String(segundos_aviso)); // set text
    }
    if (menu == 3)
    {
        if (estadoBOT_UP) {
            autoreset = !autoreset;
            estadoBOT_UP = 0;
        }

        oled.setTextSize(2);
        oled.setCursor(50, 18);       // set position to display
        if (autoreset) {
            oled.println("ON"); // set text
            oled.setTextSize(1);
            oled.setCursor(0, 25);       // set position to display
            oled.println("AUTO"); // set text
        }
        else {
            oled.println("OFF"); // set text
        }

    }
    if (menu == 4)
    {
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
    oled.println("-MS");
    oled.display();              // display on OLED
}

//Funcion para verificar si activamos la bocin  
void eval_bocina()
{
    if (contador == segundos_aviso)
    {
        //Serial.println("Aviso 5 segundos");
        activar_aviso = 2;
    }
    if (contador == 0)
    {
        //Serial.println("Piiiiiii");
        activar_final = 2;
    }
}
//Funcion para activar la bocina
void activar_bocina()
{
    if (tiempo_bocina <= tiempo_actual)
    {
        digitalWrite(BOCINA, 1);

    }
    if (activar_aviso == 2)
    {
        digitalWrite(BOCINA, 0);
        tiempo_bocina = tiempo_actual + 500;
        activar_aviso = 0;

    }
    if (activar_final == 2)
    {
        digitalWrite(BOCINA, 0);
        tiempo_bocina = tiempo_actual + 1500;
        activar_final = 0;
    }
}

//Funcion autoreset
void eval_autoreset()
{
    if (contador == 0 && autoreset == true)
    {
        contador = contador_max;
    }
    else if (contador == 0 && autoreset == false)
    {
        contador = 1;
    }
}
//Funcion actualizar display
void display(int dec, int unds)
{
    bool display_unds[8] = { numero_lcd[unds][0],numero_lcd[unds][1],numero_lcd[unds][2],numero_lcd[unds][3],
                            numero_lcd[unds][4],numero_lcd[unds][5],numero_lcd[unds][6],numero_lcd[unds][7] };
    bool display_dec[8] = { numero_lcd[dec][0],numero_lcd[dec][1],numero_lcd[dec][2],numero_lcd[dec][3],
                            numero_lcd[dec][4],numero_lcd[dec][5],numero_lcd[dec][6],numero_lcd[dec][7] };

    digitalWrite(SEG_UN_a, display_unds[7]);
    digitalWrite(SEG_UN_b, display_unds[6]);
    digitalWrite(SEG_UN_c, display_unds[5]);
    digitalWrite(SEG_UN_d, display_unds[4]);
    digitalWrite(SEG_UN_e, display_unds[3]);
    digitalWrite(SEG_UN_f, display_unds[2]);
    digitalWrite(SEG_UN_g, display_unds[1]);
    digitalWrite(SEG_UN_dot, display_unds[0]);

    digitalWrite(SEG_DC_a, display_dec[7]);
    digitalWrite(SEG_DC_b, display_dec[6]);
    digitalWrite(SEG_DC_c, display_dec[5]);
    digitalWrite(SEG_DC_d, display_dec[4]);
    //digitalWrite(SEG_DC_e, display_dec[3]);
    //digitalWrite(SEG_DC_f, display_dec[2]);
    digitalWrite(SEG_DC_g, display_dec[1]);
    digitalWrite(SEG_DC_dot, display_dec[0]);
}

//Interrupciones
void IRAM_ATTR ISR_BOT_CH() //Boton cambio de funcion #####NO FUNCIONA PRINTLN (resetea el micro) 
{
    if (millis() - tiempo_previo_boton > timeThreshold)
    {
        estadoBOT_CH = 1;
        if (numScreen < 4) {
            numScreen++;
        }
        else {
            numScreen = 0;
        }
        tiempo_previo_boton = millis();
    }


}
void IRAM_ATTR ISR_BOT_UP() //Boton cambio de opcion/Resetear el contador#####NO FUNCIONA PRINTLN (resetea el micro) 
{
    if (millis() - tiempo_previo_boton > timeThreshold)
    {
        estadoBOT_UP = 1;
        tiempo_previo_boton = millis();
    }
}



void setup()
{
    /*Iniciamos monitor serie*/
    Serial.begin(115200);
    /*Definimos frecuencia de trabajo del micro*/
    setCpuFrequencyMhz(cpu_freq_mhz);

    /*Inicializamos Reles*/
    pinMode(SEG_UN_a, OUTPUT);
    pinMode(SEG_UN_b, OUTPUT);
    pinMode(SEG_UN_c, OUTPUT);
    pinMode(SEG_UN_d, OUTPUT);
    pinMode(SEG_UN_e, OUTPUT);
    pinMode(SEG_UN_f, OUTPUT);
    pinMode(SEG_UN_g, OUTPUT);
    pinMode(SEG_UN_dot, OUTPUT);
    pinMode(SEG_DC_a, OUTPUT);
    pinMode(SEG_DC_b, OUTPUT);
    pinMode(SEG_DC_c, OUTPUT);
    pinMode(SEG_DC_d, OUTPUT);
    //pinMode(SEG_DC_e, OUTPUT);
    //pinMode(SEG_DC_f, OUTPUT);
    pinMode(SEG_DC_g, OUTPUT);
    pinMode(SEG_DC_dot, OUTPUT);

    pinMode(BOCINA, OUTPUT);//Bocina

    /*Pines botones*/
    pinMode(BOT_CH, INPUT_PULLUP);//Boton cambio menu
    pinMode(BOT_UP, INPUT_PULLUP);//Botion cambio opcion


    /*Conectamos a WIFI*/
    WiFi.mode(WIFI_STA);//Debemos inicializar WIFI antes de ESP-NOW
    //WiFi.begin(ssid, password);
    //if (WiFi.waitForConnectResult() != WL_CONNECTED)
    // {
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
    attachInterrupt(BOT_CH, ISR_BOT_CH, FALLING);//Boton para cambio de funcion
    attachInterrupt(BOT_UP, ISR_BOT_UP, FALLING);//Boton para resetear el tiempo / cambio de opcion

    /*********************Inicializamos OLED***************/
    if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("failed to start SSD1306 OLED"));
        while (1);
    }
    delay(1000);                  // wait two seconds for initializing
    oled.clearDisplay();          // clear display
    delay(200);
    oled.setTextSize(2);         // set text size
    oled.setTextColor(WHITE);    // set text color
    oled.setCursor(0, 0);         // set position to display
    oled.println(" Marcador ");   // set text
    oled.setCursor(0, 17);        // set position to display
    oled.println(" v0.1 MS");       // set text
    oled.display();               // display on OLED
    delay(1000);
    /*********************FIN Inicializamos OLED*****************/

     /* DEBUGGER */
    frecuencia = getCpuFrequencyMhz();  // In MHz  
    Serial.print(frecuencia);
    Serial.println(" MHz");
}

/*####################### BUCLE PRINCIPAL ######################*/
void loop()
{
    tiempo_actual = millis();
    activar_bocina();

    if (tiempo_actual - tiempo_previo >= intervalo)
    {
        tiempo_previo = tiempo_actual;

        //Reseteamos el tiempo de forma local    
        if (numScreen == 0 && estadoBOT_UP == 1) {
            contador = contador_max;
            estadoBOT_UP = 0;
        }
        //Reseteamos el tiempo de forma remota   
        if (resetear == 1) {
            contador = contador_max;
            resetear = 0;
        }

        contador--;

        decenas = (contador - (contador % 10)) / 10;
        unidades = contador - decenas * 10;

        display(decenas, unidades);//Actualizar LCD
        actualizarOLED(numScreen);


        /*Enviamos info ESP-NOW*/
        //Actualizar datos de envio
        aviso_setting = segundos_aviso;
        tiempo_setting = contador_max;
        auto_setting = autoreset;
        datos_master.id = id_pcb;
        datos_master.cnt = contador;
        datos_master.rst = resetear;
        datos_master.aut = auto_setting;
        datos_master.set_tiempo = tiempo_setting;
        datos_master.set_aviso = aviso_setting;
        // Enviamos mensaje ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t*)&datos_master, sizeof(datos_master));
        if (result == ESP_OK) {
            //Serial.println("Envio a esclavo OK");
        }
        else {
            Serial.println("Envio a esclavo NOK");
        }
        /*FIN Enviamos info ESP-NOW*/
        eval_bocina();
        eval_autoreset();
    }

}

