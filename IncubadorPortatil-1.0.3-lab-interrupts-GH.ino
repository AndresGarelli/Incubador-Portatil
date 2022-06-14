//usa la biblioteca bajada de https://www.naylampmechatronics.com/blog/35_Tutorial--LCD-con-I2C-controla-un-LCD-con-so.html
//https://codeload.github.com/marcoschwartz/LiquidCrystal_I2C/zip/master

char vers[] = "1.0.3-lab-interrupts";

#include <DallasTemperature.h>
#include <OneWire.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <PID_v1.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <EEPROM.h>
#include <ESP_Mail_Client.h>

#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465

/* The sign in credentials */
#define AUTHOR_EMAIL "xxxxxxxxx@gmail.com"
#define AUTHOR_PASSWORD "xxxxxxxxxxxxx"
String asunto;
String hora;

// Datos de red
char ssid1[] = "xxxxxxxx";
char pass1[] = "";
char ssid2[] = "xxxxxxxx";
char pass2[] = "xxxxxxx";

bool mailControlEnviado = false;
bool resetTemp = false;
bool mailEncendido = true;
bool mailControl = false;
float temp1;
float tempMAX;
float tempMIN;
float resistencia = 1023;
unsigned long previousMillisMedicion = 0;   //esto es para determinar el tiempo entre medidas de sensor
unsigned long timeFromReading;
bool isTempOK = false;  //evalua si el valor de temp es diferente a -127 (true). En cada ciclo, deja de medir temp cuando recibe un valor diferente a -127 (que es la respuesta cuando el sensor está desconectado)
unsigned long prevLoad = 0; //para enviar datos
unsigned long prevLoad2 = 0; //para enviar datos
unsigned long prevLCD = 0;       //actualizacion LCDpara refresh serialport
//esto es para determinar el tiempo entre medidas de sensor
unsigned long prevCheck = 0;
unsigned long previousBLINK = 0;
//unsigned long ultimaAct =0;
//unsigned long periodoAct = 8640000; //1 dia
unsigned long windowStartTime = 0;
//bool enviar= false;
//bool timeToSync=false;
bool todoBien = true;
bool ventiladorOK = true;
bool on_state = false;
long countVent = 10;
bool recienEncendido = true;
float velocidad;           //velocidad del ventilador
long prevVuelta = 0;
String estado;
int vent1 = 1; //variable para registro error ventilador
int sens1 = 1; //variable para registro de error de sensor
int porcentaje;
int porcentaje2;
String percent;
unsigned long lcdOnTime;
bool motorApagado = false; //variable de registro de motor apagado cuando output es 0
bool prevVelocidadState = 1;
bool velocidadState = 1;
long encendioMotorTime;
long bajoTime;  //variable para registro de velocidad menor 120
long apagoMotorTime;
int valorEeprom; //variable para leer el dato almacenado en eeprom

bool WifiConectado;


int error1 = 0; //registro de veces que da -127 y descarta el valor
int error2 = 0;  // registro de veces que da -127
//////variables para boton
int botonState = HIGH;
int prevBotonState = HIGH;
bool ignoreUp = false;
unsigned long btnUpTime;
unsigned long btnDnTime;
int holdTime = 5000;
//
//const int numReadings = 30;       //número de lecturas que promedia
//float readings[numReadings];      // the readings from the analog input. Crea un array(readings) con numReadings elementos, en este caso, 30.
//int readIndex = 0;                // the index of the current reading
//float total = 0;
//float average = 0;


//Las resistencias para calentar están controladas por un PID
//Variables para el PID
//Constantes para PID
double Setpoint, Input, Output;
//double Kp = 65, Ki = 10, Kd = 1; //antes kp 150
//double Kp = 0, Ki = 2, Kd = 1;
//double Kp = 2, Ki = 2, Kd = 1;
//double Kp = 20, Ki = 2, Kd = 1; este OK, poco overshooting
//double Kp = 20, Ki = 1, Kd = 1; no me gusta, va saltando mucho
//double Kp = 100, Ki = 10, Kd = 1; //ESTA ES LA QUE ESTABA EN USO ANTES DE PONER LOS VALORES ACTUALES
double Kp = 800, Ki = 20, Kd = 900;
PID myPIDcalor(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_M, DIRECT);
int prevOutput;
int rOutput;


///////DEFINICION DE PINES
const int pinRelayCalor = 12;   //es D6 en nodemcurelay resistencias
const int pinVent = 13;    //es D7 en nodemcu
//const int buzz = 13;            //buzzer. D7 in nodemcu
//const int buzzLED = 15;         //nodemcu D8 led testigo de alarma
//pin = 4; // D2 queda libre para CO2
//const int puerta = 16; //pin sensor puerta abierta, D0
//const int resetPin = 2; //pin para botón reset alarma, D4
const int LED = 2; //led azul para boot
const int pinMotor = 4; //pin del conrol del motor del ventilador es D2
const int pinLCD = 3; //es el pin RX

////CODIGO DS18B20
// Pin donde se conecta el bus 1-Wire
const int pinDatosDQ = 14;   ///////////////////D5
// Instancia a las clases OneWire y DallasTemperature
OneWire oneWireObjeto(pinDatosDQ);
DallasTemperature sensorDS18B20(&oneWireObjeto);

// Variables con las direcciones únicas de los 4 sensores DS18B20
DeviceAddress incubadorPortatil = {0x28, 0xFF, 0xB5, 0xB8, 0x21, 0x17, 0x03, 0x07};
String direccion = "28616412266819B3";
//DeviceAddress fusible = {0x28, 0xFF, 0xDC, 0xA9, 0x21, 0x17, 0x03, 0x52};

////FIN DS18B20

//Crear el objeto lcd  dirección  0x3F y 16 columnas x 2 filas
LiquidCrystal_I2C lcd(0x27, 16, 2);
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, POSITIVE);




// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
/*
  //Week Days
  String weekDays[7]={"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

  //Month names
  String months[12]={"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
*/


void setup() {
  
  WiFi.begin(ssid1, pass1);
  Serial.begin(115200);
  Serial.println ("setup");
  int trial = 0;
  pinMode(LED, OUTPUT);
  while (trial < 3) {
    digitalWrite(LED, LOW);
    delay(300);
    digitalWrite(LED, HIGH);
    delay(100);
    trial++;
  }
  trial = 0;
  analogWriteFreq(200);

  pinMode(pinRelayCalor, OUTPUT);
  digitalWrite(pinRelayCalor, LOW);
  pinMode (pinVent, INPUT_PULLUP);
  pinMode (pinMotor, OUTPUT);
  digitalWrite(pinMotor, HIGH);
  pinMode (pinLCD, INPUT_PULLUP);



  // Inicializar el LCD
  Wire.begin(0, 5); //inicia biblioteca wire, asigna SDA a pin0 (D3) y SCL a pin 5 (D1);
  lcd.init();
  //  lcd.begin();
  //Encender la luz de fondo.
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("buscando red");
  while (WiFi.status() != WL_CONNECTED & trial < 5)
  { lcd.setCursor(trial, 1);
    lcd.print(".");
    delay(500);
    trial++;
  }
  
  if(WiFi.status()!= WL_CONNECTED){
     WiFi.disconnect(true);
     WiFi.begin(ssid2, pass2);
  }
    while (WiFi.status() != WL_CONNECTED & trial < 10)
  { lcd.setCursor(trial, 1);
    lcd.print(".");
    delay(500);
    trial++;
  }

  sensorDS18B20.begin();
  sensorDS18B20.setWaitForConversion(false);
  int contar = sensorDS18B20.getDeviceCount();
  sensorDS18B20.requestTemperatures();
  lcd.clear();
  while (contar != 1) {
    lcd.setCursor(0, 0);
    lcd.print("ERROR SENSOR");
    lcd.setCursor(0, 1);
    lcd.print(contar);
    lcd.print(" sensores");
    delay(1000);
  }

  lcd.clear();
  //lcd.print("Port. ");
  lcd.print(vers);
  lcd.setCursor(0, 1);
  delay(1000);
  if (WiFi.status() == WL_CONNECTED) {
    lcd.print(WiFi.SSID());
    //Serial.print(ssid);
  } else {
    lcd.print("no conectado");
  }
  delay(1500);
  /*
    lcd.clear();
    lcd.print("MOSCAS 1.0.1");
    lcd.setCursor(0, 1);
    lcd.print(contar);
    lcd.print(" sensor OK!");
    delay(1500);
  */
  // IMPORTANTE: antes conectaba con client.wifiConnection, pero de esa manera, si no tiene red,
  // queda colgado en el setup. con WiFi.begin, si  no hay red, entra en el loop
  // client.wifiConnection(WIFISSID, PASSWORD);
  Output = 0;
  rOutput = 0;
  prevOutput = 1; //le asigno 1 porque si output es 0 desde el inicio, nunca va a asignar un valor a apagoMotor



  EEPROM.begin(4);
  valorEeprom = EEPROM.read(0);
  if (valorEeprom == 18 || valorEeprom == 25 || valorEeprom == 30) {
    Setpoint = valorEeprom;
  } else {
    Setpoint = 25.0;
    for (int i = 0; i < 4; i++) {
      EEPROM.write(i, 0);
    }
  }
  EEPROM.end();

  tempMAX = Setpoint;
  tempMIN = Setpoint;

  //CAMBIADA. antes era Setpoint = (Setdeseado - 0.10);
  //indica el rango de tiempo en mseg que pueden estar encendidas las resistencias
  myPIDcalor.SetOutputLimits(0, 1023); //indiqué que el valor máximo fuera igual al sample time
  myPIDcalor.SetSampleTime(2000);        // tiempo entre cálculos de PID

  myPIDcalor.SetMode(AUTOMATIC);
  lcdOnTime = millis();
  //lcd.clear();
  encendioMotorTime = millis();

  // Initialize a NTPClient to get time
  timeClient.begin();
  timeClient.setTimeOffset(-10800);
  timeClient.update();

//  for (int i = 0; i < numReadings; i++) {
//    readings[i] = 0;
//  }

attachInterrupt(pinVent, tachometer,FALLING);
}

void loop() {


  /*if (millis()-ultimaAct >= periodoAct || ultimaAct==0){
      //timeToSync=true;
      syncClock();
    }*/

  checkButton();

  if (WiFi.status() == WL_CONNECTED) {
    WifiConectado = true;
  } else {
    WifiConectado = false;
  }

  if (millis() - lcdOnTime > 120000) {
    lcd.noBacklight();
  }

  if (millis() - prevCheck >= 15000) {
    checkWifi();
  }

  if (WifiConectado == true) {
    blinkslow();
  } else {
    blinkfast();
  }

  if (micros() - prevVuelta > 2000000) {
    velocidad = countVent * 15; //dos counts por vuelta, cuento durante dos segundos: countVent*15 es RPM
//    total = total - readings[readIndex];
//    readings[readIndex] = velocidad;
//    total = total + readings[readIndex];
//    readIndex = readIndex + 1;
//    if (readIndex >= numReadings) {
//      readIndex = 0;
//    }
//    average = total / (float)numReadings;
    prevVuelta = micros();
    countVent = 0;
  }


  timeFromReading = millis() - previousMillisMedicion;

  if (timeFromReading < 1500) {
    isTempOK = false;
  }

  if (timeFromReading >= 1500 && timeFromReading < 2000 && isTempOK == false) {
    float temp = sensorDS18B20.getTempC(incubadorPortatil);
    Serial.print("temp ");
    Serial.println(temp);
    if (temp != -127) {
      temp1 = temp;
      isTempOK = true;
    } else {
      error1++;
      Serial.println("error1");
      sensorDS18B20.setWaitForConversion(true);  //it will delay until the temp is converted. should be 750ms if DS18B20 is original, but is much shorter in my case because it is not original
      //this loop will try to get a valid temp value, so we need to wait for the required time while we are in the loop - ie, while we do not get a valid temp value
      sensorDS18B20.requestTemperatures();
    }
  }

  if (timeFromReading >= 2000) {
    previousMillisMedicion = millis();
    
    if (isTempOK == false) {
      float temp = sensorDS18B20.getTempC(incubadorPortatil);
      temp1 = temp;
            if (temp == -127) {
        error2++;
        
      }
    }
    sensorDS18B20.setWaitForConversion(false); // it will not wait until temp is converted, so will proceed to the next step without delay.
    sensorDS18B20.requestTemperatures();
    if (temp1 > 10 && temp1 < 35 ) {
      myPIDcalor.SetMode(AUTOMATIC);
      todoBien = true;
    } else {
      todoBien = false;
      
      myPIDcalor.SetMode(MANUAL);
      Output = 0;
      sens1 = 2;
    }
  }
  unsigned long ahora = millis();
  if (ahora - windowStartTime > 1000) {
    //    Serial.println(puertaAbierta);
  }

  if (temp1 > tempMAX) {
    tempMAX = temp1;
  } else if (temp1 < tempMIN) {
    tempMIN = temp1;
  }

  prevOutput = rOutput;
  Input = temp1;
  myPIDcalor.Compute();
  yield();

  if (Output != 0 && Output  < 11) {
    porcentaje = 1;
  } else {
    porcentaje = round(Output * 100 / 1023);
  }

  resistencia = 1023 - Output;
  analogWrite(pinRelayCalor, resistencia);


  prevVelocidadState = velocidadState;

  if (velocidad < 120.0) {
    velocidadState = 0;
  } else {
    velocidadState = 1;
    myPIDcalor.SetMode(AUTOMATIC);
    ventiladorOK = true;
  }

  //si bajó la velocidad, marca el tiempo
  if (prevVelocidadState != velocidadState && velocidadState == 0) {
    bajoTime = millis();
  }

  // si pasaron 7.5seg de encendido y 5 segundos desde q volvio a arrancar el motor luego de haber apagado por Output=0
  //evalúa si la velocidad sigue menor que 120 y si pasaron 5 segundos de esa situación, apaga el PID.

  if (motorApagado == false && millis() > 7500 && millis() - encendioMotorTime > 5000) {
    if (prevVelocidadState == velocidadState &&  velocidadState == 0) {
      if (millis() - bajoTime > 5000) {
        myPIDcalor.SetMode(MANUAL);
        Output = 0;
        ventiladorOK = false;
        vent1 = 0;
      }
    }
  }

  //si no calienta resistencias, apaga el motor. Si estaba apagado y ahora output es distinto de 0, enciende el motor y cuenta tiempo

  rOutput = (int)Output;
  if (rOutput != 0) {
    digitalWrite(pinMotor, HIGH);
    motorApagado = false;
  }

  if (prevOutput == 0 && rOutput > 0) {
    encendioMotorTime = millis();
  }

  if (prevOutput != rOutput && rOutput == 0) {
    apagoMotorTime = millis();
    sens1 = 0;
  }

  if (prevOutput == rOutput && rOutput == 0) {
    if (ventiladorOK == false) {
      digitalWrite(pinMotor, HIGH);
    } else if (ventiladorOK == true) {
      if (millis() - apagoMotorTime > 300000) {
        digitalWrite(pinMotor, LOW);      //CAMBIAR A LOW PARA QUE APAGUE EL MOTOR CUANDO OUTPUT ES 0. NO SE SI ES CONVENIENTE PORQUE NO SE HOMOGEINIZA LA TEMPERATURA
        motorApagado = true;
      }
    }
  }




  if (temp1 >= Setpoint + 0.3) {
    estado = "ALTO";
  } else if (temp1 <= Setpoint - 0.3) {
    estado = "BAJO";
  } else {
    estado = "JOYA";
  }

  if (millis() - prevLoad2 >= 60000) {
    enviarTempTW();
    prevLoad2 = millis();
  }

  if (porcentaje < 10) {
    percent = "  ";
  } else if (porcentaje >= 10 && porcentaje < 100) {
    percent = " ";
  } else if (porcentaje == 100) {
    percent = "";
  }

  if (timeClient.getMinutes() == 00 ) {
    if (mailControlEnviado == false) {
      if (timeClient.getHours() == 21 || timeClient.getHours() == 9) {
        asunto =  timeClient.getFormattedTime() + " - Tmin: " + String(tempMIN) + " - Tmax: " + String(tempMAX) + " - error1: " + String(error1) + " - error2: " + String(error2);
        resetTemp = true;
        mailControl = true;
        error1 = 0;
        error2 = 0;
        enviarEmail();
      }
    }
  } else {
    mailControlEnviado = false;
  }


  if (millis() - prevLCD >= 2000) {
    Serial.println("LCD");
    prevLCD = millis();
    if (todoBien == true && ventiladorOK == true) {
      lcd.setCursor(0, 0);
      lcd.print("SP");
      lcd.print(Setpoint, 1);
      lcd.print(" |");
      lcd.print(percent);
      lcd.print(porcentaje);
      lcd.print("%");
      lcd.setCursor(12, 0);
      if (error2 == 0) {
        lcd.print("|RED");
      } else {
        lcd.print("|");
        lcd.print(error2);
        lcd.print("  ");
      }

      lcd.setCursor(0, 1);
      lcd.print("T:");
      lcd.print(temp1, 1);
      lcd.print(" |");
      lcd.print(estado);
      //lcd.print(velMotor);
      lcd.print("| ");
      if (WifiConectado == true) {
        lcd.print("SI");
      } else {
        lcd.print("NO");
      }
    } else if (todoBien == true && ventiladorOK == false) {
      lcdOnTime = millis();
      lcd.backlight();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ERROR VENTILADOR");
      lcd.setCursor(0, 1);
      lcd.print("Output ");
      lcd.print((Output * 100 / 1023), 0);
      lcd.print("%");
    } else if  (todoBien == false) {
      lcdOnTime = millis();
      lcd.backlight();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ERROR sensor!");
      lcd.setCursor(0, 1);
      lcd.print(" OUT ");
      lcd.print((Output * 100 / 1023), 0);
      lcd.print("%");
      lcd.print(" T:");
      lcd.print(temp1, 1);
    }
  }

  if (recienEncendido == true && WifiConectado == true) {
    timeClient.update();
    int aa = millis() / 1000;
    mailEncendido = true;
    asunto = timeClient.getFormattedTime() + " - Inicio Incubador - " + String(aa) + " segs";
    enviarEmail();
  }

}

void checkButton() {
  botonState = digitalRead(pinLCD);

  // Test for button pressed and store the down time
  if (botonState == LOW && prevBotonState == HIGH && (millis() - btnUpTime) > 50)
  {
    btnDnTime = millis();
  }

  // Test for button release and store the up time
  if (botonState == HIGH && prevBotonState == LOW && (millis() - btnDnTime) > 50) {
    btnUpTime = millis();
    if (ignoreUp == false) {
      lcdOnTime = millis();
      lcd.backlight();
    } else {
      ignoreUp = false;
    }
  }
  // Test for button held down for longer than the hold time
  if (botonState == LOW && (millis() - btnDnTime) > holdTime ) {
    changeSetpoint();
    ignoreUp = true;
    btnDnTime = millis();
  }

  prevBotonState = botonState;
}

void blinkslow() {
  if (millis() - previousBLINK >= 0 && millis() - previousBLINK < 50) {
    digitalWrite(LED, LOW);
  } else if (millis() - previousBLINK >= 50 && millis() - previousBLINK < 100) {
    digitalWrite(LED, HIGH);
  } else if (millis() - previousBLINK >= 100 && millis() - previousBLINK < 150) {
    digitalWrite(LED, LOW);
  }
  else if (millis() - previousBLINK >= 150 && millis() - previousBLINK < 5000) {
    digitalWrite(LED, HIGH);
  }
  else {
    previousBLINK = millis();
  }
}


void blinkfast() {
  if (millis() - previousBLINK >= 0 && millis() - previousBLINK < 50) {
    digitalWrite(LED, LOW);
  } else if (millis() - previousBLINK >= 50 && millis() - previousBLINK < 100) {
    digitalWrite(LED, HIGH);
  } else (previousBLINK = millis());
}


void checkWifi() {
  if (WifiConectado == false) {
    
    if (WiFi.SSID() == ssid1) {
      WiFi.disconnect(true);
      WiFi.begin(ssid2, pass2);
    }
    else if (WiFi.SSID() == ssid2) {
      WiFi.disconnect(true);
      WiFi.begin(ssid1, pass1);
    }
  }
  prevCheck = millis();
  //Serial.println(WiFi.SSID());
  //Serial.println("");
}


void enviarTempTW() {
  String pedido = "http://xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
  //Serial.println(direccion);
  pedido = pedido + direccion;
  pedido = pedido + "&temperatura=";
  pedido = pedido + String(temp1);

  if (WiFi.status() == WL_CONNECTED) {

    HTTPClient http;

    // Your Domain name with URL path or IP address with path
    http.begin(pedido);
    int httpCode = http.GET();

    /*
      if (httpCode>0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpCode);
      Serial.println("");
       }
       else {
         Serial.print("Error code: ");
         Serial.println(httpCode);
         Serial.println("");
             }*/
    // Free resources
    http.end();
  }
  else {
    //    Serial.println("WiFi Disconnected");
  }
}


void changeSetpoint() {
  if (Setpoint == 25) {
    Setpoint = 30;
  } else if (Setpoint == 30) {
    Setpoint = 18;
  } else if (Setpoint == 18) {
    Setpoint = 25;
  }
  tempMIN = Setpoint;
  tempMAX = Setpoint;
  EEPROM.begin(4);
  valorEeprom = Setpoint;
  EEPROM.write(0, valorEeprom);
  EEPROM.commit();
  int check;
  check = EEPROM.read(0);
  if (check == valorEeprom) {
    for (int i = 0; i < 2; i++) {
      lcd.noBacklight();
      delay(300);
      lcd.backlight();
      delay(300);
    }
  }
}

void enviarEmail() {
  if (WiFi.status() == WL_CONNECTED) {
    timeClient.update();
    SMTPSession smtp;
    smtp.debug(0);

    /* Declare the session config data */
    ESP_Mail_Session session;

    /* Set the session config */
    session.server.host_name = SMTP_HOST;
    session.server.port = SMTP_PORT;
    session.login.email = AUTHOR_EMAIL;
    session.login.password = AUTHOR_PASSWORD;
    //session.login.user_domain = "mydomain.net";

    /* Declare the message class */
    SMTP_Message message;

    /* Set the message headers */
    message.sender.name = "Incubador portatil";
    message.sender.email = AUTHOR_EMAIL;
    message.subject = asunto.c_str(); //agregué .c_str() porque no podía convertir String a char*
    message.addRecipient("Andres", "andres.garelli@gmail.com");

    //message.text.content = "This is simple plain text message";
    message.text.charSet = "us-ascii";
    message.text.transfer_encoding = Content_Transfer_Encoding::enc_7bit;
    message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_high;
    message.response.notify = esp_mail_smtp_notify_success | esp_mail_smtp_notify_failure | esp_mail_smtp_notify_delay;

    /* Set the custom message header */
    //message.addHeader("Message-ID: <abcde.fghij@gmail.com>");

    /* Connect to server with the session config */
    if (!smtp.connect(&session))
      return;

    /* Start sending Email and close the session */
    if (!MailClient.sendMail(&smtp, &message)) {
      //Serial.println("Error sending Email, " + smtp.errorReason());

    } else {
      if (mailEncendido == true) {
        recienEncendido = false;
        mailEncendido = false;
      }
      if (mailControl == true) {
        mailControlEnviado = true;
        mailControl = false;
        tempMIN = Setpoint;
        tempMAX = Setpoint;
      }

    }
  }
}

IRAM_ATTR void tachometer(){
  countVent++;
}
