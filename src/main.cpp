#include <Arduino.h>

#include <HX711.h> //Library created by bogde
#include <openscale.h> //Contains EPPROM locations for settings
#include <Wire.h> //Needed to talk to on board TMP102 temp sensor
#include <EEPROM.h> //Needed to record user settings
#include <OneWire.h> //Needed to read DS18B20 temp sensors
#include <LowPower.h> //https://github.com/rocketscream/Low-Power
#include <SoftwareSerial.h>
#include <Akeru.h>

#define TX 4
#define RX 5
// Sigfox instance management 
Akeru akeru(RX, TX);
//SoftwareSerial akeru(RX, TX);
#include <DallasTemperature.h>
//15 ms, 30 ms, 60 ms, 120 ms, 250 ms, 500 ms, 1 s, 2 s, 4 s, 8 s, and forever

#include <avr/sleep.h> //Needed for sleep_mode
#include <avr/power.h> //Needed for powering down perihperals such as the ADC/TWI and Timers


#define ATSIGFOXTX "AT$SS="
#define N 75//nombre de cycle de veille.
#define MOD 10 // nombre de cycle de veille entre 2 mesures
//#define SERIAL_DEBUG 1
#define ONE_WIRE_BUS 6

const byte statusLED = 13;  //Flashes with each reading

HX711 scale; //Setup interface to scale
//OneWire remoteSensor(4);  //Setup reading one wire temp sensor on pin 4 (a 4.7K resistor is necessary)
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//byte remoteSensorAddress[8];
//boolean remoteSensorAttached = true;
boolean FirstRead = true; // permet d'envoyer un message sigfox au démarrage du programme

unsigned int count = 0;
float currentReading=0;
float NewReading=0;
int LocalTemperature=0;
float scalesum=0;
int tempsum=0;
float RTempsum=0;
char first=1;
char event=0;
float scales ;
float temp ;
String LTemps_hex;
String RTemps_hex;
String poids_g_hex;
String event_hex;
String msg;
String ATCommand ;
float RemoteTemp;

String longtoHex(long l)
{
  byte * b = (byte*) & l;
  
  String bytes = "";
  for (int i=0; i<4; i++)
  {
    if (b[i] <= 0xF) // single char
    {
      bytes.concat("0"); // add a "0" to make sure every byte is read correctly
    }
    bytes.concat(String(b[i], 16));
  }
  return bytes;
}
String inttoHex(int i)
{
  byte * b = (byte*) & i;
  
  String bytes = "";
  for (int j=0; j<2; j++)
  {
    if (b[j] <= 0xF) // single char
    {
      bytes.concat("0"); // add a "0" to make sure every byte is read correctly
    }
    bytes.concat(String(b[j], 16));
  }
  return bytes;
}
String uinttoHex(unsigned int ui)
{
  byte * b = (byte*) & ui;
  
  String bytes = "";
  for (int i=0; i<2; i++)
  {
    if (b[i] <= 0xF) // single char
    {
      bytes.concat("0"); // add a "0" to make sure every byte is read correctly
    }
    bytes.concat(String(b[i], 16));
  }
  return bytes;
}
String ulongtoHex(unsigned long ul)
{
  byte * b = (byte*) & ul;
  
  String bytes = "";
  for (int i=0; i<4; i++)
  {
    if (b[i] <= 0xF) // single char
    {
      bytes.concat("0"); // add a "0" to make sure every byte is read correctly
    }
    bytes.concat(String(b[i], 16));
  }
  return bytes;
}
String floattoHex(float f)
{
  byte * b = (byte*) & f;

  String bytes = "";
  for (int i=0; i<4; i++)
  {
    if (b[i] <= 0xF) // single char
    {
      bytes.concat("0"); // add a "0" to make sure every byte is read correctly
    }
    bytes.concat(String(b[i], 16));
  }
  return bytes;
}
String doubletoHex(double d)
{
  byte * b = (byte*) & d;

  String bytes = "";
  for (int i=0; i<4; i++)
  {
    if (b[i] <= 0xF) // single char
    {
      bytes.concat("0"); // add a "0" to make sure every byte is read correctly
    }
    bytes.concat(String(b[i], 16));
  }
  return bytes;
}
String chartoHex(char c)
{
  byte *b = (byte*) & c;
  
  String bytes = "";
  if (b[0] <= 0xF) // single char
  {
    bytes.concat("0"); // add a "0" to make sure every byte is read correctly
  }
  bytes.concat(String(b[0], 16));
  return bytes;
}
String stringtoHex(char *c, int length)
{
  byte * b = (byte*) c;
  
  String bytes = "";
  for (int i=0; i<length; i++)
  {
    if (b[i] <= 0xF) // single char
    {
      bytes.concat("0"); // add a "0" to make sure every byte is read correctly
    }
    bytes.concat(String(b[i], 16));
  }
  return bytes;
}

void send_mes(float sclaes, int temp, int Rtemp, int event)
{   
    poids_g_hex=longtoHex((long)(scales*1000)); // poids non compense en gramme
    RTemps_hex = uinttoHex((short)(Rtemp*100));
    LTemps_hex = uinttoHex((short)(temp*100));
    event_hex = chartoHex((char)event);
    msg = poids_g_hex + LTemps_hex + RTemps_hex + event_hex; // Put everything together
    #ifdef SERIAL_DEBUG
      Serial.println(msg);
       #endif
    if (akeru.sendPayload(msg))
    {
      #ifdef SERIAL_DEBUG
      Serial.println("Message sent !");
       #endif
    }
    else
    {
       #ifdef SERIAL_DEBUG
      Serial.println("Message not sent !");
       #endif
    }
}

void print_mes(float sclaes, int temp, int Rtemp )
{
  
  #ifdef SERIAL_DEBUG
    Serial.print(F("Readings: Poids="));
    Serial.print(scales, 2);
    Serial.print(F("kg"));
    Serial.print(",LocalTemp=");
    Serial.print(temp);
    Serial.print(F(",RemoteTemp="));
    Serial.print(Rtemp, 2);
    Serial.println();
    Serial.flush();
    #endif
}

//Read the on board TMP102 digital temperature sensor
//Return celsius
//Code comes from bildr
float getLocalTemperature()
{
  Wire.requestFrom(tmp102Address, 2);

  byte MSB = Wire.read();
  byte LSB = Wire.read();

  //It's a 12bit int, using two's compliment for negative
  int TemperatureSum = ((MSB << 8) | LSB) >> 4;

  float celsius = TemperatureSum * 0.0625;
  return celsius;
}

void setup()
{
  pinMode(statusLED, OUTPUT);

  //Setup UART
  Serial.begin(9600);
  delay(1000);

  akeru.begin();
  delay(500);
  Wire.begin();
  sensors.begin();

   // Check TD1208 communication
  if (!akeru.begin())
  {
    Serial.println("TD1208 KO");
    while(1);
  }

  pinMode(AMP_EN, OUTPUT);
  digitalWrite(AMP_EN, LOW); //Turn on power to HX711
  
  delay(1000);
  #ifdef SERIAL_DEBUG
    //akeru.echoOn(); // comment this line to hide AT commands
    Serial.println("Test1");
  #endif
  //Réglage jauge 200 Kg
   scale.begin(2,3);
   scale.set_scale(-21000); //Calibrate scale from EEPROM value
   scale.set_offset(-586962); //Zero out the scale using a previously known zero point

/* Réglage Bosch 
  scale.set_scale(28161); //Calibrate scale from EEPROM value
  scale.set_offset(127195); //Zero out the scale using a previously known zero point*/
  #ifdef SERIAL_DEBUG
    Serial.println("Test2");
  #endif
  delay(1000);
  //Power down various bits of hardware to lower power usage  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //Power down everything, wake up from WDT
  sleep_enable();
  //Shut off Timer2, Timer1, ADC
  ADCSRA &= ~(1<<ADEN); //Disable ADC
  ACSR = (1<<ACD); //Disable the analog comparator
  DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-ADC5 pins
  DIDR1 = (1<<AIN1D)|(1<<AIN0D); //Disable digital input buffer on AIN1/0

  power_timer1_disable();
  power_timer2_disable();
  power_adc_disable();
  power_spi_disable();
}


void loop()
{
 if(count==N) 
 {
  count=0;
  scalesum = scalesum/(N/MOD);
  LocalTemperature = getLocalTemperature();
  akeru.getTemperature(&LocalTemperature);
  if(LocalTemperature<60 && LocalTemperature)  tempsum = LocalTemperature;
  
  sensors.requestTemperatures(); 
  RemoteTemp=sensors.getTempCByIndex(0);
  //RTempsum = RemoteTemp;
  if((RemoteTemp < 85)&&(RemoteTemp > -50)) RTempsum = RemoteTemp;
  print_mes(scalesum, tempsum, RTempsum);
  send_mes(scalesum, tempsum, RTempsum, event);
  scalesum = 0; 
  delay(5000);
 } 

 LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, 
                SPI_OFF, USART0_OFF, TWI_OFF);
 count++; 

 #ifdef SERIAL_DEBUG
    Serial.print(F("Count ="));
    Serial.print(count);
    Serial.println();
    Serial.flush();
  #endif

 if ((count%MOD)==0) // test de perte de poids toute les 80 secondes
 {
   event =0;
   scale.power_up();
   NewReading = scale.get_units(4);
   scale.power_down();
   scalesum = scalesum + NewReading;
  
    if((currentReading-NewReading)>1.0) // détection d'un perte de poids conséquente
    {
      event = 12; // alerte sur la détection d'une perte de poids subite (supérieure à 1Kg)
      poids_g_hex=longtoHex((long)(NewReading*1000)); // temperature non compense en gramme
      event_hex = chartoHex((char)event);
      msg = poids_g_hex + LTemps_hex + RTemps_hex + event_hex; // Put everything together
      if (akeru.sendPayload(msg))
      {
        #ifdef SERIAL_DEBUG
        Serial.println("Message sent !");
         #endif
      }
      else
      {
         #ifdef SERIAL_DEBUG
        Serial.println("Message not sent !");
         #endif
      }
      


      
    }  
    #ifdef SERIAL_DEBUG
        float dif = currentReading-NewReading;
         //Serial.print(F("Perte de poids brutale"));
        Serial.print(F("Readings: Nouveau Poids="));
        Serial.print(NewReading, 2);
        Serial.print(F("kg"));
        Serial.print(F(",Ancien poids ="));
        Serial.print(currentReading, 2);
        Serial.print(F(",Différence="));
        Serial.print(dif, 2);
        Serial.println();
        Serial.flush();
      #endif
      
    
    currentReading = NewReading;
  
 }
 
 if(FirstRead)
 {
    FirstRead = false;
    scale.power_up();
    currentReading = scale.get_units(4);
    scale.power_down();
    if(currentReading>0) scales = currentReading;
    
    akeru.getTemperature(&LocalTemperature);
    if(LocalTemperature < 85)  tempsum = LocalTemperature;
    #ifdef SERIAL_DEBUG
      Serial.print("LocalTemp=");
      Serial.print(LocalTemperature);
      Serial.println();
      Serial.flush();
    #endif
        
    sensors.requestTemperatures(); 
    RemoteTemp=sensors.getTempCByIndex(0);
    delay(1000);
    sensors.requestTemperatures(); 
    RemoteTemp=sensors.getTempCByIndex(0);
    //RTempsum = RemoteTemp;
    if((RemoteTemp < 85)&&(RemoteTemp > -50)) RTempsum = RemoteTemp;
  
    print_mes(scales, tempsum, RTempsum);
    send_mes(scales, tempsum, RTempsum, event);
 }

 
}



