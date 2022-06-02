#include "Arduino.h"
#include <WiFi.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include "sps30.h"
#include <TinyGPS++.h> // Include TinyGPS++ library
#include <HardwareSerial.h>
#include <SimpleDHT.h>
#include <esp_pm.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp32-hal-cpu.h"
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include "cert.h"
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "I2Cdev.h"
#include "MPU6050.h"

const int LabelPin1 = 5;  // the number of the pushbutton pin
const int LabelPin2 = 23;  // the number of the pushbutton pin
const int LabelPin3 = 18;  // the number of the pushbutton pin
const int LabelPin4 = 19;  // the number of the pushbutton pin

int LabelState1 = 0;
int LabelState2 = 0;
int LabelState3 = 0;
int LabelState4 = 0;

String Label1 = "Speedbump";
String Label2 = "Pothole";
String Label3 = "Trafficsignal";
String Label4 = "Roughroad";
String Label5 = "Smoothroad";

//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int Gyro_deadzone=1;     //Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
MPU6050 accelgyro(0x68); // <-- use for AD0 high

int16_t ax, ay, az,gx, gy, gz;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;



int const potPin = 35;
int potVal;
int angle;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
String formattedDate;
int voltage_offset = 20;// set the correction offset value

String DeviceID = String("M5"); //change according to the device
const char* topicsen="M5/sen"; //change according to the device
const char* topicacc="M5/acc";
const char* topic1="dev/reg";
#define LENG 31 //0x42 + 31 bytes equal to 32 bytes
unsigned char buf[LENG];
char col;
int PM01Value = 0; //define PM1.0 value of the air detector module
int PM2_5Value = 0; //define PM2.5 value of the air detector module
int PM10Value = 0; //define PM10 value of the air detector module

#define RXD2 12
#define TXD2 13
#define TXD1 27
#define RXD1 26

int pinDHT22 = 2;
SimpleDHT22 dht22;


float temperature = 0;
float humidity = 0;
  
esp_mqtt_client_config_t mqtt_cfg;
esp_mqtt_client_handle_t client;

TinyGPSPlus gps;
TinyGPSCustom pdop(gps, "GPGSA", 15); // $GPGSA sentence, 15th element
TinyGPSCustom hdop(gps, "GPGSA", 16); // $GPGSA sentence, 16th element
TinyGPSCustom vdop(gps, "GPGSA", 17); // $GPGSA sentence, 17th element

String FirmwareVer = {
  "2.7"
};
#define URL_fw_Version "https://raw.githubusercontent.com/kaatrusenai/camp_a/main/M5.txt"
#define URL_fw_Bin "https://raw.githubusercontent.com/kaatrusenai/camp_b/main/M5.bin"

void firmwareUpdate();
int FirmwareVersionCheck();

const char* WIFI_SSID = "KDONGLE20";
const char* WIFI_PASSWD = "K@@tru!820";

const char* MQTT_HOST = "3.111.139.83";

const uint32_t MQTT_PORT = 1883;


#define I2C_SDA 32
#define I2C_SCL 33

char delimiter1 = '"';
char delimiter = ',';

char charBuf[5000];
char charBuf1[5000];


String finalOutStr = "";
String l,p;


TwoWire I2CSensors = TwoWire(1);


const int MPU_addr=0x68; // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ,AcXC,AcYC,AcZC,GyXC,GyYC,GyZC,B,A,Label;

static esp_err_t mqtt_event_handler (esp_mqtt_event_handle_t event) {
 if (event->event_id == MQTT_EVENT_CONNECTED) {
 ESP_LOGI ("TEST", "MQTT msgid= %d event: %d. MQTT_EVENT_CONNECTED", event->msg_id, event->event_id);
 } 
 else if (event->event_id == MQTT_EVENT_DISCONNECTED) {
 ESP_LOGI ("TEST", "MQTT event: %d. MQTT_EVENT_DISCONNECTED", event->event_id);
 //esp_mqtt_client_reconnect (event->client); //not needed if autoconnect is enabled
 } else if (event->event_id == MQTT_EVENT_SUBSCRIBED) {
 ESP_LOGI ("TEST", "MQTT msgid= %d event: %d. MQTT_EVENT_SUBSCRIBED", event->msg_id, event->event_id);
 } else if (event->event_id == MQTT_EVENT_UNSUBSCRIBED) {
 ESP_LOGI ("TEST", "MQTT msgid= %d event: %d. MQTT_EVesp_mqtt_client_publish (client, "esp32/test",buffer6,160000,0,false);ENT_UNSUBSCRIBED", event->msg_id, event->event_id);
 } else if (event->event_id == MQTT_EVENT_PUBLISHED) {
 ESP_LOGI ("TEST", "MQTT event: %d. MQTT_EVENT_PUBLISHED", event->event_id);
 } else if (event->event_id == MQTT_EVENT_DATA) {
 ESP_LOGI ("TEST", "MQTT msgid= %d event: %d. MQTT_EVENT_DATA", event->msg_id, event->event_id);
 ESP_LOGI ("TEST", "Topic length %d. Data length %d", event->topic_len, event->data_len);
 ESP_LOGI ("TEST","Incoming data: %.*s %.*s\n", event->topic_len, event->topic, event->data_len, event->data);

 } else if (event->event_id == MQTT_EVENT_BEFORE_CONNECT) {
 ESP_LOGI ("TEST", "MQTT event: %d. MQTT_EVENT_BEFORE_CONNECT", event->event_id);
 }
 return ESP_OK;
}

unsigned long previous;
unsigned long present;
unsigned long startm;
unsigned long previous1=1132125;
unsigned long previous2;
unsigned long delay1;
unsigned long pre2;
unsigned long pre1;

int i=0,j=0,flag=0,k=0,z=0,y=0,q=0;

struct SpiRamAllocator {
 void* allocate(size_t size) {
 return ps_malloc(size);

 }
 void deallocate(void* pointer) {
 free(pointer);
 }
};

void addOutStr(String val)
{
 //concat output string
 //output string is global and needs to be cleaned/initialised before using.
 finalOutStr += String(val);
}
String getOutStr(void)
{
 return finalOutStr;
}

String quoteMyString(String inp)
{
 return "\"" + inp + "\"";
}

char checkValue(unsigned char *thebuf, char leng)
{
 char receiveflag=0;
 int receiveSum=0;

 for(int i=0; i<(leng-2); i++){
 receiveSum=receiveSum+thebuf[i];
 }
 receiveSum=receiveSum + 0x42;

 if(receiveSum == ((thebuf[leng-2]<<8)+thebuf[leng-1])) //check the serial data
 {
 receiveSum = 0;
 receiveflag = 1;
 }
 return receiveflag;
}

int transmitPM01(unsigned char *thebuf)
{
 int PM01Val;
 PM01Val=((thebuf[3]<<8) + thebuf[4]); //count PM1.0 value of the air detector module
 return PM01Val;
}

//transmit PM Value to PC
int transmitPM2_5(unsigned char *thebuf)
{
 int PM2_5Val;
 PM2_5Val=((thebuf[5]<<8) + thebuf[6]);//count PM2.5 value of the air detector module
 return PM2_5Val;
 }

//transmit PM Value to PC
int transmitPM10(unsigned char *thebuf)
{
 int PM10Val;
 PM10Val=((thebuf[7]<<8) + thebuf[8]); //count PM10 value of the air detector module
 return PM10Val;
}

String getPMSenValues()
{
 String localstr = " ";
// PMSerial.begin(9600);
 //PMSerial.setTimeout(1500);
// PMSerial.listen();
 if (Serial2.find(0x42)) {
 Serial2.readBytes(buf, LENG);

 if (buf[0] == 0x4d) {
 if (checkValue(buf, LENG)) {
 PM01Value = transmitPM01(buf); //count PM1.0 value of the air detector module
 PM2_5Value = transmitPM2_5(buf); //count PM2.5 value of the air detector module
 PM10Value = transmitPM10(buf); //count PM10 value of the air detector module
 }
 }
 }


 static unsigned long OledTimer = millis();
 if (millis() - OledTimer >= 1000)
 {
 OledTimer = millis();

 localstr = quoteMyString("DF_PM1") + ":"+ PM01Value + delimiter;
 localstr += quoteMyString("DF_PM25") + ":"+ PM2_5Value + delimiter;
 localstr += quoteMyString("DF_PM10") + ":"+ PM10Value;
 }
 return localstr;
// PMSerial.end(9600);
 
}

String GetTimestampValues() // dateparser for gps
{
  //String localstr = " ";
  String localstr = " ";
  String delimiter1 = " ";
  String delimiter3 = "/";
  String delimiter4 = ":";
   localstr = (gps.date.year() + delimiter3 + gps.date.month() + delimiter3 + gps.date.day() + delimiter1 + gps.time.hour() + delimiter4 + gps.time.minute() + delimiter4 + gps.time.second() );
  return localstr;
}



String getGpsValues()
{
  String localstr = " ";
  String nso = String("N");
  String ewo = String("E");
  String voltage = String("null");
  String provider = String("none");
  while(!timeClient.update()) {
    timeClient.forceUpdate();
  }
  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // We need to extract date and time
  formattedDate = timeClient.getFormattedDate();
  formattedDate.replace("T"," ");
  formattedDate.replace("Z","");



if (gps.altitude.isUpdated() || gps.satellites.isUpdated() ||
    pdop.isUpdated() || hdop.isUpdated() || vdop.isUpdated())
  {
    
  localstr = quoteMyString("P_Id") + ":"+ quoteMyString(String(z)) + delimiter;
  localstr += quoteMyString("name") + ":"+ quoteMyString(DeviceID) + delimiter;
  localstr += quoteMyString("lat") + ":"+ String(gps.location.lat(),6) + delimiter;
  localstr += quoteMyString("NSO") + ":"+quoteMyString(nso) + delimiter;
  localstr += quoteMyString("lon") + ":"+ String( gps.location.lng(),6) + delimiter;
  localstr += quoteMyString("EWO") + ":"+quoteMyString(ewo) + delimiter;
  localstr += quoteMyString("Alt") + ":"+gps.altitude.meters() + delimiter;
  localstr += quoteMyString("sog") + ":"+gps.speed.kmph() + delimiter;
  localstr += quoteMyString("cog") + ":"+gps.course.deg() + delimiter;
  localstr += quoteMyString("hdop") + ":"+ hdop.value() + delimiter;
  localstr += quoteMyString("vdop") + ":"+vdop.value() + delimiter;
  localstr += quoteMyString("pdop") + ":"+pdop.value() + delimiter;
  localstr += quoteMyString("D_Time") + ":"+quoteMyString(String(formattedDate)) + delimiter;
}

 
  while (Serial1.available() > 0)
    gps.encode(Serial1.read());
 return localstr;
}



String getDhtSenValues()
{
  String localstr = " ";

  
  dht22.read2(pinDHT22, &temperature, &humidity, NULL);
  localstr = quoteMyString("DHT_RH") + ":"+ humidity + delimiter;
  localstr += quoteMyString("DHT_TEMP") + ":"+ temperature + delimiter;
  return localstr;
}


String getBrakeData()
{
  String localstr = " ";

  int volt = analogRead(25);// read the input - GPIO_NUM_34
  double voltage = map(volt,0, 4095, 0, 1650) + voltage_offset;
  voltage /= 100; // divide by 100 to get the decimal values 
  localstr = String(voltage) ;
  
  return localstr;
}

String getAccDeacData()
{
  String localstr = " ";

   potVal = analogRead(potPin);
  
  angle = map(potVal,0,4095,0,90);
  
  localstr = String(angle);
  
  return localstr;
}
  

String getlabeldata() {
  // read the state of the pushbutton value
  LabelState1 = digitalRead(LabelPin1);
 // Serial.println(LabelState1);
  LabelState2 = digitalRead(LabelPin2);
//  Serial.println(LabelState2);
  LabelState3 = digitalRead(LabelPin3);
//  Serial.println(LabelState3);
  LabelState4 = digitalRead(LabelPin4);
//  Serial.println(LabelState4);
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH
  if (LabelState1 == HIGH) {
    // turn LED on
    return Label1;
  } 
  else if (LabelState2 == HIGH) {
    // turn LED on
    return Label2;
  } 
  else if (LabelState3 == HIGH) {
    // turn LED on
    return Label3;
  } 
  else if (LabelState4 == HIGH) {
    // turn LED on
    return Label4;
  } 
  else
{
  // do Thing C
  return Label5;
}
delay(100);
}

String getJsonDataValues()
{
 String fullvalue = ("{" + getOutStr() + "}" );
 //String fullvalue = ("{" + getGpsValues() + "}" );
 return fullvalue;
}



void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=Gyro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(Gyro_deadzone+1);

    if (abs(mean_gy)<=Gyro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(Gyro_deadzone+1);

    if (abs(mean_gz)<=Gyro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(Gyro_deadzone+1);

    if (ready==6) break;
  }
}

TaskHandle_t Task2;

  char * buffer5 = (char *) ps_malloc (2040000 * sizeof (char));
    char * buffer6 = (char *) ps_malloc (2040000 * sizeof (char));

char brack1='{';
char brack2='}';
char macBuf[5000];
String M = String("Mac_ID");
String finalStrings;
String D = String("Device_ID");

void setup() {

 mqtt_cfg.host = MQTT_HOST;
 mqtt_cfg.port = MQTT_PORT;
 //mqtt_cfg.username = MQTT_USER;
 //mqtt_cfg.password = MQTT_PASSWD;
 mqtt_cfg.keepalive = 15;
#ifdef SECURE_MQTT
 mqtt_cfg.transport = MQTT_TRANSPORT_OVER_SSL;
#else
 mqtt_cfg.transport = MQTT_TRANSPORT_OVER_TCP;
#endif // SECURE_MQTT
 mqtt_cfg.event_handle = mqtt_event_handler;
 mqtt_cfg.lwt_topic = "M5/acc";
 mqtt_cfg.lwt_msg = "0";
 mqtt_cfg.lwt_msg_len = 1;

   setCpuFrequencyMhz(240);
 
 Serial.begin (115200);
 delay(500);
 Serial.println();

 Serial2.begin(9600, SERIAL_8N1, RXD1, TXD1);
 Serial2.setTimeout(1500);
 Serial1.begin(9600,SERIAL_8N1,RXD2,TXD2);

esp_bt_controller_disable();
  esp_bt_controller_deinit(); //**
  delay(1000);
  WiFi.begin (WIFI_SSID,WIFI_PASSWD);
  while (!WiFi.isConnected ()) {
    Serial.print ('.');
    delay (100);
  }
  Serial.println ();
  esp_wifi_set_ps(WIFI_PS_NONE);
  
client = esp_mqtt_client_init (&mqtt_cfg);
  //esp_mqtt_client_register_event (client, ESP_EVENT_ANY_ID, mqtt_event_handler, client); // not implemented in current Arduino core
  esp_mqtt_client_start (client);
  ESP_LOGI ("TEST", "Client connect. Error = %d %s", err, esp_err_to_name (err));
finalStrings=String(brack1)+delimiter1+D+delimiter1+":"+delimiter1+String(DeviceID)+delimiter1+delimiter+delimiter1+String(M)+delimiter1+":"+delimiter1+String(WiFi.macAddress())+delimiter1+brack2;
 finalStrings.toCharArray(macBuf,finalStrings.length() + 1);
         esp_mqtt_client_publish (client, topic1,macBuf,0,0,false);
 
 FirmwareVersionCheck();

  pinMode(LabelPin1, INPUT);
  pinMode(LabelPin2, INPUT);
  pinMode(LabelPin3, INPUT);
  pinMode(LabelPin4, INPUT);

Wire.begin(32,33,100000);

  // initialize serial communication
 // Serial.begin(115200);

  // initialize device
  accelgyro.initialize();


  

  // start message
  Serial.println("\nMPU6050 Calibration Sketch");
  delay(2000);
  Serial.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
  delay(3000);
  // verify connection
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(1000);
  // reset offsets
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);

    if (state==0){
    Serial.println("\nReading sensors for first time...");
    meansensors();
    state++;
    delay(1000);
  }

  if (state==1) {
    Serial.println("\nCalculating offsets...");
    calibration();
    state++;
    delay(1000);
  }

  if (state==2) {
    meansensors();
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor readings with offsets:\t");
    Serial.print(mean_ax); 
    Serial.print("\t");
    Serial.print(mean_ay); 
    Serial.print("\t");
    Serial.print(mean_az); 
    Serial.print("\t");
    Serial.print(mean_gx); 
    Serial.print("\t");
    Serial.print(mean_gy); 
    Serial.print("\t");
    Serial.println(mean_gz);
    Serial.print("Your offsets:\t");
    Serial.print(ax_offset); 
    Serial.print("\t");
    Serial.print(ay_offset); 
    Serial.print("\t");
    Serial.print(az_offset); 
    Serial.print("\t");
    Serial.print(gx_offset); 
    Serial.print("\t");
    Serial.print(gy_offset); 
    Serial.print("\t");
    Serial.println(gz_offset); 
    Serial.println("\nData is printed as: acelX acelY acelZ GyroX GyroY GyroZ");
    Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
   // Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
   // while (1);
  }

  //Wire.setClock(400000); //experimental I2C speed! 400KHz, default 100KHz
 I2CSensors.begin(I2C_SDA, I2C_SCL, 400000);
 I2CSensors.beginTransmission(MPU_addr);
 I2CSensors.write(0x6B); // PWR_MGMT_1 register
 I2CSensors.write(0); // set to zero (wakes up the MPU-6050)
 I2CSensors.endTransmission(true);
 Serial.println("Wrote to IMU");

    
xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    50000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 1 */
    delay(500); 

timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(0);


}

unsigned long a,b,sm,em;
unsigned long v,u;


void Task2code( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
  
  for(;;){ 
          if(q==0)
    {
      //v=millis();
      finalOutStr = "";
        addOutStr(String(getGpsValues()));
        addOutStr(String(getDhtSenValues()));
        addOutStr(String(getPMSenValues()));
        //Serial.println(getJsonDataValues());
        l= "";
        l=getJsonDataValues();
         //Serial.println(millis()-v);
      q=2;
    }

    else if(q==1)
    {  
        //v=millis();
        finalOutStr = "";
        addOutStr(String(getGpsValues()));
        addOutStr(String(getDhtSenValues()));
        addOutStr(String(getPMSenValues()));
        //Serial.println(getJsonDataValues());
        p = "";
        p=getJsonDataValues();
        //Serial.println(millis()-v);
        q=3;
    }
    if(k==0)
    {
      l.toCharArray(charBuf,l.length() + 1);
       v=millis();
        esp_mqtt_client_publish (client, topicsen,charBuf,0,0,false);
        esp_mqtt_client_publish (client, topicacc,buffer6,0,0,false);
        Serial.println(millis()-v);
      //Serial.println(z);
      k=2;
    }
    else if(k==1)
    {
      p.toCharArray(charBuf1,p.length() + 1);
       v=millis();
      esp_mqtt_client_publish (client, topicsen,charBuf1,0,0,false);
      esp_mqtt_client_publish (client, topicacc,buffer6,0,0,false);
      Serial.println(millis()-v);

      //Serial.println(z);
      k=3;
    }
    if(millis()-sm>86400000)
  {
    FirmwareVersionCheck();
    sm=millis();
  }
    delay(500);
  }
  }
void loop() {
      z++;
  a=millis();
    buffer5=collect();
    Serial.println(millis()-a);
    buffer6=buffer5;
    if(q==2)
    {
      q=1;
      k=0;
    }
    if(q==3)
    {
        q=0;
        k=1;
    }
    
}//doc["CogAcc"] = 
char * collect()
{

  using SpiRamJsonDocument = BasicJsonDocument<SpiRamAllocator>;
  SpiRamJsonDocument doc(2040000);
  doc["P_Id"] = z;
  doc["name"]=DeviceID;
  doc["LatAcc"] = String(gps.location.lat(),6);
  doc["LonAcc"] = String( gps.location.lng(),6);
 // doc["SogAcc"] = String(gps.speed.kmph());
 // doc["CogAcc"] = String(gps.course.deg());
  while (Serial1.available() > 0)
    gps.encode(Serial1.read());
  //JsonArray latitudeAcc = doc.createNestedArray("LatAcc");
  //JsonArray longitudeAcc = doc.createNestedArray("LonAcc");
  
  JsonArray Label = doc.createNestedArray("Label");
  JsonArray SogAcc = doc.createNestedArray("SogAcc");
  JsonArray CogAcc = doc.createNestedArray("CogAcc");
  JsonArray acx = doc.createNestedArray("AcX");
  JsonArray acy = doc.createNestedArray("AcY");
  JsonArray acz = doc.createNestedArray("AcZ");
  JsonArray gcx = doc.createNestedArray("GcX");
  JsonArray gcy = doc.createNestedArray("GcY");
  JsonArray gcz = doc.createNestedArray("GcZ");
  JsonArray temp = doc.createNestedArray("Tmp");
  JsonArray T = doc.createNestedArray("Time");
  JsonArray brake = doc.createNestedArray("B");
  JsonArray accdeac = doc.createNestedArray("A");  
i =0;
startm=micros();
//Serial.println("Start loop");
  while(i<1000){
    previous=micros();
 I2CSensors.beginTransmission(MPU_addr);
 I2CSensors.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
 I2CSensors.endTransmission(false);
 I2CSensors.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
 AcX=I2CSensors.read()<<8|I2CSensors.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
 AcY=I2CSensors.read()<<8|I2CSensors.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
 AcZ=I2CSensors.read()<<8|I2CSensors.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
 Tmp=I2CSensors.read()<<8|I2CSensors.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
 GyX=I2CSensors.read()<<8|I2CSensors.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
 GyY=I2CSensors.read()<<8|I2CSensors.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
 GyZ=I2CSensors.read()<<8|I2CSensors.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  
 I2CSensors.endTransmission(true);
 
  int volt = analogRead(34);// read the input - GPIO_NUM_34
  double B = map(volt,0, 4095, 0, 1650) + voltage_offset;
  B /= 100; // divide by 100 to get the decimal values 
  //float  B = ( volt * 3.3 ) / (4095);
  
  potVal = analogRead(potPin);
  
  A = map(potVal,0,4095,0,90);
  
 //latitudeAcc.add(String(gps.location.lat(),6));
// longitudeAcc.add(String(gps.location.lng(),6));
Label.add(getlabeldata());
SogAcc.add(gps.speed.kmph());
CogAcc.add(gps.course.deg());
 acx.add(AcX);
 acy.add(AcY);
 acz.add(AcZ);
 gcx.add(GyX);
 gcy.add(GyY);
 gcz.add(GyZ);
 temp.add(Tmp/340.00+36.53); 
 brake.add(B);
 accdeac.add(A);
 i++;
 delayMicroseconds(9700);
 T.add(micros()-previous);
 doc["End_Time"]=micros()-startm; 
}
      
//Serial.println(micros()-startm);
char * buffer1 = (char *) ps_malloc (2040000 * sizeof (char));
free(buffer1);
serializeJson(doc,buffer1,2040000);

return buffer1;
}
void firmwareUpdate(void) {
  WiFiClientSecure client;
  client.setCACert(rootCACertificate);
  t_httpUpdate_return ret = httpUpdate.update(client, URL_fw_Bin);

  switch (ret) {
  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
    break;

  case HTTP_UPDATE_NO_UPDATES:
    Serial.println("HTTP_UPDATE_NO_UPDATES");
    break;

  case HTTP_UPDATE_OK:
    Serial.println("HTTP_UPDATE_OK");
    break;
  }
}
int FirmwareVersionCheck(void) {
  String payload;
  int httpCode;
  String fwurl = "";
  fwurl += URL_fw_Version;
  fwurl += "?";
  fwurl += String(rand());
  Serial.println(fwurl);
  WiFiClientSecure * client = new WiFiClientSecure;

  if (client) 
  {
    client -> setCACert(rootCACertificate);

    // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is 
    HTTPClient https;

    if (https.begin( * client, fwurl)) 
    { // HTTPS      
      Serial.print("[HTTPS] GET...\n");
      // start connection and send HTTP header
      delay(100);
      httpCode = https.GET();
      delay(100);
      if (httpCode == HTTP_CODE_OK) // if version received
      {
        payload = https.getString(); // save received version
      } else {
        Serial.print("error in downloading version file:");
        Serial.println(httpCode);
      }
      https.end();
    }
    delete client;
  }
      
  if (httpCode == HTTP_CODE_OK) // if version received
  {
    payload.trim();
    if (payload.equals(FirmwareVer)) {
      Serial.printf("\nDevice already on latest firmware version:%s\n", FirmwareVer);
      return 0;
    } 
    else 
    {
      Serial.println(payload);
      Serial.println("New firmware detected");
      firmwareUpdate();
      return 1;
    }
  } 
  return 0;  
}