/* 
 * Project Smart House Plant Midterm
 * Author: Andres S Cordova
 * Date: 18-MARCH-2024
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_BME280.h"
#include "Grove_Air_quality_Sensor.h"
#include "IoTTimer.h"
#include "math.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "credentials.h"




// Global State 
TCPClient TheClient; 

//Declare Variables
float lastTime;
const int soilSensor = A1;                        //Soil Sensor
int moister;
int moisterPumpRelay;

const int waterPump = D18;                        // water pump 
bool buttonOnOff;

float tempF;                                      //BME
float tempC;
float pressPA;
int humidRH;
                             
bool status;                                      //OLED 
const int OLED_RESET = -1;

AirQualitySensor airqualitysensor(A5);             // Airqulity Sensor
int current_quality =-1;

const int dustSensor = D16;                       // Dust Sensor
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000;              //sampe 30s ;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;



// Declare Function
SYSTEM_MODE(AUTOMATIC);      // connection to the Particle Cloud

// MQTT Server and Wifi Login
IoTTimer plantTimer;
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

//Feeds 
Adafruit_MQTT_Subscribe Button = Adafruit_MQTT_Subscribe(&mqtt,AIO_USERNAME "/feeds/water-pump");

Adafruit_MQTT_Publish watersensor = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME "/feeds/water-sensor");
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME "/feeds/Temperature");
Adafruit_MQTT_Publish hummidity = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME "/feeds/humidity");
Adafruit_MQTT_Publish airtSensor = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME "/feeds/air-quality");
Adafruit_MQTT_Publish polutionsensor = Adafruit_MQTT_Publish(&mqtt,AIO_USERNAME "/feeds/dust-sensor");

//Functions
void MQTT_connect();
bool MQTT_ping(); 

Adafruit_SSD1306 display(OLED_RESET);
Adafruit_BME280 bme;
IoTTimer pixelTimer;

// setup() runs once, when the device is first turned on
void setup() {
Serial.begin(9600);
  waitFor(Serial.isConnected,10000);

  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }

  mqtt.subscribe(&Button);                          //MQTT subscription

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);        //OLED
  display.setTextSize(1);
  Particle.connect;  
  Time.zone (-7);                                   // MST = -7
  Particle.syncTime ();                             // Sync with Particle Cloud
  display.setTextColor(WHITE);
  display.setRotation(0);
  display.display();

  display.clearDisplay();
  display.display();
 


  status = bme.begin(0x76);                             // BME
  if (status == false){
  Serial.printf ("BME280 failed to start");
  display.printf ("BME280 failed to start");
  display.display();
  }

  pinMode(soilSensor,INPUT);                                // soil sensor 

  airqualitysensor.init();                          // Air Sensor

  pinMode(waterPump,OUTPUT);                              // water pump 

  Serial.begin(9600);                 
  pinMode(moister, OUTPUT);
  pinMode(soilSensor, INPUT);
  Serial.printf ("Reading From the Sensor");
  delay(2000);

  pinMode(dustSensor,INPUT);                               // Dust Sensor
  plantTimer.startTimer(120000);
  starttime = millis();                             //get the current time;

}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  MQTT_connect();
  MQTT_ping();
  
  if((millis()-lastTime > 120000)) {
    moister = analogRead(soilSensor);

    if(mqtt.Update()) {
      hummidity.publish(humidRH);
      temperature.publish(tempF);
      watersensor.publish(soilSensor);
      airtSensor.publish(current_quality);
      polutionsensor.publish(dustSensor);
            
      } 
    lastTime = millis();
  
  }

  display.setCursor(0,0);

  moister = analogRead (soilSensor);              //Moister Sensor 
  Serial.printf("%i\n",moister);
  display.printf("Moi:%i\n",moister);
  
  
  tempF = (bme.readTemperature()*(9.0/5.0)+32);   //BME deg in Celsius 
  pressPA = (bme.readPressure ()*(.000295));   //pascals 
  humidRH = bme.readHumidity ();  //%RH
  
  Serial.printf("%0.01f\n",tempF);              //Monitor display 
  display.printf("Temp:%0.1f\n",tempF);
  display.printf("Pres:%0.1f\n",pressPA); 
  display.printf("Humid:%0i\n",humidRH);        // OLED display
  display.printf("Dust:%0i\n",dustSensor);
  display.display();
  display.clearDisplay();


  current_quality=airqualitysensor.slope();          // Air Sensor 
    if (current_quality >= 0)                       // if a valid data returned.
    {
      if (current_quality==0)
        Serial.println("High pollution! Force signal active");
      else if (current_quality==1)
        Serial.println("High pollution!");
      else if (current_quality==2)
         Serial.println("Low pollution!");
      else if (current_quality ==3)
            Serial.println("Fresh air");
    }


    duration = pulseIn(dustSensor, LOW);                     // Dust Sensor
    lowpulseoccupancy = lowpulseoccupancy+duration;

    if ((millis()-starttime) > sampletime_ms)               //if the sampel time == 30s
    {
        ratio = lowpulseoccupancy/(sampletime_ms*10.0);     // Integer percentage 0=>100
        concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62;   // using spec sheet curve
        Serial.print(lowpulseoccupancy);
        Serial.print(",");
        Serial.print(ratio);
        Serial.print(",");
        Serial.println(concentration);
        lowpulseoccupancy = 0;
        starttime = millis();
    }
 

  if(plantTimer.isTimerReady()){
    moisterPumpRelay= analogRead(soilSensor);
    Serial.printf ("Moisture:%i\n",moisterPumpRelay);

    if(moisterPumpRelay>2600){
      digitalWrite(waterPump, HIGH);
      delay(50);
      digitalWrite(waterPump, LOW);       
    }
       plantTimer.startTimer(1800000);                            // 30 minute timer

  }

  if (buttonOnOff == 1);{
    digitalWrite(waterPump, HIGH);
    }
    plantTimer.startTimer(1000);





    //digitalWrite(D18,HIGH);                                    // Water pump 
    //delay(30000);
    //digitalWrite(D18,LOW);
    //delay(30000);


}


void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
    Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) {                         // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);                                              // wait 5 seconds and try again
  }
    Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}
