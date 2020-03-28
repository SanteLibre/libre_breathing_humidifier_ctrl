//V3 - Inclut une thermistance de 10K dans l'eau et un arrêt si temp_max_eau est atteinte

//Libraries
#include <DHT.h>;

//Constants
#define DHTPIN 7     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
const int ledPin =  12;// the number of the LED pin
const long interval = 1000;           // interval at which to blink (milliseconds)

//Variables
int chk;
float hum;  //Stores humidity value
float temp; //Stores temperature value
float temp_set_air = 37;
float temp_airwaylow = 35.5; //Temp for indicator warning
float time_airwaylow = 25000; //Temp for indicator warning
float temp_airwaymax = 43;
float temp_max_eau = 90;
float toneState = 1000;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
int ledState = LOW;             // ledState used to set the LED
unsigned long previousMillis = 0;        // will store last time LED was updated

//PID constants
int PID_value = 0;
int kp = 24;   int ki = 0.3;   int kd = 1;
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;

//Initializing PWM Pin
int beep_pin = 13;
int thermistor_pin = 0;
int Vo;
float R1 = 10000;
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

void setup() {
  //DHT22
  Serial.begin(9600);
  dht.begin();
  Time = millis(); 
  
  //Declaring PWM pin as output
  pinMode(beep_pin, OUTPUT);
  pinMode(ledPin, OUTPUT);
}

void loop() {

  //Read data and store it to variables hum and temp
  hum = dht.readHumidity();
  temp= dht.readTemperature();

  //Print temp and humidity values to serial monitor
  Serial.print("Air Humidity: ");
  Serial.print(hum);
  Serial.print(" %, Air Temp: ");
  Serial.print(temp);
  Serial.println(" Celsius");

  ///Alarm///


  ///Alarmes et Indicateurs///
  
  ////Over airway indicator////
  if(temp > temp_airwaymax)
  {
   PID_value = 0; //Stop tout chauffage au niveau de la bouilloire.
   tone(beep_pin,toneState);
   ledState = HIGH;
    }

    ////Under airway temp////
  if(temp < temp_airwaymax)
  {
   PID_value = 0; //Stop tout chauffage au niveau de la bouilloire.
   noTone(beep_pin);
   ledState = LOW;
    }

  
  ////////////////////////////////////////////////////////////////////////////////////
   
   digitalWrite(ledPin, ledState);

  //Error between the setpoint and the real value
  PID_error = temp_set_air - temp;
  
  //Calculate the P value
  PID_p = kp * PID_error;
  
  //Calculate the I value in a range on +-3
  if(-3 < PID_error <3)
  {
    PID_i = PID_i + (ki * PID_error);
  }
  
  //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 
  //Now we can calculate the D calue
  PID_d = kd*((PID_error - previous_error)/elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;

  //We define PWM range between 0 and 100 out of 255
  if(PID_value < 0)
  {    PID_value = 0;    }
  if(PID_value > 100)  
  {    PID_value = 100;  }


  previous_error = PID_error;     //Remember to store the previous error for next loop.
  
  //Serial Print PWM Value
  Serial.print("PWM Value :");
  Serial.println(PID_value);

}
