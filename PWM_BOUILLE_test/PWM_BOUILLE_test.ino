//V4 - Inclut une thermistance de 10K dans l'eau et un arrêt si temp_max_eau est atteinte

//Libraries
#include <DHT.h>;

//Constants
#define DHTPIN 7     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

//Variables
int chk;
float hum;  //Stores humidity value
float temp; //Stores temperature value
float temp_set_air = 37;
float temp_max_eau = 90;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;

//V4
bool boolAlarmStatus=false;
bool boolWarningStatus=false;
float tAlarm;
float dtAlarm=0;
float warnLevel = 35.5;
#define warnLedPin 13

//PID constants
int PID_value = 0;
int kp = 24;   int ki = 0.3;   int kd = 1;
int PID_p = 0;    int PID_i = 0;    int PID_d = 0;

//Initializing PWM Pin
int beep_pin = 13;
int led_pin = 6;
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
  pinMode(led_pin, OUTPUT);
  pinMode(beep_pin, OUTPUT);
  pinMode(warnLedPin, OUTPUT); //V4
  
}

void loop() {

  //Read data and store it to variables hum and temp
  hum = dht.readHumidity();
  temp= dht.readTemperature();

  Vo = analogRead(thermistor_pin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;

  //Print temp and humidity values to serial monitor
  
  Serial.print("Water Temp: "); 
  Serial.print(T);
  Serial.println(" Celsius"); 

  Serial.print("Air Humidity: ");
  Serial.print(hum);
  Serial.print(" %, Air Temp: ");
  Serial.print(temp);
  Serial.println(" Celsius");

  ///Beep Beep 
  
  //V3
  if(temp > temp_set_air)
  {
      tone(beep_pin,1000,500);
    }
  
  //V4
  chkAlarm(); //Call for function chkAlarm
    
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
  
  //Check température eau
    if(T > temp_max_eau)
  {    PID_value = 0;    }
  
  analogWrite(led_pin,PID_value);

  previous_error = PID_error;     //Remember to store the previous error for next loop.

  //Delay 0.5 sec.
  delay(300); 
  
  //Serial Print PWM Value
  Serial.print("PWM Value :");
  Serial.println(PID_value);

}

//V4
void chkAlarm() {
  if(temp < warnLevel){
     
      if(boolWarningStatus==false){
          tAlarm=Time; //Warning start time
      }
      dtAlarm=Time-tAlarm; //Update time since alarm start
      
      if(dtAlarm>25000 && boolWarningStatus==false){
          boolWarningStatus=true;
      }
      
      //Check if alarm is on or off
      if(boolAlarmStatus==false){
        if(temp<25.43*pow(dtAlarm/1000,0.0777)){  //Set alam
          boolAlarmStatus=true;
        }
      }
      else{
        if(temp>28.03*pow(dtAlarm/1000,0.05959)){ //Mute alarm
            boolAlarmStatus=false;
        } 
      }
  }
  else if (boolWarningStatus ==true|| boolAlarmStatus==true) //If above warning, disable alarm/warning
    {
      boolWarningStatus=false;
      boolAlarmStatus=false;
  }  

setAlarm();
setWarning();

}

void setAlarm() {
  if(boolAlarmStatus==false){
    noTone(beep_pin);
  }
  else {
    tone(beep_pin,1000,500);
  }
  
}

void setWarning() {
    if(boolWarningStatus==false){
    digitalWrite(warnLedPin, LOW);
  }
  else {
    digitalWrite(warnLedPin, HIGH);
  }
}
