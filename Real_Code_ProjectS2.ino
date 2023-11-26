//library
#include <ESP32Servo.h> 
#include <Wire.h>
#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
  #include <ESPAsyncWebServer.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <AsyncTCP.h>
  #include <ESPAsyncWebServer.h>
#endif
#include <ESPDash.h>

//SSID
const char* ssid = "Beam Balancing Project"; // SSID
const char* password = "qwertyuiop"; // Password

//Pin Sensor
const int trigPinSensor = 26 ;
const int echoPinSensor = 25 ;

//motor
Servo myservo;

int Read = 0;
int distance = 0.0;
float elapsedTime, times, timePrev;        //Variabel kontrol waktu
float distance_previous_error, distance_error; //Variabel eror jarak
int period = 50;  //Refresh rate period of the loop is 50ms
float durationSensor, distanceSensor;

/*PID*/
float kp = 2.5;//=3.7;
float ki = 0;//=0.01; 
float kd = 1;//=3000;
int distance_setpoint = 15;//16;           //Should be the distance from sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d, PID_total;

AsyncWebServer server(80);

ESPDash dashboard(&server); 
// Slider Chart data
Card slider1(&dashboard, SLIDER_CARD, "Proportional", "", 0, 100);
Card slider2(&dashboard, SLIDER_CARD, "Integral", "", 0,100);
Card slider3(&dashboard, SLIDER_CARD, "Deritative", "", 0, 100);
Card slider4(&dashboard, SLIDER_CARD, "Set-Point", "", 0, 32);

// Bar Chart Data
int XAxis[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int YAxis[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

Chart bar(&dashboard, BAR_CHART, "Response PID");

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  //default 192.168.4.1
  // WiFi.softAPConfig(IPAddress(192, 168, 1, 1), IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, password);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
  server.begin();
  
  //Pin Motor   
  myservo.attach(13);  
  myservo.write(125); //Put the servco at angle 125, so the balance is in the middle
  pinMode(trigPinSensor, OUTPUT);
  pinMode(echoPinSensor, INPUT);
  
  bar.updateX(XAxis, 32);
  times = millis();
  
}
void loop() { 
    for(int i=0; i < distanceSensor; i++){
    YAxis[i] = distanceSensor;
    }
    for(int j=0; j <= 32; j++){
    XAxis[j] = j+1  ;
    
    }
    //Serial.println(c);
    bar.updateY(YAxis, 32);

    slider1.attachCallback([&](float value){
    kp = value / 10;
    //Serial.println(kp);
    slider1.update(value);
    dashboard.sendUpdates();
  });
    slider2.attachCallback([&](float value){
    ki = value / 1000;
    //Serial.println(ki);
    slider2.update(value);
    dashboard.sendUpdates();
  });
    slider3.attachCallback([&](int value){
    kd = value / 10;
    slider3.update(value);
    dashboard.sendUpdates();
  });
   
  slider4.attachCallback([&](int value){
    distance_setpoint = value;
    slider4.update(value);
    dashboard.sendUpdates();
  });
  //Send Updates to our Dashboard (realtime)
  dashboard.sendUpdates();
  delay(100);
  
  if (millis() > times+period)
  {
    times = millis();    
    distance = getdist();   
    distance_error = distance_setpoint - distance;   
    PID_p = kp * distance_error;
    float dist_diference = distance_error - distance_previous_error;     
    PID_d = kd*((distance_error - distance_previous_error)/period);
      
    if(-3 < distance_error && distance_error < 3)
    {
      PID_i = PID_i + (ki * distance_error);
    }
    else
    {
      PID_i = 0;
    }
  
    PID_total = PID_p + PID_i + PID_d;  
    PID_total = map(PID_total, -150, 150, 0, 150);
  
    if(PID_total < 20){PID_total = 20;}
    if(PID_total > 150) {PID_total = 150;} 
    PID_total = PID_total+30;
    Serial.println(PID_total);
    myservo.write(PID_total);  
    distance_previous_error = distance_error;
  }
}
float getdist()
{
  digitalWrite(trigPinSensor, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinSensor, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinSensor, LOW);

  durationSensor = pulseIn(echoPinSensor, HIGH);
  distanceSensor = (durationSensor*.0343)/2;
  //Serial.print("DistanceSensor: ");
  if(distanceSensor >34){ distanceSensor = 34;}  
  return(distanceSensor);
} 
  
