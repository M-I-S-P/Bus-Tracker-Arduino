// == BUS TRACKER ==
// This code uses the gps tracker neo-6m to get the latitude
// & longitude of the bus and using module SIM900A through
// celluar mobile network uploads the location onto the DB.

#include "ArduinoJson.h"
#include "TinyGPS++.h"
#include <Wire.h>

#define client Serial1
#define gpsClient Serial3

// Define PI
#define PI 3.1415926535897932384626433832795

bool changeRoute = false;
String newRouteId = "";
String newRoute = "";

// ------------------------------------------------------------------

TinyGPSPlus gps;

double currLat = -1, currLon = -1;

// ------------------------------------------------------------------



void setup() {
  // put your setup code here, to run once:
  gpsClient.begin(9600);
  client.begin(9600);
  Serial.begin(9600);
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event
  
  // Establish GPRS connection
  connectGPRS();

}

void loop() {

  if(changeRoute){
    String routeNames[2] = {"Route 1", "Route 2"};
    String routeIds[2] = {"qFJLN3NN2ebgEvjoDI1Y", "hF7nxVqJy7v8ADmBiIV0"};

    int i = 0;
    for(i; i<2; i++){
      if(newRoute == routeNames[i]){
        break;
      }
    }
    newRouteId = routeIds[i];
    String body = "{\"route\": \"" + newRouteId + "\"}";
    Serial.print("starting change route request");
    connectHTTP(body, "route");
    changeRoute = false;
    newRouteId = "";
  }
  
  smartDelay(1000);
  setGPSInfo();


}

static void smartDelay(unsigned long ms){
  unsigned long start = millis();
  do
  {
    // If data has come in from the GPS module
    while (gpsClient.available())
      gps.encode(gpsClient.read());
    // gps.encode(char) continues to "load" the gps object with new
    // data coming in from the Neo-6m GPS module. As full NMEA strings begin to come in
    // the tinyGPS library will be able to start parsing them
  } while (millis() - start < ms);
}

double getDistance(double tempLat, double tempLon, double currLat, double currLon){
    
   // Convert the coordinates into Radians
   double tempLatRad = (tempLat*PI)/180;
   double tempLonRad = (tempLon*PI)/180;
   double currLatRad = (currLat*PI)/180;
   double currLonRad = (currLon*PI)/180;


    double dlong = currLonRad - tempLonRad; 
    double dlat = currLatRad - tempLatRad;

    double ans = pow(sin(dlat / 2), 2) + cos(tempLatRad) * cos(currLatRad) * pow(sin(dlong / 2), 2);
    ans = 2 * asin(sqrt(ans)); 
    
    // Radius of Earth in Kilometers, R = 6371 
    // Use R = 3956 for miles 
    double R = 6371; 
      
    // Calculate the result 
    ans = ans * R;
  
    return ans*1000;
}


//// Connect to GPRS Code
void connectGPRS(){ 
  
  client.println("AT+SAPBR=3,1,\"Contype\",\"GPRS\"");
  ShowSerialData();

  client.println("AT+SAPBR=3,1,\"APN\",\"www\"");//APN
  delay(1000);
  ShowSerialData();

  client.println("AT+SAPBR=1,1");
  delay(1000);
  ShowSerialData();

  client.println("AT+SAPBR=2,1");
  delay(1000);
  ShowSerialData();
  
}

// Code for POST REQUEST
void connectHTTP(String sendtoserver, String whatToUpdate){
  connectGPRS();
  String url = "";

  if(whatToUpdate == "gps") {
    url = "\"http://bus-tracker-proxy-server.herokuapp.com/buses/busTrackerBT-1/location\"";
  }
  else if(whatToUpdate == "route") url = "\"http://bus-tracker-proxy-server.herokuapp.com/buses/busTrackerBT-1/route\"";

  
  client.println("AT+HTTPINIT");
  delay(1000);
  ShowSerialData();

  client.println("AT+HTTPPARA=\"CID\",1");
  delay(1000);
  ShowSerialData();

  client.println("AT+HTTPPARA=\"URL\"," + url); //Public server address
  delay(1000);
  ShowSerialData();

  client.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  delay(1000);
  ShowSerialData();

  
  client.println("AT+HTTPDATA=" + String(sendtoserver.length()) + ",100000");
  delay(1000);
  ShowSerialData();

  client.println(sendtoserver);
  delay(1000);
  ShowSerialData();

  client.println("AT+HTTPACTION=1");
  delay(1000);
  ShowSerialData();

  client.println("AT+HTTPREAD");
  delay(1000);
  ShowSerialData();

  // Check if to remove this
  client.println("AT+HTTPTERM");
  delay(1000);
  ShowSerialData();

}


void setGPSInfo(){
  double tempLat = gps.location.lat();
  double tempLon = gps.location.lng();
  if(tempLat != 0 && tempLon != 0){

    StaticJsonDocument<100> root;
    root["location"] = String(String(tempLat, 6) + ", " + String(tempLon, 6));
    String coordsToServer;
    serializeJsonPretty(root, coordsToServer);
      
    if(currLat != -1 && currLon != -1){
      double distance = getDistance(tempLat, tempLon, currLat, currLon); 
  
      // Check if distance covered is greater than 10 meters
      if(distance >= 10){
        
        // Update currLon & currLat with new values
        currLon = tempLon;
        currLat = tempLat;
        
        // Function to post coords to DB
        connectHTTP(coordsToServer, "gps");
  
      }
    }
  
    else{
      currLat = tempLat;
      currLon = tempLon;
      connectHTTP(coordsToServer, "gps");
    }
  }
}

void receiveEvent(int howMany){
  String received = "";
  while(Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    received += c;
  }

  changeRoute = true;
  newRoute = received;
  
  Serial.println("end isr");
}

// For debugging of AT commands
void ShowSerialData(){
  while(client.available()){
    Serial2.write(client.read());
  }
}
