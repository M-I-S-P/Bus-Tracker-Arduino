// == BUS TRACKER ==
// This code uses the gps tracker neo-6m to get the latitude
// & longitude of the bus and using module SIM900A through
// celluar mobile network uploads the location onto the DB.

#include "ArduinoJson.h"
#include "TinyGPS++.h"
#include <Wire.h>
#include<SoftwareSerial.h>

#define client Serial1
#define gpsClient Serial2

// Define PI
#define PI 3.1415926535897932384626433832795

// Define unique bus tracker ID
#define UNIQUE_TRACKER_ID "busTrackerBT-1"

int gpsPowerPin = 2;

bool changeRoute = false;
String newRouteId = "";
String newRoute = "";

// ------------------------------------------------------------------

TinyGPSPlus gps;

//String routeIds[2] = {"qFJLN3NN2ebgEvjoDI1Y", "hF7nxVqJy7v8ADmBiIV0"};
//StaticJsonDocument<100> allRoutes;


// Object of JSONBuffer used in sending
// JSON data to server
//StaticJsonBuffer<200> jsonBuffer;
//char jsonChar[100];

double currLat = NULL, currLon = NULL;

// ------------------------------------------------------------------



void setup() {
  // put your setup code here, to run once:
  gpsClient.begin(9600);
  client.begin(9600);
  Serial.begin(9600);
  
  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(receiveEvent); // register event

//  for(int i = 0; i<2; i++){
//    allRoutes["Route " + String(i+1)] = routeIds[i];
//  }

  // Power for GPS
  pinMode(gpsPowerPin, OUTPUT);
  digitalWrite(gpsPowerPin, HIGH);
  
  // Establish GPRS connection
  connectGPRS();

}

void loop() {

  smartDelay(1000);
//  Serial.print("Lat: ");
//  Serial.println(gps.location.lat(), 6);
//  Serial.println(gps.satellites.value(), 6);
  setGPSInfo();

  if(changeRoute){
    String routeNames[2] = {"Route 1", "Route 2"};
    String routeIds[2] = {"qFJLN3NN2ebgEvjoDI1Y", "hF7nxVqJy7v8ADmBiIV0"};
    Serial.print("inside change route and new route is: ");
    Serial.println(newRoute);
    int i = 0;
    for(i; i<2; i++){
      if(newRoute == routeNames[i]){
        break;
      }
    }
    newRouteId = routeIds[i];
  Serial.print("new routeID is: ");
  Serial.println(newRouteId);
//    Serial.println("CHANGED ROUTE IF");
//    String routeIdNew = allRoutes[newRoute];
    String body = "{\"route\": \"" + newRouteId + "\"}";
//    String body = "{ \"route\": \"hF7nxVqJy7v8ADmBiIV0\"}";
    connectHTTP(body, "route");
    changeRoute = false;
    newRouteId = "";
  }
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

//   Serial.print("SPEED: ");
//   Serial.println(gps.speed.mph());

//   double distanceTravelled = 1.609344*3963*(acos(
//     (sin(tempLatRad) * sin(currLatRad)) + 
//     (cos(tempLatRad) * cos(currLatRad) * cos(tempLonRad-currLonRad))
//    ));

    double dlong = currLonRad - tempLonRad; 
    double dlat = currLatRad - tempLatRad;

    double ans = pow(sin(dlat / 2), 2) + cos(tempLatRad) * cos(currLatRad) * pow(sin(dlong / 2), 2);
    ans = 2 * asin(sqrt(ans)); 
    
    // Radius of Earth in Kilometers, R = 6371 
    // Use R = 3956 for miles 
    double R = 6371; 
      
    // Calculate the result 
    ans = ans * R;

    // Debugging
//    Serial.print("Distance in m: ");
//    Serial.println(ans*1000, 10);
  
    return ans*1000;
}


//// Connect to GPRS Code
void connectGPRS(){ 
  
//  Serial.println("Connecting to gprs");
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
//  Serial.println("end gprs");
  
}

// Code for POST REQUEST
void connectHTTP(String sendtoserver, String whatToUpdate){

//  Serial.println("Start post");
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
//  delay(1000);
  ShowSerialData();

//  Serial.println("End post");
}


//void connectHTTP(String sendtoserver){
//  client.println("AT+HTTPINIT");
//  delay(1000);
//  ShowSerialData();
//
//  client.println("AT+HTTPPARA=\"CID\",1");
//  delay(1000);
//  ShowSerialData();
//
//  client.println("AT+HTTPPARA=\"URL\",\"http://bus-tracker-proxy-server.herokuapp.com/updateLocation\""); //Public server address
//  delay(1000);
//  ShowSerialData();
//
//  client.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
//  delay(1000);
//  ShowSerialData();
//
//
//  client.println("AT+HTTPDATA=" + String(sendtoserver.length()) + ",100000");
//  delay(1000);
//  ShowSerialData();
//
//  client.println(sendtoserver);
//  delay(1000);
//  ShowSerialData();
//
//  client.println("AT+HTTPACTION=1");
//  delay(1000);
//  ShowSerialData();
//
//  client.println("AT+HTTPREAD");
//  delay(1000);
//  ShowSerialData();
//
//  client.println("AT+HTTPTERM");
//  delay(1000);
//  ShowSerialData();
//}

void setGPSInfo(){
//  Serial.println("Reached GPS");
  double tempLat = gps.location.lat();
  double tempLon = gps.location.lng();
//  Serial.print("This is TempLAT ");
//  Serial.println(tempLat, 6);
//  Serial.print("This is TempLON ");
//  Serial.println(tempLon, 6);
  if(tempLat != 0 && tempLon != 0){

//    Serial.println("Inside temp if");
    StaticJsonDocument<100> root;
    root["location"] = String(String(tempLat, 6) + ", " + String(tempLon, 6));
    String coordsToServer;
    serializeJsonPretty(root, coordsToServer);
//    Serial.print("Coordstoserver: ");
//    Serial.println(coordsToServer);
//
//    Serial.println("below coordstoserver");
      
    if(currLat != NULL && currLon != NULL){
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
//    Serial.print(c);         // print the character
  }
//  int x = Wire.read();    // receive byte as an integer
  Serial2.println(received);         // print the integer
  changeRoute = true;
  
  newRoute = received;
  Serial.println(newRoute);

  
  Serial.println("Just finished uploading the code, my lord.");
}

// For debugging of AT commands
void ShowSerialData(){
  while(client.available()){
    Serial2.write(client.read());
  }
}
