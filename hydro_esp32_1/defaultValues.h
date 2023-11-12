//Default settings

#define sensorReadDelay 60000
#define ecInterval 1000*60*1 //600000
#define ecAdjustInterval 60000 //300000
#define ECPumpOnFactor 5000

#define phInterval 1000*60*1 //600000 
#define phAdjustInterval 60000 //300000 
#define pHPumpOn 10000
#define pHTolerance 0.2 //pH will be kept within +- this value

#define phChemCounterLimit 20
#define nutrientCounterLimit 20

#define updateDBTime 900000 //15 minutes

#define mainPumpTime 60000 //run time
#define mainPumpWait 30000  //time between runs

#define wifiConnect 20000 //try to connect to wifi for 20 seconds

#define timeCheck 60000 //check the time once per minute

bool displayStatus[] = {0,0,0,0,0,0,0,0,0,0,0,0};
 
String statusMessages[] = {"WIFI NOT CONNECTED",
                          "Connecting WiFi",
                          "WiFi Connected",   
                          "Updated Database",
                          "Checking EC", 
                          "Checking pH", 
                          "Could not \nread pH", 
                          "Adjusting \nnutrients", 
                          "Tank Level High", 
                          "Adjusting pH Up", 
                          "Adjusting pH Down", 
                          "Circulation Pump\n ON"};
