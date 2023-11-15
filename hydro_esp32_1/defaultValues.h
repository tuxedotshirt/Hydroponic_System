//Default settings

#define sensorReadDelay 1000*60*1
#define ecInterval 1000*60*10 
//#define ecAdjustInterval 60000 //300000
#define ECPumpOnFactor 5000

#define phInterval 1000*60*10 
//#define phAdjustInterval 60000 //300000 
#define pHPumpOn 1000*8
#define pHTolerance 0.25 //pH will be kept within +- this value

#define phChemCounterLimit 10
#define nutrientCounterLimit 10

#define updateDBTime 1000*60*15

#define mainPumpTime 1000*60*5 //run time
#define mainPumpWait 1000*60*1  //time between runs

#define wifiConnect 1000*20 //try to connect to wifi for 20 seconds

#define timeCheck 1000*60*60 //check the time once per hour
