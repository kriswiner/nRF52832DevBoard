/* SensorBoard v.01

   This example code is in the public domain.
*/

#include "Wire.h"   
#include <SPI.h>
#include <BLEPeripheral.h>
#include "BMA280.h"
#include "BME280.h"
#include "VEML6040.h"
#include "SPIFlash.h"

#define myLed1 7 // green led
#define myLed2 8 // red led
#define myLed3 9 // blue led
#define VbatMon A2 // resistor divider 

#define SerialDebug true  // set to true to get Serial output for debugging

//BMA280 definitions
#define BMA280_intPin1  0  // interrupt1 pin definitions
#define BMA280_intPin2  1  // interrupt2 pin definitions

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      BW_7_81Hz, BW_15_63Hz, BW_31_25Hz, BW_62_5Hz, BW_125Hz, BW_250Hz, BW_500Hz, BW_1000Hz
      normal_Mode, deepSuspend_Mode, lowPower_Mode, suspend_Mode
      sleep_0_5ms, sleep_1ms, sleep_2ms, sleep_4ms, sleep_6ms, sleep_10ms, sleep_25ms, sleep_50ms, sleep_100ms, sleep_500ms, sleep_1000ms
*/ 
uint8_t Ascale = AFS_2G, BW = BW_62_5Hz, power_Mode = lowPower_Mode, sleep_dur = sleep_50ms, tapStatus, tapType;

float aRes;             // scale resolutions per LSB for the sensor
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float ax, ay, az;       // variables to hold latest sensor data values 

bool newBMA280Data = false;
bool newBMA280Tap  = false;

BMA280 BMA280(BMA280_intPin1, BMA280_intPin2); // instantiate BMA280 class


// BME280 definitions
/* Specify BME280 configuration
 *  Choices are:
 P_OSR_01, P_OSR_02, P_OSR_04, P_OSR_08, P_OSR_16 // pressure oversampling
 H_OSR_01, H_OSR_02, H_OSR_04, H_OSR_08, H_OSR_16 // humidity oversampling
 T_OSR_01, T_OSR_02, T_OSR_04, T_OSR_08, T_OSR_16 // temperature oversampling
 full, BW0_223ODR,BW0_092ODR, BW0_042ODR, BW0_021ODR // bandwidth at 0.021 x sample rate
 BME280Sleep, forced,, forced2, normal //operation modes
 t_00_5ms = 0, t_62_5ms, t_125ms, t_250ms, t_500ms, t_1000ms, t_10ms, t_20ms // determines sample rate
 */
uint8_t Posr = P_OSR_16, Hosr = H_OSR_16, Tosr = T_OSR_02, Mode = normal, IIRFilter = BW0_021ODR, SBy = t_62_5ms;     // set pressure amd temperature output data rate

uint32_t rawPress, rawTemp, compHumidity, compTemp, compPress;   // pressure, humidity, and temperature raw count output for BME280
uint16_t rawHumidity;  // variables to hold raw BME280 humidity value

float temperature_C, temperature_F, pressure, humidity, altitude; // Scaled output of the BME280

BME280 BME280; // instantiate BME280 class

// Specify VEML6040 Integration time
/*Choices are:
 IT_40  40 ms, IT_80 80 ms, IT_160  160 ms, IT_320  320 ms, IT_640 640 ms, IT_1280 1280 ms*/
uint8_t IT = IT_160;  // integration time variable
uint8_t ITime = 160;  // integration time in milliseconds
int16_t RGBWData[4] = {0, 0, 0, 0};
float GSensitivity = 0.25168/((float) (IT + 1)); // ambient light sensitivity increases with integration time
float redLight, greenLight, blueLight, ambientLight;

VEML6040 VEML6040;

// 8 MBit (1 MByte) SPI Flash 4096, 256-byte pages
#define csPin 10 // SPI Flash chip select pin

uint16_t page_number = 0;     // set the page mumber for flash page write
uint8_t  sector_number = 0;   // set the sector number for sector write
uint8_t  flashPage[256];      // array to hold the data for flash page write

SPIFlash SPIFlash(csPin);

BLEPeripheral blePeripheral = BLEPeripheral();

// Environmental Sensing Service
BLEService env_sensingService = BLEService("181A");
BLEShortCharacteristic tempCharacteristic = BLEShortCharacteristic("2A6E", BLERead | BLENotify); // temperature is int16_t
BLEDescriptor tempDescriptor = BLEDescriptor("2901", "Temp C x 100");

BLEUnsignedLongCharacteristic pressureCharacteristic = BLEUnsignedLongCharacteristic("2A6D", BLERead | BLENotify); // pressure is uint32_t
BLEDescriptor pressureDescriptor = BLEDescriptor("2901", "Pressure Pa x 10");

BLEUnsignedShortCharacteristic humidityCharacteristic = BLEUnsignedShortCharacteristic("2A6F", BLERead | BLENotify); // humidity is uint16_t
BLEDescriptor humidityDescriptor = BLEDescriptor("2901", "Humidity %rH x 10");

BLELongCharacteristic elevationCharacteristic = BLELongCharacteristic("2A6C", BLERead | BLENotify); // elevation is int24_t
BLEDescriptor elevationDescriptor = BLEDescriptor("2901", "Elevation meters x 100");

// Battery Service
BLEService batteryService = BLEService("180F");
BLEUnsignedShortCharacteristic battlevelCharacteristic = BLEUnsignedShortCharacteristic("2A19", BLERead | BLENotify); // battery level is uint8_t
BLEDescriptor battlevelDescriptor = BLEDescriptor("2901", "Battery Level 0 - 100");

int16_t  lastTempReading;
uint32_t lastPressureReading;
uint16_t lastHumidityReading;
int32_t  lastElevationReading;
uint8_t  lastBattLevelReading;

float   VBAT; // battery voltage monitor by VbatMon
uint16_t rawVbat;
int16_t level;  // result of analog red of voltage divider monitoring battery voltage
float pi = 3.14159f;
uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate

void setup() 
{
  Serial.begin(38400);
  delay(2000);
  Serial.println("Serial enabled!");

  Wire.begin(); // set default I2C on pins 6 and 7
  Wire.setClock(100000);

  BME280.I2Cscan(); // should detect BME280 at 0x77, BMA280 at 0x18 
 
  pinMode(myLed1, OUTPUT);
  pinMode(myLed2, OUTPUT);
  pinMode(myLed3, OUTPUT);
  digitalWrite(myLed1, HIGH);  // start with leds off, since active HIGH
  digitalWrite(myLed2, HIGH);  // start with leds off, since active HIGH
  digitalWrite(myLed3, HIGH);  // start with leds off, since active HIGH

  // Voltage divider 27K/100K to monitor LiPo battery voltage
  pinMode(VbatMon, INPUT);
  analogReadResolution(12); // take advantage of 12-bit ADCs

  // Read the BMS280 Chip ID register, this is a good test of communication
  Serial.println("BMA280 accelerometer...");
  byte c = BMA280.getChipID();  // Read CHIP_ID register for BMA280
  Serial.print("BMA280 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0xFB, HEX);
  Serial.println(" ");
  delay(1000); 
  
  // Read the WHO_AM_I register of the BME280 this is a good test of communication
  byte d = BME280.getChipID();  // Read WHO_AM_I register for BME280
  Serial.print("BME280 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x60, HEX);
  Serial.println(" ");
  delay(1000); 

  if(c == 0xFB && d == 0x60 ) {

  Serial.println("BMA280 + BME280 online..."); Serial.println(" ");

   aRes = BMA280.getAres(Ascale);                                     // get sensor resolutions, only need to do this once
   BMA280.selfTestBMA280();                                           // perform sensor self test
   BMA280.resetBMA280();                                              // software reset before initialization
   delay(1000);                                                       // give some time to read the screen
   BMA280.initBMA280(Ascale, BW, power_Mode, sleep_dur);              // initialize sensor for data acquisition
   BMA280.fastCompensationBMA280();                                   // quickly estimate offset bias


  BME280.resetBME280(); // reset BME280 before initilization
  delay(100);

  BME280.BME280Init(Posr, Hosr, Tosr, Mode, IIRFilter, SBy); // Initialize BME280 altimeter

  blePeripheral.setLocalName("Environmental Sensor Sketch");

 // Environmental Sensor service
  blePeripheral.setAdvertisedServiceUuid(env_sensingService.uuid());
  blePeripheral.addAttribute(env_sensingService);
  blePeripheral.addAttribute(tempCharacteristic);
  blePeripheral.addAttribute(tempDescriptor);
  blePeripheral.addAttribute(pressureCharacteristic);
  blePeripheral.addAttribute(pressureDescriptor);
  blePeripheral.addAttribute(humidityCharacteristic);
  blePeripheral.addAttribute(humidityDescriptor);
  blePeripheral.addAttribute(elevationCharacteristic);
  blePeripheral.addAttribute(elevationDescriptor);

  // Battery service
  blePeripheral.setAdvertisedServiceUuid(batteryService.uuid());
  blePeripheral.addAttribute(batteryService);
  blePeripheral.addAttribute(battlevelCharacteristic);
  blePeripheral.addAttribute(battlevelDescriptor);

  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  blePeripheral.begin();

  Serial.println(F("BLE MENTAID Peripheral"));
  
  }
  else 
  {
//    if(c != 0xFB) Serial.println(" BMA280 not functioning!");
    if(d != 0x60) Serial.println(" BME280 not functioning!");    
  }

  SPI.begin();
  delay(100);
  // check SPI Flash ID
  SPIFlash.SPIFlashinit();
  SPIFlash.getChipID();
  // Check VBAT to avoud brown-out loss of data!
  rawVbat = analogRead(VbatMon);
  VBAT = (127.0f/100.0f) * 3.30f * ((float)rawVbat)/4095.0f;
  if(VBAT > 3.70) SPIFlash.flash_chip_erase(true); // full erase only if the battery is still good  

  VEML6040.enableVEML6040(IT); // initalize sensor
  delay(150);  

  attachInterrupt(BMA280_intPin1, myinthandler1, RISING);  // define interrupt for INT1 pin output of BMA280
  attachInterrupt(BMA280_intPin2, myinthandler2, RISING);  // define interrupt for INT2 pin output of BMA280


 } /* end of setup */

void loop() 
{
  blePeripheral.poll();

   // BMA280 acceleration
   if(newBMA280Data == true) {  // On interrupt, read data
     newBMA280Data = false;  // reset newData flag

     BMA280.readBMA280AccelData(accelCount); // get 14-bit signed accel data
     
     // Now we'll calculate the accleration value into actual g's
     ax = (float)accelCount[0]*aRes/4.0f;  // get actual g value, this depends on scale being set
     ay = (float)accelCount[1]*aRes/4.0f;   
     az = (float)accelCount[2]*aRes/4.0f;  
    }

    // Check for BMA280 taps
    if(newBMA280Tap == true) {  // On interrupt, identify tap
      newBMA280Tap = false;
 
      tapType = BMA280.getTapType();
      if(tapType & 0x20) {
        Serial.println("Single tap detected!");
        digitalWrite(myLed2, LOW); delay(10); digitalWrite(myLed2, HIGH);
      }
      if(tapType & 0x10) {
        Serial.println("Double tap detected!"); 
        digitalWrite(myLed3, LOW); delay(10); digitalWrite(myLed3, HIGH);
      }
      
      tapStatus = BMA280.getTapStatus();  // Read tap status register

      if(tapStatus & 0x80) {
        Serial.println("Tap is negative");
        }
      else {
        Serial.println("Tap is positive");
      }

       if(tapStatus & 0x40) {
        Serial.println("Tap is on x");
       }
       if(tapStatus & 0x20) {
        Serial.println("Tap is on y");
       }
       if(tapStatus & 0x10) {
        Serial.println("Tap is on z");
       }      
       
    }

    
  // Serial print and/or display at 0.5 s rate independent of data rates
  delt_t = millis() - count;
  if (delt_t > 500) { // update LCD once per half-second independent of read rate

    if(SerialDebug) {
    Serial.print("ax = ");  Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    }

    tempCount = BMA280.readBMA280TempData();  // Read the accel chip temperature adc values
    temperature = 0.5f * ((float) tempCount) + 23.0f; // Accel chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
      Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C        
    }


    rawTemp =  BME280.readBME280Temperature();
    setTempCharacteristicValue();  // set BLE temperature value
    compTemp = BME280.BME280_compensate_T(rawTemp);
    temperature_C = (float) compTemp/100.0f;
    temperature_F = 9.0f*temperature_C/5.0f + 32.0f;
     
    rawPress =  BME280.readBME280Pressure();
    setPressureCharacteristicValue(); // set BLE pressure value
    compPress = BME280.BME280_compensate_P(rawPress);
    pressure = (float) compPress/25600.f; // Pressure in mbar
    altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));   
    setElevationCharacteristicValue(); // set BLE altitude value
   
    rawHumidity =  BME280.readBME280Humidity();
    setHumidityCharacteristicValue(); // set BLE humidity value
    compHumidity = BME280.BME280_compensate_H(rawHumidity);
    humidity = (float)compHumidity/1024.0f; // Humidity in %RH

    level = analogRead(VbatMon);
    VBAT = (127.0f/100.0f) * 3.30f * ((float)level)/4095.0f;
     if(SerialDebug) Serial.print("VBAT = "); Serial.println(VBAT, 2); 
    uint8_t battlevel = map(level, 0, 4095, 0, 100); 
    battlevelCharacteristic.setValue((uint8_t)battlevel); // set BLE battery level value

 
    if(SerialDebug){
    Serial.println("BME280:");
    Serial.print("Altimeter temperature = "); 
    Serial.print( temperature_C, 2); 
    Serial.println(" C"); // temperature in degrees Celsius
    Serial.print("Altimeter temperature = "); 
    Serial.print(temperature_F, 2); 
    Serial.println(" F"); // temperature in degrees Fahrenheit
    Serial.print("Altimeter pressure = "); 
    Serial.print(pressure, 2);  
    Serial.println(" mbar");// pressure in millibar
    Serial.print("Altitude = "); 
    Serial.print(altitude, 2); 
    Serial.println(" feet");
    Serial.print("Altimeter humidity = "); 
    Serial.print(humidity, 1);  
    Serial.println(" %RH");// pressure in millibar
    Serial.println(" ");

    // VEML6040 Data
    VEML6040.enableVEML6040(IT); // enable VEML6040 sensor
    delay(ITime);  // wait for integration of light sensor data
    VEML6040.getRGBWdata(RGBWData); // read light sensor data
    VEML6040.disableVEML6040(IT); // disable VEML6040 sensor
    
    if(SerialDebug){
    Serial.print("Red raw counts = ");   Serial.println(RGBWData[0]);
    Serial.print("Green raw counts = "); Serial.println(RGBWData[1]);
    Serial.print("Blue raw counts = ");  Serial.println(RGBWData[2]);
    Serial.print("White raw counts = "); Serial.println(RGBWData[3]);
    Serial.print("Inferred IR raw counts = "); Serial.println(RGBWData[3] - RGBWData[0] - RGBWData[1] - RGBWData[2]);
    Serial.println("  ");
 
    Serial.print("Red   light power density = "); Serial.print((float)RGBWData[0]/96.0f, 2); Serial.println(" microWatt/cm^2");
    Serial.print("Green light power density = "); Serial.print((float)RGBWData[1]/74.0f, 2); Serial.println(" microWatt/cm^2");
    Serial.print("Blue  light power density = "); Serial.print((float)RGBWData[2]/56.0f, 2); Serial.println(" microWatt/cm^2");
    Serial.println("  ");

    Serial.print("Ambient light intensity = "); Serial.print((float)RGBWData[1]*GSensitivity, 2); Serial.println(" lux");
    Serial.println("  ");
    }

    // Empirical estimation of the correlated color temperature CCT:
    // see https://www.vishay.com/docs/84331/designingveml6040.pdf
    float temp = ( (float) (RGBWData[0] - RGBWData[2])/(float) RGBWData[1] );
    float CCT = 4278.6f*pow(temp, -1.2455f) + 0.5f;

     if(SerialDebug) {
     Serial.print("Correlated Color Temperature = "); Serial.print(CCT); Serial.println(" Kelvin");
     Serial.println("  ");
     }
     
    }
    
       count = millis(); 
       digitalWrite(myLed1, LOW); delay(10); digitalWrite(myLed1, HIGH);
  }
    
 
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void myinthandler1()
{
  newBMA280Data = true;  // Just set flag when interrupt received, don't try reading data in an interrupt handler
}

void myinthandler2()
{
  newBMA280Tap = true;
}


void setTempCharacteristicValue() {
    int16_t reading = BME280.BME280_compensate_T(rawTemp);

  if (!isnan(reading) && significantChange(lastTempReading, reading, 0.1)) {
   // sends the data LSByte first, resolved to 0.01 C
   // sending temperature as Centigrade x 100
   tempCharacteristic.setValue((int16_t)(reading));  // temperature in hC

    Serial.print(F("Temperature: ")); Serial.print((int16_t)(reading)); Serial.println(F(" C x 100"));

    lastTempReading = reading;
  }
}

void setPressureCharacteristicValue() {
    uint32_t reading = BME280.BME280_compensate_P(rawPress);

  if (!isnan(reading) && significantChange(lastPressureReading, reading, 1.0)) {
   // sends the data LSByte first, resolved to 0.1 Pa
   // sending data as Pa x 10
    pressureCharacteristic.setValue((uint32_t)((10 * reading)/256)); // pressure, convert to Pa x 10

    Serial.print(F("Pressure: ")); Serial.print((uint32_t)((10 * reading)/256)); Serial.println(F(" Pa x 10"));

    lastPressureReading = reading;
  }
}

void setElevationCharacteristicValue() {
    int32_t reading = 100*altitude*0.3048f;  // elevation in meters

  if (!isnan(reading) && significantChange(lastElevationReading, reading, 1.0)) {
   // sends the data LSByte first, resolved to 0.01 m
   // sending data as m x 100
    elevationCharacteristic.setValue((int32_t)(reading)); // elevation, convert to m x 100

    Serial.print(F("Elevation: ")); Serial.print((int32_t)(reading)); Serial.println(F(" meters x 100"));

    lastPressureReading = reading;
  }
}

void setHumidityCharacteristicValue() {
    uint32_t reading = BME280.BME280_compensate_H(rawHumidity);

  if (!isnan(reading) && significantChange(lastHumidityReading, reading, 1.0)) {
   // sends the data LSByte first, resolved to 0.01 %rH
   // sending data as % rH x 100
    humidityCharacteristic.setValue((uint16_t)((100 * reading)/1024)); // humidity, convert to %rH x 100

    Serial.print(F("Humidity: ")); Serial.print((uint16_t)((100 * reading)/1024)); Serial.println(F(" %rH x 100"));

    lastHumidityReading = reading;
  }
}

void setBattlevelCharacteristicValue() {
    uint8_t reading = map(level, 0, 4095, 0, 100);;

  if (!isnan(reading) && significantChange(lastBattLevelReading, reading, 1.0)) {
   // sends single Byte
   // sending data as 0 - 100
    battlevelCharacteristic.setValue((uint8_t)(reading)); // battery level, 0 - 100

    Serial.print(F("Battery Level: ")); Serial.print((uint8_t)(reading)); Serial.println(F(" %"));

    lastBattLevelReading = reading;
  }
}

boolean significantChange(float val1, float val2, float threshold) {
  return (abs(val1 - val2) >= threshold);
}

void blePeripheralConnectHandler(BLECentral& central) {
  Serial.print(F("Connected event, central: "));
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  Serial.print(F("Disconnected event, central: "));
  Serial.println(central.address());
}

