/* BMP280_nRF52
 *  
 *  Simple program to configure and read Bosch's BMP280 pressure/temperature sensor, in this case
 *  mounted on an nRF52 development board (https://www.tindie.com/products/onehorse/nrf52832-development-board/).
 *  The sketch is using the nRF52 DK variant and the variant.cpp file has been changed to account
 *  for SDA/SCL on pins P0.06 and P0.07, led on P0.24, interrupt on pin P0.08, and UART RX/TX on any pair of convenient
 *  remnant GPIOs, like P0.12 and P0.14.
 *  
 *  The sketch uses BLEPeripheral to create an environmental sensor service and defines pressure, temperature, 
 *  and elevation characteristics as well as a battery service to monitor the battery voltage level via a battery 
 *  level characteristic. A resistor divider needs to be added to any analog pin so the voltage can be sampled 
 *  between 0 and the 3V3 of the board (27K/100K will do) such that the ADC will read 0 at 0 V and 4095 at 
 *  4.2 V of LiPo battery voltage.
 *  
 *  The sketch initializes the BMP280, extracts the calibration parameters, reads the raw data, compensates 
 *  the data to get properly scaled pressure and temperature and then estimates altitude (elevation) from the pressure data.
 *  All of these are available to a smart device and the data can be read directly there. The BLEPeripheral API seems to
 *  supply the data LSByte first, and the data is represented in HEX on the smart device. But, in fact, all of the information is 
 *  available at the full resolution of the characteristic; some rearrangement and scaling might be required for an app that makes 
 *  use of this data.
 *  
 *  Lastly, this sketch requires that softdevice S132 be loaded into the nRF52 before the sketch is flashed 
 *  otherwise the BLE radio will not work. 
 *  
 *  This sketch was based on a template provided by Sandeep Mistry,
 *  according to the following license:
 *  
 *  Copyright (c) Sandeep Mistry. All rights reserved.
 *  Licensed under the MIT license. See LICENSE file in the project root for full license information.
 *
 *  Please refer to https://github.com/sandeepmistry/arduino-nRF5 for more information.
*/
#include "Wire.h"   
#include <SPI.h>
#include <BLEPeripheral.h>

// BMP280 registers
#define BMP280_TEMP_XLSB  0xFC
#define BMP280_TEMP_LSB   0xFB
#define BMP280_TEMP_MSB   0xFA
#define BMP280_PRESS_XLSB 0xF9
#define BMP280_PRESS_LSB  0xF8
#define BMP280_PRESS_MSB  0xF7
#define BMP280_CONFIG     0xF5
#define BMP280_CTRL_MEAS  0xF4
#define BMP280_STATUS     0xF3
#define BMP280_RESET      0xE0
#define BMP280_ID         0xD0  // should be 0x58
#define BMP280_CALIB00    0x88

#define BMP280_ADDRESS    0x77   // Address of BMP280 altimeter when ADO = 0

enum Posr {
  P_OSR_00 = 0,  // no op
  P_OSR_01,
  P_OSR_02,
  P_OSR_04,
  P_OSR_08,
  P_OSR_16
};

enum Tosr {
  T_OSR_00 = 0,  // no op
  T_OSR_01,
  T_OSR_02,
  T_OSR_04,
  T_OSR_08,
  T_OSR_16
};

enum IIRFilter {
  full = 0,  // bandwidth at full sample rate
  BW0_223ODR,
  BW0_092ODR,
  BW0_042ODR,
  BW0_021ODR // bandwidth at 0.021 x sample rate
};

enum Mode {
  BMP280Sleep = 0,
  forced,
  forced2,
  normal
};

enum SBy {
  t_00_5ms = 0,
  t_62_5ms,
  t_125ms,
  t_250ms,
  t_500ms,
  t_1000ms,
  t_2000ms,
  t_4000ms,
};

// Specify BMP280 configuration
uint8_t Posr = P_OSR_16, Tosr = T_OSR_02, Mode = normal, IIRFilter = BW0_042ODR, SBy = t_62_5ms;     // set pressure amd temperature output data rate
// t_fine carries fine temperature as global value for BMP280
int32_t t_fine;

// Pin definitions
int myLed1     = 12;  // red LED on the nRF52 development board
int VbatMon    = A4;  // pick P0.28

// BMP280 compensation parameters
uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
double Temperature, Pressure; // stores BMP280 pressures sensor pressure and temperature
int32_t rawPress, rawTemp;   // pressure and temperature raw count output for BMP280

int16_t tempCount, rawPressure, rawTemperature, level;   // pressure, temperature raw count output
float   temperature, pressure, altitude, VBAT; // Stores the MPU9250 internal chip temperature in degrees Celsius

uint32_t delt_t = 0, count = 0, sumCount = 0, slpcnt = 0;  // used to control display output rate

// Define the BLE services and characteristocs
BLEPeripheral blePeripheral = BLEPeripheral();

// Environmental Sensing Service
BLEService env_sensingService = BLEService("181A");
BLEShortCharacteristic tempCharacteristic = BLEShortCharacteristic("2A6E", BLERead | BLENotify); // temperature is int16_t
BLEDescriptor tempDescriptor = BLEDescriptor("2901", "Temp C x 100");

BLEUnsignedLongCharacteristic pressureCharacteristic = BLEUnsignedLongCharacteristic("2A6D", BLERead | BLENotify); // pressure is uint32_t
BLEDescriptor pressureDescriptor = BLEDescriptor("2901", "Pressure Pa x 10");

BLEUnsignedLongCharacteristic elevationCharacteristic = BLEUnsignedLongCharacteristic("2A6C", BLERead | BLENotify); // elevation is int24_t
BLEDescriptor elevationDescriptor = BLEDescriptor("2901", "Elevation meters x 100");

// Battery Service
BLEService batteryService = BLEService("180F");
BLEUnsignedShortCharacteristic battlevelCharacteristic = BLEUnsignedShortCharacteristic("2A19", BLERead | BLENotify); // battery level is uint8_t
BLEDescriptor battlevelDescriptor = BLEDescriptor("2901", "Battery Level 0 - 100");

int16_t  lastTempReading = 0;
uint32_t lastPressureReading = 0;
int32_t  lastElevationReading = 0;
uint8_t  lastBattLevelReading = 0;

void setup() {
  
  Serial.begin(38400);
  delay(2000);  // wait for the serial moinitor to be opened

  Wire.begin(); // set default I2C on pins 6 and 7
  Wire.setClock(100000);  // 100 kHz I2C bus, could use 400 kHz too

  pinMode(myLed1, OUTPUT);
  digitalWrite(myLed1, HIGH); // leds are active LOW

  // Voltage divider 27K/100K to monitor LiPo battery voltage
  pinMode(VbatMon, INPUT);
  analogReadResolution(12); // take advantage of 12-bit ADCs

  I2Cscan(); // should detect BMP280 at 0x77
  
  // Read the WHO_AM_I register of the BMP280 this is a good test of communication
  byte f = readByte(BMP280_ADDRESS, BMP280_ID);  // Read WHO_AM_I register for BMP280
  Serial.print("BMP280 "); 
  Serial.print("I AM "); 
  Serial.print(f, HEX); 
  Serial.print(" I should be "); 
  Serial.println(0x58, HEX);
  Serial.println(" ");
  
  delay(1000); 

  writeByte(BMP280_ADDRESS, BMP280_RESET, 0xB6); // reset BMP280 before initilization
  delay(100);

  BMP280Init(); // Initialize BMP280 altimeter
  Serial.println("Calibration coeficients:");
  Serial.print("dig_T1 ="); 
  Serial.println(dig_T1);
  Serial.print("dig_T2 ="); 
  Serial.println(dig_T2);
  Serial.print("dig_T3 ="); 
  Serial.println(dig_T3);
  Serial.print("dig_P1 ="); 
  Serial.println(dig_P1);
  Serial.print("dig_P2 ="); 
  Serial.println(dig_P2);
  Serial.print("dig_P3 ="); 
  Serial.println(dig_P3);
  Serial.print("dig_P4 ="); 
  Serial.println(dig_P4);
  Serial.print("dig_P5 ="); 
  Serial.println(dig_P5);
  Serial.print("dig_P6 ="); 
  Serial.println(dig_P6);
  Serial.print("dig_P7 ="); 
  Serial.println(dig_P7);
  Serial.print("dig_P8 ="); 
  Serial.println(dig_P8);
  Serial.print("dig_P9 ="); 
  Serial.println(dig_P9);
  
  delay(1000);  

  digitalWrite(myLed1, LOW); // Indicates successful initialization
  
  // instantiate the BLE Peripheral services and define attributes
  blePeripheral.setLocalName("Environmental Sensor Sketch");

 // Environmental Sensor service
  blePeripheral.setAdvertisedServiceUuid(env_sensingService.uuid());
  blePeripheral.addAttribute(env_sensingService);
  blePeripheral.addAttribute(tempCharacteristic);
  blePeripheral.addAttribute(tempDescriptor);
  blePeripheral.addAttribute(pressureCharacteristic);
  blePeripheral.addAttribute(pressureDescriptor);
  blePeripheral.addAttribute(elevationCharacteristic);
  blePeripheral.addAttribute(elevationDescriptor);

  // Battery service
  blePeripheral.setAdvertisedServiceUuid(batteryService.uuid());
  blePeripheral.addAttribute(batteryService);
  blePeripheral.addAttribute(battlevelCharacteristic);
  blePeripheral.addAttribute(battlevelDescriptor);

  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  blePeripheral.begin(); // enable BLE Peripheral

  Serial.println(F("BLE BMP280 Environmental Sensor Peripheral"));
}

void loop() {
  
    blePeripheral.poll(); // start radio polling

    rawTemp =   readBMP280Temperature();  // read raw temperature register data
    setTempCharacteristicValue();         // set the temperature characteristic value
    temperature = (float) bmp280_compensate_T(rawTemp)/100.0f;  // temperature in C
    rawPress =  readBMP280Pressure(); // read raw pressure register data
    setPressureCharacteristicValue(); // set the pressure characteristic value
    pressure = (float) bmp280_compensate_P(rawPress)/25600.0f; // Pressure in mbar

      Serial.println("BMP280:");
      Serial.print("Altimeter temperature = "); 
      Serial.print( temperature, 2); 
      Serial.println(" C"); // temperature in degrees Celsius
      Serial.print("Altimeter temperature = "); 
      Serial.print(9.0f*temperature/5.0f + 32.0f, 2); 
      Serial.println(" F"); // temperature in degrees Fahrenheit
      Serial.print("Altimeter pressure = "); 
      Serial.print(pressure, 2);  
      Serial.println(" mbar");// pressure in millibar
      altitude = 145366.45f*(1.0f - powf((pressure/1013.25f), 0.190284f));
      setElevationCharacteristicValue(); // set the altitude characteristic value
      Serial.print("Altitude = "); 
      Serial.print(altitude, 2); 
      Serial.println(" feet");
      Serial.println(" ");

      level = analogRead(VbatMon); // sample ADC to get battery voltage level
      VBAT = (127.0f/100.0f) * 3.30f * ((float)level)/4095.0f; // convert to the LiPo battery voltage
      Serial.print("VBAT = "); Serial.println(VBAT, 2); 
      uint8_t battlevel = map(level, 0, 4095, 0, 100); // map battery level from 0 - 100 %
      battlevelCharacteristic.setValue((uint8_t)battlevel); // set the battery level characteristic value
      
      digitalWrite(myLed1, HIGH); delay(100); digitalWrite(myLed1, LOW); // blink the led
      delay(1000);   
}


/* Useful functions */

void setTempCharacteristicValue() {
    int16_t reading = bmp280_compensate_T(rawTemp);

  if (!isnan(reading) && significantChange(lastTempReading, reading, 0.1)) {
   // sends the data LSByte first, resolved to 0.01 C
   // sending temperature as Centigrade x 100
   tempCharacteristic.setValue((int16_t)(reading));  // temperature in hC

    Serial.print(F("Temperature: ")); Serial.print((int16_t)(reading)); Serial.println(F(" C x 100"));

    lastTempReading = reading;
  }
}

void setPressureCharacteristicValue() {
    uint32_t reading = bmp280_compensate_P(rawPress);

  if (!isnan(reading) && significantChange(lastPressureReading, reading, 1.0)) {
   // sends the data LSByte first, resolved to 0.1 Pa
   // sending data as Pa x 10
    pressureCharacteristic.setValue((uint32_t)((10 * reading)/256)); // pressure, convert to Pa x 10

    Serial.print(F("Pressure: ")); Serial.print((uint32_t)((10 * reading)/256)); Serial.println(F(" Pa x 10"));

    lastPressureReading = reading;
  }
}

void setElevationCharacteristicValue() {
    int32_t reading = altitude*0.3048f;  // elevation in meters

  if (!isnan(reading) && significantChange(lastElevationReading, reading, 1.0)) {
   // sends the data LSByte first, resolved to 0.01 m
   // sending data as m x 100
    elevationCharacteristic.setValue((int32_t)(100 * reading)); // elevation, convert to Pa x 10

    Serial.print(F("Elevation: ")); Serial.print((int32_t)(100 * reading)); Serial.println(F(" meters x 10"));

    lastPressureReading = reading;
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


int32_t readBMP280Temperature()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BMP280_ADDRESS, BMP280_TEMP_MSB, 3, &rawData[0]);  
  return (int32_t) (((int32_t) rawData[0] << 16 | (int32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

int32_t readBMP280Pressure()
{
  uint8_t rawData[3];  // 20-bit pressure register data stored here
  readBytes(BMP280_ADDRESS, BMP280_PRESS_MSB, 3, &rawData[0]);  
  return (int32_t) (((int32_t) rawData[0] << 16 | (int32_t) rawData[1] << 8 | rawData[2]) >> 4);
}

void BMP280Init()
{
  // Configure the BMP280
  // Set T and P oversampling rates and sensor mode
  writeByte(BMP280_ADDRESS, BMP280_CTRL_MEAS, Tosr << 5 | Posr << 2 | Mode);
  // Set standby time interval in normal mode and bandwidth
  writeByte(BMP280_ADDRESS, BMP280_CONFIG, SBy << 5 | IIRFilter << 2);
  // Read and store calibration data
  uint8_t calib[24];
  readBytes(BMP280_ADDRESS, BMP280_CALIB00, 24, &calib[0]);

  for (int index = 0; index < 24; index++) {
    Serial.print(index); Serial.print(" = "); Serial.println(calib[index]);
  }

  dig_T1 = (uint16_t)(((uint16_t) calib[1] << 8) | calib[0]);
  dig_T2 = ( int16_t)((( int16_t) calib[3] << 8) | calib[2]);
  dig_T3 = ( int16_t)((( int16_t) calib[5] << 8) | calib[4]);
  dig_P1 = (uint16_t)(((uint16_t) calib[7] << 8) | calib[6]);
  dig_P2 = ( int16_t)((( int16_t) calib[9] << 8) | calib[8]);
  dig_P3 = ( int16_t)((( int16_t) calib[11] << 8) | calib[10]);
  dig_P4 = ( int16_t)((( int16_t) calib[13] << 8) | calib[12]);
  dig_P5 = ( int16_t)((( int16_t) calib[15] << 8) | calib[14]);
  dig_P6 = ( int16_t)((( int16_t) calib[17] << 8) | calib[16]);
  dig_P7 = ( int16_t)((( int16_t) calib[19] << 8) | calib[18]);
  dig_P8 = ( int16_t)((( int16_t) calib[21] << 8) | calib[20]);
  dig_P9 = ( int16_t)((( int16_t) calib[23] << 8) | calib[22]);
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of
// “5123” equals 51.23 DegC.
int32_t bmp280_compensate_T(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8
//fractional bits).
//Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t bmp280_compensate_P(int32_t adc_P)
{
  long long var1, var2, p;
  var1 = ((long long)t_fine) - 128000;
  var2 = var1 * var1 * (long long)dig_P6;
  var2 = var2 + ((var1*(long long)dig_P5)<<17);
  var2 = var2 + (((long long)dig_P4)<<35);
  var1 = ((var1 * var1 * (long long)dig_P3)>>8) + ((var1 * (long long)dig_P2)<<12);
  var1 = (((((long long)1)<<47)+var1))*((long long)dig_P1)>>33;
  if(var1 == 0)
  {
    return 0;
    // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - var2)*3125)/var1;
  var1 = (((long long)dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((long long)dig_P8) * p)>> 19;
  p = ((p + var1 + var2) >> 8) + (((long long)dig_P7)<<4);
  return (uint32_t)p;
}


// simple function to scan for I2C devices on the bus
void I2Cscan() 
{
    // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}


// I2C read/write functions for the BMP280 sensors

  void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

  uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data   
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, 1);            // Read one byte from slave register address 
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

  void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address 
  while (Wire.available()) {dest[i++] = Wire.read(); } // Put read results in the Rx buffer
}
