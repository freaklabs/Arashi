/*******************************************/
// Arashi Pressure, Temperature, Humidity Board Firmware
// 
// This firmware makes use of the Adafruit soft I2C 
// library available from Adafruit to communicate with the I2C
// devices on the board.
/*******************************************/

#include <Adafruit_BMP085_soft.h>
#include <I2cMaster.h>
#include <chibi.h>
#include <Wire.h>

// turn on and off debug prints from this macro define
#define DEBUG 1

// allows printing or not printing based on the DEBUG VAR
#if (DEBUG == 1)
  #define DBG_PRINT(...)   Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
#endif

// general defines
#define MAX_BUFSZ 20
#define I2C_SLAVE_ADDR 4
#define DEFAULT_WIDTH 10
#define DEFAULT_PREC 2

// command defines
#define READ_OP 1
#define WRITE_OP 2  

#define TEMPERATURE 1
#define HUMIDITY 2
#define PRESSURE 3

// conversion factor to convert ADC value to voltage 
#define ADC_TO_VOLTAGE_CONV 0.0024   

// HIH constants for 25 deg C
// scale constant = 1/(0.00636 * Vcc) = 47.6463
// offset constant = 0.1515/0.00636 = 23.8208
#define HIH_SCALE_CONST 47.6463
#define HIH_OFFSET_CONST 23.8208

#define USE_SOFT_I2C 1
#define SDA_PIN 6
#define SCL_PIN 7

#define PWR_SWITCH 5

// I2C address of temp sensor
#define TEMP_ADDR 0x1F

uint8_t led_pin = 13;
uint8_t humidity_pin = 14;
uint8_t read_op = 0;

// Create an instance of soft I2C and then pass it to the adafruit pressure driver
SoftI2cMaster i2c(SDA_PIN, SCL_PIN);
Adafruit_BMP085_soft bmp(&i2c);

/**************************************************************************/
/*!
    Setup routine
*/
/**************************************************************************/
void setup()
{
  // turn on power
  pinMode(PWR_SWITCH, OUTPUT);
  digitalWrite(PWR_SWITCH, LOW);
  
  // turn off aux LED
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);
  
  // enable external voltage reference 
  analogReference(EXTERNAL);
  
  // set humidity pin to input (default)
  pinMode(humidity_pin, INPUT);
  
  // init command line interface
  chibiCmdInit(57600);
  
  // init pressure sensor
  bmp.begin();  
  
  // init I2C slave
  Wire.begin(I2C_SLAVE_ADDR);                
  Wire.onReceive(receiveEvent); 
  Wire.onRequest(requestEvent);   
  
  // initialize commands for command line interface
  chibiCmdAdd("temp", cmdReadTemp);  
  chibiCmdAdd("press", cmdReadPress);
  chibiCmdAdd("hum", cmdReadHum); 
  chibiCmdAdd("on", cmdPowerOn);  
  chibiCmdAdd("mcuwr", cmd_write_regs);
  chibiCmdAdd("mcurd", cmd_read_regs);
}

void loop()
{
  chibiCmdPoll();
}

/**************************************************************************/
/*!
    Event handler for when a write operation happens on the I2C interface
*/
/**************************************************************************/
void receiveEvent(int bytes)
{
  unsigned char i, numBytes;
  char x[MAX_BUFSZ];

/*  
  Serial.println("receiveEvent");
  
  i = 0;
  while (Wire.available())
  {
    x[i] = Wire.read();
    Serial.print(x[i]);
  }
*/  

  DBG_PRINTLN("receiveEvent");
  
  i = 0;
  while (Wire.available())
  {
    x[i] = Wire.read();
    DBG_PRINT(x[i], DEC); DBG_PRINT(" ");
    i++;
  }
  DBG_PRINTLN();
  
  numBytes = x[0];
  DBG_PRINT("numBytes: "); DBG_PRINTLN(numBytes, DEC);
    
  if (numBytes > MAX_BUFSZ)
  {
    // stop immediately if we overflow
    return;
  }
  
  switch (x[1])
  {
    case READ_OP:
      read_op = x[2];      
    break;
    
    case WRITE_OP:
      switch(x[2])
      {
        case TEMPERATURE:
        // get the address and then write value here
        break;
        
        case PRESSURE:
        // get address and write value here
        break;
        
        default:
        break;
      }
    break;
    
    default:
      read_op = 0;
    break;
  }
}

/**************************************************************************/
/*!
    Event handler for when a read operation arrives on the I2C interface
*/
/**************************************************************************/
void requestEvent()
{
  uint8_t i;
  char x[MAX_BUFSZ];
  float val;

  DBG_PRINT("REQ: ");
  
  switch (read_op)
  {
    case TEMPERATURE:
    val = getTemp();
    break;
    
    case HUMIDITY:
    val = getHumidity();
    break;
    
    case PRESSURE:
    val = bmp.readPressure();
    break;
    
    default:
    Serial.println("Error in requestEvent()");
    digitalWrite(led_pin, HIGH);
    return;
  }
  dtostrf(val, DEFAULT_WIDTH, DEFAULT_PREC, x);
  x[DEFAULT_WIDTH] = '\0';
  Wire.write(x);
  DBG_PRINTLN(x);
}

/**************************************************************************/
/*!
    Get Temperature
*/
/**************************************************************************/
double getTemp()
{
  uint8_t temp[2], temp_final;
  double temperature;
  bool sign;
  
  // Start read action. First point to temperature register
  i2c.start(TEMP_ADDR<<1 | I2C_WRITE);
  i2c.write(0x05);
  i2c.restart(TEMP_ADDR<<1 | I2C_READ);
  
  for (uint8_t i = 0; i<2; i++)
  {
    temp[i] = i2c.read(i == 1);
  }
  i2c.stop();
  
  // grab the sign bit
  sign = temp[0] & 0x10;
  
  // temperature = temp_upper * 16 + temp_lower/16
  temp[0] <<= 4; // mult upper temp by 16
  temperature = (double)temp[0] + ((double)temp[1]/16);
  
  temperature = (sign) ? (256 - temperature) : temperature;
  return temperature;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void cmdReadTemp(int arg_cnt, char **args)
{
  Serial.print("Temperature = "); Serial.println(getTemp(), 2);
}

/**************************************************************************/
/*!
    Get Humidity
*/
/**************************************************************************/
uint8_t getHumidity()
{
  int val, humidity;
  double humid, voltage;
  val = analogRead(humidity_pin);
  voltage = (double)val * ADC_TO_VOLTAGE_CONV;
  humid = (voltage * HIH_SCALE_CONST) - HIH_OFFSET_CONST;
  return humid;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void cmdReadHum(int arg_cnt, char **args)
{
  Serial.print("Humidity = "); Serial.println(getHumidity(), DEC);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void cmdReadPress(int arg_cnt, char **args)
{
  Serial.print("Pressure = "); Serial.println(bmp.readPressure());
  Serial.print("Altitude = "); Serial.println(round(bmp.readAltitude())); 
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void cmdPowerOn(int arg_cnt, char **args)
{
  int val = chibiCmdStr2Num(args[1], 10);
  digitalWrite(PWR_SWITCH, val == 0);
  digitalWrite(led_pin, val);
}

/**************************************************************************/
/*!
    Write regs
*/
/**************************************************************************/
void cmd_write_regs(int arg_cnt, char **args)
{
  unsigned char addr, val;
  
  addr = chibiCmdStr2Num(args[1], 16);
  val = chibiCmdStr2Num(args[2], 16);
  
  *(volatile unsigned char *)addr = val;
  
  val = *(volatile unsigned char *)addr;
  Serial.print("Addr "); Serial.print(addr, HEX); Serial.print(" = "); Serial.println(val, HEX);
}

/**************************************************************************/
/*!
    Read regs
*/
/**************************************************************************/
void cmd_read_regs(int arg_cnt, char **args)
{
  unsigned char addr, val;
  
  addr = chibiCmdStr2Num(args[1], 16);

  val = *(volatile unsigned char *)addr;
  Serial.print("Addr "); Serial.print(addr, HEX); Serial.print(" = "); Serial.println(val, HEX);
}
