/*
 * Connections
 * 
 * NodeMCU D1 --> MMC58883MA-B P2-2 (SCL)
 * NodeMCU D2 --> MMC58883MA-B P2-3 (SDA)
 * NodeMCU 3V3 --> MMC5883MA-B P1-2, P1-3 (3V)
 * NodeMCU GND --> MMC5883MA-B P1-1 (GND)
 */
 
#include <Wire.h>
#include "UbidotsMicroESP8266.h"
#include "Arduino.h"

#define TOKEN  "Your_token_here"  // Put here your Ubidots TOKEN
#define WIFISSID "Your_WiFi_SSID" // Put here your Wi-Fi SSID
#define PASSWORD "Your_WiFi_Password" // Put here your Wi-Fi password

Ubidots client(TOKEN);

//*****************************************************
// MMC5883MA Register map
//*****************************************************
#define XOUT_LSB    0x00
#define XOUT_MSB    0x01
#define YOUT_LSB    0x02
#define YOUT_MSB    0x03
#define ZOUT_LSB    0x04
#define ZOUT_MSB    0x05
#define TEMPERATURE 0x06
#define STATUS      0x07
#define INT_CTRL0   0x08
#define INT_CTRL1   0x09
#define INT_CTRL2   0x0A
#define X_THRESHOLD 0x0B
#define Y_THRESHOLD 0x0C
#define Z_THRESHOLD 0x0D
#define PROD_ID1    0x2F

#define MMC5883MA   0x30 // Sensor I2C address

#define MMC5883MA_OFFSET        32768
#define MMC5883MA_SENSITIVITY   4096

//*****************************************************
// Global variables
//*****************************************************
float mag_out [3]; // Magnetic field output - x, y, z

#define Trig D5
#define Echo D6

//*****************************************************
// Functions declaration
//*****************************************************
uint8_t read_register(uint8_t REG_ADDR);
void write_register(uint8_t REG_ADDR, uint8_t VALUE);
void wait_meas(void);
void reset_sensor(void);
void set_sensor(void);
void read_measurement(void);
void enable_sensor(void);
float parser(char axis_msb, char axis_lsb);

float read_ultrasonic (void);

//*****************************************************
// Setup
//*****************************************************
void setup() 
{
  Wire.begin();                                   // Join I2C bus (address optional for master)
  Serial.begin(9600);                             // Start USB serial port
  client.wifiConnection(WIFISSID, PASSWORD);

  enable_sensor();

  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
}

//*****************************************************
// Main
//*****************************************************
void loop()
{
  Serial.println("---------------");
  
  // Variables initialization
  uint8_t status_reg = 0;
  float x_reset, y_reset, z_reset;
  float x_set, y_set, z_set;
  float x, y, z;
  float offset_x, offset_y, offset_z;  
  float ultrasonic_cm;
  //------------------------------------------
  
  ultrasonic_cm = read_ultrasonic();              // Ultrasonic sensor measurement
  
  //------------------------------------------
  // RESET operation
  //------------------------------------------
  reset_sensor();
  write_register(INT_CTRL0, 0X01);                // Start magnetic field measurement
  wait_meas();
  read_measurement(); // Magnetic field in Gauss

  x_reset = mag_out[0];
  y_reset = mag_out[1];
  z_reset = mag_out[2];

  //------------------------------------------
  // SET operation
  //------------------------------------------
  set_sensor();
  write_register(INT_CTRL0, 0X01);                // Start magnetic field measurement
  wait_meas();
  read_measurement(); // Magnetic field in Gauss

  write_register(INT_CTRL0, 0X01);                // Start magnetic field measurement
  wait_meas();

  x_set = mag_out[0];
  y_set = mag_out[1];
  z_set = mag_out[2];

  //------------------------------------------
  
  x = (float)((x_set - x_reset) / 2) * 1000;    // Measurement in mili Gauss
  y = (float)((y_set - y_reset) / 2) * 1000;
  z = (float)((z_set - z_reset) / 2) * 1000;

  //------------------------------------------
  //Serial.println("Computed magnetic field");
  Serial.print("x: ");
  Serial.print(x, 5);
  Serial.print(", y: ");
  Serial.print(y, 5);
  Serial.print(", z: ");
  Serial.print(z, 5);
  Serial.println(" [mG]");
  Serial.print("Ultrasonic distance: ");
  Serial.print(ultrasonic_cm);
  Serial.println(" cm");
  
  //------------------------------------------
  // Ubidots payload
  
  client.setDataSourceName("parking_test_3");
  client.setDataSourceLabel("parking_test_3");
  client.add("x_mmc5883ma_mG", x);
  client.add("y_mmc5883ma_mG", y);
  client.add("z_mmc5883ma_mG", z);
  client.add("ultrasonic_cm", ultrasonic_cm);
  client.sendAll(true);
  //------------------------------------------  
  
  delay(1800);
}

//*****************************************************
// Functions definition
//*****************************************************
uint8_t read_register(uint8_t REG_ADDR)
{ 
  uint8_t reg_value = 0;
  bool flag;                                      // Flag is zero if there is no error
  int written;                                    // Number of bytes written

  Wire.beginTransmission(MMC5883MA);              // Adress of I2C device
  written = Wire.write(REG_ADDR);                 // Register address
  flag = Wire.endTransmission(false);
  
  Wire.requestFrom(MMC5883MA, 1);                 // Request 1 byte from I2C slave device
  while(Wire.available() < 1);
  reg_value = Wire.read();                        // Receive a byte as character
  
  return reg_value;
}
//-------------------------------------------------
void write_register(uint8_t REG_ADDR, uint8_t VALUE)
{
  Wire.beginTransmission(MMC5883MA);              // Adress of I2C device
  Wire.write(REG_ADDR);                           // Register address
  Wire.write(VALUE);                              // Value to be written
  Wire.endTransmission();
}
//-------------------------------------------------
void set_sensor(void)
{
  //Serial.println("Performing SET");
  write_register(INT_CTRL0, 0x08);                // SET instruction
  delay(10);
}
//-------------------------------------------------
void reset_sensor(void)
{
  //Serial.println("Performing RESET");
  write_register(INT_CTRL0, 0x10);                // RESET instruction
  delay(10);
}
//-------------------------------------------------
void wait_meas()
{
  uint8_t status_reg = 0;

  status_reg = read_register(STATUS);
  
  while((status_reg & 0x01) != 0x01)
  {
    status_reg = read_register(STATUS);
    delay(5);
  }
}
//-------------------------------------------------
void read_measurement(void)
{
  uint8_t x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb;
  float x_val, y_val, z_val; // In Gauss

  x_lsb = read_register(XOUT_LSB);                // Read magnetic field - x lsb
  x_msb = read_register(XOUT_MSB);                // Read magnetic field - x msb
  y_lsb = read_register(YOUT_LSB);                // Read magnetic field - y lsb
  y_msb = read_register(YOUT_MSB);                // Read magnetic field - y msb
  z_lsb = read_register(ZOUT_LSB);                // Read magnetic field - z lsb
  z_msb = read_register(ZOUT_MSB);                // Read magnetic field - z msb

  x_val = parser(x_msb, x_lsb);
  y_val = parser(y_msb, y_lsb);
  z_val = parser(z_msb, z_lsb);

  mag_out[0] = x_val;
  mag_out[1] = y_val;
  mag_out[2] = z_val;

  return;
}
//-------------------------------------------------
void enable_sensor(void)
{
  set_sensor();
  write_register(INT_CTRL1, 0x00);                // Output resolution: 16bit - 100 Hz
  write_register(INT_CTRL0, 0x01);                // Start magnetic field measurement
  delay(10);
}
//-------------------------------------------------
float parser(uint8_t MSB, uint8_t LSB)
{   
  uint16_t temp = (uint16_t)(MSB << 8 | LSB);     // 16 bits
  float ans = ((float)temp - MMC5883MA_OFFSET) / MMC5883MA_SENSITIVITY; // Measurement in Gauss
  return ans;
}
//-------------------------------------------------
float read_ultrasonic ()
{
  float duration, ultra_cm;

  // Trigger: HIGH pulse of 10 or more us. Recommended short LOW pulse before.
  digitalWrite(Trig, LOW);
  delayMicroseconds(5);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
 
  pinMode(Echo, INPUT);
  duration = pulseIn(Echo, HIGH);
 
  // Time into distance conversion
  ultra_cm = (duration/2) / 29.1;

  return ultra_cm;
}
