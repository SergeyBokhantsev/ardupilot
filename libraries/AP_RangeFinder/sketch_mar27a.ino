#include <Wire.h> // I2C head file

#define TFmini_Addr   0x10   

int dist;
int count;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Initiate the Wire library and join the I2C bus as a master or Slave.
  Wire.begin(); 
  Serial.print("Ready to Read TFmini\r\n");
  delay(10);

  count = 0;

  //SetMode(false);

return;

  Wire.beginTransmission(TFmini_Addr); // Begin a transmission to the I2C Slave device with the given address.
  Wire.write(00); // Reg's Address_H
  Wire.write(81); // Reg's Address_L
  Wire.write(1); // Data Length
  Wire.endTransmission(0);  // Send a START Sign

  Wire.beginTransmission(TFmini_Addr); // Begin a transmission to the I2C Slave device with the given address.
  Wire.write(0); // Data Length
  Wire.endTransmission(1);  // Send a START Sign

delay(100);


return;

  Wire.beginTransmission(TFmini_Addr); // Begin a transmission to the I2C Slave device with the given address.
  Wire.write(00); // Reg's Address_H
  Wire.write(80); // Reg's Address_L
  Wire.write(1); // Data Length
  Wire.endTransmission(0);  // Send a START Sign

  Wire.beginTransmission(TFmini_Addr); // Begin a transmission to the I2C Slave device with the given address.
  Wire.write(7); // Data Length
  Wire.endTransmission(1);  // Send a START Sign

delay(1000);



return;

  Serial.print("1\r\n");
  Wire.beginTransmission(TFmini_Addr); // Begin a transmission to the I2C Slave device with the given address.
  Serial.print("1.1\r\n");
  Wire.write(00); // Reg's Address_H - 00
  Wire.write(39); // Reg's Address_L - 27
  Wire.write(1); // Data Length   
  Serial.print("1.2\r\n");
  Wire.endTransmission(0);  // Send a START Sign

Wire.beginTransmission(TFmini_Addr); // Begin a transmission to the I2C Slave device with the given address.
  Wire.write(1); // Data Length   
  Serial.print("1.3\r\n");
Wire.endTransmission(1);  // Send a START Sign

Serial.print("Configured\r\n");

//while( Wire.available())
  //Wire.read();

Serial.print("Done\r\n");

  delay(10);

 // Wire.begin(); 
}






void loop() {

count++;

if (count > 100)
{
   Read();
   count = 0;
}

Trigger();

delay(10);
  
}

void Read()
{
    // put your main code here, to run repeatedly:
  byte i = 0;
  byte rx_Num = 0;  // the bytes of received by I2C
  byte rx_buf[7] = {0}; // received buffer by I2C

  
  Wire.beginTransmission(TFmini_Addr); // Begin a transmission to the I2C Slave device with the given address.
//  Serial.print("r-2\r\n");
  Wire.write(1); // Reg's Address_H
  Wire.write(2); // Reg's Address_L
  Wire.write(7); // Data Length
  Wire.endTransmission(0);  // Send a START Sign
 // Serial.print("r-3\r\n");
  
  // Wire.requestFrom（AA,BB）;receive the data form slave.
  // AA: Slave Address ; BB: Data Bytes 
  rx_Num = Wire.requestFrom(TFmini_Addr, 7); 
 // Serial.print("r-4\r\n");

  // Wire.available: Retuens the number of bytes available for retrieval with read().
  while( Wire.available())
  {
      rx_buf[i] = Wire.read(); // received one byte
      i++;
  }

 //Serial.print(rx_buf[2]|(rx_buf[3] << 8));
 // Serial.print("\r\n");
  
  // OUTPUT

dist = rx_buf[2]|(rx_buf[3] << 8);
  
  Serial.print("TrigFlag= ");
  Serial.print(rx_buf[0]);
  Serial.print(",Dist= ");
  Serial.print(dist);
  Serial.print(",Strength= ");
  Serial.print(rx_buf[4]|(rx_buf[5] << 8));
  Serial.print(",Inttime= ");
  Serial.print(rx_buf[6]);
  Serial.print("\r\n");
}

void Trigger()
{
  
 Wire.beginTransmission(TFmini_Addr); // Begin a transmission to the I2C Slave device with the given address.
  Wire.write(1); // Reg's Address_H
  Wire.write(0); // Reg's Address_L
  Wire.write(1); // Data Length
  Wire.endTransmission(0);  // Send a START Sign

  Wire.beginTransmission(TFmini_Addr); // Begin a transmission to the I2C Slave device with the given address.
  Wire.write(1); // Data Length
  Wire.endTransmission(1);  // Send a START Sign
}

void SetMode(bool internalTrigger)
{
  byte mode = internalTrigger ? 0 : 1;

  Serial.print("Mode: ");
  Serial.print(mode);
  Serial.print("\r\n");
  
  Wire.beginTransmission(TFmini_Addr); // Begin a transmission to the I2C Slave device with the given address.
  //Serial.print("1.1\r\n");
  Wire.write(00); // Reg's Address_H - 00
  Wire.write(39); // Reg's Address_L - 27
  Wire.write(1); // Data Length   
  //Serial.print("1.2\r\n");
  Wire.endTransmission(0);  // Send a START Sign

  Wire.beginTransmission(TFmini_Addr); // Begin a transmission to the I2C Slave device with the given address.
  Wire.write(mode); // Data Length   
  Wire.endTransmission(1);  // Send a START Sign

  delay(100);
}


