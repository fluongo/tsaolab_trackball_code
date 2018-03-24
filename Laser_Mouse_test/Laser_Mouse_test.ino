#include <SPI.h>
#include <avr/pgmspace.h>



// Registers
#define REG_Product_ID                           0x00
#define REG_Revision_ID                          0x01
#define REG_Motion                               0x02
#define REG_Delta_X_L                            0x03
#define REG_Delta_X_H                            0x04
#define REG_Delta_Y_L                            0x05
#define REG_Delta_Y_H                            0x06
#define REG_SQUAL                                0x07
#define REG_Pixel_Sum                            0x08
#define REG_Maximum_Pixel                        0x09
#define REG_Minimum_Pixel                        0x0a
#define REG_Shutter_Lower                        0x0b
#define REG_Shutter_Upper                        0x0c
#define REG_Frame_Period_Lower                   0x0d
#define REG_Frame_Period_Upper                   0x0e
#define REG_Configuration_I                      0x0f
#define REG_Configuration_II                     0x10
#define REG_Frame_Capture                        0x12
#define REG_SROM_Enable                          0x13
#define REG_Run_Downshift                        0x14
#define REG_Rest1_Rate                           0x15
#define REG_Rest1_Downshift                      0x16
#define REG_Rest2_Rate                           0x17
#define REG_Rest2_Downshift                      0x18
#define REG_Rest3_Rate                           0x19
#define REG_Frame_Period_Max_Bound_Lower         0x1a
#define REG_Frame_Period_Max_Bound_Upper         0x1b
#define REG_Frame_Period_Min_Bound_Lower         0x1c
#define REG_Frame_Period_Min_Bound_Upper         0x1d
#define REG_Shutter_Max_Bound_Lower              0x1e
#define REG_Shutter_Max_Bound_Upper              0x1f
#define REG_LASER_CTRL0                          0x20
#define REG_Observation                          0x24
#define REG_Data_Out_Lower                       0x25
#define REG_Data_Out_Upper                       0x26
#define REG_SROM_ID                              0x2a
#define REG_Lift_Detection_Thr                   0x2e
#define REG_Configuration_V                      0x2f
#define REG_Configuration_IV                     0x39
#define REG_Power_Up_Reset                       0x3a
#define REG_Shutdown                             0x3b
#define REG_Inverse_Product_ID                   0x3f
#define REG_Motion_Burst                         0x50
#define REG_SROM_Load_Burst                      0x62
#define REG_Pixel_Burst                          0x64

// Accumulates the counts since last read request  
int slave1distance=0;
int slave2distance=0;

byte rx;

byte xydat[4];
int16_t* x = (int16_t*)&xydat[0];
int16_t* y = (int16_t*)&xydat[2];

boolean StartPolling = false;

const int slave1 = 10;

extern const unsigned short firmware_length;
extern const char firmware_data[];

void setup() {
  Serial.begin(9600);
  
  pinMode (slave1, OUTPUT); 
  
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(8);
}

void adns_com_begin(int slave){
  digitalWrite(slave, LOW);
}

void adns_com_end(int slave){
  digitalWrite(slave, HIGH);
}

byte adns_read_reg(int slave, byte reg_addr){
  adns_com_begin(slave);
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-slave1 for read operation is 120ns
  adns_com_end(slave);
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-slave1

  return data;
}

void adns_write_reg(int slave, byte reg_addr, byte data){
  adns_com_begin(slave);
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-slave1 for write operation
  adns_com_end(slave);
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-slave1. Could be shortened, but is looks like a safe lower bound 
}

void adns_upload_firmware(int slave){
  // send the firmware to the chip, cf p.18 of the datasheet
  // set the configuration_IV register in 3k firmware mode
  adns_write_reg(slave, REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved 
  
  // write 0x1d in SROM_enable reg for initializing
  adns_write_reg(slave, REG_SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(20); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns_write_reg(slave, REG_SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns_com_begin(slave);
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(20);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    //adns_write_reg(REG_SROM_Load_Burst,c);
    delayMicroseconds(20);
  }
  adns_com_end(slave);
  }


void performStartup(int slave)
{
  // Reset the SPI port
  adns_com_end(slave); 
  adns_com_begin(slave);
  
  
  adns_com_end(slave); // ensure that the serial port is reset
  adns_write_reg(slave, REG_Power_Up_Reset, 0x5a); // force reset
  
  // Wait for reboot
  delay(60);
  
  // read registers 0x02 to 0x06 (and discard the data)
  adns_read_reg(slave1, REG_Motion);
  adns_read_reg(slave1, REG_Delta_X_L);
  adns_read_reg(slave1, REG_Delta_X_H);
  adns_read_reg(slave1, REG_Delta_Y_L);
  adns_read_reg(slave1, REG_Delta_Y_H);
  
  // SROM download
  adns_upload_firmware(slave);
  delay(20);
  
  // Enable laser by setting Forced_Disable bit of LASER_CTRL0
  byte laser_ctrl0 = adns_read_reg(slave, REG_LASER_CTRL0);
  adns_write_reg(slave, REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );
  
  delay(1);
}

int convTwosComp(int b){
  //Convert from 2's complement
  if(b & 0x80){
    b = -1 * ((b ^ 0xff) + 1);
    }
  return b;
  }

void loop() 
{
  if (StartPolling == true)
  {    
    digitalWrite(slave1,LOW);
    xydat[0] = (byte)adns_read_reg(slave1, REG_Delta_X_L);
    xydat[1] = (byte)adns_read_reg(slave1, REG_Delta_X_H);    
    xydat[2] = (byte)adns_read_reg(slave1, REG_Delta_Y_L);
    xydat[3] = (byte)adns_read_reg(slave1, REG_Delta_Y_H);
    slave1distance = slave1distance + *y;
    slave2distance = slave2distance + *x;

    digitalWrite(slave1,HIGH);   
    delay(2);

  }
}
  
  
void serialEvent()
{ 
    while(Serial.available())
    {
      rx = Serial.read();
      switch (rx)
      {
        case 0x01:  // Sensor initialization requested from host
           performStartup(slave1);  
           //DisplayRegisters(slave1);
           delay(100);
           Serial.write(0x01);
           break;        
      
        case 0x03:  // Get DPI and LIFT DISTANCE request from host
           rx = adns_read_reg(slave1, REG_Configuration_I);
           Serial.write(rx);
           rx = adns_read_reg(slave1, REG_Lift_Detection_Thr);
           Serial.write(rx);
           break;
       
        case 0x04: // Host requests to set sensor DPI and LIFT DISTANCE
           delay(10);   // Ensure that data has arrived in the buffer
           rx = Serial.read();
           adns_write_reg(slave1, REG_Configuration_I, rx);
           rx = Serial.read();
           adns_write_reg(slave1, REG_Lift_Detection_Thr, rx);
           break;
           
        case 0x05: // Host requests start of sensor polling
          slave1distance = 0;          
          slave2distance = 0;
           StartPolling = true;
           break;
        
        case 0x06: // Host requests end of sensor polling
           StartPolling = false;
           break;
           
        case 0x07: // Host requests motion data
           Serial.write(lowByte(slave1distance));
           Serial.write(highByte(slave1distance));
           Serial.write(lowByte(slave2distance));
           Serial.write(highByte(slave2distance));
           slave1distance = 0; // X value
           slave2distance = 0; // Y value
           
           break;
      }      
    }
}
