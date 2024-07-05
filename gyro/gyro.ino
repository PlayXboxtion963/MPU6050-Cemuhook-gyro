#include <Wire.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "MPU6050.h" // by jrowberg,
#include "I2Cdev.h"  // https://github.com/jrowberg/i2cdevlib
#include "CRC32.h"   // by bakercp, https://github.com/bakercp/CRC32
WiFiUDP udp;
uint8_t  udpIn[28];
uint8_t  udpInfoOut[32];
uint8_t  udpDataOut[100];

#define gyrosenstive 1


int16_t accXI, accYI, accZI, gyrPI, gyrYI, gyrRI; // Raw integer orientation data
float   accXF, accYF, accZF, gyrPF, gyrYF, gyrRF; // Float orientation data
uint32_t dataPacketNumber = 0; // Current data packet count


// All time variables are in microseconds
uint32_t u32_Data_SendTime   = 0; // Current time
uint32_t u32_LstFIFO_ChkTime = 0;
uint32_t u32_Recv_ChkTime    = 0;
//Time Period
#define FIFO_SEND_TIME  1000*1
#define RECV_CHECKTIME  1000*30
#define DATA_SEND_TIME  1000*20
//NetWork Manage
uint16_t DATA_REPLY_PORT = 0;
IPAddress DATA_REPLY_IP;

uint16_t INFO_REPLY_PORT = 0;
IPAddress INFO_REPLY_IP;

//NetWork Packet define
const uint32_t infoResponseSize = 32;
const uint32_t dataResponseSize = 100;

/***************************************************************************************
 * FIFO Defined Data
****************************************************************************************/
#define FIFOSIZE 100
typedef struct 
{
  uint8_t udpDataOut[100];//数据包
  uint32_t udpDataSize;
  uint16_t Port;
  IPAddress IP;
  /* data */
}UdpPacket;

typedef struct
{
  uint32_t FIFO_Count;
  uint32_t Rear;
  uint32_t Front;
  UdpPacket udppack[FIFOSIZE]; //fifo size;
}UdpFifo;

UdpFifo mUdpFifo;

void Fifoinit()
{
  mUdpFifo.FIFO_Count = 0;
  mUdpFifo.Rear       = 0;
  mUdpFifo.Front      = 0;

}

void FIFO_WritePacket(uint32_t packet_length, uint8_t *packet_Data,  uint16_t Port, IPAddress IP)
{
  // 检查FIFO是否已满
  if(mUdpFifo.FIFO_Count < FIFOSIZE)
  {
    // 复制数据到队列中
    memcpy(mUdpFifo.udppack[mUdpFifo.Rear].udpDataOut, packet_Data, packet_length);
    // 设置数据包大小
    mUdpFifo.udppack[mUdpFifo.Rear].udpDataSize = packet_length;
    mUdpFifo.udppack[mUdpFifo.Rear].Port        = Port;
    mUdpFifo.udppack[mUdpFifo.Rear].IP          = IP;
    // 更新队尾指针
    mUdpFifo.Rear = (mUdpFifo.Rear + 1) % FIFOSIZE;
    // 增加队列中的数据包计数
    mUdpFifo.FIFO_Count++;
  }
  // 如果FIFO已满，不执行任何操作
}

void FIFO_SendPacket()
{
  // 检查FIFO是否为空
  if( mUdpFifo.FIFO_Count != 0)
  {
    if(mUdpFifo.udppack[mUdpFifo.Front].Port == 0)
    {
       // 更新队首指针
      mUdpFifo.Front = (mUdpFifo.Front + 1) % FIFOSIZE;
      // 减少队列中的数据包计数
      mUdpFifo.FIFO_Count--;
      return;
    }
    // 发送队首的数据包
    udp.beginPacket(mUdpFifo.udppack[mUdpFifo.Front].IP, mUdpFifo.udppack[mUdpFifo.Front].Port);
    udp.write(mUdpFifo.udppack[mUdpFifo.Front].udpDataOut, mUdpFifo.udppack[mUdpFifo.Front].udpDataSize);
   // Serial.println("sendone");
    udp.endPacket();
    // 更新队首指针
    mUdpFifo.Front = (mUdpFifo.Front + 1) % FIFOSIZE;
    // 减少队列中的数据包计数
    mUdpFifo.FIFO_Count--;
  }
  // 如果FIFO为空，不执行任何操作
}

/***************************************************************************************
 * User Defined Data
****************************************************************************************/

char wifiSSID[] = "CMCC-b74H";
char wifiPass[] = "nw7kqrm6";
uint16_t udpPort = 26760;

const uint8_t MPU6050_sda = 4, MPU6050_scl = 5; // MPU6050 I2C GPIO connection。scl d1 sda d2
const uint8_t MPU_addr = 0x68; // I2C address of the MPU-6050


int16_t* swapTable[] =
{
  &accZI,
  &accXI,
  &accYI,
  &gyrRI,
  &gyrPI,
  &gyrYI,
};
bool signTable[] = // If true change sign
{
  false, // accXI
  false, // accYI
  true,  // accZI
  false,  // gyrPI
  false, // gyrYI
  false,  // gyrRI
};
int16_t offsetTable[] =
{
  -2818, // accXI
  2461, // accYI
  1148, // accZI
  1, // gyrPI
  1, // gyrYI
  1, // gyrRI
};


float gyrOffYF = 0; // Optional: Set this value to Gyro Y from PadTest when the gamepad is immobile to remove drift


/***************************************************************************************
 * 
****************************************************************************************/

MPU6050 accgyr;
// Gyro sensitivity 0: +/-250 deg/s, 1: +/-500 deg/s, 2: +/-1000 deg/s, 3: +/-2000 deg/s
// If set too low it will introduce clipping
// If set too high it will decrease sensitity
const uint8_t gyroSens = gyrosenstive;
const float gyroLSB = 131.0f / pow(2, gyroSens);

// Info package response packet
uint8_t makeInfoPackage(uint8_t* output, uint8_t portNumber)
{
  // Magic server string
  output[0] = (uint8_t)'D';
  output[1] = (uint8_t)'S';
  output[2] = (uint8_t)'U';
  output[3] = (uint8_t)'S';
  // Protocol version (1001)
  output[4] = 0xE9;
  output[5] = 0x03;
  // Packet length without header plus the length of event type (16)
  output[6] = (uint8_t)(16);
  output[7] = 0;
  // Zero out CRC32 field
  output[8] = 0;
  output[9] = 0;
  output[10] = 0;
  output[11] = 0;
  // Set server id to some value (0)
  output[12] = 0;
  output[13] = 0;
  output[14] = 0;
  output[15] = 0;
  // Event type, controller information (0x00100001)
  output[16] = 0x01;
  output[17] = 0x00;
  output[18] = 0x10;
  output[19] = 0x00;

  if (portNumber == 0) // Controller 0 is the only active controller
  {
    output[20] = 0x00; // Slot of the device we are reporting about (0)
    output[21] = 0x02; // Slot state, connected (2)
    output[22] = 0x02; // Device model, full gyro aka DS4 (2)
    output[23] = 0x02; // Connection type, bluetooth (2). (May be either USB (1) or Bluetooth (2))
    // MAC address of device (0x000000000001)
    output[24] = 0x01; 
    output[25] = 0x00;
    output[26] = 0x00;
    output[27] = 0x00;
    output[28] = 0x00;
    output[29] = 0x00;
    // Battery status, full (5)
    output[30] = 0x05; // ...
    output[31] = 0x00; // Termination byte
  }
  else // Set controllers other than 0 to unconected state  
  {
      output[20] = portNumber;    // Slot of the device we are reporting about (i)
      output[21] = 0x00; // Slot state, not connected (0)
      output[22] = 0x00; // Device model, not applicable (0)
      output[23] = 0x00; // Connection type, not applicable (0)
      // MAC address of device, not applicable (0x000000000000)
      output[24] = 0x00;
      output[25] = 0x00;
      output[26] = 0x00;
      output[27] = 0x00;
      output[28] = 0x00;
      output[29] = 0x00;
      // Battery status, not applicable (0)
      output[30] = 0x00; // ...
      output[31] = 0x00; // Termination byte
  }

  uint32_t Checksum = CRC32::calculate(output, 32);
  memcpy(&output[8], &Checksum, sizeof(Checksum)); // Copy bytes from Checksum to packet array
  
  return 32; // Return the number of bytes in packet
}
// Data package response packet
uint8_t makeDataPackage(uint8_t* output,       uint32_t packetCount,  uint32_t timestamp, 
                        float accellerometerX, float accellerometerY, float accellerometerZ,
                        float gyroscopePit,    float gyroscopeYaw,    float gyroscopeRol)
{
  // Magic server string
  output[0] = (uint8_t)'D';
  output[1] = (uint8_t)'S';
  output[2] = (uint8_t)'U';
  output[3] = (uint8_t)'S';
  // Protocol version (1001)
  output[4] = 0xE9;
  output[5] = 0x03;
  // Packet length without header plus the length of event type (4)
  output[6] = (uint8_t)(80 + 4);
  output[7] = 0;
  // Zero out CRC32 field
  output[8] = 0;
  output[9] = 0;
  output[10] = 0;
  output[11] = 0;
  // Set server id to some value (0)
  output[12] = 0;
  output[13] = 0;
  output[14] = 0;
  output[15] = 0;
  // Event type, controller data (0x00100002)
  output[16] = 0x02;
  output[17] = 0x00;
  output[18] = 0x10;
  output[19] = 0x00;

  output[20] = 0x00; // Slot of the device we are reporting about (0)
  output[21] = 0x02; // Slot state, connected (2)
  output[22] = 0x02; // Device model, full gyro aka DS4 (2)
  output[23] = 0x02; // Connection type, bluetooth (2). (May be either USB (1) or Bluetooth (2))
  // MAC address of device (0x000000000001)
  output[24] = 0x01; 
  output[25] = 0x00;
  output[26] = 0x00;
  output[27] = 0x00;
  output[28] = 0x00;
  output[29] = 0x00;
  // Battery status, full (5)
  output[30] = 0x05; // ...

  output[31] = 0x01; // Device state, active (1)
  memcpy(&output[32], &packetCount, sizeof(packetCount)); // Copy from packetCount to packet array 
  // We don't care about button, joystick and touchpad data, so we just their bytes to zero.
  output[36] = 0x00; // D-Pad Left, D-Pad Down, D-Pad Right, D-Pad Up, Options (?), R3, L3, Share (?)
  output[37] = 0x00; // Y, B, A, X, R1, L1, R2, L2 
  output[38] = 0x00; // PS Button (unused)
  output[39] = 0x00; // Touch Button (unused)
  output[40] = 0x00; // Left stick X (plus rightward)
  output[41] = 0x00; // Left stick Y (plus upward)
  output[42] = 0x00; // Right stick X (plus rightward)
  output[43] = 0x00; // Right stick Y (plus upward)
  output[44] = 0x00; // Analog D-Pad Left
  output[45] = 0x00; // Analog D-Pad Down
  output[46] = 0x00; // Analog D-Pad Right
  output[47] = 0x00; // Analog D-Pad Up
  output[48] = 0x00; // Analog Y
  output[49] = 0x00; // Analog B
  output[50] = 0x00; // Analog A
  output[51] = 0x00; // Analog X
  output[52] = 0x00; // Analog R1
  output[53] = 0x00; // Analog L1
  output[54] = 0x00; // Analog R2
  output[55] = 0x00; // Analog L2
  output[56] = 0x00; // Is first touch active?
  output[57] = 0x00; // First touch id
  output[58] = 0x00; // First touch X (16 bit)
  output[59] = 0x00; //...
  output[60] = 0x00; // First touch Y (16 bit)
  output[61] = 0x00; //...
  output[62] = 0x00; // Is second touch active?
  output[63] = 0x00; // Second touch id
  output[64] = 0x00; // Second touch X (16 bit)
  output[65] = 0x00; //...
  output[66] = 0x00; // Second touch Y (16 bit)
  output[67] = 0x00; //...
  // Clear 4 higher bytes of timestamp and copy 4 lower bytes of timestamp
  output[72] = 0x00; 
  output[73] = 0x00; 
  output[74] = 0x00; 
  output[75] = 0x00;
  memcpy(&output [68], &timestamp, sizeof(timestamp)); // Copy from timestamp to packet array   
  // Move accelerometer and gyroscope data
  memcpy(&output [76], &accellerometerX, sizeof(accellerometerX));
  memcpy(&output [80], &accellerometerY, sizeof(accellerometerY));
  memcpy(&output [84], &accellerometerZ, sizeof(accellerometerZ));
  memcpy(&output [88], &gyroscopePit, sizeof(gyroscopePit));
  memcpy(&output [92], &gyroscopeYaw, sizeof(gyroscopeYaw));
  memcpy(&output [96], &gyroscopeRol, sizeof(gyroscopeRol));

  uint32_t Checksum = CRC32::calculate(output, 100);
  memcpy(&output[8], &Checksum, sizeof(Checksum)); // Copy from Checksum to packet array
  
  return 100; // Return the number of bytes in packet
}

void setup() 
{
  Serial.begin(74880);

  Serial.print("\nConnecting");
  WiFi.begin(wifiSSID, wifiPass);  
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }   
  Serial.print("\nConnected, IP address: ");          
  Serial.print(WiFi.localIP());

  udp.begin(udpPort);
  Serial.print("\nUDP server has been set up at port: ");
  Serial.println(udpPort);  
  
  Serial.println("Initializing MPU6050");        
  Wire.begin(MPU6050_sda, MPU6050_scl); // Connect MPU6050 to GPIO pins defined in MPU6050_sda and MPU6050_scl
  accgyr.initialize();    
  Serial.println(accgyr.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accgyr.setFullScaleGyroRange(gyroSens); // Set selected gyro sensitivity
  accgyr.setXAccelOffset(offsetTable[0]);
  accgyr.setYAccelOffset(offsetTable[1]);
  accgyr.setZAccelOffset(offsetTable[2]);
  accgyr.setXGyroOffset(offsetTable[3]);
  accgyr.setYGyroOffset(offsetTable[4]);
  accgyr.setZGyroOffset(offsetTable[5]);  

  Serial.println("Setup done!");     
}

void loop() 
{
  //1. True control data send
  if ((micros() - u32_Data_SendTime > DATA_SEND_TIME)) // Check if enough time has elapsed between data packets
  {            
    dataPacketNumber++; 
    u32_Data_SendTime = micros();
    
    accgyr.getMotion6(swapTable[0], swapTable[1], swapTable[2], swapTable[3], swapTable[4], swapTable[5]);  

    // For MPU-6050 the biggest possible number is 32768 and smallest is -32767,
    // But because it uses 2's complement 16 bit signed numbers the number 32728 is interpreted as -32728
    // So if we want to prevent the wrap around from positive to negative we need
    // to subtract 1 from the raw data
    accXI--; accYI--; accZI--;
    gyrPI--; gyrYI--; gyrRI--;      
 
    // Inverse selected axis
    accXI *= signTable[0] ? -1 : 1;
    accYI *= signTable[1] ? -1 : 1;
    accZI *= signTable[2] ? -1 : 1;
    gyrPI *= signTable[3] ? -1 : 1;
    gyrYI *= signTable[4] ? -1 : 1;
    gyrRI *= signTable[5] ? -1 : 1;
    
    // Convert raw data to float.
    accXF = accXI / 16384.0f; // Divide by LSB/mg, for 2g sensitivity it is 16384
    accYF = accYI / 16384.0f;
    accZF = accZI / 16384.0f;

    gyrPF = gyrPI / gyroLSB;  // Divide by LSB/deg/s
    gyrYF = gyrYI / gyroLSB;
    gyrRF = gyrRI / gyroLSB;
    
    gyrYF -= gyrOffYF;
    makeDataPackage(&udpDataOut[0], dataPacketNumber, u32_Data_SendTime, accXF, accYF, accZF, gyrPF, gyrYF, gyrRF);
    FIFO_WritePacket(dataResponseSize,udpDataOut,DATA_REPLY_PORT,DATA_REPLY_IP);  
    
  }

  //2. Client msg check
  if(micros() - u32_Recv_ChkTime > RECV_CHECKTIME)
  {
    u32_Recv_ChkTime = micros();
    uint8_t packetInSize = udp.parsePacket();
    if (packetInSize)
    {
      udp.read(udpIn, sizeof(udpIn));
      switch(udpIn[16]) // udpIn[16] - Least significant byte of event type
      {
        case 0x01: // Information about controllers
            for (uint8_t i = 0; i < 4; i++) // udpIn[20] - Amount of ports we should report about
            {
              makeInfoPackage(udpInfoOut, i); // udpIn[24 + i] - Slot numbers to report about
              FIFO_WritePacket(infoResponseSize,udpInfoOut,udp.remotePort(),udp.remoteIP());  
            }
            INFO_REPLY_PORT = udp.remotePort();
            INFO_REPLY_IP   = udp.remoteIP();
        // Serial.println("Got info request!");       
        break;
        case 0x02: // Controller input data
              DATA_REPLY_IP   = udp.remoteIP();
              DATA_REPLY_PORT = udp.remotePort();
        break;      
      }  
    }
  }

  //3. msg fifo send
  if ((micros() - u32_LstFIFO_ChkTime > FIFO_SEND_TIME)) // Check if enough time has elapsed between data packets
  { 
    u32_LstFIFO_ChkTime  = micros();
    FIFO_SendPacket();
  }

}
