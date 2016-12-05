#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04

#define ID_NUM 1

Dynamixel Dxl(DXL_BUS_SERIAL1);

void setup() {
  // Initialize the dynamixel bus:
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps  
  Dxl.begin(3);  
  Dxl.jointMode(ID_NUM); //jointMode() is to use position mode
}

void loop() {
  Dxl.goalPosition(ID_NUM, 0); //ID 1 dynamixel moves to position 1
  delay(1000);
  Dxl.goalPosition(ID_NUM, 500);//ID 1 dynamixel moves to position 1023
  delay(1000);
}



