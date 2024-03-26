/*This code is made for the imu (mpu6050)*/
/******************************************** Libraries *********************************************/
#include <Wire.h>
#include <math.h>
#include <mcp_can.h>
#include <SPI.h>

/****************************************Macros and constants ***************************************/
#define wheelbase_length 2.25                  // L  ( the distance between the rear wheels axis and the front wheels axis )
#define Cg_fromback 0.9                        // Lr ( the distance from the axis of rear wheels and the center of gravity )
#define width_bet_wheels 1.25                  // Dr ( the distance between the rear wheels )
#define WHEEL_RADIUS 0.3125                    // the radius of the wheels
#define Zero_Ref_Steering_Angle 90             //Zero Refernce of the Steering angle

#define Max_Right_Speed 100                    // the maximum speed of the right raer wheel
#define Max_left_Speed 100                     // the maximum speed of the left raer wheel
#define Max_Speed_Differnce 60                 // the maximum speed Difference between the rear wheels
#define Max_Car_Speed 80                       // the maximum speed of the car
#define left_speed_max_deviation 20            //maximum left wheel speed deviation
#define right_speed_max_deviation 20           //maximum right wheel speed deviation
#define Steeringangle_max_dev_right 180        //maximum steering angle at right
#define Steeringangle_max_dev_left 0           //maximum steering angle at left

#define Pressed HIGH                           //The Button is Preased
#define Relesed LOW                            //The Button is Realesed

#define FAILED 0                               //Test the state of the module saftey
#define SUCCESS 1                              //Test the state of the module saftey
#define INNER false
#define OUTER true


/**************************** For CAN ************************************/
#define CAN0_INT 2                              // Set INT to pin 2
#define id 0x103                                //id to be received
#define sender_id 0x105                         //the id of the arduino u want to send to  
#define MsgReceived_ID1       (0x110)           //the id of the message from Front side which includes (Steering Angle & Throttle)  
#define MsgReceived_ID2       (0x150)           //the id of the message from Steering which includes (The state of the system from the button)    
#define MsgSend_ID1           (0x140)           //the address of our module (Our message ID)
#define Msg_With_Error_Only false               //sending our message with the error bit
#define Msg_With_Velocities_Only true           //sending our message without the error bit
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];                         //because we are recieving only 4 variables
MCP_CAN CAN0(10);                               // Set CS to pin 10
unsigned char sendData[8]={0,0,0,0,0,0,0,0};
unsigned short TempSend=1;
/***variables to control sending time***/
unsigned long previousMillissend = 0;           // Variable to store the last time messege was sent
const long intervalsend = 30;
unsigned long currentMillissend=0;

/************************Some Constants and Variables ******************************/
const int MPU = 0x68;                           // MPU6050 I2C address
float yawRate;
float gyroAngleZ;
float GyroErrorZ;
float right_velocity, left_velocity;
float prev_right_velocity = 0, prev_left_velocity = 0;
float slip_angle;
float carSpeed ;
char  Button_Statue;
float throttle;
float steering_angle ;
char error_checker;

/******************************* Functions Prototypes ********************************************/
void calculate_IMU_error();                                           //Calculates the error of the IMU (MPU6050)
void get_YawRate();                                                   //Calculates the yawrate from the IMU (MPU6050)
void Read_Data();                                                     //Reading Data From the CAN
char Module_Saftey_Check();                                           //Saftey check function for the module
void Send_Data(char error_checker);                                   //Sending the Velocities Using the CAN to the rear wheels 
void fillData(char MessageNumber);                                    //Filling the data into the array

/************************************* Void Setup *******************************************/
void setup() {
  
  Serial.begin(115200);

  /********************* Initialization of I2C for mpu6050  *************************************/
  
  Wire.begin();                                                      // Initialize I2C comunication
  Wire.beginTransmission(MPU);                                       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                                                  // Talk to the register 6B
  Wire.write(0x00);                                                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);                                        //end the transmission

  calculate_IMU_error();                                             //Calculating the Error of the IMU               

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  
  if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");
  pinMode(2, INPUT);
  CAN0.setMode(MCP_NORMAL);                   // Change to normal mode to allow messages to be transmitted
}

/************************************ Void Loop *************************************************/

void loop() {
  
  currentMillissend=millis();
  Read_Data();
  get_YawRate();
  carSpeed =map(throttle,0,1023,0,Max_Car_Speed);
  float slipAngle = atan((Cg_fromback / wheelbase_length) * tan(steering_angle ));
  float Longitudinal_Speed = carSpeed * cos(slipAngle);

  if (steering_angle  >= Zero_Ref_Steering_Angle && steering_angle  < Steeringangle_max_dev_right) // Turning Right
  {
    right_velocity = Longitudinal_Speed - ((width_bet_wheels / 2) * yawRate); // The Right Wheel is The Inner Wheel
    left_velocity = Longitudinal_Speed + ((width_bet_wheels / 2) * yawRate);  // The Left Wheel is The Outer Wheel
  }
  else if (steering_angle  >= Steeringangle_max_dev_left && steering_angle  < Zero_Ref_Steering_Angle)   //Turning Left
  {
    right_velocity = Longitudinal_Speed + ((width_bet_wheels / 2) * yawRate); // The Right Wheel is The Outer Wheel
    left_velocity = Longitudinal_Speed - ((width_bet_wheels / 2) * yawRate);  // The Left Wheel is The Inner Wheel
  } 

  /************The Cancellation of the electronic differential(Emergency)**********************
    happends if either the left velocity or the right velocity has changed abruptly or if the speed differnce of the rear wheels is increased
    or the speed of any of the rear wheels exceeds the maximum speed or the disable electronic differential button is pressed
  */
  if (Module_Saftey_Check() == SUCCESS)
    {  
    Send_Data(SUCCESS);                                         //Sending the two Velocities without sending the error
    Serial.print("the right velocity = ");
    Serial.print(right_velocity);
    Serial.print(" , ");
    Serial.print("the left velocity = ");
    Serial.println(left_velocity);
    
    }
  else
    {
    Serial.print("System is failed 💀💀💀💀💀💀💀💀 ");
    Send_Data(FAILED);                                        //Send the Error using the CAN (Cancell the system (without sending the velocities))
    }
                   
  prev_right_velocity = right_velocity;
  prev_left_velocity = left_velocity;

}
/*********************************************** Function Definations **************************************************************/

void calculate_IMU_error() {
  char c = 0 ;
  // We can call this funtion in the setup section to calculate the gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read gyro values 200 times
  for (c = 0 ; c < 100; c++)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    yawRate = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorZ = GyroErrorZ + (yawRate / 131.0);
  }
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}

void get_YawRate() {
  // === Read gyroscope data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x43);               // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  yawRate = (Wire.read() << 8 | Wire.read()) / 131.0;

  // Correct the outputs with the calculated error values
  yawRate = abs((yawRate + GyroErrorZ) * (PI / 180)); // GyroErrorZ ~ (-0.8)
  Serial.print("Yaw Rate = ");
  Serial.println(yawRate);

}

bool checkI2CData() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);               // Gyro data first register address 0x43
  if (Wire.endTransmission() == 0) { // Check if the transmission was successful
    Wire.requestFrom(MPU, 6, true); // Read 6 bytes of data
    if (Wire.available() >= 6) { // Check if there are enough bytes available
      // Data available, return true
      return true;
    } else {
      // Not enough data available from I2C
      return false;
    }
  } else {
    // Failed to communicate with MPU
    return false;
  }
}

void fillData(char error_state)
{
  switch (error_state)
  {
    case Msg_With_Velocities_Only:        //Message without Error
    TempSend=left_velocity*10;
    sendData[0]=TempSend>>8;              //hold the most signefecnt bits in the first byte (of the left Velocity)
    sendData[1]=TempSend ;                //hold the least signeficant bits in the second byte  (of the left Velocity)
    TempSend=right_velocity*10;
    sendData[2]=TempSend>>8;              //hold the most signefecnt bits in the first byte (of the right velocity)
    sendData[3]=TempSend;                 //hold the least signeficant bits in the second byte  (of the right velocity)
    break;

    case Msg_With_Error_Only:             //Message with Error Only
    sendData[4]= 'E' ;                    //Sending the Error flag
    break;

    default:
    break;
  }
}

void Send_Data(char error_checker) {
/* we need to edit this function */
  if (error_checker == 1){
        //send the message with the Velocities only without the error 
  if (currentMillissend - previousMillissend >= intervalsend) {
    previousMillissend = currentMillissend;  
 
    fillData(Msg_With_Velocities_Only);
    byte sndStat1 = CAN0.sendMsgBuf(MsgSend_ID1, 0, 8, sendData);
    if(sndStat1 == CAN_OK){
    //Serial.println("Message Sent Successfully!");
  } else {
    //Serial.println("Error Sending Message...");
  }
  }
    
    }
   else if (error_checker ==0 ){
    //send the message with the error only without the speed 
    if (currentMillissend - previousMillissend >= intervalsend) 
    {
    previousMillissend = currentMillissend;  
 
    fillData(Msg_With_Error_Only);
    byte sndStat1 = CAN0.sendMsgBuf(MsgSend_ID1, 0, 8, sendData);
    if(sndStat1 == CAN_OK)
    {
    //Serial.println("Message Sent Successfully!");
    } 
  else 
  {
    //Serial.println("Error Sending Message...");
      }
    }
   }
}

void Read_Data() {
  /*read the two messages from the CAN bus*/
  if (!digitalRead(2)) // If pin 2 is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
    switch(rxId)
      {
        case MsgReceived_ID1: //the id of the message from Front side which includes (Steering Angle & Throttle) |Speed at bit 0 |Steering angle at bits 2 &3
        /*knowing the structure of the incoming message i can choose which rxBuf element that has my variable value*/
        throttle=rxBuf[0];
        Serial.print("The Speed from the Throttle = ");
        Serial.println(throttle);
        steering_angle=(float)((rxBuf[2]<<8)|rxBuf[3])/10.0;
        Serial.print("The Steering angle = ");
        Serial.println(steering_angle);
        break;
        
        case MsgReceived_ID2: //the id of the message from Steering which includes (The state of the system from the button) | Differential off at bit 1
        /*knowing the structure of the incoming message i can choose which rxBuf element that has my variable value*/
        Button_Statue=rxBuf[1];
        Serial.print("The Statue of the Button = ");
        Serial.println(Button_Statue);
        break;
        default:
        break;
      }
  }
}

char Module_Saftey_Check() {

  if ( abs(left_velocity - prev_left_velocity) > left_speed_max_deviation ||
       abs(right_velocity - prev_right_velocity) > right_speed_max_deviation ||
       (abs(right_velocity - left_velocity) >= Max_Speed_Differnce) ||
       (right_velocity > Max_Right_Speed) ||
       (left_velocity > Max_left_Speed) || Button_Statue == Pressed || !checkI2CData())
  {
    return FAILED ;
  }
  else
  {
    return SUCCESS ;
  }
}
