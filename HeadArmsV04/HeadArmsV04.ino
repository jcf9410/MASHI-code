/*
2015/12/15: arreglando funciones; ADD EVENT:RHAND
2015/11/15: TEST_HEAD
2015/03/12: Add operations (CONFIG)
2015/03/10: Reading alarm shutdown, max torque, torque enable, present voltage and present temperature from one dynamixel
2015/02/06: Changing pitch values for new mechanism

19DEC: CHANGING FRAME FORMAT
FRAME FORMAT:
0          1         2       3
OPERATION; PosPitch; PosYaw; PosRoll

OPERATION (1BYTE)
    1 HEAD_ZERO
    2 HEAD_POSITION
 
                 Compatibility
 CM900                  O
 OpenCM9.04             O
 
                   Dynamixel Compatibility
                AX    MX      RX    XL-320    Pro
 CM900          O      O      O        O      X
 OpenCM9.04     O      O      O        O      X
 **** OpenCM 485 EXP board is needed to use 4 pin Dynamixel and Pro Series ****
 
 */
 
# define DEBUG         0xFF            // Debug Print.  Set to FF to debug

#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP


/* Control table defines */
#define CCW_Angle_Limit 8
#define GOAL_SPEED 32
#define MAX_TORQUE 14
#define MAX_TORQUE_L 14
#define MAX_TORQUE_H 15
#define ALARM_SHUTDOWN 18
#define TORQUE_ENABLE 24
#define TORQUE_LIMIT 34
#define TORQUE_LIMIT_L 34
#define TORQUE_LIMIT_H 35
#define PRESENT_VOLTAGE 42
#define PRESENT_TEMPERATURE 43

//COMMAND OPERATIONS
# define REST         0x00 
# define ZERO         0x01 
# define POSI         0x02 
# define WAKE         0x03 
# define ARMS         0x04
# define UP_LONG      0x05
# define DOWN_LONG    0x06






Dynamixel Dxl(DXL_BUS_SERIAL1);


//#define MCA01 9 //yaw movement
//#define MCA02 8 //pitch movement
//#define MCA03 2     //roll movement

#define MYAW 9 //yaw movement MOTOR
#define MPITCH 8 //pitch movement
#define MROLL 2 //roll movement
#define MLEFT 4 //left arm movement
#define MRIGHT 2 //right arm movement
#define MSHOULDERLEFT 13 //left arm movement


#define POS_MAX 100 //relative maximun position

#define PITCH_UP 240//545
#define PITCH_ZERO 355//520
#define PITCH_DOWN 450//475

#define YAW_LEFT 570 //max 640
#define YAW_ZERO 500
#define YAW_RIGHT 430 // max 360

#define ROLL_LEFT 560//540
#define ROLL_ZERO 520
#define ROLL_RIGHT 480//480


#define ARM_ZERO 512

#define ARM_LEFT_MAX 200//540
#define ARM_LEFT_MIN 512//480

#define ARM_RIGHT_MAX 800//540
#define ARM_RIGHT_MIN 512//480



byte alarmShutdown, maxTorqueL, maxTorqueH, torqueEnable, presentVoltage, presentTemperature, torqueLimitH, torqueLimitL;
int maxTorque2;
int n; // valor de la seleccion de posicion del motor en ascii char 1 -> ASCII 49; CHAR 2 -> ASCII 50; CHAR 3 -> ASCII 51
int idMotor;
int idOp;
int posPitch;
int posYaw;
int posRoll;
int posLeft;
int posRight;
int Vel;
int difPitch = PITCH_UP - PITCH_DOWN;
int difYaw =  YAW_LEFT - YAW_RIGHT;
int difRoll =  ROLL_LEFT - ROLL_RIGHT;

int difArmLeft = ARM_LEFT_MAX - ARM_LEFT_MIN;
int difArmRight = ARM_RIGHT_MAX - ARM_RIGHT_MIN;

//int goalPosition;
int varMaxSpeed = 50;//50 FOR HEAD SERVOS; 
int varMaxSpeedArm = 150;//
int i;
//MashiHand sensor variables
const int analogInPin = 1; // Analog input pin that the potentiometer
int sensorValue;
float voltage = 0;
float voltage_anterior = 0;
float diferencia=0;
//Moving speed in JOINT MODE 0~1023 (0X3FF) can be used, and the unit is about 0.111rpm.
//If it is set to 0, it means the maximum rpm of the motor is used without controlling the speed.
//If it is 1023, it is about 114rpm.
//For example, if it is set to 300, it is about 33.3 rpm.



void setup() {
  // Initialize the dynamixel bus:
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps  
  Dxl.begin(3);  
  //MOTOR SETTINGS
  Dxl.setPacketType(DXL_PACKET_TYPE1);

  Dxl.writeWord(MPITCH, GOAL_SPEED, varMaxSpeed);
  Dxl.writeWord(MYAW, GOAL_SPEED, varMaxSpeed);
  Dxl.writeWord(MROLL, GOAL_SPEED, varMaxSpeed);
  Dxl.jointMode(MPITCH); //jointMode() is to use position mode
  Dxl.jointMode(MYAW); //jointMode() is to use position mode
  Dxl.jointMode(MROLL); //jointMode() is to use position mode
  
  Dxl.writeWord(MSHOULDERLEFT, CCW_Angle_Limit, 0);

//  Dxl.setPacketType(DXL_PACKET_TYPE2);
  Dxl.writeWord(MLEFT, GOAL_SPEED, varMaxSpeedArm);
  Dxl.writeWord(MRIGHT, GOAL_SPEED, varMaxSpeedArm);
  Dxl.jointMode(MLEFT); //jointMode() is to use position mode
  Dxl.jointMode(MRIGHT); //jointMode() is to use position mode
  //You can attach your serialUSB interrupt
  //or, also detach the interrupt by detachInterrupt(void) method
  SerialUSB.attachInterrupt(usbInterrupt);
  pinMode(BOARD_LED_PIN, OUTPUT);  //toggleLED_Pin_Out
  // Configure the ADC pin
  pinMode(analogInPin, INPUT_ANALOG);
  delay(3000);

  
  
}

//USB max packet data is maximum 64byte, so nCount can not exceeds 64 bytes
void usbInterrupt(byte* buffer, byte nCount){
  if (DEBUG){
    SerialUSB.print("nCount =");
    SerialUSB.print(nCount);
    SerialUSB.print("buffer =");
    SerialUSB.println(buffer[0]);
  }
  // Leo VALOR (siguientes)
  for(unsigned int i=0; i < nCount;i++)  //printf_SerialUSB_Buffer[N]_receive_Data
  {
    switch (i)
    {
      case 0:
        idOp = buffer[i];
        break;
      case 1:
        posPitch = (int)buffer[i]; 
        break;
      case 2:
        posYaw = (int)buffer[i]; 
        break;
      case 3:
        posRoll = (int)buffer[i]; 
        break;
      default:
        SerialUSB.println("default");
    }

  }
    switch (idOp)
    {
      case REST:
        if (DEBUG){SerialUSB.println("HEAD_REST");} 
        Dxl.setPacketType(DXL_PACKET_TYPE1);
        Dxl.setPosition(MPITCH, PITCH_ZERO, varMaxSpeed); //position and speed values
        Dxl.setPosition(MYAW, YAW_ZERO, varMaxSpeed);
        Dxl.setPosition(MROLL, ROLL_ZERO, varMaxSpeed);
        delay(3000);
        Dxl.writeWord(MPITCH, TORQUE_LIMIT, 0); // se ha puesto esta linea para setear la velocidad
        Dxl.writeWord(MYAW, TORQUE_LIMIT, 0); // se ha puesto esta linea para setear la velocidad
        Dxl.writeWord(MROLL, TORQUE_LIMIT, 0); // se ha puesto esta linea para setear la velocidad
        break;
      case ZERO:
        if (DEBUG){SerialUSB.println("HEAD_ZERO");} 
        Dxl.setPacketType(DXL_PACKET_TYPE1);
        Dxl.setPosition(MPITCH, PITCH_ZERO, varMaxSpeed); //position and speed values
        Dxl.setPosition(MYAW, YAW_ZERO, varMaxSpeed);
        Dxl.setPosition(MROLL, ROLL_ZERO, varMaxSpeed);
        break;
      case POSI:
        if (DEBUG){SerialUSB.println("HEAD_POSITION");} 
        posPitch = (posPitch*difPitch)/POS_MAX + PITCH_DOWN; 
        posYaw= (posYaw*difYaw)/POS_MAX + YAW_RIGHT;        
        posRoll= (posRoll*difRoll)/POS_MAX + ROLL_RIGHT;
        Dxl.setPacketType(DXL_PACKET_TYPE1);        
        Dxl.setPosition(MPITCH, posPitch, varMaxSpeed); //position and speed values
        Dxl.setPosition(MYAW, posYaw, varMaxSpeed);
        Dxl.setPosition(MROLL, posRoll, varMaxSpeed);
        break;
      case WAKE:
        if (DEBUG){
          SerialUSB.print("HEAD_WAKE");
        }
        Dxl.setPacketType(DXL_PACKET_TYPE1);
        Dxl.writeWord(MPITCH, GOAL_SPEED, varMaxSpeed); // se ha puesto esta linea para setear la velocidad
        Dxl.writeWord(MYAW, GOAL_SPEED, varMaxSpeed); // se ha puesto esta linea para setear la velocidad
        Dxl.writeWord(MROLL, GOAL_SPEED, varMaxSpeed); // se ha puesto esta linea para setear la velocidad
        Dxl.writeWord(MLEFT, GOAL_SPEED, varMaxSpeedArm); // se ha puesto esta linea para setear la velocidad
        Dxl.writeWord(MRIGHT, GOAL_SPEED, varMaxSpeedArm); // se ha puesto esta linea para setear la velocidad        Dxl.goalPosition(MPITCH, PITCH_DOWN);
        Dxl.goalPosition(MPITCH, PITCH_ZERO);
        Dxl.goalPosition(MYAW, YAW_ZERO);
        Dxl.goalPosition(MROLL, ROLL_ZERO);
        Dxl.goalPosition(MLEFT, ARM_ZERO);
        Dxl.goalPosition(MRIGHT, ARM_ZERO);

        
        Dxl.writeWord(MPITCH, TORQUE_LIMIT, 1023); // torque limit to maximum 1023
        Dxl.writeWord(MYAW, TORQUE_LIMIT, 1023); // torque limit to maximum 1023
        Dxl.writeWord(MROLL, TORQUE_LIMIT, 1023); // torque limit to maximum 1023
        Dxl.writeWord(MLEFT, TORQUE_LIMIT, 1023); // torque limit to maximum 1023
        Dxl.writeWord(MRIGHT, TORQUE_LIMIT, 1023); // torque limit to maximum 1023
        i++;
        break;
      case ARMS:
        if (DEBUG){SerialUSB.println("ARMS...");} 
        posLeft = (posPitch*difArmLeft)/POS_MAX + ARM_LEFT_MIN; 
        posRight = (posYaw*difArmRight)/POS_MAX + ARM_RIGHT_MIN;  
        //Dxl.setPacketType(DXL_PACKET_TYPE2);      
//        Dxl.writeWord(MLEFT, GOAL_SPEED, varMaxSpeedArm); // se ha puesto esta linea para setear la velocidad
//        Dxl.writeWord(MRIGHT, GOAL_SPEED, varMaxSpeedArm); // se ha puesto esta linea para setear la velocidad
//        Dxl.goalPosition(MLEFT, posLeft);
//        Dxl.goalPosition(MRIGHT, posRight);
        
        Dxl.setPosition(MLEFT, posLeft, varMaxSpeedArm);
        Dxl.setPosition(MRIGHT, posRight, varMaxSpeedArm);

        break;
      case UP_LONG:
        Dxl.writeWord(MSHOULDERLEFT, GOAL_SPEED, 1000| 0x400);
        delay(300);
        Dxl.writeWord(MSHOULDERLEFT, GOAL_SPEED, 0);
        break;
      case DOWN_LONG:
        Dxl.writeWord(MSHOULDERLEFT, GOAL_SPEED, 1000);
        delay(300);
        Dxl.writeWord(MSHOULDERLEFT, GOAL_SPEED, 0);
        break;
    }
  
}   
  

void loop() {
  // read the analog in value:
  sensorValue = analogRead(analogInPin);
//  SerialUSB.print("sensorValue: ");
//  SerialUSB.println(sensorValue);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  voltage = sensorValue * (5.0 / 1023.0);
  // print out the value you read:
//  SerialUSB.print("V: ");
//  SerialUSB.println(voltage);
  diferencia = abs(voltage - voltage_anterior);
  if (diferencia > 2.0) {
    toggleLED();
    SerialUSB.print("EVENT:RHAND"); 
  } 
  voltage_anterior = voltage;
  delay(100);      
        if (DEBUG){ 

          presentTemperature = Dxl.readByte(MPITCH, PRESENT_TEMPERATURE);  
          SerialUSB.print("T MPITCH: ");
          SerialUSB.print(presentTemperature);  

          presentTemperature = Dxl.readByte(MYAW, PRESENT_TEMPERATURE);  
          SerialUSB.print("; T MYAW: ");
          SerialUSB.print(presentTemperature);  


          presentTemperature = Dxl.readByte(MROLL, PRESENT_TEMPERATURE);  
          SerialUSB.print("; T MROLL: ");
          SerialUSB.println(presentTemperature);  
    //    alarmShutdown = Dxl.readByte(MPITCH, ALARM_SHUTDOWN);
    //    SerialUSB.print("Alarm shutdown register: ");
    //    SerialUSB.println(alarmShutdown);  
    //  
    //    maxTorqueH = Dxl.readByte(MPITCH, MAX_TORQUE_H);  
    //    maxTorqueL = Dxl.readByte(MPITCH, MAX_TORQUE_L);  
    //    SerialUSB.print("Max Torque H;L: ");
    //    SerialUSB.print(maxTorqueH);  
    //    SerialUSB.print(";");
    //    SerialUSB.print(maxTorqueL);  
    //  
    //    torqueEnable = Dxl.readByte(MPITCH, TORQUE_ENABLE);  
    //    SerialUSB.print("Torque enable: ");
    //    SerialUSB.println(torqueEnable);  
    //  
    //    presentVoltage = Dxl.readByte(MPITCH, PRESENT_VOLTAGE);  
    //    SerialUSB.print("Present voltaje: ");
    //    SerialUSB.println(presentVoltage);  
    //  
    //    presentTemperature = Dxl.readByte(MPITCH, PRESENT_TEMPERATURE);  
    //    SerialUSB.print("MOTOR PITCH Present temperature: ");
    //    SerialUSB.println(presentTemperature);  
    //    presentTemperature = Dxl.readByte(MYAW, PRESENT_TEMPERATURE);  
    //    SerialUSB.print("MOTOR YAW Present temperature: ");
    //    SerialUSB.println(presentTemperature);  
    //    presentTemperature = Dxl.readByte(MROLL, PRESENT_TEMPERATURE);  
    //    SerialUSB.print("ROLL PITCH Present temperature: ");
    //    SerialUSB.println(presentTemperature);  
    //    torqueLimitH = Dxl.readByte(MPITCH, TORQUE_LIMIT_H);  
    //    torqueLimitL = Dxl.readByte(MPITCH, TORQUE_LIMIT_L);  
    //    SerialUSB.print("MOTOR PITCH TorqueLimit H;L: ");
    //    SerialUSB.print(torqueLimitH);  
    //    SerialUSB.print(";");
    //    SerialUSB.println(torqueLimitL);  
        }
  
//    toggleLED();
  
  
//  delay(5000);
}



