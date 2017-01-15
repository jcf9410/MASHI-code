/*
2015/07/17: Add Set PID from data frame
2015/06/11: Turn and go forward issue.
2015/05/19: adding emergency stop!
2014/01/07: velocity reduction

22DEC2014
- Add frame format
- PID Speed motor control

*/
//String readString;
#include <Servo.h> 

# define DEBUG         0x00            // Debug Print.  Set to 00 OR FF to debug
# define LED_PIN 13                      // LED indicated built into the Arduino board
# define LOOPTIME        50         
# define REAL_MAX_SPEED  35.0 //15.0 20.0  //Maximum real speed is 35, that correspond to 3.29km/h. This must be LIMITED.

//MOTOR COMMANDS
# define EMER	0X01 
# define RARM	0X02
# define VELO 0X03
# define SMON 0X04
# define SMOF 0X05
# define PIDC 0X06
# define PIDA 0X07


 // these define the hexadecimal commands for Parallax Motor Controller
 // I'm using the same commands that Parallax prints in their documentation for the 27971 motor controller. 
# define QPOS         0x08            //Query Position
# define QSPD         0x10            //Query Speed
# define CHFA         0x18            //Check for Arrival
# define TRVL         0x20            //Travel Number of Positions
# define CLRP         0x28            //Clear Position
# define SREV         0x30            //Set Orientation as Reversed
# define STXD         0x38            //Set TX Delay
# define SMAX         0x40            //Set Speed Maximum
# define SSRR         0x48            //Set Speed Ramp Rate
# define Left_Wheel   0x01            //Set address for left wheel
# define Right_Wheel  0x02            //Set address for right wheel
# define Both_Wheels  0x00            //Set address for both wheels


//const float w = 0.00897; //para un periodo de 700ms-> w=2*pi/700 = 0.00897
const float pi = 3.1416; 
const float gaitPeriod = 1000; //en ms
float w = 2*pi/gaitPeriod;
Servo servoL;  // create servo object to control a servo 
Servo servoR;  // create servo object to control a servo 

boolean flagEmergency = false;
unsigned long time;
//int incremento = 10;
// velDeseada es un valor entre -100 y +100
int velDeseadaL = 0;
int velDeseadaR = 0;

int velActualL = 0;
int velActualR = 0;

int deltaV = 15;

int velZero = 1500;
int indexL = 0;
int indexR = 0;
float velLRaw;
float velRRaw;

float factorPotencia = 1.0; // Numero entre 0 y 1

boolean giro = false;
boolean socialMotion = false;

///////////////////////////
// PID MOTOR CONSTANTS
//////////////////////////
// Critical Gain Kc = 6
// Time per loop dT = 200
// Oscillation period Pc = 2660 ms
// Only P -> Kp = 0.5 Kc = 3
// PI -> Kp = 0.45 * Kc = 2.7; Ki = 1.2*Kp*dT/ Pc = 1.2 (2.7)(200)/2660 = 0.24
// PD -> Kp = 0.80 * Kc = 4.8; Ki = 0; Kd = Kp * Pc /(8dT) = 4.8 (2660)/(8*200) = 7.98
// PID -> Kp = 0.60 * Kc = 0.6 * 6 = 3.6; Ki = 2KpdT / Pc = 2(3.6)(200)/2660 = 0.54; Kd = Kp * Pc /(8dT) = 3.6 (2660)/(8*200) = 6.0
// Conservative Kp = 1; Ki= 0.05; Kd= 0.25;
// Aggresive Kp = 4; Ki= 0.2; Kd =1;
float Kp = 3.0;
float Ki = 0.2;
float Kd = 1.0;



///////////////////////////
// PID MOTOR L
//////////////////////////
  float pidTermL = 0;                                                           // PID correction
  double errorL=0;                                 
  double lastErrorL=0;     
  double integralL = 0;  
///////////////////////////
// PID MOTOR R
//////////////////////////
  double pidTermR = 0;                                                           // PID correction
  double errorR=0;  
  double lastErrorR=0;      
  double integralR = 0;  
///////////////////////////
// PID TURN
//////////////////////////
///////////////////////////
// PID TURN CONSTANTS
//////////////////////////
// Critical Gain Kc = 2.5
// Time per loop dT = 50
// Oscillation period Pc = 1120 ms
// Only P -> Kp = 0.5 Kc = 1.25
// PI -> Kp = 0.45 * Kc = 0.56; Ki = 1.2*Kp*dT/ Pc = 1.2 (0.56)(200)/1120 = 0.12
// PD -> Kp = 0.80 * Kc = 2.0; Ki = 0; Kd = Kp * Pc /(8dT) = 2.0 (1120)/(8*200) = 1.4
// PID -> Kp = 0.60 * Kc = 0.6 * 2.5 = 1.5; Ki = 2(Kp)dT / Pc = 2(1.5)(200)/1120 = 0.54; Kd = Kp * Pc /(8dT) = 1.5 (1120)/(8*200) = 1.0

float KpTurn = 1.25;//10.0; //5.55;          //Kp = 0.4 setting Kp 
float KiTurn = 0.0;
float KdTurn = 1.4;            //Kd = 1 setting Kd
float pidTermTurn = 0;                                                           // PID correction
int errorTurn=0;  
int lastErrorTurn=0; 
int integralTurn = 0;  

double tolerance = 0.25;


double PWM_val_L = 1500;
double PWM_val_R = 1500;    

double speed_req_L = 0;                            // Set Point)
double speed_req_R = 0;                            // Set Point)
double speed_act_L = 0;                              //actual value
double speed_act_R = 0;  //actual value
double speed_L = 0;
double speed_R = 0;
double fi_L_desire;
double fi_R_desire;

int turnDesired = 0; // Velocity Turn desired 
int turnPwm = 0;
int turnVal= 0;
int turnActual = 0;

//float speed_req_R_float = 0.0;
//float speed_req_L_float = 0.0;

int offset = 100;

unsigned long lastMilli = 0;   

byte inputBuffer[4]; //for incoming messages
int op;
int i = 0;

//positioning
double delta_t_pos;
double t;
double delta_t;
float X;
float Y;
float w_R;
float w_L;
float delta_x;
float delta_y;
float theta;
float delta_theta;
float d;
float L; //distance between wheels
float R; //wheel radius
float v;
float omega;

void setup() {
  pinMode(LED_PIN, OUTPUT);   //Blink the LED so you know the Arduino is resetting
                                 // notice the Serial3 in next line  change for your arduino!
  extern HardwareSerial Serial3; // initial Arduino Mega number 3 serial port, pins 14 and 15 on Mega board.
  Serial3.begin(19200);  // the encoder controller runs at 19200
  Serial.begin(115200);   // this is output serial line monitoring for debugging. use serial monitor if debugging1
  //Serial.begin(57600);   // this is output serial line monitoring for debugging. use serial monitor if debugging1

   Serial.println(" Testing the motors...") ;
  
  Set_Orientation_As_Reversed(Right_Wheel);   // Reverse the right wheel
                                             // this causes left wheel and right wheel to turn in opposite directions
                                             // which is a good thing on a 2-wheeled robot.
  delay (10);

  
  X = 0;
  Y = 0;
  theta = 0;
  t = 0;
  L = 0.4;
  R = 0.075;
  delta_t = 0;

  servoR.attach(6);  //the pin for the servo control, and range if desired
  servoL.attach(7);  //the pin for the servo control, and range if desired

  servoL.writeMicroseconds(1500); //set initial servo position if desired
  servoR.writeMicroseconds(1500); //set initial servo position if desired
  
  delay(100); //??
  
  Serial.println("Motors ready!..."); // so I can keep track of what is loaded
}

void loop() {
  while (Serial.available()) {
    inputBuffer[i] = Serial.read();  //gets one byte from serial buffer
    i++;
//    if (DEBUG){
//      Serial.print("Debug: readString: ");
//      Serial.println(readString);
//    }  //so you can see the captured string 
    delay(2);  //slow looping to allow buffer to fill with next character
  }
  i=0;
  op = inputBuffer[0];
    switch (op) {
    case EMER:
      Serial.println("EMERGENCY STOP!");
      flagEmergency = true;
      break;
    case RARM:
      Serial.println("REARM MOTORS!");
      /////////////////////////////////////
      // INICIALIZE VARIABLES
      ///////////////////////////
      // PID MOTOR L
      //////////////////////////
      pidTermL = 0;                                                           // PID correction
      errorL=0;                                 
      lastErrorL=0;     
      integralL = 0;  
      ///////////////////////////
      // PID MOTOR R
      //////////////////////////
      pidTermR = 0;                                                           // PID correction
      errorR=0;  
      lastErrorR=0;      
      integralR = 0;  
      ///////////////////////////
      // PID TURN
      //////////////////////////
      pidTermTurn = 0;                                                           // PID correction
      errorTurn=0;  
      lastErrorTurn=0; 
      integralTurn = 0;  
      speed_req_L = 0;
      speed_req_R = 0;
      speed_act_L = 0;
      PWM_val_L = 1500;
      speed_req_R = 0;
      speed_act_R = 0;
      speed_L = 0;
      speed_R = 0;
      PWM_val_R = 1500;       
      flagEmergency = false;
      break;
    case VELO:
      Serial.println("VELOCITY...");
      //speed_req_L = (inputBuffer[1] - (float)offset)*REAL_MAX_SPEED/100.0;
      //speed_req_R = (inputBuffer[2] - (float)offset)*REAL_MAX_SPEED/100.0;
      speed_req_L = (float)inputBuffer[1] - (float)offset;
      speed_req_R = (float)inputBuffer[2] - (float)offset;
      if (speed_req_L!=speed_req_R)
        giro = true;
      else
      {
        velActualL = speed_req_L;
        velActualR = speed_req_R;
        giro = false;
      }
      break;
    case SMON:
      Serial.println("Social Motion ON...");      
      socialMotion = true;
      break;
    case SMOF:
      Serial.println("Social Motion OFF");      
      socialMotion = false; 
      break;
    case PIDC:
      Serial.println("PID Conservative...");
      // Conservative Kp = 1; Ki= 0.05; Kd= 0.25;
      Kp = 3.0;
      Ki = 0.2;
      Kd = 1.0;
      // Aggresive Kp = 4; Ki= 0.2; Kd =1;
      break;
    case PIDA:
      Serial.println("PID Aggresive...");
      Kp = 3.5;
      Ki = 0.2;
      Kd = 1.0;
      break;
    default: 
      break;
      // si nada coincide, ejecuta el "default"
      // el "default" es opcional
      //velDeseadaL=0;
      //velDeseadaR=0;
    }
    //readString=""; //empty for next input
  inputBuffer[0] = 0;

  ///////////////////////////
//Positioning system
//////////////////////////

    delta_t_pos = (millis()-t)/1000;
    t = millis();
    speed_act_L = speed_req_L;
    speed_act_R = speed_req_R;
    /*
    speed_act_L = Query_Speed(Left_Wheel);
    speed_act_R = Query_Speed(Right_Wheel);
    */
    delta_x = 0;
    delta_y = 0;
    delta_theta= 0;
    w_R = speed_act_R;
    w_L = speed_act_L;

      //odometry. Verificar uds. de query_speed
      
        d = w_R*L/(w_L-w_R);
        omega = (w_R-w_L)*R/L;
        if (w_R-w_L<0.01){
          v = R*w_R;
        }
        else{
          v = 0.5*R*(w_R-w_L);
        }
      delta_theta = omega*delta_t_pos;
      theta = theta + delta_theta;
      delta_x = v*delta_t_pos*cos(theta);
      delta_y = v*delta_t_pos*sin(theta);
      X = X + delta_x;
      Y = Y + delta_y;
  
  if (flagEmergency){
    servoL.writeMicroseconds(1500); //set initial servo position if desired
    servoR.writeMicroseconds(1500); //set initial servo position if desired
  }
  else if((millis()-lastMilli) >= LOOPTIME)  
  {                                                                                 // enter timed loop
      lastMilli = millis();


      if (!giro && socialMotion){
        time = millis();
        speed_req_L = (velActualL+deltaV*sin(w*time+pi))*factorPotencia;
        speed_req_R = (velActualR+deltaV*sin(w*time))*factorPotencia;
      }

      speed_act_L = Query_Speed(Left_Wheel);
      speed_act_R = Query_Speed(Right_Wheel);
      //turnActual = speed_act_L - speed_act_R;
      PWM_val_L= updatePid_L(PWM_val_L, speed_req_L, speed_req_L);         // compute PWM value Left
      PWM_val_R= updatePid_R(PWM_val_R, speed_req_R, speed_req_R);         // compute PWM value Right
//      turnVal = updatePid_Turn(0, turnDesired, turnActual);         // compute PWM value Right
//      PWM_val_L = PWM_val_L + turnVal;
//      PWM_val_R = PWM_val_R - turnVal;
      if (DEBUG){

//        Serial.print("speed_req_L: ");
//        Serial.print(speed_req_L);
//        Serial.print("; speed_req_R: ");
//        Serial.println(speed_req_R);
//
        Serial.print(" speed_act_L: ") ;
        Serial.print(speed_act_L);  
        Serial.print(" ; speed_act_R: ") ;
        Serial.print(speed_act_R);  
        Serial.println("") ;

        Serial.print("PWM_val_L: ");
        Serial.print(PWM_val_L);
        Serial.print("; PWM_val_R: ");
        Serial.println(PWM_val_R);
      }

      speed_L = speed_req_L*15.9+1500;
      speed_R = speed_req_R*15.9+1500;
 /*
      if(speed_req_L==speed_req_R){
        if (speed_req_L>0){
          Serial.println("arriba!");
          servoL.writeMicroseconds(1600);
          servoR.writeMicroseconds(1600);
        }
        else if (speed_req_L==0){
          Serial.println("quieto!");
          servoL.writeMicroseconds(1500);
          servoR.writeMicroseconds(1500);
        }
        else{
          Serial.println("abajo!");
          servoL.writeMicroseconds(1300);
          servoR.writeMicroseconds(1300);
        }
      }
      else {
        if (speed_req_L>0){
          Serial.println("derecha!");
            servoL.writeMicroseconds(1700);
            servoR.writeMicroseconds(1300);
        }
        else{
          Serial.println("izquierda!");
            servoL.writeMicroseconds(1300);
            servoR.writeMicroseconds(1700);
        }
      }      
      */
       servoL.writeMicroseconds((int)speed_L);
      servoR.writeMicroseconds((int)speed_R);
      //servoL.writeMicroseconds(constrain((int)speed_L,800,2200));
      //servoR.writeMicroseconds(constrain((int)speed_R,800,2200)); //uncomment when using PID
  }
  
      Serial.print("|");
      Serial.flush();
      Serial.print("X:");
      Serial.flush();
      Serial.print(X);
      Serial.flush();
      Serial.print("|");
      Serial.flush();
      Serial.print("Y:");
      Serial.flush();
      Serial.print(Y);
      Serial.flush();
      Serial.print("|");
      Serial.flush();
      Serial.print("T:");
      Serial.flush();
      Serial.print(theta);
      Serial.flush();
      Serial.println("||");
      Serial.flush();

  
//  while (velDeseadaL == velActualL && velDeseadaR == velActualR){
  
}

//int updatePid_Turn(int command, int targetTurn, int currentTurn)   {      // compute PWM value
// 
//  //  error = abs(targetValue) - abs(currentValue);
//    errorTurn = targetTurn - currentTurn;
//    integralTurn = integralTurn + errorTurn;
//    //pidTermTurn = (KpTurn * errorTurn) + (KdTurn * (errorTurn - lastErrorTurn));                           
//    pidTermTurn = (KpTurn * errorTurn) + (KiTurn*integralTurn) + (KdTurn * (errorTurn - lastErrorTurn));                           
//    //pidTermR = Kp * errorR;                           
//  
//    lastErrorTurn = errorTurn;
//    
//    //return constrain(command + int(pidTermTurn), 800, 2200); //MIN AND MAX PWM VALUES
//    return (command + int(pidTermTurn)); //MIN AND MAX PWM VALUES
//}


double updatePid_L(double command, double targetValue, double currentValue)   {      // compute PWM value
    errorL = targetValue - currentValue;
    integralL = integralL + errorL;
    //pidTermR = Kp * errorL;                           
    //pidTermL = (Kp * errorL)  + (Kd * (errorL - lastErrorL));                    
    pidTermL = (Kp * errorL) + (Ki*integralL) + (Kd * (errorL - lastErrorL));                    
  
    lastErrorL = errorL;

//  if (DEBUG){
//    Serial.print(" errorL: ") ;
//    Serial.print(errorL) ;
//    Serial.print(" command: ");
//    Serial.print(command) ;
//    Serial.print(" pidTermL: ");
//    Serial.print(pidTermL) ;
//  }
    
    if (abs(errorL)<tolerance && speed_req_L == 0){
    integralL = 0;
    lastErrorL = 0;
    errorL = 0;
      
      return (1500); 
    }
    else
      return (command + pidTermL); //MIN AND MAX PWM VALUES
}

double updatePid_R(double command, double targetValue, double currentValue)   {      // compute PWM value
      errorR = targetValue - currentValue;
     integralR = integralR + errorR;
    //pidTermR = Kp * errorR;                           
    //  pidTermR = (Kp * errorR) +  (Kd * (errorR - lastErrorR));                                 
    pidTermR = (Kp * errorR) + (Ki*integralR) + (Kd * (errorR - lastErrorR));      
  
    lastErrorR = errorR;

    if (abs(errorR)<tolerance && speed_req_R == 0){
    integralR = 0;
    lastErrorR = 0;
    errorR = 0;
      return (1500); 
    }
    else
      return (command + pidTermR); //MIN AND MAX PWM VALUES
}


int Query_Speed(byte Wheel){
  int Returned_Result; 
   Returned_Result = Tell_Motor( QSPD,  Wheel,  0);   // query speed
   return Returned_Result;
}

void Set_Orientation_As_Reversed (byte Wheel) {
  int Returned_Result;
  Returned_Result = Tell_Motor( SREV,  Wheel,  0);   //  wheel reversing 
}


int Tell_Motor(byte Command, byte Address, int In_Parameter) {
  

  int Number_Bytes_To_Write; 
  int Number_Bytes_To_Read;
  int Number_Bytes_In_Input_Queue;
  byte Output_Buffer[4];
  byte Input_Buffer[4];
  int Returned_Result;
  int Returned_Data;
  
  
  Output_Buffer[0] = 0;
  Output_Buffer[1] = 0;
  Output_Buffer[2] = 0;
  Output_Buffer[3] = 0;
 
  Output_Buffer[0] = Command + Address;
  
 
 //  flush the input buffer
  Serial3.flush();   //empty the input buffer
  
  if (Command == QPOS) {       // Query Position. sends 1 byte Command+wheel. Receive integer
    Number_Bytes_To_Write = 1; // 
    Number_Bytes_To_Read = 2;
     //if (DEBUG) {Serial.print("QPOS "); }
   }
  else if (Command == QSPD) {  // Query Position. sends 1 byte Command+wheel. Receive integer
    Number_Bytes_To_Write = 1; // 
    Number_Bytes_To_Read = 2;  // 
      //if (DEBUG) {Serial.print("QSPD ");} 
    }
  else if (Command == CHFA) {  // Check for Arrival. sends 1 byte Command+wheel, 1 byte Tolerance. Receive 1 byte return (ether 0 or FF)
    Number_Bytes_To_Write = 2; // 
    Number_Bytes_To_Read = 1;  // 
    Output_Buffer[1] = byte(In_Parameter);  // 1 byte parameter - the tolerance
     //if (DEBUG) {Serial.print("CHFA "); }
  }
  else if (Command == TRVL) { // Travel Number of Positions. sends 1 byte Command+wheel, signed integer distance. Receives nothing
    Number_Bytes_To_Write = 3; // 
    Number_Bytes_To_Read = 0;  //
   Output_Buffer[1] = byte(In_Parameter / 256);  // high byte
   Output_Buffer[2] = byte(In_Parameter);        //least significant of 2 bytes
      //if (DEBUG) {Serial.print("TRVL ");}  
  }
  else if (Command == CLRP) {   // clear position.  Command and wheel, nothing returned
    Number_Bytes_To_Write = 1; // 
    Number_Bytes_To_Read = 0;  // 
      //if (DEBUG) {Serial.print("CLRP ");}
  }
  else if (Command == SREV) {  // set orientation as reversed.  Command and wheel, nothing returned
    Number_Bytes_To_Write = 1; // 
    Number_Bytes_To_Read = 0;  //
      //if (DEBUG) {Serial.print("SREV "); }
  }
  else if (Command == STXD) {   // set transmit delay. Command and wheel and 1 byte delay.  Nothing returned
    Number_Bytes_To_Write = 2; // 
    Number_Bytes_To_Read = 0;  //
    Output_Buffer[1] = byte(In_Parameter);  // the transmit delay in units of about 4.3 microseconds
      //if (DEBUG) {Serial.print("STXD ");}
  }
  else if (Command == SMAX) {  // set speed maximum.  Command and wheel, then integer speed sent.  Nothing returned
    Number_Bytes_To_Write = 3; // 
    Number_Bytes_To_Read = 0;  //
    Output_Buffer[1] = byte(In_Parameter / 256);  // high byte of the integer speed
    Output_Buffer[2] = byte(In_Parameter);        //least significant byte of integer speed
      //if (DEBUG) {Serial.print("SMAX ");} 
  }
  else if (Command == SSRR) {  // set speed ramp rate.  command+weel, then 1 byte rate.  Nothing returned
    Number_Bytes_To_Write = 2; // 
    Number_Bytes_To_Read = 0;  //
    Output_Buffer[1] = byte(In_Parameter);  // Acceleration / deceleration for travfel, in units of positions/0.25sec/sec.  power on defaults to 15. 
     //if (DEBUG) {Serial.print("SSRR "); }
  }
  
//  if (DEBUG) {  
//    Serial.print("Tell_Motor Command= ");
//    Serial.print(Command, BIN);
//    
//    Serial.print(" Address= ");
//    Serial.print(Address, BIN);
//    
//    Serial.print(" Parameter= ");
//    Serial.print(In_Parameter, HEX);
//  
//    Serial.print(" Output_Buffer= ");
//    for (int i = 0; i < Number_Bytes_To_Write; i++){
//      Serial.print(Output_Buffer[i], HEX);
//      Serial.print("");
//    }
//    Serial.print(" Send this many bytes= ");
//    Serial.print(Number_Bytes_To_Write, DEC);
//    
//    Serial.print(" Rcve this many bytes= ");
//    Serial.print(Number_Bytes_To_Read, DEC);
//  
//    Serial.println("");
//  }
  
  //  DO THE DEED
  //  SEND THE COMMAND to the PARALLSAX ENCODER/WHEEL MOTOR!!!

  for (int i=0; i<Number_Bytes_To_Write; i++) {
     //dp Serial3.print(Output_Buffer[i],BYTE);
     Serial3.write(Output_Buffer[i]);
  }
  
  
  //   Ok, the command is sent over the serial line.
  delay(100);  // wait a little for things to settle
  
  /// GET THE ECHO and possibly a REPLY
  // (at the start of this big function, we flushed the input buffer)  
  // but even if there is no data from the motor to read,
  // we must read it anyways, since there is the echo of our command
  
  // find the number of bytes that are waiting for us
  Number_Bytes_In_Input_Queue = Serial3.available();  
  
//  if (DEBUG) {
//    Serial.print("Number_Bytes_In_Input_Queue=");  
//    Serial.print(Number_Bytes_In_Input_Queue, DEC);
//    Serial.print(" input bytes are ");
//  }

  // Here, we read the bytes echoed by the encoder/motor and also any extra bytes (like a numeric position)
  for(int i = 0; i<Number_Bytes_In_Input_Queue; i++) {
    Input_Buffer[i] = Serial3.read();   // read each byte that is waiting for us
//          if (DEBUG) {Serial.print(Input_Buffer[i],HEX);  
//          Serial.print(".");}
  
}


   if(Number_Bytes_To_Read == 0) {   // perhaps this is a command which has no data returned, like set speed.
     Returned_Data = 0;              
   }
   
   else if (Number_Bytes_To_Read == 1) {  // or maybe this command returns one byte of information, like set transmit delay
     // the command caused the encoder/motor to return only 1 byte.
     // so return the topmost byte.
     //  Ignore the first bytes in the Input Buffer, which are the echo of the outgoing command
     Returned_Data = Input_Buffer[Number_Bytes_In_Input_Queue-1];     
   }
   
   else if (Number_Bytes_To_Read == 2) {   //  or a command that returns 2 bytes, like query position
     // this command causes the encoder/motor to return 2 bytes.
     // so return only the topmost two bytes.
     //  Ignore the first bytes in the Input Buffer, which are the echo of the outgoing command
     //  I am doing an 8 bit shift by multiplying by 256. I'm Lazy...
     Returned_Data = (256*Input_Buffer[Number_Bytes_In_Input_Queue-2]) +Input_Buffer[Number_Bytes_In_Input_Queue-1];     
   }
   
//       if (DEBUG) {     
//          Serial.print(" Returned Data=");
//          Serial.print(Returned_Data, DEC);
//          Serial.println("");
//       }
      return Returned_Data;
      
}
