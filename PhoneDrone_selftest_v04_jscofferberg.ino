//Realeased under Creative Commons!
//This just a basic demo code...
//By Jordi Munoz & DeadFalkon
//Adapted by John Scoferberg aka Alexander Bulezyuk

//Added GPS port testing
#include <Usb.h>
#include <adk.h>
#include <avr/interrupt.h>

#define COMMAND_TEMPERATURE 0x4   //define needed for the temperature android
#define INPUT_PIN_0 0x0   

#define COMMAND_AIRSPEED 0x7   //define needed for the airspeed android
#define INPUT_PIN_2 0x2

volatile unsigned int Start_Pulse =0,  Stop_Pulse =0,  Pulse_Width =0;

volatile int Counter=0;
volatile byte PPM_Counter=0;
volatile int PWM_RAW[8] = {
  2400,2400,2400,2400,2400,2400,2400,2400};
int All_PWM=1500;

int offset0 = 0, offset1 = 0, offset2 = 0, offset3 = 0;
int loopCount = 0;

int pin_temp = A0, pin_airspeed = A2, tempC, tempK, reading;  // for temperature and airspeed measurements

USB Usb;
ADK adk(&Usb,"Google, Inc.",
             "PhoneDrone",
             "Phone Drone ADK by 3DRobotics",
             "1.0",
             "http://www.falkorichter.de",
             "0000000012345678");

            void setup()
            {
              Serial.begin(115200);
              Serial.print("\r\n\n");    Serial.print("\r\nTest demo start\n");
              
              if (Usb.Init() == -1) {
                Serial.print("\r\nOSCOKIRQ failed to assert");
                while(1); //halt
              }
            
              digitalWrite(2,LOW); //PG5 por esto
              pinMode(2,INPUT);
              
              Init_PWM1();      //OUT2&3  //here we're just setting up our registers ;)
              Init_PWM3();      //OUT6&7  throttle
              Init_PWM5();      //AILE & ELEV  out1&2
              Init_PPM_PWM4();  //OUT4&5 is actually not used for detection of rc output
              Serial.print("\r\nInitialisation of PWM registers done, now move to loop():\n");
            }

void loop()
{
//    Serial.print("Ch0:");    Serial.print(InputCh(0));

    Usb.Task();
    if (adk.isReady()){
        Serial.print("\r\nNow our phone is connected succesfully:\n");
            byte sntmsg[6];
            send_Temperature_2Android();
            
            send_Airspeed_2Android();
        
            receiveMessageFromAndroid();


    loopCount++;
    if (loopCount == 10){
      loopCount = 0;
        int ch0 = InputCh(0),  ch1 = InputCh(1),  ch2 = InputCh(2),  ch3 = InputCh(3);
        
      byte msg[9];
      //now we are sending message to android
      msg[0] = 0x1;    //for the switch checking part in public void run() android
      msg[1] = highByte(ch0);
      msg[2] = lowByte(ch0);
      msg[3] = highByte(ch1);
      msg[4] = lowByte(ch1);
      
      msg[5] = highByte(ch2);
      msg[6] = lowByte(ch2);
      msg[7] = highByte(ch3);
      msg[8] = lowByte(ch3);      
      adk.SndData(9, msg);
    }

     
    OutputCh(0,InputCh(7)+offset0);   
    OutputCh(1,InputCh(6)+offset1);
    OutputCh(2,InputCh(4)+offset2);   
    OutputCh(3,InputCh(5)+offset3);    
  } 
  else {
    OutputCh(0,InputCh(7));
    OutputCh(1,InputCh(6));  
    OutputCh(2,InputCh(4));    //rudd on output 7
    OutputCh(3,InputCh(5));    //throttle on output 8  
  }
//   Serial.print(" ");
//  Serial.print(highByte(InputCh(1)),DEC);
//  Serial.print(" ");
//  Serial.print(lowByte(InputCh(1)),DEC);
  
   //OutputCh(2,InputCh(1));     //out3 controlled by channel 2 = ELEV   
}




void changeAnalogReference(int i)
{
switch(i)
  {
    case 0:
        analogReference(INTERNAL1V1);
    break;
    case 1:
        analogReference(DEFAULT);
    break;
  }
}

void sendMessageToAndroid()
{
  
}

void receiveMessageFromAndroid()
{
        uint8_t received_code;
        byte msg1[3];
        uint16_t len = sizeof(msg1);
        received_code = adk.RcvData(&len, msg1);

      if (len > 0) {
          Serial.print(" msg[0]: ");     Serial.print(msg1[0],DEC);    //msg[0] represent the command that our android is sending
          Serial.print(" msg[1]: ");     Serial.print(msg1[1],DEC);
          Serial.print(" msg[2]: ");     Serial.print(msg1[2],DEC);
          Serial.print(" msg[3]: ");     Serial.print(msg1[3],DEC);

            if (msg1[0] == 0x2) {  //AILERON Message
              offset0 = msg1[1];
              offset0 += msg1[2]*256;
              offset0 = offset0 -1000;
            } 
            
            else if (msg1[0] == 0x3) {    //ELEV 
              offset1 = msg1[1];
              offset1 += msg1[2]*256;
              offset1 = offset1 - 1000;
            } 

            else if (msg1[0] == 0x5) {  //RUDD
              offset2 = msg1[1];
              offset2 += msg1[2]*256;
              offset2 = offset2 - 1000;
            } 
            
            else if (msg1[0] == 0x6) {  //THRO
              offset3 = msg1[1];
              offset3 += msg1[2]*256;
              offset3 = offset3 - 1000;
            } 
        }
    
//        Serial.print(" offset 0: ");      Serial.print(offset0,DEC);
//        Serial.print(" offset 1: ");      Serial.print(offset1,DEC);
//        Serial.print(" offset 2: ");      Serial.print(offset2,DEC);
//        Serial.print(" offset 3: ");      Serial.print(offset3,DEC);
}

void send_Temperature_2Android()
{
            byte sntmsg[6];
            //Here the code for our temperature measurement
            changeAnalogReference(0);  //function call, cause we're working with small output voltages from the LM35 sensor
            reading = analogRead(pin_temp);

            // conversion to Celsius
            tempC = round(reading / 9.31);

            //since we read an erronous temperature out (probably because of some errors inside the LM35) and the error is known => +7 ° C
            tempC = round(tempC - 7.00);
            
            int ConvertedValue = tempC * 10;  //convert for an easier transmission! to non floating point value
                        // sntmsg[0] sntmsg[1] in the setup()
                        sntmsg[0] = COMMAND_TEMPERATURE;
                        sntmsg[1] = INPUT_PIN_0;
                        sntmsg[2] = (byte) (ConvertedValue >> 24);
                        sntmsg[3] = (byte) (ConvertedValue >> 16);
                        sntmsg[4] = (byte) (ConvertedValue >> 8);
                        sntmsg[5] = (byte)  ConvertedValue;
                        adk.SndData(6, sntmsg);
}

void send_Airspeed_2Android()
{
            byte sntmsg[6];
            //Here the code for our airspeed measurement (need to be adapted..)
            changeAnalogReference(1);
            int V_analog = analogRead(pin_airspeed);
            //Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
            float V_out = V_analog * (5.0 / 1023.0);
            //Serial.println("\n\nThe analog voltage output is:");    //Serial.println(V_out);
           
            float V_source=5.0, Pressure, Vfss=3.5, V_in, V_true, rho_0 = 1.225, rho_actual, R_gas =8.31432;
            float Temperature=tempC; //replace this by temperature sensor value
            float P_static = 0;
            Pressure = ((V_out-0.06)/V_source - 0.5) / 0.2;   //the value 0.04 is a guess value token by us right now (because the error provided in datasheet is still to big)
            //Serial.println("The calculated Differential Pressure is:");    Serial.print(Pressure);    Serial.print("[kPa]");
            
            
            V_in = sqrt((2*(Pressure*1000))/rho_0);
            //rho_actual = P_static / (R_gas * Temperature);  //could be used once we have a barometer on board, otherwise our measurements are also good
            //V_true = V_in * sqrt(rho_0/rho_actual);
            //Serial.println("\nThe calculated Airspeed is:");      Serial.print(V_in);    Serial.print("[m/s]");
            
                        int ConvertedValue2 = V_in * 10;  //convert for an easier transmission! to non floating point value
                        // sntmsg[0] sntmsg[1] in the setup()
                        sntmsg[0] = COMMAND_AIRSPEED;
                        sntmsg[1] = INPUT_PIN_2;
                        sntmsg[2] = (byte) (ConvertedValue2 >> 24);
                        sntmsg[3] = (byte) (ConvertedValue2 >> 16);
                        sntmsg[4] = (byte) (ConvertedValue2 >> 8);
                        sntmsg[5] = (byte)  ConvertedValue2;
                        adk.SndData(6, sntmsg);
}
