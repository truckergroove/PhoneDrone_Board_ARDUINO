//Realeased under Creative Commons!
//This just a basic demo code...
//By Jordi Munoz & DeadFalkon
//Adapted by John Scoferberg aka Alexander Bulezyuk

//Added GPS port testing
#include <Usb.h>
#include <adk.h>
#include <avr/interrupt.h>

#define VERSION 0 //Old version =0;

volatile unsigned int Start_Pulse =0,  Stop_Pulse =0,  Pulse_Width =0;

volatile int Counter=0;
volatile byte PPM_Counter=0;
volatile int PWM_RAW[8] = {
  2400,2400,2400,2400,2400,2400,2400,2400};
int All_PWM=1500;


long timer=0;  long timer2=0;
byte Status=0;

int offset0 = 0, offset1 = 0, offset2 = 0, offset3 = 0;

int loopCount = 0;

int pin_temp = A0, pin_airspeed = A2;    // defining the analog inputs for temperature & airspeed
int tempC, tempK, reading;  // for temperature measurements

USB Usb;
ADK adk(&Usb,"Google, Inc.",
             "PhoneDrone",
             "Phone Drone ADK by 3DRobotics",
             "1.0",
             "http://www.falkorichter.de",
             "0000000012345678");


void setup()
{

  /*
  MCUCR = 0x80; //Disable JTAG
   MCUCR = 0x80;
   */

  /*
  //This also works
   bitSet(MCUCR,JTD); 
   bitSet(MCUCR,JTD);
   */

  Serial.begin(115200);
  Serial.print("\r\n\n");    Serial.print("\r\nTest demo start\n");
  
  if (Usb.Init() == -1) {
    Serial.print("\r\nOSCOKIRQ failed to assert");
    while(1); //halt
  }

  digitalWrite(2,LOW); //PG5 por esto
  pinMode(2,INPUT);
  
// Init_PWM1();      //OUT2&3  //here we're just setting up our registers ;)
//  Init_PWM3();      //OUT6&7
  Init_PWM5();      //AILE & ELEV  out1&2
  Init_PPM_PWM4();  //OUT4&5
  Serial.print("\r\nInitialisation of PWM registers done, now move to loop():\n");
}

void loop()
{
  //Here the code for our airspeed measurement (need to be adapted
            int V_analog = analogRead(pin_airspeed);
            //Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
            float V_out = V_analog * (5.0 / 1023.0);
            Serial.println("\n\nThe analog voltage output is:");
            Serial.println(V_out);
           
            float V_source=5.0, Pressure, Vfss=3.5, V_in, V_true, rho_0 = 1.225, rho_actual, R_gas =8.31432;
            float Temperature=0; //replace this by temperature sensor value
            float P_static = 0;
            Pressure = ((V_out-0.06)/V_source - 0.5) / 0.2;   //the value 0.04 is a guess value token by us right now (because the error provided in datasheet is still to big)
            Serial.println("The calculated Differential Pressure is:");
            Serial.print(Pressure);
            Serial.print("[kPa]");
            
            
            V_in = sqrt((2*(Pressure*1000))/rho_0);
            
            rho_actual = P_static / (R_gas * Temperature);
            V_true = V_in * sqrt(rho_0/rho_actual);
            Serial.println("\nThe calculated Airspeed is:");
            Serial.print(V_in);
            Serial.print("[m/s]");
  
  //Here the code for our temperature measurement
            reading = analogRead(pin_temp);
            
            // conversion to Celsius
            tempC = round(reading / 9.31);
            float tempC2 = (1.1 * reading * 100.0)/1024.0;  //convert the analog data to temperature
            //1024 is the resolution of our Arduino ADC (=2^10 = 1024)
            // ((1.1/1024) * reading) provide us the volts on port. 
            // multiplicate with 1000 (for mV) and divide by 10 (because 10mV/C° of the LM35)
            // or simply multiplicate by 100
            
            
            //since we read an erronous temperature out (probably because of some errors inside the LM35) and the error is known => +7 ° C
            tempC = round(tempC - 7.00);
            
            Serial.println((byte)tempC);             //send the data to the computer
            
            // conversion en Kelvin
            tempK = tempC + 273;
                        byte msg_try[1];
                        msg_try[0] = tempC;
                        adk.SndData(1, msg_try);

  
  
  //Printing all values.
  
    Serial.print("Ch0:");    Serial.print(InputCh(0));
    Serial.print(" Ch1:");   Serial.print(InputCh(1));
    Serial.print(" Ch2:");   Serial.print(InputCh(2));
    Serial.print(" Ch3:");   Serial.print(InputCh(3));
    
    
    byte msg[9];
 
     uint8_t received_code;
    Usb.Task();
    if (adk.isReady()){
        Serial.print("\r\nNow our phone is connected succesfully:\n");
        byte msg[3];
        uint16_t len = sizeof(msg);
        received_code = adk.RcvData(&len, msg);

      if (len > 0) {
          Serial.print(" msg[0] ");
          Serial.print(msg[0],DEC);
          Serial.print(" msg[1]: ");
          Serial.print(msg[1],DEC);
          Serial.print(" msg[2]: ");
          Serial.print(msg[2],DEC);
//          Serial.print(" msg[3]: ");
//          Serial.print(msg[3],DEC);
//          Serial.print(" msg[4]: ");
//          Serial.print(msg[4],DEC);

            if (msg[0] == 0x2) {
              offset0 = msg[1];
              offset0 += msg[2]*256;
              offset0 = offset0 -1000;
          
            } 
            else if (msg[0] == 0x3) {
              offset1 = msg[1];
              offset1 += msg[2]*256;
              offset1 = offset1 - 1000;
  
            } 
            else if (msg[0] == 0x4) {
              offset2 = msg[1];
              offset2 += msg[3]*256;
              offset2 = offset2 - 1000;
            } 
      //      else if (msg[0] == 0x5) {  //deze nog aanpassen voor throttle
      //        offset1 = msg[1];
      //        offset1 += msg[4]*256;
      //        offset1 = offset1 - 1000;
      //      } 
    
    }
        Serial.print(" offset 0: ");
        Serial.print(offset0,DEC);
        Serial.print(" offset 1: ");
        Serial.print(offset1,DEC);
        Serial.print(" offset 2: ");
        Serial.print(offset2,DEC);

    loopCount++;
    if (loopCount == 10){
      loopCount = 0;
      int ch0 = InputCh(0);
      int ch1 = InputCh(1);
      int ch2 = InputCh(2);
      int ch3 = InputCh(3);
      msg[0] = 0x1;
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

     
    OutputCh(0,InputCh(0)+offset0);   
    OutputCh(1,InputCh(1)+offset1);
    OutputCh(2,InputCh(2)+offset2);   
    //OutputCh(3,InputCh(3)+offset3);    
  } 
  else {
    OutputCh(0,InputCh(0));
    OutputCh(1,InputCh(1));  
    OutputCh(2,InputCh(2));
//    OutputCh(3,InputCh(3));      
  }
//   Serial.print(" ");
//  Serial.print(highByte(InputCh(1)),DEC);
//  Serial.print(" ");
//  Serial.print(lowByte(InputCh(1)),DEC);
  Serial.println(" ");
  
   OutputCh(2,InputCh(1));      
    
//    Serial.println("");
  
}


