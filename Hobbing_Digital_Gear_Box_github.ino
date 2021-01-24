/* Code to drive a hobbing machine.  
   
  This code drives a hobbing machine for gear cutting as described in Digital Machinist Article January 2021 by D Plewes 
  It is based on a Teensy3.2 microcontroller and uses a LS7184 quardautre encoder chip. The code can drive a TMC2088 stepper driver directly which provides
  microstepping through two levels MS1 and MS2.  These levels for the driver are set in code.  Here is the code for these levels. The default setting in this code
  is to deliver 1/8 microstepping. 

      TMC2088 Stepper Motor microstepping code..........................................
                      MS2=HIGH  MS2=LOW
         MS1=HIGH        1/16      1/2
         MS1=LOW         1/4       1/8
      ...........................................................................
  This code assume two rotary encoders.  The first is a 20 position mechanical rotary encoder (Bourns PEC12R-4225F-S0024)with a push button.  This is used to select parameters for the hobbing machine.
  The second encoder is a 1024 step/revolution optical encoder (YUMO E6B2-CWZ-3E).  This encoder is attached to the hobbing spindle of the hobbing machine. Output goes to the input to the 
  LS7184 for quadrature decoding. The code then calculates the required step frequency to drive a stepper motor attached to the gear blank spindle. The code also calculates the gear blank diameter. 
  Following definitions: N=number of teeth, PD=pitch diameter (mm or inches) 
    For imperial we use 'diametral pitch'(DP)=N/PD(inches),  PD(inches)PD=N/DP,  blank diameter=(N+2)/DP, tooth depth=2.25/DP (https://www.engineersedge.com/gear_formula.htm)
    For metric we use 'module' (m)=PD(mm)/N,                 PD(mm)=m*N,       blank diameter=PD+2m,    tooth depth=2.25m (https://khkgears.net/new/gear_knowledge/abcs_of_gears-b/basic_gear_terminology_calculation.html)
 
    The code is currently set up for a range of imperial and metric threads in the routine "gear_parameters()" which can be edited for altenate choices. The push button
    is used to step through the selection process and set the system to drive the stepper. When in "prog" mode, data is entered. When in "run" mode the code will deliver stepper pulses.
    The direction of hte stepper motor rotation is set by the parameter "Direction" 
    In order to use this code, it is necessary to add a library to drive the I2C liquidCrystal display. 
    Information on how to set up this library is here: https://learn.adafruit.com/rgb-lcd-shield/using-the-rgb-lcd-shield.
 
 This program is free software. It can be redistributed and/or modified under the terms of the GNU General Public License as published by the Free Software Foundation. 
 This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 PURPOSE.  See the GNU General Public License for more details. You can read a copy of the GNU General Public License at http://www.gnu.org/licenses/. 
 */
 #include <Adafruit_LiquidCrystal.h>
 Adafruit_LiquidCrystal lcd(0);

 static byte pinA = 2;                                // Our first hardware interrupt pin is digital pin 2 for data select rotary encoder to drive the PinA interrupt routine 
 static byte pinB = 3;                                // Our second hardware interrupt pin is digital pin 3 for data select rotary encoder to drive the PinB interrupt routine  
 static byte button_pin=4;                            // the button for the knob rotary encoder push button linevolatile byte aFlag = 0;   
 static byte trig_Z_pin=5;                            // the pin which is connected to the index pin of the encoder.  We are not using this is this routine but still available.
 static byte trig_AB_pin=6;                           // the pin which is triggered by the LS7184 as input stepper pulses to the count interrupt division routine
 static byte direction_in_pin=7;                      // the pin which senses the direction by the LS7184
 static byte direction_out_pin=8;                     // the output pin which sets the direction level
 static byte GPIO1_pin=9;                             // the pin which is triggered by the LS7184 as input stepper pulses to the count interrupt division routine
 static byte GPIO2_pin=10;                            // the pin which is triggered by the LS7184 as input stepper pulses to the count interrupt division routine
 static byte MS2_pin=11;                              // the pin to drive MS2
 static byte MS1_pin=12;                              // the pin to drive MS1        
 static byte stepper_pin=13;                          // the output pin for the stepper pin for the motor driver
 static byte MS1=0;                                   // set value of MS1 (see table above for other settings)
 static byte MS2=0;                                   // set value of MS2 (see table above for other settings)
 volatile byte aFlag = 0;                            // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
 volatile byte bFlag = 0;                            // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
 volatile int encoderPos = 0;                        // this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
 volatile int oldEncPos = 0;                         // stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
 volatile byte reading = 0;                          // somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
 int lower_tooth=6;                                  // defines the smallest number of teeth that can be cut 
 int upper_tooth=160;                                // defines the largest number of teeth that can be cut 
 int hob_encoder_resolution=4096;                    // number of pulses per revolution of the spindle encoder after the LS7184 demodulation
 int spindle_steps_per_rev=16000;                    // the number of motor steps needed to drive the spindle one revolution. Needs to be chosen in conjunction with stepper driver settings, pulley reduction(x2) and worm gear reduction (x10).
 int gear_tooth_number;                              // the desired number of teeth to cut
 byte ButtonState = 1;                               // a parameter for the button push algorithm
 byte oldButtonState=0;                              // a parameter for the button push algorithm
 byte mode_select=1;                                 // a parameter to define the programming versus operation settings, "0" allows the machine to hobb, "1" puts it in programming mode
 volatile long input_counter=0;                      // a parameter for the interrupt to count input pulses
 volatile float factor;                              // the ratio of needs steps/rev for stepper to spindle for each hob rotation, this is calculated in the programme 
 volatile long delivered_stepper_pulses=0;           // number of steps delivered to the lead screw stepper motor
 volatile float Direction;                           // a variable to register direction
 volatile int old_time;                              // a dummy variable 
 volatile float calculated_stepper_pulses=0;         // defines the lower divisor for the initial condition which is turning-normal (divisor 128)
 float rev_in;                                       // debugging parameter to track the number of input rotations to encoder
 float rev_out;                                      // debugging parameter to measure the number of spindle rotations delivered
 float gear_diameter;                                // the gear blank diameter
 float cut_depth;                                    // the depth of the gear cut
 float DP;                                           // the diametral pitch of the hobb cutter in imperial units
 float mod;                                           // the module of the hobb cutter in metric units
 int button_status=0;                                // the button status, this number ranges from 0-->1-->2 to run the hobbing machine (0), innput the number of teeth(1) or input the hobb_pitch (2) and 
 int encoder_counter;


void setup() {
  pinMode(pinA, INPUT_PULLUP);                      // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP);                      // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(button_pin,INPUT_PULLUP);                 // set the button pin as an input
  pinMode(trig_Z_pin,INPUT);                        // enables trig_Z_pin to act as an input from the optical encoder
  pinMode(trig_AB_pin,INPUT);                       // set trig_pin_A&B as an input pin from output of the LS7184 quad decoder
  pinMode(direction_in_pin,INPUT);                  // set the direction_in_in as an input
  pinMode(direction_out_pin,OUTPUT);                // set the direction_out_pin as an output
  pinMode(GPIO1_pin,INPUT);                         // set the GPIO1_pin as an input  
  pinMode(GPIO2_pin,INPUT);                         // set the GPIO1_pin as an input 
  pinMode(MS1_pin,OUTPUT);                          // enable MS2_pin as an output
  pinMode(MS2_pin,OUTPUT);                          // enable MS2_pin as an output
  pinMode(stepper_pin,OUTPUT);                      // set stepper pin as an output pin
  attachInterrupt(2,PinA,RISING);                   // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(3,PinB,RISING);                   // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)
  attachInterrupt(6,count, RISING);                 // enable the interrupt for the hobb encoder driven by the LM7184 encoder quad chip
  Serial.begin(9600);                               // start the serial monitor link
  lcd.begin(20,4);                                  // initiate the LCD driver for a 20x4 pixel display
  gear_tooth_number=lower_tooth;                    //start of with gear number at lowest value
  digitalWrite(MS1_pin,MS1);                        //assigns value of MS1 to the TMC2088  
  digitalWrite(MS2_pin,MS2);                        //assigns value of MS2 to the TMC2088
}


// The next two sections "PinA()" and "PinB()" are interupt routines for the data selection rotary encoder and outputs encoderPos as a variable. 

void PinA(){                
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; // read all eight pin values then strip away all but pinA and pinB's values
  if(reading == B00001100 && aFlag) {           //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00000100) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

void PinB(){
  cli(); //stop interrupts happening before we read pin values
  reading = PIND & 0xC; //read all eight pin values then strip away all but pinA and pinB's values
  if (reading == B00001100 && bFlag) {        //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (reading == B00001000) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

//This section detects the button push  on the data selection rotary encoder

void button_detect()                //detects a button push 
        { 
          ButtonState = digitalRead(button_pin);                    // get the current state of the button
          if (ButtonState == LOW && oldButtonState == HIGH)         // has the button gone high since we last read it?
         { mode_select++;
         encoderPos=1; }                                          // toggle the mode_select parameter 
         if(mode_select>2){mode_select=0;}                          // allows the mode_select parameter to range from 0-->2  
          oldButtonState = ButtonState;                             // a parameter for toggling the button state
         }            

//The next section "gear_parameters" collects the data from the data selection rotary encoder to set up the stepper motor parameters for various desire thread sizes. 

void gear_parameters()                //this defines the factors
{ 
         
         if (mode_select == 0)                                         //mode_select==0 to arm the hobbing machine to cut a gear, which I call "run"   
           {       
                                rev_in=1.0*input_counter/hob_encoder_resolution;
                                rev_out=1.0*delivered_stepper_pulses/spindle_steps_per_rev;
                        
                                
         
         lcd.setCursor(15,0);                                         // set up position on the LCD screen 
         lcd.print(" run ");                                           // display "run" on the LCD screen to show that we are armed to operate the machine for hobbing 
            }
           
  
  if(mode_select==1)                                                  //mode_select==1 for inputing the gear tooth number which calculates the reduction factor      
          {
                      lcd.setCursor(16,0); lcd.print("prog");     // displays "prog" on the LCD screen near the bottom right and the stepper motor is inhibited.
                      if(encoderPos>upper_tooth){gear_tooth_number=upper_tooth;encoderPos=upper_tooth;}
                      if(encoderPos<lower_tooth){gear_tooth_number=lower_tooth;encoderPos=lower_tooth;}
                      if((encoderPos>=lower_tooth)&&(encoderPos<upper_tooth)){gear_tooth_number=encoderPos;}
                      factor=spindle_steps_per_rev*1.0/(gear_tooth_number*hob_encoder_resolution);                 //the factor needed for stepper motor #pulses/rev and encoder resolution
                      lcd.setCursor(0,0);  lcd.print("# Teeth= "); lcd.print(gear_tooth_number);lcd.print("   ");    // record number of desired teeth 
                      lcd.setCursor(0,1); lcd.print("                    ");
                      lcd.setCursor(0,2);lcd.print("                    ");
                      lcd.setCursor(0,3);lcd.print("                    ");
          }
          
  if(mode_select==2)                                                  //mode_select==2 for programming the hobb pitch to calculate the gear blank diameter      
          {
            if(encoderPos>10){encoderPos=10;}
            if(encoderPos<0){encoderPos=0;}
          
          switch(encoderPos) {
                                             case(1):     DP=32.0;     break;     //set gear hobb gear pitch for common imperial and metric gears, DP=diametral pitch, mod=module
                                             case(2):     DP=36.0;     break;
                                             case(3):     DP=48.0;     break;             
                                             case(4):     DP=60.0;     break;
                                             case(5):     DP=60.8;     break;
                                             case(6):     DP=64.0;     break;
                                             case(7):     DP=80.0;     break;
                                             case(8):     DP=105.0;    break;
                                             case(9):     DP=126.0;    break;
                                             case(10):    mod=1.0;     break;     //a single metric pitch expressed in module.
                                             
                               }
                                    if(encoderPos<10)                        //calculates gear blank and cut depth for imperial gears
                                    {
                                    gear_diameter=(gear_tooth_number+2)/(1.0*DP);
                                    cut_depth=2.25/(1.0*DP);
                                    lcd.setCursor(0,1); lcd.print("Dia Pitch= ");lcd.print(DP);lcd.print(" ");                    // print out the hobb data for imperial                 
                                    lcd.setCursor(0,2); lcd.print("Blank dia="); lcd.print(gear_diameter,3);lcd.print(" inch");   // print out the diameter of the blank to be machined (inches)
                                    lcd.setCursor(0,3); lcd.print("Cut depth="); lcd.print(cut_depth,3);lcd.print(" inch");       // print out the total depth of cut needed (inches)
                                    
                                    }                                    
                                    else
                                    {
                                      gear_diameter=(gear_tooth_number+2)*mod;       //calculates gear blank and cut depth for metric gears
                                      cut_depth=2.25*mod;
                                      lcd.setCursor(0,1); lcd.print("Module= ");lcd.print(mod);lcd.print("        ");               // print out the hobb data for imperial   
                                      lcd.setCursor(0,2); lcd.print("Blank dia="); lcd.print(gear_diameter/25.4,3);lcd.print(" inch");  // print out the diameter of the blank to be machined (mm)
                                      lcd.setCursor(0,3); lcd.print("Cut depth="); lcd.print(cut_depth/25.4,3);lcd.print(" inch");      // print out the total depth of cut needed (mm)
                                    }
                                    
                               delivered_stepper_pulses=0;
                               input_counter=0;
          }
                  
      }
             

 // The next section "count()" is an interrupt routine that detects the output from the LS7184 and delivers the stepper pulse output for the desired thread parameter
 
void count()    //this is the interrupt routine for the floating point division algorithm
  {
    Direction=digitalRead(direction_in_pin);
    digitalWrite(direction_out_pin,Direction);
    input_counter++;                                                           // increments a counter for the number of spindle pulses received
    calculated_stepper_pulses=round(factor*input_counter);                     // calculates the required number of stepper pulses which should have occured based on the number spindle pulses (input_counter number)
    if((calculated_stepper_pulses>delivered_stepper_pulses)&&(mode_select==0)) // if the calculated number of pulses is greated than the delivered pulses, we deliver one more stepper pulse only if mode_select is set for lathe (==0)
       {
      digitalWrite(stepper_pin,HIGH);                                          // turns the stepper_pin output pin to HIGH
      delayMicroseconds(5);                                                    // keeps that level HIGH for 10 microseconds
      digitalWrite(stepper_pin,LOW);                                           // turns the stepper_pin output pin to LOW
      delivered_stepper_pulses++;                                              // increment the number of delivered_stepper_pulses by one to reflect the pulse just delivered
      }
   }

// This assembles all the code in a loop.
     
void loop()
{
  button_detect();
  gear_parameters();
}
