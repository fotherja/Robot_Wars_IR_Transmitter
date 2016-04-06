/* IR Transmitter program.

Code to run on Josh's or my IR transmitters. These can take input from either a Game cube controller or a PPM signal from an RF transmitter trainer port.
Configurable IR transmission frequency and channel. 
    
    To Do:
  - LED Feedback, Low battery detection
  - Allow for transmission of a special packet that sets the PWM output on the GMC. Interrupt based or something 
  - Advanced: Get time division multiplexing working by addition of an IR receiver onto Josh's board.
  - Make rising edge immediate! (like Josh). Use a pin register define instead of digitalWrite/Mode etc
  
  DIP Switched affect:
   - Frequency - Although this isn't a reliable multiplexing method
   - Channel
   - Enable Joystick range control check on startup and store min/max values in EEPROM
   - RF or Gamecube connected
    
    IR PROTOCOL DESCRIPTION:
    - Transmits using Manchester encoding
      # each bit period is 0.8ms but this includes 2 sub-bits of the Manchester code:  
      
              1                        0
      --------                          --------
              |                        |  
              |                        |
              |                        |
               --------        --------
     <-----0.8ms------>        <-----0.8ms------>          
                 
    We send a start bit which is a little different for channel 1 or 2:
    - Channel 1: 200us on, 600us off
    - Channel 2: 600us on, 200us off             
      
    So all in all, we send 21 bits of data: 1 start, 8 angle, 8 speed, 4 data bits.
      
  HARDWARE:
  PPM is fed in on 3 if that's being used
  GC is connected to pin 2 if that's being used.
  - See pin defines for more info   
    
    Gamecube Controls:
    Main Joystick sets Bot direction
    Trigger sliders set speed.
      
    L/R Keypad adjusts for rotation drift, U/D adjusts for speed drift
    Start button - recalibrates angle
      
    C stick does either Speed & L/R Rate if main joystick centered or just speed if main joystick being used
    Both trigger buttons depressed resets trims
      
    B,Y,X Increase PID parameters respectively. This is continuous whilst pressed. If R is pressed at the same time the values are decreased.
    A activates Weapon   
     
    PIN Requirements:
    The pin-out for the Gamecube wires can be found here (Along with the N64 controller pinouts):
    http://svn.navi.cx/misc/trunk/wasabi/devices/cube64/hardware/cube64-basic.pdf
    
    Arduino Digital I/O 2: Gamecube controller serial line pulled high to 3.3v via a 1K resistor
    Simples 
     
  To use, hook up the following to the Arduino:
  Digital I/O 2: Gamecube controller serial line
  Digital I/O 8: N64 serial line
  All appropriate grounding and power lines
  A 1K resistor to bridge digital I/O 2 and the 3.3V supply
  
  The pin-out for the N64 and Gamecube wires can be found here:
  http://svn.navi.cx/misc/trunk/wasabi/devices/cube64/hardware/cube64-basic.pdf
  Note: that diagram is not for this project, but for a similar project which
  uses a PIC microcontroller. However, the diagram does describe the pinouts
  of the Gamecube and N64 wires.
   
                          white   yellow   brown
                          blue    NC       red  
 */ 
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#####################################################################################################################################################################
//-----Includes--------------------------------------------------------------------------------------------------------------------------------------------------------
#include <math.h>
#include "crc_table.h"
#include "pins_arduino.h"
#include <Arduino.h>
#include <EEPROM.h>

//-----Constants---------------------------------------------
const int           BIT_PERIOD              = 800;      
const unsigned int  SIGNAL_PERIOD           = 40000;

const int           START_BUTTON            = 4;
const int           Y_BUTTON                = 3;
const int           X_BUTTON                = 2;
const int           B_BUTTON                = 1;
const int           A_BUTTON                = 0;

const int           L_BUTTON                = 6;
const int           R_BUTTON                = 5;
const int           Z_BUTTON                = 4;
const int           U_ARROW_BUTTON          = 3;
const int           D_ARROW_BUTTON          = 2;
const int           R_ARROW_BUTTON          = 1;
const int           L_ARROW_BUTTON          = 0;

#define             SCLAE_FACTOR              0.75

//-----Pin Defines------------------------------------------
#define     PPM_Pin     3                                                               // If PPM input pin
#define     IR_Pin      10                                                              // IR output pin                              

#define     DIP_1       6                                                               // On - RF Transmitter, Off - Game Cube
#define     DIP_2       7                                                               // On - Perform RF Joystick range check test on startup
#define     DIP_3       8                                                               // On - 56KHz, Off - 38KHz
#define     DIP_4       9                                                               // On - Channel 2, Off - Channel 1

#define     LED_RED     12
#define     LED_GREEN   A3

// these two macros set Arduino pin 2 to input or output, which with an
// external 1K pull-up resistor to the 3.3V rail, is like pulling it high or
// low.  These operations translate to 1 op code, which takes 2 cycles
#define       GC_PIN            2
#define       GC_PIN_DIR        DDRD

#define       GC_HIGH DDRD      &= ~0x04
#define       GC_LOW DDRD       |= 0x04
#define       GC_QUERY          (PIND & 0x04)

//-----Globals----------------------------------------------
// 8 bytes of data that we get from the controller. This is a global variable (not a struct definition)
static struct {    
    unsigned char data1;                                                                // bits: 0, 0, 0, start, y, x, b, a    
    unsigned char data2;                                                                // bits: 1, L, R, Z, Dup, Ddown, Dright, Dleft
    unsigned char stick_x;
    unsigned char stick_y;
    unsigned char cstick_x;
    unsigned char cstick_y;
    unsigned char left;
    unsigned char right;
} gc_status;

// Zero points for the GC controller stick
static unsigned char zero_x, czero_x;
static unsigned char zero_y, czero_y;

// Store for PPM Pulsewidths
static struct {
  volatile int Ch[6]; 
  int Ch_Max[6];
  int Ch_Min[6];
} PPM_Channel; 

//-----Functions--------------------------------------------
char Transmit(unsigned long Data, char Bits, int Bit_Period, long Signal_Period, char Channel);
static void gc_send(unsigned char *buffer, char length);
static int gc_get();
static void init_gc_controller();
static void print_gc_status();
void Configure_PPM_Input();
void UsePPM();
void Interrupt_Fxn();
void Print_PPM_Channel_Values();

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#####################################################################################################################################################################
//-----Program---------------------------------------------------------------------------------------------------------------------------------------------------------
void setup()
{
  // Start Serial port
  Serial.begin(115200);  
  Serial.println("Power On");
    
  // Configure Pins
  pinMode(IR_Pin, INPUT);    

  digitalWrite(GC_PIN, LOW);                                                            // Communication with game cube controller on this pin
  pinMode(GC_PIN, INPUT);                                                               // Don't remove these lines, we don't want to push +5V to the controller

  pinMode(DIP_1, INPUT_PULLUP); pinMode(DIP_2, INPUT_PULLUP);
  pinMode(DIP_3, INPUT_PULLUP); pinMode(DIP_4, INPUT_PULLUP);
  
  pinMode(LED_GREEN, OUTPUT); pinMode(LED_RED, OUTPUT); 
  
  // Configure timer to output 38KHz or 56KHz IR pin depending on DIP_3
  if(digitalRead(DIP_3) == 1)  {
  ICR1 = 210;                                                                           // 38KHz - Freq = Clk / (2 * (ICR1 + 1)   
  Serial.println("Using 38KHz...");
  }  
  else  {
  ICR1 = 142;                                                                           // 56KHz  
  Serial.println("Using 56KHz...");
  }    
  TCCR1A = 0b00010000;
  TCCR1B = 0b00011001;  
  
  // Depending on DIP_1 perform GC or RF control
  if(digitalRead(DIP_1) == 0) { 
    Serial.println("Using PPM Control...");      
    Configure_PPM_Input();                       
    UsePPM();
  }     

  // From here on it's GC control
  Serial.println("Using Game Cube Control...");
  noInterrupts();
  init_gc_controller();

  do {
      // Query for the game cube controller's status. We do this
      // to get the 0 point for the control stick.
      unsigned char command[] = {0x40, 0x03, 0x00};
      gc_send(command, 3);
      // read in data and dump it to gc_raw_dump
      gc_get();
      interrupts();
      zero_x = gc_status.stick_x;
      zero_y = gc_status.stick_y;
      czero_x = gc_status.cstick_x;
      czero_y = gc_status.cstick_y;
      Serial.print("GC zero point read: ");
      Serial.print(zero_x, DEC);
      Serial.print(", ");
      Serial.println(zero_y, DEC);
      Serial.flush();
      
      // some crappy/broken controllers seem to give bad readings
      // occasionally. This is a cheap hack to keep reading the
      // controller until we get a reading that is less erroneous.
  } while (zero_x == 0 || zero_y == 0);  
}

static void init_gc_controller()
{
  // Initialize the Gamecube controller by sending it a null byte.
  // This is unnecessary for a standard controller, but is required for the
  // Wave bird.
  unsigned char initialize = 0x00;
  gc_send(&initialize, 1);

  // Stupid routine to wait for the Gamecube controller to stop
  // sending its response. We don't care what it is, but we
  // can't start asking for status if it's still responding
  int x;
  for (x=0; x<64; x++) {
      // make sure the line is idle for 64 iterations, should
      // be plenty.
      if (!GC_QUERY)
          x = 0;
  }
}


/**
 * This sends the given byte sequence to the controller
 * length must be at least 1
 * hardcoded for Arduino DIO 2 and external pull-up resistor
 */
static void gc_send(unsigned char *buffer, char length)
{
    asm volatile (
            "; Start of gc_send assembly\n"

            // passed in to this block are:
            // the Z register (r31:r30) is the buffer pointer
            // %[length] is the register holding the length of the buffer in bytes

            // Instruction cycles are noted in parentheses
            // branch instructions have two values, one if the branch isn't
            // taken and one if it is

            // r25 will be the current buffer byte loaded from memory
            // r26 will be the bit counter for the current byte. when this
            // reaches 0, we need to decrement the length counter, load
            // the next buffer byte, and loop. (if the length counter becomes
            // 0, that's our exit condition)
            
            "ld r25, Z\n" // load the first byte

            // This label starts the outer loop, which sends a single byte
            ".L%=_byte_loop:\n"
            "ldi r26,lo8(8)\n" // (1)

            // This label starts the inner loop, which sends a single bit
            ".L%=_bit_loop:\n"
            "sbi 0xa,2\n" // (2) pull the line low

            // line needs to stay low for 1µs for a 1 bit, 3µs for a 0 bit
            // this block figures out if the next bit is a 0 or a 1
            // the strategy here is to shift the register left, then test and
            // branch on the carry flag
            "lsl r25\n" // (1) shift left. MSB goes into carry bit of status reg
            "brcc .L%=_zero_bit\n" // (1/2) branch if carry is cleared

            
            // this block is the timing for a 1 bit (1µs low, 3µs high)
            // Stay low for 16 - 2 (above lsl,brcc) - 2 (below cbi) = 12 cycles
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\n" // (2)
            "cbi 0xa,2\n" // (2) set the line high again
            // Now stay high for 2µs of the 3µs to sync up with the branch below
            // 2*16 - 2 (for the rjmp) = 30 cycles
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "rjmp .L%=_finish_bit\n" // (2)


            // this block is the timing for a 0 bit (3µs low, 1µs high)
            // Need to go high in 3*16 - 3 (above lsl,brcc) - 2 (below cbi) = 43 cycles
            ".L%=_zero_bit:\n"
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\n" // (3)
            "cbi 0xa,2\n" // (2) set the line high again


            // The two branches meet up here.
            // We are now *exactly* 3µs into the sending of a bit, and the line
            // is high again. We have 1µs to do the looping and iteration
            // logic.
            ".L%=_finish_bit:\n"
            "subi r26,1\n" // (1) subtract 1 from our bit counter
            "breq .L%=_load_next_byte\n" // (1/2) branch if we've sent all the bits of this byte

            // At this point, we have more bits to send in this byte, but the
            // line must remain high for another 1µs (minus the above
            // instructions and the jump below and the sbi instruction at the
            // top of the loop)
            // 16 - 2(above) - 2 (rjmp below) - 2 (sbi after jump) = 10
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "rjmp .L%=_bit_loop\n"


            // This block starts 3 cycles into the last 1µs of the line being high
            // We need to decrement the byte counter. If it's 0, that's our exit condition.
            // If not we need to load the next byte and go to the top of the byte loop
            ".L%=_load_next_byte:\n"
            "subi %[length], 1\n" // (1)
            "breq .L%=_loop_exit\n" // (1/2) if the byte counter is 0, exit
            "adiw r30,1\n" // (2) increment byte pointer
            "ld r25, Z\n" // (2) load the next byte
            // delay block:
            // needs to go high after 1µs or 16 cycles
            // 16 - 9 (above) - 2 (the jump itself) - 3 (after jump) = 2
            "nop\nnop\n" // (2)
            "rjmp .L%=_byte_loop\n" // (2)


            // Loop exit
            ".L%=_loop_exit:\n"

            // final task: send the stop bit, which is a 1 (1µs low 3µs high)
            // the line goes low in:
            // 16 - 6 (above since line went high) - 2 (sbi instruction below) = 8 cycles
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\n" // (3)
            "sbi 0xa,2\n" // (2) pull the line low
            // stay low for 1µs
            // 16 - 2 (below cbi) = 14
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\nnop\n" // (5)
            "nop\nnop\nnop\nnop\n" // (4)
            "cbi 0xa,2\n" // (2) set the line high again

            // just stay high. no need to wait 3µs before returning

            :
            // outputs:
            "+z" (buffer) // (read and write)
            :
            // inputs:
            [length] "r" (length)
            :
            // clobbers:
                "r25", "r26"
            );

}

// Read 8 bytes from the gamecube controller
// hardwired to read from Arduino DIO2 with external pullup resistor
static int gc_get()
{
    // listen for the expected 8 bytes of data back from the controller and
    // and pack it into the gc_status struct.
    asm volatile (";Starting to listen");

    // treat the 8 byte struct gc_status as a raw char array.
    unsigned char *bitbin = (unsigned char*) &gc_status;

    unsigned char retval;

    asm volatile (
            "; START OF MANUAL ASSEMBLY BLOCK\n"
            // r25 is our bit counter. We read 64 bits and increment the byte
            // pointer every 8 bits
            "ldi r25,lo8(0)\n"
            // read in the first byte of the gc_status struct
            "ld r23,Z\n"
            // default exit value is 1 (success)
            "ldi %[retval],lo8(1)\n"

            // Top of the main read loop label
            "L%=_read_loop:\n"

            // This first spinloop waits for the line to go low. It loops 64
            // times before it gives up and returns
            "ldi r24,lo8(64)\n" // r24 is the timeout counter
            "L%=_1:\n"
            "sbis 0x9,2\n" // reg 9 bit 2 is PIND2, or arduino I/O 2
            "rjmp L%=_2\n" // line is low. jump to below
            // the following happens if the line is still high
            "subi r24,lo8(1)\n"
            "brne L%=_1\n" // loop if the counter isn't 0
            // timeout? set output to 0 indicating failure and jump to
            // the end
            "ldi %[retval],lo8(0)\n"
            "rjmp L%=_exit\n"
            "L%=_2:\n"

            // Next block. The line has just gone low. Wait approx 2µs
            // each cycle is 1/16 µs on a 16Mhz processor
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"
            "nop\nnop\nnop\nnop\nnop\n"

            // This block left shifts the current gc_status byte in r23,
            // and adds the current line state as the LSB
            "lsl r23\n" // left shift
            "sbic 0x9,2\n" // read PIND2
            "sbr r23,lo8(1)\n" // set bit 1 in r23 if PIND2 is high
            "st Z,r23\n" // save r23 back to memory. We technically only have
            // to do this every 8 bits but this simplifies the branches below

            // This block increments the bitcount(r25). If bitcount is 64, exit
            // with success. If bitcount is a multiple of 8, then increment Z
            // and load the next byte.
            "subi r25,lo8(-1)\n" // increment bitcount
            "cpi r25,lo8(64)\n" // == 64?
            "breq L%=_exit\n" // jump to exit
            "mov r24,r25\n" // copy bitcounter(r25) to r24 for tmp
            "andi r24,lo8(7)\n" // get lower 3 bits
            "brne L%=_3\n" // branch if not 0 (is not divisble by 8)
            "adiw r30,1\n" // if divisible by 8, increment pointer
            "ld r23,Z\n" // ...and load the new byte into r23
            "L%=_3:\n"

            // This next block waits for the line to go high again. again, it
            // sets a timeout counter of 64 iterations
            "ldi r24,lo8(64)\n" // r24 is the timeout counter
            "L%=_4:\n"
            "sbic 0x9,2\n" // checks PIND2
            "rjmp L%=_read_loop\n" // line is high. ready for next loop
            // the following happens if the line is still low
            "subi r24,lo8(1)\n"
            "brne L%=_4\n" // loop if the counter isn't 0
            // timeout? set output to 0 indicating failure and fall through to
            // the end
            "ldi %[retval],lo8(0)\n"


            "L%=_exit:\n"
            ";END OF MANUAL ASSEMBLY BLOCK\n"
            // ----------
            // outputs:
            : [retval] "=r" (retval),
            // About the bitbin pointer: The "z" constraint tells the
            // compiler to put the pointer in the Z register pair (r31:r30)
            // The + tells the compiler that we are both reading and writing
            // this pointer. This is important because otherwise it will
            // allocate the same register for retval (r30).
            "+z" (bitbin)
            // clobbers (registers we use in the assembly for the compiler to
            // avoid):
            :: "r25", "r24", "r23"
            );

    return retval;
}

static void print_gc_status()
{
    Serial.println();
    Serial.print("Start: ");
    Serial.println(gc_status.data1 & 0x10 ? 1:0);

    Serial.print("Y:     ");
    Serial.println(gc_status.data1 & 0x08 ? 1:0);

    Serial.print("X:     ");
    Serial.println(gc_status.data1 & 0x04 ? 1:0);

    Serial.print("B:     ");
    Serial.println(gc_status.data1 & 0x02 ? 1:0);

    Serial.print("A:     ");
    Serial.println(gc_status.data1 & 0x01 ? 1:0);

    Serial.print("L:     ");
    Serial.println(gc_status.data2 & 0x40 ? 1:0);
    Serial.print("R:     ");
    Serial.println(gc_status.data2 & 0x20 ? 1:0);
    Serial.print("Z:     ");
    Serial.println(gc_status.data2 & 0x10 ? 1:0);

    Serial.print("Dup:   ");
    Serial.println(gc_status.data2 & 0x08 ? 1:0);
    Serial.print("Ddown: ");
    Serial.println(gc_status.data2 & 0x04 ? 1:0);
    Serial.print("Dright:");
    Serial.println(gc_status.data2 & 0x02 ? 1:0);
    Serial.print("Dleft: ");
    Serial.println(gc_status.data2 & 0x01 ? 1:0);

    Serial.print("Stick X:");
    Serial.println(gc_status.stick_x, DEC);
    Serial.print("Stick Y:");
    Serial.println(gc_status.stick_y, DEC);

    Serial.print("cStick X:");
    Serial.println(gc_status.cstick_x, DEC);
    Serial.print("cStick Y:");
    Serial.println(gc_status.cstick_y, DEC);

    Serial.print("L:     ");
    Serial.println(gc_status.left, DEC);
    Serial.print("R:     ");
    Serial.println(gc_status.right, DEC);
    Serial.flush();
}

static bool rumble = false;
void loop()
{
    int status;
    unsigned char data, addr;

    // clear out incoming raw data buffer
    // this should be unnecessary
    //memset(gc_raw_dump, 0, sizeof(gc_raw_dump));
    //memset(n64_raw_dump, 0, sizeof(n64_raw_dump));

    // Command to send to the Gamecube
    // The last bit is rumble, flip it to rumble
    unsigned char command[] = {0x40, 0x03, 0x00};
    if (rumble) {
        command[2] = 0x01;
    }

    // turn on the led, so we can visually see things are happening
    digitalWrite(13, LOW);
    // don't want interrupts getting in the way
    noInterrupts();
    // send those 3 bytes
    gc_send(command, 3);
    // read in data and dump it to gc_raw_dump
    status = gc_get();
    // end of time sensitive code
    interrupts();
    digitalWrite(13, HIGH);

    if (status == 0) {
        // problem with getting the Gamecube controller status. Maybe it's unplugged?
        // set a neutral N64 string
        Serial.print(millis(), DEC);
        Serial.println(" | GC controller read error. Trying to re-initialize");
        Serial.flush();       
        memset(&gc_status, 0, sizeof(gc_status));
        gc_status.stick_x = zero_x;
        gc_status.stick_y = zero_y;
        // this may not work if the controller isn't plugged in, but if it
        // fails we'll try again next loop
        noInterrupts();
        init_gc_controller();
        interrupts();
    }

//-----IR Transmit Code---------------------------------------------------------------
/*       
 *      Having got the GC controller data we now process it before sending it over IR. This involves:  
 *        - Calculating an angle vector from the joystick
 *        - Re-zeroing this angle vector when the start button is pressed
 *        - Accounting for drift by using the L/R arrow keys.
 *        - If Joystick ~centered then ignore Joystick angle since these will be erratic
 *                
 *        - Encoding the trigger sliders as a speed scalar
 *        - Encoding the A button as a weapon activation button.
 *        DATA_1 bits: 0, 0, 0, start, y, x, b, a
 *        DATA_2 bits: 1, L, R, Z, Dup, Ddown, Dright, Dleft
 */
  char PID_Adjust_Buttons;
 
  static float Yaw_Setpoint, Yaw_Setpoint_CStick, Drift_Compensation;
  static float JoyStick_Angle, JoyStick_Sq_Magnitude;
   
  static char Start_Button_Flag;
  
  float Centred_Stick_x = (float)gc_status.stick_x - (float)zero_x;
  float Centred_Stick_y = (float)gc_status.stick_y - (float)zero_y;
  float Centred_CStick_x = (float)gc_status.cstick_x - (float)czero_x;
  float Centred_CStick_y = (float)gc_status.cstick_y - (float)czero_y; 
//---
  static float Speed_Offset = 0.0;
  static float cStick_Contribution;
//---
  unsigned char Yaw_Setpoint_Tx, Speed_Tx; 
  unsigned long IR_Tx_Data; 
    
// ---Button stuff-----------------------------------------------------------------------  
  // If both triggers are pressed reset the trims
  if(bitRead(gc_status.data2, L_BUTTON) && bitRead(gc_status.data2, R_BUTTON)) {  
    Speed_Offset = 0.0;
    Drift_Compensation = 0.0;
  }     

  // For PID tuning on the fly and spinner activation. 
  PID_Adjust_Buttons = 0;
  if(bitRead(gc_status.data1, B_BUTTON))
    PID_Adjust_Buttons = 0b11;
  else if(bitRead(gc_status.data1, Y_BUTTON))
    PID_Adjust_Buttons = 0b01;
  else if(bitRead(gc_status.data1, X_BUTTON))
    PID_Adjust_Buttons = 0b10;    
  if(bitRead(gc_status.data2, Z_BUTTON))
    PID_Adjust_Buttons += 0b100;

  // If L/R Buttons are pressed adjust the drift compensation. If U/B Buttons are pressed adjust the drift compensation. 
  if(bitRead(gc_status.data2, L_ARROW_BUTTON))  
    Drift_Compensation -= 0.003;            
  if(bitRead(gc_status.data2, R_ARROW_BUTTON))  
    Drift_Compensation += 0.003; 
  if(bitRead(gc_status.data2, D_ARROW_BUTTON))  
    Speed_Offset += 0.1;            
  if(bitRead(gc_status.data2, U_ARROW_BUTTON))  
    Speed_Offset -= 0.1;    

  // If the start button is pressed re-zero the Joystick 
  if(bitRead(gc_status.data1, START_BUTTON) && !Start_Button_Flag)
  {
    Yaw_Setpoint += JoyStick_Angle;
    Start_Button_Flag = 1;
  }
  else if(!bitRead(gc_status.data1, START_BUTTON)) 
    Start_Button_Flag = 0;           

// ---Angle stuff-----------------------------------------------------------------------
  JoyStick_Sq_Magnitude = pow(Centred_Stick_y, 2.0) + pow(Centred_Stick_x, 2.0);
  Yaw_Setpoint += Drift_Compensation;
         
  if(JoyStick_Sq_Magnitude > 600.0)
  {
    Yaw_Setpoint -= JoyStick_Angle;                                                     // Remove old values
    Yaw_Setpoint -= Yaw_Setpoint_CStick;
    Yaw_Setpoint_CStick = 0.0;
    JoyStick_Angle = atan2(Centred_Stick_x, Centred_Stick_y) * (180/PI);                // Calculate new value
    Yaw_Setpoint += JoyStick_Angle;                                                     // Add it in

    if(abs(Centred_CStick_x) >= 5 || abs(Centred_CStick_y) >= 5)  
      cStick_Contribution = Centred_CStick_y * 2.0;
    else
      cStick_Contribution = 0.0;   
  }
  else if(abs(Centred_CStick_x) >= 5 || abs(Centred_CStick_y) >= 5) {
    cStick_Contribution = Centred_CStick_y * 2.0;

    Yaw_Setpoint -= Yaw_Setpoint_CStick;
    Yaw_Setpoint_CStick += Centred_CStick_x * 0.2;    
    Yaw_Setpoint += Yaw_Setpoint_CStick;            

    while(Yaw_Setpoint_CStick >= 360.0)                                                 // Keep Yaw_Setpoint_CStick within 0-360 limits
      Yaw_Setpoint_CStick -= 360.0;  
    while(Yaw_Setpoint_CStick < 0.0)       
      Yaw_Setpoint_CStick += 360.0; 
  } 
  else
    cStick_Contribution = 0.0;    

  while(Yaw_Setpoint >= 360.0)                                                          // Keep Yaw_setpoint within 0-360 limits
    Yaw_Setpoint -= 360.0;  
  while(Yaw_Setpoint < 0.0)       
    Yaw_Setpoint += 360.0;  

// ---Speed stuff-----------------------------------------------------------------------
  float Speed = (float)gc_status.right - (float)gc_status.left - Speed_Offset + cStick_Contribution;
  Speed *= SCLAE_FACTOR;
  Speed += 128.0;

// ---Package for TXing ----------------------------------------------------------------  
  Yaw_Setpoint_Tx = map(int(Yaw_Setpoint), 0, 359, 0, 255); 
  Speed_Tx = (unsigned char)constrain(Speed, 0.0, 255.0);

  Serial.print("Yaw_Setpoint_Tx: ");
  Serial.print(Yaw_Setpoint_Tx, DEC);
  Serial.print(", Speed_Tx:");
  Serial.print(Speed_Tx, DEC);

  IR_Tx_Data = ((long)Yaw_Setpoint_Tx & 0xFF) << 24;
  IR_Tx_Data |= (((long)Speed_Tx & 0xFF) << 16);
  bitSet(IR_Tx_Data, 15);                                                               // This indicates a Type A Packet
    
  //IR_Tx_Data |= ((PID_Adjust_Buttons & 0b111) << 12);
  //bitWrite(IR_Tx_Data, 15, bitRead(gc_status.data1, A_BUTTON));

  Serial.print(", IR_Tx_Data:");
  Serial.println(IR_Tx_Data, BIN);
    
  while(!Transmit(IR_Tx_Data, 17, BIT_PERIOD, SIGNAL_PERIOD, 1)) {}     
}

//------------------------------------------------------------------------------------
void Configure_PPM_Input()
{
  pinMode(PPM_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_Pin), Interrupt_Fxn, CHANGE);
    
  if(digitalRead(DIP_2) == 0)                                                           // Perform range calibration
  {
    delay(100);
    PPM_Channel.Ch[0] = 0; PPM_Channel.Ch[1] = 0; PPM_Channel.Ch[2] = 0;                // Clear all channel values to see if they're being updated
    PPM_Channel.Ch[3] = 0; PPM_Channel.Ch[4] = 0; PPM_Channel.Ch[5] = 0;    
    delay(100);
    
    if(!PPM_Channel.Ch[0] && !PPM_Channel.Ch[1] && !PPM_Channel.Ch[2] && !PPM_Channel.Ch[3])      // If they're not being updated... Keep looping
    {
      Serial.println("We're not receiving a PPM signal at the moment!");
      Configure_PPM_Input();
    }
    Serial.println("A PPM signal is being received. Performing Joystick range setting. Move the joysticks to their min and max positions...");
     
    PPM_Channel.Ch_Max[0] = PPM_Channel.Ch[0]; PPM_Channel.Ch_Max[1] = PPM_Channel.Ch[1]; PPM_Channel.Ch_Max[2] = PPM_Channel.Ch[2];      // Set min and max values to whatever is being received atm.
    PPM_Channel.Ch_Max[3] = PPM_Channel.Ch[3]; PPM_Channel.Ch_Max[4] = PPM_Channel.Ch[4]; PPM_Channel.Ch_Max[5] = PPM_Channel.Ch[5];
      
    PPM_Channel.Ch_Min[0] = PPM_Channel.Ch[0]; PPM_Channel.Ch_Min[1] = PPM_Channel.Ch[1]; PPM_Channel.Ch_Min[2] = PPM_Channel.Ch[2];
    PPM_Channel.Ch_Min[3] = PPM_Channel.Ch[3]; PPM_Channel.Ch_Min[4] = PPM_Channel.Ch[4]; PPM_Channel.Ch_Min[5] = PPM_Channel.Ch[5];   
    
    unsigned long Time = millis();
    
    while(millis() - Time < 10000)                                                      // 10 seconds to move the joysticks...
    {
      for(int i = 0;i < 6;i++)
      {     
      if(millis() % 200 == 0)                                                           // Toggle LED every 150ms
      {
        digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
      }

      PPM_Channel.Ch_Max[i] = max(PPM_Channel.Ch[i], PPM_Channel.Ch_Max[i]);
      PPM_Channel.Ch_Min[i] = min(PPM_Channel.Ch[i], PPM_Channel.Ch_Min[i]);        
      }           
    }
    
    digitalWrite(LED_GREEN, HIGH);                                                      // Turns LED off! 
    
    EEPROM.put(0, PPM_Channel);
    Serial.println("Written Min, Max values to EEPROM");
    Print_PPM_Channel_Values();
  }
  else
  {
    EEPROM.get(0, PPM_Channel);
    Serial.println("Read Min, Max values from EEPROM");
    Print_PPM_Channel_Values();
  }
  
  Serial.println("Configuration Complete, Beginning IR Transmission...");
}

//------------------------------------------------------------------------------------
void Print_PPM_Channel_Values()
{
  int i;
  for(i = 0;i < 6;i++)
  {
    Serial.print("Channel ");
    Serial.print(i + 1);
    Serial.print(":   Min: ");
    Serial.print(PPM_Channel.Ch_Min[i]);
    Serial.print("   Max: ");
    Serial.println(PPM_Channel.Ch_Max[i]);    
  }
}

//------------------------------------------------------------------------------------
void UsePPM()
{  
  static float Yaw_Setpoint, Drift_Compensation;
  static float JoyStick_Angle, JoyStick_Sq_Magnitude;

  unsigned char Yaw_Setpoint_Tx, Speed_Tx;
  unsigned long IR_Tx_Data;  
  
  while(1)
  {
  /*    
    for(int i = 0;i <= 5;i++)
    {
      Serial.print("   Ch ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(PPM_Channel.Ch[i]);
    }
    Serial.println();   
  */
    
  // -------------------------Channel 1 can be speed, channel 2 yaw rate------------------------------------------------------------------------
  /*
  Speed_Tx = map(PPM_Channel.Ch[1], PPM_Channel.Ch_Min[1], PPM_Channel.Ch_Max[1], 0, 255);            
  Yaw_Setpoint += ((float)map(PPM_Channel.Ch[0], PPM_Channel.Ch_Min[0], PPM_Channel.Ch_Max[0], -1000, 1000) * 0.01);    

  */    
  // -------------------------Channel 4 can be speed, channel 1 & 2 Yaw_Setpoint----------------------------------------------------------------
  Speed_Tx = map(PPM_Channel.Ch[2], PPM_Channel.Ch_Min[2], PPM_Channel.Ch_Max[2], 0, 255);

  float Centered_ChX = (float)map(PPM_Channel.Ch[0], PPM_Channel.Ch_Min[0], PPM_Channel.Ch_Max[0], -128, 128);
  float Centered_ChY = (float)map(PPM_Channel.Ch[1], PPM_Channel.Ch_Min[1], PPM_Channel.Ch_Max[1], -128, 128);
   
  JoyStick_Sq_Magnitude = pow(Centered_ChY, 2.0) + pow(Centered_ChX, 2.0);  

  if(JoyStick_Sq_Magnitude > 400.0)
  {
    //Yaw_Setpoint -= JoyStick_Angle;                                                   // Remove old values
    JoyStick_Angle = atan2(Centered_ChX, Centered_ChY) * (180/PI);                                      // Calculate new value
    Yaw_Setpoint = JoyStick_Angle;                                                      // Add it in
  }   
  
  //--------------------------Ensure 0-360 domain-----------------------------------------------------------------------------------------------  
  while(Yaw_Setpoint >= 360.0)                                                          // Keep Yaw_setpoint within 0-360 limits
  Yaw_Setpoint -= 360.0;
  while(Yaw_Setpoint < 0.0)
  Yaw_Setpoint += 360.0;

  // ---Package for TXing ----------------------------------------------------------------
  Yaw_Setpoint_Tx = map(int(Yaw_Setpoint), 0, 359, 0, 255);

  Serial.print("Yaw_Setpoint_Tx: ");
  Serial.print(Yaw_Setpoint_Tx, DEC);
  Serial.print(", Speed_Tx:");
  Serial.print(Speed_Tx, DEC);

  IR_Tx_Data = ((long)Yaw_Setpoint_Tx & 0xFF) << 24;
  IR_Tx_Data |= (((long)Speed_Tx & 0xFF) << 16);
  bitSet(IR_Tx_Data, 15);                                                               // This indicates a Type A Packet

  Serial.print(", IR_Tx_Data:");
  Serial.println(IR_Tx_Data, BIN);

  while(!Transmit(IR_Tx_Data, 17, BIT_PERIOD, SIGNAL_PERIOD, digitalRead(DIP_4))) {}    
  }
}

//------------------------------------------------------------------------------------
char Transmit(unsigned long Data, char Bits, int Bit_Period, long Signal_Period, char Channel)
{
  // Data is sent MSB first, and a max of 32 bits of data can be sent. This excludes a start bit of half the bit period.
  // The function returns zero if the Signal_Period hasn't elapsed before the next call

  static unsigned long Time_at_Previous_Transmit = 0;
  if(micros() - Time_at_Previous_Transmit < Signal_Period)  {
    return(0);
  }
  Time_at_Previous_Transmit += Signal_Period;                                           // Instead of "= micros();" Improves relative timing accuracy.

  char index;  
  int Half_Bit_Period = Bit_Period / 2;   
  unsigned long Wait_Until;

  // Start half bit - 200us high for channel 1, 600us high for channel 2
  pinMode(IR_Pin, OUTPUT);
  
  if(Channel == 1)  {
    Wait_Until += Half_Bit_Period - 200; while(micros() - Time_at_Previous_Transmit < Wait_Until) {}     
    pinMode(IR_Pin, INPUT);
    Wait_Until += Half_Bit_Period + 200; while(micros() - Time_at_Previous_Transmit < Wait_Until) {}
  }
  else  {
    Wait_Until += Half_Bit_Period + 200; while(micros() - Time_at_Previous_Transmit < Wait_Until) {}     
    pinMode(IR_Pin, INPUT);
    Wait_Until += Half_Bit_Period - 200; while(micros() - Time_at_Previous_Transmit < Wait_Until) {}    
  }
    
  // Next send the data:
  for(index = 0; index < Bits; index++)
  {
    if(!bitRead(Data, (31 - index)))
      {
        pinMode(IR_Pin, OUTPUT);
        Wait_Until += Half_Bit_Period; while(micros() - Time_at_Previous_Transmit < Wait_Until) {}
        pinMode(IR_Pin, INPUT);
        Wait_Until += Half_Bit_Period; while(micros() - Time_at_Previous_Transmit < Wait_Until) {}          
      }       
    else
      {
        pinMode(IR_Pin, INPUT);
        Wait_Until += Half_Bit_Period; while(micros() - Time_at_Previous_Transmit < Wait_Until) {}
        pinMode(IR_Pin, OUTPUT);
        Wait_Until += Half_Bit_Period; while(micros() - Time_at_Previous_Transmit < Wait_Until) {}   
      }      
  }

  pinMode(IR_Pin, INPUT);
  return(1);
}


//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#### Interrupts #####################################################################################################################################################
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Interrupt_Fxn ()
{
  // Detects start of pulse chain by ignoring until no rising edge for 5 milliseconds
  // Then it reads each of the pulse widths and overwrites to Ch[5]         

  unsigned long Time_at_Start_of_Interrupt = micros();
  char state = digitalRead(PPM_Pin);

  static unsigned long Time, Time_Since_Previous_Signal;
  static int Pulses_Remaining = 0;    
  
  if(Pulses_Remaining && (Time_at_Start_of_Interrupt - Time_Since_Previous_Signal) < 3000)
  {
    if(state)
    {
      Time = Time_at_Start_of_Interrupt;
    }
    else
    {
      Pulses_Remaining--;
      PPM_Channel.Ch[5 - Pulses_Remaining] = Time_at_Start_of_Interrupt - Time;
    }
  } 
  
  if(Time_at_Start_of_Interrupt - Time_Since_Previous_Signal > 5000)
  { 
    Time = Time_at_Start_of_Interrupt;    
    Pulses_Remaining = 6;
  }   
 
  
  Time_Since_Previous_Signal = Time_at_Start_of_Interrupt;
}
