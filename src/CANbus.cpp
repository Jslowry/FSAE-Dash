/*
Bulldog Motorsports 2023 Shifter Motor Controller
Contact Patrick Younes for detailed explaination
Version 1.1 uses CAN for communication between the teensy and the ECU
V1.1 does use the Teensy Bounce library instead of ButtonDebounce.h
V1.1 does NOT include display function but does read information from CAN for the display
*/

#include <Bounce2.h>
#include <FlexCAN_T4.h>

// right side outputs
const byte PUL = 15; //A1 or SEN1 in schematic
const byte DIR = 18; //A4 or SEN2 in schematic

// left side inputs
const byte SFT_UP = 16; //A2 or PDU in schematic
const byte SFT_DN = 17; //A3 or PDD in schematic

// other variables
const int U_L_RPM = 5000; //upper limit RPM for down shift rejection
const int DB_TIME = 40; //debounce time for paddles in ms
const byte REJTRY = 3; //how many tries it takes to shift incase of fail
const byte upShiftTime = 100; //100 ms
const byte dnShiftTime = 100; //100 ms
const byte nuShiftTime = 30; //30 ms

bool rejectFlag = false;
bool upState, dnState = true;
bool upNewState, dnNewState = true;
byte shiftTry = 1;
byte oldGear = 0;
static CAN_message_t txmsg, rxmsg;

// variables read from CAN
// can RPM, can BAT, can current gear, can AFR target, can Coolent Temp, can AFR, can speed, can lost sync count
int cRPM, cBAT, cGear, cAFRtgt, cCLT, cAFR, cSPD, cSYNC  = 0;

Bounce upPaddle = Bounce();
Bounce dnPaddle = Bounce();

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

void setup() {

// set up shifting buttons
  upPaddle.attach(SFT_UP, INPUT_PULLUP);
  dnPaddle.attach(SFT_DN, INPUT_PULLUP);
  upPaddle.interval(DB_TIME);
  dnPaddle.interval(DB_TIME);

// set up output pins
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);

// set up can bus
  can1.begin();
  can1.setBaudRate(500000);
  can1.enableMBInterrupts();  
  can1.onReceive(canSniff);

}

// every time a can message is received, sniff it and make sure it's something we want
void canSniff(const CAN_message_t &msg) {

  // switch statement better than nested ifs
  switch(msg.id) { // ID's 1520+ are Megasquirt CAN broadcast frames

    case 1520: // group 0
      cRPM = (int)(word(msg.buf[6], rxmsg.buf[7]));
    break;
    
    case 1521: // group 1
      cAFRtgt = (int)(word(0x00, msg.buf[4]));
    break;

    case 1522: // group 2
      cCLT = (int)(word(msg.buf[6], msg.buf[7]));
    break;

    case 1523: // group 3
      cBAT = (int)(word(msg.buf[2], msg.buf[3]));
    break;

    case 1551: // group 31
      cAFR = (int)(word(0x00, msg.buf[0]));
    break;

    case 1553: // group 33
      cGear = (int)(word(0x00, msg.buf[6]));
    break;

    case 1562: // group 42
      cSPD = (int)(word(msg.buf[0], msg.buf[1]));
    break;

    case 1563: // group 43
      cSYNC = (int)(word(0x00, msg.buf[0]));
    break;

    default: // in the case of not a broadcast packet
    break;
  }
}

void shiftUp() {
  oldGear = cGear;
  
  //digitalWrite(C_SPAR, 1); // cut spark
  digitalWrite(DIR, 1); // set direction
  digitalWrite(PUL, 1); // shift motor
  delay(upShiftTime);   // keep shifting
  digitalWrite(PUL, 0); // stop shifting
  //digitalWrite(C_SPAR, 0); // resume spark
  
  if(oldGear == cGear){ // if the shift didn't occur, aka gear before = gear after
    if(oldGear != 6){
      if(shiftTry < REJTRY){
        shiftTry = shiftTry + 1; //keep a count of how many tries it took
        shiftUp();
      } else {
        rejectFlag = true; // if it took REJTRY tries, send a reject signal
      }
    } else {
      rejectFlag = true; //if the gear is 6, GPOS can not be accurately determined
    }
  }
}

void shiftDown() {
  oldGear = cGear;
  if(cRPM < U_L_RPM) { //if RPM too high, don't even attempt to shift
    
    digitalWrite(DIR, 0);
    digitalWrite(PUL, 1);
    delay(dnShiftTime);
    digitalWrite(PUL, 0);
    
    if(oldGear == cGear){ // if the shift didn't occur, aka gear before = gear after
      if(oldGear != 6){ 
        if(shiftTry < REJTRY){
          shiftTry = shiftTry + 1; //keep a count of how many tries it took
          shiftDown(); //try again
        } else {
          rejectFlag = true; // if it took REJTRY tries, send a reject signal
        }
      } else {
        rejectFlag = true; //if the gear is 6, GPOS can not be accurately determined
      }
    }
  } else {
    rejectFlag = true;
  }
}

void shiftNeutral() {
   // car can go to neutral if in 2nd or 1st gear
  while(cGear > 2){
    shiftDown();
  }
  oldGear = cGear;
  
  if (oldGear == 2){
    // neutral requires a half shift

    digitalWrite(DIR, 0); // shift down
    digitalWrite(PUL, 1);
    delay(nuShiftTime);
    digitalWrite(PUL, 0);
    
  } else if(oldGear == 1) {
    // neutral requires a half shift

    digitalWrite(DIR, 1); // shift up
    digitalWrite(PUL, 1);
    delay(nuShiftTime);
    digitalWrite(PUL, 0);
  }

  if (cGear == oldGear){ //if the shift didn't work, try again
    if(oldGear != 6){
      if(shiftTry < REJTRY){
        shiftNeutral(); 
        shiftTry = shiftTry + 1; //keep a count of how many tries it took
      } else {
        rejectFlag = true; //if it took REJTRY tries, send a reject signal
      }
    } else {
      rejectFlag = true;
    }
  }
}

void loop() {

  // updates the states of the pins
  upPaddle.update();
  dnPaddle.update();

  //makes it so the rest of the code is executed only on a change in button state
  if (upPaddle.changed() || dnPaddle.changed()){
      
    // actual shifting up and down
    if(upPaddle.fell() && dnPaddle.fell()){
      shiftNeutral();
      shiftTry = 1;
    } else if(upPaddle.fell() && !dnPaddle.changed()){
      shiftUp();
      shiftTry = 1;
    } else if(!upPaddle.changed() && dnPaddle.fell()){
      shiftDown();
      shiftTry = 1;
    }
  
    // sends a signal to dash for rejected shift
    if(rejectFlag == true){
      //digitalWrite(FAIL, 1);
      delay(10);
      rejectFlag = false;
      //digitalWrite(FAIL, 0);
    }
  }

}
