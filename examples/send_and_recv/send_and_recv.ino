//------------------------------------------------------------------------------
// test_fdss.ino
//
// a testing environment for Full-Duplex Software Serial library
//
// - include fdss.h and init
// - transfer COM port input to FDSS port tx
// - transfer FDSS port rx input to COM port
// - for standalone test, sending "hello, world" every 5 seconds
//------------------------------------------------------------------------------

#include <fdss.h>

#define PIN_FDSS_RX        2
#define PIN_FDSS_TX        3
#define FDSS_BAUDRATE      9600

unsigned long prev_tick;
unsigned long curr_tick;

// execute once after power on
void setup() {
  // COM port
  Serial.begin(9600);

  // init full duplex serial
  fdss_init(PIN_FDSS_RX, PIN_FDSS_TX, FDSS_BAUDRATE, 2, 1);

  // re-setup pins
  //pinMode(PIN_FDSS_RX, INPUT_PULLUP);
  //pinMode(PIN_FDSS_TX, OUTPUT);
  
  // initialize previous tick time
  prev_tick = millis() - 4000;
}

// execute repeatedly
void loop() {
  curr_tick = millis();

  if (curr_tick - prev_tick > 2000) {
    // send hello
    fdss_putc('H');
    fdss_putc('e');
    fdss_putc('l');
    fdss_putc('l');
    fdss_putc('o');
    fdss_putc(',');
    fdss_putc(' ');
    fdss_putc('w');
    fdss_putc('o');
    fdss_putc('r');
    fdss_putc('l');
    fdss_putc('d');
    fdss_putc('\r');
    fdss_putc('\n');
    Serial.println("`Hello, world` sent");
    
    // timer reset
    prev_tick = curr_tick;
  }

  // read COM port input and transfer to FDSS TX
  if (Serial.available() > 0) {
    fdss_putc(Serial.read());
  }

  // read FDSS RX and transfer to COM port output
  char c;
  if (fdss_getc(&c) != -1) {
    Serial.print(c);
  }
}
