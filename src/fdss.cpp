//------------------------------------------------------------------------------
// fdss.cpp
//
// Full-Duplex Software Serial Library
//------------------------------------------------------------------------------

#include "Arduino.h"
#include "fdss.h"

//------------------------------------------------------------------------------
// performance tuning parameters
//------------------------------------------------------------------------------
// ring fifo depth in bytes
#define  FDSS_TX_RING_LEN  64
#define  FDSS_RX_RING_LEN  64

// recovery time after send / recv
#define  FDSS_TX_COOL_TIME 3
#define  FDSS_RX_COOL_TIME 2

// atmega328p in Arduino UNO runs at 16 MHz
#define  CLOCKRATE  16000000

// idx of this array will be applied to TCCR2B[2:0]
// e.g) TCCR2B[2:0] == 0x02 means frequency divider set to 8
// idx 0x00 is ignored (0x00 : disable timer2 counter)
const uint16_t FDSS_DIVIDER[8] = {65535,     1,     8,    32,    64,   128,   256,  1024};

uint8_t  fdss_divider_mode;    // calculated divider mode
uint8_t  fdss_reset_num;       // calculated reset count
uint8_t  fdss_sampling_factor; // user specified rx sampling factor
uint8_t  fdss_rx_phase_offset; // user specified rx phase offset

uint8_t  fdss_sampling_cnt;    // sampling counter to be reset at fdss_sampling_factor
uint8_t  fdss_rx_phase;        // rx read phase determined at detection of start bit

int      fdss_tx_pin;          // tx pin number
uint8_t  fdss_tx_state;        // tx FSM state (0 : idle & start bit, 1-8: data, 9: stop bit)
uint8_t  fdss_tx_byte;
uint8_t  fdss_tx_ring[FDSS_TX_RING_LEN];
uint8_t  fdss_tx_ring_head;
uint8_t  fdss_tx_ring_tail;
uint8_t  fdss_tx_cool_time;

int      fdss_rx_pin;          // rx pin number
uint8_t  fdss_rx_state;        // rx FSM state (0 : idle, 1: start bit , 2-9: data)
uint8_t  fdss_rx_byte;
uint8_t  fdss_rx_ring[FDSS_RX_RING_LEN];
uint8_t  fdss_rx_ring_head;
uint8_t  fdss_rx_ring_tail;
uint8_t  fdss_rx_cool_time;

void fdss_rx_proc(){
  if (fdss_rx_state == 0) {
    if (fdss_rx_cool_time == 0) {
      // When in recv idle state, rx sense is executed on any fdss_interval_cnt phase
      int t = digitalRead(fdss_rx_pin);
      if (t == LOW) {
        // if startbit is detected, set init offset
        fdss_rx_phase = fdss_sampling_cnt + fdss_rx_phase_offset;
        if (fdss_rx_phase >= fdss_sampling_factor) {
          fdss_rx_phase -= fdss_sampling_factor;
        }
        fdss_rx_state ++;
      }
    } else {
      fdss_rx_cool_time --;
    }
  } else {
    if (fdss_sampling_cnt == fdss_rx_phase) {
      // When in recv working state, recv process is executed only at fdss_interval_cnt phase == fdss_rx_phase
      // read bit and push to byte buffer   
      int t = digitalRead(fdss_rx_pin);
      fdss_rx_byte = (fdss_rx_byte >> 1) | (t == HIGH ? 0x80 : 0x00);
      
      if (fdss_rx_state == 9) {
        uint8_t tail_plus_one = (fdss_rx_ring_tail == (FDSS_RX_RING_LEN - 1) ? 0 : fdss_rx_ring_tail + 1);
        if (tail_plus_one != fdss_rx_ring_head) {      
          // if there is enough space left in ring (tail + 1 != head), complete 1 byte and push to fifo ring
          fdss_rx_ring[fdss_rx_ring_tail] = fdss_rx_byte;
          fdss_rx_ring_tail = tail_plus_one;
        }
        fdss_rx_state = 0;  // reset rx state
        fdss_rx_cool_time = FDSS_RX_COOL_TIME * fdss_sampling_factor;
      } else {
        fdss_rx_state ++;   // increment rx state
      }
    }
  }
}

void fdss_tx_proc(){
  if (fdss_sampling_cnt == 0) {
    // tx proc do work only fdss_interval_cnt phase 0
    if (fdss_tx_state == 0) {
      if (fdss_tx_cool_time == 0) {
        if (fdss_tx_ring_head != fdss_tx_ring_tail) {
          // if there is any data to send (head != tail), pop 1 byte from fifo ring
          fdss_tx_byte = fdss_tx_ring[fdss_tx_ring_head];
          fdss_tx_ring_head = (fdss_tx_ring_head == (FDSS_TX_RING_LEN - 1) ? 0 : fdss_tx_ring_head + 1);
          // issue start bit
          digitalWrite(fdss_tx_pin, LOW);
          fdss_tx_state ++;
        }
      } else {
        fdss_tx_cool_time --;
      }
    } else {
      if (fdss_tx_state == 9) {
        digitalWrite(fdss_tx_pin, HIGH);      // issue stop bit
        fdss_tx_state = 0;                       // reset tx state
        fdss_tx_cool_time = FDSS_TX_COOL_TIME;   // set recover time to restart tx
      } else {
        // write fdss_tx_byte lsb to output pin
        int t = ((fdss_tx_byte & 0x01) == 0x01 ? HIGH : LOW);
        digitalWrite(fdss_tx_pin, t);
        fdss_tx_byte = fdss_tx_byte >> 1;
        fdss_tx_state ++;                        // increment tx state
      }
    }
  }
}

// Timer2 callback function
ISR(TIMER2_COMPA_vect) {
  // call tx & rx proc
  fdss_tx_proc();
  fdss_rx_proc();

  // increment & wrap around interval cnt
  fdss_sampling_cnt ++;
  if (fdss_sampling_cnt == fdss_sampling_factor) {
    fdss_sampling_cnt = 0;
  }
}

int fdss_init(int rx_pin, int tx_pin, uint16_t baud_rate, uint8_t sampling_factor, uint8_t phase_offset)
{
  // register pins and settings
  fdss_rx_pin = rx_pin;
  fdss_tx_pin = tx_pin;
  pinMode(fdss_rx_pin,  INPUT);
  pinMode(fdss_tx_pin, OUTPUT);
  
  fdss_sampling_factor = sampling_factor;
  fdss_rx_phase_offset = phase_offset;

  // calc divider and reset count
  fdss_divider_mode = 255;
  uint32_t reset_num;
  for (uint8_t i = 1; i < 8; i ++) {
    reset_num = CLOCKRATE / (uint32_t)(FDSS_DIVIDER[i]) / (uint32_t)baud_rate / (uint32_t)sampling_factor;
    if (reset_num < 256) {
      fdss_divider_mode = i;
      fdss_reset_num = (uint8_t)reset_num;
      break;
    }
  }
  if (fdss_divider_mode == 255) {
    return -1;
  }

  // set default value to transmitter
  digitalWrite(fdss_tx_pin, HIGH);

  // set Timer2 callback
  TCCR2A = 0;
  TCCR2B = 0;
  OCR2A  = fdss_reset_num;
  TCCR2A = (1 << WGM21);      // enable CTC mode
  TCCR2B = fdss_divider_mode; // set clock prescale divider
  TIMSK2 = (1 << OCIE2A);     // enable programmable counter A interrupt

  // init globals
  fdss_sampling_cnt = 0;
  fdss_rx_phase = 0;

  fdss_tx_state = 0;
  fdss_tx_ring_head = 0;
  fdss_tx_ring_tail = 0;
  fdss_tx_cool_time = 0;
  
  fdss_rx_state = 0;
  fdss_rx_ring_head = 0;
  fdss_rx_ring_tail = 0;
  fdss_rx_cool_time = 0;
  
  return 0;
}

// push 1 character to tx ring fifo
int fdss_putc(char c)
{
  uint8_t tail_plus_one = (fdss_tx_ring_tail == (FDSS_TX_RING_LEN - 1) ? 0 : fdss_tx_ring_tail + 1);
  if (tail_plus_one != fdss_tx_ring_head) {
    fdss_tx_ring[fdss_tx_ring_tail] = c;
    fdss_tx_ring_tail = tail_plus_one;
    return 0;
  } else {
    return -1;
  }
}

// pop 1 character from rx ring fifo
int fdss_getc(char *c)
{
  if (fdss_rx_ring_head != fdss_rx_ring_tail) {
    (*c) = fdss_rx_ring[fdss_rx_ring_head];
    fdss_rx_ring_head = (fdss_rx_ring_head == (FDSS_RX_RING_LEN - 1) ? 0 : fdss_rx_ring_head + 1);
    return 0;
  } else {
    return -1;
  }
}
