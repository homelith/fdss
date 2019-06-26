//------------------------------------------------------------------------------
// fdss.h 
//
// Full-Duplex Software Serial Library
//
// usage : 
//   - include fdss.h
//   - call fdss_init(RX_PIN, TX_PIN, BAUD_RATE, SAMPLING_FACTOR, PHASE_OFFSET)
//      + RX_PIN          : Pin number for RX
//      + TX_PIN          : Pin number for TX
//      + BAUD_RATE       : Target baudrate
//      + SAMPLING_FACTOR : RX sampling rate is multiplication of BAUD_RATE and SAMPLING_FACTOR
//                        : try use SAMPLING_FACTOR 3 as default
//      + PHASE_OFFSET    : RX data sampling point is shifted as multiply PHASE_OFFSET by RX sampling interval
//                        : try use PHASE_OFFSET 1 as default
//   - to send byte, call fdss_putchar(CHARACTER)
//      + function issues queueing CHARACTER to tx fifo
//   - to recv byte, call fdss_getchar(&CHARACTER)
//      + function issues popping CHARACTER from rx fifo
//      + if there is no data available, function returns -1
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// recommended parameters
//  - higher sampling factor value makes rx sampling more accurate but requires higher cpu load
//  - rx init offset should be around a 1/2 value of rx sampling factor
//---------------------------------------+-------------------------------------------------+
//                                       | automatically calcurated on library             |
//----------+-------------+--------------+-------------------------------------------------+
// target   | rx sampling | rx init      | target     | sense      | sense cnt | sense     |
// baudrate | factor      | phase offset | sense freq | divider    | per sec   | reset cnt |
//----------+-------------+--------------+------------+------------+-----------+-----------+
//       75 |           3 |            1 |        225 | 1024(0x07) |     15625 |        69 |
//      150 |           3 |            1 |        450 |  256(0x06) |     62500 |       138 |
//      300 |           3 |            1 |        900 |  128(0x05) |    125000 |       138 |
//      600 |           3 |            1 |       1800 |   64(0x04) |    250000 |       138 |
//     1200 |           3 |            1 |       3600 |   32(0x03) |    500000 |       138 |
//     2400 |           3 |            1 |       7200 |   32(0x03) |    500000 |        69 |
//     4800 |           3 |            1 |      14400 |    8(0x02) |   2000000 |       138 |
//     9600 |           3 |            1 |      28800 |    8(0x02) |   2000000 |        69 |
//    19200 |           3 |            1 |      57600 |    8(0x02) |   2000000 |        35 |

#ifndef __FDSS_H__
  #define __FDSS_H__
  int fdss_init(int rx_pin, int tx_pin, uint16_t baud_rate, uint8_t sampling_factor, uint8_t phase_offset);
  int fdss_putc(char c);
  int fdss_getc(char *c);
#endif
