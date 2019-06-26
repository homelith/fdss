# Full-Duplex Software Serial

- software emulated UART serial communication library supports background transmitting and receiving simultaneously

- yet another implementation of AltSoftSerial Library (https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html) for my study

- currently works only in Arduino UNO (atmega328p)

## quick usage

- copy whole directory structure into your Documents/Arduino/libraries.

- start arduino IDE and select `fdss -> send_and_recv` from custom sketch examples.

- connect D3 pin and D2 pin to loopback communication.

- run and type any characters to console then you can see the characters will echo back to console.

- `Hello, world` string will be transmitted every 2 sec which is useful to see waveform standalone with oscilloscopes.