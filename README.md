#SPI Library for Teensy#

http://www.pjrc.com/teensy/td_libs_SPI.html

![](http://www.pjrc.com/teensy/td_libs_SPI_1.jpg)

SPI Async Extension Proposal
=====================

This branch contains an API to allow async SPI requests on Arduinos. It is just a proof-of-concept and still requires work. 

This extension uses interrupts to transfer an entire buffer via SPI asynchronously while the main Arduino loop runs. It interacts cleanly with beginTransaction()/endTransaction() and interrupt handlers can safely queue additional transfers. It is not currently safe to use transfer() outside of a transaction block, but protections can be added for that case. 

Usage
--------

Here's an example program that kicks off three SPI transactions:

```C++
#include "SPI.h"
uint8_t buf1[] = { 0x01, 0x02 },
        buf2[] = { 0x03, 0x04 },
        buf3[] = { 0x05, 0x06 };

SPISettings theSettings = SPISettings(100000, MSBFIRST, SPI_MODE0);

SPIQueuedTransfer t1(buf1, 2, theSettings, 0, 0),
                  t2(buf2, 2, theSettings, 0, 0),
                  t3(buf3, 2, theSettings, 0, 0);

void setup() {
  pinMode(13, OUTPUT); //High while transfers are queued
  pinMode(14, OUTPUT); //High while SPI ISR is running
  pinMode(15, OUTPUT); //High while workTransferQueue
  
  digitalWrite(13, 0);
  SPI.begin();

  SPI.queueTransfer(&t1);
  SPI.queueTransfer(&t2);
  SPI.queueTransfer(&t3);
  digitalWrite(13, 1);

  //Do some things...

  //Hold up execution until all transfers are complete
  while(!t1.is_completed && !t2.is_completed && !t3.is_completed);
}
```

Here's a trace of the output from the above program running on a Teensy 2.0 at 16Mhz with an SPI bus speed of 125kBps: 

![screen shot 2014-12-28 at 10 46 07 pm](https://cloud.githubusercontent.com/assets/17287/5566620/60398f4c-8ee3-11e4-9b1b-b143adb34447.png)

Implementation Notes
-------------------------
* No ordering is guaranteed across queued transfers. You can see in the trace that they are processed in a LIFO order if the SPI module is busy
* The inTransaction flag becomes mandatory in order to interoperate gracefully with begin/endTransaction()
* Memory overhead is approximately 5-bytes + 8-bytes per queued transfer
* Interrupt latency is ~2.5µS per byte + ~10µS per transfer

Performance
----------------

Using the ISR consumes more cycles than polling when the SPI bus is running very fast. In the trace above, the per-queued-transfer cost is about 19µS, implying that async transfers are advantageous for transfers when the SPI bus is running slower than about 2Mbps. This may not seem very useful, but the gains should be quite marked on the Teensy 3.x and other ARM-based Arduinos where the CPU runs at 48+Mhz and the FIFO queues allow transfers up to 8 bytes between interrupts. 

TODO
-------

I have only implemented a proof-of-concept for AVR; there is no ARM implementation yet. The naming can probably also use a pass. endTransaction() still needs a few lines of code to kick off any queued transfers. 