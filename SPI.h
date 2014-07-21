/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * Copyright (c) 2014 by Matthijs Kooijman <matthijs@stdin.nl> (SPISettings)
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include <Arduino.h>

// SPI_HAS_TRANSACTION means SPI has beginTransaction(), endTransaction(),
// usingInterrupt(), and SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1

#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

#define SPI_MODE0 0x00
#define SPI_MODE1 0x04
#define SPI_MODE2 0x08
#define SPI_MODE3 0x0C

#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06

#define SPI_MODE_MASK 0x0C  // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 0x03  // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR


/**********************************************************/
/*     8 bit AVR-based boards				  */
/**********************************************************/

#if defined(__AVR__)

// define SPI_AVR_EIMSK for AVR boards with external interrupt pins
#if defined(EIMSK)
  #define SPI_AVR_EIMSK	 EIMSK
#elif defined(GICR)
  #define SPI_AVR_EIMSK	 GICR
#elif defined(GIMSK)
  #define SPI_AVR_EIMSK	 GIMSK
#endif

class SPISettings {
public:
	SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
		if (__builtin_constant_p(clock)) {
			init_AlwaysInline(clock, bitOrder, dataMode);
		} else {
			init_MightInline(clock, bitOrder, dataMode);
		}
	}
	SPISettings() {
		init_AlwaysInline(4000000, MSBFIRST, SPI_MODE0);
	}
private:
	void init_MightInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
		init_AlwaysInline(clock, bitOrder, dataMode);
	}
	void init_AlwaysInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
	  __attribute__((__always_inline__)) {
		// Clock settings are defined as follows. Note that this shows SPI2X
		// inverted, so the bits form increasing numbers. Also note that
		// fosc/64 appears twice
		// SPR1 SPR0 ~SPI2X Freq
		//   0	  0	0   fosc/2
		//   0	  0	1   fosc/4
		//   0	  1	0   fosc/8
		//   0	  1	1   fosc/16
		//   1	  0	0   fosc/32
		//   1	  0	1   fosc/64
		//   1	  1	0   fosc/64
		//   1	  1	1   fosc/128

		// We find the fastest clock that is less than or equal to the
		// given clock rate. The clock divider that results in clock_setting
		// is 2 ^^ (clock_div + 1). If nothing is slow enough, we'll use the
		// slowest (128 == 2 ^^ 7, so clock_div = 6).
		uint8_t clockDiv;

		// When the clock is known at compiletime, use this if-then-else
		// cascade, which the compiler knows how to completely optimize
		// away. When clock is not known, use a loop instead, which generates
		// shorter code.
		if (__builtin_constant_p(clock)) {
			if (clock >= F_CPU / 2) {
				clockDiv = 0;
			} else if (clock >= F_CPU / 4) {
				clockDiv = 1;
			} else if (clock >= F_CPU / 8) {
				clockDiv = 2;
			} else if (clock >= F_CPU / 16) {
				clockDiv = 3;
			} else if (clock >= F_CPU / 32) {
				clockDiv = 4;
			} else if (clock >= F_CPU / 64) {
				clockDiv = 5;
			} else {
				clockDiv = 6;
			}
		} else {
			uint32_t clockSetting = F_CPU / 2;
			clockDiv = 0;
			while (clockDiv < 6 && clock < clockSetting) {
				clockSetting /= 2;
				clockDiv++;
			}
		}

		// Compensate for the duplicate fosc/64
		if (clockDiv == 6)
		clockDiv = 7;

		// Invert the SPI2X bit
		clockDiv ^= 0x1;

		// Pack into the SPISettings class
		spcr = _BV(SPE) | _BV(MSTR) | ((bitOrder == LSBFIRST) ? _BV(DORD) : 0) |
			(dataMode & SPI_MODE_MASK) | ((clockDiv >> 1) & SPI_CLOCK_MASK);
		spsr = clockDiv & SPI_2XCLOCK_MASK;
	}
	uint8_t spcr;
	uint8_t spsr;
	friend class SPIClass;
};



class SPIClass {
public:
	// Initialize the SPI library
	static void begin();

	// If SPI is to used from within an interrupt, this function registers
	// that interrupt with the SPI library, so beginTransaction() can
	// prevent conflicts.  The input interruptNumber is the number used
	// with attachInterrupt.  If SPI is used from a different interrupt
	// (eg, a timer), interruptNumber should be 255.
	static void usingInterrupt(uint8_t interruptNumber);

	// Before using SPI.transfer() or asserting chip select pins,
	// this function is used to gain exclusive access to the SPI bus
	// and configure the correct settings.
	inline static void beginTransaction(SPISettings settings) {
		if (interruptMode > 0) {
			#ifdef SPI_AVR_EIMSK
			if (interruptMode == 1) {
				interruptSave = SPI_AVR_EIMSK;
				SPI_AVR_EIMSK &= ~interruptMask;
			} else
			#endif
			{
				interruptSave = SREG;
				cli();
			}
		}
		SPCR = settings.spcr;
		SPSR = settings.spsr;
	}

	// Write to the SPI bus (MOSI pin) and also receive (MISO pin)
	inline static uint8_t transfer(uint8_t data) {
		SPDR = data;
		asm volatile("nop");
		while (!(SPSR & _BV(SPIF))) ; // wait
		return SPDR;
	}
	inline static uint16_t transfer16(uint16_t data) {
		union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } in, out;
		in.val = data;
		if (!(SPCR & _BV(DORD))) {
			SPDR = in.msb;
			while (!(SPSR & _BV(SPIF))) ;
			out.msb = SPDR;
			SPDR = in.lsb;
			while (!(SPSR & _BV(SPIF))) ;
			out.lsb = SPDR;
		} else {
			SPDR = in.lsb;
			while (!(SPSR & _BV(SPIF))) ;
			out.lsb = SPDR;
			SPDR = in.msb;
			while (!(SPSR & _BV(SPIF))) ;
			out.msb = SPDR;
		}
		return out.val;
	}
	inline static void transfer(void *buf, size_t count) {
		if (count == 0) return;
		uint8_t *p = (uint8_t *)buf;
		SPDR = *p;
		while (--count > 0) {
			uint8_t out = *(p + 1);
			while (!(SPSR & _BV(SPIF))) ;
			uint8_t in = SPDR;
			SPDR = out;
			*p++ = in;
		}
		while (!(SPSR & _BV(SPIF))) ;
		*p = SPDR;
	}

	// After performing a group of transfers and releasing the chip select
	// signal, this function allows others to access the SPI bus
	inline static void endTransaction(void) {
		if (interruptMode > 0) {
			#ifdef SPI_AVR_EIMSK
			if (interruptMode == 1) {
				SPI_AVR_EIMSK = interruptSave;
			} else
			#endif
			{
				SREG = interruptSave;
			}
		}
	}

	// Disable the SPI bus
	static void end();

	// This function is deprecated.	 New applications should use
	// beginTransaction() to configure SPI settings.
	inline static void setBitOrder(uint8_t bitOrder) {
		if (bitOrder == LSBFIRST) SPCR |= _BV(DORD);
		else SPCR &= ~(_BV(DORD));
	}
	// This function is deprecated.	 New applications should use
	// beginTransaction() to configure SPI settings.
	inline static void setDataMode(uint8_t dataMode) {
		SPCR = (SPCR & ~SPI_MODE_MASK) | dataMode;
	}
	// This function is deprecated.	 New applications should use
	// beginTransaction() to configure SPI settings.
	inline static void setClockDivider(uint8_t clockDiv) {
		SPCR = (SPCR & ~SPI_CLOCK_MASK) | (clockDiv & SPI_CLOCK_MASK);
		SPSR = (SPSR & ~SPI_2XCLOCK_MASK) | ((clockDiv >> 2) & SPI_2XCLOCK_MASK);
	}
	// These undocumented functions should not be used.  SPI.transfer()
	// polls the hardware flag which is automatically cleared as the
	// AVR responds to SPI's interrupt
	inline static void attachInterrupt() { SPCR |= _BV(SPIE); }
	inline static void detachInterrupt() { SPCR &= ~_BV(SPIE); }

private:
	static uint8_t interruptMode; // 0=none, 1=mask, 2=global
	static uint8_t interruptMask; // which interrupts to mask
	static uint8_t interruptSave; // temp storage, to restore state
};



/**********************************************************/
/*     32 bit Teensy 3.0 and 3.1			  */
/**********************************************************/

#elif defined(__arm__) && defined(TEENSYDUINO)

class SPISettings {
public:
	SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
		if (__builtin_constant_p(clock)) {
			init_AlwaysInline(clock, bitOrder, dataMode);
		} else {
			init_MightInline(clock, bitOrder, dataMode);
		}
	}
	SPISettings() {
		init_AlwaysInline(4000000, MSBFIRST, SPI_MODE0);
	}
private:
	void init_MightInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {
		init_AlwaysInline(clock, bitOrder, dataMode);
	}
	void init_AlwaysInline(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
	  __attribute__((__always_inline__)) {
		uint32_t t, c = SPI_CTAR_FMSZ(7);
		if (bitOrder == LSBFIRST) c |= SPI_CTAR_LSBFE;
		if (__builtin_constant_p(clock)) {
			if	  (clock >= F_BUS / 2) {
				t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_DBR
				  | SPI_CTAR_CSSCK(0);
			} else if (clock >= F_BUS / 3) {
				t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_DBR
				  | SPI_CTAR_CSSCK(0);
			} else if (clock >= F_BUS / 4) {
				t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0);
			} else if (clock >= F_BUS / 5) {
				t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(0) | SPI_CTAR_DBR
				  | SPI_CTAR_CSSCK(0);
			} else if (clock >= F_BUS / 6) {
				t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0);
			} else if (clock >= F_BUS / 8) {
				t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1);
			} else if (clock >= F_BUS / 10) {
				t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0);
			} else if (clock >= F_BUS / 12) {
				t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1);
			} else if (clock >= F_BUS / 16) {
				t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2);
			} else if (clock >= F_BUS / 20) {
				t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(0);
			} else if (clock >= F_BUS / 24) {
				t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2);
			} else if (clock >= F_BUS / 32) {
				t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(4) | SPI_CTAR_CSSCK(3);
			} else if (clock >= F_BUS / 40) {
				t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2);
			} else if (clock >= F_BUS / 56) {
				t = SPI_CTAR_PBR(3) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2);
			} else if (clock >= F_BUS / 64) {
				t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(5) | SPI_CTAR_CSSCK(4);
			} else if (clock >= F_BUS / 96) {
				t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(5) | SPI_CTAR_CSSCK(4);
			} else if (clock >= F_BUS / 128) {
				t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(6) | SPI_CTAR_CSSCK(5);
			} else if (clock >= F_BUS / 192) {
				t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(6) | SPI_CTAR_CSSCK(5);
			} else if (clock >= F_BUS / 256) {
				t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(7) | SPI_CTAR_CSSCK(6);
			} else if (clock >= F_BUS / 384) {
				t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(7) | SPI_CTAR_CSSCK(6);
			} else if (clock >= F_BUS / 512) {
				t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(8) | SPI_CTAR_CSSCK(7);
			} else if (clock >= F_BUS / 640) {
				t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(7) | SPI_CTAR_CSSCK(6);
			} else {	 /* F_BUS / 768 */
				t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(8) | SPI_CTAR_CSSCK(7);
			}
		} else {
			for (uint32_t i=0; i<23; i++) {
				t = ctar_clock_table[i];
				if (clock >= F_BUS / ctar_div_table[i]) break;
			}
		}
		if (dataMode & 0x08) {
			c |= SPI_CTAR_CPOL;
		}
		if (dataMode & 0x04) {
			c |= SPI_CTAR_CPHA;
			t = (t & 0xFFFF0FFF) | ((t & 0xF000) >> 4);
		}
		ctar = c | t;
	}
	static const uint16_t ctar_div_table[23];
	static const uint32_t ctar_clock_table[23];
	uint32_t ctar;
	friend class SPIClass;
};



class SPIClass {
public:
	// Initialize the SPI library
	static void begin();

	// If SPI is to used from within an interrupt, this function registers
	// that interrupt with the SPI library, so beginTransaction() can
	// prevent conflicts.  The input interruptNumber is the number used
	// with attachInterrupt.  If SPI is used from a different interrupt
	// (eg, a timer), interruptNumber should be 255.
	static void usingInterrupt(uint8_t interruptNumber);
	static void usingInterrupt(IRQ_NUMBER_t interruptName);

	// Before using SPI.transfer() or asserting chip select pins,
	// this function is used to gain exclusive access to the SPI bus
	// and configure the correct settings.
	inline static void beginTransaction(SPISettings settings) {
		if (interruptMode > 0) {
			#ifdef SPI_AVR_EIMSK
			if (interruptMode == 1) {
				interruptSave = SPI_AVR_EIMSK;
				SPI_AVR_EIMSK &= ~interruptMask;
			} else
			#endif
			{
				interruptSave = SREG;
				cli();
			}
		}
		if (SPI0_CTAR0 != settings.ctar) {
			SPI0_MCR = SPI_MCR_MDIS | SPI_MCR_HALT | SPI_MCR_PCSIS(0x1F);
			SPI0_CTAR0 = settings.ctar;
			SPI0_CTAR1 = settings.ctar| SPI_CTAR_FMSZ(8);
			SPI0_MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F);
		}
	}

	// Write to the SPI bus (MOSI pin) and also receive (MISO pin)
	inline static uint8_t transfer(uint8_t data) {
		SPI0_SR = SPI_SR_TCF;
		SPI0_PUSHR = data;
		while (!(SPI0_SR & SPI_SR_TCF)) ; // wait
		return SPI0_POPR;
	}
	inline static uint8_t transfer16(uint16_t data) {
		SPI0_SR = SPI_SR_TCF;
		SPI0_PUSHR = data | SPI_PUSHR_CTAS(1);
		while (!(SPI0_SR & SPI_SR_TCF)) ; // wait
		return SPI0_POPR;
	}
	inline static void transfer(void *buf, size_t count) {
		if (count == 0) return;
		uint8_t *p = (uint8_t *)buf;
		SPDR = *p;
		while (--count > 0) {
			uint8_t out = *(p + 1);
			while (!(SPSR & _BV(SPIF))) ;
			uint8_t in = SPDR;
			SPDR = out;
			*p++ = in;
		}
		while (!(SPSR & _BV(SPIF))) ;
		*p = SPDR;
	}

	// After performing a group of transfers and releasing the chip select
	// signal, this function allows others to access the SPI bus
	inline static void endTransaction(void) {
		if (interruptMode > 0) {
			#ifdef SPI_AVR_EIMSK
			if (interruptMode == 1) {
				SPI_AVR_EIMSK = interruptSave;
			} else
			#endif
			{
				SREG = interruptSave;
			}
		}
	}

	// Disable the SPI bus
	static void end();

	// This function is deprecated.	 New applications should use
	// beginTransaction() to configure SPI settings.
	static void setBitOrder(uint8_t bitOrder);

	// This function is deprecated.	 New applications should use
	// beginTransaction() to configure SPI settings.
	static void setDataMode(uint8_t dataMode);

	// This function is deprecated.	 New applications should use
	// beginTransaction() to configure SPI settings.
	inline static void setClockDivider(uint8_t clockDiv) {
		if (clockDiv == SPI_CLOCK_DIV2) {
			setClockDivider_noInline(SPISettings(8000000, MSBFIRST, SPI_MODE0).ctar);
		} else if (clockDiv == SPI_CLOCK_DIV4) {
			setClockDivider_noInline(SPISettings(4000000, MSBFIRST, SPI_MODE0).ctar);
		} else if (clockDiv == SPI_CLOCK_DIV8) {
			setClockDivider_noInline(SPISettings(2000000, MSBFIRST, SPI_MODE0).ctar);
		} else if (clockDiv == SPI_CLOCK_DIV16) {
			setClockDivider_noInline(SPISettings(1000000, MSBFIRST, SPI_MODE0).ctar);
		} else if (clockDiv == SPI_CLOCK_DIV32) {
			setClockDivider_noInline(SPISettings(500000, MSBFIRST, SPI_MODE0).ctar);
		} else if (clockDiv == SPI_CLOCK_DIV64) {
			setClockDivider_noInline(SPISettings(250000, MSBFIRST, SPI_MODE0).ctar);
		} else { /* clockDiv == SPI_CLOCK_DIV128 */
			setClockDivider_noInline(SPISettings(125000, MSBFIRST, SPI_MODE0).ctar);
		}
	}
	static void setClockDivider_noInline(uint32_t clk);

	// These undocumented functions should not be used.  SPI.transfer()
	// polls the hardware flag which is automatically cleared as the
	// AVR responds to SPI's interrupt
	inline static void attachInterrupt() { }
	inline static void detachInterrupt() { }

	// Teensy 3.x can use alternate pins for these 3 SPI signals.
	inline static void setMOSI(uint8_t pin) __attribute__((always_inline)) {
		SPCR.setMOSI(pin);
	}
	inline static void setMISO(uint8_t pin) __attribute__((always_inline)) {
		SPCR.setMISO(pin);
	}
	inline static void setSCK(uint8_t pin) __attribute__((always_inline)) {
		SPCR.setSCK(pin);
	}
	// return true if "pin" has special chip select capability
	static bool pinIsChipSelect(uint8_t pin);
	// return true if both pin1 and pin2 have independent chip select capability
	static bool pinIsChipSelect(uint8_t pin1, uint8_t pin2);
	// configure a pin for chip select and return its SPI_MCR_PCSIS bitmask
	static uint8_t setCS(uint8_t pin);

private:
	static uint8_t interruptMode; // 0=none, 1=mask, 2=global
	static uint8_t interruptMask; // which interrupts to mask
	static uint8_t interruptSave; // temp storage, to restore state
};


/**********************************************************/
/*     32 bit Arduino Due				  */
/**********************************************************/

#elif defined(__arm__) && defined(__SAM3X8E__)

#undef SPI_MODE0
#undef SPI_MODE1
#undef SPI_MODE2
#undef SPI_MODE3
#define SPI_MODE0 0x02
#define SPI_MODE1 0x00
#define SPI_MODE2 0x03
#define SPI_MODE3 0x01

#undef SPI_CLOCK_DIV2
#undef SPI_CLOCK_DIV4
#undef SPI_CLOCK_DIV8
#undef SPI_CLOCK_DIV16
#undef SPI_CLOCK_DIV32
#undef SPI_CLOCK_DIV64
#undef SPI_CLOCK_DIV128
#define SPI_CLOCK_DIV2	 11
#define SPI_CLOCK_DIV4	 21
#define SPI_CLOCK_DIV8	 42
#define SPI_CLOCK_DIV16	 84
#define SPI_CLOCK_DIV32	 168
#define SPI_CLOCK_DIV64	 255
#define SPI_CLOCK_DIV128 255

enum SPITransferMode {
	SPI_CONTINUE,
	SPI_LAST
};

class SPIClass {
  public:
	SPIClass(Spi *_spi, uint32_t _id, void(*_initCb)(void));

	byte transfer(uint8_t _data, SPITransferMode _mode = SPI_LAST) { return transfer(BOARD_SPI_DEFAULT_SS, _data, _mode); }
	byte transfer(byte _channel, uint8_t _data, SPITransferMode _mode = SPI_LAST);

	// Transaction Functions
	void usingInterrupt(uint8_t interruptNumber);
	void beginTransaction(uint8_t clockDiv, BitOrder bitOrder, uint8_t dataMode);
	void endTransaction(void);

	// SPI Configuration methods
	void attachInterrupt(void);
	void detachInterrupt(void);

	void begin(void);
	void end(void);

	// Attach/Detach pin to/from SPI controller
	void begin(uint8_t _pin);
	void end(uint8_t _pin);

	// These methods sets a parameter on a single pin
	void setBitOrder(uint8_t _pin, BitOrder);
	void setDataMode(uint8_t _pin, uint8_t);
	void setClockDivider(uint8_t _pin, uint8_t);

	// These methods sets the same parameters but on default pin BOARD_SPI_DEFAULT_SS
	void setBitOrder(BitOrder _order) { setBitOrder(BOARD_SPI_DEFAULT_SS, _order); };
	void setDataMode(uint8_t _mode) { setDataMode(BOARD_SPI_DEFAULT_SS, _mode); };
	void setClockDivider(uint8_t _div) { setClockDivider(BOARD_SPI_DEFAULT_SS, _div); };

  private:
	void init();

	Spi *spi;
	uint32_t id;
	BitOrder bitOrder[SPI_CHANNELS_NUM];
	uint32_t divider[SPI_CHANNELS_NUM];
	uint32_t mode[SPI_CHANNELS_NUM];
	void (*initCb)(void);
	bool initialized;
	static uint8_t interruptMode;  // 0=none, 1=mask, 2=global
	static uint8_t interruptMask;  // bits 0:3=pin change
	static uint8_t interruptSave;  // temp storage, to restore state
};






#endif



extern SPIClass SPI;

#endif
