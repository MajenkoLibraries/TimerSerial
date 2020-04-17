/*
 * Copyright (c) 2014, Majenko Technologies
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 * 
 * * Neither the name of Majenko Technologies nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _TIMERSERIAL_H
#define _TIMERSERIAL_H

#if (ARDUINO >= 100)
# include <Arduino.h>
#else
# include <WProgram.h>
#endif

#include <Timer.h>

#define TS_BUFSZ 64
#define BSAMP 4

class CircularBuffer {
    private:
        uint8_t *_data;
        int _head;
        int _tail;
        int _size;

    public:
        CircularBuffer(uint32_t);
        CircularBuffer(uint8_t *, uint32_t);
        int read();
        void write(uint8_t);
        int available();
        void clear();
        uint8_t getEntry(uint32_t p);
        uint32_t getHead();
        uint32_t getTail();

        
};

class TimerSerial : public Stream {
	private:
		uint8_t _pin_tx;
		uint8_t _pin_rx;

		uint8_t _tx_buffer[TS_BUFSZ];
		uint8_t _rx_buffer[TS_BUFSZ];

		CircularBuffer *_txBuffer;
		CircularBuffer *_rxBuffer;

		p32_ioport *_rxPort;
		uint16_t _rxBit;
		
		p32_ioport *_txPort;
		uint16_t _txBit;

		Timer1 _baudClock;
		
	public:
		TimerSerial(uint8_t rx, uint8_t tx);
		void begin(uint32_t baud);

		static void interruptHandler();
		void mainInterrupt();

		size_t write(uint8_t c) { 
			uint32_t newhead = (_txBuffer->getHead() + 1) % TS_BUFSZ;
			while (newhead == _txBuffer->getTail()) {
				newhead = (_txBuffer->getHead() + 1) % TS_BUFSZ;
			}
			_txBuffer->write(c); 
            return 1;
		}
		
		int available() { return _rxBuffer->available(); }
		int read() { return _rxBuffer->read(); }
		int peek() { return _rxBuffer->getEntry(_rxBuffer->getHead()); }
		void flush() { while (_txBuffer->available()); }
};

#endif
