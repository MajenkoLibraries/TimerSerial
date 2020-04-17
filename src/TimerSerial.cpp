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

#include <TimerSerial.h>

TimerSerial *theConfiguredDevice;

TimerSerial::TimerSerial(uint8_t rx, uint8_t tx) {
	_pin_tx = tx;
	_pin_rx = rx;
	theConfiguredDevice = this;
	_rxBuffer = new CircularBuffer(_rx_buffer, 64);
	_txBuffer = new CircularBuffer(_tx_buffer, 64);
}

void TimerSerial::begin(uint32_t baud) {
	uint8_t port;
	
	pinMode(_pin_rx, INPUT);
	pinMode(_pin_tx, OUTPUT);
	if ((port = digitalPinToPort(_pin_rx)) == NOT_A_PIN) {
        return;
    }
    _rxPort = (p32_ioport *)portRegisters(port);
    _rxBit = digitalPinToBitMask(_pin_rx);

    if ((port = digitalPinToPort(_pin_tx)) == NOT_A_PIN) {
        return;
    }
    _txPort = (p32_ioport *)portRegisters(port);
    _txBit = digitalPinToBitMask(_pin_tx);

	digitalWrite(_pin_tx, HIGH);

	_baudClock.setFrequency(baud * BSAMP);
	_baudClock.attachInterrupt(interruptHandler);
	_baudClock.start();
}


enum {
	EDGEFIND,
	SAMPLING,	
	RXDONE
};

enum {
	IDLE,
	TXRUN,
	TXSTOP,
	TXDONE
};

void __attribute__((interrupt)) TimerSerial::interruptHandler() {
	theConfiguredDevice->mainInterrupt();
}

void TimerSerial::mainInterrupt() {
	static int state = HIGH;
	
	static uint8_t rxMode = EDGEFIND;
	static uint8_t rxShiftRegister = 0;
	static uint8_t rxBitNum = 0;
	static uint8_t rxBitTicker = 0;

	static uint8_t txMode = IDLE;
	static uint8_t txShiftRegister = 0;
	static uint8_t txBitNum = 0;

	static uint8_t txBitTicker = 0;

	static uint8_t rx = HIGH;

	clearIntFlag(_TIMER_1_IRQ);

	state = rx;
	rx = _rxPort->port.reg & _rxBit ? 1 : 0;

	if (rxMode == EDGEFIND) {
		if (state == HIGH && rx == LOW) { // Found a start bit
			rxMode = SAMPLING;
			rxShiftRegister = 0;
			rxBitNum = 0;
			rxBitTicker = BSAMP + (BSAMP / 2); // We have an edge, let's look for the middle from here on in.
		}
	} else if (rxMode == SAMPLING) { // Read 8 bits
		rxBitTicker--;
		if (rxBitTicker == 0) {
			rxShiftRegister >>= 1;
			rxShiftRegister |= (rx<<7);
			rxBitNum++;
			if (rxBitNum == 8) {
				rxMode = RXDONE;
			}
			rxBitTicker = BSAMP;
		}
	} else if (rxMode == RXDONE) {
		rxBitTicker--;
		if (rxBitTicker == 0) {
			if (rx == HIGH) { // Stop bit
				// We have a byte in our shift register.  Time to place it into the ring buffer.
				_rxBuffer->write(rxShiftRegister);
			}
			rxMode = EDGEFIND;
		}
	}

	if (txMode == IDLE) {
		if (_txBuffer->available()) {
			txShiftRegister = _txBuffer->read();
			txBitTicker = BSAMP;
			_txPort->lat.clr = _txBit; // Start bit
			txMode = TXRUN;
			txBitNum = 0;
		}
	} else if (txMode == TXRUN) {
		txBitTicker--;
		if (txBitTicker == 0) {
			if (txShiftRegister & 0x01) {
				_txPort->lat.set = _txBit;
			} else {
				_txPort->lat.clr = _txBit;
			}
			txShiftRegister >>= 1;
			txBitTicker = BSAMP;
			txBitNum++;
			if (txBitNum == 8) {
				txMode = TXSTOP;
			}
		}
	} else if (txMode == TXSTOP) {
		txBitTicker--;
		if (txBitTicker == 0) {
			_txPort->lat.set = _txBit;
			txBitTicker = BSAMP;
			txMode = TXDONE;
		}
	} else if (txMode == TXDONE) {
		txBitTicker--;
		if (txBitTicker == 0) {
			txMode = IDLE;
		}
	}
}

// Circular buffer support

CircularBuffer::CircularBuffer(uint8_t *buffer, uint32_t size) {
    _size = size;
    _data = buffer;
    _head = 0;
    _tail = 0;
}

CircularBuffer::CircularBuffer(uint32_t size) {
    _size = size;
    _data = (uint8_t *)malloc(size);
    _head = 0;
    _tail = 0;
}

int CircularBuffer::read() {
    int16_t chr;
    if (_head == _tail) {
        return -1;
    }
    chr = _data[_tail];
    _tail = (_tail + 1) % _size;
    return chr;
}

void CircularBuffer::write(uint8_t d) {
    uint32_t newhead = (_head + 1) % _size;
    if (newhead != _tail) {
        _data[_head] = d;
        _head = newhead;
    }
}

int CircularBuffer::available() {
    return (_size + _head - _tail) % _size;
}

void CircularBuffer::clear() {
    _head = _tail = 0;
}

uint8_t CircularBuffer::getEntry(uint32_t p) {
    if (p >= _size) {
        return 0;
    }
    return _data[p];
}

uint32_t CircularBuffer::getHead() {
    return _head;
}

uint32_t CircularBuffer::getTail() {
    return _tail;
}

