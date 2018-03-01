/*
  Copyright (c) 2014 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _SERCOM_CLASS_
#define _SERCOM_CLASS_

#include "sam.h"

#define SERCOM_FREQ_REF      48000000
#define SERCOM_NVIC_PRIORITY ((1<<__NVIC_PRIO_BITS) - 1)

typedef enum
{
	UART_EXT_CLOCK = 0,
	UART_INT_CLOCK = 0x1u
} SercomUartMode;

typedef enum
{
	SPI_SLAVE_OPERATION = 0x2u,
	SPI_MASTER_OPERATION = 0x3u
} SercomSpiMode;

typedef enum
{
	I2C_SLAVE_OPERATION = 0x4u,
	I2C_MASTER_OPERATION = 0x5u
} SercomI2CMode;

typedef enum
{
	SERCOM_EVEN_PARITY = 0,
	SERCOM_ODD_PARITY,
	SERCOM_NO_PARITY
} SercomParityMode;

typedef enum
{
	SERCOM_STOP_BIT_1 = 0,
	SERCOM_STOP_BITS_2
} SercomNumberStopBit;

typedef enum
{
	MSB_FIRST = 0,
	LSB_FIRST
} SercomDataOrder;

typedef enum
{
	UART_CHAR_SIZE_8_BITS = 0,
	UART_CHAR_SIZE_9_BITS,
	UART_CHAR_SIZE_5_BITS = 0x5u,
	UART_CHAR_SIZE_6_BITS,
	UART_CHAR_SIZE_7_BITS
} SercomUartCharSize;

typedef enum
{
	SERCOM_RX_PAD_0 = 0,
	SERCOM_RX_PAD_1,
	SERCOM_RX_PAD_2,
	SERCOM_RX_PAD_3
} SercomRXPad;

typedef enum
{
	UART_TX_PAD_0 = 0x0ul,	// Only for UART
	UART_TX_PAD_2 = 0x1ul,  // Only for UART
	UART_TX_RTS_CTS_PAD_0_2_3 = 0x2ul,  // Only for UART with TX on PAD0, RTS on PAD2 and CTS on PAD3
} SercomUartTXPad;

typedef enum
{
	SAMPLE_RATE_x16 = 0x1,	//Fractional
	SAMPLE_RATE_x8 = 0x3,	//Fractional
} SercomUartSampleRate;

typedef enum
{
	SERCOM_SPI_MODE_0 = 0,	// CPOL : 0  | CPHA : 0
	SERCOM_SPI_MODE_1,		// CPOL : 0  | CPHA : 1
	SERCOM_SPI_MODE_2,		// CPOL : 1  | CPHA : 0
	SERCOM_SPI_MODE_3		// CPOL : 1  | CPHA : 1
} SercomSpiClockMode;

typedef enum
{
	SPI_PAD_0_SCK_1 = 0,
	SPI_PAD_2_SCK_3,
	SPI_PAD_3_SCK_1,
	SPI_PAD_0_SCK_3
} SercomSpiTXPad;

typedef enum
{
	SPI_CHAR_SIZE_8_BITS = 0x0ul,
	SPI_CHAR_SIZE_9_BITS
} SercomSpiCharSize;

typedef enum
{
	WIRE_UNKNOWN_STATE = 0x0ul,
	WIRE_IDLE_STATE,
	WIRE_OWNER_STATE,
	WIRE_BUSY_STATE
} SercomWireBusState;

typedef enum
{
	WIRE_WRITE_FLAG = 0x0ul,
	WIRE_READ_FLAG
} SercomWireReadWriteFlag;

typedef enum
{
	WIRE_MASTER_ACT_NO_ACTION = 0,
	WIRE_MASTER_ACT_REPEAT_START,
	WIRE_MASTER_ACT_READ,
	WIRE_MASTER_ACT_STOP
} SercomMasterCommandWire;

typedef enum
{
	WIRE_MASTER_ACK_ACTION = 0,
	WIRE_MASTER_NACK_ACTION
} SercomMasterAckActionWire;

class SERCOM
{
	public:
		SERCOM(Sercom* s) ;

		/* ========== UART ========== */
		void initUART(SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate=0) ;
		void initFrame(SercomUartCharSize charSize, SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits) ;
		void initPads(SercomUartTXPad txPad, SercomRXPad rxPad) ;

		void resetUART( void ) ;
		void enableUART( void ) ;
		void flushUART( void ) ;
		void clearStatusUART( void ) ;
		bool availableDataUART( void ) ;
		bool isBufferOverflowErrorUART( void ) ;
		bool isFrameErrorUART( void ) ;
		bool isParityErrorUART( void ) ;
		bool isDataRegisterEmptyUART( void ) ;
		uint8_t readDataUART( void ) ;
		int writeDataUART(uint8_t data) ;
		bool isUARTError() ;
		void acknowledgeUARTError() ;
		void enableDataRegisterEmptyInterruptUART();
		void disableDataRegisterEmptyInterruptUART();

		/* ========== SPI ========== */
		void initSPI(SercomSpiTXPad mosi, SercomRXPad miso, SercomSpiCharSize charSize, SercomDataOrder dataOrder) ;
		void initSPIClock(SercomSpiClockMode clockMode, uint32_t baudrate) ;

		void resetSPI( void ) ;
		void enableSPI( void ) ;
		void disableSPI( void ) ;
		void setDataOrderSPI(SercomDataOrder dataOrder) ;
		SercomDataOrder getDataOrderSPI( void ) ;
		void setBaudrateSPI(uint8_t divider) ;
		void setClockModeSPI(SercomSpiClockMode clockMode) ;
		uint8_t transferDataSPI(uint8_t data) ;
		bool isBufferOverflowErrorSPI( void ) ;
		bool isDataRegisterEmptySPI( void ) ;
		bool isTransmitCompleteSPI( void ) ;
		bool isReceiveCompleteSPI( void ) ;

		/* ========== WIRE ========== */
		void initSlaveWIRE(uint8_t address, bool enableGeneralCall = false) ;
		void initMasterWIRE(uint32_t baudrate) ;

		void resetWIRE( void ) ;
		void enableWIRE( void ) ;
    void disableWIRE( void );
    void prepareNackBitWIRE( void ) ;
    void prepareAckBitWIRE( void ) ;
    void prepareCommandBitsWire(uint8_t cmd);
		bool startTransmissionWIRE(uint8_t address, SercomWireReadWriteFlag flag) ;
		bool sendDataMasterWIRE(uint8_t data) ;
		bool sendDataSlaveWIRE(uint8_t data) ;
		bool isMasterWIRE( void ) ;
		bool isSlaveWIRE( void ) ;
		bool isBusIdleWIRE( void ) ;
		bool isBusOwnerWIRE( void ) ;
		bool isDataReadyWIRE( void ) ;
		bool isStopDetectedWIRE( void ) ;
		bool isRestartDetectedWIRE( void ) ;
		bool isAddressMatch( void ) ;
		bool isMasterReadOperationWIRE( void ) ;
    bool isRXNackReceivedWIRE( void ) ;
		int availableWIRE( void ) ;
		uint8_t readDataWIRE( void ) ;

	protected:
		Sercom* sercom;
		uint8_t calculateBaudrateSynchronous(uint32_t baudrate) ;
		uint32_t division(uint32_t dividend, uint32_t divisor) ;
		void initClockNVIC( void ) ;
};


class SercomI2C : public SERCOM  
{
public:

	void initSlave(uint8_t address) ;
	void initMaster(uint32_t baudrate) ;

	void reset( void ) ;
	void enable( void ) ;
	void disable( void );

	inline void prepareNackBit( void ) __attribute__ ((always_inline)) {
		if (isMaster())
			sercom->I2CM.CTRLB.bit.ACKACT = 1;
		else
			sercom->I2CS.CTRLB.bit.ACKACT = 1;
		}
	
	inline void prepareAckBit( void ) __attribute__ ((always_inline)) {
		if ( isMaster() ) {
			sercom->I2CM.CTRLB.bit.ACKACT = 0;
		} else {
			sercom->I2CS.CTRLB.bit.ACKACT = 0;
		}
	}	 

	inline void prepareCommandBits(uint8_t cmd) __attribute__ ((always_inline)) {
		if (isMaster()) {
			sercom->I2CM.CTRLB.bit.CMD = cmd;
			while(sercom->I2CM.SYNCBUSY.bit.SYSOP) {} // Waiting for synchronization
		} else {
			sercom->I2CS.CTRLB.bit.CMD = cmd;
		}
	}

	inline bool startTransmission(uint8_t address, SercomWireReadWriteFlag flag) __attribute__ ((always_inline)) {
		address = (address << 0x1ul) | flag; // 7-bits address + 1-bits R/W
		while ( !isBusIdle() && !isBusOwner() ); // Wait idle or owner bus mode
		sercom->I2CM.ADDR.bit.ADDR = address; // Send start and address

		// Address Transmitted
		if ( flag == WIRE_WRITE_FLAG ) { // Write mode 
			while( !(sercom->I2CM.INTFLAG.bit.MB) ) {} // Wait transmission complete
		} else  { // Read mode 
			while( !(sercom->I2CM.INTFLAG.bit.SB) ) {
				// If the slave NACKS the address, the MB bit will be set.
				// In that case, send a stop condition and return false.
				if (sercom->I2CM.INTFLAG.bit.MB) {
					sercom->I2CM.CTRLB.bit.CMD = 3; // Stop condition
					return false;
				}
				// Wait transmission complete
			}
		}
		// ACK received (0: ACK, 1: NACK)
		return !(sercom->I2CM.STATUS.bit.RXNACK);
	}

	inline bool sendDataMaster(uint8_t data) __attribute__ ((always_inline, hot)) {
		sercom->I2CM.DATA.bit.DATA = data;
		while ( !sercom->I2CM.INTFLAG.bit.MB ) {
			// If a bus error occurs, the MB bit may never be set.
			// Check the bus error bit and bail if it's set.
			if (sercom->I2CM.STATUS.bit.BUSERR) 
				return false;
		}

		//Problems on line? nack received?
		return !(sercom->I2CM.STATUS.bit.RXNACK);
	}

	inline bool sendDataSlave(uint8_t data) __attribute__ ((always_inline)) {
		sercom->I2CS.DATA.bit.DATA = data;
		return  !(!sercom->I2CS.INTFLAG.bit.DRDY || sercom->I2CS.STATUS.bit.RXNACK);
	}

	inline bool isMaster( void ) const __attribute__ ((always_inline, hot)) {
		return (sercom->I2CS.CTRLA.bit.MODE == I2C_MASTER_OPERATION);
	}

	inline bool isSlave( void ) __attribute__ ((always_inline)) {
  		return sercom->I2CS.CTRLA.bit.MODE == I2C_SLAVE_OPERATION;
	}

	inline bool isBusIdle( void ) const __attribute__ ((always_inline, hot)) {
		return (sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_IDLE_STATE);
	}

	inline bool isBusOwner( void ) const __attribute__ ((always_inline, hot)) {
		return (sercom->I2CM.STATUS.bit.BUSSTATE == WIRE_OWNER_STATE);
	}

	inline bool isDataReady( void ) __attribute__ ((always_inline)) {
 		return sercom->I2CS.INTFLAG.bit.DRDY;
	}

	inline bool isStopDetected( void ) __attribute__ ((always_inline)) {
  		return sercom->I2CS.INTFLAG.bit.PREC;
	}

	inline bool isRestartDetected( void ) __attribute__ ((always_inline)) {
  		return sercom->I2CS.STATUS.bit.SR;
	}

	inline bool isAddressMatch( void ) __attribute__ ((always_inline)) {
  		return sercom->I2CS.INTFLAG.bit.AMATCH;
	}

	inline bool isMasterReadOperation( void ) __attribute__ ((always_inline)) {
  		return sercom->I2CS.STATUS.bit.DIR;
	}

	inline bool isRXNackReceived( void ) __attribute__ ((always_inline))  {
  		return sercom->I2CM.STATUS.bit.RXNACK;
	}

	inline int available( void ) __attribute__ ((always_inline, hot)) {
		if (isMaster())
			return sercom->I2CM.INTFLAG.bit.SB;
		else
			return sercom->I2CS.INTFLAG.bit.DRDY;
	}

	inline uint8_t readData( void ) __attribute__ ((always_inline, hot)) 
	{
		if ( isMasterWIRE() ) {
			while( sercom->I2CM.INTFLAG.bit.SB == 0 ) {} // Waiting complete receive
			return sercom->I2CM.DATA.bit.DATA ;
		} else {
			return sercom->I2CS.DATA.reg ;
		}
	}

	// no member data: layout compatibility with SERCOM
};

class SercomSPI : public SERCOM
{
public:

	void init(SercomSpiTXPad mosi, SercomRXPad miso, SercomSpiCharSize charSize, SercomDataOrder dataOrder) ;
	void initClock(SercomSpiClockMode clockMode, uint32_t baudrate) ;

	inline void reset( void ) {
		sercom->SPI.CTRLA.bit.SWRST = 1;
		while (sercom->SPI.CTRLA.bit.SWRST || sercom->SPI.SYNCBUSY.bit.SWRST);
	}

	inline void enable( void ) __attribute__ ((always_inline)) {
		sercom->SPI.CTRLA.bit.ENABLE = 1;
		while (sercom->SPI.SYNCBUSY.bit.ENABLE);
	}

	inline void disable( void ) __attribute__ ((always_inline)) {
		while (sercom->SPI.SYNCBUSY.bit.ENABLE);
		sercom->SPI.CTRLA.bit.ENABLE = 0;
	}
	
	void setDataOrder(SercomDataOrder dataOrder) ;
	
	inline SercomDataOrder getDataOrder( void ) {
  		return (sercom->SPI.CTRLA.bit.DORD ? LSB_FIRST : MSB_FIRST);
	}

	void setBaudrate(uint8_t divider) ;
	void setClockMode(SercomSpiClockMode clockMode) ;
	
	inline uint8_t transfer(uint8_t data) __attribute__((always_inline, hot)) {
		sercom->SPI.DATA.bit.DATA = data; 
		while( sercom->SPI.INTFLAG.bit.RXC == 0 ) {}
		return sercom->SPI.DATA.bit.DATA;  
	}

	inline void send(uint8_t data) __attribute__((always_inline, hot)) {
		sercom->SPI.DATA.bit.DATA = data; 
		while( sercom->SPI.INTFLAG.bit.TXC == 0 ) {} 
	}

	inline uint8_t recv() __attribute__((always_inline, hot)) {
		while( sercom->SPI.INTFLAG.bit.RXC == 0 ) {}
		return sercom->SPI.DATA.bit.DATA; 
	}

	inline bool isBufferOverflowError( void ) __attribute__((always_inline))  {
  		return sercom->SPI.STATUS.bit.BUFOVF;
	}

	inline bool isDataRegisterEmpty( void ) __attribute__((always_inline)) {
  		return sercom->SPI.INTFLAG.bit.DRE;
	}

	// no member data: layout compatibility with SERCOM
};


#endif
