/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++}
{																			}
{       Filename: spi.c														}
{       Copyright(c): Leon de Boer(LdB) 2019								}
{       Version: 1.10														}
{																			}
{***************************************************************************}
{                                                                           }
{     Defines an API interface for the SPI devices on Pic24F & Linux		}
{																            }
{++++++++++++++++++++++++[ REVISIONS ]++++++++++++++++++++++++++++++++++++++}
{  1.00 Initial version														}
{  1.10 Compacted mode														}
{++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdbool.h>			// C standard unit for bool, true, false
#include <stdint.h>				// C standard unit for uint32_t etc

#ifdef __XC16
#include <xc.h>                 // PIC header
#else
#include <fcntl.h>				// Needed for SPI port
#include <sys/ioctl.h>			// Needed for SPI port
#include <linux/spi/spidev.h>	// Needed for SPI port
#include <unistd.h>				// Needed for SPI port
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#endif

#include "platform.h"
#include "spi.h"

#if SPI_DRIVER_VERSION != 1100
#error "Header does not match this version of file"
#endif

#ifdef __XC16												// COMPILING FOR PIC24F
#define FPB 32000000UL
#endif

struct spi_device
{
	int spi_fd;									// File descriptor for the SPI device
	uint32_t spi_speed;							// SPI speed
	uint8_t spi_bitsPerWord;					// SPI bits per word
	LOCK lock;									// SPI access lock		
    #ifdef __XC16
    volatile uint16_t* SPIxBUFL;                // SPI buffer pointer
    volatile uint16_t* SPIxSTATUS;              // SPI status pointer  
    volatile uint16_t* SPIxCS;                  // SPI CS register pointer
    uint16_t SPIxCSBIT;                         // SPI CS register bit
    #endif
	struct {
        uint8_t spi_num : 4;					// SPI device table number
        uint8_t _reserved: 2;        			// reserved
		uint8_t uselocks : 1;					// Locks to be used for access
		uint8_t inuse : 1;						// In use flag
	};
};

/* Global table of SPI devices.  */
static struct spi_device spitab[NSPI] = { {0} };

/*-[ SpiOpenPort ]----------------------------------------------------------}
. Creates a SPI handle which provides access to the SPI device number.
. The SPI device is setup to the bits, sped and mode provided.
. RETURN: valid SPI_HANDLE for success, NULL for any failure
.--------------------------------------------------------------------------*/
SPI_HANDLE SpiOpenPort (uint8_t spi_devicenum, uint8_t bit_exchange_size, uint32_t speed, uint8_t mode, bool useLock)
{
	SPI_HANDLE spi = 0;												// Preset null handle
	if (spi_devicenum < NSPI && speed != 0 && mode < 4)
	{
		struct spi_device* spi_ptr = &spitab[spi_devicenum];		// SPI device pointer 
		spi_ptr->spi_fd = 0;										// Zero SPI file device
		spi_ptr->spi_num = spi_devicenum;							// Hold spi device number
		spi_ptr->spi_bitsPerWord = bit_exchange_size;				// Hold SPI exchange size
		spi_ptr->spi_speed = speed;									// Hold SPI speed setting
		spi_ptr->uselocks = (useLock == true) ? 1 : 0;				// Set use lock
		if (useLock) Lock_Init(&spi_ptr->lock);						// Initialize the lock

		#ifdef __XC16												// COMPILING FOR PIC24F
            uint16_t divisor = (uint16_t)(FPB/(2*speed));           // Initial divisor calc
            if (divisor > 0) divisor--;                             // If not zero subtract 1 ...  divisor = (FPB/(2*speed))-1
            switch (spi_devicenum)
            {
                case 0:                                             // SPI 0
                {
                    spi_ptr->SPIxBUFL = (uint16_t*)&SPI1BUFL;       // Set pointer to SPI0 buffer register
                    spi_ptr->SPIxSTATUS = (uint16_t*)&SPI1STATL;    // Set pointer to SPI0 status register
                    SPI1CON1bits.SPIEN = 0;                         // Disable SPI port, scant change setting while enabled
                    SPI1CON1bits.SPISIDL = 0;                       // Continue module operation in Idle mode 
                    SPI1STATLbits.SPIROV = 0;                       // Clear receive overrun
                    IFS0bits.SPI1IF = 0;                            // Clear interrupt flag
                    IEC0bits.SPI1IE = 0;                            // Disable interrupt
                    SPI1IMSKLbits.SRMTEN = 1;                       // Enable interrupt mask for shift register empty
                    SPI1CON1Hbits.IGNROV = 1;                       // Ignore receive overrun
                    SPI1CON1Hbits.IGNTUR = 1;                       // Ignore transmit underrun
                    SPI1CON1Hbits.AUDEN = 0;                        // Audio protocol disabled
                    SPI1CON1Hbits.SPISGNEXT = 0;                    // Data from RX not sign extended 
                    SPI1CON1bits.DISSCK	= 0;                        // Internal SPIx clock is enabled
                    SPI1CON1bits.DISSDO	= 0;                        // SDOx pin is controlled by the module
                    SPI1CON1bits.DISSDI = 0;                        // SDIx pin is controlled by the module
                    SPI1CON1bits.MODE16 = 0;                        // set in 16-bit mode, clear in 8-bit mode
                    SPI1CON1bits.MODE32 = 0;                        // set in 32-bit mode, clear in 16/8-bit mode
                    SPI1CON1bits.SMP = 0;                           // Input data sampled at middle of data output time
                    switch (mode)
                    {
                        case SPI_MODE_0:
                        {
                            SPI1CON1bits.CKP = 0;                   // Clock idle low
                            SPI1CON1bits.CKE = 1;                   // Transmit on transition from active clock state to Idle clock state
                        }
                        break;
                        case SPI_MODE_1:
                        {
                            SPI1CON1bits.CKP = 0;                   // Clock idle low
                            SPI1CON1bits.CKE = 0;                   // Transmit on transition from Idle clock state to active clock state
                        }
                        break;
                        case SPI_MODE_2:
                        {
                            SPI1CON1bits.CKP = 1;                   // Clock idle high
                            SPI1CON1bits.CKE = 1;                   // Transmit on transition from active clock state to Idle clock state
                        }
                        break;
                        case SPI_MODE_3:
                        {
                            SPI1CON1bits.CKP = 1;                   // Clock idle high
                            SPI1CON1bits.CKE = 0;                   // Transmit on transition from Idle clock state to active clock state
                        }
                        break;
                    };
                    SPI1CON1bits.MSTEN = 1;                         // 1 =  Master mode; 0 =  Slave mode... based on your communication mode.
                    SPI1CON1bits.MCLKEN = 0;                        // Peripheral clock is used
                    SPI1CON1bits.ENHBUF = 0;                        // Normal buffer modes
                    SPI1CON2Lbits.WLENGTH = (bit_exchange_size-1);	// Variable world length set
                    SPI1BRGLbits.BRG = divisor;                     // Set baud rate 
                    SPI1CON1bits.SPIEN = 1;                         // enable SPI port, clear status                    
                }
                break;               
                case 1:                                             // SPI 1
                {
                    spi_ptr->SPIxBUFL = (uint16_t*)&SPI2BUFL;       // Set pointer to SPI1 buffer
                    spi_ptr->SPIxSTATUS = (uint16_t*)&SPI2STATL;    // Set pointer to SPI1 status register                    
                    SPI2CON1bits.SPIEN = 0;							// Disable SPI port, cant change setting while enabled
        			SPI2CON1bits.SPISIDL = 0;                       // Continue module operation in Idle mode 
                    SPI2STATLbits.SPIROV = 0;                       // Clear receive overrun
                    IFS2bits.SPI2IF = 0;                            // Clear interrupt flag
                    IEC2bits.SPI2IE = 0;                            // Disable interrupt
                    SPI2IMSKLbits.SRMTEN = 1;                       // Enable interrupt mask for shift register empty
                    SPI2CON1Hbits.IGNROV = 1;                       // Ignore receive overrun
                    SPI2CON1Hbits.IGNTUR = 1;                       // Ignore transmit underrun
                    SPI2CON1Hbits.AUDEN = 0;                        // Audio protocol disabled
                    SPI2CON1Hbits.SPISGNEXT = 0;                    // Data from RX not sign extended
                    SPI2CON1bits.DISSCK = 0;                        // Internal SPIx clock is enabled
                    SPI2CON1bits.DISSDO = 0;                        // SDOx pin is controlled by the module
                    SPI2CON1bits.DISSDI = 0;                        // SDIx pin is controlled by the modules
                    SPI2CON1bits.MODE16 = 0;                        // set in 16-bit mode, clear in 8-bit mode
                    SPI2CON1bits.MODE32 = 0;                        // set in 32-bit mode, clear in 16/8-bit mode
                    SPI2CON1bits.SMP = 0;                           // Input data sampled at middle of data output time
                    switch (mode)
                    {
                        case SPI_MODE_0:
                        {
                            SPI2CON1bits.CKP = 0;                   // Clock idle low
                            SPI2CON1bits.CKE = 1;                   // Transmit on transition from active clock state to Idle clock state
                        }
                        break;
                        case SPI_MODE_1:
                        {
                            SPI2CON1bits.CKP = 0;                   // Clock idle low
                            SPI2CON1bits.CKE = 0;                   // Transmit on transition from Idle clock state to active clock state
                        }
                        break;
                        case SPI_MODE_2:
                        {
                            SPI2CON1bits.CKP = 1;                   // Clock idle high
                            SPI2CON1bits.CKE = 1;                   // Transmit on transition from active clock state to Idle clock state
                        }
                        break;
                        case SPI_MODE_3:
                        {
                            SPI2CON1bits.CKP = 1;                   // Clock idle high
                            SPI2CON1bits.CKE = 0;                   // Transmit on transition from Idle clock state to active clock state
                        }
                        break;
                    };
                    SPI2CON1bits.MSTEN = 1;                         // 1 =  Master mode; 0 =  Slave mode
                    SPI2CON1bits.MCLKEN = 0;                        // Peripheral clock is used
                    SPI2CON1bits.ENHBUF = 0;                        // Normal buffer mode
                    SPI2CON2Lbits.WLENGTH = (bit_exchange_size-1);	// Variable world length set
                    SPI2BRGLbits.BRG = divisor;                     // Set baud rate
                    SPI2CON1bits.SPIEN = 1; 						// Enable SPI port, clear status
                }
                break;
                case 2:                                             // SPI 2
                {
                    spi_ptr->SPIxBUFL = (uint16_t*)&SPI3BUFL;       // Set pointer to SPI2 buffer
                    spi_ptr->SPIxSTATUS = (uint16_t*)&SPI3STATL;    // Set pointer to SPI2 status register 
                    SPI3CON1bits.SPIEN = 0;                         // Disable SPI port, cant change setting while enabled
                    SPI3CON1bits.SPISIDL = 0;                       // Continue module operation in Idle mode 
                    SPI3STATLbits.SPIROV = 0;                       // Clear receive overrun
                    IFS5bits.SPI3IF = 0;                            // Clear interrupt flag
                    IEC5bits.SPI3IE = 0;                            // Disable interrupt
                    SPI3IMSKLbits.SRMTEN = 1;                       // Enable interrupt mask for shift register empty
                    SPI3CON1Hbits.IGNROV = 1;                       // Ignore receive overrun
                    SPI3CON1Hbits.IGNTUR = 1;                       // Ignore transmit underrun
                    SPI3CON1Hbits.AUDEN = 0;                        // Audio protocol disabled
                    SPI3CON1Hbits.SPISGNEXT = 0;                    // Data from RX not sign extended 
                    SPI3CON1bits.DISSCK	= 0;                        // Internal SPIx clock is enabled
                    SPI3CON1bits.DISSDO	= 0;                        // SDOx pin is controlled by the module
                    SPI3CON1bits.DISSDI = 0;                        // SDIx pin is controlled by the module
                    SPI3CON1bits.MODE16 = 0;                        // set in 16-bit mode, clear in 8-bit mode
                    SPI3CON1bits.MODE32 = 0;                        // set in 32-bit mode, clear in 16/8-bit mode
                    SPI3CON1bits.SMP = 0;                           // Input data sampled at middle of data output time
                    switch (mode)
                    {
                        case SPI_MODE_0:
                        {
                            SPI3CON1bits.CKP = 0;                   // Clock idle low
                            SPI3CON1bits.CKE = 1;                   // Transmit on transition from active clock state to Idle clock state
                        }
                        break;
                        case SPI_MODE_1:
                        {
                            SPI3CON1bits.CKP = 0;                   // Clock idle low
                            SPI3CON1bits.CKE = 0;                   // Transmit on transition from Idle clock state to active clock state
                        }
                        break;
                        case SPI_MODE_2:
                        {
                            SPI3CON1bits.CKP = 1;                   // Clock idle high
                            SPI3CON1bits.CKE = 1;                   // Transmit on transition from active clock state to Idle clock state
                        }
                        break;
                        case SPI_MODE_3:
                        {
                            SPI3CON1bits.CKP = 1;                   // Clock idle high
                            SPI3CON1bits.CKE = 0;                   // Transmit on transition from Idle clock state to active clock state
                        }
                        break;
                    };
                    SPI3CON1bits.MSTEN = 1;                         // 1 =  Master mode; 0 =  Slave mode... based on your communication mode.
                    SPI3CON1bits.MCLKEN = 0;                        // Peripheral clock is used
                    SPI3CON1bits.ENHBUF = 0;                        // Normal buffer modes
                    SPI3CON2Lbits.WLENGTH = (bit_exchange_size-1);	// Variable world length set
                    SPI3BRGLbits.BRG = divisor;                     // Set Baud Rate 
                    SPI3CON1bits.SPIEN = 1;                         // enable SPI port, clear status                    
                }
                break;                
            };
			spi_ptr->inuse = 1;										// The handle is now in use
			spi = spi_ptr;											// Return SPI handle
		#else														// COMPILING FOR LINUX
            uint8_t spi_mode = mode;
            char buf[256] = { 0 };
			sprintf(&buf[0], "/dev/spidev0.%c", (char)(0x30 + spi_devicenum));
			int fd = open(&buf[0], O_RDWR);							// Open the SPI device
			if (fd >= 0)											// SPI device opened correctly
			{
				spi_ptr->spi_fd = fd;								// Hold the file device to SPI
				if (ioctl(spi_ptr->spi_fd, SPI_IOC_WR_MODE, &spi_mode) >= 0 &&
					ioctl(spi_ptr->spi_fd, SPI_IOC_RD_MODE, &spi_mode) >= 0 &&
					ioctl(spi_ptr->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_ptr->spi_bitsPerWord) >= 0 &&
					ioctl(spi_ptr->spi_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_ptr->spi_bitsPerWord) >= 0 &&
					ioctl(spi_ptr->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_ptr->spi_speed) >= 0 &&
					ioctl(spi_ptr->spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_ptr->spi_speed) >= 0)
				{
					spi_ptr->inuse = 1;								// The handle is now in use
					spi = spi_ptr;									// Return SPI handle
				}
			}
		#endif

	}
	return(spi);													// Return SPI handle result			
}

/*-[ SpiClosePort ]---------------------------------------------------------}
. Given a valid SPI handle the access is released and the handle freed.
. RETURN: true for success, false for any failure
.--------------------------------------------------------------------------*/
bool SpiClosePort (SPI_HANDLE spiHandle)
{
	if (spiHandle && spiHandle->inuse)								// SPI handle valid and SPI handle is in use
	{
		if (spiHandle->uselocks) Lock_Done(&spiHandle->lock);		// Done with SPI lock
		#ifdef __XC16												// COMPILING FOR PIC24F
		#else														// COMPILING FOR LINUX
		close(spiHandle->spi_fd);									// Close the spi handle
		#endif	
		spiHandle->spi_fd = 0;										// Zero the SPI handle
		spiHandle->spi_num = 0;										// Zero SPI handle number
		spiHandle->inuse = 0;										// The SPI handle is now free
		return true;												// Return success
	}
	return false;													// Return failure
}

/*-[ SpiAssignCS ]----------------------------------------------------------}
.  On the PIC SPI implementation the CS line is just a general IO pin it is
.  not part of the SPI itself. This call is used to assigned the IO pin port
.  and bitmask and so is only valid for PIC compilation.
.  RETURN: true for success, false for any failure
.--------------------------------------------------------------------------*/
#ifdef __XC16
bool SpiAssignCS (SPI_HANDLE spiHandle, volatile uint16_t* IOPin, uint16_t bitmask)
{
	if (spiHandle && spiHandle->inuse)								// SPI handle valid and SPI handle is in use  
    {
        spiHandle->SPIxCS = IOPin;                                  // Hold pointer to IO pin
        spiHandle->SPIxCSBIT = bitmask;                             // Hold bitmask 
        return true;                                                // Return success
    }
    return false;                                                   // Return failure
}
#endif

/*-[ SpiWriteAndRead ]------------------------------------------------------}
. Given a valid SPI handle and valid data pointers the call will send and
. receive data to and from the buffer pointers. As the write occurs before
. the read the buffer pointers can be the same buffer space.
. RETURN: >= 0 transfer count for success, < 0 for any error  
.--------------------------------------------------------------------------*/
int SpiWriteAndRead (SPI_HANDLE spiHandle, uint8_t* TxData, uint8_t* RxData, uint16_t Length, bool LeaveCsLow)
{
	int retVal = -1;												// Preset -1
	if (spiHandle && spiHandle->inuse)								// SPI handle valid and SPI handle is in use
	{
		if (spiHandle->uselocks) Lock(&spiHandle->lock);			// Apply access lock
		#ifdef __XC16												// COMPILING FOR PIC24F
		uint16_t i;
        if (spiHandle->SPIxCS)                                      // CS pin assigned
        {
            *spiHandle->SPIxCS &= ~spiHandle->SPIxCSBIT;            // Take port low
        }
		for (i = 0; i < Length; i++)								// For each character
		{
			uint8_t a;
            if (TxData == 0) *spiHandle->SPIxBUFL = 0;              // Write zero if no transmit data buffer
                else *spiHandle->SPIxBUFL = TxData[i];          	// Write data to transmit buffer                
            while ((*spiHandle->SPIxSTATUS & 0x0080) == 0) {};      // Wait for transmit buffer empty flag
            if (RxData) RxData[i] = *spiHandle->SPIxBUFL;			// Read the byte received
               else a = *spiHandle->SPIxBUFL;   					// Read received data           
		}
        if (!LeaveCsLow && spiHandle->SPIxCS)                       // Not leaving low and CS pin assigned                    
        {
            *spiHandle->SPIxCS |= spiHandle->SPIxCSBIT;             // Take CS port high
        }
		#else														// COMPILING FOR LINUX
		struct spi_ioc_transfer spi = { 0 };
		spi.tx_buf = (unsigned long)TxData;							// transmit from "data"
		spi.rx_buf = (unsigned long)RxData;							// receive into "data"
		spi.len = Length;											// length of data to tx/rx
		spi.delay_usecs = 0;										// Delay before sending
		spi.speed_hz = spiHandle->spi_speed;						// Speed for transfer
		spi.bits_per_word = spiHandle->spi_bitsPerWord;				// Bits per exchange
		spi.cs_change = LeaveCsLow;									// 0=Set CS high after a transfer, 1=leave CS set low
		retVal = ioctl(spiHandle->spi_fd, SPI_IOC_MESSAGE(1), &spi);// Execute exchange
		#endif
		if (spiHandle->uselocks) UnLock(&spiHandle->lock);			// Release access lock
	}
	return retVal;													// Return result
}

/*-[ SpiWriteBlockRepeat ]--------------------------------------------------}
. Given a valid SPI handle and valid data pointers the call will send the
. data block repeat times. It is used to speed up things like writing to SPI
. LCD screen with areas a fixed colour.
. RETURN: >= 0 blocks transfered for success, < 0 for any error  
.--------------------------------------------------------------------------*/
int SpiWriteBlockRepeat (SPI_HANDLE spiHandle, uint8_t* TxBlock, uint16_t TxBlockLen, uint32_t Repeats, bool LeaveCsLow)
{
	int retVal = -1;												// Preset -1
	if (spiHandle && TxBlock && spiHandle->inuse)					// SPI handle and TxBlock valid and SPI handle is in use
	{
		if (spiHandle->uselocks) Lock(&spiHandle->lock);			// Apply access lock
		#ifdef __XC16												// COMPILING FOR PIC24F
        if (spiHandle->SPIxCS)                                      // CS pin assigned
        {
            *spiHandle->SPIxCS &= ~spiHandle->SPIxCSBIT;            // Take port low
        }
        uint32_t j;
        for (j = 0; j < Repeats; j++)                               // For each block repeat
        {
            uint16_t i;
            for (i = 0; i < TxBlockLen; i++)						// For each block byte
            {
                *spiHandle->SPIxBUFL = TxBlock[i];                  // Write block data to transmit buffer                
                while ((*spiHandle->SPIxSTATUS & 0x0080) == 0) {};  // Wait for transmit buffer empty flag
                *spiHandle->SPIxBUFL;               				// Read received data and throw          
            }
        }
        retVal = j;                                                 // Return repeats done
        if (!LeaveCsLow && spiHandle->SPIxCS)                       // Not leaving low and CS pin assigned                    
        {
            *spiHandle->SPIxCS |= spiHandle->SPIxCSBIT;             // Take CS port high
        }
		#else														// COMPILING FOR LINUX
		struct spi_ioc_transfer spi = { 0 };
		spi.tx_buf = (unsigned long)TxBlock;						// transmit from "data"
		spi.rx_buf = (unsigned long)0;          					// receive into "data"
		spi.len = TxBlockLen;										// length of data to tx/rx
		spi.delay_usecs = 0;										// Delay before sending
		spi.speed_hz = spiHandle->spi_speed;						// Speed for transfer
		spi.bits_per_word = spiHandle->spi_bitsPerWord;				// Bits per exchange
		spi.cs_change = LeaveCsLow;									// 0=Set CS high after a transfer, 1=leave CS set low
        retVal = 0;                                                 // Zero retVal 
        uint32_t j;
        for (j = 0; j < Repeats && retVal == TxBlockLen; j++)       // For each block repeat
        {    
            retVal = ioctl(spiHandle->spi_fd, SPI_IOC_MESSAGE(1), &spi);// Execute exchange
        }
        retVal = j;                                                 // Return block count
		#endif
		if (spiHandle->uselocks) UnLock(&spiHandle->lock);			// Release access lock
	}
	return retVal;													// Return result
}
