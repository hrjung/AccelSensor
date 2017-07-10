/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/** ============================================================================
 *  @file       Board.h
 *
 *  @brief      CC2650EM_5XD Board Specific header file.
 *              The project options should point to this file if this is the
 *              CC2650EM you are developing code for.
 *
 *  The CC2650 header file should be included in an application as follows:
 *  @code
 *  #include <Board.h>
 *  @endcode
 *
 *  ============================================================================
 */
#ifndef __CC2650EM_5XD_H__
#define __CC2650EM_5XD_H__

#ifdef __cplusplus
extern "C" {
#endif

/** ============================================================================
 *  Symbol by generic Board.c to include the correct kit specific Board.c
 *  ==========================================================================*/
#define CC2650EM_5XD

/** ============================================================================
 *  Includes
 *  ==========================================================================*/
#include <ti/drivers/PIN.h>
#include <driverlib/ioc.h>

/** ============================================================================
 *  Externs
 *  ==========================================================================*/
extern const PIN_Config BoardGpioInitTable[];

/** ============================================================================
 *  Defines
 *  ==========================================================================*/

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>                <pin mapping>
 */
/* Leds */
#define Board_LED_ON                        0 /* LEDs on CC2650 are active high */
#define Board_LED_OFF                       1
#define Board_DK_LED1                       PIN_UNASSIGNED
#define Board_DK_LED2                       PIN_UNASSIGNED
#define Board_DK_LED3                       PIN_UNASSIGNED          /* P1.2  */
#define Board_DK_LED4                       IOID_12          /* P1.4  */

/* UART Board */
#define Board_UART_RX                       IOID_1          /* P1.7  */
#define Board_UART_TX                       IOID_0          /* P1.9  */
#define Board_UART_CTS                      PIN_UNASSIGNED
#define Board_UART_RTS                      PIN_UNASSIGNED

/* SPI Board */
#define Board_SPI0_MISO                     IOID_7         /* P1.20 */
#define Board_SPI0_MOSI                     IOID_8         /* P1.18 */
#define Board_SPI0_CLK                      IOID_3         /* P1.16 */
#define Board_SPI0_CSN                      PIN_UNASSIGNED
/* Power Board */
//#define Board_3V3_EN                        IOID_14         /* P1.15 */

//hrjung add for sensor board
#define Board_GPIO_CSN						IOID_4
#define Board_GPIO_5						IOID_5      // can be test bit
#define Board_GPIO_6						IOID_6		// test bit
#define	Board_GPIO_9						IOID_9		// INT2
#define	Board_GPIO_10						IOID_10		// INT1
#define Board_GPIO_11						IOID_11		// External power
/** ============================================================================
 *  Instance identifiers
 *  ==========================================================================*/
/* Generic SPI instance identifiers */
#define Board_SPI0                  CC2650DK_5XD_SPI0
/* Generic UART instance identifiers */
#define Board_UART                  CC2650DK_5XD_UART0
/* Generic Crypto instance identifiers */
#define Board_CRYPTO                CC2650DK_5XD_CRYPTO0

/** ============================================================================
 *  Number of peripherals and their names
 *  ==========================================================================*/

/*!
 *  @def    CC2650DK_5XD_CryptoName
 *  @brief  Enum of Crypto names on the CC2650 dev board
 */
typedef enum CC2650DK_5XD_CryptoName {
    CC2650DK_5XD_CRYPTO0 = 0,
    CC2650DK_5XD_CRYPTOCOUNT
} CC2650DK_5XD_CryptoName;

/*!
 *  @def    CC2650DK_5XD_SPIName
 *  @brief  Enum of SPI names on the CC2650 dev board
 */
typedef enum CC2650DK_5XD_SPIName {
    CC2650DK_5XD_SPI0 = 0,
    CC2650DK_5XD_SPICOUNT
} CC2650DK_5XD_SPIName;

/*!
 *  @def    CC2650DK_5XD_UARTName
 *  @brief  Enum of UARTs on the CC2650 dev board
 */
typedef enum CC2650DK_5XD_UARTName {
    CC2650DK_5XD_UART0 = 0,
    CC2650DK_5XD_UARTCOUNT
} CC2650DK_5XD_UARTName;

/*!
 *  @def    CC2650DK_5XD_UdmaName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC2650DK_5XD_UdmaName {
    CC2650DK_5XD_UDMA0 = 0,
    CC2650DK_5XD_UDMACOUNT
} CC2650DK_5XD_UdmaName;

#ifdef __cplusplus
}
#endif

#endif /* __CC2650EM_H__ */
