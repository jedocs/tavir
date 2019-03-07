#include <p24Fxxxx.h>
#include <stdlib.h>

#define GetSystemClock()    (8000000ul)
#define GetPeripheralClock()    (GetSystemClock() / 2)
#define GetInstructionClock()   (GetSystemClock() / 2)

typedef unsigned char BYTE; // 8-bit unsigned
typedef unsigned short int WORD; // 16-bit unsigned
//typedef unsigned long		DWORD;
typedef signed short int SHORT; // 16-bit signed
typedef void VOID;
typedef signed int INT;
typedef signed char INT8;
typedef signed short int INT16;
typedef signed long int INT32;
typedef signed long long INT64;

typedef unsigned int UINT;
typedef unsigned char UINT8;
typedef unsigned short int UINT16;
typedef unsigned long int UINT32; // other name for 32-bit integer
typedef unsigned long long UINT64;

//typedef enum _BOOL_ {
  //  FALSE = 0, TRUE
//} BOOL; /* Undefined size */

#define USE_AND_OR /* To enable AND_OR mask setting */

#define XCHAR   char

//****************************************************************

#define LED_TRIS 	TRISAbits.TRISA2
#define BUZZER_TRIS 	TRISBbits.TRISB15
#define LED 		LATAbits.LATA2
#define BUZZER 		LATBbits.LATB15

#define START_BUTTON    PORTAbits.RA5
#define STOP_BUTTON     PORTBbits.RB0
#define L_JOY_BUTTON    PORTBbits.RB5
#define R_JOY_BUTTON    PORTBbits.RB6
#define MOTOR           LATAbits.LATA7



//******************* RF
#define FREQ_Band       0x0020                                 //868MHz
#define FREQ_start      710
#define FREQ_step       90
#define FREQ_maxid      13
#define MAX_FREQ        "13"
#define RF_DEV      	90

#define SDOdir          TRISBbits.TRISB13
#define SCKdir          TRISBbits.TRISB11
#define SDIdir          TRISBbits.TRISB10

#define SPI_SDO         LATBbits.LATB13
#define SPI_SCK         LATBbits.LATB11
#define SPI_SDI         PORTBbits.RB10
#define NSEL            PORTBbits.RB2                              // chip select, active low output
#define NFFS            PORTAbits.RA3                              // rx fifo select, active low output
#define NIRQ		PORTBbits.RB7
#define FFIT		PORTAbits.RA6

// Application constants:

#define PAYLOAD_LEN 20                                         // TX (RX) payload size
#define PACKET_LEN  PAYLOAD_LEN + 4                            // TX (RX) packet size ([AAAA]AA2DD4 + PL + payload)
#define TXBUF_LEN   PACKET_LEN + 2                             // TX buffer size (TX packet + 2 bytes dummy)

void Delay10us(UINT32 tenMicroSecondCounter);
void DelayMs(UINT16 ms);

union int_char {
    int int_;
    char char_ [2];
};


///////////////////////////////////////////////////////////6

#define RF12_MAXDATA    128
/// Max transmit/receive buffer: 4 header + data + 2 crc bytes
#define RF_MAX          (RF12_MAXDATA + 6)

#define RF12_433MHZ     1
#define RF12_868MHZ     2
#define RF12_915MHZ     3

#define RF12_HDR_IDMASK      0x7F
#define RF12_HDR_ACKCTLMASK  0x80
#define RF12_DESTID   (rf12_hdr1 & RF12_HDR_IDMASK)
#define RF12_SOURCEID (rf12_hdr2 & RF12_HDR_IDMASK)

// shorthands to simplify sending out the proper ACK when requested
#define RF12_WANTS_ACK ((rf12_hdr2 & RF12_HDR_ACKCTLMASK) && !(rf12_hdr1 & RF12_HDR_ACKCTLMASK))

/// Shorthand for RF12 group byte in rf12_buf.
#define rf12_grp        rf12_buf[0]
/// pointer to 1st header byte in rf12_buf (CTL + DESTINATIONID)
#define rf12_hdr1        rf12_buf[1]
/// pointer to 2nd header byte in rf12_buf (ACK + SOURCEID)
#define rf12_hdr2        rf12_buf[2]

/// Shorthand for RF12 length byte in rf12_buf.
#define rf12_len        rf12_buf[3]
/// Shorthand for first RF12 data byte in rf12_buf.
#define rf12_data       (rf12_buf + 4)

// RF12 command codes
#define RF_RECEIVER_ON  0x82DD
#define RF_XMITTER_ON   0x823D
#define RF_IDLE_MODE    0x820D
#define RF_SLEEP_MODE   0x8205
#define RF_WAKEUP_MODE  0x8207
#define RF_TXREG_WRITE  0xB800
#define RF_RX_FIFO_READ 0xB000
#define RF_WAKEUP_TIMER 0xE000

//RF12 status bits
#define RF_LBD_BIT      0x0400
#define RF_RSSI_BIT     0x0100

//#define DEBUG

// transceiver states, these determine what to do with each interrupt
enum {
    TXCRC1, TXCRC2, TXTAIL, TXDONE, TXIDLE, TXRECV, TXPRE1, TXPRE2, TXPRE3, TXSYN1, TXSYN2, TXLEN
};

#define ACK_TIME                 15000        //R of ms to wait for an ack (usually between 600ms and 1300ms)
#define CLIENT_MOTEINO_NODE          1
#define CLIENT_MBED_NODE             2

#define SERVER_MOTEINO_NODE         10
#define SERVER_MBED_NODE            11

#define NETWORD_ID                   5 //GROUP / NETWORK ID

#define MBED_TO_MBED                 0
#define MBED_TO_ARDUINO              1
#define ARDUINO_TO_MBED              2

void SendStart(unsigned char sendLen);


