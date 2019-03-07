#include "..\include\define.h"

volatile unsigned char rf12_buf[RF_MAX]; // recv/xmit buf, including hdr & crc bytes
volatile unsigned char nodeID; // address of this node
volatile unsigned char networkID; // network group
volatile unsigned char rxfill; // number of data bytes in rf12_buf
volatile char rxstate; // current transceiver state
volatile unsigned int rf12_crc; // running crc value
long rf12_seq;

extern unsigned char rxstatus;
// rf input/output buffers:
unsigned char RxPacket[PAYLOAD_LEN]; // Receive data puffer (payload only)
unsigned char TxPacket[TXBUF_LEN]; // Transmit data puffer
unsigned char TxPacketPtr; // Next byte to send
unsigned char RxPacketPtr; // Next byte to send
unsigned char TX_len;
//BOOL hasPacket;

unsigned int crc16_update(unsigned int crc, unsigned char data) {
    int i;

    crc ^= data;
    for (i = 0; i < 8; ++i) {
        if (crc & 1)
            crc = (crc >> 1) ^ 0xA001;
        else
            crc = (crc >> 1);
    }
    return crc;
}

/********************************SPIPut***********************************/
void SPIPut(unsigned char v) {
    unsigned char i;
    IFS0bits.SPI1IF = 0;
    i = SPI1BUF;
    SPI1BUF = v;
    while (IFS0bits.SPI1IF == 0) {
    }
}

//*************************byte********************************************

unsigned char byte(unsigned char out) {
    SPIPut(out);
    return SPI1BUF;
}

/***********************SPIGet******************************************/

unsigned char SPIGet(void) {
    SPIPut(0x00);
    return SPI1BUF;
}

/************************XFER********************************************/

unsigned int xfer(unsigned int cmd) {
    NSEL = 0;
    unsigned int reply = byte(cmd >> 8) << 8;
    reply |= byte(cmd);
    NSEL = 1;
    return reply;
}

// Send SPI command to the rf chip                              // send 16-bit SPI command with NSEL control

void send_cmd(unsigned int spicmd) {
    NSEL = 0;
    SPIPut(spicmd >> 8);
    SPIPut(spicmd);
    NSEL = 1;
}

//==============================================================
// initialization
//==============================================================

void RF_init(unsigned char nodeid, unsigned char freqBand, unsigned char groupid, unsigned char txPower, unsigned char airKbps) {

    SDIdir = 1;
    SDOdir = 0;
    SCKdir = 0;
    SPI_SDO = 0;
    SPI_SCK = 0;
    SPI_SDI = 1;

    //----  configuring the RF link --------------------------------

    nodeID = nodeid;
    networkID = groupid;

    send_cmd(0x0000); // initial SPI transfer added to avoid power-up problem
    send_cmd(RF_SLEEP_MODE); // DC (disable clk pin), enable lbd

    // wait until RFM12B is out of power-up reset, this takes several *seconds*
    send_cmd(RF_TXREG_WRITE); // in case we're still in OOK mode

    while (NIRQ == 0)
        send_cmd(0x0000);

    send_cmd(0x80C7 | (freqBand << 4)); // EL (ena TX), EF (ena RX FIFO), 12.0pF
    send_cmd(0xA640); // Frequency is exactly 434/868/915MHz (whatever freqBand is)
    // send_cmd(0xC600 + airKbps);         //Air transmission baud rate: 0x08= ~38.31Kbps
    send_cmd(0x94A2); // VDI,FAST,134kHz,0dBm,-91dBm
    send_cmd(0xC2AC); // AL,!ml,DIG,DQD4

    send_cmd(0xCA81); // FIFO8,1-SYNC,!ff,DR

    send_cmd(0xC483); // @PWR,NO RSTRIC,!st,!fi,OE,EN
    send_cmd(0x9850 | (txPower > 7 ? 7 : txPower)); // !mp,90kHz,MAX OUT
    send_cmd(0xCC77); // OB1, OB0, LPX, ddy, DDIT, BW0

    rxstate = TXIDLE;
    IFS0bits.INT0IF = 0;
}

//*********************************************************************
//* Function:        void HighISR(void)
// ********************************************************************

void _ISRFAST __attribute__((interrupt, auto_psv)) _INT0Interrupt(void) {

    if (IFS0bits.INT0IF) {
        unsigned char out;
        send_cmd(0x0000);

        if (rxstate < 0) {
            unsigned char pos = TX_len - 1 + rxstate++;
            out = rf12_buf[pos];
            rf12_crc = crc16_update(rf12_crc, out);
        } else {
            switch (rxstate++) {
                case TXSYN1:
                    out = 0x2d;
                    break;
                case TXSYN2:
                    out = 0xd4;
                    break;
                case TXLEN:
                    out = TX_len;
                    rxstate = -(TX_len);
                    break;
                case TXCRC1:
                    out = rf12_crc;
                    break;
                case TXCRC2:
                    out = rf12_crc >> 8;
                    break;
                case TXDONE:
                    xfer(RF_IDLE_MODE); // fall through
                    out = 0xAA;
                    break;
                default:
                    out = 0xAA;
            }
        }
        xfer(RF_TXREG_WRITE + out);

        IFS0bits.INT0IF = 0;
    }
}

void SendStart(unsigned char sendLen) {

    TX_len = sendLen;

    rf12_crc = ~0;
    rf12_crc = crc16_update(rf12_crc, TX_len);
    rxstate = TXPRE2;

    xfer(RF_XMITTER_ON); // bytes will be fed via interrupts
}
