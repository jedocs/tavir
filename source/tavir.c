// 11,28ms

#include <p24Fxxxx.h>
#include<timer.h>

// PIC24F16KA102 Configuration Bit Settings

int FBS __attribute__((space(prog), address(0xF80000))) = 0xF;
//_FBS(
//    BWRP_OFF &           // Table Write Protect Boot (Boot segment may be written)
//    BSS_OFF              // Boot segment Protect (No boot program Flash segment)
//);
int FGS __attribute__((space(prog), address(0xF80004))) = 0x3;
//_FGS(
//    GWRP_OFF &           // General Segment Code Flash Write Protection bit (General segment may be written)
//    GCP_OFF              // General Segment Code Flash Code Protection bit (No protection)
//);
int FOSCSEL __attribute__((space(prog), address(0xF80006))) = 0x7;
//_FOSCSEL(
//    FNOSC_FRCDIV &       // Oscillator Select (8 MHz FRC oscillator with divide-by-N (FRCDIV))
//    IESO_OFF             // Internal External Switch Over bit (Internal External Switchover mode disabled (Two-Speed Start-up disabled))
//);
int FOSC __attribute__((space(prog), address(0xF80008))) = 0xDB;
//_FOSC(
//    POSCMOD_NONE &       // Primary Oscillator Configuration bits (Primary oscillator disabled)
//    OSCIOFNC_ON &        // CLKO Enable Configuration bit (CLKO output disabled; pin functions as port I/O)
//    POSCFREQ_HS &        // Primary Oscillator Frequency Range Configuration bits (Primary oscillator/external clock input frequency greater than 8 MHz)
//    SOSCSEL_SOSCLP &     // SOSC Power Selection Configuration bits (Secondary oscillator configured for low-power operation)
//    FCKSM_CSDCMD         // Clock Switching and Monitor Selection (Both Clock Switching and Fail-safe Clock Monitor are disabled)
//);
int FWDT __attribute__((space(prog), address(0xF8000A))) = 0x5F;
//_FWDT(
//    WDTPS_PS32768 &      // Watchdog Timer Postscale Select bits (1:32,768)
//    FWPSA_PR128 &        // WDT Prescaler (WDT prescaler ratio of 1:128)
//    WINDIS_OFF &         // Windowed Watchdog Timer Disable bit (Standard WDT selected; windowed WDT disabled)
//    FWDTEN_OFF           // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
//);
int FPOR __attribute__((space(prog), address(0xF8000C))) = 0x7B;
//_FPOR(
//    BOREN_BOR3 &         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware; SBOREN bit disabled)
//    PWRTEN_ON &          // Power-up Timer Enable bit (PWRT enabled)
//    I2C1SEL_PRI &        // Alternate I2C1 Pin Mapping bit (Default location for SCL1/SDA1 pins)
//    BORV_V18 &           // Brown-out Reset Voltage bits (Brown-out Reset set to lowest voltage (1.8V))
//    MCLRE_OFF            // MCLR Pin Enable bit (RA5 input pin enabled; MCLR disabled)
//);
int FICD __attribute__((space(prog), address(0xF8000E))) = 0xC3;
//_FICD(
//    ICS_PGx1             // ICD Pin Placement Select bits (PGC1/PGD1 are used for programming and debugging the device)
//);
int FDS __attribute__((space(prog), address(0xF80010))) = 0xFF;
//_FDS(
//    DSWDTPS_DSWDTPSF &   // Deep Sleep Watchdog Timer Postscale Select bits (1:2,147,483,648 (25.7 Days))
//    DSWDTOSC_LPRC &      // DSWDT Reference Clock Select bit (DSWDT uses LPRC as reference clock)
//    RTCOSC_SOSC &        // RTCC Reference Clock Select bit (RTCC uses SOSC as reference clock)
//    DSBOREN_ON &         // Deep Sleep Zero-Power BOR Enable bit (Deep Sleep BOR enabled in Deep Sleep)
//    DSWDTEN_ON           // Deep Sleep Watchdog Timer Enable bit (DSWDT enabled)
//);

#include "..\include\define.h"
#include "PwrMgnt.h"
#include "Rtcc.h"

int key;
//char join[12] = "csatlakozas";

extern volatile unsigned char rf12_buf[];
volatile unsigned char counter = 0;

// rf input/output buffers:
extern unsigned char RxPacket[]; // Receive data puffer (payload only)
extern unsigned char TxPacket[]; // Transmit data puffer
extern unsigned char RxPacketLen;
extern BOOL hasPacket;
extern WORD rcrc, ccrc;
BOOL kuld;
//unsigned char mp, ccx;

//unsigned char rxstatus = 9;

//*************************************************************************************
//
//		Init
//
//*************************************************************************************

void init(void) {

    BUZZER = 1; //PIEZO
    BUZZER_TRIS = 0;

    LED = 0; //LED
    LED_TRIS = 0;

    // RF =====================

    TRISA = 0b0000000001110011;
    TRISB = 0b0101011111111011;

    TRISBbits.TRISB2 = 0; // NSEL  !      	PORTBbits.RB2                              // chip select, active low output
    TRISAbits.TRISA3 = 1; // NFFS !       	                              // rx fifo select, active low output
    TRISBbits.TRISB7 = 1; // NIRQ

    NSEL = 1; // nSEL inactive
    //    NFFS = 1; // nFFS inactive

    CNPU1bits.CN4PUE = 1; //RB0	key input
    CNPU2bits.CN24PUE = 1; //RB6	key input
    CNPU2bits.CN27PUE = 1; //RB5	key input

    LATBbits.LATB0 = 1; //csak így mûködik a pullup-os bemenet...?

    CNPD1 = 0; // pulldowns disable
    CNPD2 = 0; // pulldowns disable

    //================= SPI setup ======================================

    SPI1CON1 = 0b0000000100111111; // SMP:0, CKP:0, CKE:1, 8bit mode
    SPI1CON2 = 0b0000000000000000; //enhanced buffer mode disable
    SPI1STATbits.SPIEN = 1; /* Enable/Disable the spi module */

    REFOCON = 0b0000110100000000; //Reference Oscillator Output disabled; ref osc divisor Base clock value divided by 128 (HP03 MCLK)

    AD1PCFG = 0b1110101111010100; // AD port config register 1 -> port is dig. I/O; 0 -> analog input
    AD1CSSL = 0b0001010000101011;
    AD1CON1bits.FORM = 0b10; //fractional
    AD1CON1bits.SSRC = 0b111;
    AD1CON3 = 0x9F00; // Sample time
    AD1CON2bits.SMPI = 0b0101; // Set AD1IF after every 6 samples
    AD1CON2bits.CSCNA = 1; //enable scanning
    AD1CON1bits.ADON = 1; // turn ADC ON

    PR1 = 25000;
    T1CON = 0b0000000000010000;
    INTCON2bits.INT0EP = 1; // INT2 lefutó él
    IFS0bits.INT0IF = 0;
    IEC0bits.INT0IE = 1;

    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    T1CONbits.TON = 1;

}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {

        rf12_buf[0] = 0b01010101;
        rf12_buf[1] = 0b10101010;
        rf12_buf[2] = counter++;

    LED = 1;
    /*
        AD1CHSbits.CH0SA = 0;
        IFS0bits.AD1IF = 0; // clear ADC interrupt flag
        AD1CON1bits.ASAM = 1; // auto start sampling for 31Tad
        while (!IFS0bits.AD1IF); // conversion done?
        AD1CON1bits.ASAM = 0; // yes then stop sample/convert

        rf12_buf[0] = (char) (ADC1BUF0 >> 8);
        rf12_buf[1] = (char) (ADC1BUF1 >> 8);
        rf12_buf[2] = (char) (ADC1BUF2 >> 8);
        rf12_buf[3] = (char) (ADC1BUF3 >> 8);
        rf12_buf[4] = (char) (ADC1BUF4 >> 8);
        rf12_buf[5] = (char) (ADC1BUF5 >> 8);

        rf12_buf[6] = counter++;
        // rf12_buf[6] = 0;
        // if (!L_JOY_BUTTON) rf12_buf[6] += 1;
        // if (!R_JOY_BUTTON) rf12_buf[6] += 2;
        // if (!STOP_BUTTON) rf12_buf[6] += 4;

        SendStart(8);

     */

    SendStart(3);

    LED = 0;
    IFS0bits.T1IF = 0;
}

//*************************************************************************
//
// Int on change interrupt
//
//*************************************************************************

void __attribute__((interrupt, shadow, auto_psv)) _CNInterrupt(void) {
   // kuld = TRUE;

    IFS1bits.CNIF = 0;
}


//************************************************************************
//
//	Main
//
//************************************************************************

int main(void) {

    init();

    RF_init(SERVER_MBED_NODE, RF12_868MHZ, NETWORD_ID, 0, 0x08);

    while (1) {

        MOTOR = R_JOY_BUTTON;
        REFOCONbits.ROEN = !L_JOY_BUTTON;
    }
}


