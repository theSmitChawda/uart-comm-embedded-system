//Smit Chawda//11132022//991561339

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define _XTAL_FREQ  8000000UL

// CONFIG2
#pragma config POSCMOD = XT      // Primary Oscillator Select->XT Oscillator mode selected
#pragma config OSCIOFNC = OFF    // Primary Oscillator Output Function->OSC2/CLKO/RC15 functions as CLKO (FOSC/2)
#pragma config FCKSM = CSDCMD    // Clock Switching and Monitor->Clock switching and Fail-Safe Clock Monitor are disabled
#pragma config FNOSC = PRI       // Oscillator Select->Primary Oscillator (XT, HS, EC)
#pragma config IESO = ON         // Internal External Switch Over Mode->IESO mode (Two-Speed Start-up) enabled
// CONFIG1
#pragma config WDTPS = PS32768   // Watchdog Timer Postscaler->1:32768
#pragma config FWPSA = PR128     // WDT Prescaler->Prescaler ratio of 1:128
#pragma config WINDIS = ON       // Watchdog Timer Window->Standard Watchdog Timer enabled,(Windowed-mode is disabled)
#pragma config FWDTEN = OFF      // Watchdog Timer Enable->Watchdog Timer is disabled
#pragma config ICS = PGx2        // Comm Channel Select->Emulator/debugger uses EMUC2/EMUD2
//#pragma config COE = OFF       // Set Clip On Emulation Mode->Reset Into Operational Mode
#pragma config BKBUG = OFF       // Background Debug->Device resets into Operational mode
#pragma config GWRP = OFF        // General Code Segment Write Protect->Writes to program memory are allowed
#pragma config GCP = OFF         // General Code Segment Code Protect->Code protection is disabled
#pragma config JTAGEN = OFF      // JTAG Port Enable->JTAG port is disabled

////////////////////////////////////////////////////////////////////////////////
//Function Proto type
void SYSTEM_Initialize(void);
void OSCILLATOR_Initialize(void);
void PIN_MANAGER_Initialize(void);
void INTERRUPT_Initialize (void);
void TIMER_Initialize(void);

void delayFunc(void);
void projKingRiderMainFunction(void);
void projReverseKingRiderMainFunction(void);
void resetPattern(void);
void startTimerAndInterrupt(void);
void __attribute__ ((__interrupt__,no_auto_psv)) _T1Interrupt(void);

////////////////////////////////////////////////////////////////////////////////
//Global variables
unsigned int count1 = 200;
unsigned int count2 = 200;
unsigned int buffer_variable = 0;
char rcvByte;
float temperature =0.0f;
float potVoltage = 0.0f;

///////////////////////////////////////////////////////////////////////////////
//Program Entry Point
int main(void)
{
 // initialize the device
 SYSTEM_Initialize();
 TRISDbits.TRISD6 = 1;
 printf("#------ PROJECT 3 - Smit Chawda - 991561339 ------#");
 while (1)
 {
     char TxString[100] = "";
     
     temperature = ADC_Operate(4); //using ANI4
     temperature = (((temperature *0.00322) -0.5)/0.01);
     
     potVoltage = ADC_Operate(5);  //using ANI5
     potVoltage = (potVoltage * 0.00322);
     
     sprintf(TxString,"Temperature = %.3f and Voltage = %.3f", potVoltage, temperature);
     U2_TxString(TxString);
     
     if(PORTDbits.RD6 == 1)
     {
         projReverseKingRiderMainFunction();
     }
     else
     {
        projKingRiderMainFunction();
     }
 }     
    return -1; //Never reach here!
}

void __attribute__ ((__interrupt__,no_auto_psv)) _T1Interrupt(void)
{
    if(buffer_variable < 100)
    {
        buffer_variable++;
    }
    if(buffer_variable >= 100)
    {
        buffer_variable = 0;
//        PORTA ^= 0x80; //toggling bit 7 of register A - RA7 - LED10
        if(PORTAbits.RA7 == 0)
        {
            PORTAbits.RA7 = 1;
        }
        else {
            PORTAbits.RA7 = 0;
        }
    }
    IFS0bits.T1IF = 0;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U2RXInterrupt( void )
{
    if(IEC1bits.U2RXIE == 1)
    {
        if(IFS1bits.U2RXIF == true) //check the status flag
        {
             rcvByte = U2RXREG; //Read the received byte
        }
    }
    IFS1bits.U2RXIF = false; //reset the status flag
}

void __attribute__ ((__interrupt__,__no_auto_psv__)) _ADC1Interrupt (void)
{
    if (IFS0bits.AD1IF == 1) {
//        v = voltageValue = ADC1BUF0;
    }
    IFS0bits.AD1IF = 0;
}

void projKingRiderMainFunction(void)
{
    PORTAbits.RA0 = 1;
    delayFunc();
    PORTAbits.RA1 = 1;
    delayFunc();
    PORTAbits.RA2 = 1;
    delayFunc();
    PORTAbits.RA3 = 1;
    delayFunc();
    PORTAbits.RA4 = 1;
    delayFunc();
    PORTAbits.RA5 = 1;
    delayFunc();
    PORTAbits.RA6 = 1;
    delayFunc();
//    PORTAbits.RA7 = 1;
    delayFunc();
    resetPattern();
}

void projReverseKingRiderMainFunction(void)
{
//    PORTAbits.RA7 = 1;
    delayFunc();
    PORTAbits.RA6 = 1;
    delayFunc();
    PORTAbits.RA5 = 1;
    delayFunc();
    PORTAbits.RA4 = 1;
    delayFunc();
    PORTAbits.RA3 = 1;
    delayFunc();
    PORTAbits.RA2 = 1;
    delayFunc();
    PORTAbits.RA1 = 1;
    delayFunc();
    PORTAbits.RA0 = 1;
    delayFunc();
    resetPattern();
//    projKingRiderMainFunction();
}

void resetPattern(void)
{
    PORTA &= (0x8000);
}

void UART2_Initialize (void)
{
    /**
     * Enable the UART (bit 15) and 
     * set parity bit to "even parity" 
     * (bit 2=0, bit 1=1)
     */
    U2MODE = 0x8002;
    
    U2STA = 0x0000; //resetting U2STA register
    /**
     * Clearing the U2 transmit register (<- lower half of the register)
     * to prepare for transmission
     */
    U2TXREG = 0x0000;
    
    /** 
     * BaudRate = 9600 = 0x0019;
     * Frequency = 4 MHz;
     */
    U2BRG = 0x0019; 
    
    /**
     * Enabling Rx Interrupt 
     * to detect receving information
     */
    IEC1bits.U2RXIE = 1; 
    
    /**
     * Enabling transmission by
     * setting the enable transmit bit = 1
     */
    U2STAbits.UTXEN = 1; 
    
    /**
     * We don't need Tx Interrupts so
     * we disable it
     */
    IEC1bits.U2TXIE = 0; //Tx interrupt disabled
    
    /**
     * Setting the interrupt flag bits for 
     * Rx and Tx interrupts
     */
    IFS1bits.U2RXIF = 0;
    IFS1bits.U2TXIF = 0;
    
    /**
     * Setting the priority of 
     * the Rx interrupts to 5
     */
    IPC7bits.U2RXIP2 = 1; 
    IPC7bits.U2RXIP1 = 0;
    IPC7bits.U2RXIP0 = 1;
}

void U2_TxByte(char value)
{
    while (!U2STAbits.TRMT);
    U2TXREG = value; 
    
    //delay
    Nop(); 
    Nop();
    Nop();
}

void U2_TxString(char *str)
{
    char nextChar;
    while(*str != '\0')
    {
        nextChar = *str++;
        Nop();
        Nop();

        U2_TxByte(nextChar);
    }
    U2_TxByte(0x0d); //CR - print \r
    U2_TxByte(0x0a); //LF - print \n
}

void ADC_Initialize ( void )
{
    
    /**
     * Setting pin4 and pin5 as a analog input
     */
    AD1PCFG = 0xFFCF;
    
    /**
     * Start the sample enable bit 
     * from bit2 and set the SSRC<2:0>
     * to 111 - (Internal counter ends sampling 
     * and starts conversion (auto-convert))
     */
    AD1CON1 = 0x00E2;
    
    
    AD1CON2 = 0x0000;
    
    /**
     * AD1CON3 value bit7 to bit0 has been set to 0 so
     * that ADCS is set to zero and TAD = Tcy
     */
    AD1CON3 = 0x1100;
    AD1CSSL = 0x0000;
    /**
     * Setting the initial value for AD1CHS to 0x0004
     */
    AD1CHS = 0x0004;
    AD1CON1bits.ADON = 1;
}

int ADC_Operate (int value)
{
    while (1) {
        AD1CHS = value;
        AD1CON1bits.SAMP = 1;
        while (!AD1CON1bits.DONE);
        return ADC1BUF0;
    }
}

////////////////////////////////////////////////////////////////////////////////
void SYSTEM_Initialize(void)
{
    PIN_MANAGER_Initialize();
    OSCILLATOR_Initialize();
    TIMER_Initialize();
    ADC_Initialize();
    UART2_Initialize();
    INTERRUPT_Initialize(); 
}
////////////////////////////////////////////////////////////////////////////////
void OSCILLATOR_Initialize(void)
{
    //NOSC PRI; SOSCEN disabled; OSWEN Switch is Complete; 
    __builtin_write_OSCCONL((uint8_t) (0x0200 & 0x00FF));
    // RCDIV FRC/2; DOZE 1:8; DOZEN disabled; ROI disabled; 
    CLKDIV = 0x3100;
    // TUN Center frequency; 
    OSCTUN = 0x0000;
    // WDTO disabled; TRAPR disabled; SWDTEN disabled; EXTR disabled; POR disabled; SLEEP disabled; BOR disabled; IDLE disabled; IOPUWR disabled; VREGS disabled; CM disabled; SWR disabled; 
    RCON = 0x0000;
}
////////////////////////////////////////////////////////////////////////////////
void PIN_MANAGER_Initialize(void)
{
    /****************************************************************************
     * Setting the Output Latch SFR(s)
     ***************************************************************************/
    LATA = 0x0000;
    LATB = 0x0000;
    LATC = 0x0000;
    LATD = 0x0000;
    LATE = 0x0000;
    LATF = 0x0000;
    LATG = 0x0000;

    /****************************************************************************
     * Setting the GPIO Direction SFR(s)
     ***************************************************************************/
    TRISA = 0xC600;
    TRISB = 0xFFFF;
    TRISC = 0xF01E;
    TRISD = 0xFFFF;
    TRISE = 0x03FF;
    TRISF = 0x31FF;
    TRISG = 0xF3CF;

    /****************************************************************************
     * Setting the Weak Pull Up and Weak Pull Down SFR(s)
     ***************************************************************************/
    CNPU1 = 0x0000;
    CNPU2 = 0x0000;

    /****************************************************************************
     * Setting the Open Drain SFR(s)
     ***************************************************************************/
    ODCA = 0x0000;
    ODCB = 0x0000;
    ODCC = 0x0000;
    ODCD = 0x0000;
    ODCE = 0x0000;
    ODCF = 0x0000;
    ODCG = 0x0000;

    /****************************************************************************
     * Setting the Analog/Digital Configuration SFR(s)
     ***************************************************************************/
    AD1PCFG = 0x00C0;
}
////////////////////////////////////////////////////////////////////////////////
void INTERRUPT_Initialize (void)
{

}

void TIMER_Initialize(void)
{
      TMR1 = 0x0000;
      //Instructions from Manual
//    1. Set the TON bit (= 1).
      /**
       * Timer setting done at the last
       */
//    2. Select the timer pre scaler ratio using the
      T1CONbits.TCKPS = 0b00;  //1:1 ratio selected
      /**
       * We are using the lowest possible pre scaler to get
       * a more precis time interval facilitating more control
       * over the time
       */
//    3. Set the Clock and Gating modes using the TCS
      T1CONbits.TCS = 0; //using internal clock so TCS=0
      T1CONbits.TGATE = 0; //TGATE is set to 0 as we want the output from the PreScaler to generate out interrupts
      
//    4. Set or clear the TSYNC bit to configure synchronous or asynchronous operation.
      T1CONbits.T1SYNC = 1; //turning on synchronization of the clock
            
//    5. Load the timer period value into the PR1
      PR1 = (uint16_t) 0xE28F; //The value of the pre scaler goes here register.
      //58000 = E290
      
      IFS0bits.T1IF = 0; 
      
      // Interrupt priority setting
      // 7 = 0b0111
      IPC0bits.T1IP2 = 1; 
      IPC0bits.T1IP1 = 1; 
      IPC0bits.T1IP0 = 1;
      
      IEC0bits.T1IE = 1;
      T1CONbits.TON = 1; 
//    Ans: Check the Interrupt_Initialize for more details
}

//////////////////////////////////////////////////////////////////////////////////
void delayFunc(void)
{
  int j,k;
  int a;
  
  for(j = 0; j < count1; j++)
  {
      for(k=0; k < count2; k++)
      {
          a = 0; //How many times (frequency) this instruction getting executed?
      }
  } 
}
////////////////////////////////////////////////////////////////////////////////
/**
 End of File
*/