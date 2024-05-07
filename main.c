// ENGPHYS 3BB4 Final Project
// PID Temperature Controller
// 2023.04.05
//-----------------------------------------------------------------
// ReadMe
//-----------------------------------------------------------------
// myscope main.c code works with the MATLAB myscope.m and
// myscope.fig files under the following conditions:
// MSP430 is configured for 16MHz clock.
// UART is set for 115200 baud,using the x16 clock mode.
// ADC input is on P1.4
// myscope.m mustbe changed to connect to the proper COMx port.
//-----------------------------------------------------------------

#include "io430.h"

#define ON 1
#define OFF 0
#define ASCII_CR 0x0D
#define ASCII_LF 0x0A

// Define MSP pins
#define GREEN_LED P1OUT_bit.P0
#define RED_LED P1OUT_bit.P6

#define NPOINTS 400

//--------------------------------------------------------
// GlobalVariables
//--------------------------------------------------------

// Declare array that stores samples
unsigned char sample_array[1];

//--------------------------------------------------------
// Miscellaneous Functions:
//--------------------------------------------------------

void delay(unsigned long d)
{
    while (d--)
        ;
}

#pragma vector = PORT1_VECTOR // Declare that this function is an interrupt service routine (ISR) for Port 1 interrupt
__interrupt void PORT1_ISR(void)
{
    GREEN_LED = OFF;
    P1IFG_bit.P3 = 0; // Clear the interrupt request flag for Pin 3 of Port 1 (P1.3) to acknowledge and handle the interrupt
}

//--------------------------------------------------------
// UART Module
//--------------------------------------------------------

void Init_UART(void)
{

    // initialize the USCI
    // RXD is on P1.1
    // TXD is on P1.2

    // configure P1.1and P1.2 for secondary peripheral function
    P1SEL_bit.P1 = 1;
    P1SEL2_bit.P1 = 1;
    P1SEL_bit.P2 = 1;
    P1SEL2_bit.P2 = 1;

    // divide by  104 for 9600b with  1MHz clock
    // divide by 1667 for 9600b with 16MHz clock
    // divide by  9 for 115200b with 16MHz clock

    UCA0BR1 = 0;
    UCA0BR0 = 9;

    // use x16 clock
    UCA0MCTL_bit.UCOS16 = 1;

    // select UART clock source
    UCA0CTL1_bit.UCSSEL1 = 1;
    UCA0CTL1_bit.UCSSEL0 = 0;

    // release UART RESET
    UCA0CTL1_bit.UCSWRST = 0;
}

// -------------------------------------------------
// receive character function
// return with 7-bit character in char
// -------------------------------------------------
unsigned char getc(void)
{
    while (!IFG2_bit.UCA0RXIFG)
        ;               // Wait for the UCA0RXIFG flag to be set, indicating that a character has been received
    return (UCA0RXBUF); // Return the received character from the UCA0RXBUF register
}

// -------------------------------------------------
// send character function
// enter with 8-bit character in char
// -------------------------------------------------
void putc(unsigned char c)
{
    while (!IFG2_bit.UCA0TXIFG)
        ;          // Wait until the UART TX buffer is ready for data
    UCA0TXBUF = c; // Write the character to the UART TX buffer
}

//--------------------------------------------------------
// ADCModule
//--------------------------------------------------------
void Init_ADC(void)
{
    // initialize 10-bit ADC using input channel 4 on P1.4
    ADC10CTL1 = INCH_4 + CONSEQ_2;

    ADC10AE0 |= BIT4; // enable analog input channel 4

    // select sample-hold time, multisample conversion and turn on the ADC
    ADC10CTL0 |= ADC10SHT_0 + MSC + ADC10ON;

    // start ADC
    ADC10CTL0 |= ADC10SC + ENC;
}

void Sample(int n)
{
    // Enables the MCU to digitize an analog input signal by
    // reading the content of the ADC register in a loop.

    for (int i = 0; i < n; i++)
    {
        // Read ADC value from ADC10MEM Register and bit shift right by 2 to reduce to 8 bits for MATLAB to receive
        // and store in sample_array at index i
        sample_array[i] = ADC10MEM >> 2;
    }
}

void Send(int n)
{
    // Enables the MCU to to transmit the sampled data
    // via the USB interface.
    int i = 0;

    while (i < n)
    {
        // Send ADC values to MATLAB
        putc(sample_array[i]);
        i += 1;
    }
}

//--------------------------------------------------------
// Initialization
//--------------------------------------------------------

void Init(void)
{
    // Stop watchdog timer to prevent timeout reset
    WDTCTL = WDTPW + WDTHOLD;

    DCOCTL = CALDCO_16MHZ;
    BCSCTL1 = CALBC1_16MHZ;

    P1REN = 0x08;    // enable output resistor
    P1OUT = 0x08;    // enable P1.3 pullup resistor
    P1DIR = 0x41;    // setup LEDs as output
    P1IE_bit.P3 = 1; // enable interrupts on P1.3 input
}

//--------------------------------------------------------
// PWM functions
//--------------------------------------------------------

// Function to control cooling using PWM
void PWM_cool(int n)
{
    // Heating: 1.7 = High, 2.1 = low
    // Bit 1.7 is direction, 2.1 is power

    P1SEL &= (~BIT7); // Set P1.7 SEL for GPIO
    P1DIR |= BIT7;    // Set P1.7 as Output
    P1OUT |= BIT7;    // Set P1.7 HIGH

    P2DIR |= BIT1; // Set pin 2.1 to the output direction.
    P2SEL |= BIT1; // Select pin 2.1 as our PWM output.

    TA1CCR0 = 1000; // Set the period in the Timer A0 Capture/Compare 0 register to 1000 us.

    TA1CCTL1 = OUTMOD_7; // Configure Capture/Compare Control 1 register for PWM mode (OUTMOD_7)

    // Scale the PID output to the PWM duty cycle range (0-1000)
    // The period in microseconds that the power is ON.
    TA1CCR1 = 1000 * (n / 128); 

    TA1CTL = TASSEL_2 + MC_1; // TASSEL_2 selects SMCLK as the clock source, and MC_1 tells it to start in up mode
}

// Function to control heating using PWM
void PWM_heat(int n)
{
    // Heating: 1.7 = High, 2.1 = low
    // Bit 1.7 is direction, 2.1 is power

    P1SEL &= (~BIT7); // Set P1.7 SEL for GPIO
    P1DIR |= BIT7;    // Set P1.7 as Output
    P1OUT &= ~BIT7;   // Set P1.7 LOW

    P2DIR |= BIT1; // Set pin 2.1 to the output direction.
    P2SEL |= BIT1; // Select pin 2.1 as our PWM output.

    TA1CCR0 = 1000; // Set the period in the Timer A0 Capture/Compare 0 register to 1000 us.
    TA1CCTL1 = OUTMOD_7;

    TA1CCR1 = (1000 * (1 - n / 128)); // The period in microseconds that the power is ON. It's half the time, which
                                      // corresponds to 1000 - (1000 * (n/128))

    TA1CTL = TASSEL_2 + MC_1; // TASSEL_2 selects SMCLK as the clock source, and MC_1 tells it to start in up mode
}

// Function to control PWM based on input value
void PWM(int n)
{
    if (n > 127)
    {
        // Call PWM_heat() with scaled input value
        PWM_heat((n - 127));
    }
    else
    {
        // Call PWM_cool() with input value
        PWM_cool(n);
    }
}

//--------------------------------------------------------
//
//
// MAIN Function
//
//
//--------------------------------------------------------
void main(void)
{
    Init();
    Init_UART();
    Init_ADC();

    while (1)
    {
        GREEN_LED = ON;
        Send(NPOINTS);
        GREEN_LED = OFF;
        Sample(NPOINTS);                     
        //PWM(0);
        PWM(getc());
    }
}