/*
 * File:   main_Prelab.c
 * Author: Javier Alejandro Pérez Marín
 * 
 * Potenciómetro en AN0 que es convertido constantemente, pero al presionar 
 * un pb en RB0 PIC entra a Modo Sleep y se enciende de nuevo cuando termina
 * la interrupción.
 *
 * Created on 16 de mayo de 2022, 01:32 PM
 */


// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*
 * CONSTANTES
 */
#define _XTAL_FREQ 4000000

/*
 * VARIABLES
 */


/*
 * PROTOTIPO DE FUNCIÓN
 */
void setup(void);

void __interrupt() isr(void){
    //Se revisa interrupción del PORTB
    if (INTCONbits.RBIF){
        if (!PORTBbits.RB0){ //Pb de SLEEP
            PORTAbits.RA1 = 1; //Led para indicar que estamos en bajo consumo
            SLEEP(); //Se pone a mimir al PIC 
            __delay_ms(1000); //Delay de 1 seg pues la conversión es rápida
            if(ADCON0bits.GO == 0){ // Si no hay proceso de conversión
                ADCON0bits.GO = 1; // Se inicia proceso de conversión
            } 
           }
         INTCONbits.RBIF = 0; // Limpiamos bandera ADC  
    }
    //Se revisa interrupción ADC
    if (PIR1bits.ADIF){       
        if (ADCON0bits.CHS == 0){ //Se verifica canal AN0        
            PORTC = ADRESH; //Se muestran los 8 bits superiores en PORTC
        }
        PIR1bits.ADIF = 0; // Limpiamos bandera ADC      
    }
    return;
}

void main(void) {  
   
    setup(); // Se pasa a configurar PIC
        
    while(1)
    {
        PORTAbits.RA1 = 0; //Se apaga LED que indica SLEEP
    }
}

void setup(void){
    ANSEL = 0x01; //Se configura PORTA0/AN0 como entrada analógica
    ANSELH = 0;   //I/O DIGITALES
    
    OSCCONbits.IRCF = 0b0110;   //Oscilador interno de 4 MHz
    OSCCONbits.SCS = 1;         //Oscilador interno
    
    TRISA = 0x01; //PORTA0/AN0 como INPUT    
    PORTA = 0;    //CLEAR DE PUERTO A 
    
    TRISBbits.TRISB0 = 1; //RB0 COMO INPUT
    PORTB = 0;      //CLEAR DE PUERTO B
    
    TRISC = 0; //PORTC como OUTPUT
    PORTC = 0; //CLEAR DE PUERTO C
    
    //Pull-ups
    OPTION_REGbits.nRBPU = 0; // Se habilitan pull-up de PORTB
    WPUBbits.WPUB0 = 1; //Se habilita pull de RB0
    
    
    //Config ADC
    ADCON0bits.ADCS = 0b11; // FRC para lograr que funcione en SLEEP
    ADCON1bits.VCFG0 = 0;  // Referencia VDD
    ADCON1bits.VCFG1 = 0;  // Referencia VSS
    ADCON0bits.CHS = 0b0000; // Se selecciona PORTA0/AN0
    ADCON1bits.ADFM = 0; // Se indica que se tendrá un justificado a la izquierda
    ADCON0bits.ADON = 1; // Se habilita el modulo ADC
    __delay_us(40);     // Delay para sample time
    
    //Config interrupciones
    INTCONbits.GIE = 1; //Se habilitan interrupciones globales
    PIE1bits.ADIE = 1;  //Se habilita interrupcion del ADC
    PIR1bits.ADIF = 0; // Limpieza de bandera del ADC
    INTCONbits.RBIE = 1; //Se habilitan interrupciones del PORTB
    IOCBbits.IOCB0 = 1;  //Se habilitan interrupción por cambio de estado de RB0
    INTCONbits.PEIE = 1; // Se habilitan interrupciones de periféricos
 }