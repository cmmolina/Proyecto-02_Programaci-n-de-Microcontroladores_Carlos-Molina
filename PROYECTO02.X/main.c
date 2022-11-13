/*
 * Universidad del Valle de Guatemala
 * Programación de Microcontroladores
 * Carlos Mauricio Molina López (#21253)
 * Proyecto 02 - Brazo robótico
 * Created on 12 de noviembre de 2022, 11:37 PM
 */

//******************************************************************************
// Palabra de Configuración
//******************************************************************************

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

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 500000
#define tmr0_value 240

//******************************************************************************
// Variables 
//******************************************************************************
int modo;
int ADC_Voltaje;
int equivalent; 

//******************************************************************************
// Prototipos de Funciones
//******************************************************************************
void setup(void);
void setupPWM(void);
void setupADC(void);
void incModo(void);
void ADC_to_PWM(int ADCVoltage);

//******************************************************************************
// Interrupción
//******************************************************************************
void __interrupt() isr (void){
    INTCONbits.T0IF = 0;
    
    /*if (INTCONbits.RBIF){ //Interrupción del Puerto B
        INTCONbits.RBIF = 0; 
    }*/
    
    if (PIR1bits.ADIF){   //Interrupción del ADC cuando la lectura termina
        //PORTBbits.RB7=1; 
        PIR1bits.ADIF=0; 
    }
    /*if (INTCONbits.T0IF){ //Interrupción del TMR0
        contador++;
        if (contador <= ADC_Voltaje3){
            PORTAbits.RA7 = 1; 
        }
        else{
            PORTAbits.RA7 = 0; 
        }
        TMR0 = tmr0_value; 
        INTCONbits.T0IF = 0;*/
}

//******************************************************************************
// Código Principal 
//******************************************************************************
void main(void) {
    setup();
    setupADC();
    setupPWM();
    modo = 1; 
    
    //Loop Principal
    while(1){
        
        //Cambio de modo (Pushbutton)
        if (PORTEbits.RE2){
            while(PORTEbits.RE2){ //Antirrebote
                ;
            }
            incModo();
        }
        
        //Modo
        switch (modo){
            case (1): //Modo Manual (Se ajusta la posición de los servos con los pots)
                
                //Lectura Canal AN0 
                ADCON0bits.CHS = 0b0000;
                __delay_us(100);
                ADCON0bits.GO = 1;
                while (ADCON0bits.GO == 1){
                    ;
                }
                ADC_Voltaje = ADRESH;
                ADC_to_PWM(ADC_Voltaje);
                CCPR1L = equivalent;
                __delay_us(100);
                
                //Lectura Canal AN1
                ADCON0bits.CHS = 0b0001;
                __delay_us(100);
                ADCON0bits.GO = 1;
                while (ADCON0bits.GO == 1){
                    ;
                }
                ADC_Voltaje = ADRESH;
                ADC_to_PWM(ADC_Voltaje);
                CCPR2L = equivalent;
                __delay_us(100);
                
                //Lectura Canal AN2
                ADCON0bits.CHS = 0b0010;
                __delay_us(100);
                ADCON0bits.GO = 1;
                while (ADCON0bits.GO == 1){
                    ;
                }
                PORTD = ADRESH;
                __delay_us(100);
                
                //Lectura Canal AN3
                ADCON0bits.CHS = 0b0011;
                __delay_us(100);
                ADCON0bits.GO = 1;
                while (ADCON0bits.GO == 1){
                    ;
                }
                //PORTD = ADRESH;
                __delay_us(100);
                
                break;
            case (2): //Modo EEPROM (Se reproducen posiciones guardadas en la EEPROM)+
                ;
                break; 
            case(3): //Modo UART (Movimiento controlado por medio de Adafruit)
                ;
                break;   
        }
        
        //End of loop
    }
}

//******************************************************************************
//Funciones
//******************************************************************************
void ADC_to_PWM(int voltaje){
    equivalent = (unsigned short) (7+( (float)(9)/(255) ) * (voltaje-0));
}

void setup(void){
    //Configuración de I/O 
    
    ANSEL = 0b00001111;             // RA0, RA1, RA2, RA3 analógicos
    ANSELH = 0; 

            //76543210
    TRISA = 0b00001111;            // RA0, RA1, RA2, RA3 como inputs
    TRISB = 0b00000000; 
    TRISC = 0b00000110; 
    TRISD = 0b00000000; 
    TRISE = 0b00000100;             //RE2 como input
    
    PORTA = 0b00000000; 
    PORTB = 0b00000000; 
    PORTC = 0b00000000; 
    PORTD = 0b00000000; 
    PORTE = 0b00000000;
     
    /*Guía para interrupción del Puerto B
    IOCBbits.IOCB7 = 1;             // RB7 con Interrupción
    WPUBbits.WPUB7 = 0;             // Pull-up enabled
    INTCONbits.RBIE = 1;            // Se habilitan las interrupciones del Puerto B
    INTCONbits.RBIF = 0;            // Flag del Puerto B en 0
    */
    
    //Configuración del Oscilador
    OSCCONbits.IRCF = 0b011;        // 500KHz
    OSCCONbits.SCS = 1;             // Oscilador Interno
    
    
    //Configuración de las Interrupciones
    INTCONbits.GIE = 1;             
    //INTCONbits.PEIE = 1;
    
    PIE1bits.ADIE = 1;              // Se habilita la interrupción del ADC
    INTCONbits.TMR0IE = 1;          // Se habilitan las interrupciones del TMR0    
    
    PIR1bits.ADIF = 0;              // Flag de ADC en 0
    INTCONbits.T0IF = 0;            // Flag de TMR0 en 0
    
    //Configuración del TMR0
    OPTION_REGbits.T0CS = 0;        // Fosc/4
    OPTION_REGbits.PSA = 0;         // Prescaler para el TMR0
    OPTION_REGbits.PS = 0b011;      // Prescaler 1:16
    TMR0 = tmr0_value;              // Asignamos valor al TMR0 para 2ms
}

void setupPWM(void){
    TRISCbits.TRISC2 = 1;           //CCP1 (pin del uC) como Input
    TRISCbits.TRISC1 = 1;           //CCP2 (pin del uC) como Input
    
    PR2 = 255;                      //Se carga al PR2 para un periodo de 0.02s
    
    CCP1CONbits.P1M = 0b00;         //Single output (ya que no quiero el puente)
    
    
    CCP1CONbits.CCP1M = 0b1100;     //P1A como PWM
                                    //Describe como están las parejas; pero ya 
                                    //que estoy con un single output no importa.
    
    CCP2CONbits.CCP2M = 0b1111;     //P2A como PWM
    
    //Calculos para 1.5ms de ancho de pulso (En 0 grados)
    CCP1CONbits.DC1B = 0b11;        //CCPxCON<5:4>
    CCPR1L = 11;                    //CCPR1L
    
    
    CCP2CONbits.DC2B1 = 0b1;        //CCPxCON<5:4>
    CCP2CONbits.DC2B0 = 0b1;
    CCPR2L = 11;                    //CCPR2L
    
    //Configutación del TMR2
    PIR1bits.TMR2IF = 0;            //Se limpia la bandera al inicio
    T2CONbits.T2CKPS = 0b11;        //Prescaler del TMR2 (1:16)
    T2CONbits.TMR2ON = 1;           //Se habilita el TMR2
    
    while(!PIR1bits.TMR2IF){        //Mientras TM2IF esté apagado...
        ;
    }
    
    TRISCbits.TRISC2=0;             //Habilitamos la salida del PWM2.
    TRISCbits.TRISC1=0;             //Habilitamos la salida del PWM1.
}

void setupADC(void){
    //Módulo de ADC
    ADCON0bits.ADCS = 0b01;         // Fosc/8
    
    ADCON1bits.VCFG1 = 0;           // Voltaje de Referencia + - VSS
    ADCON1bits.VCFG0 = 0;           // Voltaje de Referencia - - VDD
    
    //Formato de Resultado 
    ADCON1bits.ADFM = 0;            // Justificado a la Izquierda
    
    //Canal
    ADCON0bits.CHS = 0b0000;        // Canal AN0 (Para empezar)
    
    ADCON0bits.ADON = 1;            // Se habilita el ADC
    
    PIR1bits.ADIF = 0;              // Apagamos la bandera
    
    //Delay (Ejemplo)
    __delay_us(100);
}

void incModo(void){
    if (modo != 3){
        modo ++; 
    }
    else{
        modo = 1;
    }
}