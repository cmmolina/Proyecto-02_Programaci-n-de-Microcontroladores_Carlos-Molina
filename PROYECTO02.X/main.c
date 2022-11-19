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
#define tmr0_value 100

//******************************************************************************
// Variables 
//******************************************************************************
int modo;
unsigned int ADC_Voltaje1;
unsigned int ADC_Voltaje2;
unsigned int ADC_Voltaje3; 
unsigned int ADC_Voltaje4; 
char option_selected;
unsigned int address; 

//******************************************************************************
// Prototipos de Funciones
//******************************************************************************
void setup(void);
void setupPWM(void);
void setupADC(void);
void initUART(void);
void incModo(void);
void delay(unsigned int sec);
void print(unsigned char *palabra);
unsigned int map(uint8_t value, int inputmin, int inputmax, int outmin, int outmax);
uint8_t read_EEPROM(uint8_t address);
void write_EEPROM(uint8_t address, uint8_t data);

//******************************************************************************
// Interrupción
//******************************************************************************
void __interrupt() isr (void){    
    
    if (PIR1bits.TXIF){
        PIR1bits.TXIF = 0;
    }
    
    if (PIR1bits.RCIF){
        PIR1bits.RCIF = 0;
    }
    
    //Interrupción del ADC cuando la lectura termina
    if (PIR1bits.ADIF){ 
        PIR1bits.ADIF=0; 
    }
    
    //Interrupción del TMR0 (PWM Manual)
    if (INTCONbits.T0IF){
        
        TMR0 = tmr0_value;          // Cargamos 20ms de nuevo al TMR0
        
        PORTAbits.RA7 = 1; 
        delay(ADC_Voltaje3); 
        PORTAbits.RA7 = 0;
        PORTAbits.RA6 = 1; 
        delay(ADC_Voltaje4); 
        PORTAbits.RA6 = 0;
        
        INTCONbits.T0IF = 0;
    }
    
    //Interrupción del Puerto B 
    if (INTCONbits.RBIF){ 
        if (PORTBbits.RB0 == 0){
            address = address + 4;  // Modificamos localidad a guardar/leer para arriba
        }
        
        else if (PORTBbits.RB1 == 0)
            address = address - 4;  // Modificamos localidad a guardar/leer para abajo
        
        else if (PORTBbits.RB2 == 0){ 
            
            //Guardamos los valores de la posición actual del brazo
            write_EEPROM(address, ADC_Voltaje1);  
            write_EEPROM(address + 1, ADC_Voltaje2);
            write_EEPROM(address + 2, ADC_Voltaje3);
            write_EEPROM(address + 3, ADC_Voltaje4);
        }
        
        else if (PORTBbits.RB3 == 0){
            if (modo == 2){
                //Leemos los valores guardados en la localidad que va de acuerdo al contador
                ADC_Voltaje1 = read_EEPROM(address);
                ADC_Voltaje2 = read_EEPROM(address+1);
                ADC_Voltaje3 = read_EEPROM(address+2);
                ADC_Voltaje4 = read_EEPROM(address+3);
            }
            else {
                ;
            }
        }
        
        INTCONbits.RBIF = 0;
    }
}

//******************************************************************************
// Código Principal 
//******************************************************************************
void main(void) {
    setup();
    setupADC();
    setupPWM();
    initUART();
    modo = 1; 
    address = 0;

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
                                
                //LEDs
                PORTDbits.RD0 = 0; 
                PORTDbits.RD1 = 0; 
                PORTDbits.RD2 = 1; 
                
                //Lectura Canal AN0 
                ADCON0bits.CHS = 0b0000;
                __delay_us(100);
                ADCON0bits.GO = 1;
                while (ADCON0bits.GO == 1){
                    ;
                }
                ADC_Voltaje1 = map(ADRESH, 0, 255, 5, 17); //Map de POT1
                CCPR1L = ADC_Voltaje1;        //Se manda valor de pulso a PWM
                __delay_us(100);
                
                //Lectura Canal AN1
                ADCON0bits.CHS = 0b0001;
                __delay_us(100);
                ADCON0bits.GO = 1;
                while (ADCON0bits.GO == 1){
                    ;
                }
                ADC_Voltaje2 = map(ADRESH, 0, 255, 5, 17); //Map de POT2
                CCPR2L = ADC_Voltaje2;        //Se manda valor de pulso a PWM
                __delay_us(100);
                
                //Lectura Canal AN2
                ADCON0bits.CHS = 0b0010;
                __delay_us(100);
                ADCON0bits.GO = 1;
                while (ADCON0bits.GO == 1){
                    ;
                }
                ADC_Voltaje3 = map(ADRESH, 0, 255, 5, 17); //Map de POT3
                __delay_us(100);
                
                //Lectura Canal AN3
                ADCON0bits.CHS = 0b0011;
                __delay_us(100);
                ADCON0bits.GO = 1;
                while (ADCON0bits.GO == 1){
                    ;
                }
                ADC_Voltaje4 = map(ADRESH, 0, 255, 5, 17); //Map de POT4
                __delay_us(100);
               
                break;
                
            case (2): //Modo EEPROM (Se reproducen posiciones guardadas en la EEPROM)
                
                //LEDs
                PORTDbits.RD0 = 1; 
                PORTDbits.RD1 = 0; 
                PORTDbits.RD2 = 0; 
                
                CCPR1L = ADC_Voltaje1; //Se actualiza pulso PWM1
                CCPR2L = ADC_Voltaje2; //Se actualiza pulso PWM2
                
                break; 
            case(3): //Modo UART (Movimiento controlado por medio de Adafruit)
                
                //LEDs
                PORTDbits.RD0 = 0; 
                PORTDbits.RD1 = 1; 
                PORTDbits.RD2 = 0;
                
                /* 
                print("¿Cuál servo desea mover?");
                //print("1. Brazo Izquierdo, 2. Brazo Derecho, 3. Pivote, 4. Garra");
                
                while(PIR1bits.RCIF == 0){
                     ;
                 }
                
                option_selected = RCREG;
                
                print("ojo");
                
                /*if (option_selected == '1'){
                    print("Seleccione el ángulo de rotación");
                    while(PIR1bits.RCIF == 0){
                        ;
                    }
                    
                    ADC_Voltaje1 = map(RCREG, 1, 4, 5, 17);
                    CCPR1L = ADC_Voltaje1; 
                    __delay_us(100);
                    }
                
                else if (option_selected == '2'){
                    print("Seleccione el ángulo de rotación");
                    while(PIR1bits.RCIF == 0){
                        ;
                    }
                    ADC_Voltaje2 = map(RCREG, 1, 4, 5, 17);
                    CCPR2L = ADC_Voltaje2; 
                    __delay_us(100);
                    }         
                
                ADC_Voltaje3 = map(RCREG, 1, 4, 5, 17);
                __delay_us(100);
                
                ADC_Voltaje4 = map(RCREG, 1, 4, 5, 17);
                __delay_us(100); */  
                
             
                break;   
        }
        //End of loop
    }
}

//******************************************************************************
//Funciones
//******************************************************************************

void write_EEPROM(uint8_t address, uint8_t data){
    while (WR);
    
    EEADR = address;
    EEDAT = data;
    
    EECON1bits.EEPGD = 0;        //Selección de acceso a memoria de datos
    EECON1bits.WREN = 1;         //Habilitamos la escritura
    
    INTCONbits.GIE = 0;          //Deshabilitamos interrupciones
    
    EECON2 = 0x55;
    EECON2 = 0xAA;
    
    EECON1bits.WR = 1;           //Empezamos la escritura de la memoria
    EECON1bits. WREN = 0;        //Deshabilitamos la escritura
    
    INTCONbits.GIE = 1;         //Habilitamos las interrupciones
}

uint8_t read_EEPROM (uint8_t address){
    while (WR||RD);
    
    EEADR = address;             //Localidad a leer
    EECON1bits.EEPGD = 0;        //Selección de acceso a memoria de datos
    EECON1bits.RD = 1;           //Empezamos la lectura de memoria
    return EEDAT;
}


void setup(void){
    //Configuración de I/O 
    
    ANSEL = 0b00001111;             // RA0, RA1, RA2, RA3 analógicos
    ANSELH = 0; 

            //76543210
    TRISA = 0b00001111;             // RA0, RA1, RA2, RA3 como inputs
    TRISB = 0b00001111;             // RB0, RB1, RB2, RB3 como inputs
    TRISC = 0b00000110;             // RC1, RC2 como inputs
    TRISD = 0b00000000; 
    TRISE = 0b00000100;             // RE2 como input
    
    PORTA = 0b00000000; 
    PORTB = 0b00000000; 
    PORTC = 0b00000000; 
    PORTD = 0b00000000; 
    PORTE = 0b00000000;
     
    /*Guía para interrupción/config. del Puerto B
    IOCBbits.IOCB7 = 1;             // RB7 con Interrupción
    WPUBbits.WPUB7 = 0;             // Pull-up enabled
    INTCONbits.RBIE = 1;            // Se habilitan las interrupciones del Puerto B
    INTCONbits.RBIF = 0;            // Flag del Puerto B en 0
    */
    
    //Configuración del Puerto B 
    IOCB = 0b00111111;              // Pines de Puerto B con Interrupción
    OPTION_REGbits.nRBPU = 0;       // Pull-Up/Pull-Down
    INTCONbits.RBIE = 1;            // Se habilitan las interrupciones del Puerto B
    
    
    //Configuración del Oscilador
    OSCCONbits.IRCF = 0b011;        // 500KHz
    OSCCONbits.SCS = 1;             // Oscilador Interno
    
    //Configuración de las Interrupciones
    INTCONbits.GIE = 1;             
    //INTCONbits.PEIE = 1;
    
    PIE1bits.ADIE = 1;              // Se habilita la interrupción del ADC
    INTCONbits.TMR0IE = 1;          // Se habilitan las interrupciones del TMR0    
    
    PIR1bits.ADIF = 0;              // Flag de ADC en 0
    INTCONbits.RBIF = 0;            // Flag de Interrupciones del Puerto B en 0

    
    //Configuración del TMR0
    OPTION_REGbits.T0CS = 0;        // Fosc/4
    OPTION_REGbits.PSA = 0;         // Prescaler para el TMR0
    OPTION_REGbits.PS = 0b011;      // Prescaler 1:16
    TMR0 = tmr0_value;              // Asignamos valor al TMR0 para 2ms
    INTCONbits.T0IF = 0;            // Flag de TMR0 en 0
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

void initUART(void){
    SPBRG = 14;                     // Baud rate (8MHz/9600)
    TXSTAbits.SYNC = 0;             // Asíncrono 
    RCSTAbits.SPEN = 1;             // Se habilita el módulo UART
    TXSTAbits.TXEN = 1;             /* Transmisión habilitada; TXIF se enciende
                                     automaticamente.*/
    
    PIR1bits.TXIF = 0;              // Apagamos la bandera de transmisión
    
    RCSTAbits.CREN = 1;             // Habilitamos la recepción
    
    BAUDCTLbits.BRG16 = 1;
}

void incModo(void){
    if (modo != 3){
        modo ++; 
    }
    else{
        modo = 1;
    }
}

void delay(unsigned int sec){
    while (sec > 0){
        __delay_us(50); 
        sec--; 
    }
}

unsigned int map(uint8_t value, int inputmin, 
                  int inputmax, int outmin, int outmax){
    return ((value - inputmin)*(outmax-outmin)) / ((inputmax-inputmin)+outmin);
}

void print(unsigned char *palabra){   
    while (*palabra != '\0'){
        while (TXIF != 1);
        TXREG = *palabra;
        *palabra++;
    }  
}