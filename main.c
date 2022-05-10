/* 
 * File:   main.c
 * Author: josej
 *
 * Created on May 9, 2022, 4:28 PM
 */

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

/*------------------------------------------------------------------------------
 * CONSTANTES 
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000
#define FLAG_SPI 0xFF
#define IN_MIN 0
#define IN_MAX 255
#define OUT_MIN 5
#define OUT_MAX 50

/*------------------------------------------------------------------------------
 * VARIABLES 
 ------------------------------------------------------------------------------*/
char cont_master = 0;
char cont_slave = 0xFF;
char val_temporal = 0;
uint8_t valor_pot = 0;
uint8_t valor_pwm = 0;
uint8_t old_pot = 0;
uint8_t contador = 0;
unsigned short CCPR = 0;

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES 
 ------------------------------------------------------------------------------*/
void setup(void);
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, 
            unsigned short out_min, unsigned short out_max);

/*------------------------------------------------------------------------------
 * INTERRUPCIONES 
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PORTBbits.RB5){          // Es maestro?
        if(PIR1bits.ADIF){      // Interrupcion del ADC
            valor_pot = ADRESH;
            PIR1bits.ADIF = 0;  // Limíamos bandera de interrupcion
        }
    }
    else{                       // Es esclavo?
        if(PIR1bits.SSPIF){     // Recibio datos?
            val_temporal = SSPBUF;
            if (val_temporal != FLAG_SPI){  // Es envio para generar pulsos?
                PORTD = val_temporal;       // Dato recibido en PORTD
                SSPBUF = contador;          // Cargar contador del eslcavo al buffer
            }
            else {
                SSPBUF = contador;
            }
            PIR1bits.SSPIF = 0;  // Limpiamos bandera de interrupcion
        }
        else if(INTCONbits.RBIF){   // Fue interrupcion de PORTB?
            if(!PORTBbits.RB0){     // Fue RB0?
                contador++;         // INcrementar contador
            }
            else if(!PORTBbits.RB1){    // Fue RB1?
                contador--;             // Decrementar contador
            }
            INTCONbits.RBIF = 0;    // Limpiamos bandera de interrupcion 
        }
    }
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void) {
    setup();
    while(1){        
        // Envío y recepción de datos en maestro
        if (PORTBbits.RB5){         // ¿Es maestro?
            if (ADCON0bits.GO == 0){    // Hay proceso de conversion?
                ADCON0bits.GO = 1;      // Iniciamos proceso de conversion
            }
        
            // Enviar valor de potenciometro
            PORTAbits.RA6 = 1;          // Deshabilitamso ss del esclavo
            __delay_ms(10);             // Esperar un tiempo para que el PIC pueda detectar el cambio en el pin
            PORTAbits.RA6 = 0;          // Habilitamos ss del esclavo
            __delay_ms(10);             
            
            if(valor_pot != old_pot){
                SSPBUF = valor_pot;     // Cargamos el valor del potenciometro al buffer
                while(!SSPSTATbits.BF){}    // Esperamos a que termine el envío 
                old_pot = valor_pot;    // Actualizamos el valor del potenciometro
            }
            PORTAbits.RA6 = 1;          // Deshabilitamos ss del esclavo
            __delay_ms(10);
            
            // Recibir contador 
            
            PORTAbits.RA7 = 1;          // Deshabilitamso ss del esclavo
            __delay_ms(10);             // Esperar un tiempo para que el PIC pueda detectar el cambio en el pin
            PORTAbits.RA7 = 0;          // Habilitamos nuevamente el esclavo 
            __delay_ms(10);
            
            SSPBUF = FLAG_SPI;          // Se envía un dato para que el maestro genere los pulsos de reloj
            while(!SSPSTATbits.BF){}    // Esperamos a que se reciba un dato 
            PORTD = SSPBUF;             // Mostramos dato en PORTD
            PORTAbits.RA7 = 1;          // Deshabilitamos SS del esclavo
            __delay_ms(10);
        }
        
        else{
            CCPR = map(PORTD, IN_MIN, IN_MAX, OUT_MIN, OUT_MAX);    // Valor de ancho de pulso 
            CCPR1L = (uint8_t)(CCPR>>2);                            // GUardamos los 8 bits mas significativos en CPR1L
            CCP1CONbits.DC1B = CCPR & 0b11;                         // Guardamos los 2 bits menos significativos en DC1B
        }
    }
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION 
 ------------------------------------------------------------------------------*/
void setup(void){
    ANSEL = 0x01;               // AN0 como entrada analógica
    ANSELH = 0;                 // I/O digitales
    
    TRISA = 0b00100001;         // SS y RA0 como entradas
    PORTA = 0;
    
    TRISB = 0xFF;               // Puerto B como entrada
    
    TRISD = 0;                  // Puerto D como salida
    PORTD = 0;
    
    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Reloj interno
    
    // Configuración de SPI
    // Configs de Maestro
    if(PORTBbits.RB5){
        TRISC = 0b00010000;         // -> SDI entrada, SCK y SD0 como salida
        PORTC = 0;
    
        // SSPCON <5:0>
        SSPCONbits.SSPM = 0b0000;   // -> SPI Maestro, Reloj -> Fosc/4 (250kbits/s)
        SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
        SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
        // SSPSTAT<7:6>
        SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
        SSPSTATbits.SMP = 1;        // -> Dato al final del pulso de reloj
        SSPBUF = valor_pot;       // Enviamos un dato inicial
        
        // Configuraciones ADC
        ADCON0bits.ADCS = 0b00;     // FOSC/2
        ADCON1bits.VCFG0 = 0;       // VDD
        ADCON1bits.VCFG1 = 0;       // VSS
        ADCON0bits.CHS = 0b0000;    // AN0
        ADCON1bits.ADFM = 0;        // Justificado a la izquierda
        ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
        __delay_us(40);             // Sample time
        
        // COnfiguracion de interrupciones 
        PIR1bits.ADIF = 0;          // Limpiamos bandera ADC
        PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
        INTCONbits.PEIE = 1;        // Habilitamos int. perifericos 
        INTCONbits.GIE = 1;         // Habilitamos int. globales
    }
    // Configs del esclavo
    else{
        TRISC = 0b00011000; // -> SDI y SCK entradas, SD0 como salida
        PORTC = 0;
        
        // SSPCON <5:0>
        SSPCONbits.SSPM = 0b0100;   // -> SPI Esclavo, SS hablitado
        SSPCONbits.CKP = 0;         // -> Reloj inactivo en 0
        SSPCONbits.SSPEN = 1;       // -> Habilitamos pines de SPI
        // SSPSTAT<7:6>
        SSPSTATbits.CKE = 1;        // -> Dato enviado cada flanco de subida
        SSPSTATbits.SMP = 0;        // -> Dato al final del pulso de reloj
        
        // Configuracion PWM
        TRISCbits.TRISC2 = 1;       // Deshabilitamos salida de CCP1
        PR2 = 62;                   // Periodo de 4 mS
        
        // Configuracion CCP 
        CCP1CON = 0;                // Apagamos CCP1
        CCP1CONbits.P1M = 0;        // Modo single output
        CCP1CONbits.CCP1M = 0b1100; // PWM
        
        CCPR1L = 5>>2;
        CCP1CONbits.DC1B = 5 & 0b11; // 0.5mS ancho de pulso / 25% ciclo de trabajo 
        
        PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion
        T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
        T2CONbits.TMR2ON = 1;       // Esperar un ciclo del TMR2
        PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion
        
        TRISCbits.TRISC2 = 0;       // Habilitamos PWM
        
        // Interrupciones
        PIR1bits.SSPIF = 0;         // Limpiamos bandera de SPI
        PIE1bits.SSPIE = 1;         // Habilitamos int. de SPI
        INTCONbits.PEIE = 1;
        INTCONbits.GIE = 1;
        INTCONbits.RBIE = 1;        // habilitamos int. de PORTB
        IOCB = 0b00000011;                // Cambio de estado para RB0 y RB1
        INTCONbits.RBIF = 0;        // Limpiamos bandera de interrupcion
    }
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1, 
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}