/*
 * File:   postlab10.c
 * Author: Carolina Paz
 *
 * Created on 5 de mayo de 2022, 01:23 PM
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

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

/*------------------------------------------------------------------------------
 * CONSTANTES
 ------------------------------------------------------------------------------*/
#define _XTAL_FREQ 1000000
#define LEN_MSG 5               // Constante para definir largo de mensaje e iteraciones al enviarlo por el serial
#define LEN_MSG_2 3             // Constante para definir largo de mensaje e iteraciones al enviarlo por el serial
/*------------------------------------------------------------------------------
 * VARIABLES
 ------------------------------------------------------------------------------*/
uint8_t indice = 0;                                      // Variable para saber que posición del mensaje enviar al serial
uint8_t valor_old = 0;                                   // Variable para guardar el valor anterior recibido
uint8_t num[3];                                          // Variable para cen, dec y unid. conversión
char valor[LEN_MSG_2] = {' ', 0x0D, 0x0A};               // Variable que guarda el valor de potenciometro en Ascii
char valor_2[LEN_MSG] = {' ', ' ', ' ', 0x0D, 0x0A};     // Variable para cen, dec y unid. convertido
char ADC;                                                // Variable para guardar el ADRESH en la interrupcion
char menu[1];                                            // Variable para las opciones de menu

/*------------------------------------------------------------------------------
 * PROTOTIPO DE FUNCIONES
 ------------------------------------------------------------------------------*/
void setup(void);                                        // Main principal
void obtener_no(void);                                   // Hacer conversión a cen, dec y unid
void USART_Tx(char data);
void Cadena(char *str);                                  // Permitir la impresión de la cadena en terminal

/*------------------------------------------------------------------------------
 * INTERRUPCIONES
 ------------------------------------------------------------------------------*/
void __interrupt() isr (void){
    if(PIR1bits.RCIF){                                   // Hay nuevo datos en el serial?
                                    
    menu[0] = RCREG;                                     // Guardar el valor que recibe en menu
        
    if(RCREG == '1'){  }                                 // Revisar si el usuario ingreso num "1"
    else if(RCREG == '2'){ }                             // Revisar si el usuario ingreso num "2"
        
    else{                                                // En caso que no sea niguna de las opciones
        //Mensaje que dará al usuario
        Cadena ("\r ¿Qué acción desea ejecutar? \r");
        Cadena ("(1) Leer potenciometro \r");
        Cadena ("(2) Enviar Ascii \r \r");     }      
    }

    if(PIR1bits.ADIF){                                   // Verificar interrupcion de ADC
        if(ADCON0bits.CHS == 0){                         // Cambio de canal al AN0
            ADC = ADRESH; }                              // ADC valor de ADRESH
        PIR1bits.ADIF = 0;                               // Limpiar la interrucpción del ADC
    }
    return;
}

/*------------------------------------------------------------------------------
 * CICLO PRINCIPAL
 ------------------------------------------------------------------------------*/
void main(void){
    setup();
    while(1){
        
        __delay_ms(100);                                 // Sample time

        if(ADCON0bits.GO == 0){ADCON0bits.GO = 1;}       //Mantenerse en el mismo canal siempre   
        
        if(menu[0]== '1'){                               // Revisar si el usuario ingreso el num 1
            valor[0]= ADRESH;                            // Guardar el valor de ADC en valor
            obtener_no();                                // Obtener las cen, dec, unid.
            
            Cadena ("El valor del potenciometro es:\r"); //Escribir la cadena a mostrar
            indice = 0;                                  // Limpiar indice para empezar en 0
            while(indice<LEN_MSG){                       // Loop para imprimir el mensaje completo
                if (PIR1bits.TXIF){                      // Esperamos a que esté libre el TXREG para poder enviar por el serial
                    TXREG = valor_2[indice];             // Cargamos caracter a enviar
                    indice++;                            // Incrementamos indice para enviar siguiente caracter
                }
            }
            menu[0]=0;                                   // Limpiar menu para empezar en 0   
            // Reiniciar mensaje que dará al usuario
            Cadena ("\r ¿Qué acción desea ejecutar? \r");
            Cadena ("(1) Leer potenciometro \r");
            Cadena ("(2) Enviar Ascii \r \r"); 
        }
        
        else if(menu[0] == '2'){                         // Revisar si el usuario ingreso el num 2
            valor_2[0]= ADRESH;                          // Enviar el valor de ADC a valor_2
            PORTB = valor_2[0];                          // Mandar el valor al Puerto B
            Cadena ("El valor fue enviado al puerto B con éxito\r");  //Escribir la cadena a mostrar
            menu[0]=0;                                   // Limpiar menu para empezar en 0
            // Reiniciar mensaje que dará al usuario
            Cadena ("\r ¿Qué acción desea ejecutar? \r");
            Cadena ("(1) Leer potenciometro \r");
            Cadena ("(2) Enviar Ascii \r \r"); 
        }      
    } 
    return;
}

/*------------------------------------------------------------------------------
 * CONFIGURACION
 ------------------------------------------------------------------------------*/
void obtener_no(void){

    num[0] = (valor[0]/100);                             // Obtener centenas
    num[1] = ((valor[0] - num[0]*100)/10);               // Obtener decenas
    num[2] = (valor[0] - num[0]*100 - num[1]*10);        // Obtener unidades

    valor_2[0] = num[0] + 48;                            // Conversion a Ascii centenas
    valor_2[1] = num[1] + 48;                            // Conversion a Ascii decenas
    valor_2[2] = num[2] + 48;                            // Conversion a Ascii unidades
}

void USART_Tx (char data){
    while (TXSTAbits.TRMT == 0);                         // Mientras TRMT sea 0
    TXREG = data;                                        // Enviar a TXREG a data
}

void Cadena (char *str){
    while(*str != '\0'){
        USART_Tx(*str);                                  //´Permitir la impresion en la cadena 
        str++;                                           // Incrementamos str para enviar siguiente caracter
    }
}

void setup(void){
    
     // Configuracion de entradas y salidas
    ANSEL = 0b00000001;         // AN0 como entrada analogica
    ANSELH = 0;                 // I/O digitales

    TRISB = 0;                  // PORTB como salida
    PORTB = 0;                  // Limpiar puerto B

    // Configuracion del reloj
    OSCCONbits.IRCF = 0b100;    // 1MHz
    OSCCONbits.SCS = 1;         // Oscilador interno

    // Configuraciones de comunicacion serial
    //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores de tabla 12-5
    TXSTAbits.SYNC = 0;         // Comunicación ascincrona (full-duplex)
    TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad
    BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate

    SPBRG = 25;
    SPBRGH = 0;                 // Baud rate ~9600, error -> 0.16%

    RCSTAbits.SPEN = 1;         // Habilitamos comunicación
    TXSTAbits.TX9 = 0;          // Utilizamos solo 8 bits
    TXSTAbits.TXEN = 1;         // Habilitamos transmisor
    RCSTAbits.CREN = 1;         // Habilitamos receptor

    // Configuraciones de interrupciones
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    INTCONbits.GIE = 1;         // Habilitamos interrupciones globales
    INTCONbits.PEIE = 1;        // Habilitamos interrupciones de perifericos
    PIE1bits.RCIE = 1;          // Habilitamos Interrupciones de recepción

    // Configuración ADC
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    
    ADCON0bits.ADCS = 0b01;     // Fosc/8
    ADCON0bits.CHS = 0;         // Seleccionamos el AN0
    ADCON0bits.ADON = 1;        // Habilitamos modulo ADC
    __delay_us(50);             // Sample time
    
    //Mensaje que dará al usuario de primero
    Cadena ("\r ¿Qué acción desea ejecutar? \r");
    Cadena ("(1) Leer potenciometro \r");
    Cadena ("(2) Enviar Ascii \r \r"); 
}
