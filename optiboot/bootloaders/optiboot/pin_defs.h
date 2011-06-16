#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega88) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega88__) || defined(__AVR_ATmega88P__)
/* Onboard LED is connected to pin PB5 in Arduino NG, Diecimila, and Duemilanove */ 
#define LED_DDR     DDRB
#define LED_PORT    PORTB
#define LED_PIN     PINB

#if defined(BLUECONTROLLER) 
#define LED            PINB6
#define LED_DDR_VAL    _BV(LED)
#define BLUEC_BT_RESET_PORT  PORTB
#define BLUEC_BT_RESET_DDR   DDRB
#define BLUEC_BT_RESET       PINB7
#define BLUEC_BTN_DDR  DDRD
#define BLUEC_BTN_PORT PORTD
#define BLUEC_BTN_PIN  PIND
#define BLUEC_BTN      PIND2
#define BLUEC_WAITTIME (35) // initial timeout in bootloader in seconds, granularity is 8.40s
#define ENTER_BL_MAGIC (0xc49e)
#else
#define LED         PINB5
#define LED_DDR_VAL (_BV(LED))
#endif


/* Ports for soft UART */
#ifdef SOFT_UART
#define UART_PORT   PORTD
#define UART_PIN    PIND
#define UART_DDR    DDRD
#define UART_TX_BIT 1
#define UART_RX_BIT 0
#endif
#endif

#if defined(__AVR_ATmega8__)
  //Name conversion R.Wiersma
  #define UCSR0A	UCSRA
  #define UDR0 		UDR
  #define UDRE0 	UDRE
  #define RXC0		RXC
  #define TIFR1 	TIFR
  #define WDTCSR	WDTCR
#endif

/* Luminet support */
#if defined(__AVR_ATtiny84__)
/* Red LED is connected to pin PA4 */ 
#define LED_DDR     DDRA
#define LED_PORT    PORTA
#define LED_PIN     PINA
#define LED         PINA4
#define LED_DDR_VAL    _BV(LED)
/* Ports for soft UART - left port only for now. TX/RX on PA2/PA3 */
#ifdef SOFT_UART
#define UART_PORT   PORTA
#define UART_PIN    PINA
#define UART_DDR    DDRA
#define UART_TX_BIT 2
#define UART_RX_BIT 3
#endif
#endif

/* Sanguino support */
#if defined(__AVR_ATmega644P__)
/* Onboard LED is connected to pin PB0 on Sanguino */ 
#define LED_DDR     DDRB
#define LED_PORT    PORTB
#define LED_PIN     PINB
#define LED         PINB0
#define LED_DDR_VAL    _BV(LED)

/* Ports for soft UART */
#ifdef SOFT_UART
#define UART_PORT   PORTD
#define UART_PIN    PIND
#define UART_DDR    DDRD
#define UART_TX_BIT 1
#define UART_RX_BIT 0
#endif
#endif

/* Mega support */
#if defined(__AVR_ATmega1280__)
/* Onboard LED is connected to pin PB7 on Arduino Mega */ 
#define LED_DDR     DDRB
#define LED_PORT    PORTB
#define LED_PIN     PINB
#define LED         PINB7
#define LED_DDR_VAL    _BV(LED)

/* Ports for soft UART */
#ifdef SOFT_UART
#define UART_PORT   PORTE
#define UART_PIN    PINE
#define UART_DDR    DDRE
#define UART_TX_BIT 1
#define UART_RX_BIT 0
#endif
#endif
