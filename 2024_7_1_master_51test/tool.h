


//#define UART_FUNC
void uart_init(uint8_t baud);
void send_buffer(uint8_t byte);
void uart_print(const uint8_t *format, ...);
void delay_ms(uint16_t ms);