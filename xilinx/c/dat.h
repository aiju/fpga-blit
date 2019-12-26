extern uchar *mmio;

enum {
	UART_IN_DATA = 0x00,
	UART_IN_STATUS = 0x04,
	KBD_IN_DATA = 0x08,
	KBD_IN_STATUS = 0x0C,
	UART_OUT = 0x10,
	MOUSE = 0x14,
	MOUSE_BUT = 0x18,
	
};
