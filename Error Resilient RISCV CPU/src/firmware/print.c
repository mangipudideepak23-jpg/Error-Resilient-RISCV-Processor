// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.

#include "firmware.h"

#define OUTPORT 0x02000008

void print_chr(char ch)
{
	*((volatile uint32_t*)OUTPORT) = ch;
}

void print_str(const char *p)
{
	while (*p != 0)
		*((volatile uint32_t*)OUTPORT) = *(p++);
}

void print_dec(unsigned int val)
{
	char buffer[10];
	char *p = buffer;
	while (val || p == buffer) {
		*(p++) = val % 10;
		val = val / 10;
	}
	while (p != buffer) {
		*((volatile uint32_t*)OUTPORT) = '0' + *(--p);
	}
}

void print_hex(unsigned int val, int digits)
{
	for (int i = (4*digits)-4; i >= 0; i -= 4)
		*((volatile uint32_t*)OUTPORT) = "0123456789ABCDEF"[(val >> i) % 16];
}


// #include "firmware.h"
// #define reg_uart_data (*(volatile uint32_t*)0x02000008)
// #define reg_uart_clkdiv (*(volatile uint32_t*)0x02000004)

// void print_chr(char c)
// {
// 	if (c == '\n')
// 		print_chr('\r');
// 	reg_uart_data = c;
// }

// void print_str(const char *p)
// {
// 	//reg_uart_clkdiv = 104;
// 	while (*p)
// 		print_chr(*(p++));
// }

// // void print_hex(uint32_t v, int digits)
// // {
// // 	for (int i = 7; i >= 0; i--) {
// // 		char c = "0123456789abcdef"[(v >> (4*i)) & 15];
// // 		if (c == '0' && i >= digits) continue;
// // 		print_chr(c);
// // 		digits = i;
// // 	}
// // }

// // void print_dec(uint32_t v)
// // {
// // 	if (v >= 1000) {
// // 		print_str(">=1000");
// // 		return;
// // 	}

// // 	if      (v >= 900) { print_chr('9'); v -= 900; }
// // 	else if (v >= 800) { print_chr('8'); v -= 800; }
// // 	else if (v >= 700) { print_chr('7'); v -= 700; }
// // 	else if (v >= 600) { print_chr('6'); v -= 600; }
// // 	else if (v >= 500) { print_chr('5'); v -= 500; }
// // 	else if (v >= 400) { print_chr('4'); v -= 400; }
// // 	else if (v >= 300) { print_chr('3'); v -= 300; }
// // 	else if (v >= 200) { print_chr('2'); v -= 200; }
// // 	else if (v >= 100) { print_chr('1'); v -= 100; }

// // 	if      (v >= 90) { print_chr('9'); v -= 90; }
// // 	else if (v >= 80) { print_chr('8'); v -= 80; }
// // 	else if (v >= 70) { print_chr('7'); v -= 70; }
// // 	else if (v >= 60) { print_chr('6'); v -= 60; }
// // 	else if (v >= 50) { print_chr('5'); v -= 50; }
// // 	else if (v >= 40) { print_chr('4'); v -= 40; }
// // 	else if (v >= 30) { print_chr('3'); v -= 30; }
// // 	else if (v >= 20) { print_chr('2'); v -= 20; }
// // 	else if (v >= 10) { print_chr('1'); v -= 10; }

// // 	if      (v >= 9) { print_chr('9'); v -= 9; }
// // 	else if (v >= 8) { print_chr('8'); v -= 8; }
// // 	else if (v >= 7) { print_chr('7'); v -= 7; }
// // 	else if (v >= 6) { print_chr('6'); v -= 6; }
// // 	else if (v >= 5) { print_chr('5'); v -= 5; }
// // 	else if (v >= 4) { print_chr('4'); v -= 4; }
// // 	else if (v >= 3) { print_chr('3'); v -= 3; }
// // 	else if (v >= 2) { print_chr('2'); v -= 2; }
// // 	else if (v >= 1) { print_chr('1'); v -= 1; }
// // 	else print_chr('0');
// // }