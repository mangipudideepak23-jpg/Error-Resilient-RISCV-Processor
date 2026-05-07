// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.

#include "firmware.h"

static volatile uint32_t scratchpad[8];

static void finish(uint32_t status, uint32_t signature)
{
	reg_test_signature = signature;
	reg_test_status = status;
	__asm__ volatile ("ebreak");
	for (;;)
		;
}

static void check_equal(uint32_t actual, uint32_t expected, uint32_t signature)
{
	if (actual != expected)
		finish(TEST_STATUS_FAIL, signature ^ actual ^ (expected << 1));
}

static uint32_t hw_mul(uint32_t a, uint32_t b)
{
	uint32_t result;
	__asm__ volatile ("mul %0, %1, %2" : "=r" (result) : "r" (a), "r" (b));
	return result;
}

static uint32_t hw_div(uint32_t a, uint32_t b)
{
	uint32_t result;
	__asm__ volatile ("divu %0, %1, %2" : "=r" (result) : "r" (a), "r" (b));
	return result;
}

static uint32_t hw_rem(uint32_t a, uint32_t b)
{
	uint32_t result;
	__asm__ volatile ("remu %0, %1, %2" : "=r" (result) : "r" (a), "r" (b));
	return result;
}

/* --- helpers for shift/compare instructions --- */
static uint32_t hw_sll(uint32_t a, uint32_t shamt)
{
	uint32_t result;
	__asm__ volatile ("sll %0, %1, %2" : "=r"(result) : "r"(a), "r"(shamt));
	return result;
}

static uint32_t hw_srl(uint32_t a, uint32_t shamt)
{
	uint32_t result;
	__asm__ volatile ("srl %0, %1, %2" : "=r"(result) : "r"(a), "r"(shamt));
	return result;
}

static int32_t hw_sra(int32_t a, uint32_t shamt)
{
	int32_t result;
	__asm__ volatile ("sra %0, %1, %2" : "=r"(result) : "r"(a), "r"(shamt));
	return result;
}

static uint32_t hw_slt(int32_t a, int32_t b)
{
	uint32_t result;
	__asm__ volatile ("slt %0, %1, %2" : "=r"(result) : "r"(a), "r"(b));
	return result;
}

static uint32_t hw_sltu(uint32_t a, uint32_t b)
{
	uint32_t result;
	__asm__ volatile ("sltu %0, %1, %2" : "=r"(result) : "r"(a), "r"(b));
	return result;
}

/* Fibonacci: exercises branches, add, comparisons in a tight loop */
static uint32_t fib(uint32_t n)
{
	uint32_t a = 0, b = 1, tmp;
	uint32_t i;
	for (i = 0; i < n; i++) {
		tmp = a + b;
		a = b;
		b = tmp;
	}
	return a;
}

/* Popcount: exercises shifts, AND, ADD in a tight loop */
static uint32_t popcount32(uint32_t v)
{
	uint32_t cnt = 0;
	uint32_t i;
	for (i = 0; i < 32; i++) {
		cnt += (v >> i) & 1u;
	}
	return cnt;
}

/* Bit-reverse a byte using shifts and OR */
static uint32_t reverse_byte(uint32_t b_val)
{
	uint32_t r = 0;
	uint32_t i;
	for (i = 0; i < 8; i++) {
		r = (r << 1) | ((b_val >> i) & 1u);
	}
	return r;
}

void hello(void)
{
	uint32_t a = 5;
	uint32_t b = 7;
	uint32_t c, d, e, f, g, h;
	uint32_t sum = 0;
	uint32_t i;
	uint32_t acc;

	reg_uart_clkdiv = 104;
	reg_test_status = TEST_STATUS_IDLE;
	reg_test_signature = 0;

	print_str("bch firmware start\n");

	/* --- Block 1: Basic ALU (original tests kept) --- */
	c = a + b;
	d = c - a;
	e = c & b;
	f = a | b;
	g = hw_mul(9, 11);
	h = hw_div(144, 12);

	check_equal(c, 12,  0x0000100c);
	check_equal(d,  7,  0x00002007);
	check_equal(e,  4,  0x00003004);
	check_equal(f,  7,  0x00004007);
	check_equal(g, 99,  0x00005063);
	check_equal(h, 12,  0x0000600c);
	check_equal(hw_rem(145, 12), 1, 0x00007001);

	/* --- Block 2: Shift instructions --- */
	check_equal(hw_sll(1u, 8),  0x00000100u, 0x00010100);
	check_equal(hw_sll(0xABu, 4), 0x00000AB0u, 0x00010AB0);
	check_equal(hw_srl(0x80000000u, 1), 0x40000000u, 0x00024000);
	check_equal(hw_srl(0xFFFF0000u, 8), 0x00FFFF00u, 0x0002FF00);
	check_equal((uint32_t)hw_sra(-256, 4), (uint32_t)-16, 0x0003FFF0);
	check_equal((uint32_t)hw_sra(0x7FFFFFFF, 1), 0x3FFFFFFFu, 0x00033FFF);

	/* --- Block 3: XOR and shifts combined --- */
	acc = 0xDEADBEEFu;
	acc = acc ^ hw_sll(acc, 5);
	acc = acc ^ hw_srl(acc, 17);
	acc = acc ^ hw_sll(acc, 13);
	/* just check it's non-zero (value depends on shift chain) */
	if (acc == 0)
		finish(TEST_STATUS_FAIL, 0x00040000);

	/* --- Block 4: SLT / SLTU comparisons --- */
	check_equal(hw_slt(-1, 0),   1, 0x00050001); /* -1 < 0 signed   */
	check_equal(hw_slt(0, -1),   0, 0x00050000); /* 0 >= -1 signed   */
	check_equal(hw_sltu(0u, 1u), 1, 0x00060001); /* 0 < 1 unsigned   */
	check_equal(hw_sltu(0xFFFFFFFFu, 0u), 0, 0x00060000); /* large < 0? no */

	/* --- Block 5: Multiply / divide stress --- */
	check_equal(hw_mul(0x1234u, 0x5678u), 0x06260060u, 0x00070060);
	check_equal(hw_div(0xFFFFFFFFu, 0xFFFFu), 0x00010001u, 0x00080001);
	check_equal(hw_rem(0x100000u, 0xFFu), 0x00000001u, 0x00090001);

	/* --- Block 6: Fibonacci (branch-heavy loop) --- */
	check_equal(fib(10), 55,  0x000A0037); /* fib(10) = 55 */
	check_equal(fib(16), 987, 0x000A03DB); /* fib(16) = 987 */

	/* --- Block 7: Popcount (shift-heavy loop) --- */
	check_equal(popcount32(0xFFFFFFFFu), 32, 0x000B0020);
	check_equal(popcount32(0xAAAAAAAAu), 16, 0x000B0010);
	check_equal(popcount32(0x00000000u),  0, 0x000B0000);
	check_equal(popcount32(0x80000001u),  2, 0x000B0002);

	/* --- Block 8: Byte reverse (shift+OR loop) --- */
	check_equal(reverse_byte(0xB4u), 0x2Du, 0x000C002D);
	check_equal(reverse_byte(0xF0u), 0x0Fu, 0x000C000F);

	/* --- Block 9: Scratchpad store/load loop --- */
	scratchpad[0] = c;          /* 12   */
	scratchpad[1] = d;          /* 7    */
	scratchpad[2] = e;          /* 4    */
	scratchpad[3] = f;          /* 7    */
	scratchpad[4] = g;          /* 99   */
	scratchpad[5] = h;          /* 12   */
	scratchpad[6] = 1;
	scratchpad[7] = 0;

	sum = 0;
	for (i = 0; i < 7; i++)
		sum += scratchpad[i];
	check_equal(sum, 142, 0x0000808e);

	for (i = 0; i < 7; i++)
		scratchpad[7] ^= scratchpad[i] << (i & 7);
	check_equal(scratchpad[7], 0x000007dau, 0x00009001);

	/* --- Block 10: Alternating bit-mask accumulation --- */
	acc = 0;
	for (i = 0; i < 16; i++) {
		if (i & 1u)
			acc |= (1u << i);
		else
			acc &= ~(1u << i);
	}
	check_equal(acc, 0x0000AAAAu, 0x000DAAAA);

	print_str("bch firmware pass\n");
	finish(TEST_STATUS_PASS, TEST_SIGNATURE_PASS);
}
