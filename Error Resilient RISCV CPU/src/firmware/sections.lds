/*
This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.
*/

MEMORY {
	/* keep the firmware image compact and deterministic for the BCH wrapper */
	mem : ORIGIN = 0x00000000, LENGTH = 0x00004000
}

SECTIONS {
	.memory : {
		. = ORIGIN(mem);
		start*(.text);
		*(.text);
		*(.rodata*);
		*(.srodata*);
		*(.data*);
		*(.sdata*);
		*(.bss*);
		*(.sbss*);
		*(COMMON);
		*(*);
		end = .;
		. = ALIGN(4);
	} > mem

	_stack_top = ORIGIN(mem) + LENGTH(mem);
}
