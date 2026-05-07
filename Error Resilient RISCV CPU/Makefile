# =============================================================
# Makefile — Combined BCH Error-Resilient RISC-V CPU
#
# Requires: iverilog (≥10, -g2012), vvp, RISC-V GCC toolchain
#
# Quick-start:
#   make sim_bch_unit      — standalone BCH encoder/decoder test (340 vectors)
#   make sim_combined      — full dual-protection CPU simulation
#   make firmware          — recompile firmware.hex from hello.c
#
# RISC-V toolchain prefix — edit to match your local installation:
# =============================================================

TOOLCHAIN_PREFIX ?= riscv-none-embed-
# Or use:  riscv32-unknown-elf-  /  riscv64-linux-gnu- (with -m32)

PYTHON   = python3
IVERILOG = iverilog
VVP      = vvp

# Simulation flags
# -g2012  : SystemVerilog-2012 (needed for begin:label local regs)
# -DPICORV32_REGS=bch_regfile : routes register file through BCH wrapper
# Note: picorv32.v already contains "`define PICORV32_REGS bch_regfile"
# so the -D flag is not strictly required but is kept for documentation.
SIM_FLAGS = -g2012

RTL_DIR = src/rtl
TB_DIR  = src/testbench
FW_DIR  = src/firmware

# Core RTL source list (order matters for some simulators)
RTL_SRCS = $(RTL_DIR)/picorv32.v     \
           $(RTL_DIR)/bch_ecc.v      \
           $(RTL_DIR)/bch_regfile.v  \
           $(RTL_DIR)/ecc_instr_memory.v \
           $(RTL_DIR)/picorv32_ecc_top.v

# -------------------------------------------------------
# Simulation targets
# -------------------------------------------------------

# 1. Standalone BCH unit test (no CPU)
sim_bch_unit: tb_bch_unit.vvp $(FW_DIR)/firmware.hex
	$(VVP) -N $<
	@echo ""
	@echo "Results written to: bch_decode_stage_results.csv"

tb_bch_unit.vvp: $(TB_DIR)/tb_bch_unit.v \
                 $(RTL_DIR)/bch_ecc.v
	$(IVERILOG) $(SIM_FLAGS) -s tb_bch_unit -o $@ $^
	chmod -x $@

# 2. Full combined CPU simulation (fetch + register-file protection)
sim_combined: tb_combined_ecc.vvp $(FW_DIR)/firmware.hex
	$(VVP) -N $<
	@echo ""
	@echo "Results written to: combined_ecc_results.csv"

tb_combined_ecc.vvp: $(TB_DIR)/tb_combined_ecc.v $(RTL_SRCS)
	$(IVERILOG) $(SIM_FLAGS) -s tb_combined_ecc -o $@ $^
	chmod -x $@

# 3. Waveform dumps (append +vcd to any sim target)
sim_combined_vcd: tb_combined_ecc.vvp $(FW_DIR)/firmware.hex
	$(VVP) -N $< +vcd
	@echo "Waveform: dump.vcd"

sim_bch_unit_vcd: tb_bch_unit.vvp
	$(VVP) -N $< +vcd

# -------------------------------------------------------
# Firmware rebuild (only needed if you change hello.c)
# -------------------------------------------------------
FIRMWARE_WORDS = 32768   # 128 KB / 4

firmware: $(FW_DIR)/firmware.hex

$(FW_DIR)/firmware.hex: $(FW_DIR)/firmware.bin $(FW_DIR)/makehex.py
	$(PYTHON) $(FW_DIR)/makehex.py $< $(FIRMWARE_WORDS) > $@

$(FW_DIR)/firmware.bin: $(FW_DIR)/firmware.elf
	$(TOOLCHAIN_PREFIX)objcopy -O binary $< $@
	chmod -x $@

$(FW_DIR)/firmware.elf: $(FW_DIR)/start.o $(FW_DIR)/print.o \
                        $(FW_DIR)/hello.o  $(FW_DIR)/sections.lds
	$(TOOLCHAIN_PREFIX)gcc -Os -mabi=ilp32 -march=rv32imc \
	    -ffreestanding -nostdlib -o $@ \
	    -Wl,--build-id=none,-Bstatic,-T,$(FW_DIR)/sections.lds,-Map,$(FW_DIR)/firmware.map,--strip-debug \
	    $(FW_DIR)/start.o $(FW_DIR)/print.o $(FW_DIR)/hello.o -lgcc
	chmod -x $@

$(FW_DIR)/start.o: $(FW_DIR)/start.S
	$(TOOLCHAIN_PREFIX)gcc -c -mabi=ilp32 -march=rv32imc -o $@ $<

$(FW_DIR)/print.o: $(FW_DIR)/print.c
	$(TOOLCHAIN_PREFIX)gcc -c -mabi=ilp32 -march=rv32imc -Os --std=c99 \
	    -ffreestanding -nostdlib -o $@ $<

$(FW_DIR)/hello.o: $(FW_DIR)/hello.c
	$(TOOLCHAIN_PREFIX)gcc -c -mabi=ilp32 -march=rv32imc -Os --std=c99 \
	    -ffreestanding -nostdlib -o $@ $<

# -------------------------------------------------------
# Housekeeping
# -------------------------------------------------------
clean:
	rm -f tb_bch_unit.vvp tb_combined_ecc.vvp
	rm -f bch_decode_stage_results.csv combined_ecc_results.csv
	rm -f dump.vcd
	rm -f $(FW_DIR)/firmware.elf $(FW_DIR)/firmware.bin \
	      $(FW_DIR)/firmware.map \
	      $(FW_DIR)/start.o $(FW_DIR)/print.o $(FW_DIR)/hello.o

.PHONY: sim_bch_unit sim_combined sim_combined_vcd sim_bch_unit_vcd \
        firmware clean
