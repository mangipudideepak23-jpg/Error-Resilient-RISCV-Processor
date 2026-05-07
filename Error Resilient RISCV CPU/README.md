# Combined BCH Error-Resilient RISC-V CPU

## Project Overview

This repository is the **combined integration** of two independent MTP (M.Tech Project) works,
both targeting BCH(45,32,t=2) error-resilience on the PicoRV32 RISC-V processor at IIT Delhi
under the supervision of **Prof. Kaushik Saha, Dept. of Electrical Engineering**.

The design protects the CPU from soft errors (single-event upsets) at **two independent layers**:

| Protection Layer | Stage | Author | Module |
|---|---|---|---|
| **Fetch-stage BCH** | Instruction Memory → CPU | Posam Naga Lingeswara Rao (2024EEN2321) | `ecc_instr_memory.v` |
| **Register-file BCH** | Decode / Writeback (x0–x31) | Deepak (2024EEN2859) | `bch_regfile.v` |

A shared BCH(45,32,t=2) engine (`bch_ecc.v`) powers both layers.

---

## BCH Code Parameters

| Parameter | Value |
|---|---|
| Code | BCH(45, 32, t=2) |
| Codeword length | 45 bits |
| Data bits | 32 bits |
| Parity bits | 12 bits + 1 pad bit |
| Galois field | GF(2^6) |
| Primitive polynomial | x^6 + x + 1 |
| Generator polynomial | x^12 + x^9 + x^8 + x^7 + x^4 + x^2 + 1 |
| Error correction | Up to 2 bits per word |
| Error detection | Up to 3+ bits per word (fatal flag) |
| Storage overhead | 40.6% (45/32 bits) |
| Decoder latency | 3 clock cycles (pipelined) |

---

## System Architecture

```
                    ┌─────────────────────────────────────────────────┐
                    │      picorv32_ecc_top  (unified top-level)       │
                    │                                                   │
  firmware.hex ─→  │  ┌──────────────────────────────────────────┐    │
                    │  │   ecc_instr_memory    [Fetch Stage]       │    │
                    │  │                                          │    │
                    │  │  instr_mem_bch[i] = encode(firmware[i])  │    │
                    │  │  On fetch:  codeword → BCH decoder (3cy) │    │
                    │  │             corrected instruction → CPU   │    │
                    │  │  On data:   raw memory (1-cy, no BCH)    │    │
                    │  │                                          │    │
                    │  │  Status: single_err_cnt / double_err_cnt │    │
                    │  │          fatal_err_cnt / bch_fatal        │    │
                    │  └──────────────────┬───────────────────────┘    │
                    │                     │ mem_rdata (corrected inst.) │
                    │  ┌──────────────────▼───────────────────────┐    │
                    │  │      PicoRV32 CPU Core (RV32IMC)         │    │
                    │  │   (PICORV32_REGS = bch_regfile)          │    │
                    │  └──────────────────┬───────────────────────┘    │
                    │                     │ register read/write         │
                    │  ┌──────────────────▼───────────────────────┐    │
                    │  │   bch_regfile      [Decode/WB Stage]     │    │
                    │  │                                          │    │
                    │  │  regfile_bch[x] = encode(wdata)          │    │
                    │  │  On read: codeword → BCH corrector (1cy) │    │
                    │  │           corrected rdata (combinational) │    │
                    │  │                                          │    │
                    │  │  Status: reg_file_single_err             │    │
                    │  │          reg_file_double_err             │    │
                    │  │          reg_file_fatal                  │    │
                    │  └──────────────────────────────────────────┘    │
                    │                                                   │
                    │  Peripherals (addr ≥ 0x0200_0000):               │
                    │    UART output   0x0200_0008                      │
                    │    test_status   0x0300_0000  (firmware signals)  │
                    │    test_sig      0x0300_0004                      │
                    └─────────────────────────────────────────────────┘
```

### Key design decisions in the integration

- **Single shared BCH engine** (`bch_ecc.v`): The same `bch_encoder_final` and
  `bch_decoder_prod` modules are compiled once and used by both `ecc_instr_memory`
  (fetch stage decoder, 3-cycle pipeline) and the `bch_regfile` (which embeds the
  same GF arithmetic as inline functions for fully-combinational 1-cycle correction).

- **PICORV32_REGS mechanism**: PicoRV32 supports a compile-time register-file
  override via `` `PICORV32_REGS ``. The `picorv32.v` used here already has
  `` `define PICORV32_REGS bch_regfile `` and a matching extended instantiation
  (connecting `resetn` and scan ports), so **no change to picorv32.v was needed
  beyond what Project 1 (register-file work) already did**.

- **Native mem_valid/mem_ready bus**: `ecc_instr_memory` uses PicoRV32's native
  memory interface. The CPU stalls gracefully by waiting for `mem_ready`, which
  `ecc_instr_memory` asserts only after the 3-cycle BCH decode completes.

- **Zero CPU modifications**: `picorv32.v` is completely unmodified.

---

## Directory Structure

```
Error Resilient RISCV CPU/
├── README.md
├── Makefile
├── src/
│   ├── rtl/
│   │   ├── picorv32.v            # Unmodified PicoRV32 CPU (RV32IMC)
│   │   ├── bch_ecc.v             # Shared BCH(45,32) encoder + decoder
│   │   ├── bch_regfile.v         # BCH register file x0-x31 [Deepak]
│   │   ├── ecc_instr_memory.v    # BCH instruction memory [Posam NL Rao]
│   │   └── picorv32_ecc_top.v    # Combined top-level (integration)
│   ├── firmware/
│   │   ├── hello.c               # 10-block test firmware (RV32IMC)
│   │   ├── start.S               # Startup code
│   │   ├── print.c               # UART print helpers
│   │   ├── irq.c                 # Interrupt handler
│   │   ├── sections.lds          # Linker script
│   │   ├── makehex.py            # .bin → Intel HEX converter
│   │   └── firmware.hex          # Pre-compiled firmware (ready to use)
│   └── testbench/
│       ├── tb_bch_unit.v         # Standalone BCH test (340 vectors)
│       └── tb_combined_ecc.v     # Full dual-protection CPU test
└── docs/
    ├── DESIGN.md                 # Detailed design document
    └── RESULTS.md                # Expected simulation results
```

---

## How to Simulate

### Prerequisites

- **Icarus Verilog** ≥ 10.x with `-g2012` support: `sudo apt install iverilog`
- **GTKWave** (optional, for waveform viewing): `sudo apt install gtkwave`
- **RISC-V GCC** (only if recompiling firmware): `riscv-none-embed-gcc`

### Run the standalone BCH unit test (no CPU needed)

```bash
make sim_bch_unit
```

Tests 340 vectors (20 RISC-V instruction patterns × clean + 8 single + 5 double + 3 triple faults).
Results written to `bch_decode_stage_results.csv`.  Expected: **340/340 PASS**.

### Run the full combined CPU simulation

```bash
make sim_combined
```

Runs the PicoRV32 CPU executing `hello.c` firmware with both protection layers active.
Tests all categories A/B/C (fetch faults, register faults, concurrent faults).
Results written to `combined_ecc_results.csv`.

### View waveforms

```bash
make sim_combined_vcd
gtkwave dump.vcd
```

### Recompile firmware (optional)

Only needed if you modify `hello.c`.

```bash
make firmware
```

---

## Fault Injection

### Fetch-stage faults

Drive the top-level ports of `picorv32_ecc_top`:

| Port | Description |
|---|---|
| `fi_fetch_en` | Enable fault injection |
| `fi_fetch_double` | Also flip a second bit (double fault) |
| `fi_fetch_word_idx` | Word address of instruction to corrupt |
| `fi_fetch_pos1` | Bit position in 45-bit codeword (0–44) |
| `fi_fetch_pos2` | Second bit position (when double enabled) |

### Register-file faults

Use hierarchical `force` in the testbench to drive internal signals of `bch_regfile`:

```verilog
// Single-bit fault on register x10, codeword bit 22
// (instance is named 'rf' inside picorv32, inside cpu_core)
force dut.cpu_core.rf.inject_reg_fault  = 1'b1;
force dut.cpu_core.rf.fault_reg_addr    = 6'd10;
force dut.cpu_core.rf.fault_reg_bit1    = 6'd22;
```

### Reading ECC status

```verilog
// Fetch stage (module outputs)
$display("single=%0d double=%0d fatal=%0d",
    dut.fetch_single_err_cnt,
    dut.fetch_double_err_cnt,
    dut.fetch_fatal_err_cnt);

// Register file (hierarchical access)
$display("rf_single=%0d rf_double=%0d rf_fatal=%0d",
    dut.cpu_core.rf.reg_file_single_err,
    dut.cpu_core.rf.reg_file_double_err,
    dut.cpu_core.rf.reg_file_fatal);
```

---

## Firmware Test Suite (hello.c)

The firmware exercises 10 computational blocks to validate that error correction
preserves correct CPU execution:

| Block | Operations | Key Checks |
|---|---|---|
| 1 | add, sub, and, or, mul, div, rem | 7 values |
| 2 | sll, srl, sra | 6 values |
| 3 | XOR-shift chains | 5 cascaded ops |
| 4 | slt, sltu (signed/unsigned compare) | 4 values |
| 5 | Multiply/divide stress (large operands) | 3 values |
| 6 | Fibonacci loop, fib(10)=55, fib(16)=987 | 2 values |
| 7 | Popcount loop (32 iterations) | 4 values |
| 8 | Byte reverse (8 iterations) | 2 values |
| 9 | Scratchpad store/load (8-element array) | 2 values |
| 10 | Alternating bitmask accumulation | 2 values |

On success, firmware writes `0x600d0001` to address `0x03000000`.

---

## Authors

| Contribution | Name | Entry Number |
|---|---|---|
| Fetch-stage BCH (AXI memory ECC) | Posam Naga Lingeswara Rao | 2024EEN2321 |
| Register-file BCH (x0–x31 ECC) | Deepak | 2024EEN2859 |
| Combined integration & top-level | Both | — |

**Supervisor**: Prof. Kaushik Saha, Department of Electrical Engineering, IIT Delhi

**Degree**: Master of Technology, Electrical Engineering — May 2026
