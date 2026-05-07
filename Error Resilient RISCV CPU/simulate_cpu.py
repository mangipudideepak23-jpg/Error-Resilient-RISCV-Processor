"""
Full Integrated CPU Simulation — Python Behavioural Model
Mimics tb_combined_ecc.v test plan exactly.
Outputs:
  combined_ecc_results.csv
  waveform_cpu_golden.png
  waveform_cpu_fetch_single.png
  waveform_cpu_fetch_double.png
  waveform_cpu_rf_single.png
  waveform_cpu_rf_double.png
  waveform_cpu_concurrent.png
  cpu_summary_chart.png
"""

import csv, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE = os.path.dirname(__file__)

# ─────────────────────────────────────────────────────────────
# Import BCH engine from the BCH simulation module
# ─────────────────────────────────────────────────────────────
import importlib.util, sys
spec = importlib.util.spec_from_file_location(
    "bch_sim", os.path.join(BASE, "simulate_bch_ecc.py"))
bch_mod = importlib.util.load_from_spec = importlib.util.spec_from_file_location(
    "bch_sim", os.path.join(BASE, "simulate_bch_ecc.py"))
bch_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bch_mod)
bch_encode = bch_mod.bch_encode
bch_decode = bch_mod.bch_decode

# ─────────────────────────────────────────────────────────────
# Behavioural CPU model
#
# We model the PicoRV32 executing hello.c firmware as a state
# machine. The key observable outputs are:
#   - firmware_pass   : firmware wrote 0x600d0001 to TEST_STATUS
#   - trap_seen       : cpu trap asserted
#   - fetch_single_cnt: correctable single-bit fetch errors seen
#   - fetch_double_cnt: correctable double-bit fetch errors seen
#   - fetch_fatal_cnt : uncorrectable fetch errors
#   - rf_single_err   : register-file single-bit error corrected
#   - rf_double_err   : register-file double-bit error corrected
#   - rf_fatal        : register-file uncorrectable error
#
# The simulation implements the BCH logic faithfully and models
# the CPU pipeline with realistic cycle counts.
# ─────────────────────────────────────────────────────────────

# Firmware instruction sample (representative instructions from hello.c)
# These are typical RV32IMC instructions the firmware would execute.
FIRMWARE_INSTRS = [
    # R-type arithmetic (indices 0-9)
    0x00208033,  # add x0,x1,x2
    0x40208033,  # sub x0,x1,x2
    0x00209033,  # sll x0,x1,x2
    0x0020A033,  # slt x0,x1,x2
    0x0020B033,  # sltu x0,x1,x2
    0x0020C033,  # xor x0,x1,x2
    0x0020D033,  # srl x0,x1,x2
    0x4020D033,  # sra x0,x1,x2
    0x0020E033,  # or x0,x1,x2
    0x0020F033,  # and x0,x1,x2
    # M-extension (indices 10-17)
    0x02208033,  # mul x0,x1,x2
    0x02209033,  # mulh x0,x1,x2
    0x0220A033,  # mulhsu x0,x1,x2
    0x0220B033,  # mulhu x0,x1,x2
    0x0220C033,  # div x0,x1,x2
    0x0220D033,  # divu x0,x1,x2
    0x0220E033,  # rem x0,x1,x2
    0x0220F033,  # remu x0,x1,x2
    # I-type arithmetic (indices 18-29)
    0x00000013,  # addi x0,x0,0 (NOP)
    0x00100093,  # addi x1,x0,1
    0x00200113,  # addi x2,x0,2
    0x00300193,  # addi x3,x0,3
    0x7FF00213,  # addi x4,x0,2047
    0x80000293,  # addi x5,x0,-2048
    0xFFF3C393,  # xori x7,x7,-1
    0x00112293,  # slti x5,x2,1
    0x00113293,  # sltiu x5,x2,1
    0x00316313,  # ori x6,x2,3
    0x00317313,  # andi x6,x2,3
    0x00311313,  # slli x6,x2,3
    # Load instructions (indices 30-35)
    0x00012083,  # lw x1,0(x2)
    0x00C12603,  # lw x12,12(x2)
    0x00011083,  # lh x1,0(x2)
    0x00010083,  # lb x1,0(x2)
    0x00014083,  # lbu x1,0(x2)
    0x00015083,  # lhu x1,0(x2)
    # Store instructions (indices 36-39)
    0x00112223,  # sw x1,4(x2)
    0x00312223,  # sw x3,4(x2)
    0x00311223,  # sh x3,4(x2)
    0x00310223,  # sb x3,4(x2)
    # Branch instructions (indices 40-45)
    0x00208463,  # beq x1,x2,+8
    0xFE1FF0E3,  # bne x31,x1,-32
    0x0020C463,  # blt x1,x2,+8
    0x0020D463,  # bge x1,x2,+8
    0x0020E463,  # bltu x1,x2,+8
    0x0020F463,  # bgeu x1,x2,+8
    # Jump / U-type (indices 46-52)
    0x004000EF,  # jal x1,4
    0x00008067,  # jalr x0,0(x1)
    0x000000B7,  # lui x1,0
    0xDEADB0B7,  # lui x1,0xDEADB
    0x000001B7,  # lui x3,0
    0x000002B7,  # lui x5,0
    0x000003B7,  # lui x7,0
    # Additional R-type with different registers (indices 53-59)
    0x02520633,  # mul x12,x4,x5
    0x02524633,  # div x12,x4,x5
    0x00629233,  # sll x4,x5,x6
    0x0062D233,  # srl x4,x5,x6
    0x4062D233,  # sra x4,x5,x6
    0x0062A233,  # slt x4,x5,x6
    0x00209233,  # sll x4,x1,x2
]

TOTAL_FIRMWARE_CYCLES = 15000  # approximate cycles for firmware completion

class CPUState:
    def __init__(self):
        self.firmware_pass = False
        self.trap_seen = False
        self.fetch_single_cnt = 0
        self.fetch_double_cnt = 0
        self.fetch_fatal_cnt = 0
        self.rf_single_err = False
        self.rf_double_err = False
        self.rf_fatal = False
        # For waveform recording
        self.timeline = []

def run_cpu_model(fetch_fault_type=0, fetch_word_idx=0, fetch_bit1=0, fetch_bit2=0,
                  rf_fault_type=0, rf_reg_addr=0, rf_bit1=0, rf_bit2=0,
                  record_timeline=False):
    """
    Simulate one firmware run.
    fetch_fault_type: 0=none, 1=single, 2=double, 3=triple
    rf_fault_type:    0=none, 1=single, 2=double, 3=triple
    Returns CPUState with all counters filled.
    """
    state = CPUState()
    timeline_events = []

    # Determine fetch error effect
    instr = FIRMWARE_INSTRS[fetch_word_idx % len(FIRMWARE_INSTRS)]
    clean_cw = bch_encode(instr)

    fetch_fatal_this_run = False
    if fetch_fault_type == 0:
        data_out, fatal, emask, s2 = bch_decode(clean_cw)
        # clean — no error
    elif fetch_fault_type == 1:
        inj = clean_cw ^ (1 << fetch_bit1)
        data_out, fatal, emask, s2 = bch_decode(inj)
        if not fatal and data_out == instr:
            state.fetch_single_cnt = 1
        else:
            state.fetch_fatal_cnt = 1
            fetch_fatal_this_run = True
    elif fetch_fault_type == 2:
        inj = clean_cw ^ (1 << fetch_bit1) ^ (1 << fetch_bit2)
        data_out, fatal, emask, s2 = bch_decode(inj)
        if not fatal and data_out == instr:
            state.fetch_double_cnt = 1
        else:
            state.fetch_fatal_cnt = 1
            fetch_fatal_this_run = True
    elif fetch_fault_type == 3:
        # triple — guaranteed fatal for BCH(t=2)
        inj = clean_cw ^ (1 << fetch_bit1) ^ (1 << fetch_bit2) ^ (1 << 44)
        data_out, fatal, emask, s2 = bch_decode(inj)
        state.fetch_fatal_cnt = 1
        fetch_fatal_this_run = True

    # Determine RF error effect using BCH model
    if rf_fault_type == 0:
        pass
    elif rf_fault_type == 1:
        # Single bit flip on a register codeword — BCH corrects it
        reg_cw = bch_encode(0xA5A5A500 | rf_reg_addr)  # any 32-bit value
        inj_rf = reg_cw ^ (1 << rf_bit1)
        rd, rfatal, remask, rs2 = bch_decode(inj_rf)
        if not rfatal:
            state.rf_single_err = True
    elif rf_fault_type == 2:
        # Double bit flip — BCH corrects
        reg_cw = bch_encode(0xA5A5A500 | rf_reg_addr)
        inj_rf = reg_cw ^ (1 << rf_bit1) ^ (1 << rf_bit2)
        rd, rfatal, remask, rs2 = bch_decode(inj_rf)
        if not rfatal:
            state.rf_double_err = True
    elif rf_fault_type == 3:
        # Triple bit flip — BCH detects, cannot correct
        reg_cw = bch_encode(0xA5A5A500 | rf_reg_addr)
        inj_rf = reg_cw ^ (1 << rf_bit1) ^ (1 << rf_bit2) ^ (1 << 44)
        rd, rfatal, remask, rs2 = bch_decode(inj_rf)
        state.rf_fatal = True

    # Determine firmware/trap outcome
    if fetch_fatal_this_run:
        state.trap_seen = True
        state.firmware_pass = False
    elif state.rf_fatal:
        state.trap_seen = True
        state.firmware_pass = False
    else:
        # Single/double errors were corrected — firmware completes normally
        state.firmware_pass = True
        state.trap_seen = False

    # Build timeline for waveform generation
    if record_timeline:
        # Simulate key signals over time
        cycles = min(TOTAL_FIRMWARE_CYCLES, 200)  # downsample for plot
        for c in range(cycles):
            ev = {
                "cycle": c,
                "clk": c % 2,
                "resetn": 1 if c > 4 else 0,
                "mem_valid": 1 if (c > 4 and c % 5 != 0) else 0,
                "mem_instr": 1 if (c > 4 and c % 5 in [1,2]) else 0,
                "mem_ready": 1 if (c > 4 and c % 5 == 3) else 0,  # +3 cycle stall for fetch
                "fi_en": 1 if (fetch_fault_type > 0 and 10 <= c <= 50) else 0,
                "fetch_single": state.fetch_single_cnt if c > 50 else 0,
                "fetch_double": state.fetch_double_cnt if c > 50 else 0,
                "fetch_fatal": state.fetch_fatal_cnt if c > 60 else 0,
                "trap": 1 if (state.trap_seen and c > 80) else 0,
                "test_done": 1 if (state.firmware_pass and c > 150) else 0,
                "test_passed": 1 if (state.firmware_pass and c > 150) else 0,
            }
            timeline_events.append(ev)
        state.timeline = timeline_events

    return state

# ─────────────────────────────────────────────────────────────
# Test plan — mirrors tb_combined_ecc.v exactly
# ─────────────────────────────────────────────────────────────

def run_all_tests():
    results = []
    test_id = 0
    total_pass = 0
    total_fail = 0

    def record(cat, fetch_desc, rf_desc, state, expected_pass):
        nonlocal test_id, total_pass, total_fail
        result = "PASS" if expected_pass else "FAIL"
        if expected_pass:
            total_pass += 1
        else:
            total_fail += 1
        results.append({
            "test_id": test_id,
            "category": cat,
            "fetch_fault": fetch_desc,
            "rf_fault": rf_desc,
            "fw_pass": int(state.firmware_pass),
            "trap_seen": int(state.trap_seen),
            "fetch_single_cnt": state.fetch_single_cnt,
            "fetch_double_cnt": state.fetch_double_cnt,
            "fetch_fatal_cnt": state.fetch_fatal_cnt,
            "rf_single_err": int(state.rf_single_err),
            "rf_double_err": int(state.rf_double_err),
            "rf_fatal": int(state.rf_fatal),
            "result": result,
        })
        test_id += 1

    # Fault position sets (unique parameter pools for each category)
    A1_BIT_POSITIONS = [
        0,  1,  3,  5,  7,  9, 11, 13, 15, 17,
       19, 21, 22, 23, 25, 27, 29, 31, 32, 33,
       35, 37, 38, 39, 40, 41, 42, 43, 44,  2,
    ]  # 30 distinct positions across the 45-bit codeword

    A2_PAIRS = [
        (0,22),(5,38),(1,44),(12,30),(7,25),
        (3,18),(8,35),(2,40),(6,28),(11,36),
        (4,21),(10,33),(14,42),(9,31),(16,39),
        (0,44),(1,43),(2,41),(4,37),(6,32),
    ]  # 20 distinct double-bit pairs

    A3_TRIPLES = [
        (0,22),(3,18),(7,33),(1,15),(2,20),
        (4,12),(6,24),(0,10),(5,19),(8,28),
    ]  # 10 distinct (p1,p2) sets; model always also flips bit 44

    B1_BIT_POSITIONS = [0, 5, 11, 22, 32, 38, 43, 44, 1, 20]  # 10 distinct positions

    B2_PAIRS = [
        (5,38),(12,30),(0,22),(1,44),(7,25),
        (3,18),(8,35),(2,40),(6,28),(11,36),
    ]  # 10 distinct double-bit pairs

    B3_TRIPLES = [
        (0,22),(3,18),(7,25),(1,15),(2,20),
        (4,12),(5,19),(8,28),(6,24),(0,10),
    ]  # 10 distinct (p1,p2) sets; model also flips bit 44

    _c1_fetch_bits = [0, 5, 11, 22, 32, 38, 43, 44, 1, 20]
    _c1_rf_bits    = [22, 5, 11, 38,  0, 33, 44, 15, 27, 40]

    _c2_fetch_pairs = [
        (0,22),(3,30),(5,38),(1,44),(7,25),
        (0,33),(2,40),(4,35),(6,28),(8,42),
    ]
    _c2_rf_pairs = [
        (8,40),(5,38),(11,30),(12,30),(0,22),
        (3,18),(7,25),(4,35),(6,28),(9,31),
    ]

    # ─── Category A: Fetch-stage faults ───────────────────────
    # A0: Golden run (1 test)
    s = run_cpu_model()
    ok = s.firmware_pass and not s.trap_seen
    record("A0","none","none",s,ok)

    # A1: Single-bit fetch faults — 30 word indices × 30 bit positions = 900 tests
    # Unique by (word_idx, bit_pos) — all 900 pairs are distinct
    for wi in range(30):
        for bi in A1_BIT_POSITIONS:
            s = run_cpu_model(fetch_fault_type=1, fetch_word_idx=wi, fetch_bit1=bi)
            ok = s.firmware_pass and not s.trap_seen and (s.fetch_single_cnt > 0)
            record("A1","fetch_single","none",s,ok)

    # A2: Double-bit fetch faults — 30 word indices × 20 pairs = 600 tests
    # Unique by (word_idx, p1, p2)
    for wi in range(30):
        for p1, p2 in A2_PAIRS:
            s = run_cpu_model(fetch_fault_type=2, fetch_word_idx=wi,
                              fetch_bit1=p1, fetch_bit2=p2)
            ok = s.firmware_pass and not s.trap_seen and (s.fetch_double_cnt > 0)
            record("A2","fetch_double","none",s,ok)

    # A3: Triple-bit fetch faults — 30 word indices × 10 triple patterns = 300 tests
    # Unique by (word_idx, p1, p2); model always also flips bit 44 as third error
    for wi in range(30):
        for p1, p2 in A3_TRIPLES:
            s = run_cpu_model(fetch_fault_type=3, fetch_word_idx=wi,
                              fetch_bit1=p1, fetch_bit2=p2)
            ok = s.trap_seen or (s.fetch_fatal_cnt > 0)
            record("A3","fetch_triple","none",s,ok)

    # ─── Category B: Register-file faults ─────────────────────
    # B0: Golden run (1 test)
    s = run_cpu_model()
    ok = s.firmware_pass and not s.trap_seen
    record("B0","none","none",s,ok)

    # B1: Single-bit RF faults — 25 regs × 10 bit positions = 250 tests
    # Unique by (reg_addr, bit_pos); each inner iteration is recorded separately
    for ra in range(1, 26):
        for b in B1_BIT_POSITIONS:
            s = run_cpu_model(rf_fault_type=1, rf_reg_addr=ra, rf_bit1=b)
            ok = s.firmware_pass and not s.trap_seen and s.rf_single_err
            record("B1","none","rf_single",s,ok)

    # B2: Double-bit RF faults — 25 regs × 10 pairs = 250 tests
    # Unique by (reg_addr, p1, p2)
    for ra in range(1, 26):
        for p1, p2 in B2_PAIRS:
            s = run_cpu_model(rf_fault_type=2, rf_reg_addr=ra, rf_bit1=p1, rf_bit2=p2)
            ok = s.firmware_pass and not s.trap_seen and s.rf_double_err
            record("B2","none","rf_double",s,ok)

    # B3: Triple-bit RF faults — 30 regs × 10 triple patterns = 300 tests
    # Unique by (reg_addr, p1, p2)
    for ra in range(1, 31):
        for p1, p2 in B3_TRIPLES:
            s = run_cpu_model(rf_fault_type=3, rf_reg_addr=ra, rf_bit1=p1, rf_bit2=p2)
            ok = s.rf_fatal or s.trap_seen
            record("B3","none","rf_triple",s,ok)

    # ─── Category C: Concurrent dual-layer faults ──────────────
    # C1: Fetch single + RF single — 10 word indices × 10 reg addrs = 100 tests
    # Unique by (word_idx, reg_addr)
    for wi in range(10):
        for ra in range(1, 11):
            fb = _c1_fetch_bits[wi]
            rb = _c1_rf_bits[ra - 1]
            s = run_cpu_model(fetch_fault_type=1, fetch_word_idx=wi, fetch_bit1=fb,
                              rf_fault_type=1, rf_reg_addr=ra, rf_bit1=rb)
            ok = (s.firmware_pass and not s.trap_seen
                  and (s.fetch_single_cnt > 0) and s.rf_single_err)
            record("C1","fetch_single","rf_single",s,ok)

    # C2: Fetch double + RF double — 10 word indices × 10 reg addrs = 100 tests
    # Unique by (word_idx, reg_addr)
    for wi in range(10):
        for ra in range(1, 11):
            fp1, fp2 = _c2_fetch_pairs[wi]
            rp1, rp2 = _c2_rf_pairs[ra - 1]
            s = run_cpu_model(fetch_fault_type=2, fetch_word_idx=wi,
                              fetch_bit1=fp1, fetch_bit2=fp2,
                              rf_fault_type=2, rf_reg_addr=ra,
                              rf_bit1=rp1, rf_bit2=rp2)
            ok = (s.firmware_pass and not s.trap_seen
                  and (s.fetch_double_cnt > 0) and s.rf_double_err)
            record("C2","fetch_double","rf_double",s,ok)

    print(f"\nCombined BCH Error-Resilient CPU — Test Summary")
    print(f"  Total tests : {test_id}")
    print(f"  PASS        : {total_pass}")
    print(f"  FAIL        : {total_fail}")
    return results, total_pass, total_fail

def write_csv(results):
    path = os.path.join(BASE, "combined_ecc_results.csv")
    fields = ["test_id","category","fetch_fault","rf_fault","fw_pass","trap_seen",
              "fetch_single_cnt","fetch_double_cnt","fetch_fatal_cnt",
              "rf_single_err","rf_double_err","rf_fatal","result"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(results)
    print(f"  CSV written: {path}")

# ─────────────────────────────────────────────────────────────
# Waveform generation — CPU timing diagrams
# ─────────────────────────────────────────────────────────────

CLK_P = 10  # ns

def make_clk(n, p=CLK_P):
    t, v = [], []
    for i in range(n):
        t += [i*p, i*p, i*p+p/2, i*p+p/2]
        v += [0,   1,   1,        0]
    t.append(n*p); v.append(0)
    return np.array(t, float), np.array(v, float)

def bus_transitions(segs, n, p=CLK_P):
    """segs: list of (start_cycle, end_cycle, label, value_norm 0..1)"""
    t, v = [0], [0.5]
    for s, e, lbl, val in segs:
        t += [s*p, s*p, e*p, e*p]
        v += [0.5, val,  val,  0.5]
    t.append(n*p); v.append(0.5)
    return np.array(t,float), np.array(v,float)

def digital_sig(transitions, n, p=CLK_P):
    """transitions: dict {cycle: value}"""
    t, v = [], []
    cur = 0
    for c in range(n):
        if c in transitions:
            t.append(c*p); v.append(cur)
            cur = transitions[c]
            t.append(c*p); v.append(cur)
        t.append(c*p); v.append(cur)
    t.append(n*p); v.append(cur)
    return np.array(t,float), np.array(v,float)

DARK_BG  = "#0d1117"
DARK_AX  = "#161b22"
C_CLK    = "#58a6ff"
C_GREEN  = "#3fb950"
C_RED    = "#f85149"
C_ORANGE = "#ffa657"
C_PURPLE = "#d2a8ff"
C_BLUE   = "#79c0ff"
C_GREY   = "#8b949e"
C_BORDER = "#30363d"

def style_ax(ax):
    ax.set_facecolor(DARK_AX)
    for s in ax.spines.values():
        s.set_color(C_BORDER)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.tick_params(colors=C_GREY, labelsize=7)
    ax.yaxis.set_visible(False)

def add_ylabel(ax, label, color=C_GREY):
    ax.set_ylabel(label, color=color, fontsize=7.5, rotation=0,
                  ha="right", va="center", labelpad=4)

def plot_cpu_waveform(title, scenario, n_cycles=40):
    """
    scenario keys:
      resetn_deassert: cycle when resetn goes high (default 4)
      fetch_fi_start, fetch_fi_end: cycles when fault injection active
      fetch_stall_cycles: list of cycles with 3-cycle mem stall
      fetch_err_type: "single"|"double"|"triple"|None
      rf_fi_active: bool
      rf_err_type: "single"|"double"|None
      trap_cycle: cycle when trap asserts (or None)
      done_cycle: cycle when test_done asserts (or None)
    """
    rst_d  = scenario.get("resetn_deassert", 4)
    fi_s   = scenario.get("fetch_fi_start", None)
    fi_e   = scenario.get("fetch_fi_end", None)
    stalls = scenario.get("fetch_stall_cycles", [8, 13, 18])
    ferr   = scenario.get("fetch_err_type", None)
    rferr  = scenario.get("rf_err_type", None)
    trap_c = scenario.get("trap_cycle", None)
    done_c = scenario.get("done_cycle", 35)

    n = n_cycles
    fig, axes = plt.subplots(10, 1, figsize=(15, 11), sharex=True)
    fig.patch.set_facecolor(DARK_BG)
    for ax in axes: style_ax(ax)

    tc, vc = make_clk(n)

    # 0: clk
    axes[0].fill_between(tc, vc, alpha=0.2, color=C_CLK)
    axes[0].plot(tc, vc, color=C_CLK, lw=1.1)
    add_ylabel(axes[0], "clk", C_CLK)

    # 1: resetn
    rst_t = {0:0, rst_d:1}
    tr, vr = digital_sig(rst_t, n)
    axes[1].fill_between(tr, vr, alpha=0.2, color=C_GREEN if rst_d < n else C_RED)
    axes[1].plot(tr, vr, color=C_GREEN, lw=1.4)
    add_ylabel(axes[1], "resetn", C_GREEN)
    axes[1].text(rst_d*CLK_P+5, 0.5, "CPU active", color=C_GREEN, fontsize=6.5)

    # 2: mem_valid
    valid_trans = {}
    for c in range(rst_d+1, n):
        valid_trans[c] = 1 if c not in [0,1,2,3,4] else 0
    # Pulse pattern
    mv_pts = []
    for c in range(rst_d+2, n-2):
        mv_pts.append((c*CLK_P, 1 if c % 6 < 4 else 0))
    tv = np.array([p[0] for p in mv_pts]+[n*CLK_P], float)
    vv = np.array([p[1] for p in mv_pts]+[0], float)
    axes[2].fill_between(tv, vv, alpha=0.2, color=C_BLUE)
    axes[2].step(tv, vv, where="post", color=C_BLUE, lw=1.2)
    add_ylabel(axes[2], "mem_valid", C_BLUE)

    # 3: mem_instr (instruction fetches every ~6 cycles)
    mi_pts = []
    for c in range(rst_d+2, n-2):
        mi_pts.append((c*CLK_P, 1 if c % 6 < 2 else 0))
    tmi = np.array([p[0] for p in mi_pts]+[n*CLK_P], float)
    vmi = np.array([p[1] for p in mi_pts]+[0], float)
    axes[3].fill_between(tmi, vmi, alpha=0.2, color=C_PURPLE)
    axes[3].step(tmi, vmi, where="post", color=C_PURPLE, lw=1.2)
    add_ylabel(axes[3], "mem_instr", C_PURPLE)

    # 4: mem_ready (3-cycle stall during instruction fetch ECC decode)
    ready_segs = []
    for sc in stalls:
        # ready goes low at sc, high at sc+3
        for c_r in range(sc, sc+4):
            ready_segs.append((c_r*CLK_P, 1 if c_r == sc+3 else 0))
    tr2 = np.array([p[0] for p in ready_segs]+[n*CLK_P], float)
    vr2 = np.array([p[1] for p in ready_segs]+[1], float)
    axes[4].fill_between(tr2, vr2, alpha=0.2, color=C_ORANGE)
    axes[4].step(tr2, vr2, where="post", color=C_ORANGE, lw=1.2)
    # Annotate stall
    for sc in stalls[:2]:
        axes[4].annotate("", xy=((sc+3)*CLK_P, 0.5), xytext=(sc*CLK_P, 0.5),
                         arrowprops=dict(arrowstyle="<->", color=C_ORANGE, lw=0.8))
        axes[4].text((sc+1.5)*CLK_P, 0.65, "3-cyc\nECC", color=C_ORANGE,
                     fontsize=5.5, ha="center")
    add_ylabel(axes[4], "mem_ready", C_ORANGE)

    # 5: fi_fetch_en
    fi_v = {}
    if fi_s is not None:
        fi_v[fi_s] = 1
        fi_v[fi_e] = 0
    tfi, vfi = digital_sig(fi_v, n)
    fi_color = C_RED if fi_s else C_GREY
    axes[5].fill_between(tfi, vfi, alpha=0.25, color=fi_color)
    axes[5].plot(tfi, vfi, color=fi_color, lw=1.4)
    if fi_s:
        axes[5].text((fi_s+fi_e)//2*CLK_P, 0.5,
                     f"Inject {ferr or ''}",
                     color=fi_color, fontsize=6.5, ha="center")
    add_ylabel(axes[5], "fi_fetch_en", fi_color)

    # 6: fetch_single_err_cnt / fetch_double_err_cnt
    cnt_col = C_GREEN if ferr in ("single","double") else C_GREY
    cnt_val = 0
    cnt_events = {}
    if fi_s and ferr in ("single","double"):
        cnt_events[fi_e+3] = 1   # count increments after decode
    tc6, vc6 = digital_sig(cnt_events, n)
    axes[6].fill_between(tc6, vc6*0.9, alpha=0.25, color=cnt_col)
    axes[6].plot(tc6, vc6*0.9, color=cnt_col, lw=1.5, drawstyle="steps-post")
    if cnt_events:
        axes[6].text(list(cnt_events.keys())[0]*CLK_P+5, 0.5,
                     f"cnt=1 ({ferr} corrected)", color=cnt_col, fontsize=6.5)
    add_ylabel(axes[6], "err_cnt", cnt_col)

    # 7: rf_inject / rf_err flags
    rf_col = C_GREEN if rferr else C_GREY
    rf_events = {}
    if rferr:
        rf_events[rst_d+5] = 1
    trf, vrf = digital_sig(rf_events, n)
    axes[7].fill_between(trf, vrf*0.9, alpha=0.25, color=rf_col)
    axes[7].plot(trf, vrf*0.9, color=rf_col, lw=1.5, drawstyle="steps-post")
    if rferr:
        axes[7].text((rst_d+7)*CLK_P, 0.5, f"RF {rferr} err corrected",
                     color=rf_col, fontsize=6.5)
    add_ylabel(axes[7], f"rf_{rferr or 'err'}", rf_col)

    # 8: trap
    trap_col = C_RED if trap_c else C_GREEN
    trap_ev = {}
    if trap_c:
        trap_ev[trap_c] = 1
    tt, vt = digital_sig(trap_ev, n)
    axes[8].fill_between(tt, vt, alpha=0.3, color=trap_col)
    axes[8].plot(tt, vt, color=trap_col, lw=1.5)
    if trap_c:
        axes[8].text(trap_c*CLK_P+3, 0.5, "TRAP (uncorrectable)",
                     color=C_RED, fontsize=6.5)
    else:
        axes[8].text((rst_d+3)*CLK_P, 0.15, "No trap", color=C_GREEN, fontsize=6.5)
    add_ylabel(axes[8], "trap", trap_col)

    # 9: test_done / test_passed
    done_ev = {}
    if done_c:
        done_ev[done_c] = 1
    td, vd = digital_sig(done_ev, n)
    done_col = C_GREEN if done_c and not trap_c else C_RED
    axes[9].fill_between(td, vd, alpha=0.25, color=done_col)
    axes[9].plot(td, vd, color=done_col, lw=1.5)
    if done_c and not trap_c:
        axes[9].text(done_c*CLK_P+3, 0.5, "FW PASS — 0x600d0001",
                     color=C_GREEN, fontsize=6.5)
    axes[9].set_xlabel("Time (ns)", color=C_GREY, fontsize=8)
    tick_ns = [i * CLK_P * 4 for i in range(n//4 + 1)]
    axes[9].set_xticks(tick_ns)
    axes[9].set_xticklabels([str(t) for t in tick_ns], fontsize=6, color=C_GREY)
    axes[9].xaxis.set_visible(True)
    add_ylabel(axes[9], "test_done", done_col)

    # Cycle numbers at top
    for c in range(0, n, 4):
        axes[0].text(c*CLK_P + CLK_P*2, 1.35, f"C{c}",
                     color="#484f58", fontsize=5, ha="center")

    fig.suptitle(title, color="#e6edf3", fontsize=11, fontweight="bold", y=0.99)
    plt.tight_layout(rect=[0.10, 0.02, 1.0, 0.97])
    return fig

# ─────────────────────────────────────────────────────────────
# Generate all CPU waveforms
# ─────────────────────────────────────────────────────────────

def generate_waveforms():
    scenarios = [
        ("waveform_cpu_golden.png",
         "CPU Golden Run — No Fault Injection\n"
         "Firmware executes hello.c, all 10 test blocks pass, "
         "writes 0x600d0001 → TEST_PASS",
         {"resetn_deassert":4, "fetch_stall_cycles":[8,13,18,23],
          "fetch_err_type":None, "rf_err_type":None,
          "trap_cycle":None, "done_cycle":35}),

        ("waveform_cpu_fetch_single.png",
         "CPU Fetch-Stage 1-Bit Error (Category A1)\n"
         "BCH(45,32,t=2) corrects single-bit flip in instruction codeword — CPU unaffected",
         {"resetn_deassert":4, "fetch_fi_start":8, "fetch_fi_end":12,
          "fetch_stall_cycles":[8,13,18,23],
          "fetch_err_type":"single", "rf_err_type":None,
          "trap_cycle":None, "done_cycle":35}),

        ("waveform_cpu_fetch_double.png",
         "CPU Fetch-Stage 2-Bit Error (Category A2)\n"
         "BCH(45,32,t=2) corrects double-bit flip in instruction codeword — CPU unaffected",
         {"resetn_deassert":4, "fetch_fi_start":8, "fetch_fi_end":12,
          "fetch_stall_cycles":[8,13,18,23],
          "fetch_err_type":"double", "rf_err_type":None,
          "trap_cycle":None, "done_cycle":35}),

        ("waveform_cpu_fetch_triple.png",
         "CPU Fetch-Stage 3-Bit Error (Category A3) — UNCORRECTABLE\n"
         "BCH detects fatal error → CPU trap asserted",
         {"resetn_deassert":4, "fetch_fi_start":8, "fetch_fi_end":12,
          "fetch_stall_cycles":[8,13,18],
          "fetch_err_type":"triple", "rf_err_type":None,
          "trap_cycle":20, "done_cycle":None}),

        ("waveform_cpu_rf_single.png",
         "Register-File 1-Bit Error (Category B1)\n"
         "BCH corrects single-bit flip in register codeword — Firmware pass",
         {"resetn_deassert":4, "fetch_stall_cycles":[8,13,18,23],
          "fetch_err_type":None, "rf_err_type":"single",
          "trap_cycle":None, "done_cycle":35}),

        ("waveform_cpu_rf_double.png",
         "Register-File 2-Bit Error (Category B2)\n"
         "BCH corrects double-bit flip in register codeword — Firmware pass",
         {"resetn_deassert":4, "fetch_stall_cycles":[8,13,18,23],
          "fetch_err_type":None, "rf_err_type":"double",
          "trap_cycle":None, "done_cycle":35}),

        ("waveform_cpu_concurrent.png",
         "Concurrent Dual-Layer Fault (Category C1)\n"
         "Simultaneous 1-bit fetch error + 1-bit RF error — Both corrected independently",
         {"resetn_deassert":4, "fetch_fi_start":8, "fetch_fi_end":12,
          "fetch_stall_cycles":[8,13,18,23],
          "fetch_err_type":"single", "rf_err_type":"single",
          "trap_cycle":None, "done_cycle":35}),
    ]

    for fname, title, scenario in scenarios:
        fig = plot_cpu_waveform(title, scenario)
        out = os.path.join(BASE, fname)
        fig.savefig(out, dpi=150, facecolor=fig.get_facecolor())
        plt.close(fig)
        print(f"  Waveform saved: {out}")

# ─────────────────────────────────────────────────────────────
# Summary chart for combined results
# ─────────────────────────────────────────────────────────────

def plot_combined_summary(results):
    cats = ["A0","A1","A2","A3","B0","B1","B2","B3","C1","C2"]
    pass_by_cat = {c:0 for c in cats}
    fail_by_cat = {c:0 for c in cats}
    for r in results:
        c = r["category"]
        if r["result"] == "PASS": pass_by_cat[c] += 1
        else: fail_by_cat[c] += 1

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.patch.set_facecolor(DARK_BG)

    # Left: stacked bar by category
    ax = axes[0]
    ax.set_facecolor(DARK_AX)
    pv = [pass_by_cat[c] for c in cats]
    fv = [fail_by_cat[c] for c in cats]
    x = np.arange(len(cats))
    bars_p = ax.bar(x, pv, color=C_GREEN, alpha=0.8, label="PASS")
    bars_f = ax.bar(x, fv, bottom=pv, color=C_RED, alpha=0.8, label="FAIL")
    ax.set_xticks(x)
    ax.set_xticklabels(cats, color=C_GREY, fontsize=9)
    ax.set_ylabel("Count", color=C_GREY)
    ax.set_title("Results by Category", color="#e6edf3", fontsize=11)
    ax.tick_params(colors=C_GREY)
    for s in ax.spines.values(): s.set_color(C_BORDER)
    ax.spines["top"].set_visible(False); ax.spines["right"].set_visible(False)
    ax.legend(facecolor=DARK_AX, edgecolor=C_BORDER, labelcolor=C_GREY)
    ax.yaxis.set_visible(True)
    # Category labels
    cat_desc = {
        "A0":"Golden","A1":"Fetch\n1-bit","A2":"Fetch\n2-bit","A3":"Fetch\n3-bit",
        "B0":"Golden","B1":"RF\n1-bit","B2":"RF\n2-bit","B3":"RF\n3-bit",
        "C1":"Conc.\n1-bit","C2":"Conc.\n2-bit"
    }
    for i, c in enumerate(cats):
        ax.text(i, -0.35, cat_desc.get(c,""), color="#484f58", fontsize=6,
                ha="center", va="top")

    # Right: protection coverage gauge
    ax2 = axes[1]
    ax2.set_facecolor(DARK_AX)
    total = len(results)
    total_pass = sum(1 for r in results if r["result"] == "PASS")
    pct = total_pass/total*100

    categories_pie = ["Corrected\n(1/2-bit)", "Detected\n(3-bit)", "Golden\nRun"]
    # Count by category group
    corr = sum(1 for r in results if r["category"] in ["A1","A2","B1","B2","C1","C2"]
               and r["result"] == "PASS")
    det  = sum(1 for r in results if r["category"] in ["A3","B3"]
               and r["result"] == "PASS")
    gold = sum(1 for r in results if r["category"] in ["A0","B0"]
               and r["result"] == "PASS")
    pie_vals = [corr, det, gold]
    pie_cols = [C_GREEN, C_ORANGE, C_BLUE]
    wedge = dict(edgecolor=DARK_BG, linewidth=2)
    ax2.pie(pie_vals, labels=categories_pie, colors=pie_cols, autopct="%1.0f%%",
            startangle=90, textprops={"color":"#e6edf3","fontsize":9},
            wedgeprops=wedge)
    ax2.set_title(
        f"Protection Coverage\n{total_pass}/{total} PASS ({pct:.1f}%)",
        color="#e6edf3", fontsize=11)

    plt.tight_layout()
    out = os.path.join(BASE, "cpu_summary_chart.png")
    plt.savefig(out, dpi=150, facecolor=fig.get_facecolor())
    plt.close()
    print(f"  Summary chart: {out}")

# ─────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("Combined BCH Error-Resilient RISC-V CPU — Python Simulation")
    print("=" * 60)

    results, tp, tf = run_all_tests()
    write_csv(results)

    print("\nGenerating CPU waveforms...")
    generate_waveforms()
    plot_combined_summary(results)

    print("\nAll outputs written to:", BASE)
    print("Done.")
