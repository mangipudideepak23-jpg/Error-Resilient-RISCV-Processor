"""
Generates bch_showcase_testcases.csv — 8 unique test cases for report use.
Each row is a DIFFERENT instruction under a DIFFERENT fault scenario.
"""

import csv, os, importlib.util

BASE = os.path.dirname(os.path.abspath(__file__))

# Load BCH engine from existing simulation module
spec = importlib.util.spec_from_file_location(
    "bch_sim", os.path.join(BASE, "simulate_bch_ecc.py"))
bch_mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bch_mod)
bch_encode = bch_mod.bch_encode
bch_decode = bch_mod.bch_decode

# ── 8 unique showcase cases ──────────────────────────────────────────────────
# Each entry: (data_hex, instruction_name, fault_type_int, fault_positions_tuple)
#   fault_type: 0=Clean, 1=Single-bit, 2=Double-bit, 3=Triple-bit
# All 8 have DIFFERENT instructions AND DIFFERENT fault scenarios.
SHOWCASE = [
    (0x00208033, "add  x1, x1, x2",       0, ()),
    (0x00000013, "addi x0, x0, 0  (NOP)", 1, (11,)),
    (0x02520633, "mul  x12, x4, x5",       1, (32,)),
    (0x00012283, "lw   x5, 0(x2)",         2, (0,  22)),
    (0x00312223, "sw   x3, 4(x2)",         2, (12, 38)),
    (0x00208463, "beq  x1, x2, +8",        3, (0,  10, 33)),
    (0x000000B7, "lui  x1, 0",             3, (3,  18, 40)),
    (0x004000EF, "jal  x1, 4",             1, (44,)),
]

FAULT_LABELS = {0: "Clean", 1: "1-bit flip", 2: "2-bit flip", 3: "3-bit flip"}

rows = []
for case_id, (word, name, ftype, fpos) in enumerate(SHOWCASE):
    clean_cw = bch_encode(word)

    # Inject faults
    inj_cw = clean_cw
    for p in fpos:
        inj_cw ^= (1 << p)

    data_out, fatal, emask, s2 = bch_decode(inj_cw)

    if ftype == 3:
        ok = fatal or (s2 != 0)     # detection is sufficient for triple-bit
    else:
        ok = (data_out == word) and (not fatal)

    pos_str = ", ".join(str(p) for p in fpos) if fpos else "N/A"
    data_recovered  = int(data_out == word and not fatal)
    # error_detected: 1 if the decoder applied any correction OR raised fatal.
    # emask!=0 means the Chien search found at least one error location.
    error_detected  = int(emask != 0 or fatal)

    rows.append({
        "case_id":          case_id,
        "instruction_hex":  f"0x{word:08X}",
        "instruction_name": name,
        "fault_scenario":   FAULT_LABELS[ftype],
        "fault_positions":  pos_str,
        "corrupted_cw_hex": f"0x{inj_cw:012X}",
        "decoded_hex":      f"0x{data_out:08X}",
        "data_recovered":   data_recovered,
        "error_detected":   error_detected,
        "result":           "PASS" if ok else "FAIL",
        "notes": (
            "No error injected - baseline"                    if ftype == 0 else
            f"BCH corrects single-bit flip at bit {fpos[0]}" if ftype == 1 else
            f"BCH corrects double-bit flip at bits {pos_str}" if ftype == 2 else
            "3-bit flip - uncorrectable; error flagged via sigma2"
        ),
    })

out_path = os.path.join(BASE, "bch_showcase_testcases.csv")
fields = ["case_id", "instruction_hex", "instruction_name", "fault_scenario",
          "fault_positions", "corrupted_cw_hex", "decoded_hex",
          "data_recovered", "error_detected", "result", "notes"]

with open(out_path, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=fields)
    w.writeheader()
    w.writerows(rows)

print(f"Wrote {len(rows)} showcase test cases -> {out_path}")
print()
print(f"{'ID':<4} {'Instruction':<26} {'Fault':<12} {'Positions':<12} {'Result':<6} Notes")
print("-" * 100)
for r in rows:
    print(f"{r['case_id']:<4} {r['instruction_name']:<26} {r['fault_scenario']:<12} "
          f"{r['fault_positions']:<12} {r['result']:<6} {r['notes']}")
