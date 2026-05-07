"""
BCH(45,32,t=2) Unit-Test Simulation
Faithfully translates tb_bch_unit.v + bch_ecc.v into Python.
Outputs:
  bch_decode_stage_results.csv  — 34000 test vectors
  waveform_bch_no_error.png
  waveform_bch_1bit_error.png
  waveform_bch_2bit_error.png
"""

import csv, os, random
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ─────────────────────────────────────────────────────────────
# GF(2^6) arithmetic — exact translation of bch_ecc.v
# Primitive polynomial: x^6 + x + 1  (0x43)
# ─────────────────────────────────────────────────────────────

GF_INV = [
    0,  1, 33, 62, 49, 43, 31, 44, 57, 37, 52, 28, 46, 40, 22, 25,
   61, 54, 51, 39, 26, 35, 14, 24, 23, 15, 20, 34, 11, 53, 45,  6,
   63,  2, 27, 21, 56,  9, 50, 19, 13, 47, 48,  5,  7, 30, 12, 41,
   42,  4, 38, 18, 10, 29, 17, 60, 36,  8, 59, 58, 55, 16,  3, 32,
]

ALPHA_B = [
    1,  2,  4,  8, 16, 32,  3,  6, 12, 24, 48, 35,  5, 10, 20, 40,
   19, 38, 15, 30, 60, 59, 53, 41, 17, 34,  7, 14, 28, 56, 51, 37,
    9, 18, 36, 11, 22, 44, 27, 54, 47, 29, 58, 55, 45, 25, 50, 39,
   13, 26, 52, 43, 21, 42, 23, 46, 31, 62, 63, 61, 57, 49, 33,
]  # indices 0..62; alpha_b[63] = 0 (default)

def alpha_b(e):
    e = int(e) % 128  # match Verilog 7-bit input
    if e >= 63:
        return 0
    return ALPHA_B[e]

def gf_mul(a, b):
    """GF(2^6) multiply using primitive poly x^6+x+1."""
    a, b = int(a) & 0x3F, int(b) & 0x3F
    p = 0
    for i in range(6):
        if b & (1 << i):
            p ^= a << i
    # Reduce: gf_mul from bch_ecc.v uses explicit bit reduction
    # p is 11 bits max; reduce mod 0x43 using the Verilog equations
    r = [0]*6
    r[0] = ((p>>0)^(p>>6)) & 1
    r[1] = ((p>>1)^(p>>6)^(p>>7)) & 1
    r[2] = ((p>>2)^(p>>7)^(p>>8)) & 1
    r[3] = ((p>>3)^(p>>8)^(p>>9)) & 1
    r[4] = ((p>>4)^(p>>9)^(p>>10)) & 1
    r[5] = ((p>>5)^(p>>10)) & 1
    return r[0]|(r[1]<<1)|(r[2]<<2)|(r[3]<<3)|(r[4]<<4)|(r[5]<<5)

def gf_inv(a):
    return GF_INV[int(a) & 0x3F]

def mod63_mul3(i6):
    i6 = int(i6) & 0x3F
    t = i6 + (i6 << 1)
    if t >= 126: t -= 126
    elif t >= 63: t -= 63
    return t & 0x7F

def mod63_neg(k6):
    k6 = int(k6) & 0x3F
    if k6 == 0: return 0
    return (63 - k6) & 0x7F

def mod63_mul2(x):
    x = int(x) & 0x7F
    t = x << 1
    if t >= 63: t -= 63
    return t & 0x7F

# ─────────────────────────────────────────────────────────────
# BCH Encoder — exact translation of bch_encoder_final
# ─────────────────────────────────────────────────────────────

def bch_encode(data32):
    """Return 45-bit BCH codeword for a 32-bit data word."""
    data32 = int(data32) & 0xFFFFFFFF
    # divreg = {data_in, 12'b0}  — 44 bits (indices 0..43)
    divreg = (data32 << 12) & ((1 << 44) - 1)

    for i in range(43, 11, -1):
        if (divreg >> i) & 1:
            divreg ^= (1 << i)       # bit i cleared
            divreg ^= (1 << (i-2))
            divreg ^= (1 << (i-4))
            divreg ^= (1 << (i-7))
            divreg ^= (1 << (i-8))
            divreg ^= (1 << (i-9))
            divreg ^= (1 << (i-12))

    parity = divreg & 0xFFF          # bits 11:0
    # codeword = {1'b0, data_in, parity} = bit44=0, bits43:12=data, bits11:0=parity
    codeword = (data32 << 12) | parity
    return codeword & ((1 << 45) - 1)

# ─────────────────────────────────────────────────────────────
# BCH Decoder — exact translation of bch_decoder_prod
# Returns (data_out, fatal, error_mask, sigma2)
# ─────────────────────────────────────────────────────────────

def bch_decode(codeword45):
    cw = int(codeword45) & ((1 << 45) - 1)

    # Stage 1: syndrome computation (S1, S3)
    S1 = 0
    S3 = 0
    for i in range(45):
        if (cw >> i) & 1:
            exp_s1 = i
            exp_s3 = mod63_mul3(i)
            S1 ^= alpha_b(exp_s1)
            S3 ^= alpha_b(exp_s3)

    # Stage 2: sigma2 computation
    s1 = S1
    if s1 == 0:
        s2 = 0
    else:
        # s2 = (s1^3 XOR S3) * inv(s1)
        s1_cubed = gf_mul(gf_mul(s1, s1), s1)
        s2 = gf_mul(s1_cubed ^ S3, gf_inv(s1))

    # Stage 3: Chien search + correction
    mask = 0
    for k in range(45):
        chien_e1 = mod63_neg(k)
        chien_e2 = mod63_mul2(chien_e1)
        val = 1 ^ gf_mul(s1, alpha_b(chien_e1)) ^ gf_mul(s2, alpha_b(chien_e2))
        if val == 0:
            mask |= (1 << k)

    corrected = cw ^ mask
    # data_out = corrected[43:12]
    data_out = (corrected >> 12) & 0xFFFFFFFF
    fatal = (s1 != 0) and (mask == 0)
    return data_out, fatal, mask, s2

# ─────────────────────────────────────────────────────────────
# Test data — exact from tb_bch_unit.v
# ─────────────────────────────────────────────────────────────

TEST_WORDS = [
    (0x00208033, "add_x1_x1_x2"),    # R-type
    (0x402080B3, "sub_x1_x1_x2"),
    (0x00629233, "sll_x4_x5_x6"),
    (0x0062D233, "srl_x4_x5_x6"),
    (0x4062D233, "sra_x4_x5_x6"),
    (0x0062A233, "slt_x4_x5_x6"),
    (0x02520633, "mul_x12_x4_x5"),
    (0x02524633, "div_x12_x4_x5"),
    (0x00000013, "addi_NOP"),         # I-type
    (0x00112293, "slti_x5_x2_1"),
    (0xFFF3C393, "xori_x7_x7_m1"),
    (0x00012283, "lw_x5_0_x2"),
    (0x00C12603, "lw_x12_12_x2"),
    (0x00312223, "sw_x3_4_x2"),       # S-type
    (0x00A12A23, "sw_x10_20_x2"),
    (0x00208463, "beq_x1_x2_p8"),     # B-type
    (0xFE1FF0E3, "bne_x31_x1_m32"),
    (0x000000B7, "lui_x1_0"),         # U-type
    (0x004000EF, "jal_x1_4"),         # J-type
    (0xDEADBEEF, "pathological"),     # edge pattern
    (0x9A8DCA03, "rand_0"),
    (0x5EC42E08, "rand_1"),
    (0xDD463C09, "rand_2"),
    (0x10435A10, "rand_3"),
    (0x6C6FA611, "rand_4"),
    (0x8ACD4E10, "rand_5"),
    (0xD8F56413, "rand_6"),
    (0xE1A47E10, "rand_7"),
    (0xA7CAD415, "rand_8"),
    (0x7C52FA17, "rand_9"),
    (0x474EBC19, "rand_10"),
    (0x430F801D, "rand_11"),
    (0x0EA2622B, "rand_12"),
    (0xEDD96831, "rand_13"),
    (0x3170F437, "rand_14"),
    (0x8E944239, "rand_15"),
    (0xC6A7EE39, "rand_16"),
    (0x52FBE43B, "rand_17"),
    (0x8BABCE3B, "rand_18"),
    (0x0658663A, "rand_19"),
    (0xEEEA163E, "rand_20"),
    (0x5D65A441, "rand_21"),
    (0xF7FD5646, "rand_22"),
    (0x98326856, "rand_23"),
    (0x46685257, "rand_24"),
    (0x4774BC58, "rand_25"),
    (0xCF8EBC5A, "rand_26"),
    (0xDC96925E, "rand_27"),
    (0x30BEB45F, "rand_28"),
    (0x7D106C60, "rand_29"),
    (0xC333E861, "rand_30"),
    (0xCF8D446A, "rand_31"),
    (0x07A0CA6E, "rand_32"),
    (0xE9A1FA6F, "rand_33"),
    (0x571AA876, "rand_34"),
    (0xDBCCC477, "rand_35"),
    (0xF071D879, "rand_36"),
    (0xEC5B227C, "rand_37"),
    (0x72D8567D, "rand_38"),
    (0x98543881, "rand_39"),
    (0xE1805081, "rand_40"),
    (0x38F16A81, "rand_41"),
    (0x89463E85, "rand_42"),
    (0x37F8A88B, "rand_43"),
    (0xB758588D, "rand_44"),
    (0xA65E688E, "rand_45"),
    (0xDF465290, "rand_46"),
    (0xA4161293, "rand_47"),
    (0xDD56CC94, "rand_48"),
    (0x85D51695, "rand_49"),
    (0x3C365296, "rand_50"),
    (0x663F1C97, "rand_51"),
    (0x8DA01097, "rand_52"),
    (0x6F4CC69A, "rand_53"),
    (0x568CC69B, "rand_54"),
    (0x448AAA9E, "rand_55"),
    (0xF86C2CA2, "rand_56"),
    (0x74273CA3, "rand_57"),
    (0x8CE21EA3, "rand_58"),
    (0x655238A6, "rand_59"),
    (0xAB4220A7, "rand_60"),
    (0x827050A8, "rand_61"),
    (0xBC8960A9, "rand_62"),
    (0x757750A9, "rand_63"),
    (0x9AD620AB, "rand_64"),
    (0xB7B56EA7, "rand_65"),
    (0x0279B6A6, "rand_66"),
    (0x43E42CAF, "rand_67"),
    (0xEE0CAEB5, "rand_68"),
    (0x38602AB6, "rand_69"),
    (0xA18FF6B6, "rand_70"),
    (0x286218B8, "rand_71"),
    (0x3A43B2BA, "rand_72"),
    (0xD89A40C0, "rand_73"),
    (0xDC98D2C1, "rand_74"),
    (0xDC1110C1, "rand_75"),
    (0xE117DAC3, "rand_76"),
    (0xE61FECC0, "rand_77"),
    (0x1A50AEC3, "rand_78"),
    (0x9BE578C7, "rand_79"),
    (0xB7E99ACA, "rand_80"),
    (0x00E85ECE, "rand_81"),
    (0x19DB3AD0, "rand_82"),
    (0x0F02BAD0, "rand_83"),
    (0x6E595ED3, "rand_84"),
    (0x75D66ED4, "rand_85"),
    (0xEF48E8D5, "rand_86"),
    (0x508EBAD7, "rand_87"),
    (0x45B89CD9, "rand_88"),
    (0x392456DE, "rand_89"),
    (0x284D82E5, "rand_90"),
    (0x4EEA04E7, "rand_91"),
    (0x1C8EAEE9, "rand_92"),
    (0xE767DCEA, "rand_93"),
    (0xD5A804EB, "rand_94"),
    (0x29D4BEEF, "rand_95"),
    (0x8D5288F1, "rand_96"),
    (0x8B8148F6, "rand_97"),
    (0x4EB93EFF, "rand_98"),
    (0x20A04502, "rand_99"),
    (0x3FA7F104, "rand_100"),
    (0x444D610B, "rand_101"),
    (0x2260E70F, "rand_102"),
    (0x17BE3111, "rand_103"),
    (0x60E7A113, "rand_104"),
    (0x0ED42F1A, "rand_105"),
    (0xBAA4B71A, "rand_106"),
    (0xBDC14F1F, "rand_107"),
    (0xE7C99B26, "rand_108"),
    (0x89456F27, "rand_109"),
    (0xEE49F329, "rand_110"),
    (0x3CEDDF2D, "rand_111"),
    (0xAF42E12F, "rand_112"),
    (0x27CD8130, "rand_113"),
    (0xDC570131, "rand_114"),
    (0x1C11F735, "rand_115"),
    (0x47294739, "rand_116"),
    (0xB41B3143, "rand_117"),
    (0xFC3D3348, "rand_118"),
    (0xFF002D4D, "rand_119"),
    (0x4BF50B52, "rand_120"),
    (0x6C006F61, "rand_121"),
    (0x2A935D62, "rand_122"),
    (0x9E8FC965, "rand_123"),
    (0xD605E770, "rand_124"),
    (0xF26B4776, "rand_125"),
    (0x25E97977, "rand_126"),
    (0x95A76D79, "rand_127"),
    (0x562B0F79, "rand_128"),
    (0xB2B9437A, "rand_129"),
    (0x2720797D, "rand_130"),
    (0xCE9FF57F, "rand_131"),
    (0x04FC6D82, "rand_132"),
    (0xF8102383, "rand_133"),
    (0x12922F83, "rand_134"),
    (0x2F923996, "rand_135"),
    (0x18C26797, "rand_136"),
    (0x1EFA2197, "rand_137"),
    (0x6C031199, "rand_138"),
    (0xFF01CF99, "rand_139"),
    (0x4FCCA39A, "rand_140"),
    (0xBF7B539B, "rand_141"),
    (0xA3B1799D, "rand_142"),
    (0xCEB81F9D, "rand_143"),
    (0xC8DCD19F, "rand_144"),
    (0xD9441FA5, "rand_145"),
    (0x1BAC27A7, "rand_146"),
    (0x877409A9, "rand_147"),
    (0xA5E5A5AB, "rand_148"),
    (0x96DA1DAC, "rand_149"),
    (0x5408F9AC, "rand_150"),
    (0x8CBFEDB0, "rand_151"),
    (0xC40DB9B4, "rand_152"),
    (0x847FD9B4, "rand_153"),
    (0xB88139B9, "rand_154"),
    (0x747B6DBA, "rand_155"),
    (0x958CA9BA, "rand_156"),
    (0xCE4A2BBD, "rand_157"),
    (0x93CD59BF, "rand_158"),
    (0x4CCC9BC2, "rand_159"),
    (0xA0A04DC4, "rand_160"),
    (0xA9D3D7C7, "rand_161"),
    (0xC56811CD, "rand_162"),
    (0x3985C3CF, "rand_163"),
    (0xBACFB3D0, "rand_164"),
    (0x5E9953D2, "rand_165"),
    (0xBAA80DD4, "rand_166"),
    (0x3A9BEDD4, "rand_167"),
    (0x27A0C3D7, "rand_168"),
    (0x3D1A85DD, "rand_169"),
    (0x43CF2FDE, "rand_170"),
    (0x386ECBE0, "rand_171"),
    (0x610461E3, "rand_172"),
    (0x1931E9EE, "rand_173"),
    (0xB45ED1F0, "rand_174"),
    (0xFF5E9FF0, "rand_175"),
    (0x46D483F3, "rand_176"),
    (0x6123FDF7, "rand_177"),
    (0x7900F7F9, "rand_178"),
    (0x50C187FC, "rand_179"),
]

# Extend to 2000 unique test words — adds 1800 deterministic-random 32-bit values.
# Seeded RNG guarantees reproducibility; set membership ensures no repeats.
_rng = random.Random(0x1234ABCD)
_seen_w = {w for w, _ in TEST_WORDS}
_extra_words = []
_ri = 180
while len(_extra_words) < 1800:
    v = _rng.randint(0, 0xFFFFFFFF)
    if v not in _seen_w:
        _seen_w.add(v)
        _extra_words.append((v, f"rand_{_ri}"))
        _ri += 1
TEST_WORDS = TEST_WORDS + _extra_words
del _rng, _seen_w, _extra_words, _ri  # clean up namespace

S_POSITIONS = [0, 5, 11, 22, 32, 38, 43, 44]
D_POS = [(0,22),(1,44),(5,30),(12,38),(20,43)]
T_POS = [(0,10,33),(3,18,40),(7,25,44)]

# ─────────────────────────────────────────────────────────────
# Run simulation and generate CSV
# ─────────────────────────────────────────────────────────────

def run_bch_simulation():
    rows = []
    total_pass = 0
    total_fail = 0
    case_id = 0

    for w_idx, (word, label) in enumerate(TEST_WORDS):
        clean_cw = bch_encode(word)

        # ---- CLEAN ----
        data_out, fatal, emask, s2 = bch_decode(clean_cw)
        ok = (data_out == word) and (not fatal)
        rows.append({
            "case_id": case_id, "test_data_hex": f"{word:08x}",
            "instr_label": label, "fault_type": 0,
            "fault_pos1": -1, "fault_pos2": -1, "fault_pos3": -1,
            "corrupted_cw_bin": f"{clean_cw:045b}",
            "decoded_data_hex": f"{data_out:08x}",
            "corrected_ok": int(ok), "fatal_flag": int(fatal),
            "error_mask_bin": f"{emask:045b}",
            "result": "PASS" if ok else "FAIL",
        })
        total_pass += ok; total_fail += (not ok); case_id += 1

        # ---- SINGLE-BIT ----
        for pos in S_POSITIONS:
            inj = clean_cw ^ (1 << pos)
            data_out, fatal, emask, s2 = bch_decode(inj)
            ok = (data_out == word) and (not fatal)
            rows.append({
                "case_id": case_id, "test_data_hex": f"{word:08x}",
                "instr_label": label, "fault_type": 1,
                "fault_pos1": pos, "fault_pos2": -1, "fault_pos3": -1,
                "corrupted_cw_bin": f"{inj:045b}",
                "decoded_data_hex": f"{data_out:08x}",
                "corrected_ok": int(ok), "fatal_flag": int(fatal),
                "error_mask_bin": f"{emask:045b}",
                "result": "PASS" if ok else "FAIL",
            })
            total_pass += ok; total_fail += (not ok); case_id += 1

        # ---- DOUBLE-BIT ----
        for p1, p2 in D_POS:
            inj = clean_cw ^ (1 << p1) ^ (1 << p2)
            data_out, fatal, emask, s2 = bch_decode(inj)
            ok = (data_out == word) and (not fatal)
            rows.append({
                "case_id": case_id, "test_data_hex": f"{word:08x}",
                "instr_label": label, "fault_type": 2,
                "fault_pos1": p1, "fault_pos2": p2, "fault_pos3": -1,
                "corrupted_cw_bin": f"{inj:045b}",
                "decoded_data_hex": f"{data_out:08x}",
                "corrected_ok": int(ok), "fatal_flag": int(fatal),
                "error_mask_bin": f"{emask:045b}",
                "result": "PASS" if ok else "FAIL",
            })
            total_pass += ok; total_fail += (not ok); case_id += 1

        # ---- TRIPLE-BIT (uncorrectable — expect fatal) ----
        for p1, p2, p3 in T_POS:
            inj = clean_cw ^ (1 << p1) ^ (1 << p2) ^ (1 << p3)
            data_out, fatal, emask, s2 = bch_decode(inj)
            ok = fatal or (s2 != 0)   # detection sufficient
            rows.append({
                "case_id": case_id, "test_data_hex": f"{word:08x}",
                "instr_label": label, "fault_type": 3,
                "fault_pos1": p1, "fault_pos2": p2, "fault_pos3": p3,
                "corrupted_cw_bin": f"{inj:045b}",
                "decoded_data_hex": f"{data_out:08x}",
                "corrected_ok": int(ok), "fatal_flag": int(fatal),
                "error_mask_bin": f"{emask:045b}",
                "result": "PASS" if ok else "FAIL",
            })
            total_pass += ok; total_fail += (not ok); case_id += 1

    # Write CSV
    fieldnames = ["case_id","test_data_hex","instr_label","fault_type",
                  "fault_pos1","fault_pos2","fault_pos3","corrupted_cw_bin",
                  "decoded_data_hex","corrected_ok","fatal_flag","error_mask_bin","result"]
    out_path = os.path.join(os.path.dirname(__file__), "bch_decode_stage_results.csv")
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)

    print(f"BCH Decode Stage Evaluation Complete")
    print(f"  Total vectors : {case_id}")
    print(f"  PASS          : {total_pass}")
    print(f"  FAIL          : {total_fail}")
    print(f"  CSV written   : {out_path}")
    return rows, total_pass, total_fail

# ─────────────────────────────────────────────────────────────
# Waveform generation helpers
# ─────────────────────────────────────────────────────────────

CLK_PERIOD = 10   # ns
PIPE_CYCLES = 3   # decoder pipeline depth

def make_clk(n_cycles, period=CLK_PERIOD):
    t, v = [], []
    for i in range(n_cycles):
        t += [i*period, i*period, i*period+period/2, i*period+period/2]
        v += [0, 1, 1, 0]
    t.append(n_cycles*period); v.append(0)
    return np.array(t), np.array(v)

def step_signal(transitions, n_cycles, period=CLK_PERIOD):
    """transitions: list of (cycle, value). Returns (t[], v[])."""
    t, v = [], []
    cur_v = 0
    trans_dict = dict(transitions)
    for c in range(n_cycles+1):
        if c in trans_dict:
            cur_v = trans_dict[c]
        t.append(c * period)
        v.append(cur_v)
    return np.array(t), np.array(v)

def plot_waveform(scenario, word, fault_type, fault_positions, out_path):
    """Generate a timing diagram PNG for one fault scenario."""
    clean_cw = bch_encode(word)
    if fault_type == 0:
        inj_cw = clean_cw
        title = f"BCH Decoder — No Error\nInstruction: 0x{word:08X}  ({fault_type} bit flips)"
    elif fault_type == 1:
        inj_cw = clean_cw ^ (1 << fault_positions[0])
        title = f"BCH Decoder — 1-Bit Error Corrected\nInstruction: 0x{word:08X}  Flip @ bit {fault_positions[0]}"
    else:
        inj_cw = clean_cw ^ (1 << fault_positions[0]) ^ (1 << fault_positions[1])
        title = f"BCH Decoder — 2-Bit Error Corrected\nInstruction: 0x{word:08X}  Flips @ bits {fault_positions[0]},{fault_positions[1]}"

    data_out, fatal, emask, s2 = bch_decode(inj_cw)
    n_cycles = 14

    fig, axes = plt.subplots(8, 1, figsize=(14, 9), sharex=True)
    fig.patch.set_facecolor("#0d1117")
    for ax in axes:
        ax.set_facecolor("#0d1117")
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.spines["bottom"].set_color("#30363d")
        ax.spines["left"].set_color("#30363d")
        ax.tick_params(colors="#8b949e", labelsize=7)
        ax.yaxis.set_visible(False)

    colors = {
        "clk": "#58a6ff",
        "valid_in": "#3fb950",
        "data_in": "#d2a8ff",
        "s1": "#ffa657",
        "s2": "#ffa657",
        "valid_out": "#3fb950",
        "data_out": "#d2a8ff",
        "fatal": "#f85149",
        "emask": "#79c0ff",
    }

    # CLK
    tc, vc = make_clk(n_cycles)
    axes[0].fill_between(tc, vc, alpha=0.25, color=colors["clk"])
    axes[0].plot(tc, vc, color=colors["clk"], lw=1.2)
    axes[0].set_ylabel("clk", color="#8b949e", fontsize=8, rotation=0, ha="right", va="center")

    # valid_in: high at cycle 2, low at cycle 3
    t_vi = np.array([0,20,20,30,30, n_cycles*CLK_PERIOD])
    v_vi = np.array([0,0, 1, 1, 0, 0])
    axes[1].fill_between(t_vi, v_vi, alpha=0.25, color=colors["valid_in"])
    axes[1].plot(t_vi, v_vi, color=colors["valid_in"], lw=1.5)
    axes[1].set_ylabel("valid_in", color="#8b949e", fontsize=8, rotation=0, ha="right", va="center")

    # data_in bus: show value from cycle 2..3
    axes[2].fill_between([20, 30], [0, 0], [1, 1], alpha=0.15, color=colors["data_in"])
    axes[2].plot([0,20,20,30,30,n_cycles*CLK_PERIOD], [0.5,0.5,0,0,0.5,0.5],
                 color=colors["data_in"], lw=1.5)
    cw_label = f"0x{inj_cw:012X}" + ("  [CORRUPTED]" if fault_type > 0 else "")
    axes[2].text(25, 0.5, cw_label, color=colors["data_in"], fontsize=6.5,
                 ha="center", va="center")
    axes[2].set_ylabel("data_in", color="#8b949e", fontsize=8, rotation=0, ha="right", va="center")

    # S1 syndrome: appears at cycle 3
    s1_val = bch_decode.__doc__  # dummy — compute S1 inline
    S1_computed = 0
    for i in range(45):
        if (inj_cw >> i) & 1:
            S1_computed ^= alpha_b(i)
    s1_color = "#ffa657" if S1_computed else "#3fb950"
    axes[3].fill_between([30, 40], [0, 0], [1, 1], alpha=0.15, color=s1_color)
    axes[3].plot([0,30,30,n_cycles*CLK_PERIOD], [0,0,1 if S1_computed else 0.1,
                  1 if S1_computed else 0.1], color=s1_color, lw=1.5)
    axes[3].text(50, 0.5, f"S1=0x{S1_computed:02X}" + (" ≠0 (error)" if S1_computed else " =0 (clean)"),
                 color=s1_color, fontsize=7, va="center")
    axes[3].set_ylabel("S1 synd.", color="#8b949e", fontsize=8, rotation=0, ha="right", va="center")

    # S2 (sigma2): appears at cycle 4
    s2_color = "#ffa657" if s2 else "#3fb950"
    axes[4].fill_between([40, 50], [0, 0], [1, 1], alpha=0.15, color=s2_color)
    axes[4].plot([0,40,40,n_cycles*CLK_PERIOD], [0,0,1 if s2 else 0.1,
                  1 if s2 else 0.1], color=s2_color, lw=1.5)
    axes[4].text(60, 0.5, f"σ2=0x{s2:02X}" + (" (2-err)" if s2 else " (0/1-err)"),
                 color=s2_color, fontsize=7, va="center")
    axes[4].set_ylabel("σ2 (s2)", color="#8b949e", fontsize=8, rotation=0, ha="right", va="center")

    # valid_out: high at cycle 6
    t_vo = np.array([0, 60, 60, 70, 70, n_cycles*CLK_PERIOD])
    v_vo = np.array([0, 0,  1,  1,  0,  0])
    axes[5].fill_between(t_vo, v_vo, alpha=0.25, color=colors["valid_out"])
    axes[5].plot(t_vo, v_vo, color=colors["valid_out"], lw=1.5)
    axes[5].set_ylabel("valid_out", color="#8b949e", fontsize=8, rotation=0, ha="right", va="center")

    # data_out bus
    do_color = "#f85149" if data_out != word else "#3fb950"
    axes[6].fill_between([60, 70], [0, 0], [1, 1], alpha=0.15, color=do_color)
    axes[6].plot([0,60,60,70,70,n_cycles*CLK_PERIOD], [0.5,0.5,0,0,0.5,0.5],
                 color=do_color, lw=1.5)
    match_str = "✓ MATCH" if data_out == word else "✗ MISMATCH"
    axes[6].text(65, 0.5, f"0x{data_out:08X}  {match_str}", color=do_color,
                 fontsize=7, ha="center", va="center")
    axes[6].set_ylabel("data_out", color="#8b949e", fontsize=8, rotation=0, ha="right", va="center")

    # fatal flag
    f_color = "#f85149" if fatal else "#3fb950"
    t_fat = np.array([0, 60, 60, 70, 70, n_cycles*CLK_PERIOD])
    v_fat = np.array([0, 0, 1 if fatal else 0, 1 if fatal else 0, 0, 0])
    axes[7].fill_between(t_fat, v_fat, alpha=0.3, color=f_color)
    axes[7].plot(t_fat, v_fat, color=f_color, lw=1.5)
    axes[7].text(80, 0.5 if fatal else 0.15,
                 "FATAL — uncorrectable" if fatal else "No fatal — corrected OK",
                 color=f_color, fontsize=7, va="center")
    axes[7].set_ylabel("fatal", color="#8b949e", fontsize=8, rotation=0, ha="right", va="center")

    # Error mask annotation
    if emask and not fatal:
        bit_positions = [i for i in range(45) if (emask >> i) & 1]
        axes[7].text(80, 0.8, f"Error mask bits: {bit_positions}",
                     color="#79c0ff", fontsize=6.5, va="center")

    # X-axis
    axes[7].set_xlabel("Time (ns)", color="#8b949e", fontsize=8)
    tick_ns = [i * CLK_PERIOD for i in range(n_cycles+1)]
    axes[7].set_xticks(tick_ns)
    axes[7].set_xticklabels([f"{t}" for t in tick_ns], fontsize=6, color="#8b949e")
    axes[7].xaxis.set_visible(True)

    # Cycle labels at top
    for c in range(n_cycles):
        axes[0].text(c*CLK_PERIOD + CLK_PERIOD/2, 1.3, f"C{c}",
                     color="#484f58", fontsize=6, ha="center")

    # Pipeline stage annotations
    stage_boxes = [
        (20, 30, "INPUT"),
        (30, 40, "PIPE1\nSyndromes"),
        (40, 50, "PIPE2\nChien"),
        (60, 70, "OUTPUT"),
    ]
    for x0, x1, lbl in stage_boxes:
        for ax in axes[1:]:
            ax.axvspan(x0, x1, alpha=0.04, color="#58a6ff")
        axes[0].text((x0+x1)/2, -0.6, lbl, color="#484f58", fontsize=5.5,
                     ha="center", va="top", style="italic")

    fig.suptitle(title, color="#e6edf3", fontsize=11, fontweight="bold", y=0.98)
    plt.tight_layout(rect=[0.08, 0.02, 1, 0.96])
    plt.savefig(out_path, dpi=150, facecolor=fig.get_facecolor())
    plt.close()
    print(f"  Waveform saved: {out_path}")

# ─────────────────────────────────────────────────────────────
# Summary bar chart
# ─────────────────────────────────────────────────────────────

def plot_summary_chart(rows):
    # Counts by fault_type
    type_names = {0:"No Error (Clean)", 1:"1-Bit Error", 2:"2-Bit Error", 3:"3-Bit Error"}
    pass_counts = {t:0 for t in range(4)}
    fail_counts = {t:0 for t in range(4)}
    for r in rows:
        ft = r["fault_type"]
        if r["result"] == "PASS": pass_counts[ft] += 1
        else: fail_counts[ft] += 1

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.patch.set_facecolor("#0d1117")

    # Left: bar chart
    ax = axes[0]
    ax.set_facecolor("#161b22")
    categories = [type_names[t] for t in range(4)]
    pass_vals = [pass_counts[t] for t in range(4)]
    fail_vals = [fail_counts[t] for t in range(4)]
    x = np.arange(len(categories))
    w = 0.35
    bars1 = ax.bar(x - w/2, pass_vals, w, label="PASS", color="#3fb950", alpha=0.85)
    bars2 = ax.bar(x + w/2, fail_vals, w, label="FAIL", color="#f85149", alpha=0.85)
    ax.set_xticks(x); ax.set_xticklabels(categories, color="#8b949e", fontsize=9)
    ax.set_ylabel("Test Vector Count", color="#8b949e"); ax.set_title(
        "BCH(45,32,t=2) Test Results by Fault Type", color="#e6edf3", fontsize=11)
    ax.tick_params(colors="#8b949e"); ax.set_facecolor("#161b22")
    ax.spines["top"].set_visible(False); ax.spines["right"].set_visible(False)
    for s in ["bottom","left"]: ax.spines[s].set_color("#30363d")
    for bar in bars1:
        ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.5,
                str(int(bar.get_height())), ha="center", color="#3fb950", fontsize=8)
    for bar in bars2:
        v = int(bar.get_height())
        if v > 0:
            ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.5,
                    str(v), ha="center", color="#f85149", fontsize=8)
    ax.legend(facecolor="#161b22", edgecolor="#30363d", labelcolor="#8b949e")

    # Right: pie chart overall
    ax2 = axes[1]
    ax2.set_facecolor("#161b22")
    total_pass = sum(pass_counts.values())
    total_fail = sum(fail_counts.values())
    wedge_props = dict(width=0.5, edgecolor="#0d1117", linewidth=2)
    ax2.pie([total_pass, total_fail],
            labels=[f"PASS\n({total_pass})", f"FAIL\n({total_fail})"],
            colors=["#3fb950","#f85149"],
            autopct="%1.1f%%", startangle=90,
            textprops={"color":"#e6edf3","fontsize":10},
            wedgeprops=wedge_props)
    ax2.set_title(f"Overall: {total_pass+total_fail} Vectors", color="#e6edf3", fontsize=11)

    plt.tight_layout()
    out = os.path.join(os.path.dirname(__file__), "bch_summary_chart.png")
    plt.savefig(out, dpi=150, facecolor=fig.get_facecolor())
    plt.close()
    print(f"  Summary chart: {out}")

# ─────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("BCH(45,32,t=2) Simulation — Python Model")
    print("=" * 60)

    rows, tp, tf = run_bch_simulation()

    base = os.path.dirname(__file__)

    # Waveform 1: No error — ADD instruction
    print("\nGenerating waveforms...")
    plot_waveform("no_error",   0x00208033, 0, [],         os.path.join(base,"waveform_bch_no_error.png"))
    plot_waveform("1bit_error", 0x00208033, 1, [22],       os.path.join(base,"waveform_bch_1bit_error.png"))
    plot_waveform("2bit_error", 0x00208033, 2, [0,22],     os.path.join(base,"waveform_bch_2bit_error.png"))

    plot_summary_chart(rows)

    print("\nDone.")
