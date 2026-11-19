#!/usr/bin/env python3
import sys
import os
import shutil
import struct

FW_PATH = "build/common/fw/target-0.2.0.951.mvcmd"
ORIG_PATH = "build/common/fw/target-0.2.0.951.mvcmd.orig"

def encode_cmp_simm13(reg, imm):
    """Encodes SPARC V8: cmp %reg, imm (subcc %reg, imm, %g0) as 4 bytes in file byte order."""
    val = (2 << 30) | (0 << 25) | (0x14 << 19) | (reg << 14) | (1 << 13) | (imm & 0x1fff)
    return val.to_bytes(4, "little")

def encode_mov_simm13(reg, imm):
    """Encodes SPARC V8: mov imm, %reg (or %g0, imm, %reg) as 4 bytes in file byte order."""
    val = (2 << 30) | (reg << 25) | (2 << 19) | (0 << 14) | (1 << 13) | (imm & 0x1fff)
    return val.to_bytes(4, "little")

def encode_branch(pc, target, cond, annul=False):
    """Encodes SPARC V8 branch instruction as 4 bytes in file byte order."""
    disp22 = ((target - pc) >> 2) & 0x3fffff
    a = 1 if annul else 0
    val = (0 << 30) | (a << 29) | (cond << 25) | (2 << 22) | disp22
    return val.to_bytes(4, "little")

def make_stream_entry(sensor_id, width, height, fmt, pitch, fps):
    # struct supported_raw_stream_libtm_message (12 bytes)
    # uint8_t bSensorID
    # uint8_t bOutputMode
    # uint16_t wWidth
    # uint16_t wHeight
    # uint8_t bPixelFormat
    # uint8_t reserved
    # uint16_t wPitch
    # uint16_t wFramesPerSecond
    return struct.pack("<BBHHBBHH", sensor_id, 0, width, height, fmt, 0, pitch, fps)

def apply_multi_fps_patch():
    if not os.path.exists(ORIG_PATH):
        print(f"Creating backup of original firmware to {ORIG_PATH}...")
        shutil.copyfile(FW_PATH, ORIG_PATH)

    with open(ORIG_PATH, "rb") as f:
        data = bytearray(f.read())

    print("[PATCH] Applying Multi-FPS (30 / 60 / 90 FPS) Patch to T265 firmware...")

    # 1. Patch video_thread FPS assertion check (0x4827c)
    # Original: bne 0x4857c [c0 00 80 12 in file]
    # Replace with: nop [00 00 00 01 in file]
    offset_vt_bne = 0x4827c
    orig_vt_bne = bytes(data[offset_vt_bne:offset_vt_bne+4])
    if orig_vt_bne == bytes.fromhex("c0008012"):
        nop_bytes = bytes.fromhex("00000001") # SPARC nop in little-endian (0x01000000 BE)
        data[offset_vt_bne:offset_vt_bne+4] = nop_bytes
        print(f"[OK] Patched video_thread check at {hex(offset_vt_bne)} -> nop ({nop_bytes.hex()})")
    else:
        print(f"[WARN] Unexpected bytes at {hex(offset_vt_bne)}: {orig_vt_bne.hex()}")

    # 2. Patch camera_set_fps dispatch block (0x85ce0 to 0x85cfb)
    #   0x85ce0: cmp %g1, 0x1e      ; is it 30 fps?
    #   0x85ce4: be  0x8600c        ; -> jumps to 0x8600c (sets VTS=2873 [0xb39])
    #   0x85ce8: cmp %g1, 0x5a      ; [delay slot] is it 90 fps?
    #   0x85cec: be,a 0x85db0       ; if 90 fps, branch annulled to 0x85db0
    #   0x85cf0: mov 0x3be, %g1     ; [delay slot, ONLY if 90] %g1 = 958 (0x3be)
    #   0x85cf4: ba  0x85db0        ; else (60 fps), jump to 0x85db0
    #   0x85cf8: mov 0x59c, %g1     ; [delay slot, always] %g1 = 1436 (0x59c)
    offset_dispatch = 0x85ce0
    orig_dispatch = bytes(data[offset_dispatch:offset_dispatch+28])
    expected_orig = bytes.fromhex("1e60a080ca0080023c60a08030008002ff3f10b008e0c7810000e881")
    if orig_dispatch == expected_orig:
        new_dispatch = bytes.fromhex("1e60a080ca0080025a60a08031008022be2310822f0080109c251082")
        data[offset_dispatch:offset_dispatch+28] = new_dispatch
        print(f"[OK] Patched camera_set_fps dispatch at {hex(offset_dispatch)} to 3-way FPS dispatch!")
    else:
        print(f"[WARN] Unexpected dispatch bytes at {hex(offset_dispatch)}: {orig_dispatch.hex()}")

    # 3. Patch Supported Streams Descriptor Table at 0x10434
    # Stock layout:
    #   [ 0] 0x10434: FE0 @ 30
    #   [ 1] 0x10440: FE1 @ 30
    #   [ 2] 0x1044c: Gyro @ 200
    #   [ 3] 0x10458: Accel @ 62
    #   [ 4] 0x10464: Gyro @ 100    -> REPLACE with FE0 @ 60
    #   [ 5] 0x10470: Accel @ 125   -> REPLACE with FE1 @ 60
    #   [ 6] 0x1047c: Gyro @ 400    -> REPLACE with FE0 @ 90
    #   [ 7] 0x10488: Accel @ 250   -> REPLACE with FE1 @ 90
    # Entries 8..14 remain untouched.
    table_base = 0x10434
    fe0_60 = make_stream_entry(0x03, 848, 800, 9, 848, 60)
    fe1_60 = make_stream_entry(0x23, 848, 800, 9, 848, 60)
    fe0_90 = make_stream_entry(0x03, 848, 800, 9, 848, 90)
    fe1_90 = make_stream_entry(0x23, 848, 800, 9, 848, 90)

    data[table_base + 4*12 : table_base + 5*12] = fe0_60
    data[table_base + 5*12 : table_base + 6*12] = fe1_60
    data[table_base + 6*12 : table_base + 7*12] = fe0_90
    data[table_base + 7*12 : table_base + 8*12] = fe1_90
    print(f"[OK] Added 60 FPS and 90 FPS Fisheye stream descriptors to table at {hex(table_base)}")

    # Write patched firmware
    with open(FW_PATH, "wb") as f:
        f.write(data)

    print(f"\n[SUCCESS] Successfully applied Multi-FPS patch to {FW_PATH}!")

def main():
    if len(sys.argv) > 1 and (sys.argv[1] == "restore" or sys.argv[1] == "revert"):
        if os.path.exists(ORIG_PATH):
            shutil.copyfile(ORIG_PATH, FW_PATH)
            print(f"Restored original firmware from {ORIG_PATH}")
            return
        else:
            print(f"Error: {ORIG_PATH} does not exist.")
            sys.exit(1)

    apply_multi_fps_patch()

if __name__ == "__main__":
    main()
