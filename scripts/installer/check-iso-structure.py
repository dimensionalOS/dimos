#!/usr/bin/env python3
# Check MBR and GPT structure of an ISO file
# Useful for determining whether the ISO is indeed bootable.
# Also useful for determining whether the ISO can indeed be
# booted from both BIOS and UEFI.

import struct
import sys


def bytes_to_guid(guid_bytes):
    """Convert GUID from mixed-endian format (GUIDs use mixed endianness)"""
    # GUID format: XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX
    # First 4 bytes: little-endian
    # Next 2 bytes: little-endian
    # Next 2 bytes: little-endian
    # Next 2 bytes: big-endian (network byte order)
    # Last 6 bytes: big-endian (network byte order)
    return f"{int.from_bytes(guid_bytes[0:4], 'little'):08X}-" \
           f"{int.from_bytes(guid_bytes[4:6], 'little'):04X}-" \
           f"{int.from_bytes(guid_bytes[6:8], 'little'):04X}-" \
           f"{int.from_bytes(guid_bytes[8:10], 'big'):04X}-" \
           f"{guid_bytes[10:16].hex().upper()}"


def check_iso_structure(iso_path):
    """Check MBR and GPT structure of an ISO file"""
    try:
        with open(iso_path, 'rb') as f:
            # Read MBR (first 512 bytes)
            mbr = f.read(512)

            # Check boot signature (bytes 510-511 should be 0x55 0xAA)
            boot_sig = mbr[510:512]
            valid_sig = b'\x55\xaa'
            is_valid = 'Valid' if boot_sig == valid_sig else 'Invalid'
            print(f"MBR Boot Signature: {boot_sig.hex()} ({is_valid})")

            # Check partition entries (4 entries starting at offset 446, 16 bytes each)
            print("\nMBR Partition Entries:")
            has_partition = False
            for i in range(4):
                offset = 446 + (i * 16)
                entry = mbr[offset:offset+16]
                if entry != b'\x00' * 16:
                    has_partition = True
                    status = entry[0]
                    part_type = entry[4]
                    lba_start = struct.unpack('<I', entry[8:12])[0]
                    lba_count = struct.unpack('<I', entry[12:16])[0]

                    if part_type == 0xee:
                        type_desc = "GPT Protective (0xEE) - Proper GPT with protective MBR"
                    elif part_type == 0xef:
                        type_desc = "EFI System (0xEF) - MBR with dummy GPT"
                    else:
                        type_desc = f"Other (0x{part_type:02x})"

                    print(f"  Partition {i+1}:")
                    print(f"    Status: 0x{status:02x}")
                    print(f"    Type: 0x{part_type:02x} ({type_desc})")
                    print(f"    LBA Start: {lba_start}")
                    print(f"    LBA Count: {lba_count}")
                    if lba_count > 0:
                        print(f"    Size: {lba_count * 512 / (1024*1024):.2f} MB")

            if not has_partition:
                print("  No MBR partitions found")

            # Read GPT header (at LBA 1, offset 512)
            f.seek(512)
            gpt_header = f.read(92)

            print("\nGPT Header (LBA 1):")
            signature = gpt_header[0:8]
            print(f"  Signature: {signature}")
            if signature == b'EFI PART':
                print("  ✓ Valid GPT header found")
                revision = struct.unpack('<I', gpt_header[8:12])[0]
                header_size = struct.unpack('<I', gpt_header[12:16])[0]
                current_lba = struct.unpack('<Q', gpt_header[24:32])[0]
                backup_lba = struct.unpack('<Q', gpt_header[32:40])[0]
                first_usable = struct.unpack('<Q', gpt_header[40:48])[0]
                last_usable = struct.unpack('<Q', gpt_header[48:56])[0]
                num_partitions = struct.unpack('<I', gpt_header[80:84])[0]

                print(f"  Revision: {revision}")
                print(f"  Header Size: {header_size} bytes")
                print(f"  Current LBA: {current_lba}")
                print(f"  Backup LBA: {backup_lba}")
                print(f"  First Usable LBA: {first_usable}")
                print(f"  Last Usable LBA: {last_usable}")
                print(f"  Number of Partitions: {num_partitions}")

                # Read partition entries (typically at LBA 2)
                partition_entry_lba = struct.unpack('<Q', gpt_header[72:80])[0]
                partition_entry_size = struct.unpack('<I', gpt_header[84:88])[0]

                print(f"\nPartition Entries (at LBA {partition_entry_lba}, size {partition_entry_size} bytes each):")

                # Known partition type GUIDs
                EFI_SYSTEM_PARTITION_GUID = "C12A7328-F81F-11D2-BA4B-00A0C93EC93B"
                BIOS_BOOT_PARTITION_GUID = "21686148-6449-6E6F-744E-656564454649"
                MICROSOFT_BASIC_DATA_GUID = "EBD0A0A2-B9E5-4433-87C0-68B6B72699C7"

                f.seek(partition_entry_lba * 512)
                for i in range(num_partitions):
                    entry = f.read(partition_entry_size)
                    if entry[:16] == b'\x00' * 16:
                        continue  # Empty partition entry

                    # Parse partition entry (first 128 bytes are standard)
                    part_type_guid_bytes = entry[0:16]
                    part_guid_bytes = entry[16:32]
                    first_lba = struct.unpack('<Q', entry[32:40])[0]
                    last_lba = struct.unpack('<Q', entry[40:48])[0]
                    flags = struct.unpack('<Q', entry[48:56])[0]
                    name_bytes = entry[56:128]

                    part_type_guid = bytes_to_guid(part_type_guid_bytes)
                    part_guid = bytes_to_guid(part_guid_bytes)

                    # Decode name (UTF-16LE)
                    try:
                        name = name_bytes.decode('utf-16le').rstrip('\x00')
                    except:
                        name = "<invalid encoding>"

                    size_mb = ((last_lba - first_lba + 1) * 512) / (1024 * 1024)

                    # Identify partition type (case-insensitive comparison)
                    part_type_name = "Unknown"
                    part_type_guid_upper = part_type_guid.upper()
                    if part_type_guid_upper == EFI_SYSTEM_PARTITION_GUID.upper():
                        part_type_name = "EFI System Partition (ESP) ✓"
                    elif part_type_guid_upper == BIOS_BOOT_PARTITION_GUID.upper():
                        part_type_name = "BIOS Boot Partition (GRUB BBP) ✓"
                    elif part_type_guid_upper == MICROSOFT_BASIC_DATA_GUID.upper():
                        part_type_name = "Microsoft Basic Data"

                    print(f"\n  Partition {i+1}:")
                    print(f"    Name: {name}")
                    print(f"    Type GUID: {part_type_guid}")
                    print(f"      → {part_type_name}")
                    print(f"    Partition GUID: {part_guid}")
                    print(f"    First LBA: {first_lba}")
                    print(f"    Last LBA: {last_lba}")
                    print(f"    Size: {size_mb:.2f} MB")
                    print(f"    Flags: 0x{flags:016x}")
                    if flags & 1:
                        print(f"      → Required Partition")
                    if flags & 2:
                        print(f"      → No Block IO Protocol")
                    if flags & 4:
                        print(f"      → Legacy BIOS Bootable")

                # Determine structure type
                print("\nStructure Analysis:")
                mbr_type = None
                for i in range(4):
                    offset = 446 + (i * 16)
                    entry = mbr[offset:offset+16]
                    if entry != b'\x00' * 16:
                        part_type = entry[4]
                        if part_type == 0xee:
                            mbr_type = "GPT Protective (0xEE)"
                        elif part_type == 0xef:
                            mbr_type = "EFI System (0xEF)"
                        break

                if mbr_type == "GPT Protective (0xEE)":
                    print("  ✓ Proper GPT structure: GPT is primary, MBR is protective")
                elif mbr_type == "EFI System (0xEF)":
                    print("  ⚠ MBR-based structure: MBR is primary, GPT may be dummy")
                else:
                    print("  ? Unknown structure")
            else:
                print("  ✗ No valid GPT header found")
                print("  → This is likely a pure MBR or ISO9660-only structure")

    except FileNotFoundError:
        print(f"Error: File not found: {iso_path}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <iso_file>", file=sys.stderr)
        sys.exit(1)

    iso_file = sys.argv[1]

    import os
    if not os.path.isfile(iso_file):
        print(f"Error: File not found: {iso_file}", file=sys.stderr)
        sys.exit(1)

    check_iso_structure(iso_file)


if __name__ == "__main__":
    main()

