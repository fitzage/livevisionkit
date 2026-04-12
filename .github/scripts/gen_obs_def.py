"""
Generate a MSVC import-library DEF file from obs.dll.

Strategy:
  1. Try dumpbin /EXPORTS (needs MSVC in PATH).  Print full output for
     diagnostics regardless of outcome.
  2. If dumpbin yields 0 names, fall back to reading the PE export table
     directly with Python's struct module (no external tools needed).

Usage:
    python gen_obs_def.py <path/to/obs.dll> <path/to/obs.def>
"""
import re
import struct
import subprocess
import sys


# ---------------------------------------------------------------------------
# Method 1 – dumpbin
# ---------------------------------------------------------------------------

def exports_from_dumpbin(dll_path):
    print("--- trying dumpbin /EXPORTS ---", flush=True)
    try:
        proc = subprocess.run(
            ["dumpbin", "/EXPORTS", dll_path],
            capture_output=True, text=True, errors="replace", timeout=120,
        )
    except FileNotFoundError:
        print("dumpbin not found in PATH", flush=True)
        return None
    except subprocess.TimeoutExpired:
        print("dumpbin timed out", flush=True)
        return None

    print(f"dumpbin exit code: {proc.returncode}", flush=True)
    # Always print output so CI logs show what happened
    print(proc.stdout[:6000] or "(empty stdout)", flush=True)
    if proc.stderr:
        print("stderr:", proc.stderr[:1000], flush=True)

    names = []
    for line in proc.stdout.splitlines():
        # Export entry lines look like (columns: ordinal hint RVA name):
        #   "          1    0 000037C0 obs_add_data_path"
        # The RVA is exactly 8 hex chars – that distinguishes data lines from
        # the column-header line "ordinal hint RVA name".
        m = re.match(
            r"^\s+\d+\s+[0-9A-Fa-f]+\s+[0-9A-Fa-f]{8}\s+(\w+)\s*$", line
        )
        if m:
            names.append(m.group(1))

    print(f"dumpbin method: extracted {len(names)} names", flush=True)
    return names or None


# ---------------------------------------------------------------------------
# Method 2 – direct PE export-table parse (no external tool)
# ---------------------------------------------------------------------------

def exports_from_pe(dll_path):
    print("--- trying direct PE parse ---", flush=True)
    with open(dll_path, "rb") as fh:
        data = fh.read()

    if data[:2] != b"MZ":
        raise ValueError("Not an MZ executable")
    pe_off = struct.unpack_from("<I", data, 0x3C)[0]
    if data[pe_off : pe_off + 4] != b"PE\0\0":
        raise ValueError("PE signature not found")

    num_sections = struct.unpack_from("<H", data, pe_off + 6)[0]
    opt_size     = struct.unpack_from("<H", data, pe_off + 20)[0]
    opt_magic    = struct.unpack_from("<H", data, pe_off + 24)[0]
    is_pe32plus  = opt_magic == 0x20B  # 64-bit PE

    # Data directory 0 = export directory
    dd_off  = pe_off + 24 + (112 if is_pe32plus else 96)
    exp_rva = struct.unpack_from("<I", data, dd_off)[0]
    if exp_rva == 0:
        print("PE has no export directory", flush=True)
        return []

    # Build section table for RVA → file-offset translation
    sec_off  = pe_off + 24 + opt_size
    sections = []
    for i in range(num_sections):
        s    = sec_off + i * 40
        va   = struct.unpack_from("<I", data, s + 12)[0]
        vsz  = struct.unpack_from("<I", data, s + 16)[0]
        raw  = struct.unpack_from("<I", data, s + 20)[0]
        rsz  = struct.unpack_from("<I", data, s + 16)[0]
        sections.append((va, max(vsz, rsz), raw))

    def rva2off(rva):
        for va, sz, raw in sections:
            if va <= rva < va + sz:
                return raw + (rva - va)
        raise ValueError(f"RVA 0x{rva:08X} not found in any section")

    # Export directory structure
    e         = rva2off(exp_rva)
    num_names = struct.unpack_from("<I", data, e + 24)[0]
    names_rva = struct.unpack_from("<I", data, e + 32)[0]

    names = []
    noff  = rva2off(names_rva)
    for i in range(num_names):
        nr   = struct.unpack_from("<I", data, noff + i * 4)[0]
        nptr = rva2off(nr)
        end  = data.index(b"\x00", nptr)
        names.append(data[nptr:end].decode("ascii", errors="replace"))

    print(f"PE parse method: extracted {len(names)} names", flush=True)
    return names


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) != 3:
        print("Usage: gen_obs_def.py <obs.dll> <obs.def>", flush=True)
        sys.exit(1)

    dll_path = sys.argv[1]
    def_path = sys.argv[2]
    print(f"DLL : {dll_path}", flush=True)
    print(f"DEF : {def_path}", flush=True)

    names = exports_from_dumpbin(dll_path)
    if not names:
        names = exports_from_pe(dll_path)

    if not names:
        print("ERROR: no exports found by either method", flush=True)
        sys.exit(1)

    with open(def_path, "w", newline="\n") as fh:
        fh.write("LIBRARY obs\nEXPORTS\n")
        for n in names:
            fh.write(n + "\n")

    print(f"Wrote {len(names)} exports to {def_path}", flush=True)


if __name__ == "__main__":
    main()
