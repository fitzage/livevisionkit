"""
Generate a MSVC import-library DEF file from obs.dll by parsing dumpbin output.

Usage:
    python gen_obs_def.py <path/to/obs.dll> <path/to/obs.def>
"""
import subprocess
import re
import sys


def main():
    dll_path = sys.argv[1]
    def_path = sys.argv[2]

    proc = subprocess.run(
        ["dumpbin", "/EXPORTS", dll_path],
        capture_output=True, text=True, errors="replace",
    )

    names = []
    for line in proc.stdout.splitlines():
        # Export entry lines look like (columns: ordinal hint RVA name):
        #   "          1    0 000037C0 obs_add_data_path"
        # The RVA is always exactly 8 hex digits; that distinguishes export
        # entries from the header "ordinal hint RVA name" and summary lines.
        m = re.match(
            r"^\s+\d+\s+[0-9A-Fa-f]+\s+[0-9A-Fa-f]{8}\s+(\w+)\s*$", line
        )
        if m:
            names.append(m.group(1))

    print(f"Extracted {len(names)} exports", flush=True)
    if not names:
        print("ERROR: no exports found – dumpbin output follows:", flush=True)
        print(proc.stdout[:4000], flush=True)
        sys.exit(1)

    with open(def_path, "w", newline="\n") as f:
        f.write("LIBRARY obs\nEXPORTS\n")
        for n in names:
            f.write(n + "\n")

    print(f"Wrote {def_path}", flush=True)


if __name__ == "__main__":
    main()
