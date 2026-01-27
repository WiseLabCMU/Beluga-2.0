import tempfile
import sys
from kconfiglib import Kconfig
import os
import shutil


def copy_kconfig(file_path: str, writer):
    with open (file_path, 'r') as f:
        for line in f:
            if line.strip() != 'source "Kconfig.zephyr"':
                writer.write(line)


def set_srctree():
    if "ZEPHYR_BASE" not in os.environ:
        raise RuntimeError("ZEPHYR_BASE must be set")
    os.environ["srctree"] = os.environ["ZEPHYR_BASE"]


def parse_kconfig(file):
    set_srctree()
    kconfig = Kconfig(file, warn=False)
    print(kconfig)


def main():
    with tempfile.TemporaryDirectory() as tmpdir:
        kconfig_file = f"{tmpdir}/Kconfig"
        with open(kconfig_file, "w") as f:
            copy_kconfig(f"{sys.argv[1]}/Kconfig", f)
        shutil.copytree(f"{sys.argv[1]}/config", f"{tmpdir}/config")

        parse_kconfig(kconfig_file)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"python3 {sys.argv[0]} <kconfig folder>")
    else:
        main()
