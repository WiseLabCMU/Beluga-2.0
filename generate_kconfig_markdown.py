import tempfile
import sys
from kconfiglib import Kconfig, Symbol, MenuNode, Choice
import os
import shutil
from dataclasses import dataclass
from pathlib import Path


@dataclass
class KconfigAttr:
    name: str = ""
    brief: str = ""
    help: str = ""
    type: str = ""
    depends: str = ""
    implies: str = ""
    selects: str = ""
    path: str = ""


def copy_kconfig(file_path: str, writer):
    with open (file_path, 'r') as f:
        for line in f:
            if line.strip() != 'source "Kconfig.zephyr"':
                writer.write(line)


def set_srctree():
    if "ZEPHYR_BASE" not in os.environ:
        raise RuntimeError("ZEPHYR_BASE must be set")
    os.environ["srctree"] = os.environ["ZEPHYR_BASE"]


def generate_markdown(symbols: dict[str, list[KconfigAttr]]):
    pass


def get_md_filename(kconfig_file: str):
    path = Path(kconfig_file)
    name = path.suffix
    if not name:
        return "index.md"
    return f"{name[1:]}.md"


def parse_simple_symbol(node: MenuNode, path: str, prefix: str) -> KconfigAttr:
    symbol: Symbol = node.item
    sym = KconfigAttr()
    sym.name = f"{prefix}{symbol.name}"
    sym.brief = node.prompt
    sym.help = node.help
    sym.type = symbol.type
    sym.depends = node.dep
    sym.implies = node.implies
    sym.selects = node.selects
    sym.path = path
    return sym


def parse_kconfig(file):
    set_srctree()
    kconf = Kconfig(file, warn=False)
    symbols: dict[str, list[KconfigAttr]] = {}

    def parse_symbol(node: Symbol | MenuNode, path: str = "(Top)", top = False):
        while node:
            if isinstance(node.item, Symbol) or isinstance(node.item, Choice):
                doc_file = get_md_filename(node.filename)
                if doc_file not in symbols:
                    symbols[doc_file] = []

                if isinstance(node.item, Symbol):
                    symbols[doc_file].append(parse_simple_symbol(node, path, kconf.config_prefix))
                else:
                    print(f"Found choice symbol: {node.item.name}")


            if node.list:
                new_path = path
                if not top:
                    new_path = f"{path} > {node.prompt[0]}"
                parse_symbol(node.list, new_path)

            node = node.next

    parse_symbol(kconf.top_node, top=True)


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
