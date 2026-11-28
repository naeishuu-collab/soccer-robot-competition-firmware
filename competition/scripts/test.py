"""Compile the vendor ZIP excerpts as untouched .ino files, then run host and camera tests."""
import argparse
import ast
import hashlib
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[1]
ZIP_SHA256 = "a300ff984b8852cb96624f5d29088f6336f0885bd7125e69d2df06b065412d9e"
BASE = "44cd80b3fd2a5137721e3f97bdd7603d21474557"


def run(*args, cwd=ROOT):
    subprocess.run(args, cwd=cwd, check=True)


def build(compiler, temp, filename):
    output = temp / (filename.stem + (".exe" if sys.platform == "win32" else ""))
    includes = (ROOT / "tests", ROOT / "libraries" / "RobotCommon" / "src", temp)
    if Path(compiler).stem.lower() == "cl":
        run(compiler, "/nologo", "/std:c++17", "/EHsc", "/utf-8", "/W4",
            *(f"/I{i}" for i in includes), str(filename), f"/Fe:{output}", cwd=temp)
    else:
        run(compiler, "-std=c++17", "-Wall", "-Wextra",
            *(f"-I{i}" for i in includes), str(filename), "-o", str(output), cwd=temp)
    run(str(output))


def main():
    cli = argparse.ArgumentParser()
    cli.add_argument("--zip", required=True, type=Path)
    cli.add_argument("--mpy-cross", action="store_true", help="Compile for STM32H743 using the mpy-cross Python package.")
    options = cli.parse_args()
    compiler = next((path for name in ("cl", "g++", "clang++") if (path := shutil.which(name))), None)
    if compiler is None:
        cli.error("Run in Visual Studio Developer PowerShell or put g++/clang++ on PATH.")
    if hashlib.sha256(options.zip.read_bytes()).hexdigest() != ZIP_SHA256:
        cli.error("The ZIP does not match the audited attachment SHA-256.")
    with tempfile.TemporaryDirectory(prefix="rcj-test-") as work:
        temp = Path(work)
        with zipfile.ZipFile(options.zip) as reference:
            for board in ("ir", "motor"):
                name = board + "_chase"
                (temp / (name + ".ino")).write_bytes(reference.read(f"chase/{name}/{name}.ino"))
        legacy = subprocess.check_output(
            ["git", "show", f"{BASE}:soccer_camera/main/main.ino"], cwd=ROOT).decode("utf-8")
        extracted = re.search(r"^bool zipPathClear\(.*?^}", legacy, re.M | re.S)
        if extracted is None:
            raise RuntimeError("The baseline no longer supplies zipPathClear().")
        (temp / "old_path.h").write_text(
            "namespace brain { inline bool sectorOpen(uint16_t mask, int i) { return mask & (1u << i); } }\n"
            + extracted[0] + "\n", encoding="utf-8")
        prior = subprocess.check_output(
            ["git", "show", "3636020:competition/main/control.h"], cwd=ROOT).decode("utf-8")
        start = prior.index("inline Decision decide(")
        stop = prior.index("// Disabled in field builds", start)
        (temp / "previous_competition.h").write_text(
            prior[start:stop].replace("inline Decision decide(", "inline Decision decidePrevious(", 1), encoding="utf-8")
        golden = subprocess.check_output(
            [sys.executable, "-B", "-c", "from vision import packet; "
             "print(','.join(map(str, packet(19, True, True, 256, 256, (359, 24), "
             "((75, 65, 8), (65535, 0, 255)), 66))))"], cwd=ROOT / "openmv"
        ).decode("ascii").strip()
        golden_v2 = subprocess.check_output(
            [sys.executable, "-B", "-c", "from vision import packet; "
             "print(','.join(map(str, packet(20, True, False, 0, 0, None, "
             "((65535, 0, 255), (65535, 0, 255)), 40, blocked=1))))"], cwd=ROOT / "openmv"
        ).decode("ascii").strip()
        (temp / "camera_golden.h").write_text(
            "const uint8_t cameraGolden[] = {" + golden + "};\n"
            "const uint8_t cameraGoldenV2[] = {" + golden_v2 + "};\n", encoding="utf-8")
        for source in ("firmware.cpp", "sweep.cpp", "basic_chase.cpp", "managed_competition.cpp"):
            build(compiler, temp, ROOT / "tests" / source)
    for source in (ROOT / "openmv").glob("*.py"):
        ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
    if options.mpy_cross:
        run(sys.executable, "-m", "mpy_cross", "--version")
        with tempfile.TemporaryDirectory(prefix="rcj-mpy-") as directory:
            for source in sorted((ROOT / "openmv").glob("*.py")):
                run(sys.executable, "-m", "mpy_cross", "-march=armv7emsp", "-o",
                    str(Path(directory) / (source.stem + ".mpy")), str(source))
                print("MicroPython ARM compile OK:", source.name, flush=True)
    run(sys.executable, "-B", "-m", "unittest", "discover", "-s", "tests", "-v")


if __name__ == "__main__":
    main()
