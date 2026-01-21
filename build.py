import os
import platform
import shutil
import subprocess
import sys


def install_pyinstaller():
    try:
        import PyInstaller  # noqa: F401
    except ImportError:
        print("[INFO] Installing PyInstaller...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pyinstaller"])


def build_executable():
    system = platform.system()
    arch = platform.machine()
    print(f"[INFO] Detected platform: {system} ({arch})")

    main_script = "main.py"
    requirements_file = "requirements.txt"

    if not os.path.exists(main_script):
        print("[ERROR] main.py not found.")
        sys.exit(1)

    # Step 1: Parse requirements.txt to collect extra options if needed
    hidden_imports = []
    if os.path.exists(requirements_file):
        with open(requirements_file, "r") as f:
            for line in f:
                pkg = line.strip()
                if pkg and not pkg.startswith("#"):
                    pkg_name = pkg.split("==")[0].split(">")[0].split("<")[0]
                    hidden_imports.append(f"--collect-all={pkg_name}")

    # Step 2: Clean old build
    for path in ["dist", "build", "main.spec"]:
        if os.path.exists(path):
            if os.path.isfile(path):
                os.remove(path)
            else:
                shutil.rmtree(path)

    # Step 3: Assemble command
    cmd = [
        "pyinstaller",
        "--noconfirm",
        "--onefile",
        "--clean",
        main_script,
        *hidden_imports
    ]

    print("[INFO] Running PyInstaller...")
    subprocess.check_call(cmd)

    output_file = os.path.join("dist", "main.exe" if system == "Windows" else "main")
    print(f"[SUCCESS] Executable generated: {output_file}")


if __name__ == "__main__":
    install_pyinstaller()
    build_executable()
