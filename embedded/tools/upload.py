"""
Description: This script compiles and uploads an Arduino program to a Raspberry Pi Pico microcontroller board using the Arduino CLI.
The script automates the process of deploying code to the Pico, making it easier for developers to test and run their programs on the board.
"""

import os
import subprocess
import glob
import shutil

# Set the path to your .ino file
ino_file = "main/main.ino"


def compile_program(ino_file):
    build_dir = "build"

    # Create the build directory if it does not exist
    if not os.path.exists(build_dir):
        os.mkdir(build_dir)

    # Copy only the src/*.cpp files to main directory temporarily (Arduino needs them in same dir for linking)
    copied_files = []
    for cpp_file in glob.glob("src/*.cpp"):
        filename = os.path.basename(cpp_file)
        dest_path = os.path.join("main", filename)
        if not os.path.exists(dest_path):  # Only copy if doesn't already exist
            shutil.copy(cpp_file, dest_path)
            copied_files.append(dest_path)

    # Get absolute paths for include directories
    current_dir = os.getcwd()
    include_dir = os.path.join(current_dir, "include")
    src_dir = os.path.join(current_dir, "src")

    try:
        # Compile the program using the Arduino CLI with build properties
        compile_cmd = [
            "arduino-cli",
            "compile",
            "--fqbn",
            "rp2040:rp2040:rpipico",
            "--build-property",
            f"compiler.cpp.extra_flags=-I{include_dir}",
            ino_file,
            "--output-dir",
            build_dir,
        ]
        subprocess.run(compile_cmd, check=True)
    finally:
        # Clean up: remove the copied files
        for file_path in copied_files:
            if os.path.exists(file_path):
                os.remove(file_path)

    # Return the path to the compiled program
    return f"{build_dir}/main.ino.uf2"


def find_pico_devices():
    # Find all connected Raspberry Pi Pico devices
    devices = glob.glob("/dev/tty.usbmodem*")
    return devices


def delete_dir(dir):
    if os.path.exists(dir):
        shutil.rmtree(dir)


def upload_to_pico(device):
    # Upload the compiled program to the specified device using the Arduino CLI
    upload_cmd = [
        "arduino-cli",
        "upload",
        "-p",
        device,
        "--fqbn",
        "rp2040:rp2040:rpipico",
        ino_file,
    ]
    subprocess.run(upload_cmd, check=True)


def main():
    print("Compiling the programm...")

    # Compile the program
    compiled_program = compile_program(ino_file)

    # Find all connected Pico devices
    pico_devices = find_pico_devices()

    if not pico_devices:
        print("No Raspberry Pi Pico devices found.")
        delete_dir("build")
        return

    # Print a message indicating which devices the program will be uploaded to
    print(f"Uploading {compiled_program} to the following devices:")
    for device in pico_devices:
        print(f"  * {device}")

    # Upload the program to each Pico device
    for device in pico_devices:
        upload_to_pico(device)

    # Remove the build directory after uploading
    delete_dir("build")


if __name__ == "__main__":
    main()
