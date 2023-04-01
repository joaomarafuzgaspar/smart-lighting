import os
import subprocess
import glob
import shutil

# Set the path to your .ino file
ino_file = "main/main.ino"

def compile_program(ino_file):
    build_dir = "build"

    if not os.path.exists(build_dir):
        os.mkdir(build_dir)

    # Compile the program using the Arduino CLI
    compile_cmd = ["arduino-cli", "compile", "--fqbn", "rp2040:rp2040:rpipico", ino_file, "--output-dir", build_dir]
    subprocess.run(compile_cmd, check=True)

    return f"{build_dir}/sketch.ino.uf2"

def find_pico_devices():
    devices = glob.glob("/dev/tty.usbmodem*")
    return devices

def upload_to_pico(device):
    upload_cmd = ["arduino-cli", "upload", "-p", device, "--fqbn", "rp2040:rp2040:rpipico", ino_file]
    subprocess.run(upload_cmd, check=True)

def main():
    print("Compiling the programm...")
    compiled_program = compile_program(ino_file)
    pico_devices = find_pico_devices()

    if not pico_devices:
        print("No Raspberry Pi Pico devices found.")
        return

    print(f"Uploading {compiled_program} to the following devices:")
    for device in pico_devices:
        print(f"  * {device}")

    for device in pico_devices:
        upload_to_pico(device)
        
    # Remove the build directory after uploading
    shutil.rmtree("build")

if __name__ == "__main__":
    main()