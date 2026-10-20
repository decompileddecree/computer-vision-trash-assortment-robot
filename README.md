# Computer Vision Trash Assortment Robot

Python and Arduino source for a robot that detects and sorts waste using a YOLO model, a camera, and a serial-controlled Arduino.

## Files

- `Robot_TA v3.0.py` — computer-vision controller and robot behavior.
- `Robot_Pemilah_Sampah_1.3V.ino` — Arduino firmware.
- `best3.pt` — trained YOLO model used by the Python controller.

## Project materials

- [`diagrams/`](diagrams/) — editable system block diagrams and flowcharts, with PNG previews.
- [`schematics/`](schematics/README.md) — Proteus schematic projects and a full-schematic preview.

Personal reports and presentations, photos, and bundled executables are not included.

## Requirements

- Python 3.10 or later
- A CUDA-enabled NVIDIA GPU and a compatible PyTorch installation
- USB camera and the robot's Arduino connected over serial
- Arduino libraries matching the sketch includes: `LiquidCrystal_I2C` and `PCA9685`

Install the Python packages:

```powershell
python -m pip install opencv-python numpy pyserial ultralytics
```

Install the PyTorch build that matches your CUDA setup by following the [official PyTorch installation selector](https://pytorch.org/get-started/locally/).

## Run

Upload `Robot_Pemilah_Sampah_1.3V.ino` to the robot's Arduino first. Then, from this directory, run:

```powershell
python "Robot_TA v3.0.py"
```

The controller uses `COM6` by default. Set `ROBOT_SERIAL_PORT` to the Arduino's serial port when it differs:

```powershell
$env:ROBOT_SERIAL_PORT = "COM3"
python "Robot_TA v3.0.py"
```

The model file is loaded from the same directory as the Python script. Connect the camera and robot hardware before running; the controller starts communicating with both during startup.

## Safety

This software directly controls motors and actuators. Test with the robot secured, keep clear of moving parts, and be ready to disconnect power.
