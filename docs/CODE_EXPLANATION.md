# How the robot software works

This guide explains the two programs in this repository:

- [`Robot_TA v3.0.py`](../Robot_TA%20v3.0.py) runs on the computer. It reads the camera, recognizes and tracks waste, decides which actions to take, and sends commands to the Arduino.
- [`Robot_Pemilah_Sampah_1.3V.ino`](../Robot_Pemilah_Sampah_1.3V.ino) runs on the Arduino. It reads those commands and controls the drive motors, arm servos, pump, ultrasonic sensors, and LCD.

The computer makes the higher-level vision and sorting decisions. The Arduino performs the hardware actions. They communicate over a USB serial connection.

## System overview

```text
Camera image
    |
    v
Python + YOLO model: identify waste class and follow the object
    |
    +-- YOLO alignment and arm pickup
    |
    +-- OpenCV color tracking to find the matching bin
    |
    v
Two signed 16-bit integers over serial: command address + value
    |
    v
Arduino: wheels, arm servos, suction/blow pump, ultrasonic sensors, LCD
```

In the current program, YOLO recognizes the item category. Later, OpenCV looks for a color associated with that category to guide the robot toward its bin. The code controls the arm using calibrated positions; it does not calculate a general-purpose 3D grasp pose.

## The Python controller

### Startup and hardware setup

When run as a script, Python opens the serial connection at **9600 baud**, loads `best3.pt` from the script's own directory, and opens camera index `0` at **640 × 480**. The preview is enlarged to **1920 × 1080** for display.

The serial port defaults to `COM6`. On Windows, select another port before launch if needed:

```powershell
$env:ROBOT_SERIAL_PORT = "COM3"
python "Robot_TA v3.0.py"
```

The program calls `torch.cuda.set_device(0)`, so the current script expects a compatible CUDA-enabled PyTorch setup. It also opens the serial port and camera while the module is starting; running the program therefore requires the hardware and model file to be available.

The Python `setup()` routine moves the wrist and elbow toward their standby positions, opens the preview, briefly tests pump suction and blowing, turns the pump off, and sends command `9` to start the Arduino's ultrasonic-guided approach. After the Arduino signals that it has reached its distance target, the controller sets its stage to `1` and begins scanning.

### Sorting stages

The global `counter` selects the current part of the sorting cycle:

| `counter` | Function | Purpose |
|---:|---|---|
| `1` | `yoloScan()` | Find and classify a waste item. |
| `2` | `yoloMove()` | Drive to align the item with the camera's pickup target. |
| `3` | `moveArm()` | Use the measured image height to choose a calibrated arm position and pick up the item. |
| `4` | `cvMove()` | Track the bin's color and align with the bin. |
| `5` | `putToTrash()` | Move over the bin, release the item, and return the arm to standby. |

After stage `5`, the controller resets the stage to `1` and looks for another item.

#### 1. Find an item: `yoloScan()`

Each call reads a camera frame and runs the YOLO model. It only accepts model labels in the controller's waste-category list:

| Model label | Internal `Trash` value | Color range used later |
|---|---:|---|
| `DAUN` | 1 | Green |
| `KERTAS` | 2 | Blue |
| `LOGAM FERRO` | 3 | Red |
| `LOGAM NONFERRO` | 4 | Red |
| `PLASTIK` | 5 | Yellow |

The controller keeps the highest-confidence matching detection across **10 calls** to `yoloScan()`. If that best confidence is at least **0.60**, it records the category, selects the corresponding OpenCV color range, resets tracking values, and advances to stage `2`.

If it cannot confirm an item, it stops the drive motors and retries. The preview shows the detection box and confidence or a low-confidence message. If scanning failures continue, the existing code eventually sets a shutdown flag and attempts to shut down the computer after stopping the robot. See [Stopping and safety](#stopping-and-safety).

#### 2. Align the item: `yoloMove()`

The controller runs YOLO again and looks for a detection with the selected label. It computes the detection's center and compares its horizontal coordinate to `SETPOINT_X` (59% of the camera width); the vertical coordinate is also measured.

The horizontal image error is passed through a PID-style calculation:

- **P** responds to the current pixel error.
- **I** accumulates recent error, with a clamp to prevent unlimited accumulation.
- **D** responds to how quickly the error changes.

The gains are `kP = 0.08`, `kI = 0.06`, and `kD = 0.025`. The horizontal output is clamped to ±40 before the controller sends it to the Arduino as command `10`. The code also computes and displays a vertical PID value, but **does not send that value as a movement command**. Instead, it saves the detected vertical error after the horizontal alignment is stable, then uses that measurement to choose a pre-calibrated arm position.

When the horizontal error is within the current tolerance for repeated checks, the stage advances to `3` and the saved vertical error is passed to the arm routine. If the matching object disappears, the controller stops and retries its scan.

#### 3. Pick up the item: `moveArm()`

The saved vertical image error is adjusted by `Y_bias`, then classified into one of five pickup zones. Each zone selects a fixed combination of wrist angle, elbow angle, and shoulder PWM from the `servoup`, `servomid`, and `servobot` arrays. These are **calibrated positions**, not angles calculated from a 3D coordinate.

The arm moves through a pre-pickup position, lowers to the selected position, and turns on pump suction with command `7, 1`. It then lifts and returns toward standby. If the measured position is outside the configured range, the controller displays an out-of-range warning and returns to scanning rather than proceeding with the pickup.

After pickup and arm return, the Python controller enters stage `4`.

#### 4. Find and align with the bin: `cvMove()`

The previously selected category determines the bin color range. OpenCV blurs the frame, converts it from BGR to HSV color space, and creates a mask for pixels within that range. Erosion and dilation help clean up small gaps and specks. The controller finds the largest remaining contour and uses its image center as the bin's target.

It applies the same style of PID calculation to align the target horizontally and sends a lateral-movement command (`10`) while the bin is off target. Repeated alignment checks narrow the tolerance. Once alignment succeeds, the stage advances to `5`.

The two metal categories currently use the **same red HSV range**, so they are not distinguished from each other by the bin-color tracking step.

#### 5. Release the item: `putToTrash()`

This routine moves the arm to the pre-bin position, lowers it over the bin, and sends command `7, 2` to blow/release the item. It waits, turns off the pump with `7, 0`, and returns the shoulder, elbow, and wrist to standby. Finally, it resets tracking data and returns the controller to stage `1`.

### Preview, stage dispatch, and stopping

The `"Detection"` window displays the camera image with boxes, target lines, labels, or status messages. Press **Q** in the window to request a stop.

`loop()` uses a series of independent `if` statements, rather than an `if`/`elif` chain or a one-stage-per-tick dispatcher. Because an action can change `counter`, a later stage may be called in the **same invocation** of `loop()`. Slow arm actions and their waits can therefore follow directly after alignment in one dispatcher pass.

When `stop` is set, the main loop sends a motor-stop command, moves the arm toward standby, resets the serial connection, releases the camera, and closes OpenCV windows. In the special low-confidence timeout path, the code additionally attempts an operating-system shutdown (`shutdown /s /t 1` on Windows or `sudo shutdown now` on Linux). That behavior is part of the current code—not a normal consequence of pressing Q—and should be reviewed before running the program on a computer that must stay on.

## The Arduino firmware

### Hardware initialization

The Arduino `setup()` routine starts serial communication at **9600 baud**, initializes I²C and the LCD, initializes two PCA9685 boards for servos, sets motor, relay, pump, and ultrasonic pin modes, and reads initial sensor distances. It then stops the drive motors, moves the arm to its initial positions, and turns the pump off.

The specific pins and addresses are hard-coded in the sketch. They must match the controller board, motor drivers, relay wiring, LCD, ultrasonic modules, and PCA9685 servo boards used on the robot.

### Serial command format

For each command, Python packs two signed 16-bit integers:

```text
[address: 2 bytes] [value: 2 bytes]
```

At 9600 baud, the Arduino waits until at least four bytes are available, reads the packet, and decodes each pair as a signed integer. The packet has no start marker, checksum, or message ID. The current Python/Arduino combination assumes the little-endian byte order used on the project's Windows PC and Arduino target.

`robot(address, data)` is the Python helper that sends this packet. The address identifies the action; the second integer is the action's value:

| Address | Arduino behavior | Meaning of the value |
|---:|---|---|
| `0` | Stop the drive motors. | Unused; send `0`. |
| `1` | Drive the `Y` direction. | Signed movement input. |
| `2` | Drive the `X` direction. | Signed movement input. |
| `3` | Rotate using the `Z` direction. | Signed movement input. |
| `4` | Set the lower/shoulder actuator via PCA9685 channel 0. | PWM count, clamped by the Arduino routine to `0`–`2100`. |
| `5` | Set the middle/elbow servos. | Servo angle in degrees. |
| `6` | Set the upper/wrist servos. | Servo angle in degrees. |
| `7` | Control the pump and its relays. | `0` off, `1` suction, `2` blow/release. |
| `9` | Run the blocking ultrasonic-guided forward approach. | Unused; send `0`. |
| `10` | Strafe while applying ultrasonic distance/alignment correction. | Signed lateral (`X`) input; `Y` and `Z` corrections are calculated on the Arduino. |
| `11` | No operation in the current sketch. | Ignored. |

**Command `11` is a current code/documentation mismatch:** Python calls `robot(11, 0)` with comments describing it as a reset, but the Arduino's `case 11` is empty. It does not currently reset the PID or perform another action.

The Arduino also has a **100 ms command timeout**. If no complete command packet has been read within that period, it sets the command to `0`, which stops the drive motors. This is a drive-motor watchdog; it is not a physical emergency stop and does not disconnect power to the arm or pump.

### Drive motors and sensors

`motorMove(Y, X, Z)` mixes forward/reverse, lateral, and rotation inputs into four wheel commands for the robot's four motor channels:

```text
left front  = Y + X + Z
right front = Y - X - Z
left rear   = Y + X - Z
right rear  = Y - X + Z
```

For each wheel, the sign selects direction pins and the absolute value selects PWM speed. `clamp()` limits the PWM value to `0`–`200`. The `kecepatan` multiplier scales the requested wheel values. `motorStop()` disables the motor outputs and clears the direction pins.

The two ultrasonic helper functions measure distance in centimeters. The main loop alternates between reading the left and right sensors and refreshes the LCD. Command `9` calls `maju()`, which repeatedly reads those distances, calculates their average and difference, and uses the Arduino's `computePID()` to approach the configured target distance (`28 cm`) while correcting heading. When the measured average reaches the stop distance, it stops the motors and sends a two-byte integer response to Python.

For command `10`, the Arduino uses the ultrasonic readings to calculate `Y` distance correction and `Z` heading correction, then combines those with the requested lateral `X` movement in `geserRobot()`.

`computePID()` is the Arduino-side PID helper. It uses elapsed time to calculate proportional, accumulated-integral, and derivative terms, clamps the integral and final output, and is used by the ultrasonic movement routines.

### Arm, pump, and LCD

The servo helpers convert the upper and middle joint angles into PCA9685 PWM values. The shoulder helper writes a PWM count directly to its PCA9685 channel; despite older comments calling it an angle, Python sends PWM counts for this joint.

`pump()` selects suction, blow, or off by switching the configured relay and pump pins. The LCD shows ultrasonic distances, motion values, wheel outputs, servo settings, and pump state to help with hardware debugging.

## Example: one complete cycle

1. Python starts both devices and sends command `9`.
2. The Arduino moves forward, using ultrasonic feedback until it reaches its distance target, then sends a completion response.
3. Python scans frames until YOLO recognizes one of the configured waste labels at confidence `0.60` or higher.
4. Python aligns the recognized object horizontally, records its vertical image position, and uses that position to choose a calibrated arm pickup zone.
5. Python moves the arm to the item, turns on suction, lifts it, and returns the arm toward standby.
6. Python switches to OpenCV color tracking and strafes until it is aligned with the bin color selected for that category.
7. Python moves the arm over the bin, turns on blow/release, turns the pump off, and returns the arm to standby.
8. Python resets the tracking state and starts a new scan.

## Important configuration and safety notes

- Check the camera index, `ROBOT_SERIAL_PORT`, model-file location, camera resolution, and Arduino wiring before running.
- YOLO and HSV thresholds assume the model labels, lighting, camera placement, and bin colors used during calibration.
- Servo angles, shoulder PWM presets, PID gains, distance targets, and timing delays are hardware-specific calibration values.
- Test with the robot raised or secured, keep clear of the wheels and arm, and have a way to disconnect motor and pump power. A software stop is not a substitute for a physical emergency stop.
- Use the Arduino serial monitor only when Python is not connected; both programs cannot own the same serial port at once.
