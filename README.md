# MirrorBallBot

A ball-balancing robot that sees through a mirror.
MirrorBallBot keeps a ball centred on a circular platform by tilting it with three stepper motors.
You can control the ball with your finger on a touchscreen, record paths, or let it auto-balance.
MirrorBallBot is fully open-source (all files available, also CAD editable files).


## Demo video
[![Watch the Demo](https://i.ytimg.com/vi/ORftNl7bZG4/maxresdefault.jpg)](https://youtu.be/ORftNl7bZG4)



## Key innovations
<img src=".//images/mbb_principle.png" width="400">

1. Unlike most designs, it uses a mirror and a side-mounted camera.
By considering the same camera and same robot height, the mirror concept increases the Field of View by 58%.
This allows for a larger platform while keeping the robot compact.

2. Finger follower: The ball follows, in real time, the finger's position on the touchscreen.



## Features

- **Touchscreen GUI:** 7" DSI display with full control interface.
- **Real-Time Video Feed:** Live camera images displayed directly on screen.
- **Autonomous Auto-Balance:** Keeps the ball perfectly centred.
- **Finger-Follow Mode:** Drag your finger across the screen to guide the ball.
- **Predefined Paths:** Execute geometric patterns (Square, Circle, Infinity, Triangle, Line).
- **Record & Replay:** Draw custom paths with your finger to record and loop them.
- **Interactive PID Tuning:** Adjust control loop parameters via the GUI in real time.
- **Automatic HSV Calibration:** Instantly adapts to different ball colours and ambient lighting.
- **Sensorless Homing:** Uses TMC2209 StallGuard—no physical limit switches required.
- **Active Thermal Management:** PWM-controlled cooling fans with automated temperature monitoring.



## How It Works
1. **Detection**: The camera looks at a mirror placed underneath the transparent platform, detects the ball via HSV thresholding, the ball position is retrieved. This flow works at more than 110 FPS.
2. **Calculation**: The ball position is compared to the target, and via a PID controller the new platform angle is calculated.
3. **Communication**: The Raspberry Pi 4B sends the information (speed and number of steps) to the three RP2040-Zero, via a custom I2C protocol.
4. **Actuation**: The three RP2040-Zero decode the I2C command to speed, direction and number of steps, and load them into the PIO buffer. Finally, the steppers move the platform. The platform actuation works at 17~25Hz.

### Architecture Overview
- **Raspberry Pi 4B:** Vision processing, PID control, GUI, and I2C Master.
- **3x RP2040-Zero:** Motion execution, I2C Slaves, and hardware-precise PIO step generation.
- **3x TMC2209:** Silent stepper drivers configured for Sensorless homing.
- **3x NEMA17:** Stepper motors executing the platform tilts.



![block_diagram](/images/mbb_block_diagram.JPG)



## Hardware Requirements
| Component | Qty |
|-----------|-----|
| Raspberry Pi 4B (2GB) | 1 |
| RP2040-Zero | 3 |
| TMC2209 stepper driver | 3 |
| NEMA17 stepper motor | 3 |
| PiCamera 3 Wide | 1 |
| 7" DSI touchscreen | 1 |
| MirrorballBot custom PCB | 1 |
| XL4015 step-down converter | 1 |
| 20V 5A power supply | 1 |


> **Looking for the full Bill of Materials?** A comprehensive, itemized BOM with all the components and raw material specs is available in the full documentation.



## Documentation
The complete instruction manual is available in the `doc/` folder as a **150+ page PDF file**. It provides exhaustive coverage of:
- PCB assembly & soldering maps (250+ pads)
- 3D printing and wood/acrylic fabrication
- Detailed step-by-step mechanical assembly (29 steps with photos)
- Software environment setup and configuration parameters
- GUI operation, PID tuning, and troubleshooting/QA



## Quick Start (for experienced makers)

1. 3D print all parts (no supports except Power_in case)
2. Laser cut acrylic (translucent plate + mirror)
3. CNC or route the wood base
4. Assemble the MirrorBallBot PCB (250+ solder pads)
5. Flash each RP2040-Zero with MicroPython v1.24 (Pimoroni Tiny2040 firmware)
6. Copy the MicroPython file into the RP2040-Zero boards (files at https://github.com/AndreaFavero71/mirrorballbot/tree/main/rp2040)
7. Set up the Raspberry Pi:
     - Flash a 32-bit Desktop OS (Trixie, or Bookworm) into a microSD
     - git clone https://github.com/AndreaFavero71/mirrorballbot.git
     - cd mirrorballbot/src
     - bash mbb_install.sh
8. Set the XL4015 voltage (OUT+)
9. Set the drivers Vref
10. Run first tests before full assembly; This includes checking the motor direction (it can be reversed via a settings in mbb_settings.json)
11. Full assembly following the step-by-step guide
12. Calibrations:
     - Camera inclination (centre the camera view to the platform centre)
     - Sensorless homing calibration
     - Ball colour calibration
     - PID tuning
13. Enjoy the robot!




## First Run
After assembly, place the ball on the middle of the platform, and start the GUI:
  ```
  cd ~/mirrorballbot/src
  python3 mbb_gui.py
  ```

The GUI will open on the touchscreen.
Auto-Balance should already be active, and the robot will start moving.



## Tools You'll Need
- 3D printer (Minimum 210x210 mm bed capacity)
- Laser cutter or CNC (for acrylic parts; commercial laser-cutting services work great)
- Hand router or CNC (for carving wire channels in the wood base), unless you prefer the fully 3D printable version
- Soldering iron and basic electronics assembly tools
- Multimeter (essential for driver Vref tuning)
- Precision screwdrivers and ball-end Allen keys
- Computer with SD card reader
- Solid foundational skills in DIY mechatronics and Python scripting



## Credits
- Low-level I2C driver for RP2040 adapted from danjperron
- PCBWay for sponsoring the MirrorBallBot PCB [https://www.pcbway.com/]
- ELECROW for sponsoring the 7" DSI touchscreen [https://www.elecrow.com/7-inch-800-480-dsi-display-touch-screen-with-bracket-compatible-with-raspberry-pi.html?idd=5]



## Feedback
If you decide to take on the challenge and build your own MirrorBallBot, I would love to hear about it! Please open an issue, share your build photos, or submit feedback to let me know how your robot turned out.



## License
MIT License
[License: MIT](https://opensource.org/licenses/MIT) | [Python 3.9+](https://www.python.org/) | [MicroPython 1.24](https://micropython.org/)

