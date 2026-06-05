# MirrorBallBot

A ball-balancing robot that sees through a mirror.<br>
MirrorBallBot keeps a ball centred on a circular platform by tilting it with three stepper motors.<br>
You can control the ball with your finger on a touchscreen, record paths, or let it auto-balance.<br>
MirrorBallBot is fully open-source (all files available, also CAD editable files).<br>

<br><br>
## Demo video
[![Watch the Demo](https://i.ytimg.com/vi/ORftNl7bZG4/maxresdefault.jpg)](https://youtu.be/ORftNl7bZG4)


<br><br>
## key innovations
<img src=".//images/mbb_principle.png" width="400">
1. Unlike most designs, it uses a mirror and a side-mounted camera.<br>
By considering a direct camera placement, having the same camera and robot height, the mirror concept increases the Field of View by 58%.<br>
This allows for a larger platform while keeping the robot compact.<br>

2. Finger follower: The ball follows, in real time, the finger's position on the touchscreen.


<br><br>
## Features

- Touchscreen GUI - 7" DSI display with full control interface
- Camera image - Real time camera's images on the screen
- Auto-balance - keeps the ball centred
- Finger-follow mode - move the ball by touching the screen
- Predefined paths - square, circle, infinity, triangle, line
- Record and replay - custom paths from finger drawing
- Interactive PID tuning - real-time adjustments via GUI
- Automatic HSV calibration - adapts to different ball colours and lighting
- Sensorless homing - no limit switches required (TMC2209 StallGuard)
- PID setting at the GUI - real time adjustment
- Active cooling - PWM-controlled fans with temperature monitoring


<br><br>
## How It Works
1. **Detection**: The camera looks at a mirror placed underneath the transparent platform, detects the ball via HSV thresholding, the ball position is retrieved. This flow works at more than 110 FPS.<br>
2. **Calculation**: The ball position is compared to the target, and via a PID controller the new platform angle is calculated.<br>
3. **Communication**: The Raspberry Pi 4B+ sends the information (speed and number of steps) to the three RP2040-Zero, via a custom I2C protocol.<br>
4. **Actuation**: The three RP2040-Zero decode the I2C command to speed, direction and number of steps, and load them into the PIO buffer. Finally, the steppers move the platform. The platform actuation works at 17~25Hz.<br>

&ensp;**Architecture:**
- Raspberry Pi 4B+: Vision processing, PID control, GUI, I2C master
- 3x RP2040-Zero: Motion execution, I2C slave, PIO step generation
- 3x TMC2209: Silent stepper drivers with StallGuard
- 3x NEMA17: Stepper motors tilting the platform
<br><br>
![block_diagram](/images/mbb_block_diagram.JPG)


<br><br>
## Hardware Requirements
| Component | Qty |
|-----------|-----|
| Raspberry Pi 4B+ (2GB) | 1 |
| RP2040-Zero | 3 |
| TMC2209 stepper driver | 3 |
| NEMA17 stepper motor | 3 |
| PiCamera 3 Wide | 1 |
| 7" DSI touchscreen | 1 |
| MirrorballBot custom PCB | 1 |
| XL4015 step-down converter | 1 |
| 20V 5A power supply | 1 |

Full BOM in the documentation.

<br><br>
## Documentation
The complete instruction manual, **a 100+ pages PDF file**, is available in the doc/ folder.<br>
It covers BOM, 3D printing, PCB assembly, wood base preparation, Raspberry Pi and RP2040 setup, first tests, full assembly (27 steps with photos), GUI operation, PID tuning, I2C protocol details, configuration parameters, troubleshooting, and Q&A.


<br><br>
## Quick Start (for experienced makers)

1. 3D print all parts (no supports except Power_in case)
2. Laser cut acrylic (translucent plate + mirror)
3. CNC or route the wood base
4. Assemble the MirrorBallBot PCB (250+ solder pads)
5. Flash each RP2040-Zero with MicroPython v1.24 (Pimoroni Tiny2040 firmware)
6. Set up the Raspberry Pi:
     - git clone https://github.com/AndreaFavero71/mirrorballbot.git<br>
     - cd mirrorballbot/src<br>
     - bash mbb_install.sh<br>
7. Run first tests before full assembly
8. Full assembly following the step-by-step guide
9. Enjoy!

Potential first-run issues: Motor direction (reverse in settings.json), Vref adjustment, ball colour calibration.


<br><br>
## First Run
After assembly, start the GUI:
  - cd ~/mirrorballbot/src<br>
  - python3 mbb_gui.py<br>

The GUI will open on the touchscreen. Enable Auto-Balance and place the ball on the platform.


<br><br>
## Tools You'll Need
- 3D printer (210x210 mm bed minimum)
- Laser cutter or CNC for acrylic parts (find for a service eventually)
- Soldering iron and basic electronics tools
- Hand router or CNC for wood base (only needed for wires hiding)
- Screwdrivers, Allen keys (ball-end recommended)
- Multimeter (for Vref adjustment)
- Computer with SD card reader
- DIY and coding skills


<br><br>
## Credits
- Low-level I2C driver for RP2040 adapted from danjperron
- PCBWay for sponsoring the MirrorBallBot PCB [https://www.pcbway.com/]
- ELECROW for sponsoring the 7" DSI touchscreen [https://www.elecrow.com/7-inch-800-480-dsi-display-touch-screen-with-bracket-compatible-with-raspberry-pi.html?idd=5]


<br><br>
## Please leave a feedback if you build it
I hope many of you will decide to build your own MirroBallBot, and that you'll enjoy it as much much as I did.<br />
If you build one, please feedback !


<br><br>
## License
MIT License<br>
[License: MIT](https://opensource.org/licenses/MIT) | [Python 3.9+](https://www.python.org/) | [MicroPython 1.24](https://micropython.org/)

