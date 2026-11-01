# Object Detection Using DE2-115

Verilog project for camera-based object recognition on the Terasic DE2-115 FPGA board. The design uses a D5M camera, background subtraction, colour and shape features, and nearest-neighbour signature matching. Results are drawn as a bounding box, class label, and similarity score on a VGA display.

## Hardware and tools

- Terasic DE2-115 with Cyclone IV E `EP4CE115F29C7`
- D5M camera connected to the board's camera/GPIO interface
- VGA monitor, USB-Blaster connection, and board power supply
- Quartus installation with support for the target Cyclone IV device

The laptop loads the FPGA configuration. The supplied recognition path runs in FPGA hardware and does not require a laptop inference program or Ethernet connection.

## Recognition method

1. Capture the empty background in learn mode.
2. Sample every fourth pixel and every fourth line of the 800 x 600 video frame.
3. Detect foreground using the difference between the live and stored background colours.
4. Accumulate a 15-byte signature containing mean RGB, an eight-bin colour histogram, size, bounding-box dimensions, and fill ratio.
5. Store up to two signatures for each of ten object labels.
6. Compare the live signature with stored signatures using a weighted L1 distance and display an accepted match.

The displayed percentage is a **similarity score**, not a calibrated probability or measured classification accuracy. This implementation uses feature matching rather than a neural-network detector.

## Object labels

| Switch value (`SW3..SW0`) | Label |
| --- | --- |
| 0 | Person |
| 1 | Bottle |
| 2 | Mobile |
| 3 | Notebook |
| 4 | Cup |
| 5 | Mouse |
| 6 | Keyboard |
| 7 | Laptop |
| 8 | Pouch |
| 9 | Bag |

## Getting started

1. Open [`FPGA/DE2_115_CAMERA.qpf`](FPGA/DE2_115_CAMERA.qpf) in Quartus.
2. Confirm the target device and compile the project.
3. Program the board with the generated `.sof` file.
4. Keep the camera stationary and use a plain background with steady lighting.
5. Set `SW14` high for learn mode and press `KEY2` with an empty scene to store the background.
6. Select a label with `SW3..SW0`, show the object, and press `KEY1` to store its signature. Use `SW4` to select a second sample.
7. Set `SW14` low for detection mode and show one learned object at a time.

Read the [step-by-step guide](DOCS/00_STEP_BY_STEP_GUIDE.txt) for connections, controls, programming, LED meanings, and troubleshooting.

## Main controls

| Control | Function |
| --- | --- |
| `SW14` | Learn / detect mode |
| `SW3..SW0` | Selected label in learn mode |
| `SW4` | Signature sample A / B |
| `SW13..SW12` | Foreground sensitivity |
| `SW15` | Hide the overlay |
| `KEY0` | Reset, including learned state |
| `KEY2` in learn mode | Store the empty background |
| `KEY1` in learn mode | Store the object's signature |

## Repository layout

- `FPGA/DE2_115_CAMERA.v`: top-level board integration
- `FPGA/DE2_115_CAMERA.qsf`: device, source, and pin assignments
- `FPGA/DE2_115_D5M_VGA.sdc`: timing constraints
- `FPGA/v/hw_object_detector.v`: foreground extraction, signatures, and matching
- `FPGA/v/video_overlay.v`: bounding boxes and text overlay
- `FPGA/v/`: camera capture, RGB conversion, VGA, PLL, fonts, and support modules
- `FPGA/Sdram_Control/`: SDRAM controller and FIFO support
- `DOCS/`: setup guide and explanation/viva notes

## Status and limitations

This repository contains the supplied source project. The included notes report simulation experiments, but simulation testbenches and raw reports are not included in this archive. Board testing is marked as pending in those notes; no fresh Quartus compilation or hardware test is claimed by this repository setup.

Recognition assumes one object at a time, a fixed camera, a plain background, and stable lighting. Objects with similar colours and shapes may be confused. Learned signatures are lost on reset or power-off.

Some inherited overlay comments refer to an earlier YOLO/Ethernet pipeline. The current top-level project uses `hw_object_detector.v` for hardware signature matching.

## Third-party notices

The supplied files include Terasic and Altera copyright and usage notices. These notices are retained unchanged. No new repository-wide license is assigned to third-party source.
