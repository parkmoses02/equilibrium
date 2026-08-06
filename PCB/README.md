# Freenove ESP32 WROOM + TMC5160T Pro Carrier Rev.C Routed

Two-layer routed carrier reconstructed from the supplied images.

## Mechanical assumptions

- PCB: 100 x 80 mm, four M3 holes
- ESP32 WROOM headers: 2.54 mm pitch, 20 pins per side, assumed 27.94 mm row spacing
- TMC5160T Pro: 2.54 mm pitch, assumed 15.24 mm row spacing

## Routing rules applied

- VMOT: 2.00 mm
- Motor phases A1/A2/B1/B2: 1.50 mm
- +5V: 1.00 mm
- +3V3: 0.50 mm
- Digital and encoder signals: 0.30 mm
- Ground zones: F.Cu and B.Cu
- Encoder: 4.7k pull-ups to 3.3 V and 100-ohm series resistors

## Before fabrication

GPIO16 and GPIO17 are left unconnected. Press B in KiCad to refill zones and run DRC. Confirm both module footprints and every connector orientation with a 1:1 paper print and the real hardware before ordering.
