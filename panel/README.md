# Pendulum Panel (PC)

Lightweight Python panel to interact with the ESP pendulum firmware.

Requirements
- Python 3.8+
- Install packages from `requirements.txt`:

```bash
python -m pip install -r panel/requirements.txt
```

Run (recommended)

```bash
python -m panel.panel
```

Or run directly (may require Python path adjustments):

```bash
python panel/panel.py
```

Usage notes
- Set the correct serial port in the GUI (e.g. COM3) and click Connect.
- The panel listens for packets in the form `0xAA addr float32` (device->PC).
- To send commands the panel writes 5-byte packets `addr + float32` (PC->device).

This is a minimal starter panel. Extend controls to send specific parameters (addresses are in the source comments).
