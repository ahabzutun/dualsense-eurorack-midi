# DualSense to Eurorack MIDI

PS5 DualSense controller to MIDI converter for eurorack systems.

## Hardware
- Raspberry Pi (running Raspberry Pi OS Lite)
- Sony DualSense controller
- MIDI interface (TBD)

## Status
🚧 In development
```

## Systemd Services

The project includes two systemd services that auto-start on boot:

1. **dualsense-midi.service** - Main controller to MIDI conversion
2. **midi-passthrough.service** - Routes MIDI to hardware output and handles software MIDI input

### Installation

1. **Copy service files**:
```bash
sudo cp systemd/dualsense-midi.service /etc/systemd/system/
sudo cp systemd/midi-passthrough.service /etc/systemd/system/
```

2. **Reload systemd and enable services**:
```bash
sudo systemctl daemon-reload
sudo systemctl enable dualsense-midi.service
sudo systemctl enable midi-passthrough.service
```

3. **Start services** (or reboot):
```bash
sudo systemctl start dualsense-midi.service
sudo systemctl start midi-passthrough.service
```

### Service Management

Check service status:
```bash
sudo systemctl status dualsense-midi.service
sudo systemctl status midi-passthrough.service
```

View logs:
```bash
sudo journalctl -u dualsense-midi.service -f
sudo journalctl -u midi-passthrough.service -f
```

Restart services:
```bash
sudo systemctl restart dualsense-midi.service
sudo systemctl restart midi-passthrough.service
```

### Service Dependencies

The passthrough service depends on the DualSense service and includes a 3-second startup delay to ensure the virtual MIDI port is available before connecting. Both services will auto-restart on failure.


### 3. Create .gitignore:
```
__pycache__/
*.pyc
.venv/
venv/
.DS_Store


