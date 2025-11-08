<div align="center">

# 🤖 AlgoRun Robot 2.0

*Autonomous micromouse with floodfill algorithm and speed optimization*


[![SLIIT ROBOFEST 2025](https://img.shields.io/badge/Competition-SLIIT%20ROBOFEST%202025-0066cc?style=for-the-badge&logoColor=white)](http://www.robofest.lk)

</div>

---

## 🚀 Features

- **Floodfill maze mapping** with intelligent exploration
- **Persistent memory** (ESP32 NVRAM) for instant speed runs
- **Advanced motion**: Curve turns, diagonal paths, PID control
- **Webots simulation** for algorithm validation before deployment
- **HC-12 wireless module** for debugging and data transmission

---

## 🔩 Hardware

**Sensors:**
- 5× VL53L0X TOF (Front, Left, Right, 45°L, 45°R)
- 2× Quadrature Encoders

**Specs:**
- ESP32 Dual-Core | 58:1 Gear Motors
- Max: 14.5×14.5cm | <24V | 43mm wheels
- HC-12 wireless module (433MHz)

---

## 📁 Repository Structure

```
algorun-robot-2.0/
├── Algorun_main_3_0_full/
│   └── Algorun_main_3_0_full.ino    # ESP32 firmware (C++)
│
└── Webots/
    ├── controllers/N03/
    │   ├── N03.py                    # Python simulation
    │   └── maze_data.json            # Exported maze data
    └── path/
        └── main maze.obj             # 16×16 Blender maze model
```

---

## 🛠️ Setup

### ESP32 Firmware

**Required Arduino Libraries:**
```cpp
Wire.h           // I2C communication
VL53L0X.h        // Pololu TOF sensor library
Preferences.h    // ESP32 NVRAM storage
```

**Upload:**
```bash
git clone https://github.com/SkyLark-19/algorun-robot-2.0.git
cd algorun-robot-2.0
# Open Algorun_main_3_0_full/Algorun_main_3_0_full.ino in Arduino IDE
# Board: ESP32 Dev Module | Upload Speed: 921600
# Upload to ESP32
```

### Webots Simulation

**Requirements:**
- Webots R2023b or later
- Python 3.8+

**Run Simulation:**
```bash
cd Webots/
# Open Webots and load the world file
# In Robot window: Controller → Select "N03"
# Press ► (Play) to start simulation
```

The Python controller will automatically explore the maze and output results to console.

---

## 🎯 Algorithm Overview

**Floodfill Strategy:**
```
RUN 1: Explore to center goal              
RUN 2: Return to start (explore all cells) 
RUN 3: Fast run using known maze           
RUN 4: Ultra-optimized (curves + diagonals)
```

**Interface:**
- Button 1  : First exploration mode
- Button 2  : Fast run with saved maze
- Button 3  : Ultra-optimized speed run<br><br>
- Green LED : Success/Setup complete
- Red LED   : Wall detection/Stop
- Blue LED  : Mode 4 activated

---

## 🔄 Development Workflow

1. **Simulate** algorithm in Webots (Python)
2. **Port** to ESP32 firmware (C++)
3. **Calibrate** sensors + PID parameters
4. **Test** on physical maze
5. **Optimize** speed runs

---
<div align="center">
   
**Enjoy solving mazes autonomously!**

</div>
