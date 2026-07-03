# Quadcopter Hardware Wiring Diagram

Here is a visual map of all hardware components connected to the FRDM-K22F Flight Controller! 

```mermaid
graph TD
    %% Define Styles
    classDef mcu fill:#1b1b1b,stroke:#8b5cf6,stroke-width:3px,color:#fff,rx:10px,ry:10px
    classDef shield fill:#374151,stroke:#10b981,stroke-width:2px,color:#fff,rx:10px,ry:10px
    classDef receiver fill:#1e3a8a,stroke:#3b82f6,stroke-width:2px,color:#fff
    classDef esc fill:#7f1d1d,stroke:#ef4444,stroke-width:2px,color:#fff
    classDef sensor fill:#7c2d12,stroke:#f59e0b,stroke-width:2px,color:#fff

    %% Components
    K22F["💻 NXP FRDM-K22F MCU (Flight Controller)"]:::mcu
    
    subgraph Sensors [Sensors]
    SHIELD["🟢 NXP STBC-AGM01 Shield (Plugged on top)"]:::shield
    DPS310["🧭 DPS310 Barometer (External Breakout)"]:::sensor
    end

    RX["📡 Turnigy 9X RC Receiver"]:::receiver

    M1["🔴 ESC 1 (Front Left)"]:::esc
    M2["🔴 ESC 2 (Front Right)"]:::esc
    M3["🔴 ESC 3 (Rear Left)"]:::esc
    M4["🔴 ESC 4 (Rear Right)"]:::esc

    %% I2C Bus (Sensors)
    K22F ---|I2C0 SCL PTB2 / I2C0 SDA PTB3| SHIELD
    SHIELD ---|Pass-through SCL / Pass-through SDA| DPS310

    %% PWM Capture (Receiver)
    RX -->|CH3 / Throttle| K22F_PTA12["PTA12"]
    RX -->|CH1 / Roll| K22F_PTB18["PTB18"]
    RX -->|CH2 / Pitch| K22F_PTC8["PTC8"]
    RX -->|CH4 / Yaw| K22F_PTC10["PTC10"]
    RX -->|CH5 / Mode| K22F_PTD0["PTD0"]
    RX -->|CH6 / Landing| K22F_PTD2["PTD2"]
    
    K22F_PTA12 -.-> K22F
    K22F_PTB18 -.-> K22F
    K22F_PTC8 -.-> K22F
    K22F_PTC10 -.-> K22F
    K22F_PTD0 -.-> K22F
    K22F_PTD2 -.-> K22F

    %% PWM Generation (Motors)
    K22F -->|PTC1| M1
    K22F -->|PTC2| M2
    K22F -->|PTC3| M3
    K22F -->|PTC4| M4

    %% Power
    BATT(("🔋 Power (3.3V & GND)")) -.-> K22F
    K22F -.-> BATT_RX(("To RX Power"))
    K22F -.-> BATT_SENS(("To DPS310 Power"))
```

> [!TIP]
> **Grounding is Critical**
> Ensure the Ground (GND) wire from your RC Receiver and the DPS310 are both connected back to the FRDM-K22F. The ESCs should also share a common ground with the flight controller if they are powered from a separate battery.
