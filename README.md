# vehicular-collision-detection-ns3

A Vehicular Ad-hoc Network (VANET) simulation project using NS-3.38 that detects potential collisions between moving vehicles and issues collision warnings via DSRC (WAVE) communication. Each vehicle broadcasts its position periodically, builds a neighbor table, calculates distances, and sends warnings when another vehicle is too close.

---

## 📌 Features

- Simulates vehicle movement using NS-3's mobility models.
- Uses WAVE (Wireless Access in Vehicular Environments) modules for communication.
- Each vehicle broadcasts "Hello" packets with its coordinates.
- Vehicles maintain neighbor tables and calculate distances to nearby vehicles.
- Sends collision warnings when another vehicle is within a threshold distance.
- Displays:
  - Vehicle positions every second.
  - Neighbor tables.
  - Colored terminal output for events (Hello messages, warnings, etc.).

---

## 🛠 Dependencies

- **NS-3.38** (https://www.nsnam.org/)
- C++ Compiler (e.g., `g++`)
- Linux/Unix environment recommended for terminal color support.

---
