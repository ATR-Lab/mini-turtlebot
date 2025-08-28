# Firmware (Pico‑W)

Reference notes for the Pico‑W implementation:
- Bring up Wi‑Fi and host three services: :8080 (MJPEG), :5601 (LiDAR UDP), :9000 (Control JSONL TCP).
- Keep handlers non‑blocking; JSONL yields simple, human‑debuggable control.
- Camera: SPI/I²C via Turtle‑Head module. 
