# Camera Calibration (Homography)

Maps camera pixel coordinates → real-world millimeters in the robot's base frame using a 3×3 homography matrix computed via DLT (Direct Linear Transform) from 4 known reference points.

## Physical setup

1. **Print** a 200×200 mm square with 4 dark circles (~10 mm diameter), one near each corner
2. **Place** the paper on the workspace, centered under the robot base
3. **Connect** via Serial Monitor (115200 baud)

The 4 dots define the robot coordinate frame:

```
                +X (robot forward)
                ▲
                │
   (-100, 100) ● ────────── ● (100, 100)
                │              │
                │    ┌─── robot│base (0,0)
                │    │         │
   (-100,-100) ● ────────── ● (100,-100)
                │
                └──────────────────► +Y
```

## Calibration procedure

| Step | Serial command | What happens |
|------|---------------|--------------|
| 1 | `CALIB` | Enter calibration mode (normal loop paused) |
| 2 | `SCAN` | Captures one frame, finds 4 dark dots, reports pixel coords |
| 3 | `SOLVE` | Computes homography matrix, prints it + per-dot reprojection error |
| 4 | `SAVE` | Persists homography to NVS (ESP32 non-volatile storage) |
| 5 | `END` | Exit calibration mode, resume normal capture + detection |

If a dot is not found (e.g. `SCAN` reports fewer than 4), adjust lighting/paper and re-`SCAN`.

## CLI reference

| Command | Description |
|---------|-------------|
| `CALIB` | Enter calibration mode |
| `SCAN` | Capture frame, detect 4 dark dots |
| `REVIEW` | Print last SCAN pixel coordinates |
| `SOLVE` | Compute homography from SCAN data |
| `SAVE` | Persist homography to NVS |
| `ERASE` | Remove saved homography from NVS |
| `END` | Exit calibration mode |
| `SERVOTEST` | Enter interactive servo test mode (see below) |
| `HOME` | Return all servos to 0° (center) |
| `HELP` | Print command list |

### Servo test mode

Send `SERVOTEST` to enter interactive mode. Type space-separated angles to move all servos simultaneously:

```
45 -30 20 10    ← move servo1=45°, servo2=-30°, servo3=20°, servo4=10°
HOME            ← return all servos to 0°
END             ← exit servo test mode
```

Angles are logical degrees (0 = center). The `toPhysical()` mapping converts them to servo.write() values automatically.

### Servo mapping

| Servo | Pin | Joint | Limits |
|-------|-----|-------|--------|
| 1 | 12 | Base (rotation) | -90° to 90° |
| 2 | 14 | Shoulder | -90° to 20° |
| 3 | 33 | Elbow | 0° to 60° |
| 4 | 32 | Gripper | -20° to 40° |

## Troubleshooting

**`SCAN` finds < 4 dots**
- Ensure all 4 dots are visible to the camera (paper fully in frame)
- Improve lighting — dots should be clearly darker than the paper
- Check camera focus
- `SCAN` again after adjustments

**Reprojection error > 10 mm after `SOLVE`**
- Paper may not be flat (tape down edges)
- Dot positions in the image may be off-center in their quadrants
- Re-`SCAN` with better lighting/alignment

**Recalibration**
- Just repeat the full procedure — `SAVE` overwrites the previous homography
- Or send `ERASE` first to clear NVS, then re-calibrate

## How it works

1. `SCAN` divides the 320×240 image into 4 quadrants (TL, TR, BL, BR)
2. In each quadrant, finds the centroid of dark pixels (R < 80, G < 80, B < 80)
3. `SOLVE` builds an 8×8 linear system from the 4 pixel→mm point pairs:
   - TL dot: pixel(u₀,v₀) → (-100, +100) mm
   - TR dot: pixel(u₁,v₁) → (+100, +100) mm
   - BL dot: pixel(u₂,v₂) → (-100, -100) mm
   - BR dot: pixel(u₃,v₃) → (+100, -100) mm
4. Solves via Gaussian elimination with partial pivoting
5. During normal operation, every detected object centroid is multiplied by H:
   ```
   x_mm = (h00·u + h01·v + h02) / (h20·u + h21·v + 1)
   y_mm = (h10·u + h11·v + h12) / (h20·u + h21·v + 1)
   ```

If no homography is saved, the system falls back to a simple linear mapping using WORKSPACE_X_MM / WORKSPACE_Y_MM.
