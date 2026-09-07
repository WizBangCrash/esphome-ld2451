# LD2451 ESPHome Component — Configuration Reference

This document summarises every sensor, binary sensor, text sensor, number, select, switch, button, and automation action exposed by the `ld2451` custom ESPHome component. Use it alongside `ld2451-example.yaml` as a quick reference.

---

## Hub

The `ld2451:` platform entry is the root component and must be declared once per sensor module.

| Key | Required | Description |
|-----|----------|-------------|
| `id` | No | ESPHome ID for referencing this hub in sub-platforms and automations |
| `uart_id` | Yes | ID of the `uart:` bus. Must be 256000 baud, no parity, 1 stop bit |

> The component validates UART settings at compile time via `FINAL_VALIDATE_SCHEMA`. Mismatched baud rate or parity will produce a build error.

---

## Sensors

Platform: `sensor`

### Top-level

| Key | Unit | Description |
|-----|------|-------------|
| `target_count` | — | Number of targets currently detected (0–5) |

### Per-target (`targets` list, max 5 entries)

Each list entry may contain any combination of the three sub-sensors. All default to a 1 s `throttle_with_priority` filter.

| Key | Unit | Device Class | Description |
|-----|------|-------------|-------------|
| `angle` | ° | — | Horizontal angle of the target relative to sensor boresight |
| `distance` | m | `distance` | Slant range to the target |
| `speed` | km/h | `speed` | Radial speed of the target (positive = moving away) |

---

## Binary Sensors

Platform: `binary_sensor`

### Top-level

| Key | Description |
|-----|-------------|
| `has_target` | `ON` when at least one target is detected. Recommended `device_class: occupancy` |
| `has_approaching_target` | `ON` when at least one target has a velocity directed toward the sensor |

All binary sensors default to a 250 ms `settle` filter to suppress rapid state chatter.

### Per-target (`targets` list, max 5 entries)

| Key | Description |
|-----|-------------|
| `approaching` | `ON` when this specific target is moving toward the sensor |

---

## Text Sensors

Platform: `text_sensor`

| Key | Entity Category | Description |
|-----|----------------|-------------|
| `firmware_version` | `diagnostic` | Firmware version string read from the LD2451 module on boot |

---

## Numbers

Platform: `number`

All numbers are writable and persist to the module's non-volatile memory.

| Key | Unit | Min | Max | Step | Description |
|-----|------|-----|-----|------|-------------|
| `max_distance` | m | 10 | 100 | 1 | Maximum detection range. Targets beyond this are ignored |
| `min_speed` | km/h | 0 | 120 | 1 | Minimum radial speed threshold. Slower targets are filtered out |
| `no_target_delay` | s | 1 | 30 | 1 | Seconds to wait after last detection before clearing `has_target` |
| `snr_threshold` | — | 3 | 8 | 1 | Signal-to-noise threshold. Higher values reduce false positives |

---

## Select

Platform: `select`

| Key | Options | Description |
|-----|---------|-------------|
| `detection_direction` | `AWAY`, `TOWARD`, `ALL` | Filter detections by movement direction relative to the sensor |

---

## Switches

Platform: `switch`

| Key | Description |
|-----|-------------|
| `require_multiple_detections` | When `ON`, requires consecutive detections before reporting a target. Reduces spurious hits at the cost of slightly higher latency |
| `bluetooth` | Enables or disables the LD2451 module's onboard Bluetooth interface |

---

## Buttons

Platform: `button`

All buttons have `entity_category: config`.

| Key | Device Class | Description |
|-----|-------------|-------------|
| `factory_reset` | — | Resets all module parameters to factory defaults |
| `restart` | `restart` | Reboots the LD2451 module |
| `refresh_config` | — | Reads current configuration from the module and updates all `number` and `select` entities in Home Assistant |

---

## Automation Actions

The following actions can be called from ESPHome automations or exposed as Home Assistant services.

| Action | Description |
|--------|-------------|
| `ld2451.factory_reset` | Equivalent to pressing the `factory_reset` button |
| `ld2451.restart` | Equivalent to pressing the `restart` button |
| `ld2451.refresh_config` | Equivalent to pressing the `refresh_config` button |

**Usage example:**

```yaml
esphome:
  on_boot:
    priority: -100
    then:
      - ld2451.refresh_config: ld2451_radar
```

Calling `refresh_config` on boot ensures Home Assistant displays the module's current persisted settings immediately after device startup.

---

## UART Requirements

| Parameter | Value |
|-----------|-------|
| Baud rate | 256000 |
| Parity | NONE |
| Stop bits | 1 |
| TX pin | Required |
| RX pin | Required |

---

## Notes

- **Target indexing** — targets are numbered by the module's internal tracking order, which may shift between frames if a target is lost and re-acquired. Do not rely on a fixed mapping between target index and a physical object.
- **Throttle filters** — all numeric sensors apply a 1 s `throttle_with_priority` filter by default to reduce Home Assistant state update noise. Override per-sensor if higher resolution is needed.
- **`MULTI_CONF: True`** — multiple LD2451 modules can coexist on a single ESP32 by declaring multiple `ld2451:` blocks with different `id` and `uart_id` values.
