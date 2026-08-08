---
title: monitor
weight: 3
---

# `monitor` — live readout, optional CSV

```sh
m0601 monitor --hz 5                 # one-line live dashboard, Ctrl+C stops
m0601 monitor --hz 20 --csv log.csv  # also log rows to log.csv (overwrites it)
```

Monitoring only *queries* — it never drives the motor, so the wheel stays put (or
keeps doing whatever another controller tells it). A transient bus error is
reported and polling continues.

## Options

| Flag | Default | Meaning |
|---|---|---|
| `--hz` | `5.0` | Poll rate in Hz (0.001–1000). |
| `--csv <FILE>` | — | Also log readings to a CSV file (overwrites it). |

## CSV format

Columns: `timestamp,motor_id,mode,speed_rpm,current_a,temp_c,position_deg,error_code,error_str,raw_hex`.

Rows are flushed as written, so a killed session keeps everything logged so far.
