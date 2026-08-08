---
title: set-id
weight: 6
---

# `set-id` — assign a bus address

```sh
m0601 set-id --new 0x02        # asks you to type 'yes' to confirm
m0601 set-id --new 0x02 --yes  # skip the prompt
```

The set-ID frame is **unaddressed**: every motor that hears it takes the new ID.
The CLI therefore polls all 254 IDs first (~40 s) to prove only one motor is
connected. **Wire one motor at a time when assigning IDs.** The ID persists across
power cycles. Avoid `0xC8` (the broadcast address).

## Options

| Flag | Default | Meaning |
|---|---|---|
| `--new <U8>` | *(required)* | New ID, `0x01..0xFE` (hex or decimal). |
| `--yes` | off | Skip the confirmation prompt. |
