# M0601 protocol & hardware reference

**This document has moved into the documentation site**, which is now the canonical
home for it:

### → [`docs/content/docs/protocol.md`](docs/content/docs/protocol.md)

It covers the same ground it always did, and stays in step with the code:

- **Identity** — the DFRobot M0601 as a rebadged Direct Drive Tech M0601C-111, and
  the FIT1042 / FIT1038 SKUs.
- **Electrical & mechanical specifications** — the full product-page table.
- **Wiring** — the 4-pin signal and 2-pin power cables, and the gotchas in the order
  they actually bite (A/B polarity first).
- **Link layer** — 115200 8N1 half-duplex, the fixed 10-byte frame, addressing, the
  adapter echo, and the polling model.
- **CRC** — CRC-8/MAXIM, its check value, and the two host frames that carry no
  checksum.
- **Host → motor frames** — drive (`0x64`), feedback query (`0x74`), mode switch
  (`0xA0`), set ID, and the broadcast ID query, byte by byte with worked vectors.
- **Motor → host telemetry** — both reply layouts and the fault byte.
- **Modes** — the three control loops and their setpoint ranges.
- **Known contradictions between sources** — every disagreement between the wiki, the
  DDT vendor sample, navigation_robot, and MotorLink, with how each was settled (or
  that it wasn't).
- **Sources** — per-claim sourcing.

To read it rendered rather than as raw Markdown:

```sh
cd docs && hugo server      # http://localhost:1313/
```

For what these frames *mean* — rather than what bytes they are — see
[`docs/content/docs/concepts/`](docs/content/docs/concepts), especially
`the-bus.md` and `telemetry-and-echo.md`.
