# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).
The `m0601` library and the `m0601-cli` binary share one version and are released in
lockstep; `m0601-quad` is a sample and is not published.

## [Unreleased]

### Added

- Unit tests for `raw`'s exit-brake path: the post-parse logic now runs over any
  `Transport`, so a `MockTransport` verifies the brake chases the addressed motor,
  falls back to `--id` on broadcast, skips non-motion frames, and still fires when
  the exchange itself errors.

### Changed

- `raw`'s exit brake now targets the motor the frame actually addressed (byte 0)
  when that is a valid unicast id, falling back to `--id` for broadcast (`0xC8`)
  and reserved addresses — and the brake is attempted (best-effort) even when the
  exchange itself errors, since the frame may already be on the wire.
