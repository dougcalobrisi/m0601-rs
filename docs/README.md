# Documentation site

The `m0601` documentation site, built with [Hugo](https://gohugo.io/) and the
[hugo-book](https://github.com/alex-shpak/hugo-book) theme. The source content in
`content/` is the canonical home for the CLI and library guides; the root
`README.md`, `USAGE.md`, and `PROTOCOL.md` remain the in-repo quick references.

## Prerequisites

- **Hugo extended** ≥ 0.158 (`hugo version` should print `+extended`).
- The theme is a git submodule. After cloning:

  ```sh
  git submodule update --init --recursive
  ```

## Preview locally

```sh
cd docs
hugo server        # live-reload at http://localhost:1313/
```

## Build

```sh
cd docs
hugo --minify      # static site into docs/public/ (gitignored)
```

## Deployment

GitHub Pages deployment is **not wired up yet** — no workflow, and Pages is not
enabled on the repo. When it is, the plan is to publish this site plus the rustdoc
API reference (`cargo doc --no-deps -p m0601`) under `/api/`.
