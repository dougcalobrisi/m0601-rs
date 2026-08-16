# Documentation site

The `m0601` documentation site, built with [Hugo](https://gohugo.io/) and the
[hugo-book](https://github.com/alex-shpak/hugo-book) theme.

**`content/` is the canonical documentation.** The root `README.md` is the project's
landing page; `USAGE.md` and `PROTOCOL.md` are pointers into `content/`, kept so
existing in-repo links and rustdoc references don't dead-end. When the CLI or the
library API changes, update the page under `content/` — that is the copy readers are
sent to, and the only one that needs to be right.

A useful backstop: the Rust snippets in `content/docs/library/` mirror
`m0601/examples/usage_doc_check.rs`, which CI compiles. A documented signature that
drifts from the real API breaks the build. Prose is not covered by that, so CLI flag
and behavior changes still need a deliberate pass over `content/docs/cli/`.

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
