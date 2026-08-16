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
- The theme is a git submodule, pinned to hugo-book **v0.14.0**. After cloning:

  ```sh
  git submodule update --init --recursive
  ```

  Without this, Hugo still exits 0 but renders a layout-less site — every page
  a bare 404. The pin is a tag rather than upstream `main`, which has since
  dropped SASS in a breaking restyle. Git stores the submodule as a commit, so
  the tag name is recorded in a comment in `.gitmodules`; to bump it, check out
  the new tag in `docs/themes/hugo-book`, commit the pointer, and edit that
  comment.

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

## CI and deployment

The site lives in its own workflow, `.github/workflows/docs-site.yml`, separate
from the Rust CI suite — the Rust jobs run on every push and PR, whereas
rebuilding the published site is a deliberate act. It is **`workflow_dispatch`
only**, with a `deploy` checkbox (default on):

- **unchecked** — build only. Checks out with `submodules: recursive`, installs
  pinned Hugo extended (checksum-verified), and runs
  `hugo --minify --panicOnWarning`, so a missing layout or a broken `ref` fails
  the build instead of shipping quietly.
- **checked** — the same build with `--baseURL` from `actions/configure-pages`,
  then deploys `docs/public/` to GitHub Pages via `actions/deploy-pages`.

Pages is **not enabled on the repo yet**, so the deploy path will fail until
Settings → Pages has *Source: GitHub Actions* selected; the build-only path
works today. The rustdoc API reference (`cargo doc --no-deps -p m0601`) is still
planned for `/api/` and is not published by this workflow.
