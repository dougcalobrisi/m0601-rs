# Releasing to crates.io

This workspace publishes two crates and keeps one private:

| Crate        | crates.io | Install                    |
|--------------|-----------|----------------------------|
| `m0601`      | yes       | `cargo add m0601`          |
| `m0601-cli`  | yes (binary `m0601`) | `cargo install m0601-cli` |
| `m0601-quad` | no (`publish = false`) | clone the repo      |

All three share `[workspace.package].version` — releases move in lockstep.

## Publish order

`m0601-cli` depends on `m0601` and resolves it from crates.io at publish time
(the `path` is stripped, the `version` from the `workspace.dependencies`
`m0601` entry is what ships). So the library must be published first:

```
cargo publish -p m0601
cargo publish -p m0601-cli   # after m0601 is live in the index
```

`cargo` (≥1.66) waits for a freshly published crate to appear in the index
before returning, so the sequential publish is safe. `m0601-quad` is skipped
automatically via `publish = false`.

## Workflows (currently disabled)

Two `workflow_dispatch`-only workflows automate this. **Both are disabled**:
every job is gated on the repository variable `RELEASE_ENABLED == 'true'`, so a
manual run is a no-op until the variable is set.

- **`.github/workflows/version-bump.yml`** — takes a version, bumps the
  workspace version + `Cargo.lock`, and opens a PR. Publishes nothing.
- **`.github/workflows/release.yml`** — runs the CI suite, publishes `m0601`
  then `m0601-cli` via crates.io **Trusted Publishing (OIDC)**, then tags and
  cuts a GitHub Release. Defaults to `dry_run: true`.

### Arming the workflows

1. Set repo variable `RELEASE_ENABLED = true`
   (Settings → Secrets and variables → Actions → Variables).
2. Configure crates.io **Trusted Publishing** for both `m0601` and
   `m0601-cli` (crate settings → Trusted Publishing), pointing at this repo,
   the `release.yml` workflow, and the `crates` environment.
   - First publish of a brand-new crate name may need a one-time
     `CARGO_REGISTRY_TOKEN` secret instead, since crates.io can't verify OIDC
     against a crate that doesn't exist yet. Switch to OIDC-only afterward.
3. Create a `crates` environment (Settings → Environments) for the OIDC
   `id-token` grant and optional approval gate.

### Cutting a release

1. Move the `## [Unreleased]` entries in [`CHANGELOG.md`](../CHANGELOG.md) under
   the new version heading, and update the two link references at the bottom of
   the file.
2. Check the docs match the code you're shipping — in particular
   `docs/content/docs/cli/` if any flag or behaviour changed. `cargo test
   --workspace` catches drifted *API* snippets (via
   `m0601/examples/usage_doc_check.rs`); it does not catch drifted prose.
3. Run **Version Bump** with the new version → merge the PR it opens.
4. Run **Release** with `dry_run: true` to validate.
5. Run **Release** with `dry_run: false` to publish, tag, and create the
   GitHub Release.

## Manual publish (no workflow)

The workflows only wrap what you can run locally. With a crates.io token:

```
cargo publish -p m0601 --dry-run     # validate the library
cargo publish -p m0601               # publish it
cargo publish -p m0601-cli --dry-run # now resolvable against the live m0601
cargo publish -p m0601-cli           # publish the CLI
```
