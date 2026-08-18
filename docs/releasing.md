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

## Workflows (armed)

Two `workflow_dispatch`-only workflows automate this. Every job is gated on the
repository variable `RELEASE_ENABLED == 'true'`; with the variable unset a manual
run is a no-op. It is now set, so both workflows are live.

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
4. Allow Actions to open pull requests: Settings → Actions → General →
   Workflow permissions → **Allow GitHub Actions to create and approve pull
   requests**. Off by default. Without it `version-bump.yml` pushes its
   branch and then fails at `gh pr create` with *"GitHub Actions is not
   permitted to create or approve pull requests"*, leaving a stale
   `chore/bump-version-*` branch behind. `release.yml` does not need this.

**Current state of this repo:** `RELEASE_ENABLED = true`, the `crates`
environment exists, and Actions may open PRs. Trusted Publishing is **not**
configured yet — see the first-release note below.

### The first release is different

Trusted Publishing cannot be configured for a crate name that does not exist
yet, so the OIDC path in `release.yml` cannot claim `m0601` or `m0601-cli`.
The first publish is therefore a manual one, from a machine with a crates.io
token:

```sh
cargo login                    # paste a token from crates.io/settings/tokens
cargo publish -p m0601         # library first
cargo publish -p m0601-cli     # resolves m0601 from the index
git tag -a v0.1.0 -m "Release v0.1.0" && git push origin v0.1.0
gh release create v0.1.0 --generate-notes
```

Then configure Trusted Publishing for both crates (crate settings → Trusted
Publishing → this repo, `release.yml`, environment `crates`) and every release
after that goes through the workflow below with no token anywhere. **Done for
`m0601` and `m0601-cli` as of 0.1.0.**

A dry run mints an OIDC token but publishes nothing, so `dry_run: true` checks
the Trusted Publishing wiring as well as the packaging. If the crates.io side
is misconfigured, the dry run fails at the *Authenticate* step rather than
letting you find out mid-publish.

### Cutting a release

1. Move the `## [Unreleased]` entries in [`CHANGELOG.md`](../CHANGELOG.md) under
   the new version heading, and update the two link references at the bottom of
   the file.
2. Check the docs match the code you're shipping — in particular
   `docs/content/docs/cli/` if any flag or behaviour changed. `cargo test
   --workspace` catches drifted *API* snippets (via
   `m0601/examples/usage_doc_check.rs`); it does not catch drifted prose.
3. Run **Version Bump** with the new version → merge the PR it opens.
4. Run **Release** with `dry_run: true` to validate — this exercises the CI
   gate, the version check, the Trusted Publishing handshake, and packaging.
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
