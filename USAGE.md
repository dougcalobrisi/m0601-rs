# Using m0601

**This guide has moved into the documentation site**, which is now the canonical home
for it. Everything that was here — hardware setup, every CLI subcommand, and the
library cookbook — lives under [`docs/content/docs/`](docs/content/docs) and is kept
in step with the code.

Start at [Getting started](docs/content/docs/getting-started.md), or jump straight to
the [CLI guide](docs/content/docs/cli/_index.md) (a page per subcommand), the
[Library guide](docs/content/docs/library/_index.md), the
[sample code](docs/content/docs/samples/_index.md),
[Safety](docs/content/docs/safety.md), or
[Troubleshooting](docs/content/docs/troubleshooting.md). Each section's `_index.md`
carries the current page inventory, so it is not repeated here. For the type-level
contract, build the API docs: `cargo doc --open -p m0601`.

## Reading it rendered

The pages above are Hugo content, so Markdown viewers show their cross-reference
shortcodes literally. To read the site properly:

```sh
git submodule update --init --recursive   # the hugo-book theme
cd docs && hugo server                    # http://localhost:1313/
```
