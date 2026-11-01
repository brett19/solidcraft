# shellcheck shell=bash
# Shared setup for the packaging scripts. Source it: `. "$(dirname "$0")/../env.sh"`.
# Adapted from PhotoCraft's packaging (storytold/photocraft, Apache-2.0; see ATTRIBUTION.md).
#
# Exports:
#   ROOT                    workspace root
#   VERSION                 [workspace.package] version from Cargo.toml (override: SOLIDCRAFT_VERSION)
#
# Every SOLIDCRAFT_* override also accepts the pre-rename SOLVECRAFT_* name when the new one is
# unset (compatibility shim, see docs/compatibility.md).
#   DIST                    output directory for release artifacts (default: $ROOT/dist/release)
#   SOLIDCRAFT_BUILD_DATE   UTC build date, YYYY-MM-DD
#   CARGO_TARGET_DIR        cargo's target dir (default: $ROOT/target)

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROOT

# The version lives in one place: `[workspace.package] version` in the root Cargo.toml.
workspace_version() {
  awk '
    /^\[/ { in_pkg = ($0 == "[workspace.package]") ; next }
    in_pkg && $1 == "version" { gsub(/[" ]/, "", $3); print $3; exit }
  ' "$ROOT/Cargo.toml"
}

VERSION="${SOLIDCRAFT_VERSION:-${SOLVECRAFT_VERSION:-$(workspace_version)}}"
if [ -z "$VERSION" ]; then
  echo "error: could not read [workspace.package] version from $ROOT/Cargo.toml" >&2
  exit 1
fi
export VERSION

DIST="${DIST:-$ROOT/dist/release}"
mkdir -p "$DIST"
DIST="$(cd "$DIST" && pwd)"
export DIST

export SOLIDCRAFT_BUILD_DATE="${SOLIDCRAFT_BUILD_DATE:-${SOLVECRAFT_BUILD_DATE:-$(date -u +%Y-%m-%d)}}"
export CARGO_TARGET_DIR="${CARGO_TARGET_DIR:-$ROOT/target}"
# SOLIDCRAFT_SOLIDCORE_ONLY=1 builds without the C++ SMLib kernel: no `smlib` feature (and no
# scripts/fetch-smlib.sh), each app with its other default features. APP_FEATURES holds the cargo
# arguments for `-p solidcraft -p solidcraft-cli`; CLI_FEATURES those for `-p solidcraft-cli` alone;
# WEB_FEATURES those for solidcraft-web (trunk).
APP_FEATURES=()
CLI_FEATURES=()
WEB_FEATURES=()
if [ -n "${SOLIDCRAFT_SOLIDCORE_ONLY:-}" ]; then
  APP_FEATURES=(--no-default-features --features solidcraft/raytrace-cpu,solidcraft/bundled-render-library,solidcraft/profile,solidcraft-cli/raytrace-cpu,solidcraft-cli/profile)
  CLI_FEATURES=(--no-default-features --features raytrace-cpu,profile)
  WEB_FEATURES=(--no-default-features --features profile)
fi
# The reverse-DNS id: Flatpak app id, desktop, metainfo and MIME file names, icon name and the
# window's Wayland app_id / X11 class. Earlier releases used ai.storyteller.solidcraft (0.8 until
# the move to Tangency Labs) and ai.storyteller.solvecraft (before 0.8); the Flatpak repository
# rebases both to this one (packaging/flatpak-site.sh, docs/compatibility.md).
APP_ID=io.tangencylabs.solidcraft
export APP_ID

# Emit a GitHub Actions warning (plain stderr outside Actions).
warn() {
  if [ -n "${GITHUB_ACTIONS:-}" ]; then echo "::warning::$*"; else echo "warning: $*" >&2; fi
}

# Copy the licence, notice and readme files into a package directory. The programs also carry
# them (Help > About > Licences, `solidcraft-cli licences`).
copy_docs() {
  local dest="$1" f
  for f in README.md LICENSE-MIT LICENSE-APACHE NOTICE THIRD-PARTY-LICENSES.txt; do
    if [ -f "$ROOT/$f" ]; then cp "$ROOT/$f" "$dest/"; fi
  done
}

# Copy canonical package exports; $1 is retained for callers' CLI argument compatibility.
make_icons() {
  local out="$2" s
  for s in 16 24 32 48 64 128 256 512; do
    mkdir -p "$out/hicolor/${s}x${s}/apps"
    cp "$ROOT/assets/icons/app/linux/exports/hicolor/${s}x${s}/apps/$APP_ID.png" "$out/hicolor/${s}x${s}/apps/$APP_ID.png" || return
  done
  cp "$ROOT/assets/icons/app/windows/exports/solidcraft.ico" "$out/solidcraft.ico"
}
