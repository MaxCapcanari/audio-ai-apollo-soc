#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage: scripts/apply_vendor_patches.sh [--check] [--reverse]

Applies Audio_AI vendor patches (currently fit_main.c) into AmbiqSuite tree.

Options:
  --check    Dry run only; report whether patch can be applied.
  --reverse  Revert patch (dry-run checked first).
USAGE
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PATCH_FILE="$REPO_ROOT/patches/fit_main.patch"
TARGET_REL="third_party/cordio/ble-profiles/sources/apps/fit/fit_main.c"

MODE="apply"
DRY_RUN=0

for arg in "$@"; do
  case "$arg" in
    --check)
      DRY_RUN=1
      ;;
    --reverse)
      MODE="reverse"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! -f "$PATCH_FILE" ]]; then
  echo "Missing patch file: $PATCH_FILE" >&2
  exit 1
fi

if [[ -n "${AMBIQSUITE_ROOT:-}" ]]; then
  SUITE_ROOT="$AMBIQSUITE_ROOT"
else
  SUITE_ROOT="$(cd "$REPO_ROOT/../../../../.." && pwd)"
fi

TARGET_FILE="$SUITE_ROOT/$TARGET_REL"

if [[ ! -f "$TARGET_FILE" ]]; then
  echo "Target file not found: $TARGET_FILE" >&2
  echo "Set AMBIQSUITE_ROOT to your AmbiqSuite root if this repo is not in the default layout." >&2
  exit 1
fi

# Prefer system patch over Anaconda's broken build
if [[ -x /usr/bin/patch ]]; then
  PATCH=/usr/bin/patch
else
  PATCH="$(command -v patch)"
fi

PATCH_OPTS=( -d "$SUITE_ROOT" -p0 )

if [[ "$MODE" == "reverse" ]]; then
  if ! "$PATCH" "${PATCH_OPTS[@]}" --dry-run -R < "$PATCH_FILE" >/dev/null; then
    echo "Reverse check failed. Patch may not be applied or target differs." >&2
    exit 1
  fi

  if [[ "$DRY_RUN" -eq 1 ]]; then
    echo "Reverse check OK: patch can be reverted in $TARGET_FILE"
    exit 0
  fi

  "$PATCH" "${PATCH_OPTS[@]}" -R < "$PATCH_FILE" >/dev/null
  echo "Reverted patch in: $TARGET_FILE"
  exit 0
fi

if "$PATCH" "${PATCH_OPTS[@]}" --dry-run --forward < "$PATCH_FILE" >/dev/null; then
  if [[ "$DRY_RUN" -eq 1 ]]; then
    echo "Check OK: patch can be applied to $TARGET_FILE"
    exit 0
  fi

  "$PATCH" "${PATCH_OPTS[@]}" --forward < "$PATCH_FILE" >/dev/null
  echo "Applied patch to: $TARGET_FILE"
  exit 0
fi

if "$PATCH" "${PATCH_OPTS[@]}" --dry-run -R < "$PATCH_FILE" >/dev/null; then
  echo "Patch already applied in: $TARGET_FILE"
  exit 0
fi

echo "Patch check failed. Target file differs from expected baseline and patch is not applied." >&2
exit 1
