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
PATCH_DIR="$REPO_ROOT/patches"

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

shopt -s nullglob
PATCH_FILES=( "$PATCH_DIR"/*.patch )
if [[ ${#PATCH_FILES[@]} -eq 0 ]]; then
  echo "No patch files found in $PATCH_DIR" >&2
  exit 1
fi

if [[ -n "${AMBIQSUITE_ROOT:-}" ]]; then
  SUITE_ROOT="$AMBIQSUITE_ROOT"
else
  SUITE_ROOT="$(cd "$REPO_ROOT/../../../../.." && pwd)"
fi

# Prefer system patch over Anaconda's broken build
if [[ -x /usr/bin/patch ]]; then
  PATCH=/usr/bin/patch
else
  PATCH="$(command -v patch)"
fi

PATCH_OPTS=( -d "$SUITE_ROOT" -p0 )

apply_one() {
  local pf="$1"
  if "$PATCH" "${PATCH_OPTS[@]}" --dry-run --forward < "$pf" >/dev/null; then
    if [[ "$DRY_RUN" -eq 1 ]]; then
      echo "Check OK: $(basename "$pf") can be applied"
      return 0
    fi
    "$PATCH" "${PATCH_OPTS[@]}" --forward < "$pf" >/dev/null
    echo "Applied: $(basename "$pf")"
    return 0
  fi

  if "$PATCH" "${PATCH_OPTS[@]}" --dry-run -R < "$pf" >/dev/null; then
    echo "Already applied: $(basename "$pf")"
    return 0
  fi

  echo "Patch check failed for $(basename "$pf"): target differs from baseline and patch is not applied." >&2
  return 1
}

reverse_one() {
  local pf="$1"
  if ! "$PATCH" "${PATCH_OPTS[@]}" --dry-run -R < "$pf" >/dev/null; then
    echo "Reverse check failed for $(basename "$pf"): patch not applied or target differs." >&2
    return 1
  fi
  if [[ "$DRY_RUN" -eq 1 ]]; then
    echo "Reverse check OK: $(basename "$pf")"
    return 0
  fi
  "$PATCH" "${PATCH_OPTS[@]}" -R < "$pf" >/dev/null
  echo "Reverted: $(basename "$pf")"
}

rc=0
if [[ "$MODE" == "reverse" ]]; then
  # Reverse in opposite order to apply
  for ((i=${#PATCH_FILES[@]}-1; i>=0; i--)); do
    reverse_one "${PATCH_FILES[$i]}" || rc=1
  done
else
  for pf in "${PATCH_FILES[@]}"; do
    apply_one "$pf" || rc=1
  done
fi
exit $rc
