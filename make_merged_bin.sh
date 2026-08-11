#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${PROJECT_DIR}/build"
OUT_BIN="${BUILD_DIR}/merged-flash.bin"
SKIP_BUILD=0
SAVE_ORG_BIN=0

usage() {
  cat <<'EOF'
Usage: ./make_merged_bin.sh [options]

Builds ESP-IDF firmware (optional) and generates a merged binary for web flashing.

Options:
  -s, --skip-build           Skip running idf.py build
  -b, --build-dir <path>     Build directory (default: ./build)
  -o, --output <path>        Output merged binary path (default: ./build/merged-flash.bin)
  --save-org-bin         Save original app image as *_org.bin before merge
  -h, --help                 Show this help message

Example:
  ./make_merged_bin.sh
  ./make_merged_bin.sh --skip-build --output build/web-flash.bin
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -s|--skip-build)
      SKIP_BUILD=1
      shift
      ;;
    -b|--build-dir)
      BUILD_DIR="$2"
      shift 2
      ;;
    -o|--output)
      OUT_BIN="$2"
      shift 2
      ;;
    --save-org-bin)
      SAVE_ORG_BIN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
done

FLASHER_JSON="${BUILD_DIR}/flasher_args.json"

if [[ ${SKIP_BUILD} -eq 0 ]]; then
  if ! command -v idf.py >/dev/null 2>&1; then
    echo "Error: idf.py not found. Activate ESP-IDF environment first." >&2
    exit 1
  fi
  echo "==> Building project"
  idf.py -C "${PROJECT_DIR}" build
fi

if [[ ! -f "${FLASHER_JSON}" ]]; then
  echo "Error: ${FLASHER_JSON} not found. Run a build first." >&2
  exit 1
fi

declare -a MERGE_ARGS
declare -a WRITE_FLASH_ARGS
CHIP=""
APP_BIN=""

while IFS='|' read -r tag field1 field2; do
  case "${tag}" in
    CHIP)
      CHIP="${field1}"
      ;;
    WFA)
      WRITE_FLASH_ARGS+=("${field1}")
      ;;
    FILE)
      MERGE_ARGS+=("${field1}" "${field2}")
      ;;
    APP)
      APP_BIN="${field1}"
      ;;
  esac
done < <(
  python3 - "${FLASHER_JSON}" "${BUILD_DIR}" <<'PY'
import json
import os
import sys

flasher_json = sys.argv[1]
build_dir = sys.argv[2]

with open(flasher_json, "r", encoding="utf-8") as f:
    data = json.load(f)

chip = data.get("extra_esptool_args", {}).get("chip", "esp32")
print(f"CHIP|{chip}|")

for arg in data.get("write_flash_args", []):
    print(f"WFA|{arg}|")

app_file = data.get("app", {}).get("file")
if app_file:
    print(f"APP|{os.path.join(build_dir, app_file)}|")

flash_files = data.get("flash_files", {})
for offset in sorted(flash_files.keys(), key=lambda x: int(x, 0)):
    file_path = os.path.join(build_dir, flash_files[offset])
    print(f"FILE|{offset}|{file_path}")
PY
)

if [[ -z "${CHIP}" ]]; then
  echo "Error: failed to parse chip type from ${FLASHER_JSON}" >&2
  exit 1
fi

if [[ ${#MERGE_ARGS[@]} -eq 0 ]]; then
  echo "Error: no flash files found in ${FLASHER_JSON}" >&2
  exit 1
fi

for ((i=1; i<${#MERGE_ARGS[@]}; i+=2)); do
  if [[ ! -f "${MERGE_ARGS[i]}" ]]; then
    echo "Error: missing flash segment file: ${MERGE_ARGS[i]}" >&2
    exit 1
  fi
done

if [[ ${SAVE_ORG_BIN} -eq 1 ]]; then
  if [[ -z "${APP_BIN}" ]]; then
    echo "Error: failed to locate app image path in ${FLASHER_JSON}" >&2
    exit 1
  fi
  if [[ ! -f "${APP_BIN}" ]]; then
    echo "Error: app image not found: ${APP_BIN}" >&2
    exit 1
  fi
  APP_ORG_BIN="${APP_BIN%.bin}_org.bin"
  cp "${APP_BIN}" "${APP_ORG_BIN}"
  echo "==> Saved original app image: ${APP_ORG_BIN}"
fi

mkdir -p "$(dirname "${OUT_BIN}")"

OUT_BIN_ABS="$(python3 - <<'PY' "${OUT_BIN}"
import os
import sys
print(os.path.realpath(sys.argv[1]))
PY
)"

for ((i=1; i<${#MERGE_ARGS[@]}; i+=2)); do
  IN_BIN_ABS="$(python3 - <<'PY' "${MERGE_ARGS[i]}"
import os
import sys
print(os.path.realpath(sys.argv[1]))
PY
)"
  if [[ "${IN_BIN_ABS}" == "${OUT_BIN_ABS}" ]]; then
    echo "Error: output path collides with input segment file: ${MERGE_ARGS[i]}" >&2
    echo "       Choose a different --output path (e.g. build/edge-device-esp32_merged.bin)." >&2
    exit 1
  fi
done

if command -v esptool.py >/dev/null 2>&1; then
  ESPTOOL_CMD=(esptool.py)
elif command -v python3 >/dev/null 2>&1; then
  if python3 -c "import esptool" >/dev/null 2>&1; then
    ESPTOOL_CMD=(python3 -m esptool)
  elif [[ -n "${IDF_PATH:-}" && -f "${IDF_PATH}/components/esptool_py/esptool/esptool.py" ]]; then
    ESPTOOL_CMD=(python3 "${IDF_PATH}/components/esptool_py/esptool/esptool.py")
  else
    echo "Error: esptool is unavailable. Install it in your Python env or export IDF_PATH." >&2
    exit 1
  fi
else
  echo "Error: python3 is required to run esptool." >&2
  exit 1
fi

echo "==> Generating merged image"
echo "    Chip: ${CHIP}"
echo "    Output: ${OUT_BIN}"

"${ESPTOOL_CMD[@]}" --chip "${CHIP}" merge_bin \
  "${WRITE_FLASH_ARGS[@]}" \
  -o "${OUT_BIN}" \
  "${MERGE_ARGS[@]}"

echo "==> Done: ${OUT_BIN}"
ls -lh "${OUT_BIN}"
