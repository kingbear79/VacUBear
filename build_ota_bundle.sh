#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${SCRIPT_DIR}"
PLATFORMIO_INI="${PROJECT_DIR}/platformio.ini"

if [[ ! -f "${PLATFORMIO_INI}" ]]; then
  echo "Fehler: ${PLATFORMIO_INI} nicht gefunden."
  exit 1
fi

if ! command -v pio >/dev/null 2>&1; then
  echo "Fehler: 'pio' ist nicht im PATH."
  exit 1
fi

extract_fw_version() {
  local line
  line="$(grep -E '^[[:space:]]*-DFW_VERSION=' "${PLATFORMIO_INI}" | head -n1 || true)"
  if [[ -z "${line}" ]]; then
    echo "Fehler: FW_VERSION konnte in platformio.ini nicht gefunden werden."
    exit 1
  fi
  line="${line#*=}"
  line="${line//\\\"/}"
  line="${line//\"/}"
  line="$(echo "${line}" | xargs)"
  if [[ -z "${line}" ]]; then
    echo "Fehler: FW_VERSION ist leer."
    exit 1
  fi
  printf '%s' "${line}"
}

extract_default_env() {
  local env_line env_name
  env_line="$(grep -E '^\[env:[^]]+\]' "${PLATFORMIO_INI}" | head -n1 || true)"
  if [[ -z "${env_line}" ]]; then
    echo "Fehler: Keine [env:...] Sektion in platformio.ini gefunden."
    exit 1
  fi
  env_name="${env_line#\[env:}"
  env_name="${env_name%]}"
  printf '%s' "${env_name}"
}

extract_repo_slug() {
  local remote slug
  remote="$(git -C "${PROJECT_DIR}" remote get-url origin 2>/dev/null || true)"
  if [[ "${remote}" =~ github\.com[:/]([^/]+/[^/.]+)(\.git)?$ ]]; then
    slug="${BASH_REMATCH[1]}"
    printf '%s' "${slug}"
    return 0
  fi
  return 1
}

extract_ota_manifest_url() {
  local line
  line="$(grep -E '^[[:space:]]*-DOTA_MANIFEST_URL=' "${PLATFORMIO_INI}" | head -n1 || true)"
  if [[ -z "${line}" ]]; then
    return 1
  fi
  line="${line#*=}"
  line="${line//\\\"/}"
  line="${line//\"/}"
  line="$(echo "${line}" | xargs)"
  if [[ -z "${line}" ]]; then
    return 1
  fi
  printf '%s' "${line}"
}

FW_VERSION="$(extract_fw_version)"
PIO_ENV="${PIO_ENV:-$(extract_default_env)}"
POST_BUILD_MODE="${VACUBEAR_POST_BUILD:-0}"

if [[ -n "${OTA_NOTES:-}" ]]; then
  NOTES="${OTA_NOTES}"
else
  if GIT_COMMIT_HASH="$(git -C "${PROJECT_DIR}" rev-parse --short=12 HEAD 2>/dev/null)"; then
    NOTES="Commit ${GIT_COMMIT_HASH}"
  else
    NOTES="Commit unbekannt"
  fi
fi

if [[ -n "${OTA_FIRMWARE_URL:-}" ]]; then
  FIRMWARE_URL="${OTA_FIRMWARE_URL}"
else
  if OTA_MANIFEST_URL_VALUE="$(extract_ota_manifest_url)"; then
    FIRMWARE_URL="${OTA_MANIFEST_URL_VALUE%manifest.json}firmware.bin"
  elif REPO_SLUG="$(extract_repo_slug)"; then
    FIRMWARE_URL="https://github.com/${REPO_SLUG}/releases/latest/download/firmware.bin"
  else
    echo "Fehler: OTA_FIRMWARE_URL nicht gesetzt und GitHub-Remote konnte nicht erkannt werden."
    echo "Setze OTA_FIRMWARE_URL, z.B.:"
    echo "  OTA_FIRMWARE_URL=https://github.com/<user>/<repo>/releases/latest/download/firmware.bin ${0}"
    exit 1
  fi
fi

BUILD_DIR="${PROJECT_DIR}/.pio/build/${PIO_ENV}"
FW_BIN="${FW_BIN_PATH:-${BUILD_DIR}/firmware.bin}"
OUT_DIR="${PROJECT_DIR}/dist/ota/latest"

echo "Projekt: ${PROJECT_DIR}"
echo "PIO_ENV: ${PIO_ENV}"
echo "FW_VERSION: ${FW_VERSION}"
echo "Firmware URL: ${FIRMWARE_URL}"
echo "Output: ${OUT_DIR}"

if [[ "${POST_BUILD_MODE}" != "1" ]]; then
  pio run -d "${PROJECT_DIR}" -e "${PIO_ENV}"
else
  echo "Post-Build-Modus aktiv: pio run wird uebersprungen."
fi

if [[ ! -f "${FW_BIN}" ]]; then
  echo "Fehler: Firmware nicht gefunden: ${FW_BIN}"
  exit 1
fi

mkdir -p "${OUT_DIR}"
cp "${FW_BIN}" "${OUT_DIR}/firmware.bin"

cat > "${OUT_DIR}/manifest.json" <<EOF
{
  "version": "${FW_VERSION}",
  "firmware_url": "${FIRMWARE_URL}",
  "notes": "${NOTES}"
}
EOF

echo
echo "OTA-Bundle erstellt:"
echo "  ${OUT_DIR}/firmware.bin"
echo "  ${OUT_DIR}/manifest.json"
