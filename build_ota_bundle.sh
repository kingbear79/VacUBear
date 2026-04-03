#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${SCRIPT_DIR}"
PLATFORMIO_INI="${PROJECT_DIR}/platformio.ini"
PLATFORMIO_INI_REL="platformio.ini"

if [[ ! -f "${PLATFORMIO_INI}" ]]; then
  echo "Fehler: ${PLATFORMIO_INI} nicht gefunden."
  exit 1
fi

if ! command -v pio >/dev/null 2>&1; then
  echo "Fehler: 'pio' ist nicht im PATH."
  exit 1
fi

extract_define_value_from_content() {
  local content macro_name line
  content="${1}"
  macro_name="${2}"
  line="$(printf '%s\n' "${content}" | grep -E "^[[:space:]]*-D${macro_name}=" | head -n1 || true)"
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

extract_build_flag_value() {
  local macro_name value line
  macro_name="${1}"

  if value="$(python3 - "${PROJECT_DIR}" "${PIO_ENV}" "${macro_name}" <<'PY'
import json
import subprocess
import sys

project_dir = sys.argv[1]
env_name = sys.argv[2]
macro_name = sys.argv[3]

try:
    output = subprocess.check_output(
        ["pio", "project", "config", "--project-dir", project_dir, "--json-output"],
        text=True,
    )
    config = json.loads(output)
except Exception:
    sys.exit(1)

target_section = f"env:{env_name}"
needle = f"-D{macro_name}="
for section_name, entries in config:
    if section_name != target_section:
        continue
    for key, value in entries:
        if key != "build_flags":
            continue
        flags = value if isinstance(value, list) else str(value).splitlines()
        for flag in flags:
            if not isinstance(flag, str):
                continue
            if flag.startswith(needle):
                resolved = flag.split("=", 1)[1].replace('\\"', '"').strip('"')
                print(resolved)
                sys.exit(0)
    break

sys.exit(1)
PY
)"; then
    if [[ -n "${value}" ]]; then
      printf '%s' "${value}"
      return 0
    fi
  fi

  line="$(grep -E "^[[:space:]]*-D${macro_name}=" "${PLATFORMIO_INI}" | head -n1 || true)"
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

extract_fw_version() {
  if ! extract_build_flag_value "FW_VERSION"; then
    echo "Fehler: FW_VERSION konnte in platformio.ini nicht gefunden werden."
    exit 1
  fi
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
  extract_build_flag_value "OTA_MANIFEST_URL"
}

extract_fw_version_at_ref() {
  local ref content
  content="$(git -C "${PROJECT_DIR}" show "${ref}:${PLATFORMIO_INI_REL}" 2>/dev/null || true)"
  if [[ -z "${content}" ]]; then
    return 1
  fi
  extract_define_value_from_content "${content}" "FW_VERSION"
}

find_release_notes_file() {
  local candidate

  if [[ -n "${OTA_NOTES_FILE:-}" ]]; then
    if [[ -f "${OTA_NOTES_FILE}" ]]; then
      printf '%s' "${OTA_NOTES_FILE}"
      return 0
    fi
    echo "Fehler: OTA_NOTES_FILE nicht gefunden: ${OTA_NOTES_FILE}"
    exit 1
  fi

  for candidate in \
    "${PROJECT_DIR}/release_notes/${FW_VERSION}.md" \
    "${PROJECT_DIR}/release_notes/${FW_VERSION}.txt" \
    "${PROJECT_DIR}/release-notes/${FW_VERSION}.md" \
    "${PROJECT_DIR}/release-notes/${FW_VERSION}.txt"; do
    if [[ -f "${candidate}" ]]; then
      printf '%s' "${candidate}"
      return 0
    fi
  done

  return 1
}

generate_release_notes_from_git() {
  local current_version head_version target_version version_regex base_commit parent range notes

  current_version="${FW_VERSION}"
  head_version="$(extract_fw_version_at_ref HEAD || true)"

  if [[ -n "${head_version}" && "${head_version}" != "${current_version}" ]]; then
    target_version="${head_version}"
  else
    target_version="${current_version}"
  fi

  version_regex="$(printf '%s' "${target_version}" | sed 's/[][(){}.^$+*?|\\/]/\\&/g')"
  base_commit="$(git -C "${PROJECT_DIR}" log \
    -G "FW_VERSION=.*${version_regex}" \
    --format=%H \
    -- "${PLATFORMIO_INI_REL}" | head -n1 || true)"

  if [[ -z "${target_version}" || -z "${base_commit}" ]]; then
    return 1
  fi

  if git -C "${PROJECT_DIR}" rev-parse "${base_commit}^" >/dev/null 2>&1; then
    parent="$(git -C "${PROJECT_DIR}" rev-parse "${base_commit}^")"
    range="${parent}..HEAD"
  else
    range="${base_commit}"
  fi

  notes="$(git -C "${PROJECT_DIR}" log \
    --reverse \
    --no-merges \
    --format='- %s' \
    "${range}" | grep -Ev '^- (Refresh OTA manifest metadata|neuer Build|Bump version to .+|Release .+|neues Manifest|Update manifest\.json)$' || true)"

  if [[ -z "${notes}" ]]; then
    printf 'Aenderungen seit Version %s:\n- Keine zusaetzlichen Commits seit dem letzten Versionssprung.' "${target_version}"
    return 0
  fi

  printf 'Aenderungen seit Version %s:\n%s' "${target_version}" "${notes}"
}

json_escape() {
  python3 -c 'import json, sys; print(json.dumps(sys.stdin.read()))'
}

FW_VERSION="$(extract_fw_version)"
PIO_ENV="${PIO_ENV:-$(extract_default_env)}"
POST_BUILD_MODE="${VACUBEAR_POST_BUILD:-0}"

if [[ -n "${OTA_NOTES:-}" ]]; then
  NOTES="${OTA_NOTES}"
elif NOTES_FILE="$(find_release_notes_file)"; then
  NOTES="$(cat "${NOTES_FILE}")"
else
  if NOTES="$(generate_release_notes_from_git)"; then
    :
  elif GIT_COMMIT_HASH="$(git -C "${PROJECT_DIR}" rev-parse --short=12 HEAD 2>/dev/null)"; then
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

NOTES_JSON="$(printf '%s' "${NOTES}" | json_escape)"

cat > "${OUT_DIR}/manifest.json" <<EOF
{
  "version": "${FW_VERSION}",
  "firmware_url": "${FIRMWARE_URL}",
  "notes": ${NOTES_JSON}
}
EOF

echo
echo "OTA-Bundle erstellt:"
echo "  ${OUT_DIR}/firmware.bin"
echo "  ${OUT_DIR}/manifest.json"
