#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${SCRIPT_DIR}"
LOCAL_ENV_FILE="${PROJECT_DIR}/.ota-deploy.env"
HTML_TEMPLATE_DIR="${PROJECT_DIR}/html"
MANIFEST_PATH="${PROJECT_DIR}/dist/ota/latest/manifest.json"
FIRMWARE_PATH="${PROJECT_DIR}/dist/ota/latest/firmware.bin"

if [[ -f "${LOCAL_ENV_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${LOCAL_ENV_FILE}"
fi

if [[ ! -f "${MANIFEST_PATH}" || ! -f "${FIRMWARE_PATH}" ]]; then
  echo "OTA-Bundle fehlt. Erzeuge es zuerst mit:"
  echo "  pio run -e esp12f"
  exit 1
fi

if [[ ! -f "${HTML_TEMPLATE_DIR}/index.html" || ! -f "${HTML_TEMPLATE_DIR}/logo.svg" ]]; then
  echo "Fehlende HTML-Templates in ${HTML_TEMPLATE_DIR}."
  echo "Benötigt: index.html und logo.svg"
  exit 1
fi

if [[ -z "${OTA_DEPLOY_HOST:-}" || -z "${OTA_DEPLOY_USER:-}" || -z "${OTA_DEPLOY_BASE:-}" ]]; then
  echo "Fehlende Deploy-Konfiguration."
  echo "Benötigt: OTA_DEPLOY_HOST, OTA_DEPLOY_USER, OTA_DEPLOY_BASE"
  echo "Optional: OTA_DEPLOY_PORT, OTA_DEPLOY_BRANCH, OTA_DEPLOY_SSH_KEY"
  echo "Diese Werte koennen als Umgebungsvariablen oder in .ota-deploy.env gesetzt werden."
  exit 1
fi

OTA_PORT="${OTA_DEPLOY_PORT:-22}"
OTA_BRANCH="${OTA_DEPLOY_BRANCH:-$(git -C "${PROJECT_DIR}" rev-parse --abbrev-ref HEAD 2>/dev/null || echo latest)}"
REMOTE_DIR="${OTA_DEPLOY_BASE%/}/${OTA_BRANCH}"
REMOTE_ROOT_DIR="${OTA_DEPLOY_BASE%/}"
REMOTE_MASTER_DIR="${OTA_DEPLOY_BASE%/}/master"
DEFAULT_OTA_KEY="${HOME}/.ssh/id_ed25519_ota"
TMP_HTML_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${TMP_HTML_DIR}"
}

trap cleanup EXIT

SSH_ARGS=(-p "${OTA_PORT}")
RSYNC_RSH="ssh -p ${OTA_PORT}"

if [[ -z "${OTA_DEPLOY_SSH_KEY:-}" && -f "${DEFAULT_OTA_KEY}" ]]; then
  OTA_DEPLOY_SSH_KEY="${DEFAULT_OTA_KEY}"
fi

if [[ -n "${OTA_DEPLOY_SSH_KEY:-}" ]]; then
  SSH_ARGS+=(-i "${OTA_DEPLOY_SSH_KEY}")
  RSYNC_RSH+=" -i ${OTA_DEPLOY_SSH_KEY}"
fi

python3 - "${MANIFEST_PATH}" "${HTML_TEMPLATE_DIR}/index.html" "${TMP_HTML_DIR}/index.html" <<'PY'
import html
import json
import pathlib
import sys

manifest_path = pathlib.Path(sys.argv[1])
template_path = pathlib.Path(sys.argv[2])
output_path = pathlib.Path(sys.argv[3])

manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
template = template_path.read_text(encoding="utf-8")

def htmlize(value: str) -> str:
    escaped = html.escape(value, quote=True)
    escaped = escaped.replace("\r\n", "\n").replace("\r", "\n")
    escaped = escaped.replace("\n", "<br>\n")
    escaped = escaped.replace("\t", "&#9;")
    result = []
    for char in escaped:
        code = ord(char)
        if char in "\n" or char == "<" or char == ">" or char == "&":
            result.append(char)
        elif 32 <= code <= 126:
            result.append(char)
        elif code >= 160:
            result.append(char)
        else:
            result.append(f"&#{code};")
    return "".join(result)

rendered = template.replace("{version}", htmlize(str(manifest.get("version", ""))))
rendered = rendered.replace("{notes}", htmlize(str(manifest.get("notes", ""))))
output_path.write_text(rendered, encoding="utf-8")
PY

cp "${HTML_TEMPLATE_DIR}/logo.svg" "${TMP_HTML_DIR}/logo.svg"

echo "Deploye OTA-Bundle nach:"
echo "  Host: ${OTA_DEPLOY_USER}@${OTA_DEPLOY_HOST}:${REMOTE_DIR}"
echo "  Branch-Ziel: ${OTA_BRANCH}"
if [[ -n "${OTA_DEPLOY_SSH_KEY:-}" ]]; then
  echo "  SSH-Key: ${OTA_DEPLOY_SSH_KEY}"
fi
echo "  Manifest: http://ota.kinkbear.de/${OTA_BRANCH}/manifest.json"
echo "  Firmware: http://ota.kinkbear.de/${OTA_BRANCH}/firmware.bin"
echo "  Landing Page: http://ota.kinkbear.de/"
echo "  Master Landing Page: http://ota.kinkbear.de/master/"

ssh "${SSH_ARGS[@]}" "${OTA_DEPLOY_USER}@${OTA_DEPLOY_HOST}" "mkdir -p '${REMOTE_DIR}' '${REMOTE_ROOT_DIR}' '${REMOTE_MASTER_DIR}'"
rsync -avz --delete -e "${RSYNC_RSH}" "${PROJECT_DIR}/dist/ota/latest/" "${OTA_DEPLOY_USER}@${OTA_DEPLOY_HOST}:${REMOTE_DIR}/"
rsync -avz -e "${RSYNC_RSH}" "${TMP_HTML_DIR}/index.html" "${TMP_HTML_DIR}/logo.svg" "${OTA_DEPLOY_USER}@${OTA_DEPLOY_HOST}:${REMOTE_ROOT_DIR}/"
rsync -avz -e "${RSYNC_RSH}" "${TMP_HTML_DIR}/index.html" "${TMP_HTML_DIR}/logo.svg" "${OTA_DEPLOY_USER}@${OTA_DEPLOY_HOST}:${REMOTE_MASTER_DIR}/"

echo
echo "OTA-Deployment abgeschlossen."
