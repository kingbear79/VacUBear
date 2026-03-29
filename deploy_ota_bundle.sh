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
  echo "Optional: OTA_DEPLOY_PORT, OTA_DEPLOY_PROJECT, OTA_DEPLOY_SSH_KEY"
  echo "Diese Werte koennen als Umgebungsvariablen oder in .ota-deploy.env gesetzt werden."
  exit 1
fi

OTA_PORT="${OTA_DEPLOY_PORT:-22}"
DEFAULT_OTA_KEY="${HOME}/.ssh/id_ed25519_ota"
TMP_HTML_DIR="$(mktemp -d)"
MANIFEST_META_PATH="${TMP_HTML_DIR}/manifest_meta.json"

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

python3 - "${MANIFEST_PATH}" "${HTML_TEMPLATE_DIR}/index.html" "${TMP_HTML_DIR}/index.html" "${MANIFEST_META_PATH}" "${OTA_DEPLOY_PROJECT:-}" <<'PY'
import html
import json
import pathlib
import sys
from urllib.parse import urlparse

manifest_path = pathlib.Path(sys.argv[1])
template_path = pathlib.Path(sys.argv[2])
output_path = pathlib.Path(sys.argv[3])
meta_path = pathlib.Path(sys.argv[4])
override_project = sys.argv[5].strip()

manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
template = template_path.read_text(encoding="utf-8")

firmware_url = str(manifest.get("firmware_url", "")).strip()
parsed = urlparse(firmware_url)
path = parsed.path or "/"
path_parts = [part for part in path.split("/") if part]

project_id = override_project
if not project_id:
    if len(path_parts) >= 2:
        project_id = path_parts[-2]
    elif path_parts:
        project_id = path_parts[0]
    else:
        project_id = "vacubear"

if parsed.scheme and parsed.netloc:
    public_base_url = f"{parsed.scheme}://{parsed.netloc}/{project_id}"
else:
    public_base_url = f"http://ota.kinkbear.de/{project_id}"

known_names = {
    "vacubear": "VacUBear",
    "squeezebear": "SqueezeBear",
}
product_name = known_names.get(project_id, project_id.replace("-", " ").replace("_", " ").title())

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
rendered = rendered.replace("{product_name}", htmlize(product_name))
rendered = rendered.replace("{firmware_url}", htmlize(firmware_url))
output_path.write_text(rendered, encoding="utf-8")
meta_path.write_text(
    json.dumps(
        {
            "project_id": project_id,
            "public_base_url": public_base_url,
            "product_name": product_name,
        }
    ),
    encoding="utf-8",
)
PY

OTA_DEPLOY_PROJECT="$(python3 - "${MANIFEST_META_PATH}" <<'PY'
import json
import pathlib
import sys
meta = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
print(meta["project_id"])
PY
)"

PUBLIC_BASE_URL="$(python3 - "${MANIFEST_META_PATH}" <<'PY'
import json
import pathlib
import sys
meta = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
print(meta["public_base_url"])
PY
)"

PRODUCT_NAME="$(python3 - "${MANIFEST_META_PATH}" <<'PY'
import json
import pathlib
import sys
meta = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
print(meta["product_name"])
PY
)"

REMOTE_DIR="${OTA_DEPLOY_BASE%/}/${OTA_DEPLOY_PROJECT}"

cp "${HTML_TEMPLATE_DIR}/logo.svg" "${TMP_HTML_DIR}/logo.svg"

echo "Deploye OTA-Bundle nach:"
echo "  Host: ${OTA_DEPLOY_USER}@${OTA_DEPLOY_HOST}:${REMOTE_DIR}"
echo "  Projekt-Ziel: ${OTA_DEPLOY_PROJECT}"
echo "  Produkt: ${PRODUCT_NAME}"
if [[ -n "${OTA_DEPLOY_SSH_KEY:-}" ]]; then
  echo "  SSH-Key: ${OTA_DEPLOY_SSH_KEY}"
fi
echo "  Manifest: ${PUBLIC_BASE_URL}/manifest.json"
echo "  Firmware: ${PUBLIC_BASE_URL}/firmware.bin"
echo "  Landing Page: ${PUBLIC_BASE_URL}/"

ssh "${SSH_ARGS[@]}" "${OTA_DEPLOY_USER}@${OTA_DEPLOY_HOST}" "mkdir -p '${REMOTE_DIR}'"
rsync -avz --delete -e "${RSYNC_RSH}" "${PROJECT_DIR}/dist/ota/latest/" "${OTA_DEPLOY_USER}@${OTA_DEPLOY_HOST}:${REMOTE_DIR}/"
rsync -avz -e "${RSYNC_RSH}" "${TMP_HTML_DIR}/index.html" "${TMP_HTML_DIR}/logo.svg" "${OTA_DEPLOY_USER}@${OTA_DEPLOY_HOST}:${REMOTE_DIR}/"

echo
echo "OTA-Deployment abgeschlossen."
