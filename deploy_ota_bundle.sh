#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${SCRIPT_DIR}"
LOCAL_ENV_FILE="${PROJECT_DIR}/.ota-deploy.env"

if [[ -f "${LOCAL_ENV_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${LOCAL_ENV_FILE}"
fi

if [[ ! -f "${PROJECT_DIR}/dist/ota/latest/manifest.json" || ! -f "${PROJECT_DIR}/dist/ota/latest/firmware.bin" ]]; then
  echo "OTA-Bundle fehlt. Erzeuge es zuerst mit:"
  echo "  pio run -e esp12f"
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
DEFAULT_OTA_KEY="${HOME}/.ssh/id_ed25519_ota"

SSH_ARGS=(-p "${OTA_PORT}")
RSYNC_RSH="ssh -p ${OTA_PORT}"

if [[ -z "${OTA_DEPLOY_SSH_KEY:-}" && -f "${DEFAULT_OTA_KEY}" ]]; then
  OTA_DEPLOY_SSH_KEY="${DEFAULT_OTA_KEY}"
fi

if [[ -n "${OTA_DEPLOY_SSH_KEY:-}" ]]; then
  SSH_ARGS+=(-i "${OTA_DEPLOY_SSH_KEY}")
  RSYNC_RSH+=" -i ${OTA_DEPLOY_SSH_KEY}"
fi

echo "Deploye OTA-Bundle nach:"
echo "  Host: ${OTA_DEPLOY_USER}@${OTA_DEPLOY_HOST}:${REMOTE_DIR}"
echo "  Branch-Ziel: ${OTA_BRANCH}"
if [[ -n "${OTA_DEPLOY_SSH_KEY:-}" ]]; then
  echo "  SSH-Key: ${OTA_DEPLOY_SSH_KEY}"
fi
echo "  Manifest: http://ota.kinkbear.de/${OTA_BRANCH}/manifest.json"
echo "  Firmware: http://ota.kinkbear.de/${OTA_BRANCH}/firmware.bin"

ssh "${SSH_ARGS[@]}" "${OTA_DEPLOY_USER}@${OTA_DEPLOY_HOST}" "mkdir -p '${REMOTE_DIR}'"
rsync -avz --delete -e "${RSYNC_RSH}" "${PROJECT_DIR}/dist/ota/latest/" "${OTA_DEPLOY_USER}@${OTA_DEPLOY_HOST}:${REMOTE_DIR}/"

echo
echo "OTA-Deployment abgeschlossen."
