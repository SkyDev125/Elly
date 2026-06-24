#!/usr/bin/env bash
set -euo pipefail

# Validate and deploy the Elly controller to an already configured Jetson.
# Usage: scripts/deploy.sh [ROBOT_IP]

readonly REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NANO_IP="${NANO_IP:-192.168.0.200}"
NANO_USER="${NANO_USER:-er}"

if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then
  readonly RESET=$'\033[0m'
  readonly BOLD=$'\033[1m'
  readonly CYAN=$'\033[38;5;45m'
  readonly GREEN=$'\033[38;5;42m'
  readonly RED=$'\033[38;5;196m'
else
  readonly RESET=''
  readonly BOLD=''
  readonly CYAN=''
  readonly GREEN=''
  readonly RED=''
fi

info() {
  printf '  %s->%s %s\n' "$CYAN" "$RESET" "$*"
}

ok() {
  printf '  %s[ok]%s %s\n' "$GREEN$BOLD" "$RESET" "$*"
}

die() {
  printf '\n  %s[x] Deployment stopped%s\n' "$RED$BOLD" "$RESET" >&2
  printf '  %s\n\n' "$*" >&2
  exit 1
}

usage() {
  cat <<EOF
Usage: $(basename "$0") [ROBOT_IP]

Validate and deploy the Elly controller to the Jetson.

Arguments:
  ROBOT_IP     Jetson address; defaults to 192.168.0.200
  -h, --help   Show this help
EOF
}

case "${1:-}" in
  -h | --help)
    usage
    exit 0
    ;;
  '')
    ;;
  -*)
    die "Unknown option: $1"
    ;;
  *)
    NANO_IP="$1"
    ;;
esac

[[ $# -le 1 ]] || die "Only one robot IP may be supplied."
REMOTE="$NANO_USER@$NANO_IP"

printf '\n%s%s' "$BOLD" "$CYAN"
cat <<'EOF'
  +--------------------------------------------------------+
  |                 ELLY OS DEPLOYMENT                     |
  +--------------------------------------------------------+
EOF
printf '%s' "$RESET"
printf '  %-16s %s\n\n' 'Target' "$REMOTE"

info "Validating local controller files."
bash -n "$REPO_DIR/scripts/brain.sh" || die "brain.sh contains invalid shell syntax."
python3 -m py_compile "$REPO_DIR/scripts/brain.py" || die "brain.py contains invalid Python syntax."
(
  cd "$REPO_DIR"
  python3 -m unittest discover -s tests >/dev/null 2>&1
) || die "Local tests failed; the controller was not uploaded."
ok "Local validation passed."

info "Checking the Jetson connection."
ssh -o BatchMode=yes -o ConnectTimeout=5 "$REMOTE" true ||
  die "Cannot connect to $REMOTE without a password. Run setup.sh first."

info "Uploading brain.sh and brain.py."
ssh "$REMOTE" "mkdir -p ~/scripts" || die "Could not create ~/scripts on the Jetson."
scp "$REPO_DIR/scripts/brain.sh" "$REPO_DIR/scripts/brain.py" "$REMOTE:~/scripts/" ||
  die "Could not copy controller files to the Jetson."

info "Validating and restarting the remote movement service."
if ! ssh "$REMOTE" "
  chmod +x ~/scripts/brain.sh ~/scripts/brain.py
  bash -n ~/scripts/brain.sh
  python3 -m py_compile ~/scripts/brain.py
  bash ~/scripts/brain.sh motion_off
  bash ~/scripts/brain.sh motion_on
"; then
  printf '\n  Remote startup log:\n' >&2
  ssh "$REMOTE" "tail -n 40 ~/scripts/motion_service.log 2>/dev/null || true" >&2 || true
  die "The files were uploaded, but the movement service did not become ready."
fi

ok "Controller deployed and movement service is ready."
printf '\n'
