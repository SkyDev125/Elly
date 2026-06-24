#!/usr/bin/env bash
set -euo pipefail

# Complete laptop + Jetson setup for Elly.
# Usage: bash scripts/setup.sh [ROBOT_IP] [--no-shell]

readonly ELLY_ROS_DISTRO="galactic"
readonly REQUIRED_UBUNTU="20.04"
readonly REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly MYAGV_REPOSITORY="https://github.com/elephantrobotics/myagv_ros2.git"
readonly MYAGV_BRANCH="galactic-JN"
readonly MYAGV_REVISION="afb054bbb54423c09291622956e950062c193ec9"
readonly ORBBEC_REPOSITORY="https://github.com/orbbec/OrbbecSDK_ROS2.git"
readonly ORBBEC_BRANCH="v2-main"
readonly ORBBEC_REVISION="94fc83f2ac15d35b2c2d992802bdaa48289ce89a"

NANO_IP="${NANO_IP:-}"
NANO_USER="${NANO_USER:-er}"
OPEN_SHELL=1

HOST_BASE_PACKAGES=(
  ca-certificates
  curl
  gnupg2
  lsb-release
  openssh-client
  python3
)

HOST_ROS_PACKAGES=(
  python3-colcon-common-extensions
  python3-rosdep
  ros-galactic-desktop
  ros-galactic-navigation2
  ros-galactic-nav2-bringup
  ros-galactic-rtabmap-ros
  ros-galactic-teleop-twist-keyboard
)

ROBOT_PACKAGES=(
  git
  libdw-dev
  libgflags-dev
  libgl1
  libgoogle-glog-dev
  libssl-dev
  libusb-1.0-0-dev
  mesa-utils
  nlohmann-json3-dev
  python3-colcon-common-extensions
  python3-rosdep
  ros-galactic-backward-ros
  ros-galactic-behaviortree-cpp-v3
  ros-galactic-bondcpp
  ros-galactic-camera-info-manager
  ros-galactic-compressed-image-transport
  ros-galactic-diagnostic-msgs
  ros-galactic-diagnostic-updater
  ros-galactic-image-publisher
  ros-galactic-image-transport
  ros-galactic-image-transport-plugins
  ros-galactic-joint-state-publisher
  ros-galactic-ros-base
  ros-galactic-navigation2
  ros-galactic-nav2-bringup
  ros-galactic-ompl
  ros-galactic-rqt-tf-tree
  ros-galactic-rtabmap-ros
  ros-galactic-statistics-msgs
  ros-galactic-test-msgs
  ros-galactic-xacro
  screen
)

readonly TOTAL_STAGES=7
CURRENT_STAGE="Startup"
CURRENT_STAGE_NUMBER=0
STAGE_STARTED_AT=0
INSTALL_STARTED_AT=0

if [[ -t 1 && -z "${NO_COLOR:-}" ]]; then
  readonly RESET=$'\033[0m'
  readonly BOLD=$'\033[1m'
  readonly DIM=$'\033[2m'
  readonly BLUE=$'\033[38;5;39m'
  readonly CYAN=$'\033[38;5;45m'
  readonly GREEN=$'\033[38;5;42m'
  readonly YELLOW=$'\033[38;5;214m'
  readonly RED=$'\033[38;5;196m'
else
  readonly RESET=''
  readonly BOLD=''
  readonly DIM=''
  readonly BLUE=''
  readonly CYAN=''
  readonly GREEN=''
  readonly YELLOW=''
  readonly RED=''
fi

format_duration() {
  local seconds="$1"
  if ((seconds >= 60)); then
    printf '%dm %02ds' "$((seconds / 60))" "$((seconds % 60))"
  else
    printf '%ds' "$seconds"
  fi
}

banner() {
  printf '\n%s%s' "$BOLD" "$CYAN"
  cat <<'EOF'
  +--------------------------------------------------------+
  |                        ELLY OS                         |
  |              Robot Workstation Installer              |
  +--------------------------------------------------------+
EOF
  printf '%s' "$RESET"
  printf '  %-16s %s\n' 'Workspace' "$REPO_DIR"
  printf '  %-16s %s\n' 'Host target' "Ubuntu $REQUIRED_UBUNTU / ROS 2 Galactic"
  printf '  %-16s %s\n\n' 'Robot' "${NANO_USER}@${NANO_IP:-not selected}"
}

stage() {
  CURRENT_STAGE_NUMBER=$((CURRENT_STAGE_NUMBER + 1))
  CURRENT_STAGE="$1"
  STAGE_STARTED_AT=$SECONDS
  printf '\n%s[%d/%d] %s%s\n' \
    "$BOLD$BLUE" "$CURRENT_STAGE_NUMBER" "$TOTAL_STAGES" "$CURRENT_STAGE" "$RESET"
  printf '%s%s%s\n' "$DIM" '  --------------------------------------------------------' "$RESET"
}

stage_done() {
  local elapsed=$((SECONDS - STAGE_STARTED_AT))
  printf '  %s[done]%s %s %s(%s)%s\n' \
    "$GREEN$BOLD" "$RESET" "$CURRENT_STAGE" "$DIM" "$(format_duration "$elapsed")" "$RESET"
}

info() {
  printf '  %s->%s %s\n' "$CYAN" "$RESET" "$*"
}

ok() {
  printf '  %s[ok]%s %s\n' "$GREEN$BOLD" "$RESET" "$*"
}

warn() {
  printf '  %s[!]%s %s\n' "$YELLOW$BOLD" "$RESET" "$*"
}

die() {
  trap - ERR
  printf '\n  %s[x] Setup stopped%s\n' "$RED$BOLD" "$RESET" >&2
  printf '  %sStage:%s %s\n' "$BOLD" "$RESET" "$CURRENT_STAGE" >&2
  printf '  %sReason:%s %s\n\n' "$BOLD" "$RESET" "$*" >&2
  printf '  Nothing needs to be undone. Fix the issue and run setup again.\n\n' >&2
  exit 1
}

on_error() {
  local exit_code="$1"
  local line="$2"
  local command="$3"
  trap - ERR
  printf '\n  %s[x] Setup stopped%s\n' "$RED$BOLD" "$RESET" >&2
  printf '  %sStage:%s %s\n' "$BOLD" "$RESET" "$CURRENT_STAGE" >&2
  printf '  %sCommand:%s %s\n' "$BOLD" "$RESET" "$command" >&2
  printf '  %sLocation:%s setup.sh:%s\n' "$BOLD" "$RESET" "$line" >&2
  printf '  %sExit code:%s %s\n\n' "$BOLD" "$RESET" "$exit_code" >&2
  printf '  Review the command output above, fix the issue, and run setup again.\n\n' >&2
  exit "$exit_code"
}

trap 'on_error "$?" "$LINENO" "$BASH_COMMAND"' ERR

usage() {
  cat <<EOF
Usage: $(basename "$0") [ROBOT_IP] [--no-shell]

Set up the Elly OS laptop and Jetson environment.

Arguments:
  ROBOT_IP     Jetson address; prompts when omitted
  --no-shell   Finish without opening a configured interactive shell
  -h, --help   Show this help
EOF
}

parse_arguments() {
  local argument
  for argument in "$@"; do
    case "$argument" in
      --no-shell)
        OPEN_SHELL=0
        ;;
      -h | --help)
        usage
        exit 0
        ;;
      -*)
        die "Unknown option: $argument. Run $(basename "$0") --help for usage."
        ;;
      *)
        [[ -z "$NANO_IP" ]] ||
          die "Only one robot IP may be supplied. Received both $NANO_IP and $argument."
        NANO_IP="$argument"
        ;;
    esac
  done
}

install_missing_apt_packages() {
  local missing=()
  local package

  for package in "$@"; do
    if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q 'ok installed'; then
      missing+=("$package")
    fi
  done

  if ((${#missing[@]} == 0)); then
    ok "Required host packages are already installed."
    return
  fi

  info "Installing host packages: ${missing[*]}"
  sudo apt-get update ||
    die "Ubuntu could not refresh its package list. Check the internet connection and apt repository errors above."
  sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y "${missing[@]}" ||
    die "Ubuntu could not install: ${missing[*]}. Check the apt error above for an unavailable or conflicting package."
}

verify_host_os() {
  [[ -r /etc/os-release ]] || die "Cannot read /etc/os-release."
  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "$REQUIRED_UBUNTU" ]] ||
    die "Elly requires Ubuntu $REQUIRED_UBUNTU. Found ${PRETTY_NAME:-unknown OS}."
  ok "Ubuntu $REQUIRED_UBUNTU detected."
}

verify_admin_access() {
  info "Requesting administrator access for package installation."
  sudo -v ||
    die "Administrator access is required. Run setup from an account that can use sudo."
  ok "Administrator access confirmed."
}

configure_ros_repository() {
  local keyring="/usr/share/keyrings/ros-archive-keyring.gpg"
  local source_file="/etc/apt/sources.list.d/ros2.list"
  local source_line

  source_line="deb [arch=$(dpkg --print-architecture) signed-by=$keyring] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main"

  info "Configuring the ROS 2 apt repository."
  if ! curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key |
    sudo tee "$keyring" >/dev/null; then
    die "The ROS signing key could not be downloaded. Check internet access to github.com."
  fi

  if [[ ! -f "$source_file" ]] || ! grep -qxF "$source_line" "$source_file"; then
    printf '%s\n' "$source_line" | sudo tee "$source_file" >/dev/null ||
      die "The ROS apt source could not be written to $source_file."
  fi
  ok "ROS 2 package repository configured."
}

configure_rosdep() {
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    info "Initializing rosdep."
    sudo rosdep init ||
      die "rosdep initialization failed. Remove a broken rosdep source file or review the message above."
  fi
  rosdep update ||
    die "rosdep could not update its dependency index. Check the internet connection and rosdep output above."
  ok "rosdep dependency index is ready."
}

configure_ssh() {
  if [[ -z "$NANO_IP" ]]; then
    printf '  %s?%s Robot IP address %s[192.168.0.200]%s: ' \
      "$CYAN$BOLD" "$RESET" "$DIM" "$RESET"
    read -r NANO_IP
    NANO_IP="${NANO_IP:-192.168.0.200}"
  fi

  if [[ ! -f "$HOME/.ssh/id_ed25519" && ! -f "$HOME/.ssh/id_rsa" ]]; then
    info "Generating an SSH key."
    ssh-keygen -t ed25519 -N '' -f "$HOME/.ssh/id_ed25519"
  fi

  info "Authorizing this laptop on $NANO_USER@$NANO_IP."
  ssh-copy-id -o ConnectTimeout=5 "$NANO_USER@$NANO_IP" ||
    die "SSH authorization failed. Confirm the robot IP, Wi-Fi connection, username, and robot password."
  ssh -o BatchMode=yes -o ConnectTimeout=5 "$NANO_USER@$NANO_IP" true ||
    die "The robot answered, but passwordless SSH verification failed. Try ssh $NANO_USER@$NANO_IP manually."
  ok "SSH connection verified."
}

install_robot_packages() {
  local package_list="${ROBOT_PACKAGES[*]}"

  info "Configuring Jetson package repositories and dependencies."
  if ! ssh -t "$NANO_USER@$NANO_IP" "
    set -eo pipefail
    source /etc/os-release
    if [ \"\$ID\" != ubuntu ] || [ \"\$VERSION_ID\" != 20.04 ]; then
      echo \"[x] Jetson recovery requires Ubuntu 20.04; found \$PRETTY_NAME.\" >&2
      exit 1
    fi

    base_missing=''
    for package in ca-certificates curl gnupg2 lsb-release; do
      dpkg -s \"\$package\" >/dev/null 2>&1 || base_missing=\"\$base_missing \$package\"
    done
    if [ -n \"\$base_missing\" ]; then
      sudo apt-get update
      sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y \$base_missing
    fi

    architecture=\$(dpkg --print-architecture)
    codename=\$(lsb_release -cs)
    keyring=/usr/share/keyrings/ros-archive-keyring.gpg
    source_file=/etc/apt/sources.list.d/ros2.list
    source_line=\"deb [arch=\$architecture signed-by=\$keyring] http://packages.ros.org/ros2/ubuntu \$codename main\"
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee \"\$keyring\" >/dev/null
    echo \"\$source_line\" | sudo tee \"\$source_file\" >/dev/null

    missing=''
    for package in $package_list; do
      dpkg -s \"\$package\" >/dev/null 2>&1 || missing=\"\$missing \$package\"
    done
    if [ -n \"\$missing\" ]; then
      echo \"[i] Installing Jetson packages:\$missing\"
      sudo apt-get update
      sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y \$missing
    else
      echo '[ok] Required Jetson packages are already installed.'
    fi
  "; then
    die "Jetson package installation failed. Check its network access, ROS apt repository, and sudo password."
  fi
}

install_robot_workspaces() {
  info "Installing and validating required robot workspaces."
  if ! ssh -t "$NANO_USER@$NANO_IP" "
    set -eo pipefail
    set +u
    source /opt/ros/galactic/setup.bash
    set -u

    rosdep_ready=0
    prepare_rosdep() {
      if [ \"\$rosdep_ready\" -eq 1 ]; then
        return
      fi
      if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
      fi
      rosdep update
      rosdep_ready=1
    }

    build_myagv=0
    if [ ! -d \"\$HOME/myagv_ros2/src\" ]; then
      echo '[i] Cloning Elephant Robotics myagv_ros2 ($MYAGV_BRANCH).'
      mkdir -p \"\$HOME/myagv_ros2\"
      git clone --branch '$MYAGV_BRANCH' '$MYAGV_REPOSITORY' \"\$HOME/myagv_ros2/src\"
      git -C \"\$HOME/myagv_ros2/src\" checkout '$MYAGV_REVISION'
      build_myagv=1
    elif [ ! -d \"\$HOME/myagv_ros2/src/.git\" ]; then
      echo '[x] ~/myagv_ros2/src exists but is not a Git workspace.' >&2
      exit 1
    fi

    for package in myagv_odometry slam_gmapping ydlidar_ros2_driver; do
      [ -d \"\$HOME/myagv_ros2/install/\$package\" ] || build_myagv=1
    done

    if [ \"\$build_myagv\" -eq 1 ]; then
      echo '[i] Resolving and building the required myAGV packages.'
      prepare_rosdep
      cd \"\$HOME/myagv_ros2\"
      rosdep install --from-paths src --ignore-src -r -y
      colcon build --symlink-install
    else
      echo '[ok] Required myAGV packages are already built.'
    fi

    build_astra=0
    if [ ! -d \"\$HOME/ros2_astra_ws/src/OrbbecSDK_ROS2\" ]; then
      echo '[i] Cloning OrbbecSDK_ROS2 ($ORBBEC_BRANCH).'
      mkdir -p \"\$HOME/ros2_astra_ws/src\"
      git clone --branch '$ORBBEC_BRANCH' '$ORBBEC_REPOSITORY' \
        \"\$HOME/ros2_astra_ws/src/OrbbecSDK_ROS2\"
      git -C \"\$HOME/ros2_astra_ws/src/OrbbecSDK_ROS2\" checkout '$ORBBEC_REVISION'
      build_astra=1
    elif [ ! -d \"\$HOME/ros2_astra_ws/src/OrbbecSDK_ROS2/.git\" ]; then
      echo '[x] OrbbecSDK_ROS2 exists but is not a Git workspace.' >&2
      exit 1
    fi

    [ -d \"\$HOME/ros2_astra_ws/install/orbbec_camera\" ] || build_astra=1
    if [ \"\$build_astra\" -eq 1 ]; then
      echo '[i] Resolving and building the required Orbbec camera packages.'
      prepare_rosdep
      cd \"\$HOME/ros2_astra_ws\"
      rosdep install --from-paths src --ignore-src -r -y
      colcon build --symlink-install
    else
      echo '[ok] Required Orbbec camera package is already built.'
    fi

    echo '[i] Installing Orbbec USB permissions.'
    sudo bash \"\$HOME/ros2_astra_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh\"

    missing=0
    for path in \
      \"\$HOME/myagv_ros2/install/setup.bash\" \
      \"\$HOME/myagv_ros2/install/myagv_odometry\" \
      \"\$HOME/myagv_ros2/install/slam_gmapping\" \
      \"\$HOME/myagv_ros2/install/ydlidar_ros2_driver\" \
      \"\$HOME/ros2_astra_ws/install/setup.bash\" \
      \"\$HOME/ros2_astra_ws/install/orbbec_camera\" \
      /etc/udev/rules.d/99-obsensor-libusb.rules; do
      if [ ! -e \"\$path\" ]; then
        echo \"[x] Required robot component was not installed: \$path\" >&2
        missing=1
      fi
    done
    exit \"\$missing\"
  "; then
    die "A required robot workspace could not be installed. Review the Git, rosdep, or colcon error above."
  fi
  ok "Base, LiDAR, Gmapping, and camera packages are installed."
}

deploy_robot_controller() {
  NANO_IP="$NANO_IP" NANO_USER="$NANO_USER" \
    bash "$REPO_DIR/scripts/deploy.sh" "$NANO_IP" ||
    die "Controller deployment failed. Review the deployment output above."
}

build_workspace() {
  info "Installing package dependencies with rosdep."
  # shellcheck disable=SC1091
  set +u
  source "/opt/ros/$ELLY_ROS_DISTRO/setup.bash"
  set -u
  cd "$REPO_DIR"
  rosdep install --from-paths src --ignore-src -r -y ||
    die "Workspace dependencies could not be installed. Review the rosdep package named above."

  info "Building the robot-description package."
  colcon build --packages-select myagv_description --symlink-install ||
    die "The myagv_description build failed. Review the colcon error above."

  info "Running local tests."
  python3 -m unittest discover -s tests ||
    die "Local validation failed. The controller was not deployed; fix the failing test shown above."
  ok "Workspace built and local tests passed."
}

apply_wsl_graphics_fix() {
  if grep -qiE '(Microsoft|WSL)' /proc/version; then
    if ! grep -qxF 'export LIBGL_ALWAYS_SOFTWARE=1' "$HOME/.bashrc"; then
      printf '%s\n' 'export LIBGL_ALWAYS_SOFTWARE=1' >>"$HOME/.bashrc"
    fi
    ok "WSL graphics compatibility configured."
  fi
}

write_shell_commands() {
  local brain="bash /home/$NANO_USER/scripts/brain.sh"

  info "Updating Elly commands in $HOME/.bashrc."
  touch "$HOME/.bashrc" || die "Could not create or access $HOME/.bashrc."
  sed -i '/# === ELLY OS START ===/,/# === ELLY OS END ===/d' "$HOME/.bashrc" ||
    die "Could not remove the previous Elly command block from $HOME/.bashrc."

  cat <<EOF >>"$HOME/.bashrc" || die "Could not write Elly commands to $HOME/.bashrc."
# === ELLY OS START ===
source /opt/ros/galactic/setup.bash

export NANO_IP='$NANO_IP'
export NANO_USER='$NANO_USER'

alias start_rviz='source $REPO_DIR/install/setup.bash && rviz2 -d $REPO_DIR/rviz/elly_dash.rviz'
teleop() {
  python3 $REPO_DIR/scripts/elly.py require lidar || return
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
}

alias camera_on='ssh \$NANO_USER@\$NANO_IP "$brain camera_on"'
alias camera_off='ssh \$NANO_USER@\$NANO_IP "$brain camera_off"'
alias lidar_on='ssh \$NANO_USER@\$NANO_IP "$brain lidar_on"'
alias lidar_off='ssh \$NANO_USER@\$NANO_IP "$brain lidar_off"'
map_2d_on() {
  python3 $REPO_DIR/scripts/elly.py require lidar || return
  ssh \$NANO_USER@\$NANO_IP "$brain map_2d_on"
}
alias map_2d_off='ssh \$NANO_USER@\$NANO_IP "$brain map_2d_off"'
map_3d_on() {
  python3 $REPO_DIR/scripts/elly.py require lidar camera || return
  ssh \$NANO_USER@\$NANO_IP "$brain map_on"
}
alias map_3d_off='ssh \$NANO_USER@\$NANO_IP "$brain map_off"'
alias motion_on='ssh \$NANO_USER@\$NANO_IP "$brain motion_on"'
alias motion_off='ssh \$NANO_USER@\$NANO_IP "$brain motion_off"'
alias nav_off='ssh \$NANO_USER@\$NANO_IP "$brain nav_off"'

alias robot_status='ssh \$NANO_USER@\$NANO_IP "screen -ls"'
alias robot_peek='ssh -t \$NANO_USER@\$NANO_IP "screen -r"'

alias map_2d_save='bash $REPO_DIR/scripts/save_room.sh 2d'
alias map_2d_load='bash $REPO_DIR/scripts/load_room.sh 2d'
alias map_3d_save='bash $REPO_DIR/scripts/save_room.sh 3d'
alias map_3d_load='bash $REPO_DIR/scripts/load_room.sh 3d'

alias elly_move='python3 $REPO_DIR/scripts/elly_move.py'
alias elly_nav='python3 $REPO_DIR/scripts/elly_navigate.py'
alias elly_autofind='python3 $REPO_DIR/scripts/elly_autofind.py'
alias elly_deploy='bash $REPO_DIR/scripts/deploy.sh'

elly_move_stop() {
  ssh "\$NANO_USER@\$NANO_IP" python3 /home/\$NANO_USER/scripts/brain.py stop_motion
}

elly_move_status() {
  ssh "\$NANO_USER@\$NANO_IP" python3 /home/\$NANO_USER/scripts/brain.py status
}

alias elly='python3 $REPO_DIR/scripts/elly.py'
# === ELLY OS END ===
EOF
}

completion_summary() {
  local elapsed=$((SECONDS - INSTALL_STARTED_AT))

  printf '\n%s%s' "$BOLD" "$GREEN"
  cat <<'EOF'
  +--------------------------------------------------------+
  |                 ELLY OS IS READY                       |
  +--------------------------------------------------------+
EOF
  printf '%s' "$RESET"
  printf '  %s[ok]%s %-20s %s\n' "$GREEN" "$RESET" 'Host system' "Ubuntu $REQUIRED_UBUNTU"
  printf '  %s[ok]%s %-20s %s\n' "$GREEN" "$RESET" 'ROS environment' "ROS 2 $ELLY_ROS_DISTRO"
  printf '  %s[ok]%s %-20s %s\n' "$GREEN" "$RESET" 'Robot link' "$NANO_USER@$NANO_IP"
  printf '  %s[ok]%s %-20s %s\n' "$GREEN" "$RESET" '3D description' 'myagv_description built'
  printf '  %s[ok]%s %-20s %s\n' "$GREEN" "$RESET" 'Controller' 'deployed and running'
  printf '  %s[ok]%s %-20s %s\n' "$GREEN" "$RESET" 'Validation' 'local tests passed'
  printf '\n  Completed in %s.\n' "$(format_duration "$elapsed")"
  printf '  Type %selly%s to open the command reference.\n\n' "$BOLD" "$RESET"
}

open_configured_shell() {
  if ((OPEN_SHELL == 0)); then
    info "Fresh shell disabled by --no-shell."
    return
  fi

  if [[ ! -t 0 || ! -t 1 ]]; then
    warn "No interactive terminal is available; open a new terminal to load Elly commands."
    return
  fi

  printf '  %s-> Opening a configured Elly OS shell...%s\n\n' "$CYAN$BOLD" "$RESET"
  exec bash --rcfile "$HOME/.bashrc" -i
}

main() {
  SECONDS=0
  INSTALL_STARTED_AT=$SECONDS
  parse_arguments "$@"
  banner

  stage 'Host preflight'
  verify_host_os
  verify_admin_access
  install_missing_apt_packages "${HOST_BASE_PACKAGES[@]}"
  stage_done

  stage 'ROS 2 environment'
  configure_ros_repository
  install_missing_apt_packages "${HOST_ROS_PACKAGES[@]}"
  configure_rosdep
  stage_done

  stage 'Secure robot link'
  configure_ssh
  ok "Connected to $NANO_USER@$NANO_IP."
  stage_done

  stage 'Jetson dependencies'
  install_robot_packages
  install_robot_workspaces
  stage_done

  stage 'Workspace build and validation'
  build_workspace
  stage_done

  stage 'Robot controller deployment'
  deploy_robot_controller
  stage_done

  stage 'Shell integration'
  apply_wsl_graphics_fix
  write_shell_commands
  stage_done

  completion_summary
  open_configured_shell
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
