# =============================================================================
# amr_container.sh -- shared helper, sourced by run_real_mapping.sh and
# run_gazebo_sim.sh. Makes sure the Isaac ROS container is running WITHOUT
# downloading anything:
#
#   container running        -> use it
#   container stopped        -> docker start (normal case after a reboot)
#   container missing        -> recreate it from the saved image
#                               amr-isaac-ready (all packages pre-installed)
#
# Never use isaac_ros_common/scripts/run_dev.sh for this project: it DELETES a
# stopped container and builds a fresh one from the base image, which loses
# every package installed inside (that is what caused the re-downloads).
#
# To refresh the saved image after installing something new in the container:
#   docker commit isaac_ros_dev-aarch64-container amr-isaac-ready:latest
# =============================================================================

AMR_CONTAINER="isaac_ros_dev-aarch64-container"
AMR_IMAGE="amr-isaac-ready:latest"
AMR_WS_HOST="$HOME/workspaces/isaac_ros-dev"

amr_ensure_container() {
  local C="$AMR_CONTAINER"
  if docker ps --format '{{.Names}}' | grep -qx "$C"; then
    return 0
  fi
  if docker ps -a --format '{{.Names}}' | grep -qx "$C"; then
    docker start "$C" >/dev/null || return 1
    sleep 2
    return 0
  fi
  if ! docker image inspect "$AMR_IMAGE" >/dev/null 2>&1; then
    echo "ERROR: container $C and saved image $AMR_IMAGE are both missing." >&2
    return 1
  fi
  echo "     Container was missing: recreating it from the saved image $AMR_IMAGE (no download) ..."
  # Same settings as the original run_dev.sh container (host network and PID,
  # privileged for USB/GPU, nvidia runtime, Jetson library mounts).
  docker run -dit \
    --name "$C" \
    --privileged --network host --pid host --runtime nvidia \
    --group-add 1001 \
    --user admin \
    --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \
    --workdir /workspaces/isaac_ros-dev \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e ISAAC_ROS_WS=/workspaces/isaac_ros-dev \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$HOME/.Xauthority:/home/admin/.Xauthority:rw" \
    -v /usr/bin/tegrastats:/usr/bin/tegrastats \
    -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra \
    -v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api \
    -v /usr/share/vpi3:/usr/share/vpi3 \
    -v /tmp/:/tmp/ \
    -v /dev/input:/dev/input \
    -v /run/jtop.sock:/run/jtop.sock:ro \
    -v "$AMR_WS_HOST:/workspaces/isaac_ros-dev" \
    -v /etc/localtime:/etc/localtime:ro \
    "$AMR_IMAGE" /bin/bash >/dev/null || return 1
  sleep 4
  return 0
}

# $1 = space-separated package list. Checks offline (dpkg). Only if something
# is really missing does it fall back to apt (and tells you to refresh the image).
amr_check_packages() {
  local pkgs="$1" missing
  missing=$(docker exec "$AMR_CONTAINER" bash -c \
    "for p in $pkgs; do dpkg -s \$p >/dev/null 2>&1 || echo \$p; done" | tr '\n' ' ')
  [ -z "${missing// /}" ] && return 0
  echo "     Missing packages: $missing-> installing once (needs internet) ..."
  docker exec "$AMR_CONTAINER" bash -c \
    "sudo apt-get update -qq && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq $missing" \
    >/tmp/amr_pkg_install.log 2>&1 || return 1
  echo "     Installed. Saving them into $AMR_IMAGE so this never happens again ..."
  docker commit "$AMR_CONTAINER" "$AMR_IMAGE" >/dev/null 2>&1 || true
  return 0
}

# Files created by a container that ran as root (an older container did) cannot
# be overwritten by the current one (user admin = UID 1000), and colcon then
# fails with "Permission denied: install/local_setup.bash". Give them back.
amr_fix_permissions() {
  local d dirs=""
  for d in install build log logs maps demo_results; do
    [ -e "$AMR_WS_HOST/$d" ] && dirs="$dirs $AMR_WS_HOST/$d"
  done
  [ -z "$dirs" ] && return 0
  if [ -n "$(find $dirs ! -user "$(id -u)" -print -quit 2>/dev/null)" ]; then
    echo "     Fixing file ownership in the workspace (files left by a root container) ..."
    docker exec -u root "$AMR_CONTAINER" chown -R "$(id -u):$(id -g)" /workspaces/isaac_ros-dev
  fi
  return 0
}

# The project is built and tested with Isaac ROS 3.2 (cuVSLAM 3.2.6, nvblox 3.2.5).
# A fresh container from the base image only knows the "legacy-release-3.1" apt
# section, so apt installs 3.1.0 -> nvblox "esdf_mode" type errors, cuVSLAM
# tracker init error 4, missing typesupport symbols. Detect and repair that.
amr_check_versions() {
  local v
  v=$(docker exec "$AMR_CONTAINER" dpkg-query -W -f='${Version}' ros-humble-isaac-ros-visual-slam 2>/dev/null)
  case "$v" in
    3.2.*) return 0 ;;
  esac
  echo "     Isaac ROS version is '$v' but the project needs 3.2 -> repairing once (needs internet) ..."
  docker exec "$AMR_CONTAINER" bash -c '
    F=$(grep -rl "isaac-ros/release-3 jammy" /etc/apt/sources.list /etc/apt/sources.list.d/ | head -1)
    grep -q "release-3.0" "$F" || sudo sed -i "s#isaac-ros/release-3 jammy #isaac-ros/release-3 jammy release-3.0 #" "$F"
    sudo apt-get update -qq
    UP=$(apt list --upgradable 2>/dev/null | grep -E "isaac|nvblox|gxf" | cut -d/ -f1)
    [ -n "$UP" ] && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq $UP' \
    >/tmp/amr_version_fix.log 2>&1 || return 1
  echo "     Repaired. Saving into $AMR_IMAGE ..."
  docker commit "$AMR_CONTAINER" "$AMR_IMAGE" >/dev/null 2>&1 || true
  return 0
}
