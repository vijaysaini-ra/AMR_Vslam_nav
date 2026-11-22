#!/usr/bin/env bash
# =============================================================================
# run_real_mapping.sh -- map a REAL room with the RealSense D455 + Jetson.
# One command does everything:
#   checks camera -> starts container -> installs/builds -> stops old runs ->
#   launches D455 + cuVSLAM + nvblox (one GPU process) + RViz ->
#   saves the map automatically when you finish -> stops everything cleanly.
#
# Usage:
#   ./run_real_mapping.sh                         # map saved as map_<date_time>
#   ./run_real_mapping.sh labroom                 # map saved as "labroom"
#   ./run_real_mapping.sh labroom cam_height:=0.8 # extra launch options after the name
#
# Launch options: cam_height:=<m> (default 1.0)  emitter:=true  imu:=true  rviz:=false
#
# While it runs:   s + Enter  = save a snapshot of the map now (keeps mapping)
#                  q + Enter  or Ctrl+C = save the final map and stop everything
#
# How to map: hold the camera LEVEL at ~1 m, keep it still 3 s at start, walk
# SLOWLY, point at walls/furniture 0.5-4 m away, finish where you started.
# Maps are saved to ~/workspaces/isaac_ros-dev/maps/<name>/
# =============================================================================

set -u
C="isaac_ros_dev-aarch64-container"
WS_HOST="$HOME/workspaces/isaac_ros-dev"
WS_CT="/workspaces/isaac_ros-dev"
STAMP="$(date +%Y%m%d_%H%M%S)"

# first argument is the map name unless it is a launch option (contains :=)
NAME="map_${STAMP}"
if [ $# -gt 0 ] && [[ "$1" != *":="* ]]; then NAME="$1"; shift; fi
LAUNCH_ARGS="$*"
LOG_HOST="$WS_HOST/logs/mapping_${STAMP}.log"
LOG_CT="$WS_CT/logs/mapping_${STAMP}.log"

say()  { echo -e "\e[1;32m==>\e[0m $*"; }
warn() { echo -e "\e[1;33m!!\e[0m  $*"; }
die()  { echo -e "\e[1;31mERROR:\e[0m $*"; exit 1; }
ros()  { docker exec "$C" bash -c "source /opt/ros/humble/setup.bash && \
         source $WS_CT/install/setup.bash 2>/dev/null; $1"; }

# The container shares the host's process list (and the host also has ROS installed),
# so processes are matched by command AND checked to belong to the container's cgroup.
# This never touches anything running outside the container.
CID=""
ct_pids() {   # $1 = regex on the command line -> PIDs of matching processes IN the container
  [ -z "$CID" ] && CID=$(docker inspect -f '{{.Id}}' "$C" 2>/dev/null)
  [ -z "$CID" ] && return 0
  local p
  for p in $(pgrep -f "$1"); do
    grep -q "docker-$CID" "/proc/$p/cgroup" 2>/dev/null && echo "$p"
  done
}
LAUNCH_RE='^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch'
PERCEPTION_RE='^/opt/ros/humble/lib/rclcpp_components/component_container_mt'

stop_stack() {
  local pids
  pids=$(ct_pids "$LAUNCH_RE")
  [ -n "$pids" ] && docker exec "$C" kill -INT $pids 2>/dev/null
  for _ in $(seq 1 15); do
    [ -z "$(ct_pids "$LAUNCH_RE")" ] && break
    sleep 1
  done
  pids=$(ct_pids '/opt/ros/humble/|component_container|ign gazebo|ruby')
  [ -n "$pids" ] && docker exec "$C" kill -9 $pids 2>/dev/null
  return 0
}

# stop the log follower started below (its tail also exits by itself via --pid when
# this script ends; the greps after it then get end-of-input and exit too)
stop_follow() { pkill -f "^tail -n \+1 -F --pid=$$ " 2>/dev/null; return 0; }

perception_alive() { [ -n "$(ct_pids "$PERCEPTION_RE")" ]; }

mem_avail_mb() { awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo; }

save_map() {   # $1 = map name, $2 = "quick" -> 2D map + PNG only, no messages
  local n="$1" dir="$WS_CT/maps/$1" quick="${2:-}"
  [ -z "$quick" ] && say "Saving map '$n' ..."
  ros "mkdir -p $dir"
  ros "timeout 30 ros2 run nav2_map_server map_saver_cli -t /nvblox_node/static_occupancy_grid \
       -f $dir/$n --fmt pgm --ros-args -p map_subscribe_transient_local:=false -p save_map_timeout:=10.0 2>&1 \
       | grep -E 'Map saved' >/dev/null && echo '   2D map      : OK' || echo '   2D map      : FAILED (no map yet?)'" \
       | { [ -n "$quick" ] && cat >/dev/null || cat; }
  ros "python3 - <<'EOF'
import cv2, numpy as np
img = cv2.imread('$dir/$n.pgm', cv2.IMREAD_GRAYSCALE)
if img is not None:
    ys, xs = np.nonzero(img != 205)
    if len(xs):
        img = img[max(ys.min()-10,0):ys.max()+11, max(xs.min()-10,0):xs.max()+11]
    col = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    col[img == 205] = (90, 90, 90); col[img == 0] = (40, 20, 160)
    col = cv2.resize(col, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
    cv2.imwrite('$dir/$n.png', col); print('   PNG preview : OK')
EOF" | { [ -n "$quick" ] && cat >/dev/null || cat; }
  if [ -n "$quick" ]; then ros "chmod -R a+rwX $WS_CT/maps" 2>/dev/null; return 0; fi
  ros "timeout 30 ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath \
       \"{file_path: '$dir/${n}_mesh.ply'}\" | grep 'success=True' >/dev/null \
       && echo '   3D mesh     : OK' || echo '   3D mesh     : FAILED'"
  ros "timeout 60 ros2 service call /visual_slam/save_map \
       isaac_ros_visual_slam_interfaces/srv/FilePath \"{file_path: '$dir/cuvslam_map'}\" \
       | grep 'success=True' >/dev/null && echo '   cuVSLAM map : OK' || echo '   cuVSLAM map : FAILED'"
  ros "chmod -R a+rwX $WS_CT/maps" 2>/dev/null
  say "Saved to: $WS_HOST/maps/$n/"
}

FINISHING=0
finish() {
  [ "$FINISHING" = 1 ] && return; FINISHING=1
  echo
  stop_follow
  if perception_alive; then
    save_map "$NAME"
  else
    warn "Mapping process is not running, so only the last autosave can be kept."
  fi
  say "Stopping everything ..."
  stop_stack
  say "Done. Map: $WS_HOST/maps/$NAME/   Full log: $LOG_HOST"
  exit 0
}

crashed() {
  stop_follow
  echo
  warn "The mapping process (camera + cuVSLAM + nvblox) CRASHED."
  if grep -q "out of memory" "$LOG_HOST"; then
    warn "Reason: the Jetson ran out of memory (CPU and GPU share the same 7.4 GB)."
    warn "Close Chrome / PDF viewer / other apps, then run this script again."
    echo "     Biggest memory users now:"
    ps -eo rss,comm --sort=-rss | awk 'NR>1 {a[$2]+=$1} END {for (k in a) printf "%8.0f MB  %s\n", a[k]/1024, k}' | sort -rn | head -5
  else
    warn "Last log lines:"; grep -E "ERROR|FATAL|died" "$LOG_HOST" | tail -5
  fi
  stop_stack
  if [ -f "$WS_HOST/maps/$NAME/$NAME.pgm" ]; then
    say "The last autosave of the 2D map is kept in $WS_HOST/maps/$NAME/"
  fi
  exit 1
}

# ---------------------------------------------------------------- 1. checks
say "Checking the RealSense camera ..."
lsusb | grep -qi "RealSense" || die "D455 not found on USB. Plug it into a USB 3 (blue) port."
lsusb -t | grep -q "5000M" || warn "No USB 3 device seen: check the D455 is on a USB 3 port."

export DISPLAY="${DISPLAY:-:0}"
say "Starting the Isaac ROS container ..."
# shellcheck source=amr_container.sh
source "$(dirname "$(readlink -f "$0")")/amr_container.sh"
amr_ensure_container || die "Could not start the container (see message above)."
xhost +local: >/dev/null 2>&1 || warn "xhost failed: RViz may not open (DISPLAY=$DISPLAY)."

# ------------------------------------------------------- 2. check + build
say "Checking packages (offline) ..."
PKGS="ros-humble-librealsense2 ros-humble-isaac-ros-visual-slam ros-humble-nvblox-ros ros-humble-nvblox-rviz-plugin ros-humble-foxglove-bridge ros-humble-nav2-map-server"
amr_check_packages "$PKGS" || die "Package install failed (see /tmp/amr_pkg_install.log)."
amr_check_versions || die "Isaac ROS version repair failed (see /tmp/amr_version_fix.log)."
amr_fix_permissions
say "Building robot_bringup ..."
docker exec "$C" bash -c "source /opt/ros/humble/setup.bash && cd $WS_CT && \
  colcon build --symlink-install --packages-select robot_bringup > /tmp/build.log 2>&1" \
  || { docker exec "$C" tail -20 /tmp/build.log; die "Build failed."; }

# ---------------------------------------------------- 3. stop old runs
say "Stopping any previous ROS run (frees GPU memory) ..."
stop_stack

# ------------------------------------------------------ memory check
# CPU and GPU share 7.4 GB. Mapping needs ~2.5 GB free or nvblox crashes with
# "CUDA out of memory" (Chrome alone can take 2-3 GB).
NEED_MB=2500
while [ "$(mem_avail_mb)" -lt "$NEED_MB" ]; do
  warn "Only $(mem_avail_mb) MB memory free; mapping needs about $NEED_MB MB."
  echo "     Biggest memory users:"
  ps -eo rss,comm --sort=-rss | awk 'NR>1 {a[$2]+=$1} END {for (k in a) printf "%8.0f MB  %s\n", a[k]/1024, k}' | sort -rn | head -6
  echo "     Close Chrome / other apps, then press Enter to check again"
  echo "     (or type  go + Enter  to start anyway, Ctrl+C to quit)."
  read -r ans || break
  [ "$ans" = "go" ] && break
done
say "Free memory: $(mem_avail_mb) MB"

# ------------------------------------------------------------ 4. launch
mkdir -p "$WS_HOST/logs" && chmod a+rwx "$WS_HOST/logs"
say "Launching D455 + cuVSLAM + nvblox + RViz  ($LAUNCH_ARGS)"
docker exec -d -e DISPLAY="$DISPLAY" "$C" bash -c "source /opt/ros/humble/setup.bash && \
  source $WS_CT/install/setup.bash && \
  ros2 launch robot_bringup mapping_launch.py $LAUNCH_ARGS > $LOG_CT 2>&1"

trap finish INT TERM
sleep 2
# show only important lines from the launch log
tail -n +1 -F --pid=$$ "$LOG_HOST" 2>/dev/null | \
  grep --line-buffered -E 'process has died|\[ERROR\]|\[FATAL\]|Tracking lost|out of memory' | \
  grep --line-buffered -v 'glsl' &
TAIL_PID=$!

say "Waiting for camera tracking (about 20 s). KEEP THE CAMERA STILL ..."
OK=0
for _ in $(seq 1 12); do
  sleep 5
  if [ "$_" -gt 2 ] && ! perception_alive; then crashed; fi
  R=$(ros "timeout 6 ros2 topic hz /visual_slam/tracking/odometry 2>/dev/null | grep -m1 -oE 'average rate: [0-9.]+'" | grep -oE '[0-9.]+$')
  if [ -n "$R" ]; then OK=1; break; fi
done
if [ "$OK" = 1 ]; then
  say "Tracking OK ($R Hz). Start walking slowly now."
else
  warn "No tracking yet. Check the log: $LOG_HOST"
fi
echo
echo "   s + Enter          : save a snapshot of the map (keeps mapping)"
echo "   q + Enter / Ctrl+C : save the final map '$NAME' and stop everything"
echo "   (the 2D map is also autosaved every 2 minutes, so a crash never loses it all)"
echo

# ------------------------------------------------------------ 5. control
N=0
AUTOSAVE_SEC=120
LAST_SAVE=$(date +%s)
while true; do
  if [ -z "$(ct_pids "$LAUNCH_RE")" ]; then
    warn "The launch stopped by itself. Last log lines:"; tail -15 "$LOG_HOST"
    stop_follow; stop_stack; exit 1
  fi
  perception_alive || crashed
  if [ $(( $(date +%s) - LAST_SAVE )) -ge "$AUTOSAVE_SEC" ]; then
    save_map "$NAME" quick
    LAST_SAVE=$(date +%s)
    echo "   [autosave $(date +%H:%M:%S)] 2D map saved · free memory $(mem_avail_mb) MB"
  fi
  if read -r -t 5 key; then
    case "$key" in
      s|S) N=$((N+1)); save_map "${NAME}_snap${N}" ;;
      q|Q) finish ;;
    esac
  elif [ $? -le 128 ]; then
    sleep 5   # no keyboard (input closed): just keep monitoring
  fi
done
