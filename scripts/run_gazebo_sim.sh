#!/usr/bin/env bash
# =============================================================================
# run_gazebo_sim.sh -- full AMR SIMULATION demo (no hardware needed).
# One command does everything:
#   starts container -> installs/builds -> stops old runs -> launches
#   Gazebo (simulated robot + D455-like camera) + nvblox GPU map + Nav2 A* + RViz
#   -> runs 12 pre-flight tests -> drives a 5-goal tour -> prints the summary and
#   saves report, CSV and final map -> stops everything cleanly on Ctrl+C.
#
# Usage:
#   ./run_gazebo_sim.sh                # RViz + automatic tour
#   ./run_gazebo_sim.sh rviz:=false    # headless (fastest, best results)
#   ./run_gazebo_sim.sh tour:=false    # no tour: click "2D Goal Pose" in RViz yourself
#   ./run_gazebo_sim.sh gz_gui:=true   # also open the Gazebo 3D window (heavy)
#
# Results: ~/workspaces/isaac_ros-dev/demo_results/
#   test_report.txt  results.csv  map_final.png  map_final.pgm/.yaml  full_launch.log
# Tip: don't use the GNOME screen recorder at the same time (it slows the sim).
# =============================================================================

set -u
C="isaac_ros_dev-aarch64-container"
WS_HOST="$HOME/workspaces/isaac_ros-dev"
WS_CT="/workspaces/isaac_ros-dev"
LAUNCH_ARGS="$*"

say()  { echo -e "\e[1;32m==>\e[0m $*"; }
warn() { echo -e "\e[1;33m!!\e[0m  $*"; }
die()  { echo -e "\e[1;31mERROR:\e[0m $*"; exit 1; }

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

FINISHING=0
finish() {
  [ "$FINISHING" = 1 ] && return; FINISHING=1
  echo
  say "Stopping everything ..."
  stop_stack
  if [ -f "$WS_HOST/demo_results/test_report.txt" ]; then
    say "Results in $WS_HOST/demo_results/ :"
    ls -1 "$WS_HOST/demo_results/" | sed 's/^/     /'
  fi
  exit 0
}

# ------------------------------------------------------------ 1. container
export DISPLAY="${DISPLAY:-:0}"
say "Starting the Isaac ROS container ..."
# shellcheck source=amr_container.sh
source "$(dirname "$(readlink -f "$0")")/amr_container.sh"
amr_ensure_container || die "Could not start the container (see message above)."
xhost +local: >/dev/null 2>&1 || warn "xhost failed: RViz may not open (DISPLAY=$DISPLAY)."

# ------------------------------------------------------- 2. check + build
say "Checking packages (offline) ..."
PKGS="ros-humble-ros-gz ros-humble-nvblox-ros ros-humble-nvblox-nav2 ros-humble-nvblox-rviz-plugin ros-humble-foxglove-bridge ros-humble-isaac-ros-visual-slam ros-humble-nav2-bringup"
amr_check_packages "$PKGS" || die "Package install failed (see /tmp/amr_pkg_install.log)."
amr_check_versions || die "Isaac ROS version repair failed (see /tmp/amr_version_fix.log)."
amr_fix_permissions
say "Building robot_bringup + robot_sim ..."
docker exec "$C" bash -c "source /opt/ros/humble/setup.bash && cd $WS_CT && \
  colcon build --symlink-install --packages-select robot_bringup robot_sim > /tmp/build.log 2>&1" \
  || { docker exec "$C" tail -20 /tmp/build.log; die "Build failed."; }

# ---------------------------------------------------- 3. stop old runs
say "Stopping any previous ROS run (frees GPU memory) ..."
stop_stack

# ------------------------------------------------------ memory check
# CPU and GPU share 7.4 GB; Gazebo + nvblox + Nav2 need ~3.5 GB free
# (Chrome alone can take 2-3 GB and cause "CUDA out of memory").
mem_avail_mb() { awk '/MemAvailable/ {print int($2/1024)}' /proc/meminfo; }
NEED_MB=3500
while [ "$(mem_avail_mb)" -lt "$NEED_MB" ]; do
  warn "Only $(mem_avail_mb) MB memory free; the simulation needs about $NEED_MB MB."
  echo "     Biggest memory users:"
  ps -eo rss,comm --sort=-rss | awk 'NR>1 {a[$2]+=$1} END {for (k in a) printf "%8.0f MB  %s\n", a[k]/1024, k}' | sort -rn | head -6
  echo "     Close Chrome / other apps, then press Enter to check again"
  echo "     (or type  go + Enter  to start anyway, Ctrl+C to quit)."
  read -r ans || break
  [ "$ans" = "go" ] && break
done
say "Free memory: $(mem_avail_mb) MB"

# ------------------------------------------------------------ 4. launch
mkdir -p "$WS_HOST/demo_results"
say "Launching Gazebo + nvblox + Nav2 + RViz  ($LAUNCH_ARGS)"
echo "     RViz opens in ~10 s; tests + tour start after ~15 s; tour takes ~3-5 min."
echo "     When the summary is printed you can keep looking at RViz. Ctrl+C = stop everything."
echo
trap finish INT TERM

# Terminal shows only the demo output (tests, A* metrics, summary) + crashes;
# everything else goes to demo_results/full_launch.log.
docker exec -e DISPLAY="$DISPLAY" "$C" bash -c "source /opt/ros/humble/setup.bash && \
  source $WS_CT/install/setup.bash && \
  ros2 launch robot_sim sim_demo_launch.py $LAUNCH_ARGS 2>&1 | \
  tee $WS_CT/demo_results/full_launch.log | \
  grep --line-buffered -E '^\\[waypoint_demo|process has died' | \
  grep --line-buffered -vE '\\] \\[INFO\\]|process started' | \
  sed -u -E 's/^\\[waypoint_demo-[0-9]+\\] //'" &
RUN_PID=$!
wait "$RUN_PID"
# launch ended by itself (crash or all nodes exited)
warn "The simulation stopped by itself. Full log: $WS_HOST/demo_results/full_launch.log"
finish
