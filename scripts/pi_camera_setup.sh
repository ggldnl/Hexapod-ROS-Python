#!/usr/bin/env bash
# Camera Module 2 / NoIR (IMX219) environment setup for Raspberry Pi 5 + Ubuntu 24.04 + ROS 2.
#
# This only prepares the environment. It starts no daemon and holds the camera only for the
# few seconds of its own smoke test, so the sensor stays free for you to launch a node
# whenever you want. It survives reboots because everything it sets is persistent:
#   /boot/firmware/config.txt      sensor overlay
#   ~/.config/pi-camera-env.sh     libcamera overrides (KEY=value)
#   ~/.local/bin/run_camera_ros.sh wrapper that applies them in the right order
#
# Run it on the Pi as your normal user, it calls sudo where needed. It is idempotent: run it
# again after every reboot it asks for and it picks up where it left off.
#
#   ./pi_camera_setup.sh            do the next thing that needs doing
#   ./pi_camera_setup.sh status     diagnose only, change nothing
#   ./pi_camera_setup.sh test       run camera_node for a few seconds and check the frames
#   ./pi_camera_setup.sh build      force the libcamera source build
#
# Options: --cam0 / --cam1 to force the CSI connector, --no-noir for a non-NoIR module,
#          --width N --height N, --format NAME (libcamera pixel format, '' to let the
#          driver choose).

set -uo pipefail

CAM_PORT=""
NOIR=1
WIDTH=640
HEIGHT=480
# XRGB8888 is what the RPi pipeline offers by default: 4 bytes per pixel with an unused
# alpha byte, which some viewers render as fully transparent and which costs 33% more
# bandwidth for nothing. RGB888 is three-channel and maps cleanly to bgr8
FORMAT="RGB888"
FORMAT_SET=0
CMD=""

while (( $# )); do
    case "$1" in
        --cam0)     CAM_PORT=0 ;;
        --cam1)     CAM_PORT=1 ;;
        --no-noir)  NOIR=0 ;;
        --width)    WIDTH="$2"; shift ;;
        --height)   HEIGHT="$2"; shift ;;
        --format)   FORMAT="$2"; FORMAT_SET=1; shift ;;
        status|test|build|setup) CMD="$1" ;;
        -h|--help)  sed -n '2,25p' "$0"; exit 0 ;;
        *) printf 'unknown argument: %s\n' "$1" >&2; exit 2 ;;
    esac
    shift
done
CMD="${CMD:-setup}"

STATE_DIR="$HOME/.local/state/pi-camera-setup"
STATE="$STATE_DIR/state"
LIBCAMERA_SRC="$HOME/src/libcamera"
ENV_FILE="$HOME/.config/pi-camera-env.sh"
WRAPPER="$HOME/.local/bin/run_camera_ros.sh"
CONFIG_TXT="/boot/firmware/config.txt"
TESTLOG="/tmp/camera_ros_test.log"

red()    { printf '\033[31m%s\033[0m\n' "$*"; }
green()  { printf '\033[32m%s\033[0m\n' "$*"; }
yellow() { printf '\033[33m%s\033[0m\n' "$*"; }

section() { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }
pass()    { green  "  ok    $*"; }
fail()    { red    "  FAIL  $*"; }
warn()    { yellow "  warn  $*"; }
note()    { printf '        %s\n' "$*"; }
die()     { red "  $*"; exit 1; }

mkdir -p "$STATE_DIR" "$(dirname "$ENV_FILE")" "$(dirname "$WRAPPER")"
touch "$STATE"
get_state() { grep -E "^$1=" "$STATE" 2>/dev/null | tail -1 | cut -d= -f2-; }
set_state() { local k="$1" v="$2"; sed -i "/^$k=/d" "$STATE"; printf '%s=%s\n' "$k" "$v" >> "$STATE"; }

# Ubuntu sets kernel.dmesg_restrict=1 so plain dmesg is empty for non-root
read_dmesg() {
    if dmesg 2>/dev/null | head -1 | grep -q .; then dmesg 2>/dev/null
    else sudo dmesg 2>/dev/null; fi
}

boot_epoch() {
    local up; up="$(cut -d' ' -f1 /proc/uptime)"
    printf '%s\n' "$(( $(date +%s) - ${up%.*} ))"
}

detect_ros() {
    if [[ -n "${ROS_DISTRO:-}" && -d "/opt/ros/${ROS_DISTRO}" ]]; then
        ROS_D="$ROS_DISTRO"
    else
        ROS_D="$(ls -1 /opt/ros 2>/dev/null | head -1)"
    fi
    [[ -n "${ROS_D:-}" && -f "/opt/ros/$ROS_D/setup.bash" ]]
}

# Keep whatever the last run chose unless this run overrides it, so re-running plain
# ./pi_camera_setup.sh does not silently revert your settings
load_prefs() {
    [[ -f "$ENV_FILE" ]] || return 0
    local w h f
    w="$(grep -E '^PI_CAMERA_WIDTH='  "$ENV_FILE" | cut -d= -f2-)"
    h="$(grep -E '^PI_CAMERA_HEIGHT=' "$ENV_FILE" | cut -d= -f2-)"
    f="$(grep -E '^PI_CAMERA_FORMAT=' "$ENV_FILE" | cut -d= -f2-)"
    [[ -n "$w" ]] && WIDTH="$w"
    [[ -n "$h" ]] && HEIGHT="$h"
    (( FORMAT_SET )) || [[ -z "$f" ]] || FORMAT="$f"
}

check_host() {
    section "Host"
    local model
    model="$( { tr -d '\0' < /proc/device-tree/model; } 2>/dev/null )"
    [[ -n "$model" ]] || die "no /proc/device-tree/model, this is not a Raspberry Pi"
    pass "$model"
    grep -q "Raspberry Pi 5" <<< "$model" || warn "not a Pi 5, the CSI port and overlay notes below assume Pi 5"
    note "kernel: $(uname -r)"
    note "distro: $(lsb_release -ds 2>/dev/null || grep PRETTY /etc/os-release | cut -d'"' -f2)"
    if detect_ros; then pass "ROS 2 $ROS_D found at /opt/ros/$ROS_D"
    else die "no ROS 2 installation under /opt/ros, install ROS 2 first"; fi
    if id -nG "$USER" | grep -qw video; then
        pass "$USER is in the 'video' group"
    else
        warn "$USER is not in the 'video' group, adding it"
        sudo usermod -aG video "$USER"
        note "log out and back in (or reboot) for this to take effect"
    fi
}

# Writes camera_auto_detect=0 plus an explicit port-qualified overlay. Leaving auto-detect on
# alongside an explicit overlay lets the firmware instantiate the sensor twice, and which one
# wins varies from boot to boot.
apply_overlay() {
    local port="$1"
    section "Boot config (CAM$port)"
    [[ -f "$CONFIG_TXT" ]] || die "no $CONFIG_TXT"
    [[ -f /boot/firmware/overlays/imx219.dtbo ]] \
        || warn "no /boot/firmware/overlays/imx219.dtbo, the overlay may not load (try: sudo apt install --reinstall linux-firmware-raspi)"

    local block
    block=$(printf '# BEGIN pi_camera_setup\n[all]\ncamera_auto_detect=0\ndtoverlay=imx219,cam%s\n# END pi_camera_setup\n' "$port")
    local current
    current=$(sed -n '/^# BEGIN pi_camera_setup$/,/^# END pi_camera_setup$/p' "$CONFIG_TXT")

    if [[ "$current" == "$block" ]]; then
        pass "config.txt already set for CAM$port"
        set_state cam_port "$port"
        return 0
    fi

    [[ -f "$CONFIG_TXT.pi_camera_setup.bak" ]] \
        || sudo cp "$CONFIG_TXT" "$CONFIG_TXT.pi_camera_setup.bak"

    sudo sed -i '/^# BEGIN pi_camera_setup$/,/^# END pi_camera_setup$/d' "$CONFIG_TXT"
    # Comment out any auto-detect left over from the stock image, ours must be the only one
    sudo sed -i 's/^[[:space:]]*camera_auto_detect=1/#&/' "$CONFIG_TXT"
    printf '%s\n' "$block" | sudo tee -a "$CONFIG_TXT" >/dev/null

    pass "wrote overlay for CAM$port (backup: $CONFIG_TXT.pi_camera_setup.bak)"
    set_state cam_port "$port"
    return 10   # signals "reboot needed"
}

reboot_pending() {
    (( $(stat -c %Y "$CONFIG_TXT" 2>/dev/null || echo 0) > $(boot_epoch) ))
}

check_sensor() {
    section "Kernel / sensor probe"
    local dm; dm="$(read_dmesg)"
    if [[ -z "$dm" ]]; then warn "could not read the kernel log"; return 1; fi

    if grep -qi 'Using sensor imx219' <<< "$dm"; then
        pass "kernel bound the IMX219 sensor"
        grep -i 'imx219' <<< "$dm" | tail -3 | sed 's/^/        /'
    elif grep -qi 'imx219' <<< "$dm"; then
        fail "the imx219 driver loaded but did not bind to a sensor"
        grep -i 'imx219' <<< "$dm" | tail -5 | sed 's/^/        /'
        return 1
    else
        fail "no imx219 line in the kernel log at all"
        return 1
    fi

    grep -qi 'rp1-cfe\|rp1_cfe' <<< "$dm" && pass "RP1 camera front end is up" \
        || warn "no rp1-cfe in the log, unexpected on a Pi 5"
    local n; n=$(ls /dev/video* 2>/dev/null | wc -l)
    (( n > 0 )) && pass "$n /dev/video* node(s), $(ls /dev/media* 2>/dev/null | wc -l) /dev/media* node(s)" \
                || { fail "no /dev/video* nodes"; return 1; }
    return 0
}

install_packages() {
    section "Packages"
    local want=("ros-$ROS_D-camera-ros" "ros-$ROS_D-libcamera" v4l-utils)
    local missing=()
    for p in "${want[@]}"; do
        dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p")
    done
    if (( ${#missing[@]} )); then
        note "installing: ${missing[*]}"
        sudo apt-get update -qq
        sudo apt-get install -y "${missing[@]}" || die "apt install failed"
    fi
    for p in "${want[@]}"; do
        dpkg -s "$p" >/dev/null 2>&1 && pass "$p $(dpkg-query -W -f='${Version}' "$p")" \
                                     || fail "$p is not installed"
    done
}

find_tuning_file() {
    (( NOIR )) || { printf '\n'; return; }
    local f
    for f in "$LIBCAMERA_SRC/src/ipa/rpi/pisp/data/imx219_noir.json" \
             "/opt/ros/$ROS_D/share/libcamera/ipa/rpi/pisp/imx219_noir.json" \
             "/usr/share/libcamera/ipa/rpi/pisp/imx219_noir.json"; do
        [[ -f "$f" ]] && { printf '%s\n' "$f"; return; }
    done
    printf '\n'
}

# One file of KEY=value lines, sourceable from a shell and parseable by anything else that
# needs the same overrides (a launch file, for instance). No variable references in the
# values, nothing here expands them.
write_env_file() {
    local use_build="$1" prefix="" ipa="" tuning d
    if (( use_build )); then
        prefix="$LIBCAMERA_SRC/build/src/libcamera:$LIBCAMERA_SRC/build/src/libcamera/base"
        for d in "$LIBCAMERA_SRC"/build/subprojects/libpisp*/src; do
            [[ -d "$d" ]] && prefix="$prefix:$d"
        done
        ipa="$LIBCAMERA_SRC/build/src/ipa/rpi/vc4:$LIBCAMERA_SRC/build/src/ipa/rpi/pisp"
    fi
    tuning="$(find_tuning_file)"

    {
        printf '# Generated by pi_camera_setup.sh, KEY=value only\n'
        printf 'PI_CAMERA_WIDTH=%s\n' "$WIDTH"
        printf 'PI_CAMERA_HEIGHT=%s\n' "$HEIGHT"
        printf 'PI_CAMERA_FORMAT=%s\n' "$FORMAT"
        [[ -n "$prefix" ]] && printf 'PI_CAMERA_LD_PREFIX=%s\n' "$prefix"
        [[ -n "$ipa"    ]] && printf 'LIBCAMERA_IPA_MODULE_PATH=%s\n' "$ipa"
        [[ -n "$tuning" ]] && printf 'LIBCAMERA_RPI_TUNING_FILE=%s\n' "$tuning"
    } > "$ENV_FILE"
    pass "wrote $ENV_FILE"
    sed 's/^/        /' "$ENV_FILE"
}

# The overrides have to be applied after sourcing setup.bash: ROS prepends /opt/ros/$D/lib to
# LD_LIBRARY_PATH, so setting ours first would put the packaged libcamera back in front.
write_wrapper() {
    cat > "$WRAPPER" <<WRAP
#!/usr/bin/env bash
# Generated by pi_camera_setup.sh. Runs camera_ros with the right libcamera in front.
# Any extra arguments are appended to the --ros-args list, for example:
#   run_camera_ros.sh -p width:=1280 -p height:=720
set -eo pipefail

ENV_FILE="$ENV_FILE"
if [[ -f "\$ENV_FILE" ]]; then
    set -a
    . "\$ENV_FILE"
    set +a
fi

# ROS's setup.bash trips over set -u
set +u
. /opt/ros/$ROS_D/setup.bash
set -u

if [[ -n "\${PI_CAMERA_LD_PREFIX:-}" ]]; then
    export LD_LIBRARY_PATH="\$PI_CAMERA_LD_PREFIX\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
fi

args=(-p width:=\${PI_CAMERA_WIDTH:-640} -p height:=\${PI_CAMERA_HEIGHT:-480})
[[ -n "\${PI_CAMERA_FORMAT:-}" ]] && args+=(-p format:=\$PI_CAMERA_FORMAT)

exec ros2 run camera_ros camera_node --ros-args "\${args[@]}" "\$@"
WRAP
    chmod +x "$WRAPPER"
    pass "wrote $WRAPPER"
}

# Runs the node for a few seconds and checks a topic actually carries frames. This is the
# only test that means anything: rpicam-hello and libcamera-tools link a different libcamera
# than the one camera_ros loads, so they can pass while ROS fails and vice versa.
run_test() {
    section "camera_ros smoke test"
    if pgrep -f 'camera_ros.*camera_node' >/dev/null; then
        # libcamera hands the sensor to one client at a time
        warn "a camera_node is already running and holds the camera, stop it first"
        return 1
    fi
    [[ -x "$WRAPPER" ]] || { fail "no $WRAPPER yet, run the setup first"; return 1; }

    : > "$TESTLOG"
    setsid "$WRAPPER" > "$TESTLOG" 2>&1 &
    local pid=$! rc=1
    note "started camera_node (pid $pid), log: $TESTLOG"

    # shellcheck disable=SC1090
    set +u
    . "/opt/ros/$ROS_D/setup.bash"
    set -u

    local i topics=""
    for i in $(seq 30); do
        if grep -qiE 'no cameras available|failed to acquire|terminate called|was not declared' "$TESTLOG"; then
            break
        fi
        topics="$(timeout 5 ros2 topic list 2>/dev/null)"
        grep -q '/camera/image_raw' <<< "$topics" && break
        sleep 1
    done

    if grep -q '/camera/image_raw' <<< "$topics"; then
        pass "/camera/image_raw is advertised"
        local hz
        hz="$(timeout 12 ros2 topic hz /camera/image_raw 2>/dev/null | grep -o 'average rate: [0-9.]*' | tail -1)"
        if [[ -n "$hz" ]]; then
            pass "frames are flowing, $hz"
            grep -m1 'configured with' "$TESTLOG" | sed 's/^/        /'
            rc=0
        else
            fail "topic exists but no frames arrived in 12s"
        fi
    else
        fail "camera_node did not come up"
        tail -15 "$TESTLOG" | sed 's/^/        /'
        grep -qi 'was not declared' "$TESTLOG" && \
            note "this build of camera_ros has no 'format' parameter: blank PI_CAMERA_FORMAT in $ENV_FILE"
    fi

    kill -TERM -"$pid" 2>/dev/null
    sleep 2
    kill -KILL -"$pid" 2>/dev/null
    pkill -f 'camera_ros.*camera_node' 2>/dev/null
    sleep 1
    return $rc
}

build_libcamera() {
    section "Building libcamera from source"
    note "this takes roughly 15-25 minutes on a Pi 5"
    sudo apt-get update -qq
    sudo apt-get install -y git build-essential meson ninja-build pkg-config \
        libyaml-dev libudev-dev libssl-dev libgnutls28-dev libevent-dev \
        python3-yaml python3-ply python3-jinja2 || die "build dependencies failed to install"

    if [[ -d "$LIBCAMERA_SRC/.git" ]]; then
        note "updating existing checkout at $LIBCAMERA_SRC"
        git -C "$LIBCAMERA_SRC" pull --ff-only || warn "pull failed, building the existing checkout"
    else
        mkdir -p "$(dirname "$LIBCAMERA_SRC")"
        git clone https://github.com/raspberrypi/libcamera.git "$LIBCAMERA_SRC" \
            || die "git clone failed"
    fi
    note "libcamera version: $(git -C "$LIBCAMERA_SRC" describe --tags 2>/dev/null)"

    local opts=(--buildtype=release
        -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp
        -Dcam=enabled -Dqcam=disabled -Dgstreamer=disabled -Dpycamera=disabled
        -Dtest=false -Dlc-compliance=disabled -Ddocumentation=disabled)
    [[ -d "$LIBCAMERA_SRC/build" ]] && opts+=(--wipe)
    ( cd "$LIBCAMERA_SRC" && meson setup build "${opts[@]}" ) || die "meson setup failed"
    ( cd "$LIBCAMERA_SRC" && ninja -C build ) || die "ninja build failed"
    pass "built $LIBCAMERA_SRC/build/src/libcamera/libcamera.so"

    section "Verifying the build sees the camera"
    local out
    out="$( cd "$LIBCAMERA_SRC/build" && \
        LD_LIBRARY_PATH="$PWD/src/libcamera:$PWD/src/libcamera/base" \
        LIBCAMERA_IPA_MODULE_PATH="$PWD/src/ipa/rpi/vc4:$PWD/src/ipa/rpi/pisp" \
        ./src/apps/cam/cam -l 2>&1 )"
    printf '%s\n' "$out" | sed 's/^/        /'
    grep -qi imx219 <<< "$out" || die "the source build still does not see the camera, stopping here"
    pass "the source build enumerates the IMX219"
}

usage_hint() {
    section "How to use the camera"
    note "Standalone:   $WRAPPER"
    note "From a launch file, apply the overrides in $ENV_FILE to the node's environment."
    note "Nothing runs on its own: the sensor is free until you start a node."
}

show_status() {
    check_host
    check_sensor
    section "Configuration"
    sed -n '/^# BEGIN pi_camera_setup$/,/^# END pi_camera_setup$/p' "$CONFIG_TXT" | sed 's/^/        /'
    [[ -f "$ENV_FILE" ]] && { pass "$ENV_FILE"; sed 's/^/        /' "$ENV_FILE"; } \
                         || warn "no $ENV_FILE yet"
    [[ -x "$WRAPPER" ]] && pass "$WRAPPER" || warn "no $WRAPPER yet"
    if [[ -d "$LIBCAMERA_SRC/build" ]]; then
        pass "source-built libcamera at $LIBCAMERA_SRC"
        note "version: $(git -C "$LIBCAMERA_SRC" describe --tags 2>/dev/null)"
        note "both the source and build trees are needed at runtime, do not delete either"
    else
        note "no source-built libcamera, using the packaged one"
    fi
    section "Camera in use"
    if pgrep -af 'camera_ros.*camera_node' | sed 's/^/        /' | grep -q .; then
        warn "a camera_node is running and holds the camera"
    else
        pass "nothing is holding the camera"
    fi
    if systemctl list-unit-files camera_ros.service >/dev/null 2>&1 \
       && systemctl is-enabled camera_ros.service >/dev/null 2>&1; then
        warn "a camera_ros.service is enabled at boot and will hold the camera"
        note "remove it with: sudo systemctl disable --now camera_ros && sudo rm /etc/systemd/system/camera_ros.service"
    fi
}

do_setup() {
    check_host
    load_prefs

    local port="${CAM_PORT:-$(get_state cam_port)}"
    port="${port:-0}"
    apply_overlay "$port"
    if (( $? == 10 )) || reboot_pending; then
        section "Reboot needed"
        yellow "  config.txt changed. Reboot, then run this script again:"
        note "sudo reboot"
        exit 0
    fi

    if ! check_sensor; then
        local tried; tried="$(get_state tried_both)"
        if [[ -z "$CAM_PORT" && "$tried" != "1" && "$port" == "0" ]]; then
            warn "nothing on CAM0, switching the overlay to CAM1 and trying again"
            set_state tried_both 1
            apply_overlay 1
            section "Reboot needed"
            yellow "  Reboot, then run this script again:"
            note "sudo reboot"
            exit 0
        fi
        section "Stop"
        red "  The kernel cannot see the sensor on either CSI connector."
        note "This is almost always the ribbon cable: check the orientation (contacts"
        note "toward the board's contacts), that it is fully seated, and that the latch"
        note "is down on both ends. The Pi 5 uses the narrow 22-pin connector, so a"
        note "Camera Module 2 needs the adapter cable that shipped with the Pi 5."
        note "Then re-run this script."
        exit 1
    fi

    install_packages

    # Try the packaged libcamera first, the source build is only a workaround
    local use_build=0
    [[ -d "$LIBCAMERA_SRC/build/src/libcamera" ]] && use_build=1
    write_env_file "$use_build"
    write_wrapper
    section "Testing $( ((use_build)) && echo "the source-built libcamera" || echo "the packaged libcamera" )"
    if run_test; then
        section "Done"
        green "  Camera is set up and the environment is persistent."
        usage_hint
        exit 0
    fi

    if (( use_build )); then
        section "Stop"
        red "  Failing even with the source build. Full log: $TESTLOG"
        exit 1
    fi

    warn "the packaged libcamera could not open the camera, falling back to a source build"
    grep -iE 'no cameras available|failed to acquire' "$TESTLOG" | tail -3 | sed 's/^/        /'

    build_libcamera
    write_env_file 1
    write_wrapper
    section "Re-testing with the source build"
    if run_test; then
        section "Done"
        green "  Camera is set up and the environment is persistent."
        usage_hint
        exit 0
    fi
    section "Stop"
    red "  Still failing after the source build. Full log: $TESTLOG"
    exit 1
}

case "$CMD" in
    status)  detect_ros; show_status ;;
    test)    detect_ros || die "no ROS 2 found"; load_prefs; run_test ;;
    build)   detect_ros || die "no ROS 2 found"; load_prefs
             build_libcamera; write_env_file 1; write_wrapper; run_test ;;
    setup)   do_setup ;;
esac
