#!/usr/bin/env bash
# Dump everything needed to work out why libcamera reports no cameras on a Pi 5.
# Answers two questions the previous rounds left open: where the IPA modules
# really live, and whether this libcamera build contains the PiSP pipeline
# handler at all. Run it with the ROS environment sourced and paste the output.
#
#   source /opt/ros/jazzy/setup.bash
#   ./dump_libcamera.sh 2>&1 | tee /tmp/libcamera_dump.txt

set -uo pipefail

ROS_PREFIX="${ROS_PREFIX:-/opt/ros/${ROS_DISTRO:-jazzy}}"

section() { printf '\n\033[1m===== %s =====\033[0m\n' "$*"; }

section "Environment"
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-<unset>}"
printf 'ROS_PREFIX=%s\n' "$ROS_PREFIX"
printf 'CONDA_DEFAULT_ENV=%s\n' "${CONDA_DEFAULT_ENV:-<unset>}"
printf 'LIBCAMERA_IPA_MODULE_PATH=%s\n' "${LIBCAMERA_IPA_MODULE_PATH:-<unset>}"
printf 'LIBCAMERA_IPA_PROXY_PATH=%s\n' "${LIBCAMERA_IPA_PROXY_PATH:-<unset>}"
printf 'LIBCAMERA_RPI_TUNING_FILE=%s\n' "${LIBCAMERA_RPI_TUNING_FILE:-<unset>}"
printf 'LD_LIBRARY_PATH=%s\n' "${LD_LIBRARY_PATH:-<unset>}"

section "libcamera shared objects on this system"
ls -la /usr/lib/*/libcamera.so* "$ROS_PREFIX"/lib/libcamera.so* 2>/dev/null

section "Every ipa_*.so on the box"
find /usr/lib /usr/local/lib "$ROS_PREFIX" -name 'ipa_*.so' 2>/dev/null | sort

section "ROS libcamera module tree"
find "$ROS_PREFIX/lib/libcamera" -maxdepth 2 2>/dev/null | sort

section "ROS libcamera tuning data"
ls "$ROS_PREFIX"/share/libcamera/ipa/rpi/*/ 2>/dev/null | head -40
printf -- '--- imx219 tuning files ---\n'
find "$ROS_PREFIX/share/libcamera" -name 'imx219*.json' 2>/dev/null | sort

section "Pipeline handlers compiled into each libcamera"
# The handler names are string literals in the binary, so this shows what the
# build actually enabled rather than what the docs claim for the version
for so in /usr/lib/*/libcamera.so.* "$ROS_PREFIX"/lib/libcamera.so.*; do
    [[ -f "$so" ]] || continue
    printf -- '--- %s ---\n' "$so"
    strings "$so" 2>/dev/null | grep -oE 'PipelineHandler[A-Za-z0-9_]+' | sort -u | sed 's/^/    /'
    printf '    rpi data paths: '
    strings "$so" 2>/dev/null | grep -oE 'rpi/(pisp|vc4)' | sort -u | tr '\n' ' '
    printf '\n'
done

section "What camera_node links against"
# Several packages ship an executable called camera_node, so pin camera_ros
CAM_NODE="$ROS_PREFIX/lib/camera_ros/camera_node"
[[ -x "$CAM_NODE" ]] || CAM_NODE="$(find "$ROS_PREFIX/lib/camera_ros" -name 'camera_node' -type f 2>/dev/null | head -1)"
if [[ -n "$CAM_NODE" ]]; then
    printf 'binary: %s\n' "$CAM_NODE"
    ldd "$CAM_NODE" 2>/dev/null | grep -i camera | sed 's/^/    /'
else
    printf 'camera_node binary not found under %s/lib\n' "$ROS_PREFIX"
fi

section "Media devices"
ls -la /dev/media* /dev/video* 2>/dev/null | head -20
if command -v media-ctl >/dev/null 2>&1; then
    for m in /dev/media0 /dev/media1; do
        [[ -e "$m" ]] || continue
        printf -- '--- %s ---\n' "$m"
        media-ctl -d "$m" -p 2>/dev/null | grep -iE 'driver|model|entity' | head -15 | sed 's/^/    /'
    done
else
    printf 'media-ctl not installed (sudo apt install v4l-utils)\n'
fi

section "camera_node debug run"
if [[ -n "$CAM_NODE" ]]; then
    LIBCAMERA_LOG_LEVELS='*:DEBUG' timeout 20 "$CAM_NODE" 2>&1 \
        | grep -iE 'libcamera v|ipa|pipeline handler|pisp|vc4|unicam|rp1|cfe|no cameras|matched' \
        | head -60 | sed 's/^/    /'
else
    printf 'skipped, no camera_node binary\n'
fi

printf '\n'
