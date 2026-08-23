#!/usr/bin/env bash
# Camera bring-up diagnostic for the Raspberry Pi 5 + Camera Module 2 NoIR (IMX219)
# Run it on the Pi. It never exits early: every stage reports PASS/FAIL/SKIP so you
# can see which link of the chain (ribbon -> kernel -> libcamera -> capture) is broken.
#
#   ./check_camera.sh            # checks only
#   ./check_camera.sh --capture  # also grab a still and a short clip into ./camera_check/

set -uo pipefail

DO_CAPTURE=0
[[ "${1:-}" == "--capture" ]] && DO_CAPTURE=1

OUT_DIR="$(pwd)/camera_check"
FAILURES=()

red()   { printf '\033[31m%s\033[0m\n' "$*"; }
green() { printf '\033[32m%s\033[0m\n' "$*"; }
yellow(){ printf '\033[33m%s\033[0m\n' "$*"; }

section() { printf '\n\033[1m== %s ==\033[0m\n' "$*"; }
pass()    { green   "  PASS  $*"; }
fail()    { red     "  FAIL  $*"; FAILURES+=("$*"); }
skip()    { yellow  "  SKIP  $*"; }
note()    { printf  '        %s\n' "$*"; }

# Ubuntu sets kernel.dmesg_restrict=1, so plain dmesg is empty for non-root
read_dmesg() {
    if dmesg 2>/dev/null | head -1 | grep -q .; then
        dmesg 2>/dev/null
    elif sudo -n true 2>/dev/null; then
        sudo dmesg 2>/dev/null
    else
        journalctl -k --no-pager 2>/dev/null
    fi
}

section "Host"
MODEL="$( { tr -d '\0' < /proc/device-tree/model; } 2>/dev/null )"
IS_PI5=0
if [[ -n "$MODEL" ]]; then
    pass "$MODEL"
    if grep -q "Raspberry Pi 5" <<< "$MODEL"; then
        IS_PI5=1
    else
        note "Not a Pi 5: the CAM connector pinout and driver notes below may differ"
    fi
else
    fail "no /proc/device-tree/model, this is not a Raspberry Pi"
fi
printf '        kernel: %s\n' "$(uname -r)"
printf '        distro: %s\n' "$(lsb_release -ds 2>/dev/null || cat /etc/os-release | grep PRETTY | cut -d'"' -f2)"

section "Boot config"
CONFIG_TXT=""
for c in /boot/firmware/config.txt /boot/config.txt; do
    [[ -f "$c" ]] && { CONFIG_TXT="$c"; break; }
done
if [[ -n "$CONFIG_TXT" ]]; then
    pass "found $CONFIG_TXT"
    CAM_LINES="$(grep -nE '^[[:space:]]*(camera_auto_detect|dtoverlay=imx|start_x|gpu_mem)' "$CONFIG_TXT")"
    if [[ -n "$CAM_LINES" ]]; then
        printf '%s\n' "$CAM_LINES" | sed 's/^/        /'
    else
        note "no camera_auto_detect or dtoverlay=imx219 line present"
    fi
    grep -qE '^[[:space:]]*camera_auto_detect=1' "$CONFIG_TXT" \
        || grep -qE '^[[:space:]]*dtoverlay=imx219' "$CONFIG_TXT" \
        || fail "neither camera_auto_detect=1 nor dtoverlay=imx219 is enabled in $CONFIG_TXT"
else
    fail "no config.txt found (looked in /boot/firmware and /boot)"
fi

section "Kernel / sensor probe"
DMESG="$(read_dmesg)"
if [[ -z "$DMESG" ]]; then
    skip "could not read the kernel log (try: sudo $0)"
else
    if grep -qi 'imx219' <<< "$DMESG"; then
        pass "kernel probed the IMX219 sensor"
        grep -i 'imx219' <<< "$DMESG" | tail -5 | sed 's/^/        /'
    else
        fail "no imx219 line in the kernel log, the sensor was not detected on the CSI bus"
        note "this is almost always the ribbon cable: wrong orientation, not fully seated, or the 22-pin Pi 5 adapter is missing"
    fi
    # Pi 5 uses the RP1 front end, not the old bcm2835-unicam
    if grep -qi 'rp1-cfe\|rp1_cfe' <<< "$DMESG"; then
        pass "RP1 camera front end (rp1-cfe) is up"
    elif grep -qi 'unicam' <<< "$DMESG"; then
        note "bcm2835-unicam is in use, that is the pre-Pi 5 path"
    fi
    ERRS="$(grep -iE 'imx219.*(fail|error|timeout)|cfe.*(fail|error)' <<< "$DMESG" | tail -5)"
    [[ -n "$ERRS" ]] && { fail "errors in the kernel log:"; printf '%s\n' "$ERRS" | sed 's/^/        /'; }
fi

section "Device nodes"
VIDEO_NODES=$(ls /dev/video* 2>/dev/null | wc -l)
MEDIA_NODES=$(ls /dev/media* 2>/dev/null | wc -l)
if (( VIDEO_NODES > 0 )); then
    pass "$VIDEO_NODES /dev/video* node(s), $MEDIA_NODES /dev/media* node(s)"
    note "on a Pi 5 these are CFE and ISP nodes, not a ready-to-use capture device, go through libcamera"
else
    fail "no /dev/video* nodes"
fi
if id -nG "$USER" | grep -qw video; then
    pass "$USER is in the 'video' group"
else
    fail "$USER is NOT in the 'video' group"
    note "fix: sudo usermod -aG video $USER   (then log out and back in)"
fi

section "Tooling"
RPICAM=""
for b in rpicam-hello libcamera-hello; do
    command -v "$b" >/dev/null 2>&1 && { RPICAM="$b"; break; }
done
if [[ -n "$RPICAM" ]]; then
    pass "found $RPICAM ($(command -v $RPICAM))"
else
    fail "neither rpicam-hello nor libcamera-hello is installed"
    note "fix: sudo apt update && sudo apt install -y rpicam-apps   (fallback: libcamera-tools, which gives you 'cam')"
fi
command -v v4l2-ctl >/dev/null 2>&1 && pass "v4l2-ctl present" \
    || note "v4l-utils not installed (optional): sudo apt install -y v4l-utils"

section "libcamera enumeration"
if [[ -n "$RPICAM" ]]; then
    LIST="$($RPICAM --list-cameras 2>&1)"
    printf '%s\n' "$LIST" | sed 's/^/        /'
    if grep -qi 'imx219' <<< "$LIST"; then
        pass "libcamera sees the IMX219"
    elif grep -qi 'no cameras available' <<< "$LIST"; then
        fail "libcamera reports no cameras available"
    else
        fail "libcamera did not report an IMX219"
    fi
else
    skip "no rpicam/libcamera binary to enumerate with"
fi

section "libcamera ISP backend"
# The Pi 5 ISP is PiSP, the vc4 backend only covers Pi 4 and earlier. A libcamera
# that ships vc4 alone is too old for this board and will report no cameras even
# though the kernel has already bound the sensor.
# A ROS install can carry its own newer libcamera under /opt/ros, which is often
# the one that actually supports this board even when the system copy does not
PISP_DIR=""
FOUND_ANY=0
# Layout differs by version: older builds drop the modules straight into
# .../libcamera, newer ones put them in an ipa/ subdirectory. Check both, the
# distinction matters because LIBCAMERA_IPA_MODULE_PATH must name the exact
# directory holding the .so files and overrides the built-in search when set
for d in /usr/lib/*/libcamera        /usr/lib/*/libcamera/ipa \
         /opt/ros/*/lib/libcamera    /opt/ros/*/lib/libcamera/ipa \
         /usr/local/lib/*/libcamera  /usr/local/lib/*/libcamera/ipa; do
    [[ -d "$d" ]] || continue
    ls "$d"/ipa_rpi_*.so >/dev/null 2>&1 || continue
    FOUND_ANY=1
    HAVE_PISP=0; HAVE_VC4=0
    [[ -f "$d/ipa_rpi_pisp.so" ]] && HAVE_PISP=1
    [[ -f "$d/ipa_rpi_vc4.so"  ]] && HAVE_VC4=1
    printf '        %s: pisp=%s vc4=%s\n' "$d" "$HAVE_PISP" "$HAVE_VC4"
    (( HAVE_PISP )) && [[ -z "$PISP_DIR" ]] && PISP_DIR="$d"
done

if (( ! FOUND_ANY )); then
    skip "no libcamera IPA modules found in any known prefix"
elif (( IS_PI5 )); then
    if [[ -n "$PISP_DIR" ]]; then
        pass "PiSP IPA available in $PISP_DIR"
        if [[ "$PISP_DIR" != /usr/lib/* ]]; then
            note "the system libcamera lacks PiSP, this prefix has it"
            note "try the default search first, that build usually knows its own path"
            note "only if it reports 'No IPA found': export LIBCAMERA_IPA_MODULE_PATH=$PISP_DIR"
        fi
    else
        fail "Pi 5 needs the PiSP backend and no ipa_rpi_pisp.so exists in any prefix"
        note "the vc4 handler will look for Unicam, find nothing, and report zero cameras"
    fi
else
    pass "IPA modules present"
fi

section "Media entity names vs libcamera expectations"
# libcamera matches the Pi 5 front end by exact entity name via DeviceMatch. The
# rp1-cfe driver renamed those nodes between kernel releases (fe_image0 became
# fe-image0), so a libcamera newer than the kernel matches nothing and reports
# zero cameras even though the handler, the IPA and the hardware are all fine
if ! command -v media-ctl >/dev/null 2>&1; then
    skip "media-ctl not installed (sudo apt install v4l-utils)"
elif ! command -v strings >/dev/null 2>&1; then
    skip "strings not installed (sudo apt install binutils)"
else
    ACTUAL="$(for m in /dev/media*; do
                  [[ -e "$m" ]] && media-ctl -d "$m" -p 2>/dev/null
              done | sed -n 's/^- entity [0-9]*: \([^ ]*\).*/\1/p' | sort -u)"
    if [[ -z "$ACTUAL" ]]; then
        skip "no media entities readable"
    else
        for so in /usr/lib/*/libcamera.so.* /opt/ros/*/lib/libcamera.so.* \
                  /usr/local/lib/*/libcamera.so.*; do
            [[ -f "$so" ]] || continue
            WANT="$(strings "$so" 2>/dev/null | grep -E '^rp1-cfe-' | sort -u)"
            [[ -n "$WANT" ]] || continue
            MISSING=""
            while read -r e; do
                [[ -n "$e" ]] || continue
                grep -qx "$e" <<< "$ACTUAL" || MISSING+=" $e"
            done <<< "$WANT"
            if [[ -z "$MISSING" ]]; then
                pass "$(basename "$so"): all expected rp1-cfe entities exist"
            else
                fail "$(basename "$so") expects rp1-cfe entities the kernel does not provide"
                note "missing:$MISSING"
                note "kernel $(uname -r) exposes: $(grep '^rp1-cfe' <<< "$ACTUAL" | tr '\n' ' ')"
                note "underscore vs hyphen here means libcamera and the kernel are different generations"
            fi
        done
    fi
fi

section "NoIR tuning file"
# libcamera cannot tell a NoIR IMX219 from the standard one, both report as 'imx219'.
# Without the NoIR tuning file the auto white balance is wrong and daylight shots come out pink.
TUNING=""
for t in /usr/share/libcamera/ipa/rpi/pisp/imx219_noir.json \
         /usr/share/libcamera/ipa/rpi/vc4/imx219_noir.json; do
    [[ -f "$t" ]] && { TUNING="$t"; break; }
done
if [[ -n "$TUNING" ]]; then
    pass "found $TUNING"
    (( IS_PI5 )) && [[ "$TUNING" == *"/vc4/"* ]] \
        && note "this is the vc4 tuning, a Pi 5 wants the pisp one, expect it to appear once libcamera is up to date"
    note "use it with: export LIBCAMERA_RPI_TUNING_FILE=$TUNING"
else
    fail "no imx219_noir.json tuning file found under /usr/share/libcamera/ipa/rpi"
    note "colours will be off until this is available, it ships with rpicam-apps / libcamera-ipa"
fi

if (( DO_CAPTURE )); then
    section "Capture"
    if [[ -z "$RPICAM" ]]; then
        skip "no capture binary"
    else
        mkdir -p "$OUT_DIR"
        STILL_BIN="${RPICAM/hello/still}"
        VID_BIN="${RPICAM/hello/vid}"
        [[ -n "$TUNING" ]] && export LIBCAMERA_RPI_TUNING_FILE="$TUNING"

        # -n disables the preview window, the Pi runs headless
        if "$STILL_BIN" -n -t 2000 -o "$OUT_DIR/still.jpg" >/dev/null 2>"$OUT_DIR/still.log"; then
            SZ=$(stat -c%s "$OUT_DIR/still.jpg" 2>/dev/null || echo 0)
            if (( SZ > 10000 )); then
                pass "still captured: $OUT_DIR/still.jpg ($SZ bytes)"
            else
                fail "still is suspiciously small ($SZ bytes), the sensor may be streaming black frames"
            fi
        else
            fail "$STILL_BIN failed, see $OUT_DIR/still.log"
            tail -5 "$OUT_DIR/still.log" | sed 's/^/        /'
        fi

        if "$VID_BIN" -n -t 3000 --codec mjpeg -o "$OUT_DIR/clip.mjpeg" >/dev/null 2>"$OUT_DIR/clip.log"; then
            SZ=$(stat -c%s "$OUT_DIR/clip.mjpeg" 2>/dev/null || echo 0)
            (( SZ > 10000 )) && pass "3s clip captured: $OUT_DIR/clip.mjpeg ($SZ bytes)" \
                             || fail "clip is suspiciously small ($SZ bytes)"
        else
            fail "$VID_BIN failed, see $OUT_DIR/clip.log"
            tail -5 "$OUT_DIR/clip.log" | sed 's/^/        /'
        fi
        note "pull them over with: scp $USER@$(hostname -I | awk '{print $1}'):$OUT_DIR/still.jpg ."
    fi
fi

section "Summary"
if (( ${#FAILURES[@]} == 0 )); then
    green "  All checks passed."
    (( DO_CAPTURE )) || note "re-run with --capture to actually grab a frame"
else
    red "  ${#FAILURES[@]} problem(s):"
    for f in "${FAILURES[@]}"; do printf '    - %s\n' "$f"; done
fi
printf '\n'
