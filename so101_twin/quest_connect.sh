#!/usr/bin/env bash
# Connect a Meta Quest 3 to this PC over USB and forward the twin server's port to it.
#
#   ./quest_connect.sh            # forward port 8000
#   ./quest_connect.sh 8000 open  # forward, then ask the Quest to open the twin page
#
# Why: WebXR (AR) only runs on https:// or http://localhost. With `adb reverse`, the Quest's
# "localhost:8000" is tunnelled over the USB cable to this PC's 127.0.0.1:8000, so the page counts
# as secure, nothing goes over the campus network, and twin_server.py can stay bound to 127.0.0.1.
#
# The forward is lost when the cable is unplugged or the headset reboots: just run this again.
set -uo pipefail
PORT="${1:-8000}"
OPEN="${2:-}"

fail() { echo; echo "  X  $*"; echo; exit 1; }
ok()   { echo "  OK $*"; }

command -v adb >/dev/null || fail "adb is not installed.  Fix:  sudo apt install adb"
adb start-server >/dev/null 2>&1

# One line per device, skipping the header. Columns: serial, state.
mapfile -t DEVS < <(adb devices | tail -n +2 | awk 'NF>=2' | tr '\t' ' ')

if [ "${#DEVS[@]}" -eq 0 ]; then
  fail "No headset found over USB. Check, in this order:
     1. The cable carries data (many USB-C cables are charge-only). Try the cable that came with the Quest.
     2. Developer Mode is ON for this headset in the Meta Horizon phone app, and the headset was rebooted after.
     3. The headset is awake (put it on once)."
fi
if [ "${#DEVS[@]}" -gt 1 ]; then
  echo "More than one Android device is connected:"; printf '     %s\n' "${DEVS[@]}"
  fail "Unplug the others (or phones) and run again."
fi

SERIAL="${DEVS[0]%% *}"; STATE="${DEVS[0]#* }"     # state can be several words ("no permissions ...")
case "$STATE" in
  device) ok "Headset connected (serial $SERIAL)";;
  unauthorized)
    fail "The headset has not allowed this PC yet. Put the Quest on: you will see 'Allow USB debugging?'.
     Tick 'Always allow from this computer' and press Allow, then run this script again.";;
  *permissions*)
    fail "Linux is blocking access to the USB device. Fix once with:
       echo 'SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2833\", MODE=\"0666\", GROUP=\"plugdev\"' | sudo tee /etc/udev/rules.d/51-oculus.rules
       sudo udevadm control --reload-rules && sudo udevadm trigger
     then unplug and replug the cable.";;
  *) fail "Headset is in state '$STATE'. Unplug/replug, put the headset on, and run again.";;
esac

MODEL=$(adb shell getprop ro.product.model 2>/dev/null | tr -d '\r')
[ -n "$MODEL" ] && ok "Model: $MODEL"

adb reverse "tcp:$PORT" "tcp:$PORT" >/dev/null || fail "adb reverse failed."
ok "Quest localhost:$PORT  ->  this PC 127.0.0.1:$PORT"

if command -v curl >/dev/null; then
  if curl -s -o /dev/null -m 2 "http://127.0.0.1:$PORT/api/config"; then
    ok "twin_server.py is answering on port $PORT"
  else
    echo "  !  twin_server.py is not running on port $PORT yet. Start it, e.g.:"
    echo "       python twin_server.py --sim"
  fi
fi

if [ "$OPEN" = "open" ]; then
  adb shell am start -a android.intent.action.VIEW -d "http://localhost:$PORT" >/dev/null 2>&1 \
    && ok "Asked the Quest to open http://localhost:$PORT" \
    || echo "  !  Could not open the page remotely. Type the address into the Quest Browser instead."
fi

echo
echo "  In the Quest Browser open:   http://localhost:$PORT"
echo "  Then press 'Enter AR' in the side panel."
echo "  Debug console from this PC:  open Google Chrome here, go to chrome://inspect/#devices"
