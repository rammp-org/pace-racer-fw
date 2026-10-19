#!/usr/bin/env python3
"""Reset (or query) a PACE board over its WebUSB vendor stream.

The WebUSB build owns the S3's only USB PHY, so esptool's DTR/RTS reset is
gone. The firmware's System module (0x20, see demo_link.hpp) answers
RESET (type 0x01) with OK and calls esp_restart() 300 ms later; INFO (0x02)
replies with a JSON blob. Uses the espp_ota package's transport + frame codec.

    tools/usb_reset.py                 # reset the one PACE board on USB (PID 0x0d41)
    tools/usb_reset.py --info          # print the device's JSON info instead
    tools/usb_reset.py --pid 0x0d32    # another product id
"""
import argparse
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "managed_components", "espp__ota", "python"))
from espp_ota import frame as sf  # noqa: E402
from espp_ota.transport import UsbVendorTransport  # noqa: E402

SYSTEM_MODULE = 0x20
SYS_RESET, SYS_INFO, SYS_OK, SYS_INFO_REPLY = 0x01, 0x02, 0x81, 0x82


def reset_via_vendor(pid=0x0d41, serial=None, timeout=3.0):
    """RESET over the vendor stream; True when the board acknowledged."""
    try:
        with UsbVendorTransport(pid=pid, serial=serial) as t:
            t.write(sf.build_frame(SYSTEM_MODULE, SYS_RESET))
            deadline = time.time() + timeout
            parser = sf.StreamParser()
            while time.time() < deadline:
                try:
                    data = t.read(sf.MAX_FRAME_SIZE, timeout_ms=500)
                except Exception:
                    continue
                for f in parser.feed(data):
                    if f.module == SYSTEM_MODULE and f.type == SYS_OK:
                        return True
    except Exception:
        return False
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pid", type=lambda x: int(x, 0), default=0x0d41)
    ap.add_argument("--serial", default=None)
    ap.add_argument("--info", action="store_true", help="query INFO instead of resetting")
    a = ap.parse_args()
    req = SYS_INFO if a.info else SYS_RESET
    with UsbVendorTransport(pid=a.pid, serial=a.serial) as t:
        t.write(sf.build_frame(SYSTEM_MODULE, req))
        deadline = time.time() + 3.0
        parser = sf.StreamParser()
        while time.time() < deadline:
            try:
                data = t.read(sf.MAX_FRAME_SIZE, timeout_ms=500)
            except Exception:
                continue
            for f in parser.feed(data):
                if f.module != SYSTEM_MODULE:
                    continue  # telemetry / log chatter
                if f.type == SYS_OK:
                    print("reset acknowledged; device restarts in ~300 ms")
                    return 0
                if f.type == SYS_INFO_REPLY:
                    print(json.dumps(json.loads(f.payload.decode()), indent=2))
                    return 0
        print("no reply from the System module", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
