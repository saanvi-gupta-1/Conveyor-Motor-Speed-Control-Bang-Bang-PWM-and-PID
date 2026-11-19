#!/usr/bin/env python3
"""Launcher for the Conveyor Motor Speed Control SCADA Web Dashboard."""

import sys
import time
import webbrowser
from pathlib import Path

# Add src to python path
src_dir = Path(__file__).parent / "src"
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))

from conveyor_control.server import start_server


def main():
    host = "127.0.0.1"
    port = 8050
    url = f"http://{host}:{port}"

    print("=" * 72)
    print("  CONVEYOR MOTOR SPEED CONTROL // SCADA CONTROL ROOM DASHBOARD")
    print("=" * 72)
    print(f"[*] Starting local HTTP simulation server on {url} ...")

    server = start_server(host=host, port=port)
    print(f"[+] Server online!")
    print(f"[+] Opening dashboard in browser: {url}")
    print("[+] Press Ctrl+C in this terminal window to stop the server.")
    print("=" * 72)

    try:
        webbrowser.open(url)
    except Exception:
        pass

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[*] Shutting down simulation server...")
        server.shutdown()
        print("[+] Goodbye!")
        return 0


if __name__ == "__main__":
    sys.exit(main())
