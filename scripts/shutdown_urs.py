#!/usr/bin/env python3
"""Controlled CB3 / PolyScope 5 shutdown through the UR Dashboard (29999)."""

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
import re
import socket
import sys
import time


class DashboardClient:
    def __init__(self, host, port, timeout):
        self.host, self.port, self.timeout = host, port, timeout
        self.sock = None
        self.file = None

    def __enter__(self):
        try:
            self.sock = socket.create_connection((self.host, self.port), self.timeout)
            self.file = self.sock.makefile("rwb", buffering=0)
            banner = self._readline()
            if "dashboard server" not in banner.lower():
                raise RuntimeError(f"Unexpected Dashboard greeting: {banner}")
            return self
        except Exception:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, *_args):
        if self.file is not None:
            self.file.close()
        if self.sock is not None:
            self.sock.close()

    def _readline(self):
        raw = self.file.readline(4096)
        if not raw or not raw.endswith(b"\n"):
            raise RuntimeError("Dashboard disconnected or returned an incomplete reply")
        return raw.decode("utf-8", errors="replace").strip()

    def query(self, command):
        self.sock.sendall((command + "\n").encode("utf-8"))
        return self._readline()


def shutdown_arm(host, port=29999, timeout=3.0, state_timeout=30.0,
                 client_factory=DashboardClient, report=None):
    """Abort this arm on any failed step; never retry an ambiguous shutdown."""
    result = {"host": host, "shutdown_accepted": False, "steps": [], "error": ""}

    def query(client, command, expected=None):
        answer = client.query(command)
        result["steps"].append({"command": command, "answer": answer})
        if report is not None:
            report(f"{host}: {command} -> {answer}")
        if expected is not None and answer.casefold() != expected.casefold():
            raise RuntimeError(f"{command}: expected {expected!r}, received {answer!r}")
        return answer

    def wait_for(client, command, expected):
        deadline = time.monotonic() + state_timeout
        while True:
            answer = query(client, command)
            # programState can append the loaded program filename.
            if answer == expected or answer.startswith(expected + " "):
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(f"Timeout waiting for {expected}: {answer}")
            time.sleep(0.2)

    try:
        with client_factory(host, port, timeout) as client:
            version = query(client, "PolyscopeVersion")
            match = re.match(r"(?:URSoftware )?(\d+)\.", version)
            if match is None or int(match.group(1)) not in (3, 5):
                raise RuntimeError(f"Unsupported PolyScope version: {version}")
            if int(match.group(1)) == 5:
                # Available from 5.6; an unknown answer also refuses shutdown.
                query(client, "is in remote control", "true")
            mode = query(client, "robotmode")
            if mode != "Robotmode: POWER_OFF":
                query(client, "stop", "Stopped")
                wait_for(client, "programState", "STOPPED")
                query(client, "power off", "Powering off")
                wait_for(client, "robotmode", "Robotmode: POWER_OFF")
            else:
                wait_for(client, "programState", "STOPPED")
            query(client, "shutdown", "Shutting down")
            result["shutdown_accepted"] = True
    except (OSError, RuntimeError) as exc:
        result["error"] = str(exc)
        if report is not None:
            report(f"{host}: ERROR: {exc}")
    return result


def positive_timeout(value):
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError("Timeout must be finite and positive")
    return parsed


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", action="append", required=True,
                        help="Selected UR Dashboard host/IP; repeat for multiple arms")
    parser.add_argument("--port", type=int, default=29999)
    parser.add_argument("--timeout", type=positive_timeout, default=3.0)
    parser.add_argument("--state-timeout", type=positive_timeout, default=30.0)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    hosts = list(dict.fromkeys(args.host))
    report = lambda line: print(line, file=sys.stderr, flush=True)
    with ThreadPoolExecutor(max_workers=min(len(hosts), 8)) as executor:
        arms = list(executor.map(
            lambda host: shutdown_arm(host, args.port, args.timeout,
                                      args.state_timeout, report=report), hosts))
    payload = {"ok": all(arm["shutdown_accepted"] for arm in arms), "arms": arms}
    if args.json:
        print(json.dumps(payload, indent=2), flush=True)
    else:
        for arm in arms:
            print(f"{arm['host']}: " + ("Shutdown accepted" if arm["shutdown_accepted"]
                                      else f"Failed: {arm['error']}"))
    return 0 if payload["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
