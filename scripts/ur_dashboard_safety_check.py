#!/usr/bin/env python3
"""Check and optionally clear UR dashboard safety popups before driver startup."""

import argparse
import json
import socket
import sys
import time


BLOCKING_SAFETY_TOKENS = (
    "PROTECTIVE_STOP",
    "ROBOT_EMERGENCY_STOP",
    "SYSTEM_EMERGENCY_STOP",
    "SAFEGUARD_STOP",
    "FAULT",
    "VIOLATION",
    "RECOVERY",
)


QUERY_COMMANDS = (
    "robotmode",
    "safetystatus",
    "safetymode",
    "programState",
    "is in remote control",
    "get loaded program",
)


CLEAR_COMMANDS = (
    "close safety popup",
    "close popup",
    "unlock protective stop",
    "close popup",
)


class DashboardClient:
    def __init__(self, host, port, timeout):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None
        self.file = None
        self.banner = ""

    def __enter__(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)
        self.file = self.sock.makefile("rwb", buffering=0)
        self.banner = self._readline()
        return self

    def __exit__(self, _exc_type, _exc, _tb):
        if self.file is not None:
            self.file.close()
        if self.sock is not None:
            self.sock.close()

    def _readline(self):
        raw = self.file.readline()
        if not raw:
            return ""
        return raw.decode("utf-8", errors="replace").strip()

    def query(self, command):
        self.file.write((command + "\n").encode("utf-8"))
        return self._readline()


def is_blocking_safety_state(lines):
    text = "\n".join(lines).upper()
    return any(token in text for token in BLOCKING_SAFETY_TOKENS)


def query_dashboard(host, port, timeout, clear=False, settle_sec=0.5):
    result = {
        "host": host,
        "reachable": False,
        "blocked": False,
        "banner": "",
        "queries": [],
        "clear": [],
        "error": "",
        "blocking_reasons": [],
        "remote_control": None,
    }
    try:
        with DashboardClient(host, port, timeout) as dashboard:
            result["reachable"] = True
            result["banner"] = dashboard.banner
            query_lines = []
            for command in QUERY_COMMANDS:
                answer = dashboard.query(command)
                result["queries"].append({"command": command, "answer": answer})
                query_lines.append(answer)
            if is_blocking_safety_state(query_lines):
                result["blocked"] = True
                result["blocking_reasons"].append("Robot in Protective Stop or Safety Stop")
            for query in result["queries"]:
                if query["command"] == "is in remote control":
                    answer = str(query["answer"]).strip().lower()
                    if answer in ("true", "remote control: true", "is in remote control: true"):
                        result["remote_control"] = True
                    elif answer in ("false", "remote control: false", "is in remote control: false"):
                        result["remote_control"] = False
                        result["blocked"] = True
                        result["blocking_reasons"].append("Robot not in remote control mode")
                    break

            if clear and result["blocked"]:
                for command in CLEAR_COMMANDS:
                    answer = dashboard.query(command)
                    result["clear"].append({"command": command, "answer": answer})
                    time.sleep(0.1)
                time.sleep(settle_sec)
                result["queries_after_clear"] = []
                after_lines = []
                for command in QUERY_COMMANDS:
                    answer = dashboard.query(command)
                    result["queries_after_clear"].append({"command": command, "answer": answer})
                    after_lines.append(answer)
                result["blocking_reasons"] = []
                result["blocked"] = False
                if is_blocking_safety_state(after_lines):
                    result["blocked"] = True
                    result["blocking_reasons"].append("Robot in Protective Stop or Safety Stop")
                for query in result["queries_after_clear"]:
                    if query["command"] == "is in remote control":
                        answer = str(query["answer"]).strip().lower()
                        if answer in ("true", "remote control: true", "is in remote control: true"):
                            result["remote_control"] = True
                        elif answer in ("false", "remote control: false", "is in remote control: false"):
                            result["remote_control"] = False
                            result["blocked"] = True
                            result["blocking_reasons"].append("Robot not in remote control mode")
                        break
    except OSError as exc:
        result["error"] = str(exc)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", action="append", required=True, help="UR dashboard host/IP")
    parser.add_argument("--port", type=int, default=29999)
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--clear", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    arms = [
        query_dashboard(host, args.port, args.timeout, clear=args.clear)
        for host in args.host
    ]
    payload = {
        "ok": all(arm["reachable"] and not arm["blocked"] for arm in arms),
        "clear_requested": bool(args.clear),
        "arms": arms,
        "note": (
            "UR Dashboard does not expose the full pendant popup body on all "
            "PolyScope versions; query answers are shown as the available popup/safety text."
        ),
    }

    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        for arm in arms:
            print(f"{arm['host']}: reachable={arm['reachable']} blocked={arm['blocked']}")
            if arm["error"]:
                print(f"  error: {arm['error']}")
            for reason in arm.get("blocking_reasons", []):
                print(f"  diagnosis: {reason}")
            for query in arm["queries"]:
                print(f"  {query['command']}: {query['answer']}")
            for command in arm["clear"]:
                print(f"  clear {command['command']}: {command['answer']}")

    if any(not arm["reachable"] for arm in arms):
        return 1
    if any(arm["blocked"] for arm in arms):
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
