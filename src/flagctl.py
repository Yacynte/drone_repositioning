#!/usr/bin/env python3
from __future__ import annotations
import socket
import threading
import time

# HOST = "127.0.0.1"
# PORT = 9020

import argparse

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9020)
    return ap.parse_args()

args = parse_args()
HOST, PORT = args.host, args.port

class TcpCommandClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.sock: socket.socket | None = None
        self.lock = threading.Lock()

    def connect(self) -> None:
        with self.lock:
            if self.sock:
                return
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((self.host, self.port))
            self.sock = s
        print(f"[tcp] connected to {self.host}:{self.port}")

    def close(self) -> None:
        with self.lock:
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None

    def send_command(self, cmd: str) -> None:
        msg = (cmd + "\n").encode("utf-8")
        with self.lock:
            if not self.sock:
                raise RuntimeError("Not connected")
            self.sock.sendall(msg)


def main() -> None:
    client = TcpCommandClient(HOST, PORT)

    # Connect once at start (typical drone console behavior)
    try:
        client.connect()
    except Exception as e:
        print(f"[tcp] connect failed: {e}")
        print("Start your C++ server first, then run this.")
        return

    print("Drone CLI (TCP) ready.")
    print("Type commands: start | stop | pause | resume | rotation_only | translation_only | status | quit")

    try:
        while True:
            cmd = input(">>> ").strip()
            if not cmd:
                continue

            if cmd in ("quit", "exit"):
                break

            # OPTIONAL: local aliases
            aliases = {
                "continue": "resume",
                "rotation": "rotation_only",
                "translation": "translation_only",
                "play": "resume",
            }
            cmd_to_send = aliases.get(cmd, cmd)

            try:
                client.send_command(cmd_to_send)
                print(f"[sent] {cmd_to_send}")
            except Exception as e:
                print(f"[tcp] send failed: {e}")
                # quick reconnect attempt
                client.close()
                try:
                    time.sleep(0.2)
                    client.connect()
                    client.send_command(cmd_to_send)
                    print(f"[sent] {cmd_to_send} (after reconnect)")
                except Exception as e2:
                    print(f"[tcp] reconnect failed: {e2}")

    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        client.close()
        print("\nExiting.")


if __name__ == "__main__":
    main()
