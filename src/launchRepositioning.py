#!/usr/bin/env python3
import argparse
import subprocess
import signal
import sys
import time
import shutil
from typing import List
import os


def parse_args():
    ap = argparse.ArgumentParser("launchRepositioning.py")

    # Modes
    ap.add_argument("--mode", choices=["stream", "live"], default="stream")
    ap.add_argument("--rtsp", default="rtsp://10.116.88.38:8554/mystream")
    ap.add_argument("--camera", type=int, default=0)

    # Network
    ap.add_argument("--cmd_ip", default="127.0.0.1")
    ap.add_argument("--cmd_port", type=int, default=9020)

    # Message relay (receive_msg.py)
    ap.add_argument("--relay_listen_ip", default="127.0.0.1")
    ap.add_argument("--relay_listen_port", type=int, default=9010)

    # Where receive_msg forwards to (e.g., Unreal Engine server)
    ap.add_argument("--ue_ip", default="10.116.88.38")
    ap.add_argument("--ue_port", type=int, default=9001)

    # Paths
    ap.add_argument("--cpp_bin", default="./build/ImageMatcher")
    ap.add_argument("--flagctl", default="src/flagctl.py")
    ap.add_argument("--recv", default="src/receive_msg.py")
    ap.add_argument("--log", default="logs/")
    ap.add_argument("--target", default="target2.png")

    # Behavior switches
    ap.add_argument("--no_flagctl", action="store_true")
    ap.add_argument("--no_recv", action="store_true")
    ap.add_argument("--no_cpp", action="store_true")

    # NEW: open flagctl in separate terminal
    ap.add_argument("--flagctl_terminal", action="store_true", default=True,
                    help="Launch flagctl in a new terminal window/session")
    ap.add_argument("--no_flagctl_terminal", dest="flagctl_terminal", action="store_false",
                    help="Launch flagctl in the same terminal (as before)")

    # NEW: choose terminal method
    ap.add_argument("--terminal", choices=["auto", "gnome-terminal", "xterm", "tmux"],
                    default="auto",
                    help="How to open a separate terminal for flagctl")

    ap.add_argument("--startup_delay", type=float, default=0.25)

    return ap.parse_args()


def _popen(cmd: List[str]) -> subprocess.Popen:
    print("[launch] " + " ".join(cmd))
    return subprocess.Popen(cmd)

def tmux_has_session(name: str) -> bool:
    return subprocess.run(["tmux", "has-session", "-t", name],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode == 0

def tmux_kill_session(name: str) -> None:
    subprocess.run(["tmux", "kill-session", "-t", name],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)



def open_in_terminal(title: str, cmd: List[str], method: str = "auto") -> None:
    """
    Launch cmd in a separate terminal.
    This does NOT return a Popen handle (we don't manage interactive flagctl).
    """
    cmd_str = " ".join(subprocess.list2cmdline([c]) for c in cmd)  # safe-ish quoting

    def have(x: str) -> bool:
        return shutil.which(x) is not None

    chosen = method
    if method == "auto":
        if have("gnome-terminal"):
            chosen = "gnome-terminal"
        elif have("xterm"):
            chosen = "xterm"
        elif have("tmux"):
            chosen = "tmux"
        else:
            chosen = "none"

    if chosen == "gnome-terminal":
        # Keep terminal open after script ends: `exec bash`
        full = ["gnome-terminal", "--title", title, "--", "bash", "-lc", f"{cmd_str}; exec bash"]
        print("[launch-terminal] " + " ".join(full))
        subprocess.Popen(full)
        return

    if chosen == "xterm":
        full = ["xterm", "-T", title, "-e", "bash", "-lc", f"{cmd_str}; exec bash"]
        print("[launch-terminal] " + " ".join(full))
        subprocess.Popen(full)
        return

    if chosen == "tmux":
        session = "flagctl"
        if tmux_has_session(session):
            print(f"[launch-terminal] Session '{session}' exists, killing it...")
            tmux_kill_session(session)
        if os.environ.get("TMUX"):
            full = ["tmux", "new-window", "-n", title, f"bash -lc '{cmd_str}; exec bash'"]
            print("[launch-terminal] " + " ".join(full))
            subprocess.Popen(full)
        else:
            # Start a new detached tmux session for flagctl
            session = "flagctl"
            full = ["tmux", "new-session", "-d", "-s", session, f"bash -lc '{cmd_str}; exec bash'"]
            print("[launch-terminal] " + " ".join(full))
            subprocess.Popen(full)
            print(f"[launch-terminal] Started detached tmux session '{session}'. Attach with: tmux attach -t {session}")
        return


    # fallback: run in same terminal
    print("[launch-terminal] No terminal method available; running in current terminal.")
    subprocess.Popen(cmd)


def main():
    args = parse_args()
    procs: List[subprocess.Popen] = []

    # 1) Start ImageMatcher (C++)
    if not args.no_cpp:
        cpp_cmd = [
            args.cpp_bin,
            "--mode", args.mode,
            "--cmd_ip", args.cmd_ip,
            "--cmd_port", str(args.cmd_port),
            "--relay_ip", args.relay_listen_ip,
            "--relay_port", str(args.relay_listen_port),
            "--log", args.log,
            "--target", args.target,
        ]
        if args.mode == "stream":
            cpp_cmd += ["--rtsp", args.rtsp]
        else:
            cpp_cmd += ["--camera", str(args.camera)]

        procs.append(_popen(cpp_cmd))
        time.sleep(args.startup_delay)

    # 2) Start flagctl (interactive) in its own terminal
    if not args.no_flagctl:
        flag_cmd = [
            sys.executable, args.flagctl,
            "--host", args.cmd_ip,
            "--port", str(args.cmd_port),
        ]

        if args.flagctl_terminal:
            open_in_terminal("flagctl", flag_cmd, method=args.terminal)
        else:
            procs.append(_popen(flag_cmd))

        time.sleep(args.startup_delay)

    # 3) Start receive_msg.py (relay) in this terminal (background)
    if not args.no_recv:
        recv_cmd = [
            sys.executable, args.recv,
            "--listen_ip", args.relay_listen_ip,
            "--listen_port", str(args.relay_listen_port),
            "--ue_ip", args.ue_ip,
            "--ue_port", str(args.ue_port),
        ]
        procs.append(_popen(recv_cmd))
        time.sleep(args.startup_delay)

    def shutdown(*_):
        print("\n[launch] stopping...")
        for p in reversed(procs):
            try:
                p.terminate()
            except Exception:
                pass
        time.sleep(0.4)
        for p in reversed(procs):
            if p.poll() is None:
                try:
                    p.kill()
                except Exception:
                    pass

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # Keep launcher alive while managed children run (ImageMatcher + receive_msg)
    rc = 0
    try:
        while True:
            for p in procs:
                r = p.poll()
                if r is not None:
                    rc = r
                    raise SystemExit
            time.sleep(0.25)
    except SystemExit:
        shutdown()
        sys.exit(rc)


if __name__ == "__main__":
    main()
