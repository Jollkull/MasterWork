#!/usr/bin/env python3
import os
import pty
import time
import threading
import select
from typing import List, Optional, Tuple

SERVO_COUNT = 6
LINK_PATH = "/tmp/ttyARDUINO"


def clamp_deg(v: int) -> int:
    return 0 if v < 0 else 180 if v > 180 else v


def parse_move(line: str) -> Optional[Tuple[List[int], int]]:
    """
    Expected:
      G a1 a2 a3 a4 a5 a6 T
    Allows commas anywhere.
    Returns (angles[6], duration_ms) or None.
    """
    s = line.strip().replace(",", " ")
    if not s:
        return None

    parts = s.split()
    if len(parts) != 8:
        return None
    if parts[0].lower() != "g":
        return None

    try:
        nums = [int(x) for x in parts[1:]]
    except ValueError:
        return None

    angles = [clamp_deg(x) for x in nums[:6]]
    duration_ms = max(0, nums[6])
    return angles, duration_ms


class ServoEmu:
    def __init__(self, write_line):
        self._write_line = write_line
        self._lock = threading.Lock()
        self._cancel = threading.Event()
        self._move_thread: Optional[threading.Thread] = None

        self.current = [90] * SERVO_COUNT

    def state_str(self) -> str:
        with self._lock:
            return " ".join(map(str, self.current))

    def _move_worker(self, start: List[int], target: List[int], duration_ms: int):
        if duration_ms == 0:
            with self._lock:
                self.current = target[:]
            self._write_line("DONE")
            return

        t0 = time.time()
        dur = duration_ms / 1000.0
        step = 0.01  # 10ms

        while not self._cancel.is_set():
            t = (time.time() - t0) / dur
            if t >= 1.0:
                t = 1.0

            new_vals = []
            for i in range(SERVO_COUNT):
                v = start[i] + (target[i] - start[i]) * t
                new_vals.append(int(v + 0.5))

            with self._lock:
                self.current = new_vals

            if t >= 1.0:
                self._write_line("DONE")
                return

            time.sleep(step)

    def start_move(self, target: List[int], duration_ms: int):
        # cancel previous move
        if self._move_thread and self._move_thread.is_alive():
            self._cancel.set()
            self._move_thread.join(timeout=1.0)

        self._cancel.clear()

        with self._lock:
            start = self.current[:]

        self._write_line(f"OK: move {duration_ms} ms")
        self._move_thread = threading.Thread(
            target=self._move_worker, args=(start, target, duration_ms), daemon=True
        )
        self._move_thread.start()


def main():
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)

    # stable symlink
    try:
        if os.path.islink(LINK_PATH) or os.path.exists(LINK_PATH):
            os.remove(LINK_PATH)
        os.symlink(slave_name, LINK_PATH)
    except Exception as e:
        print(f"[WARN] can't create symlink {LINK_PATH}: {e}")

    # write helper: Arduino Serial.println style
    def write_line(s: str):
        os.write(master_fd, (s + "\r\n").encode("utf-8", errors="ignore"))

    emu = ServoEmu(write_line)

    print("=== Virtual Arduino Serial Emulator (PTY) ===")
    print(f"Slave device: {slave_name}")
    print(f"Symlink:      {LINK_PATH}")
    print("Protocol:     G a1 a2 a3 a4 a5 a6 T(ms)")
    print("Also:         S -> STATE")
    print("============================================")

    # send boot text (если клиент подключится позже — он может это не увидеть, это нормально)
    write_line("Ready.")
    write_line("Format: G a1 a2 a3 a4 a5 a6 T(ms)")
    write_line("Example: G 90 45 120 10 170 60 1500")

    buf = b""
    last_err_ts = 0.0

    while True:
        r, _, _ = select.select([master_fd], [], [], 0.2)
        if not r:
            continue

        chunk = os.read(master_fd, 4096)
        if not chunk:
            continue

        buf += chunk

        # split by \n OR \r (support CR, LF, CRLF)
        # easiest: normalize \r -> \n then split by \n
        buf = buf.replace(b"\r\n", b"\n").replace(b"\r", b"\n")

        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)

            # sanitize: drop NULs and decode
            raw = raw.replace(b"\x00", b"").strip()
            if not raw:
                # ключевое: НЕ отвечаем ошибками на пустые строки
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            # Optional: ignore pure terminal control garbage
            # (оставим минимум: если после strip осталась пустота — пропускаем)
            if not line:
                continue

            # Handle status
            if line.lower() == "s":
                write_line("STATE: " + emu.state_str())
                continue

            # Move command
            cmd = parse_move(line)
            if cmd is None:
                # anti-spam: не чаще 2 раз/сек
                now = time.time()
                if now - last_err_ts > 0.5:
                    write_line("ERR: use -> G a1 a2 a3 a4 a5 a6 T(ms)")
                    last_err_ts = now
                continue

            angles, duration_ms = cmd
            emu.start_move(angles, duration_ms)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[STOP]")
