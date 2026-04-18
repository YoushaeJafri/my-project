#!/usr/bin/env python3
import argparse
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial


def main() -> None:
    parser = argparse.ArgumentParser(description="Live plot: angle,accel,gyro from STM32 UART")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("--window", type=int, default=300, help="Number of samples on screen")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)

    xs = deque(maxlen=args.window)
    angle = deque(maxlen=args.window)
    accel = deque(maxlen=args.window)
    gyro = deque(maxlen=args.window)
    idx = 0

    fig, ax = plt.subplots()
    line_angle, = ax.plot([], [], label="angle")
    line_accel, = ax.plot([], [], label="accel")
    line_gyro, = ax.plot([], [], label="gyro")
    ax.set_xlabel("sample")
    ax.set_ylabel("value")
    ax.set_title("STM32 IMU Live Data")
    ax.grid(True)
    ax.legend(loc="upper left")

    def update(_frame):
        nonlocal idx

        for _ in range(10):
            raw = ser.readline().decode(errors="ignore").strip()
            if not raw:
                continue

            parts = raw.split(",")
            if len(parts) != 3:
                continue

            try:
                a, ac, g = map(float, parts)
            except ValueError:
                continue

            xs.append(idx)
            angle.append(a)
            accel.append(ac)
            gyro.append(g)
            idx += 1

        if len(xs) == 0:
            return line_angle, line_accel, line_gyro

        line_angle.set_data(xs, angle)
        line_accel.set_data(xs, accel)
        line_gyro.set_data(xs, gyro)
        ax.set_xlim(xs[0], xs[-1] if xs[-1] > xs[0] else xs[0] + 1)

        y_all = list(angle) + list(accel) + list(gyro)
        y_min = min(y_all)
        y_max = max(y_all)
        pad = max(1.0, 0.1 * (y_max - y_min if y_max != y_min else 1.0))
        ax.set_ylim(y_min - pad, y_max + pad)

        return line_angle, line_accel, line_gyro

    ani = FuncAnimation(fig, update, interval=50, blit=False)

    try:
        plt.show()
    finally:
        _ = ani
        ser.close()


if __name__ == "__main__":
    main()
