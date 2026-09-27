#!/usr/bin/env python3
"""Run a small hardware test against ros2bot_master_lib."""

import argparse
import time

from ros2bot_master_lib import Ros2botMasterDriver


READ_FUNCTIONS = {
    "get_version",
    "get_battery_voltage",
    "get_motion_data",
    "get_motor_encoder",
    "get_accelerometer_data",
    "get_gyroscope_data",
    "get_magnetometer_data",
    "get_imu_attitude_data",
}


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Test a read function or set_beep on a connected master board. "
            "Opening the driver enables UART servo torque."
        )
    )
    parser.add_argument(
        "function",
        choices=sorted(READ_FUNCTIONS | {"set_beep"}),
        help="library function to test",
    )
    parser.add_argument("--port", default="/dev/r2bserial", help="serial port")
    parser.add_argument("--bot-type", type=int, default=1, help="robot type (default: 1)")
    parser.add_argument(
        "--value",
        type=int,
        help="argument for set_beep (milliseconds; for example, 50)",
    )
    parser.add_argument(
        "--wait",
        type=float,
        default=0.2,
        help="seconds to wait for telemetry (default: 0.2)",
    )
    parser.add_argument("--debug", action="store_true", help="enable driver debug output")
    args = parser.parse_args()

    if args.wait < 0:
        parser.error("--wait must be zero or greater")
    if args.function == "set_beep" and args.value is None:
        parser.error("set_beep requires --value")

    driver = None
    try:
        driver = Ros2botMasterDriver(bot_type=args.bot_type, com=args.port, debug=args.debug)
        if args.function == "set_beep":
            driver.set_beep(args.value)
            result = "beep command sent"
        else:
            driver.create_receive_thread()
            time.sleep(args.wait)
            result = getattr(driver, args.function)()
        print("{}: {}".format(args.function, result))
    except Exception as exc:
        parser.exit(1, "Test failed: {}\n".format(exc))
    finally:
        if driver is not None:
            driver.close()


if __name__ == "__main__":
    main()