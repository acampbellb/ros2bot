#!/usr/bin/env python3
"""Run a small hardware test against ros2bot_speach_lib."""

import argparse

from ros2bot_speach_lib import Ros2botSpeachDriver


def main():
    parser = argparse.ArgumentParser(
        description="Test speech_read or send a numeric command with void_write."
    )
    parser.add_argument(
        "function",
        choices=("speech_read", "void_write"),
        help="library function to test",
    )
    parser.add_argument("--port", default="/dev/r2bspeach", help="serial port")
    parser.add_argument(
        "--value",
        type=int,
        help="argument for void_write; must be between 0 and 999",
    )
    args = parser.parse_args()

    if args.function == "void_write" and args.value is None:
        parser.error("void_write requires --value")

    driver = None
    try:
        driver = Ros2botSpeachDriver(com=args.port)
        if args.function == "void_write":
            driver.void_write(args.value)
            result = "command sent"
        else:
            result = driver.speech_read()
            if result == 999:
                print("No complete speech response is currently available.")
        print("{}: {}".format(args.function, result))
    except Exception as exc:
        parser.exit(1, "Test failed: {}\n".format(exc))
    finally:
        if driver is not None:
            driver.close()


if __name__ == "__main__":
    main()