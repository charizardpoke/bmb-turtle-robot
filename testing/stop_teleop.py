#!/usr/bin/env python3

import subprocess


process_names = [
    "twist_stamper",
    "teleop_node",
    "joy_node",
]


def run_command(command):
    print(f"Running: {command}")

    result = subprocess.run(
        command,
        shell=True,
        text=True,
        capture_output=True
    )

    if result.stdout:
        print(result.stdout)

    if result.stderr:
        print(result.stderr)

    return result.returncode


def main():
    print("Stopping teleop/control nodes...")

    for name in process_names:
        run_command(f"pkill -f {name}")

    print("Done.")


if __name__ == "__main__":
    main()