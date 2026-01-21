#!/usr/bin/env python3
import argparse
import subprocess
import sys

# 用你已經 build 好、內建 rosbridge 的 Humble image
DEFAULT_IMAGE = "rosbridge:humble"
DEFAULT_NAME = "rosbridge_humble"
DEFAULT_PORT = 9090


def rm_container_if_exists(name: str) -> None:
    subprocess.run(
        ["docker", "rm", "-f", name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def image_exists(image: str) -> bool:
    r = subprocess.run(
        ["docker", "image", "inspect", image],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return r.returncode == 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Run rosbridge (ROS 2 Humble) in foreground. Ctrl+C to stop."
    )
    ap.add_argument(
        "--image",
        default=DEFAULT_IMAGE,
        help="Docker image (default: rosbridge:humble)",
    )
    ap.add_argument(
        "--name",
        default=DEFAULT_NAME,
        help="Container name",
    )
    ap.add_argument(
        "--port",
        type=int,
        default=DEFAULT_PORT,
        help="Host port to expose rosbridge (container 9090)",
    )
    args = ap.parse_args()

    # 確認 image 存在（避免忘記先 build）
    if not image_exists(args.image):
        print(f"[ERROR] Docker image not found: {args.image}", file=sys.stderr)
        print("[HINT] Build it first, e.g.:", file=sys.stderr)
        print("       docker build -t rosbridge:humble .", file=sys.stderr)
        return 1

    # 清掉同名容器，避免 conflict
    rm_container_if_exists(args.name)

    # Humble 專用 ROS 環境
    rosbridge_cmd = (
        "set -eo pipefail; "
        "source /opt/ros/humble/setup.bash; "
        "ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
    )

    docker_cmd = [
        "docker", "run",
        "--rm", "-it",
        "--name", args.name,
        "-p", f"{args.port}:9090",
        args.image,
        "bash", "-lc", rosbridge_cmd,
    ]

    print("[INFO] Starting rosbridge (ROS 2 Humble, foreground). Ctrl+C to stop.")
    print(f"[INFO] Image: {args.image}")
    print(f"[INFO] Port : {args.port} -> 9090")
    print()

    try:
        return subprocess.call(docker_cmd)
    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C received. Stopping...")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
