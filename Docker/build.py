#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


DEFAULT_TAG = "rosbridge:humble"
DEFAULT_BASE = "ros:humble-ros-base"
DEFAULT_DOCKERFILE = "Dockerfile.rosbridge-humble"


DOCKERFILE_TEMPLATE = """\
FROM {base}

RUN apt-get update && apt-get install -y --no-install-recommends \\
    ros-humble-rosbridge-server \\
 && rm -rf /var/lib/apt/lists/*

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml"]
"""


def run(cmd: list[str], check: bool = True) -> subprocess.CompletedProcess:
    print(">", " ".join(cmd))
    return subprocess.run(cmd, check=check)


def docker_available() -> bool:
    try:
        run(["docker", "version"], check=True)
        return True
    except subprocess.CalledProcessError:
        return False
    except FileNotFoundError:
        return False


def ensure_dockerfile(dockerfile_path: Path, base: str, force: bool) -> None:
    if dockerfile_path.exists() and not force:
        print(f"[INFO] Using existing Dockerfile: {dockerfile_path}")
        return

    dockerfile_path.parent.mkdir(parents=True, exist_ok=True)
    content = DOCKERFILE_TEMPLATE.format(base=base)
    dockerfile_path.write_text(content, encoding="utf-8")
    print(f"[OK] Wrote Dockerfile: {dockerfile_path}")


def main() -> int:
    ap = argparse.ArgumentParser(description="Build rosbridge:humble Docker image (preinstalled rosbridge_server).")
    ap.add_argument(
        "--tag",
        default=DEFAULT_TAG,
        help=f"Image tag to build (default: {DEFAULT_TAG})",
    )
    ap.add_argument(
        "--base",
        default=DEFAULT_BASE,
        help=f"Base image (default: {DEFAULT_BASE})",
    )
    ap.add_argument(
        "--context",
        default=".",
        help="Docker build context directory (default: current directory)",
    )
    ap.add_argument(
        "--dockerfile-dir",
        default="docker",
        help="Directory to place/find the Dockerfile (default: docker/)",
    )
    ap.add_argument(
        "--dockerfile-name",
        default=DEFAULT_DOCKERFILE,
        help=f"Dockerfile filename (default: {DEFAULT_DOCKERFILE})",
    )
    ap.add_argument(
        "--force-write-dockerfile",
        action="store_true",
        help="Overwrite Dockerfile even if it exists",
    )
    ap.add_argument("--no-cache", action="store_true", help="Pass --no-cache to docker build")
    ap.add_argument("--pull", action="store_true", help="Pass --pull to docker build")
    args = ap.parse_args()

    if not docker_available():
        print("[ERROR] Docker is not available. Is Docker Desktop running?", file=sys.stderr)
        return 1

    context_dir = Path(args.context).resolve()
    dockerfile_path = (Path(args.dockerfile_dir) / args.dockerfile_name).resolve()

    if not context_dir.exists():
        print(f"[ERROR] Build context does not exist: {context_dir}", file=sys.stderr)
        return 1

    # Ensure Dockerfile exists (create if missing)
    ensure_dockerfile(dockerfile_path, base=args.base, force=args.force_write_dockerfile)

    build_cmd = [
        "docker", "build",
        "-t", args.tag,
        "-f", str(dockerfile_path),
    ]

    if args.no_cache:
        build_cmd.append("--no-cache")
    if args.pull:
        build_cmd.append("--pull")

    build_cmd.append(str(context_dir))

    print("[INFO] Building image...")
    print(f"[INFO] Tag:       {args.tag}")
    print(f"[INFO] Base:      {args.base}")
    print(f"[INFO] Context:   {context_dir}")
    print(f"[INFO] Dockerfile:{dockerfile_path}")
    run(build_cmd, check=True)

    print(f"[OK] Built image: {args.tag}")
    print("[INFO] Run it like:")
    print("  docker rm -f rosbridge_humble 2>/dev/null || true")
    print(f"  docker run --rm -it --name rosbridge_humble -p 9090:9090 {args.tag}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
