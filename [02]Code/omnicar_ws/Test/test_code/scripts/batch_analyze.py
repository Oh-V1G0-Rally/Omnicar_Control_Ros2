from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the path following analyzer on every matching bag folder.")
    parser.add_argument("--bags-root", required=True, help="Workspace bags root")
    parser.add_argument("--analysis-script", required=True, help="Path to analyze_path_following_bag.py")
    parser.add_argument("--config", required=True, help="Analysis config YAML")
    parser.add_argument("--output-root", required=True, help="Results root")
    parser.add_argument("--path-file", required=True, help="Reference path file")
    parser.add_argument("--controller-name", required=True)
    parser.add_argument("--controller-version", required=True)
    parser.add_argument("--frame", required=True, choices=["MAP", "ODOM"])
    parser.add_argument("--path-name", required=True)
    parser.add_argument("--prefix", default="PF_", help="Only analyze folders starting with this prefix")
    parser.add_argument("--debug-topic", help="Optional debug topic override passed to each analysis run")
    parser.add_argument("--force", action="store_true", help="Allow overwriting existing test result folders")
    parser.add_argument("--no-global-update", action="store_true", help="Skip global summary CSV updates")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    bags_root = Path(args.bags_root).resolve()
    script = Path(args.analysis_script).resolve()

    for bag_dir in sorted(path for path in bags_root.iterdir() if path.is_dir() and path.name.startswith(args.prefix)):
        command = [
            sys.executable,
            str(script),
            "--bag",
            str(bag_dir),
            "--test-id",
            bag_dir.name,
            "--path-name",
            args.path_name,
            "--frame",
            args.frame,
            "--controller-name",
            args.controller_name,
            "--controller-version",
            args.controller_version,
            "--path-file",
            args.path_file,
            "--config",
            args.config,
            "--output-root",
            args.output_root,
        ]
        if args.debug_topic:
            command.extend(["--debug-topic", args.debug_topic])
        if args.force:
            command.append("--force")
        if args.no_global_update:
            command.append("--no-global-update")
        subprocess.run(command, check=True)


if __name__ == "__main__":
    main()
