#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_physics_step_rows(csv_path: Path) -> tuple[list[int], list[float]]:
    frames: list[int] = []
    total_times: list[float] = []

    with csv_path.open(newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            if row["Label"] != "physics_step":
                continue
            frames.append(int(row["Frame index"]))
            total_times.append(float(row["Total time"]))

    if not frames:
        raise ValueError(f"No rows with label 'physics_step' were found in {csv_path}")

    return frames, total_times


def build_plot(frames: list[int], total_times: list[float], output_path: Path) -> None:
    plt.figure(figsize=(12, 6))
    plt.plot(frames, total_times, color="#1f77b4", linewidth=0.9)
    plt.title("physics_step Total Time Across Frames")
    plt.xlabel("Frame index")
    plt.ylabel("Total time")
    plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.6)
    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    plt.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot Total time for the physics_step label across all frames."
    )
    parser.add_argument(
        "csv_path",
        nargs="?",
        default="bandura.prof.csv",
        help="Path to the profiler CSV file.",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="bandura_profiling.png",
        help="Path to the output PNG image.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    csv_path = Path(args.csv_path)
    output_path = Path(args.output)

    frames, total_times = load_physics_step_rows(csv_path)
    build_plot(frames, total_times, output_path)


if __name__ == "__main__":
    main()
