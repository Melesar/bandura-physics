#!/usr/bin/env python3

import argparse
import csv
import subprocess
import sys
from pathlib import Path

import matplotlib
from matplotlib.backends.backend_pdf import PdfPages

matplotlib.use("Agg")
import matplotlib.pyplot as plt

PLOT_LABELS = [
    "bnd_simulate",
    "integrate_bodies",
    "contacts_generate",
    "contacts_resolve",
]


def load_rows_for_label(
    csv_path: Path,
    label: str,
) -> tuple[list[int], list[float], list[int], list[int]]:
    frames: list[int] = []
    total_times: list[float] = []
    body_counts: list[int] = []
    contacts_counts: list[int] = []

    with csv_path.open(newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            if row["Label"] != label:
                continue
            frames.append(int(row["Frame index"]))
            total_times.append(float(row["Total time"]))
            body_counts.append(int(row["Body count"]))
            contacts_counts.append(int(row["Contacts count"]))

    if not frames:
        raise ValueError(f"No rows with label '{label}' were found in {csv_path}")

    return frames, total_times, body_counts, contacts_counts


def build_plot_figure(
    label: str,
    frames: list[int],
    total_times: list[float],
    body_counts: list[int],
    contacts_counts: list[int],
) -> plt.Figure:
    figure, axis_time = plt.subplots(figsize=(12, 6))
    axis_counts = axis_time.twinx()

    (time_line,) = axis_time.plot(
        frames,
        total_times,
        color="#1f77b4",
        linewidth=0.9,
        label="Total time",
    )
    (body_line,) = axis_counts.plot(
        frames,
        body_counts,
        color="orange",
        linewidth=0.8,
        linestyle="--",
        label="Body count",
    )
    (contacts_line,) = axis_counts.plot(
        frames,
        contacts_counts,
        color="green",
        linewidth=0.8,
        linestyle="--",
        label="Contacts count",
    )

    axis_time.set_title(label)
    axis_time.set_xlabel("Frame")
    axis_time.set_ylabel("Milliseconds")
    axis_counts.set_ylabel("Count")
    axis_time.grid(True, linestyle="--", linewidth=0.5, alpha=0.6)
    axis_time.legend(
        [time_line, body_line, contacts_line],
        ["Total time", "Body count", "Contacts count"],
        loc="upper right",
    )

    figure.tight_layout()
    return figure


def write_pdf_document(csv_path: Path, output_path: Path) -> None:
    with PdfPages(output_path) as pdf:
        for label in PLOT_LABELS:
            frames, total_times, body_counts, contacts_counts = load_rows_for_label(
                csv_path,
                label,
            )
            figure = build_plot_figure(
                label,
                frames,
                total_times,
                body_counts,
                contacts_counts,
            )
            pdf.savefig(figure)
            plt.close(figure)


def open_output_document(output_path: Path) -> None:
    if sys.platform == "darwin":
        command = ["open", str(output_path)]
    elif sys.platform.startswith("linux"):
        command = ["xdg-open", str(output_path)]
    else:
        raise RuntimeError(
            "Opening the output document is only supported on macOS and Linux"
        )

    subprocess.run(command, check=True)


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
        default="bandura_profiling.pdf",
        help="Path to the output plot image or PDF.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    csv_path = Path(args.csv_path)
    output_path = Path(args.output)

    write_pdf_document(csv_path, output_path)
    open_output_document(output_path)


if __name__ == "__main__":
    main()
