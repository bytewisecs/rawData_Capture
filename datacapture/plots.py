"""Headless PNG export from the saved FFT arrays; never opens the radar."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


def save_fft_plots(npz_path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "Plotting requires Matplotlib. Install requirements-plot.txt in this project's .venv, "
            "or use --no-plots for numerical checks only.") from exc

    npz_path = Path(npz_path)
    output = npz_path.parent
    with np.load(str(npz_path), allow_pickle=False) as data:
        ranges = np.asarray(data["range_m"])
        velocity = np.asarray(data["velocity_m_s"])
        power = np.asarray(data["mean_range_doppler_power"])
        profile = np.asarray(data["mean_range_power"])
    if (ranges.ndim != 1 or velocity.ndim != 1
            or power.shape != (len(ranges), len(velocity)) or profile.shape != ranges.shape
            or len(ranges) < 2 or len(velocity) < 2
            or not np.all(np.diff(ranges) > 0) or not np.all(np.diff(velocity) > 0)):
        raise ValueError("FFT plot axes and saved arrays do not match")
    if any(not np.isfinite(a).all() for a in (ranges, velocity, power, profile)):
        raise ValueError("FFT plots require finite data")
    if np.any(power < 0) or np.any(profile < 0) or power.max() <= 0 or profile.max() <= 0:
        raise ValueError("FFT plots require nonnegative, nonzero power")
    report_path = output / "report.json"
    report = json.loads(report_path.read_text()) if report_path.is_file() else {}
    count = report.get("fft_frames")
    subtitle = ("Mean power over %s frames" % count) if count is not None else "Mean power"
    label = "Power relative to peak (dB)"

    def db(values):
        return 10 * np.log10(np.maximum(values / values.max(), 1e-12))

    def style(ax):
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)
        ax.grid(True, alpha=0.2)
        ax.set_axisbelow(True)

    def save(fig, name):
        path = output / name
        try:
            fig.savefig(str(path), dpi=160, facecolor="white")
        finally:
            plt.close(fig)
        return str(path.resolve())

    files = {}
    with plt.rc_context({"font.size": 11, "axes.titlesize": 15,
                         "axes.titlepad": 12, "figure.constrained_layout.use": True}):
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(ranges, db(profile), color="#126b99", linewidth=1.7)
        ax.set(xlabel="Range (m)", ylabel=label, xlim=(ranges[0], ranges[-1]),
               ylim=(-80, 3), title="Range FFT\n" + subtitle + "; TX/RX averaged")
        style(ax)
        files["range_fft"] = save(fig, "range_fft.png")

        fig, ax = plt.subplots(figsize=(10, 5))
        # Linear power is summed across range before converting to dB.
        ax.plot(velocity, db(power.sum(axis=0)), color="#9e5b19", linewidth=1.8, marker=".")
        ax.set(xlabel="Radial velocity (m/s, FFT sign convention)", ylabel=label,
               xlim=(velocity[0], velocity[-1]), ylim=(-80, 3),
               title="Doppler FFT\n" + subtitle + "; power summed over all range bins")
        style(ax)
        files["doppler_fft"] = save(fig, "doppler_fft.png")

        fig, ax = plt.subplots(figsize=(10, 6))
        mesh = ax.pcolormesh(velocity, ranges, db(power), shading="nearest",
                             cmap="viridis", vmin=-70, vmax=0, rasterized=True)
        ax.set(xlabel="Radial velocity (m/s, FFT sign convention)", ylabel="Range (m)",
               title="Range-Doppler FFT\n" + subtitle + "; TX/RX averaged")
        fig.colorbar(mesh, ax=ax, label=label)
        files["range_doppler"] = save(fig, "range_doppler.png")

    metadata = {"source_npz": str(npz_path.resolve()),
                "source_npz_sha256": hashlib.sha256(npz_path.read_bytes()).hexdigest(),
                "matplotlib_version": matplotlib.__version__, "images": files,
                "power_scale": "10*log10(linear power / plot peak); relative dB, not dBm",
                "doppler_profile": "Sum mean linear Range-Doppler power over all range bins",
                "range_fft_axis": "Full saved FFT range grid",
                "slow_time_dc_removed": report.get("slow_time_dc_removed", False)}
    (output / "plots.json").write_text(json.dumps(metadata, indent=2) + "\n")
    for name in files.values():
        print("PNG: " + name, flush=True)
    return files
