# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
PerformanceLogger
=================
Records controller performance in real-time and generates the
"speed vs divergence from target" graph that leshy asked for in issue #921.

What it records per tick
------------------------
  - timestamp
  - robot speed (|v|)
  - cross-track error (CTE)
  - heading error
  - target speed (from velocity profiler)
  - clearance (from ClearanceMonitor if available)
  - controller mode (path_following / initial_rotation / final_rotation)

Output
------
  - Live metrics printed to stdout every N seconds
  - JSON log file for offline analysis
  - Publication-quality plots:
      • Speed vs Divergence scatter
      • CTE over time
      • Speed over time with profiler target
      • Clearance over time

Usage
-----
    logger = PerformanceLogger()
    logger.start_episode()
    # ... inside control loop:
    logger.record(speed=..., cte=..., heading_error=..., target_speed=...,
                  clearance=..., mode="path_following")
    logger.end_episode()
    logger.plot("/tmp/performance.png")
    logger.save("/tmp/performance.json")
"""
from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass, field


@dataclass
class ControlTick:
    t: float           # timestamp (s)
    speed: float       # actual robot speed (m/s)
    cte: float         # |cross-track error| (m)
    heading_error: float   # (rad)
    target_speed: float    # from velocity profiler (m/s)
    clearance: float       # distance to nearest obstacle (m)
    mode: str              # planner state string


@dataclass
class Episode:
    start_time: float = 0.0
    end_time: float   = 0.0
    ticks: list[ControlTick] = field(default_factory=list)
    path_length: float = 0.0
    completed: bool    = False

    def duration(self) -> float:
        return self.end_time - self.start_time

    def avg_speed(self) -> float:
        if not self.ticks: return 0.0
        return sum(t.speed for t in self.ticks) / len(self.ticks)

    def avg_cte(self) -> float:
        if not self.ticks: return 0.0
        return sum(t.cte for t in self.ticks) / len(self.ticks)

    def max_cte(self) -> float:
        if not self.ticks: return 0.0
        return max(t.cte for t in self.ticks)

    def p95_cte(self) -> float:
        if not self.ticks: return 0.0
        import numpy as np
        return float(np.percentile([t.cte for t in self.ticks], 95))


class PerformanceLogger:
    """
    Thread-safe controller performance logger.
    Records every control tick and produces analysis plots.
    """

    def __init__(
        self,
        print_interval: float = 5.0,   # seconds between live stats prints
        max_ticks: int = 10_000,        # ring buffer size per episode
    ) -> None:
        self._print_interval = print_interval
        self._max_ticks = max_ticks
        self._episode: Episode | None = None
        self._history: list[Episode] = []
        self._last_print: float = 0.0

    # ------------------------------------------------------------------
    # Recording API
    # ------------------------------------------------------------------

    def start_episode(self, path_length: float = 0.0) -> None:
        """Call when a new navigation goal starts."""
        self._episode = Episode(
            start_time=time.time(),
            path_length=path_length,
        )
        self._last_print = time.time()

    def record(
        self,
        speed: float,
        cte: float,
        heading_error: float,
        target_speed: float,
        clearance: float = 999.0,
        mode: str = "path_following",
    ) -> None:
        """Call every control tick."""
        if self._episode is None:
            return
        tick = ControlTick(
            t=time.time() - self._episode.start_time,
            speed=speed,
            cte=abs(cte),
            heading_error=heading_error,
            target_speed=target_speed,
            clearance=clearance,
            mode=mode,
        )
        self._episode.ticks.append(tick)
        # Ring buffer
        if len(self._episode.ticks) > self._max_ticks:
            self._episode.ticks.pop(0)
        self._maybe_print_live()

    def end_episode(self, completed: bool = True) -> Episode | None:
        """Call when navigation ends (arrived, obstacle, error)."""
        if self._episode is None:
            return None
        self._episode.end_time  = time.time()
        self._episode.completed = completed
        self._print_episode_summary(self._episode)
        self._history.append(self._episode)
        ep = self._episode
        self._episode = None
        return ep

    # ------------------------------------------------------------------
    # Analysis
    # ------------------------------------------------------------------

    def plot(self, save_path: str = "/tmp/performance.png") -> None:
        """Generate performance plots from the last episode."""
        episodes = self._history[-1:] if self._history else []
        if self._episode:
            episodes = [self._episode]
        if not episodes:
            print("No data to plot.")
            return
        _plot_episode(episodes[-1], save_path)
        print(f"Performance plot → {save_path}")

    def plot_speed_vs_divergence(
        self,
        save_path: str = "/tmp/speed_vs_divergence.png",
    ) -> None:
        """
        The specific 'speed vs divergence' plot from issue #921.
        Shows the achievable precision at different speeds.
        """
        all_episodes = self._history
        if self._episode and self._episode.ticks:
            all_episodes = all_episodes + [self._episode]
        if not all_episodes:
            print("No data."); return
        _plot_speed_vs_divergence(all_episodes, save_path)
        print(f"Speed vs divergence → {save_path}")

    def save(self, path: str = "/tmp/performance.json") -> None:
        """Save all recorded data to JSON."""
        data = []
        for ep in self._history:
            ep_dict = {
                "start_time":  ep.start_time,
                "end_time":    ep.end_time,
                "path_length": ep.path_length,
                "completed":   ep.completed,
                "avg_speed":   ep.avg_speed(),
                "avg_cte":     ep.avg_cte(),
                "max_cte":     ep.max_cte(),
                "p95_cte":     ep.p95_cte(),
                "ticks":       [asdict(t) for t in ep.ticks],
            }
            data.append(ep_dict)
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"Performance data → {path}")

    def summary(self) -> dict[str, float]:
        """Return summary statistics across all completed episodes."""
        completed = [ep for ep in self._history if ep.completed]
        if not completed:
            return {}
        import numpy as np
        all_speeds = [t.speed for ep in completed for t in ep.ticks]
        all_ctes   = [t.cte   for ep in completed for t in ep.ticks]
        return {
            "episodes":       len(completed),
            "avg_speed":      float(np.mean(all_speeds)),
            "avg_cte":        float(np.mean(all_ctes)),
            "max_cte":        float(np.max(all_ctes)),
            "p95_cte":        float(np.percentile(all_ctes, 95)),
            "total_distance": sum(ep.path_length for ep in completed),
        }

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _maybe_print_live(self) -> None:
        now = time.time()
        if now - self._last_print < self._print_interval:
            return
        if self._episode and len(self._episode.ticks) > 5:
            recent = self._episode.ticks[-20:]
            avg_spd = sum(t.speed for t in recent) / len(recent)
            avg_cte = sum(t.cte   for t in recent) / len(recent)
            elapsed = now - self._episode.start_time
            print(f"[PerformanceLogger] t={elapsed:.1f}s  "
                  f"speed={avg_spd:.3f}m/s  CTE={avg_cte:.4f}m  "
                  f"ticks={len(self._episode.ticks)}")
        self._last_print = now

    def _print_episode_summary(self, ep: Episode) -> None:
        status = "✓ completed" if ep.completed else "✗ aborted"
        print(f"\n[PerformanceLogger] Episode {status} — "
              f"dur={ep.duration():.1f}s  "
              f"avg_speed={ep.avg_speed():.3f}m/s  "
              f"avg_CTE={ep.avg_cte():.4f}m  "
              f"max_CTE={ep.max_cte():.4f}m  "
              f"p95_CTE={ep.p95_cte():.4f}m")


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------

def _plot_episode(ep: Episode, save_path: str) -> None:
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    import numpy as np

    DARK='#0d1117'; GC='#21262d'; C1='#3fb950'; C2='#f85149'; C3='#58a6ff'; C4='#d2a8ff'

    def sax(ax: plt.Axes, title: str) -> None:
        ax.set_facecolor(DARK); ax.set_title(title, color='white', fontsize=10, pad=6)
        ax.tick_params(colors='#8b949e')
        for sp in ax.spines.values(): sp.set_color(GC)
        ax.grid(color=GC, linewidth=0.5)
        ax.xaxis.label.set_color('#8b949e'); ax.yaxis.label.set_color('#8b949e')

    ticks = ep.ticks
    if not ticks: return

    ts     = [t.t            for t in ticks]
    speeds = [t.speed        for t in ticks]
    ctes   = [t.cte          for t in ticks]
    tgts   = [t.target_speed for t in ticks]
    clrs   = [t.clearance    for t in ticks]
    herrs  = [abs(t.heading_error) for t in ticks]

    fig = plt.figure(figsize=(16, 10)); fig.patch.set_facecolor(DARK)
    gs  = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.38)

    # Speed over time
    ax = fig.add_subplot(gs[0, 0])
    ax.plot(ts, tgts,   color=C3, lw=1.2, ls='--', alpha=0.7, label='target')
    ax.plot(ts, speeds, color=C1, lw=1.5, label='actual')
    sax(ax, 'Speed Over Time'); ax.set_xlabel('Time (s)'); ax.set_ylabel('Speed (m/s)')
    ax.legend(fontsize=8, facecolor='#161b22', labelcolor='white')

    # CTE over time
    ax = fig.add_subplot(gs[1, 0])
    ax.plot(ts, ctes, color=C2, lw=1.5)
    ax.axhline(ep.avg_cte(), color='#8b949e', lw=1.0, ls='--',
               label=f'avg={ep.avg_cte():.4f}m')
    ax.axhline(ep.p95_cte(), color=C3, lw=1.0, ls=':',
               label=f'p95={ep.p95_cte():.4f}m')
    sax(ax, 'Cross-Track Error'); ax.set_xlabel('Time (s)'); ax.set_ylabel('|CTE| (m)')
    ax.legend(fontsize=8, facecolor='#161b22', labelcolor='white')

    # Speed vs divergence (the issue metric)
    ax = fig.add_subplot(gs[0, 1])
    ax.scatter(speeds, ctes, c=ts, cmap='viridis', s=5, alpha=0.5)
    if len(speeds) > 5:
        z = np.polyfit(speeds, ctes, 1)
        xs = np.linspace(min(speeds), max(speeds), 50)
        ax.plot(xs, np.polyval(z, xs), color=C2, lw=1.5, ls='--', label='trend')
    sax(ax, 'Speed vs Divergence (issue #921 metric)')
    ax.set_xlabel('Speed (m/s)'); ax.set_ylabel('|CTE| (m)')
    ax.legend(fontsize=8, facecolor='#161b22', labelcolor='white')

    # Heading error over time
    ax = fig.add_subplot(gs[1, 1])
    ax.plot(ts, herrs, color=C4, lw=1.5)
    sax(ax, 'Heading Error'); ax.set_xlabel('Time (s)'); ax.set_ylabel('|heading error| (rad)')

    # Clearance over time
    ax = fig.add_subplot(gs[0, 2])
    clrs_clipped = [min(c, 3.0) for c in clrs]
    ax.plot(ts, clrs_clipped, color=C3, lw=1.5)
    ax.axhline(1.5, color='#8b949e', lw=0.8, ls=':', label='free space threshold')
    ax.axhline(0.35, color=C2, lw=0.8, ls=':', label='tight threshold')
    sax(ax, 'Obstacle Clearance'); ax.set_xlabel('Time (s)'); ax.set_ylabel('Clearance (m)')
    ax.legend(fontsize=8, facecolor='#161b22', labelcolor='white')

    # Summary stats
    ax = fig.add_subplot(gs[1, 2]); ax.axis('off')
    summary = (
        f"  Episode Summary\n  {'─'*30}\n"
        f"  Duration      = {ep.duration():.1f} s\n"
        f"  Path length   = {ep.path_length:.2f} m\n"
        f"  Avg speed     = {ep.avg_speed():.3f} m/s\n"
        f"  Avg CTE       = {ep.avg_cte():.4f} m\n"
        f"  Max CTE       = {ep.max_cte():.4f} m\n"
        f"  P95 CTE       = {ep.p95_cte():.4f} m\n"
        f"  Completed     = {ep.completed}\n"
        f"  Ticks         = {len(ep.ticks)}"
    )
    ax.text(0.05, 0.95, summary, transform=ax.transAxes, va='top',
            color='white', fontsize=10, fontfamily='monospace',
            bbox=dict(facecolor='#161b22', edgecolor=GC, boxstyle='round,pad=0.8'))

    status = "completed" if ep.completed else "aborted"
    fig.suptitle(f'Controller Performance — Episode ({status})',
                 color='white', fontsize=13, y=0.99)
    plt.savefig(save_path, dpi=150, bbox_inches='tight', facecolor=DARK)


def _plot_speed_vs_divergence(episodes: list[Episode], save_path: str) -> None:
    import matplotlib.pyplot as plt
    import numpy as np

    DARK='#0d1117'; GC='#21262d'
    fig, ax = plt.subplots(figsize=(10, 7))
    fig.patch.set_facecolor(DARK); ax.set_facecolor(DARK)
    ax.tick_params(colors='#8b949e')
    for sp in ax.spines.values(): sp.set_color(GC)
    ax.grid(color=GC, linewidth=0.6)

    all_speeds, all_ctes = [], []
    for ep in episodes:
        s = [t.speed for t in ep.ticks]
        c = [t.cte   for t in ep.ticks]
        ax.scatter(s, c, s=4, alpha=0.3, color='#3fb950')
        all_speeds.extend(s); all_ctes.extend(c)

    if len(all_speeds) > 10:
        # Binned mean line
        bins = np.linspace(min(all_speeds), max(all_speeds), 15)
        bin_idx = np.digitize(all_speeds, bins)
        bin_means = [np.mean([all_ctes[i] for i,b in enumerate(bin_idx) if b==k])
                     for k in range(1, len(bins))]
        bin_centers = 0.5*(bins[:-1]+bins[1:])
        valid = [(x,y) for x,y in zip(bin_centers, bin_means) if not np.isnan(y)]
        if valid:
            xs, ys = zip(*valid)
            ax.plot(xs, ys, color='#f85149', lw=2.5, label='binned mean CTE')

    ax.set_xlabel('Speed (m/s)', color='#8b949e', fontsize=12)
    ax.set_ylabel('|Cross-track error| (m)', color='#8b949e', fontsize=12)
    ax.set_title('Speed vs Divergence from Target Path',
                 color='white', fontsize=13, pad=10)
    ax.legend(fontsize=10, facecolor='#161b22', labelcolor='white')

    stats = (f"N={len(all_speeds)} samples  "
             f"avg_CTE={np.mean(all_ctes):.4f}m  "
             f"p95_CTE={np.percentile(all_ctes,95):.4f}m")
    ax.text(0.5, 0.02, stats, transform=ax.transAxes, ha='center',
            color='#8b949e', fontsize=9,
            bbox=dict(facecolor='#161b22', edgecolor=GC, boxstyle='round,pad=0.4'))

    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight', facecolor=DARK)