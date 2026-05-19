#!/usr/bin/env python3
"""Diagnostic: subscribe to /global_costmap and /odom via LCM, print costmap stats.

Run with the dimos venv while the agent + bridge are running:
  cd DimSim
  ../dimos/.venv/bin/python dimos-cli/test/diagnose_costmap.py

Prints: cell counts (FREE/OCCUPIED/UNKNOWN), robot grid position,
and whether frontier-eligible cells exist.
"""

import sys
import time
import numpy as np

try:
    import lcm as lcm_lib
except ImportError:
    print("ERROR: 'lcm' Python package not found. Run with dimos venv.")
    sys.exit(1)

from dimos_lcm.nav_msgs.OccupancyGrid import OccupancyGrid as LCMOccupancyGrid
from dimos_lcm.geometry_msgs.PoseStamped import PoseStamped as LCMPoseStamped


class CostmapDiagnostic:
    def __init__(self):
        self.lc = lcm_lib.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.latest_costmap = None
        self.latest_odom = None
        self.costmap_count = 0
        self.odom_count = 0

    def _on_costmap(self, channel, data):
        try:
            msg = LCMOccupancyGrid.lcm_decode(data)
            self.latest_costmap = msg
            self.costmap_count += 1
        except Exception as e:
            print(f"Costmap decode error: {e}")

    def _on_odom(self, channel, data):
        try:
            msg = LCMPoseStamped.lcm_decode(data)
            self.latest_odom = msg
            self.odom_count += 1
        except Exception as e:
            print(f"Odom decode error: {e}")

    def analyze(self):
        if self.latest_costmap is None:
            print("No costmap received yet.")
            return

        msg = self.latest_costmap
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        grid = np.array(msg.data, dtype=np.int8).reshape(h, w)

        free_count = int(np.sum(grid == 0))
        occupied_count = int(np.sum(grid == 100))
        unknown_count = int(np.sum(grid == -1))
        other_count = int(np.sum((grid != 0) & (grid != 100) & (grid != -1)))
        total = w * h

        print(f"\n{'='*60}")
        print(f"COSTMAP #{self.costmap_count}  ({w}x{h} cells, {res:.3f} m/cell)")
        print(f"Origin: ({ox:.2f}, {oy:.2f})")
        print(f"World extent: X=[{ox:.2f}, {ox + w*res:.2f}]  Y=[{oy:.2f}, {oy + h*res:.2f}]")
        print(f"  FREE (0):        {free_count:>8} ({100*free_count/total:.1f}%)")
        print(f"  OCCUPIED (100):  {occupied_count:>8} ({100*occupied_count/total:.1f}%)")
        print(f"  UNKNOWN (-1):    {unknown_count:>8} ({100*unknown_count/total:.1f}%)")
        print(f"  OTHER (1-99):    {other_count:>8} ({100*other_count/total:.1f}%)")

        if other_count > 0:
            mask = (grid != 0) & (grid != 100) & (grid != -1)
            other_vals = grid[mask]
            unique, counts = np.unique(other_vals, return_counts=True)
            print(f"  Other cost distribution (top 10):")
            for v, c in sorted(zip(unique, counts), key=lambda x: -x[1])[:10]:
                print(f"    cost={v:>4}: {c} cells")

        # Check robot position
        if self.latest_odom is not None:
            rx = self.latest_odom.pose.position.x
            ry = self.latest_odom.pose.position.y
            rz = self.latest_odom.pose.position.z
            gx = int((rx - ox) / res)
            gy = int((ry - oy) / res)
            print(f"\nRobot world pos: ({rx:.2f}, {ry:.2f}, {rz:.2f})")
            print(f"Robot grid cell: ({gx}, {gy})")
            if 0 <= gx < w and 0 <= gy < h:
                print(f"Robot cell cost: {grid[gy, gx]}")
                r = 5
                region = grid[max(0,gy-r):gy+r+1, max(0,gx-r):gx+r+1]
                rfree = int(np.sum(region == 0))
                rocc = int(np.sum(region == 100))
                runk = int(np.sum(region == -1))
                roth = int(np.sum((region != 0) & (region != 100) & (region != -1)))
                print(f"11x11 neighborhood: FREE={rfree} OCC={rocc} UNK={runk} OTHER={roth}")
            else:
                print(f"WARNING: Robot is OUTSIDE costmap bounds!")
        else:
            print(f"\nNo odom received (count={self.odom_count})")

        # Frontier analysis
        unk_mask = (grid == -1)
        free_mask = (grid == 0)
        occ_mask = (grid >= 100)

        from scipy import ndimage
        kernel = np.ones((3, 3))
        free_dilated = ndimage.binary_dilation(free_mask, structure=kernel)
        occ_dilated = ndimage.binary_dilation(occ_mask, structure=kernel)

        candidates = unk_mask & free_dilated
        unknown_near_free = int(np.sum(candidates))
        frontier_eligible = candidates & ~occ_dilated
        frontier_count = int(np.sum(frontier_eligible))
        frontier_blocked = unknown_near_free - frontier_count

        print(f"\nFRONTIER ANALYSIS (pre-inflation):")
        print(f"  Unknown cells adjacent to free:         {unknown_near_free}")
        print(f"  ...blocked by adjacent occupied:        {frontier_blocked}")
        print(f"  VALID FRONTIER CELLS:                   {frontier_count}")

        # Simulate inflation effect
        if frontier_count > 0:
            inflate_radius = 0.25  # meters, default in frontier explorer
            cell_radius = int(np.ceil(inflate_radius / res))
            y, x = np.ogrid[-cell_radius:cell_radius+1, -cell_radius:cell_radius+1]
            inflate_kernel = (x**2 + y**2 <= cell_radius**2).astype(np.uint8)
            inflated_occ = ndimage.binary_dilation(occ_mask, structure=inflate_kernel)
            inflated_occ_dilated = ndimage.binary_dilation(inflated_occ, structure=kernel)
            frontier_after_inflate = candidates & ~inflated_occ_dilated
            print(f"\n  After 0.25m inflation:")
            print(f"  VALID FRONTIER CELLS:                   {int(np.sum(frontier_after_inflate))}")

        if frontier_count == 0 and unknown_near_free > 0:
            print(f"\n  *** DIAGNOSIS: ALL {unknown_near_free} unknown-near-free cells")
            print(f"      are also adjacent to occupied cells. Obstacles border every")
            print(f"      free/unknown boundary. The height_cost algorithm may produce")
            print(f"      high-gradient costs at edges, or the LiDAR sees obstacles")
            print(f"      exactly at the boundary of observed space.")
        elif frontier_count == 0 and free_count == 0:
            print(f"\n  *** DIAGNOSIS: No FREE (cost=0) cells at all!")
            print(f"      The height_cost algorithm is not seeing flat ground.")
            print(f"      Check if LiDAR produces ground-hitting points (Z<0.1 in robotics frame).")
        elif frontier_count == 0 and unknown_near_free == 0 and free_count > 0:
            print(f"\n  *** DIAGNOSIS: FREE cells exist but no UNKNOWN cells border them.")
            print(f"      The free space is fully enclosed by occupied/other-cost cells.")

    def run(self):
        self.lc.subscribe("/global_costmap#nav_msgs.OccupancyGrid", self._on_costmap)
        self.lc.subscribe("/odom#geometry_msgs.PoseStamped", self._on_odom)

        print("Listening on LCM for /global_costmap and /odom...")
        print("Start the dimos agent + bridge in other terminals. Ctrl+C to stop.\n")

        try:
            last_print = 0
            while True:
                self.lc.handle_timeout(500)
                now = time.time()
                if now - last_print > 3.0:
                    print(f"\n--- msgs: costmap={self.costmap_count}, odom={self.odom_count} ---")
                    self.analyze()
                    last_print = now
        except KeyboardInterrupt:
            print("\nDone.")


if __name__ == "__main__":
    CostmapDiagnostic().run()
