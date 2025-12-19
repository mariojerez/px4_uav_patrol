import json
import os
import re
from pathlib import Path
from typing import Dict, List, Tuple
import argparse
import math


METERS_PER_DEG_LAT = 111_320.0


def parse_tsp_coords(tsp_path: Path) -> Dict[int, Tuple[float, float]]:
    coords = {}
    in_section = False

    with tsp_path.open("r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            if line == "NODE_COORD_SECTION":
                in_section = True
                continue

            if line == "EOF" or line.startswith("INVALID_EDGE_SECTION"):
                break

            if in_section:
                parts = line.split()
                if len(parts) >= 3:
                    node_id = int(parts[0])
                    x = float(parts[1])
                    y = float(parts[2])
                    coords[node_id] = (x, y)

    if not coords:
        raise RuntimeError(f"No coordinates parsed from {tsp_path}")

    return coords


def load_tour_json(tour_path: Path) -> List[int]:
    with tour_path.open("r") as f:
        data = json.load(f)

    tour = data.get("tour")
    if not tour:
        raise RuntimeError(f"No 'tour' field found in {tour_path}")

    return tour


def find_matching_file(directory: Path, pattern: str) -> Path:
    for p in directory.iterdir():
        if p.is_file() and re.match(pattern, p.name):
            return p
    raise FileNotFoundError(f"No file matching {pattern} in {directory}")


def local_to_global(
    x: float,
    y: float,
    lat0: float,
    lon0: float
) -> Tuple[float, float]:
    dlat = y / METERS_PER_DEG_LAT
    dlon = x / (METERS_PER_DEG_LAT * math.cos(math.radians(lat0)))
    return lat0 + dlat, lon0 + dlon


def gen_patrol_mission(
    scenarioName: str,
    altitude: float,
    groundspeed: float,
    max_heading_rate: float,
    dir_to_tsp: str,
    dir_to_tour: str,
    out_dir: str,
    global_frame: bool,
    origin_lat: float = 0.0,
    origin_lon: float = 0.0,
):
    """
    Generate a mission (mission.json) that can be executed by PX4 that traverses through all the waypoints from the scenario (TSP file)
    in the order specified by the tour (json file).
    
    TSP convention:
        x = east (meters)
        y = north (meters)
    PX4 global frame:
        x = longitude (deg)
        y = latitude (deg)


    :param scenarioName: Name of the scenario a mission is being generated for. Used to find the waypoints (.tsp) file and tour (.json) file.
    :param altitude: Altitude from takeoff location the UAV will remain at.
    :param groundspeed: The target groundspeed of the UAV (independent of windspeed).
    :param max_heading_rate: Restricts how quickly UAV can rotate.
    :param dir_to_tsp: Directory containing the (.TSP) file which has all the waypoints for the scenario.
    :param dir_to_tour: Directory containing the (.json) file containing the order of waypoints to go to.
    :param out_dir: The directory to save the (.json) mission file that this function generates.
    :param global_frame: If True, coordinates from the TSP file are given in latitude-longitude global coordinates. If False, coordinates are interpreted as relative to each other (in meters) and then converted to global coordinates centered at (origin_lat, origin_lon).
    """
    dir_to_tsp = Path(dir_to_tsp)
    dir_to_tour = Path(dir_to_tour)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    tsp_pattern = rf".*{re.escape(scenarioName)}.*\.tsp$"
    tour_pattern = rf"{re.escape(scenarioName)}\..*\.json$"

    tsp_path = find_matching_file(dir_to_tsp, tsp_pattern)
    tour_path = find_matching_file(dir_to_tour, tour_pattern)

    coords = parse_tsp_coords(tsp_path)
    tour = load_tour_json(tour_path)

    items = [{"type": "takeoff"}]

    # First waypoint defines origin if local
    first_node = tour[0]
    x0, y0 = coords[first_node]

    for idx, node_id in enumerate(tour):
        if node_id not in coords:
            raise KeyError(f"Node {node_id} not found in TSP")

        x, y = coords[node_id]

        if global_frame:
            # TSP coords already (lon, lat)
            lon = x
            lat = y
        else:
            # Local meters → global degrees
            lat, lon = local_to_global(
                x - x0,
                y - y0,
                origin_lat,
                origin_lon
            )

        items.append({
            "id": f"wp{idx:03d}",
            "type": "navigation",
            "navigationType": "waypoint",
            "x": lon,                 # longitude (deg)
            "y": lat,                 # latitude (deg)
            "z": float(altitude),     # altitude (m)
            "frame": "global"
        })

    items.append({"type": "rtl"})

    mission = {
        "version": 1,
        "mission": {
            "defaults": {
                "horizontalVelocity": groundspeed,
                "maxHeadingRate": max_heading_rate
            },
            "items": items
        }
    }

    out_path = out_dir / f"tour_{scenarioName}.mission.json"
    with out_path.open("w") as f:
        json.dump(mission, f, indent=2)

    print(f"Mission written to {out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate a PX4 patrol mission JSON from a TSP and tour file"
    )

    parser.add_argument("scenarioName", type=str)

    parser.add_argument("--altitude", type=float, default=10.0)
    parser.add_argument("--groundspeed", type=float, default=5.0)
    parser.add_argument("--max_heading_rate", type=float, default=60.0)

    parser.add_argument(
        "--global-frame",
        action="store_true",
        help="Interpret TSP coords as (lon, lat) instead of meters"
    )

    parser.add_argument("--origin-lat", type=float, default=0.0)
    parser.add_argument("--origin-lon", type=float, default=0.0)

    parser.add_argument("--dir-to-tsp", type=str, default="scenarios")
    parser.add_argument("--dir-to-tour", type=str, default="tours")
    parser.add_argument("--out-dir", type=str, default="missions")

    args = parser.parse_args()

    gen_patrol_mission(
        scenarioName=args.scenarioName,
        altitude=args.altitude,
        groundspeed=args.groundspeed,
        max_heading_rate=args.max_heading_rate,
        dir_to_tsp=args.dir_to_tsp,
        dir_to_tour=args.dir_to_tour,
        out_dir=args.out_dir,
        global_frame=args.global_frame,
        origin_lat=args.origin_lat,
        origin_lon=args.origin_lon,
    )