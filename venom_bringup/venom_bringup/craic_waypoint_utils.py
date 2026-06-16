"""Utilities for loading CRAIC waypoint tasks."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import List, Sequence


EARTH_RADIUS_METERS = 6378137.0
WGS84_ECCENTRICITY_SQUARED = 0.0066943799901413165
WGS84_SECOND_ECCENTRICITY_SQUARED = (
    WGS84_ECCENTRICITY_SQUARED / (1.0 - WGS84_ECCENTRICITY_SQUARED)
)
UTM_SCALE_FACTOR = 0.9996
UTM_FALSE_EASTING_METERS = 500000.0
UTM_FALSE_NORTHING_METERS = 10000000.0

ACTION_LABELS = {
    0: 'unknown',
    1: 'straight',
    2: 'turn_right',
    3: 'turn_left',
    4: 'lane_change_left',
    5: 'lane_change_right',
    6: 'overtake',
    7: 'u_turn',
    8: 'park',
}


@dataclass(frozen=True)
class CraicWaypoint:
    """One parsed CRAIC waypoint."""

    index: int
    x: float
    y: float
    yaw: float
    action: int
    source_a: float
    source_b: float
    action_label: str


@dataclass(frozen=True)
class UtmCoordinate:
    """One WGS84 UTM coordinate."""

    easting: float
    northing: float
    zone_number: int
    hemisphere: str


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _utm_zone_number(longitude_deg: float, latitude_deg: float) -> int:
    """Resolve the WGS84 UTM zone for one geodetic coordinate."""
    if not -80.0 <= latitude_deg <= 84.0:
        raise ValueError(
            'UTM projection only supports latitudes between -80 and 84 degrees. '
            f'Got latitude={latitude_deg:.6f}.'
        )

    if math.isclose(longitude_deg, 180.0):
        zone_number = 60
    else:
        zone_number = int((longitude_deg + 180.0) / 6.0) + 1

    # Norway special case.
    if 56.0 <= latitude_deg < 64.0 and 3.0 <= longitude_deg < 12.0:
        zone_number = 32

    # Svalbard special cases.
    if 72.0 <= latitude_deg < 84.0:
        if 0.0 <= longitude_deg < 9.0:
            zone_number = 31
        elif 9.0 <= longitude_deg < 21.0:
            zone_number = 33
        elif 21.0 <= longitude_deg < 33.0:
            zone_number = 35
        elif 33.0 <= longitude_deg < 42.0:
            zone_number = 37

    return min(max(zone_number, 1), 60)


def geodetic_to_utm(longitude_deg: float, latitude_deg: float) -> UtmCoordinate:
    """Project one longitude/latitude pair into a WGS84 UTM coordinate."""
    zone_number = _utm_zone_number(longitude_deg, latitude_deg)
    hemisphere = 'N' if latitude_deg >= 0.0 else 'S'

    lat_rad = math.radians(latitude_deg)
    lon_rad = math.radians(longitude_deg)
    lon_origin_deg = (zone_number - 1) * 6 - 180 + 3
    lon_origin_rad = math.radians(lon_origin_deg)

    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    tan_lat = math.tan(lat_rad)

    n_value = EARTH_RADIUS_METERS / math.sqrt(
        1.0 - WGS84_ECCENTRICITY_SQUARED * sin_lat * sin_lat
    )
    t_value = tan_lat * tan_lat
    c_value = WGS84_SECOND_ECCENTRICITY_SQUARED * cos_lat * cos_lat
    a_value = cos_lat * (lon_rad - lon_origin_rad)

    ecc2 = WGS84_ECCENTRICITY_SQUARED
    ecc4 = ecc2 * ecc2
    ecc6 = ecc4 * ecc2
    meridional_arc = EARTH_RADIUS_METERS * (
        (1.0 - ecc2 / 4.0 - 3.0 * ecc4 / 64.0 - 5.0 * ecc6 / 256.0) * lat_rad
        - (3.0 * ecc2 / 8.0 + 3.0 * ecc4 / 32.0 + 45.0 * ecc6 / 1024.0)
        * math.sin(2.0 * lat_rad)
        + (15.0 * ecc4 / 256.0 + 45.0 * ecc6 / 1024.0) * math.sin(4.0 * lat_rad)
        - (35.0 * ecc6 / 3072.0) * math.sin(6.0 * lat_rad)
    )

    easting = UTM_FALSE_EASTING_METERS + UTM_SCALE_FACTOR * n_value * (
        a_value
        + (1.0 - t_value + c_value) * a_value**3 / 6.0
        + (
            5.0
            - 18.0 * t_value
            + t_value * t_value
            + 72.0 * c_value
            - 58.0 * WGS84_SECOND_ECCENTRICITY_SQUARED
        )
        * a_value**5
        / 120.0
    )

    northing = UTM_SCALE_FACTOR * (
        meridional_arc
        + n_value
        * tan_lat
        * (
            a_value * a_value / 2.0
            + (5.0 - t_value + 9.0 * c_value + 4.0 * c_value * c_value)
            * a_value**4
            / 24.0
            + (
                61.0
                - 58.0 * t_value
                + t_value * t_value
                + 600.0 * c_value
                - 330.0 * WGS84_SECOND_ECCENTRICITY_SQUARED
            )
            * a_value**6
            / 720.0
        )
    )

    if hemisphere == 'S':
        northing += UTM_FALSE_NORTHING_METERS

    return UtmCoordinate(
        easting=easting,
        northing=northing,
        zone_number=zone_number,
        hemisphere=hemisphere,
    )


def infer_coordinate_mode(value_a: float, value_b: float) -> str:
    """Infer whether a row looks like lon/lat or planar coordinates."""
    if -180.0 <= value_a <= 180.0 and -90.0 <= value_b <= 90.0:
        if abs(value_a) > 20.0 or abs(value_b) > 20.0:
            return 'geodetic'
    return 'cartesian_m'


def geodetic_to_local_xy(
    longitude_deg: float,
    latitude_deg: float,
    origin_longitude_deg: float,
    origin_latitude_deg: float,
    map_origin_yaw_rad: float,
    map_origin_x_m: float,
    map_origin_y_m: float,
) -> tuple[float, float]:
    """Project lon/lat into the local map frame using UTM offsets."""
    point_utm = geodetic_to_utm(longitude_deg, latitude_deg)
    origin_utm = geodetic_to_utm(origin_longitude_deg, origin_latitude_deg)

    if (
        point_utm.zone_number != origin_utm.zone_number
        or point_utm.hemisphere != origin_utm.hemisphere
    ):
        raise ValueError(
            'Geodetic waypoint and map origin must be in the same UTM zone. '
            f'Waypoint zone={point_utm.zone_number}{point_utm.hemisphere}, '
            f'origin zone={origin_utm.zone_number}{origin_utm.hemisphere}.'
        )

    east = point_utm.easting - origin_utm.easting
    north = point_utm.northing - origin_utm.northing

    cos_yaw = math.cos(map_origin_yaw_rad)
    sin_yaw = math.sin(map_origin_yaw_rad)

    map_x = map_origin_x_m + cos_yaw * east + sin_yaw * north
    map_y = map_origin_y_m - sin_yaw * east + cos_yaw * north
    return map_x, map_y


def _compute_yaws(points_xy: Sequence[tuple[float, float]]) -> List[float]:
    if not points_xy:
        return []
    if len(points_xy) == 1:
        return [0.0]

    yaws: List[float] = []
    for idx, (x_value, y_value) in enumerate(points_xy):
        if idx < len(points_xy) - 1:
            next_x, next_y = points_xy[idx + 1]
            yaw = math.atan2(next_y - y_value, next_x - x_value)
        else:
            prev_x, prev_y = points_xy[idx - 1]
            yaw = math.atan2(y_value - prev_y, x_value - prev_x)
        yaws.append(_normalize_angle(yaw))
    return yaws


def load_craic_waypoints(
    file_path: str,
    coordinate_mode: str = 'geodetic',
    origin_longitude_deg: float = 0.0,
    origin_latitude_deg: float = 0.0,
    map_origin_yaw_rad: float = 0.0,
    map_origin_x_m: float = 0.0,
    map_origin_y_m: float = 0.0,
    use_first_waypoint_as_origin: bool = True,
) -> List[CraicWaypoint]:
    """Load the competition waypoint.txt file.

    Geodetic input rows are first projected into WGS84 UTM coordinates and
    then shifted into the local map frame using the configured origin.
    """
    path = Path(file_path)
    if not path.is_file():
        raise FileNotFoundError(f'Waypoint file not found: {file_path}')

    parsed_rows = []
    for line_no, raw_line in enumerate(path.read_text(encoding='utf-8').splitlines(), start=1):
        stripped = raw_line.strip()
        if not stripped or stripped.startswith('#'):
            continue

        fields = stripped.replace(',', ' ').split()
        if len(fields) < 4:
            raise ValueError(
                f'Invalid waypoint row at line {line_no}: expected at least 4 fields, got {len(fields)}'
            )

        try:
            seq = int(fields[0])
            value_a = float(fields[1])
            value_b = float(fields[2])
            action = int(fields[3])
        except ValueError as exc:
            raise ValueError(f'Failed to parse waypoint row at line {line_no}: {stripped}') from exc

        parsed_rows.append((seq, value_a, value_b, action))

    if not parsed_rows:
        raise ValueError(f'Waypoint file is empty: {file_path}')

    resolved_mode = coordinate_mode
    if coordinate_mode == 'auto':
        resolved_mode = infer_coordinate_mode(parsed_rows[0][1], parsed_rows[0][2])

    if resolved_mode not in {'geodetic', 'cartesian_m', 'cartesian_cm'}:
        raise ValueError(
            f'Unsupported coordinate_mode "{coordinate_mode}". '
            'Use geodetic, cartesian_m, cartesian_cm, or auto.'
        )

    if resolved_mode == 'geodetic' and use_first_waypoint_as_origin:
        if math.isclose(origin_longitude_deg, 0.0) and math.isclose(origin_latitude_deg, 0.0):
            origin_longitude_deg = parsed_rows[0][1]
            origin_latitude_deg = parsed_rows[0][2]

    points_xy = []
    for _, value_a, value_b, _ in parsed_rows:
        if resolved_mode == 'geodetic':
            x_value, y_value = geodetic_to_local_xy(
                longitude_deg=value_a,
                latitude_deg=value_b,
                origin_longitude_deg=origin_longitude_deg,
                origin_latitude_deg=origin_latitude_deg,
                map_origin_yaw_rad=map_origin_yaw_rad,
                map_origin_x_m=map_origin_x_m,
                map_origin_y_m=map_origin_y_m,
            )
        elif resolved_mode == 'cartesian_cm':
            x_value = value_a * 0.01
            y_value = value_b * 0.01
        else:
            x_value = value_a
            y_value = value_b
        points_xy.append((x_value, y_value))

    yaws = _compute_yaws(points_xy)
    result = []
    for idx, (seq, value_a, value_b, action) in enumerate(parsed_rows):
        result.append(
            CraicWaypoint(
                index=seq,
                x=points_xy[idx][0],
                y=points_xy[idx][1],
                yaw=yaws[idx],
                action=action,
                source_a=value_a,
                source_b=value_b,
                action_label=ACTION_LABELS.get(action, f'action_{action}'),
            )
        )
    return result
