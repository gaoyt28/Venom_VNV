from pathlib import Path

import pytest
import yaml

from venom_bringup.craic_waypoint_utils import geodetic_to_local_xy
from venom_bringup.road_network_waypoint_utils import (
    load_planned_road_route,
    load_route_waypoints,
    route_to_nav2_waypoints,
    write_competition_waypoint_txt,
    write_waypoints_yaml,
)


def _write_yaml(path: Path, payload):
    path.write_text(
        yaml.safe_dump(payload, sort_keys=False),
        encoding='utf-8',
    )


def test_named_route_preserves_action_and_order(tmp_path: Path):
    road_network_file = tmp_path / 'road_network.yaml'
    _write_yaml(
        road_network_file,
        {
            'coordinate_mode': 'cartesian_cm',
            'nodes': {
                'A': {'x': 0, 'y': 0, 'attr': 1},
                'B': {'x': 400, 'y': 0, 'attr': 2},
                'C': {'x': 400, 'y': 200, 'attr': 8},
            },
            'routes': {
                'main': ['A', 'B', 'C'],
            },
        },
    )

    route = load_planned_road_route(str(road_network_file), route_name='main')

    assert route.route_node_ids == ['A', 'B', 'C']
    assert route.waypoints[1].action == 2
    assert route.waypoints[2].action == 8


def test_graph_search_uses_edges_and_replans_when_blocked(tmp_path: Path):
    road_network_file = tmp_path / 'road_network.yaml'
    _write_yaml(
        road_network_file,
        {
            'coordinate_mode': 'cartesian_cm',
            'nodes': {
                'A': {'x': 0, 'y': 0, 'attr': 1},
                'B': {'x': 300, 'y': 0, 'attr': 1},
                'C': {'x': 300, 'y': 300, 'attr': 3},
                'D': {'x': 0, 'y': 300, 'attr': 1},
            },
            'edges': [
                {'from': 'A', 'to': 'B'},
                {'from': 'B', 'to': 'C'},
                {'from': 'A', 'to': 'D'},
                {'from': 'D', 'to': 'C'},
            ],
        },
    )

    default_route = load_planned_road_route(
        str(road_network_file),
        start_node_id='A',
        goal_node_id='C',
    )
    blocked_route = load_planned_road_route(
        str(road_network_file),
        start_node_id='A',
        goal_node_id='C',
        blocked_edges='A->B',
    )

    assert default_route.route_node_ids == ['A', 'B', 'C']
    assert blocked_route.route_node_ids == ['A', 'D', 'C']


def test_nav2_waypoint_export_contains_action_metadata(tmp_path: Path):
    road_network_file = tmp_path / 'road_network.yaml'
    _write_yaml(
        road_network_file,
        {
            'coordinate_mode': 'cartesian_cm',
            'nodes': {
                'A': {'x': 0, 'y': 0, 'attr': 1},
                'B': {'x': 200, 'y': 0, 'attr': 6},
            },
            'routes': {'main': ['A', 'B']},
        },
    )

    nav2_waypoints = load_route_waypoints(str(road_network_file), route_name='main')

    assert nav2_waypoints[1]['action'] == 6
    assert nav2_waypoints[1]['node_id'] == 'B'


def test_competition_waypoint_txt_uses_attr_and_cm(tmp_path: Path):
    road_network_file = tmp_path / 'road_network.yaml'
    _write_yaml(
        road_network_file,
        {
            'coordinate_mode': 'cartesian_cm',
            'nodes': {
                'start': {'x': 0, 'y': 0, 'attr': 1},
                'goal': {'x': 580, 'y': 400, 'attr': 8},
            },
            'routes': {'main': ['start', 'goal']},
        },
    )

    route = load_planned_road_route(str(road_network_file), route_name='main')
    output_file = tmp_path / 'waypoint.txt'
    write_competition_waypoint_txt(route, str(output_file), output_coordinate_mode='cartesian_cm')

    lines = output_file.read_text(encoding='utf-8').splitlines()

    assert lines[1] == '0 0 0 1'
    assert lines[2] == '1 580 400 8'


def test_write_waypoints_yaml_round_trip(tmp_path: Path):
    road_network_file = tmp_path / 'road_network.yaml'
    _write_yaml(
        road_network_file,
        {
            'coordinate_mode': 'cartesian_cm',
            'nodes': {
                'A': {'x': 0, 'y': 0, 'attr': 1},
                'B': {'x': 100, 'y': 0, 'attr': 1},
            },
            'routes': {'demo': ['A', 'B']},
        },
    )

    route = load_planned_road_route(str(road_network_file), route_name='demo')
    output_file = tmp_path / 'waypoints.yaml'
    write_waypoints_yaml(route_to_nav2_waypoints(route), str(output_file), route_name='demo')

    payload = yaml.safe_load(output_file.read_text(encoding='utf-8'))

    assert payload['route_name'] == 'demo'
    assert payload['waypoints'][0]['x'] == 0.0


def test_geodetic_route_uses_local_utm_offsets(tmp_path: Path):
    road_network_file = tmp_path / 'road_network.yaml'
    origin_lon = 116.33547
    origin_lat = 39.20677
    goal_lon = 116.33552
    goal_lat = 39.20684
    _write_yaml(
        road_network_file,
        {
            'coordinate_mode': 'geodetic',
            'nodes': {
                'start': {'longitude': origin_lon, 'latitude': origin_lat, 'attr': 1},
                'goal': {'longitude': goal_lon, 'latitude': goal_lat, 'attr': 8},
            },
            'routes': {'main': ['start', 'goal']},
        },
    )

    route = load_planned_road_route(
        str(road_network_file),
        route_name='main',
        map_origin_longitude_deg=origin_lon,
        map_origin_latitude_deg=origin_lat,
    )

    expected_x, expected_y = geodetic_to_local_xy(
        longitude_deg=goal_lon,
        latitude_deg=goal_lat,
        origin_longitude_deg=origin_lon,
        origin_latitude_deg=origin_lat,
        map_origin_yaw_rad=0.0,
        map_origin_x_m=0.0,
        map_origin_y_m=0.0,
    )

    assert route.waypoints[0].x_m == pytest.approx(0.0, abs=1e-6)
    assert route.waypoints[0].y_m == pytest.approx(0.0, abs=1e-6)
    assert route.waypoints[1].x_m == pytest.approx(expected_x, abs=1e-6)
    assert route.waypoints[1].y_m == pytest.approx(expected_y, abs=1e-6)


def test_geodetic_projection_rejects_cross_utm_zone_origin():
    with pytest.raises(ValueError, match='same UTM zone'):
        geodetic_to_local_xy(
            longitude_deg=6.1,
            latitude_deg=0.0,
            origin_longitude_deg=5.9,
            origin_latitude_deg=0.0,
            map_origin_yaw_rad=0.0,
            map_origin_x_m=0.0,
            map_origin_y_m=0.0,
        )
