from dataclasses import dataclass


@dataclass(frozen=True)
class RouteNames:
    cruise_route: str
    overtake_left_route: str
    return_route: str


class LaneRouteManager:
    def __init__(self, route_names: RouteNames) -> None:
        self._route_names = route_names

    def cruise_route(self) -> str:
        return self._route_names.cruise_route

    def overtake_left_route(self) -> str:
        return self._route_names.overtake_left_route

    def return_route(self) -> str:
        return self._route_names.return_route
