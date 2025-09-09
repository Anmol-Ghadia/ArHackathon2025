"""
Amazon Robotics Hackathon - Routing API

This module defines the routing API for the Amazon Robotics Hackathon.
Students will implement the route_package function in this module.

*****IMPORTANT*****
Team name: ALGOWIZ
Email address: divij.kapoor@ubc.ca, me@aghadia.com, jinoh0902@gmail.com
*******************
"""

from typing import Optional, Dict, Tuple, List
import heapq

from ar_hackathon.models.game_state import GameState
from ar_hackathon.models.package import Package
from ar_hackathon.models.connection import Connection


def _capacity_ok(conn: Connection) -> bool:
    """
    Returns True if the connection can accept more packages this step.
    - bandwidth=None or available_bandwidth=None => treat as unlimited.
    - otherwise require available_bandwidth > 0
    """
    if conn is None:
        return False
    if conn.bandwidth is None or conn.available_bandwidth is None:
        return True
    return conn.available_bandwidth > 0


def _neighbors_with_capacity(state: GameState, fc_id: str) -> List[Tuple[str, float, Connection]]:
    """
    Returns list of (neighbor_id, weight, connection) from fc_id that currently have capacity.
    """
    out = []
    for conn in state.connections:
        if conn.from_fc == fc_id and _capacity_ok(conn):
            out.append((conn.to_fc, float(conn.weight), conn))
    return out


def _shortest_path_capacity_aware(state: GameState, src: str, dst: str) -> Optional[List[str]]:
    """
    Dijkstra over current graph considering ONLY edges with capacity.
    Returns the full path [src, ..., dst] if reachable, else None.
    Tie-breakers are stable and deterministic:
      - lower total distance first
      - then lexicographically smaller current node id
    """
    if src == dst:
        return [src]

    # Distances and predecessor map
    dist: Dict[str, float] = {src: 0.0}
    prev: Dict[str, Optional[str]] = {src: None}

    # (distance, tie_breaker_node_id, node_id)
    heap: List[Tuple[float, str, str]] = [(0.0, src, src)]

    visited = set()

    while heap:
        d, _, u = heapq.heappop(heap)
        if u in visited:
            continue
        visited.add(u)

        if u == dst:
            break

        for v, w, _ in _neighbors_with_capacity(state, u):
            nd = d + w
            if v not in dist or nd < dist[v] or (nd == dist[v] and u < (prev.get(v) or "")):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v, v))

    if dst not in dist:
        return None

    # Reconstruct path
    path = []
    cur = dst
    while cur is not None:
        path.append(cur)
        cur = prev.get(cur)
    path.reverse()
    return path


def route_package(state: GameState, package: Package) -> Optional[str]:
    """
    Determine the next FC to route a package to.

    Args:
        state: GameState object containing the current state of the network
        package: Package object containing information about the package

    Returns:
        next_fc_id: ID of the next FC to route the package to, or None to stay at current FC
    """
    # If already delivered or currently moving, don't issue a new route.
    if package.in_transit:
        return None
    if package.current_fc == package.destination_fc:
        return None

    src = package.current_fc
    dst = package.destination_fc

    # 1) Fast path: direct edge to destination with capacity → take it.
    direct = state.get_connection(src, dst)
    if _capacity_ok(direct):
        return dst

    # 2) Otherwise, compute a capacity-aware shortest path and take the first hop.
    path = _shortest_path_capacity_aware(state, src, dst)
    if not path or len(path) < 2:
        # No capacity-feasible route right now → wait at current FC.
        return None

    next_hop = path[1]

    # Defensive: ensure the chosen hop still has capacity (race-proofing).
    conn = state.get_connection(src, next_hop)
    if not _capacity_ok(conn):
        return None

    return next_hop
