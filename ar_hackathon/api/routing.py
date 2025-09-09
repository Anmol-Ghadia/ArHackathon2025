"""
Amazon Robotics Hackathon - Routing API

This module defines the routing API for the Amazon Robotics Hackathon.
Students will implement the route_package function in this module.

*****IMPORTANT*****
Team name:
Email address:
*******************
"""

from typing import Optional
from ar_hackathon.models.game_state import GameState
from ar_hackathon.models.package import Package
from collections import deque
from typing import Optional, List, Dict, Set

def route_package(state: GameState, package: Package) -> Optional[str]:
    """
    Determine the next FC to route a package to.
    
    This is the function that students will implement. The game engine will call
    this function for each package at each time step to determine where to route it.
    
    Args:
        state: GameState object containing the current state of the network
        package: Package object containing information about the package
        
    Returns:
        next_fc_id: ID of the next FC to route the package to, or None to stay at current FC
    """
    # Student implementation here
    start = package.current_fc
    goal = package.destination_fc
    
    queue = deque()
    visited: Set[str] = set()
    parent: Dict[str, Optional[str]] = {}

    queue.append(start)
    visited.add(start)
    parent[start] = None

    while queue:
        current = queue.popleft()
        if current == goal:
            break
        for fc in state.fulfillment_centers:
            neighbor = fc.id
            if neighbor not in visited and state.get_connection(current, neighbor):
                queue.append(neighbor)
                visited.add(neighbor)
                parent[neighbor] = current

    if goal not in parent:
        return None

    # Trace back to find the next hop
    current = goal
    while parent[current] != start:
        current = parent[current]
    return current