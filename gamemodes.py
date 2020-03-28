from __future__ import annotations

from typing import TYPE_CHECKING, Union

from routines import DiagonalKickoff, Atba, GotoBoost, OffCenterKickoff, ShortShot
from utils import closest_boost

if TYPE_CHECKING:
    from hive import MyHivemind
    from objects import CarObject


def run_3v3_hiveminds(agent: MyHivemind):
    agent.debug_stack()
    empty_stack = False
    for drone in agent.drones:
        if len(drone.stack) < 1:
            empty_stack = True
    if agent.kickoff_flag and empty_stack:
        x_pos = [round(drone.location.x) for drone in agent.drones]
        if sorted(x_pos) in [[-2048, -256, 2048], [-2048, 0, 2048], [-2048, 256, 2048]]:
            for drone in agent.drones:
                if round(drone.location.x) == -2048:
                    drone.push(DiagonalKickoff())
                elif round(drone.location.x) == 2048:
                    drone.push(ShortShot(agent.ball.location))
                else:
                    drone.push(GotoBoost(closest_boost(agent, drone.location), agent.ball.location))
        elif sorted(x_pos) == [-256, 0, 256]:
            for drone in agent.drones:
                if round(drone.location.x) == -256:
                    drone.push(OffCenterKickoff())
                elif round(drone.location.x) == 256:
                    drone.push(ShortShot(agent.ball.location))
                else:
                    drone.push(GotoBoost(closest_boost(agent, drone.location), agent.ball.location))
        elif -2048 in x_pos or 2048 in x_pos:
            for drone in agent.drones:
                if round(abs(drone.location.x)) == 2048:
                    drone.push(DiagonalKickoff())
                elif round(drone.location.x) == -256:
                    drone.push(ShortShot(agent.ball.location))
                elif round(drone.location.x) == 0:
                    drone.push(GotoBoost(closest_boost(agent, drone.location), agent.ball.location))
                else:
                    if 0 in x_pos:
                        drone.push(ShortShot(agent.ball.location))
                    else:
                        drone.push(GotoBoost(closest_boost(agent, drone.location), agent.ball.location))
    else:
        for drone in agent.drones:
            if len(drone.stack) < 1:
                drone.push(ShortShot(agent.ball.location))
