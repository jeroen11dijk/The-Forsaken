from __future__ import annotations

from typing import TYPE_CHECKING

from objects import Action
from routines import DiagonalKickoff, GotoBoost, OffCenterKickoff, Goto, CenterKickoff, Shadow
from tools import find_hits, push_shot
from utils import closest_boost, closest_foe

if TYPE_CHECKING:
    from hive import MyHivemind


def run_1v1_hiveminds(agent: MyHivemind):
    agent.debug_stack()
    drone = agent.drones[0]
    if agent.kickoff_flag and len(drone.stack) < 1:
        if abs(drone.location.x) < 250:
            drone.push(CenterKickoff())
            drone.action = Action.Going
        elif abs(drone.location.x) < 1000:
            drone.push(OffCenterKickoff())
            drone.action = Action.Going
        else:
            drone.push(DiagonalKickoff())
            drone.action = Action.Going
    elif not agent.kickoff_flag:
        on_side = (drone.location - agent.friend_goal.location).magnitude() < (
                agent.ball.location - agent.friend_goal.location).magnitude()
        if agent.prev_kickoff_flag:
            if drone.action == Action.Going:
                if on_side and (drone.location - agent.ball.location).magnitude() < 2000:
                    push_shot(drone, agent)
                if len(drone.stack) < 1:
                    drone.push(Shadow(agent.ball.location))
                    drone.action = Action.Shadowing
        else:
            if len(drone.stack) < 1:
                if drone.action == Action.Going:
                    if on_side and (drone.location - agent.ball.location).magnitude() < 2000:
                        push_shot(drone, agent)
                    if len(drone.stack) < 1:
                        drone.push(Shadow(agent.ball.location))
                        drone.action = Action.Shadowing
                elif Action.Shadowing:
                    push_shot(drone, agent)
                    if len(drone.stack) < 1:
                        drone.push(Shadow(agent.ball.location))
                        drone.action = Action.Shadowing


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
                    drone.action = Action.Going
                elif round(drone.location.x) == 2048:
                    rotation_target = agent.friend_goal.location + (
                            agent.ball.location - agent.friend_goal.location) / 2
                    drone.push(Goto(rotation_target, agent.ball.location))
                    drone.action = Action.Shadowing
                else:
                    drone.push(GotoBoost(closest_boost(agent, drone.location), agent.ball.location))
                    drone.action = Action.Boost
        elif sorted(x_pos) == [-256, 0, 256]:
            for drone in agent.drones:
                if round(drone.location.x) == -256:
                    drone.push(OffCenterKickoff())
                    drone.action = Action.Going
                elif round(drone.location.x) == 256:
                    rotation_target = agent.friend_goal.location + (
                            agent.ball.location - agent.friend_goal.location) / 2
                    drone.push(Goto(rotation_target, agent.ball.location))
                    drone.action = Action.Shadowing
                else:
                    drone.push(GotoBoost(closest_boost(agent, drone.location), agent.ball.location))
                    drone.action = Action.Boost
        elif -2048 in x_pos or 2048 in x_pos:
            for drone in agent.drones:
                if round(abs(drone.location.x)) == 2048:
                    drone.push(DiagonalKickoff())
                    drone.action = Action.Going
                elif round(drone.location.x) == -256:
                    rotation_target = agent.friend_goal.location + (
                            agent.ball.location - agent.friend_goal.location) / 2
                    drone.push(Goto(rotation_target, agent.ball.location))
                    drone.action = Action.Shadowing
                elif round(drone.location.x) == 0:
                    drone.push(GotoBoost(closest_boost(agent, drone.location), agent.ball.location))
                    drone.action = Action.Boost
                else:
                    if 0 in x_pos:
                        rotation_target = agent.friend_goal.location + (
                                agent.ball.location - agent.friend_goal.location) / 2
                        drone.push(Goto(rotation_target, agent.ball.location))
                        drone.action = Action.Shadowing
                    else:
                        drone.push(GotoBoost(closest_boost(agent, drone.location), agent.ball.location))
                        drone.action = Action.Boost
    elif not agent.kickoff_flag:
        # Post kick off setup
        if agent.prev_kickoff_flag:
            for drone in agent.drones:
                if drone.action == Action.Going:
                    drone.push(GotoBoost(closest_boost(agent, drone.location), agent.ball.location))
                    drone.action = Action.Boost
                elif drone.action == Action.Shadowing:
                    targets = {"goal": (agent.foe_goal.left_post, agent.foe_goal.right_post)}
                    shots = find_hits(drone, agent, targets)
                    if len(shots["goal"]) > 0:
                        drone.push(shots["goal"][0])
                        drone.action = Action.Going
                    elif len(agent.foes) > 0:
                        drone.push(Goto(closest_foe(agent, drone.location).location))
                        drone.action = Action.Bumping
                    else:
                        rotation_target = agent.friend_goal.location + (
                                agent.ball.location - agent.friend_goal.location) / 2
                        drone.push(Goto(rotation_target, agent.ball.location))
                        drone.action = Action.Shadowing
        else:
            actions = []
            for drone in agent.drones:
                actions.append(drone.action)
            for drone in agent.drones:
                if len(drone.stack) < 1:
                    if drone.action == Action.Going or drone.action == Action.Bumping:
                        drone.push(GotoBoost(closest_boost(agent, drone.location), agent.ball.location))
                        drone.action = Action.Boost
                    elif drone.action == Action.Boost:
                        rotation_target = agent.friend_goal.location + (
                                agent.ball.location - agent.friend_goal.location) / 2
                        drone.push(Goto(rotation_target, agent.ball.location))
                        drone.action = Action.Shadowing
                    elif drone.action == Action.Shadowing:
                        targets = {"goal": (agent.foe_goal.left_post, agent.foe_goal.right_post)}
                        shots = find_hits(drone, agent, targets)
                        if len(shots["goal"]) > 0:
                            drone.push(shots["goal"][0])
                            drone.action = Action.Going
                        else:
                            rotation_target = agent.friend_goal.location + (
                                    agent.ball.location - agent.friend_goal.location) / 2
                            drone.push(Goto(rotation_target, agent.ball.location))
                            drone.action = Action.Shadowing
