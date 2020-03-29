from __future__ import annotations

import time
from typing import TYPE_CHECKING

from objects import Action
from routines import DiagonalKickoff, GotoBoost, OffCenterKickoff, Goto, CenterKickoff, Shadow
from tools import find_hits, push_shot, setup_3s_kickoff
from utils import closest_boost, closest_foe, side

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
        if len(drone.stack) < 1:
            if drone.action == Action.Going:
                if on_side and (drone.location - agent.ball.location).magnitude() < 2000:
                    push_shot(drone, agent)
                if len(drone.stack) < 1:
                    drone.push(Shadow(agent.ball.location))
                    drone.action = Action.Shadowing
            elif drone.action == Action.Shadowing:
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
        setup_3s_kickoff(agent)
    elif not agent.kickoff_flag:
        for drone in agent.drones:
            on_side = (drone.location - agent.friend_goal.location).magnitude() < (
                        agent.ball.location - agent.friend_goal.location).magnitude()
            if len(drone.stack) < 1:
                if drone.action == Action.Going:
                    if on_side and (drone.location - agent.ball.location).magnitude() < 2000:
                        push_shot(drone, agent)
                    if len(drone.stack) < 1:
                        drone.push(Shadow(agent.ball.location))
                        drone.action = Action.Shadowing
                elif drone.action == Action.Shadowing:
                    push_shot(drone, agent)
                    if len(drone.stack) < 1:
                        drone.push(Shadow(agent.ball.location))
                        drone.action = Action.Shadowing
                elif drone.action == Action.Boost:
                    drone.push(Shadow(agent.ball.location))
                    drone.action = Action.Shadowing


def run_podracers(agent: MyHivemind):
    agent.debug_stack()
    drone = agent.drones[0]
    if agent.kickoff_flag and len(drone.stack) < 1:
        setup_3s_kickoff(agent, False)
    elif not agent.kickoff_flag:
        me_on_side = (drone.location - agent.friend_goal.location).magnitude() < (
                    agent.ball.location - agent.friend_goal.location).magnitude()
        team0_on_side = (agent.friends[0].location - agent.friend_goal.location).magnitude() < (
                    agent.ball.location - agent.friend_goal.location).magnitude()
        team1_on_side = (agent.friends[1].location - agent.friend_goal.location).magnitude() < (
                    agent.ball.location - agent.friend_goal.location).magnitude()
        ball_distance = (drone.location - agent.ball.location).magnitude()
        if team0_on_side and team1_on_side:
            closest = ball_distance < (agent.friends[0].location - agent.ball.location).magnitude() and \
                  ball_distance < (agent.friends[1].location - agent.ball.location).magnitude()
        elif team0_on_side:
            closest = ball_distance < (agent.friends[0].location - agent.ball.location).magnitude()
        elif team1_on_side:
            closest = ball_distance < (agent.friends[1].location - agent.ball.location).magnitude()
        else:
            closest = True
        ball_prediction = agent.get_ball_prediction_struct()
        conceding = False
        for i in range(ball_prediction.num_slices):
            prediction_slice = ball_prediction.slices[i]
            physics = prediction_slice.physics
            if physics.location.y * side(agent.team) > 5120:
                conceding = True
                break
        if len(drone.stack) < 1:
            if drone.action == Action.Going:
                if me_on_side and closest or conceding:
                    push_shot(drone, agent)
                if len(drone.stack) < 1:
                    if team0_on_side and team1_on_side:
                        drone.push(GotoBoost(closest_boost(agent, drone.location)))
                        drone.action = Action.Boost
                    else:
                        drone.push(Shadow(agent.ball.location))
                        drone.action = Action.Shadowing
            elif drone.action == Action.Shadowing:
                if me_on_side and closest or conceding:
                    push_shot(drone, agent)
                if len(drone.stack) < 1:
                    drone.push(Shadow(agent.ball.location))
                    drone.action = Action.Shadowing
            elif drone.action == Action.Boost:
                drone.push(Shadow(agent.ball.location))
                drone.action = Action.Shadowing
        elif drone.action == Action.Shadowing:
            if me_on_side and closest or conceding:
                push_shot(drone, agent)
