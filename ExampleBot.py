import random
from math import sin, cos

from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Rotator, GameInfoState
from rlbot.utils.game_state_util import Vector3 as vec3
from objects import *
from routines import *


# This file is for strategy

class ExampleBot(GoslingAgent):
    def run(agent):
        # run_dodge_sim(agent)
        if len(agent.stack) < 1:
            if agent.kickoff_flag:
                small_boosts = [boost for boost in agent.boosts if not boost.large and boost.active]
                closest = small_boosts[0]
                closest_distance = (closest.location - agent.me.location).magnitude()
                for boost in small_boosts:
                    boost_distance = (boost.location - agent.me.location).magnitude()
                    if boost_distance < closest_distance:
                        closest = boost
                        closest_distance = boost_distance
                agent.push(goto_boost(closest, agent.ball.location))
                agent.push(kickoff())
            else:
                agent.push(atba())


def run_dodge_sim(agent):
    if len(agent.stack) < 1:
        car_state = CarState(boost_amount=33, physics=Physics(location=vec3(-1000, 0, 18), velocity=vec3(500, 0, 0),
                                                              rotation=Rotator(0, 0, 0),
                                                              angular_velocity=vec3(0, 0, 0)))

        ball_state = BallState(Physics(location=vec3(3000, 0, 100), velocity=vec3(0, 0, 0)))
        game_info_state = GameInfoState(game_speed=0.5)
        game_state = GameState(ball=ball_state, cars={agent.index: car_state}, game_info=game_info_state)
        agent.set_game_state(game_state)
        agent.push(flip(Vector3(2500, 0, 0), duration=0.05, delay=0.4, angle=25, boost=True, cancel=True))
        # agent.push(flip(Vector3(2500, 0, 0), duration=0.05, delay=0.3, angle=40, boost=True, cancel=True))
        # agent.push(flip(Vector3(2500, 0, 0), duration=0.05, delay=0.4, angle=0, boost=True, cancel=False))