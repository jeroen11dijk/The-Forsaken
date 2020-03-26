import random
from math import sin, cos

from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Rotator, GameInfoState
from rlbot.utils.game_state_util import Vector3 as vec3
from objects import *
from routines import *


# This file is for strategy

class ExampleBot(GoslingAgent):
    def run(agent):
        agent.debug_stack()
        if len(agent.stack) < 1:
            if agent.kickoff_flag:
                if abs(agent.me.location.x) < 250:
                    agent.set_state = True
                    agent.push(center_kickoff())
                elif abs(agent.me.location.x) < 1000:
                    agent.set_state = True
                    agent.push(offcenter_kickoff())
                else:
                    agent.set_state = True
                    agent.push(diagonal_kickoff())
            else:
                if agent.set_state:
                    agent.push(set_state())
                else:
                    agent.push(short_shot(agent.foe_goal.location))


def run_dodge_sim(agent):
    agent.debug_stack()
    if len(agent.stack) < 1:
        car_state = CarState(boost_amount=33, physics=Physics(location=vec3(-1000, 0, 18), velocity=vec3(1000, 0, 0),
                                                              rotation=Rotator(0, 0, 0),
                                                              angular_velocity=vec3(0, 0, 0)))

        ball_state = BallState(Physics(location=vec3(3000, 500, 100), velocity=vec3(0, 0, 0)))
        game_info_state = GameInfoState(game_speed=0.5)
        game_state = GameState(ball=ball_state, cars={agent.index: car_state}, game_info=game_info_state)
        agent.set_game_state(game_state)
        agent.push(wait())