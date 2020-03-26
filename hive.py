from typing import Dict

from rlbot.agents.base_agent import SimpleControllerState
from rlbot.agents.hivemind.drone_agent import DroneAgent
from rlbot.agents.hivemind.python_hivemind import PythonHivemind
from rlbot.utils.structures.bot_input_struct import PlayerInput
from rlbot.utils.structures.game_data_struct import GameTickPacket

from objects import *
# Dummy agent to call request MyHivemind.
from routines import atba


class Drone(DroneAgent):
    hive_path = __file__
    hive_key = 'TheGame'
    hive_name = 'Reddit'


class MyHivemind(PythonHivemind):

    def initialize_hive(self, packet: GameTickPacket) -> None:
        self.logger.info('Initialised!')

        # Find out team by looking at packet.
        # drone_indices is a set, so you cannot just pick first element.
        index = next(iter(self.drone_indices))
        self.team = packet.game_cars[index].team

        # A list of cars for both teammates and opponents
        self.friends = []
        self.foes = []
        # This holds the carobjects for our agent
        self.drones = [car_object(i) for i in self.drone_indices]

        self.ball = ball_object()
        self.game = game_object()
        # A list of boosts
        self.boosts = []
        # goals
        self.friend_goal = goal_object(self.team)
        self.foe_goal = goal_object(not self.team)
        # A list that acts as the routines stack
        self.stack = []
        # Game time
        self.time = 0.0
        # Whether or not GoslingAgent has run its get_ready() function
        self.ready = False
        # a flag that tells us when kickoff is happening
        self.kickoff_flag = False

        self.last_time = 0
        self.my_score = 0
        self.foe_score = 0

    def get_ready(self, packet):
        # Preps all of the objects that will be updated during play
        field_info = self.get_field_info()
        for i in range(field_info.num_boosts):
            boost = field_info.boost_pads[i]
            self.boosts.append(boost_object(i, boost.location, boost.is_full_boost))
        self.refresh_player_lists(packet)
        self.ball.update(packet)
        self.ready = True

    def refresh_player_lists(self, packet):
        # makes new friend/foe lists
        # Useful to keep separate from get_ready because humans can join/leave a match
        drone_indices = [drone.index for drone in self.drones]
        self.friends = [car_object(i, packet) for i in range(packet.num_cars) if
                        packet.game_cars[i].team == self.team and i not in drone_indices]
        self.foes = [car_object(i, packet) for i in range(packet.num_cars) if packet.game_cars[i].team != self.team]

    def push(self, routine):
        # Shorthand for adding a routine to the stack
        self.stack.append(routine)

    def pop(self):
        # Shorthand for removing a routine from the stack, returns the routine
        return self.stack.pop()

    def line(self, start, end, color=None):
        color = color if color != None else [255, 255, 255]
        self.renderer.draw_line_3d(start, end, self.renderer.create_color(255, *color))

    def debug_stack(self):
        # Draws the stack on the screen
        white = self.renderer.white()
        for i in range(len(self.stack) - 1, -1, -1):
            text = self.stack[i].__class__.__name__
            self.renderer.draw_string_2d(10, 50 + (50 * (len(self.stack) - i)), 3, 3, text, white)

    def clear(self):
        # Shorthand for clearing the stack of all routines
        self.stack = []

    def preprocess(self, packet):
        # Calling the update functions for all of the objects
        if packet.num_cars != len(self.friends) + len(self.foes) + len(self.drones): self.refresh_player_lists(packet)
        for car in self.friends: car.update(packet)
        for car in self.foes: car.update(packet)
        for pad in self.boosts: pad.update(packet)
        for drone in self.drones: drone.update(packet)
        self.ball.update(packet)
        self.game.update(packet)
        self.time = packet.game_info.seconds_elapsed
        # When a new kickoff begins we empty the stack
        if not self.kickoff_flag and packet.game_info.is_round_active and packet.game_info.is_kickoff_pause:
            self.stack = []
        # Tells us when to go for kickoff
        self.kickoff_flag = packet.game_info.is_round_active and packet.game_info.is_kickoff_pause

    def get_outputs(self, packet: GameTickPacket) -> Dict[int, PlayerInput]:
        # Get ready, then preprocess
        if not self.ready:
            self.get_ready(packet)
        self.preprocess(packet)

        self.renderer.begin_rendering()
        # Run our strategy code
        self.run()
        # run the routine on the end of the stack
        for drone in self.drones:
            if len(self.stack) > 0:
                self.stack[-1].run(drone, self.ball.location)
        self.renderer.end_rendering()
        # send our updated controller back to rlbot
        # return self.controller

        return {drone.index: drone.controller for drone in self.drones}

    def run(self):
        print(self.drone_indices)
        print(len(self.friends))
        if len(self.stack) < 1:
            self.push(atba())


class car_object:
    # The carObject, and kin, convert the gametickpacket in something a little friendlier to use,
    # and are updated by GoslingAgent as the game runs
    def __init__(self, index, packet=None):
        self.location = Vector3(0, 0, 0)
        self.orientation = Matrix3(0, 0, 0)
        self.velocity = Vector3(0, 0, 0)
        self.angular_velocity = [0, 0, 0]
        self.demolished = False
        self.airborne = False
        self.supersonic = False
        self.jumped = False
        self.doublejumped = False
        self.team = 0
        self.boost = 0
        self.index = index
        self.controller = PlayerInput()
        if packet != None:
            self.team = packet.game_cars[self.index].team
            self.update(packet)

    def local(self, value):
        # Shorthand for self.orientation.dot(value)
        return self.orientation.dot(value)

    def update(self, packet):
        car = packet.game_cars[self.index]
        self.location.data = [car.physics.location.x, car.physics.location.y, car.physics.location.z]
        self.velocity.data = [car.physics.velocity.x, car.physics.velocity.y, car.physics.velocity.z]
        self.orientation = Matrix3(car.physics.rotation.pitch, car.physics.rotation.yaw, car.physics.rotation.roll)
        self.angular_velocity = self.orientation.dot(
            [car.physics.angular_velocity.x, car.physics.angular_velocity.y, car.physics.angular_velocity.z]).data
        self.demolished = car.is_demolished
        self.airborne = not car.has_wheel_contact
        self.supersonic = car.is_super_sonic
        self.jumped = car.jumped
        self.doublejumped = car.double_jumped
        self.boost = car.boost
        # Reset controller
        self.controller.__init__()

    @property
    def forward(self):
        # A vector pointing forwards relative to the cars orientation. Its magnitude is 1
        return self.orientation.forward

    @property
    def left(self):
        # A vector pointing left relative to the cars orientation. Its magnitude is 1
        return self.orientation.left

    @property
    def up(self):
        # A vector pointing up relative to the cars orientation. Its magnitude is 1
        return self.orientation.up
