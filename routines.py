from __future__ import annotations

import math
from copy import copy
from typing import TYPE_CHECKING

import virxrlcu

from objects import Vector3, Routine
from utils import cap, defaultDrive, sign, backsolve, shot_valid, defaultPD, defaultThrottle, cap_in_field, \
    dodge_impulse, side

if TYPE_CHECKING:
    from hive import MyHivemind
    from objects import CarObject, BoostObject

gravity: Vector3 = Vector3(0, 0, -650)

# Aerial constants
max_speed: float = 2300
boost_accel: float = 1060
throttle_accel: float = 200 / 3
boost_per_second: float = 30

# Jump constants

jump_speed: float = 291.667
jump_acc = 1458.3333
jump_min_duration = 0.025
jump_max_duration = 0.2


# This file holds all of the mechanical tasks, called "routines", that the bot can do

class CenterKickoff(Routine):
    def __init__(self):
        super().__init__()

    def run(self, drone: CarObject, agent: MyHivemind):
        target = Vector3(0, 3800 * agent.side(), 0)
        local_target = drone.local(target - drone.location)
        defaultDrive(drone, 2300, local_target)
        if local_target.magnitude() < 100:
            drone.pop()
            drone.push(DiagonalKickoff())
            drone.push(Flip(Vector3(1, 0, 0)))


class OffCenterKickoff(Routine):
    def __init__(self):
        super().__init__()

    def run(self, drone: CarObject, agent: MyHivemind):
        target = Vector3(0, 3116 * agent.side(), 0)
        local_target = drone.local(target - drone.location)
        defaultDrive(drone, 2300, local_target)
        if local_target.magnitude() < 400:
            drone.pop()
            drone.push(DiagonalKickoff())
            drone.push(Flip(drone.local(agent.ball.location - drone.location)))


class DiagonalKickoff(Routine):
    def __init__(self):
        super().__init__()

    def run(self, drone: CarObject, agent: MyHivemind):
        target = agent.ball.location + Vector3(0, 200 * agent.side(), 0)
        local_target = drone.local(target - drone.location)
        defaultDrive(drone, 2300, local_target)
        if local_target.magnitude() < 650:
            drone.pop()
            drone.push(Flip(drone.local(agent.foe_goal.location - drone.location)))


class WaveDash(Routine):
    def __init__(self, target=None):
        super().__init__()
        self.step = -1
        # 0 = forward, 1 = right, 2 = backwards, 3 = left
        self.direction = 0
        self.start_time = -1
        self.target = target

        if self.target is not None:
            self.direction = 0 if abs(self.target.x) > abs(self.target.y) else 1

            if (self.direction == 0 and self.target.x < 0) or (self.direction == 1 and self.target.y < 0):
                self.direction += 2

    def run(self, drone: CarObject, agent: MyHivemind):
        if self.start_time == -1:
            self.start_time = agent.time

        T = agent.time - self.start_time

        self.step += 1

        forward_target = drone.velocity.flatten().normalize() * (drone.hitbox.length / 2)

        target_switch = {
            0: forward_target + Vector3(0, 0, 25),
            1: forward_target,
            2: forward_target - Vector3(0, 0, 25),
            3: forward_target
        }

        target_up = {
            0: Vector3(0, 0, 1),
            1: Vector3(0, -1, 1),
            2: Vector3(0, 0, 1),
            3: Vector3(0, 1, 1)
        }

        defaultPD(drone, drone.local(target_switch[self.direction]), up=drone.local(target_up[self.direction]))
        if self.direction == 0:
            drone.controller.throttle = 1
        elif self.direction == 2:
            drone.controller.throttle = -1
        else:
            drone.controller.handbrake = True

        if self.step < 1:
            drone.controller.jump = True
        elif self.step < 4:
            pass
        elif not drone.airborne:
            drone.pop()
        elif T > 2:
            drone.pop()
            drone.push(Recovery())
        elif drone.location.z + (drone.velocity.z * 0.15) < 5:
            drone.jump = True
            drone.yaw = 0
            if self.direction in {0, 2}:
                drone.roll = 0
                drone.pitch = -1 if self.direction is 0 else 1
            else:
                drone.roll = 1 if self.direction is 1 else -1
                drone.pitch = 0


class DoubleJump(Routine):
    # Hits a target point at a target time towards a target direction
    def __init__(self, intercept_time, targets=None):
        super().__init__()
        self.ball_location = None
        self.shot_vector = None
        self.offset_target = None
        self.intercept_time = intercept_time
        self.targets = targets
        # Flags for what part of the routine we are in
        self.jumping = False
        self.dodged = False
        self.jump_time = -1
        self.needed_jump_time = -1
        self.counter = 0

    def update(self, shot):
        self.intercept_time = shot.intercept_time
        self.targets = shot.targets

    def run(self, drone: CarObject, agent: MyHivemind):
        # This routine is the same as jump_shot,
        # but it's designed to hit the ball above 300uus and below 450uus without requiring boost

        T = self.intercept_time - agent.time
        # Capping T above 0 to prevent division problems
        time_remaining = cap(T, 0.000001, 6)

        if (not self.jumping and T > 0.1 and agent.odd_tick % 2 == 0) or self.ball_location is None:
            slice_n = round(T * 60) - 1
            ball = drone.ball_prediction_struct.slices[slice_n].physics.location

            self.ball_location = Vector3(ball.x, ball.y, ball.z)
            self.needed_jump_time = virxrlcu.get_double_jump_time(ball.z - drone.location.z, drone.velocity.z,
                                                                  drone.gravity[2])

        direction = (self.ball_location - drone.location).normalize()
        self.shot_vector = direction if self.targets is None else direction.clamp(
            (self.targets[0] - self.ball_location).normalize(), (self.targets[1] - self.ball_location).normalize())
        self.offset_target = self.ball_location - (self.shot_vector * 92.75)

        car_to_ball = self.ball_location - drone.location
        final_target = self.offset_target.copy().flatten()
        Tj = T - self.needed_jump_time * 1.075

        if Tj > 0 and self.targets is not None:
            # whether we are to the left or right of the shot vector
            side_of_shot = sign(self.shot_vector.cross(Vector3(0, 0, 1)).dot(car_to_ball))
            car_to_offset_target = final_target - drone.location
            car_to_dodge_perp = car_to_offset_target.cross(Vector3(0, 0, side_of_shot))  # perpendicular

            # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector
            # The adjustment slowly decreases to 0 as the bot nears the time to jump
            adjustment = car_to_offset_target.angle2D(self.shot_vector) * min(Tj, 3) * 750  # size of adjustment
            final_target += car_to_dodge_perp.normalize() * adjustment

        distance_remaining = self.offset_target.flat_dist(drone.location)

        # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts or walls to reach it
        final_target = cap_in_field(drone, final_target)
        local_final_target = drone.local_location(final_target)

        # whether we should go forwards or backwards
        angle_to_target = abs(Vector3(1, 0, 0).angle2D(local_final_target))
        direction = 1 if angle_to_target < 1.6 or drone.local_velocity().x > 1000 else -1

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(drone.location, self.offset_target, agent.renderer.white())
        agent.line(self.offset_target - Vector3(0, 0, 100), self.offset_target + Vector3(0, 0, 100),
                   agent.renderer.green())
        agent.line(final_target - Vector3(0, 0, 100), final_target + Vector3(0, 0, 100), agent.renderer.purple())

        vf = drone.velocity + drone.gravity * T

        distance_remaining = drone.local_location(self.offset_target).x if drone.airborne else distance_remaining
        distance_remaining -= drone.hitbox.length * 0.45
        distance_remaining = max(distance_remaining, 0)
        speed_required = distance_remaining / time_remaining

        if not self.jumping:
            velocity = defaultDrive(drone, speed_required * direction, local_final_target)[1]
            if velocity == 0: velocity = 1

            local_offset_target = drone.local_location(self.offset_target.flatten())
            true_angle_to_target = abs(Vector3(1, 0, 0).angle2D(local_offset_target))
            local_vf = drone.local(vf.flatten())
            true_distance_remaining = self.offset_target.flat_dist(drone.location)
            time = true_distance_remaining / (abs(velocity) + dodge_impulse(drone))

            if ((abs(velocity) < 100 and true_distance_remaining < drone.hitbox.length / 2) or (
                    abs(local_offset_target.y) < 92.75 and direction * local_vf.x >= direction * (
                    local_offset_target.x - drone.hitbox.length * 0.45) and direction * local_offset_target.x > 0)) and T <= self.needed_jump_time * 1.025:
                self.jumping = True
            elif drone.airborne:
                drone.push(Recovery(final_target if Tj > 0 else None))
            elif T <= self.needed_jump_time or (Tj > 0 and true_distance_remaining > drone.hitbox.length / 2 and (
                    not virxrlcu.double_jump_shot_is_viable(T, drone.boost_accel, tuple(drone.gravity),
                                                            drone.get_raw(),
                                                            self.offset_target.z,
                                                            tuple((final_target - drone.location).normalize()),
                                                            distance_remaining))):
                # If we're out of time or the ball was hit away or we just can't get enough speed, pop
                drone.pop()
                if drone.airborne:
                    drone.push(ball_recovery())
            elif drone.boost != 'unlimited' and self.needed_jump_time * 1.075 > time:
                time -= self.needed_jump_time * 1.075
                if drone.boost < 48 and angle_to_target < 0.03 and (
                        true_angle_to_target < 0.1 or distance_remaining > 4480) and velocity > 600 and time >= 1:
                    drone.push(flip(drone.local_location(self.offset_target)))
                elif direction == -1 and velocity < 200 and time >= 1.5:
                    drone.push(flip(drone.local_location(self.offset_target), True))
        else:
            # Mark the time we started jumping so we know when to dodge
            if self.jump_time == -1:
                self.jump_time = agent.time

            jump_elapsed = agent.time - self.jump_time
            tau = jump_max_duration - jump_elapsed

            xf = drone.location + drone.velocity * T + 0.5 * drone.gravity * T * T

            if jump_elapsed == 0:
                vf += drone.up * jump_speed
                xf += drone.up * jump_speed * T

            hf = vf
            vf += drone.up * jump_acc * tau
            xf += drone.up * jump_acc * tau * (T - 0.5 * tau)

            hf += drone.up * jump_speed
            vf += drone.up * jump_speed
            xf += drone.up * jump_speed * (T - tau)

            delta_x = self.offset_target - xf
            d_direction = delta_x.normalize()

            if direction == 1 and abs(drone.forward.dot(d_direction)) > 0.5:
                delta_v = delta_x.dot(drone.forward) / T
                if drone.boost > 0 and delta_v >= drone.boost_accel * 0.1:
                    drone.controller.boost = True
                else:
                    drone.controller.throttle = cap(delta_v / (throttle_accel * 0.1), -1, 1)

            if T <= -0.4 or (not drone.airborne and self.counter == 4):
                drone.pop()
                drone.push(ball_recovery())
            elif jump_elapsed < jump_max_duration and hf.z <= self.offset_target.z:
                drone.controller.jump = True
            elif self.counter < 4:
                self.counter += 1

            if self.counter == 3:
                drone.controller.jump = True
            elif self.counter == 4:
                defaultPD(drone, drone.local_location(self.offset_target) * direction, upside_down=True)

            if self.counter < 3:
                defaultPD(drone, drone.local_location(self.offset_target.flatten()) * direction)

        l_vf = vf + drone.location
        agent.line(l_vf - Vector3(0, 0, 100), l_vf + Vector3(0, 0, 100), agent.renderer.red())


class Aerial(Routine):

    def __init__(self, intercept_time, targets=None, fast_aerial=True):
        super().__init__()
        self.intercept_time = intercept_time
        self.fast_aerial = fast_aerial
        self.targets = targets
        self.shot_vector = None
        self.target = None
        self.ball = None

        self.jump_type_fast = None
        self.jumping = False
        self.dodging = False
        self.ceiling = False
        self.jump_time = -1
        self.counter = 0

    def update(self, shot):
        self.intercept_time = shot.intercept_time
        self.fast_aerial = shot.fast_aerial
        self.targets = shot.targets

    def run(self, drone: CarObject, agent: MyHivemind):
        if not agent.shooting:
            agent.shooting = True

        T = self.intercept_time - agent.time
        xf = drone.location + drone.velocity * T + 0.5 * drone.gravity * T * T
        vf = drone.velocity + drone.gravity * T

        slice_n = math.ceil(T * 60) - 1

        if (T > 0.1 and agent.odd_tick % 2 == 0) or self.ball is None:
            ball = drone.ball_prediction_struct.slices[slice_n].physics.location
            self.ball = Vector3(ball.x, ball.y, ball.z)
            self.ceiling = drone.location.z > 2044 - drone.hitbox.height * 2 and not drone.jumped

        direction = (agent.ball.location - drone.location).normalize()
        self.shot_vector = direction if self.targets is None else direction.clamp(
            (self.targets[0] - self.ball).normalize(), (self.targets[1] - self.ball).normalize())
        self.target = self.ball - (self.shot_vector * 92.75)

        if self.ceiling:
            self.target -= Vector3(0, 0, 92.75)

        if self.jumping or self.jump_time == -1:

            if not self.jumping or self.jump_time == -1:
                self.jump_type_fast = self.fast_aerial
                self.jumping = True
                self.jump_time = agent.time
                self.counter = 0

            jump_elapsed = agent.time - self.jump_time

            # how much of the jump acceleration time is left
            tau = jump_max_duration - jump_elapsed

            # impulse from the first jump
            if jump_elapsed == 0:
                vf += drone.up * jump_speed
                xf += drone.up * jump_speed * T

            # acceleration from holding jump
            vf += drone.up * jump_acc * tau
            xf += drone.up * jump_acc * tau * (T - 0.5 * tau)

            if self.jump_type_fast:
                # impulse from the second jump
                vf += drone.up * jump_speed
                xf += drone.up * jump_speed * (T - tau)

                if jump_elapsed <= jump_max_duration:
                    drone.controller.jump = True
                elif self.counter < 4:
                    self.counter += 1

                if self.counter == 3:
                    drone.controller.jump = True
                    self.dodging = True
                elif self.counter == 4:
                    self.dodging = self.jumping = False
            elif jump_elapsed <= jump_max_duration:
                drone.controller.jump = True
            else:
                self.jumping = False

        delta_x = self.target - xf
        direction = delta_x.normalize() if not self.jumping else delta_x.flatten().normalize()

        agent.line(drone.location, drone.location + (direction * 250), agent.renderer.black())
        c_vf = vf + drone.location
        agent.line(c_vf - Vector3(0, 0, 100), c_vf + Vector3(0, 0, 100), agent.renderer.blue())
        agent.line(xf - Vector3(0, 0, 100), xf + Vector3(0, 0, 100), agent.renderer.red())
        agent.line(self.target - Vector3(0, 0, 100), self.target + Vector3(0, 0, 100), agent.renderer.green())

        if not self.dodging:
            target = delta_x if delta_x.magnitude() >= drone.boost_accel * drone.delta_time * 0.1 else self.shot_vector
            target = drone.local(target)

            if drone.controller.jump:
                defaultPD(drone, target.flatten(), up=drone.up)
            elif virxrlcu.find_landing_plane(tuple(drone.location), tuple(drone.velocity), drone.gravity.z) == 4:
                defaultPD(drone, target, upside_down=True)
            else:
                defaultPD(drone, target, upside_down=self.shot_vector.z < 0)

        # only boost/throttle if we're facing the right direction
        if abs(drone.forward.dot(direction)) > 0.75 and T > 0:
            if T > 1 and not self.jumping: drone.controller.roll = 1 if self.shot_vector.z < 0 else -1
            # the change in velocity the bot needs to put it on an intercept course with the target
            delta_v = delta_x.dot(drone.forward) / T
            if not self.jumping and drone.boost > 0 and delta_v >= drone.boost_accel * drone.delta_time * 0.1:
                drone.controller.boost = True
                delta_v -= drone.boost_accel * drone.delta_time * 0.1

            if abs(delta_v) >= throttle_accel * drone.delta_time:
                drone.controller.throttle = cap(delta_v / (throttle_accel * drone.delta_time), -1, 1)

        if T <= -0.2 or (not self.jumping and not drone.airborne) or (
                not self.jumping and T > 1.5 and not virxrlcu.aerial_shot_is_viable(T, drone.boost_accel,
                                                                                    tuple(drone.gravity),
                                                                                    drone.get_raw(),
                                                                                    tuple(self.target))):
            drone.pop()
            drone.push(ball_recovery())
        elif (self.ceiling and self.target.dist(
                drone.location) < 92.75 + drone.hitbox.length and not drone.doublejumped and drone.location.z < agent.ball.location.z + drone.ball_radius and self.target.y * side(
            agent.team) > -4240) or (not self.ceiling and not drone.doublejumped and T < 0.1):
            vector = drone.local_location(self.target).flatten().normalize()
            scale = 1 / max(abs(vector.x), abs(vector.y))
            self.p = cap(-vector.x * scale, -1, 1)
            self.y = cap(vector.y * scale, -1, 1)
            drone.controller.jump = True


class Flip(Routine):
    # Flip takes a vector in local coordinates and flips/dodges in that direction
    # cancel causes the flip to cancel halfway through, which can be used to half-flip
    def __init__(self, vector, cancel=False):
        super().__init__()
        vector = vector.flatten().normalize().normalize()
        scale = 1 / max(abs(vector.x), abs(vector.y))
        self.pitch = cap(-vector.x * scale, -1, 1)
        self.yaw = cap(vector.y * scale, -1, 1)
        self.cancel = cancel

        # the time the jump began
        self.time = -1
        # keeps track of the frames the jump button has been released
        self.counter = 0

    def run(self, drone: CarObject, agent: MyHivemind):
        if self.time == -1:
            self.time = agent.time

        elapsed = agent.time - self.time

        if elapsed < 0.1:
            drone.controller.jump = True
        elif elapsed >= 0.1 and self.counter < 3:
            drone.controller.jump = False
            drone.controller.pitch = self.pitch
            drone.controller.yaw = self.yaw
            self.counter += 1
        elif drone.airborne and (elapsed < 0.4 or (not self.cancel and elapsed < 0.9)):
            drone.controller.jump = True
            drone.controller.pitch = self.pitch
            drone.controller.yaw = self.yaw
        else:
            drone.pop()
            drone.push(recovery())
            return True


class Brake:
    @staticmethod
    def run(drone: CarObject, manual=False):
        # current forward velocity
        speed = drone.local_velocity().x
        if abs(speed) > 100:
            # apply our throttle in the opposite direction
            drone.controller.throttle = -cap(speed / throttle_accel, -1, 1)
        elif not manual:
            drone.pop()


class Goto(Routine):
    # Drives towards a designated (stationary) target
    # Optional vector controls where the car should be pointing upon reaching the target
    # Brake brings the car to slow down to 0 when it gets to it's destination
    # Slow is for small targets, and it forces the car to slow down a bit when it gets close to the target
    def __init__(self, target, vector=None, brake=False, slow=False):
        super().__init__()
        self.target = target
        self.vector = vector
        self.brake = brake
        self.slow = slow

        self.f_brake = False
        self.rule1_timer = -1

    def run(self, drone: CarObject, agent: MyHivemind, manual=False):
        car_to_target = self.target - drone.location
        distance_remaining = car_to_target.flatten().magnitude()

        agent.line(self.target - Vector3(0, 0, 500), self.target + Vector3(0, 0, 500), (255, 0, 255))

        if self.brake and (
                self.f_brake or distance_remaining * 0.95 < (drone.local_velocity().x ** 2 * -1) / (2 * brake_accel.x)):
            self.f_brake = True
            Brake.run(drone, manual=manual)
            return

        if not self.brake and not manual and distance_remaining < 320:
            drone.pop()
            return

        final_target = self.target.copy().flatten()

        if self.vector is not None:
            # See comments for adjustment in jump_shot for explanation
            side_of_vector = sign(self.vector.cross(Vector3(0, 0, 1)).dot(car_to_target))
            car_to_target_perp = car_to_target.cross(Vector3(0, 0, side_of_vector)).normalize()
            adjustment = car_to_target.angle2D(self.vector) * distance_remaining / 3.14
            final_target += car_to_target_perp * adjustment

        final_target = cap_in_field(drone,
                                    final_target)  # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts to reach it
        local_target = drone.local_location(final_target)
        angle_to_target = abs(Vector3(1, 0, 0).angle2D(local_target))
        direction = 1 if angle_to_target < 1.6 or drone.local_velocity().x > 1000 else -1

        velocity = defaultDrive(drone, (
            2300 if distance_remaining > 1280 or not self.slow else cap(distance_remaining * 2, 1200,
                                                                        2300)) * direction, local_target)[1]
        if distance_remaining < 1280: drone.controller.boost = False
        if velocity == 0: velocity = 1

        time = distance_remaining / (abs(velocity) + dodge_impulse(drone))

        # this is to break rule 1's with TM8'S ONLY
        # 251 is the distance between center of the 2 longest cars in the game, with a bit extra
        if len(agent.friends) > 0 and drone.local_velocity().x < 50 and drone.controller.throttle == 1 and min(
                drone.location.flat_dist(car.location) for car in agent.friends) < 251:
            if self.rule1_timer == -1:
                self.rule1_timer = agent.time
            elif agent.time - self.rule1_timer > 1.5:
                agent.push(Flip(Vector3(0, 250, 0)))
                return
        elif self.rule1_timer != -1:
            self.rule1_timer = -1

        if drone.airborne:
            drone.push(recovery(self.target))
        elif drone.boost != 'unlimited' and drone.boost < 60 and angle_to_target < 0.03 and velocity > 500 and time > 1.5:
            drone.push(Flip(drone.local_location(self.target)))
        elif drone.boost != 'unlimited' and direction == -1 and velocity < 200 and time > 1.5:
            drone.push(Flip(drone.local_location(self.target), True))


class Shadow(Routine):
    def __init__(self):
        super().__init__()
        self.goto = Goto(Vector3(), brake=True)
        self.retreat = retreat()

    def run(self, drone: CarObject, agent: MyHivemind):
        ball_loc = self.get_ball_loc(drone, agent)
        target = self.get_target(drone, agent, ball_loc)

        self_to_target = drone.location.flat_dist(target)

        if self_to_target < 100 * (
                drone.velocity.magnitude() / 500) and ball_loc.y < -640 and drone.velocity.magnitude() < 50 and abs(
            Vector3(1, 0, 0).angle2D(drone.local_location(agent.ball.location))) > 1:
            drone.pop()
            if len(agent.friends) > 1:
                drone.push(face_target(ball=True))
        else:
            self.goto.target = target
            self.goto.vector = ball_loc * Vector3(0, side(agent.team), 0) if target.y * side(
                agent.team) < 1280 else None
            self.goto.run(drone, agent)

    def is_viable(self, drone: CarObject, agent: MyHivemind):
        ball_loc = self.get_ball_loc(drone, agent)
        target = self.get_target(drone, agent, ball_loc)
        self_to_target = drone.location.flat_dist(target)

        return self_to_target > 320

    def get_ball_loc(self, drone: CarObject, agent: MyHivemind):
        ball_slice = drone.ball_prediction_struct.slices[min(round(180 * 1.1), 6)].physics.location
        ball_loc = Vector3(ball_slice.x, ball_slice.y, 0)
        ball_loc.y *= side(drone.team)

        if ball_loc.y < -2560 or (ball_loc.y < agent.ball.location.y * side(drone.team)):
            ball_loc = Vector3(agent.ball.location.x, agent.ball.location.y * side(agent.team) - 640)

        return ball_loc

    def get_target(self, drone: CarObject, agent: MyHivemind, ball_loc=None):
        if ball_loc is None:
            ball_loc = self.get_ball_loc(drone, agent)

        distance = 2560

        target = Vector3(0, (ball_loc.y + distance) * side(agent.team), 0)
        if target.y * side(agent.team) > -1280:
            # use linear algebra to find the proper x coord for us to stop a shot going to the net
            # y = mx + b <- yes, finally! 7th grade math is paying off xD
            p1 = self.retreat.get_target(agent)
            p2 = ball_loc * Vector3(1, side(agent.team), 0)
            try:
                m = (p2.y - p1.y) / (p2.x - p1.x)
                b = p1.y - (m * p1.x)
                # x = (y - b) / m
                target.x = (target.y - b) / m
            except ZeroDivisionError:
                target.x = 0
        else:
            target.x = (abs(ball_loc.x) + 640) * sign(ball_loc.x)

        return Vector3(target.x, target.y, 0)
