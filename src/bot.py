from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.messages.flat.QuickChatSelection import QuickChatSelection
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.ball_prediction_analysis import find_slice_at_time
from util.boost_pad_tracker import BoostPadTracker
from util.drive import *
from util.sequence import Sequence, ControlStep
from util.vec import Vec3

def angle_to_target_radians(car: PlayerInfo, target: Vec3) -> float:
    relative = relative_location(Vec3(car.physics.location), Orientation(car.physics.rotation), target)
    return math.atan2(relative.y, relative.x)

def radians_to_degrees(a: float) -> float:
    return normalize_angle_degrees(a * 180.0 / math.pi)

def angle_to_target_degrees(car: PlayerInfo, target: Vec3) -> float:
    return radians_to_degrees(angle_to_target_radians(car, target))

def compass_to_target(src: Vec3, target: Vec3) -> float:
    return radians_to_degrees(math.atan2(target.y - src.y, target.x - src.x) * 180.0 / 3.14)

def normalize_angle_degrees(a: float) -> float:
    if a < -180.0:
        return a + 360.0
    if a > 180.0:
        return a - 360.0
    return a

class MyBot(BaseAgent):

    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.team = team
        self.active_sequence: Sequence = None
        self.boost_pad_tracker = BoostPadTracker()

    def initialize_agent(self):
        # Set up information about the boost pads now that the game is active and the info is available
        self.boost_pad_tracker.initialize_boosts(self.get_field_info())

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        """
        This function will be called by the framework many times per second. This is where you can
        see the motion of the ball, etc. and return controls to drive your car.
        """

        # Keep our boost pad info updated with which pads are currently active
        self.boost_pad_tracker.update_boost_status(packet)

        # This is good to keep at the beginning of get_output. It will allow you to continue
        # any sequences that you may have started during a previous call to get_output.
        if self.active_sequence is not None and not self.active_sequence.done:
            controls = self.active_sequence.tick(packet)
            if controls is not None:
                return controls

        # Gather some information about our car and the ball
        my_car = packet.game_cars[self.index]
        car_location = Vec3(my_car.physics.location)
        car_velocity = Vec3(my_car.physics.velocity)
        car_compass = my_car.physics.rotation.yaw
        ball_location = Vec3(packet.game_ball.physics.location)
        ball_velocity = Vec3(packet.game_ball.physics.velocity)

        # other_car_index = 1 - self.index
        # other_car = packet.game_cars[other_car_index]
        # car_location = Vec3(my_car.physics.location)
        
        ball_radius = 92

        distance_to_ball = car_location.dist(ball_location)
        car_to_ball_compass = compass_to_target(car_location, ball_location)

        if self.team == 0:
            opponent_goal_location = Vec3(0, 5120, 0)
            my_goal_location = Vec3(0, -5120, 0)
        else:
            opponent_goal_location = Vec3(0, -5120, 0)
            my_goal_location = Vec3(0, 5120, 0)
        
        ball_distance_to_opponent_goal = opponent_goal_location.dist(ball_location)
        ball_distance_to_my_goal = my_goal_location.dist(ball_location)

        ball_to_car_vector = car_location - ball_location
        closest_point_on_ball_to_car = ball_location + ball_radius * ball_to_car_vector.normalized()
        self.renderer.draw_rect_3d(closest_point_on_ball_to_car, 4, 4, True, self.renderer.cyan(), centered=True)


        if ball_distance_to_opponent_goal - ball_radius < ball_distance_to_my_goal:
            # offense
            goal_to_ball = ball_location - opponent_goal_location
            goal_to_ball = goal_to_ball.normalized()

            self.renderer.draw_string_3d(car_location, 1, 1, f'shoot!', self.renderer.white())
            point_on_ball_opposite_of_goal = ball_location + ball_radius * goal_to_ball
            self.renderer.draw_rect_3d(point_on_ball_opposite_of_goal, 4, 4, True, self.renderer.green(), centered=True)

            target_location = closest_point_on_ball_to_car
            aiming_adjustment = point_on_ball_opposite_of_goal - closest_point_on_ball_to_car
            target_location = target_location + 2.5 * aiming_adjustment
            target_location = ball_location + ball_radius * (target_location - ball_location).normalized()
        else:
            # defense
            self.renderer.draw_string_3d(car_location, 1, 1, f'           distance_to_ball: {distance_to_ball:.1f}', self.renderer.white())

            self.renderer.draw_string_3d(car_location, 1, 1, f'AH DEFENSE!', self.renderer.white())
            ball_to_my_goal = (my_goal_location - ball_location).normalized()
            target_location = ball_location + 0.9*ball_radius*ball_to_my_goal

        # By default we will chase the ball, but target_location can be changed later
        

        # if car_location.dist(ball_location) > 1500:
        #     # We're far away from the ball, let's try to lead it a little bit
        #     ball_prediction = self.get_ball_prediction_struct()  # This can predict bounces, etc
        #     ball_in_future = find_slice_at_time(ball_prediction, packet.game_info.seconds_elapsed + 2)

        #     # ball_in_future might be None if we don't have an adequate ball prediction right now, like during
        #     # replays, so check it to avoid errors.
        #     if ball_in_future is not None:
        #         target_location = Vec3(ball_in_future.physics.location)
        #         self.renderer.draw_line_3d(ball_location, target_location, self.renderer.cyan())

        # Draw some things to help understand what the bot is thinking
        self.renderer.draw_line_3d(car_location, target_location, self.renderer.white())
        #self.renderer.draw_string_3d(car_location, 1, 1, f'Speed: {car_velocity.length():.1f}', self.renderer.white())
        self.renderer.draw_rect_3d(target_location, 8, 8, True, self.renderer.cyan(), centered=True)

        # if 750 < car_velocity.length() < 800:
        #     # We'll do a front flip if the car is moving at a certain speed.
        #     return self.begin_front_flip(packet)

        controls = SimpleControllerState()
        # You can set more controls if you want, like controls.boost.

        # boost during kickoff
        ball_kickoff_location = Vec3(0.0, 0.0, ball_radius)
        ball_to_kickoff_distance = ball_location.dist(ball_kickoff_location)
        is_ball_at_center = ball_to_kickoff_distance < 0.1 * ball_radius
        ball_speed = ball_velocity.length()
        is_ball_still = ball_speed < 0.1
        is_kickoff = is_ball_at_center and is_ball_still
        if is_kickoff:
            controls.boost = True

        # boost to catch up
        car_to_target_compass = compass_to_target(car_location, target_location)
        is_car_kinda_facing_the_right_way = abs(car_to_target_compass - car_compass) < 22.5
        is_ball_far_away = ball_location.flat().dist(car_location.flat()) > 4.0 * ball_radius
        if is_ball_far_away and is_car_kinda_facing_the_right_way:
            controls.boost = True

        steering_angle = angle_to_target_degrees(my_car, target_location)
        # self.renderer.draw_string_3d(car_location, 1, 10, f'Angle: {steering_angle:.1f}', self.renderer.white())
        steering_strength = 0.1
        controls.steer = limit_to_safe_range(steering_angle * steering_strength)

        if steering_angle > 90:
            # ball is BEHIND US to the right
            # so reverse while turning left
            controls.throttle = -1.0
            controls.steer = -1.0
        elif steering_angle < -90:
            # ball is behind us to the LEFT
            # so reverse while turning right
            controls.throttle = -1.0
            controls.steer = 1.0
        else:
            controls.steer = limit_to_safe_range(steering_angle * steering_strength)
            controls.throttle = 1.0

        if controls.throttle < 0.0:
            controls.boost = False

        return controls

    def begin_front_flip(self, packet):
        # Send some quickchat just for fun
        self.send_quick_chat(team_only=False, quick_chat=QuickChatSelection.Information_IGotIt)

        # Do a front flip. We will be committed to this for a few seconds and the bot will ignore other
        # logic during that time because we are setting the active_sequence.
        self.active_sequence = Sequence([
            ControlStep(duration=0.05, controls=SimpleControllerState(jump=True)),
            ControlStep(duration=0.05, controls=SimpleControllerState(jump=False)),
            ControlStep(duration=0.2, controls=SimpleControllerState(jump=True, pitch=-1)),
            ControlStep(duration=0.8, controls=SimpleControllerState()),
        ])

        # Return the controls associated with the beginning of the sequence so we can start right away.
        return self.active_sequence.tick(packet)
