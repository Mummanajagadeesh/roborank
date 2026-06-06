import math

class RobotPolicy:
    def __init__(self):

        self.K = [
            -100.0,      
            -160.255,  
            -1172.383, 
            -307.333,  
        ]

    def step(self, cart_pole):
        state = cart_pole.get_state()

        x     = state.cart_position_m    - cart_pole.target_position_m
        xd    = state.cart_velocity_mps
        th    = state.pole_angle_rad
        thd   = state.pole_angular_velocity_radps

        force = -(
            self.K[0] * x  +
            self.K[1] * xd +
            self.K[2] * th +
            self.K[3] * thd
        )

        force = max(-cart_pole.max_force_n, min(cart_pole.max_force_n, force))
        cart_pole.set_force(force)