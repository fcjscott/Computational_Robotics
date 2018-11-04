from animation import animation
from utility import *

x = [(10, 10, 20), (11, 11, 18), (12, 12, 16), (13, 13, 14), (14, 14, 12)]
t = [10, 20, 25, 20, 15]


class MyController:
    def __init__(self, u=(3, 3), block_mass = 1, beam_mass = 1, initial_state = (10, 10, 0), target_state = (20, 20, 0)):
        """

        :param u: 2 member list style input variable contains [velocity, direction]
        """
        self.u = u
        self.mBlock = block_mass
        self.mBeam = beam_mass
        self.xi = initial_state
        self.xf = target_state
        self.a = np.array((0.0, -9.81))
        self.dt = 0.1
        self.traj = []
        self.T = 0

    def dynamics(self):
        self.T = int(self.xf[0]/self.u[0])
        time = np.arange(0, self.T, self.dt)
        pos = (0, 0)
        for t in time:
            pos = 0.5 * self.a * t**2 + self.u * t
            self.traj.append(pos[0], pos[1], 0)

        return self.traj


animation(x, t)















