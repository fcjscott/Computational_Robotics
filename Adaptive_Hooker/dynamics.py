from animation import animation
from utility import *


x = [(0, 0, 0), (11, 11, 18), (12, 12, 16), (13, 13, 14), (14, 14, 12)]
t = [10, 20, 25, 20, 15]


class MyController:
    def __init__(self, u=(3, 3), block_mass = 1, beam_mass = 1, initial_state = (0, 0, 0), target_state = (20, 20, 0)):
        """

        :param u: 2 member list style input variable contains [velocity, direction]
        """
        self.u = u
        self.mBlock = block_mass
        self.mBeam = beam_mass
        self.xi = initial_state
        self.xf = target_state-(4,4,0)
        self.a = np.array((0.0, -9.81))
        self.dt = 0.1
        self.traj = []
        self.T = 0

    def dynamics(self):
        V_vert = self.u[0]*math.sin(self.u[1])
        V_hori = self.u[0]*math.cos(self.u[1])
        self.T = int(self.xf[0]-self.xi[0]/V_hori)
        height = V_vert*self.T+0.5 * self.a * self.T**2
        if height != self.xf[2]:
            print('cant reach target')
        time_line = np.arange(0, self.T, self.dt)
        pos = (0, 0)
        V_vert_temp = V_vert
        for t in time_line:
            pos[0] = pos[0]+V_hori*dt
            pos[1] = pos[1]+0.5 * self.a * dt**2 + V_vert_temp * dt
            V_vert_temp = V_vert_temp - self.a * dt
            self.traj.append(pos[0], pos[1], 0)
        return self.traj


animation(x, t)
#if __name__ == '__main__':
















