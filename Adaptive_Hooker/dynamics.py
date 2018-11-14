from animation import animation
from utility import *


#x = [(0, 0, 20), (11, 11, 18), (12, 12, 16), (13, 13, 14), (14, 14, 0)]
#t = [20, 30, 40, 50, 60]


class MyController:
    def __init__(self, u=(40, 40), block_mass = 1, beam_mass = 1, initial_state = (0, 0, 0), target_state = (40, 40, 0)):
        """

        :param u: 2 member list style input variable contains [velocity, direction]
        """
        self.u = u
        self.angle = np.deg2rad(self.u[1])
        self.mBlock = block_mass
        self.mBeam = beam_mass
        self.xi = initial_state
        self.target_state = target_state
        self.a = np.array((0.0, -9.81))
        self.dt = 0.01
        self.T = 0

    def dynamics(self):
        traj = []
        xf = (self.target_state[0], self.target_state[1]-5,0)
        angle_r = np.deg2rad(self.u[1])
        V_vert = self.u[0] * np.sin(angle_r)
        V_hori = self.u[0] * np.cos(angle_r)
        self.T = (xf[0]-self.xi[0])/V_hori
        height = V_vert*self.T+0.5 * self.a[1] * self.T**2
        if height != xf[2]:
            print('cant reach target')
        time_line = np.arange(0, self.T, self.dt)
        pos = [0, 0]
        V_vert_temp = V_vert
        for t in time_line:
            pos[0] = pos[0]+V_hori*self.dt
            pos[1] = pos[1]+0.5 * self.a[1] * self.dt**2 + V_vert_temp * self.dt
            V_vert_temp = V_vert_temp + self.a[1] * self.dt
            traj.append((pos[0], pos[1], self.u[1]))
        return traj


#animation(x, t,(14,20))
if __name__ == '__main__':
    object =  MyController()
    x = object.dynamics()
    t= np.ones(len(x))
    animation(x,t, object.target_state)














