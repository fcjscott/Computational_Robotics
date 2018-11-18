from animation import animation
from utility import *

# global variables
dt = 0.01
tail_l = 1
tail_w = 15
block_w = 8
block_mass = 1
tail_mass = 1
I_b = 1/12 * block_mass * block_w ** 4
I_t = 1/12 * tail_mass * tail_l * tail_w ** 3


class MyController:
    def __init__(self, u=(40, np.pi/3), initial_state=(0, 0, np.pi/3), target_state=(50, 50, 0)):
        """

        :param u: 2 member list style input variable contains [velocity, direction]
        """
        self.u = u
        self.initial_state = initial_state
        self.target_state = target_state
        self.motorTorque = 200

        if self.initial_state[2] != self.u[1]:
            raise Exception("Initial angle doesn't match with shooting angle")

        # set block state variable
        pos_x = self.initial_state[0]
        pos_y = self.initial_state[1]
        theta_z = self.initial_state[2]
        vel_x = self.u[0] * np.cos(self.u[1])
        vel_y = self.u[0] * np.sin(self.u[1])
        omega_z = 0
        acc_x = 0
        acc_y = -9.81
        alpha_z = - self.motorTorque / I_b
        self.block_state = np.array([[pos_x, vel_x, acc_x], [pos_y, vel_y, acc_y], [theta_z, omega_z, alpha_z]]).T

        # set tail state variable
        self.tail_state = np.array([0, 0, self.motorTorque / I_t])

        # set run time
        self.T = (self.target_state[0] - self.initial_state[0]) / vel_x

    def dynamics(self):
        block_traj = [self.initial_state]
        tail_traj = [self.tail_state[0]]
        time_line = np.arange(0, self.T, dt)
        for _ in time_line:

            # update block dynamics
            self._updateBlock()

            # update tail dynamics
            self._updateTail()

            # append trajectories
            block_traj.append((self.block_state[0][0], self.block_state[0][1], self.block_state[0][2]))
            tail_traj.append(self.tail_state[0])

        return block_traj, tail_traj

    def _updateBlock(self):
        self.block_state = np.array([[1, dt, 0.5*dt**2], [0, 1, dt], [0, 0, 1]]).dot(self.block_state)

    def _updateTail(self):
        self.tail_state = np.array([[1, dt, 0.5*dt**2], [0, 1, dt], [0, 0, 1]]).dot(self.tail_state)


#x = [(0, 0, np.pi/4), (11, 11, np.pi/4), (12, 12, np.pi/4), (13, 13, np.pi/4), (14, 14, np.pi/4)]
#t = [np.pi/2, np.pi/3, np.pi/4, np.pi/5, np.pi/6]
#animation(x, t,(14,20))

if __name__ == '__main__':
    object = MyController()
    x, t = object.dynamics()
    animation(x, t, object.target_state)














