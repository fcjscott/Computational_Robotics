from animation import animation
from utility import *

# global variables
dt = 0.01
tail_w = 0.05
block_w = 0.08
block_mass = 5
tail_mass = 1
I_b = 1/12 * block_mass * 2 * block_w ** 2
I_t = block_mass * tail_mass * (tail_w/2) ** 2 /(block_mass + tail_mass)


class MyController:
    def __init__(self, u=(40, np.pi/3), initial_state=(0, 0, np.pi/3), target_state=(50, 50, 0)):
        """

        :param u: 2 member list style input variable contains [velocity, direction]
        """
        self.u = u
        self.initial_state = initial_state
        self.target_state = target_state
        self.motorTorque = 0

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
        acc_y = 0
        alpha_z = 0
        self.block_state = np.array([[pos_x, vel_x, acc_x], [pos_y, vel_y, acc_y], [theta_z, omega_z, alpha_z]])

        # set tail state variable
        self.tail_state = np.array([0, 0, self.motorTorque / I_t])

        # set run time
        self.T = 15

    def dynamics(self):
        torque_ls = []
        block_traj = [self.initial_state]
        tail_traj = [self.tail_state[0]]
        block_vel = []
        tail_vel = []
        time_line = np.arange(0, self.T, dt)
        for t in time_line:

            # update block dynamics
            self._update(t)

            # append trajectories and velocities
            torque_ls.append(self.motorTorque)
            block_traj.append((self.block_state[0][0], self.block_state[1][0], self.block_state[2][0]))
            block_vel.append((self.block_state[0][1], self.block_state[1][1], self.block_state[2][1]))
            tail_vel.append(self.tail_state[1])
            tail_traj.append(self.tail_state[0])

        return block_traj, tail_traj

    def _update(self, t):
        pos_x, vel_x, acc_x = self.block_state[0][0], self.block_state[0][1], self.block_state[0][2]
        pos_y, vel_y, acc_y = self.block_state[1][0], self.block_state[1][1], self.block_state[1][2]
        theta_z, omega_z, alpha_z = self.block_state[2][0], self.block_state[2][1], self.block_state[2][2]
        theta_t, omega_t, alpha_t = self.tail_state[0], self.tail_state[1], self.tail_state[2]

        self.motorTorque = -0.005
        if t > self.T / 2:
            self.motorTorque = +0.005

        alpha_z = self.motorTorque / I_b
        omega_z += alpha_z * dt
        theta_z += omega_z * dt

        alpha_t = - self.motorTorque / I_t
        omega_t += alpha_t * dt
        theta_t += omega_t * dt

        acc_x = 0 + self.motorTorque / (tail_w * block_mass / 2) * np.sin(theta_z + theta_t)
        acc_y = -9.81 + self.motorTorque / (tail_w * block_mass / 2) * (-np.cos(theta_z + theta_t))

        vel_x += acc_x * dt
        vel_y += acc_y * dt

        pos_x += vel_x * dt
        pos_y += vel_y * dt

        self.block_state = np.array([[pos_x, vel_x, acc_x], [pos_y, vel_y, acc_y], [theta_z, omega_z, alpha_z]])
        self.tail_state = np.array([theta_t, omega_t, alpha_t])


if __name__ == '__main__':
    object = MyController()
    block_traj, tail_traj, block_vel, tail_vel, torque = object.dynamics()

    #animation(x, t, object.target_state)














