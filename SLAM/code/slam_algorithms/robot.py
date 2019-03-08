import numpy as np
import copy


class MyRobot:
    def __init__(self, env, u=None, Q=None, R=None, s0=None, P0=None):
        """

        :param u: sequence of control input
        :param Q: covariance matrix for process noise
        :param R: covariance matrix for observation noise
        :param s0: initial state
        :param P0: initial covariance matrix
        :param env: a copy of environment
        """

        self.u = u
        self.Q = Q
        self.R = R
        self.P = P0
        self.s = s0
        self.env = env
        self.landmarks = env.generate_landmarks(self.env.n_landmarks)
        self.process_dt = 0.1
        self.observation_dt = 0.5
        self.timer = 0.0
        self.H = np.identity(2)
        self.F = np.identity(2)
        self.Kalman_Gain = np.identity(2)
        self.traj_log = {'true': [], 'estimate': []}


        #print(self.env.landmarks)
        #print(self.landmarks)

    def visualize(self):

        # generate a plot for true environment
        fig, ax = self.env.plot_env()

        # generate a plot for true trajectory
        traj_x, traj_y = zip(*self.traj_log['true'])
        ax.scatter(traj_x, traj_y, marker='o')

        # generate a plot for estimated environment
        lm_x, lm_y = zip(*self.landmarks.values())
        ax.scatter(lm_x, lm_y, marker='*', s=100)

        # generate a plot for true trajectory
        traj_x, traj_y = zip(*self.traj_log['estimate'])
        ax.scatter(traj_x, traj_y, marker='o')

        ax.legend(['True Landmarks', 'True Trajectory', 'Estimated Landmarks', 'Estimated Trajectory'])

        return fig, ax

    def time_update(self, s, u=None, noise=True):

        s_true = np.copy(self.s)
        s_prior = self.F.dot(s)
        self.traj_log['true'].append(s_true)
        self.traj_log['estimate'].append(s_prior)

        return s_true, s_prior

        #s_prior = s

        #if noise:
        #    s_true = s_prior + np.sqrt(self.Q).dot(np.random.randn(*s_prior.shape))
        #    return s_true, s_prior

        #return s_prior

    def map_construction(self, s, ob):
        s_prior = np.copy(s)
        print("prior state:{}".format(s_prior))

        # unpack observation
        for idx in range(self.env.n_landmarks):
            noise_id = "noise {}".format(idx)
            estimate_id = "estimate {}".format(idx)
            S = self.H @ self.P @ self.H.T + self.R
            self.Kalman_Gain = self.P @ self.H.T @ np.linalg.pinv(S)
            self.P = self.P - self.Kalman_Gain @ self.H @ self.P
            s_prior += self.Kalman_Gain @ -(ob[noise_id] - ob[estimate_id])

        s_posterior = s_prior
        print("post state:{}".format(s_posterior))

        # update landmark estimate
        for idx, landmark in enumerate(self.landmarks):
            estimate_id = "estimate {}".format(idx)
            self.landmarks[idx] = ob[estimate_id] + s_posterior

        return s_posterior

    def observation_update(self, s):
        true_landmarks = self.env.landmarks.values()
        observation = {}

        for idx, landmark in enumerate(true_landmarks):
            landmark = np.array(landmark)
            noise_id = "noise {}".format(idx)
            estimate_id = "estimate {}".format(idx)
            observation[estimate_id] = self.H.dot(landmark - s)
            observation[noise_id] = self.H.dot(landmark - self.s) + np.sqrt(self.R).dot(np.random.randn(*landmark.shape))

            #print("observed landmark #{}, at {} as {}".format(idx, landmark, observation[idx]))

        return observation










