from utility import *
class Robot(object):
    def __init__(self, u, Q, R, x0, P0, env, path):
        '''
        y: meansurements
        u: controls
        Q: covariance matrix for measurements
        R: covariance matrix for motion
        x0: initial state
        P0: initial covariance matrix
        '''
        self.u = u
        self.Q = Q
        self.R = R
        self.x0 = x0
        self.P0 = P0
        self.env = env
        self.n_lm = env.n_landmarks
        self.dt = 1
        self.T = len(u)
        self.log = None
        # self.theta = env.theta

        # landmarks
        self.theta = env.generate_landmarks(self.n_lm)
        self.theta0 = self.theta

        self.path = path

        '''
        N = self.n_lm
        c3 = np.array(range(1, N+1)).reshape(N,1)
        mean = env.theta[:, :2].reshape(N*2, )
        cov = 200 * np.eye(2*N)
        sample = np.random.multivariate_normal(mean, cov).reshape(N, 2)
        self.theta = np.hstack((sample, c3))
        '''
        self.e_step_res = {}
        self.img = 0

    def execute(self):
        x = self.x0
        theta = self.env.theta

        traj = [x]
        y = []
        log = {}

        # for each input command, execute robot
        for u_i in self.u:

            # motion model: x = f(x, u)
            x = self.f(x, u_i, noise=True)
            traj.append(x)

            # measurement model: y = h(x)
            y_i = self.h(x, theta, noise=True)
            y.append(y_i)

        # store trajectory and observation in a log
        log['traj'] = traj
        log['obs'] = y
        self.log = log

    def visualize(self, exe=False, em=False):

        plt.clf()

        # plot env
        theta = self.env.theta

        x, y, _ = zip(*theta)
        colors = cm.rainbow(np.linspace(0, 1, len(y)))
        plt.scatter(x, y, c=colors, s=36, marker='s', label='true landmarks')

        x, y, _ = zip(*self.theta)
        plt.scatter(x, y, c=colors, s=36, marker='*', label='estimated landmarks')

        # plot traj
        if exe:
            traj, y = self.log['traj'], self.log['obs']
            x, y, _ = zip(*traj)
            plt.plot(x, y, 'k-o', label='true traj')

        # plot estimation
        if em:
            x, y, _ = zip(*map(lambda x: x[0], [(self.x0, self.P0)] + self.e_step_res['filter']))
            plt.plot(x, y, 'c--x', label='estimated traj')

        # settings
        plt.title('iteration {}'.format(self.img + 1))
        plt.xlim(-100, 800)
        plt.ylim(-100, 500)
        plt.grid()
        plt.legend()
        plt.pause(0.1)
        plt.savefig('./{}/image{}.png'.format(self.path, self.img), dpi=100, format='png')
        self.img += 1

    # motion model
    def f(self, x, u, noise=True):

        dt = self.dt

        # v = velocity, w = direction
        v, w = u[0], u[1]

        delta = np.zeros((3, 1))
        x_ = x.reshape(3, )
        if w != 0:
            r = v / w
            delta[0] = -r * np.sin(x_[2]) + r * np.sin(x_[2] + w * dt)
            delta[1] = r * np.cos(x_[2]) - r * np.cos(x_[2] + w * dt)
            delta[2] = w * dt
        else:
            delta[0] = np.cos(x_[2]) * dt * v
            delta[1] = np.sin(x_[2]) * dt * v
            delta[2] = 0

        x_new = x + delta
        if noise:
            x_new += np.random.multivariate_normal(np.zeros(3), self.R).reshape(3, 1)

        return x_new

    # sensor model
    def h(self, x, theta, noise=True):

        m, _ = theta.shape
        y = np.array([]).reshape(0, 3)
        x_ = x.reshape(3, )
        for i in range(m):
            z = np.zeros((3, 1))

            # z[0] represents distance for ith landmark to robot
            z[0] = np.linalg.norm(x_[:2] - theta[i, :2])

            # z[1] represents robot's angle
            z[1] = np.arctan2(theta[i, 1] - x_[1], theta[i, 0] - x_[0]) - x_[2]

            if noise:
                z += np.random.multivariate_normal(np.zeros(3), self.Q).reshape(3, 1)

            # z[2] represents landmarks' ID
            z[2] = theta[i, 2]

            '''
            if z[1] > np.pi:
                z[1] = z[1] - 2*np.pi
            if z[1] < -np.pi:
                z[1] = z[1] + 2*np.pi
            '''
            # if z[0] < self.range:
            y = np.vstack((y, z.T))

        m, _ = y.shape

        return y if m != 0 else None

    # Jacobian of f wrt x
    def jacobian_F(self, x, u):

        dt = self.dt
        v, w = u[0], u[1]
        F = np.zeros((3, 3))
        x_ = x.reshape(3, )
        if w != 0:
            r = v / w
            # step1
            F[1][2] = -r * np.sin(x_[2]) + r * np.sin(x_[2] + w * dt)
            F[0][2] = -(r * np.cos(x_[2]) - r * np.cos(x_[2] + w * dt))
        else:
            F[1][2] = np.cos(x_[2]) * dt * v
            F[0][2] = -np.sin(x_[2]) * dt * v

        return F

    # Jacobian of h wrt x
    def jacobian_H(self, x, lm):
        x_ = x.reshape(3, )
        H = np.zeros((3, 3))
        dx = lm[0] - x_[0]
        dy = lm[1] - x_[1]
        q = (lm[0] - x_[0]) ** 2 + (lm[1] - x_[1]) ** 2
        sq = np.sqrt(q)
        H[0, :] = [-(dx) / sq, -(dy) / sq, 0]
        H[1, :] = [dy / q, -dx / q, -1]

        return H

    def e_step(self, theta_old, u):
        '''
        Return: Q_func = Q(theta, theta_k)
        '''

        x = self.x0
        P = self.P0
        y = self.log['obs']

        log_time = []
        log_obs = []
        log_F = []
        for t in range(self.T):
            # time update
            x = self.f(x, u[t], False)
            F = self.jacobian_F(x, u[t])
            P = F @ P @ F.T + self.R

            # log
            log_F.append(np.copy(F))
            log_time.append((np.copy(x), np.copy(P)))

            # observation update
            for ob in y[t]:

                c = int(ob[2])

                # extract landmark for certain observation
                lm = theta_old[c - 1]
                H = self.jacobian_H(x, lm)

                ob = ob.reshape(1, 3)
                lm = lm.reshape(1, 3)

                S = H @ P @ H.T + self.Q
                K = P @ H.T @ np.linalg.pinv(S)
                P = P - K @ H @ P
                x = x + K @ (ob - self.h(x, lm, False)).T

            # log
            log_obs.append((np.copy(x), np.copy(P)))

        self.e_step_res['time'] = log_time.copy()
        self.e_step_res['filter'] = log_obs.copy()

        # smooth
        log_smooth = [log_obs.pop()]
        for t in range(self.T - 1, 0, -1):
            F = log_F.pop()
            x_time, P_time = log_time.pop()
            x_obs, P_obs = log_obs.pop()
            x_smooth, P_smooth = log_smooth[-1]

            S = P_obs @ F.T @ np.linalg.pinv(P_time)
            x = x_obs + S @ (x_smooth - x_time)
            # print('-'*80)
            # print(x_smooth - x_time)
            # print('#'*80)
            # print(S @ (x_smooth - x_time))
            P = P_obs + S @ (P_smooth - P_time) @ S.T

            log_smooth.append((x, P))

        # for test
        self.e_step_res['smooth'] = log_smooth.copy()

        #

        # assemble Q_func
        # Q_func = self.assemble()
        def Q_func(theta):
            N = self.n_lm
            theta_ = theta.reshape(N, 2)
            theta_ = np.hstack((theta_, np.array(range(1, N + 1)).reshape(N, 1)))

            res = 0
            for t in range(self.T):

                # x, P = log_smooth[-t - 1]

                # comment out to debug for M step
                x, P = self.e_step_res['filter'][t]
                y = self.log['obs'][t]
                yt_ = self.h(x, theta_, False)

                # Debugging code for M step
                #x = self.log['traj'][t+1]
                #P = np.ones(shape=(3,3))*0.0
                #y = self.log['obs'][t]
                #yt_ = self.h(x, theta_, False)

                for i in range(N):
                    ob = y[i]
                    ob_ = yt_[i]

                    innov = (ob - ob_).reshape(3,1)


                    Q_inv = np.linalg.pinv(self.Q)
                    R_inv = np.linalg.pinv(self.R)

                    H = self.jacobian_H(x, theta_[i])

                    res += innov.T @ Q_inv @ innov
                    #res += innov
                    res += np.trace(R_inv @ H @ P @ H.T)
            return res

        return Q_func

    def m_step(self, Q_func, theta_old):
        '''
        Return: theta_k+1
        '''
        # theta0 = self.theta0.reshape(self.n_lm * 3, )
        # print(theta0)
        # theta_init = np.delete(theta0, [i for i in range(2, len(theta0), 3)])
        # print(theta_init)
        # theta_init = self.env.generate_landmarks().reshape(self.n_lm * 3)
        # def Q_func_wrapper(theta):
        # return Q_func(theta.reshape(self.n_lm, 3))
        N = self.n_lm
        theta_init = theta_old[:, :2].reshape(N * 2, )
        print("-"*10)
        bnds = zip([0] * 2 * N, [400, 300] * N)

        res = minimize(Q_func, theta_init, method='L-BFGS-B', bounds=tuple(bnds))

        return np.hstack((res['x'].reshape(N, 2), np.array(range(1, N + 1)).reshape(N, 1)))

    def em_slam(self, max_iters=100, plot=False):

        y, u = self.log['obs'], self.u
        Q, R = self.Q, self.R
        x0, P0 = self.x0, self.P0

        theta_true = self.env.theta

        # Debugging theta old
        #print(theta_true)
        #theta_old = copy.deepcopy(theta_true)
        #print(theta_old)
        #self.theta = copy.deepcopy(theta_true)

        theta_old = self.theta

        res = {}
        errors = []
        for i in range(max_iters):
            # execute e & m step
            Q_func = self.e_step(theta_old, u)
            self.visualize(exe=True, em=True)
            theta_new = self.m_step(Q_func, theta_old)

            # cal error
            diff = np.abs(np.linalg.norm(theta_new - theta_true))
            errors.append(diff)

            print('iteration{}: '.format(i + 1))
            print('mapping error: ', diff)

            self.theta = theta_new
            self.visualize(exe=True, em=True)

            # break out check
            #if i > 5 and diff > errors[-2]:
            #   break

            if np.allclose(theta_old, theta_new, atol=10**-2):
                break
            else:
                theta_old = theta_new

        self.theta = theta_new

        if plot:
            plt.figure('error plot')
            plt.plot(errors)
            plt.xlabel('iterations')
            plt.ylabel('RSS error of map')
