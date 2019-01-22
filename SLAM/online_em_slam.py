
# coding: utf-8

# In[1]:


import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from math import *
import matplotlib.cm as cm
from timeit import default_timer as timer


# In[2]:


class Environment(object):
    def __init__(self, n_landmarks = 10, L = 400, W = 300):
        self.L = L
        self.W = W
        self.grid = np.zeros((self.W, self.L))
        self.theta = self.generate_landmarks(n_landmarks)
        self.n_landmarks = n_landmarks
        
    def generate_landmarks(self, n_landmarks = 10):
        
        x = np.random.randint(0, self.L, n_landmarks)
        y = np.random.randint(0, self.W, n_landmarks)
        c = range(1, n_landmarks + 1) 
        
        return np.sort(np.array(list(zip(x, y, c))), axis = 0)
       
    def plot_env(self):
        x, y, _ = zip(*self.theta)
        plt.grid()
        plt.scatter(x, y, marker = '*')
        
    def __str__(self):
        return '\n'.join('{} {} {}'.format(row[0], row[1], row[2]) for row in self.theta)
        


def normalize(x):
    if x > np.pi:
        return x - 2*np.pi
    
    if x < -np.pi:
        return x + 2*np.pi
    
    return x


# In[3]:


class Robot(object):
    
    def __init__(self, u, Q, R, x0, P0, env):
        '''
        y: meansurements
        u: controls
        Q: covariance matrix for measurements
        R: covariance matrix for motion
        x0: initial state
        P0: initial covariance matrix
        '''
        np.random.seed(10)
        self.u = u
        self.Q = Q
        self.R = R
        self.x0 = x0
        self.P0 = P0
        self.env = env
        self.n_lm = env.n_landmarks
        self.dt = 1
        self.T = len(u)
       
        self.log = None # execute results, contains obs & traj
        
        self.theta = env.generate_landmarks(self.n_lm)
        
        self.e_step_res = {}
        self.execute()
        
        
    
    def execute(self):
        x = self.x0
        theta = self.env.theta
        
        traj = [x]
        y = []
        log = {}
        for u_i in self.u:
            
            x = self.f(x, u_i, noise = True)
            traj.append(x)
            
            y_i = self.h(x, theta, noise = True)
            y.append(y_i)
            
        log['traj'] = traj
        log['obs'] = y
        self.log = log
        
    def visualize(self, exe = False, em = False):
        
        
        # plot env
        theta = self.env.theta
        x, y, _ = zip(*theta)
        print('true lm:')
        for i in range(len(x)):
            xi = x[i]
            yi = y[i]
            #print(xi, yi)
            plt.scatter(x[i], y[i], marker = '*', label = str(i+1))
            
        
        x, y, _ = zip(*self.theta)
        plt.scatter(x, y, marker = 'D')
        
        # plot traj
        if exe:
            traj, y = self.log['traj'], self.log['obs']
            x, y, _ = zip(*traj)
            plt.plot(x, y, 'k-o')
            
        
        # plot estimation 
        if em:
            x, y, _ = zip(*map(lambda x: x[0], [(self.x0, self.P0)] + self.e_step_res['filter']))
            plt.plot(x, y, 'c--x')
            
        
        # settings
        #plt.xlim(0, 600)
        #plt.ylim(0, 450)
        plt.grid()
        plt.legend()
        plt.show()
        
        
        #for i in range(len(y)):
            #print(y[i])
        
    # motion model
    def f(self, x, u, noise = True):
        
        dt = self.dt
        v, w = u[0], u[1]
        delta = np.zeros((3, 1))
        x_ = x.reshape(3, )
        if w != 0:
            r = v / w
            delta[0] = -r*np.sin(x_[2]) + r*np.sin(x_[2]+w*dt)
            delta[1] = r*np.cos(x_[2]) - r*np.cos(x_[2]+w*dt)
            delta[2] = w*dt  
        else:
            delta[0] = np.cos(x_[2])*dt*v
            delta[1] = np.sin(x_[2])*dt*v
            delta[2] = 0  
            
        x_new = x + delta
        if noise:
            x_new += np.random.multivariate_normal(np.zeros(3), self.R).reshape(3, 1)
            
        #x_new[2][0] = normalize(x_new[2][0])
        
        return x_new
    
    # sensor model
    def h(self, x, theta, noise = True):
        
        m, _ = theta.shape
        y = np.array([]).reshape(0, 3)
        x_ = x.reshape(3, )
        for i in range(m):
            z = np.zeros((3, 1))
            z[0] = np.linalg.norm(x_[:2] - theta[i,:2])
            z[1] = np.arctan2(theta[i,1] - x_[1], theta[i,0] - x_[0]) - x_[2]
            
            if noise:
                z += np.random.multivariate_normal(np.zeros(3), self.Q).reshape(3, 1)
            
            z[2] = theta[i,2]
            #z[1][0] = normalize(z[1][0])
            
            
            y = np.vstack((y, z.T))
            
        m, _ = y.shape
       
        return y if m != 0 else None
    
    # Jacobian of f wrt x
    def jacobian_F(self, x, u):
        
        dt = self.dt
        v, w = u[0], u[1]
        F = np.zeros((3, 3))
        x_ = x.reshape(3,)
        if w != 0:
            r = v / w
            # step1  
            F[1][2] = -r*np.sin(x_[2]) + r*np.sin(x_[2]+w*dt)
            F[0][2] = -(r*np.cos(x_[2]) - r*np.cos(x_[2]+w*dt))
        else:
            F[1][2] = np.cos(x_[2])*dt*v
            F[0][2] = -np.sin(x_[2])*dt*v
           
        return F
    
    
    # Jacobian of h wrt x
    def jacobian_H(self, x, lm):
        x_ = x.reshape(3,)
        H = np.zeros((3,3))
        dx = lm[0] - x_[0]
        dy = lm[1] - x_[1]
        q = (lm[0] - x_[0])**2 + (lm[1] - x_[1])**2
        sq = np.sqrt(q)
        H[0, :] = [-(dx)/sq, -(dy)/sq, 0]
        H[1, :] = [dy/q, -dx/q, -1]
        
        return H
    
    
    
    def e_step(self, theta_old):
        '''
        Return: Q_func = Q(theta, theta_k)
        '''
        
        x = self.x0
        P = self.P0
        y = self.log['obs']
        
        log_time = []
        log_obs = []
        log_F = []
        for t in range(T):
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
                lm = theta_old[c - 1]
                H = self.jacobian_H(x, lm)
                
                ob = ob.reshape(1, 3)
                lm = lm.reshape(1, 3)
                
                S = H @ P @ H.T + self.Q
                K = P @ H.T @ np.linalg.pinv(S)
                P = P - K @ H @ P
                x = x + K @ (ob - self.h(x, lm, False)).T
                #x[2][0] = normalize(x[2][0])
                
            # log
            log_obs.append((np.copy(x), np.copy(P)))
                
        self.e_step_res['time'] = log_time.copy()
        self.e_step_res['filter'] = log_obs.copy()
        # smooth
        log_smooth = [log_obs.pop()]
        for t in range(T - 1, 0, -1):
            F = log_F.pop()
            x_time, P_time = log_time.pop()
            x_obs, P_obs = log_obs.pop()
            x_smooth, P_smooth = log_smooth[-1]
            
            S = P_obs @ F.T @ np.linalg.pinv(P_time)
            x = x_obs + S @ (x_smooth - x_time)
            #x[2][0] = normalize(x[2][0])
            P = P_obs + S @ (P_smooth - P_time) @ S.T
        
            log_smooth.append((x, P))
            
        # for test
        self.e_step_res['smooth'] = log_smooth.copy()
        #
        
        def Q_func(theta):
            N = self.n_lm
            theta_ = theta.reshape(N, 2)
            theta_ = np.hstack((theta_, np.array(range(1, N+1)).reshape(N,1)))
            
            res = 0
            for t in range(T):
                
                #x, P = log_smooth[-t - 1]
                x, P = self.e_step_res['filter'][t]
                y = self.log['obs']
                yt_ = self.h(x, theta_, False)
                
                for i in range(N):
                    ob = y[t][i]
                    ob_ = yt_[i]
                    innov = (ob - ob_).reshape(3, 1) 
                    
                    Q_inv = np.linalg.pinv(self.Q)
                    H = self.jacobian_H(x, theta_[i])
                    
                    res += innov.T @ Q_inv @ innov
                    res += np.trace(Q_inv @ H @ P @ H.T)
            
            return res
            
        
        return Q_func
    
    def m_step(self, Q_func, theta_old):
        '''
        Return: theta_k+1
        '''
        N = self.n_lm
        theta_init = theta_old[:, :2].reshape(N * 2, )
        bnds = zip([0]*2*N, [400, 300]*N)
        
        res = minimize(Q_func, theta_init, method = 'L-BFGS-B', bounds = tuple(bnds))
        #res = minimize(Q_func, theta_init, method = 'BFGS')
        
        theta_new = np.sort(res['x'].reshape(N, 2), axis = 0)
        theta_new = np.hstack((theta_new, np.array(range(1, N+1)).reshape(N,1)))
        
        return theta_new
    
    def em_slam(self, max_iters = 100):
        
        traj, y = self.log['traj'][1:], self.log['obs']
        u = self.u
        Q, R = self.Q, self.R
        x0, P0 = self.x0, self.P0
        
        theta_true = self.env.theta
        theta_old = self.theta
        
        
        errors = []
        for i in range(max_iters):
            # execute e & m step
            Q_func = self.e_step(theta_old)
            theta_new = self.m_step(Q_func, theta_old)
            
            # cal error
            poses, _ = zip(*self.e_step_res['filter'])
            #print(poses[0].shape)
            #print()
            local_error = np.sum((np.linalg.norm(poses[i] - traj[i]) for i in range(self.T)))
            map_error = np.linalg.norm(theta_new - theta_true)

            errors.append(map_error + local_error)
            
            if (i+1) % 10 == 0:
                print('iter {}...'.format(i+1))
            
            
            # break out check
            #if i > 5 and diff > errors[-2]:
                #break
            
            if np.allclose(theta_old, theta_new, atol = 10**-1):
                break
            else:
                theta_old = theta_new
                
        self.theta = theta_new
        return errors
        
    def online_e_step(self, theta_old, lm_idx):
       
        x = self.x0
        P = self.P0
        y = self.log['obs']
        
        log_obs = []
        for t in range(self.T):
            # time update
            x = self.f(x, u[t], False)
            F = self.jacobian_F(x, u[t])
            P = F @ P @ F.T + self.R
            
            # online observation update
            for idx in lm_idx:
                ob = y[t][idx]
                c = int(ob[2])
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
            self.e_step_res['filter'] = log_obs.copy()
        def Q_func(theta):
            N = self.n_lm
            theta_ = theta.reshape(N, 2)
            theta_ = np.hstack((theta_, np.array(range(1, N+1)).reshape(N,1)))
            
            res = 0
            for t in range(T):
                x, P = log_obs[t]
                y = self.log['obs']
                yt_ = self.h(x, theta_, False)
                
                for i in lm_idx:
                    ob = y[t][i]
                    ob_ = yt_[i]
                    innov = (ob - ob_).reshape(3, 1) 
                    
                    Q_inv = np.linalg.pinv(self.Q)
                    H = self.jacobian_H(x, theta_[i])
                    
                    res += innov.T @ Q_inv @ innov
                    res += np.trace(Q_inv @ H @ P @ H.T)
            return res

        return Q_func
        
    
        
    def online_em_slam(self, max_iters = 100):

        traj, y = self.log['traj'][1:], self.log['obs']
        u = self.u
        Q, R = self.Q, self.R
        x0, P0 = self.x0, self.P0

        theta_true = self.env.theta
        theta_old = self.theta

        errors = []
        N = self.n_lm
        cnter = 0
        rand_lm_idx = list(range(N))
        for i in range(max_iters):

            np.random.seed(i)
            np.random.shuffle(rand_lm_idx)
            #p = np.random.randint(1, N-1)
            p = np.random.randint(1, 3)

            cnter += 1

            Q_func = self.online_e_step(theta_old, rand_lm_idx[:2])
            theta_new = self.m_step(Q_func, theta_old)

            # cal error
            poses, _ = zip(*self.e_step_res['filter'])
            
            local_error = np.sum((np.linalg.norm(poses[i] - traj[i]) for i in range(self.T)))
            map_error = np.linalg.norm(theta_new - theta_true)

            errors.append(map_error + local_error)
            
            if (i+1) % 10 == 0:
                print('iter {}...'.format(i+1))
            
            
            # break out check
            #if i > 5 and diff > errors[-2]:
                #break
            
            if np.allclose(theta_old, theta_new, atol = 10**-1):
                break
            else:
                theta_old = theta_new
                
        self.theta = theta_new
        return errors
    

def plot_traj(robot, ax, title = None):

    theta = robot.env.theta
    #plot ground true lm    
    x, y, _ = zip(*theta)
    colors = cm.rainbow(np.linspace(0, 1, len(y)))
    ax.scatter(x, y, c = colors, s = 36, marker = 's', label = 'true landmarks')
        
    #plot estiamted lm
    x, y, _ = zip(*robot.theta)
    ax.scatter(x, y, c = colors, s = 36, marker = '*', label = 'estimated landmarks')

    traj, y = robot.log['traj'], robot.log['obs']
    x, y, _ = zip(*traj)
    ax.plot(x, y, 'k-o', label = 'true traj')

    x, y, _ = zip(*map(lambda x: x[0], [(robot.x0, robot.P0)] + robot.e_step_res['filter']))
    ax.plot(x, y, 'c--x', label = 'estimated traj')

    ax.set_title(title)
    ax.legend()


def plot_errors(error, ax, n_iters = 100, title = None):
    x = list(range(1, n_iters+1))
    ax.plot(x, error)
    ax.set_title(title)
    ax.set_xlabel('iterations')
    ax.set_ylabel('Localization and Mapping Errors')

if __name__ == '__main__':
    print('simulation starts!')
    np.random.seed(8)
    n_iters = 10

    R = 5*np.eye(3)
    R[2][2] = 0.1

    Q = 3*np.eye(3)
    Q[1][1] = 0.1
    Q[2][2] = 0

    x0 = np.array([150, 200, 0]).reshape(3, 1)
    P0 = np.eye(3)
    P0[2][2] = 0.1

    T = 7
    u = [(70, .4) for _ in range(T)]

    env = Environment(7)
    robot1 = Robot(u, Q, R, x0, P0, env)
    robot2 = Robot(u, Q, R, x0, P0, env)

    print('em-slam is running...')
    start = timer()
    e1 = robot1.em_slam(n_iters)
    end = timer()
    print('em-slam is done.')
    print('time consumed: %.3f secs' % (end - start))


    print('online em-slam is running...')
    start = timer()
    e2 = robot2.online_em_slam(n_iters)
    end = timer()
    print('online em-slam is done.')
    print('time consumed: %.3f secs' % (end - start))
    
    fig = plt.figure(figsize = (12, 9))
    ax1 = fig.add_subplot(2, 2, 1)
    ax2 = fig.add_subplot(2, 2, 2)
    ax3 = fig.add_subplot(2, 2, 3)
    ax4 = fig.add_subplot(2, 2, 4)

    plot_traj(robot1, ax1, 'em-slam')
    plot_traj(robot2, ax2, 'online em-slam')
    plot_errors(e1, ax3, n_iters, 'em-slam')
    plot_errors(e2, ax4, n_iters, 'online_em_slam')
    plt.show()





