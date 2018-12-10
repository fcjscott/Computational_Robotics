from animation import animation
from utility import *


# global parameters
N = 50
tail_mass = 1
tail_w = 0.05
block_mass = 5
block_w = 0.08
rho = tail_w/2
g = 9.81
I_b = 1/12 * block_mass * 2 * block_w ** 2
I_t = block_mass * tail_mass * (tail_w/2) ** 2 /(block_mass + tail_mass)

# bounds for motor torque
b = (-0.127, 0.127)
bx = (-100, 100)
angle_bnd = (-np.pi, np.pi)
vel_bnd = (0, 80)
# Target
goal_position = [100, 0]


# objective function
def objective(decision_variable):

    # minimizing input energy
    u = decision_variable[:N]

    return u.dot(u)


# defines system dynamics
def system_dynamics(state, motortorque):
    """

    :param state: Xi for i in 1 to N. Xi = [xi, yi, theta_bi, theta_ti, dot{xi}, dot{yi}, dot{theta_bi}, dot{theta_ti}]
    :param motortorque: ui for i in 1 to N
    :return: dot{Xi} = f(Xi, ui)
    """

    # naive iteration
    f = np.zeros(8)
    f[0] = state[4]
    f[1] = state[5]
    f[2] = state[6]
    f[3] = state[7]
    f[4] = motortorque/(rho*block_mass) * np.sin(state[2] + state[3])
    f[5] = -g - motortorque/(rho*block_mass) * np.cos(state[2] + state[3])
    f[6] = 1/I_b * motortorque
    f[7] = -1/I_t * motortorque

    return f


# delta constraint
def constraint1(decision_variable):
    """

    :param decision_variable: All inputs and states [u0, u1, ..., uN, X1, X2, ..., XN].
    In total 9N decision variables.
    :return: delta (from direct collocation)
    """

    # Naive iteration
    decision_variable_copy = decision_variable.copy()
    delta = []
    T = decision_variable_copy[-1]
    time_line = np.arange(0.0, T, T/N, dtype='float32')
    for k in range(N-1):
        uk = decision_variable_copy[k]
        xk = decision_variable_copy[(8*k+N):(8*(k+1)+N)]
        h = time_line[k + 1] - time_line[k]
        xkp1 = decision_variable_copy[(8*(k+1)+N):(8*(k+2)+N)]
        ukp1 = decision_variable_copy[k+1]
        fk = system_dynamics(xk, uk)
        fkp1 = system_dynamics(xkp1, ukp1)
        fc = system_dynamics(0.5*(xk + xkp1), 0.5*(uk + ukp1))
        delta_k = xk - xkp1 + h/6 * (fk + 4*fc + fkp1)
        delta.append(delta_k)
    delta = np.concatenate(delta, axis=None)

    return delta


# initial state constraint

def constraint2(decision_variable):
    pos = decision_variable[N:N+2]
    #initial_state = [0, 0, np.pi/3, 0, 40*np.cos(np.pi/3), 40*np.sin(np.pi/3),0,0]
    tail_angle = decision_variable[N+3]
    rot = decision_variable[N+6:N+8]


    return np.concatenate((pos, tail_angle, rot), axis=None)


#final state constraint
def constraint3(decision_variable):
    #xf = decision_variable[25*N-12:25*N-9]
    #final_state = [141.2, 0.0827, 0]
    pos = decision_variable[-9:-7]
    angle = decision_variable[-7]
    omega = decision_variable[-3]
    angle = np.mod(angle, 2*np.pi)

    #final_state = np.concatenate(final_state, axis=None)

    return np.concatenate((pos-goal_position, angle, omega), axis=None)


def constraint4(decision_variable):
    body_angle = decision_variable[N+2]

    # angle normalize [-pi, pi]
    body_angle = body_angle % (2*np.pi)
    if body_angle >= np.pi:
        body_angle -= 2 * np.pi

    x_vel = decision_variable[N+4]
    y_vel = decision_variable[N+5]
    angle = np.arctan2(y_vel, x_vel)

    return body_angle - angle


# bounds for motor torque
bnds = []
for i in range(9*N+1):
    if i < N:
        bnds.append(b)
    elif i == N+2:
        bnds.append(angle_bnd)
    elif i == N+4 or i == N+5:
        bnds.append(vel_bnd)
    elif i == 9*N:
        bnds.append((0.01, 10))
    else:
        bnds.append(bx)


cons = ({'type': 'eq', 'fun': constraint1},
{'type': 'eq', 'fun':constraint2},
{'type': 'eq', 'fun':constraint3},
{'type': 'eq', 'fun':constraint4})

# optimization process
# initial guess on decision variables
np.random.seed(8)
u0 = np.asarray(np.random.uniform(-0.127, 0.127, N))
x0 = np.asarray(np.random.rand(8*N))
T0 = 5

X0 = np.concatenate((u0, x0, T0), axis=None)
print(X0)
sol = opt.minimize(objective, X0, method='SLSQP',bounds=bnds, constraints=cons, options={'maxiter': 500})
print(sol)
x = sol.x


# Get output
u = x[:N]
pos_x = x[np.arange(N, 9*N, 8)]
pos_y = x[np.arange(N+1, 9*N, 8)]
theta_block = x[np.arange(N+2, 9*N, 8)]
theta_tail = x[np.arange(N+3, 9*N, 8)]
vel_x = x[np.arange(N+4, 9*N, 8)]
vel_y = x[np.arange(N+5, 9*N, 8)]
omega_block = x[np.arange(N+6, 9*N, 8)]
omega_tail = x[np.arange(N+7, 9*N, 8)]


fig, axs = plt.subplots(4, 2, figsize=(16, 8))
fig.suptitle("State Trajectories")
T = x[-1]
print(T)
t = np.arange(0, T, T/N)
axs[0][0].plot(t, pos_x)
axs[0][0].set_ylabel('x(t)')
axs[1][0].plot(t, pos_y)
axs[1][0].set_ylabel('y(t)')
axs[2][0].plot(t, theta_block)
axs[2][0].set_ylabel('$\\theta_b$')
axs[3][0].plot(t, theta_tail)
axs[3][0].set_ylabel('$\\theta_t$')
axs[0][1].plot(t, vel_x)
axs[0][1].set_ylabel('$vel_x$')
axs[1][1].plot(t, vel_y)
axs[1][1].set_ylabel('$vel_y$')
axs[2][1].plot(t, omega_block)
axs[2][1].set_ylabel('$\omega_b(t)$')
axs[3][1].plot(t, omega_tail)
axs[3][1].set_ylabel('$\omega_t(t)$')

fig2 = plt.figure("Input Trajectory")
plt.plot(t, u)
plt.ylabel("torque")
plt.xlabel('Time (s)')

block_traj = []
tail_traj = []
for i in range(N):
    block_traj.append((pos_x[i], pos_y[i], theta_block[i]))
    tail_traj.append(theta_tail[i])

animation(block_traj, tail_traj, goal_position)

