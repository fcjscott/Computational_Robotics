from animation import animation
from dynamics import MyController
from utility import *

# global parameters
N = 50
T = 1
time_line = np.arange(0, T, T / N)
tail_mass = 1
tail_w = 0.05
block_mass = 5
block_w = 0.08
rho = tail_w / 2
g = 9.81
I_b = 1 / 12 * block_mass * 2 * block_w ** 2
I_t = block_mass * tail_mass * (tail_w / 2) ** 2 / (block_mass + tail_mass)


# bounds for motor torque
b = (-0.127, 0.127)
bx = (-100, 100)
angle_bnd = (-np.pi, np.pi)

# provide target state proposed to achieve
final_state = [1.95745428e+01,  2.92362288e+01, 0]


# objective function
def objective(decision_variable):
    # minimizing input energy
    u = decision_variable[:N]

    return u.dot(u)


# defines system dynamics
def system_dynamics(state, motortorque):
    """

    :param state: [x, y, theta_b, theta_t, xdot, ydot, omega_b, omega_t]
    :param motortorque: u
    :return: [xdot, ydot, omega_b, omega_t, x2dot, y2dot, alpha_b, alpha_t]
    """

    """
    # matrix form
    f = np.zeros((8, N-1))
    f[0, :] = state[4, :]
    f[1, :] = state[5, :]
    f[2, :] = state[6, :]
    f[3, :] = state[7, :]
    f[4, :] = motortorque / (rho*block_mass) * np.sin(state[2, :] + state[3, :])
    f[5, :] = -g - motortorque / (rho*block_mass) * np.cos(state[2, :] + state[3, :])
    f[6, :] = 1/I_b * motortorque
    f[7, :] = -1/I_t * motortorque

    """

    # naive iteration
    f = np.zeros(8)
    f[0] = state[4]
    f[1] = state[5]
    f[2] = state[6]
    f[3] = state[7]
    f[4] = motortorque / (rho * block_mass) * np.sin(state[2] + state[3])
    f[5] = -g - motortorque / (rho * block_mass) * np.cos(state[2] + state[3])
    f[6] = 1 / I_b * motortorque
    f[7] = -1 / I_t * motortorque

    return f


# delta constraint
def constraint1(decision_variable):
    """
    X State: x,y,theta body,theta tail, xdot, ydot, theta body dot, theta tail dot
    Xdot f(): xdot, ydot, theta body dot, theta tail dot, x2dot, y2dot, theta body 2dot, theta tail 2dot,
    """

    """

    # matrix form
    decision_variable_copy = decision_variable.copy()
    decision_variable_copy = decision_variable_copy.reshape((9, N))
    h = T/N
    uk = decision_variable_copy[0, :-1]
    xk = decision_variable_copy[1:, :-1]
    ukp1 = decision_variable_copy[0, 1:]
    xkp1 = decision_variable_copy[1:, 1:]
    fk = system_dynamics(xk, uk)
    fkp1 = system_dynamics(xkp1, ukp1)
    fc = system_dynamics(0.5 * (xk + xkp1), 0.5 * (uk + ukp1))
    delta = xk - xkp1 + h / 6 * (fk + 4 * fc + fkp1)
    delta = np.concatenate(delta, axis=None)

    """

    # Naive iteration
    decision_variable_copy = decision_variable.copy()
    delta = []
    for k in range(N - 1):
        uk = decision_variable_copy[k]
        xk = decision_variable_copy[(8 * k + N):(8 * (k + 1) + N)]
        h = time_line[k + 1] - time_line[k]
        xkp1 = decision_variable_copy[(8 * (k + 1) + N):(8 * (k + 2) + N)]
        ukp1 = decision_variable_copy[k + 1]
        fk = system_dynamics(xk, uk)
        fkp1 = system_dynamics(xkp1, ukp1)
        fc = system_dynamics(0.5 * (xk + xkp1), 0.5 * (uk + ukp1))
        delta_k = xk - xkp1 + h / 6 * (fk + 4 * fc + fkp1)
        delta.append(delta_k)
    delta = np.concatenate(delta, axis=None)

    return delta


# initial state constraint
def constraint2(decision_variable):
    pos_i = decision_variable[N:N + 2]
    tail_angle = decision_variable[N+3]
    omega_block = decision_variable[N+6]
    omega_tail = decision_variable[N+7]
    xi = np.concatenate((pos_i, tail_angle, omega_block, omega_tail), axis=None)

    return xi


# final state constraint
def constraint3(decision_variable):
    # xf = decision_variable[25*N-12:25*N-9]
    # final_state = [141.2, 0.0827, 0]
    xf = decision_variable[-8:-5]
    xf[2] = np.mod(xf[2], 2 * np.pi)

    return np.array(xf - final_state)

def constraint4(decision_variable):
    body_angle = decision_variable[N+2]

    x_vel = decision_variable[N+4]
    y_vel = decision_variable[N+5]
    angle = np.arctan2(y_vel, x_vel)

    return body_angle - angle


# bounds for motor torque
bnds = []
for i in range(9 * N):
    if i < N:
        bnds.append(b)
    elif i == N+2:
        bnds.append(angle_bnd)
    else:
        bnds.append(bx)

cons = ({'type': 'eq', 'fun': constraint1},
        {'type': 'eq', 'fun': constraint2},
        {'type': 'eq', 'fun': constraint3},
        {'type': 'eq', 'fun': constraint4})

# optimization process
np.random.seed(8)
x0 = np.asarray(np.random.rand(9 * N, 1))
sol = opt.minimize(objective, x0, method='SLSQP', bounds=bnds, constraints=cons, options={'maxiter': 500})
print(sol)
x = sol.x

# Get output
u = x[:N]
pos_x = x[np.arange(N, 9 * N, 8)]
pos_y = x[np.arange(N + 1, 9 * N, 8)]
theta_block = x[np.arange(N + 2, 9 * N, 8)]
theta_tail = x[np.arange(N + 3, 9 * N, 8)]
vel_x = x[np.arange(N + 4, 9 * N, 8)]
vel_y = x[np.arange(N + 5, 9 * N, 8)]
omega_block = x[np.arange(N + 6, 9 * N, 8)]
omega_tail = x[np.arange(N + 7, 9 * N, 8)]

fig, axs = plt.subplots(4, 2, figsize=(16, 8))
fig.suptitle("State Trajectories")
t = np.arange(0, T, T / N)
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

target_pt = (block_traj[-1][0], block_traj[-1][1])
animation(block_traj, tail_traj, target_pt)