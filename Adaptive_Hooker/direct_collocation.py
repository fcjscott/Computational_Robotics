from animation import animation
from dynamics import MyController
from utility import *


# global parameters
N = 50
T = 5
time_line = np.arange(0, T, T/N)
tail_mass = 1
tail_w = 0.05
block_mass = 5
block_w = 0.08
rho = tail_w/2
g = 9.81
I_b = 1/12 * block_mass * 2 * block_w ** 2
I_t = block_mass * tail_mass * (tail_w/2) ** 2 /(block_mass + tail_mass)

# bounds for motor torque
b = (-0.1, 0.1)
bx = (-1000, 1000)


# objective function
def objective(decision_variable):

    # minimizing input energy
    u = decision_variable[:N]

    return u.dot(u)


# defines system dynamics
def system_dynamics(state, motortorque):

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
    X State: x,y,theta body,theta tail, xdot, ydot, theta body dot, theta tail dot
    Xdot f(): xdot, ydot, theta body dot, theta tail dot, x2dot, y2dot, theta body 2dot, theta tail 2dot,
    """
    decision_variable_copy = decision_variable.copy()

    delta = []
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
    xi = decision_variable[N:N+6]
    initial_state = [0, 0, np.pi/3, 0, 40*np.cos(np.pi/3), 40*np.sin(np.pi/3)]

    return np.array(xi - initial_state)


#final state constraint
def constraint3(decision_variable):
    #xf = decision_variable[25*N-12:25*N-9]
    #final_state = [141.2, 0.0827, 0]
    xf = decision_variable[9*N-6]
    xf = np.mod(xf, 2*np.pi)

    return xf




# bounds for motor torque
bnds = []
for i in range(9*N):
    if i < N:
        bnds.append(b)
    else:
        bnds.append(bx)


cons = ({'type': 'eq', 'fun': constraint1},
{'type': 'eq', 'fun':constraint2},
{'type': 'eq', 'fun':constraint3},)

# optimization process
np.random.seed(8)
x0 = np.asarray(np.random.rand(9*N,1))
sol = opt.minimize(objective, x0, method='SLSQP',bounds=bnds, constraints=cons, options={'maxiter': 500})
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

fig, axs = plt.subplots(9, 1)
t = np.arange(0, T, T/N)
axs[0].set_title('State Trajactory')
axs[0].plot(t, pos_x)
axs[0].set_ylabel('x(t)')
axs[1].plot(t, pos_y)
axs[1].set_ylabel('z(t)')
axs[2].plot(t, theta_block)
axs[2].set_ylabel('\u03F4b(t)')
axs[3].plot(t, theta_tail)
axs[3].set_ylabel('\u03F4t(t)')
axs[4].plot(t, vel_x)
axs[4].set_ylabel('velx(t)')
axs[5].plot(t, vel_y)
axs[5].set_ylabel('vely(t)')
axs[6].plot(t, omega_block)
axs[6].set_ylabel('wb(t)')
axs[7].plot(t, omega_tail)
axs[7].set_ylabel('wt(t)')
axs[8].plot(t, u)
axs[8].set_title('Input Trajactory')
axs[8].set_ylabel('tao(t)')
axs[8].set_xlabel('Time')

block_traj = []
tail_traj = []
for i in range(N):
    block_traj.append((pos_x[i], pos_y[i], theta_block[i]))
    tail_traj.append(theta_tail[i])


# plot animation
object = MyController(input_torque=u, T=T)
#block_traj, tail_traj = object.dynamics()
animation(block_traj, theta_tail, [0, 0, 0])


