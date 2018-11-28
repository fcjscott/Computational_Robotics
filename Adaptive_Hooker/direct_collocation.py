from animation import animation
from utility import *

N = 100
NC = N*2
T = 10
time_line = np.arange(0, T, T/NC)
tail_mass = 1
tail_w = 0.05
block_mass = 5
block_w = 0.08
rho = tail_w/2
g = 9.81
I_b = 1/12 * block_mass * 2 * block_w ** 2
I_t = block_mass * tail_mass * (tail_w/2) ** 2 /(block_mass + tail_mass)

# bounds for motor torque
b = (0.0, 0.05)
bx = (-10000, 10000)





# objective function
def objective(decision_variable):
    u = decision_variable[:N]

    return u.dot(u)


# dynamic constraints
def constraint1(decision_variable):

    # N equalities constraint for x2dot
    u = decision_variable[:N]

    # extract x2dot from decision variable
    id_x2dot = np.arange(N+8, 25*N, 24)
    x2dot = decision_variable[id_x2dot]

    # extract theta block from decision variable
    id_thetablock = np.arange(N+3, 25*N, 24)
    theta_block = decision_variable[id_thetablock]

    # extract theta tail from decision variable
    id_thetatail = np.arange(N+4, 25*N, 24)
    theta_tail = decision_variable[id_thetatail]

    return np.array(x2dot - u / (rho * block_mass) * np.sin(theta_block + theta_tail))


def constraint2(decision_variable):

    # N equalities constraint for x2dot
    u = decision_variable[:N]

    # extract y2dot from decision variable
    id_y2dot = np.arange(N + 9, 25 * N, 24)
    y2dot = decision_variable[id_y2dot]

    # extract theta block from decision variable
    id_thetablock = np.arange(N + 3, 25 * N, 24)
    theta_block = decision_variable[id_thetablock]

    # extract theta tail from decision variable
    id_thetatail = np.arange(N + 4, 25 * N, 24)
    theta_tail = decision_variable[id_thetatail]

    return np.array(y2dot - g - u / (rho * block_mass) * np.cos(theta_block + theta_tail))


def constraint3(decision_variable):

    # extract motor input
    u = decision_variable[:N]

    # extract block angular acceleration from decision variable
    id_alphablock = np.arange(N+10, 25*N, 24)
    alpha_block = decision_variable[id_alphablock]

    return np.array(alpha_block - u/I_b)


def constraint4(decision_variable):

    # extract motor input
    u = decision_variable[:N]

    # extract tail angular acceleration from decision variable
    id_alphatail = np.arange(N + 10, 25 * N, 24)
    alpha_tail = decision_variable[id_alphatail]

    # N equalities constraint for tail angle
    return np.array(alpha_tail - u/I_b)


# direct collocation constraints
def constraint5(decision_variable):
    id_X = []
    id_Xdot = []
    for i in range(N, 25*N-12, 12):
        id_X.append(np.arange(i, i+8))
        id_Xdot.append(np.arange(i+4, i+12))

    id_X = np.concatenate((id_X), axis=None)
    id_Xdot = np.concatenate((id_Xdot), axis=None)

    X = decision_variable[id_X]
    X = X.reshape((8, NC-1))
    Xdot = decision_variable[id_Xdot]
    Xdot = Xdot.reshape((8, NC-1))

    delta = np.zeros((8, N))
    for k in range(0, NC-2, 2):
        h = time_line[k + 2] - time_line[k]
        for state_id in range(8):
           delta[state_id][int((k+2)/2)] = X[state_id][k] - X[state_id][k+2] + h/6 * \
                                (Xdot[state_id][k] + 4*Xdot[state_id][k+1] +
                                Xdot[state_id][k+2])

    return np.array(delta.reshape((8*N, 1)))


def constraint6(decision_variable):
    xf = decision_variable[25*N-12:25*N-8]
    final_state = [100, 100, 0, 0]

    return np.array(xf - final_state)



# bounds for motor torque
bnds = []
for i in range(25*N):
    if i < N:
        bnds.append(b)
    else:
        bnds.append(bx)


con1 = NonlinearConstraint(constraint1, lb=0, ub=0)
con2 = NonlinearConstraint(constraint2, lb=0, ub=0)
con3 = NonlinearConstraint(constraint3, lb=0, ub=0)
con4 = NonlinearConstraint(constraint4, lb=0, ub=0)
con5 = NonlinearConstraint(constraint5, lb=0, ub=0)
con6 = NonlinearConstraint(constraint6, lb=0, ub=0)

cons=[con1, con2, con3, con4, con5, con6]

x0 = np.random.rand(25*N)
sol = opt.minimize(objective, x0, method='SLSQP', bounds=bnds, constraints=cons)


