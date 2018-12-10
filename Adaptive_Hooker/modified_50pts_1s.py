from utility import *
from dynamics import MyController
x = np.array([-2.45968992e-04, -1.01687363e-03, -2.07834851e-03, -2.23983547e-03,
       -2.20508282e-03, -3.24878932e-03, -5.72351612e-03, -9.00683364e-03,
       -1.18758316e-02, -1.33109277e-02, -1.38609479e-02, -1.47337314e-02,
       -1.68341325e-02, -1.99560827e-02, -2.27742160e-02, -2.41117363e-02,
       -2.39288022e-02, -2.29490225e-02, -2.23609875e-02, -2.25713633e-02,
       -2.30109040e-02, -2.24959259e-02, -2.02014097e-02, -1.67797791e-02,
       -1.38714241e-02, -1.23593752e-02, -1.24526003e-02, -1.34598669e-02,
       -1.35394028e-02, -1.19513219e-02, -8.89885988e-03, -5.71742170e-03,
       -3.94189478e-03, -4.24863187e-03, -6.11635013e-03, -8.56262167e-03,
       -1.05423857e-02, -1.17906527e-02, -1.17518167e-02, -9.94231082e-03,
       -6.15093407e-03, -5.51522815e-04,  6.23097688e-03,  1.24915477e-02,
        1.63540704e-02,  1.71566211e-02,  1.53657474e-02,  1.17567011e-02,
        7.08322324e-03,  2.38062324e-03,  4.82090036e-19,  5.94877008e-19,
        1.04714684e+00, -7.19292424e-19,  2.00033289e+01,  3.46427250e+01,
       -5.76133038e-19,  5.56470344e-18,  4.00065704e-01,  6.90893004e-01,
        1.04712316e+00,  2.42465784e-04,  2.00032414e+01,  3.44465755e+01,
       -2.36782992e-03,  2.42465784e-02,  8.00128387e-01,  1.37786375e+00,
        1.04701777e+00,  1.32168000e-03,  2.00030269e+01,  3.42504991e+01,
       -8.17137145e-03,  8.36748436e-02,  1.20018593e+00,  2.06091345e+00,
        1.04677338e+00,  3.82426820e-03,  2.00027273e+01,  3.40544712e+01,
       -1.62679664e-02,  1.66583976e-01,  1.60023739e+00,  2.74004264e+00,
        1.04636468e+00,  8.00937203e-03,  2.00024185e+01,  3.38584473e+01,
       -2.46021882e-02,  2.51926407e-01,  2.00028196e+00,  3.41525173e+00,
        1.04577037e+00,  1.40950436e-02,  2.00020384e+01,  3.36624617e+01,
       -3.48281985e-02,  3.56640752e-01,  2.40031645e+00,  4.08654244e+00,
        1.04490558e+00,  2.29505413e-02,  2.00014108e+01,  3.34666099e+01,
       -5.16512711e-02,  5.28909017e-01,  2.80033430e+00,  4.75391826e+00,
        1.04359636e+00,  3.63569488e-02,  2.00003746e+01,  3.32709712e+01,
       -7.92706769e-02,  8.11731732e-01,  3.20032699e+00,  5.41738341e+00,
        1.04161939e+00,  5.66010552e-02,  1.99988938e+01,  3.30755446e+01,
       -1.18425674e-01,  1.21267890e+00,  3.60028680e+00,  6.07694123e+00,
        1.03877863e+00,  8.56904910e-02,  1.99970877e+01,  3.28802376e+01,
       -1.65650848e-01,  1.69626468e+00,  4.00020878e+00,  6.73259302e+00,
        1.03495614e+00,  1.24832785e-01,  1.99951106e+01,  3.26849409e+01,
       -2.16598115e-01,  2.21796470e+00,  4.40008982e+00,  7.38433850e+00,
        1.03008803e+00,  1.74682257e-01,  1.99929935e+01,  3.24896069e+01,
       -2.70213139e-01,  2.76698254e+00,  4.79992587e+00,  8.03217700e+00,
        1.02409187e+00,  2.36082938e-01,  1.99906110e+01,  3.22942435e+01,
       -3.29402883e-01,  3.37308553e+00,  5.19970978e+00,  8.67610789e+00,
        1.01681399e+00,  3.10608370e-01,  1.99877797e+01,  3.20988456e+01,
       -3.98384537e-01,  4.07945766e+00,  5.59943189e+00,  9.31612966e+00,
        1.00804511e+00,  4.00401740e-01,  1.99844317e+01,  3.19033311e+01,
       -4.78503847e-01,  4.89987939e+00,  5.99908327e+00,  9.95223853e+00,
        9.97595921e-01,  5.07401431e-01,  1.99807065e+01,  3.17075562e+01,
       -5.66415007e-01,  5.80008967e+00,  6.39865899e+00,  1.05844280e+01,
        9.85366860e-01,  6.32627008e-01,  1.99768654e+01,  3.15113922e+01,
       -6.56491017e-01,  6.72246801e+00,  6.79815906e+00,  1.12126897e+01,
        9.71358081e-01,  7.76076910e-01,  1.99731411e+01,  3.13147753e+01,
       -7.44386938e-01,  7.62252225e+00,  7.19758678e+00,  1.18370143e+01,
        9.55620779e-01,  9.37226877e-01,  1.99696312e+01,  3.11176827e+01,
       -8.29343207e-01,  8.49247444e+00,  7.59694637e+00,  1.24573919e+01,
        9.38191434e-01,  1.11570338e+00,  1.99663277e+01,  3.09200755e+01,
       -9.13591365e-01,  9.35517558e+00,  7.99624230e+00,  1.30738117e+01,
        9.19064939e-01,  1.31155868e+00,  1.99632651e+01,  3.07219049e+01,
       -9.99058116e-01,  1.02303551e+01,  8.39548115e+00,  1.36862628e+01,
        8.98230523e-01,  1.52490310e+00,  1.99606197e+01,  3.05232120e+01,
       -1.08438342e+00,  1.11040862e+01,  8.79467383e+00,  1.42947372e+01,
        8.75742280e-01,  1.75518271e+00,  1.99586485e+01,  3.03242299e+01,
       -1.16444093e+00,  1.19238751e+01,  9.19383521e+00,  1.48992327e+01,
        8.51760064e-01,  2.00076060e+00,  1.99574898e+01,  3.01253143e+01,
       -1.23378066e+00,  1.26339139e+01,  9.59298072e+00,  1.54997529e+01,
        8.26509741e-01,  2.25932391e+00,  1.99570612e+01,  2.99267056e+01,
       -1.29125166e+00,  1.32224170e+01,  9.99212327e+00,  1.60963041e+01,
        8.00192880e-01,  2.52880856e+00,  1.99571940e+01,  2.97284165e+01,
       -1.34043441e+00,  1.37260484e+01,  1.03912733e+01,  1.66888916e+01,
        7.72918968e-01,  2.80809343e+00,  1.99578046e+01,  2.95303333e+01,
       -1.38695686e+00,  1.42024383e+01,  1.07904405e+01,  1.72775189e+01,
        7.44693972e-01,  3.09711739e+00,  1.99589228e+01,  2.93323947e+01,
       -1.43554274e+00,  1.46999577e+01,  1.11896350e+01,  1.78621903e+01,
        7.15476880e-01,  3.39630040e+00,  1.99605195e+01,  2.91347499e+01,
       -1.48616637e+00,  1.52183436e+01,  1.15888635e+01,  1.84429142e+01,
        6.85275602e-01,  3.70556149e+00,  1.99623373e+01,  2.89376402e+01,
       -1.53396148e+00,  1.57077655e+01,  1.19881266e+01,  1.90197020e+01,
        6.54205431e-01,  4.02372004e+00,  1.99639713e+01,  2.87411344e+01,
       -1.57305557e+00,  1.61080890e+01,  1.23874176e+01,  1.95925638e+01,
        6.22470265e-01,  4.34868815e+00,  1.99651305e+01,  2.85450525e+01,
       -1.60046110e+00,  1.63887216e+01,  1.27867273e+01,  2.01615059e+01,
        5.90279931e-01,  4.67831717e+00,  1.99658401e+01,  2.83491512e+01,
       -1.61857232e+00,  1.65741805e+01,  1.31860491e+01,  2.07265312e+01,
        5.57754912e-01,  5.01137336e+00,  1.99663359e+01,  2.81533759e+01,
       -1.63392955e+00,  1.67314386e+01,  1.35853801e+01,  2.12876437e+01,
        5.24881977e-01,  5.34799221e+00,  1.99667698e+01,  2.79578788e+01,
       -1.65336389e+00,  1.69304463e+01,  1.39847184e+01,  2.18448506e+01,
        4.91539469e-01,  5.68941950e+00,  1.99670566e+01,  2.77628128e+01,
       -1.68088697e+00,  1.72122825e+01,  1.43840586e+01,  2.23981601e+01,
        4.57563511e-01,  6.03733331e+00,  1.99669651e+01,  2.75681322e+01,
       -1.71670886e+00,  1.75790987e+01,  1.47833914e+01,  2.29475773e+01,
        4.22810589e-01,  6.39320323e+00,  1.99663130e+01,  2.73735874e+01,
       -1.75858330e+00,  1.80078930e+01,  1.51827056e+01,  2.34931014e+01,
        3.87197502e-01,  6.75788124e+00,  1.99651091e+01,  2.71788247e+01,
       -1.80272543e+00,  1.84599084e+01,  1.55819930e+01,  2.40347248e+01,
        3.50736228e-01,  7.13124468e+00,  1.99636321e+01,  2.69835204e+01,
       -1.84340192e+00,  1.88764357e+01,  1.59812532e+01,  2.45724360e+01,
        3.13566441e-01,  7.51186330e+00,  1.99623800e+01,  2.67875922e+01,
       -1.87357676e+00,  1.91854260e+01,  1.63804954e+01,  2.51062253e+01,
        2.75969235e-01,  7.89685869e+00,  1.99618484e+01,  2.65913406e+01,
       -1.88614386e+00,  1.93141132e+01,  1.67797362e+01,  2.56360925e+01,
        2.38352848e-01,  8.28205050e+00,  1.99622319e+01,  2.63953820e+01,
       -1.87549489e+00,  1.92050676e+01,  1.71789906e+01,  2.61620494e+01,
        2.01193997e-01,  8.66255713e+00,  1.99632076e+01,  2.62003090e+01,
       -1.84039015e+00,  1.88455952e+01,  1.75782634e+01,  2.66841149e+01,
        1.64927049e-01,  9.03393067e+00,  1.99640724e+01,  2.60062369e+01,
       -1.78630462e+00,  1.82917593e+01,  1.79775466e+01,  2.72023043e+01,
        1.29829283e-01,  9.39333180e+00,  1.99642390e+01,  2.58127009e+01,
       -1.72347207e+00,  1.76483540e+01,  1.83768249e+01,  2.77166214e+01,
        9.59696355e-02,  9.74005459e+00,  1.99635931e+01,  2.56190103e+01,
       -1.66249263e+00,  1.70239246e+01,  1.87760853e+01,  2.82270579e+01,
        6.32283288e-02,  1.00753256e+01,  1.99624526e+01,  2.54246465e+01,
       -1.61163804e+00,  1.65031735e+01,  1.91753231e+01,  2.87335988e+01,
        3.13488165e-02,  1.04017718e+01,  1.99613254e+01,  2.52294390e+01,
       -1.57631318e+00,  1.61414470e+01,  1.95745428e+01,  2.92362288e+01,
        2.15338918e-20,  1.07227837e+01,  1.99606431e+01,  2.50335615e+01,
       -1.55856847e+00,  1.59597411e+01])


T = 1
N = 50
u = x[:N]
pos_x = x[np.arange(N, 9*N, 8)]
pos_y = x[np.arange(N+1, 9*N, 8)]
theta_block = x[np.arange(N+2, 9*N, 8)]
theta_tail = x[np.arange(N+3, 9*N, 8)]
vel_x = x[np.arange(N+4, 9*N, 8)]
vel_y = x[np.arange(N+5, 9*N, 8)]
omega_block = x[np.arange(N+6, 9*N, 8)]
omega_tail = x[np.arange(N+7, 9*N, 8)]

'''
#optimization plot
fig1, axs1= plt.subplots(4, 2, figsize=(16, 8))
fig1.suptitle("State Trajectories")
t = np.arange(0, T, T/N)
axs1[0][0].plot(t, pos_x)
axs1[0][0].set_ylabel('x(t)')
axs1[1][0].plot(t, pos_y)
axs1[1][0].set_ylabel('y(t)')
axs1[2][0].plot(t, theta_block)
axs1[2][0].set_ylabel('$\\theta_b$')
axs1[3][0].plot(t, theta_tail)
axs1[3][0].set_ylabel('$\\theta_t$')
axs1[0][1].plot(t, vel_x)
axs1[0][1].set_ylabel('$vel_x$')
axs1[1][1].plot(t, vel_y)
axs1[1][1].set_ylabel('$vel_y$')
axs1[2][1].plot(t, omega_block)
axs1[2][1].set_ylabel('$\omega_b(t)$')
axs1[3][1].plot(t, omega_tail)
axs1[3][1].set_ylabel('$\omega_t(t)$')
'''


# dynamic plot
object = MyController(u=(vel_x[0]/np.cos(theta_block[0]), theta_block[0]),
                      initial_state=(0, 0, theta_block[0]), input_torque=u, T=T)
block_traj, block_vel, tail_traj, tail_vel = object.dynamics()
print(vel_x[0]/np.cos(theta_block[0]))
print(np.rad2deg(theta_block[0]))

pos_x2=[]
pos_y2=[]
theta_b2=[]
theta_t2=[]
vel_x2=[]
vel_y2=[]
omega_block2=[]
omega_tail2=[]

for i in range(len(block_traj)):
    pos_x2.append(block_traj[i][0])
    pos_y2.append(block_traj[i][1])
    theta_b2.append(block_traj[i][2])
    theta_t2.append(tail_traj[i])

    vel_x2.append(block_vel[i][0])
    vel_y2.append(block_vel[i][1])
    omega_block2.append(block_vel[i][2])
    omega_tail2.append(tail_vel[i])

'''
# for dynamic
pos_x=[]
pos_y=[]
theta_b=[]
theta_t=[]
vel_x=[]
vel_y=[]
omega_block=[]
omega_tail=[]

for i in range(len(block_traj)):
    pos_x.append(block_traj[i][0])
    pos_y.append(block_traj[i][1])
    theta_b.append(block_traj[i][2])
    theta_t.append(tail_traj[i])

    vel_x.append(block_vel[i][0])
    vel_y.append(block_vel[i][1])
    omega_block.append(block_vel[i][2])
    omega_tail.append(tail_vel[i])
'''
''''
#dynamic plot
fig2, axs2 = plt.subplots(4, 2, figsize=(16, 8))
fig2.suptitle("State Trajectories")
t = np.arange(0, T, T/N)
axs2[0][0].plot(t, pos_x)
axs2[0][0].set_ylabel('x(t)')
axs2[1][0].plot(t, pos_y)
axs2[1][0].set_ylabel('y(t)')
axs2[2][0].plot(t, theta_b)
axs2[2][0].set_ylabel('$\\theta_b$')
axs2[3][0].plot(t, theta_t)
axs2[3][0].set_ylabel('$\\theta_t$')
axs2[0][1].plot(t, vel_x)
axs2[0][1].set_ylabel('$vel_x$')
axs2[1][1].plot(t, vel_y)
axs2[1][1].set_ylabel('$vel_y$')
axs2[2][1].plot(t, omega_block)
axs2[2][1].set_ylabel('$\omega_b(t)$')
axs2[3][1].plot(t, omega_tail)
axs2[3][1].set_ylabel('$\omega_t(t)$')

#torque plot
fig3 = plt.figure("Input Trajectory")
plt.plot(t, u)
plt.ylabel("torque")
plt.xlabel('Time (s)')
plt.show()

'''
#comparison plot
fig, axs= plt.subplots(4, 2, figsize=(16, 8))
fig.suptitle("Comparison of State Trajectories", fontsize=20)
t = np.arange(0, T, T/N)
axs[0][0].plot(t, pos_x,'r',label='optimization')
axs[0][0].plot(t,pos_x2,'b',label='dynamics')
axs[0][0].legend(loc='upper left')
axs[0][0].set_ylabel('x(t)')
axs[1][0].plot(t, pos_y,'r',t, pos_y2,'b')
axs[1][0].set_ylabel('y(t)')
axs[2][0].plot(t, theta_block,'r',t, theta_b2,'b')
axs[2][0].set_ylabel('$\\theta_b$')
axs[2][0].set_ylim((-10, 10))
axs[3][0].plot(t, theta_tail,'r',t, theta_t2,'b')
axs[3][0].set_ylabel('$\\theta_t$')
axs[3][0].set_ylim((-10, 40))
axs[0][1].plot(t, vel_x,'r',t, vel_x2,'b')
axs[0][1].set_ylabel('$vel_x$')
axs[0][1].set_ylim((10, 30))
axs[1][1].plot(t, vel_y,'r',t, vel_y2,'b')
axs[1][1].set_ylabel('$vel_y$')
axs[1][1].set_ylim((20, 40))
axs[2][1].plot(t, omega_block,'r',t, omega_block2,'b')
axs[2][1].set_ylabel('$\omega_b(t)$')
axs[2][1].set_ylim((-10, 10))
axs[3][1].plot(t, omega_tail,'r',t, omega_tail2,'b')
axs[3][1].set_ylabel('$\omega_t(t)$')
axs[3][1].set_ylim((-70, 70))
plt.show()



err_pos_x = pos_x - pos_x2
err_pos_y = pos_y - pos_y2
err_theta_block = theta_block - theta_b2
err_theta_tail = theta_tail - theta_t2
err_vel_x = vel_x - vel_x2
err_vel_y = vel_y - vel_y2
err_omega_block = omega_block - omega_block2
err_omega_tail = omega_tail - omega_tail2

fig4, axs4 = plt.subplots(4, 2, figsize=(16, 8))
fig4.suptitle("Error of State Trajectories", fontsize=20)
t = np.arange(0, T, T/N)
axs4[0][0].plot(t, err_pos_x)
axs4[0][0].set_ylabel('x(t)')
axs4[0][0].set_ylim((-1,1))
axs4[1][0].plot(t, err_pos_y)
axs4[1][0].set_ylabel('y(t)')
axs4[1][0].set_ylim((-1,1))
axs4[2][0].plot(t, err_theta_block)
axs4[2][0].set_ylabel('$\\theta_b$')
axs4[2][0].set_ylim((-1,1))
axs4[3][0].plot(t, err_theta_tail)
axs4[3][0].set_ylabel('$\\theta_t$')
axs4[3][0].set_ylim((-1,5))
axs4[0][1].plot(t, err_vel_x)
axs4[0][1].set_ylabel('$vel_x$')
axs4[0][1].set_ylim((-1, 1))
axs4[1][1].plot(t, err_vel_y)
axs4[1][1].set_ylabel('$vel_y$')
axs4[1][1].set_ylim((-1,1))
axs4[2][1].plot(t, err_omega_block)
axs4[2][1].set_ylabel('$\omega_b(t)$')
axs4[2][1].set_ylim((-1,1))
axs4[3][1].plot(t, err_omega_tail)
axs4[3][1].set_ylabel('$\omega_t(t)$')
axs4[3][1].set_ylim((-1,10))
plt.show()
