from utility import *
from dynamics import MyController

x = np.array([-1.31605145e-02, -2.65568337e-02, -1.80995520e-02, -3.21047262e-03,
          5.54013108e-03, 5.81057280e-03, 2.54972476e-05, 4.05147430e-03,
          2.98102673e-03, -1.41669838e-03, 2.95564268e-03, 3.34587464e-03,
          1.38864942e-03, 2.40210389e-04, -3.66297892e-04, 4.86564971e-03,
          7.84061968e-04, -3.85564563e-03, 1.06944602e-03, 9.71773432e-04,
          2.80518113e-03, 6.28144735e-03, 2.76526802e-03, 6.80768972e-04,
          -1.11006250e-03, 2.76947679e-04, 3.26092471e-03, 4.68453197e-04,
          4.04726651e-03, 3.84226969e-03, 9.66815374e-04, -5.58473660e-04,
          -2.18578520e-03, 3.60950482e-03, 8.76841447e-04, 2.70274458e-03,
          5.20289270e-03, -5.41955535e-06, 4.41809313e-03, 2.67507486e-03,
          6.94800745e-04, 1.25755057e-03, -4.91751942e-04, 1.51141089e-03,
          1.39976360e-03, 4.01832263e-03, -1.68301348e-03, -7.35772562e-03,
          -1.09040620e-02, -6.82060314e-03, 4.24431269e-19, -4.73271189e-19,
          1.90557666e-01, -7.42610726e-19, 5.03808477e+01, 9.71837445e+00,
          5.76592797e-19, 1.82524006e-18, 2.04057116e+00, 3.85708389e-01,
          1.87503410e-01, 3.12755811e-02, 5.03795299e+01, 9.32733383e+00,
          -1.50814630e-01, 1.54434181e+00, 4.08107796e+00, 7.55592285e-01,
          1.77960831e-01, 1.28991589e-01, 5.03776700e+01, 8.93698444e+00,
          -3.20383763e-01, 3.28072973e+00, 6.12152256e+00, 1.10958957e+00,
          1.63345449e-01, 2.78653104e-01, 5.03764586e+01, 8.54287631e+00,
          -4.01302143e-01, 4.10933394e+00, 8.16194701e+00, 1.44755239e+00,
          1.47270481e-01, 4.43260782e-01, 5.03766752e+01, 8.14522689e+00,
          -3.92455969e-01, 4.01874912e+00, 1.02023987e+01, 1.76938595e+00,
          1.32247529e-01, 5.97095806e-01, 5.03778025e+01, 7.74643705e+00,
          -3.49355100e-01, 3.57739623e+00, 1.22428865e+01, 2.07508282e+00,
          1.18546237e-01, 7.37397037e-01, 5.03784616e+01, 7.34842090e+00,
          -3.27194387e-01, 3.35047053e+00, 1.42833985e+01, 2.36466444e+00,
          1.05607256e-01, 8.69892206e-01, 5.03789930e+01, 6.95069025e+00,
          -3.11713320e-01, 3.19194439e+00, 1.63239410e+01, 2.63813264e+00,
          9.35225905e-02, 9.93639176e-01, 5.03799684e+01, 6.55276374e+00,
          -2.85009522e-01, 2.91849750e+00, 1.83645078e+01, 2.89549297e+00,
          8.20990198e-02, 1.11061654e+00, 5.03801933e+01, 6.15530778e+00,
          -2.79069458e-01, 2.85767124e+00, 2.04050839e+01, 3.13675596e+00,
          7.09140902e-02, 1.22515022e+00, 5.03804333e+01, 5.75790063e+00,
          -2.73225781e-01, 2.79783200e+00, 2.24456852e+01, 3.36191936e+00,
          6.03320904e-02, 1.33350990e+00, 5.03814281e+01, 5.36033432e+00,
          -2.49297673e-01, 2.55280817e+00, 2.44863219e+01, 3.57098243e+00,
          5.05987600e-02, 1.43317920e+00, 5.03821876e+01, 4.96288991e+00,
          -2.31319749e-01, 2.36871423e+00, 2.65269793e+01, 3.76394947e+00,
          4.13547725e-02, 1.52783763e+00, 5.03824511e+01, 4.56553679e+00,
          -2.25134646e-01, 2.30537877e+00, 2.85676417e+01, 3.94082260e+00,
          3.22263479e-02, 1.62131270e+00, 5.03824307e+01, 4.16819580e+00,
          -2.25613425e-01, 2.31028147e+00, 3.06083183e+01, 4.10160414e+00,
          2.34342264e-02, 1.71134402e+00, 5.03831525e+01, 3.77095807e+00,
          -2.08528496e-01, 2.13533179e+00, 3.26490277e+01, 4.24629767e+00,
          1.54225659e-02, 1.79338343e+00, 5.03840509e+01, 3.37379400e+00,
          -1.87075423e-01, 1.91565233e+00, 3.46897456e+01, 4.37489825e+00,
          7.60916283e-03, 1.87339268e+00, 5.03835744e+01, 2.97631183e+00,
          -1.98738833e-01, 2.03508565e+00, 3.67304452e+01, 4.48739940e+00,
          -6.54702551e-04, 1.95801466e+00, 5.03831467e+01, 2.57882852e+00,
          -2.09318585e-01, 2.14342231e+00, 3.87711423e+01, 4.58380680e+00,
          -8.97585676e-03, 2.04322328e+00, 5.03834480e+01, 2.18162588e+00,
          -2.01567671e-01, 2.06405295e+00, 4.08118563e+01, 4.66412937e+00,
          -1.68495946e-02, 2.12385035e+00, 5.03839832e+01, 1.78458346e+00,
          -1.87225827e-01, 1.91719247e+00, 4.28526062e+01, 4.72838049e+00,
          -2.37341255e-02, 2.19434795e+00, 5.03852208e+01, 1.38804153e+00,
          -1.52722100e-01, 1.56387431e+00, 4.48934053e+01, 4.77657145e+00,
          -2.92242060e-02, 2.25056637e+00, 5.03864120e+01, 9.91556539e-01,
          -1.18369932e-01, 1.21210810e+00, 4.69342374e+01, 4.80869311e+00,
          -3.37535963e-02, 2.29694733e+00, 5.03868505e+01, 5.94563229e-01,
          -1.05284648e-01, 1.07811479e+00, 4.89750773e+01, 4.82472722e+00,
          -3.80509997e-02, 2.34095274e+00, 5.03867991e+01, 1.97177539e-01,
          -1.06914760e-01, 1.09480714e+00, 5.10159142e+01, 4.82466495e+00,
          -4.24454821e-02, 2.38595224e+00, 5.03866995e+01, -2.00252372e-01,
          -1.10078262e-01, 1.12720140e+00, 5.30567572e+01, 4.80851555e+00,
          -4.66319692e-02, 2.42882187e+00, 5.03870999e+01, -5.97181005e-01,
          -9.66442604e-02, 9.89637226e-01, 5.50976166e+01, 4.77628985e+00,
          -5.02596061e-02, 2.46596887e+00, 5.03875103e+01, -9.94076408e-01,
          -8.24830745e-02, 8.44626683e-01, 5.71384939e+01, 4.72799079e+00,
          -5.32531968e-02, 2.49662324e+00, 5.03879863e+01, -1.39085962e+00,
          -6.53359933e-02, 6.69040571e-01, 5.91793973e+01, 4.66362937e+00,
          -5.52928258e-02, 2.51750904e+00, 5.03887988e+01, -1.78721169e+00,
          -3.53778632e-02, 3.62269319e-01, 6.12203271e+01, 4.58320666e+00,
          -5.63559335e-02, 2.52839526e+00, 5.03892860e+01, -2.18394242e+00,
          -1.71168156e-02, 1.75276192e-01, 6.32612675e+01, 4.48670373e+00,
          -5.70178221e-02, 2.53517300e+00, 5.03893271e+01, -2.58122936e+00,
          -1.55662613e-02, 1.59398516e-01, 6.53022033e+01, 4.37410113e+00,
          -5.78593422e-02, 2.54379017e+00, 5.03890549e+01, -2.97891972e+00,
          -2.59867552e-02, 2.66104373e-01, 6.73431364e+01, 4.24540156e+00,
          -5.88024115e-02, 2.55344720e+00, 5.03891936e+01, -3.37607423e+00,
          -2.05806102e-02, 2.10745449e-01, 6.93840811e+01, 4.10062390e+00,
          -5.92909980e-02, 2.55845032e+00, 5.03896307e+01, -3.77283226e+00,
          -3.54506564e-03, 3.63014721e-02, 7.14250418e+01, 3.93977380e+00,
          -5.91593159e-02, 2.55710190e+00, 5.03899785e+01, -4.16970697e+00,
          1.00473309e-02, -1.02884668e-01, 7.34660251e+01, 3.76286018e+00,
          -5.81444226e-02, 2.54670939e+00, 5.03907525e+01, -4.56602521e+00,
          4.00665999e-02, -4.10281983e-01, 7.55070346e+01, 3.56988712e+00,
          -5.61219026e-02, 2.52599879e+00, 5.03912682e+01, -4.96269839e+00,
          5.98024338e-02, -6.12376922e-01, 7.75480637e+01, 3.36084513e+00,
          -5.33603644e-02, 2.49772063e+00, 5.03917226e+01, -5.35948524e+00,
          7.65582286e-02, -7.83956260e-01, 7.95891172e+01, 3.13573833e+00,
          -4.97140293e-02, 2.46038216e+00, 5.03924740e+01, -5.75595459e+00,
          1.03492391e-01, -1.05976208e+00, 8.16301935e+01, 2.89456366e+00,
          -4.52630883e-02, 2.41480453e+00, 5.03928451e+01, -6.15289298e+00,
          1.16288475e-01, -1.19079399e+00, 8.36712818e+01, 2.63730799e+00,
          -4.04028692e-02, 2.36503588e+00, 5.03930713e+01, -6.55001065e+00,
          1.23701940e-01, -1.26670786e+00, 8.57123766e+01, 2.36396491e+00,
          -3.53336248e-02, 2.31312682e+00, 5.03931622e+01, -6.94726502e+00,
          1.26609829e-01, -1.29648464e+00, 8.77534759e+01, 2.07453203e+00,
          -3.01270792e-02, 2.25981179e+00, 5.03932920e+01, -7.34450158e+00,
          1.30481675e-01, -1.33613236e+00, 8.97945854e+01, 1.76901328e+00,
          -2.46182532e-02, 2.20340142e+00, 5.03936720e+01, -7.74156109e+00,
          1.41535981e-01, -1.44932845e+00, 9.18357176e+01, 1.44741627e+00,
          -1.84689088e-02, 2.14043213e+00, 5.03944085e+01, -8.13842255e+00,
          1.62109526e-01, -1.66000155e+00, 9.38768713e+01, 1.10973931e+00,
          -1.17233299e-02, 2.07135740e+00, 5.03947321e+01, -8.53556505e+00,
          1.70977158e-01, -1.75080609e+00, 9.59180048e+01, 7.55959722e-01,
          -5.49339714e-03, 2.00756289e+00, 5.03934159e+01, -8.93354618e+00,
          1.36647682e-01, -1.39927227e+00, 9.79590568e+01, 3.86049605e-01,
          -1.36302314e-03, 1.96526786e+00, 5.03907037e+01, -9.33206683e+00,
          6.73040610e-02, -6.89193585e-01, 1.00000000e+02, 6.61363325e-20,
          2.53567121e-19, 1.95131050e+00, 5.03880448e+01, -9.73049019e+00,
          9.39698352e-20, -7.49253562e-13, 2.02517220e+00])


T = x[-1]
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

print(u)
print(pos_x[-1])
print(pos_y[-1])
print(theta_block[-1])
print(theta_tail[-1])
print(vel_x[-1])
print(vel_y[-1])
print(omega_block[-1])
print(omega_tail[-1])

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

print(vel_x[0]/np.cos(theta_block[0]))
print(np.rad2deg(theta_block[0]))
print(vel_x[0])

'''
# dynamic plot
object = MyController(u=(vel_x[0]/np.cos(theta_block[0]), theta_block[0]),
                      initial_state=(0, 0, theta_block[0]), input_torque=u, T=T)
block_traj, block_vel, tail_traj, tail_vel = object.dynamics()

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
axs[2][0].set_ylim((-1, 1))
axs[3][0].plot(t, theta_tail,'r',t, theta_t2,'b')
axs[3][0].set_ylabel('$\\theta_t$')
axs[3][0].set_ylim((-10, 10))
axs[0][1].plot(t, vel_x,'r',t, vel_x2,'b')
axs[0][1].set_ylabel('$vel_x$')
axs[0][1].set_ylim((40, 60))
axs[1][1].plot(t, vel_y,'r',t, vel_y2,'b')
axs[1][1].set_ylabel('$vel_y$')
axs[1][1].set_ylim((-10, 10))
axs[2][1].plot(t, omega_block,'r',t, omega_block2,'b')
axs[2][1].set_ylabel('$\omega_b(t)$')
axs[2][1].set_ylim((-10, 10))
axs[3][1].plot(t, omega_tail,'r',t, omega_tail2,'b')
axs[3][1].set_ylabel('$\omega_t(t)$')
axs[3][1].set_ylim((-10, 10))
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
axs4[3][0].set_ylim((-1,1))
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
axs4[3][1].set_ylim((-1,1))
plt.show()


t = np.arange(0, T, T/N)
fig5 = plt.figure("Input Trajectory")
plt.plot(t, u)
plt.ylabel("torque")
plt.xlabel('Time (s)')
plt.ylim((-0.127, 0.127))
plt.show()