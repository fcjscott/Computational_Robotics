import numpy as np
from utility import *
from dynamics import MyController


x = np.array([-7.56802512e-02, -1.27000000e-01, -1.27000000e-01, -1.27000000e-01,
       -1.27000000e-01, -1.27000000e-01, -1.26969909e-01, -1.25795828e-01,
       -1.20791557e-01, -1.21177561e-01, -1.23765148e-01, -1.26047695e-01,
       -1.26296784e-01, -1.22746211e-01, -1.15468070e-01, -1.07588109e-01,
       -9.88778486e-02, -8.52933772e-02, -7.45339138e-02, -7.83892902e-02,
       -7.95812821e-02, -6.56464167e-02, -5.45318538e-02, -4.62943212e-02,
       -3.32184123e-02, -2.79788413e-02, -2.65380568e-02, -1.75924977e-02,
       -7.06792848e-03, -4.47357325e-03,  2.57770053e-03,  1.44235910e-02,
        1.11254829e-02, -3.03589900e-03, -1.05539529e-02, -1.33929430e-02,
       -1.15252229e-02,  8.12501389e-04,  1.53281985e-02,  2.49819407e-02,
        2.05546613e-02,  7.14864696e-03, -7.41941664e-04, -1.79422853e-03,
       -4.12250692e-04, -5.81236608e-03, -1.67432573e-02, -1.95735323e-02,
       -5.80036659e-03,  2.26509841e-03, -2.61258394e-19, -4.41932138e-19,
        7.32912186e+00,  5.08911379e-19,  2.00292821e+01,  3.45909265e+01,
       -2.88574735e-20,  6.97670372e-18,  4.00443808e-01,  6.89935089e-01,
        7.32532160e+00,  3.89146082e-02,  2.00150988e+01,  3.44025824e+01,
       -3.80025471e-01,  3.89146082e+00,  8.00561405e-01,  1.37610988e+00,
        7.31295859e+00,  1.65511825e-01,  1.99966609e+01,  3.42148971e+01,
       -8.56275471e-01,  8.76826082e+00,  1.20029936e+00,  2.05850079e+00,
        7.29107058e+00,  3.89645041e-01,  1.99771344e+01,  3.40241936e+01,
       -1.33252547e+00,  1.36450608e+01,  1.59963964e+00,  2.73702836e+00,
        7.25965757e+00,  7.11314258e-01,  1.99568936e+01,  3.38285633e+01,
       -1.80877547e+00,  1.85218608e+01,  1.99858491e+00,  3.41157675e+00,
        7.21871956e+00,  1.13051947e+00,  1.99376335e+01,  3.36262760e+01,
       -2.28502547e+00,  2.33986608e+01,  2.39718733e+00,  4.08200624e+00,
        7.16825712e+00,  1.64725491e+00,  1.99226082e+01,  3.34166732e+01,
       -2.76121905e+00,  2.82748831e+01,  2.79557430e+00,  4.74818901e+00,
        7.10829338e+00,  2.26128360e+00,  1.99160887e+01,  3.32016035e+01,
       -3.23515481e+00,  3.31279852e+01,  3.19394647e+00,  5.41007182e+00,
        7.03896677e+00,  2.97118808e+00,  1.99211289e+01,  3.29866773e+01,
       -3.69750616e+00,  3.78624630e+01,  3.59252289e+00,  6.06773293e+00,
        6.96047973e+00,  3.77489541e+00,  1.99365132e+01,  3.27794334e+01,
       -4.15119825e+00,  4.25082701e+01,  3.99144180e+00,  6.72138734e+00,
        6.87286309e+00,  4.67208981e+00,  1.99553779e+01,  3.25860076e+01,
       -4.61046583e+00,  4.72111701e+01,  4.39065405e+00,  7.37130796e+00,
        6.77596978e+00,  5.66427728e+00,  1.99658474e+01,  3.24060553e+01,
       -5.07886491e+00,  5.20075767e+01,  4.78990175e+00,  8.01764820e+00,
        6.66966102e+00,  6.75287895e+00,  1.99589220e+01,  3.22279687e+01,
       -5.55201081e+00,  5.68525907e+01,  5.18889338e+00,  8.66028007e+00,
        6.55395125e+00,  7.93774702e+00,  1.99402409e+01,  3.20352180e+01,
       -6.01896642e+00,  6.16342162e+01,  5.58758975e+00,  9.29887839e+00,
        6.42910540e+00,  9.21616849e+00,  1.99293964e+01,  3.18246142e+01,
       -6.46561820e+00,  6.62079304e+01,  5.98626477e+00,  9.93326561e+00,
        6.29561074e+00,  1.05831539e+01,  1.99381052e+01,  3.16141073e+01,
       -6.88384854e+00,  7.04906090e+01,  6.38517611e+00,  1.05636230e+01,
        6.15406253e+00,  1.20326075e+01,  1.99530287e+01,  3.14216349e+01,
       -7.27097221e+00,  7.44547554e+01,  6.78423559e+00,  1.11902299e+01,
        6.00518987e+00,  1.35570635e+01,  1.99529195e+01,  3.12390535e+01,
       -7.61629326e+00,  7.79908430e+01,  7.18317805e+00,  1.18130673e+01,
        5.84986725e+00,  1.51475672e+01,  1.99413264e+01,  3.10446838e+01,
       -7.91596943e+00,  8.10595269e+01,  7.58197776e+00,  1.24318907e+01,
        5.68868055e+00,  1.67981190e+01,  1.99386448e+01,  3.08376563e+01,
       -8.20270044e+00,  8.39956525e+01,  7.98085994e+00,  1.30466478e+01,
        5.52166459e+00,  1.85083624e+01,  1.99495728e+01,  3.06380567e+01,
       -8.49889526e+00,  8.70286874e+01,  8.37988529e+00,  1.36575458e+01,
        5.34896367e+00,  2.02768199e+01,  1.99529629e+01,  3.04517446e+01,
       -8.77119719e+00,  8.98170593e+01,  8.77886198e+00,  1.42646427e+01,
        5.17128638e+00,  2.20962353e+01,  1.99447062e+01,  3.02579378e+01,
       -8.99653145e+00,  9.21244821e+01,  9.17774259e+00,  1.48677690e+01,
        4.98946526e+00,  2.39580835e+01,  1.99433542e+01,  3.00546925e+01,
       -9.18558053e+00,  9.40603446e+01,  9.57666582e+00,  1.54668958e+01,
        4.80426279e+00,  2.58545569e+01,  1.99489692e+01,  2.98579934e+01,
       -9.33466690e+00,  9.55869891e+01,  9.97564149e+00,  1.60621367e+01,
        4.61642200e+00,  2.77780465e+01,  1.99485980e+01,  2.96660976e+01,
       -9.44941175e+00,  9.67619764e+01,  1.03745766e+01,  1.66534862e+01,
        4.42641157e+00,  2.97237533e+01,  1.99449088e+01,  2.94688451e+01,
       -9.55163094e+00,  9.78087008e+01,  1.07734874e+01,  1.72408727e+01,
        4.23455151e+00,  3.16884004e+01,  1.99461726e+01,  2.92698110e+01,
       -9.63437573e+00,  9.86560075e+01,  1.11724256e+01,  1.78243162e+01,
        4.04140161e+00,  3.36662553e+01,  1.99476471e+01,  2.90745379e+01,
       -9.68061403e+00,  9.91294876e+01,  1.15713725e+01,  1.84038504e+01,
        3.84757292e+00,  3.56510611e+01,  1.99470456e+01,  2.88788800e+01,
       -9.70225434e+00,  9.93510845e+01,  1.19703115e+01,  1.89794654e+01,
        3.65349229e+00,  3.76384468e+01,  1.99468559e+01,  2.86826236e+01,
       -9.70580910e+00,  9.93874852e+01,  1.23692365e+01,  1.95511561e+01,
        3.45969488e+00,  3.96229322e+01,  1.99456407e+01,  2.84864478e+01,
       -9.67393168e+00,  9.90610604e+01,  1.27681485e+01,  2.01189052e+01,
        3.26669529e+00,  4.15992480e+01,  1.99455579e+01,  2.82884579e+01,
       -9.62602717e+00,  9.85705182e+01,  1.31670658e+01,  2.06827106e+01,
        3.07432643e+00,  4.35691052e+01,  1.99461725e+01,  2.80920843e+01,
       -9.61085920e+00,  9.84151982e+01,  1.35659944e+01,  2.12425822e+01,
        2.88185444e+00,  4.55400184e+01,  1.99466942e+01,  2.78950716e+01,
       -9.63634017e+00,  9.86761234e+01,  1.39649416e+01,  2.17985318e+01,
        2.68867863e+00,  4.75181387e+01,  1.99480259e+01,  2.76998940e+01,
       -9.68124060e+00,  9.91359038e+01,  1.43638895e+01,  2.23505797e+01,
        2.49458660e+00,  4.95056410e+01,  1.99467558e+01,  2.75048894e+01,
       -9.72796216e+00,  9.96143326e+01,  1.47628187e+01,  2.28987101e+01,
        2.29982649e+00,  5.14999845e+01,  1.99461668e+01,  2.73081598e+01,
       -9.74804852e+00,  9.98200168e+01,  1.51617303e+01,  2.34429115e+01,
        2.10516816e+00,  5.34932858e+01,  1.99449984e+01,  2.71119789e+01,
       -9.71778470e+00,  9.95101154e+01,  1.55606307e+01,  2.39831608e+01,
        1.91156828e+00,  5.54757486e+01,  1.99450415e+01,  2.69129494e+01,
       -9.64220319e+00,  9.87361607e+01,  1.59595633e+01,  2.45194618e+01,
        1.71957803e+00,  5.74417288e+01,  1.99482131e+01,  2.67171518e+01,
       -9.55682206e+00,  9.78618579e+01,  1.63585234e+01,  2.50518621e+01,
        1.52896103e+00,  5.93936469e+01,  1.99478009e+01,  2.65228754e+01,
       -9.50487836e+00,  9.73299544e+01,  1.67574747e+01,  2.55803567e+01,
        1.33898359e+00,  6.13390159e+01,  1.99473309e+01,  2.63265828e+01,
       -9.49286579e+00,  9.72069457e+01,  1.71564200e+01,  2.61049275e+01,
        1.14907872e+00,  6.32836418e+01,  1.99471966e+01,  2.61305023e+01,
       -9.49762111e+00,  9.72556401e+01,  1.75553628e+01,  2.66255745e+01,
        9.59084922e-01,  6.52291782e+01,  1.99470787e+01,  2.59341974e+01,
       -9.50175826e+00,  9.72980045e+01,  1.79543087e+01,  2.71422955e+01,
        7.68933046e-01,  6.71763334e+01,  1.99475186e+01,  2.57378946e+01,
       -9.51342941e+00,  9.74175172e+01,  1.83532615e+01,  2.76551072e+01,
        5.78241540e-01,  6.91290145e+01,  1.99477546e+01,  2.55432787e+01,
       -9.55572121e+00,  9.78505852e+01,  1.87521911e+01,  2.81640125e+01,
        3.86446176e-01,  7.10929990e+01,  1.99452107e+01,  2.53472559e+01,
       -9.62381519e+00,  9.85478675e+01,  1.91510945e+01,  2.86689777e+01,
        1.93494111e-01,  7.30688281e+01,  1.99451296e+01,  2.51492565e+01,
       -9.67139125e+00,  9.90350464e+01,  1.95500000e+01,  2.91700000e+01,
        1.42154427e-20,  7.50502078e+01,  1.99454181e+01,  2.49529778e+01,
       -9.67801988e+00,  9.91029235e+01])

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
print(theta_block)
print('--------------------------')
print(vel_x)
print('----------------------------')
print(vel_y)


# dynamic plot
object = MyController(input_torque=u, T=T)
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


#comparison plot
def normalize(x):
    for i in range(len(x)):
        x[i] = x[i] % (2 * np.pi)
        if x[i] >= np.pi:
            x[i] -= 2*np.pi
    return x

fig, axs= plt.subplots(4, 2, figsize=(16, 8))
fig.suptitle("Comparison of State Trajectories")
t = np.arange(0, T, T/N)
axs[0][0].plot(t, pos_x,'r',label='optimization')
axs[0][0].plot(t,pos_x2,'b',label='dynamics')
axs[0][0].legend(loc='upper left')
axs[0][0].set_ylabel('x(t)')
axs[1][0].plot(t, pos_y,'r',t, pos_y2,'b')
axs[1][0].set_ylabel('y(t)')
axs[2][0].plot(t, normalize(theta_block),'r',t, normalize(theta_b2),'b')
axs[2][0].set_ylabel('$\\theta_b$')
axs[2][0].set_ylim((-10, 10))
axs[3][0].plot(t, theta_tail,'r',t, theta_t2,'b')
axs[3][0].set_ylabel('$\\theta_t$')
axs[3][0].set_ylim((-10, 80))
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
axs[3][1].set_ylim((-20, 100))
plt.show()

err_pos_x = pos_x - pos_x2
err_pos_y = pos_y - pos_y2
err_theta_block = normalize(theta_block) - normalize(theta_b2)
err_theta_tail = theta_tail - theta_t2
err_vel_x = vel_x - vel_x2
err_vel_y = vel_y - vel_y2
err_omega_block = omega_block - omega_block2
err_omega_tail = omega_tail - omega_tail2

fig4, axs4 = plt.subplots(4, 2, figsize=(16, 8))
fig4.suptitle("Error of State Trajectories")
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
axs4[3][0].set_ylim((-2,2))
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
axs4[3][1].set_ylim((-5,5))
plt.show()