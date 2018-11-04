from utility import *


def animation(x, theta_b):
    """
    :param x: block state variable input as list of tuple [t1, t2, t3 ...] with
     each tuple ti specified as (x, y, theta). x is position along x axis, y is
     position along y axis, theta is angle for block

    :param theta_b: list of angle state for beam [a1, a2, a3 ...]

    :return: None
    """
    # define block
    length = 10
    width = 10
    dl = 0.5 * np.sqrt(length**2 + width**2)
    offset = -np.pi/2 - np.arctan(width/length)

    # define balance beam
    length_b = 3
    width_b = 20
    dl_b = 0.5 * np.sqrt(length_b**2 + width_b**2)
    offset_b = - np.pi/2 - np.arctan(length_b/width_b)

    # define lower left corner point of block
    angle_i = np.deg2rad(x[0][2])
    pos_i = (x[0][0], x[0][1])
    ri = (dl * np.cos(offset + angle_i), dl * np.sin(offset + angle_i))
    pi = (pos_i[0] + ri[0], pos_i[1] + ri[1])

    # define lower left corner point of beam
    theta_bi = np.deg2rad(theta_b[0])
    angle_bi = angle_i + theta_bi
    rbi = (dl_b * np.cos(offset_b + angle_bi), dl_b * np.sin(offset_b + angle_bi))
    pbi = (pos_i[0] + rbi[0], pos_i[1] + rbi[1])

    # initialize plot
    fig = plt.figure()
    fig.set_size_inches(7, 6.5)
    ax = plt.axes(xlim=(0, 50), ylim=(0, 50))
    patch = plt.Rectangle(pi, length, width, angle_i, fc='y')
    beam = plt.Rectangle(pbi, length_b, width_b, angle_bi, fc='r')

    def init():
        patch.xy = pi
        patch.angle = np.rad2deg(angle_i)
        ax.add_patch(patch)

        beam.xy = pbi
        beam.angle = np.rad2deg(angle_bi)
        ax.add_patch(beam)

        return patch, beam

    def animate(i):

        # universal center update
        pos = (x[i][0], x[i][1])

        # update on block
        angle = np.deg2rad(x[i][2])
        patch.angle = x[i][2]

        # converting center to lower left corner for block
        r = (dl * np.cos(offset + angle), dl * np.sin(offset + angle))
        p = (pos[0] + r[0], pos[1] + r[1])
        patch.xy = p

        # update on beam
        angle_b = np.deg2rad(theta_b[i]) + angle
        beam.angle = theta_b[i] + patch.angle

        # converting center to lower left corner for beam
        rb = (dl_b * np.cos(offset_b + angle_b), dl_b * np.sin(offset_b + angle_b))
        pb = (pos[0] + rb[0], pos[1] + rb[1])
        beam.xy = pb

        return patch, beam

    _ = FuncAnimation(fig, animate, init_func=init, frames=len(x), interval=1000, blit=False)
    plt.show()


