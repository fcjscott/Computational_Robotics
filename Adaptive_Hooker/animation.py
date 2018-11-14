from utility import *


def animation(x, theta_b, target_point):
    """
    :param x: block state variable input as list of tuple [t1, t2, t3 ...] with
     each tuple ti specified as (x, y, theta). x is position along x axis, y is
     position along y axis, theta is angle for block

    :param theta_b: list of angle state for beam [a1, a2, a3 ...]

    :param target_point: target point for hook (x,y)

    :return: None
    """
    # define block
    length = 8
    width = 8
    dl = 0.5 * np.sqrt(length**2 + width**2)
    offset = -np.pi/2 - np.arctan(width/length)


    # define balance beam
    length_b = 1
    width_b = 15
    dl_b = 0.5 * np.sqrt(length_b**2 + width_b**2)
    offset_b = - np.pi/2 - np.arctan(length_b/width_b)


    # define lower left corner point of block
    angle_patch = x[0][2]
    angle_i = np.deg2rad(angle_patch)
    pos_i = (x[0][0], x[0][1])
    ri = (dl * np.cos(offset + angle_i), dl * np.sin(offset + angle_i))
    pi = (pos_i[0] + ri[0], pos_i[1] + ri[1])

    # define points of power board

    p_powerboard = ( -(6-6*np.tan(angle_i)) * np.cos(angle_i) , -(6/np.cos(angle_i)+(6-6*np.tan(angle_i)) * np.sin(angle_i)))
    p_powercircle = ( -(7-5*np.tan(angle_i)) * np.cos(angle_i) , -(5/np.cos(angle_i)+(7-5*np.tan(angle_i)) * np.sin(angle_i)))

    # define hook

    pos_line1 = (x[0][0] - 0.5* width * np.sin(angle_i), x[0][0] - (0.5* width+1) * np.sin(angle_i))
    pos_line2 = (x[0][1] + 0.5* width * np.cos(angle_i), x[0][1] + (0.5* width + 1) * np.cos(angle_i))

    pos_arc0 = (x[0][0] - (0.5* width+1) * np.sin(angle_i), x[0][1] + (0.5* width + 1) * np.cos(angle_i))
    pos_arc1 = (x[0][0] - (0.5* width + 2 - np.tan(angle_i)) * np.sin(angle_i) - 3/np.cos(angle_i), x[0][1] + (0.5* width + 2 - np.tan(angle_i)) * np.cos(angle_i))
    pos_arc2 = (x[0][0] - (0.5* width + 3) * np.sin(angle_i), x[0][1] + (0.5* width + 3) * np.cos(angle_i))

    # define lower left corner point of beam
    theta_bi = np.deg2rad(theta_b[0])
    angle_beam = angle_patch + theta_b[0]
    angle_bi = angle_i + theta_bi
    rbi = (dl_b * np.cos(offset_b + angle_bi), dl_b * np.sin(offset_b + angle_bi))
    pbi = (pos_i[0] + rbi[0], pos_i[1] + rbi[1])

    # initialize plot
    fig = plt.figure()
    fig.set_size_inches(7, 6.5)
    ax = plt.axes(xlim=(-20, 100), ylim=(-20, 100))
    patch = mpatches.Rectangle(pi, length, width,angle_patch , fc='r')
    beam = mpatches.Rectangle(pbi, length_b, width_b, angle_beam, fc='y')
    Power_board = mpatches.Rectangle(p_powerboard, 12, 2, angle_patch, fc='b')
    Power_circle = mpatches.Circle(p_powercircle,radius=2,color='b')
    target_circle = mpatches.Circle(target_point, radius=0.7, color='green')
    line = matplotlib.lines.Line2D(pos_line1,pos_line2,linewidth=2, color='k')
    Path = mpath.Path([pos_arc0,pos_arc1,pos_arc2], [1, 3, 3])
    arc = mpatches.PathPatch(Path)
    arc.set_fill(False)


    def init():

        ax.add_patch(patch)
        ax.add_patch(beam)
        ax.add_patch(Power_board)
        ax.add_patch(Power_circle)
        ax.add_patch(target_circle)
        ax.add_line(line)
        ax.add_patch(arc)



        return patch, beam, line, arc


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

        # update on hook

        posi_line1 = (x[i][0] - 0.5 * width * np.sin(angle), x[i][0] - (0.5 * width + 1) * np.sin(angle))

        posi_line2 = (x[i][1] + 0.5 * width * np.cos(angle), x[i][1] + (0.5 * width + 1) * np.cos(angle))

        posi_arc0 = ( x[i][0] - (0.5 * width + 1) * np.sin(angle),x[i][1] + (0.5 * width + 1) * np.cos(angle))

        posi_arc1 = (x[i][0] - (0.5 * width + 2 - np.tan(angle)) * np.sin(angle) - 3 / np.cos(angle),
                    x[i][1] + (0.5 * width + 2 - np.tan(angle)) * np.cos(angle))

        posi_arc2 = (x[i][0] - (0.5 * width + 3) * np.sin(angle), x[i][1] + (0.5 * width + 3) * np.cos(angle))

        line.set_data(posi_line1,posi_line2)

        arc._path = mpath.Path([posi_arc0, posi_arc1, posi_arc2], [1, 3, 3])
        arc.set_fill(False)
        return patch, beam, line, arc

    _ = FuncAnimation(fig, animate, init_func=init, frames=len(x), interval=20, blit=False)
    plt.show()


