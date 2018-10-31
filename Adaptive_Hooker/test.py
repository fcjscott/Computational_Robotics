from utility import *


s = [(10, 10, 90), (10, 15, 45), (15, 10, 90)]
print(s[0][2])


def animation(x):
    """
    :param x: state variable input as class of tuple [t1, t2, t3 ...] with
     each tuple specified as (x, y, theta). x is position along x axis, y is
     position along y axis, theta is angle for object

    :return: None
    """
    pi = (x[0][0], x[0][1])
    ti = x[0][2]
    fig = plt.figure()
    fig.set_size_inches(7, 6.5)

    ax = plt.axes(xlim=(0, 20), ylim=(0, 20))
    patch = plt.Rectangle(pi, 1, 1, ti, fc='y')

    def init():
        patch.xy = pi
        patch.angle = ti
        ax.add_patch(patch)
        return patch,

    def animate(i):
        p = (x[i][0], x[i][1])
        patch.angle = x[i][2]
        patch.xy = p
        print(p)
        return patch,

    _ = FuncAnimation(fig, animate, init_func=init, frames=3, interval=2000, blit=False)
    plt.show()


animation(s)

