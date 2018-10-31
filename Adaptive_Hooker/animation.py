from utility import *



def animation():
    rectangle = plt.Rectangle((10, 10), 100, 100, fc='r')





fig, ax = plt.subplots(1)
xdata, ydata = [], []
ln, = plt.plot([], [], 'ro', animated=True)


def init():
    ax.set_xlim(0, 2*np.pi)
    ax.set_ylim(-1, 1)
    return ln,


def update(frame):
    xdata.append(frame)
    ydata.append(np.sin(frame))
    ln.set_data(xdata, ydata)
    return ln,


ani = FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 128),
                    init_func=init, blit=True)
plt.show()