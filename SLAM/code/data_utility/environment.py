from data_utility.utility import *


class Environment:
    def __init__(self, n_landmarks=20, L=400, W=300):
        self.L = L
        self.W = W
        self.n_landmarks = n_landmarks
        self.grid = np.zeros((self.L, self.W))
        rand_lm = self.generate_landmarks(n_landmarks)
        self.landmarks = rand_lm

    def generate_landmarks(self, n_landmarks=20):

        landmark = {}
        for c in range(n_landmarks):
            x = np.random.randint(0, self.L)
            y = np.random.randint(0, self.W)
            landmark[c] = [x, y]

        return landmark

    def plot_env(self, show=False):
        x, y = zip(*self.landmarks.values())
        fig, ax = plt.subplots()
        ax.set_xlim(0, self.L)
        ax.set_ylim(0, self.W)
        ax.grid()
        ax.scatter(x, y, marker='*', s=100)
        ax.legend(['True Landmarks'])

        return fig, ax


