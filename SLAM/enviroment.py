from utility import *
class Environment(object):
    def __init__(self, n_landmarks=20, L=400, W=300):
        self.L = L
        self.W = W
        self.grid = np.zeros((self.W, self.L))
        self.theta = self.generate_landmarks(n_landmarks)

    def generate_landmarks(self, n_landmarks=20):
        self.n_landmarks = n_landmarks
        x = np.random.randint(0, self.L, n_landmarks)
        y = np.random.randint(0, self.W, n_landmarks)
        c = range(1, n_landmarks + 1)
        return np.array(list(zip(x, y, c)))
        # print(self.theta)

    def plot_env(self):
        x, y, _ = zip(*self.theta)
        plt.grid()
        plt.scatter(x, y, marker='*')