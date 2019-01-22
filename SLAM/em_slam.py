# coding: utf-8
from utility import *
from enviroment import Environment
from robot import Robot
from generate_gif import generate_gif


def main():
    # set background information
    sns.set_style('white')
    img_sav_dir = input('plz enter image saving dir: ')
    os.makedirs(img_sav_dir)
    _ = plt.figure(figsize=(8, 6))

    # set initialization parameter
    np.random.seed()

    # covariance matrix for motion
    R = np.eye(3)
    R[2][2] = 0.1

    # covariance matrix for measurements
    Q = 3*np.eye(3)
    Q[1][1] = 0.5
    Q[2][2] = 0
    # R = np.zeros((3, 3))
    # Q = np.zeros((3, 3))

    # initial position
    x0 = np.array([150, 200, 0.2]).reshape(3, 1)

    # Initial Guess of Posteriori Covariance Matrix
    P0 = np.eye(3)
    P0[2][2] = 0.1
    T = 10
    u = [(50, .5) for _ in range(T)]

    # Another Test Case
    # u = [(np.random.randint(30, 50), np.random.random()*2*np.pi-np.pi) for _ in range(T)]

    # Run Algorithm
    env = Environment(n_landmarks=5)
    robot = Robot(u, Q, R, x0, P0, env, img_sav_dir)
    robot.execute()
    robot.visualize(exe=True)
    robot.em_slam()
    plt.show()

    # generate gif
    generate_gif(img_sav_dir)


if __name__ == "__main__":
    main()