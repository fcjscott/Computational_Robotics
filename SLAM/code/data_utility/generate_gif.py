from data_utility.utility import *
def generate_gif(img_sav_dir):
    png_dir = img_sav_dir
    images = []
    for file_name in sorted(os.listdir(img_sav_dir), key = lambda x: int(re.findall(r'\d+', x)[0])):
        #if file_name.endswith('.png'):
        file_path = os.path.join(png_dir, file_name)
        images.append(imageio.imread(file_path))
    imageio.mimsave('em_slam_{}.gif'.format(img_sav_dir), images, duration = 0.5)