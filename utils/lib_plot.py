'''
Functions:
    show()
    showImg()
    plot_3d_points()
'''
import numpy as np
import time
import matplotlib.pyplot as plt
import cv2

# from mpl_toolkits.mplot3d import Axes3D
# from matplotlib import gridspec


def show(imgs, figsize=(6, 10), layout=None, titles=[],
         show_colorbar=False, if_show=True, color_format='RGB', new_fig=True):

    def convert(img):
        '''change image color from "BGR" to "RGB" for plt.plot()'''
        if isinstance(img.flat[0], np.float):
            # img = (img*255).astype(np.uint8)
            maxi = img.max()
            if maxi > 1.0:
                img = img/maxi
        if color_format == 'BGR' and len(img.shape) == 3 and img.shape[2] == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    # Check input
    if isinstance(imgs, np.ndarray):
        imgs = [imgs]
    imgs = [convert(img) for img in imgs]

    # Init figure
    if new_fig:
        plt.figure(figsize=figsize)

    # Set subplot size
    N = len(imgs)
    if layout is not None:
        r, c = layout[0], layout[1]
    else:
        if N <= 4:
            r, c = 1, N
        else:
            r, c = N//4+1, 4

    # Plot
    for i in range(N):
        ax = plt.subplot(r, c, i+1)
        plt.imshow(imgs[i])
        if titles:
            ax.set_title(titles[i], fontsize=15)

        if show_colorbar:
            plt.colorbar()

    if if_show:
        plt.show()


def showImg(I):
    cv2.imshow('img', I)
    cv2.waitKey()
    cv2.destroyAllWindows()


def plot_3d_points(points):
    from mpl_toolkits.mplot3d import Axes3D

    # -- Check input.
    if not isinstance(points, np.ndarray):
        points = np.array(points)

    # Convert input points to be Nx3
    if points.shape[0] == 3:  # 3xN --> Nx3.
        points = points.T

    # -- Plot
    fig = plt.figure().gca(projection='3d')
    fig.scatter(xs=points[:, 0], ys=points[:, 1], zs=points[:, 2])
