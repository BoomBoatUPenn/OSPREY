import cv2
import numpy as np

def plot_AR(april_im, origins, ground_plane):
    AXES = {'0': ('x', (0, 0, 255)),
        '1': ('y', (0, 255, 0)),
        '2': ('z', (255, 0, 0))}

    for key, value in origins.items():
        for i, pt in enumerate(value[0][1:]): # boat coordinate system
            axis, color = AXES[str(i)]
            boat_c = value[0][0]
            april_im = cv2.arrowedLine(april_im, boat_c, pt, color)
            april_im = cv2.putText(april_im, axis + '_' + key, pt, fontFace=1, fontScale=1.0, color=color)
    if ground_plane is not None:
        for row in ground_plane:
            for pt in row:
                april_im = cv2.circle(april_im, pt, 3, (0, 0, 255), -1)
    return april_im

def displayResults(imgs):
    """
    Displays processed images.
    inputs:
        - imgs: dictionary of images to be displayed
        - PARAMS: dictionary of parameters for display
    outputs:
        - (None)
    """
    if len(imgs) > 0:
        for i, (window, img) in enumerate(imgs.items()):
            if img is not None:
                cv2.imshow(window, img)

def plot_Occ(ground_plane, mapped, threshes):
    n, m = len(ground_plane), len(ground_plane[0])
    grid = np.ones((n,m,3), dtype=np.float32)
    for i in range(n):
        for j in range(m):
            if (i,j) in mapped["occupied"].keys() and (i,j) not in mapped["free"]:
                density = mapped["occupied"][(i,j)]
                if density > threshes["density"]:
                    grid[i,j,:] = np.array([1.0 - density, 0.0, density])
    scale_factor = 2.5
    new_dims = (int(scale_factor*m), int(scale_factor*n))
    grid = cv2.resize(grid, new_dims)
    cv2.imshow("Occupancy", grid)
    return;