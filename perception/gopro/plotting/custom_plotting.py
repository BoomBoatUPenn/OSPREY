import cv2

AXES = {'0': 'x',
        '1': 'y',
        '2': 'z'}

def plot_AR(april_im, origins, ground_plane):
    if "boat" in origins.keys():
        for i, pt in enumerate(origins["boat"][0][1:]): # boat coordinate system
            axis = AXES[str(i)]
            boat_c = origins["boat"][0][0]
            if axis == 'x':
                cv2.arrowedLine(april_im, boat_c, pt, (0, 0, 255))
                april_im = cv2.putText(april_im, axis + '_b', pt, fontFace=1, fontScale=1.0, color=(0, 0, 255))
            elif axis == 'y':
                cv2.arrowedLine(april_im, boat_c, pt, (0, 255, 0))
                april_im = cv2.putText(april_im, axis + '_b', pt, fontFace=1, fontScale=1.0, color=(0, 255, 0))
            elif axis == 'z':
                cv2.arrowedLine(april_im, boat_c, pt, (255, 0, 0))
                april_im = cv2.putText(april_im, axis + '_b', pt, fontFace=1, fontScale=1.0, color=(255, 0, 0))

    if "ground" in origins.keys():
        for i, pt in enumerate(origins["boat"][0][1:]): # ground coordinate system overlayed onto boat center
            axis = AXES[str(i)]
            ground_c = origins["boat"][0][0]
            if axis == 'x':
                cv2.arrowedLine(april_im, ground_c, pt, (0, 0, 255))
                april_im = cv2.putText(april_im, axis + '_w', pt, fontFace=1, fontScale=1.0, color=(0, 0, 255))
            elif axis == 'y':
                cv2.arrowedLine(april_im, ground_c, pt, (0, 255, 0))
                april_im = cv2.putText(april_im, axis + '_w', pt, fontFace=1, fontScale=1.0, color=(0, 255, 0))
            elif axis == 'z':
                cv2.arrowedLine(april_im, ground_c, pt, (255, 0, 0))
                april_im = cv2.putText(april_im, axis + '_w', pt, fontFace=1, fontScale=1.0, color=(255, 0, 0))
    if ground_plane is not None:
        for row in ground_plane:
            for pt in row:
                cv2.circle(april_im, pt, 3, (0, 0, 255), -1)
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