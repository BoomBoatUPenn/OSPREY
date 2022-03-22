import numpy as np
import matplotlib.pyplot as plt


def write_points_file(output_filename, x_points, y_points, num_decimals=4):
    point_tuples = zip(x_points, y_points)
    with open(output_filename, 'w') as f:
        for point in point_tuples:
            f.write(f'{round(point[0], num_decimals)},{round(point[1], num_decimals)}\n')


def straight_line_path(output_filename, line_length, angle=0, num_points=1000, plot=False):
    """
    angle is angle from straight line forward in y direction. that is why sin is used for x and cos is used for y
    """
    x_i, y_i = 0, 0
    x_f = line_length * np.sin(angle)
    y_f = line_length * np.cos(angle)
    x_points = np.linspace(x_i, x_f, num=num_points)
    y_points = np.linspace(y_i, y_f, num=num_points)
    write_points_file(output_filename, x_points, y_points)
    if plot:
        plt.plot(x_points, y_points)
        plt.show()


def sin_wave_path(output_filename, line_length, left_first=False, amplitude=1, stretch_factor=1, num_points=1000,
                  plot=False):
    y_points = np.linspace(0, line_length, num=num_points)
    x_points = np.sin(y_points * (1/stretch_factor)) * amplitude
    if left_first:
        x_points = -1 * x_points
    write_points_file(output_filename, x_points, y_points)
    if plot:
        plt.plot(x_points, y_points)
        plt.show()


def circle_path(output_filename, radius, left_first=True, num_points=1000, plot=False):
    theta = np.linspace(np.pi/2, 5/2 * np.pi, num=num_points)
    x_points = radius * np.cos(theta) * (1 if left_first else -1)
    y_points = (radius * np.sin(theta) - radius) * -1
    write_points_file(output_filename, x_points, y_points)
    if plot:
        plt.plot(x_points, y_points)
        plt.show()


# straight_line_path("line.txt", 10, angle=0, plot=True)
# sin_wave_path("sin.txt", 10, left_first=False, amplitude=2, stretch_factor=2, plot=True)
circle_path("circle.txt", 10, num_points=100, plot=True, left_first=True)

