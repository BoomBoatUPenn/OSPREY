import numpy as np
import matplotlib.pyplot as plt


def write_points_file(output_filename, x_points, y_points, num_decimals=4):
    point_tuples = zip(x_points, y_points)
    with open(output_filename, 'w') as f:
        for point in point_tuples:
            f.write(f'{round(point[0], num_decimals)},{round(point[1], num_decimals)}\n')


def plot_points(x_points, y_points, show_start=True):
    plt.axis('equal')
    plt.plot(x_points, y_points)
    if show_start:
        plt.plot(x_points[0], y_points[0], 'ro')
        plt.arrow(x=x_points[0], y=y_points[0], dx=x_points[int(len(x_points) * 0.05)]-x_points[0],
                  dy=y_points[int(len(y_points) * 0.05)]-y_points[0], facecolor='red', edgecolor='none', width=0.1)
    plt.show()


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
        plot_points(x_points, y_points)


def sin_wave_path(output_filename, line_length, left_first=False, amplitude=1, stretch_factor=1.0, num_points=1000,
                  plot=False):
    y_points = np.linspace(0, line_length, num=num_points)
    x_points = np.sin(y_points * (1/stretch_factor)) * amplitude
    if left_first:
        x_points = -1 * x_points
    write_points_file(output_filename, x_points, y_points)
    if plot:
        plot_points(x_points, y_points)


def circle_path(output_filename, radius, left_first=True, fraction_of_circle=1.0, x_transform=0, num_points=1000,
                plot=False):
    transformed_fraction = (fraction_of_circle * 2 + 0.5) / 2.5  # get right fraction based on pi/2 to 5pi/2 range
    theta = np.linspace(np.pi/2, 5/2 * np.pi * transformed_fraction, num=num_points)
    direction = (1 if left_first else -1)
    x_points = radius * np.cos(theta) * direction + x_transform
    y_points = (radius * np.sin(theta) - radius) * -1
    write_points_file(output_filename, x_points, y_points)
    if plot:
        plot_points(x_points, y_points)


# straight_line_path("line.txt", 10, angle=20, plot=True)
# sin_wave_path("sin.txt", 10, left_first=False, amplitude=2, stretch_factor=1/5, plot=True)
circle_path("circle.txt", 10, fraction_of_circle=0.5, x_transform=0, num_points=100, plot=True, left_first=False)

