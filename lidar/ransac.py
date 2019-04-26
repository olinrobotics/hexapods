#!/usr/bin/env python

#   Adapted from OARS waterline detection code by Jeremy Ryan

#   Python libraries
import random

#   Outside libraries
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.patches as ptch
import sys

#   Recursion is fun!
# sys.setrecursionlimit(10000)

def ransac(xs, ys, trials=200, threshold = .1, plot=False):
    """ Runs ransac on an image, sampling 'points' number of points 'trials'
    number of times. Returns a slope and intercept of line of best fit. """

    if plot:
        plt.plot(xs, [y for y in ys], 'r.')

    #   Number of points in array
    pnum = len(xs)

    #   Maximum number of points found with ransac
    max_good_points = 0
    good_line = (0, 0)

    #   Perform ransac 'trials' number of times and find best line
    for i in range(trials):

        points_to_connect = []

        #   Lines have two points --- theoretically could smooth with more
        for j in range(2):
            idx = int(random.random()*pnum)
            points_to_connect.append((xs[idx], ys[idx]))

        dists = []
        m, b = line_from_points(points_to_connect[0], points_to_connect[1])

        #   Determines how many points fall within a threshold distance of
        #   ransac line
        for k in range(pnum):
            x = xs[k]
            y = ys[k]
            d = dist_point_to_line((x, y), (m, b))
            if d <= threshold:
                dists.append(d)

        #       Keep track of best line so far
        if len(dists) > max_good_points:
            max_good_points = len(dists)
            good_line = (m, b)

    print(good_line)
    bad_pts = filter(lambda x, y: dist_point_to_line((x, y), (m,b)) > threshold, zip(xs, ys))
    if plot:
        plt.plot([-10, 10], [m*x + b for x in [-10, 10]], 'b', linewidth=2.0)
        plt.plot(x, y, 'k.')
        plt.show()
    return (good_line, 


def dist_point_to_line(p, l):
    #   TODO make a more effective distance function than vertical distance
    return abs(p[1] - (l[0]*p[0] + l[1]))

def line_from_points(p1, p2):
    """ Returns tuple (m, b) from two points, where m is the slope and b is the
    y-intercept of the line that connects them. """
    #   TODO Make this find line of best fit of several points?

    try:
        if p2[0] == p1[0]:
            p2 = (p2[0]+.0000001, p2[1])
        m = float(p2[1] - p1[1])/(p2[0] - p1[0])
        b = p1[1] - m*p1[0]
    except ZeroDivisionError:
        m = float(p2[1] - p1[1])/(p2[0] - p1[0])
        b = p1[1] - m*p1[0]
    return (m, b)

if __name__ == '__main__':
    pass