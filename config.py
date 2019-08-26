from lib import *
import numpy as np
import matplotlib.pyplot as plt
from scipy import optimize

# serial device from which to read the measurements
SERIAL_PORT = '/dev/ttyACM0'

### Physical parameters on the cameras and the fiducials
# We have the following design:
# The origin is the center of the back of the chamber. Every measurement and every object is relative to that point.
# The bubble chamber currently described looks like this (ignoring the fact that all the cameras are not forming a line but a triangle):
# |<--D-->|<-d->|<-----h----->|
# |  n1   |  n2 |      n3     CAM1
# |       |  G  |             |
# O       |  L  |             CAM2
# |       |  A  |             |
# |       |  S  |             CAM3
# |       |  S  |             |

GLASS_WIDTH = 0.3
CAMERA_GLASS_DISTANCE = 1.9
CHAMBER_DEPTH = 1.1

AIR_REFRACTIVE_INDEX = 1.000277 #(STP)
GLASS_REFRACTIVE_INDEX = 1.52 #(crown glass)
CHAMBER_REFRACTIVE_INDEX = 1.34 #(propane) to verify !

CHAMBER_DIAMETER = 2.6

# The degree of precision to which we want to operate. Usefule to consider whether a number is nearing zero (ah, the joys of floating point !).
EPSILON = 1e-10

CAMERAS_ANGLES = [
    0,
    2*math.pi/3,
    4*math.pi/3
]

CAMERAS_POSITIONS = [
    Vec3D.Cylindrical(0.6/math.sqrt(3), CAMERAS_ANGLES[0], 3.3),
    Vec3D.Cylindrical(0.6/math.sqrt(3), CAMERAS_ANGLES[1], 3.3),
    Vec3D.Cylindrical(0.6/math.sqrt(3), CAMERAS_ANGLES[2], 3.3)
]

CAMERAS_SCALES = [
    0.17,
    2.5,
    0.34
]

CAMERAS = [
    Camera(1, CAMERAS_POSITIONS[0], CAMERAS_SCALES[0]),
    Camera(2, CAMERAS_POSITIONS[1], CAMERAS_SCALES[1]),
    Camera(3, CAMERAS_POSITIONS[2], CAMERAS_SCALES[2])
]

VIEWS_OFFSETS = [
    Vec3D(0.3, 0.16, 0),
    Vec3D(0.07, 0.41, 0),
    Vec3D(-0.06, 0.05, 0)
]

# known positions of two fiducials
BASE_FIDUCIALS = [
    Vec3D(0.15, 0.37, 0),
    Vec3D(0.06, 0.18, 0)
]

FIDUCIALS = [
    Fiducial(CAMERAS[0], CAMERAS[0].project_from_chamber(BASE_FIDUCIALS[0])+VIEWS_OFFSETS[0]),
    Fiducial(CAMERAS[1], CAMERAS[1].project_from_chamber(BASE_FIDUCIALS[0])+VIEWS_OFFSETS[1]),
    Fiducial(CAMERAS[0], CAMERAS[0].project_from_chamber(BASE_FIDUCIALS[1])+VIEWS_OFFSETS[0]),
    Fiducial(CAMERAS[1], CAMERAS[1].project_from_chamber(BASE_FIDUCIALS[1])+VIEWS_OFFSETS[1])
]

REFERENCES = [
    Reference.new(BASE_FIDUCIALS[0], (FIDUCIALS[0], FIDUCIALS[1]), BASE_FIDUCIALS[1], (FIDUCIALS[2], FIDUCIALS[3]))
]

BASE_POINTS = [
]

POINTS = [
]

def generate_circle(radius, center, nb_points, arc_length=2*math.pi):
    base = center+Vec3D(radius, 0, 0)
    for i in range(0, nb_points):
        BASE_POINTS.append(base.rotate_around_origin(i*arc_length/nb_points, origin=center))
        POINTS.append(Measurement(CAMERAS[0], CAMERAS[0].project_from_chamber(BASE_POINTS[i])+VIEWS_OFFSETS[0]))
        POINTS.append(Measurement(CAMERAS[1], CAMERAS[1].project_from_chamber(BASE_POINTS[i])+VIEWS_OFFSETS[1]))

generate_circle(0.25, Vec3D(0.3, 0.4, 0.8), 150, arc_length = 0.5*math.pi)

#maxd = 0
#for i in range(0, len(BASE_POINTS)):
#    mu = 0
#    sigma = 1
#    delta = 1e-6
#    POINTS_a = Measurement(POINTS[2*i].camera, POINTS[2*i].position+Vec3D(delta*np.random.normal(mu, sigma), delta*np.random.normal(mu, sigma), delta*np.random.normal(mu, sigma)))
#    POINTS_b = Measurement(POINTS[2*i+1].camera, POINTS[2*i+1].position+Vec3D(delta*np.random.normal(mu, sigma), delta*np.random.normal(mu, sigma), delta*np.random.normal(mu, sigma)))
#    tmp = BASE_POINTS[i].distance(POINTS_a.merge(POINTS_b, REFERENCES[0]))
#    if tmp > maxd:
#        maxd = tmp
#print(maxd)

# Approximate the points as an arc
points_a = [0, 45, 74, 89]
points_b = [0, 23, 79, 127]

def approximate_curve(points) -> Tuple[Type[Vec3D], float]:
    def curve_func(x, *points):
        radius = x[0]
        center_x = x[1]
        center_y = x[2]
        total = 0
        for p in points:
            total += ((p.x-center_x)**2+(p.y-center_y)**2-radius**2)**2
        return total

    def jac_curve_func(x, *points):
        radius = x[0]
        center_x = x[1]
        center_y = x[2]
        total_x = 0
        total_y = 0
        for p in points:
            total_x += (p.x-center_x)*((p.x-center_x)**2+(p.y-center_y)**2-radius**2)
            total_y += (p.y-center_y)*((p.x-center_x)**2+(p.y-center_y)**2-radius**2)
        return -4*np.array([x[0]*curve_func(x, *points), total_x, total_y])

    v = optimize.least_squares(curve_func, [0, 0, 0], args=points, jac=jac_curve_func, xtol=EPSILON, ftol=EPSILON, gtol=EPSILON, bounds=([0, 0, 0], [CHAMBER_DIAMETER, CHAMBER_DIAMETER, CHAMBER_DIAMETER]))

approximate_curve([BASE_POINTS[i] for i in points_a])
approximate_curve([BASE_POINTS[i] for i in points_b])
