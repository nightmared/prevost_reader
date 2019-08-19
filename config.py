from lib import *

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


# The degree of precision to which we want to operate. Usefule to consider whether a number is nearing zero (ah, the joys of floating point !).
EPSILON = 1e-9

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
    Vec3D(0.15, 0.25, 0.4),
    Vec3D(0, 0.03, 0.05),
    Vec3D(-1.7, 2.55, 0.67),
]

POINTS = [
    Measurement(CAMERAS[0], CAMERAS[0].project_from_chamber(BASE_POINTS[0])+VIEWS_OFFSETS[0]),
    Measurement(CAMERAS[1], CAMERAS[1].project_from_chamber(BASE_POINTS[0])+VIEWS_OFFSETS[1]),
    Measurement(CAMERAS[0], CAMERAS[0].project_from_chamber(BASE_POINTS[1])+VIEWS_OFFSETS[0]),
    Measurement(CAMERAS[1], CAMERAS[1].project_from_chamber(BASE_POINTS[1])+VIEWS_OFFSETS[1]),
    Measurement(CAMERAS[0], CAMERAS[0].project_from_chamber(BASE_POINTS[2])+VIEWS_OFFSETS[0]),
    Measurement(CAMERAS[1], CAMERAS[1].project_from_chamber(BASE_POINTS[2])+VIEWS_OFFSETS[1])
]

print(BASE_POINTS[0]-POINTS[0].merge(POINTS[1], REFERENCES[0]))
print(BASE_POINTS[1]-POINTS[2].merge(POINTS[3], REFERENCES[0]))
print(BASE_POINTS[2]-POINTS[4].merge(POINTS[5], REFERENCES[0]))
