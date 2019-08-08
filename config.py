from lib import *

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

BASE_FIDUCIALS = [
    Vec3D(0.15, 0.37, 0),
    Vec3D(0.06, 0.18, 0)
]

FIDUCIALS = [
    Fiducial(CAMERAS[0], BASE_FIDUCIALS[0]*CAMERAS_SCALES[0]+VIEWS_OFFSETS[0]),
    Fiducial(CAMERAS[1], BASE_FIDUCIALS[0].rotate_around_origin(CAMERAS_ANGLES[1])*CAMERAS_SCALES[1]+VIEWS_OFFSETS[1]),
    Fiducial(CAMERAS[0], BASE_FIDUCIALS[1]*CAMERAS_SCALES[0]+VIEWS_OFFSETS[0]),
    Fiducial(CAMERAS[1], BASE_FIDUCIALS[1].rotate_around_origin(CAMERAS_ANGLES[1])*CAMERAS_SCALES[1]+VIEWS_OFFSETS[1])
]

REFERENCES = [
    Reference.new(BASE_FIDUCIALS[0], (FIDUCIALS[0], FIDUCIALS[1]), (FIDUCIALS[2], FIDUCIALS[3]))
]

BASE_POINTS = [
    Vec3D(0.15, 0.25, 0.4)
]

POINTS = [
    Measurement(CAMERAS[0], CAMERAS[0].project_point(BASE_POINTS[0], VIEWS_OFFSETS[0])),
    Measurement(CAMERAS[1], CAMERAS[1].project_point(BASE_POINTS[0], VIEWS_OFFSETS[1]))
]

print(POINTS[0].merge(POINTS[1], REFERENCES[0]))
