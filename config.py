from lib import *

EPSILON = 1e-9

CAMERAS = [
    Camera(1, Vec3D.Cylindrical(0.6/math.sqrt(3), 0, 3.3), 1.7),
    Camera(2, Vec3D.Cylindrical(0.6/math.sqrt(3), 2*math.pi/3, 3.3), 2.5),
    Camera(3, Vec3D.Cylindrical(0.6/math.sqrt(3), 4*math.pi/3, 3.3), 3.4)
]

CAMERAS_DELTAS = [
    Vec3D(0.3, 0.16, 0),
    Vec3D(0.07, 0.41, 0),
    Vec3D(-0.06, 0.05, 0)
]

FIDUCIALS = [
    Fiducial(CAMERAS[0], Vec3D(0.15, 0.37, 0)+CAMERAS_DELTAS[0]),
    Fiducial(CAMERAS[1], (Vec3D(0.15, 0.37, 0)+CAMERAS_DELTAS[1]).rotate_around_origin(2*math.pi/3, origin=CAMERAS_DELTAS[1])),
    Fiducial(CAMERAS[0], Vec3D(0.06, 0.18, 0)+CAMERAS_DELTAS[0]),
    Fiducial(CAMERAS[1], (Vec3D(0.06, 0.18, 0)+CAMERAS_DELTAS[1]).rotate_around_origin(2*math.pi/3, origin=CAMERAS_DELTAS[1]))
]

REFERENCES = [
    Reference.new(Vec3D(0.15, 0.37, 0), (FIDUCIALS[0], FIDUCIALS[1]), (FIDUCIALS[2], FIDUCIALS[3]))
]
