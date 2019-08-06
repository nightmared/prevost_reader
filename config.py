from lib import *

CAMERAS = [
    Camera(Vec3D.Cylindrical(0.6/math.sqrt(3), 0, 3.3)),
    Camera(Vec3D.Cylindrical(0.6/math.sqrt(3), 2*math.pi/3, 3.3)),
    Camera(Vec3D.Cylindrical(0.6/math.sqrt(3), 4*math.pi/3, 3.3))
]

FIDUCIALS = [
    Fiducial(CAMERAS[0], Vec3D(0.15, 0.37, 0)),
    Fiducial(CAMERAS[1], Vec3D(0.15, 0.37, 0).rotate_around_origin(2*math.pi/3)),
    Fiducial(CAMERAS[0], Vec3D(0.06, 0.18, 0)),
    Fiducial(CAMERAS[1], Vec3D(0.06, 0.18, 0).rotate_around_origin(2*math.pi/3))
]

REFERENCES = [
    Reference(FIDUCIALS[0], FIDUCIALS[1]),
    Reference(FIDUCIALS[2], FIDUCIALS[3])
]
