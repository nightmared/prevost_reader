from __future__ import annotations
import math
from typing import Type, Tuple
import config

class Vec2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def to_Vec3D(self, z) -> Vec3D:
        return Vec3D(self.x, self.y, z)

    def rotate_around_origin(self, theta) -> Vec2D:
        return Vec2D(self.x*math.cos(theta)-self.y*math.sin(theta), self.x*math.sin(theta)+self.y*math.cos(theta))


class Vec3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def Cylindrical(cls, r, theta, z):
        return cls(r*math.cos(theta), r*math.sin(theta), z)

    @classmethod
    def Spherical(cls, r, theta, phi):
        return cls(r*math.cos(theta)*math.cos(phi), r*math.sin(theta)*math.cos(phi), r*math.sin(phi))

    def __add__(self, other: Type[Vec3D]):
        return Vec3D(self.x+other.x, self.y+other.y, self.z+other.z)

    def __neg__(self):
        return Vec3D(-self.x, -self.y, -self.z)

    def __sub__(self, other: Type[Vec3D]):
        return Vec3D(self.x-other.x, self.y-other.y, self.z-other.z)

    # multiplication means only scaling every component by a factor x
    def __mul__(self, factor: float):
        return Vec3D(self.x * factor, self.y * factor, self.z * factor)

    # same story as the multiplication
    def __truediv__(self, factor: float):
        return Vec3D(self.x / factor, self.y / factor, self.z / factor)

    def __str__(self):
        return f"""Vec3D({self.x}, {self.y}, {self.z})"""

    def norm_xy(self):
        return math.sqrt((self.x**2)+(self.y**2))

    def norm(self):
        return math.sqrt((self.x**2)+(self.y**2)+(self.z**2))

    def distance(self, other: Type[Vec3D]):
        (self-other).norm()

    def rotate_around_origin(self, theta, phi = None, origin = None) -> Vec3D:
        obj = self
        if origin is not None:
            obj = self-origin

        if phi is None or phi == 0.0 or phi == -0.0:
            # Plane rotation
            res = Vec3D(obj.x*math.cos(theta)-obj.y*math.sin(theta), obj.x*math.sin(theta)+obj.y*math.cos(theta), obj.z)
        else:
            r, theta_cur, phi_cur = obj.get_coordinates_spherical()
            res = Vec3D(r*math.cos(theta+theta_cur)*math.sin(phi+phi_cur), r*math.sin(theta+theta_cur)*math.sin(phi+phi_cur), r*math.cos(phi+phi_cur))

        if origin is not None:
            return res+origin
        else:
            return res

    # Return the coordinates in the cartesian coordinates system
    def get_coordinates(self) -> Tuple[float, float, float]:
        return (self.x, self.y, self.z)

    # Return the coordinates in the spherical coordinates system
    def get_coordinates_spherical(self) -> Tuple[float, float, float]:
        r = self.norm()
        theta = math.atan2(self.y, self.x)
        # beware of division by zero
        if r != 0.:
            phi = math.acos(self.z/r)
        else:
            phi = 0.
        return (r, theta, phi)

    # Compute the angle between the two vectors, with regard to the origin
    def get_angle_with_origin(self, other: Type[Vec3D]) -> Tuple[float, float]:
        _, theta1, phi1 = self.get_coordinates_spherical()
        _, theta2, phi2 = other.get_coordinates_spherical()
        theta_res = theta1-theta2
        # minimize the angle
        if abs(theta_res) > abs(2*math.pi-theta_res):
            theta_res = theta_res-2*math.pi
        phi_res = phi1-phi2
        if abs(phi_res) > abs(2*math.pi-phi_res):
            phi_res = phi_res-2*math.pi
        return (theta_res, phi_res)


class BubbleChamber:
    pass


class Camera:
    # We assume the camera is always oriented towards the origin (0, 0, 0) and we specify its position with regard to this origin
    def __init__(self, number: int, position: Type[Vec3D], scaling: float):
        self.number = number
        self.position = position
        self.scaling = scaling

    def __str__(self):
        return f"""Camera(id={self.number}, position={self.position}, scale_factor={self.scaling})"""


class Reference:
    def __init__(self, cam1, offset_cam1, cam2, offset_cam2):
        self.cam1 = cam1
        self.offset_cam1 = offset_cam1
        self.cam2 = cam2
        self.offset_cam2 = offset_cam2
        print(self)

    def __str__(self):
        return f"""Reference(offset_camera{self.cam1.number}={self.offset_cam1}, offset_camera{self.cam2.number}={self.offset_cam2})"""

    # Create a new reference from two fiducials and the real position of the first one, given two views per fiducial
    @classmethod
    def new(cls, fiducial1_real_pos: Vec3D, fiducial1: Tuple[Type[Fiducial], Type[Fiducial]], fiducial2: Tuple[Type[Fiducial], Type[Fiducial]]) -> Type[Reference]:
        if fiducial1[0].camera.number == fiducial1[1].camera.number or fiducial2[0].camera.number == fiducial2[1].camera.number:
            raise AssertionError("The two measurements come from the same camera !")
        # The two cameras used must be identical in the two pairs
        if fiducial1[0].camera.number != fiducial2[0].camera.number or fiducial1[1].camera.number != fiducial2[1].camera.number:
            raise AssertionError("The cameras used must be identical for the two fiducials !")

        # Try to compute the rotation between the two cameras, relative to the origin
        (theta_cam, phi_cam) = fiducial1[0].camera.position.get_angle_with_origin(fiducial1[1].camera.position)


        ### Try to guess the position of the origin

        # vector between the two fiducials
        delta_x_fid, delta_y_fid, _ = ((fiducial2[0].position-fiducial1[0].position)/fiducial1[0].camera.scaling).get_coordinates()
        # Sanity check: ensure this is the same fiducial by comparing the vector from the first to the second fiducial in both camera views
        delta_x_fid_cam1, delta_y_fid_cam1, _ = ((fiducial2[1].position-fiducial1[1].position)/fiducial1[1].camera.scaling).rotate_around_origin(theta_cam, phi_cam).get_coordinates()
        if abs(delta_x_fid_cam1-delta_x_fid) > config.EPSILON or abs(delta_y_fid_cam1-delta_y_fid) > config.EPSILON:
            raise AssertionError("You must use the same fiducials when trying to establish a reference")

        # Offset between the real origin and the origin in the coordinates system of camera1
        cam1 = fiducial1[0].camera
        offset_cam1 = fiducial1[0].position-fiducial1_real_pos*fiducial1[0].camera.scaling
        # Offset between the real origin and the origin in the coordinates system of camera2
        cam2 = fiducial1[1].camera
        offset_cam2 = fiducial1[1].position-fiducial1_real_pos.rotate_around_origin(-theta_cam, -phi_cam)*fiducial1[1].camera.scaling

        return cls(cam1, offset_cam1, cam2, offset_cam2)

    # Return the reference with cameras inverted
    def __neg__(self):
        return Vec3D(self.cam2, self.offset_cam2, self.cam1, self.offset_cam1)


class Measurement:
    def __init__(self, camera: Type[Camera], position: Type[Vec2D]):
        self.camera = camera
        self.position = position

    # Take another measurement (from a different camera)
    def merge(self, other: Type[Measurement], ref: Type[Reference]) -> Vec3D:
        if other.camera == self.camera:
            raise AssertionError("The two measurements come from the same camera !")
        



class Fiducial:
    def __init__(self, camera: Type[Camera], position: Type[Vec3D]):
        self.camera = camera
        self.position = position

