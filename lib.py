from __future__ import annotations
import math
from typing import Type, Tuple

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

    def norm_xy(self):
        return math.sqrt((self.x**2)+(self.y**2))

    def norm(self):
        return math.sqrt((self.x**2)+(self.y**2)+(self.z**2))

    def distance(self, other: Type[Vec3D]):
        (self-other).norm()

    def rotate_around_origin(self, theta, phi = None) -> Vec3D:
        if phi is None:
            # Plane rotation
            return Vec3D(self.x*math.cos(theta)-self.y*math.sin(theta), self.x*math.sin(theta)+self.y*math.cos(theta), self.z)
        else:
            r = self.norm()
            return Vec3D(r*math.cos(theta)*math.cos(phi), r*math.sin(theta)*math.cos(phi), r*math.sin(phi))

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
            theta_res = 2*math.pi-theta_res
        phi_res = phi1-phi2
        if abs(phi_res) > abs(2*math.pi-phi_res):
            phi_res = 2*math.pi-phi_res
        return (theta_res, phi_res)


class BubbleChamber:
    pass


class Camera:
    # We assume the camera is always oriented towards the origin (0, 0, 0) and we specify its position with regard to this origin
    def __init__(self, position: Type[Vec3D]):
        self.position = position


class Reference:
    # Create a new reference from two views of the same fiducial
    def __init__(self, p1: Type[Fiducial], p2: Type[Fiducial]):
        if p1.camera == p2.camera:
            raise AssertionError("The two measurements come from the same camera !")

        # Try to compute the rotation between the two cameras, relative to the origin
        (theta, phi) = p1.position.get_angle_with_origin(p2.position)
        print((theta, phi))


class Measurement:
    def __init__(self, camera: Type[Camera], position: Type[Vec2D]):
        self.camera = camera
        self.position = position

    # Take another measurement (from a different camera)
    def merge(self, other, ref: Reference) -> Vec3D:
        if other.camera == self.camera:
            raise AssertionError("The two measurements come from the same camera !")
        



class Fiducial:
    def __init__(self, camera: Type[Camera], position: Type[Vec3D]):
        self.camera = camera
        self.position = position

