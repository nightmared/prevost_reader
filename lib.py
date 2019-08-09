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

    # multiplication means only scaling every component by a factor x (except z, because it's not supposed to change in the experiments)
    def __mul__(self, factor: float):
        return Vec3D(self.x * factor, self.y * factor, self.z)

    # same story as the multiplication
    def __truediv__(self, factor: float):
        return Vec3D(self.x / factor, self.y / factor, self.z)

    def __str__(self):
        return f"""Vec3D({self.x}, {self.y}, {self.z})"""

    def __repr__(self):
        return str(self)

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

        if phi is None or phi == 0.0 or phi == -0.0 or phi == 0:
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

    # Project a point onto the camera screen, given its position in 3D space and the offset applied by the camera wrt to the origin
    def project_point(self, pos: Vec3D, camera_offset: Vec3D) -> Vec3D:
        # direction vector
        # TODO: compute the variation in scaling depending of z
        theta = self.position.get_coordinates_spherical()[1]
        vec = ((pos-camera_offset)/self.scaling).rotate_around_origin(-theta, 0)-self.position
        if abs(vec.z) < config.EPSILON:
            return AssertionError("The point to project must not be on the same level as the camera")
        # We are going to use a parametric equation with the position of the camera
        # solve t so that vec.z*t+self.position.z=0
        t = -self.position.z/vec.z
        x = vec.x*t+self.position.x
        y = vec.y*t+self.position.y

        # Apply the offset and camera scaling again
        return Vec3D(x, y, 0).rotate_around_origin(theta, 0)*self.scaling+camera_offset


class Reference:
    def __init__(self, cam1, offset_cam1, cam2, offset_cam2):
        self.cam1 = cam1
        self.offset_cam1 = offset_cam1
        self.cam2 = cam2
        self.offset_cam2 = offset_cam2

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
        if other.camera.number == self.camera.number:
            raise AssertionError("The two measurements come from the same camera !")

        if ref.cam1.number != self.camera.number or ref.cam2.number != other.camera.number:
            raise AssertionError("The reference use to localize measurements is based on different cameras !")


        # correct the offset of each view and scale the position of the object according to the camera scaling factor
        theta1 = self.camera.position.get_coordinates_spherical()[1]
        vec1 = ((self.position-ref.offset_cam1)/self.camera.scaling).rotate_around_origin(-theta1, 0)-self.camera.position
        theta2 = other.camera.position.get_coordinates_spherical()[1]
        vec2 = ((other.position-ref.offset_cam2)/other.camera.scaling).rotate_around_origin(-theta2, 0)-other.camera.position

        # This time we can generate the following system:
        # vec1.x*t1+self.camera.position.x=vec2.x*t2+other.camera.position.x=realx
        # vec1.y*t1+self.camera.position.y=vec2.y*t2+other.camera.position.y=realy (useless for us)
        # vec1.z*t1+self.camera.position.z=vec2.z*t2+other.camera.position.z=realz
        # Great, so let's solve this and get *real_z* ;)
        if abs(vec1.x) < config.EPSILON:
            if abs(vec1.x) < config.EPSILON and abs(vec1.y) < config.EPSILON:
                # We are just under the camera, so we know the x and y coordinates: thoses of the camera
                if abs(vec2.x) > config.EPSILON:
                    t2 = (self.camera.position.x-other.camera.position.x)/vec2.x
                else:
                    t2 = (self.camera.position.y-other.camera.position.y)/vec2.y
                realz = vec2.z*t2+other.camera.position.z
            else:
                # let's use the y coordinate instead
                t2 = (vec1.z*(other.camera.position.y-self.camera.position.y)+vec1.y*(other.camera.position.z-self.camera.position.z))/(vec1.y*vec2.z-vec1.z*vec2.y)

                realx = vec2.x*t2+other.camera.position.x
                realy = vec2.y*t2+other.camera.position.y
                realz = vec2.z*t2+other.camera.position.z
        else:
            t2 = (vec1.z*(other.camera.position.x-self.camera.position.x)+vec1.x*(other.camera.position.z-self.camera.position.z))/(vec1.x*vec2.z-vec1.z*vec2.x)
            realx = vec2.x*t2+other.camera.position.x
            realy = vec2.y*t2+other.camera.position.y
            realz = vec2.z*t2+other.camera.position.z

        return Vec3D(realx, realy, realz)

    def __str__(self):
        return f"""Measurement(camera={self.camera.number}, position={self.position})"""


class Fiducial:
    def __init__(self, camera: Type[Camera], position: Type[Vec3D]):
        self.camera = camera
        self.position = position
