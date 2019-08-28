from __future__ import annotations
import math
from typing import Type, Tuple
import config


class Vec2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def norm(self):
        return math.sqrt((self.x**2) + (self.y**2))

    def distance(self, other: Type[Vec2D]):
        return (self - other).norm()

    def __add__(self, other: Type[Vec2D]):
        return Vec2D(self.x + other.x, self.y + other.y)

    def __neg__(self):
        return Vec2D(-self.x, -self.y)

    def __sub__(self, other: Type[Vec2D]):
        return Vec2D(self.x - other.x, self.y - other.y)

    def __mul__(self, factor: float):
        return Vec2D(self.x * factor, self.y * factor)

    def __str__(self):
        return f"""Vec2D({self.x}, {self.y})"""

    def __repr__(self):
        return str(self)


    # same story as the multiplication
    def __truediv__(self, factor: float):
        return Vec2D(self.x / factor, self.y / factor)

    def to_Vec3D(self, z) -> Vec3D:
        return Vec3D(self.x, self.y, z)

    def rotate_around_origin(self, theta) -> Vec2D:
        return Vec2D(
            self.x * math.cos(theta) - self.y * math.sin(theta),
            self.x * math.sin(theta) + self.y * math.cos(theta))


class Vec3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


    def copy(self) -> Type[Vec3D]:
        return Vec3D(self.x, self.y, self.z)

    @classmethod
    def Cylindrical(cls, r, theta, z):
        return cls(r * math.cos(theta), r * math.sin(theta), z)

    @classmethod
    def Spherical(cls, r, theta, phi):
        return cls(
            r * math.cos(theta) * math.cos(phi),
            r * math.sin(theta) * math.cos(phi),
            r * math.sin(phi))

    # Just throw away the z component
    def to_Vec2D(self) -> Type[Vec2D]:
        return Vec2D(self.x, self.y)

    def __add__(self, other: Type[Vec3D]):
        return Vec3D(self.x + other.x, self.y + other.y, self.z + other.z)

    def __neg__(self):
        return Vec3D(-self.x, -self.y, -self.z)

    def __sub__(self, other: Type[Vec3D]):
        return Vec3D(self.x - other.x, self.y - other.y, self.z - other.z)

    # multiplication means only scaling every component by a factor x (except
    # z, because it's not supposed to change in the experiments)
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
        return math.sqrt((self.x**2) + (self.y**2))

    def norm(self):
        return math.sqrt((self.x**2) + (self.y**2) + (self.z**2))

    def distance(self, other: Type[Vec3D]):
        return (self - other).norm()

    def rotate_around_origin(self, theta, phi=None, origin=None) -> Vec3D:
        obj = self
        if origin is not None:
            obj = self - origin

        if phi is None or phi == 0.0 or phi == -0.0 or phi == 0:
            # Plane rotation
            res = Vec3D(
                obj.x * math.cos(theta) - obj.y * math.sin(theta),
                obj.x * math.sin(theta) + obj.y * math.cos(theta),
                obj.z)
        else:
            r, theta_cur, phi_cur = obj.get_coordinates_spherical()
            res = Vec3D(
                r * math.cos(theta + theta_cur) * math.sin(phi + phi_cur),
                r * math.sin(theta + theta_cur) * math.sin(phi + phi_cur),
                r * math.cos(phi + phi_cur))

        if origin is not None:
            return res + origin
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
            phi = math.acos(self.z / r)
        else:
            phi = 0.
        return (r, theta, phi)

    # Compute the angle between the two vectors, with regard to the origin
    def get_angle(self, other: Type[Vec3D]) -> Tuple[float, float]:
        _, theta1, phi1 = self.get_coordinates_spherical()
        _, theta2, phi2 = other.get_coordinates_spherical()
        theta_res = theta1 - theta2
        # minimize the angle
        if abs(theta_res) > abs(2 * math.pi - theta_res):
            theta_res = theta_res - 2 * math.pi
        phi_res = phi1 - phi2
        if abs(phi_res) > abs(2 * math.pi - phi_res):
            phi_res = phi_res - 2 * math.pi
        return (theta_res, phi_res)

    # Move the point along a line to a position with a given z
    def move_along_z(self, line: Type[Vec3D], z: float):
        if abs(line.z) < config.EPSILON:
            return AssertionError(
                "The point to project must not be on the same level as the reference")
        # We are going to use a parametric equation with the position of the object.
        # We just need to solve t so that line.z*t+self.position.z=z
        t = (z - self.z) / line.z
        x = line.x * t + self.x
        y = line.y * t + self.y

        return Vec3D(x, y, z)

    # Get the normal vector of a plane formed by this vector and the z-axis
    def get_normal_vector(self):
        return Vec3D(-self.y, self.x, 0)


class Camera:
    # We assume the camera is always oriented towards the origin (0, 0, 0) and
    # we specify its position with regard to this origin
    def __init__(self, number: int, position: Type[Vec3D], scaling: float):
        self.number = number
        self.position = position
        self.scaling = scaling

    def __str__(self):
        return f"""Camera(id={self.number}, position={self.position}, scale_factor={self.scaling})"""

    # Get the source, given the virtual image, assuming the point is IN the chamber
    # This function also gives us the vector that goes from the real point to the frontier between the chamber and the glass, aka. the ray direction
    def reverse_refraction_with_ray_direction(self, position: Type[Vec3D]):
        # Convert the position from the virtual position (aka. *after* refraction) against the glass to the (offseted, and without z-axis information) position:
        # 1) project the virtual position on the ray that goes from the virtual position to the camera, with a z-value of CHAMBER_DEPTH + GLASS_WIDTH
        # 2) get the refraction angle
        # 3) get the refracion angle on the other side
        # 4) get the (x, y) coordinates of the ray, projected on the back of the chamber (z=0)
        point1 = position.move_along_z(
                self.position - position,
                config.CHAMBER_DEPTH + config.GLASS_WIDTH)
        camera_glass_vec2D = self.position.to_Vec2D() - point1.to_Vec2D()
        camera_glass_dist2D = camera_glass_vec2D.norm()
        direction_vec = camera_glass_vec2D/camera_glass_dist2D
        if camera_glass_dist2D > config.EPSILON:
            theta1 = math.atan2(camera_glass_dist2D, config.CAMERA_GLASS_DISTANCE)

            # Snell-Descartes law in action !
            sin_theta2 = config.AIR_REFRACTIVE_INDEX / \
                config.GLASS_REFRACTIVE_INDEX * math.sin(theta1)
            point2 = point1 + (direction_vec * config.GLASS_WIDTH * math.tan(
                math.asin(sin_theta2))).to_Vec3D(0)
            point2.z = config.CHAMBER_DEPTH

            theta3 = math.asin(config.GLASS_REFRACTIVE_INDEX /
                               config.CHAMBER_REFRACTIVE_INDEX * sin_theta2)
            point = point2 + (direction_vec * (position.z-config.CHAMBER_DEPTH) *
                              math.tan(theta3)).to_Vec3D(0)

            point.z = position.z
            ray = point2 - point

            return (point, ray)
        else:
            # the camera is just on top of the point, so no refraction is
            # taking place
            point = position.copy()
            return Vec3D(0, 0, config.CHAMBER_DEPTH-position.z)


    def reverse_refraction(self, position: Type[Vec3D]):
        return self.reverse_refraction_with_ray_direction(position)[0]

    # Project a point onto the camera screen, AND apply refraction effects
    def project_from_chamber(self, pos: Type[Vec3D]) -> Type[Vec3D]:
        theta = self.position.get_coordinates_spherical()[1]
        # just on top, do not apply any refraction to it
        if pos.to_Vec2D().distance(self.position.to_Vec2D()) < config.EPSILON:
            v = pos.copy()
            v.z = 0
            # Apply the offset and camera scaling
            return v.rotate_around_origin(theta, 0) * self.scaling

        # determine the plane where the refractions occur
        direction_vec = pos - self.position
        direction_vec.z = 0

        # This iterative algorithm try to determine the correct path for a ray going from 'pos'
        # and reaching the camera, undergoing two diffractions en route
        x = 0

        delta = config.CHAMBER_DIAMETER/20
        nb_iter = 0
        while nb_iter < 500:
            (rev_point, ray_direction) = self.reverse_refraction_with_ray_direction(pos+direction_vec*x)
            if rev_point.distance(pos) < config.EPSILON:
                break;
            if (pos-rev_point).x/direction_vec.x < 0:
                x = x-delta
            else:
                x = x+delta

            nb_iter += 1
            if nb_iter-(nb_iter//20)*20 == 0:
                delta /= 20

        # Now that we know the virtual origin, we can project this point so that its z coordinate is zero
        vec = self.position.move_along_z((pos + direction_vec * x) - self.position, 0.)

        return vec.rotate_around_origin(theta, 0) * self.scaling


class Reference:
    def __init__(self, cam1, offset_cam1, cam2, offset_cam2):
        self.cam1 = cam1
        self.offset_cam1 = offset_cam1
        self.cam2 = cam2
        self.offset_cam2 = offset_cam2

    def __str__(self):
        return f"""Reference(offset_camera{self.cam1.number}={self.offset_cam1}, offset_camera{self.cam2.number}={self.offset_cam2})"""

    # Create a new reference from two fiducials and the real position of the
    # first one, given two views per fiducial
    @classmethod
    def new(cls,
            fiducial1_real_pos: Vec3D,
            fiducial1: Tuple[Type[Fiducial],
                             Type[Fiducial]],
            fiducial2_real_pos: Vec3D,
            fiducial2: Tuple[Type[Fiducial],
                             Type[Fiducial]]) -> Type[Reference]:
        if fiducial1[0].camera.number == fiducial1[1].camera.number or fiducial2[0].camera.number == fiducial2[1].camera.number:
            raise AssertionError(
                "The two measurements come from the same camera !")
        # The two cameras used must be identical in the two pairs
        if fiducial1[0].camera.number != fiducial2[0].camera.number or fiducial1[1].camera.number != fiducial2[1].camera.number:
            raise AssertionError(
                "The cameras used must be identical for the two fiducials !")

        # Offset between the real origin and the origin in the coordinates
        # system of the cameras
        cam1 = fiducial1[0].camera
        offset_cam1 = fiducial1[0].position-cam1.project_from_chamber(fiducial1_real_pos)
        cam2 = fiducial1[1].camera
        offset_cam2 = fiducial1[1].position-cam2.project_from_chamber(fiducial1_real_pos)

        # Angle of each cameras with the x axis
        theta_cam1 = cam1.position.get_coordinates_spherical()[1]
        theta_cam2 = cam2.position.get_coordinates_spherical()[1]

        # vector between the two fiducials
        delta_fid_cam1 = cam1.reverse_refraction(((fiducial1[0].position-offset_cam1)/cam1.scaling).rotate_around_origin(-theta_cam1, 0)) - cam1.reverse_refraction(((fiducial2[0].position-offset_cam1)/cam1.scaling).rotate_around_origin(-theta_cam1, 0))
        # Sanity check: ensure this is the same fiducial in both camera views by comparing the vectors from the first to the second one
        delta_fid_cam2 = cam2.reverse_refraction(((fiducial1[1].position-offset_cam2)/cam2.scaling).rotate_around_origin(-theta_cam2, 0)) - cam2.reverse_refraction(((fiducial2[1].position-offset_cam2)/cam2.scaling).rotate_around_origin(-theta_cam2, 0))
        if delta_fid_cam1.distance(delta_fid_cam2) > 4*config.EPSILON:
            raise AssertionError(
                "You must use the same fiducials when trying to establish a reference")

        return cls(cam1, offset_cam1, cam2, offset_cam2)

    # Return the reference with cameras inverted
    def __neg__(self):
        return Vec3D(self.cam2, self.offset_cam2, self.cam1, self.offset_cam1)


class Measurement:
    def __init__(self, camera: Type[Camera], position: Type[Vec3D]):
        self.camera = camera
        self.position = position

    # Take another measurement (from a different camera) and return the "real"
    # position of the object
    def merge(self, other: Type[Measurement], ref: Type[Reference]) -> Vec3D:
        if ref.cam1.number != self.camera.number or ref.cam2.number != other.camera.number:
            raise AssertionError(
                "The reference use to localize measurements is based on different cameras !")

        # correct the offset of each view and scale the position of the object
        # according to the camera scaling factor
        theta1 = ref.cam1.position.get_coordinates_spherical()[1]
        (vec1, ray1) = ref.cam1.reverse_refraction_with_ray_direction(((self.position - ref.offset_cam1) /
                ref.cam1.scaling).rotate_around_origin(-theta1, 0))
        vec1 = ref.cam1.position - vec1.move_along_z(ray1, 0)
        theta2 = ref.cam2.position.get_coordinates_spherical()[1]
        (vec2, ray2) = ref.cam2.reverse_refraction_with_ray_direction(((other.position - ref.offset_cam2) /
                ref.cam2.scaling).rotate_around_origin(-theta2, 0))
        vec2 = ref.cam2.position - vec2.move_along_z(ray2, 0)

        # This time we can generate the following system:
        # vec1.x*t1+ref.cam1.position.x=vec2.x*t2+ref.cam2.position.x=realx
        # vec1.y*t1+ref.cam1.position.y=vec2.y*t2+ref.cam2.position.y=realy (useless for us)
        # vec1.z*t1+ref.cam1.position.z=vec2.z*t2+ref.cam2.position.z=realz
        # Great, so let's solve this and get *real_z* ;)
        if abs(vec1.x) < config.EPSILON:
            if abs(vec1.x) < config.EPSILON and abs(vec1.y) < config.EPSILON:
                # We are just under the camera, so we know the x and y
                # coordinates: thoses of the camera
                if abs(vec2.x) > config.EPSILON:
                    t2 = (ref.cam1.position.x - ref.cam2.position.x) / vec2.x
                else:
                    t2 = (ref.cam1.position.y - ref.cam2.position.y) / vec2.y
                realz = vec2.z * t2 + ref.cam2.position.z
            else:
                # let's use the y coordinate instead
                t2 = (vec1.z * (ref.cam2.position.y - ref.cam1.position.y) + vec1.y *
                      (ref.cam2.position.z - ref.cam1.position.z)) / (vec1.y * vec2.z - vec1.z * vec2.y)

                realx = vec2.x * t2 + ref.cam2.position.x
                realy = vec2.y * t2 + ref.cam2.position.y
                realz = vec2.z * t2 + ref.cam2.position.z
        else:
            t2 = (vec1.z * (ref.cam2.position.x - ref.cam1.position.x) + vec1.x *
                  (ref.cam2.position.z - ref.cam1.position.z)) / (vec1.x * vec2.z - vec1.z * vec2.x)
            realx = vec2.x * t2 + ref.cam2.position.x
            realy = vec2.y * t2 + ref.cam2.position.y
            realz = vec2.z * t2 + ref.cam2.position.z

        return Vec3D(realx, realy, realz)

    def __str__(self):
        return f"""Measurement(camera={self.camera.number}, position={self.position})"""

    def __repr__(self):
        return str(self)

class Fiducial(Measurement):
    def __init__(self, camera: Type[Camera], position: Type[Vec3D]):
        super().__init__(camera, position)
