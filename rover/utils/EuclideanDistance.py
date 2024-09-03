import numpy as np
import math

class Point3D:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def from_lat_lon(cls, lat, lon, home_lat, home_lon, altitude):
        x, y = DistanceCalculator.lat_lon_to_meters(lat, lon, home_lat, home_lon)
        z = -1 * altitude
        return cls(x, y, z)

    def to_array(self):
        return np.array([self.x, self.y, self.z])

class DistanceCalculator:
    @staticmethod
    def lat_lon_to_meters(lat1, lon1, lat2, lon2):
        R = 6371000

        # Converting degrees to radians
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        # Haversine formula
        a = math.sin(delta_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(delta_lambda/2)**2
        c = 2*math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Convert latitude and longitude differences to meters
        x = math.cos((phi1+phi2)/2) * delta_lambda * R * -1
        y = delta_phi * R * -1

        return y, x
   

    @staticmethod
    def shortest_distance(p0, p1, p2):
        v = p2.to_array() - p1.to_array()
        w = p0.to_array() - p1.to_array()

        length_squared = np.dot(v, v)

        if length_squared == 0:
            return np.linalg.norm(p0.to_array() - p1.to_array())
        else:
            dot_product = np.dot(v, w)
            t = dot_product / length_squared
            t = max(0, min(t, 1))

            p_closest = p1.to_array() + t * v
            return np.linalg.norm(p0.to_array() - p_closest)

    @staticmethod
    def distance_between_points(p0, p1):
        return np.linalg.norm(p0.to_array() - p1.to_array())

    @staticmethod
    def get_distance_to_path(p0, past_wp, current_wp):
        v = current_wp.to_array() - past_wp.to_array()
        w = p0.to_array() - past_wp.to_array()

        c1 = np.dot(w, v)
        if c1 <= 0:
            return np.linalg.norm(p0.to_array() - past_wp.to_array())

        c2 = np.dot(v, v)
        if c2 <= c1:
            return np.linalg.norm(p0.to_array() - current_wp.to_array())

        b = c1 / c2
        p_closest = past_wp.to_array() + b * v
        return np.linalg.norm(p0.to_array() - p_closest)
