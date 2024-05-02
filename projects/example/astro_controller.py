from astropy.coordinates import get_moon
from astropy.time import Time
from astropy.coordinates import AltAz
from astropy.coordinates import EarthLocation

class AstroController:
    def __init__(self, motor_controller):
        self.motor_controller = motor_controller

    def point_at_moon(self):
        # Get the current time
        now = Time.now()

        # Get the current location of the moon
        moon = get_moon(now)

        # Convert the moon's location to altitude and azimuth
        location = EarthLocation(lat=51.5074*u.deg, lon=0.1278*u.deg, height=0*u.m)
        moon_altaz = moon.transform_to(AltAz(obstime=now,location=location))

        # Point the camera at the moon
        self.motor_controller.pan(moon_altaz.az.deg)
        self.motor_controller.tilt(90 - moon_altaz.alt.deg)
