from astropy.coordinates import get_body, AltAz, EarthLocation
from astropy.time import Time
from astropy import units as u
from motor_controller import MotorController

class AstroController:
    def __init__(self, pan_motor_controller, tilt_motor_controller, current_location):
        self.pan_motor_controller = pan_motor_controller
        self.tilt_motor_controller = tilt_motor_controller
        self.location = current_location

    def point_at_moon(self):
        #see here: https://docs.astropy.org/en/stable/generated/examples/coordinates/plot_obs-planning.html
        # Get the current time
        now = Time.now()

        # Get the current location of the moon using the updated function
        moon = get_body("moon", now)

        # Convert the moon's location to altitude and azimuth
        moon_altaz = moon.transform_to(AltAz(obstime=now, location=self.location))

        # Point the camera at the moon
        pan_angle = moon_altaz.az.deg
        tilt_angle = 90 - moon_altaz.alt.deg
        self.pan_motor_controller.step_to_angle(pan_angle)
        self.tilt_motor_controller.step_to_angle(tilt_angle)

