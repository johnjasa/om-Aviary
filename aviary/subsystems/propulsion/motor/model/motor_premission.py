import openmdao.api as om

from ttbw.motor.motor_variables import Aircraft
from aviary.utils.aviary_values import AviaryValues
from ttbw.motor.model.motor_weight import MotorWeight

class MotorPreMission(om.Group):
    """
    Calculate electric motor mass
    """

    def initialize(self):
        self.options.declare(
            "aviary_inputs", types=AviaryValues,
            desc="collection of Aircraft/Mission specific options",
            default=None,
        )

    def setup(self):

        motor_keywords = self.options["aviary_inputs"].get_val(Aircraft.Motor.TYPE)
        num_motors = self.options["aviary_inputs"].get_val(Aircraft.Motor.COUNT)

        self.add_subsystem(name="motor_weight", subsys=MotorWeight(keywords=motor_keywords, num_motors=num_motors))

        self.promotes(
            "motor_weight",
            inputs=[
                Aircraft.Motor.POWER_MAX
            ],
            outputs=[
                Aircraft.Motor.MASS
            ])