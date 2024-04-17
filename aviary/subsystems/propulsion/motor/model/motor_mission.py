import numpy as np

import openmdao.api as om
from aviary.utils.aviary_values import AviaryValues

from ttbw.motor.motor_variables import Aircraft, Mission
from ttbw.propulsion.pycycle_variables import Dynamic
from ttbw.motor.model.motor_efficiency import MotorEfficiency
from ttbw.motor.model.converter import Converter
from ttbw.motor.model.motor_power import MotorPower


class MotorMission(om.Group):

    def initialize(self):
        self.options.declare("num_nodes", types=int)
        self.options.declare(
            'aviary_inputs', types=AviaryValues,
            desc='collection of Aircraft/Mission specific options',
            default=None,
        )

    def setup(self):
        n = self.options["num_nodes"]

        self.add_subsystem(name="motor_efficiency",
                           subsys=MotorEfficiency(num_nodes=n))

        # self.add_subsystem(name="converter",
        #                    subsys=Converter(num_nodes=n))

        self.add_subsystem(name="motor_power",
                           subsys=MotorPower(num_nodes=n))

        self.promotes(
            "motor_efficiency",
            inputs=[
                Mission.Motor.RPM,
                Mission.Motor.POWER_OUT,
                # Dynamic.Engine.LP_MECH_SPEED,
                # Mission.Motor.GEAR_RATIO
            ],
            outputs=[Mission.Motor.EFFICIENCY]
        )

        # self.promotes(
        #     "converter",
        #     inputs=[Mission.Motor.VOLTAGE],
        #     outputs=[Mission.Converter.EFFICIENCY]
        # )

        self.promotes(
            "motor_power",
            inputs=[
                Mission.Motor.POWER_OUT,
                Mission.Motor.EFFICIENCY,
                # Mission.Converter.EFFICIENCY
            ],
            outputs=[Mission.Motor.POWER_IN]
        )
