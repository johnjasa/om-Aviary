import numpy as np

import openmdao.api as om
from aviary.utils.aviary_values import AviaryValues

from ttbw.motor.motor_variables import Dynamic, Aircraft, Mission
from ttbw.motor.model.motor_map import MotorMap


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
        num_motors = self.options["aviary_inputs"].get_val(Aircraft.Motor.COUNT)

        self.add_subsystem('motor_map', MotorMap(num_nodes=n),
                           promote_inputs=[Dynamic.Mission.THROTTLE,
                                           Aircraft.Engine.SCALE_FACTOR,
                                           Mission.Motor.RPM],
                           promote_outputs=[Mission.Motor.TORQUE,
                                            Mission.Motor.EFFICIENCY])

        self.add_subsystem('power_comp',
                           om.ExecComp('P = T * pi * RPM / 30',
                                       'P_elec = P / eff * num_motors',
                                       P={'val': np.ones(n), 'units': 'kW'},
                                       T={'val': np.ones(n), 'units': 'kN*m'},
                                       RPM={'val': np.ones(n), 'units': 'rpm'},
                                       P_elec={'val': np.ones(n), 'units': 'kW'},
                                       eff={'val': np.ones(n), 'units': 'unitless'},
                                       num_motors={'val': num_motors, 'units': 'unitless'}),
                           promote_inputs=[("T", Mission.Motor.TORQUE),
                                           ("RPM", Mission.Motor.RPM),
                                           ("eff", Mission.Motor.EFFICIENCY)],
                           promote_outputs=[("P", Dynamic.Mission.SHAFT_POWER),
                                            ("P_elec", Dynamic.Mission.ELECTRIC_POWER),])
