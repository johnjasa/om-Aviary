import numpy as np

import openmdao.api as om
from aviary.utils.aviary_values import AviaryValues

from aviary.subsystems.propulsion.motor.motor_variables import Dynamic, Aircraft, Mission
from aviary.subsystems.propulsion.motor.model.motor_map import MotorMap


class MotorMission(om.Group):

    def initialize(self):
        self.options.declare("num_nodes", types=int)
        self.options.declare(
            'aviary_inputs', types=AviaryValues,
            desc='collection of Aircraft/Mission specific options',
            default=None,
        )
        self.name = 'motor_mission'

    def setup(self):
        n = self.options["num_nodes"]
        num_motors = self.options["aviary_inputs"].get_val(Aircraft.Motor.COUNT)

        ivc = om.IndepVarComp()
        ivc.add_output(Dynamic.Mission.FUEL_FLOW_RATE_NEGATIVE,
                       val=np.zeros(n), units='kg/s')
        ivc.add_output(Dynamic.Mission.ELECTRIC_POWER, val=np.zeros(n), units='kW')
        ivc.add_output(Dynamic.Mission.NOX_RATE, val=np.zeros(n), units='kg/s')
        self.add_subsystem('ivc', ivc, promotes=['*'])

        self.add_subsystem('motor_map', MotorMap(num_nodes=n),
                           promotes_inputs=[Dynamic.Mission.THROTTLE,
                                            Aircraft.Engine.SCALE_FACTOR,
                                            Aircraft.Motor.RPM],
                           promotes_outputs=[Dynamic.Mission.Motor.TORQUE,
                                             Dynamic.Mission.Motor.EFFICIENCY])

        self.add_subsystem('power_comp',
                           om.ExecComp('P = T * pi * RPM / 30',
                                       P={'val': np.ones(n), 'units': 'kW'},
                                       T={'val': np.ones(n), 'units': 'kN*m'},
                                       RPM={'val': np.ones(n), 'units': 'rpm'}),
                           promotes_inputs=[("T", Dynamic.Mission.Motor.TORQUE),
                                            ("RPM", Aircraft.Motor.RPM)],
                           promotes_outputs=[("P", Dynamic.Mission.Motor.SHAFT_POWER)])

        self.add_subsystem('energy_comp',
                           om.ExecComp('P_elec = P / eff * num_motors',
                                       P={'val': np.ones(n), 'units': 'kW'},
                                       P_elec={'val': np.ones(n), 'units': 'kW'},
                                       eff={'val': np.ones(n), 'units': 'unitless'},
                                       num_motors={'val': num_motors, 'units': 'unitless'}),
                           promotes_inputs=[("P", Dynamic.Mission.Motor.SHAFT_POWER),
                                            ("eff", Dynamic.Mission.Motor.EFFICIENCY)],
                           promotes_outputs=[("P_elec", Dynamic.Mission.Motor.ELECTRIC_POWER)])

        self.add_subsystem('torque_con',
                           om.ExecComp('torque_con = torque_max - torque_mission',
                                       torque_con={'val': np.ones(n), 'units': 'kN*m'},
                                       torque_max={'val': np.ones(n), 'units': 'kN*m'},
                                       torque_mission={'val': np.ones(n), 'units': 'kN*m'}),
                           promotes_inputs=[('torque_mission', Dynamic.Mission.Motor.TORQUE),
                                            ('torque_max', Aircraft.Motor.TORQUE_MAX)],
                           promotes_outputs=[('torque_con', Dynamic.Mission.Motor.TORQUE_CON)])
