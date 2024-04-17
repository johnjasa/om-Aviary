import numpy as np

import openmdao.api as om

from aviary.variable_info.functions import add_aviary_input, add_aviary_output
from ttbw.motor.motor_variables import Mission
from ttbw.motor.motor_variable_meta_data import ExtendedMetaData


class MotorPower(om.ExplicitComponent):

    def initialize(self):
        self.options.declare("num_nodes", types=int)

    def setup(self):
        n = self.options["num_nodes"]

        # Inputs
        add_aviary_input(self, Mission.Motor.POWER_OUT,
                         val=np.ones(n), meta_data=ExtendedMetaData)
        add_aviary_input(self, Mission.Motor.EFFICIENCY,
                         val=np.ones(n), meta_data=ExtendedMetaData)
        # add_aviary_input(self, Mission.Converter.EFFICIENCY,
        #                  val=np.ones(n), meta_data=ExtendedMetaData)

        # Outputs
        add_aviary_output(self, Mission.Motor.POWER_IN,
                          val=np.ones(n), meta_data=ExtendedMetaData)

        # Partials
        ar = np.arange(n)
        self.declare_partials(Mission.Motor.POWER_IN, [
            Mission.Motor.POWER_OUT, Mission.Motor.EFFICIENCY],
            rows=ar, cols=ar)

    def compute(self, inputs, outputs):

        power_out = inputs[Mission.Motor.POWER_OUT]
        motor_eff = inputs[Mission.Motor.EFFICIENCY]
        # converter_eff = inputs[Mission.Converter.EFFICIENCY]

        outputs[Mission.Motor.POWER_IN] = -power_out/(motor_eff)

        # print()
        # print('--show power--')
        # print('power_out', power_out)
        # print('motor_eff', motor_eff)
        # print('converter_eff', converter_eff)
        # print('power_in', outputs[Mission.Motor.POWER_IN])

    def compute_partials(self, inputs, partials):

        power_out = inputs[Mission.Motor.POWER_OUT]
        motor_eff = inputs[Mission.Motor.EFFICIENCY]
        # converter_eff = inputs[Mission.Converter.EFFICIENCY]

        partials[Mission.Motor.POWER_IN, Mission.Motor.POWER_OUT] = -1.0/(motor_eff)
        partials[Mission.Motor.POWER_IN,
                 Mission.Motor.EFFICIENCY] = power_out/(motor_eff**2)
        # partials[Mission.Motor.POWER_IN, Mission.Converter.EFFICIENCY] = power_out/(motor_eff*converter_eff**2)
