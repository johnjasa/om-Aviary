import numpy as np
import openmdao.api as om
import aviary.api as av
from ttbw.motor.motor_variables import Aircraft, Mission, Dynamic


ExtendedMetaData = av.CoreMetaData

##### MOTOR VALUES #####

av.add_meta_data(
    Aircraft.Motor.COUNT,
    units=None,
    desc="Number of motors",
    default_value=1,
    option=True,
    meta_data=ExtendedMetaData
)

av.add_meta_data(
    Aircraft.Motor.MASS,
    units="kg",
    desc="Total motor mass (considers number of motors)",
    default_value=1.0,
    meta_data=ExtendedMetaData
)

av.add_meta_data(
    Aircraft.Motor.TORQUE_MAX,
    units="N*m",
    desc="Max torque value that can be output from a single motor",
    meta_data=ExtendedMetaData
)

##### MOTOR MISSION VALUES #####

av.add_meta_data(
    Mission.Motor.EFFICIENCY,
    units=None,
    desc="Motor efficiency",
    default_value=None,
    meta_data=ExtendedMetaData
)

av.add_meta_data(
    Mission.Motor.ELECTRIC_POWER,
    units="kW",
    desc="Power used by all the motors combined",
    default_value=None,
    meta_data=ExtendedMetaData
)

av.add_meta_data(
    Mission.Motor.RPM,
    units="rpm",
    desc="Motor RPM",
    default_value=None,
    meta_data=ExtendedMetaData
)

# TBD I think Jason originally wanted this to be Dynamic.Mission.SHAFT_POWER
# I think these need to be Mission.Motor. values
av.add_meta_data(
    Mission.Motor.SHAFT_POWER,
    units="kW",
    desc="Power output from a single motor",
    default_value=None,
    meta_data=ExtendedMetaData
)

av.add_meta_data(
    Mission.Motor.TORQUE,
    units="N*m",
    desc="Motor torque",
    default_value=None,
    meta_data=ExtendedMetaData
)
