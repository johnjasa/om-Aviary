import numpy as np
import openmdao.api as om
import aviary.api as av
from ttbw.motor.motor_variables import Aircraft, Mission


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
    Aircraft.Motor.POWER_MAX,
    units="kW",
    desc="Maximum motor power; used for computing motor mass",
    default_value=100.0,
    meta_data=ExtendedMetaData
)

av.add_meta_data(
    Aircraft.Motor.TYPE,
    units=None,
    desc="""list of keywords to descirbe the motor; used to estimate weight.
         See motor_weight.py for the list of valid keywords""",
    default_value=["Axial"],
    option=True,
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

# av.add_meta_data(
#     Mission.Motor.ENERGY_CON,
#     units="kW*h",
#     desc="Total energy constraint: battery energy available - motor energy required >= 0",
#     default_value=None,
#     meta_data=ExtendedMetaData
# )

av.add_meta_data(
    Mission.Motor.ENERGY_REQUIRED,
    units="kW*h",
    desc="Total motor energy required: integral of POWER_IN",
    default_value=None,
    meta_data=ExtendedMetaData
)

av.add_meta_data(
    Mission.Motor.GEAR_RATIO,
    units=None,
    desc='gear ratio between engine and motor RPM',
    default_value=1,
    meta_data=ExtendedMetaData
)

av.add_meta_data(
    Mission.Motor.POWER_CON,
    units="kW",
    desc="Power slack constraint: POWER_MAX >= max(POWER)",
    default_value=None,
    meta_data=ExtendedMetaData
)

av.add_meta_data(
    Mission.Motor.POWER_IN,
    units="kW",
    desc="Power required to drive the motor",
    default_value=None,
    meta_data=ExtendedMetaData
)

av.add_meta_data(
    Mission.Motor.POWER_OUT,
    units="kW",
    desc="Power output from the motor",
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

av.add_meta_data(
    Mission.Motor.TORQUE,
    units="N*m",
    desc="Motor torque",
    default_value=None,
    meta_data=ExtendedMetaData
)

# av.add_meta_data(
#     Mission.Motor.VOLTAGE,
#     units="V",
#     desc="Motor voltage",
#     # *** This is meant to replace Mission.Battery.Voltage so it's not really motor voltage??
#     default_value=None,
#     meta_data=ExtendedMetaData
# )

# ##### DC CONVERTER MISSION VALUES #####

# av.add_meta_data(
#     Mission.Converter.EFFICIENCY,
#     units=None,
#     desc="DC voltage converter efficiency",
#     default_value=None,
#     meta_data=ExtendedMetaData
# )
