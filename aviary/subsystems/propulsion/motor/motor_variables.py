from aviary.variable_info.variables import Aircraft as av_Aircraft
from aviary.variable_info.variables import Dynamic as av_Dynamic
from aviary.variable_info.variables import Mission as av_Mission

# ---------------------------
# Aircraft data hierarchy
# ---------------------------


class Aircraft(av_Aircraft):

    class Motor:
        COUNT = "aircraft:motor:count"
        MASS = "aircraft:motor:mass"
        RPM = "aircraft:motor:rpm"
        TORQUE_MAX = "aircraft:motor:torque_max"

# ---------------------------
# Mission data hierarchy
# ---------------------------


class Dynamic(av_Dynamic):

    class Mission(av_Dynamic.Mission):

        class Motor:
            EFFICIENCY = "dynamic:mission:motor:efficiency"
            ELECTRIC_POWER = "dynamic:mission.motor.electric_power"
            SHAFT_POWER = "dynamic:mission:motor:power_out"
            TORQUE = "dynamic:mission:motor:torque"
            TORQUE_CON = "Dynamic.Mission.Motor.TORQUE_CON"


class Mission(av_Mission):

    class Motor:
        ELECTRIC_ENERGY = "mission.motor.electric_energy"
