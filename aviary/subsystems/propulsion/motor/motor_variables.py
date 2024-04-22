from aviary.variable_info.variables import Aircraft as av_Aircraft
from aviary.variable_info.variables import Mission as av_Mission

AviaryAircraft = av_Aircraft
AviaryMission = av_Mission


# ---------------------------
# Aircraft data hierarchy
# ---------------------------

class Aircraft(AviaryAircraft):

    class Motor:
        COUNT = "aircraft:motor:count"
        MASS = "aircraft:motor:mass"
        TORQUE_MAX = "aircraft:motor:torque_max"

# ---------------------------
# Mission data hierarchy
# ---------------------------


class Mission(AviaryMission):

    class Motor:
        EFFICIENCY = "mission:motor:efficiency"
        ELECTRIC_POWER = "mission.motor.electric_power"
        RPM = "mission:motor:rpm"
        SHAFT_POWER = "mission:motor:power_out"
        TORQUE = "mission:motor:torque"
