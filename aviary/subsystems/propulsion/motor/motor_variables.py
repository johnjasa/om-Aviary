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
        POWER_MAX = "aircraft:motor:power_max"
        TYPE = "aircraft:motor:type"

# ---------------------------
# Mission data hierarchy
# ---------------------------


class Mission(AviaryMission):

    class Motor:
        EFFICIENCY = "mission:motor:efficiency"
        # ENERGY_CON = "aircraft:motor:energy_con"
        ENERGY_REQUIRED = "aircraft:motor:energy_required"
        GEAR_RATIO = 'mission:motor:gear_ratio'
        POWER_CON = "mission:motor:power_con"
        POWER_IN = "mission:motor:power_in"
        POWER_OUT = "mission:motor:power_out"
        RPM = "mission:motor:rpm"
        TORQUE = "mission:motor:torque"
        VOLTAGE = "mission:motor:voltage"

    # class Converter:
    #     EFFICIENCY = "mission:converter:efficiency"
