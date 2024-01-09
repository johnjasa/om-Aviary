'''
NOTES:
Includes:
Takeoff, Climb, Cruise, Descent, Landing
Computed Aero
Large Single Aisle 1 data
'''
import unittest

import numpy as np
from openmdao.utils.testing_utils import use_tempdirs

from aviary.interface.methods_for_level1 import run_aviary
from aviary.validation_cases.benchmark_utils import \
    compare_against_expected_values
from aviary.variable_info.variables import Dynamic


@use_tempdirs
class ProblemPhaseTestCase(unittest.TestCase):
    def setUp(self):
        expected_dict = {}
        expected_dict['times'] = np.array([[120.],
                                           [163.76271231],
                                           [224.1463232],
                                           [243.25752959],
                                           [243.25752959],
                                           [336.40818102],
                                           [464.93706768],
                                           [505.61602085],
                                           [505.61602085],
                                           [626.4698655],
                                           [793.2235045],
                                           [846.0004611],
                                           [846.0004611],
                                           [966.85430575],
                                           [1133.60794474],
                                           [1186.38490134],
                                           [1186.38490134],
                                           [1279.53555278],
                                           [1408.06443943],
                                           [1448.74339261],
                                           [1448.74339261],
                                           [1492.50610492],
                                           [1552.88971581],
                                           [1572.0009222],
                                           [1572.0009222],
                                           [10224.86854151],
                                           [22164.05978091],
                                           [25942.77298722],
                                           [25942.77298722],
                                           [26009.89831864],
                                           [26102.51757551],
                                           [26131.83125364],
                                           [26131.83125364],
                                           [26268.92025959],
                                           [26458.07510754],
                                           [26517.94197018],
                                           [26517.94197018],
                                           [26680.92723458],
                                           [26905.81363368],
                                           [26976.98941009],
                                           [26976.98941009],
                                           [27114.07841604],
                                           [27303.23326399],
                                           [27363.10012663],
                                           [27363.10012663],
                                           [27430.22545805],
                                           [27522.84471492],
                                           [27552.15839306]])

        expected_dict['altitudes'] = np.array([[10.668],
                                               [0.],
                                               [1176.78911697],
                                               [1673.72886901],
                                               [1673.72886901],
                                               [3865.01102367],
                                               [6195.61836863],
                                               [6782.34302216],
                                               [6782.34302216],
                                               [8109.95869793],
                                               [9272.93435889],
                                               [9556.37792864],
                                               [9556.37792864],
                                               [10094.94809064],
                                               [10559.35775736],
                                               [10627.1932692],
                                               [10627.1932692],
                                               [10668.],
                                               [10650.08598518],
                                               [10649.95573356],
                                               [10649.95573356],
                                               [10657.29116463],
                                               [10668.],
                                               [10668.],
                                               [10668.],
                                               [10668.],
                                               [10668.],
                                               [10668.],
                                               [10668.],
                                               [10668.],
                                               [10307.54164946],
                                               [10143.7274239],
                                               [10143.7274239],
                                               [9248.78597393],
                                               [7827.69756506],
                                               [7363.41307882],
                                               [7363.41307882],
                                               [6126.8939629],
                                               [4470.55591283],
                                               [3951.86624585],
                                               [3951.86624585],
                                               [2939.92813668],
                                               [1471.34739294],
                                               [982.44499735],
                                               [982.44499735],
                                               [449.58751714],
                                               [10.83591043],
                                               [10.668]])

        expected_dict['masses'] = np.array([[68463.53921924],
                                            [68388.16396576],
                                            [68245.77323713],
                                            [68201.21751908],
                                            [68201.21751908],
                                            [68008.46518048],
                                            [67807.59357082],
                                            [67754.96222353],
                                            [67754.96222353],
                                            [67619.16435342],
                                            [67463.77290019],
                                            [67418.37147793],
                                            [67418.37147793],
                                            [67321.58684141],
                                            [67204.00255349],
                                            [67170.96641995],
                                            [67170.96641995],
                                            [67116.2987407],
                                            [67044.08894595],
                                            [67020.83854678],
                                            [67020.83854678],
                                            [66995.00611866],
                                            [66954.5572371],
                                            [66939.76926892],
                                            [66939.76926892],
                                            [61880.26581581],
                                            [55221.27982315],
                                            [53175.67231214],
                                            [53175.67231214],
                                            [53158.01594442],
                                            [53144.299632],
                                            [53140.88521042],
                                            [53140.88521042],
                                            [53127.49149749],
                                            [53111.02960542],
                                            [53105.82637933],
                                            [53105.82637933],
                                            [53090.66670078],
                                            [53066.21593581],
                                            [53057.26691014],
                                            [53057.26691014],
                                            [53038.09861563],
                                            [53006.77727746],
                                            [52995.48090755],
                                            [52995.48090755],
                                            [52981.90805198],
                                            [52962.16924161],
                                            [52955.84313972]])

        expected_dict['ranges'] = np.array([[1166.20699576],
                                            [5593.37409715],
                                            [15160.34254633],
                                            [18452.29987419],
                                            [18452.29987419],
                                            [35953.4905749],
                                            [61610.20915607],
                                            [69693.60643145],
                                            [69693.60643145],
                                            [94109.378342],
                                            [128861.35815474],
                                            [140228.92014561],
                                            [140228.92014561],
                                            [166663.67193054],
                                            [203797.88470247],
                                            [215616.8251792],
                                            [215616.8251792],
                                            [236507.17277307],
                                            [265348.67959522],
                                            [274478.82401463],
                                            [274478.82401463],
                                            [284316.3485801],
                                            [298061.71411326],
                                            [302496.32074313],
                                            [302496.32074313],
                                            [2329499.36229261],
                                            [5126349.84245871],
                                            [6011543.45679734],
                                            [6011543.45679734],
                                            [6026497.11291725],
                                            [6044998.00245228],
                                            [6050451.95986915],
                                            [6050451.95986915],
                                            [6074179.71278433],
                                            [6103973.32165406],
                                            [6113145.04319224],
                                            [6113145.04319224],
                                            [6137396.13533869],
                                            [6169641.84998078],
                                            [6179602.4331379],
                                            [6179602.4331379],
                                            [6198630.75436767],
                                            [6224716.73281895],
                                            [6232974.51970883],
                                            [6232974.51970883],
                                            [6242235.03685456],
                                            [6254092.42671214],
                                            [6257352.4]])

        expected_dict['velocities'] = np.array([[64.4147653],
                                                [133.69502146],
                                                [174.31936373],
                                                [180.24083219],
                                                [180.24083219],
                                                [196.61398458],
                                                [200.67345509],
                                                [200.77798066],
                                                [200.77798066],
                                                [204.32902616],
                                                [213.53269554],
                                                [216.30794138],
                                                [216.30794138],
                                                [220.90502555],
                                                [223.85447586],
                                                [224.18464453],
                                                [224.18464453],
                                                [224.33459955],
                                                [224.43913124],
                                                [224.63981271],
                                                [224.63981271],
                                                [225.38258712],
                                                [230.7187408],
                                                [234.25795132],
                                                [234.25795132],
                                                [234.25795132],
                                                [234.25795132],
                                                [234.25795132],
                                                [234.25795132],
                                                [212.06605788],
                                                [188.96924746],
                                                [183.2491682],
                                                [183.2491682],
                                                [164.64559845],
                                                [153.53645555],
                                                [151.57537965],
                                                [151.57537965],
                                                [146.52626765],
                                                [140.88518413],
                                                [139.7046818],
                                                [139.7046818],
                                                [138.44302405],
                                                [138.07991553],
                                                [138.08432897],
                                                [138.08432897],
                                                [136.46253134],
                                                [116.34759903],
                                                [102.07377559]])

        self.expected_dict = expected_dict

    def bench_test_swap_1_GwFm(self):
        phase_info = {
            "pre_mission": {"include_takeoff": True, "optimize_mass": True},
            "climb": {
                "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
                "user_options": {
                    'fix_initial': {Dynamic.Mission.MASS: False, Dynamic.Mission.RANGE: False},
                    'input_initial': True,
                    "optimize_mach": True,
                    "optimize_altitude": True,
                    "use_polynomial_control": False,
                    "num_segments": 6,
                    "order": 3,
                    "solve_for_range": False,
                    "initial_mach": (0.2, "unitless"),
                    "final_mach": (0.79, "unitless"),
                    "mach_bounds": ((0.1, 0.8), "unitless"),
                    "initial_altitude": (0.0, "ft"),
                    "final_altitude": (35000.0, "ft"),
                    "altitude_bounds": ((0.0, 36000.0), "ft"),
                    "throttle_enforcement": "path_constraint",
                    "constrain_final": False,
                    "fix_duration": False,
                    "initial_bounds": ((0.0, 0.0), "min"),
                    "duration_bounds": ((5.0, 50.0), "min"),
                    "no_descent": True,
                    "add_initial_mass_constraint": False,
                },
                "initial_guesses": {"times": ([0, 40.0], "min")},
            },
            "cruise": {
                "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
                "user_options": {
                    "optimize_mach": True,
                    "optimize_altitude": True,
                    "polynomial_control_order": 1,
                    "use_polynomial_control": True,
                    "num_segments": 1,
                    "order": 3,
                    "solve_for_range": False,
                    "initial_mach": (0.79, "unitless"),
                    "final_mach": (0.79, "unitless"),
                    "mach_bounds": ((0.78, 0.8), "unitless"),
                    "initial_altitude": (35000.0, "ft"),
                    "final_altitude": (35000.0, "ft"),
                    "altitude_bounds": ((35000.0, 35000.0), "ft"),
                    "throttle_enforcement": "boundary_constraint",
                    "fix_initial": False,
                    "constrain_final": False,
                    "fix_duration": False,
                    "initial_bounds": ((64.0, 192.0), "min"),
                    "duration_bounds": ((60.0, 7200.0), "min"),
                },
                "initial_guesses": {"times": ([128, 113], "min")},
            },
            "descent": {
                "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
                "user_options": {
                    "optimize_mach": True,
                    "optimize_altitude": True,
                    "use_polynomial_control": False,
                    "num_segments": 5,
                    "order": 3,
                    "solve_for_range": False,
                    "initial_mach": (0.79, "unitless"),
                    "final_mach": (0.3, "unitless"),
                    "mach_bounds": ((0.2, 0.8), "unitless"),
                    "initial_altitude": (35000.0, "ft"),
                    "final_altitude": (500.0, "ft"),
                    "altitude_bounds": ((0.0, 35000.0), "ft"),
                    "throttle_enforcement": "path_constraint",
                    "fix_initial": False,
                    "constrain_final": True,
                    "fix_duration": False,
                    "initial_bounds": ((120.5, 361.5), "min"),
                    "duration_bounds": ((5.0, 60.0), "min"),
                    "no_climb": False
                },
                "initial_guesses": {"times": ([241, 58], "min")},
            },
            "post_mission": {
                "include_landing": True,
                "constrain_range": True,
                "target_range": (3360.0, "nmi"),
            },
        }

        prob = run_aviary('models/test_aircraft/aircraft_for_bench_GwFm.csv', phase_info,
                          mission_method="simple", mass_method="GASP", max_iter=15)

        compare_against_expected_values(prob, self.expected_dict)


if __name__ == '__main__':
    z = ProblemPhaseTestCase()
    z.setUp()
    z.bench_test_swap_1_GwFm()