import numpy as np

import openmdao.api as om

from motor_variables import Dynamic, Aircraft, Mission


class MotorMap(om.Group):

    '''
    This function takes in 0-1 values for electric motor throttle,
    scales those values into 0 to max_torque on the motor map
    this also allows us to solve for motor efficiency
    then we scale the torque up based on the actual scale factor of the motor.
    This avoids the need to rescale the map values, and still allows for the motor scale to be optimized.
    Scaling only effects Torque. RPM is not scaled and is assumed to be maxed at 6,000 rpm.
    The original maps were put together for a 746kw (1,000 hp) electric motor published in the TTBW paper:
    https://ntrs.nasa.gov/api/citations/20230016987/downloads/TTBW_SciTech_2024_Final_12_5_2023.pdf
    The map is shown in Figure 4.

    Inputs
    ----------
    Dynamic.Mission.THROTTLE : float (unitless) (0 to 1)
        The throttle command which will be translated into torque output from the engine
    Aircraft.Engine.SCALE_FACTOR : float (unitless) (positive) 
    Mission.Motor.RPM : float (rpm) (0 to 6000)

    Outputs
    ----------
    Mission.Motor.TORQUE : float (positive)
    Mission.Motor.EFFICIENCY : float (positive)

    '''

    def initialize(self):
        self.options.declare("num_nodes", types=int)

    def setup(self):
        n = self.options["num_nodes"]

        # Training data
        rpm_vals = np.linspace(0., 6000., 14)
        torque_vals = np.linspace(0., 1800., 17)

        # Create a structured metamodel to compute motor efficiency from rpm
        motor = om.MetaModelStructuredComp(method="slinear",
                                           vec_size=n,
                                           extrapolate=True)
        motor.add_input(Mission.Motor.RPM, val=np.ones(n),
                        training_data=rpm_vals,
                        units="rpm")
        motor.add_input("T_unscaled", val=np.ones(n),  # unscaled torque
                        training_data=torque_vals,
                        units="N*m")
        motor.add_output(Mission.Motor.EFFICIENCY, val=np.ones(n),
                         training_data=np.array([
                             # speed----     0    250    500    750   1000   1250   1500   1750   2000   2250   2500   2750   3000   3250
                             [0.500, 0.500, 0.500, 0.500, 0.500, 0.500, 0.500, 0.500,
                                 0.500, 0.500, 0.500, 0.500, 0.500, 0.500],  # 0
                             [0.500, 0.600, 0.735, 0.795, 0.827, 0.848, 0.861, 0.869,
                                 0.875, 0.880, 0.880, 0.882, 0.882, 0.882],  # 50
                             [0.500, 0.658, 0.775, 0.840, 0.868, 0.886, 0.898, 0.905,
                                 0.910, 0.915, 0.920, 0.922, 0.923, 0.924],  # 100
                             [0.500, 0.667, 0.790, 0.848, 0.877, 0.894, 0.903, 0.913,
                                 0.922, 0.925, 0.930, 0.930, 0.935, 0.940],  # 150
                             [0.500, 0.660, 0.780, 0.848, 0.878, 0.897, 0.908, 0.918,
                                 0.923, 0.928, 0.930, 0.935, 0.945, 0.945],  # 200
                             [0.500, 0.650, 0.775, 0.842, 0.872, 0.892, 0.906, 0.917,
                                 0.922, 0.927, 0.930, 0.935, 0.950, 0.950],  # 250
                             [0.500, 0.630, 0.770, 0.832, 0.863, 0.888, 0.902, 0.911,
                                 0.919, 0.924, 0.929, 0.930, 0.935, 0.940],  # 300
                             [0.500, 0.620, 0.760, 0.823, 0.859, 0.882, 0.894, 0.908,
                                 0.913, 0.922, 0.926, 0.929, 0.930, 0.930],  # 350
                             [0.500, 0.600, 0.750, 0.817, 0.851, 0.874, 0.889, 0.902,
                                 0.909, 0.916, 0.922, 0.925, 0.928, 0.930],  # 400
                             [0.500, 0.580, 0.730, 0.802, 0.841, 0.867, 0.883, 0.894,
                                 0.904, 0.911, 0.915, 0.921, 0.924, 0.923],  # 450
                             [0.500, 0.565, 0.720, 0.785, 0.830, 0.858, 0.873, 0.888,
                                 0.897, 0.905, 0.911, 0.916, 0.918, 0.917],  # 500
                             [0.500, 0.545, 0.700, 0.775, 0.819, 0.847, 0.866, 0.881,
                                 0.890, 0.898, 0.905, 0.908, 0.909, 0.903],  # 550
                             [0.500, 0.525, 0.680, 0.765, 0.807, 0.836, 0.855, 0.871,
                                 0.882, 0.890, 0.897, 0.902, 0.900, 0.880],  # 600
                             [0.500, 0.500, 0.665, 0.743, 0.785, 0.822, 0.845, 0.860,
                                 0.872, 0.881, 0.889, 0.891, 0.879, 0.840],  # 650
                             [0.500, 0.500, 0.640, 0.725, 0.775, 0.810, 0.832, 0.850,
                                 0.863, 0.872, 0.880, 0.879, 0.840, 0.840],  # 700
                             [0.500, 0.500, 0.620, 0.717, 0.760, 0.792, 0.818, 0.837,
                                 0.850, 0.862, 0.869, 0.850, 0.840, 0.840],  # 750
                             [0.500, 0.500, 0.590, 0.680, 0.738, 0.775, 0.803, 0.823,
                                 0.837, 0.845, 0.852, 0.840, 0.840, 0.840]  # 800
                         ]).T,
                         units=None)

        self.add_subsystem('throttle_to_torque',
                           om.ExecComp('T_unscaled = T_max * throttle',
                                       T_unscaled={'val': np.ones(n), 'units': 'kN*m'},
                                       T_max={'val': torque_vals[-1], 'units': 'kN*m'},
                                       throttle={'val': np.ones(n), 'units': 'unitless'}),
                           promotes=["T_unscaled",
                                     ("throttle", Dynamic.Mission.THROTTLE)])

        self.add_subsystem(name="motor_efficiency",
                           subsys=motor, promotes=["*"])

        # now that we know the efficiency, scale up the torque correctly for the engine size selected
        # Note: This allows the optimizer to optimize the motor size if desired
        self.add_subsystem('scale_motor_torque',
                           om.ExecComp('T = T_unscaled * scale_factor',
                                       T={'val': np.ones(n), 'units': 'kN*m'},
                                       T_unscaled={'val': np.ones(n), 'units': 'kN*m'},
                                       scale_factor={'val': 1, 'units': 'unitless'}),
                           promotes=[("T", Mission.Motor.TORQUE),
                                     "T_unscaled",
                                     ("scale_factor", Aircraft.Engine.SCALE_FACTOR)])
