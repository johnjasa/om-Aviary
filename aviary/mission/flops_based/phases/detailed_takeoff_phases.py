'''
Define utilities for building detailed takeoff phases and the typical takeoff trajectory.

Classes
-------
TakeoffBrakeReleaseToDecisionSpeed : a phase builder for the first phase of takeoff, from
brake release to decision speed, the maximum speed at which takeoff can be safely brought
to full stop using zero thrust while braking

TakeoffDecisionSpeedToRotate : a phase builder for the second phase of takeoff, from
decision speed to rotation

TakeoffDecisionSpeedBrakeDelay : a phase builder for the second phase of aborted takeoff,
from decision speed to brake application

TakeoffRotateToLiftoff : a phase builder for the third phase of takeoff, from rotation to
liftoff

TakeoffLiftoffToObstacle : a phase builder for the fourth phase of takeoff, from liftoff
to clearing the required obstacle

TakeoffObstacleToMicP2 : a phase builder for the fifth phase of takeoff, from
clearing the required obstacle to the P2 mic lication; this phase is required for
acoustic calculations

TakeoffMicP2ToEngineCutback : a phase builder for the sixth phase of takeoff, from the
P2 mic location to engine cutback; this phase is required for acoustic calculations

TakeoffEngineCutback : a phase builder for the seventh phase of takeoff, from
start to finish of engine cutback; this phase is required for acoustic calculations

TakeoffEngineCutbackToMicP1 : a phase builder for the eighth phase of takeoff, from
engine cutback to the P1 mic lication; this phase is required for acoustic calculations

TakeoffMicP1ToClimb : a phase builder for the ninth phase of takeoff, from
P1 mic location to climb; this phase is required for acoustic calculations

TakeoffBrakeToAbort : a phase builder for the last phase of aborted takeoff, from brake
application to full stop

TakeoffTrajectory : a trajectory builder for detailed takeoff
'''
import dymos as dm
import openmdao.api as om

from aviary.mission.flops_based.ode.takeoff_ode import TakeoffODE
from aviary.mission.phase_builder_base import PhaseBuilderBase
from aviary.mission.initial_guess_builders import InitialGuessControl, InitialGuessParameter, InitialGuessState, InitialGuessTime
from aviary.utils.aviary_values import AviaryValues
from aviary.variable_info.functions import setup_trajectory_params
from aviary.variable_info.variables import Dynamic, Mission


def _init_initial_guess_meta_data(cls: PhaseBuilderBase):
    '''
    Create default initial guess meta data preset with common items.
    '''
    cls._initial_guesses_meta_data_ = {}

    cls._add_initial_guess_meta_data(
        InitialGuessTime(),
        desc='initial guess for initial time and duration specified as a tuple')

    cls._add_initial_guess_meta_data(
        InitialGuessState('distance'),
        desc='initial guess for horizontal distance traveled')

    cls._add_initial_guess_meta_data(
        InitialGuessState('velocity'),
        desc='initial guess for speed')

    cls._add_initial_guess_meta_data(
        InitialGuessState('mass'),
        desc='initial guess for mass')

    cls._add_initial_guess_meta_data(
        InitialGuessControl('throttle'),
        desc='initial guess for throttle')

    return cls


@_init_initial_guess_meta_data
class TakeoffPhase(PhaseBuilderBase):
    __slots__ = ('phase_type')

    # region : derived type customization points
    _meta_data_ = {}

    default_name = 'takeoff_phase'

    default_ode_class = TakeoffODE
    # endregion : derived type customization points

    def __init__(
        self, name=None, subsystem_options=None, user_options=None, initial_guesses=None,
        ode_class=None, transcription=None, core_subsystems=None, phase_type=None,
    ):
        super().__init__(
            name=name, subsystem_options=subsystem_options, user_options=user_options,
            initial_guesses=initial_guesses, ode_class=ode_class, transcription=transcription,
            core_subsystems=core_subsystems, is_analytic_phase=False,
        )
        self.phase_type = phase_type

    def build_phase(self, aviary_options=None):
        phase_type = self.phase_type

        phase: dm.Phase = super().build_phase(aviary_options)

        user_options: AviaryValues = self.user_options

        max_duration, units = user_options.get_item('max_duration')
        duration_ref = user_options.get_val('duration_ref', units)

        if not phase_type == '1':
            fix_initial = False
            initial_ref = user_options.get_val('initial_ref', units)
            phase.set_time_options(
                fix_initial=fix_initial, duration_bounds=(1, max_duration),
                initial_bounds=(1, initial_ref),
                duration_ref=duration_ref, initial_ref=initial_ref,
                units=units)
        elif phase_type in ['2b', '7']:
            phase.set_time_options(
                fix_initial=False, fix_duration=True,
                initial_bounds=(1, initial_ref),
                initial_ref=initial_ref,
                units=units)
        else:
            fix_initial = True
            phase.set_time_options(
                fix_initial=fix_initial, duration_bounds=(1, max_duration),
                duration_ref=duration_ref, units=units)

        distance_max, units = user_options.get_item('distance_max')

        phase.add_state(
            Dynamic.Mission.DISTANCE, fix_initial=fix_initial, lower=0, ref=distance_max,
            defect_ref=distance_max, units=units, upper=distance_max,
            rate_source=Dynamic.Mission.DISTANCE_RATE)

        max_velocity, units = user_options.get_item('max_velocity')

        if phase_type == '10':
            fix_final = True
        else:
            fix_final = False

        phase.add_state(
            Dynamic.Mission.VELOCITY, fix_initial=fix_initial, lower=0, ref=max_velocity,
            defect_ref=max_velocity, units=units, upper=max_velocity, fix_final=fix_final,
            rate_source=Dynamic.Mission.VELOCITY_RATE)

        if phase_type in ['4', '5', '6', '7', '8', '9']:
            flight_path_angle_ref, units = user_options.get_item('flight_path_angle_ref')

            if phase_type == '4':
                fix_initial_fpa = True
            else:
                fix_initial_fpa = False

            phase.add_state(
                Dynamic.Mission.FLIGHT_PATH_ANGLE, fix_initial=fix_initial_fpa, lower=0,
                ref=flight_path_angle_ref,
                defect_ref=flight_path_angle_ref, units=units,
                rate_source=Dynamic.Mission.FLIGHT_PATH_ANGLE_RATE)

            altitude_ref, units = user_options.get_item('altitude_ref')

            phase.add_state(
                Dynamic.Mission.ALTITUDE, fix_initial=fix_initial_fpa, lower=0, ref=altitude_ref,
                defect_ref=altitude_ref, units=units, upper=altitude_ref,
                rate_source=Dynamic.Mission.ALTITUDE_RATE)

        phase.add_state(
            Dynamic.Mission.MASS, fix_initial=fix_initial, fix_final=False,
            lower=0.0, upper=1e9, ref=5e4, units='kg',
            rate_source=Dynamic.Mission.FUEL_FLOW_RATE_NEGATIVE_TOTAL,
            targets=Dynamic.Mission.MASS,
        )

        # TODO: Energy phase places this under an if num_engines > 0.
        phase.add_control(
            Dynamic.Mission.THROTTLE,
            units='unitless',
            opt=False
        )

        if phase_type in ['3']:
            max_angle_of_attack, units = user_options.get_item('max_angle_of_attack')
            phase.add_polynomial_control(
                'angle_of_attack', opt=True, units=units, order=1,
                lower=0, upper=max_angle_of_attack,
                ref=max_angle_of_attack)

        elif phase_type in ['4', '5', '6', '7', '8', '9']:
            lower_angle_of_attack, units = user_options.get_item('lower_angle_of_attack')
            upper_angle_of_attack = user_options.get_val('upper_angle_of_attack', units)
            angle_of_attack_ref = user_options.get_val('angle_of_attack_ref', units)

            phase.add_control(
                'angle_of_attack', opt=True, units=units,
                lower=lower_angle_of_attack, upper=upper_angle_of_attack,
                ref=angle_of_attack_ref)

        else:
            phase.add_parameter('angle_of_attack', val=0.0, opt=False, units='deg')

        if phase_type in ['2a', '2b', '3', '7', '8', '9']:
            phase.add_boundary_constraint(
                'v_over_v_stall', loc='final', lower=1.2, ref=1.2)
            phase.add_timeseries_output(
                'v_over_v_stall', output_name='v_over_v_stall', units='unitless'
            )

        if phase_type == '4':
            obstacle_height, units = aviary_options.get_item(
                Mission.Takeoff.OBSTACLE_HEIGHT)

            if obstacle_height is None:
                raise TypeError(
                    f'missing required aviary_option: {Mission.Takeoff.OBSTACLE_HEIGHT}')

            airport_altitude = aviary_options.get_val(
                Mission.Takeoff.AIRPORT_ALTITUDE, units)

            h = obstacle_height + airport_altitude

            phase.add_boundary_constraint(
                Dynamic.Mission.ALTITUDE, loc='final', equals=h, ref=h, units=units, linear=True)

            phase.add_path_constraint(
                'v_over_v_stall', lower=1.25, ref=2.0)

            phase.add_boundary_constraint('eoms.forces_vertical', loc='initial', equals=0,
                                          ref=100000)

        if phase_type in ['5', '6']:
            phase.add_timeseries_output(
                Dynamic.Mission.THRUST_TOTAL,
                output_name=Dynamic.Mission.THRUST_TOTAL, units='lbf'
            )

            if phase_type == '5':
                final_altitude, units = user_options.get_item('mic_altitude')

                airport_altitude = aviary_options.get_val(
                    Mission.Takeoff.AIRPORT_ALTITUDE, units)

                h = final_altitude + airport_altitude

                phase.add_boundary_constraint(
                    Dynamic.Mission.ALTITUDE, loc='final', equals=h, ref=h, units=units, linear=True)

            if phase_type == '6':
                # start engine cutback phase at this range, where this phase ends
                # TODO: what is the difference between distance_max and final_range?
                #    - should final_range replace distance_max?
                #    - is there any reason to support both in this phase?
                final_range, units = user_options.get_item('final_range')

                phase.add_boundary_constraint(
                    Dynamic.Mission.DISTANCE, loc='final', equals=final_range, ref=final_range,
                    units=units, linear=True)

            phase.add_boundary_constraint(
                'v_over_v_stall', loc='final', lower=1.25, ref=1.25)

        if phase_type in ['8', '9']:
            mic_range, units = user_options.get_item('mic_range')

            phase.add_boundary_constraint(
                Dynamic.Mission.DISTANCE, loc='final', equals=mic_range, ref=mic_range,
                units=units, linear=True)

        phase.add_timeseries_output(
            Dynamic.Mission.THRUST_TOTAL,
            output_name=Dynamic.Mission.THRUST_TOTAL, units='lbf'
        )

        phase.add_timeseries_output(
            Dynamic.Mission.DRAG, output_name=Dynamic.Mission.DRAG, units='lbf'
        )

        return phase

    def make_default_transcription(self):
        '''
        Return a transcription object to be used by default in build_phase.
        '''
        num_segments = 3
        if self.phase_type in ['6', '9']:
            num_segments = 7
        elif self.phase_type in ['5']:
            num_segments = 9
        transcription = dm.Radau(num_segments=num_segments, order=3, compressed=True)

        return transcription

    def _extra_ode_init_kwargs(self):
        """
        Return extra kwargs required for initializing the ODE.
        """
        if self.phase_type in ['4', '5', '6', '7', '8', '9']:
            climbing = True
        else:
            climbing = False

        if self.phase_type == '10':
            friction_key = Mission.Takeoff.BRAKING_FRICTION_COEFFICIENT
        else:
            friction_key = Mission.Takeoff.ROLLING_FRICTION_COEFFICIENT

        return {
            'climbing': climbing,
            'friction_key': friction_key}


TakeoffPhase._add_meta_data('max_duration', val=1000.0, units='s')

TakeoffPhase._add_meta_data('duration_ref', val=10.0, units='s')

TakeoffPhase._add_meta_data('initial_ref', val=10.0, units='s')

TakeoffPhase._add_meta_data('distance_max', val=1000.0, units='ft')

TakeoffPhase._add_meta_data(
    'max_velocity', val=100.0, units='ft/s')

TakeoffPhase._add_initial_guess_meta_data(InitialGuessParameter('angle_of_attack'))

TakeoffPhase._add_meta_data('max_angle_of_attack', val=10.0, units='deg')

TakeoffPhase._add_meta_data('altitude_ref', val=1., units='ft')

TakeoffPhase._add_meta_data('flight_path_angle_ref', val=5., units='deg')

TakeoffPhase._add_meta_data('lower_angle_of_attack', val=-10., units='deg')

TakeoffPhase._add_meta_data('upper_angle_of_attack', val=15., units='deg')

TakeoffPhase._add_meta_data('angle_of_attack_ref', val=10., units='deg')

TakeoffPhase._add_initial_guess_meta_data(InitialGuessState('altitude'))

TakeoffPhase._add_initial_guess_meta_data(
    InitialGuessState(Dynamic.Mission.FLIGHT_PATH_ANGLE))

TakeoffPhase._add_meta_data('mic_altitude', val=1.0, units='ft')

TakeoffPhase._add_meta_data('final_range', val=1000., units='ft')

TakeoffPhase._add_meta_data('mic_range', val=1000.0, units='ft')


class TakeoffTrajectory:
    '''
    Define a trajectory builder for detailed takeoff.

    Identify, collect, and call the necessary phase builders to create a typical takeoff
    trajectory.
    '''
    default_name = 'detailed_takeoff'

    def __init__(self, name=None):
        if name is None:
            name = self.default_name

        self.name = name

        self._brake_release_to_decision_speed = None
        self._decision_speed_to_rotate = None
        self._rotate_to_liftoff = None
        self._liftoff_to_obstacle = None
        self._obstacle_to_mic_p2 = None
        self._mic_p2_to_engine_cutback = None
        self._engine_cutback = None
        self._engine_cutback_to_mic_p1 = None
        self._mic_p1_to_climb = None
        self._decision_speed_to_brake = None
        self._brake_to_abort = None

        self._phases = {}
        self._traj = None

    def get_phase_names(self):
        '''
        Return a list of base names for available phases.
        '''
        keys = list(self._phases)

        return keys

    def get_phase(self, key) -> dm.Phase:
        '''
        Return the phase associated with the specified base name.

        Raises
        ------
        KeyError
            if the specified base name is not found
        '''
        return self._phases[key][0]

    def build_trajectory(
        self, *, aviary_options: AviaryValues, model: om.Group = None,
        traj: dm.Trajectory = None
    ) -> dm.Trajectory:
        '''
        Return a new trajectory for detailed takeoff analysis.

        Call only after assigning phase builders for required phases.

        Parameters
        ----------
        aviary_options : AviaryValues
            collection of Aircraft/Mission specific options

        model : openmdao.api.Group (None)
            the model handling trajectory parameter setup; if `None`, trajectory
            parameter setup will not be handled

        traj : dymos.Trajectory (None)
            the trajectory to update; if `None`, a new trajetory will be updated and
            returned

        Returns
        -------
        the updated trajectory; if the specified trajectory is `None`, a new trajectory
        will be updated and returned

        Notes
        -----
        Do not modify this object or any of its referenced data between the call to
        `build_trajectory()` and the call to `apply_initial_guesses()`, or the behavior
        is undefined, no diagnostic required.
        '''
        if traj is None:
            traj = dm.Trajectory()

        self._traj = traj

        self._add_phases(aviary_options)
        self._link_phases()

        if model is not None:
            phase_names = self.get_phase_names()

            setup_trajectory_params(model, traj, aviary_options, phase_names)

        return traj

    def apply_initial_guesses(self, prob: om.Problem, traj_name):
        '''
        Call `prob.set_val()` for states/parameters/etc. for each phase in this
        trajectory.

        Call only after `build_trajectory()` and `prob.setup()`.

        Returns
        -------
        not_applied : dict[str, list[str]]
            for any phase with missing initial guesses that cannot be applied, a list of
            those missing initial guesses; if a given phase has no missing initial
            guesses, the returned mapping will not contain the name of that phase
        '''
        not_applied = {}
        phase_builder: PhaseBuilderBase = None  # type hint

        for (phase, phase_builder) in self._phases.values():
            tmp = phase_builder.apply_initial_guesses(prob, traj_name, phase)

            if tmp:
                not_applied[phase_builder.name] = tmp

        return not_applied

    def _add_phases(self, aviary_options: AviaryValues):
        self._phases = {}

        self._add_phase(
            self._brake_release_to_decision_speed, aviary_options)

        self._add_phase(
            self._decision_speed_to_rotate, aviary_options)

        self._add_phase(
            self._rotate_to_liftoff, aviary_options)

        self._add_phase(
            self._liftoff_to_obstacle, aviary_options)

        obstacle_to_mic_p2 = self._obstacle_to_mic_p2

        if obstacle_to_mic_p2 is not None:
            self._add_phase(
                obstacle_to_mic_p2, aviary_options)

            self._add_phase(
                self._mic_p2_to_engine_cutback, aviary_options)

            self._add_phase(
                self._engine_cutback, aviary_options)

            self._add_phase(
                self._engine_cutback_to_mic_p1, aviary_options)

            self._add_phase(
                self._mic_p1_to_climb, aviary_options)

        decision_speed_to_brake = self._decision_speed_to_brake

        if decision_speed_to_brake is not None:
            self._add_phase(
                decision_speed_to_brake, aviary_options)

            self._add_phase(
                self._brake_to_abort, aviary_options)

    def _link_phases(self):
        traj: dm.Trajectory = self._traj

        brake_release_name = self._brake_release_to_decision_speed.name
        decision_speed_name = self._decision_speed_to_rotate.name

        basic_vars = ['time', 'distance', 'velocity', 'mass']

        traj.link_phases([brake_release_name, decision_speed_name], vars=basic_vars)

        rotate_name = self._rotate_to_liftoff.name

        ext_vars = basic_vars + ['angle_of_attack']

        liftoff_name = self._liftoff_to_obstacle.name

        traj.link_phases([decision_speed_name, rotate_name, liftoff_name], vars=ext_vars)

        obstacle_to_mic_p2 = self._obstacle_to_mic_p2

        if obstacle_to_mic_p2 is not None:
            obstacle_to_mic_p2_name = obstacle_to_mic_p2.name
            mic_p2_to_engine_cutback_name = self._mic_p2_to_engine_cutback.name
            engine_cutback_name = self._engine_cutback.name
            engine_cutback_to_mic_p1_name = self._engine_cutback_to_mic_p1.name
            mic_p1_to_climb_name = self._mic_p1_to_climb.name

            acoustics_vars = ext_vars + [Dynamic.Mission.FLIGHT_PATH_ANGLE, 'altitude']

            traj.link_phases(
                [liftoff_name, obstacle_to_mic_p2_name, mic_p2_to_engine_cutback_name,
                    engine_cutback_name, engine_cutback_to_mic_p1_name, mic_p1_to_climb_name],
                vars=acoustics_vars)

        decision_speed_to_brake = self._decision_speed_to_brake

        if decision_speed_to_brake is not None:
            brake_name = decision_speed_to_brake.name
            abort_name = self._brake_to_abort.name

            traj.link_phases([brake_release_name, brake_name,
                             abort_name], vars=basic_vars)

            traj.add_linkage_constraint(
                phase_a=abort_name, var_a='distance', loc_a='final',
                phase_b=liftoff_name, var_b='distance', loc_b='final',
                ref=self._balanced_field_ref)

    def _add_phase(self, phase_builder: PhaseBuilderBase, aviary_options: AviaryValues):
        name = phase_builder.name
        phase = phase_builder.build_phase(aviary_options)

        self._traj.add_phase(name, phase)

        self._phases[name] = (phase, phase_builder)
