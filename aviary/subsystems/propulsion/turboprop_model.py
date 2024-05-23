import numpy as np
import openmdao.api as om

from aviary.subsystems.propulsion.engine_model import EngineModel
from aviary.subsystems.propulsion.engine_deck import EngineDeck
from aviary.subsystems.propulsion.utils import EngineModelVariables
from aviary.utils.named_values import NamedValues
from aviary.utils.aviary_values import AviaryValues
from aviary.variable_info.variables import Aircraft, Dynamic
from aviary.subsystems.propulsion.propeller_performance import PropellerPerformance
from aviary.subsystems.propulsion.utils import UncorrectData
from aviary.mission.gasp_based.flight_conditions import FlightConditions
from aviary.variable_info.enums import SpeedType


class TurbopropModel(EngineModel):
    """
    EngineModel that combines a model for shaft power generation (default is EngineDeck)
    and a model for propeller performance (default is Hamilton Standard).

    Attributes
    ----------
    name : str ('engine')
        Object label.
    options : AviaryValues (<empty>)
        Inputs and options related to engine model.
    data : NamedVaues (<empty>)
        If using an engine deck, engine performance data (optional). If provided, used
        instead of tabular data file.
    shaft_power_model : SubsystemBuilderBase (<empty>)
        Subsystem builder for the shaft power generating component. If None, an 
        EngineDeck built using provided options is used.
    propeller_model : SubsystemBuilderBase (<empty>)
        Subsystem builder for the propeller. If None, the Hamilton Standard methodology 
        is used to model the propeller.

    Methods
    -------
    build_pre_mission
    build_mission
    build_post_mission
    get_val
    set_val
    update
    """

    def __init__(self, name='turboprop_model', options: AviaryValues = None,
                 data: NamedValues = None, shaft_power_model=None, propeller_model=None):

        # also calls _preprocess_inputs() as part of EngineModel __init__
        super().__init__(name, options)

        self.shaft_power_model = shaft_power_model
        self.propeller_model = propeller_model

        # Initialize turboshaft engine deck. New required variable set w/o thrust
        if shaft_power_model is None:
            self.shaft_power_model = EngineDeck(name=name + '_engine_deck',
                                                options=options,
                                                data=data,
                                                required_variables={EngineModelVariables.ALTITUDE,
                                                                    EngineModelVariables.MACH,
                                                                    EngineModelVariables.THROTTLE})
            # Manually check if shaft horsepower was provided (can't flag as required variable since either version
            # is acceptable)
            if not self.shaft_power_model.use_shaft_power:
                # repurpose custom error message code from EngineDeck
                # custom error messages depending on data type
                if self.shaft_power_model.read_from_file:
                    message = f'<{self.shaft_power_model.get_val(Aircraft.Engine.DATA_FILE)}>'
                else:
                    message = f'EngineDeck for <{self.name}>'
                raise UserWarning(
                    f'No shaft horsepower variable was provided in {message}')

    # BUG if using both custom subsystems that happen to share a kwarg but need different values, this breaks
    def build_pre_mission(self, aviary_inputs, **kwargs) -> om.Group:
        shp_model = self.shaft_power_model
        propeller_model = self.propeller_model
        turboprop_group = om.Group()
        # TODO engine scaling for turboshafts requires EngineSizing to be refactored to
        # accept target scaling variable as an option, skipping for now
        if type(shp_model) is not EngineDeck:
            shp_model_pre_mission = shp_model.build_pre_mission(aviary_inputs, **kwargs)
            if shp_model_pre_mission is not None:
                turboprop_group.add_subsystem(shp_model_pre_mission.name,
                                              subsys=shp_model_pre_mission,
                                              promotes=['*'])

        if propeller_model is not None and propeller_model != 'hamilton_standard':
            propeller_model_pre_mission = propeller_model.build_pre_mission(
                aviary_inputs, **kwargs)
            if propeller_model_pre_mission is not None:
                turboprop_group.add_subsystem(propeller_model_pre_mission.name,
                                              subsys=propeller_model_pre_mission,
                                              promotes=['*'])

        return turboprop_group

    def build_mission(self, num_nodes, aviary_inputs, **kwargs):
        shp_model = self.shaft_power_model
        propeller_model = self.propeller_model
        turboprop_group = om.Group()
        max_thrust_group = om.Group()

        try:
            shp_kwargs = kwargs[shp_model.name]
        except (AttributeError, KeyError):
            shp_kwargs = {}
        shp_model_mission = shp_model.build_mission(
            num_nodes, aviary_inputs, **shp_kwargs)
        if shp_model_mission is not None:
            turboprop_group.add_subsystem(shp_model.name,
                                          subsys=shp_model_mission,
                                          promotes_inputs=['*'],
                                          promotes_outputs=['*',
                                                            (Dynamic.Mission.THRUST,
                                                             'turboshaft_thrust'),
                                                            (Dynamic.Mission.THRUST_MAX,
                                                             'turboshaft_thrust_max'),
                                                            (Dynamic.Mission.SHAFT_POWER,
                                                             'dummy_shp'),
                                                            (Dynamic.Mission.SHAFT_POWER_MAX, 'dummy_shp_max')])

        # ensure uncorrected shaft horsepower is avaliable
        # TODO also make sure corrected is avaliable
        # TODO see if this can be done for non-EngineDecks
        if isinstance(shp_model, EngineDeck):
            if EngineModelVariables.SHAFT_POWER_CORRECTED in shp_model.engine_variables \
               and EngineModelVariables.SHAFT_POWER not in shp_model.engine_variables:
                turboprop_group.add_subsystem('uncorrect_shaft_power',
                                              subsys=UncorrectData(num_nodes=num_nodes,
                                                                   aviary_options=self.options),
                                              promotes_inputs=[('corrected_data', Dynamic.Mission.SHAFT_POWER_CORRECTED),
                                                               Dynamic.Mission.TEMPERATURE,
                                                               Dynamic.Mission.STATIC_PRESSURE,
                                                               Dynamic.Mission.MACH],
                                              promotes_outputs=[('uncorrected_data', Dynamic.Mission.SHAFT_POWER)]),

                turboprop_group.add_subsystem('uncorrect_shaft_power_max',
                                              subsys=UncorrectData(num_nodes=num_nodes,
                                                                   aviary_options=self.options),
                                              promotes_inputs=[('corrected_data', Dynamic.Mission.SHAFT_POWER_CORRECTED_MAX),
                                                               Dynamic.Mission.TEMPERATURE,
                                                               Dynamic.Mission.STATIC_PRESSURE,
                                                               Dynamic.Mission.MACH],
                                              promotes_outputs=[('uncorrected_data', Dynamic.Mission.SHAFT_POWER_MAX)]),

        try:
            propeller_kwargs = kwargs[propeller_model.name]
        except (AttributeError, KeyError):
            propeller_kwargs = {}
        if propeller_model is not None and propeller_model != 'hamilton_standard':

            propeller_model_mission = propeller_model.build_mission(num_nodes, self.options,
                                                                    **propeller_kwargs)
            if propeller_model_mission is not None:
                turboprop_group.add_subsystem(propeller_model.name,
                                              subsys=propeller_model_mission,
                                              promotes_inputs=['*'],
                                              promotes_outputs=['*',
                                                                (Dynamic.Mission.THRUST,
                                                                 'propeller_thrust')])

                propeller_model_mission_max = propeller_model.build_mission(num_nodes,
                                                                            self.options, **kwargs)
                max_thrust_group.add_subsystem(propeller_model.name + '_max',
                                               subsys=propeller_model_mission_max,
                                               promotes_inputs=['*',
                                                                (Dynamic.Mission.SHAFT_POWER,
                                                                 Dynamic.Mission.SHAFT_POWER_MAX)],
                                               promotes_outputs=[(Dynamic.Mission.THRUST,
                                                                 'propeller_thrust_max')])

        else:  # use the Hamilton Standard model
            # only promote top-level inputs to avoid conflicts with max group
            prop_inputs = [Dynamic.Mission.TEMPERATURE,
                           Dynamic.Mission.MACH,
                           Aircraft.Engine.PROPELLER_TIP_SPEED_MAX,
                           Dynamic.Mission.DENSITY,
                           Dynamic.Mission.VELOCITY,
                           Aircraft.Engine.PROPELLER_DIAMETER,
                           Aircraft.Engine.PROPELLER_ACTIVITY_FACTOR,
                           Aircraft.Engine.PROPELLER_INTEGRATED_LIFT_COEFFICIENT,
                           Aircraft.Nacelle.AVG_DIAMETER,
                           Dynamic.Mission.SPEED_OF_SOUND]
            try:
                propeller_kwargs = kwargs['hamilton_standard']
            except KeyError:
                propeller_kwargs = {}

            turboprop_group.add_subsystem('propeller_model',
                                          PropellerPerformance(aviary_options=self.options,
                                                               num_nodes=num_nodes,
                                                               **propeller_kwargs),
                                          promotes_inputs=[Dynamic.Mission.SHAFT_POWER,
                                                           *prop_inputs],
                                          promotes_outputs=['*',
                                                            (Dynamic.Mission.THRUST,
                                                             'propeller_thrust')])

            max_thrust_group.add_subsystem('propeller_model_max',
                                           PropellerPerformance(aviary_options=self.options,
                                                                num_nodes=num_nodes,
                                                                **propeller_kwargs),
                                           promotes_inputs=[*prop_inputs,
                                                            (Dynamic.Mission.SHAFT_POWER,
                                                             Dynamic.Mission.SHAFT_POWER_MAX)],
                                           promotes_outputs=[(Dynamic.Mission.THRUST,
                                                              'propeller_thrust_max')])

        thrust_adder = om.ExecComp('turboprop_thrust=turboshaft_thrust+propeller_thrust',
                                   turboprop_thrust={'val': np.zeros(num_nodes),
                                                     'units': 'lbf'},
                                   turboshaft_thrust={'val': np.zeros(num_nodes),
                                                      'units': 'lbf'},
                                   propeller_thrust={'val': np.zeros(num_nodes),
                                                     'units': 'lbf'})

        max_thrust_adder = om.ExecComp('turboprop_thrust=turboshaft_thrust_max+propeller_thrust_max',
                                       turboprop_thrust={'val': np.zeros(num_nodes),
                                                         'units': 'lbf'},
                                       turboshaft_thrust_max={'val': np.zeros(num_nodes),
                                                              'units': 'lbf'},
                                       propeller_thrust_max={'val': np.zeros(num_nodes),
                                                             'units': 'lbf'})

        turboprop_group.add_subsystem('thrust_adder',
                                      subsys=thrust_adder,
                                      promotes_inputs=['*'],
                                      promotes_outputs=[('turboprop_thrust',
                                                         Dynamic.Mission.THRUST)])

        max_thrust_group.add_subsystem('max_thrust_adder',
                                       subsys=max_thrust_adder,
                                       promotes_inputs=['*'],
                                       promotes_outputs=[('turboprop_thrust',
                                                          Dynamic.Mission.THRUST_MAX)])

        turboprop_group.add_subsystem('turboprop_max_group', max_thrust_group,
                                      promotes_inputs=['*'],
                                      promotes_outputs=[Dynamic.Mission.THRUST_MAX])

        return turboprop_group

    def build_post_mission(self, aviary_inputs, **kwargs):
        shp_model = self.shaft_power_model
        propeller_model = self.propeller_model
        turboprop_group = om.Group()
        if type(shp_model) is not EngineDeck:
            shp_model_post_mission = shp_model.build_post_mission(
                aviary_inputs, **kwargs)
            if shp_model_post_mission is not None:
                turboprop_group.add_subsystem(shp_model_post_mission.name,
                                              subsys=shp_model_post_mission,
                                              aviary_options=aviary_inputs,)

        if self.propeller_model is not None:
            propeller_model_post_mission = propeller_model.build_mission(
                aviary_inputs, **kwargs)
            if propeller_model_post_mission is not None:
                turboprop_group.add_subsystem(propeller_model_post_mission.name,
                                              subsys=propeller_model_post_mission,
                                              aviary_options=aviary_inputs,)

        return turboprop_group
