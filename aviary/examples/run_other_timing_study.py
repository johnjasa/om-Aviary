
from matplotlib.colors import LogNorm
import pandas as pd
import seaborn as sns
import dill
import aviary.api as av
from time import time
import matplotlib.pyplot as plt
import numpy as np

phase_info = {
    "pre_mission": {"include_takeoff": False, "optimize_mass": True},
    "climb_1": {
        "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
        "user_options": {
            "optimize_mach": False,
            "optimize_altitude": False,
            "polynomial_control_order": 1,
            "num_segments": 5,
            "order": 3,
            "solve_for_distance": False,
            "initial_mach": (0.2, "unitless"),
            "final_mach": (0.72, "unitless"),
            "mach_bounds": ((0.18, 0.84), "unitless"),
            "initial_altitude": (0.0, "ft"),
            "final_altitude": (32500.0, "ft"),
            "altitude_bounds": ((0.0, 33000.0), "ft"),
            "throttle_enforcement": "path_constraint",
            "fix_initial": True,
            "constrain_final": False,
            "fix_duration": False,
            "initial_bounds": ((0.0, 0.0), "min"),
            "duration_bounds": ((35.0, 105.0), "min"),
        },
        "initial_guesses": {"time": ([0, 70], "min")},
    },
    "cruise": {
        "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
        "user_options": {
            "optimize_mach": False,
            "optimize_altitude": False,
            "polynomial_control_order": 1,
            "num_segments": 5,
            "order": 3,
            "solve_for_distance": False,
            "initial_mach": (0.72, "unitless"),
            "final_mach": (0.80, "unitless"),
            "mach_bounds": ((0.7, 0.84), "unitless"),
            "initial_altitude": (32500.0, "ft"),
            "final_altitude": (36000.0, "ft"),
            "altitude_bounds": ((32000.0, 36500.0), "ft"),
            "throttle_enforcement": "boundary_constraint",
            "fix_initial": False,
            "constrain_final": False,
            "fix_duration": False,
            "initial_bounds": ((35.0, 105.0), "min"),
            "duration_bounds": ((91.5, 274.5), "min"),
        },
        "initial_guesses": {"time": ([70, 183], "min")},
    },
    "descent_1": {
        "subsystem_options": {"core_aerodynamics": {"method": "computed"}},
        "user_options": {
            "optimize_mach": False,
            "optimize_altitude": False,
            "polynomial_control_order": 1,
            "num_segments": 5,
            "order": 3,
            "solve_for_distance": False,
            "initial_mach": (0.72, "unitless"),
            "final_mach": (0.21, "unitless"),
            "mach_bounds": ((0.19, 0.84), "unitless"),
            "initial_altitude": (36000.0, "ft"),
            "final_altitude": (0.0, "ft"),
            "altitude_bounds": ((0.0, 36500.0), "ft"),
            "throttle_enforcement": "path_constraint",
            "fix_initial": False,
            "constrain_final": True,
            "fix_duration": False,
            "initial_bounds": ((126.5, 379.5), "min"),
            "duration_bounds": ((25.0, 75.0), "min"),
        },
        "initial_guesses": {"time": ([253, 50], "min")},
    },
    "post_mission": {
        "include_landing": False,
        "constrain_range": True,
        "target_range": (2080, "nmi"),
    },
}

times_dict = {
    'approx_totals': {'beginning': [], 'end': []},
    'analytic_totals': {'beginning': [], 'end': []},
}

derivs_dict = {
    'approx_totals': {'beginning': [], 'end': []},
    'analytic_totals': {'beginning': [], 'end': []},
}


def run_problem(max_iter, approx_totals):
    timing_list = []
    aircraft_filename = 'models/test_aircraft/aircraft_for_bench_FwFm.csv'
    make_plots = True

    prob = av.AviaryProblem(av.AnalysisScheme.COLLOCATION)

    # Load aircraft and options data from user
    # Allow for user overrides here
    prob.load_inputs(aircraft_filename, phase_info)

    prob.check_and_preprocess_inputs()
    # Preprocess inputs
    prob.add_pre_mission_systems()

    prob.add_phases()

    prob.add_post_mission_systems()

    # Link phases and variables
    prob.link_phases()

    prob.add_driver("SNOPT", max_iter=max_iter)

    prob.add_design_variables()

    prob.model.add_design_var(av.Aircraft.Engine.SCALED_SLS_THRUST,
                              lower=25.e3, upper=30.e3, units='lbf', ref=28.e3)
    prob.model.add_design_var(av.Aircraft.Wing.ASPECT_RATIO,
                              lower=10., upper=14., ref=12.)

    # Load optimization problem formulation
    # Detail which variables the optimizer can control
    prob.add_objective()

    if approx_totals:
        prob.model.approx_totals(step=1e-6, form='central')

    prob.setup()

    prob.set_initial_guesses()

    prob.run_aviary_problem(make_plots=make_plots)

    for i in range(10):
        s = time()
        totals = prob.compute_totals()

        timing_list.append(time() - s)

    return np.mean(timing_list), totals


recreate_data = False

if recreate_data:
    times_dict['approx_totals']['beginning'], derivs_dict['approx_totals']['beginning'] = run_problem(
        0, True)
    times_dict['analytic_totals']['beginning'], derivs_dict['analytic_totals']['beginning'] = run_problem(
        0, False)
    times_dict['approx_totals']['end'], derivs_dict['approx_totals']['end'] = run_problem(
        100, True)
    times_dict['analytic_totals']['end'], derivs_dict['analytic_totals']['end'] = run_problem(
        100, False)

    import dill

    with open('times_dict.dill', 'wb') as f:
        dill.dump(times_dict, f)

    with open('derivs_dict.dill', 'wb') as f:
        dill.dump(derivs_dict, f)

else:
    with open('times_dict.dill', 'rb') as f:
        times_dict = dill.load(f)

    with open('derivs_dict.dill', 'rb') as f:
        derivs_dict = dill.load(f)


def make_bar_chart():
    # make a matplotlib bar chart comparing the two methods for both beginning and end of optimization

    # Data
    labels = ['Beginning of optimization', 'End of optimization']
    approx_times = [times_dict['approx_totals']
                    ['beginning'], times_dict['approx_totals']['end']]
    analytic_times = [times_dict['analytic_totals']
                      ['beginning'], times_dict['analytic_totals']['end']]

    x = np.arange(len(labels)) * 0.5  # the label locations
    width = 0.2  # the width of the bars

    fig, ax = plt.subplots(figsize=(10, 5))

    # Bars for the approx_totals
    rects1 = ax.bar(x - width/2, approx_times, width, label='Approx Totals')
    # Bars for the analytic_totals
    rects2 = ax.bar(x + width/2, analytic_times, width, label='Analytic Totals')

    # Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_ylabel('Time (s)', rotation=0, labelpad=20)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)

    # Remove spines and y-tick labels to reduce clutter
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.yaxis.set_ticks([])

    # Function to add labels
    def add_labels(rects, label_text):
        for rect in rects:
            height = rect.get_height()
            ax.annotate(f'{height:.3f} seconds',
                        xy=(rect.get_x() + rect.get_width() / 2, height),
                        xytext=(0, 3),  # 3 points vertical offset
                        textcoords="offset points",
                        ha='center', va='bottom')
            ax.annotate(label_text,
                        xy=(rect.get_x() + rect.get_width() / 2, height),
                        xytext=(0, -12),  # 12 points vertical offset inside the bar
                        textcoords="offset points",
                        ha='center', va='bottom', color='white', weight='bold')

    add_labels(rects1, 'Approximated')
    add_labels(rects2, 'Analytic')

    plt.tight_layout()
    plt.show()


# Get the analytic and approximated derivatives for the throttle constraint and engine thrust
def get_derivs(key_pair):
    analytic = derivs_dict['analytic_totals']['beginning'][key_pair]
    approx = derivs_dict['approx_totals']['beginning'][key_pair]
    # compute the relative error
    relative_error = np.abs((analytic - approx) / analytic)
    return np.mean(analytic), np.mean(approx), np.mean(relative_error)


# loop through all key_pairs in the analytic_totals dictionary
for key_pair in derivs_dict['analytic_totals']['beginning']:
    analytic, approx, relative_error = get_derivs(key_pair)
    if not np.isnan(relative_error):
        print('---------------------------------')
        print(key_pair)
        print(
            f'Analytic: {analytic}, Approximated: {approx}, Relative Error: {relative_error}')


# make_bar_chart()


# Data from your example
data = [
    ('mission:constraints:mass_residual', 'mission:design:gross_mass', 3.0531124894770836e-06),
    ('mission:constraints:mass_residual',
     'aircraft:engine:scaled_sls_thrust', 1.11062305502207e-05),
    ('mission:constraints:mass_residual', 'aircraft:wing:aspect_ratio', 8.052837730421727e-09),
    ('traj.climb_1.collocation_constraint.defects:mass',
     'aircraft:engine:scaled_sls_thrust', 38.73709412468275),
    ('traj.climb_1.collocation_constraint.defects:mass',
     'aircraft:wing:aspect_ratio', 1.7071883976275675e-07),
    ('traj.climb_1.collocation_constraint.defects:mass',
     'traj.climb_1.t_duration', 0.32087467780810214),
    ('traj.climb_1.collocation_constraint.defects:distance',
     'traj.climb_1.t_duration', 5.922212395269047e-07),
    ('traj.cruise.collocation_constraint.defects:mass',
     'aircraft:engine:scaled_sls_thrust', 18.707118796725645),
    ('traj.cruise.collocation_constraint.defects:mass',
     'aircraft:wing:aspect_ratio', 1.4623708823312296e-07),
    ('traj.cruise.collocation_constraint.defects:mass',
     'traj.cruise.t_duration', 0.011558781015073157),
    ('traj.cruise.collocation_constraint.defects:distance',
     'traj.cruise.t_duration', 1.190717161870995e-06),
    ('traj.descent_1.collocation_constraint.defects:mass',
     'aircraft:engine:scaled_sls_thrust', 2.5100314034551396),
    ('traj.descent_1.collocation_constraint.defects:mass',
     'aircraft:wing:aspect_ratio', 1.659450740524522e-07),
    ('traj.descent_1.collocation_constraint.defects:mass',
     'traj.descent_1.t_duration', 0.44973251211719034),
    ('traj.descent_1.collocation_constraint.defects:distance',
     'traj.descent_1.t_duration', 4.0518186531267816e-07),
    ('traj.phases.climb_1->path_constraint->throttle',
     'aircraft:engine:scaled_sls_thrust', 1.0),
    ('traj.phases.climb_1->path_constraint->throttle',
     'aircraft:wing:aspect_ratio', 1.6865131568666602e-07),
    ('traj.phases.climb_1->path_constraint->throttle', 'traj.climb_1.t_duration', 1.0),
    ('traj.phases.cruise->initial_boundary_constraint->throttle',
     'aircraft:engine:scaled_sls_thrust', 1.0),
    ('traj.phases.cruise->initial_boundary_constraint->throttle',
     'aircraft:wing:aspect_ratio', 1.4868617602873288e-07),
    ('traj.phases.cruise->initial_boundary_constraint->throttle', 'traj.cruise.t_duration', 1.0),
    ('traj.phases.cruise->final_boundary_constraint->throttle',
     'aircraft:engine:scaled_sls_thrust', 1.0),
    ('traj.phases.cruise->final_boundary_constraint->throttle',
     'aircraft:wing:aspect_ratio', 1.5510946442315895e-07),
    ('traj.phases.cruise->final_boundary_constraint->throttle', 'traj.cruise.t_duration', 1.0),
    ('traj.phases.descent_1->path_constraint->throttle',
     'aircraft:engine:scaled_sls_thrust', 1.0),
    ('traj.phases.descent_1->path_constraint->throttle',
     'aircraft:wing:aspect_ratio', 1.6831198702305132e-07),
    ('traj.phases.descent_1->path_constraint->throttle', 'traj.descent_1.t_duration', 1.0),
]

# Convert to DataFrame for easier plotting
df = pd.DataFrame(data, columns=['of', 'wrt', 'relative_error'])

# Define a mapping for nicer formatted names
formatted_names = {
    'mission:constraints:mass_residual': 'Mass residual',
    'mission:design:gross_mass': 'Gross mass',
    'aircraft:engine:scaled_sls_thrust': 'Scaled engine thrust',
    'aircraft:wing:aspect_ratio': 'Wing aspect ratio',
    'traj.climb_1.collocation_constraint.defects:mass': 'Mass defect constraint (climb phase)',
    'traj.climb_1.collocation_constraint.defects:distance': 'Distance defect constraint (climb phase)',
    'traj.climb_1.t_duration': 'Duration (climb phase)',
    'traj.cruise.collocation_constraint.defects:mass': 'Mass defect constraint (cruise phase)',
    'traj.cruise.collocation_constraint.defects:distance': 'Distance defect constraint (cruise phase)',
    'traj.cruise.t_duration': 'Duration (cruise phase)',
    'traj.descent_1.collocation_constraint.defects:mass': 'Mass defect constraint (descent phase)',
    'traj.descent_1.collocation_constraint.defects:distance': 'Distance defect constraint (descent phase)',
    'traj.descent_1.t_duration': 'Duration (descent phase)',
    'traj.phases.climb_1->path_constraint->throttle': 'Throttle constraint (climb phase)',
    'traj.phases.cruise->initial_boundary_constraint->throttle': 'Throttle constraint (cruise start)',
    'traj.phases.cruise->final_boundary_constraint->throttle': 'Throttle constraint (cruise end)',
    'traj.phases.descent_1->path_constraint->throttle': 'Throttle constraint (descent phase)',
}

# Apply the mapping to the DataFrame
df['of'] = df['of'].map(formatted_names)
df['wrt'] = df['wrt'].map(formatted_names)

# Convert relative error to percentage error
df['relative_error'] = df['relative_error'] * 100

# Pivot the DataFrame to create a matrix
matrix = df.pivot(index='of', columns='wrt', values='relative_error')

# Plot the heatmap
plt.figure(figsize=(14, 10))
ax = sns.heatmap(matrix, cmap='viridis', norm=LogNorm(
    vmin=1e-6, vmax=1e4), cbar_kws={'label': 'Percentage Error (%)'})

# Add text annotations
for i in range(matrix.shape[0]):
    for j in range(matrix.shape[1]):
        value = matrix.iloc[i, j]
        if not np.isnan(value):
            if value > 1:
                color = 'black'
            else:
                color = 'white'

            if value > 0.01:
                ax.text(j + 0.5, i + 0.5, f'{value:.2f}%',
                        ha='center', va='center', color=color)
            else:
                ax.text(j + 0.5, i + 0.5, f'{value:.5f}%',
                        ha='center', va='center', color=color)


# Customize the plot
plt.title('Percentage Error Heatmap of Derivatives')
plt.xlabel('Design Variables')
plt.ylabel('Constraints/Responses')
plt.xticks(rotation=45, ha='right')
plt.yticks(rotation=0)

# Show plot
plt.tight_layout()
plt.savefig('derivatives_heatmap.png')
