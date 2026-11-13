"""
Nightshift front suspension + steering geometry -- PRECOMPUTED version (no SciPy needed).

These numbers were produced by running the original solver version of this file
(the one that uses scipy.optimize) once. The model only reads the final results,
so it runs in CQ-editor without SciPy.

To change the suspension design targets (KPI, caster, scrub, arm pivots...),
edit and re-run the original solver version, then paste the new results here.
"""
import numpy as np

IN = 25.4
TIRE_D = 32 * IN            # 32 x 10-15
TIRE_W = 10 * IN
R = TIRE_D / 2
HALF_TRACK = 64 * IN / 2
WHEELBASE = 2400 + 14 * IN  # 108.5 in
H = 16 * IN                 # ground to the bottom of the frame
KPI = 10.0
CASTER = 7.0
SCRUB = 30.0

# Tuned hardpoints, mm, left front corner (x forward from front axle, y outboard, z up from ground)
_HP = {
    "LBJ": np.array([15.34807, 733.181588, 281.4]),
    "UBJ": np.array([-15.34807, 689.099842, 531.4]),
    "PLf": np.array([190.0, 200.0, 406.4]),
    "PLr": np.array([-190.0, 200.0, 406.4]),
    "PUf": np.array([134.65193, 358.0, 579.37328]),
    "PUr": np.array([-165.34807, 358.0, 579.37328]),
    "T": np.array([110.0, 736.117933, 425.0]),
    "E": np.array([50.0, 338.545435, 497.837658]),
    "C": np.array([0.0, 812.8, 406.4]),
}
_HP.update({'ic': (-1987.2, 919.170895), 'roll_center': 266.82218, 'lower_angle': 13.194234, 'upper_angle': 8.244252})

# Front-end report (camber, bump steer, Ackermann, turning radius...)
_REP = {'tire': '32x10-15', 'ground_clearance_in': 16.0, 'kpi_deg': 10.0, 'caster_deg': 7.0, 'scrub_mm': 30.0, 'trail_mm': 49.899646, 'roll_center_mm': 266.82218, 'lower_arm_deg': 13.194234, 'upper_arm_deg': 8.244252, 'lower_arm_len': 547.638207, 'upper_arm_len': 334.557232, 'bump_steer_deg': 0.097254, 'camber_at_bump100': -3.066535, 'camber_at_droop100': 0.714838, 'track_change_bump100': 28.629798, 'track_change_droop100': -36.949662, 'rack_travel_mm': 54.938878, 'rack_len_mm': 677.09087, 'steer_inner': 35.0, 'steer_outer': 28.767855, 'ackermann_pct': 74.2586, 'turn_radius_ft': 18.786283, 'table': [(-150, -0.470447, -0.278673, -57.734682), (-125, 0.322124, -0.021259, -47.29478), (-100, 0.714838, 0.064437, -36.949662), (-75, 0.826117, 0.075955, -26.962813), (-50, 0.720779, 0.056259, -17.449101), (-25, 0.437406, 0.026965, -8.456528), (0, -0.0, 0.0, 0.0), (25, -0.576342, -0.017569, 7.922933), (50, -1.2824, -0.021299, 15.324044), (75, -2.113107, -0.008241, 22.220014), (100, -3.066535, 0.023669, 28.629798), (125, -4.143296, 0.075951, 34.572898), (150, -5.346201, 0.14979, 40.068052)]}


def final():
    """Same return as the solver version: (hardpoints, report)."""
    hp = {k: (v.copy() if isinstance(v, np.ndarray) else v) for k, v in _HP.items()}
    return hp, dict(_REP)
