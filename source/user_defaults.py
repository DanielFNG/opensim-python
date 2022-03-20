"""A set of constants defining default for use with opensim_tools module

Each of these default settings can be overridden via function arguments in the public
run_method functions, or modified in this file for convenience if working with a
similar set of default settings over many files.
"""
import math

IK_SETTINGS = None
ID_SETTINGS = None
RRA_SETTINGS = None
CMC_SETTINGS = None
ANALYZE_SETTINGS = None
FD_SETTINGS = None
ADJUSTMENT_BODY = "torso"
MODEL_OUT = "model_adjusted.osim"
POINT_ACTUATORS = ("FX", "FY", "FZ")
TIMERANGE = (-math.inf, math.inf)
IK_FILENAME = "ik.mot"
ID_FILENAME = "id.sto"
