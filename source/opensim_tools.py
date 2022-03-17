"""Python wrappers for a subset of the suite of OpenSim tools."""
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from typing import Optional

import opensim

_IK_FILE = "ik.mot"
_SUPPORTED_TOOLS = [
    "InverseKinematicsTool",
    "RRATool",
    "CMCTool",
    "AnalyzeTool",
    "InverseDynamicsTool",
]
_TIMERANGE = [-math.inf, math.inf]


def _get_tool(settings: str):
    """Parse settings file to find correct OpenSim tool function."""
    xml = ET.parse(settings)
    root = xml.getroot()
    name = root[0].tag
    if name not in _SUPPORTED_TOOLS:
        raise ValueError(f"Tool ({name}) not supported.")
    return getattr(opensim, name)


def run_tool(settings: str) -> bool:
    """Run an OpenSim tool from settings file."""
    tool_function = _get_tool(settings)
    tool = tool_function(settings)
    return tool.run()


def run_ik(
    settings: str,
    model: str,
    markers: str,
    output: str,
    timerange: Optional[list] = None,
) -> bool:
    """Run inverse kinematics.

    Allows easier modification of: input model, input markers, timerange, and output directory
    """

    if timerange is None:
        timerange = _TIMERANGE

    # Load tool from settings file
    tool = opensim.InverseKinematicsTool(settings, False)

    # Set model
    osim = opensim.Model(model)
    osim.initSystem()
    tool.setModel(osim)

    # Set timerange
    tool.setStartTime(timerange[0])
    tool.setEndTime(timerange[1])

    # Set paths
    tool.setMarkerDataFileName(markers)
    tool.setResultsDir(output)
    tool.setOutputMotionFileName(os.path.join(output, _IK_FILE))

    # Run tool
    return tool.run()


def _first_directed_element(element: ET.Element, directions: list) -> ET.Element:
    """Return the first element of an XML tree matching a set of directions."""
    for step in directions:
        new_element = element.find(step)
        if new_element is None:
            raise ValueError("Failed parsing XML file.")
        element = new_element
    return element


def modify_rra_actuators(model, actuators_in, actuators_out):
    """Adjust the location of point actuators in an RRA actuator file to match an input model

    This function assumes that point actuators are applied to only one body. This should hold
    for RRA actuator files by the nature of the algorithm.
    """
    # Load XML & get root element
    xml = ET.parse(actuators_in)
    root = xml.getroot()

    # Get point actuator body as string
    body_element = _first_directed_element(
        root, ["ForceSet", "objects", "PointActuator", "body"]
    )
    body_str = body_element.text
    if body_str is None:
        raise ValueError("Failed parsing XML file.")
    body_str = str.strip(body_str)

    # Get the CoM of that body in the input model
    osim = opensim.Model(model)
    body = osim.getBodySet().get(body_str)
    com = body.getMassCenter()

    # Convert CoM to a printable string
    com_str = str(com[0]) + " " + str(com[1]) + " " + str(com[2])

    # Adjust CoM of actuators in XML
    objects = _first_directed_element(root, ["ForceSet", "objects"])
    for point_actuator in objects.iter("PointActuator"):
        _first_directed_element(point_actuator, ["point"]).text = com_str

    # Print modified XML file
    xml.write(actuators_out)


def run_rra(
    settings: str,
    model_in: str,
    kinematics: str,
    output: str,
    grfs: Optional[str] = None,
    load: Optional[str] = None,
    timerange: Optional[list] = None,
    adjust: bool = False,
    body: str = "torso",
    model_out: Optional[str] = None,
) -> bool:
    """Run RRA

    Allows easier modification of: input model, input kinematics, input grfs, output directory,
    and timerange. Also allows for RRA-based model adjustment.
    """

    if timerange is None:
        timerange = _TIMERANGE

    with tempfile.TemporaryDirectory() as temp_dir:

        tool = opensim.RRATool(settings, False)

        # Modify pelvis COM in actuators file
        actuators_in = tool.getForceSetFiles().get(0)  # What if there are none?
        actuators_out = os.path.join(temp_dir, "actuators.xml")
        modify_rra_actuators(model_in, actuators_in, actuators_out)
        tool.setForceSetFiles(opensim.ArrayStr(actuators_out, 1))  # type: ignore

        # Set model
        tool.setModelFilename(model_in)
        tool.loadModel(settings)
        tool.updateModelForces(tool.getModel(), settings)

        # Set parameters
        tool.setInitialTime(timerange[0])
        tool.setFinalTime(timerange[1])

        # Set paths
        tool.setDesiredKinematicsFileName(kinematics)
        tool.setResultsDir(output)

        # Handle external loads, if present
        if grfs is not None:
            tool.setExternalLoadsFileName(load)
            loads = tool.updExternalLoads()
            loads.setDataFileName(grfs)

        # Handle model adjustment options
        tool.setAdjustCOMToReduceResiduals(adjust)
        if adjust:
            tool.setAdjustedCOMBody(body)
            tool.setOutputModelFileName(model_out)

        # Run tool
        return tool.run()


def main():
    """Boilerplate."""


if __name__ == "__main__":
    main()
