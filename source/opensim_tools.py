"""Python wrappers for a subset of the suite of OpenSim tools.

"""
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from abc import ABC, abstractmethod
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
_POINT_ACTUATORS = ["FX", "FY", "FZ"]
_TIMERANGE = [-math.inf, math.inf]


def _get_classname_from_xml(xml: ET.ElementTree) -> str:
    root = xml.getroot()
    return root[0].tag


def _get_tool(settings: str):
    """Parse settings file to find correct OpenSim tool function."""
    xml = ET.parse(settings)
    name = _get_classname_from_xml(xml)
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


def _modify_actuators(model, actuators_in, actuators_out):
    """Adjust the location of specific point actuators in an actuator file to match an input model

    This function assumes that at least one 'hand-of-god' point actuator is available, with a name
    drawn from _TOOL_ACTUATORS = [FX, FY, FZ, MX, MY, MZ], as is OpenSim convention.
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


def _modify_actuators_new(loaded_model: opensim.Model) -> None:
    # Get point actuators
    force_set = loaded_model.updForceSet()
    point_actuators = [
        force for force in force_set if force.getName() in _POINT_ACTUATORS
    ]
    if not point_actuators:
        return

    # Get the body of application
    body_str = point_actuators[0].get_body()

    # Get the CoM of that body in the input model
    body = loaded_model.getBodySet().get(body_str)
    com = body.getMassCenter()

    # Update the application point of each point actuator
    for point_actuator in point_actuators:
        point_actuator.set_point(com)


class _AbstractToolWrapper(ABC):
    """Abstract interface for tools which inherit from opensim.AbtractTool

    A note on the behaviour of optional arguments:
        If timerange = None, the [-inf, inf] is used, resulting in processing of the entire file.
        If grfs = None, the tool is run with no GRFs.
        If load = None, the default load file present in the tool is left as is.
    """

    tool: opensim.AbstractTool
    settings: str

    def __init__(
        self,
        settings: str,
        model_in: str,
        output: str,
        grfs: str = "",
        load: Optional[str] = None,
        timerange: Optional[list] = None,
    ):
        self.settings = settings
        self.initialise_tool()
        self.pre_load(model_in, grfs, load)
        self.load()
        self.modify_actuators()
        self.set_parameters(timerange, output)

    @abstractmethod
    def initialise_tool(self):
        """Abstract method for creating tool from the appropriate OpenSim class."""

    def pre_load(self, model_in: str, grfs: str = "", load: Optional[str] = None):
        """Assigning of variable parameters (model, grfs) before loading."""
        self.tool.setModelFilename(model_in)
        if load is not None:
            self.tool.setExternalLoadsFileName(load)
        loads = self.tool.updExternalLoads()
        loads.setDataFileName(grfs)

    def load(self) -> opensim.Model:
        """Common interface for loading a model & updating model forces."""
        self.tool.loadModel(self.settings)
        model = self.tool.getModel()
        self.tool.updateModelForces(model, self.settings)
        return model

    def modify_actuators(self):
        """Adjust point actuator points of action to match those of the model

        Model must have been loaded before calling this function"""
        # Get point actuators
        force_set = self.tool.getModel().updForceSet()
        point_actuators = [
            force for force in force_set if force.getName() in _POINT_ACTUATORS
        ]
        if not point_actuators:
            return

        # Get the body of application
        body_str = point_actuators[0].get_body()

        # Get the CoM of that body in the input model
        body = self.tool.getModel().getBodySet().get(body_str)
        com = body.getMassCenter()

        # Update the application point of each point actuator
        for point_actuator in point_actuators:
            point_actuator.set_point(com)

    def set_parameters(self, timerange, output):
        """Common interface for setting timerange and output directory."""
        if timerange is None:
            timerange = _TIMERANGE
        self.tool.setInitialTime(timerange[0])
        self.tool.setFinalTime(timerange[1])
        self.tool.setResultsDir(output)

    def run(self) -> bool:
        """Run tool."""
        return self.tool.run()


class _RRAToolWrapper(_AbstractToolWrapper):
    """Concrete RRATool"""

    tool: opensim.RRATool

    def initialise_tool(self):
        self.tool = opensim.RRATool(self.settings)

    def set_kinematics(self, kinematics):
        """Set desired kinematics path"""
        self.tool.setDesiredKinematicsFileName(kinematics)

    def set_adjustment(self, adjust, body, model_out):
        """Set RRA adjustment settings"""
        self.tool.setAdjustCOMToReduceResiduals(adjust)
        if not adjust:
            return
        self.tool.setAdjustedCOMBody(body)
        self.tool.setOutputModelFileName(model_out)


class _AnalyzeToolWrapper(_AbstractToolWrapper):
    """Concrete AnalyzeTool

    If no controls input is provided the tool ignores any controls specified in the settings file.
    """

    tool: opensim.AnalyzeTool

    def initialise_tool(self):
        self.tool = opensim.AnalyzeTool(self.settings)

    def set_kinematics(self, kinematics):
        """Choose between states and motion kinematics based on filename"""
        ext = os.path.splitext(kinematics)
        if ext == ".mot":
            self.tool.setStatesFileName("")
            self.tool.setCoordinatesFileName(kinematics)
        elif ext == ".sto":
            self.tool.setStatesFileName(kinematics)
            self.tool.setCoordinatesFileName("")
        else:
            raise ValueError("Wrong format of input kinematics.")

    def set_controls(self, controls):
        """Add controls to the analysis"""
        self.tool.setControlsFileName(controls)


def run_rra(
    settings: str,
    model_in: str,
    kinematics: str,
    output: str,
    grfs: str = "",
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

    # General AbstractTool behaviour
    rra_tool = _RRAToolWrapper(settings, model_in, output, grfs, load, timerange)

    # RRA-specific settings
    rra_tool.set_kinematics(kinematics)
    rra_tool.set_adjustment(adjust, body, model_out)

    # Run
    return rra_tool.run()


def run_analyze(
    settings: str,
    model_in: str,
    kinematics: str,
    output: str,
    grfs: str = "",
    load: Optional[str] = None,
    controls: str = "",
    timerange: Optional[list] = None,
) -> bool:
    """Run the analyze tool

    Allows easier modification of: input model, input kinematics, input grfs, output directory,
    timerange, and the use of a controls file."""

    # General AbstractTool behaviour
    analyze_tool = _AnalyzeToolWrapper(
        settings, model_in, output, grfs, load, timerange
    )

    # Analyze-specific settings
    analyze_tool.set_kinematics(kinematics)
    analyze_tool.set_controls(controls)

    # Run
    return analyze_tool.run()


def run_rra_old(
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
        _modify_actuators(model_in, actuators_in, actuators_out)
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
