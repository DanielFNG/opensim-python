"""Python wrappers for a subset of the suite of OpenSim tools."""
import os
import xml.etree.ElementTree as ET
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import opensim

import user_defaults

_SUPPORTED_TOOLS = [
    "InverseKinematicsTool",
    "RRATool",
    "CMCTool",
    "AnalyzeTool",
    "InverseDynamicsTool",
]


def run_tool_from_settings(settings: str) -> bool:
    """Run an OpenSim tool from a supplied settings file.

    Returns boolean which indicates tool success.

    Requires args:
        settings: Path to an XML setting file for the desired OpenSim tool.
    """
    tool_function = _get_tool_from_settings(settings)
    tool = tool_function(settings)
    return tool.run()


def run_ik(model: str, markers: str, output: str, settings="", **kwargs) -> bool:
    """Run the OpenSim Inverse Kinematics tool.

    Returns boolean which indicates tool success.

    Required args:
        model: Path to input OpenSim model.
        markers: Path to input markers (.trc) file.
        output: Path to (existing) desired results directory.
        settings: Path to an XML setting file for the OpenSim IK Tool. This can be
            specified manually, or alternatively loaded from user_settings.py

    Optional args:
        timerange: Tuple of size 2 containing [start_time, end_time] for the tool.
        filename: A string specifying the name of the output file in the output directory.
    """

    settings = _parse_settings(settings, user_defaults.IK_SETTINGS)
    return _run_tool(model, markers, output, settings, **kwargs)


def run_id(model: str, kinematics: str, output: str, settings="", **kwargs):
    """Run the OpenSim Inverse Dynamics tool.

    Returns boolean which indicates tool success.

    Required args:
        model: Path to input OpenSim model.
        kinematics: Path to input kinematics file.
        output: Path to (existing) desired results directory.
        settings: Path to an XML setting file for the OpenSim ID Tool. This can be
            specified manually, or alternatively loaded from user_settings.py

    Optional args:
        grfs: Path to input GRF file.
        load: Path to loads descriptor XML file. If None, the existing entry
            in input settings file is used as is.
        timerange: Tuple of size 2 containing [start_time, end_time] for the tool.
        filename: A string specifying the name of the output file in the output directory.
    """

    settings = _parse_settings(settings, user_defaults.ID_SETTINGS)
    return _run_tool(model, kinematics, output, settings, **kwargs)


def run_rra(model: str, motion: str, output: str, settings="", **kwargs) -> bool:
    """Run the OpenSim RRA tool.

    Returns boolean which indicates tool success.

    Required args:
        model: Path to input OpenSim model.
        motion: Path to input motion data, either a kinematics (.mot) or states (.sto) file.
        output: Path to (existing) desired results directory.
        settings: Path to an XML setting file for the OpenSim RRA Tool. This can be
            specified manually, or alternatively loaded from user_settings.py

    Optional args:
        grfs: Path to input GRF file.
        load: Path to loads descriptor XML file. If None, the existing entry
            in input settings file is used as is.
        adjust: Boolean flag indicating whether or not to produced an RRA-adjusted output model.
        body: Body to adjust if running with adjustment.
        model_out: Path at which to write adjusted model file.
        timerange: Tuple of size 2 containing [start_time, end_time] for the tool.
        point_actuator_names: A tuple of strings which overrides the default residual
            actuator names ("FX", "FY", "FZ") if needed.
    """

    settings = _parse_settings(settings, user_defaults.RRA_SETTINGS)
    return _run_tool(model, motion, output, settings, **kwargs)


def run_analyze(model: str, motion: str, output: str, settings="", **kwargs) -> bool:
    """Run the OpenSim Analyze tool.

    Returns boolean which indicates tool success.

    Required args:
        model: Path to input OpenSim model.
        motion: Path to input motion data, either a kinematics (.mot) or states (.sto) file.
        output: Path to (existing) desired results directory.
        settings: Path to an XML setting file for the OpenSim Analyze Tool. This can be
            specified manually, or alternatively loaded from user_settings.py

    Optional args:
        grfs: Path to input GRF file.
        load: Path to loads descriptor XML file. If None, the existing entry
            in input settings file is used as is.
        controls: Path to actuator controls file.
        timerange: Tuple of size 2 containing [start_time, end_time] for the tool.
        point_actuator_names: A tuple of strings which overrides the default residual
            actuator names ("FX", "FY", "FZ") if needed.
    """

    settings = _parse_settings(settings, user_defaults.ANALYZE_SETTINGS)
    return _run_tool(model, motion, output, settings, **kwargs)


def run_cmc(model: str, motion: str, output: str, settings="", **kwargs) -> bool:
    """Run the OpenSim CMC tool.

    Returns boolean which indicates tool success.

    Required args:
        model: Path to input OpenSim model.
        motion: Path to input motion data, either a kinematics (.mot) or states (.sto) file.
        output: Path to (existing) desired results directory.
        settings: Path to an XML setting file for the OpenSim CMC Tool. This can be
            specified manually, or alternatively loaded from user_settings.py

    Optional args:
        grfs: Path to input GRF file.
        load: Path to loads descriptor XML file. If None, the existing entry
            in input settings file is used as is.
        constraints: Path to control constraints file.
        controls: Path to controls from an appropriate RRA simulation. This imposes additional
            constraints on the values of residual actuators.
        timerange: Tuple of size 2 containing [start_time, end_time] for the tool.
        point_actuator_names: A tuple of strings which overrides the default residual
            actuator names ("FX", "FY", "FZ") if needed.
    """

    settings = _parse_settings(settings, user_defaults.CMC_SETTINGS)
    return _run_tool(model, motion, output, settings, **kwargs)


def run_fd(
    model: str, states: str, controls: str, output: str, settings="", **kwargs
) -> bool:
    """Run the OpenSim Forward Dynamics tool.

    Returns boolean which indicates tool success.

    Required args:
        model: Path to input OpenSim model.
        states: Path to input states file.
        controls: Path to input controls file
        output: Path to (existing) desired results directory.
        settings: Path to an XML setting file for the OpenSim FD Tool. This can be
            specified manually, or alternatively loaded from user_settings.py

    Optional args:
        grfs: Path to input GRF file.
        load: Path to loads descriptor XML file. If None, the existing entry
            in input settings file is used as is.
        timerange: Tuple of size 2 containing [start_time, end_time] for the tool.
        point_actuator_names: A tuple of strings which overrides the default residual
            actuator names ("FX", "FY", "FZ") if needed.
    """

    settings = _parse_settings(settings, user_defaults.FD_SETTINGS)
    return _run_tool(model, states, output, settings, controls=controls, **kwargs)


def _get_tool_from_settings(settings: str):
    """Parse settings file to find correct OpenSim tool function."""
    xml = ET.parse(settings)
    name = _get_classname_from_xml(xml)
    if name not in _SUPPORTED_TOOLS:
        raise ValueError(f"Tool ({name}) not supported.")
    return getattr(opensim, name)


def _get_classname_from_xml(xml: ET.ElementTree) -> str:
    root = xml.getroot()
    return root[0].tag


def _parse_settings(arg_settings: str, default_settings: Optional[str] = None) -> str:
    """Determine settings file location based on user input.

    Preference is given to any settings location specified as an argument. Failing
    that, we check the user_settings.py file. If this fails an exception is raised."""

    if arg_settings:
        return arg_settings
    if default_settings is not None:
        return default_settings
    raise ValueError("No settings file provided.")


@dataclass
class _ToolParameters:
    """A class for storing the potential fields which can be input to the OpenSim tool wrappers."""

    model_in: str
    kinematics: str
    output: str
    grfs: str = ""
    load: Optional[str] = None
    timerange: tuple = user_defaults.TIMERANGE
    adjust: bool = False
    body: str = user_defaults.ADJUSTMENT_BODY
    model_out: Optional[str] = None
    point_actuator_names: tuple = user_defaults.POINT_ACTUATORS
    controls: str = ""
    constraints: str = ""
    ik_filename = user_defaults.IK_FILENAME
    id_filename = user_defaults.ID_FILENAME

    def __post_init__(self):
        if self.model_out is None:
            self.model_out = os.path.join(self.output, user_defaults.MODEL_OUT)


def _run_tool(
    model: str,
    input_motion: str,
    output_dir: str,
    settings: str,
    **kwargs,
) -> bool:
    """Test docstring."""
    wrapper = _wrapper_factory(settings)
    parameters = _ToolParameters(model, input_motion, output_dir, **kwargs)
    wrapper.setup(parameters)
    return wrapper.run()


def _wrapper_factory(settings):
    """Use settings file to create the appropriate _ToolWrapper"""
    wrappers = {
        opensim.InverseKinematicsTool: _IKToolWrapper,
        opensim.InverseDynamicsTool: _IDToolWrapper,
        opensim.RRATool: _RRAToolWrapper,
        opensim.CMCTool: _CMCToolWrapper,
        opensim.AnalyzeTool: _AnalyzeToolWrapper,
        opensim.ForwardTool: _ForwardToolWrapper,
    }
    tool = _get_tool_from_settings(settings)
    wrapper = wrappers[tool]
    return wrapper(settings)


class _ToolWrapper(ABC):

    tool: opensim.AbstractTool or opensim.Tool
    settings: str

    def __init__(self, settings: str):
        self.settings = settings
        self.create()

    @abstractmethod
    def create(self):
        """Abstract method, subclasses will implement the appropriate tool constructor"""

    @abstractmethod
    def setup(self, parameters):
        """Abstract tool encapsulating the necessary setup steps"""

    def run(self) -> bool:
        """Run the tool"""
        return self.tool.run()


class _IKToolWrapper(_ToolWrapper):

    tool: opensim.InverseKinematicsTool

    def create(self):
        self.tool = opensim.InverseKinematicsTool(self.settings, False)

    def setup(self, parameters):
        self.tool.set_model_file(parameters.model_in)
        self.tool.setStartTime(parameters.timerange[0])
        self.tool.setEndTime(parameters.timerange[1])
        self.tool.setMarkerDataFileName(parameters.kinematics)
        self.tool.setResultsDir(parameters.output)
        self.tool.setOutputMotionFileName(
            os.path.join(parameters.output, parameters.ik_filename)
        )


class _ForceToolWrapper(_ToolWrapper):

    tool: opensim.InverseDynamicsTool or opensim.AbstractTool

    def set_grfs(self, grfs, load):
        """Assign force-related parameters for running the ID Tool"""
        if load is not None:
            self.tool.setExternalLoadsFileName(load)
        loads = self.tool.updExternalLoads()
        loads.setDataFileName(grfs)


class _IDToolWrapper(_ForceToolWrapper):

    tool: opensim.InverseDynamicsTool

    def create(self):
        self.tool = opensim.InverseDynamicsTool(self.settings, False)

    def setup(self, parameters):
        self.tool.setModelFileName(parameters.model_in)
        self.tool.setResultsDir(parameters.output)
        self.tool.setOutputGenForceFileName(parameters.filename)
        self.tool.setStartTime(parameters.timerange[0])
        self.tool.setEndTime(parameters.timerange[1])
        self.tool.setCoordinatesFileName(parameters.kinematics)
        self.set_grfs(parameters.grfs, parameters.load)


class _AbstractToolWrapper(_ForceToolWrapper):
    """Abstract interface for tools which inherit from opensim.AbtractTool

    A note on the behaviour of optional arguments:
        If timerange = None, the [-inf, inf] is used, resulting in processing of the entire file.
        If grfs = None, the tool is run with no GRFs.
        If load = None, the default load file present in the tool is left as is.
    """

    tool: opensim.AbstractTool

    def setup(self, parameters):
        """Overall setup procedure."""
        self.generic_setup(parameters)
        self.additional_setup(parameters)

    def generic_setup(self, parameters):
        """Generic setup steps for any AnalyzeTool subclass."""
        self.pre_load(parameters.model_in, parameters.grfs, parameters.load)
        self.load()
        self.modify_actuators(parameters.point_actuator_names)
        self.set_kinematics(parameters.kinematics)
        self.set_generic_parameters(parameters.timerange, parameters.output)

    @abstractmethod
    def additional_setup(self, parameters):
        """Subclass-specific Additional steps required by subclasses of _AbstractToolWrapper."""

    def pre_load(self, model_in: str, grfs: str = "", load: Optional[str] = None):
        """Assigning of variable parameters (model, grfs) before loading."""
        self.tool.setModelFilename(model_in)
        self.set_grfs(grfs, load)

    def load(self) -> opensim.Model:
        """Common interface for loading a model & updating model forces."""
        self.tool.loadModel(self.settings)
        model = self.tool.getModel()
        self.tool.updateModelForces(model, self.settings)
        return model

    def modify_actuators(self, point_actuator_names: Optional[list] = None):
        """Adjust point actuator points of action to match those of the model

        Model must have been loaded before calling this function"""

        # Get the first valid point actuator from point_actuator_names
        force_set = self.tool.getModel().updForceSet()
        point_actuators = [
            force for force in force_set if force.getName() in point_actuator_names
        ]
        if not point_actuators:
            return
        body_str = point_actuators[0].get_body()

        # Get the CoM of that body in the input model
        body = self.tool.getModel().getBodySet().get(body_str)
        com = body.getMassCenter()

        # Update the application point of each point actuator
        for point_actuator in point_actuators:
            point_actuator.set_point(com)

    def set_generic_parameters(self, timerange, output):
        """Common interface for setting timerange and output directory."""
        self.tool.setInitialTime(timerange[0])
        self.tool.setFinalTime(timerange[1])
        self.tool.setResultsDir(output)

    @abstractmethod
    def set_kinematics(self, kinematics):
        """Abstract method for setting kinematics for OpenSim AbstractTool inheritors"""


class _RRAToolWrapper(_AbstractToolWrapper):
    """Concrete RRATool"""

    tool: opensim.RRATool

    def create(self):
        self.tool = opensim.RRATool(self.settings, False)

    def additional_setup(self, parameters):
        self.set_adjustment(parameters.adjust, parameters.body, parameters.model_out)

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

    def create(self):
        self.tool = opensim.AnalyzeTool(self.settings, False)

    def additional_setup(self, parameters):
        if parameters.controls:
            self.set_controls(parameters.controls)

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


class _CMCToolWrapper(_AbstractToolWrapper):

    tool: opensim.CMCTool

    def create(self):
        self.tool = opensim.CMCTool(self.settings, False)

    def additional_setup(self, parameters):
        self.set_constraints(parameters.constraints, parameters.controls)

    def set_kinematics(self, kinematics):
        """Set desired kinematics path"""
        self.tool.setDesiredKinematicsFileName(kinematics)

    def set_constraints(self, constraints, controls):
        """Set constraints on actuator controls."""
        self.tool.setConstraintsFileName(constraints)
        self.tool.setRRAControlsFileName(controls)


class _ForwardToolWrapper(_AbstractToolWrapper):

    tool: opensim.ForwardTool

    def create(self):
        self.tool = opensim.ForwardTool(self.settings, False)

    def additional_setup(self, parameters):
        self.set_controls(parameters.controls)

    def set_kinematics(self, kinematics):
        """Set states, should include specified start time"""
        self.tool.setStatesFileName(kinematics)

    def set_controls(self, controls):
        """Add controls to the analysis"""
        self.tool.setControlsFileName(controls)


def main():
    """Boilerplate."""


if __name__ == "__main__":
    main()
