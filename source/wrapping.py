"""A module containing wrappers for OpenSim Tool classes as well as a set of dataclasses
representing the tool parameters."""
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import opensim

import user_defaults


@dataclass
class _GenericToolParameters(ABC):
    """A dataclass for storing the generic parameters which are shared by all tools."""

    model_in: str
    motion: str
    output_dir: str
    timerange: tuple = user_defaults.TIMERANGE


@dataclass
class IKToolParameters(_GenericToolParameters):
    """Parameters specific to the IK tool."""

    ik_filename = user_defaults.IK_FILENAME


@dataclass
class _ForceToolParameters(_GenericToolParameters):
    """Parameters to specific to tools which can handle external forces."""

    grfs: str = ""
    load: Optional[str] = None


@dataclass
class IDToolParameters(_ForceToolParameters):
    """Parameters specific to the ID tool."""

    id_filename = user_defaults.ID_FILENAME


@dataclass
class _AbstractToolParameters(_ForceToolParameters):
    """Parameters specific to tool which inherit from opensim.AbstractTool."""

    point_actuator_names: tuple = user_defaults.POINT_ACTUATORS


@dataclass
class RRAToolParameters(_AbstractToolParameters):
    """Parameters (and a special method) specific to the RRA tool."""

    adjust: bool = False
    body: str = user_defaults.ADJUSTMENT_BODY
    model_out: Optional[str] = None

    def __post_init__(self):
        if self.model_out is not None:
            return
        self.model_out = os.path.join(self.output_dir, user_defaults.MODEL_OUT)


@dataclass
class CMCToolParameters(_AbstractToolParameters):
    """Parameters specific to the CMC tool."""

    control_constraints: str = ""
    rra_constraints: str = ""


@dataclass
class AnalyzeToolParameters(_AbstractToolParameters):
    """Parameters specific to the Analyze tool."""

    controls: str = ""


@dataclass
class ForwardToolParameters(_AbstractToolParameters):
    """Parameters specific to the Forward tool."""

    controls: str = ""


class _ToolWrapper(ABC):
    """Abstract interface for OpenSim tool wrappers.

    Wrappers consist of an OpenSim tool object and the path to a tool settings file.
    More specifically, tools can be either AbstractTool objects (RRA, CMC, Analyze,
    FD) or Tool objects (IK, ID).

    All wrappers have three core non-init methods: create, setup and run. Subclasses
    implement create and setup depending using tool-specific OpenSim API calls. The
    run method is implemented concretely here.
    """

    tool: opensim.AbstractTool or opensim.Tool
    settings: str

    def __init__(self, settings: str):
        """Initialise tool based on settings file, and call class create method."""
        self.settings = settings
        self.create()

    @abstractmethod
    def create(self):
        """Abstract method encapsulating tool creation.

        Subclasses will implement a call to the appropriate OpenSim API class.
        """

    @abstractmethod
    def setup(self, parameters):
        """Abstract method encapsulating the necessary setup steps"""

    def run(self) -> bool:
        """Run the tool"""
        return self.tool.run()


class IKToolWrapper(_ToolWrapper):
    """Concrete wrapper for IK tool."""

    tool: opensim.InverseKinematicsTool

    def create(self):
        self.tool = opensim.InverseKinematicsTool(self.settings, False)

    def setup(self, parameters):
        self.tool.set_model_file(parameters.model_in)
        self.tool.setStartTime(parameters.timerange[0])
        self.tool.setEndTime(parameters.timerange[1])
        self.tool.setMarkerDataFileName(parameters.motion)
        self.tool.setResultsDir(parameters.output_dir)
        self.tool.setOutputMotionFileName(
            os.path.join(parameters.output_dir, parameters.ik_filename)
        )


class _ForceToolWrapper(_ToolWrapper):
    """Abstract interface for tools which involve GRFs."""

    tool: opensim.InverseDynamicsTool or opensim.AbstractTool

    def set_grfs(self, grfs, load):
        """Assign force-related parameters."""
        if load is not None:
            self.tool.setExternalLoadsFileName(load)
        loads = self.tool.updExternalLoads()
        loads.setDataFileName(grfs)


class IDToolWrapper(_ForceToolWrapper):
    """Concrete wrapper for ID tool."""

    tool: opensim.InverseDynamicsTool

    def create(self):
        self.tool = opensim.InverseDynamicsTool(self.settings, False)

    def setup(self, parameters):
        self.tool.setModelFileName(parameters.model_in)
        self.tool.setResultsDir(parameters.output_dir)
        self.tool.setOutputGenForceFileName(parameters.filename)
        self.tool.setStartTime(parameters.timerange[0])
        self.tool.setEndTime(parameters.timerange[1])
        self.tool.setCoordinatesFileName(parameters.motion)
        self.set_grfs(parameters.grfs, parameters.load)


class _AbstractToolWrapper(_ForceToolWrapper):
    """Abstract interface for tools which inherit from opensim.AbtractTool."""

    tool: opensim.AbstractTool

    def setup(self, parameters):
        self.generic_setup(parameters)
        self.additional_setup(parameters)

    def generic_setup(self, parameters):
        """Generic setup steps for any AnalyzeTool subclass."""
        self.pre_load(parameters.model_in, parameters.grfs, parameters.load)
        self.load()
        self.modify_actuators(parameters.point_actuator_names)
        self.set_kinematics(parameters.motion)
        self.set_generic_parameters(parameters.timerange, parameters.output_dir)

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

    def set_generic_parameters(self, timerange, output_dir):
        """Common interface for setting timerange and output directory."""
        self.tool.setInitialTime(timerange[0])
        self.tool.setFinalTime(timerange[1])
        self.tool.setResultsDir(output_dir)

    @abstractmethod
    def set_kinematics(self, kinematics):
        """Abstract method for setting kinematics for OpenSim AbstractTool inheritors"""


class RRAToolWrapper(_AbstractToolWrapper):
    """Concrete wrapper for RRA tool."""

    tool: opensim.RRATool

    def create(self):
        self.tool = opensim.RRATool(self.settings, False)

    def additional_setup(self, parameters):
        self.set_adjustment(parameters.adjust, parameters.body, parameters.model_out)

    def set_kinematics(self, kinematics):
        self.tool.setDesiredKinematicsFileName(kinematics)

    def set_adjustment(self, adjust, body, model_out):
        """Controls whether to adjust input model, which body to adjust, & output model path."""
        self.tool.setAdjustCOMToReduceResiduals(adjust)
        if not adjust:
            return
        self.tool.setAdjustedCOMBody(body)
        self.tool.setOutputModelFileName(model_out)


class AnalyzeToolWrapper(_AbstractToolWrapper):
    """Concrete wrapper for AnalyzeTool."""

    tool: opensim.AnalyzeTool

    def create(self):
        self.tool = opensim.AnalyzeTool(self.settings, False)

    def additional_setup(self, parameters):
        if parameters.controls:
            self.set_controls(parameters.controls)

    def set_kinematics(self, kinematics):
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
        """Specify input controls from file name."""
        self.tool.setControlsFileName(controls)


class CMCToolWrapper(_AbstractToolWrapper):
    """Concrete wrapper for CMC Tool."""

    tool: opensim.CMCTool

    def create(self):
        self.tool = opensim.CMCTool(self.settings, False)

    def additional_setup(self, parameters):
        self.set_constraints(parameters.control_constraints, parameters.rra_constraints)

    def set_kinematics(self, kinematics):
        self.tool.setDesiredKinematicsFileName(kinematics)

    def set_constraints(self, control_constraints, rra_constraints):
        """Set constraints from file and/or an RRA controls file."""
        self.tool.setConstraintsFileName(control_constraints)
        self.tool.setRRAControlsFileName(rra_constraints)


class ForwardToolWrapper(_AbstractToolWrapper):
    """Concrete wrapper for Forward tool."""

    tool: opensim.ForwardTool

    def create(self):
        self.tool = opensim.ForwardTool(self.settings, False)

    def additional_setup(self, parameters):
        self.set_controls(parameters.controls)

    def set_kinematics(self, kinematics):
        self.tool.setStatesFileName(kinematics)

    def set_controls(self, controls):
        """Specify input controls from file name."""
        self.tool.setControlsFileName(controls)
