"""Python wrappers for a subset of the suite of OpenSim tools."""
import xml.etree.ElementTree as ET
from typing import Optional

import opensim

import user_defaults
import wrapping

_TOOL_MAPPING = {
    opensim.InverseKinematicsTool: (wrapping.IKToolParameters, wrapping.IKToolWrapper),
    opensim.InverseDynamicsTool: (wrapping.IDToolParameters, wrapping.IDToolWrapper),
    opensim.RRATool: (wrapping.RRAToolParameters, wrapping.RRAToolWrapper),
    opensim.CMCTool: (wrapping.CMCToolParameters, wrapping.CMCToolWrapper),
    opensim.ForwardTool: (wrapping.ForwardToolParameters, wrapping.ForwardToolWrapper),
    opensim.AnalyzeTool: (wrapping.AnalyzeToolParameters, wrapping.AnalyzeToolWrapper),
}


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
    """Parse an OpenSim tool settings file to find the corresponding class."""
    xml = ET.parse(settings)
    name = _get_classname_from_xml(xml)
    tool = getattr(opensim, name)
    if tool not in _TOOL_MAPPING:
        raise ValueError("Unsupported tool.")
    return tool


def _get_classname_from_xml(xml: ET.ElementTree) -> str:
    """Get the class of an OpenSim object from its XML representation."""
    root = xml.getroot()
    return root[0].tag


def _parse_settings(arg_settings: str, default_settings: Optional[str] = None) -> str:
    """Determine settings file location based on user input.

    Preference is given to any settings location specified as an argument. Failing
    that, we check the user_settings.py file. If this fails an exception is raised.
    """

    if arg_settings:
        return arg_settings
    if default_settings is not None:
        return default_settings
    raise ValueError("No settings file provided.")


def _run_tool(
    model: str,
    input_motion: str,
    output_dir: str,
    settings: str,
    **kwargs,
) -> bool:
    """Run the wrapper for the OpenSim tool specified by the input settings file & parameters."""
    tool = _get_tool_from_settings(settings)
    wrapper = _wrapper_factory(tool, settings)
    parameters = _parameter_factory(
        tool, model=model, kinematics=input_motion, output=output_dir, **kwargs
    )
    wrapper.setup(parameters)
    return wrapper.run()


def _wrapper_factory(tool, settings):
    """Constructs the wrapper for a given tool & settings file."""
    return _TOOL_MAPPING[tool][0](settings)


def _parameter_factory(tool, **kwargs):
    """Constructs the parameters object for a given tool & key word arguments."""
    return _TOOL_MAPPING[tool][1](**kwargs)


def main():
    """Boilerplate."""


if __name__ == "__main__":
    main()
