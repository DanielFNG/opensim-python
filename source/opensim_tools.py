"""Python wrappers for a subset of the suite of OpenSim tools."""
import xml.etree.ElementTree as ET
from typing import Optional

import opensim

import user_defaults
import wrapping


def run_tool_from_settings(settings: str) -> bool:
    """Run an OpenSim tool from a supplied settings file.

    Returns boolean which indicates tool success.

    Requires args:
        settings: Path to an XML setting file for the desired OpenSim tool.
    """
    tool = _get_tool_from_settings(settings)
    return tool.run()


def run_ik(
    model: str,
    markers: str,
    output: str,
    settings: Optional[str] = user_defaults.IK_SETTINGS,
    timerange: tuple = user_defaults.TIMERANGE,
    filename: str = user_defaults.IK_FILENAME,
) -> bool:
    """Run the OpenSim Inverse Kinematics tool.

    Returns boolean which indicates tool success.

    Required args:
        model: Path to input OpenSim model.
        markers: Path to input markers (.trc) file.
        output: Path to (existing) desired results directory.

    Optional args:
        settings: Path to an XML setting file for the OpenSim IK Tool. This can be
            specified manually, or alternatively loaded from user_settings.py.
        timerange: Tuple of size 2 containing [start_time, end_time] for the tool.
        filename: A string specifying the name of the output file in the output directory.
    """

    return _run_tool(
        model,
        markers,
        output,
        settings=settings,
        timerange=timerange,
        filename=filename,
    )


def run_id(
    model: str,
    kinematics: str,
    output: str,
    settings: Optional[str] = user_defaults.ID_SETTINGS,
    grfs: str = "",
    load: Optional[str] = None,
    timerange: tuple = user_defaults.TIMERANGE,
    filename: str = user_defaults.ID_FILENAME,
) -> bool:
    """Run the OpenSim Inverse Dynamics tool.

    Returns boolean which indicates tool success.

    Required args:
        model: Path to input OpenSim model.
        kinematics: Path to input kinematics file.
        output: Path to (existing) desired results directory.

    Optional args:
        settings: Path to an XML setting file for the OpenSim ID Tool. This can be
            specified manually, or alternatively loaded from user_settings.py
        grfs: Path to input GRF file.
        load: Path to loads descriptor XML file. If None, the existing entry
            in input settings file is used as is.
        timerange: Tuple of size 2 containing [start_time, end_time] for the tool.
        filename: A string specifying the name of the output file in the output directory.
    """

    return _run_tool(
        model,
        kinematics,
        output,
        settings=settings,
        grfs=grfs,
        load=load,
        timerange=timerange,
        filename=filename,
    )


def run_rra(
    model: str,
    motion: str,
    output: str,
    settings: Optional[str] = user_defaults.RRA_SETTINGS,
    grfs: str = "",
    load: Optional[str] = None,
    adjust: bool = False,
    body: str = user_defaults.ADJUSTMENT_BODY,
    model_out: str = user_defaults.MODEL_OUT,
    timerange: tuple = user_defaults.TIMERANGE,
    point_actuator_names: tuple = user_defaults.POINT_ACTUATORS,
) -> bool:
    """Run the OpenSim RRA tool.

    Returns boolean which indicates tool success.

    Required args:
        model: Path to input OpenSim model.
        motion: Path to input motion data, either a kinematics (.mot) or states (.sto) file.
        output: Path to (existing) desired results directory.

    Optional args:
        settings: Path to an XML setting file for the OpenSim RRA Tool. This can be
            specified manually, or alternatively loaded from user_settings.py
        grfs: Path to input GRF file.
        load: Path to loads descriptor XML file. If None, the existing entry
            in input settings file is used as is.
        adjust: Boolean flag indicating whether or not to produced an RRA-adjusted output model.
        body: Body to adjust if running with adjustment.
        model_out: Filename for adjusted model file.
        timerange: Tuple of size 2 containing [start_time, end_time] for the tool.
        point_actuator_names: A tuple of strings which overrides the default residual
            actuator names ("FX", "FY", "FZ") if needed.
    """

    return _run_tool(
        model,
        motion,
        output,
        settings=settings,
        grfs=grfs,
        load=load,
        adjust=adjust,
        body=body,
        model_out=model_out,
        timerange=timerange,
        point_actuator_names=point_actuator_names,
    )


def run_analyze(
    model: str,
    motion: str,
    output: str,
    settings: Optional[str] = user_defaults.ANALYZE_SETTINGS,
    grfs: str = "",
    load: Optional[str] = None,
    controls: str = "",
    timerange: tuple = user_defaults.TIMERANGE,
    point_actuator_names: tuple = user_defaults.POINT_ACTUATORS,
) -> bool:
    """Run the OpenSim Analyze tool.

    Returns boolean which indicates tool success.

    Required args:
        model: Path to input OpenSim model.
        motion: Path to input motion data, either a kinematics (.mot) or states (.sto) file.
        output: Path to (existing) desired results directory.

    Optional args:
        settings: Path to an XML setting file for the OpenSim Analyze Tool. This can be
            specified manually, or alternatively loaded from user_settings.py
        grfs: Path to input GRF file.
        load: Path to loads descriptor XML file. If None, the existing entry
            in input settings file is used as is.
        controls: Path to actuator controls file.
        timerange: Tuple of size 2 containing [start_time, end_time] for the tool.
        point_actuator_names: A tuple of strings which overrides the default residual
            actuator names ("FX", "FY", "FZ") if needed.
    """

    return _run_tool(
        model,
        motion,
        output,
        settings=settings,
        grfs=grfs,
        load=load,
        controls=controls,
        timerange=timerange,
        point_actuator_names=point_actuator_names,
    )


def run_cmc(
    model: str,
    motion: str,
    output: str,
    settings: Optional[str] = user_defaults.CMC_SETTINGS,
    grfs: str = "",
    load: Optional[str] = None,
    constraints: str = "",
    controls: str = "",
    timerange: tuple = user_defaults.TIMERANGE,
    point_actuator_names: tuple = user_defaults.POINT_ACTUATORS,
) -> bool:
    """Run the OpenSim CMC tool.

    Returns boolean which indicates tool success.

    Required args:
        model: Path to input OpenSim model.
        motion: Path to input motion data, either a kinematics (.mot) or states (.sto) file.
        output: Path to (existing) desired results directory.

    Optional args:
        settings: Path to an XML setting file for the OpenSim CMC Tool. This can be
            specified manually, or alternatively loaded from user_settings.py
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

    return _run_tool(
        model,
        motion,
        output,
        settings=settings,
        grfs=grfs,
        load=load,
        constraints=constraints,
        controls=controls,
        timerange=timerange,
        point_actuator_names=point_actuator_names,
    )


def run_fd(
    model: str,
    states: str,
    controls: str,
    output: str,
    settings: Optional[str] = user_defaults.FD_SETTINGS,
    grfs: str = "",
    load: Optional[str] = None,
    timerange: tuple = user_defaults.TIMERANGE,
    point_actuator_names: tuple = user_defaults.POINT_ACTUATORS,
) -> bool:
    """Run the OpenSim Forward Dynamics tool.

    Returns boolean which indicates tool success.

    Required args:
        model: Path to input OpenSim model.
        states: Path to input states file.
        controls: Path to input controls file
        output: Path to (existing) desired results directory.

    Optional args:
        settings: Path to an XML setting file for the OpenSim FD Tool. This can be
            specified manually, or alternatively loaded from user_settings.py
        grfs: Path to input GRF file.
        load: Path to loads descriptor XML file. If None, the existing entry
            in input settings file is used as is.
        timerange: Tuple of size 2 containing [start_time, end_time] for the tool.
        point_actuator_names: A tuple of strings which overrides the default residual
            actuator names ("FX", "FY", "FZ") if needed.
    """

    return _run_tool(
        model,
        states,
        output,
        settings=settings,
        controls=controls,
        grfs=grfs,
        load=load,
        timerange=timerange,
        point_actuator_names=point_actuator_names,
    )


def _get_tool_from_settings(settings: str):
    """Parse an OpenSim tool settings file to find the corresponding class."""
    xml = ET.parse(settings)
    name = _get_classname_from_xml(xml)
    tool = getattr(opensim, name)
    wrapping.check_tool_validity(tool)
    return tool(settings)


def _get_classname_from_xml(xml: ET.ElementTree) -> str:
    """Get the class of an OpenSim object from its XML representation."""
    root = xml.getroot()
    return root[0].tag


def _run_tool(
    model: str,
    input_motion: str,
    output_dir: str,
    settings: Optional[str] = None,
    **kwargs,
) -> bool:
    """Run the wrapper for the OpenSim tool specified by the input settings file & parameters."""
    if settings is None:
        raise ValueError("No settings file provided in arguments or user_defaults.")
    tool = _get_tool_from_settings(settings)
    wrapper = wrapping.wrapper_factory(tool, settings)
    parameters = wrapping.parameter_factory(
        tool, model=model, kinematics=input_motion, output=output_dir, **kwargs
    )
    wrapper.setup(parameters)
    return wrapper.run()
