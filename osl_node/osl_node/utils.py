"""
Utility module to support the osl controller

@author: ldevillez
"""

import pyudev
from bl_osl_controller.defs import TYPE_JOINT
import bl_interface.msg as bl_interface_msg
from numpy import pi


RAD_TO_DEG = 180.0 / pi
DEG_TO_RAD = pi / 180.0

BUS_KNEE_ID  = "4"
BUS_ANKLE_ID = "2"

def find_tty_for_usb(bus_num: str) -> str:
    """
    Find tty device nodes associated with a USB device given its vendor and product IDs.

    parameters
    ----------
    bus_num : str
        Bus number of the USB device
    dev_num : str
        Device number of the USB device

    return
    ------
    list[str]
        List of tty device nodes associated with the USB device
    """

    context = pyudev.Context()
    tty_devices = []

    for device in context.list_devices(subsystem='tty'):
        parent = device.find_parent('usb', 'usb_device')
        if parent is not None:
            bnum = parent.attributes.get("busnum")

            # Avoid select the root USB hub
            vid = parent.attributes.get('idVendor')
            if bnum and bnum.decode() == bus_num and vid and vid.decode() != "1d6b":
                tty_devices.append(device.device_node)

    return tty_devices

def get_tty(bus_joint_id) -> str | None:
    """
    Get the tty device node for the specified bus joint ID.

    parameters
    ----------
    bus_joint_id : str
        The bus joint ID of the USB device

    return
    ------
    str | None
        The tty device node if found, otherwise None
    """
    tty_devices = find_tty_for_usb(bus_joint_id)
    if tty_devices:
        return tty_devices[0]
    else:
        return None

def get_tty_knee() -> str | None:
    """
    Get the tty device node for the knee USB device.

    return
    ------
    str | None
        The tty device node if found, otherwise None
    """

    return get_tty(BUS_KNEE_ID)

def get_tty_ankle() -> str | None:
    """
    Get the tty device node for the ankle USB device.

    return
    ------
    str | None
    """

    return get_tty(BUS_ANKLE_ID)

def get_tty_joint(joint_type: TYPE_JOINT = TYPE_JOINT.ANKLE) -> str | None:
    """
    Get the tty device node for the specified joint type.

    parameters
    ----------
    joint_type : TYPE_JOINT
        The type of joint (ANKLE or KNEE)
        default is TYPE_JOINT.ANKLE

    return
    ------
    str | None
        The tty device node if found, otherwise None
    """
    if joint_type == TYPE_JOINT.KNEE:
        return get_tty_knee()
    elif joint_type == TYPE_JOINT.ANKLE:
        return get_tty_ankle()
    else:
        return None

dict_name_to_control_mode = {
    "MANUAL": bl_interface_msg.ControlMode.MANUAL,
    "TRACKING": bl_interface_msg.ControlMode.TRACKING,
}


dict_control_mode_to_name = {}
for key, val in dict_name_to_control_mode.items():
    dict_control_mode_to_name[val] = key


def name_to_control_mode(id_mode) -> int:
    """
    Convert a name to a control_mode

    parameters
    ----------
    id_mode : str
        The name of the control mode

    return
    ------
    int
        The corresponding control mode integer value
    """
    if id_mode in dict_name_to_control_mode:
        return dict_name_to_control_mode[id_mode]
    return 99


def control_mode_to_name(control_mode) -> str:
    """
    Convert a control_mode to a name

    parameters
    ----------
    control_mode : bl_interface_msg.ControlMode
        The control mode object

    return
    ------
    str
        The corresponding name of the control mode
    """
    if control_mode.mode in dict_control_mode_to_name:
        return dict_control_mode_to_name[control_mode.mode]
    return "UNKNOWN"

if __name__ == "__main__":
    knee_tty = get_tty_joint(TYPE_JOINT.KNEE)
    ankle_tty = get_tty_joint(TYPE_JOINT.ANKLE)

    print(f"Knee TTY: {knee_tty}")
    print(f"Ankle TTY: {ankle_tty}")
