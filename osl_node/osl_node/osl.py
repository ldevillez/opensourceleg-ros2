"""
OSL module to control joints of the osl

Made to listen to a ROS2 network

@author: ldevillez
"""

from numpy import pi

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.logging import get_logging_directory

# OSL imports
from opensourceleg.actuators.base import CONTROL_MODES
from opensourceleg.sensors.encoder import AS5048B
from opensourceleg.actuators.dephy import DephyActuator, IMPEDANCE_A, IMPEDANCE_C, DEFAULT_POSITION_GAINS
from opensourceleg.logging import LOGGER

# OSL ROS2 Interface
import osl_interface.msg as osl_msg
import osl_interface.srv as osl_srv
import osl_interface.action as osl_act

# OSL ROS2 node
from osl_node.defs import TYPE_JOINT, TYPE_CONTROLLER, TORQUE_CONST_OLD, TORQUE_CONST_NEW
from osl_node.utils import get_tty_joint, control_mode_to_name, DEG_TO_RAD, RAD_TO_DEG


MAX_ANGLE_ANKLE = 30

GEAR_RATIO = 9.0
BELT_RATIO = 83.0 / 18.0
TR = GEAR_RATIO * BELT_RATIO

# Torque constants
TORQUE_CONST_KNEE =  TORQUE_CONST_OLD
TORQUE_CONST_ANKLE = TORQUE_CONST_NEW

# Set logging directory
LOGGER.set_file_name("osl_ros.log")

class Joint(Node):
    """
    Node representing a prosthetic joint using the OSL harware
    """

    joint_type: TYPE_JOINT

    encoder: AS5048B
    last_encoder_position: float
    actuator: DephyActuator

    ref_time: rclpy.time.Time # Used to smooth transition with control

    status: osl_msg.InterfaceStatus
    control_mode: osl_msg.ControlMode

    # For position control
    target_position: float
    target_velocity: float

    # For impedance control
    impedance_k: float
    impedance_b: float
    impedance_theta_eq: float

    type_controller: TYPE_CONTROLLER # Position or Impedance
    target_type_controller: TYPE_CONTROLLER


    def __init__(self, joint_type: TYPE_JOINT):

        art_name = str(joint_type).lower()
        self.joint_type  = joint_type
        super().__init__(f"OSL_{art_name}")
        self.get_logger().info(f"Starting OSL {art_name} node")

        self.status = osl_msg.InterfaceStatus.INIT
        self.control_mode = osl_msg.ControlMode()
        self.control_mode.mode = osl_msg.ControlMode.MANUAL
        self.ref_time = self.get_clock().now()

        self.target_type_controller = TYPE_CONTROLLER.POSITION
        self.type_controller = None

        # Subscription callback group
        self.subscription_kinematic = self.create_subscription(
            osl_msg.JointKinematic, f"{art_name}/position_control", self.kinematic_callback, 10
        )

        self.publisher_impedance = self.create_subscription(
            osl_msg.JointImpedance, f"/{art_name}/impedance_control", self.impedance_callback, 10
        )

        # Publishing
        dt_pub = 1e-2
        self._dt_pub = 1/dt_pub
        # self.timer = self.create_timer(dt_pub, self.publish_callback)

        self.kin_joint_publisher = self.create_publisher(
            osl_msg.JointKinematic, f"/{art_name}/joint_kinematic", 10
        )

        self.motor_info_publisher = self.create_publisher(
            osl_msg.MotorInfo, f"/{art_name}/motor_info", 10
        )


        # Service
        self.srv_motor_interface_status = self.create_service(
            osl_srv.SetInterfaceStatus,
            f"/{art_name}/set_interface_status",
            self.handle_motor_interface_status,
            callback_group=ReentrantCallbackGroup(),
        )

        self.srv_ctrl_mode = self.create_service(
            osl_srv.SetMotorControl,
            f"/{art_name}/set_motor_control",
            self.control_mode_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # Actions

        self.act_homing = ActionServer(
            self,
            osl_act.JointHome,
            f"/{art_name}/home",
            self.homing_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.act_goto = ActionServer(
            self,
            osl_act.JointGoTo,
            f"/{art_name}/go_to",
            self.goto_callback,
            callback_group=ReentrantCallbackGroup(),
        )


        # Control loop
        self.dt = 2.5e-3 # 400 Hz
        self.motor_control_timer = self.create_timer(
            self.dt,
            self.motor_control_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        # Sensors
        bus_number = 2
        if joint_type == TYPE_JOINT.ANKLE:
            bus_number = 3
        elif joint_type == TYPE_JOINT.KNEE:
            bus_number = 2

        self.get_logger().info(f"Using I2C bus {bus_number} for AS5048B")
        try:
            self.encoder = AS5048B(
                tag=f"encoder_{art_name}",
                bus=f"/dev/i2c-{bus_number}",
                A1_adr_pin=False,
                A2_adr_pin=False,
                zero_position=0,
                enable_diagnostics=False,
            )

            self.encoder.start()

            self.last_encoder_position = 0
            self.last_encoder_time = self.get_clock().now().nanoseconds * 1e-9

            self.get_logger().info("AS5048B module initialized")
        except FileNotFoundError:
            self.get_logger().error("Could not find AS5048B module")
            raise SystemExit(1)

        # Actuators
        tty_joint = get_tty_joint(joint_type)
        if tty_joint is None:
            self.get_logger().error("Could not find actuator")
            raise SystemExit(1)
        try:

            self.actuator = DephyActuator(
                port=tty_joint,
                gear_ratio=TR,
                debug_level=0,
                dephy_log=False,
            )


            torque_const = TORQUE_CONST_ANKLE
            if joint_type == TYPE_JOINT.KNEE:
                torque_const = TORQUE_CONST_KNEE

            self.actuator.MOTOR_CONSTANTS.NM_PER_AMP = torque_const
            self.actuator.MOTOR_CONSTANTS.NM_PER_RAD_TO_K=((2 * pi / 16384) / IMPEDANCE_C * 1e3 / torque_const)
            self.actuator.MOTOR_CONSTANTS.NM_S_PER_RAD_TO_B=((pi / 180) / IMPEDANCE_A * 1e3 / torque_const)

            self.actuator.start()
        except IOError:
            self.get_logger().error("Could not find Dephy actuator")
            raise SystemExit(1)

    @property
    def position(self):
        """
        Return the current joint position in rad
        """
        self.encoder.update()
        pos = self.encoder.position

        return -pos

    @property
    def velocity(self):
        """
        Return the current joint velocity in rad/s
        """

        _, vel = self.position_velocity

        return vel

    @property
    def position_velocity(self):
        """
        Return the current joint position in rad and velocity in rad/s
        """

        pos = self.position
        encoder_time = self.get_clock().now().nanoseconds * 1e-9

        vel = (pos - self.last_encoder_position) / (encoder_time - self.last_encoder_time)

        self.last_encoder_position = pos
        self.last_encoder_time = encoder_time

        return pos, vel

    @property
    def is_enabled(self):
        """
        Return if the joint is enabled
        """
        return self.status == osl_msg.InterfaceStatus.ENABLED

    @property
    def is_manual(self):
        """
        Return if the joint is in manual mode
        """
        return self.control_mode.mode == osl_msg.ControlMode.MANUAL


    # Subscription callbacks
    def kinematic_callback(self, msg):
        """
        Receive kinematic control messages
        """
        self.target_position = msg.position
        self.target_velocity = msg.velocity

        self.target_type_controller = TYPE_CONTROLLER.POSITION

    def impedance_callback(self, msg):
        """
        Receive impedance control messages
        """
        self.impedance_k = msg.k
        self.impedance_b = msg.b
        self.impedance_theta_eq = msg.theta_eq

        self.target_type_controller = TYPE_CONTROLLER.IMPEDANCE


    # Publishing callback
    def publish_callback(self):
        """
        Publish joint control messages.
        Called at fixed dt
        """

        kin_msg = osl_msg.JointKinematic()

        pos, vel = self.position_velocity

        kin_msg.position = float(pos)
        kin_msg.velocity = float(vel)

        self.kin_joint_publisher.publish(kin_msg)


        motor_info = osl_msg.MotorInfo()
        self.actuator.update()

        motor_info.position = (self.actuator.motor_position)

        # Current, and power here
        current = self.actuator.battery_current / 1000 # in A
        voltage = self.actuator.battery_voltage / 1000 # in V

        motor_info.current = float(current)
        motor_info.power_e = float(voltage * current)  # in W

        torque = self.actuator.motor_torque  # in Nm
        velocity = self.actuator.motor_velocity  # in rad/s

        motor_info.power_m = float(torque * velocity)  # in W

        self.motor_info_publisher.publish(motor_info)

    def handle_motor_interface_status(self, request, response):
        """
        Service to ask for motor interface status
        """

        # always respond that
        # Should manage from the input request
        input_status = request.status.status
        status_available = osl_msg.InterfaceStatus
        ret_status = status_available.ERROR

        res = False

        if input_status == status_available.ASKING:
            ret_status = self.status
            res = True

        if input_status == status_available.INIT:
            # Do initialization procedure
            self.status = status_available.INIT
            ret_status = self.status
            res = True

        if input_status == status_available.ENABLED:
            # Do initialization procedure
            self.status = status_available.ENABLED
            ret_status = self.status
            res = True



        response.status.status = ret_status
        response.success = bool(res)

        if input_status != status_available.ASKING:
            if response.success:
                self.get_logger().info(f"Status {input_status} set")
            else:
                self.get_logger().info(f"Could not set status {name_status}")

        return response

    def control_mode_callback(self, request, response):
        """
        callback when asked to change control mode
        """
        if self.control_mode.mode != request.mode.mode:
            self.ref_time = self.get_clock().now()

        self.control_mode = request.mode

        self.get_logger().info(
            f"Control mode switch to {control_mode_to_name(request.mode)}"
        )

        if self.control_mode == osl_msg.ControlMode.TRACKING:
            # We reset the current controller as we do not know which it is
            self.type_controller = None
            self.actuator.set_control_mode(mode=CONTROL_MODES.VOLTAGE)
            self.actuator.set_motor_voltage(0)

        else:
            self.type_controller = None
            self.actuator.set_control_mode(mode=CONTROL_MODES.VOLTAGE)
            self.actuator.set_motor_voltage(0)

        response.mode = request.mode
        response.success = True
        return response


    def homing_callback(self, goal_handle):
        """
        Callback to use when a homing sequence is used
        """

        is_not_manual =  not self.is_manual
        is_not_enabled = not self.is_enabled

        self.encoder.update()

        # No homing if not in MANUAL mode
        if is_not_manual or is_not_enabled:
            if is_not_manual:
                self.get_logger().info(
                    f"Can not home in {control_mode_to_name(self.control_mode)} mode"
                )
            if is_not_enabled:
                self.get_logger().info(
                    f"Can not home if not enabled"
                )

            goal_handle.succeed()

            result = osl_act.JointHome.Result()
            result.qc_position = float(self.encoder.position)

            return result

        self.get_logger().info("Homing joint")

        def homing_callback():
            feedback_msg = osl_act.JointHome.Feedback()
            feedback_msg.qc_position = int(self.encoder.position)
            goal_handle.publish_feedback(feedback_msg)

            self.encoder.update()
            if self.joint_type == TYPE_JOINT.ANKLE:
                self.encoder.zero_position = self.encoder.counts - self.encoder.deg_to_counts(
                   MAX_ANGLE_ANKLE
                )

            else:
                self.encoder.zero_position = self.encoder.counts

        position_offset = 0.0
        if self.joint_type == TYPE_JOINT.ANKLE:
            position_offset = MAX_ANGLE_ANKLE * DEG_TO_RAD  # Set the value for the knee
        self.actuator.home(
                output_position_offset=position_offset,
                callback=homing_callback
            )

        goal_handle.succeed()

        result = osl_act.JointHome.Result()

        self.encoder.update()
        self.actuator.update()
        result.qc_position = int(self.encoder.position)

        return result

    def goto_callback(self, goal_handle):
        """
        Callback when a goto action is asked
        """

        is_not_manual =  not self.is_manual
        is_not_enabled = not self.is_enabled

        if is_not_manual or is_not_enabled:
            if is_not_manual:
                self.get_logger().info(
                    f"Can not manual move in {control_mode_to_name(self.control_mode)} mode"
                )
            elif is_not_enabled:
                self.get_logger().info(
                    f"Can not move if not enabled"
                )
            goal_handle.succeed()

            result = osl_act.JointGoTo.Result()

            self.encoder.update()
            self.actuator.update()
            result.qc_position = int(self.actuator.motor_position)
            result.rad_position = float(self.encoder.position)
            return result

        req = goal_handle.request

        target = req.position

        if not req.is_rad:
            target = target  * RAD_TO_DEG

        if req.relative:
            self.actuator.update
            target = self.actuator.motor_position() + target

        self.get_logger().info(f"Moving {'relative' if req.relative else 'absolute'} to {target}")


        feedback_msg = osl_act.JointGoTo.Feedback()
        goal_handle.publish_feedback(feedback_msg)

        self.actuator.set_control_mode(mode=CONTROL_MODES.POSITION)

        self.actuator.update()
        old_pos = self.actuator.motor_position
        mot_pos = 0.0

        self.actuator.set_position_gains(kp=6)
        self.actuator.set_output_position(target)


        self.actuator.set_control_mode(mode=CONTROL_MODES.IDLE)

        goal_handle.succeed()

        result = osl_act.JointGoTo.Result()

        self.encoder.update()
        result.qc_position = int(mot_pos)
        result.rad_position = float(self.encoder.position)

        return result

    def motor_control_callback(self):
        """
        Main function which is used to control the motor
        """

        # Control only if in tracking mode
        if self.control_mode.mode != osl_msg.ControlMode.TRACKING:
            return

        # Control only if enabled
        if self.status != osl_msg.InterfaceStatus.ENABLED:
            return

        self.actuator.update()


        if self.target_type_controller == TYPE_CONTROLLER.POSITION:
            # Position ontroller
            self.get_logger().debug("Start Position control")

            if self.target_type_controller != self.type_controller:
                self.get_logger().info("Switching to position control")
                self.type_controller = TYPE_CONTROLLER.POSITION
                self.actuator.update()
                self.actuator.set_control_mode(mode=CONTROL_MODES.POSITION)
                if self.joint_type == TYPE_JOINT.ANKLE:
                    self.actuator.set_position_gains(kp=130, ki=10, kd=5)
                elif self.joint_type == TYPE_JOINT.KNEE:
                    self.actuator.set_position_gains(kp=140, ki=1.5, kd=2.5)
                else:
                    self.actuator.set_position_gains()
                self.actuator.set_motor_position(self.actuator.motor_position)  # Hold current position


            cur_t = self.get_clock().now() - self.ref_time
            cur_t = cur_t.nanoseconds * 1e-9
            transition_time = 5.0  # seconds


            target = self.target_position

            if cur_t < transition_time:
                # Smooth transition
                self.actuator.update()
                current_pos = self.actuator.motor_position / TR
                desired_pos = target
                smooth_pos = current_pos + (desired_pos - current_pos) * (cur_t / transition_time)
                target = smooth_pos

            self.actuator.set_output_position(target)
            self.get_logger().debug("End Position control")

        else:
            # Impedance controller
            if self.target_type_controller != self.type_controller:
                self.get_logger().info("Switching to impedance control")
                self.type_controller = TYPE_CONTROLLER.IMPEDANCE
                self.actuator.set_control_mode(mode=CONTROL_MODES.IMPEDANCE)
                if self.joint_type == TYPE_JOINT.ANKLE:
                    self.actuator.set_impedance_cc_pidf_gains(
                            kp=140, ki=150, kd=0, ff=120
                        )
                elif self.joint_type == TYPE_JOINT.KNEE:
                    self.actuator.set_impedance_cc_pidf_gains(
                            kp=100, ki=200, kd=0, ff=128
                        )
                else:
                    self.actuator.set_impedance_cc_pidf_gains()

                self.actuator.set_output_impedance()


            self.actuator.set_output_impedance(
                k=self.impedance_k,
                b=self.impedance_b,
                )

            cur_t = self.get_clock().now() - self.ref_time
            cur_t = cur_t.nanoseconds * 1e-9
            transition_time = 5.0  # seconds

            if cur_t < transition_time:
                # Smooth transition
                current_pos = self.actuator.motor_position / TR
                desired_pos = self.impedance_theta_eq
                smooth_pos = current_pos + (desired_pos - current_pos) * (cur_t / transition_time)
                target = smooth_pos
            else:
                target = self.impedance_theta_eq

            self.actuator.set_output_position(target)




def main(type_joint: TYPE_JOINT = TYPE_JOINT.ANKLE, args=None):
    """
    Main logic of the estimator

    parameters
    ----------
    type_joint : TYPE_JOINT
        The type of joint (ANKLE or KNEE)
    args : list
        List of arguments to pass to rclpy.init
    """

    try:
        rclpy.init(args=args)

        joint = Joint(type_joint)

        rclpy.spin(joint)

        joint.destroy_node()
    except KeyboardInterrupt:
        print("Shutting down")

    joint.actuator.stop()

    rclpy.shutdown()


def osl_ankle():
    """
    Launch OSL node for ankle joint
    """
    main(TYPE_JOINT.ANKLE)

def osl_knee():
    """
    Launch OSL node for knee joint
    """
    main(TYPE_JOINT.KNEE)
