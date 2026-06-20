.. Dynamically generated list of documented parameters
.. This page was generated using Tools\/autotest\/param\_metadata\/param\_parse\.py

.. DO NOT EDIT


.. _parameters:

Complete Parameter List
=======================

This is a complete list of the parameters which can be set \(e\.g\. via the MAVLink protocol\) to control vehicle behaviour\. They are stored in persistent storage on the vehicle\.

This list is automatically generated from the latest ardupilot source code\, and so may contain parameters which are not yet in the stable released versions of the code\.




.. _parameters_MiniDP:

MiniDP Parameters
-----------------


.. _FORMAT_VERSION:

FORMAT\_VERSION: MiniDP parameter storage format
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Internal MiniDP storage format marker\. MiniDP resets parameters when the expected format changes\.


.. _LOG_BITMASK:

LOG\_BITMASK: MiniDP log bitmask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Controls MiniDP standard and vehicle\-specific DataFlash logging\. Set to \-1 to enable all useful logs\.


+-----+-------------------------------------+
| Bit | Meaning                             |
+=====+=====================================+
| 0   | Attitude AHRS EKF                   |
+-----+-------------------------------------+
| 1   | GPS                                 |
+-----+-------------------------------------+
| 2   | Battery and power                   |
+-----+-------------------------------------+
| 3   | IMU and vibration                   |
+-----+-------------------------------------+
| 4   | Compass                             |
+-----+-------------------------------------+
| 5   | RC input and servo output           |
+-----+-------------------------------------+
| 6   | MiniDP controller output and status |
+-----+-------------------------------------+




.. _AUTH_MAV_MAN:

AUTH\_MAV\_MAN: Allow MAVLink manual control authority
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Allows MAVLink MANUAL\_CONTROL input to own MiniDP manual control when MAN\_ENABLE is also enabled\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _AUTH_MAV_TGT:

AUTH\_MAV\_TGT: Allow MAVLink target authority
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Allows MAVLink mode requests to own MiniDP target modes including HEADING\_HOLD and DP\_HOLD\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _AUTH_TEST:

AUTH\_TEST: Allow actuator test authority
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Allows MAVLink motor test commands to enter MiniDP actuator test mode\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _AUTH_RC_TAKE:

AUTH\_RC\_TAKE: Allow RC takeover
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Allows healthy RC manual input to take over from MAVLink manual or target authority\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _AUTH_RC_FB:

AUTH\_RC\_FB: RC fallback on MAVLink loss
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Falls back to healthy RC authority when MAVLink manual or target authority is lost\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _AUTH_MAV_FB:

AUTH\_MAV\_FB: MAVLink fallback on RC loss
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Allows MAVLink manual authority to take over when RC authority is lost and MAVLink manual input is healthy\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _AUTH_TGT_LOSS:

AUTH\_TGT\_LOSS: Target link loss action
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Action when MAVLink target authority loses its link while MiniDP is in a target mode\.


+-------+------------------+
| Value | Meaning          |
+=======+==================+
| 0     | Failsafe         |
+-------+------------------+
| 1     | Hold last target |
+-------+------------------+
| 2     | Fall back to RC  |
+-------+------------------+




.. _AUTH_RC_TMO:

AUTH\_RC\_TMO: RC health timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


RC input is unhealthy if no fresh RC input arrives within this time\.


+-----------+-----------+---------+
| Increment | Range     | Units   |
+===========+===========+=========+
| 0.05      | 0.05 to 5 | seconds |
+-----------+-----------+---------+




.. _AUTH_MAV_TMO:

AUTH\_MAV\_TMO: MAVLink health timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


MAVLink is unhealthy if no GCS heartbeat or valid GCS traffic arrives within this time\.


+-----------+-----------+---------+
| Increment | Range     | Units   |
+===========+===========+=========+
| 0.1       | 0.1 to 30 | seconds |
+-----------+-----------+---------+




.. _FRAME_TYPE:

FRAME\_TYPE: MiniDP frame type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Selects the MiniDP output mixer frame\. Frame 901 is OMNI\_PLUS\: port and starboard propulsion screws plus bow and stern tunnel thrusters\.


+-------+-----------+
| Value | Meaning   |
+=======+===========+
| 901   | OMNI_PLUS |
+-------+-----------+




.. _FRAME_SCR_POS:

FRAME\_SCR\_POS: Propulsion screw position
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Position of the Motor1 and Motor2 propulsion screws relative to the yaw center for frame 901\. Aft preserves the default differential screw yaw sign\, center removes screw yaw contribution\, and forward flips the screw yaw sign\.


+-------+---------------+
| Value | Meaning       |
+=======+===============+
| -1    | Aft           |
+-------+---------------+
| 0     | Center no yaw |
+-------+---------------+
| 1     | Forward       |
+-------+---------------+




.. _FRAME_SCR_YAW:

FRAME\_SCR\_YAW: Propulsion screw yaw scale
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scales the Motor1 and Motor2 differential yaw contribution for frame 901\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 0.05      | 0 to 2 |
+-----------+--------+




.. _IN_RC_SURGE:

IN\_RC\_SURGE: RC surge channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


One\-based RC input channel for manual surge command\. Set to 0 to disable this axis\.


+---------+
| Range   |
+=========+
| 0 to 16 |
+---------+




.. _IN_RC_SWAY:

IN\_RC\_SWAY: RC sway channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


One\-based RC input channel for manual sway command\. Set to 0 to disable this axis\.


+---------+
| Range   |
+=========+
| 0 to 16 |
+---------+




.. _IN_RC_YAW:

IN\_RC\_YAW: RC yaw channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~


One\-based RC input channel for manual yaw command\. Set to 0 to disable this axis\.


+---------+
| Range   |
+=========+
| 0 to 16 |
+---------+




.. _IN_RC_KILL:

IN\_RC\_KILL: RC kill channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


One\-based RC input channel for output kill\. Set to 0 to disable RC kill\.


+---------+
| Range   |
+=========+
| 0 to 16 |
+---------+




.. _IN_KILL_PWM:

IN\_KILL\_PWM: RC kill threshold PWM
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


RC kill is active when IN\_RC\_KILL is at or above this PWM\.


+-----------+-------------+---------------------+
| Increment | Range       | Units               |
+===========+=============+=====================+
| 1         | 800 to 2200 | PWM in microseconds |
+-----------+-------------+---------------------+




.. _IN_RC_ARM:

IN\_RC\_ARM: RC arm switch channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


One\-based RC input channel for switch arming\. Set to 0 to disable RC switch arming\.


+---------+
| Range   |
+=========+
| 0 to 16 |
+---------+




.. _IN_ARM_PWM:

IN\_ARM\_PWM: RC arm threshold PWM
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


RC arm switch arms MiniDP on a rising edge when IN\_RC\_ARM is at or above this PWM\.


+-----------+-------------+---------------------+
| Increment | Range       | Units               |
+===========+=============+=====================+
| 1         | 800 to 2200 | PWM in microseconds |
+-----------+-------------+---------------------+




.. _IN_RC_DISARM:

IN\_RC\_DISARM: RC disarm switch channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


One\-based RC input channel for switch disarming\. Set to 0 to disable RC switch disarming\.


+---------+
| Range   |
+=========+
| 0 to 16 |
+---------+




.. _IN_DISARM_PWM:

IN\_DISARM\_PWM: RC disarm threshold PWM
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


RC disarm switch disarms MiniDP on a falling edge when IN\_RC\_DISARM is at or below this PWM\.


+-----------+-------------+---------------------+
| Increment | Range       | Units               |
+===========+=============+=====================+
| 1         | 800 to 2200 | PWM in microseconds |
+-----------+-------------+---------------------+




.. _MAN_ENABLE:

MAN\_ENABLE: Enable manual control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enables manual RC and authorized MAVLink manual output\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _MAN_DZ:

MAN\_DZ: Manual input deadband
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Deadband applied to manual RC and MAVLink manual axes before manual limits\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 0.01      | 0 to 1 |
+-----------+--------+




.. _MAN_SRG_LIM:

MAN\_SRG\_LIM: Manual surge limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum normalized manual surge output\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 0.01      | 0 to 1 |
+-----------+--------+




.. _MAN_SWY_LIM:

MAN\_SWY\_LIM: Manual sway limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum normalized manual sway output\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 0.01      | 0 to 1 |
+-----------+--------+




.. _MAN_YAW_LIM:

MAN\_YAW\_LIM: Manual yaw limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum normalized manual yaw output\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 0.01      | 0 to 1 |
+-----------+--------+




.. _MAN_MAV_TMO:

MAN\_MAV\_TMO: MAVLink manual timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Authorized MAVLink manual command times out after this many seconds without a fresh MANUAL\_CONTROL message\.


+-----------+----------+---------+
| Increment | Range    | Units   |
+===========+==========+=========+
| 0.05      | 0.1 to 5 | seconds |
+-----------+----------+---------+




.. _AXIS_DZ:

AXIS\_DZ: Output axis deadband
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Deadband applied after manual or DP controller output and before frame mixing\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 0.01      | 0 to 1 |
+-----------+--------+




.. _AXIS_SRG_MAX:

AXIS\_SRG\_MAX: Output surge limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Final normalized surge output limit after manual or DP controller output\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 0.01      | 0 to 1 |
+-----------+--------+




.. _AXIS_SWY_MAX:

AXIS\_SWY\_MAX: Output sway limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Final normalized sway output limit after manual or DP controller output\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 0.01      | 0 to 1 |
+-----------+--------+




.. _AXIS_YAW_MAX:

AXIS\_YAW\_MAX: Output yaw limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Final normalized yaw output limit after manual or DP controller output\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 0.01      | 0 to 1 |
+-----------+--------+




.. _AXIS_SRG_SLW:

AXIS\_SRG\_SLW: Output surge slew rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum surge command change rate in normalized output per second\. Set to 0 to disable slew limiting\.


+-----------+---------+------------+
| Increment | Range   | Units      |
+===========+=========+============+
| 0.1       | 0 to 10 | per second |
+-----------+---------+------------+




.. _AXIS_SWY_SLW:

AXIS\_SWY\_SLW: Output sway slew rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum sway command change rate in normalized output per second\. Set to 0 to disable slew limiting\.


+-----------+---------+------------+
| Increment | Range   | Units      |
+===========+=========+============+
| 0.1       | 0 to 10 | per second |
+-----------+---------+------------+




.. _AXIS_YAW_SLW:

AXIS\_YAW\_SLW: Output yaw slew rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum yaw command change rate in normalized output per second\. Set to 0 to disable slew limiting\.


+-----------+---------+------------+
| Increment | Range   | Units      |
+===========+=========+============+
| 0.1       | 0 to 10 | per second |
+-----------+---------+------------+




.. _DP_YAW_P:

DP\_YAW\_P: DP yaw proportional gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Heading hold proportional gain\. Multiplies yaw error in radians to produce normalized yaw output\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 0.01      | 0 to 5 |
+-----------+--------+




.. _DP_YAW_D:

DP\_YAW\_D: DP yaw damping gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Heading hold yaw\-rate damping gain\. Multiplies yaw rate in radians per second to reduce yaw output\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 0.01      | 0 to 5 |
+-----------+--------+




.. _DP_POS_P:

DP\_POS\_P: DP position proportional gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Position hold proportional gain\. Multiplies body\-frame position error in meters to produce surge and sway output\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 0.01      | 0 to 5 |
+-----------+--------+




.. _DP_VEL_D:

DP\_VEL\_D: DP velocity damping gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Position hold velocity damping gain\. Multiplies body\-frame velocity in meters per second to reduce surge and sway output\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 0.01      | 0 to 5 |
+-----------+--------+




.. _DP_SRG_MAX:

DP\_SRG\_MAX: DP surge limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum normalized surge output from the DP controller before the final AXIS limits\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 0.01      | 0 to 1 |
+-----------+--------+




.. _DP_SWY_MAX:

DP\_SWY\_MAX: DP sway limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum normalized sway output from the DP controller before the final AXIS limits\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 0.01      | 0 to 1 |
+-----------+--------+




.. _DP_YAW_MAX:

DP\_YAW\_MAX: DP yaw limit
~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum normalized yaw output from the DP controller before the final AXIS limits\.


+-----------+--------+
| Increment | Range  |
+===========+========+
| 0.01      | 0 to 1 |
+-----------+--------+




.. _DP_RETARGET:

DP\_RETARGET: Relatch DP target on mode request
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Relatches current position and heading as the target whenever DP\_HOLD is requested\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _DP_POS_RAD:

DP\_POS\_RAD: DP soft position radius
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Position error is gently shaped inside this radius before entering the position controller\.


+-----------+---------+--------+
| Increment | Range   | Units  |
+===========+=========+========+
| 0.1       | 0 to 50 | meters |
+-----------+---------+--------+




.. _DP_POS_DZ:

DP\_POS\_DZ: DP position deadband
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Position correction is zero inside this distance from the DP target\.


+-----------+---------+--------+
| Increment | Range   | Units  |
+===========+=========+========+
| 0.1       | 0 to 10 | meters |
+-----------+---------+--------+




.. _DP_HACC_MAX:

DP\_HACC\_MAX: DP maximum GPS horizontal accuracy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum GPS horizontal accuracy accepted for entering or continuing DP\_HOLD\. Set to 0 to disable this check\.


+-----------+----------+--------+
| Increment | Range    | Units  |
+===========+==========+========+
| 0.1       | 0 to 100 | meters |
+-----------+----------+--------+




.. _DP_SACC_MAX:

DP\_SACC\_MAX: DP maximum GPS speed accuracy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum GPS speed accuracy accepted for entering or continuing DP\_HOLD\. Set to 0 to disable this check\.


+-----------+---------+-------------------+
| Increment | Range   | Units             |
+===========+=========+===================+
| 0.1       | 0 to 20 | meters per second |
+-----------+---------+-------------------+




.. _ARMING_REQUIRE:

ARMING\_REQUIRE: Require arming for active outputs
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Requires MiniDP to be armed before active motor output\. Hardware safety can still block outputs\.


+-------+--------------+
| Value | Meaning      |
+=======+==============+
| 0     | Not required |
+-------+--------------+
| 1     | Required     |
+-------+--------------+




.. _ARM_GPS_REQ:

ARM\_GPS\_REQ: GPS arming requirement
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


GPS and EKF requirement for arming MiniDP\.


+-------+-------------------------------+
| Value | Meaning                       |
+=======+===============================+
| 0     | None                          |
+-------+-------------------------------+
| 1     | GPS fix and accuracy          |
+-------+-------------------------------+
| 2     | DP-ready EKF and GPS accuracy |
+-------+-------------------------------+




.. _ARM_HACC_MAX:

ARM\_HACC\_MAX: Arming maximum GPS horizontal accuracy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum GPS horizontal accuracy accepted for GPS\-required arming\. Set to 0 to disable this accuracy check\.


+-----------+----------+--------+
| Increment | Range    | Units  |
+===========+==========+========+
| 0.1       | 0 to 100 | meters |
+-----------+----------+--------+




.. _ARM_SACC_MAX:

ARM\_SACC\_MAX: Arming maximum GPS speed accuracy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum GPS speed accuracy accepted for GPS\-required arming\. Set to 0 to disable this accuracy check\.


+-----------+---------+-------------------+
| Increment | Range   | Units             |
+===========+=========+===================+
| 0.1       | 0 to 20 | meters per second |
+-----------+---------+-------------------+




.. _OUT_DARM_ACT:

OUT\_DARM\_ACT: Disarmed output action
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Output behavior for managed MiniDP motor channels when disarmed\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| 0     | Disable PWM       |
+-------+-------------------+
| 1     | Send neutral trim |
+-------+-------------------+
| 2     | Send OUT_FS_PWM   |
+-------+-------------------+




.. _OUT_FS_ACT:

OUT\_FS\_ACT: Failsafe output action
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Output behavior for managed MiniDP motor channels in failsafe\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| 0     | Disable PWM       |
+-------+-------------------+
| 1     | Send neutral trim |
+-------+-------------------+
| 2     | Send OUT_FS_PWM   |
+-------+-------------------+




.. _OUT_KILL_ACT:

OUT\_KILL\_ACT: RC kill output action
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Output behavior for managed MiniDP motor channels when RC kill is active\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| 0     | Disable PWM       |
+-------+-------------------+
| 1     | Send neutral trim |
+-------+-------------------+
| 2     | Send OUT_FS_PWM   |
+-------+-------------------+




.. _OUT_FS_PWM:

OUT\_FS\_PWM: Failsafe PWM
~~~~~~~~~~~~~~~~~~~~~~~~~~


PWM used when an output safe action is set to send failsafe PWM\. Set to 0 to use each motor channel trim\.


+-----------+-----------+---------------------+
| Increment | Range     | Units               |
+===========+===========+=====================+
| 1         | 0 to 2200 | PWM in microseconds |
+-----------+-----------+---------------------+





.. _parameters_Lua Script:

Lua Script Parameters
---------------------


.. _DR_ENABLE:

DR\_ENABLE: Deadreckoning Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Deadreckoning Enable


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _DR_ENABLE_DIST:

DR\_ENABLE\_DIST: Deadreckoning Enable Distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Distance from home \(in meters\) beyond which the dead reckoning will be enabled


+--------+
| Units  |
+========+
| meters |
+--------+




.. _DR_GPS_SACC_MAX:

DR\_GPS\_SACC\_MAX: Deadreckoning GPS speed accuracy maximum threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


GPS speed accuracy maximum\, above which deadreckoning home will begin \(default is 0\.8\)\.  Lower values trigger with good GPS quality\, higher values will allow poorer GPS before triggering\. Set to 0 to disable use of GPS speed accuracy


+---------+
| Range   |
+=========+
| 0 to 10 |
+---------+




.. _DR_GPS_SAT_MIN:

DR\_GPS\_SAT\_MIN: Deadreckoning GPS satellite count min threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


GPS satellite count threshold below which deadreckoning home will begin \(default is 6\)\.  Higher values trigger with good GPS quality\, Lower values trigger with worse GPS quality\. Set to 0 to disable use of GPS satellite count


+---------+
| Range   |
+=========+
| 0 to 30 |
+---------+




.. _DR_GPS_TRIGG_SEC:

DR\_GPS\_TRIGG\_SEC: Deadreckoning GPS check trigger seconds
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


GPS checks must fail for this many seconds before dead reckoning will be triggered


+---------+
| Units   |
+=========+
| seconds |
+---------+




.. _DR_FLY_ANGLE:

DR\_FLY\_ANGLE: Deadreckoning Lean Angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


lean angle \(in degrees\) during deadreckoning


+---------+---------+
| Range   | Units   |
+=========+=========+
| 0 to 45 | degrees |
+---------+---------+




.. _DR_FLY_ALT_MIN:

DR\_FLY\_ALT\_MIN: Deadreckoning Altitude Min
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Copter will fly at at least this altitude \(in meters\) above home during deadreckoning


+-----------+--------+
| Range     | Units  |
+===========+========+
| 0 to 1000 | meters |
+-----------+--------+




.. _DR_FLY_TIMEOUT:

DR\_FLY\_TIMEOUT: Deadreckoning flight timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Copter will attempt to switch to NEXT\_MODE after this many seconds of deadreckoning\.  If it cannot switch modes it will continue in Guided\_NoGPS\.  Set to 0 to disable timeout


+---------+
| Units   |
+=========+
| seconds |
+---------+




.. _DR_NEXT_MODE:

DR\_NEXT\_MODE: Deadreckoning Next Mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Copter switch to this mode after GPS recovers or DR\_FLY\_TIMEOUT has elapsed\.  Default is 6\/RTL\.  Set to \-1 to return to mode used before deadreckoning was triggered


+-------+--------------+
| Value | Meaning      |
+=======+==============+
| 2     | AltHold      |
+-------+--------------+
| 3     | Auto         |
+-------+--------------+
| 4     | Guided       |
+-------+--------------+
| 5     | Loiter       |
+-------+--------------+
| 6     | RTL          |
+-------+--------------+
| 7     | Circle       |
+-------+--------------+
| 9     | Land         |
+-------+--------------+
| 16    | PosHold      |
+-------+--------------+
| 17    | Brake        |
+-------+--------------+
| 20    | Guided_NoGPS |
+-------+--------------+
| 21    | Smart_RTL    |
+-------+--------------+
| 27    | Auto RTL     |
+-------+--------------+




.. _RTUN_ENABLE:

RTUN\_ENABLE: Rover Quicktune enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable quicktune system


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _RTUN_AXES:

RTUN\_AXES: Rover Quicktune axes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


axes to tune


+-----+----------+
| Bit | Meaning  |
+=====+==========+
| 0   | Steering |
+-----+----------+
| 1   | Speed    |
+-----+----------+




.. _RTUN_STR_FFRATIO:

RTUN\_STR\_FFRATIO: Rover Quicktune Steering Rate FeedForward ratio
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Ratio between measured response and FF gain\. Raise this to get a higher FF gain


+----------+
| Range    |
+==========+
| 0 to 1.0 |
+----------+




.. _RTUN_STR_P_RATIO:

RTUN\_STR\_P\_RATIO: Rover Quicktune Steering FF to P ratio
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Ratio between steering FF and P gains\. Raise this to get a higher P gain\, 0 to leave P unchanged


+----------+
| Range    |
+==========+
| 0 to 2.0 |
+----------+




.. _RTUN_STR_I_RATIO:

RTUN\_STR\_I\_RATIO: Rover Quicktune Steering FF to I ratio
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Ratio between steering FF and I gains\. Raise this to get a higher I gain\, 0 to leave I unchanged


+----------+
| Range    |
+==========+
| 0 to 2.0 |
+----------+




.. _RTUN_SPD_FFRATIO:

RTUN\_SPD\_FFRATIO: Rover Quicktune Speed FeedForward \(equivalent\) ratio
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Ratio between measured response and CRUISE\_THROTTLE value\. Raise this to get a higher CRUISE\_THROTTLE value


+----------+
| Range    |
+==========+
| 0 to 1.0 |
+----------+




.. _RTUN_SPD_P_RATIO:

RTUN\_SPD\_P\_RATIO: Rover Quicktune Speed FF to P ratio
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Ratio between speed FF and P gain\. Raise this to get a higher P gain\, 0 to leave P unchanged


+----------+
| Range    |
+==========+
| 0 to 2.0 |
+----------+




.. _RTUN_SPD_I_RATIO:

RTUN\_SPD\_I\_RATIO: Rover Quicktune Speed FF to I ratio
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Ratio between speed FF and I gain\. Raise this to get a higher I gain\, 0 to leave I unchanged


+----------+
| Range    |
+==========+
| 0 to 2.0 |
+----------+




.. _RTUN_AUTO_FILTER:

RTUN\_AUTO\_FILTER: Rover Quicktune auto filter enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


When enabled the PID filter settings are automatically set based on INS\_GYRO\_FILTER


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _RTUN_AUTO_SAVE:

RTUN\_AUTO\_SAVE: Rover Quicktune auto save
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of seconds after completion of tune to auto\-save\. This is useful when using a 2 position switch for quicktune


+---------+
| Units   |
+=========+
| seconds |
+---------+




.. _RTUN_RC_FUNC:

RTUN\_RC\_FUNC: Rover Quicktune RC function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


RCn\_OPTION number to use to control tuning stop\/start\/save


+-------+------------+
| Value | Meaning    |
+=======+============+
| 300   | Scripting1 |
+-------+------------+
| 301   | Scripting2 |
+-------+------------+
| 302   | Scripting3 |
+-------+------------+
| 303   | Scripting4 |
+-------+------------+
| 304   | Scripting5 |
+-------+------------+
| 305   | Scripting6 |
+-------+------------+
| 306   | Scripting7 |
+-------+------------+
| 307   | Scripting8 |
+-------+------------+




.. _RTUN_SPEED_MIN:

RTUN\_SPEED\_MIN: Rover Quicktune minimum speed for tuning
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The mimimum speed in m\/s required for tuning to start


+------------+-------------------+
| Range      | Units             |
+============+===================+
| 0.1 to 0.5 | meters per second |
+------------+-------------------+




.. _FOLT_ENABLE:

FOLT\_ENABLE: Follow Target Send Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Follow Target Send Enable


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _FOLT_MAV_CHAN:

FOLT\_MAV\_CHAN: Follow Target Send MAVLink Channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


MAVLink channel to which FOLLOW\_TARGET should be sent


+---------+
| Range   |
+=========+
| 0 to 10 |
+---------+




.. _QUIK_ENABLE:

QUIK\_ENABLE: Quicktune enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable quicktune system


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _QUIK_AXES:

QUIK\_AXES: Quicktune axes
~~~~~~~~~~~~~~~~~~~~~~~~~~


axes to tune


+-----+---------+
| Bit | Meaning |
+=====+=========+
| 0   | Roll    |
+-----+---------+
| 1   | Pitch   |
+-----+---------+
| 2   | Yaw     |
+-----+---------+




.. _QUIK_DOUBLE_TIME:

QUIK\_DOUBLE\_TIME: Quicktune doubling time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Time to double a tuning parameter\. Raise this for a slower tune\.


+---------+---------+
| Range   | Units   |
+=========+=========+
| 5 to 20 | seconds |
+---------+---------+




.. _QUIK_GAIN_MARGIN:

QUIK\_GAIN\_MARGIN: Quicktune gain margin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reduction in gain after oscillation detected\. Raise this number to get a more conservative tune


+----------+---------+
| Range    | Units   |
+==========+=========+
| 20 to 80 | percent |
+----------+---------+




.. _QUIK_OSC_SMAX:

QUIK\_OSC\_SMAX: Quicktune oscillation rate threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Threshold for oscillation detection\. A lower value will lead to a more conservative tune\.


+---------+
| Range   |
+=========+
| 1 to 10 |
+---------+




.. _QUIK_YAW_P_MAX:

QUIK\_YAW\_P\_MAX: Quicktune Yaw P max
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum value for yaw P gain


+----------+
| Range    |
+==========+
| 0.1 to 3 |
+----------+




.. _QUIK_YAW_D_MAX:

QUIK\_YAW\_D\_MAX: Quicktune Yaw D max
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum value for yaw D gain


+------------+
| Range      |
+============+
| 0.001 to 1 |
+------------+




.. _QUIK_RP_PI_RATIO:

QUIK\_RP\_PI\_RATIO: Quicktune roll\/pitch PI ratio
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Ratio between P and I gains for roll and pitch\. Raise this to get a lower I gain


+------------+
| Range      |
+============+
| 0.5 to 1.0 |
+------------+




.. _QUIK_Y_PI_RATIO:

QUIK\_Y\_PI\_RATIO: Quicktune Yaw PI ratio
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Ratio between P and I gains for yaw\. Raise this to get a lower I gain


+-----------+
| Range     |
+===========+
| 0.5 to 20 |
+-----------+




.. _QUIK_AUTO_FILTER:

QUIK\_AUTO\_FILTER: Quicktune auto filter enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


When enabled the PID filter settings are automatically set based on INS\_GYRO\_FILTER


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _QUIK_AUTO_SAVE:

QUIK\_AUTO\_SAVE: Quicktune auto save
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of seconds after completion of tune to auto\-save\. This is useful when using a 2 position switch for quicktune


+---------+
| Units   |
+=========+
| seconds |
+---------+




.. _QUIK_RC_FUNC:

QUIK\_RC\_FUNC: Quicktune RC function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


RCn\_OPTION number to use to control tuning stop\/start\/save


.. _QUIK_MAX_REDUCE:

QUIK\_MAX\_REDUCE: Quicktune maximum gain reduction
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls how much quicktune is allowed to lower gains from the original gains\. If the vehicle already has a reasonable tune and is not oscillating then you can set this to zero to prevent gain reductions\. The default of 20\% is reasonable for most vehicles\. Using a maximum gain reduction lowers the chance of an angle P oscillation happening if quicktune gets a false positive oscillation at a low gain\, which can result in very low rate gains and a dangerous angle P oscillation\.


+----------+---------+
| Range    | Units   |
+==========+=========+
| 0 to 100 | percent |
+----------+---------+




.. _QUIK_OPTIONS:

QUIK\_OPTIONS: Quicktune options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Additional options\. When the Two Position Switch option is enabled then a high switch position will start the tune\, low will disable the tune\. you should also set a QUIK\_AUTO\_SAVE time so that you will be able to save the tune\.


+-----+----------------------+
| Bit | Meaning              |
+=====+======================+
| 0   | UseTwoPositionSwitch |
+-----+----------------------+




.. _QUIK_ANGLE_MAX:

QUIK\_ANGLE\_MAX: maximum angle error for tune abort
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


If while tuning the angle error goes over this limit then the tune will aborts to prevent a bad oscillation in the case of the tuning algorithm failing\. If you get an error \"Tuning\: attitude error ABORTING\" and you think it is a false positive then you can either raise this parameter or you can try increasing the QUIK\_DOUBLE\_TIME to do the tune more slowly\. A value of zero disables this check\.


+---------+
| Units   |
+=========+
| degrees |
+---------+




.. _ESRC_EXTN_THRESH:

ESRC\_EXTN\_THRESH: EKF Source ExternalNav Innovation Threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


ExternalNav may be used if innovations are below this threshold


+--------+
| Range  |
+========+
| 0 to 1 |
+--------+




.. _ESRC_EXTN_QUAL:

ESRC\_EXTN\_QUAL: EKF Source ExternalNav Quality Threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


ExternalNav may be used if quality is above this threshold


+----------+---------+
| Range    | Units   |
+==========+=========+
| 0 to 100 | percent |
+----------+---------+




.. _ESRC_FLOW_THRESH:

ESRC\_FLOW\_THRESH: EKF Source OpticalFlow Innovation Threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


OpticalFlow may be used if innovations are below this threshold


+--------+
| Range  |
+========+
| 0 to 1 |
+--------+




.. _ESRC_FLOW_QUAL:

ESRC\_FLOW\_QUAL: EKF Source OpticalFlow Quality Threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


OpticalFlow may be used if quality is above this threshold


+----------+---------+
| Range    | Units   |
+==========+=========+
| 0 to 100 | percent |
+----------+---------+




.. _ESRC_RNGFND_MAX:

ESRC\_RNGFND\_MAX: EKF Source Rangefinder Max
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


OpticalFlow may be used if rangefinder distance is below this threshold


+---------+--------+
| Range   | Units  |
+=========+========+
| 0 to 50 | meters |
+---------+--------+




.. _ALAND_ENABLE:

ALAND\_ENABLE: Auto land enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


enable Auto land script action


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _ALAND_WP_ALT:

ALAND\_WP\_ALT: Final approach waypoint alt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Altitude of final approach waypoint created by script


+----------+--------+
| Range    | Units  |
+==========+========+
| 1 to 100 | meters |
+----------+--------+




.. _ALAND_WP_DIST:

ALAND\_WP\_DIST: Final approach waypoint distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Distance from landing point \(HOME\) to final approach waypoint created by script in the opposite direction of initial takeoff


+-----------+--------+
| Range     | Units  |
+===========+========+
| 0 to 1000 | meters |
+-----------+--------+




.. _AHRS_ORIG_LAT:

AHRS\_ORIG\_LAT: AHRS\/EKF Origin Latitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


AHRS\/EKF origin will be set to this latitude if not already set


+-------------+
| Range       |
+=============+
| -180 to 180 |
+-------------+




.. _AHRS_ORIG_LON:

AHRS\_ORIG\_LON: AHRS\/EKF Origin Longitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


AHRS\/EKF origin will be set to this longitude if not already set


+-------------+
| Range       |
+=============+
| -180 to 180 |
+-------------+




.. _AHRS_ORIG_ALT:

AHRS\_ORIG\_ALT: AHRS\/EKF Origin Altitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


AHRS\/EKF origin will be set to this altitude \(in meters above sea level\) if not already set


+------------+
| Range      |
+============+
| 0 to 10000 |
+------------+




.. _BTAG_ENABLE:

BTAG\_ENABLE: enable battery info support
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


enable battery info support


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _BTAG_MAX_CYCLES:

BTAG\_MAX\_CYCLES: max battery cycles
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


max battery cycles for arming


+------------+
| Range      |
+============+
| 0 to 10000 |
+------------+




.. _BTAG_CUR_CYCLES:

BTAG\_CUR\_CYCLES: current battery cycles
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

this is the highest value for battery cycles for all connected batteries


+------------+
| Range      |
+============+
| 0 to 10000 |
+------------+




.. _VID1_CAMMODEL:

VID1\_CAMMODEL: Camera1 Video Stream Camera Model
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video stream camera model


+-------+------------------------+
| Value | Meaning                |
+=======+========================+
| 0     | Unknown                |
+-------+------------------------+
| 1     | Siyi A8                |
+-------+------------------------+
| 2     | Siyi ZR10              |
+-------+------------------------+
| 3     | Siyi ZR30              |
+-------+------------------------+
| 4     | Siyi ZT30 Zoom         |
+-------+------------------------+
| 5     | Siyi ZT30 Wide         |
+-------+------------------------+
| 6     | Siyi ZT30 IR           |
+-------+------------------------+
| 7     | Siyi ZT6 RGB           |
+-------+------------------------+
| 8     | Siyi ZT6 IR            |
+-------+------------------------+
| 9     | Herelink WifiAP        |
+-------+------------------------+
| 10    | Herelink USB-tethering |
+-------+------------------------+
| 11    | Topotek 1080p          |
+-------+------------------------+
| 12    | Topotek 480p           |
+-------+------------------------+
| 13    | Viewpro                |
+-------+------------------------+




.. _VID1_ID:

VID1\_ID: Camera1 Video Stream Id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video stream id


+---------+
| Range   |
+=========+
| 0 to 50 |
+---------+




.. _VID1_TYPE:

VID1\_TYPE: Camera1 Video Stream Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video stream type


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | RTSP     |
+-------+----------+
| 1     | RTPUDP   |
+-------+----------+
| 2     | TCP_MPEG |
+-------+----------+
| 3     | MPEG_TS  |
+-------+----------+




.. _VID1_FLAG:

VID1\_FLAG: Camera1 Video Stream Flags
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video stream flags


+-----+-----------------------+
| Bit | Meaning               |
+=====+=======================+
| 0   | Running               |
+-----+-----------------------+
| 1   | Thermal               |
+-----+-----------------------+
| 2   | Thermal Range Enabled |
+-----+-----------------------+




.. _VID1_FRAME_RATE:

VID1\_FRAME\_RATE: Camera1 Video Stream Frame Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video stream frame rate


+---------+
| Range   |
+=========+
| 0 to 50 |
+---------+




.. _VID1_HRES:

VID1\_HRES: Camera1 Video Stream Horizontal Resolution
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video stream horizontal resolution


+-----------+
| Range     |
+===========+
| 0 to 4096 |
+-----------+




.. _VID1_VRES:

VID1\_VRES: Camera1 Video Stream Vertical Resolution
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video stream vertical resolution


+-----------+
| Range     |
+===========+
| 0 to 4096 |
+-----------+




.. _VID1_BITRATE:

VID1\_BITRATE: Camera1 Video Stream Bitrate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video stream bitrate


+------------+
| Range      |
+============+
| 0 to 10000 |
+------------+




.. _VID1_HFOV:

VID1\_HFOV: Camera1 Video Stream Horizontal FOV
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video stream horizontal FOV


+----------+
| Range    |
+==========+
| 0 to 360 |
+----------+




.. _VID1_ENCODING:

VID1\_ENCODING: Camera1 Video Stream Encoding
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video stream encoding


+-------+---------+
| Value | Meaning |
+=======+=========+
| 0     | Unknown |
+-------+---------+
| 1     | H264    |
+-------+---------+
| 2     | H265    |
+-------+---------+




.. _VID1_IPADDR0:

VID1\_IPADDR0: Camera1 Video Stream IP Address 0
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video stream IP Address first octet


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _VID1_IPADDR1:

VID1\_IPADDR1: Camera1 Video Stream IP Address 1
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video stream IP Address second octet


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _VID1_IPADDR2:

VID1\_IPADDR2: Camera1 Video Stream IP Address 2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video stream IP Address third octet


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _VID1_IPADDR3:

VID1\_IPADDR3: Camera1 Video Stream IP Address 3
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video stream IP Address fourth octet


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _VID1_IPPORT:

VID1\_IPPORT: Camera1 Video Stream IP Address Port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video stream IP Address Port


+------------+
| Range      |
+============+
| 0 to 65535 |
+------------+




.. _WINCH_RATE_UP:

WINCH\_RATE\_UP: WinchControl Rate Up
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum rate when retracting line


+------------+
| Range      |
+============+
| 0.1 to 5.0 |
+------------+




.. _WINCH_RATE_DN:

WINCH\_RATE\_DN: WinchControl Rate Down
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum rate when releasing line


+------------+
| Range      |
+============+
| 0.1 to 5.0 |
+------------+




.. _WINCH_RC_FUNC:

WINCH\_RC\_FUNC: Winch Rate Control RC function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


RCn\_OPTION number to use to control winch rate


+-------+------------+
| Value | Meaning    |
+=======+============+
| 300   | Scripting1 |
+-------+------------+
| 301   | Scripting2 |
+-------+------------+
| 302   | Scripting3 |
+-------+------------+
| 303   | Scripting4 |
+-------+------------+
| 304   | Scripting5 |
+-------+------------+
| 305   | Scripting6 |
+-------+------------+
| 306   | Scripting7 |
+-------+------------+
| 307   | Scripting8 |
+-------+------------+




.. _POI_DIST_MAX:

POI\_DIST\_MAX: Mount POI distance max
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


POI\'s max distance \(in meters\) from the vehicle


+------------+
| Range      |
+============+
| 0 to 10000 |
+------------+




.. _BATT_SOC_COUNT:

BATT\_SOC\_COUNT: Count of SOC estimators
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of battery SOC estimators


+--------+
| Range  |
+========+
| 0 to 4 |
+--------+




.. _BATT_SOC1_IDX:

BATT\_SOC1\_IDX: Battery estimator index
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator index


+--------+
| Range  |
+========+
| 0 to 4 |
+--------+




.. _BATT_SOC1_NCELL:

BATT\_SOC1\_NCELL: Battery estimator cell count
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator cell count


+---------+
| Range   |
+=========+
| 0 to 48 |
+---------+




.. _BATT_SOC1_C1:

BATT\_SOC1\_C1: Battery estimator coefficient1
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator coefficient1


+------------+
| Range      |
+============+
| 100 to 200 |
+------------+




.. _BATT_SOC1_C2:

BATT\_SOC1\_C2: Battery estimator coefficient2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator coefficient2


+--------+
| Range  |
+========+
| 2 to 5 |
+--------+




.. _BATT_SOC1_C3:

BATT\_SOC1\_C3: Battery estimator coefficient3
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator coefficient3


+-------------+
| Range       |
+=============+
| 0.01 to 0.5 |
+-------------+




.. _BATT_SOC1_C4:

BATT\_SOC1\_C4: Battery estimator coefficient4
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator coefficient4


+----------+
| Range    |
+==========+
| 5 to 100 |
+----------+




.. _BATT_SOC2_IDX:

BATT\_SOC2\_IDX: Battery estimator index
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator index


+--------+
| Range  |
+========+
| 0 to 4 |
+--------+




.. _BATT_SOC2_NCELL:

BATT\_SOC2\_NCELL: Battery estimator cell count
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator cell count


+---------+
| Range   |
+=========+
| 0 to 48 |
+---------+




.. _BATT_SOC2_C1:

BATT\_SOC2\_C1: Battery estimator coefficient1
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator coefficient1


+------------+
| Range      |
+============+
| 100 to 200 |
+------------+




.. _BATT_SOC2_C2:

BATT\_SOC2\_C2: Battery estimator coefficient2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator coefficient2


+--------+
| Range  |
+========+
| 2 to 5 |
+--------+




.. _BATT_SOC2_C3:

BATT\_SOC2\_C3: Battery estimator coefficient3
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator coefficient3


+-------------+
| Range       |
+=============+
| 0.01 to 0.5 |
+-------------+




.. _BATT_SOC2_C4:

BATT\_SOC2\_C4: Battery estimator coefficient4
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator coefficient4


+----------+
| Range    |
+==========+
| 5 to 100 |
+----------+




.. _BATT_SOC3_IDX:

BATT\_SOC3\_IDX: Battery estimator index
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator index


+--------+
| Range  |
+========+
| 0 to 4 |
+--------+




.. _BATT_SOC3_NCELL:

BATT\_SOC3\_NCELL: Battery estimator cell count
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator cell count


+---------+
| Range   |
+=========+
| 0 to 48 |
+---------+




.. _BATT_SOC3_C1:

BATT\_SOC3\_C1: Battery estimator coefficient1
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator coefficient1


+------------+
| Range      |
+============+
| 100 to 200 |
+------------+




.. _BATT_SOC3_C2:

BATT\_SOC3\_C2: Battery estimator coefficient2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator coefficient2


+--------+
| Range  |
+========+
| 2 to 5 |
+--------+




.. _BATT_SOC3_C3:

BATT\_SOC3\_C3: Battery estimator coefficient3
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator coefficient3


+-------------+
| Range       |
+=============+
| 0.01 to 0.5 |
+-------------+




.. _BATT_SOC3_C4:

BATT\_SOC3\_C4: Battery estimator coefficient4
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator coefficient4


+----------+
| Range    |
+==========+
| 5 to 100 |
+----------+




.. _BATT_SOC4_IDX:

BATT\_SOC4\_IDX: Battery estimator index
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator index


+--------+
| Range  |
+========+
| 0 to 4 |
+--------+




.. _BATT_SOC4_NCELL:

BATT\_SOC4\_NCELL: Battery estimator cell count
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator cell count


+---------+
| Range   |
+=========+
| 0 to 48 |
+---------+




.. _BATT_SOC4_C1:

BATT\_SOC4\_C1: Battery estimator coefficient1
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator coefficient1


+------------+
| Range      |
+============+
| 100 to 200 |
+------------+




.. _BATT_SOC4_C2:

BATT\_SOC4\_C2: Battery estimator coefficient2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator coefficient2


+--------+
| Range  |
+========+
| 2 to 5 |
+--------+




.. _BATT_SOC4_C3:

BATT\_SOC4\_C3: Battery estimator coefficient3
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator coefficient3


+-------------+
| Range       |
+=============+
| 0.01 to 0.5 |
+-------------+




.. _BATT_SOC4_C4:

BATT\_SOC4\_C4: Battery estimator coefficient4
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Battery estimator coefficient4


+----------+
| Range    |
+==========+
| 5 to 100 |
+----------+




.. _ARM_SYSID:

ARM\_SYSID: MAV\_SYSID must be set
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Check that MAV\_SYSID \(or SYDID\_THISMAV\) has been set\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_FOLL_SYSID:

ARM\_FOLL\_SYSID: FOLL\_SYSID must be set
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


If FOLL\_ENABLE \= 1\, check that FOLL\_SYSID has been set\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_FOLL_SYSID_X:

ARM\_FOLL\_SYSID\_X: Vehicle should not follow itself
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


If FOLL\_ENABLE \= 1\, check that FOLL\_SYSID is different to MAV\_SYSID\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_FOLL_OFS_DEF:

ARM\_FOLL\_OFS\_DEF: Follow Offsets defaulted
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Follow offsets should not be left as default \(zero\) if FOLL\_ENABLE \= 1\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_MNTX_SYSID:

ARM\_MNTX\_SYSID: Follow and Mount should follow the same vehicle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


If FOLL\_ENABLE \= 1 and MNTx\_SYSID\_DEFLT is set\, check that FOLL\_SYSID is equal MNTx\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_RTL_CLIMB:

ARM\_RTL\_CLIMB: RTL\_CLIMB\_MIN should be a valid value
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


RTL\_CLIMB\_MIN should be \< 120m \(400ft\)\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_ESTOP:

ARM\_ESTOP: Motors EStopped
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Emergency Stop disables arming\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_FENCE:

ARM\_FENCE: Fence not enabled
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Fences loaded but no fence enabled\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_RALLY:

ARM\_RALLY: Rally too far
~~~~~~~~~~~~~~~~~~~~~~~~~


Rally Point more than RALLY\_LIMIT\_KM kilometers away\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_C_RTL_ALT:

ARM\_C\_RTL\_ALT: RTL\_ALT should be a valid value
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


RTL\_ALT should be \< 120m \(400ft\)\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_P_Q_FS_LAND:

ARM\_P\_Q\_FS\_LAND: Warn if Q failsafe will land
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Notify the user that on failsafe a QuadPlan will land\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_P_Q_FS_RTL:

ARM\_P\_Q\_FS\_RTL: Warn if Q failsafe will QRTL
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Notify the user that on failsafe a QuadPlan will QRTL\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_P_AIRSPEED:

ARM\_P\_AIRSPEED: Check AIRSPEED\_ parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Validate that AIRSPEED\_STALL\(if set\) \< MIN \< CRUISE \< MAX d\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_P_STALL:

ARM\_P\_STALL: AIRSPEED\_MIN should be 25\% above STALL
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Validate that AIRSPEED\_MIN is at least 25\% above AIRSPEED\_STALL\(if set\)\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_P_SCALING:

ARM\_P\_SCALING: SCALING\_SPEED valid
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Validate that SCALING\_SPEED is within 20\% of AIRSPEED\_CRUISE\. If SCALING\_SPEED changes the vehicle may need to be retuned\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_P_RTL_ALT:

ARM\_P\_RTL\_ALT: RTL\_ALTITUDE should be a valid value
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


RTL\_ALTITITUDE should be \< 120m \(400ft\)\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_P_QRTL_ALT:

ARM\_P\_QRTL\_ALT: Q\_RTL\_ALT should be a valid value
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Q\_RTL\_ALT should be \< 120m \(400ft\)\. 3 or less to prevent arming\. \-1 to disable\.


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| -1    | Disabled          |
+-------+-------------------+
| 0     | Emergency(PreArm) |
+-------+-------------------+
| 1     | Alert(PreArm)     |
+-------+-------------------+
| 2     | Critical(PreArm)  |
+-------+-------------------+
| 3     | Error(PreArm)     |
+-------+-------------------+
| 4     | Warning           |
+-------+-------------------+
| 5     | Notice            |
+-------+-------------------+
| 6     | Info              |
+-------+-------------------+
| 7     | Debug             |
+-------+-------------------+




.. _ARM_V_ALT_LEGAL:

ARM\_V\_ALT\_LEGAL: Legal max altitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Legal max altitude for UAV\/RPAS\/drones in your jurisdiction


+--------+
| Units  |
+========+
| meters |
+--------+




.. _PREV_ENABLE:

PREV\_ENABLE: parameter reversion enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable parameter reversion system


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _PREV_RC_FUNC:

PREV\_RC\_FUNC: param reversion RC function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


RCn\_OPTION number to used to trigger parameter reversion


.. _PLND_ALT_CUTOFF:

PLND\_ALT\_CUTOFF: Precland altitude cutoff
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The altitude \(rangefinder distance\) below which we stop using the precision landing sensor and continue landing


+---------+--------+
| Range   | Units  |
+=========+========+
| 0 to 20 | meters |
+---------+--------+




.. _DIST_CUTOFF:

DIST\_CUTOFF: Precland distance cutoff
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The distance from target beyond which the target is ignored


+----------+--------+
| Range    | Units  |
+==========+========+
| 0 to 100 | meters |
+----------+--------+




.. _TA_ACT_FN:

TA\_ACT\_FN: Activation Function for Terrain Avoidance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting an RC channel\'s \_OPTION to this value will use it for Terrain Avoidance enable\/disable


+------------+
| Range      |
+============+
| 300 to 307 |
+------------+




.. _TA_PTCH_DWN_MIN:

TA\_PTCH\_DWN\_MIN: down distance minimum for Pitching
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


If the downward distance is less than this value then start Pitching up to gain altitude\.


+--------+
| Units  |
+========+
| meters |
+--------+




.. _TA_PTCH_FWD_MIN:

TA\_PTCH\_FWD\_MIN: forward distance minimum for Pitching
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


If the farwardward distance is less than this value then start Pitching up to gain altitude\.


+--------+
| Units  |
+========+
| meters |
+--------+




.. _TA_QUAD_DWN_MIN:

TA\_QUAD\_DWN\_MIN: Downward distance minimum Quading
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


If the downward distance is less than this value then start Quading up to gain altitude\.


+--------+
| Units  |
+========+
| meters |
+--------+




.. _TA_QUAD_FWD_MIN:

TA\_QUAD\_FWD\_MIN: minimum forward distance for Quading
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


If the farwardward distance is less than this value then start Quading up to gain altitude\.


+--------+
| Units  |
+========+
| meters |
+--------+




.. _TA_PTCH_GSP_MIN:

TA\_PTCH\_GSP\_MIN: minimum ground speed for Pitching
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum Groundspeed \(not airspeed\) to be flying for Pitching to be used\.


+-------------------+
| Units             |
+===================+
| meters per second |
+-------------------+




.. _TA_PTCH_TIMEOUT:

TA\_PTCH\_TIMEOUT: timeout Pitching
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Minimum down or forward distance must be triggered for more than this many seconds to start Pitching


+---------+
| Units   |
+=========+
| seconds |
+---------+




.. _TA_HOME_DIST:

TA\_HOME\_DIST: safe distance around home
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Terrain avoidance will not be applied if the vehicle is less than this distance from home


+--------+
| Units  |
+========+
| meters |
+--------+




.. _TA_ALT_MAX:

TA\_ALT\_MAX: ceiling for pitching\/quading
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is a limit on how high the terrain avoidane will take the vehicle\. It acts a failsafe to prevent vertical flyaways\.


+------------+--------+
| Range      | Units  |
+============+========+
| 20 to 1000 | meters |
+------------+--------+




.. _TA_GSP_MAX:

TA\_GSP\_MAX: Maximum Groundspeed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is a limit on how fast in groundspeeed terrain avoidance will take the vehicle\. This is to allow for reliable sensor readings\. \-1 for disabled\.


+----------+-------------------+
| Range    | Units             |
+==========+===================+
| 10 to 40 | meters per second |
+----------+-------------------+




.. _TA_GSP_AIRBRAKE:

TA\_GSP\_AIRBRAKE: Groudspeed Airbrake limt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the limit for triggering airbrake to slow groundspeed as a difference between the airspeed and groundspeed\. \-1 for disabled\.


+-----------+-------------------+
| Range     | Units             |
+===========+===================+
| -1 to -10 | meters per second |
+-----------+-------------------+




.. _TA_CMTC_HGT:

TA\_CMTC\_HGT: CMTC Height
~~~~~~~~~~~~~~~~~~~~~~~~~~


The minimum Height above terrain to maintain when following an AUTO mission or RTL\. If zero\(0\) use TA\_PTCH\_DOW\_MIN\.


+--------+
| Units  |
+========+
| meters |
+--------+




.. _TA_CMTC_ENABLE:

TA\_CMTC\_ENABLE: CMTC Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Whether to enable Can\'t Make That Climb while running Terrain Avoidance


+--------+
| Range  |
+========+
| 0 to 1 |
+--------+




.. _TA_UPDATE_RATE:

TA\_UPDATE\_RATE: Frequency to process avoidance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Avoidance processing rate


+-------+
| Units |
+=======+
| hertz |
+-------+




.. _TA_CMTC_RAD:

TA\_CMTC\_RAD: CMTC loiter radius
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Use this radius for the loiter when trying to gain altitude\. If not set or \<\=0 use WP\_LOITER\_RAD


+--------+
| Units  |
+========+
| meters |
+--------+




.. _CGA_RATIO:

CGA\_RATIO: CoG adjustment ratio
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The ratio between the front and back motor outputs during steady\-state hover\. Positive when the CoG is in front of the motors midpoint \(front motors work harder\)\.


+----------+
| Range    |
+==========+
| 0.5 to 2 |
+----------+




.. _PARAM_SET_ENABLE:

PARAM\_SET\_ENABLE: Param Set enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Param Set enable


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _CAM1_THERM_PAL:

CAM1\_THERM\_PAL: Camera1 Thermal Palette
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


thermal image colour palette


+-------+-----------------+
| Value | Meaning         |
+=======+=================+
| -1    | Leave Unchanged |
+-------+-----------------+
| 0     | WhiteHot        |
+-------+-----------------+
| 2     | Sepia           |
+-------+-----------------+
| 3     | IronBow         |
+-------+-----------------+
| 4     | Rainbow         |
+-------+-----------------+
| 5     | Night           |
+-------+-----------------+
| 6     | Aurora          |
+-------+-----------------+
| 7     | RedHot          |
+-------+-----------------+
| 8     | Jungle          |
+-------+-----------------+
| 9     | Medical         |
+-------+-----------------+
| 10    | BlackHot        |
+-------+-----------------+
| 11    | GloryHot        |
+-------+-----------------+




.. _CAM1_THERM_GAIN:

CAM1\_THERM\_GAIN: Camera1 Thermal Gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


thermal image temperature range


+-------+-------------------------+
| Value | Meaning                 |
+=======+=========================+
| -1    | Leave Unchanged         |
+-------+-------------------------+
| 0     | LowGain (50C to 550C)   |
+-------+-------------------------+
| 1     | HighGain (-20C to 150C) |
+-------+-------------------------+




.. _CAM1_THERM_RAW:

CAM1\_THERM\_RAW: Camera1 Thermal Raw Data
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


save images with raw temperatures


+-------+------------------+
| Value | Meaning          |
+=======+==================+
| -1    | Leave Unchanged  |
+-------+------------------+
| 0     | Disabled (30fps) |
+-------+------------------+
| 1     | Enabled (25 fps) |
+-------+------------------+




+--------+
| Units  |
+========+
| meters |
+--------+




.. _WEB_ENABLE:

WEB\_ENABLE: enable web server
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


enable web server


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _WEB_BIND_PORT:

WEB\_BIND\_PORT: web server TCP port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


web server TCP port


+------------+
| Range      |
+============+
| 1 to 65535 |
+------------+




.. _WEB_DEBUG:

WEB\_DEBUG: web server debugging
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

web server debugging


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _WEB_BLOCK_SIZE:

WEB\_BLOCK\_SIZE: web server block size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

web server block size for download


+------------+
| Range      |
+============+
| 1 to 65535 |
+------------+




.. _WEB_TIMEOUT:

WEB\_TIMEOUT: web server timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

timeout for inactive connections


+-----------+---------+
| Range     | Units   |
+===========+=========+
| 0.1 to 60 | seconds |
+-----------+---------+




.. _WEB_SENDFILE_MIN:

WEB\_SENDFILE\_MIN: web server minimum file size for sendfile
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

sendfile is an offloading mechanism for faster file download\. If this is non\-zero and the file is larger than this size then sendfile will be used for file download


+---------------+
| Range         |
+===============+
| 0 to 10000000 |
+---------------+




.. _SHIP_ENABLE:

SHIP\_ENABLE: Ship landing enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable ship landing system


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _SHIP_LAND_ANGLE:

SHIP\_LAND\_ANGLE: Ship landing angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Angle from the stern of the ship for landing approach\. Use this to ensure that on a go\-around that ship superstructure and cables are avoided\. A value of zero means to approach from the rear of the ship\. A value of 90 means the landing will approach from the port \(left\) side of the ship\. A value of \-90 will mean approaching from the starboard \(right\) side of the ship\. A value of 180 will approach from the bow of the ship\. This parameter is combined with the sign of the RTL\_RADIUS parameter to determine the holdoff pattern\. If RTL\_RADIUS is positive then a clockwise loiter is performed\, if RTL\_RADIUS is negative then a counter\-clockwise loiter is used\.


+-------------+---------+
| Range       | Units   |
+=============+=========+
| -180 to 180 | degrees |
+-------------+---------+




.. _SHIP_AUTO_OFS:

SHIP\_AUTO\_OFS: Ship automatic offset trigger
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Settings this parameter to one triggers an automatic follow offset calculation based on current position of the vehicle and the landing target\. NOTE\: This parameter will auto\-reset to zero once the offset has been calculated\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Trigger  |
+-------+----------+




.. _RCK_FORCEHL:

RCK\_FORCEHL: Force enable High Latency mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Automatically enables High Latency mode if not already enabled


+-------+---------------------------+
| Value | Meaning                   |
+=======+===========================+
| 0     | Disabled                  |
+-------+---------------------------+
| 1     | Enabled                   |
+-------+---------------------------+
| 2     | Enabled on telemetry loss |
+-------+---------------------------+




.. _RCK_PERIOD:

RCK\_PERIOD: Update rate
~~~~~~~~~~~~~~~~~~~~~~~~


When in High Latency mode\, send Rockblock updates every N seconds


+----------+---------+
| Range    | Units   |
+==========+=========+
| 0 to 600 | seconds |
+----------+---------+




.. _RCK_DEBUG:

RCK\_DEBUG: Display Rockblock debugging text
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sends Rockblock debug text to GCS via statustexts


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _RCK_ENABLE:

RCK\_ENABLE: Enable Message transmission
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enables the Rockblock sending and recieving


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _RCK_TIMEOUT:

RCK\_TIMEOUT: GCS timeout to start sendin Rockblock messages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


If RCK\_FORCEHL\=2\, this is the number of seconds of GCS timeout until High Latency mode is auto\-enabled


+----------+---------+
| Range    | Units   |
+==========+=========+
| 0 to 600 | seconds |
+----------+---------+




.. _SLUP_ENABLE:

SLUP\_ENABLE: Slung Payload enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Slung Payload enable


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _SLUP_VEL_P:

SLUP\_VEL\_P: Slung Payload Velocity P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Slung Payload Velocity P gain\, higher values will result in faster movements in sync with payload


+----------+
| Range    |
+==========+
| 0 to 0.8 |
+----------+




.. _SLUP_DIST_MAX:

SLUP\_DIST\_MAX: Slung Payload horizontal distance max
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Oscillation is suppressed when vehicle and payload are no more than this distance horizontally\.  Set to 0 to always suppress


+---------+
| Range   |
+=========+
| 0 to 30 |
+---------+




.. _SLUP_SYSID:

SLUP\_SYSID: Slung Payload mavlink system id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Slung Payload mavlink system id\.  0 to use any\/all system ids


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _SLUP_WP_POS_P:

SLUP\_WP\_POS\_P: Slung Payload return to WP position P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


WP position P gain\. higher values will result in vehicle moving more quickly back to the original waypoint


+--------+
| Range  |
+========+
| 0 to 1 |
+--------+




.. _SLUP_RESTOFS_TC:

SLUP\_RESTOFS\_TC: Slung Payload resting offset estimate filter time constant
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


payload\'s position estimator\'s time constant used to compensate for GPS errors and wind\.  Higher values result in smoother estimate but slower response


+---------+
| Range   |
+=========+
| 1 to 20 |
+---------+




.. _SLUP_DEBUG:

SLUP\_DEBUG: Slung Payload debug output
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Slung payload debug output\, set to 1 to enable debug


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _TERR_BRK_ENABLE:

TERR\_BRK\_ENABLE: terrain brake enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


terrain brake enable


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _TERR_BRK_ALT:

TERR\_BRK\_ALT: terrain brake altitude
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


terrain brake altitude\. The altitude above the ground below which BRAKE mode will be engaged if in LOITER mode\.


+----------+--------+
| Range    | Units  |
+==========+========+
| 1 to 100 | meters |
+----------+--------+




.. _TERR_BRK_HDIST:

TERR\_BRK\_HDIST: terrain brake home distance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


terrain brake home distance\. The distance from home where the auto BRAKE will be enabled\. When within this distance of home the script will not activate


+-----------+--------+
| Range     | Units  |
+===========+========+
| 0 to 1000 | meters |
+-----------+--------+




.. _TERR_BRK_SPD:

TERR\_BRK\_SPD: terrain brake speed threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


terrain brake speed threshold\. Don\'t trigger BRAKE if both horizontal speed and descent rate are below this threshold\. By setting this to a small value this can be used to allow the user to climb up to a safe altitude in LOITER mode\. A value of 0\.5 is recommended if you want to use LOITER to recover from an emergency terrain BRAKE mode change\.


+--------+-------------------+
| Range  | Units             |
+========+===================+
| 0 to 5 | meters per second |
+--------+-------------------+




.. _AEROM_ANG_ACCEL:

AEROM\_ANG\_ACCEL: Angular acceleration limit
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum angular acceleration in maneuvers


+---------------------------+
| Units                     |
+===========================+
| degrees per square second |
+---------------------------+




.. _AEROM_ANG_TC:

AEROM\_ANG\_TC: Roll control filtertime constant
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the time over which we filter the desired roll to smooth it


+---------+
| Units   |
+=========+
| seconds |
+---------+




.. _AEROM_THR_PIT_FF:

AEROM\_THR\_PIT\_FF: Throttle feed forward from pitch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls how much extra throttle to add based on pitch ange\. The value is for 90 degrees and is applied in proportion to pitch


+---------+
| Units   |
+=========+
| percent |
+---------+




.. _AEROM_SPD_P:

AEROM\_SPD\_P: P gain for speed controller
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls how rapidly the throttle is raised to compensate for a speed error


+---------+
| Units   |
+=========+
| percent |
+---------+




.. _AEROM_SPD_I:

AEROM\_SPD\_I: I gain for speed controller
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls how rapidly the throttle is raised to compensate for a speed error


+---------+
| Units   |
+=========+
| percent |
+---------+




.. _AEROM_ROL_COR_TC:

AEROM\_ROL\_COR\_TC: Roll control time constant
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the time constant for correcting roll errors\. A smaller value leads to faster roll corrections


+---------+
| Units   |
+=========+
| seconds |
+---------+




.. _AEROM_TIME_COR_P:

AEROM\_TIME\_COR\_P: Time constant for correction of our distance along the path
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the time constant for correcting path position errors


+---------+
| Units   |
+=========+
| seconds |
+---------+




.. _AEROM_ERR_COR_P:

AEROM\_ERR\_COR\_P: P gain for path error corrections
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls how rapidly we correct back onto the desired path


.. _AEROM_ERR_COR_D:

AEROM\_ERR\_COR\_D: D gain for path error corrections
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls how rapidly we correct back onto the desired path


.. _AEROM_ENTRY_RATE:

AEROM\_ENTRY\_RATE: The roll rate to use when entering a roll maneuver
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls how rapidly we roll into a new orientation


+--------------------+
| Units              |
+====================+
| degrees per second |
+--------------------+




.. _AEROM_THR_LKAHD:

AEROM\_THR\_LKAHD: The lookahead for throttle control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls how far ahead we look in time along the path for the target throttle


+---------+
| Units   |
+=========+
| seconds |
+---------+




.. _AEROM_DEBUG:

AEROM\_DEBUG: Debug control
~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls the printing of extra debug information on paths


.. _AEROM_THR_MIN:

AEROM\_THR\_MIN: Minimum Throttle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Lowest throttle used during maneuvers


+---------+
| Units   |
+=========+
| percent |
+---------+




.. _AEROM_THR_BOOST:

AEROM\_THR\_BOOST: Throttle boost
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the extra throttle added in schedule elements marked as needing a throttle boost


+---------+
| Units   |
+=========+
| percent |
+---------+




.. _AEROM_YAW_ACCEL:

AEROM\_YAW\_ACCEL: Yaw acceleration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is maximum yaw acceleration to use


+---------------------------+
| Units                     |
+===========================+
| degrees per square second |
+---------------------------+




.. _AEROM_LKAHD:

AEROM\_LKAHD: Lookahead
~~~~~~~~~~~~~~~~~~~~~~~


This is how much time to look ahead in the path for calculating path rates


+---------+
| Units   |
+=========+
| seconds |
+---------+




.. _AEROM_PATH_SCALE:

AEROM\_PATH\_SCALE: Path Scale
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scale factor for Path\/Box size\. 0\.5 would half the distances in maneuvers\. Radii are unaffected\.


+------------+
| Range      |
+============+
| 0.1 to 100 |
+------------+




.. _AEROM_BOX_WIDTH:

AEROM\_BOX\_WIDTH: Box Width
~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Length of aerobatic \"box\"


+--------+
| Units  |
+========+
| meters |
+--------+




.. _AEROM_STALL_THR:

AEROM\_STALL\_THR: Stall turn throttle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Amount of throttle to reduce to for a stall turn


+---------+
| Units   |
+=========+
| percent |
+---------+




.. _AEROM_STALL_PIT:

AEROM\_STALL\_PIT: Stall turn pitch threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Pitch threashold for moving to final stage of stall turn


+---------+
| Units   |
+=========+
| degrees |
+---------+




.. _AEROM_KE_RUDD:

AEROM\_KE\_RUDD: KnifeEdge Rudder
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Percent of rudder normally uses to sustain knife\-edge at trick speed


+---------+
| Units   |
+=========+
| percent |
+---------+




.. _AEROM_KE_RUDD_LK:

AEROM\_KE\_RUDD\_LK: KnifeEdge Rudder lookahead
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Time to look ahead in the path to calculate rudder correction for bank angle


+---------+
| Units   |
+=========+
| seconds |
+---------+




.. _AEROM_ALT_ABORT:

AEROM\_ALT\_ABORT: Altitude Abort
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum allowable loss in altitude during a trick or sequence from its starting altitude\.


+--------+
| Units  |
+========+
| meters |
+--------+




.. _AEROM_TS_P:

AEROM\_TS\_P: Timesync P gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls how rapidly two aircraft are brought back into time sync


.. _AEROM_TS_I:

AEROM\_TS\_I: Timesync I gain
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls how rapidly two aircraft are brought back into time sync


.. _AEROM_TS_SPDMAX:

AEROM\_TS\_SPDMAX: Timesync speed max
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the maximum speed adjustment for time sync between aircraft


+-------------------+
| Units             |
+===================+
| meters per second |
+-------------------+




.. _AEROM_TS_RATE:

AEROM\_TS\_RATE: Timesync rate of send of NAMED\_VALUE\_FLOAT data
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the rate we send data for time sync between aircraft


+-------+
| Units |
+=======+
| hertz |
+-------+




.. _AEROM_MIS_ANGLE:

AEROM\_MIS\_ANGLE: Mission angle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


When set to a non\-zero value\, this is the assumed direction of the mission\. Otherwise the waypoint angle is used


+---------+
| Units   |
+=========+
| degrees |
+---------+




.. _AEROM_OPTIONS:

AEROM\_OPTIONS: Aerobatic options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Options to control aerobatic behavior


+-----+--------------------------+
| Bit | Meaning                  |
+=====+==========================+
| 0   | UseRTLOnAbort            |
+-----+--------------------------+
| 1   | AddAtToMessages          |
+-----+--------------------------+
| 2   | DualAircraftSynchronised |
+-----+--------------------------+




+---------+
| Units   |
+=========+
| degrees |
+---------+




.. _TRIK_ENABLE:

TRIK\_ENABLE: Tricks on Switch Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enables Tricks on Switch\. TRIK params hidden until enabled


.. _TRIK_SEL_FN:

TRIK\_SEL\_FN: Trik Selection Scripting Function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting an RC channel\'s \_OPTION to this value will use it for trick selection


+------------+
| Range      |
+============+
| 301 to 307 |
+------------+




.. _TRIK_ACT_FN:

TRIK\_ACT\_FN: Trik Action Scripting Function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Setting an RC channel\'s \_OPTION to this value will use it for trick action \(abort\,announce\,execute\)


+------------+
| Range      |
+============+
| 301 to 307 |
+------------+




.. _TRIK_COUNT:

TRIK\_COUNT: Trik Count
~~~~~~~~~~~~~~~~~~~~~~~


Number of tricks which can be selected over the range of the trik selection RC channel


+---------+
| Range   |
+=========+
| 1 to 11 |
+---------+




.. _BATT_ANX_ENABLE:

BATT\_ANX\_ENABLE: Enable ANX battery support
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable ANX battery support


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _BATT_ANX_CANDRV:

BATT\_ANX\_CANDRV: Set ANX CAN driver
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Set ANX CAN driver


+-------+--------------+
| Value | Meaning      |
+=======+==============+
| 0     | None         |
+-------+--------------+
| 1     | 1stCANDriver |
+-------+--------------+
| 2     | 2ndCanDriver |
+-------+--------------+




.. _BATT_ANX_INDEX:

BATT\_ANX\_INDEX: ANX CAN battery index
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


ANX CAN battery index


+---------+
| Range   |
+=========+
| 1 to 10 |
+---------+




.. _BATT_ANX_OPTIONS:

BATT\_ANX\_OPTIONS: ANX CAN battery options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

ANX CAN battery options


+-----+--------------+
| Bit | Meaning      |
+=====+==============+
| 0   | LogAllFrames |
+-----+--------------+




.. _EFI_INF_ENABLE:

EFI\_INF\_ENABLE: EFI INF\-Inject enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable EFI INF\-Inject driver


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _EFI_INF_OPTIONS:

EFI\_INF\_OPTIONS: EFI INF\-Inject options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


EFI INF driver options


+-----+---------------+
| Bit | Meaning       |
+=====+===============+
| 0   | EnableLogging |
+-----+---------------+




.. _EFI_INF_THR_HZ:

EFI\_INF\_THR\_HZ: EFI INF\-Inject throttle rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


EFI INF throttle output rate


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 to 50 | hertz |
+---------+-------+




.. _EFI_INF_IGN_AUX:

EFI\_INF\_IGN\_AUX: EFI INF\-Inject ignition aux function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


EFI INF throttle ignition aux function


.. _EFI_DLA_ENABLE:

EFI\_DLA\_ENABLE: EFI DLA enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable EFI DLA driver


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _EFI_DLA_LPS:

EFI\_DLA\_LPS: EFI DLA fuel scale
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


EFI DLA litres of fuel per second of injection time


+--------------+--------+
| Range        | Units  |
+==============+========+
| 0.00001 to 1 | litres |
+--------------+--------+




.. _VIEP_DEBUG:

VIEP\_DEBUG: ViewPro debug
~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

ViewPro debug


+-------+--------------------------------------+
| Value | Meaning                              |
+=======+======================================+
| 0     | Disabled                             |
+-------+--------------------------------------+
| 1     | Enabled                              |
+-------+--------------------------------------+
| 2     | Enabled including attitude reporting |
+-------+--------------------------------------+




.. _VIEP_CAM_SWLOW:

VIEP\_CAM\_SWLOW: ViewPro Camera For Switch Low
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Camera selection when switch is in low position


+-------+-------------------------------+
| Value | Meaning                       |
+=======+===============================+
| 0     | No change in camera selection |
+-------+-------------------------------+
| 1     | EO1                           |
+-------+-------------------------------+
| 2     | IR thermal                    |
+-------+-------------------------------+
| 3     | EO1 + IR Picture-in-picture   |
+-------+-------------------------------+
| 4     | IR + EO1 Picture-in-picture   |
+-------+-------------------------------+
| 5     | Fusion                        |
+-------+-------------------------------+
| 6     | IR1 13mm                      |
+-------+-------------------------------+
| 7     | IR2 52mm                      |
+-------+-------------------------------+




.. _VIEP_CAM_SWMID:

VIEP\_CAM\_SWMID: ViewPro Camera For Switch Mid
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Camera selection when switch is in middle position


+-------+-------------------------------+
| Value | Meaning                       |
+=======+===============================+
| 0     | No change in camera selection |
+-------+-------------------------------+
| 1     | EO1                           |
+-------+-------------------------------+
| 2     | IR thermal                    |
+-------+-------------------------------+
| 3     | EO1 + IR Picture-in-picture   |
+-------+-------------------------------+
| 4     | IR + EO1 Picture-in-picture   |
+-------+-------------------------------+
| 5     | Fusion                        |
+-------+-------------------------------+
| 6     | IR1 13mm                      |
+-------+-------------------------------+
| 7     | IR2 52mm                      |
+-------+-------------------------------+




.. _VIEP_CAM_SWHIGH:

VIEP\_CAM\_SWHIGH: ViewPro Camera For Switch High
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Camera selection when switch is in high position


+-------+-------------------------------+
| Value | Meaning                       |
+=======+===============================+
| 0     | No change in camera selection |
+-------+-------------------------------+
| 1     | EO1                           |
+-------+-------------------------------+
| 2     | IR thermal                    |
+-------+-------------------------------+
| 3     | EO1 + IR Picture-in-picture   |
+-------+-------------------------------+
| 4     | IR + EO1 Picture-in-picture   |
+-------+-------------------------------+
| 5     | Fusion                        |
+-------+-------------------------------+
| 6     | IR1 13mm                      |
+-------+-------------------------------+
| 7     | IR2 52mm                      |
+-------+-------------------------------+




.. _VIEP_ZOOM_SPEED:

VIEP\_ZOOM\_SPEED: ViewPro Zoom Speed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


ViewPro Zoom Speed\.  Higher numbers result in faster zooming


+--------+
| Range  |
+========+
| 0 to 7 |
+--------+




.. _VIEP_ZOOM_MAX:

VIEP\_ZOOM\_MAX: ViewPro Zoom Times Max
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


ViewPro Zoom Times Max


+---------+
| Range   |
+=========+
| 0 to 30 |
+---------+




.. _ESC_HW_ENABLE:

ESC\_HW\_ENABLE: Hobbywing ESC Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable Hobbywing ESC telemetry


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _ESC_HW_POLES:

ESC\_HW\_POLES: Hobbywing ESC motor poles
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of motor poles for eRPM scaling


+---------+
| Range   |
+=========+
| 1 to 50 |
+---------+




.. _ESC_HW_OFS:

ESC\_HW\_OFS: Hobbywing ESC motor offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Motor number offset of first ESC


+---------+
| Range   |
+=========+
| 0 to 31 |
+---------+




.. _DJIR_DEBUG:

DJIR\_DEBUG: DJIRS2 debug
~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enable DJIRS2 debug


+-------+---------------------------------+
| Value | Meaning                         |
+=======+=================================+
| 0     | Disabled                        |
+-------+---------------------------------+
| 1     | Enabled                         |
+-------+---------------------------------+
| 2     | Enabled with attitude reporting |
+-------+---------------------------------+




.. _DJIR_UPSIDEDOWN:

DJIR\_UPSIDEDOWN: DJIRS2 upside down
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


DJIRS2 upside down


+-------+---------------+
| Value | Meaning       |
+=======+===============+
| 0     | Right side up |
+-------+---------------+
| 1     | Upside down   |
+-------+---------------+




.. _EFI_H6K_ENABLE:

EFI\_H6K\_ENABLE: Enable Halo6000 EFI driver
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable Halo6000 EFI driver


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _EFI_H6K_CANDRV:

EFI\_H6K\_CANDRV: Halo6000 CAN driver
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Halo6000 CAN driver\. Use 1 for first CAN scripting driver\, 2 for 2nd driver


+-------+-----------+
| Value | Meaning   |
+=======+===========+
| 0     | Disabled  |
+-------+-----------+
| 1     | FirstCAN  |
+-------+-----------+
| 2     | SecondCAN |
+-------+-----------+




.. _EFI_H6K_START_FN:

EFI\_H6K\_START\_FN: Halo6000 start auxilliary function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The RC auxilliary function number for start\/stop of the generator\. Zero to disable start function


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 300   | 300      |
+-------+----------+
| 301   | 301      |
+-------+----------+
| 302   | 302      |
+-------+----------+
| 303   | 303      |
+-------+----------+
| 304   | 304      |
+-------+----------+
| 305   | 305      |
+-------+----------+
| 306   | 306      |
+-------+----------+
| 307   | 307      |
+-------+----------+




.. _EFI_H6K_TELEM_RT:

EFI\_H6K\_TELEM\_RT: Halo6000 telemetry rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The rate that additional generator telemetry is sent


+-------+
| Units |
+=======+
| hertz |
+-------+




.. _EFI_H6K_FUELTOT:

EFI\_H6K\_FUELTOT: Halo6000 total fuel capacity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The capacity of the tank in litres


+--------+
| Units  |
+========+
| litres |
+--------+




.. _EFI_H6K_OPTIONS:

EFI\_H6K\_OPTIONS: Halo6000 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Halo6000 options


+-----+------------------+
| Bit | Meaning          |
+=====+==================+
| 0   | LogAllCanPackets |
+-----+------------------+




.. _EFI_SVF_ENABLE:

EFI\_SVF\_ENABLE: Generator SVFFI enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable SVFFI generator support


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _EFI_SVF_ARMCHECK:

EFI\_SVF\_ARMCHECK: Generator SVFFI arming check
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Check for Generator ARM state before arming


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _UM_SERVO_MASK:

UM\_SERVO\_MASK: Mask of UltraMotion servos
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Mask of UltraMotion servos


+-----+---------+
| Bit | Meaning |
+=====+=========+
| 0   | SERVO1  |
+-----+---------+
| 1   | SERVO2  |
+-----+---------+
| 2   | SERVO3  |
+-----+---------+
| 3   | SERVO4  |
+-----+---------+
| 4   | SERVO5  |
+-----+---------+
| 5   | SERVO6  |
+-----+---------+
| 6   | SERVO7  |
+-----+---------+
| 7   | SERVO8  |
+-----+---------+
| 8   | SERVO9  |
+-----+---------+
| 9   | SERVO10 |
+-----+---------+
| 10  | SERVO11 |
+-----+---------+
| 11  | SERVO12 |
+-----+---------+




.. _UM_CANDRV:

UM\_CANDRV: Set CAN driver
~~~~~~~~~~~~~~~~~~~~~~~~~~


Set CAN driver


+-------+--------------+
| Value | Meaning      |
+=======+==============+
| 0     | None         |
+-------+--------------+
| 1     | 1stCANDriver |
+-------+--------------+
| 2     | 2ndCanDriver |
+-------+--------------+




.. _UM_RATE_HZ:

UM\_RATE\_HZ: Update rate for UltraMotion servos
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Update rate for UltraMotion servos


+----------+-------+
| Range    | Units |
+==========+=======+
| 1 to 400 | hertz |
+----------+-------+




.. _UM_OPTIONS:

UM\_OPTIONS: Optional settings
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Optional settings


+-----+--------------------------+
| Bit | Meaning                  |
+=====+==========================+
| 0   | LogAllFrames             |
+-----+--------------------------+
| 1   | ParseTelemetry           |
+-----+--------------------------+
| 2   | SendPosAsNamedValueFloat |
+-----+--------------------------+




.. _EFI_SP_ENABLE:

EFI\_SP\_ENABLE: Enable SkyPower EFI support
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable SkyPower EFI support


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _EFI_SP_CANDRV:

EFI\_SP\_CANDRV: Set SkyPower EFI CAN driver
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Set SkyPower EFI CAN driver


+-------+--------------+
| Value | Meaning      |
+=======+==============+
| 0     | None         |
+-------+--------------+
| 1     | 1stCANDriver |
+-------+--------------+
| 2     | 2ndCanDriver |
+-------+--------------+




.. _EFI_SP_UPDATE_HZ:

EFI\_SP\_UPDATE\_HZ: SkyPower EFI update rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

SkyPower EFI update rate


+-----------+-------+
| Range     | Units |
+===========+=======+
| 10 to 200 | hertz |
+-----------+-------+




.. _EFI_SP_THR_FN:

EFI\_SP\_THR\_FN: SkyPower EFI throttle function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


SkyPower EFI throttle function\. This sets which SERVOn\_FUNCTION to use for the target throttle\. This should be 70 for fixed wing aircraft and 31 for helicopter rotor speed control


+-------+-----------+
| Value | Meaning   |
+=======+===========+
| 0     | Disabled  |
+-------+-----------+
| 70    | FixedWing |
+-------+-----------+
| 31    | HeliRSC   |
+-------+-----------+




.. _EFI_SP_THR_RATE:

EFI\_SP\_THR\_RATE: SkyPower EFI throttle rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

SkyPower EFI throttle rate\. This sets rate at which throttle updates are sent to the engine


+-----------+-------+
| Range     | Units |
+===========+=======+
| 10 to 100 | hertz |
+-----------+-------+




.. _EFI_SP_START_FN:

EFI\_SP\_START\_FN: SkyPower EFI start function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


SkyPower EFI start function\. This is the RCn\_OPTION value to use to find the R\/C channel used for controlling engine start


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 300   | 300      |
+-------+----------+
| 301   | 301      |
+-------+----------+
| 302   | 302      |
+-------+----------+
| 303   | 303      |
+-------+----------+
| 304   | 304      |
+-------+----------+
| 305   | 305      |
+-------+----------+
| 306   | 306      |
+-------+----------+
| 307   | 307      |
+-------+----------+




.. _EFI_SP_GEN_FN:

EFI\_SP\_GEN\_FN: SkyPower EFI generator control function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


SkyPower EFI generator control function\. This is the RCn\_OPTION value to use to find the R\/C channel used for controlling generator start\/stop


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 300   | 300      |
+-------+----------+
| 301   | 301      |
+-------+----------+
| 302   | 302      |
+-------+----------+
| 303   | 303      |
+-------+----------+
| 304   | 304      |
+-------+----------+
| 305   | 305      |
+-------+----------+
| 306   | 306      |
+-------+----------+
| 307   | 307      |
+-------+----------+




.. _EFI_SP_MIN_RPM:

EFI\_SP\_MIN\_RPM: SkyPower EFI minimum RPM
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

SkyPower EFI minimum RPM\. This is the RPM below which the engine is considered to be stopped


+-----------+
| Range     |
+===========+
| 1 to 1000 |
+-----------+




.. _EFI_SP_TLM_RT:

EFI\_SP\_TLM\_RT: SkyPower EFI telemetry rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

SkyPower EFI telemetry rate\. This is the rate at which extra telemetry values are sent to the GCS


+---------+-------+
| Range   | Units |
+=========+=======+
| 1 to 10 | hertz |
+---------+-------+




.. _EFI_SP_LOG_RT:

EFI\_SP\_LOG\_RT: SkyPower EFI log rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

SkyPower EFI log rate\. This is the rate at which extra logging of the SkyPower EFI is performed


+---------+-------+
| Range   | Units |
+=========+=======+
| 1 to 50 | hertz |
+---------+-------+




.. _EFI_SP_ST_DISARM:

EFI\_SP\_ST\_DISARM: SkyPower EFI allow start disarmed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


SkyPower EFI allow start disarmed\. This controls if starting the engine while disarmed is allowed


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _EFI_SP_MODEL:

EFI\_SP\_MODEL: SkyPower EFI ECU model
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


SkyPower EFI ECU model


+-------+---------+
| Value | Meaning |
+=======+=========+
| 0     | SRE_180 |
+-------+---------+
| 1     | SP_275  |
+-------+---------+




.. _EFI_SP_GEN_CTRL:

EFI\_SP\_GEN\_CTRL: SkyPower EFI enable generator control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


SkyPower EFI enable generator control


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _EFI_SP_RST_TIME:

EFI\_SP\_RST\_TIME: SkyPower EFI restart time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


SkyPower EFI restart time\. If engine should be running and it has stopped for this amount of time then auto\-restart\. To disable this feature set this value to zero\.


+---------+---------+
| Range   | Units   |
+=========+=========+
| 0 to 10 | seconds |
+---------+---------+




.. _TRQL_ENABLE:

TRQL\_ENABLE: Torqeedo TorqLink Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Torqeedo TorqLink Enable


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _TRQL_DEBUG:

TRQL\_DEBUG: Torqeedo TorqLink Debug Level
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Torqeedo TorqLink Debug Level


+-------+---------+
| Value | Meaning |
+=======+=========+
| 0     | None    |
+-------+---------+
| 1     | Low     |
+-------+---------+
| 2     | Medium  |
+-------+---------+
| 3     | High    |
+-------+---------+




.. _EFI_HFE_ENABLE:

EFI\_HFE\_ENABLE: Enable HFE EFI driver
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable HFE EFI driver


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _EFI_HFE_RATE_HZ:

EFI\_HFE\_RATE\_HZ: HFI EFI Update rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


HFI EFI Update rate


+----------+
| Range    |
+==========+
| 0 to 400 |
+----------+




.. _EFI_HFE_ECU_IDX:

EFI\_HFE\_ECU\_IDX: HFI EFI ECU index
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


HFI EFI ECU index\, 0 for automatic


+---------+
| Range   |
+=========+
| 0 to 10 |
+---------+




.. _EFI_HFE_FUEL_DTY:

EFI\_HFE\_FUEL\_DTY: HFI EFI fuel density
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


HFI EFI fuel density in gram per litre


+-----------+
| Range     |
+===========+
| 0 to 2000 |
+-----------+




.. _EFI_HFE_REL_IDX:

EFI\_HFE\_REL\_IDX: HFI EFI relay index
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


HFI EFI relay index


+---------+
| Range   |
+=========+
| 0 to 10 |
+---------+




.. _EFI_HFE_CANDRV:

EFI\_HFE\_CANDRV: HFI EFI CAN driver
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


HFI EFI CAN driver


+-------+--------------+
| Value | Meaning      |
+=======+==============+
| 0     | None         |
+-------+--------------+
| 1     | 1stCANDriver |
+-------+--------------+
| 2     | 2ndCanDriver |
+-------+--------------+




.. _EFI_HFE_OPTIONS:

EFI\_HFE\_OPTIONS: HFI EFI options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


HFI EFI options


+-----+------------------+
| Bit | Meaning          |
+=====+==================+
| 1   | EnableCANLogging |
+-----+------------------+




.. _EFI_DLA64_ENABLE:

EFI\_DLA64\_ENABLE: EFI DLA64 enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable EFI DLA64 driver


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _LTE_ENABLE:

LTE\_ENABLE: LTE Enable
~~~~~~~~~~~~~~~~~~~~~~~


Enable or disable the LTE modem driver


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _LTE_SERPORT:

LTE\_SERPORT: Serial Port
~~~~~~~~~~~~~~~~~~~~~~~~~


Serial port to use for the LTE modem\. This is the index of the SERIALn\_ ports that are set to 28 for \"scripting\"


+--------+
| Range  |
+========+
| 0 to 8 |
+--------+




.. _LTE_SCRPORT:

LTE\_SCRPORT: Scripting Serial Port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Scripting Serial port to use for the LTE modem\. This is the index of the SCR\_SDEV ports that are set to 2 for \"MAVLink2\"


+--------+
| Range  |
+========+
| 0 to 8 |
+--------+




.. _LTE_SERVER_IP0:

LTE\_SERVER\_IP0: Server IP 0
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


First octet of the server IP address to connect to


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _LTE_SERVER_IP1:

LTE\_SERVER\_IP1: Server IP 1
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Second octet of the server IP address to connect to


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _LTE_SERVER_IP2:

LTE\_SERVER\_IP2: Server IP 2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Third octet of the server IP address to connect to


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _LTE_SERVER_IP3:

LTE\_SERVER\_IP3: Server IP 3
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Fourth octet of the server IP address to connect to


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _LTE_SERVER_PORT:

LTE\_SERVER\_PORT: Server Port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


IPv4 Port of the server to connect to


+------------+
| Range      |
+============+
| 1 to 65525 |
+------------+




.. _LTE_BAUD:

LTE\_BAUD: Serial Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Baud rate for the serial port to the LTE modem when connected\. Initial power on baudrate is in LTE\_IBAUD


+---------+---------+
| Value   | Meaning |
+=========+=========+
| 19200   | 19200   |
+---------+---------+
| 38400   | 38400   |
+---------+---------+
| 57600   | 57600   |
+---------+---------+
| 115200  | 115200  |
+---------+---------+
| 230400  | 230400  |
+---------+---------+
| 460800  | 460800  |
+---------+---------+
| 921600  | 921600  |
+---------+---------+
| 3686400 | 3686400 |
+---------+---------+




.. _LTE_TIMEOUT:

LTE\_TIMEOUT: Timeout
~~~~~~~~~~~~~~~~~~~~~


Timeout in seconds for the LTE connection\. If no data is received for this time\, the connection will be reset\. A value of zero disables the timeout


+---------+---------+
| Range   | Units   |
+=========+=========+
| 0 to 60 | seconds |
+---------+---------+




.. _LTE_PROTOCOL:

LTE\_PROTOCOL: LTE protocol
~~~~~~~~~~~~~~~~~~~~~~~~~~~


The protocol that we will use in communication with the LTE modem\. If this is PPP then the LTE\_SERVER parameters are not used and instead a PPP connection will be established and you should use the NET\_ parameters to enable network ports\. If this is MAVLink2 then the LTE\_SERVER parameters are used to create a TCP or UDP connection to a single server\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 2     | MavLink2 |
+-------+----------+
| 48    | PPP      |
+-------+----------+




.. _LTE_OPTIONS:

LTE\_OPTIONS: LTE options
~~~~~~~~~~~~~~~~~~~~~~~~~


Options to control the LTE modem driver\. If VerboseSignalInfoGCS is set then additional NAMED\_VALUE\_FLOAT values are sent with verbose signal information


+-----+----------------------+
| Bit | Meaning              |
+=====+======================+
| 0   | LogAllData           |
+-----+----------------------+
| 1   | VerboseSignalInfoGCS |
+-----+----------------------+
| 2   | DisableMultiplexing  |
+-----+----------------------+
| 3   | DisableSignalQueries |
+-----+----------------------+
| 4   | UseTCP               |
+-----+----------------------+




.. _LTE_IBAUD:

LTE\_IBAUD: LTE initial baudrate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is the initial baud rate on power on for the modem\. This is set in the modem with the AT\+IREX\=baud command


+---------+---------+
| Value   | Meaning |
+=========+=========+
| 19200   | 19200   |
+---------+---------+
| 38400   | 38400   |
+---------+---------+
| 57600   | 57600   |
+---------+---------+
| 115200  | 115200  |
+---------+---------+
| 230400  | 230400  |
+---------+---------+
| 460800  | 460800  |
+---------+---------+
| 921600  | 921600  |
+---------+---------+
| 3686400 | 3686400 |
+---------+---------+




.. _LTE_MCCMNC:

LTE\_MCCMNC: LTE operator selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This allows selection of network operator


+--------------+----------+
| Value        | Meaning  |
+==============+==========+
| -1           | NoChange |
+--------------+----------+
| 0            | Default  |
+--------------+----------+
| AU-Telstra   | 50501    |
+--------------+----------+
| AU-Optus     | 50502    |
+--------------+----------+
| AU-Vodaphone | 50503    |
+--------------+----------+




.. _LTE_TX_RATE:

LTE\_TX\_RATE: Max transmit rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Maximum data transmit rate to the modem in bytes\/second\. Use zero for unlimited


.. _LTE_BAND:

LTE\_BAND: LTE band selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This allows selection of LTE band\. A value of \-1 means no band setting change is made\. A value of 0 sets all bands\. Otherwise the specified band is set\.


+----------+
| Range    |
+==========+
| -1 to 50 |
+----------+




.. _EFI_2K_ENABLE:

EFI\_2K\_ENABLE: Enable NMEA 2000 EFI driver
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable NMEA 2000 EFI driver


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _EFI_2K_CANDRV:

EFI\_2K\_CANDRV: NMEA 2000 CAN driver
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


NMEA 2000 CAN driver\. Use 1 for first CAN scripting driver\, 2 for 2nd driver


+-------+-----------+
| Value | Meaning   |
+=======+===========+
| 0     | Disabled  |
+-------+-----------+
| 1     | FirstCAN  |
+-------+-----------+
| 2     | SecondCAN |
+-------+-----------+




.. _EFI_2K_OPTIONS:

EFI\_2K\_OPTIONS: NMEA 2000 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


NMEA 2000 driver options


+-----+---------------+
| Bit | Meaning       |
+=====+===============+
| 0   | EnableLogging |
+-----+---------------+




.. _TOFSENSE_PRX:

TOFSENSE\_PRX: TOFSENSE\-M to be used as Proximity sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Set 0 if sensor is to be used as a 1\-D rangefinder \(minimum of all distances will be sent\, typically used for height detection\)\. Set 1 if it should be used as a 3\-D proximity device \(Eg\. Obstacle Avoidance\)


+-------+-------------------------+
| Value | Meaning                 |
+=======+=========================+
| 0     | Set as Rangefinder      |
+-------+-------------------------+
| 1     | Set as Proximity sensor |
+-------+-------------------------+




.. _TOFSENSE_NO:

TOFSENSE\_NO: TOFSENSE\-M Connected
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of TOFSENSE\-M CAN sensors connected


+--------+
| Range  |
+========+
| 1 to 3 |
+--------+




.. _TOFSENSE_MODE:

TOFSENSE\_MODE: TOFSENSE\-M mode to be used
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


TOFSENSE\-M mode to be used\. 0 for 8x8 mode\. 1 for 4x4 mode


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | 8x8 mode |
+-------+----------+
| 1     | 4x4 mode |
+-------+----------+




.. _TOFSENSE_INST1:

TOFSENSE\_INST1: TOFSENSE\-M First Instance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


First TOFSENSE\-M sensors backend Instance\. Setting this to 1 will pick the first backend from PRX\_ or RNG\_ Parameters \(Depending on TOFSENSE\_PRX\)


+--------+
| Range  |
+========+
| 1 to 3 |
+--------+




.. _TOFSENSE_ID1:

TOFSENSE\_ID1: TOFSENSE\-M First ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


First TOFSENSE\-M sensor ID\. Leave this at 0 to accept all IDs and if only one sensor is present\. You can change ID of sensor from NAssistant Software


+----------+
| Range    |
+==========+
| 1 to 255 |
+----------+




.. _TOFSENSE_INST2:

TOFSENSE\_INST2: TOFSENSE\-M Second Instance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Second TOFSENSE\-M sensors backend Instance\. Setting this to 2 will pick the second backend from PRX\_ or RNG\_ Parameters \(Depending on TOFSENSE\_PRX\)


+--------+
| Range  |
+========+
| 1 to 3 |
+--------+




.. _TOFSENSE_ID2:

TOFSENSE\_ID2: TOFSENSE\-M Second ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Second TOFSENSE\-M sensor ID\. This cannot be 0\. You can change ID of sensor from NAssistant Software


+----------+
| Range    |
+==========+
| 1 to 255 |
+----------+




.. _TOFSENSE_INST3:

TOFSENSE\_INST3: TOFSENSE\-M Third Instance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Third TOFSENSE\-M sensors backend Instance\. Setting this to 3 will pick the second backend from PRX\_ or RNG\_ Parameters \(Depending on TOFSENSE\_PRX\)


+--------+
| Range  |
+========+
| 1 to 3 |
+--------+




.. _TOFSENSE_ID3:

TOFSENSE\_ID3: TOFSENSE\-M Thir ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Third TOFSENSE\-M sensor ID\. This cannot be 0\. You can change ID of sensor from NAssistant Software


+----------+
| Range    |
+==========+
| 1 to 255 |
+----------+




.. _TOFSENSE_S1_PRX:

TOFSENSE\_S1\_PRX: TOFSENSE\-M to be used as Proximity sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Set 0 if sensor is to be used as a 1\-D rangefinder \(minimum of all distances will be sent\, typically used for height detection\)\. Set 1 if it should be used as a 3\-D proximity device \(Eg\. Obstacle Avoidance\)


+-------+-------------------------+
| Value | Meaning                 |
+=======+=========================+
| 0     | Set as Rangefinder      |
+-------+-------------------------+
| 1     | Set as Proximity sensor |
+-------+-------------------------+




.. _TOFSENSE_S1_SP:

TOFSENSE\_S1\_SP: TOFSENSE\-M serial port config
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


UART instance sensor is connected to\. Set 1 if sensor is connected to the port with fist SERIALx\_PROTOCOL \= 28\.


+--------+
| Range  |
+========+
| 1 to 4 |
+--------+




.. _TOFSENSE_S1_BR:

TOFSENSE\_S1\_BR: TOFSENSE\-M serial port baudrate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Serial Port baud rate\. Sensor baud rate can be changed from Nassistant software



.. _parameters_AIS_:

AIS\_ Parameters
----------------


.. _AIS_TYPE:

AIS\_TYPE: AIS receiver type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

AIS receiver type


+-------+--------------------+
| Value | Meaning            |
+=======+====================+
| 0     | None               |
+-------+--------------------+
| 1     | NMEA AIVDM message |
+-------+--------------------+




.. _AIS_LIST_MAX:

AIS\_LIST\_MAX: AIS vessel list size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

AIS list size of nearest vessels\. Longer lists take longer to refresh with lower SRx\_ADSB values\.


+----------+
| Range    |
+==========+
| 1 to 100 |
+----------+




.. _AIS_TIME_OUT:

AIS\_TIME\_OUT: AIS vessel time out
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

if no updates are received in this time a vessel will be removed from the list


+-----------+---------+
| Range     | Units   |
+===========+=========+
| 1 to 2000 | seconds |
+-----------+---------+




.. _AIS_LOGGING:

AIS\_LOGGING: AIS logging options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Bitmask of AIS logging options


+-----+-------------------------------------+
| Bit | Meaning                             |
+=====+=====================================+
| 0   | Log all AIVDM messages              |
+-----+-------------------------------------+
| 1   | Log only unsupported AIVDM messages |
+-----+-------------------------------------+
| 2   | Log decoded messages                |
+-----+-------------------------------------+





.. _parameters_ARSPD:

ARSPD Parameters
----------------


.. _ARSPD_ENABLE:

ARSPD\_ENABLE: Airspeed Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable airspeed sensor support


+-------+---------+
| Value | Meaning |
+=======+=========+
| 0     | Disable |
+-------+---------+
| 1     | Enable  |
+-------+---------+




.. _ARSPD_TUBE_ORDER:

ARSPD\_TUBE\_ORDER: Control pitot tube order
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters\. If you set this to 0 then the first \(often the top\) connector on the sensor needs to be the stagnation pressure \(the pressure at the tip of the pitot tube\)\. If set to 1 then the second \(often the bottom\) connector needs to be the stagnation pressure\. If set to 2 \(the default\) then the airspeed driver will accept either order\. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall\, which would otherwise be seen as a positive airspeed\.


+-------+-------------+
| Value | Meaning     |
+=======+=============+
| 0     | Normal      |
+-------+-------------+
| 1     | Swapped     |
+-------+-------------+
| 2     | Auto Detect |
+-------+-------------+




.. _ARSPD_PRIMARY:

ARSPD\_PRIMARY: Primary airspeed sensor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This selects which airspeed sensor will be the primary if multiple sensors are found


+-------+-------------+
| Value | Meaning     |
+=======+=============+
| 0     | FirstSensor |
+-------+-------------+
| 1     | 2ndSensor   |
+-------+-------------+




.. _ARSPD_OPTIONS:

ARSPD\_OPTIONS: Airspeed options bitmask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Bitmask of options to use with airspeed\. 0\:Disable use based on airspeed\/groundspeed mismatch \(see ARSPD\_WIND\_MAX\)\, 1\:Automatically reenable use based on airspeed\/groundspeed mismatch recovery \(see ARSPD\_WIND\_MAX\) 2\:Disable voltage correction\, 3\:Check that the airspeed is statistically consistent with the navigation EKF vehicle and wind velocity estimates using EKF3 \(requires AHRS\_EKF\_TYPE \= 3\)\, 4\:Report cal offset to GCS


+-----+----------------------------+
| Bit | Meaning                    |
+=====+============================+
| 0   | SpeedMismatchDisable       |
+-----+----------------------------+
| 1   | AllowSpeedMismatchRecovery |
+-----+----------------------------+
| 2   | DisableVoltageCorrection   |
+-----+----------------------------+
| 3   | UseEkf3Consistency         |
+-----+----------------------------+
| 4   | ReportOffset               |
+-----+----------------------------+




.. _ARSPD_WIND_MAX:

ARSPD\_WIND\_MAX: Maximum airspeed and ground speed difference
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

If the difference between airspeed and ground speed is greater than this value the sensor will be marked unhealthy\. Using ARSPD\_OPTIONS this health value can be used to disable the sensor\.


+-------------------+
| Units             |
+===================+
| meters per second |
+-------------------+




.. _ARSPD_WIND_WARN:

ARSPD\_WIND\_WARN: Airspeed and GPS speed difference that gives a warning
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

If the difference between airspeed and GPS speed is greater than this value the sensor will issue a warning\. If 0 ARSPD\_WIND\_MAX is used\.


+-------------------+
| Units             |
+===================+
| meters per second |
+-------------------+




.. _ARSPD_WIND_GATE:

ARSPD\_WIND\_GATE: Re\-enable Consistency Check Gate Size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Number of standard deviations applied to the re\-enable EKF consistency check that is used when ARSPD\_OPTIONS bit position 3 is set\. Larger values will make the re\-enabling of the airspeed sensor faster\, but increase the likelihood of re\-enabling a degraded sensor\. The value can be tuned by using the ARSP\.TR log message by setting ARSPD\_WIND\_GATE to a value that is higher than the value for ARSP\.TR observed with a healthy airspeed sensor\. Occasional transients in ARSP\.TR above the value set by ARSPD\_WIND\_GATE can be tolerated provided they are less than 5 seconds in duration and less than 10\% duty cycle\.


+-------------+
| Range       |
+=============+
| 0.0 to 10.0 |
+-------------+




.. _ARSPD_OFF_PCNT:

ARSPD\_OFF\_PCNT: Maximum offset cal speed error
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The maximum percentage speed change in airspeed reports that is allowed due to offset changes between calibrations before a warning is issued\. This potential speed error is in percent of AIRSPEED\_MIN\. 0 disables\. Helps warn of calibrations without pitot being covered\.


+-------------+---------+
| Range       | Units   |
+=============+=========+
| 0.0 to 10.0 | percent |
+-------------+---------+





.. _parameters_ARSPD2_:

ARSPD2\_ Parameters
-------------------


.. _ARSPD2_TYPE:

ARSPD2\_TYPE: Airspeed type
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Type of airspeed sensor


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| 0     | None              |
+-------+-------------------+
| 1     | I2C-MS4525D0      |
+-------+-------------------+
| 2     | Analog            |
+-------+-------------------+
| 3     | I2C-MS5525        |
+-------+-------------------+
| 4     | I2C-MS5525 (0x76) |
+-------+-------------------+
| 5     | I2C-MS5525 (0x77) |
+-------+-------------------+
| 6     | I2C-SDP3X         |
+-------+-------------------+
| 7     | I2C-DLVR-5in      |
+-------+-------------------+
| 8     | DroneCAN          |
+-------+-------------------+
| 9     | I2C-DLVR-10in     |
+-------+-------------------+
| 10    | I2C-DLVR-20in     |
+-------+-------------------+
| 11    | I2C-DLVR-30in     |
+-------+-------------------+
| 12    | I2C-DLVR-60in     |
+-------+-------------------+
| 13    | NMEA water speed  |
+-------+-------------------+
| 14    | MSP               |
+-------+-------------------+
| 15    | ASP5033           |
+-------+-------------------+
| 16    | ExternalAHRS      |
+-------+-------------------+
| 17    | AUAV-10in         |
+-------+-------------------+
| 18    | AUAV-5in          |
+-------+-------------------+
| 19    | AUAV-30in         |
+-------+-------------------+
| 100   | SITL              |
+-------+-------------------+




.. _ARSPD2_USE:

ARSPD2\_USE: Airspeed use
~~~~~~~~~~~~~~~~~~~~~~~~~


Enables airspeed use for automatic throttle modes and replaces control from THR\_TRIM\. Continues to display and log airspeed if set to 0\. Uses airspeed for control if set to 1\. Only uses airspeed when throttle \= 0 if set to 2 \(useful for gliders with airspeed sensors behind propellers\)\.


+-------+---------------------+
| Value | Meaning             |
+=======+=====================+
| 0     | DoNotUse            |
+-------+---------------------+
| 1     | Use                 |
+-------+---------------------+
| 2     | UseWhenZeroThrottle |
+-------+---------------------+




.. _ARSPD2_OFFSET:

ARSPD2\_OFFSET: Airspeed offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Airspeed calibration offset


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _ARSPD2_RATIO:

ARSPD2\_RATIO: Airspeed ratio
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Calibrates pitot tube pressure to velocity\. Increasing this value will indicate a higher airspeed at any given dynamic pressure\.


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _ARSPD2_PIN:

ARSPD2\_PIN: Airspeed pin
~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The pin number that the airspeed sensor is connected to for analog sensors\. Values for some autopilots are given as examples\. Search wiki for \"Analog pins\"\.


+-------+--------------------------------------+
| Value | Meaning                              |
+=======+======================================+
| -1    | Disabled                             |
+-------+--------------------------------------+
| 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 |
+-------+--------------------------------------+
| 5     | Navigator                            |
+-------+--------------------------------------+
| 13    | Pixhawk2_PM2/CubeOrange_PM2          |
+-------+--------------------------------------+
| 14    | CubeOrange                           |
+-------+--------------------------------------+
| 16    | Durandal                             |
+-------+--------------------------------------+
| 100   | PX4-v1                               |
+-------+--------------------------------------+




.. _ARSPD2_AUTOCAL:

ARSPD2\_AUTOCAL: Automatic airspeed ratio calibration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enables automatic adjustment of airspeed ratio during a calibration flight based on estimation of ground speed and true airspeed\. New ratio saved every 2 minutes if change is \> 5\%\. Should not be left enabled\.


.. _ARSPD2_TUBE_ORDR:

ARSPD2\_TUBE\_ORDR: Control pitot tube order
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters\. If you set this to 0 then the first \(often the top\) connector on the sensor needs to be the stagnation pressure \(the pressure at the tip of the pitot tube\)\. If set to 1 then the second \(often the bottom\) connector needs to be the stagnation pressure\. If set to 2 \(the default\) then the airspeed driver will accept either order\. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall\, which would otherwise be seen as a positive airspeed\.


+-------+-------------+
| Value | Meaning     |
+=======+=============+
| 0     | Normal      |
+-------+-------------+
| 1     | Swapped     |
+-------+-------------+
| 2     | Auto Detect |
+-------+-------------+




.. _ARSPD2_SKIP_CAL:

ARSPD2\_SKIP\_CAL: Skip airspeed offset calibration on startup
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to skip airspeed offset calibration on startup\, instead using the offset from the last calibration or requiring a manual calibration\. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot\.


+-------+---------------------------------------------------------------------------------------------------------------+
| Value | Meaning                                                                                                       |
+=======+===============================================================================================================+
| 0     | Disable                                                                                                       |
+-------+---------------------------------------------------------------------------------------------------------------+
| 1     | Do not require offset calibration before flight. Manual calibration should be performed during initial setup. |
+-------+---------------------------------------------------------------------------------------------------------------+
| 2     | Do not calibrate on start up. Manual calibration must be performed once per boot.                             |
+-------+---------------------------------------------------------------------------------------------------------------+




.. _ARSPD2_PSI_RANGE:

ARSPD2\_PSI\_RANGE: The PSI range of the device
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to set the PSI \(pounds per square inch\) range for your sensor\. You should not change this unless you examine the datasheet for your device


.. _ARSPD2_BUS:

ARSPD2\_BUS: Airspeed I2C bus
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Bus number of the I2C bus where the airspeed sensor is connected\. May not correspond to board\'s I2C bus number labels\. Retry another bus and reboot if airspeed sensor fails to initialize\.


+-------+---------+
| Value | Meaning |
+=======+=========+
| 0     | Bus0    |
+-------+---------+
| 1     | Bus1    |
+-------+---------+
| 2     | Bus2    |
+-------+---------+
| 3     | Bus3    |
+-------+---------+




.. _ARSPD2_DEVID:

ARSPD2\_DEVID: Airspeed ID
~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Airspeed sensor ID\, taking into account its type\, bus and instance


+----------+
| ReadOnly |
+==========+
| True     |
+----------+





.. _parameters_ARSPD_:

ARSPD\_ Parameters
------------------


.. _ARSPD_TYPE:

ARSPD\_TYPE: Airspeed type
~~~~~~~~~~~~~~~~~~~~~~~~~~


Type of airspeed sensor


+-------+-------------------+
| Value | Meaning           |
+=======+===================+
| 0     | None              |
+-------+-------------------+
| 1     | I2C-MS4525D0      |
+-------+-------------------+
| 2     | Analog            |
+-------+-------------------+
| 3     | I2C-MS5525        |
+-------+-------------------+
| 4     | I2C-MS5525 (0x76) |
+-------+-------------------+
| 5     | I2C-MS5525 (0x77) |
+-------+-------------------+
| 6     | I2C-SDP3X         |
+-------+-------------------+
| 7     | I2C-DLVR-5in      |
+-------+-------------------+
| 8     | DroneCAN          |
+-------+-------------------+
| 9     | I2C-DLVR-10in     |
+-------+-------------------+
| 10    | I2C-DLVR-20in     |
+-------+-------------------+
| 11    | I2C-DLVR-30in     |
+-------+-------------------+
| 12    | I2C-DLVR-60in     |
+-------+-------------------+
| 13    | NMEA water speed  |
+-------+-------------------+
| 14    | MSP               |
+-------+-------------------+
| 15    | ASP5033           |
+-------+-------------------+
| 16    | ExternalAHRS      |
+-------+-------------------+
| 17    | AUAV-10in         |
+-------+-------------------+
| 18    | AUAV-5in          |
+-------+-------------------+
| 19    | AUAV-30in         |
+-------+-------------------+
| 100   | SITL              |
+-------+-------------------+




.. _ARSPD_USE:

ARSPD\_USE: Airspeed use
~~~~~~~~~~~~~~~~~~~~~~~~


Enables airspeed use for automatic throttle modes and replaces control from THR\_TRIM\. Continues to display and log airspeed if set to 0\. Uses airspeed for control if set to 1\. Only uses airspeed when throttle \= 0 if set to 2 \(useful for gliders with airspeed sensors behind propellers\)\.


+-------+---------------------+
| Value | Meaning             |
+=======+=====================+
| 0     | DoNotUse            |
+-------+---------------------+
| 1     | Use                 |
+-------+---------------------+
| 2     | UseWhenZeroThrottle |
+-------+---------------------+




.. _ARSPD_OFFSET:

ARSPD\_OFFSET: Airspeed offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Airspeed calibration offset


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _ARSPD_RATIO:

ARSPD\_RATIO: Airspeed ratio
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Calibrates pitot tube pressure to velocity\. Increasing this value will indicate a higher airspeed at any given dynamic pressure\.


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _ARSPD_PIN:

ARSPD\_PIN: Airspeed pin
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The pin number that the airspeed sensor is connected to for analog sensors\. Values for some autopilots are given as examples\. Search wiki for \"Analog pins\"\.


+-------+--------------------------------------+
| Value | Meaning                              |
+=======+======================================+
| -1    | Disabled                             |
+-------+--------------------------------------+
| 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 |
+-------+--------------------------------------+
| 5     | Navigator                            |
+-------+--------------------------------------+
| 13    | Pixhawk2_PM2/CubeOrange_PM2          |
+-------+--------------------------------------+
| 14    | CubeOrange                           |
+-------+--------------------------------------+
| 16    | Durandal                             |
+-------+--------------------------------------+
| 100   | PX4-v1                               |
+-------+--------------------------------------+




.. _ARSPD_AUTOCAL:

ARSPD\_AUTOCAL: Automatic airspeed ratio calibration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enables automatic adjustment of airspeed ratio during a calibration flight based on estimation of ground speed and true airspeed\. New ratio saved every 2 minutes if change is \> 5\%\. Should not be left enabled\.


.. _ARSPD_TUBE_ORDR:

ARSPD\_TUBE\_ORDR: Control pitot tube order
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters\. If you set this to 0 then the first \(often the top\) connector on the sensor needs to be the stagnation pressure \(the pressure at the tip of the pitot tube\)\. If set to 1 then the second \(often the bottom\) connector needs to be the stagnation pressure\. If set to 2 \(the default\) then the airspeed driver will accept either order\. The reason you may wish to specify the order is it will allow your airspeed sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall\, which would otherwise be seen as a positive airspeed\.


+-------+-------------+
| Value | Meaning     |
+=======+=============+
| 0     | Normal      |
+-------+-------------+
| 1     | Swapped     |
+-------+-------------+
| 2     | Auto Detect |
+-------+-------------+




.. _ARSPD_SKIP_CAL:

ARSPD\_SKIP\_CAL: Skip airspeed offset calibration on startup
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to skip airspeed offset calibration on startup\, instead using the offset from the last calibration or requiring a manual calibration\. This may be desirable if the offset variance between flights for your sensor is low and you want to avoid having to cover the pitot tube on each boot\.


+-------+---------------------------------------------------------------------------------------------------------------+
| Value | Meaning                                                                                                       |
+=======+===============================================================================================================+
| 0     | Disable                                                                                                       |
+-------+---------------------------------------------------------------------------------------------------------------+
| 1     | Do not require offset calibration before flight. Manual calibration should be performed during initial setup. |
+-------+---------------------------------------------------------------------------------------------------------------+
| 2     | Do not calibrate on start up. Manual calibration must be performed once per boot.                             |
+-------+---------------------------------------------------------------------------------------------------------------+




.. _ARSPD_PSI_RANGE:

ARSPD\_PSI\_RANGE: The PSI range of the device
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This parameter allows you to set the PSI \(pounds per square inch\) range for your sensor\. You should not change this unless you examine the datasheet for your device


.. _ARSPD_BUS:

ARSPD\_BUS: Airspeed I2C bus
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Bus number of the I2C bus where the airspeed sensor is connected\. May not correspond to board\'s I2C bus number labels\. Retry another bus and reboot if airspeed sensor fails to initialize\.


+-------+---------+
| Value | Meaning |
+=======+=========+
| 0     | Bus0    |
+-------+---------+
| 1     | Bus1    |
+-------+---------+
| 2     | Bus2    |
+-------+---------+
| 3     | Bus3    |
+-------+---------+




.. _ARSPD_DEVID:

ARSPD\_DEVID: Airspeed ID
~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Airspeed sensor ID\, taking into account its type\, bus and instance


+----------+
| ReadOnly |
+==========+
| True     |
+----------+





.. _parameters_CUST_ROT:

CUST\_ROT Parameters
--------------------


.. _CUST_ROT_ENABLE:

CUST\_ROT\_ENABLE: Enable Custom rotations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

This enables custom rotations


+-------+---------+
| Value | Meaning |
+=======+=========+
| 0     | Disable |
+-------+---------+
| 1     | Enable  |
+-------+---------+





.. _parameters_CUST_ROT1_:

CUST\_ROT1\_ Parameters
-----------------------


.. _CUST_ROT1_ROLL:

CUST\_ROT1\_ROLL: Custom roll
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Custom euler roll\, euler 321 \(yaw\, pitch\, roll\) ordering


+---------+
| Units   |
+=========+
| degrees |
+---------+




.. _CUST_ROT1_PITCH:

CUST\_ROT1\_PITCH: Custom pitch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Custom euler pitch\, euler 321 \(yaw\, pitch\, roll\) ordering


+---------+
| Units   |
+=========+
| degrees |
+---------+




.. _CUST_ROT1_YAW:

CUST\_ROT1\_YAW: Custom yaw
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Custom euler yaw\, euler 321 \(yaw\, pitch\, roll\) ordering


+---------+
| Units   |
+=========+
| degrees |
+---------+





.. _parameters_CUST_ROT2_:

CUST\_ROT2\_ Parameters
-----------------------


.. _CUST_ROT2_ROLL:

CUST\_ROT2\_ROLL: Custom roll
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Custom euler roll\, euler 321 \(yaw\, pitch\, roll\) ordering


+---------+
| Units   |
+=========+
| degrees |
+---------+




.. _CUST_ROT2_PITCH:

CUST\_ROT2\_PITCH: Custom pitch
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Custom euler pitch\, euler 321 \(yaw\, pitch\, roll\) ordering


+---------+
| Units   |
+=========+
| degrees |
+---------+




.. _CUST_ROT2_YAW:

CUST\_ROT2\_YAW: Custom yaw
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Custom euler yaw\, euler 321 \(yaw\, pitch\, roll\) ordering


+---------+
| Units   |
+=========+
| degrees |
+---------+





.. _parameters_DDS:

DDS Parameters
--------------


.. _DDS_ENABLE:

DDS\_ENABLE: DDS enable
~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Enable DDS subsystem


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _DDS_UDP_PORT:

DDS\_UDP\_PORT: DDS UDP port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

UDP port number for DDS


+------------+
| Range      |
+============+
| 1 to 65535 |
+------------+




.. _DDS_DOMAIN_ID:

DDS\_DOMAIN\_ID: DDS DOMAIN ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Set the ROS\_DOMAIN\_ID


+----------+
| Range    |
+==========+
| 0 to 232 |
+----------+




.. _DDS_TIMEOUT_MS:

DDS\_TIMEOUT\_MS: DDS ping timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

The time in milliseconds the DDS client will wait for a response from the XRCE agent before reattempting\.


+-----------+------------+--------------+
| Increment | Range      | Units        |
+===========+============+==============+
| 1         | 1 to 10000 | milliseconds |
+-----------+------------+--------------+




.. _DDS_MAX_RETRY:

DDS\_MAX\_RETRY: DDS ping max attempts
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

The maximum number of times the DDS client will attempt to ping the XRCE agent before exiting\. Set to 0 to allow unlimited retries\.


+-----------+----------+
| Increment | Range    |
+===========+==========+
| 1         | 0 to 100 |
+-----------+----------+





.. _parameters_DDS_IP:

DDS\_IP Parameters
------------------


.. _DDS_IP0:

DDS\_IP0: IPv4 Address 1st byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: 192\.xxx\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _DDS_IP1:

DDS\_IP1: IPv4 Address 2nd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.168\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _DDS_IP2:

DDS\_IP2: IPv4 Address 3rd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.144\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _DDS_IP3:

DDS\_IP3: IPv4 Address 4th byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.xxx\.14


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+





.. _parameters_DID_:

DID\_ Parameters
----------------


.. _DID_ENABLE:

DID\_ENABLE: Enable ODID subsystem
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Enable ODID subsystem


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _DID_MAVPORT:

DID\_MAVPORT: MAVLink serial port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Serial port number to send OpenDroneID MAVLink messages to\. Can be \-1 if using DroneCAN\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| -1    | Disabled |
+-------+----------+
| 0     | Serial0  |
+-------+----------+
| 1     | Serial1  |
+-------+----------+
| 2     | Serial2  |
+-------+----------+
| 3     | Serial3  |
+-------+----------+
| 4     | Serial4  |
+-------+----------+
| 5     | Serial5  |
+-------+----------+
| 6     | Serial6  |
+-------+----------+




.. _DID_CANDRIVER:

DID\_CANDRIVER: DroneCAN driver number
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


DroneCAN driver index\, 0 to disable DroneCAN


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Driver1  |
+-------+----------+
| 2     | Driver2  |
+-------+----------+




.. _DID_OPTIONS:

DID\_OPTIONS: OpenDroneID options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Options for OpenDroneID subsystem


+-----+---------------------------+
| Bit | Meaning                   |
+=====+===========================+
| 0   | EnforceArming             |
+-----+---------------------------+
| 1   | AllowNonGPSPosition       |
+-----+---------------------------+
| 2   | LockUASIDOnFirstBasicIDRx |
+-----+---------------------------+




.. _DID_BARO_ACC:

DID\_BARO\_ACC: Barometer vertical accuraacy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Barometer Vertical Accuracy when installed in the vehicle\. Note this is dependent upon installation conditions and thus disabled by default


+--------+
| Units  |
+========+
| meters |
+--------+





.. _parameters_EAHRS:

EAHRS Parameters
----------------


.. _EAHRS_TYPE:

EAHRS\_TYPE: AHRS type
~~~~~~~~~~~~~~~~~~~~~~


Type of AHRS device


+-------+--------------+
| Value | Meaning      |
+=======+==============+
| 0     | None         |
+-------+--------------+
| 1     | VectorNav    |
+-------+--------------+
| 2     | MicroStrain5 |
+-------+--------------+
| 5     | InertialLabs |
+-------+--------------+
| 7     | MicroStrain7 |
+-------+--------------+




.. _EAHRS_RATE:

EAHRS\_RATE: AHRS data rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Requested rate for AHRS device


+-------+
| Units |
+=======+
| hertz |
+-------+




.. _EAHRS_OPTIONS:

EAHRS\_OPTIONS: External AHRS options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


External AHRS options bitmask


+-----+-------------------------------------------------------------+
| Bit | Meaning                                                     |
+=====+=============================================================+
| 0   | Vector Nav use uncompensated values for accel gyro and mag. |
+-----+-------------------------------------------------------------+




.. _EAHRS_SENSORS:

EAHRS\_SENSORS: External AHRS sensors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

External AHRS sensors bitmask


+-----+---------+
| Bit | Meaning |
+=====+=========+
| 0   | GPS     |
+-----+---------+
| 1   | IMU     |
+-----+---------+
| 2   | Baro    |
+-----+---------+
| 3   | Compass |
+-----+---------+




.. _EAHRS_LOG_RATE:

EAHRS\_LOG\_RATE: AHRS logging rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Logging rate for EARHS devices


+-------+
| Units |
+=======+
| hertz |
+-------+





.. _parameters_EFI:

EFI Parameters
--------------


.. _EFI_TYPE:

EFI\_TYPE: EFI communication type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

What method of communication is used for EFI \#1


+-------+---------------+
| Value | Meaning       |
+=======+===============+
| 0     | None          |
+-------+---------------+
| 1     | Serial-MS     |
+-------+---------------+
| 2     | NWPMU         |
+-------+---------------+
| 3     | Serial-Lutan  |
+-------+---------------+
| 4     | Loweheiser    |
+-------+---------------+
| 5     | DroneCAN      |
+-------+---------------+
| 6     | Currawong-ECU |
+-------+---------------+
| 7     | Scripting     |
+-------+---------------+
| 8     | Hirth         |
+-------+---------------+
| 9     | MAVLink       |
+-------+---------------+




.. _EFI_COEF1:

EFI\_COEF1: EFI Calibration Coefficient 1
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to calibrate fuel flow for MS protocol \(Slope\)\. This should be calculated from a log at constant fuel usage rate\. Plot \(ECYL\[0\]\.InjT\*EFI\.Rpm\)\/600\.0 to get the duty\_cycle\. Measure actual fuel usage in cm\^3\/min\, and set EFI\_COEF1 \= fuel\_usage\_cm3permin \/ duty\_cycle


+--------+
| Range  |
+========+
| 0 to 1 |
+--------+




.. _EFI_COEF2:

EFI\_COEF2: EFI Calibration Coefficient 2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to calibrate fuel flow for MS protocol \(Offset\)\. This can be used to correct for a non\-zero offset in the fuel consumption calculation of EFI\_COEF1


+---------+
| Range   |
+=========+
| 0 to 10 |
+---------+




.. _EFI_FUEL_DENS:

EFI\_FUEL\_DENS: ECU Fuel Density
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Used to calculate fuel consumption


+------------+---------------------------+
| Range      | Units                     |
+============+===========================+
| 0 to 10000 | kilograms per cubic meter |
+------------+---------------------------+





.. _parameters_EFI_THRLIN:

EFI\_THRLIN Parameters
----------------------


.. _EFI_THRLIN_EN:

EFI\_THRLIN\_EN: Enable throttle linearisation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Enable EFI throttle linearisation


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _EFI_THRLIN_COEF1:

EFI\_THRLIN\_COEF1: Throttle linearisation \- First Order
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

First Order Polynomial Coefficient\. \(\=1\, if throttle is first order polynomial trendline\)


+---------+
| Range   |
+=========+
| -1 to 1 |
+---------+




.. _EFI_THRLIN_COEF2:

EFI\_THRLIN\_COEF2: Throttle linearisation \- Second Order
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Second Order Polynomial Coefficient \(\=0\, if throttle is second order polynomial trendline\)


+---------+
| Range   |
+=========+
| -1 to 1 |
+---------+




.. _EFI_THRLIN_COEF3:

EFI\_THRLIN\_COEF3: Throttle linearisation \- Third Order
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Third Order Polynomial Coefficient\. \(\=0\, if throttle is third order polynomial trendline\)


+---------+
| Range   |
+=========+
| -1 to 1 |
+---------+




.. _EFI_THRLIN_OFS:

EFI\_THRLIN\_OFS: throttle linearization offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Offset for throttle linearization


+----------+
| Range    |
+==========+
| 0 to 100 |
+----------+





.. _parameters_ESC_TLM:

ESC\_TLM Parameters
-------------------


.. _ESC_TLM_MAV_OFS:

ESC\_TLM\_MAV\_OFS: ESC Telemetry mavlink offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Offset to apply to ESC numbers when reporting as ESC\_TELEMETRY packets over MAVLink\. This allows high numbered motors to be displayed as low numbered ESCs for convenience on GCS displays\. A value of 4 would send ESC on output 5 as ESC number 1 in ESC\_TELEMETRY packets


+-----------+---------+
| Increment | Range   |
+===========+=========+
| 1         | 0 to 31 |
+-----------+---------+





.. _parameters_FENCE_:

FENCE\_ Parameters
------------------


.. _FENCE_ENABLE:

FENCE\_ENABLE: Fence enable\/disable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Allows you to enable \(1\) or disable \(0\) the fence functionality\. Fences can still be enabled and disabled via mavlink or an RC option\, but these changes are not persisted\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _FENCE_ACTION:

FENCE\_ACTION: Fence Action
~~~~~~~~~~~~~~~~~~~~~~~~~~~


What action should be taken when fence is breached


+-------+-------------+
| Value | Meaning     |
+=======+=============+
| 0     | Report Only |
+-------+-------------+
| 1     | RTL or Land |
+-------+-------------+




.. _FENCE_RADIUS:

FENCE\_RADIUS: Circular Fence Radius
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Circle fence radius which when breached will cause an RTL


+-------------+--------+
| Range       | Units  |
+=============+========+
| 30 to 10000 | meters |
+-------------+--------+




.. _FENCE_MARGIN:

FENCE\_MARGIN: Fence Margin
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Distance that autopilot\'s should maintain from the fence to avoid a breach


+---------+--------+
| Range   | Units  |
+=========+========+
| 1 to 10 | meters |
+---------+--------+




.. _FENCE_TOTAL:

FENCE\_TOTAL: Fence polygon point total
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Number of polygon points saved in eeprom \(do not update manually\)


+---------+
| Range   |
+=========+
| 1 to 20 |
+---------+




.. _FENCE_OPTIONS:

FENCE\_OPTIONS: Fence options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


When bit 0 is set disable mode change following fence action until fence breach is cleared\. When bit 1 is set the allowable flight areas is the union of all polygon and circle fence areas instead of the intersection\, which means a fence breach occurs only if you are outside all of the fence areas\.


+-----+--------------------------------------------------------------------------+
| Bit | Meaning                                                                  |
+=====+==========================================================================+
| 0   | Disable mode change following fence action until fence breach is cleared |
+-----+--------------------------------------------------------------------------+
| 1   | Allow union of inclusion areas                                           |
+-----+--------------------------------------------------------------------------+
| 2   | Notify on margin breaches                                                |
+-----+--------------------------------------------------------------------------+




.. _FENCE_NTF_FREQ:

FENCE\_NTF\_FREQ: Fence margin notification frequency in hz
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

When bit 2 of FENCE\_OPTIONS is set this parameter controls the frequency of margin breach notifications\. If set to 0 only new margin breaches are notified\.


+---------+-------+
| Range   | Units |
+=========+=======+
| 0 to 10 | hertz |
+---------+-------+





.. _parameters_FFT_:

FFT\_ Parameters
----------------


.. _FFT_ENABLE:

FFT\_ENABLE: Enable
~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Enable Gyro FFT analyser


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _FFT_MINHZ:

FFT\_MINHZ: Minimum Frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Lower bound of FFT frequency detection in Hz\. On larger vehicles the minimum motor frequency is likely to be significantly lower than for smaller vehicles\.


+-----------+-------+
| Range     | Units |
+===========+=======+
| 20 to 400 | hertz |
+-----------+-------+




.. _FFT_MAXHZ:

FFT\_MAXHZ: Maximum Frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Upper bound of FFT frequency detection in Hz\. On smaller vehicles the maximum motor frequency is likely to be significantly higher than for larger vehicles\.


+-----------+-------+
| Range     | Units |
+===========+=======+
| 20 to 495 | hertz |
+-----------+-------+




.. _FFT_SAMPLE_MODE:

FFT\_SAMPLE\_MODE: Sample Mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Sampling mode \(and therefore rate\)\. 0\: Gyro rate sampling\, 1\: Fast loop rate sampling\, 2\: Fast loop rate \/ 2 sampling\, 3\: Fast loop rate \/ 3 sampling\. Takes effect on reboot\.


+--------+
| Range  |
+========+
| 0 to 4 |
+--------+




.. _FFT_WINDOW_SIZE:

FFT\_WINDOW\_SIZE: FFT window size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Size of window to be used in FFT calculations\. Takes effect on reboot\. Must be a power of 2 and between 32 and 512\. Larger windows give greater frequency resolution but poorer time resolution\, consume more CPU time and may not be appropriate for all vehicles\. Time and frequency resolution are given by the sample\-rate \/ window\-size\. Windows of 256 are only really recommended for F7 class boards\, windows of 512 or more H7 class\.


+------------+
| Range      |
+============+
| 32 to 1024 |
+------------+




.. _FFT_WINDOW_OLAP:

FFT\_WINDOW\_OLAP: FFT window overlap
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Percentage of window to be overlapped before another frame is process\. Takes effect on reboot\. A good default is 50\% overlap\. Higher overlap results in more processed frames but not necessarily more temporal resolution\. Lower overlap results in lost information at the frame edges\.


+----------+
| Range    |
+==========+
| 0 to 0.9 |
+----------+




.. _FFT_FREQ_HOVER:

FFT\_FREQ\_HOVER: FFT learned hover frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The learned hover noise frequency


+----------+
| Range    |
+==========+
| 0 to 250 |
+----------+




.. _FFT_THR_REF:

FFT\_THR\_REF: FFT learned thrust reference
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

FFT learned thrust reference for the hover frequency and FFT minimum frequency\.


+-------------+
| Range       |
+=============+
| 0.01 to 0.9 |
+-------------+




.. _FFT_SNR_REF:

FFT\_SNR\_REF: FFT SNR reference threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

FFT SNR reference threshold in dB at which a signal is determined to be present\.


+--------------+
| Range        |
+==============+
| 0.0 to 100.0 |
+--------------+




.. _FFT_ATT_REF:

FFT\_ATT\_REF: FFT attenuation for bandwidth calculation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

FFT attenuation level in dB for bandwidth calculation and peak detection\. The bandwidth is calculated by comparing peak power output with the attenuated version\. The default of 15 has shown to be a good compromise in both simulations and real flight\.


+----------+
| Range    |
+==========+
| 0 to 100 |
+----------+




.. _FFT_BW_HOVER:

FFT\_BW\_HOVER: FFT learned bandwidth at hover
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

FFT learned bandwidth at hover for the attenuation frequencies\.


+----------+
| Range    |
+==========+
| 0 to 200 |
+----------+




.. _FFT_HMNC_FIT:

FFT\_HMNC\_FIT: FFT harmonic fit frequency threshold
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

FFT harmonic fit frequency threshold percentage at which a signal of the appropriate frequency is determined to be the harmonic of another\. Signals that have a harmonic relationship that varies at most by this percentage are considered harmonics of each other for the purpose of selecting the harmonic notch frequency\. If a match is found then the lower frequency harmonic is always used as the basis for the dynamic harmonic notch\. A value of zero completely disables harmonic matching\.


+----------+
| Range    |
+==========+
| 0 to 100 |
+----------+




.. _FFT_HMNC_PEAK:

FFT\_HMNC\_PEAK: FFT harmonic peak target
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The FFT harmonic peak target that should be returned by FTN1\.PkAvg\. The resulting value will be used by the harmonic notch if configured to track the FFT frequency\. By default the appropriate peak is auto\-detected based on the harmonic fit between peaks and the energy\-weighted average frequency on roll on pitch is used\. Setting this to 1 will always target the highest energy peak\. Setting this to 2 will target the highest energy peak that is lower in frequency than the highest energy peak\. Setting this to 3 will target the highest energy peak that is higher in frequency than the highest energy peak\. Setting this to 4 will target the highest energy peak on the roll axis only and only the roll frequency will be used \(some vehicles have a much more pronounced peak on roll\)\. Setting this to 5 will target the highest energy peak on the pitch axis only and only the pitch frequency will be used \(some vehicles have a much more pronounced peak on roll\)\.


+-------+--------------------------+
| Value | Meaning                  |
+=======+==========================+
| 0     | Auto                     |
+-------+--------------------------+
| 1     | Center Frequency         |
+-------+--------------------------+
| 2     | Lower-Shoulder Frequency |
+-------+--------------------------+
| 3     | Upper-Shoulder Frequency |
+-------+--------------------------+
| 4     | Roll-Axis                |
+-------+--------------------------+
| 5     | Pitch-Axis               |
+-------+--------------------------+




.. _FFT_NUM_FRAMES:

FFT\_NUM\_FRAMES: FFT output frames to retain and average
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Number of output frequency frames to retain and average in order to calculate final frequencies\. Averaging output frames can drastically reduce noise and jitter at the cost of latency as long as the input is stable\. The default is to perform no averaging\. For rapidly changing frequencies \(e\.g\. smaller aircraft\) fewer frames should be averaged\.


+--------+
| Range  |
+========+
| 0 to 8 |
+--------+




.. _FFT_OPTIONS:

FFT\_OPTIONS: FFT options
~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

FFT configuration options\. Values\: 1\:Apply the FFT \*after\* the filter bank\,2\:Check noise at the motor frequencies using ESC data as a reference


+-----+------------------------+
| Bit | Meaning                |
+=====+========================+
| 0   | Enable post-filter FFT |
+-----+------------------------+
| 1   | Check motor noise      |
+-----+------------------------+





.. _parameters_FILT1_:

FILT1\_ Parameters
------------------


.. _FILT1_TYPE:

FILT1\_TYPE: Filter Type
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Filter Type


+-------+--------------+
| Value | Meaning      |
+=======+==============+
| 0     | Disable      |
+-------+--------------+
| 1     | Notch Filter |
+-------+--------------+




.. _FILT1_NOTCH_FREQ:

FILT1\_NOTCH\_FREQ: Notch Filter center frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter center frequency in Hz\.


+-----------+-------+
| Range     | Units |
+===========+=======+
| 10 to 495 | hertz |
+-----------+-------+




.. _FILT1_NOTCH_Q:

FILT1\_NOTCH\_Q: Notch Filter quality factor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter quality factor given by the notch centre frequency divided by its bandwidth\.


+---------+
| Range   |
+=========+
| 1 to 10 |
+---------+




.. _FILT1_NOTCH_ATT:

FILT1\_NOTCH\_ATT: Notch Filter attenuation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter attenuation in dB\.


+---------+---------+
| Range   | Units   |
+=========+=========+
| 5 to 50 | decibel |
+---------+---------+





.. _parameters_FILT2_:

FILT2\_ Parameters
------------------


.. _FILT2_TYPE:

FILT2\_TYPE: Filter Type
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Filter Type


+-------+--------------+
| Value | Meaning      |
+=======+==============+
| 0     | Disable      |
+-------+--------------+
| 1     | Notch Filter |
+-------+--------------+




.. _FILT2_NOTCH_FREQ:

FILT2\_NOTCH\_FREQ: Notch Filter center frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter center frequency in Hz\.


+-----------+-------+
| Range     | Units |
+===========+=======+
| 10 to 495 | hertz |
+-----------+-------+




.. _FILT2_NOTCH_Q:

FILT2\_NOTCH\_Q: Notch Filter quality factor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter quality factor given by the notch centre frequency divided by its bandwidth\.


+---------+
| Range   |
+=========+
| 1 to 10 |
+---------+




.. _FILT2_NOTCH_ATT:

FILT2\_NOTCH\_ATT: Notch Filter attenuation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter attenuation in dB\.


+---------+---------+
| Range   | Units   |
+=========+=========+
| 5 to 50 | decibel |
+---------+---------+





.. _parameters_FILT3_:

FILT3\_ Parameters
------------------


.. _FILT3_TYPE:

FILT3\_TYPE: Filter Type
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Filter Type


+-------+--------------+
| Value | Meaning      |
+=======+==============+
| 0     | Disable      |
+-------+--------------+
| 1     | Notch Filter |
+-------+--------------+




.. _FILT3_NOTCH_FREQ:

FILT3\_NOTCH\_FREQ: Notch Filter center frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter center frequency in Hz\.


+-----------+-------+
| Range     | Units |
+===========+=======+
| 10 to 495 | hertz |
+-----------+-------+




.. _FILT3_NOTCH_Q:

FILT3\_NOTCH\_Q: Notch Filter quality factor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter quality factor given by the notch centre frequency divided by its bandwidth\.


+---------+
| Range   |
+=========+
| 1 to 10 |
+---------+




.. _FILT3_NOTCH_ATT:

FILT3\_NOTCH\_ATT: Notch Filter attenuation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter attenuation in dB\.


+---------+---------+
| Range   | Units   |
+=========+=========+
| 5 to 50 | decibel |
+---------+---------+





.. _parameters_FILT4_:

FILT4\_ Parameters
------------------


.. _FILT4_TYPE:

FILT4\_TYPE: Filter Type
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Filter Type


+-------+--------------+
| Value | Meaning      |
+=======+==============+
| 0     | Disable      |
+-------+--------------+
| 1     | Notch Filter |
+-------+--------------+




.. _FILT4_NOTCH_FREQ:

FILT4\_NOTCH\_FREQ: Notch Filter center frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter center frequency in Hz\.


+-----------+-------+
| Range     | Units |
+===========+=======+
| 10 to 495 | hertz |
+-----------+-------+




.. _FILT4_NOTCH_Q:

FILT4\_NOTCH\_Q: Notch Filter quality factor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter quality factor given by the notch centre frequency divided by its bandwidth\.


+---------+
| Range   |
+=========+
| 1 to 10 |
+---------+




.. _FILT4_NOTCH_ATT:

FILT4\_NOTCH\_ATT: Notch Filter attenuation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter attenuation in dB\.


+---------+---------+
| Range   | Units   |
+=========+=========+
| 5 to 50 | decibel |
+---------+---------+





.. _parameters_FILT5_:

FILT5\_ Parameters
------------------


.. _FILT5_TYPE:

FILT5\_TYPE: Filter Type
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Filter Type


+-------+--------------+
| Value | Meaning      |
+=======+==============+
| 0     | Disable      |
+-------+--------------+
| 1     | Notch Filter |
+-------+--------------+




.. _FILT5_NOTCH_FREQ:

FILT5\_NOTCH\_FREQ: Notch Filter center frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter center frequency in Hz\.


+-----------+-------+
| Range     | Units |
+===========+=======+
| 10 to 495 | hertz |
+-----------+-------+




.. _FILT5_NOTCH_Q:

FILT5\_NOTCH\_Q: Notch Filter quality factor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter quality factor given by the notch centre frequency divided by its bandwidth\.


+---------+
| Range   |
+=========+
| 1 to 10 |
+---------+




.. _FILT5_NOTCH_ATT:

FILT5\_NOTCH\_ATT: Notch Filter attenuation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter attenuation in dB\.


+---------+---------+
| Range   | Units   |
+=========+=========+
| 5 to 50 | decibel |
+---------+---------+





.. _parameters_FILT6_:

FILT6\_ Parameters
------------------


.. _FILT6_TYPE:

FILT6\_TYPE: Filter Type
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Filter Type


+-------+--------------+
| Value | Meaning      |
+=======+==============+
| 0     | Disable      |
+-------+--------------+
| 1     | Notch Filter |
+-------+--------------+




.. _FILT6_NOTCH_FREQ:

FILT6\_NOTCH\_FREQ: Notch Filter center frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter center frequency in Hz\.


+-----------+-------+
| Range     | Units |
+===========+=======+
| 10 to 495 | hertz |
+-----------+-------+




.. _FILT6_NOTCH_Q:

FILT6\_NOTCH\_Q: Notch Filter quality factor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter quality factor given by the notch centre frequency divided by its bandwidth\.


+---------+
| Range   |
+=========+
| 1 to 10 |
+---------+




.. _FILT6_NOTCH_ATT:

FILT6\_NOTCH\_ATT: Notch Filter attenuation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter attenuation in dB\.


+---------+---------+
| Range   | Units   |
+=========+=========+
| 5 to 50 | decibel |
+---------+---------+





.. _parameters_FILT7_:

FILT7\_ Parameters
------------------


.. _FILT7_TYPE:

FILT7\_TYPE: Filter Type
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Filter Type


+-------+--------------+
| Value | Meaning      |
+=======+==============+
| 0     | Disable      |
+-------+--------------+
| 1     | Notch Filter |
+-------+--------------+




.. _FILT7_NOTCH_FREQ:

FILT7\_NOTCH\_FREQ: Notch Filter center frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter center frequency in Hz\.


+-----------+-------+
| Range     | Units |
+===========+=======+
| 10 to 495 | hertz |
+-----------+-------+




.. _FILT7_NOTCH_Q:

FILT7\_NOTCH\_Q: Notch Filter quality factor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter quality factor given by the notch centre frequency divided by its bandwidth\.


+---------+
| Range   |
+=========+
| 1 to 10 |
+---------+




.. _FILT7_NOTCH_ATT:

FILT7\_NOTCH\_ATT: Notch Filter attenuation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter attenuation in dB\.


+---------+---------+
| Range   | Units   |
+=========+=========+
| 5 to 50 | decibel |
+---------+---------+





.. _parameters_FILT8_:

FILT8\_ Parameters
------------------


.. _FILT8_TYPE:

FILT8\_TYPE: Filter Type
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Filter Type


+-------+--------------+
| Value | Meaning      |
+=======+==============+
| 0     | Disable      |
+-------+--------------+
| 1     | Notch Filter |
+-------+--------------+




.. _FILT8_NOTCH_FREQ:

FILT8\_NOTCH\_FREQ: Notch Filter center frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter center frequency in Hz\.


+-----------+-------+
| Range     | Units |
+===========+=======+
| 10 to 495 | hertz |
+-----------+-------+




.. _FILT8_NOTCH_Q:

FILT8\_NOTCH\_Q: Notch Filter quality factor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter quality factor given by the notch centre frequency divided by its bandwidth\.


+---------+
| Range   |
+=========+
| 1 to 10 |
+---------+




.. _FILT8_NOTCH_ATT:

FILT8\_NOTCH\_ATT: Notch Filter attenuation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Notch Filter attenuation in dB\.


+---------+---------+
| Range   | Units   |
+=========+=========+
| 5 to 50 | decibel |
+---------+---------+





.. _parameters_FRSKY_:

FRSKY\_ Parameters
------------------


.. _FRSKY_UPLINK_ID:

FRSKY\_UPLINK\_ID: Uplink sensor id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Change the uplink sensor id \(SPort only\)


+-------+---------+
| Value | Meaning |
+=======+=========+
| -1    | Disable |
+-------+---------+
| 7     | 7       |
+-------+---------+
| 8     | 8       |
+-------+---------+
| 9     | 9       |
+-------+---------+
| 10    | 10      |
+-------+---------+
| 11    | 11      |
+-------+---------+
| 12    | 12      |
+-------+---------+
| 13    | 13      |
+-------+---------+
| 14    | 14      |
+-------+---------+
| 15    | 15      |
+-------+---------+
| 16    | 16      |
+-------+---------+
| 17    | 17      |
+-------+---------+
| 18    | 18      |
+-------+---------+
| 19    | 19      |
+-------+---------+
| 20    | 20      |
+-------+---------+
| 21    | 21      |
+-------+---------+
| 22    | 22      |
+-------+---------+
| 23    | 23      |
+-------+---------+
| 24    | 24      |
+-------+---------+
| 25    | 25      |
+-------+---------+
| 26    | 26      |
+-------+---------+




.. _FRSKY_DNLINK1_ID:

FRSKY\_DNLINK1\_ID: First downlink sensor id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Change the first extra downlink sensor id \(SPort only\)


+-------+---------+
| Value | Meaning |
+=======+=========+
| -1    | Disable |
+-------+---------+
| 7     | 7       |
+-------+---------+
| 8     | 8       |
+-------+---------+
| 9     | 9       |
+-------+---------+
| 10    | 10      |
+-------+---------+
| 11    | 11      |
+-------+---------+
| 12    | 12      |
+-------+---------+
| 13    | 13      |
+-------+---------+
| 14    | 14      |
+-------+---------+
| 15    | 15      |
+-------+---------+
| 16    | 16      |
+-------+---------+
| 17    | 17      |
+-------+---------+
| 18    | 18      |
+-------+---------+
| 19    | 19      |
+-------+---------+
| 20    | 20      |
+-------+---------+
| 21    | 21      |
+-------+---------+
| 22    | 22      |
+-------+---------+
| 23    | 23      |
+-------+---------+
| 24    | 24      |
+-------+---------+
| 25    | 25      |
+-------+---------+
| 26    | 26      |
+-------+---------+




.. _FRSKY_DNLINK2_ID:

FRSKY\_DNLINK2\_ID: Second downlink sensor id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Change the second extra downlink sensor id \(SPort only\)


+-------+---------+
| Value | Meaning |
+=======+=========+
| -1    | Disable |
+-------+---------+
| 7     | 7       |
+-------+---------+
| 8     | 8       |
+-------+---------+
| 9     | 9       |
+-------+---------+
| 10    | 10      |
+-------+---------+
| 11    | 11      |
+-------+---------+
| 12    | 12      |
+-------+---------+
| 13    | 13      |
+-------+---------+
| 14    | 14      |
+-------+---------+
| 15    | 15      |
+-------+---------+
| 16    | 16      |
+-------+---------+
| 17    | 17      |
+-------+---------+
| 18    | 18      |
+-------+---------+
| 19    | 19      |
+-------+---------+
| 20    | 20      |
+-------+---------+
| 21    | 21      |
+-------+---------+
| 22    | 22      |
+-------+---------+
| 23    | 23      |
+-------+---------+
| 24    | 24      |
+-------+---------+
| 25    | 25      |
+-------+---------+
| 26    | 26      |
+-------+---------+




.. _FRSKY_DNLINK_ID:

FRSKY\_DNLINK\_ID: Default downlink sensor id
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Change the default downlink sensor id \(SPort only\)


+-------+---------+
| Value | Meaning |
+=======+=========+
| -1    | Disable |
+-------+---------+
| 7     | 7       |
+-------+---------+
| 8     | 8       |
+-------+---------+
| 9     | 9       |
+-------+---------+
| 10    | 10      |
+-------+---------+
| 11    | 11      |
+-------+---------+
| 12    | 12      |
+-------+---------+
| 13    | 13      |
+-------+---------+
| 14    | 14      |
+-------+---------+
| 15    | 15      |
+-------+---------+
| 16    | 16      |
+-------+---------+
| 17    | 17      |
+-------+---------+
| 18    | 18      |
+-------+---------+
| 19    | 19      |
+-------+---------+
| 20    | 20      |
+-------+---------+
| 21    | 21      |
+-------+---------+
| 22    | 22      |
+-------+---------+
| 23    | 23      |
+-------+---------+
| 24    | 24      |
+-------+---------+
| 25    | 25      |
+-------+---------+
| 26    | 26      |
+-------+---------+
| 27    | 27      |
+-------+---------+




.. _FRSKY_OPTIONS:

FRSKY\_OPTIONS: FRSky Telemetry Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


A bitmask to set some FRSky Telemetry specific options


+-----+------------------------------+
| Bit | Meaning                      |
+=====+==============================+
| 0   | EnableAirspeedAndGroundspeed |
+-----+------------------------------+





.. _parameters_GEN_:

GEN\_ Parameters
----------------


.. _GEN_TYPE:

GEN\_TYPE: Generator type
~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Generator type


+-------+------------------------+
| Value | Meaning                |
+=======+========================+
| 0     | Disabled               |
+-------+------------------------+
| 1     | IE 650w 800w Fuel Cell |
+-------+------------------------+
| 2     | IE 2.4kW Fuel Cell     |
+-------+------------------------+
| 3     | Richenpower            |
+-------+------------------------+
| 4     | Loweheiser             |
+-------+------------------------+




.. _GEN_OPTIONS:

GEN\_OPTIONS: Generator Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Bitmask of options for generators


+-----+----------------------------------------+
| Bit | Meaning                                |
+=====+========================================+
| 0   | Suppress Maintenance-Required Warnings |
+-----+----------------------------------------+





.. _parameters_GRIP_:

GRIP\_ Parameters
-----------------


.. _GRIP_ENABLE:

GRIP\_ENABLE: Gripper Enable\/Disable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Gripper enable\/disable


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _GRIP_TYPE:

GRIP\_TYPE: Gripper Type
~~~~~~~~~~~~~~~~~~~~~~~~


Gripper enable\/disable


+-------+---------+
| Value | Meaning |
+=======+=========+
| 0     | None    |
+-------+---------+
| 1     | Servo   |
+-------+---------+
| 2     | EPM     |
+-------+---------+




.. _GRIP_GRAB:

GRIP\_GRAB: Gripper Grab PWM
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

PWM value in microseconds sent to Gripper to initiate grabbing the cargo


+--------------+---------------------+
| Range        | Units               |
+==============+=====================+
| 1000 to 2000 | PWM in microseconds |
+--------------+---------------------+




.. _GRIP_RELEASE:

GRIP\_RELEASE: Gripper Release PWM
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

PWM value in microseconds sent to Gripper to release the cargo


+--------------+---------------------+
| Range        | Units               |
+==============+=====================+
| 1000 to 2000 | PWM in microseconds |
+--------------+---------------------+




.. _GRIP_NEUTRAL:

GRIP\_NEUTRAL: Neutral PWM
~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

PWM value in microseconds sent to grabber when not grabbing or releasing


+--------------+---------------------+
| Range        | Units               |
+==============+=====================+
| 1000 to 2000 | PWM in microseconds |
+--------------+---------------------+




.. _GRIP_REGRAB:

GRIP\_REGRAB: EPM Gripper Regrab interval
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Time in seconds that EPM gripper will regrab the cargo to ensure grip has not weakened\; 0 to disable


+----------+---------+
| Range    | Units   |
+==========+=========+
| 0 to 255 | seconds |
+----------+---------+




.. _GRIP_CAN_ID:

GRIP\_CAN\_ID: EPM UAVCAN Hardpoint ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Refer to https\:\/\/docs\.zubax\.com\/opengrab\_epm\_v3\#UAVCAN\_interface


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _GRIP_AUTOCLOSE:

GRIP\_AUTOCLOSE: Gripper Autoclose time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Time in seconds that gripper close the gripper after opening\; 0 to disable


+-------------+---------+
| Range       | Units   |
+=============+=========+
| 0.25 to 255 | seconds |
+-------------+---------+





.. _parameters_KDE_:

KDE\_ Parameters
----------------


.. _KDE_NPOLE:

KDE\_NPOLE: Number of motor poles
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the number of motor poles to calculate the correct RPM value



.. _parameters_LOG:

LOG Parameters
--------------


.. _LOG_BACKEND_TYPE:

LOG\_BACKEND\_TYPE: AP\_Logger Backend Storage type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Bitmap of what Logger backend types to enable\. Block\-based logging is available on SITL and boards with dataflash chips\. Multiple backends can be selected\.


+-----+---------+
| Bit | Meaning |
+=====+=========+
| 0   | File    |
+-----+---------+
| 1   | MAVLink |
+-----+---------+
| 2   | Block   |
+-----+---------+




.. _LOG_FILE_BUFSIZE:

LOG\_FILE\_BUFSIZE: Logging File and Block Backend buffer size max \(in kibibytes\)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The File and Block backends use a buffer to store data before writing to the block device\.  Raising this value may reduce \"gaps\" in your SD card logging but increases memory usage\.  This buffer size may be reduced to free up available memory


+----------+-----------+
| Range    | Units     |
+==========+===========+
| 4 to 200 | kibibytes |
+----------+-----------+




.. _LOG_DISARMED:

LOG\_DISARMED: Enable logging while disarmed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


If LOG\_DISARMED is set to 1 then logging will be enabled at all times including when disarmed\. Logging before arming can make for very large logfiles but can help a lot when tracking down startup issues and is necessary if logging of EKF replay data is selected via the LOG\_REPLAY parameter\. If LOG\_DISARMED is set to 2\, then logging will be enabled when disarmed\, but not if a USB connection is detected\. This can be used to prevent unwanted data logs being generated when the vehicle is connected via USB for log downloading or parameter changes\. If LOG\_DISARMED is set to 3 then logging will happen while disarmed\, but if the vehicle never arms then the logs using the filesystem backend will be discarded on the next boot\.


+-------+--------------------------------------+
| Value | Meaning                              |
+=======+======================================+
| 0     | Disabled                             |
+-------+--------------------------------------+
| 1     | Enabled                              |
+-------+--------------------------------------+
| 2     | Disabled on USB connection           |
+-------+--------------------------------------+
| 3     | Discard log on reboot if never armed |
+-------+--------------------------------------+




.. _LOG_REPLAY:

LOG\_REPLAY: Enable logging of information needed for Replay
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


If LOG\_REPLAY is set to 1 then the EKF2 and EKF3 state estimators will log detailed information needed for diagnosing problems with the Kalman filter\. LOG\_DISARMED must be set to 1 or 2 or else the log will not contain the pre\-flight data required for replay testing of the EKF\'s\. It is suggested that you also raise LOG\_FILE\_BUFSIZE to give more buffer space for logging and use a high quality microSD card to ensure no sensor data is lost\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _LOG_FILE_DSRMROT:

LOG\_FILE\_DSRMROT: Stop logging to current file on disarm
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


When set\, the current log file is closed when the vehicle is disarmed\.  If LOG\_DISARMED is set then a fresh log will be opened\. Applies to the File and Block logging backends\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _LOG_MAV_BUFSIZE:

LOG\_MAV\_BUFSIZE: Maximum AP\_Logger MAVLink Backend buffer size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Maximum amount of memory to allocate to AP\_Logger\-over\-mavlink


+-----------+
| Units     |
+===========+
| kilobytes |
+-----------+




.. _LOG_FILE_TIMEOUT:

LOG\_FILE\_TIMEOUT: Timeout before giving up on file writes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This controls the amount of time before failing writes to a log file cause the file to be closed and logging stopped\.


+---------+
| Units   |
+=========+
| seconds |
+---------+




.. _LOG_FILE_MB_FREE:

LOG\_FILE\_MB\_FREE: Old logs on the SD card will be deleted to maintain this amount of free space
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Set this such that the free space is larger than your largest typical flight log


+-----------+----------+
| Range     | Units    |
+===========+==========+
| 2 to 1000 | megabyte |
+-----------+----------+




.. _LOG_FILE_RATEMAX:

LOG\_FILE\_RATEMAX: Maximum logging rate for file backend
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the maximum rate that streaming log messages will be logged to the file backend\. A value of zero means that rate limiting is disabled\.


+-----------+-----------+-------+
| Increment | Range     | Units |
+===========+===========+=======+
| 0.1       | 0 to 1000 | hertz |
+-----------+-----------+-------+




.. _LOG_MAV_RATEMAX:

LOG\_MAV\_RATEMAX: Maximum logging rate for mavlink backend
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the maximum rate that streaming log messages will be logged to the mavlink backend\. A value of zero means that rate limiting is disabled\.


+-----------+-----------+-------+
| Increment | Range     | Units |
+===========+===========+=======+
| 0.1       | 0 to 1000 | hertz |
+-----------+-----------+-------+




.. _LOG_BLK_RATEMAX:

LOG\_BLK\_RATEMAX: Maximum logging rate for block backend
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the maximum rate that streaming log messages will be logged to the block backend\. A value of zero means that rate limiting is disabled\.


+-----------+-----------+-------+
| Increment | Range     | Units |
+===========+===========+=======+
| 0.1       | 0 to 1000 | hertz |
+-----------+-----------+-------+




.. _LOG_DARM_RATEMAX:

LOG\_DARM\_RATEMAX: Maximum logging rate when disarmed
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This sets the maximum rate that streaming log messages will be logged to any backend when disarmed\. A value of zero means that the normal backend rate limit is applied\.


+-----------+-----------+-------+
| Increment | Range     | Units |
+===========+===========+=======+
| 0.1       | 0 to 1000 | hertz |
+-----------+-----------+-------+




.. _LOG_MAX_FILES:

LOG\_MAX\_FILES: Maximum number of log files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

This sets the maximum number of log file that will be written on dataflash or sd card before starting to rotate log number\. Limit is capped at 500 logs\.


+-----------+----------+
| Increment | Range    |
+===========+==========+
| 1         | 2 to 500 |
+-----------+----------+





.. _parameters_MSP:

MSP Parameters
--------------


.. _MSP_OSD_NCELLS:

MSP\_OSD\_NCELLS: Cell count override
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Used for average cell voltage calculation


+-------+---------+
| Value | Meaning |
+=======+=========+
| 0     | Auto    |
+-------+---------+
| 1     | 1       |
+-------+---------+
| 2     | 2       |
+-------+---------+
| 3     | 3       |
+-------+---------+
| 4     | 4       |
+-------+---------+
| 5     | 5       |
+-------+---------+
| 6     | 6       |
+-------+---------+
| 7     | 7       |
+-------+---------+
| 8     | 8       |
+-------+---------+
| 9     | 9       |
+-------+---------+
| 10    | 10      |
+-------+---------+
| 11    | 11      |
+-------+---------+
| 12    | 12      |
+-------+---------+
| 13    | 13      |
+-------+---------+
| 14    | 14      |
+-------+---------+




.. _MSP_OPTIONS:

MSP\_OPTIONS: MSP OSD Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


A bitmask to set some MSP specific options\: EnableTelemetryMode\-allows \"push\" mode telemetry when only rx line of OSD ic connected to autopilot\,  EnableBTFLFonts\-uses indexes corresponding to Betaflight fonts if OSD uses those instead of ArduPilot fonts\. EnableINAVFonts uses INAV fonts and overrides EnableBTFLFonts if that option is enabled\.


+-----+---------------------+
| Bit | Meaning             |
+=====+=====================+
| 0   | EnableTelemetryMode |
+-----+---------------------+
| 1   | unused              |
+-----+---------------------+
| 2   | EnableBTFLFonts     |
+-----+---------------------+
| 3   | EnableINAVFonts     |
+-----+---------------------+





.. _parameters_NET_:

NET\_ Parameters
----------------


.. _NET_ENABLE:

NET\_ENABLE: Networking Enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Networking Enable


+-------+---------+
| Value | Meaning |
+=======+=========+
| 0     | Disable |
+-------+---------+
| 1     | Enable  |
+-------+---------+




.. _NET_NETMASK:

NET\_NETMASK: IP Subnet mask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Allows setting static subnet mask\. The value is a count of consecutive bits\. Examples\: 24 \= 255\.255\.255\.0\, 16 \= 255\.255\.0\.0


+---------+
| Range   |
+=========+
| 0 to 32 |
+---------+




.. _NET_DHCP:

NET\_DHCP: DHCP client
~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Enable\/Disable DHCP client


+-------+---------+
| Value | Meaning |
+=======+=========+
| 0     | Disable |
+-------+---------+
| 1     | Enable  |
+-------+---------+




.. _NET_TESTS:

NET\_TESTS: Test enable flags
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Enable\/Disable networking tests


+-----+------------------+
| Bit | Meaning          |
+=====+==================+
| 0   | UDP echo test    |
+-----+------------------+
| 1   | TCP echo test    |
+-----+------------------+
| 2   | TCP discard test |
+-----+------------------+
| 3   | TCP reflect test |
+-----+------------------+




.. _NET_OPTIONS:

NET\_OPTIONS: Networking options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Networking options


+-----+--------------------------------+
| Bit | Meaning                        |
+=====+================================+
| 0   | EnablePPP Ethernet gateway     |
+-----+--------------------------------+
| 1   | Enable CAN1 multicast endpoint |
+-----+--------------------------------+
| 2   | Enable CAN2 multicast endpoint |
+-----+--------------------------------+
| 3   | Enable CAN1 multicast bridged  |
+-----+--------------------------------+
| 4   | Enable CAN2 multicast bridged  |
+-----+--------------------------------+
| 5   | DisablePPPTimeout              |
+-----+--------------------------------+
| 6   | DisablePPPEchoLimit            |
+-----+--------------------------------+
| 7   | Capture to file                |
+-----+--------------------------------+





.. _parameters_NET_GWADDR:

NET\_GWADDR Parameters
----------------------


.. _NET_GWADDR0:

NET\_GWADDR0: IPv4 Address 1st byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: 192\.xxx\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_GWADDR1:

NET\_GWADDR1: IPv4 Address 2nd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.168\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_GWADDR2:

NET\_GWADDR2: IPv4 Address 3rd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.144\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_GWADDR3:

NET\_GWADDR3: IPv4 Address 4th byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.xxx\.14


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+





.. _parameters_NET_IPADDR:

NET\_IPADDR Parameters
----------------------


.. _NET_IPADDR0:

NET\_IPADDR0: IPv4 Address 1st byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: 192\.xxx\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_IPADDR1:

NET\_IPADDR1: IPv4 Address 2nd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.168\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_IPADDR2:

NET\_IPADDR2: IPv4 Address 3rd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.144\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_IPADDR3:

NET\_IPADDR3: IPv4 Address 4th byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.xxx\.14


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+





.. _parameters_NET_MACADDR:

NET\_MACADDR Parameters
-----------------------


.. _NET_MACADDR0:

NET\_MACADDR0: MAC Address 1st byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

MAC address 1st byte


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_MACADDR1:

NET\_MACADDR1: MAC Address 2nd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

MAC address 2nd byte


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_MACADDR2:

NET\_MACADDR2: MAC Address 3rd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

MAC address 3rd byte


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_MACADDR3:

NET\_MACADDR3: MAC Address 4th byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

MAC address 4th byte


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_MACADDR4:

NET\_MACADDR4: MAC Address 5th byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

MAC address 5th byte


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_MACADDR5:

NET\_MACADDR5: MAC Address 6th byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

MAC address 6th byte


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+





.. _parameters_NET_P1_:

NET\_P1\_ Parameters
--------------------


.. _NET_P1_TYPE:

NET\_P1\_TYPE: Port type
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Port type for network serial port\. For the two client types a valid destination IP address must be set\. For the two server types either 0\.0\.0\.0 or a local address can be used\. The UDP client type will use broadcast if the IP is set to 255\.255\.255\.255 and will use UDP multicast if the IP is in the multicast address range\.


+-------+------------+
| Value | Meaning    |
+=======+============+
| 0     | Disabled   |
+-------+------------+
| 1     | UDP client |
+-------+------------+
| 2     | UDP server |
+-------+------------+
| 3     | TCP client |
+-------+------------+
| 4     | TCP server |
+-------+------------+




.. _NET_P1_PROTOCOL:

NET\_P1\_PROTOCOL: Protocol
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Networked serial port protocol


+-------+----------------------------------+
| Value | Meaning                          |
+=======+==================================+
| -1    | None                             |
+-------+----------------------------------+
| 1     | MAVLink1                         |
+-------+----------------------------------+
| 2     | MAVLink2                         |
+-------+----------------------------------+
| 3     | Frsky D                          |
+-------+----------------------------------+
| 4     | Frsky SPort                      |
+-------+----------------------------------+
| 5     | GPS                              |
+-------+----------------------------------+
| 7     | Alexmos Gimbal Serial            |
+-------+----------------------------------+
| 8     | Gimbal                           |
+-------+----------------------------------+
| 9     | Rangefinder                      |
+-------+----------------------------------+
| 10    | FrSky SPort Passthrough (OpenTX) |
+-------+----------------------------------+
| 11    | Lidar360                         |
+-------+----------------------------------+
| 13    | Beacon                           |
+-------+----------------------------------+
| 14    | Volz servo out                   |
+-------+----------------------------------+
| 15    | SBus servo out                   |
+-------+----------------------------------+
| 16    | ESC Telemetry                    |
+-------+----------------------------------+
| 17    | Devo Telemetry                   |
+-------+----------------------------------+
| 18    | OpticalFlow                      |
+-------+----------------------------------+
| 19    | RobotisServo                     |
+-------+----------------------------------+
| 20    | NMEA Output                      |
+-------+----------------------------------+
| 21    | WindVane                         |
+-------+----------------------------------+
| 22    | SLCAN                            |
+-------+----------------------------------+
| 23    | RCIN                             |
+-------+----------------------------------+
| 24    | EFI Serial                       |
+-------+----------------------------------+
| 25    | LTM                              |
+-------+----------------------------------+
| 26    | RunCam                           |
+-------+----------------------------------+
| 27    | HottTelem                        |
+-------+----------------------------------+
| 28    | Scripting                        |
+-------+----------------------------------+
| 29    | Crossfire VTX                    |
+-------+----------------------------------+
| 30    | Generator                        |
+-------+----------------------------------+
| 31    | Winch                            |
+-------+----------------------------------+
| 32    | MSP                              |
+-------+----------------------------------+
| 33    | DJI FPV                          |
+-------+----------------------------------+
| 34    | AirSpeed                         |
+-------+----------------------------------+
| 35    | ADSB                             |
+-------+----------------------------------+
| 36    | AHRS                             |
+-------+----------------------------------+
| 37    | SmartAudio                       |
+-------+----------------------------------+
| 38    | FETtecOneWire                    |
+-------+----------------------------------+
| 39    | Torqeedo                         |
+-------+----------------------------------+
| 40    | AIS                              |
+-------+----------------------------------+
| 41    | CoDevESC                         |
+-------+----------------------------------+
| 42    | DisplayPort                      |
+-------+----------------------------------+
| 43    | MAVLink High Latency             |
+-------+----------------------------------+
| 44    | IRC Tramp                        |
+-------+----------------------------------+
| 45    | DDS XRCE                         |
+-------+----------------------------------+
| 46    | IMUDATA                          |
+-------+----------------------------------+
| 48    | PPP                              |
+-------+----------------------------------+
| 49    | i-BUS Telemetry                  |
+-------+----------------------------------+
| 50    | IOMCU                            |
+-------+----------------------------------+




.. _NET_P1_PORT:

NET\_P1\_PORT: Port number
~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Port number


+------------+
| Range      |
+============+
| 0 to 65535 |
+------------+





.. _parameters_NET_P1_IP:

NET\_P1\_IP Parameters
----------------------


.. _NET_P1_IP0:

NET\_P1\_IP0: IPv4 Address 1st byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: 192\.xxx\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_P1_IP1:

NET\_P1\_IP1: IPv4 Address 2nd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.168\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_P1_IP2:

NET\_P1\_IP2: IPv4 Address 3rd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.144\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_P1_IP3:

NET\_P1\_IP3: IPv4 Address 4th byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.xxx\.14


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+





.. _parameters_NET_P2_:

NET\_P2\_ Parameters
--------------------


.. _NET_P2_TYPE:

NET\_P2\_TYPE: Port type
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Port type for network serial port\. For the two client types a valid destination IP address must be set\. For the two server types either 0\.0\.0\.0 or a local address can be used\. The UDP client type will use broadcast if the IP is set to 255\.255\.255\.255 and will use UDP multicast if the IP is in the multicast address range\.


+-------+------------+
| Value | Meaning    |
+=======+============+
| 0     | Disabled   |
+-------+------------+
| 1     | UDP client |
+-------+------------+
| 2     | UDP server |
+-------+------------+
| 3     | TCP client |
+-------+------------+
| 4     | TCP server |
+-------+------------+




.. _NET_P2_PROTOCOL:

NET\_P2\_PROTOCOL: Protocol
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Networked serial port protocol


+-------+----------------------------------+
| Value | Meaning                          |
+=======+==================================+
| -1    | None                             |
+-------+----------------------------------+
| 1     | MAVLink1                         |
+-------+----------------------------------+
| 2     | MAVLink2                         |
+-------+----------------------------------+
| 3     | Frsky D                          |
+-------+----------------------------------+
| 4     | Frsky SPort                      |
+-------+----------------------------------+
| 5     | GPS                              |
+-------+----------------------------------+
| 7     | Alexmos Gimbal Serial            |
+-------+----------------------------------+
| 8     | Gimbal                           |
+-------+----------------------------------+
| 9     | Rangefinder                      |
+-------+----------------------------------+
| 10    | FrSky SPort Passthrough (OpenTX) |
+-------+----------------------------------+
| 11    | Lidar360                         |
+-------+----------------------------------+
| 13    | Beacon                           |
+-------+----------------------------------+
| 14    | Volz servo out                   |
+-------+----------------------------------+
| 15    | SBus servo out                   |
+-------+----------------------------------+
| 16    | ESC Telemetry                    |
+-------+----------------------------------+
| 17    | Devo Telemetry                   |
+-------+----------------------------------+
| 18    | OpticalFlow                      |
+-------+----------------------------------+
| 19    | RobotisServo                     |
+-------+----------------------------------+
| 20    | NMEA Output                      |
+-------+----------------------------------+
| 21    | WindVane                         |
+-------+----------------------------------+
| 22    | SLCAN                            |
+-------+----------------------------------+
| 23    | RCIN                             |
+-------+----------------------------------+
| 24    | EFI Serial                       |
+-------+----------------------------------+
| 25    | LTM                              |
+-------+----------------------------------+
| 26    | RunCam                           |
+-------+----------------------------------+
| 27    | HottTelem                        |
+-------+----------------------------------+
| 28    | Scripting                        |
+-------+----------------------------------+
| 29    | Crossfire VTX                    |
+-------+----------------------------------+
| 30    | Generator                        |
+-------+----------------------------------+
| 31    | Winch                            |
+-------+----------------------------------+
| 32    | MSP                              |
+-------+----------------------------------+
| 33    | DJI FPV                          |
+-------+----------------------------------+
| 34    | AirSpeed                         |
+-------+----------------------------------+
| 35    | ADSB                             |
+-------+----------------------------------+
| 36    | AHRS                             |
+-------+----------------------------------+
| 37    | SmartAudio                       |
+-------+----------------------------------+
| 38    | FETtecOneWire                    |
+-------+----------------------------------+
| 39    | Torqeedo                         |
+-------+----------------------------------+
| 40    | AIS                              |
+-------+----------------------------------+
| 41    | CoDevESC                         |
+-------+----------------------------------+
| 42    | DisplayPort                      |
+-------+----------------------------------+
| 43    | MAVLink High Latency             |
+-------+----------------------------------+
| 44    | IRC Tramp                        |
+-------+----------------------------------+
| 45    | DDS XRCE                         |
+-------+----------------------------------+
| 46    | IMUDATA                          |
+-------+----------------------------------+
| 48    | PPP                              |
+-------+----------------------------------+
| 49    | i-BUS Telemetry                  |
+-------+----------------------------------+
| 50    | IOMCU                            |
+-------+----------------------------------+




.. _NET_P2_PORT:

NET\_P2\_PORT: Port number
~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Port number


+------------+
| Range      |
+============+
| 0 to 65535 |
+------------+





.. _parameters_NET_P2_IP:

NET\_P2\_IP Parameters
----------------------


.. _NET_P2_IP0:

NET\_P2\_IP0: IPv4 Address 1st byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: 192\.xxx\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_P2_IP1:

NET\_P2\_IP1: IPv4 Address 2nd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.168\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_P2_IP2:

NET\_P2\_IP2: IPv4 Address 3rd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.144\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_P2_IP3:

NET\_P2\_IP3: IPv4 Address 4th byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.xxx\.14


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+





.. _parameters_NET_P3_:

NET\_P3\_ Parameters
--------------------


.. _NET_P3_TYPE:

NET\_P3\_TYPE: Port type
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Port type for network serial port\. For the two client types a valid destination IP address must be set\. For the two server types either 0\.0\.0\.0 or a local address can be used\. The UDP client type will use broadcast if the IP is set to 255\.255\.255\.255 and will use UDP multicast if the IP is in the multicast address range\.


+-------+------------+
| Value | Meaning    |
+=======+============+
| 0     | Disabled   |
+-------+------------+
| 1     | UDP client |
+-------+------------+
| 2     | UDP server |
+-------+------------+
| 3     | TCP client |
+-------+------------+
| 4     | TCP server |
+-------+------------+




.. _NET_P3_PROTOCOL:

NET\_P3\_PROTOCOL: Protocol
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Networked serial port protocol


+-------+----------------------------------+
| Value | Meaning                          |
+=======+==================================+
| -1    | None                             |
+-------+----------------------------------+
| 1     | MAVLink1                         |
+-------+----------------------------------+
| 2     | MAVLink2                         |
+-------+----------------------------------+
| 3     | Frsky D                          |
+-------+----------------------------------+
| 4     | Frsky SPort                      |
+-------+----------------------------------+
| 5     | GPS                              |
+-------+----------------------------------+
| 7     | Alexmos Gimbal Serial            |
+-------+----------------------------------+
| 8     | Gimbal                           |
+-------+----------------------------------+
| 9     | Rangefinder                      |
+-------+----------------------------------+
| 10    | FrSky SPort Passthrough (OpenTX) |
+-------+----------------------------------+
| 11    | Lidar360                         |
+-------+----------------------------------+
| 13    | Beacon                           |
+-------+----------------------------------+
| 14    | Volz servo out                   |
+-------+----------------------------------+
| 15    | SBus servo out                   |
+-------+----------------------------------+
| 16    | ESC Telemetry                    |
+-------+----------------------------------+
| 17    | Devo Telemetry                   |
+-------+----------------------------------+
| 18    | OpticalFlow                      |
+-------+----------------------------------+
| 19    | RobotisServo                     |
+-------+----------------------------------+
| 20    | NMEA Output                      |
+-------+----------------------------------+
| 21    | WindVane                         |
+-------+----------------------------------+
| 22    | SLCAN                            |
+-------+----------------------------------+
| 23    | RCIN                             |
+-------+----------------------------------+
| 24    | EFI Serial                       |
+-------+----------------------------------+
| 25    | LTM                              |
+-------+----------------------------------+
| 26    | RunCam                           |
+-------+----------------------------------+
| 27    | HottTelem                        |
+-------+----------------------------------+
| 28    | Scripting                        |
+-------+----------------------------------+
| 29    | Crossfire VTX                    |
+-------+----------------------------------+
| 30    | Generator                        |
+-------+----------------------------------+
| 31    | Winch                            |
+-------+----------------------------------+
| 32    | MSP                              |
+-------+----------------------------------+
| 33    | DJI FPV                          |
+-------+----------------------------------+
| 34    | AirSpeed                         |
+-------+----------------------------------+
| 35    | ADSB                             |
+-------+----------------------------------+
| 36    | AHRS                             |
+-------+----------------------------------+
| 37    | SmartAudio                       |
+-------+----------------------------------+
| 38    | FETtecOneWire                    |
+-------+----------------------------------+
| 39    | Torqeedo                         |
+-------+----------------------------------+
| 40    | AIS                              |
+-------+----------------------------------+
| 41    | CoDevESC                         |
+-------+----------------------------------+
| 42    | DisplayPort                      |
+-------+----------------------------------+
| 43    | MAVLink High Latency             |
+-------+----------------------------------+
| 44    | IRC Tramp                        |
+-------+----------------------------------+
| 45    | DDS XRCE                         |
+-------+----------------------------------+
| 46    | IMUDATA                          |
+-------+----------------------------------+
| 48    | PPP                              |
+-------+----------------------------------+
| 49    | i-BUS Telemetry                  |
+-------+----------------------------------+
| 50    | IOMCU                            |
+-------+----------------------------------+




.. _NET_P3_PORT:

NET\_P3\_PORT: Port number
~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Port number


+------------+
| Range      |
+============+
| 0 to 65535 |
+------------+





.. _parameters_NET_P3_IP:

NET\_P3\_IP Parameters
----------------------


.. _NET_P3_IP0:

NET\_P3\_IP0: IPv4 Address 1st byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: 192\.xxx\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_P3_IP1:

NET\_P3\_IP1: IPv4 Address 2nd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.168\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_P3_IP2:

NET\_P3\_IP2: IPv4 Address 3rd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.144\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_P3_IP3:

NET\_P3\_IP3: IPv4 Address 4th byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.xxx\.14


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+





.. _parameters_NET_P4_:

NET\_P4\_ Parameters
--------------------


.. _NET_P4_TYPE:

NET\_P4\_TYPE: Port type
~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Port type for network serial port\. For the two client types a valid destination IP address must be set\. For the two server types either 0\.0\.0\.0 or a local address can be used\. The UDP client type will use broadcast if the IP is set to 255\.255\.255\.255 and will use UDP multicast if the IP is in the multicast address range\.


+-------+------------+
| Value | Meaning    |
+=======+============+
| 0     | Disabled   |
+-------+------------+
| 1     | UDP client |
+-------+------------+
| 2     | UDP server |
+-------+------------+
| 3     | TCP client |
+-------+------------+
| 4     | TCP server |
+-------+------------+




.. _NET_P4_PROTOCOL:

NET\_P4\_PROTOCOL: Protocol
~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Networked serial port protocol


+-------+----------------------------------+
| Value | Meaning                          |
+=======+==================================+
| -1    | None                             |
+-------+----------------------------------+
| 1     | MAVLink1                         |
+-------+----------------------------------+
| 2     | MAVLink2                         |
+-------+----------------------------------+
| 3     | Frsky D                          |
+-------+----------------------------------+
| 4     | Frsky SPort                      |
+-------+----------------------------------+
| 5     | GPS                              |
+-------+----------------------------------+
| 7     | Alexmos Gimbal Serial            |
+-------+----------------------------------+
| 8     | Gimbal                           |
+-------+----------------------------------+
| 9     | Rangefinder                      |
+-------+----------------------------------+
| 10    | FrSky SPort Passthrough (OpenTX) |
+-------+----------------------------------+
| 11    | Lidar360                         |
+-------+----------------------------------+
| 13    | Beacon                           |
+-------+----------------------------------+
| 14    | Volz servo out                   |
+-------+----------------------------------+
| 15    | SBus servo out                   |
+-------+----------------------------------+
| 16    | ESC Telemetry                    |
+-------+----------------------------------+
| 17    | Devo Telemetry                   |
+-------+----------------------------------+
| 18    | OpticalFlow                      |
+-------+----------------------------------+
| 19    | RobotisServo                     |
+-------+----------------------------------+
| 20    | NMEA Output                      |
+-------+----------------------------------+
| 21    | WindVane                         |
+-------+----------------------------------+
| 22    | SLCAN                            |
+-------+----------------------------------+
| 23    | RCIN                             |
+-------+----------------------------------+
| 24    | EFI Serial                       |
+-------+----------------------------------+
| 25    | LTM                              |
+-------+----------------------------------+
| 26    | RunCam                           |
+-------+----------------------------------+
| 27    | HottTelem                        |
+-------+----------------------------------+
| 28    | Scripting                        |
+-------+----------------------------------+
| 29    | Crossfire VTX                    |
+-------+----------------------------------+
| 30    | Generator                        |
+-------+----------------------------------+
| 31    | Winch                            |
+-------+----------------------------------+
| 32    | MSP                              |
+-------+----------------------------------+
| 33    | DJI FPV                          |
+-------+----------------------------------+
| 34    | AirSpeed                         |
+-------+----------------------------------+
| 35    | ADSB                             |
+-------+----------------------------------+
| 36    | AHRS                             |
+-------+----------------------------------+
| 37    | SmartAudio                       |
+-------+----------------------------------+
| 38    | FETtecOneWire                    |
+-------+----------------------------------+
| 39    | Torqeedo                         |
+-------+----------------------------------+
| 40    | AIS                              |
+-------+----------------------------------+
| 41    | CoDevESC                         |
+-------+----------------------------------+
| 42    | DisplayPort                      |
+-------+----------------------------------+
| 43    | MAVLink High Latency             |
+-------+----------------------------------+
| 44    | IRC Tramp                        |
+-------+----------------------------------+
| 45    | DDS XRCE                         |
+-------+----------------------------------+
| 46    | IMUDATA                          |
+-------+----------------------------------+
| 48    | PPP                              |
+-------+----------------------------------+
| 49    | i-BUS Telemetry                  |
+-------+----------------------------------+
| 50    | IOMCU                            |
+-------+----------------------------------+




.. _NET_P4_PORT:

NET\_P4\_PORT: Port number
~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Port number


+------------+
| Range      |
+============+
| 0 to 65535 |
+------------+





.. _parameters_NET_P4_IP:

NET\_P4\_IP Parameters
----------------------


.. _NET_P4_IP0:

NET\_P4\_IP0: IPv4 Address 1st byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: 192\.xxx\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_P4_IP1:

NET\_P4\_IP1: IPv4 Address 2nd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.168\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_P4_IP2:

NET\_P4\_IP2: IPv4 Address 3rd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.144\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_P4_IP3:

NET\_P4\_IP3: IPv4 Address 4th byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.xxx\.14


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+





.. _parameters_NET_REMPPP_IP:

NET\_REMPPP\_IP Parameters
--------------------------


.. _NET_REMPPP_IP0:

NET\_REMPPP\_IP0: IPv4 Address 1st byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: 192\.xxx\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_REMPPP_IP1:

NET\_REMPPP\_IP1: IPv4 Address 2nd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.168\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_REMPPP_IP2:

NET\_REMPPP\_IP2: IPv4 Address 3rd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.144\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_REMPPP_IP3:

NET\_REMPPP\_IP3: IPv4 Address 4th byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.xxx\.14


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+





.. _parameters_NET_TEST_IP:

NET\_TEST\_IP Parameters
------------------------


.. _NET_TEST_IP0:

NET\_TEST\_IP0: IPv4 Address 1st byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: 192\.xxx\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_TEST_IP1:

NET\_TEST\_IP1: IPv4 Address 2nd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.168\.xxx\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_TEST_IP2:

NET\_TEST\_IP2: IPv4 Address 3rd byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.144\.xxx


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+




.. _NET_TEST_IP3:

NET\_TEST\_IP3: IPv4 Address 4th byte
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

IPv4 address\. Example\: xxx\.xxx\.xxx\.14


+----------+
| Range    |
+==========+
| 0 to 255 |
+----------+





.. _parameters_NMEA_:

NMEA\_ Parameters
-----------------


.. _NMEA_RATE_MS:

NMEA\_RATE\_MS: NMEA Output rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


NMEA Output rate\. This controls the interval at which all the enabled NMEA messages are sent\. Most NMEA systems expect 100ms \(10Hz\) or slower\.


+-----------+------------+--------------+
| Increment | Range      | Units        |
+===========+============+==============+
| 1         | 20 to 2000 | milliseconds |
+-----------+------------+--------------+




.. _NMEA_MSG_EN:

NMEA\_MSG\_EN: Messages Enable bitmask
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


This is a bitmask of enabled NMEA messages\. All messages will be sent consecutively at the same rate interval


+-----+---------+
| Bit | Meaning |
+=====+=========+
| 0   | GPGGA   |
+-----+---------+
| 1   | GPRMC   |
+-----+---------+
| 2   | PASHR   |
+-----+---------+





.. _parameters_RPM1_:

RPM1\_ Parameters
-----------------


.. _RPM1_TYPE:

RPM1\_TYPE: RPM type
~~~~~~~~~~~~~~~~~~~~


What type of RPM sensor is connected


+-------+------------------------------+
| Value | Meaning                      |
+=======+==============================+
| 0     | None                         |
+-------+------------------------------+
| 1     | Not Used                     |
+-------+------------------------------+
| 2     | GPIO                         |
+-------+------------------------------+
| 3     | EFI                          |
+-------+------------------------------+
| 4     | Harmonic Notch               |
+-------+------------------------------+
| 5     | ESC Telemetry Motors Bitmask |
+-------+------------------------------+
| 6     | Generator                    |
+-------+------------------------------+
| 7     | DroneCAN                     |
+-------+------------------------------+




.. _RPM1_SCALING:

RPM1\_SCALING: RPM scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between sensor reading and RPM\.


+-----------+
| Increment |
+===========+
| 0.001     |
+-----------+




.. _RPM1_MAX:

RPM1\_MAX: Maximum RPM
~~~~~~~~~~~~~~~~~~~~~~


Maximum RPM to report\. Only used on type \= GPIO\.


+-----------+
| Increment |
+===========+
| 1         |
+-----------+




.. _RPM1_MIN:

RPM1\_MIN: Minimum RPM
~~~~~~~~~~~~~~~~~~~~~~


Minimum RPM to report\. Only used on type \= GPIO\.


+-----------+
| Increment |
+===========+
| 1         |
+-----------+




.. _RPM1_MIN_QUAL:

RPM1\_MIN\_QUAL: Minimum Quality
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum data quality to be used


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _RPM1_PIN:

RPM1\_PIN: Input pin number
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Which digital GPIO pin to use\. Only used on type \= GPIO\. Some common values are given\, but see the Wiki\'s \"GPIOs\" page for how to determine the pin number for a given autopilot\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| -1    | Disabled |
+-------+----------+
| 50    | AUX1     |
+-------+----------+
| 51    | AUX2     |
+-------+----------+
| 52    | AUX3     |
+-------+----------+
| 53    | AUX4     |
+-------+----------+
| 54    | AUX5     |
+-------+----------+
| 55    | AUX6     |
+-------+----------+




.. _RPM1_ESC_MASK:

RPM1\_ESC\_MASK: Bitmask of ESC telemetry channels to average
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Mask of channels which support ESC rpm telemetry\. RPM telemetry of the selected channels will be averaged


+-----+-----------+
| Bit | Meaning   |
+=====+===========+
| 0   | Channel1  |
+-----+-----------+
| 1   | Channel2  |
+-----+-----------+
| 2   | Channel3  |
+-----+-----------+
| 3   | Channel4  |
+-----+-----------+
| 4   | Channel5  |
+-----+-----------+
| 5   | Channel6  |
+-----+-----------+
| 6   | Channel7  |
+-----+-----------+
| 7   | Channel8  |
+-----+-----------+
| 8   | Channel9  |
+-----+-----------+
| 9   | Channel10 |
+-----+-----------+
| 10  | Channel11 |
+-----+-----------+
| 11  | Channel12 |
+-----+-----------+
| 12  | Channel13 |
+-----+-----------+
| 13  | Channel14 |
+-----+-----------+
| 14  | Channel15 |
+-----+-----------+
| 15  | Channel16 |
+-----+-----------+




.. _RPM1_ESC_INDEX:

RPM1\_ESC\_INDEX: ESC Telemetry Index to write RPM to
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

ESC Telemetry Index to write RPM to\. Use 0 to disable\.


+-----------+---------+
| Increment | Range   |
+===========+=========+
| 1         | 0 to 10 |
+-----------+---------+




.. _RPM1_DC_ID:

RPM1\_DC\_ID: DroneCAN Sensor ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

DroneCAN sensor ID to assign to this backend


+-----------+----------+
| Increment | Range    |
+===========+==========+
| 1         | -1 to 10 |
+-----------+----------+





.. _parameters_RPM2_:

RPM2\_ Parameters
-----------------


.. _RPM2_TYPE:

RPM2\_TYPE: RPM type
~~~~~~~~~~~~~~~~~~~~


What type of RPM sensor is connected


+-------+------------------------------+
| Value | Meaning                      |
+=======+==============================+
| 0     | None                         |
+-------+------------------------------+
| 1     | Not Used                     |
+-------+------------------------------+
| 2     | GPIO                         |
+-------+------------------------------+
| 3     | EFI                          |
+-------+------------------------------+
| 4     | Harmonic Notch               |
+-------+------------------------------+
| 5     | ESC Telemetry Motors Bitmask |
+-------+------------------------------+
| 6     | Generator                    |
+-------+------------------------------+
| 7     | DroneCAN                     |
+-------+------------------------------+




.. _RPM2_SCALING:

RPM2\_SCALING: RPM scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between sensor reading and RPM\.


+-----------+
| Increment |
+===========+
| 0.001     |
+-----------+




.. _RPM2_MAX:

RPM2\_MAX: Maximum RPM
~~~~~~~~~~~~~~~~~~~~~~


Maximum RPM to report\. Only used on type \= GPIO\.


+-----------+
| Increment |
+===========+
| 1         |
+-----------+




.. _RPM2_MIN:

RPM2\_MIN: Minimum RPM
~~~~~~~~~~~~~~~~~~~~~~


Minimum RPM to report\. Only used on type \= GPIO\.


+-----------+
| Increment |
+===========+
| 1         |
+-----------+




.. _RPM2_MIN_QUAL:

RPM2\_MIN\_QUAL: Minimum Quality
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum data quality to be used


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _RPM2_PIN:

RPM2\_PIN: Input pin number
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Which digital GPIO pin to use\. Only used on type \= GPIO\. Some common values are given\, but see the Wiki\'s \"GPIOs\" page for how to determine the pin number for a given autopilot\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| -1    | Disabled |
+-------+----------+
| 50    | AUX1     |
+-------+----------+
| 51    | AUX2     |
+-------+----------+
| 52    | AUX3     |
+-------+----------+
| 53    | AUX4     |
+-------+----------+
| 54    | AUX5     |
+-------+----------+
| 55    | AUX6     |
+-------+----------+




.. _RPM2_ESC_MASK:

RPM2\_ESC\_MASK: Bitmask of ESC telemetry channels to average
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Mask of channels which support ESC rpm telemetry\. RPM telemetry of the selected channels will be averaged


+-----+-----------+
| Bit | Meaning   |
+=====+===========+
| 0   | Channel1  |
+-----+-----------+
| 1   | Channel2  |
+-----+-----------+
| 2   | Channel3  |
+-----+-----------+
| 3   | Channel4  |
+-----+-----------+
| 4   | Channel5  |
+-----+-----------+
| 5   | Channel6  |
+-----+-----------+
| 6   | Channel7  |
+-----+-----------+
| 7   | Channel8  |
+-----+-----------+
| 8   | Channel9  |
+-----+-----------+
| 9   | Channel10 |
+-----+-----------+
| 10  | Channel11 |
+-----+-----------+
| 11  | Channel12 |
+-----+-----------+
| 12  | Channel13 |
+-----+-----------+
| 13  | Channel14 |
+-----+-----------+
| 14  | Channel15 |
+-----+-----------+
| 15  | Channel16 |
+-----+-----------+




.. _RPM2_ESC_INDEX:

RPM2\_ESC\_INDEX: ESC Telemetry Index to write RPM to
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

ESC Telemetry Index to write RPM to\. Use 0 to disable\.


+-----------+---------+
| Increment | Range   |
+===========+=========+
| 1         | 0 to 10 |
+-----------+---------+




.. _RPM2_DC_ID:

RPM2\_DC\_ID: DroneCAN Sensor ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

DroneCAN sensor ID to assign to this backend


+-----------+----------+
| Increment | Range    |
+===========+==========+
| 1         | -1 to 10 |
+-----------+----------+





.. _parameters_RPM3_:

RPM3\_ Parameters
-----------------


.. _RPM3_TYPE:

RPM3\_TYPE: RPM type
~~~~~~~~~~~~~~~~~~~~


What type of RPM sensor is connected


+-------+------------------------------+
| Value | Meaning                      |
+=======+==============================+
| 0     | None                         |
+-------+------------------------------+
| 1     | Not Used                     |
+-------+------------------------------+
| 2     | GPIO                         |
+-------+------------------------------+
| 3     | EFI                          |
+-------+------------------------------+
| 4     | Harmonic Notch               |
+-------+------------------------------+
| 5     | ESC Telemetry Motors Bitmask |
+-------+------------------------------+
| 6     | Generator                    |
+-------+------------------------------+
| 7     | DroneCAN                     |
+-------+------------------------------+




.. _RPM3_SCALING:

RPM3\_SCALING: RPM scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between sensor reading and RPM\.


+-----------+
| Increment |
+===========+
| 0.001     |
+-----------+




.. _RPM3_MAX:

RPM3\_MAX: Maximum RPM
~~~~~~~~~~~~~~~~~~~~~~


Maximum RPM to report\. Only used on type \= GPIO\.


+-----------+
| Increment |
+===========+
| 1         |
+-----------+




.. _RPM3_MIN:

RPM3\_MIN: Minimum RPM
~~~~~~~~~~~~~~~~~~~~~~


Minimum RPM to report\. Only used on type \= GPIO\.


+-----------+
| Increment |
+===========+
| 1         |
+-----------+




.. _RPM3_MIN_QUAL:

RPM3\_MIN\_QUAL: Minimum Quality
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum data quality to be used


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _RPM3_PIN:

RPM3\_PIN: Input pin number
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Which digital GPIO pin to use\. Only used on type \= GPIO\. Some common values are given\, but see the Wiki\'s \"GPIOs\" page for how to determine the pin number for a given autopilot\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| -1    | Disabled |
+-------+----------+
| 50    | AUX1     |
+-------+----------+
| 51    | AUX2     |
+-------+----------+
| 52    | AUX3     |
+-------+----------+
| 53    | AUX4     |
+-------+----------+
| 54    | AUX5     |
+-------+----------+
| 55    | AUX6     |
+-------+----------+




.. _RPM3_ESC_MASK:

RPM3\_ESC\_MASK: Bitmask of ESC telemetry channels to average
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Mask of channels which support ESC rpm telemetry\. RPM telemetry of the selected channels will be averaged


+-----+-----------+
| Bit | Meaning   |
+=====+===========+
| 0   | Channel1  |
+-----+-----------+
| 1   | Channel2  |
+-----+-----------+
| 2   | Channel3  |
+-----+-----------+
| 3   | Channel4  |
+-----+-----------+
| 4   | Channel5  |
+-----+-----------+
| 5   | Channel6  |
+-----+-----------+
| 6   | Channel7  |
+-----+-----------+
| 7   | Channel8  |
+-----+-----------+
| 8   | Channel9  |
+-----+-----------+
| 9   | Channel10 |
+-----+-----------+
| 10  | Channel11 |
+-----+-----------+
| 11  | Channel12 |
+-----+-----------+
| 12  | Channel13 |
+-----+-----------+
| 13  | Channel14 |
+-----+-----------+
| 14  | Channel15 |
+-----+-----------+
| 15  | Channel16 |
+-----+-----------+




.. _RPM3_ESC_INDEX:

RPM3\_ESC\_INDEX: ESC Telemetry Index to write RPM to
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

ESC Telemetry Index to write RPM to\. Use 0 to disable\.


+-----------+---------+
| Increment | Range   |
+===========+=========+
| 1         | 0 to 10 |
+-----------+---------+




.. _RPM3_DC_ID:

RPM3\_DC\_ID: DroneCAN Sensor ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

DroneCAN sensor ID to assign to this backend


+-----------+----------+
| Increment | Range    |
+===========+==========+
| 1         | -1 to 10 |
+-----------+----------+





.. _parameters_RPM4_:

RPM4\_ Parameters
-----------------


.. _RPM4_TYPE:

RPM4\_TYPE: RPM type
~~~~~~~~~~~~~~~~~~~~


What type of RPM sensor is connected


+-------+------------------------------+
| Value | Meaning                      |
+=======+==============================+
| 0     | None                         |
+-------+------------------------------+
| 1     | Not Used                     |
+-------+------------------------------+
| 2     | GPIO                         |
+-------+------------------------------+
| 3     | EFI                          |
+-------+------------------------------+
| 4     | Harmonic Notch               |
+-------+------------------------------+
| 5     | ESC Telemetry Motors Bitmask |
+-------+------------------------------+
| 6     | Generator                    |
+-------+------------------------------+
| 7     | DroneCAN                     |
+-------+------------------------------+




.. _RPM4_SCALING:

RPM4\_SCALING: RPM scaling
~~~~~~~~~~~~~~~~~~~~~~~~~~


Scaling factor between sensor reading and RPM\.


+-----------+
| Increment |
+===========+
| 0.001     |
+-----------+




.. _RPM4_MAX:

RPM4\_MAX: Maximum RPM
~~~~~~~~~~~~~~~~~~~~~~


Maximum RPM to report\. Only used on type \= GPIO\.


+-----------+
| Increment |
+===========+
| 1         |
+-----------+




.. _RPM4_MIN:

RPM4\_MIN: Minimum RPM
~~~~~~~~~~~~~~~~~~~~~~


Minimum RPM to report\. Only used on type \= GPIO\.


+-----------+
| Increment |
+===========+
| 1         |
+-----------+




.. _RPM4_MIN_QUAL:

RPM4\_MIN\_QUAL: Minimum Quality
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Minimum data quality to be used


+-----------+
| Increment |
+===========+
| 0.1       |
+-----------+




.. _RPM4_PIN:

RPM4\_PIN: Input pin number
~~~~~~~~~~~~~~~~~~~~~~~~~~~


Which digital GPIO pin to use\. Only used on type \= GPIO\. Some common values are given\, but see the Wiki\'s \"GPIOs\" page for how to determine the pin number for a given autopilot\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| -1    | Disabled |
+-------+----------+
| 50    | AUX1     |
+-------+----------+
| 51    | AUX2     |
+-------+----------+
| 52    | AUX3     |
+-------+----------+
| 53    | AUX4     |
+-------+----------+
| 54    | AUX5     |
+-------+----------+
| 55    | AUX6     |
+-------+----------+




.. _RPM4_ESC_MASK:

RPM4\_ESC\_MASK: Bitmask of ESC telemetry channels to average
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Mask of channels which support ESC rpm telemetry\. RPM telemetry of the selected channels will be averaged


+-----+-----------+
| Bit | Meaning   |
+=====+===========+
| 0   | Channel1  |
+-----+-----------+
| 1   | Channel2  |
+-----+-----------+
| 2   | Channel3  |
+-----+-----------+
| 3   | Channel4  |
+-----+-----------+
| 4   | Channel5  |
+-----+-----------+
| 5   | Channel6  |
+-----+-----------+
| 6   | Channel7  |
+-----+-----------+
| 7   | Channel8  |
+-----+-----------+
| 8   | Channel9  |
+-----+-----------+
| 9   | Channel10 |
+-----+-----------+
| 10  | Channel11 |
+-----+-----------+
| 11  | Channel12 |
+-----+-----------+
| 12  | Channel13 |
+-----+-----------+
| 13  | Channel14 |
+-----+-----------+
| 14  | Channel15 |
+-----+-----------+
| 15  | Channel16 |
+-----+-----------+




.. _RPM4_ESC_INDEX:

RPM4\_ESC\_INDEX: ESC Telemetry Index to write RPM to
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

ESC Telemetry Index to write RPM to\. Use 0 to disable\.


+-----------+---------+
| Increment | Range   |
+===========+=========+
| 1         | 0 to 10 |
+-----------+---------+




.. _RPM4_DC_ID:

RPM4\_DC\_ID: DroneCAN Sensor ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

DroneCAN sensor ID to assign to this backend


+-----------+----------+
| Increment | Range    |
+===========+==========+
| 1         | -1 to 10 |
+-----------+----------+





.. _parameters_SCR_:

SCR\_ Parameters
----------------


.. _SCR_ENABLE:

SCR\_ENABLE: Enable Scripting
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Controls if scripting is enabled


+-------+-------------+
| Value | Meaning     |
+=======+=============+
| 0     | None        |
+-------+-------------+
| 1     | Lua Scripts |
+-------+-------------+




.. _SCR_VM_I_COUNT:

SCR\_VM\_I\_COUNT: Scripting Virtual Machine Instruction Count
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

The number virtual machine instructions that can be run before considering a script to have taken an excessive amount of time


+-----------+-----------------+
| Increment | Range           |
+===========+=================+
| 10000     | 1000 to 1000000 |
+-----------+-----------------+




.. _SCR_HEAP_SIZE:

SCR\_HEAP\_SIZE: Scripting Heap Size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Amount of memory available for scripting


+-----------+-----------------+
| Increment | Range           |
+===========+=================+
| 1024      | 1024 to 1048576 |
+-----------+-----------------+




.. _SCR_DEBUG_OPTS:

SCR\_DEBUG\_OPTS: Scripting Debug Level
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Debugging options


+-----+----------------------------------------------------------------------------------------+
| Bit | Meaning                                                                                |
+=====+========================================================================================+
| 0   | No Scripts to run message if all scripts have stopped                                  |
+-----+----------------------------------------------------------------------------------------+
| 1   | Runtime messages for memory usage and execution time                                   |
+-----+----------------------------------------------------------------------------------------+
| 2   | Suppress logging scripts to dataflash                                                  |
+-----+----------------------------------------------------------------------------------------+
| 3   | log runtime memory usage and execution time                                            |
+-----+----------------------------------------------------------------------------------------+
| 4   | Disable pre-arm check                                                                  |
+-----+----------------------------------------------------------------------------------------+
| 5   | Save CRC of current scripts to loaded and running checksum parameters enabling pre-arm |
+-----+----------------------------------------------------------------------------------------+
| 6   | Disable heap expansion on allocation failure                                           |
+-----+----------------------------------------------------------------------------------------+




.. _SCR_USER1:

SCR\_USER1: Scripting User Parameter1
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


General purpose user variable input for scripts


.. _SCR_USER2:

SCR\_USER2: Scripting User Parameter2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


General purpose user variable input for scripts


.. _SCR_USER3:

SCR\_USER3: Scripting User Parameter3
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


General purpose user variable input for scripts


.. _SCR_USER4:

SCR\_USER4: Scripting User Parameter4
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


General purpose user variable input for scripts


.. _SCR_USER5:

SCR\_USER5: Scripting User Parameter5
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


General purpose user variable input for scripts


.. _SCR_USER6:

SCR\_USER6: Scripting User Parameter6
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


General purpose user variable input for scripts


.. _SCR_DIR_DISABLE:

SCR\_DIR\_DISABLE: Directory disable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

This will stop scripts being loaded from the given locations


+-----+-------------+
| Bit | Meaning     |
+=====+=============+
| 0   | ROMFS       |
+-----+-------------+
| 1   | APM/scripts |
+-----+-------------+




.. _SCR_LD_CHECKSUM:

SCR\_LD\_CHECKSUM: Loaded script checksum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Required XOR of CRC32 checksum of loaded scripts\, vehicle will not arm with incorrect scripts loaded\, \-1 disables


.. _SCR_RUN_CHECKSUM:

SCR\_RUN\_CHECKSUM: Running script checksum
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Required XOR of CRC32 checksum of running scripts\, vehicle will not arm with incorrect scripts running\, \-1 disables


.. _SCR_THD_PRIORITY:

SCR\_THD\_PRIORITY: Scripting thread priority
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

This sets the priority of the scripting thread\. This is normally set to a low priority to prevent scripts from interfering with other parts of the system\. Advanced users can change this priority if scripting needs to be prioritised for realtime applications\. WARNING\: changing this parameter can impact the stability of your flight controller\. The scipting thread priority in this parameter is chosen based on a set of system level priorities for other subsystems\. It is strongly recommended that you use the lowest priority that is sufficient for your application\. Note that all scripts run at the same priority\, so if you raise this priority you must carefully audit all lua scripts for behaviour that does not interfere with the operation of the system\.


+-------+------------------+
| Value | Meaning          |
+=======+==================+
| 0     | Normal           |
+-------+------------------+
| 1     | IO Priority      |
+-------+------------------+
| 2     | Storage Priority |
+-------+------------------+
| 3     | UART Priority    |
+-------+------------------+
| 4     | I2C Priority     |
+-------+------------------+
| 5     | SPI Priority     |
+-------+------------------+
| 6     | Timer Priority   |
+-------+------------------+
| 7     | Main Priority    |
+-------+------------------+
| 8     | Boost Priority   |
+-------+------------------+




.. _SCR_SDEV_EN:

SCR\_SDEV\_EN: Scripting serial device enable
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Enable scripting serial devices


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 0     | Disabled |
+-------+----------+
| 1     | Enabled  |
+-------+----------+




.. _SCR_SDEV1_PROTO:

SCR\_SDEV1\_PROTO: Serial protocol of scripting serial device
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Serial protocol of scripting serial device


+-------+----------------------------------+
| Value | Meaning                          |
+=======+==================================+
| -1    | None                             |
+-------+----------------------------------+
| 1     | MAVLink1                         |
+-------+----------------------------------+
| 2     | MAVLink2                         |
+-------+----------------------------------+
| 3     | Frsky D                          |
+-------+----------------------------------+
| 4     | Frsky SPort                      |
+-------+----------------------------------+
| 5     | GPS                              |
+-------+----------------------------------+
| 7     | Alexmos Gimbal Serial            |
+-------+----------------------------------+
| 8     | Gimbal                           |
+-------+----------------------------------+
| 9     | Rangefinder                      |
+-------+----------------------------------+
| 10    | FrSky SPort Passthrough (OpenTX) |
+-------+----------------------------------+
| 11    | Lidar360                         |
+-------+----------------------------------+
| 13    | Beacon                           |
+-------+----------------------------------+
| 14    | Volz servo out                   |
+-------+----------------------------------+
| 15    | SBus servo out                   |
+-------+----------------------------------+
| 16    | ESC Telemetry                    |
+-------+----------------------------------+
| 17    | Devo Telemetry                   |
+-------+----------------------------------+
| 18    | OpticalFlow                      |
+-------+----------------------------------+
| 19    | RobotisServo                     |
+-------+----------------------------------+
| 20    | NMEA Output                      |
+-------+----------------------------------+
| 21    | WindVane                         |
+-------+----------------------------------+
| 22    | SLCAN                            |
+-------+----------------------------------+
| 23    | RCIN                             |
+-------+----------------------------------+
| 24    | EFI Serial                       |
+-------+----------------------------------+
| 25    | LTM                              |
+-------+----------------------------------+
| 26    | RunCam                           |
+-------+----------------------------------+
| 27    | HottTelem                        |
+-------+----------------------------------+
| 28    | Scripting                        |
+-------+----------------------------------+
| 29    | Crossfire VTX                    |
+-------+----------------------------------+
| 30    | Generator                        |
+-------+----------------------------------+
| 31    | Winch                            |
+-------+----------------------------------+
| 32    | MSP                              |
+-------+----------------------------------+
| 33    | DJI FPV                          |
+-------+----------------------------------+
| 34    | AirSpeed                         |
+-------+----------------------------------+
| 35    | ADSB                             |
+-------+----------------------------------+
| 36    | AHRS                             |
+-------+----------------------------------+
| 37    | SmartAudio                       |
+-------+----------------------------------+
| 38    | FETtecOneWire                    |
+-------+----------------------------------+
| 39    | Torqeedo                         |
+-------+----------------------------------+
| 40    | AIS                              |
+-------+----------------------------------+
| 41    | CoDevESC                         |
+-------+----------------------------------+
| 42    | DisplayPort                      |
+-------+----------------------------------+
| 43    | MAVLink High Latency             |
+-------+----------------------------------+
| 44    | IRC Tramp                        |
+-------+----------------------------------+
| 45    | DDS XRCE                         |
+-------+----------------------------------+
| 46    | IMUDATA                          |
+-------+----------------------------------+
| 48    | PPP                              |
+-------+----------------------------------+
| 49    | i-BUS Telemetry                  |
+-------+----------------------------------+
| 50    | IOMCU                            |
+-------+----------------------------------+




.. _SCR_SDEV2_PROTO:

SCR\_SDEV2\_PROTO: Serial protocol of scripting serial device
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Serial protocol of scripting serial device


+-------+----------------------------------+
| Value | Meaning                          |
+=======+==================================+
| -1    | None                             |
+-------+----------------------------------+
| 1     | MAVLink1                         |
+-------+----------------------------------+
| 2     | MAVLink2                         |
+-------+----------------------------------+
| 3     | Frsky D                          |
+-------+----------------------------------+
| 4     | Frsky SPort                      |
+-------+----------------------------------+
| 5     | GPS                              |
+-------+----------------------------------+
| 7     | Alexmos Gimbal Serial            |
+-------+----------------------------------+
| 8     | Gimbal                           |
+-------+----------------------------------+
| 9     | Rangefinder                      |
+-------+----------------------------------+
| 10    | FrSky SPort Passthrough (OpenTX) |
+-------+----------------------------------+
| 11    | Lidar360                         |
+-------+----------------------------------+
| 13    | Beacon                           |
+-------+----------------------------------+
| 14    | Volz servo out                   |
+-------+----------------------------------+
| 15    | SBus servo out                   |
+-------+----------------------------------+
| 16    | ESC Telemetry                    |
+-------+----------------------------------+
| 17    | Devo Telemetry                   |
+-------+----------------------------------+
| 18    | OpticalFlow                      |
+-------+----------------------------------+
| 19    | RobotisServo                     |
+-------+----------------------------------+
| 20    | NMEA Output                      |
+-------+----------------------------------+
| 21    | WindVane                         |
+-------+----------------------------------+
| 22    | SLCAN                            |
+-------+----------------------------------+
| 23    | RCIN                             |
+-------+----------------------------------+
| 24    | EFI Serial                       |
+-------+----------------------------------+
| 25    | LTM                              |
+-------+----------------------------------+
| 26    | RunCam                           |
+-------+----------------------------------+
| 27    | HottTelem                        |
+-------+----------------------------------+
| 28    | Scripting                        |
+-------+----------------------------------+
| 29    | Crossfire VTX                    |
+-------+----------------------------------+
| 30    | Generator                        |
+-------+----------------------------------+
| 31    | Winch                            |
+-------+----------------------------------+
| 32    | MSP                              |
+-------+----------------------------------+
| 33    | DJI FPV                          |
+-------+----------------------------------+
| 34    | AirSpeed                         |
+-------+----------------------------------+
| 35    | ADSB                             |
+-------+----------------------------------+
| 36    | AHRS                             |
+-------+----------------------------------+
| 37    | SmartAudio                       |
+-------+----------------------------------+
| 38    | FETtecOneWire                    |
+-------+----------------------------------+
| 39    | Torqeedo                         |
+-------+----------------------------------+
| 40    | AIS                              |
+-------+----------------------------------+
| 41    | CoDevESC                         |
+-------+----------------------------------+
| 42    | DisplayPort                      |
+-------+----------------------------------+
| 43    | MAVLink High Latency             |
+-------+----------------------------------+
| 44    | IRC Tramp                        |
+-------+----------------------------------+
| 45    | DDS XRCE                         |
+-------+----------------------------------+
| 46    | IMUDATA                          |
+-------+----------------------------------+
| 48    | PPP                              |
+-------+----------------------------------+
| 49    | i-BUS Telemetry                  |
+-------+----------------------------------+
| 50    | IOMCU                            |
+-------+----------------------------------+




.. _SCR_SDEV3_PROTO:

SCR\_SDEV3\_PROTO: Serial protocol of scripting serial device
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Serial protocol of scripting serial device


+-------+----------------------------------+
| Value | Meaning                          |
+=======+==================================+
| -1    | None                             |
+-------+----------------------------------+
| 1     | MAVLink1                         |
+-------+----------------------------------+
| 2     | MAVLink2                         |
+-------+----------------------------------+
| 3     | Frsky D                          |
+-------+----------------------------------+
| 4     | Frsky SPort                      |
+-------+----------------------------------+
| 5     | GPS                              |
+-------+----------------------------------+
| 7     | Alexmos Gimbal Serial            |
+-------+----------------------------------+
| 8     | Gimbal                           |
+-------+----------------------------------+
| 9     | Rangefinder                      |
+-------+----------------------------------+
| 10    | FrSky SPort Passthrough (OpenTX) |
+-------+----------------------------------+
| 11    | Lidar360                         |
+-------+----------------------------------+
| 13    | Beacon                           |
+-------+----------------------------------+
| 14    | Volz servo out                   |
+-------+----------------------------------+
| 15    | SBus servo out                   |
+-------+----------------------------------+
| 16    | ESC Telemetry                    |
+-------+----------------------------------+
| 17    | Devo Telemetry                   |
+-------+----------------------------------+
| 18    | OpticalFlow                      |
+-------+----------------------------------+
| 19    | RobotisServo                     |
+-------+----------------------------------+
| 20    | NMEA Output                      |
+-------+----------------------------------+
| 21    | WindVane                         |
+-------+----------------------------------+
| 22    | SLCAN                            |
+-------+----------------------------------+
| 23    | RCIN                             |
+-------+----------------------------------+
| 24    | EFI Serial                       |
+-------+----------------------------------+
| 25    | LTM                              |
+-------+----------------------------------+
| 26    | RunCam                           |
+-------+----------------------------------+
| 27    | HottTelem                        |
+-------+----------------------------------+
| 28    | Scripting                        |
+-------+----------------------------------+
| 29    | Crossfire VTX                    |
+-------+----------------------------------+
| 30    | Generator                        |
+-------+----------------------------------+
| 31    | Winch                            |
+-------+----------------------------------+
| 32    | MSP                              |
+-------+----------------------------------+
| 33    | DJI FPV                          |
+-------+----------------------------------+
| 34    | AirSpeed                         |
+-------+----------------------------------+
| 35    | ADSB                             |
+-------+----------------------------------+
| 36    | AHRS                             |
+-------+----------------------------------+
| 37    | SmartAudio                       |
+-------+----------------------------------+
| 38    | FETtecOneWire                    |
+-------+----------------------------------+
| 39    | Torqeedo                         |
+-------+----------------------------------+
| 40    | AIS                              |
+-------+----------------------------------+
| 41    | CoDevESC                         |
+-------+----------------------------------+
| 42    | DisplayPort                      |
+-------+----------------------------------+
| 43    | MAVLink High Latency             |
+-------+----------------------------------+
| 44    | IRC Tramp                        |
+-------+----------------------------------+
| 45    | DDS XRCE                         |
+-------+----------------------------------+
| 46    | IMUDATA                          |
+-------+----------------------------------+
| 48    | PPP                              |
+-------+----------------------------------+
| 49    | i-BUS Telemetry                  |
+-------+----------------------------------+
| 50    | IOMCU                            |
+-------+----------------------------------+





.. _parameters_SERIAL:

SERIAL Parameters
-----------------


.. _SERIAL0_BAUD:

SERIAL0\_BAUD: Serial0 baud rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used on the USB console\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+----------+-----------+
| Value    | Meaning   |
+==========+===========+
| 1        | 1200      |
+----------+-----------+
| 2        | 2400      |
+----------+-----------+
| 4        | 4800      |
+----------+-----------+
| 9        | 9600      |
+----------+-----------+
| 19       | 19200     |
+----------+-----------+
| 38       | 38400     |
+----------+-----------+
| 57       | 57600     |
+----------+-----------+
| 111      | 111100    |
+----------+-----------+
| 115      | 115200    |
+----------+-----------+
| 230      | 230400    |
+----------+-----------+
| 256      | 256000    |
+----------+-----------+
| 460      | 460800    |
+----------+-----------+
| 500      | 500000    |
+----------+-----------+
| 921      | 921600    |
+----------+-----------+
| 1500     | 1.5MBaud  |
+----------+-----------+
| 2000     | 2MBaud    |
+----------+-----------+
| 12500000 | 12.5MBaud |
+----------+-----------+




.. _SERIAL0_PROTOCOL:

SERIAL0\_PROTOCOL: Console protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol to use on the console\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| 1     | MAVLink1 |
+-------+----------+
| 2     | MAVLink2 |
+-------+----------+




.. _SERIAL1_PROTOCOL:

SERIAL1\_PROTOCOL: Telem1 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol to use on the Telem1 port\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+-------+----------------------------------+
| Value | Meaning                          |
+=======+==================================+
| -1    | None                             |
+-------+----------------------------------+
| 1     | MAVLink1                         |
+-------+----------------------------------+
| 2     | MAVLink2                         |
+-------+----------------------------------+
| 3     | Frsky D                          |
+-------+----------------------------------+
| 4     | Frsky SPort                      |
+-------+----------------------------------+
| 5     | GPS                              |
+-------+----------------------------------+
| 7     | Alexmos Gimbal Serial            |
+-------+----------------------------------+
| 8     | Gimbal                           |
+-------+----------------------------------+
| 9     | Rangefinder                      |
+-------+----------------------------------+
| 10    | FrSky SPort Passthrough (OpenTX) |
+-------+----------------------------------+
| 11    | Lidar360                         |
+-------+----------------------------------+
| 13    | Beacon                           |
+-------+----------------------------------+
| 14    | Volz servo out                   |
+-------+----------------------------------+
| 15    | SBus servo out                   |
+-------+----------------------------------+
| 16    | ESC Telemetry                    |
+-------+----------------------------------+
| 17    | Devo Telemetry                   |
+-------+----------------------------------+
| 18    | OpticalFlow                      |
+-------+----------------------------------+
| 19    | RobotisServo                     |
+-------+----------------------------------+
| 20    | NMEA Output                      |
+-------+----------------------------------+
| 21    | WindVane                         |
+-------+----------------------------------+
| 22    | SLCAN                            |
+-------+----------------------------------+
| 23    | RCIN                             |
+-------+----------------------------------+
| 24    | EFI Serial                       |
+-------+----------------------------------+
| 25    | LTM                              |
+-------+----------------------------------+
| 26    | RunCam                           |
+-------+----------------------------------+
| 27    | HottTelem                        |
+-------+----------------------------------+
| 28    | Scripting                        |
+-------+----------------------------------+
| 29    | Crossfire VTX                    |
+-------+----------------------------------+
| 30    | Generator                        |
+-------+----------------------------------+
| 31    | Winch                            |
+-------+----------------------------------+
| 32    | MSP                              |
+-------+----------------------------------+
| 33    | DJI FPV                          |
+-------+----------------------------------+
| 34    | AirSpeed                         |
+-------+----------------------------------+
| 35    | ADSB                             |
+-------+----------------------------------+
| 36    | AHRS                             |
+-------+----------------------------------+
| 37    | SmartAudio                       |
+-------+----------------------------------+
| 38    | FETtecOneWire                    |
+-------+----------------------------------+
| 39    | Torqeedo                         |
+-------+----------------------------------+
| 40    | AIS                              |
+-------+----------------------------------+
| 41    | CoDevESC                         |
+-------+----------------------------------+
| 42    | DisplayPort                      |
+-------+----------------------------------+
| 43    | MAVLink High Latency             |
+-------+----------------------------------+
| 44    | IRC Tramp                        |
+-------+----------------------------------+
| 45    | DDS XRCE                         |
+-------+----------------------------------+
| 46    | IMUDATA                          |
+-------+----------------------------------+
| 48    | PPP                              |
+-------+----------------------------------+
| 49    | i-BUS Telemetry                  |
+-------+----------------------------------+
| 50    | IOMCU                            |
+-------+----------------------------------+




.. _SERIAL1_BAUD:

SERIAL1\_BAUD: Telem1 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used on the Telem1 port\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+----------+-----------+
| Value    | Meaning   |
+==========+===========+
| 1        | 1200      |
+----------+-----------+
| 2        | 2400      |
+----------+-----------+
| 4        | 4800      |
+----------+-----------+
| 9        | 9600      |
+----------+-----------+
| 19       | 19200     |
+----------+-----------+
| 38       | 38400     |
+----------+-----------+
| 57       | 57600     |
+----------+-----------+
| 111      | 111100    |
+----------+-----------+
| 115      | 115200    |
+----------+-----------+
| 230      | 230400    |
+----------+-----------+
| 256      | 256000    |
+----------+-----------+
| 460      | 460800    |
+----------+-----------+
| 500      | 500000    |
+----------+-----------+
| 921      | 921600    |
+----------+-----------+
| 1500     | 1.5MBaud  |
+----------+-----------+
| 2000     | 2MBaud    |
+----------+-----------+
| 12500000 | 12.5MBaud |
+----------+-----------+




.. _SERIAL2_PROTOCOL:

SERIAL2\_PROTOCOL: Telemetry 2 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol to use on the Telem2 port\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+-------+----------------------------------+
| Value | Meaning                          |
+=======+==================================+
| -1    | None                             |
+-------+----------------------------------+
| 1     | MAVLink1                         |
+-------+----------------------------------+
| 2     | MAVLink2                         |
+-------+----------------------------------+
| 3     | Frsky D                          |
+-------+----------------------------------+
| 4     | Frsky SPort                      |
+-------+----------------------------------+
| 5     | GPS                              |
+-------+----------------------------------+
| 7     | Alexmos Gimbal Serial            |
+-------+----------------------------------+
| 8     | Gimbal                           |
+-------+----------------------------------+
| 9     | Rangefinder                      |
+-------+----------------------------------+
| 10    | FrSky SPort Passthrough (OpenTX) |
+-------+----------------------------------+
| 11    | Lidar360                         |
+-------+----------------------------------+
| 13    | Beacon                           |
+-------+----------------------------------+
| 14    | Volz servo out                   |
+-------+----------------------------------+
| 15    | SBus servo out                   |
+-------+----------------------------------+
| 16    | ESC Telemetry                    |
+-------+----------------------------------+
| 17    | Devo Telemetry                   |
+-------+----------------------------------+
| 18    | OpticalFlow                      |
+-------+----------------------------------+
| 19    | RobotisServo                     |
+-------+----------------------------------+
| 20    | NMEA Output                      |
+-------+----------------------------------+
| 21    | WindVane                         |
+-------+----------------------------------+
| 22    | SLCAN                            |
+-------+----------------------------------+
| 23    | RCIN                             |
+-------+----------------------------------+
| 24    | EFI Serial                       |
+-------+----------------------------------+
| 25    | LTM                              |
+-------+----------------------------------+
| 26    | RunCam                           |
+-------+----------------------------------+
| 27    | HottTelem                        |
+-------+----------------------------------+
| 28    | Scripting                        |
+-------+----------------------------------+
| 29    | Crossfire VTX                    |
+-------+----------------------------------+
| 30    | Generator                        |
+-------+----------------------------------+
| 31    | Winch                            |
+-------+----------------------------------+
| 32    | MSP                              |
+-------+----------------------------------+
| 33    | DJI FPV                          |
+-------+----------------------------------+
| 34    | AirSpeed                         |
+-------+----------------------------------+
| 35    | ADSB                             |
+-------+----------------------------------+
| 36    | AHRS                             |
+-------+----------------------------------+
| 37    | SmartAudio                       |
+-------+----------------------------------+
| 38    | FETtecOneWire                    |
+-------+----------------------------------+
| 39    | Torqeedo                         |
+-------+----------------------------------+
| 40    | AIS                              |
+-------+----------------------------------+
| 41    | CoDevESC                         |
+-------+----------------------------------+
| 42    | DisplayPort                      |
+-------+----------------------------------+
| 43    | MAVLink High Latency             |
+-------+----------------------------------+
| 44    | IRC Tramp                        |
+-------+----------------------------------+
| 45    | DDS XRCE                         |
+-------+----------------------------------+
| 46    | IMUDATA                          |
+-------+----------------------------------+
| 48    | PPP                              |
+-------+----------------------------------+
| 49    | i-BUS Telemetry                  |
+-------+----------------------------------+
| 50    | IOMCU                            |
+-------+----------------------------------+




.. _SERIAL2_BAUD:

SERIAL2\_BAUD: Telemetry 2 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate of the Telem2 port\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+----------+-----------+
| Value    | Meaning   |
+==========+===========+
| 1        | 1200      |
+----------+-----------+
| 2        | 2400      |
+----------+-----------+
| 4        | 4800      |
+----------+-----------+
| 9        | 9600      |
+----------+-----------+
| 19       | 19200     |
+----------+-----------+
| 38       | 38400     |
+----------+-----------+
| 57       | 57600     |
+----------+-----------+
| 111      | 111100    |
+----------+-----------+
| 115      | 115200    |
+----------+-----------+
| 230      | 230400    |
+----------+-----------+
| 256      | 256000    |
+----------+-----------+
| 460      | 460800    |
+----------+-----------+
| 500      | 500000    |
+----------+-----------+
| 921      | 921600    |
+----------+-----------+
| 1500     | 1.5MBaud  |
+----------+-----------+
| 2000     | 2MBaud    |
+----------+-----------+
| 12500000 | 12.5MBaud |
+----------+-----------+




.. _SERIAL3_PROTOCOL:

SERIAL3\_PROTOCOL: Serial 3 \(GPS\) protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol Serial 3 \(GPS\) should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+-------+----------------------------------+
| Value | Meaning                          |
+=======+==================================+
| -1    | None                             |
+-------+----------------------------------+
| 1     | MAVLink1                         |
+-------+----------------------------------+
| 2     | MAVLink2                         |
+-------+----------------------------------+
| 3     | Frsky D                          |
+-------+----------------------------------+
| 4     | Frsky SPort                      |
+-------+----------------------------------+
| 5     | GPS                              |
+-------+----------------------------------+
| 7     | Alexmos Gimbal Serial            |
+-------+----------------------------------+
| 8     | Gimbal                           |
+-------+----------------------------------+
| 9     | Rangefinder                      |
+-------+----------------------------------+
| 10    | FrSky SPort Passthrough (OpenTX) |
+-------+----------------------------------+
| 11    | Lidar360                         |
+-------+----------------------------------+
| 13    | Beacon                           |
+-------+----------------------------------+
| 14    | Volz servo out                   |
+-------+----------------------------------+
| 15    | SBus servo out                   |
+-------+----------------------------------+
| 16    | ESC Telemetry                    |
+-------+----------------------------------+
| 17    | Devo Telemetry                   |
+-------+----------------------------------+
| 18    | OpticalFlow                      |
+-------+----------------------------------+
| 19    | RobotisServo                     |
+-------+----------------------------------+
| 20    | NMEA Output                      |
+-------+----------------------------------+
| 21    | WindVane                         |
+-------+----------------------------------+
| 22    | SLCAN                            |
+-------+----------------------------------+
| 23    | RCIN                             |
+-------+----------------------------------+
| 24    | EFI Serial                       |
+-------+----------------------------------+
| 25    | LTM                              |
+-------+----------------------------------+
| 26    | RunCam                           |
+-------+----------------------------------+
| 27    | HottTelem                        |
+-------+----------------------------------+
| 28    | Scripting                        |
+-------+----------------------------------+
| 29    | Crossfire VTX                    |
+-------+----------------------------------+
| 30    | Generator                        |
+-------+----------------------------------+
| 31    | Winch                            |
+-------+----------------------------------+
| 32    | MSP                              |
+-------+----------------------------------+
| 33    | DJI FPV                          |
+-------+----------------------------------+
| 34    | AirSpeed                         |
+-------+----------------------------------+
| 35    | ADSB                             |
+-------+----------------------------------+
| 36    | AHRS                             |
+-------+----------------------------------+
| 37    | SmartAudio                       |
+-------+----------------------------------+
| 38    | FETtecOneWire                    |
+-------+----------------------------------+
| 39    | Torqeedo                         |
+-------+----------------------------------+
| 40    | AIS                              |
+-------+----------------------------------+
| 41    | CoDevESC                         |
+-------+----------------------------------+
| 42    | DisplayPort                      |
+-------+----------------------------------+
| 43    | MAVLink High Latency             |
+-------+----------------------------------+
| 44    | IRC Tramp                        |
+-------+----------------------------------+
| 45    | DDS XRCE                         |
+-------+----------------------------------+
| 46    | IMUDATA                          |
+-------+----------------------------------+
| 48    | PPP                              |
+-------+----------------------------------+
| 49    | i-BUS Telemetry                  |
+-------+----------------------------------+
| 50    | IOMCU                            |
+-------+----------------------------------+




.. _SERIAL3_BAUD:

SERIAL3\_BAUD: Serial 3 \(GPS\) Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for the Serial 3 \(GPS\)\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+----------+-----------+
| Value    | Meaning   |
+==========+===========+
| 1        | 1200      |
+----------+-----------+
| 2        | 2400      |
+----------+-----------+
| 4        | 4800      |
+----------+-----------+
| 9        | 9600      |
+----------+-----------+
| 19       | 19200     |
+----------+-----------+
| 38       | 38400     |
+----------+-----------+
| 57       | 57600     |
+----------+-----------+
| 111      | 111100    |
+----------+-----------+
| 115      | 115200    |
+----------+-----------+
| 230      | 230400    |
+----------+-----------+
| 256      | 256000    |
+----------+-----------+
| 460      | 460800    |
+----------+-----------+
| 500      | 500000    |
+----------+-----------+
| 921      | 921600    |
+----------+-----------+
| 1500     | 1.5MBaud  |
+----------+-----------+
| 2000     | 2MBaud    |
+----------+-----------+
| 12500000 | 12.5MBaud |
+----------+-----------+




.. _SERIAL4_PROTOCOL:

SERIAL4\_PROTOCOL: Serial4 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol Serial4 port should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+-------+----------------------------------+
| Value | Meaning                          |
+=======+==================================+
| -1    | None                             |
+-------+----------------------------------+
| 1     | MAVLink1                         |
+-------+----------------------------------+
| 2     | MAVLink2                         |
+-------+----------------------------------+
| 3     | Frsky D                          |
+-------+----------------------------------+
| 4     | Frsky SPort                      |
+-------+----------------------------------+
| 5     | GPS                              |
+-------+----------------------------------+
| 7     | Alexmos Gimbal Serial            |
+-------+----------------------------------+
| 8     | Gimbal                           |
+-------+----------------------------------+
| 9     | Rangefinder                      |
+-------+----------------------------------+
| 10    | FrSky SPort Passthrough (OpenTX) |
+-------+----------------------------------+
| 11    | Lidar360                         |
+-------+----------------------------------+
| 13    | Beacon                           |
+-------+----------------------------------+
| 14    | Volz servo out                   |
+-------+----------------------------------+
| 15    | SBus servo out                   |
+-------+----------------------------------+
| 16    | ESC Telemetry                    |
+-------+----------------------------------+
| 17    | Devo Telemetry                   |
+-------+----------------------------------+
| 18    | OpticalFlow                      |
+-------+----------------------------------+
| 19    | RobotisServo                     |
+-------+----------------------------------+
| 20    | NMEA Output                      |
+-------+----------------------------------+
| 21    | WindVane                         |
+-------+----------------------------------+
| 22    | SLCAN                            |
+-------+----------------------------------+
| 23    | RCIN                             |
+-------+----------------------------------+
| 24    | EFI Serial                       |
+-------+----------------------------------+
| 25    | LTM                              |
+-------+----------------------------------+
| 26    | RunCam                           |
+-------+----------------------------------+
| 27    | HottTelem                        |
+-------+----------------------------------+
| 28    | Scripting                        |
+-------+----------------------------------+
| 29    | Crossfire VTX                    |
+-------+----------------------------------+
| 30    | Generator                        |
+-------+----------------------------------+
| 31    | Winch                            |
+-------+----------------------------------+
| 32    | MSP                              |
+-------+----------------------------------+
| 33    | DJI FPV                          |
+-------+----------------------------------+
| 34    | AirSpeed                         |
+-------+----------------------------------+
| 35    | ADSB                             |
+-------+----------------------------------+
| 36    | AHRS                             |
+-------+----------------------------------+
| 37    | SmartAudio                       |
+-------+----------------------------------+
| 38    | FETtecOneWire                    |
+-------+----------------------------------+
| 39    | Torqeedo                         |
+-------+----------------------------------+
| 40    | AIS                              |
+-------+----------------------------------+
| 41    | CoDevESC                         |
+-------+----------------------------------+
| 42    | DisplayPort                      |
+-------+----------------------------------+
| 43    | MAVLink High Latency             |
+-------+----------------------------------+
| 44    | IRC Tramp                        |
+-------+----------------------------------+
| 45    | DDS XRCE                         |
+-------+----------------------------------+
| 46    | IMUDATA                          |
+-------+----------------------------------+
| 48    | PPP                              |
+-------+----------------------------------+
| 49    | i-BUS Telemetry                  |
+-------+----------------------------------+
| 50    | IOMCU                            |
+-------+----------------------------------+




.. _SERIAL4_BAUD:

SERIAL4\_BAUD: Serial 4 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for Serial4\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+----------+-----------+
| Value    | Meaning   |
+==========+===========+
| 1        | 1200      |
+----------+-----------+
| 2        | 2400      |
+----------+-----------+
| 4        | 4800      |
+----------+-----------+
| 9        | 9600      |
+----------+-----------+
| 19       | 19200     |
+----------+-----------+
| 38       | 38400     |
+----------+-----------+
| 57       | 57600     |
+----------+-----------+
| 111      | 111100    |
+----------+-----------+
| 115      | 115200    |
+----------+-----------+
| 230      | 230400    |
+----------+-----------+
| 256      | 256000    |
+----------+-----------+
| 460      | 460800    |
+----------+-----------+
| 500      | 500000    |
+----------+-----------+
| 921      | 921600    |
+----------+-----------+
| 1500     | 1.5MBaud  |
+----------+-----------+
| 2000     | 2MBaud    |
+----------+-----------+
| 12500000 | 12.5MBaud |
+----------+-----------+




.. _SERIAL5_PROTOCOL:

SERIAL5\_PROTOCOL: Serial5 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol Serial5 port should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+-------+----------------------------------+
| Value | Meaning                          |
+=======+==================================+
| -1    | None                             |
+-------+----------------------------------+
| 1     | MAVLink1                         |
+-------+----------------------------------+
| 2     | MAVLink2                         |
+-------+----------------------------------+
| 3     | Frsky D                          |
+-------+----------------------------------+
| 4     | Frsky SPort                      |
+-------+----------------------------------+
| 5     | GPS                              |
+-------+----------------------------------+
| 7     | Alexmos Gimbal Serial            |
+-------+----------------------------------+
| 8     | Gimbal                           |
+-------+----------------------------------+
| 9     | Rangefinder                      |
+-------+----------------------------------+
| 10    | FrSky SPort Passthrough (OpenTX) |
+-------+----------------------------------+
| 11    | Lidar360                         |
+-------+----------------------------------+
| 13    | Beacon                           |
+-------+----------------------------------+
| 14    | Volz servo out                   |
+-------+----------------------------------+
| 15    | SBus servo out                   |
+-------+----------------------------------+
| 16    | ESC Telemetry                    |
+-------+----------------------------------+
| 17    | Devo Telemetry                   |
+-------+----------------------------------+
| 18    | OpticalFlow                      |
+-------+----------------------------------+
| 19    | RobotisServo                     |
+-------+----------------------------------+
| 20    | NMEA Output                      |
+-------+----------------------------------+
| 21    | WindVane                         |
+-------+----------------------------------+
| 22    | SLCAN                            |
+-------+----------------------------------+
| 23    | RCIN                             |
+-------+----------------------------------+
| 24    | EFI Serial                       |
+-------+----------------------------------+
| 25    | LTM                              |
+-------+----------------------------------+
| 26    | RunCam                           |
+-------+----------------------------------+
| 27    | HottTelem                        |
+-------+----------------------------------+
| 28    | Scripting                        |
+-------+----------------------------------+
| 29    | Crossfire VTX                    |
+-------+----------------------------------+
| 30    | Generator                        |
+-------+----------------------------------+
| 31    | Winch                            |
+-------+----------------------------------+
| 32    | MSP                              |
+-------+----------------------------------+
| 33    | DJI FPV                          |
+-------+----------------------------------+
| 34    | AirSpeed                         |
+-------+----------------------------------+
| 35    | ADSB                             |
+-------+----------------------------------+
| 36    | AHRS                             |
+-------+----------------------------------+
| 37    | SmartAudio                       |
+-------+----------------------------------+
| 38    | FETtecOneWire                    |
+-------+----------------------------------+
| 39    | Torqeedo                         |
+-------+----------------------------------+
| 40    | AIS                              |
+-------+----------------------------------+
| 41    | CoDevESC                         |
+-------+----------------------------------+
| 42    | DisplayPort                      |
+-------+----------------------------------+
| 43    | MAVLink High Latency             |
+-------+----------------------------------+
| 44    | IRC Tramp                        |
+-------+----------------------------------+
| 45    | DDS XRCE                         |
+-------+----------------------------------+
| 46    | IMUDATA                          |
+-------+----------------------------------+
| 48    | PPP                              |
+-------+----------------------------------+
| 49    | i-BUS Telemetry                  |
+-------+----------------------------------+
| 50    | IOMCU                            |
+-------+----------------------------------+




.. _SERIAL5_BAUD:

SERIAL5\_BAUD: Serial 5 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for Serial5\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+----------+-----------+
| Value    | Meaning   |
+==========+===========+
| 1        | 1200      |
+----------+-----------+
| 2        | 2400      |
+----------+-----------+
| 4        | 4800      |
+----------+-----------+
| 9        | 9600      |
+----------+-----------+
| 19       | 19200     |
+----------+-----------+
| 38       | 38400     |
+----------+-----------+
| 57       | 57600     |
+----------+-----------+
| 111      | 111100    |
+----------+-----------+
| 115      | 115200    |
+----------+-----------+
| 230      | 230400    |
+----------+-----------+
| 256      | 256000    |
+----------+-----------+
| 460      | 460800    |
+----------+-----------+
| 500      | 500000    |
+----------+-----------+
| 921      | 921600    |
+----------+-----------+
| 1500     | 1.5MBaud  |
+----------+-----------+
| 2000     | 2MBaud    |
+----------+-----------+
| 12500000 | 12.5MBaud |
+----------+-----------+




.. _SERIAL6_PROTOCOL:

SERIAL6\_PROTOCOL: Serial6 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol Serial6 port should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+-------+----------------------------------+
| Value | Meaning                          |
+=======+==================================+
| -1    | None                             |
+-------+----------------------------------+
| 1     | MAVLink1                         |
+-------+----------------------------------+
| 2     | MAVLink2                         |
+-------+----------------------------------+
| 3     | Frsky D                          |
+-------+----------------------------------+
| 4     | Frsky SPort                      |
+-------+----------------------------------+
| 5     | GPS                              |
+-------+----------------------------------+
| 7     | Alexmos Gimbal Serial            |
+-------+----------------------------------+
| 8     | Gimbal                           |
+-------+----------------------------------+
| 9     | Rangefinder                      |
+-------+----------------------------------+
| 10    | FrSky SPort Passthrough (OpenTX) |
+-------+----------------------------------+
| 11    | Lidar360                         |
+-------+----------------------------------+
| 13    | Beacon                           |
+-------+----------------------------------+
| 14    | Volz servo out                   |
+-------+----------------------------------+
| 15    | SBus servo out                   |
+-------+----------------------------------+
| 16    | ESC Telemetry                    |
+-------+----------------------------------+
| 17    | Devo Telemetry                   |
+-------+----------------------------------+
| 18    | OpticalFlow                      |
+-------+----------------------------------+
| 19    | RobotisServo                     |
+-------+----------------------------------+
| 20    | NMEA Output                      |
+-------+----------------------------------+
| 21    | WindVane                         |
+-------+----------------------------------+
| 22    | SLCAN                            |
+-------+----------------------------------+
| 23    | RCIN                             |
+-------+----------------------------------+
| 24    | EFI Serial                       |
+-------+----------------------------------+
| 25    | LTM                              |
+-------+----------------------------------+
| 26    | RunCam                           |
+-------+----------------------------------+
| 27    | HottTelem                        |
+-------+----------------------------------+
| 28    | Scripting                        |
+-------+----------------------------------+
| 29    | Crossfire VTX                    |
+-------+----------------------------------+
| 30    | Generator                        |
+-------+----------------------------------+
| 31    | Winch                            |
+-------+----------------------------------+
| 32    | MSP                              |
+-------+----------------------------------+
| 33    | DJI FPV                          |
+-------+----------------------------------+
| 34    | AirSpeed                         |
+-------+----------------------------------+
| 35    | ADSB                             |
+-------+----------------------------------+
| 36    | AHRS                             |
+-------+----------------------------------+
| 37    | SmartAudio                       |
+-------+----------------------------------+
| 38    | FETtecOneWire                    |
+-------+----------------------------------+
| 39    | Torqeedo                         |
+-------+----------------------------------+
| 40    | AIS                              |
+-------+----------------------------------+
| 41    | CoDevESC                         |
+-------+----------------------------------+
| 42    | DisplayPort                      |
+-------+----------------------------------+
| 43    | MAVLink High Latency             |
+-------+----------------------------------+
| 44    | IRC Tramp                        |
+-------+----------------------------------+
| 45    | DDS XRCE                         |
+-------+----------------------------------+
| 46    | IMUDATA                          |
+-------+----------------------------------+
| 48    | PPP                              |
+-------+----------------------------------+
| 49    | i-BUS Telemetry                  |
+-------+----------------------------------+
| 50    | IOMCU                            |
+-------+----------------------------------+




.. _SERIAL6_BAUD:

SERIAL6\_BAUD: Serial 6 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for Serial6\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+----------+-----------+
| Value    | Meaning   |
+==========+===========+
| 1        | 1200      |
+----------+-----------+
| 2        | 2400      |
+----------+-----------+
| 4        | 4800      |
+----------+-----------+
| 9        | 9600      |
+----------+-----------+
| 19       | 19200     |
+----------+-----------+
| 38       | 38400     |
+----------+-----------+
| 57       | 57600     |
+----------+-----------+
| 111      | 111100    |
+----------+-----------+
| 115      | 115200    |
+----------+-----------+
| 230      | 230400    |
+----------+-----------+
| 256      | 256000    |
+----------+-----------+
| 460      | 460800    |
+----------+-----------+
| 500      | 500000    |
+----------+-----------+
| 921      | 921600    |
+----------+-----------+
| 1500     | 1.5MBaud  |
+----------+-----------+
| 2000     | 2MBaud    |
+----------+-----------+
| 12500000 | 12.5MBaud |
+----------+-----------+




.. _SERIAL1_OPTIONS:

SERIAL1\_OPTIONS: Telem1 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards\.  NOTE that two bits have moved from this parameter into MAVn\_OPTIONS\!


+-----+------------------------------------------------------------+
| Bit | Meaning                                                    |
+=====+============================================================+
| 0   | InvertRX                                                   |
+-----+------------------------------------------------------------+
| 1   | InvertTX                                                   |
+-----+------------------------------------------------------------+
| 2   | HalfDuplex                                                 |
+-----+------------------------------------------------------------+
| 3   | SwapTXRX                                                   |
+-----+------------------------------------------------------------+
| 4   | RX_PullDown                                                |
+-----+------------------------------------------------------------+
| 5   | RX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 6   | TX_PullDown                                                |
+-----+------------------------------------------------------------+
| 7   | TX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 8   | RX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 9   | TX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 10  | Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7) |
+-----+------------------------------------------------------------+
| 11  | DisableFIFO                                                |
+-----+------------------------------------------------------------+
| 12  | Ignore Streamrate (moved to MAVn_OPTIONS >4.7)             |
+-----+------------------------------------------------------------+




.. _SERIAL2_OPTIONS:

SERIAL2\_OPTIONS: Telem2 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards\.  NOTE that two bits have moved from this parameter into MAVn\_OPTIONS\!


+-----+------------------------------------------------------------+
| Bit | Meaning                                                    |
+=====+============================================================+
| 0   | InvertRX                                                   |
+-----+------------------------------------------------------------+
| 1   | InvertTX                                                   |
+-----+------------------------------------------------------------+
| 2   | HalfDuplex                                                 |
+-----+------------------------------------------------------------+
| 3   | SwapTXRX                                                   |
+-----+------------------------------------------------------------+
| 4   | RX_PullDown                                                |
+-----+------------------------------------------------------------+
| 5   | RX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 6   | TX_PullDown                                                |
+-----+------------------------------------------------------------+
| 7   | TX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 8   | RX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 9   | TX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 10  | Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7) |
+-----+------------------------------------------------------------+
| 11  | DisableFIFO                                                |
+-----+------------------------------------------------------------+
| 12  | Ignore Streamrate (moved to MAVn_OPTIONS >4.7)             |
+-----+------------------------------------------------------------+




.. _SERIAL3_OPTIONS:

SERIAL3\_OPTIONS: Serial3 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards\.  NOTE that two bits have moved from this parameter into MAVn\_OPTIONS\!


+-----+------------------------------------------------------------+
| Bit | Meaning                                                    |
+=====+============================================================+
| 0   | InvertRX                                                   |
+-----+------------------------------------------------------------+
| 1   | InvertTX                                                   |
+-----+------------------------------------------------------------+
| 2   | HalfDuplex                                                 |
+-----+------------------------------------------------------------+
| 3   | SwapTXRX                                                   |
+-----+------------------------------------------------------------+
| 4   | RX_PullDown                                                |
+-----+------------------------------------------------------------+
| 5   | RX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 6   | TX_PullDown                                                |
+-----+------------------------------------------------------------+
| 7   | TX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 8   | RX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 9   | TX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 10  | Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7) |
+-----+------------------------------------------------------------+
| 11  | DisableFIFO                                                |
+-----+------------------------------------------------------------+
| 12  | Ignore Streamrate (moved to MAVn_OPTIONS >4.7)             |
+-----+------------------------------------------------------------+




.. _SERIAL4_OPTIONS:

SERIAL4\_OPTIONS: Serial4 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards\.  NOTE that two bits have moved from this parameter into MAVn\_OPTIONS\!


+-----+------------------------------------------------------------+
| Bit | Meaning                                                    |
+=====+============================================================+
| 0   | InvertRX                                                   |
+-----+------------------------------------------------------------+
| 1   | InvertTX                                                   |
+-----+------------------------------------------------------------+
| 2   | HalfDuplex                                                 |
+-----+------------------------------------------------------------+
| 3   | SwapTXRX                                                   |
+-----+------------------------------------------------------------+
| 4   | RX_PullDown                                                |
+-----+------------------------------------------------------------+
| 5   | RX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 6   | TX_PullDown                                                |
+-----+------------------------------------------------------------+
| 7   | TX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 8   | RX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 9   | TX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 10  | Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7) |
+-----+------------------------------------------------------------+
| 11  | DisableFIFO                                                |
+-----+------------------------------------------------------------+
| 12  | Ignore Streamrate (moved to MAVn_OPTIONS >4.7)             |
+-----+------------------------------------------------------------+




.. _SERIAL5_OPTIONS:

SERIAL5\_OPTIONS: Serial5 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards\.  NOTE that two bits have moved from this parameter into MAVn\_OPTIONS\!


+-----+------------------------------------------------------------+
| Bit | Meaning                                                    |
+=====+============================================================+
| 0   | InvertRX                                                   |
+-----+------------------------------------------------------------+
| 1   | InvertTX                                                   |
+-----+------------------------------------------------------------+
| 2   | HalfDuplex                                                 |
+-----+------------------------------------------------------------+
| 3   | SwapTXRX                                                   |
+-----+------------------------------------------------------------+
| 4   | RX_PullDown                                                |
+-----+------------------------------------------------------------+
| 5   | RX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 6   | TX_PullDown                                                |
+-----+------------------------------------------------------------+
| 7   | TX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 8   | RX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 9   | TX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 10  | Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7) |
+-----+------------------------------------------------------------+
| 11  | DisableFIFO                                                |
+-----+------------------------------------------------------------+
| 12  | Ignore Streamrate (moved to MAVn_OPTIONS >4.7)             |
+-----+------------------------------------------------------------+




.. _SERIAL6_OPTIONS:

SERIAL6\_OPTIONS: Serial6 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards\.  NOTE that two bits have moved from this parameter into MAVn\_OPTIONS\!


+-----+------------------------------------------------------------+
| Bit | Meaning                                                    |
+=====+============================================================+
| 0   | InvertRX                                                   |
+-----+------------------------------------------------------------+
| 1   | InvertTX                                                   |
+-----+------------------------------------------------------------+
| 2   | HalfDuplex                                                 |
+-----+------------------------------------------------------------+
| 3   | SwapTXRX                                                   |
+-----+------------------------------------------------------------+
| 4   | RX_PullDown                                                |
+-----+------------------------------------------------------------+
| 5   | RX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 6   | TX_PullDown                                                |
+-----+------------------------------------------------------------+
| 7   | TX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 8   | RX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 9   | TX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 10  | Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7) |
+-----+------------------------------------------------------------+
| 11  | DisableFIFO                                                |
+-----+------------------------------------------------------------+
| 12  | Ignore Streamrate (moved to MAVn_OPTIONS >4.7)             |
+-----+------------------------------------------------------------+




.. _SERIAL_PASS1:

SERIAL\_PASS1: Serial passthru first port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets one side of pass\-through between two serial ports\. Once both sides are set then all data received on either port will be passed to the other port


+-------+----------+
| Value | Meaning  |
+=======+==========+
| -1    | Disabled |
+-------+----------+
| 0     | Serial0  |
+-------+----------+
| 1     | Serial1  |
+-------+----------+
| 2     | Serial2  |
+-------+----------+
| 3     | Serial3  |
+-------+----------+
| 4     | Serial4  |
+-------+----------+
| 5     | Serial5  |
+-------+----------+
| 6     | Serial6  |
+-------+----------+




.. _SERIAL_PASS2:

SERIAL\_PASS2: Serial passthru second port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets one side of pass\-through between two serial ports\. Once both sides are set then all data received on either port will be passed to the other port\. This parameter is normally reset to \-1 on reboot\, disabling passthrough\. If SERIAL\_PASSTIMO is set to \-1 then it is not reset on reboot\.


+-------+----------+
| Value | Meaning  |
+=======+==========+
| -1    | Disabled |
+-------+----------+
| 0     | Serial0  |
+-------+----------+
| 1     | Serial1  |
+-------+----------+
| 2     | Serial2  |
+-------+----------+
| 3     | Serial3  |
+-------+----------+
| 4     | Serial4  |
+-------+----------+
| 5     | Serial5  |
+-------+----------+
| 6     | Serial6  |
+-------+----------+




.. _SERIAL_PASSTIMO:

SERIAL\_PASSTIMO: Serial passthru timeout
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

This sets a timeout for serial pass\-through in seconds\. When the pass\-through is enabled by setting the SERIAL\_PASS1 and SERIAL\_PASS2 parameters then it remains in effect until no data comes from the first port for SERIAL\_PASSTIMO seconds\. This allows the port to revent to its normal usage \(such as MAVLink connection to a GCS\) when it is no longer needed\. A value of 0 means no timeout\. A value of \-1 means no timeout and the SERIAL\_PASS2 parameter is not reset on reboot\.


+-----------+---------+
| Range     | Units   |
+===========+=========+
| -1 to 120 | seconds |
+-----------+---------+




.. _SERIAL7_PROTOCOL:

SERIAL7\_PROTOCOL: Serial7 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol Serial7 port should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+-------+----------------------------------+
| Value | Meaning                          |
+=======+==================================+
| -1    | None                             |
+-------+----------------------------------+
| 1     | MAVLink1                         |
+-------+----------------------------------+
| 2     | MAVLink2                         |
+-------+----------------------------------+
| 3     | Frsky D                          |
+-------+----------------------------------+
| 4     | Frsky SPort                      |
+-------+----------------------------------+
| 5     | GPS                              |
+-------+----------------------------------+
| 7     | Alexmos Gimbal Serial            |
+-------+----------------------------------+
| 8     | Gimbal                           |
+-------+----------------------------------+
| 9     | Rangefinder                      |
+-------+----------------------------------+
| 10    | FrSky SPort Passthrough (OpenTX) |
+-------+----------------------------------+
| 11    | Lidar360                         |
+-------+----------------------------------+
| 13    | Beacon                           |
+-------+----------------------------------+
| 14    | Volz servo out                   |
+-------+----------------------------------+
| 15    | SBus servo out                   |
+-------+----------------------------------+
| 16    | ESC Telemetry                    |
+-------+----------------------------------+
| 17    | Devo Telemetry                   |
+-------+----------------------------------+
| 18    | OpticalFlow                      |
+-------+----------------------------------+
| 19    | RobotisServo                     |
+-------+----------------------------------+
| 20    | NMEA Output                      |
+-------+----------------------------------+
| 21    | WindVane                         |
+-------+----------------------------------+
| 22    | SLCAN                            |
+-------+----------------------------------+
| 23    | RCIN                             |
+-------+----------------------------------+
| 24    | EFI Serial                       |
+-------+----------------------------------+
| 25    | LTM                              |
+-------+----------------------------------+
| 26    | RunCam                           |
+-------+----------------------------------+
| 27    | HottTelem                        |
+-------+----------------------------------+
| 28    | Scripting                        |
+-------+----------------------------------+
| 29    | Crossfire VTX                    |
+-------+----------------------------------+
| 30    | Generator                        |
+-------+----------------------------------+
| 31    | Winch                            |
+-------+----------------------------------+
| 32    | MSP                              |
+-------+----------------------------------+
| 33    | DJI FPV                          |
+-------+----------------------------------+
| 34    | AirSpeed                         |
+-------+----------------------------------+
| 35    | ADSB                             |
+-------+----------------------------------+
| 36    | AHRS                             |
+-------+----------------------------------+
| 37    | SmartAudio                       |
+-------+----------------------------------+
| 38    | FETtecOneWire                    |
+-------+----------------------------------+
| 39    | Torqeedo                         |
+-------+----------------------------------+
| 40    | AIS                              |
+-------+----------------------------------+
| 41    | CoDevESC                         |
+-------+----------------------------------+
| 42    | DisplayPort                      |
+-------+----------------------------------+
| 43    | MAVLink High Latency             |
+-------+----------------------------------+
| 44    | IRC Tramp                        |
+-------+----------------------------------+
| 45    | DDS XRCE                         |
+-------+----------------------------------+
| 46    | IMUDATA                          |
+-------+----------------------------------+
| 48    | PPP                              |
+-------+----------------------------------+
| 49    | i-BUS Telemetry                  |
+-------+----------------------------------+
| 50    | IOMCU                            |
+-------+----------------------------------+




.. _SERIAL7_BAUD:

SERIAL7\_BAUD: Serial 7 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for Serial7\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+----------+-----------+
| Value    | Meaning   |
+==========+===========+
| 1        | 1200      |
+----------+-----------+
| 2        | 2400      |
+----------+-----------+
| 4        | 4800      |
+----------+-----------+
| 9        | 9600      |
+----------+-----------+
| 19       | 19200     |
+----------+-----------+
| 38       | 38400     |
+----------+-----------+
| 57       | 57600     |
+----------+-----------+
| 111      | 111100    |
+----------+-----------+
| 115      | 115200    |
+----------+-----------+
| 230      | 230400    |
+----------+-----------+
| 256      | 256000    |
+----------+-----------+
| 460      | 460800    |
+----------+-----------+
| 500      | 500000    |
+----------+-----------+
| 921      | 921600    |
+----------+-----------+
| 1500     | 1.5MBaud  |
+----------+-----------+
| 2000     | 2MBaud    |
+----------+-----------+
| 12500000 | 12.5MBaud |
+----------+-----------+




.. _SERIAL7_OPTIONS:

SERIAL7\_OPTIONS: Serial7 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards\.  NOTE that two bits have moved from this parameter into MAVn\_OPTIONS\!


+-----+------------------------------------------------------------+
| Bit | Meaning                                                    |
+=====+============================================================+
| 0   | InvertRX                                                   |
+-----+------------------------------------------------------------+
| 1   | InvertTX                                                   |
+-----+------------------------------------------------------------+
| 2   | HalfDuplex                                                 |
+-----+------------------------------------------------------------+
| 3   | SwapTXRX                                                   |
+-----+------------------------------------------------------------+
| 4   | RX_PullDown                                                |
+-----+------------------------------------------------------------+
| 5   | RX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 6   | TX_PullDown                                                |
+-----+------------------------------------------------------------+
| 7   | TX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 8   | RX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 9   | TX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 10  | Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7) |
+-----+------------------------------------------------------------+
| 11  | DisableFIFO                                                |
+-----+------------------------------------------------------------+
| 12  | Ignore Streamrate (moved to MAVn_OPTIONS >4.7)             |
+-----+------------------------------------------------------------+




.. _SERIAL8_PROTOCOL:

SERIAL8\_PROTOCOL: Serial8 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol Serial8 port should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+-------+----------------------------------+
| Value | Meaning                          |
+=======+==================================+
| -1    | None                             |
+-------+----------------------------------+
| 1     | MAVLink1                         |
+-------+----------------------------------+
| 2     | MAVLink2                         |
+-------+----------------------------------+
| 3     | Frsky D                          |
+-------+----------------------------------+
| 4     | Frsky SPort                      |
+-------+----------------------------------+
| 5     | GPS                              |
+-------+----------------------------------+
| 7     | Alexmos Gimbal Serial            |
+-------+----------------------------------+
| 8     | Gimbal                           |
+-------+----------------------------------+
| 9     | Rangefinder                      |
+-------+----------------------------------+
| 10    | FrSky SPort Passthrough (OpenTX) |
+-------+----------------------------------+
| 11    | Lidar360                         |
+-------+----------------------------------+
| 13    | Beacon                           |
+-------+----------------------------------+
| 14    | Volz servo out                   |
+-------+----------------------------------+
| 15    | SBus servo out                   |
+-------+----------------------------------+
| 16    | ESC Telemetry                    |
+-------+----------------------------------+
| 17    | Devo Telemetry                   |
+-------+----------------------------------+
| 18    | OpticalFlow                      |
+-------+----------------------------------+
| 19    | RobotisServo                     |
+-------+----------------------------------+
| 20    | NMEA Output                      |
+-------+----------------------------------+
| 21    | WindVane                         |
+-------+----------------------------------+
| 22    | SLCAN                            |
+-------+----------------------------------+
| 23    | RCIN                             |
+-------+----------------------------------+
| 24    | EFI Serial                       |
+-------+----------------------------------+
| 25    | LTM                              |
+-------+----------------------------------+
| 26    | RunCam                           |
+-------+----------------------------------+
| 27    | HottTelem                        |
+-------+----------------------------------+
| 28    | Scripting                        |
+-------+----------------------------------+
| 29    | Crossfire VTX                    |
+-------+----------------------------------+
| 30    | Generator                        |
+-------+----------------------------------+
| 31    | Winch                            |
+-------+----------------------------------+
| 32    | MSP                              |
+-------+----------------------------------+
| 33    | DJI FPV                          |
+-------+----------------------------------+
| 34    | AirSpeed                         |
+-------+----------------------------------+
| 35    | ADSB                             |
+-------+----------------------------------+
| 36    | AHRS                             |
+-------+----------------------------------+
| 37    | SmartAudio                       |
+-------+----------------------------------+
| 38    | FETtecOneWire                    |
+-------+----------------------------------+
| 39    | Torqeedo                         |
+-------+----------------------------------+
| 40    | AIS                              |
+-------+----------------------------------+
| 41    | CoDevESC                         |
+-------+----------------------------------+
| 42    | DisplayPort                      |
+-------+----------------------------------+
| 43    | MAVLink High Latency             |
+-------+----------------------------------+
| 44    | IRC Tramp                        |
+-------+----------------------------------+
| 45    | DDS XRCE                         |
+-------+----------------------------------+
| 46    | IMUDATA                          |
+-------+----------------------------------+
| 48    | PPP                              |
+-------+----------------------------------+
| 49    | i-BUS Telemetry                  |
+-------+----------------------------------+
| 50    | IOMCU                            |
+-------+----------------------------------+




.. _SERIAL8_BAUD:

SERIAL8\_BAUD: Serial 8 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for Serial8\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+----------+-----------+
| Value    | Meaning   |
+==========+===========+
| 1        | 1200      |
+----------+-----------+
| 2        | 2400      |
+----------+-----------+
| 4        | 4800      |
+----------+-----------+
| 9        | 9600      |
+----------+-----------+
| 19       | 19200     |
+----------+-----------+
| 38       | 38400     |
+----------+-----------+
| 57       | 57600     |
+----------+-----------+
| 111      | 111100    |
+----------+-----------+
| 115      | 115200    |
+----------+-----------+
| 230      | 230400    |
+----------+-----------+
| 256      | 256000    |
+----------+-----------+
| 460      | 460800    |
+----------+-----------+
| 500      | 500000    |
+----------+-----------+
| 921      | 921600    |
+----------+-----------+
| 1500     | 1.5MBaud  |
+----------+-----------+
| 2000     | 2MBaud    |
+----------+-----------+
| 12500000 | 12.5MBaud |
+----------+-----------+




.. _SERIAL8_OPTIONS:

SERIAL8\_OPTIONS: Serial8 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards\.  NOTE that two bits have moved from this parameter into MAVn\_OPTIONS\!


+-----+------------------------------------------------------------+
| Bit | Meaning                                                    |
+=====+============================================================+
| 0   | InvertRX                                                   |
+-----+------------------------------------------------------------+
| 1   | InvertTX                                                   |
+-----+------------------------------------------------------------+
| 2   | HalfDuplex                                                 |
+-----+------------------------------------------------------------+
| 3   | SwapTXRX                                                   |
+-----+------------------------------------------------------------+
| 4   | RX_PullDown                                                |
+-----+------------------------------------------------------------+
| 5   | RX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 6   | TX_PullDown                                                |
+-----+------------------------------------------------------------+
| 7   | TX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 8   | RX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 9   | TX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 10  | Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7) |
+-----+------------------------------------------------------------+
| 11  | DisableFIFO                                                |
+-----+------------------------------------------------------------+
| 12  | Ignore Streamrate (moved to MAVn_OPTIONS >4.7)             |
+-----+------------------------------------------------------------+




.. _SERIAL9_PROTOCOL:

SERIAL9\_PROTOCOL: Serial9 protocol selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Control what protocol Serial9 port should be used for\. Note that the Frsky options require external converter hardware\. See the wiki for details\.


+-------+----------------------------------+
| Value | Meaning                          |
+=======+==================================+
| -1    | None                             |
+-------+----------------------------------+
| 1     | MAVLink1                         |
+-------+----------------------------------+
| 2     | MAVLink2                         |
+-------+----------------------------------+
| 3     | Frsky D                          |
+-------+----------------------------------+
| 4     | Frsky SPort                      |
+-------+----------------------------------+
| 5     | GPS                              |
+-------+----------------------------------+
| 7     | Alexmos Gimbal Serial            |
+-------+----------------------------------+
| 8     | Gimbal                           |
+-------+----------------------------------+
| 9     | Rangefinder                      |
+-------+----------------------------------+
| 10    | FrSky SPort Passthrough (OpenTX) |
+-------+----------------------------------+
| 11    | Lidar360                         |
+-------+----------------------------------+
| 13    | Beacon                           |
+-------+----------------------------------+
| 14    | Volz servo out                   |
+-------+----------------------------------+
| 15    | SBus servo out                   |
+-------+----------------------------------+
| 16    | ESC Telemetry                    |
+-------+----------------------------------+
| 17    | Devo Telemetry                   |
+-------+----------------------------------+
| 18    | OpticalFlow                      |
+-------+----------------------------------+
| 19    | RobotisServo                     |
+-------+----------------------------------+
| 20    | NMEA Output                      |
+-------+----------------------------------+
| 21    | WindVane                         |
+-------+----------------------------------+
| 22    | SLCAN                            |
+-------+----------------------------------+
| 23    | RCIN                             |
+-------+----------------------------------+
| 24    | EFI Serial                       |
+-------+----------------------------------+
| 25    | LTM                              |
+-------+----------------------------------+
| 26    | RunCam                           |
+-------+----------------------------------+
| 27    | HottTelem                        |
+-------+----------------------------------+
| 28    | Scripting                        |
+-------+----------------------------------+
| 29    | Crossfire VTX                    |
+-------+----------------------------------+
| 30    | Generator                        |
+-------+----------------------------------+
| 31    | Winch                            |
+-------+----------------------------------+
| 32    | MSP                              |
+-------+----------------------------------+
| 33    | DJI FPV                          |
+-------+----------------------------------+
| 34    | AirSpeed                         |
+-------+----------------------------------+
| 35    | ADSB                             |
+-------+----------------------------------+
| 36    | AHRS                             |
+-------+----------------------------------+
| 37    | SmartAudio                       |
+-------+----------------------------------+
| 38    | FETtecOneWire                    |
+-------+----------------------------------+
| 39    | Torqeedo                         |
+-------+----------------------------------+
| 40    | AIS                              |
+-------+----------------------------------+
| 41    | CoDevESC                         |
+-------+----------------------------------+
| 42    | DisplayPort                      |
+-------+----------------------------------+
| 43    | MAVLink High Latency             |
+-------+----------------------------------+
| 44    | IRC Tramp                        |
+-------+----------------------------------+
| 45    | DDS XRCE                         |
+-------+----------------------------------+
| 46    | IMUDATA                          |
+-------+----------------------------------+
| 48    | PPP                              |
+-------+----------------------------------+
| 49    | i-BUS Telemetry                  |
+-------+----------------------------------+
| 50    | IOMCU                            |
+-------+----------------------------------+




.. _SERIAL9_BAUD:

SERIAL9\_BAUD: Serial 9 Baud Rate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


The baud rate used for Serial8\. Most stm32\-based boards can support rates of up to 1500\. If you setup a rate you cannot support and then can\'t connect to your board you should load a firmware from a different vehicle type\. That will reset all your parameters to defaults\.


+----------+-----------+
| Value    | Meaning   |
+==========+===========+
| 1        | 1200      |
+----------+-----------+
| 2        | 2400      |
+----------+-----------+
| 4        | 4800      |
+----------+-----------+
| 9        | 9600      |
+----------+-----------+
| 19       | 19200     |
+----------+-----------+
| 38       | 38400     |
+----------+-----------+
| 57       | 57600     |
+----------+-----------+
| 111      | 111100    |
+----------+-----------+
| 115      | 115200    |
+----------+-----------+
| 230      | 230400    |
+----------+-----------+
| 256      | 256000    |
+----------+-----------+
| 460      | 460800    |
+----------+-----------+
| 500      | 500000    |
+----------+-----------+
| 921      | 921600    |
+----------+-----------+
| 1500     | 1.5MBaud  |
+----------+-----------+
| 2000     | 2MBaud    |
+----------+-----------+
| 12500000 | 12.5MBaud |
+----------+-----------+




.. _SERIAL9_OPTIONS:

SERIAL9\_OPTIONS: Serial9 options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Control over UART options\. The InvertRX option controls invert of the receive pin\. The InvertTX option controls invert of the transmit pin\. The HalfDuplex option controls half\-duplex \(onewire\) mode\, where both transmit and receive is done on the transmit wire\. The Swap option allows the RX and TX pins to be swapped on STM32F7 based boards\.  NOTE that two bits have moved from this parameter into MAVn\_OPTIONS\!


+-----+------------------------------------------------------------+
| Bit | Meaning                                                    |
+=====+============================================================+
| 0   | InvertRX                                                   |
+-----+------------------------------------------------------------+
| 1   | InvertTX                                                   |
+-----+------------------------------------------------------------+
| 2   | HalfDuplex                                                 |
+-----+------------------------------------------------------------+
| 3   | SwapTXRX                                                   |
+-----+------------------------------------------------------------+
| 4   | RX_PullDown                                                |
+-----+------------------------------------------------------------+
| 5   | RX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 6   | TX_PullDown                                                |
+-----+------------------------------------------------------------+
| 7   | TX_PullUp                                                  |
+-----+------------------------------------------------------------+
| 8   | RX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 9   | TX_NoDMA                                                   |
+-----+------------------------------------------------------------+
| 10  | Don't forward mavlink to/from (moved to MAVn_OPTIONS >4.7) |
+-----+------------------------------------------------------------+
| 11  | DisableFIFO                                                |
+-----+------------------------------------------------------------+
| 12  | Ignore Streamrate (moved to MAVn_OPTIONS >4.7)             |
+-----+------------------------------------------------------------+





.. _parameters_STAT:

STAT Parameters
---------------


.. _STAT_BOOTCNT:

STAT\_BOOTCNT: Boot Count
~~~~~~~~~~~~~~~~~~~~~~~~~


Number of times board has been booted


+----------+
| ReadOnly |
+==========+
| True     |
+----------+




.. _STAT_FLTTIME:

STAT\_FLTTIME: Total FlightTime
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Total FlightTime \(seconds\)


+----------+---------+
| ReadOnly | Units   |
+==========+=========+
| True     | seconds |
+----------+---------+




.. _STAT_RUNTIME:

STAT\_RUNTIME: Total RunTime
~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Total time autopilot has run


+----------+---------+
| ReadOnly | Units   |
+==========+=========+
| True     | seconds |
+----------+---------+




.. _STAT_RESET:

STAT\_RESET: Statistics Reset Time
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Seconds since January 1st 2016 \(Unix epoch\+1451606400\) since statistics reset \(set to 0 to reset statistics\, other set values will be ignored\)


+----------+---------+
| ReadOnly | Units   |
+==========+=========+
| True     | seconds |
+----------+---------+




.. _STAT_FLTCNT:

STAT\_FLTCNT: Total Flight Count
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Total number of flights


+----------+
| ReadOnly |
+==========+
| True     |
+----------+





.. _parameters_TEMP:

TEMP Parameters
---------------


.. _TEMP_LOG:

TEMP\_LOG: Logging
~~~~~~~~~~~~~~~~~~


Enables temperature sensor logging


+-------+---------------------------------------------------+
| Value | Meaning                                           |
+=======+===================================================+
| 0     | Disabled                                          |
+-------+---------------------------------------------------+
| 1     | Log all instances                                 |
+-------+---------------------------------------------------+
| 2     | Log only instances with sensor source set to None |
+-------+---------------------------------------------------+





.. _parameters_TEMP1_:

TEMP1\_ Parameters
------------------


.. _TEMP1_TYPE:

TEMP1\_TYPE: Temperature Sensor Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Enables temperature sensors


+-------+----------------------+
| Value | Meaning              |
+=======+======================+
| 0     | Disabled             |
+-------+----------------------+
| 1     | TSYS01               |
+-------+----------------------+
| 2     | MCP9600              |
+-------+----------------------+
| 3     | MAX31865 2 or 4 wire |
+-------+----------------------+
| 4     | TSYS03               |
+-------+----------------------+
| 5     | Analog               |
+-------+----------------------+
| 6     | DroneCAN             |
+-------+----------------------+
| 7     | MLX90614             |
+-------+----------------------+
| 8     | SHT3x                |
+-------+----------------------+
| 9     | MAX31865 3 wire      |
+-------+----------------------+




.. _TEMP1_BUS:

TEMP1\_BUS: Temperature sensor bus
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor bus number\, typically used to select from multiple I2C buses


+--------+
| Range  |
+========+
| 0 to 3 |
+--------+




.. _TEMP1_ADDR:

TEMP1\_ADDR: Temperature sensor address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor address\, typically used for I2C address


+----------+
| Range    |
+==========+
| 0 to 127 |
+----------+




.. _TEMP1_SRC:

TEMP1\_SRC: Sensor Source
~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source is used to designate which device\'s temperature report will be replaced by this temperature sensor\'s data\. If 0 \(None\) then the data is only available via log\. In the future a new Motor temperature report will be created for returning data directly\.


+-------+---------------------------+
| Value | Meaning                   |
+=======+===========================+
| 0     | None                      |
+-------+---------------------------+
| 1     | ESC                       |
+-------+---------------------------+
| 2     | Motor                     |
+-------+---------------------------+
| 3     | Battery Index             |
+-------+---------------------------+
| 4     | Battery ID/SerialNumber   |
+-------+---------------------------+
| 5     | CAN based Pitot tube      |
+-------+---------------------------+
| 6     | DroneCAN-out on AP_Periph |
+-------+---------------------------+




.. _TEMP1_SRC_ID:

TEMP1\_SRC\_ID: Sensor Source Identification
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source Identification is used to replace a specific instance of a system component\'s temperature report with the temp sensor\'s\. Examples\: TEMP\_SRC \= 1 \(ESC\)\, TEMP\_SRC\_ID \= 1 will set the temp of ESC1\. TEMP\_SRC \= 3 \(BatteryIndex\)\,TEMP\_SRC\_ID \= 2 will set the temp of BATT2\. TEMP\_SRC \= 4 \(BatteryId\/SerialNum\)\,TEMP\_SRC\_ID\=42 will set the temp of all batteries that have param BATTn\_SERIAL \= 42\.


.. _TEMP1_PIN:

TEMP1\_PIN: Temperature sensor analog voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the analog input pin that should be used for temprature monitoring\. Values for some autopilots are given as examples\. Search wiki for \"Analog pins\"\.


+-------+--------------------------------------+
| Value | Meaning                              |
+=======+======================================+
| -1    | Disabled                             |
+-------+--------------------------------------+
| 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 |
+-------+--------------------------------------+
| 5     | Navigator                            |
+-------+--------------------------------------+
| 13    | Pixhawk2_PM2/CubeOrange_PM2          |
+-------+--------------------------------------+
| 14    | CubeOrange                           |
+-------+--------------------------------------+
| 16    | Durandal                             |
+-------+--------------------------------------+
| 100   | PX4-v1                               |
+-------+--------------------------------------+




.. _TEMP1_A0:

TEMP1\_A0: Temperature sensor analog 0th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a0 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP1_A1:

TEMP1\_A1: Temperature sensor analog 1st polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a1 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP1_A2:

TEMP1\_A2: Temperature sensor analog 2nd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a2 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP1_A3:

TEMP1\_A3: Temperature sensor analog 3rd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a3 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP1_A4:

TEMP1\_A4: Temperature sensor analog 4th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a4 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP1_A5:

TEMP1\_A5: Temperature sensor analog 5th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a5 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP1_MSG_ID:

TEMP1\_MSG\_ID: Temperature sensor DroneCAN message ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the message device ID this backend listens for


+------------+
| Range      |
+============+
| 0 to 65535 |
+------------+




.. _TEMP1_RTD_NOM:

TEMP1\_RTD\_NOM: Nominal RTD resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Nominal RTD resistance used to calculate temperature\, typically 100 or 1000 ohms\.


.. _TEMP1_RTD_REF:

TEMP1\_RTD\_REF: RTD reference resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reference resistance used to calculate temperature\, in ohms



.. _parameters_TEMP2_:

TEMP2\_ Parameters
------------------


.. _TEMP2_TYPE:

TEMP2\_TYPE: Temperature Sensor Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Enables temperature sensors


+-------+----------------------+
| Value | Meaning              |
+=======+======================+
| 0     | Disabled             |
+-------+----------------------+
| 1     | TSYS01               |
+-------+----------------------+
| 2     | MCP9600              |
+-------+----------------------+
| 3     | MAX31865 2 or 4 wire |
+-------+----------------------+
| 4     | TSYS03               |
+-------+----------------------+
| 5     | Analog               |
+-------+----------------------+
| 6     | DroneCAN             |
+-------+----------------------+
| 7     | MLX90614             |
+-------+----------------------+
| 8     | SHT3x                |
+-------+----------------------+
| 9     | MAX31865 3 wire      |
+-------+----------------------+




.. _TEMP2_BUS:

TEMP2\_BUS: Temperature sensor bus
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor bus number\, typically used to select from multiple I2C buses


+--------+
| Range  |
+========+
| 0 to 3 |
+--------+




.. _TEMP2_ADDR:

TEMP2\_ADDR: Temperature sensor address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor address\, typically used for I2C address


+----------+
| Range    |
+==========+
| 0 to 127 |
+----------+




.. _TEMP2_SRC:

TEMP2\_SRC: Sensor Source
~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source is used to designate which device\'s temperature report will be replaced by this temperature sensor\'s data\. If 0 \(None\) then the data is only available via log\. In the future a new Motor temperature report will be created for returning data directly\.


+-------+---------------------------+
| Value | Meaning                   |
+=======+===========================+
| 0     | None                      |
+-------+---------------------------+
| 1     | ESC                       |
+-------+---------------------------+
| 2     | Motor                     |
+-------+---------------------------+
| 3     | Battery Index             |
+-------+---------------------------+
| 4     | Battery ID/SerialNumber   |
+-------+---------------------------+
| 5     | CAN based Pitot tube      |
+-------+---------------------------+
| 6     | DroneCAN-out on AP_Periph |
+-------+---------------------------+




.. _TEMP2_SRC_ID:

TEMP2\_SRC\_ID: Sensor Source Identification
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source Identification is used to replace a specific instance of a system component\'s temperature report with the temp sensor\'s\. Examples\: TEMP\_SRC \= 1 \(ESC\)\, TEMP\_SRC\_ID \= 1 will set the temp of ESC1\. TEMP\_SRC \= 3 \(BatteryIndex\)\,TEMP\_SRC\_ID \= 2 will set the temp of BATT2\. TEMP\_SRC \= 4 \(BatteryId\/SerialNum\)\,TEMP\_SRC\_ID\=42 will set the temp of all batteries that have param BATTn\_SERIAL \= 42\.


.. _TEMP2_PIN:

TEMP2\_PIN: Temperature sensor analog voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the analog input pin that should be used for temprature monitoring\. Values for some autopilots are given as examples\. Search wiki for \"Analog pins\"\.


+-------+--------------------------------------+
| Value | Meaning                              |
+=======+======================================+
| -1    | Disabled                             |
+-------+--------------------------------------+
| 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 |
+-------+--------------------------------------+
| 5     | Navigator                            |
+-------+--------------------------------------+
| 13    | Pixhawk2_PM2/CubeOrange_PM2          |
+-------+--------------------------------------+
| 14    | CubeOrange                           |
+-------+--------------------------------------+
| 16    | Durandal                             |
+-------+--------------------------------------+
| 100   | PX4-v1                               |
+-------+--------------------------------------+




.. _TEMP2_A0:

TEMP2\_A0: Temperature sensor analog 0th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a0 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP2_A1:

TEMP2\_A1: Temperature sensor analog 1st polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a1 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP2_A2:

TEMP2\_A2: Temperature sensor analog 2nd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a2 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP2_A3:

TEMP2\_A3: Temperature sensor analog 3rd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a3 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP2_A4:

TEMP2\_A4: Temperature sensor analog 4th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a4 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP2_A5:

TEMP2\_A5: Temperature sensor analog 5th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a5 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP2_MSG_ID:

TEMP2\_MSG\_ID: Temperature sensor DroneCAN message ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the message device ID this backend listens for


+------------+
| Range      |
+============+
| 0 to 65535 |
+------------+




.. _TEMP2_RTD_NOM:

TEMP2\_RTD\_NOM: Nominal RTD resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Nominal RTD resistance used to calculate temperature\, typically 100 or 1000 ohms\.


.. _TEMP2_RTD_REF:

TEMP2\_RTD\_REF: RTD reference resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reference resistance used to calculate temperature\, in ohms



.. _parameters_TEMP3_:

TEMP3\_ Parameters
------------------


.. _TEMP3_TYPE:

TEMP3\_TYPE: Temperature Sensor Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Enables temperature sensors


+-------+----------------------+
| Value | Meaning              |
+=======+======================+
| 0     | Disabled             |
+-------+----------------------+
| 1     | TSYS01               |
+-------+----------------------+
| 2     | MCP9600              |
+-------+----------------------+
| 3     | MAX31865 2 or 4 wire |
+-------+----------------------+
| 4     | TSYS03               |
+-------+----------------------+
| 5     | Analog               |
+-------+----------------------+
| 6     | DroneCAN             |
+-------+----------------------+
| 7     | MLX90614             |
+-------+----------------------+
| 8     | SHT3x                |
+-------+----------------------+
| 9     | MAX31865 3 wire      |
+-------+----------------------+




.. _TEMP3_BUS:

TEMP3\_BUS: Temperature sensor bus
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor bus number\, typically used to select from multiple I2C buses


+--------+
| Range  |
+========+
| 0 to 3 |
+--------+




.. _TEMP3_ADDR:

TEMP3\_ADDR: Temperature sensor address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor address\, typically used for I2C address


+----------+
| Range    |
+==========+
| 0 to 127 |
+----------+




.. _TEMP3_SRC:

TEMP3\_SRC: Sensor Source
~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source is used to designate which device\'s temperature report will be replaced by this temperature sensor\'s data\. If 0 \(None\) then the data is only available via log\. In the future a new Motor temperature report will be created for returning data directly\.


+-------+---------------------------+
| Value | Meaning                   |
+=======+===========================+
| 0     | None                      |
+-------+---------------------------+
| 1     | ESC                       |
+-------+---------------------------+
| 2     | Motor                     |
+-------+---------------------------+
| 3     | Battery Index             |
+-------+---------------------------+
| 4     | Battery ID/SerialNumber   |
+-------+---------------------------+
| 5     | CAN based Pitot tube      |
+-------+---------------------------+
| 6     | DroneCAN-out on AP_Periph |
+-------+---------------------------+




.. _TEMP3_SRC_ID:

TEMP3\_SRC\_ID: Sensor Source Identification
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source Identification is used to replace a specific instance of a system component\'s temperature report with the temp sensor\'s\. Examples\: TEMP\_SRC \= 1 \(ESC\)\, TEMP\_SRC\_ID \= 1 will set the temp of ESC1\. TEMP\_SRC \= 3 \(BatteryIndex\)\,TEMP\_SRC\_ID \= 2 will set the temp of BATT2\. TEMP\_SRC \= 4 \(BatteryId\/SerialNum\)\,TEMP\_SRC\_ID\=42 will set the temp of all batteries that have param BATTn\_SERIAL \= 42\.


.. _TEMP3_PIN:

TEMP3\_PIN: Temperature sensor analog voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the analog input pin that should be used for temprature monitoring\. Values for some autopilots are given as examples\. Search wiki for \"Analog pins\"\.


+-------+--------------------------------------+
| Value | Meaning                              |
+=======+======================================+
| -1    | Disabled                             |
+-------+--------------------------------------+
| 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 |
+-------+--------------------------------------+
| 5     | Navigator                            |
+-------+--------------------------------------+
| 13    | Pixhawk2_PM2/CubeOrange_PM2          |
+-------+--------------------------------------+
| 14    | CubeOrange                           |
+-------+--------------------------------------+
| 16    | Durandal                             |
+-------+--------------------------------------+
| 100   | PX4-v1                               |
+-------+--------------------------------------+




.. _TEMP3_A0:

TEMP3\_A0: Temperature sensor analog 0th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a0 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP3_A1:

TEMP3\_A1: Temperature sensor analog 1st polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a1 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP3_A2:

TEMP3\_A2: Temperature sensor analog 2nd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a2 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP3_A3:

TEMP3\_A3: Temperature sensor analog 3rd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a3 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP3_A4:

TEMP3\_A4: Temperature sensor analog 4th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a4 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP3_A5:

TEMP3\_A5: Temperature sensor analog 5th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a5 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP3_MSG_ID:

TEMP3\_MSG\_ID: Temperature sensor DroneCAN message ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the message device ID this backend listens for


+------------+
| Range      |
+============+
| 0 to 65535 |
+------------+




.. _TEMP3_RTD_NOM:

TEMP3\_RTD\_NOM: Nominal RTD resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Nominal RTD resistance used to calculate temperature\, typically 100 or 1000 ohms\.


.. _TEMP3_RTD_REF:

TEMP3\_RTD\_REF: RTD reference resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reference resistance used to calculate temperature\, in ohms



.. _parameters_TEMP4_:

TEMP4\_ Parameters
------------------


.. _TEMP4_TYPE:

TEMP4\_TYPE: Temperature Sensor Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Enables temperature sensors


+-------+----------------------+
| Value | Meaning              |
+=======+======================+
| 0     | Disabled             |
+-------+----------------------+
| 1     | TSYS01               |
+-------+----------------------+
| 2     | MCP9600              |
+-------+----------------------+
| 3     | MAX31865 2 or 4 wire |
+-------+----------------------+
| 4     | TSYS03               |
+-------+----------------------+
| 5     | Analog               |
+-------+----------------------+
| 6     | DroneCAN             |
+-------+----------------------+
| 7     | MLX90614             |
+-------+----------------------+
| 8     | SHT3x                |
+-------+----------------------+
| 9     | MAX31865 3 wire      |
+-------+----------------------+




.. _TEMP4_BUS:

TEMP4\_BUS: Temperature sensor bus
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor bus number\, typically used to select from multiple I2C buses


+--------+
| Range  |
+========+
| 0 to 3 |
+--------+




.. _TEMP4_ADDR:

TEMP4\_ADDR: Temperature sensor address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor address\, typically used for I2C address


+----------+
| Range    |
+==========+
| 0 to 127 |
+----------+




.. _TEMP4_SRC:

TEMP4\_SRC: Sensor Source
~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source is used to designate which device\'s temperature report will be replaced by this temperature sensor\'s data\. If 0 \(None\) then the data is only available via log\. In the future a new Motor temperature report will be created for returning data directly\.


+-------+---------------------------+
| Value | Meaning                   |
+=======+===========================+
| 0     | None                      |
+-------+---------------------------+
| 1     | ESC                       |
+-------+---------------------------+
| 2     | Motor                     |
+-------+---------------------------+
| 3     | Battery Index             |
+-------+---------------------------+
| 4     | Battery ID/SerialNumber   |
+-------+---------------------------+
| 5     | CAN based Pitot tube      |
+-------+---------------------------+
| 6     | DroneCAN-out on AP_Periph |
+-------+---------------------------+




.. _TEMP4_SRC_ID:

TEMP4\_SRC\_ID: Sensor Source Identification
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source Identification is used to replace a specific instance of a system component\'s temperature report with the temp sensor\'s\. Examples\: TEMP\_SRC \= 1 \(ESC\)\, TEMP\_SRC\_ID \= 1 will set the temp of ESC1\. TEMP\_SRC \= 3 \(BatteryIndex\)\,TEMP\_SRC\_ID \= 2 will set the temp of BATT2\. TEMP\_SRC \= 4 \(BatteryId\/SerialNum\)\,TEMP\_SRC\_ID\=42 will set the temp of all batteries that have param BATTn\_SERIAL \= 42\.


.. _TEMP4_PIN:

TEMP4\_PIN: Temperature sensor analog voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the analog input pin that should be used for temprature monitoring\. Values for some autopilots are given as examples\. Search wiki for \"Analog pins\"\.


+-------+--------------------------------------+
| Value | Meaning                              |
+=======+======================================+
| -1    | Disabled                             |
+-------+--------------------------------------+
| 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 |
+-------+--------------------------------------+
| 5     | Navigator                            |
+-------+--------------------------------------+
| 13    | Pixhawk2_PM2/CubeOrange_PM2          |
+-------+--------------------------------------+
| 14    | CubeOrange                           |
+-------+--------------------------------------+
| 16    | Durandal                             |
+-------+--------------------------------------+
| 100   | PX4-v1                               |
+-------+--------------------------------------+




.. _TEMP4_A0:

TEMP4\_A0: Temperature sensor analog 0th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a0 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP4_A1:

TEMP4\_A1: Temperature sensor analog 1st polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a1 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP4_A2:

TEMP4\_A2: Temperature sensor analog 2nd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a2 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP4_A3:

TEMP4\_A3: Temperature sensor analog 3rd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a3 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP4_A4:

TEMP4\_A4: Temperature sensor analog 4th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a4 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP4_A5:

TEMP4\_A5: Temperature sensor analog 5th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a5 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP4_MSG_ID:

TEMP4\_MSG\_ID: Temperature sensor DroneCAN message ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the message device ID this backend listens for


+------------+
| Range      |
+============+
| 0 to 65535 |
+------------+




.. _TEMP4_RTD_NOM:

TEMP4\_RTD\_NOM: Nominal RTD resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Nominal RTD resistance used to calculate temperature\, typically 100 or 1000 ohms\.


.. _TEMP4_RTD_REF:

TEMP4\_RTD\_REF: RTD reference resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reference resistance used to calculate temperature\, in ohms



.. _parameters_TEMP5_:

TEMP5\_ Parameters
------------------


.. _TEMP5_TYPE:

TEMP5\_TYPE: Temperature Sensor Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Enables temperature sensors


+-------+----------------------+
| Value | Meaning              |
+=======+======================+
| 0     | Disabled             |
+-------+----------------------+
| 1     | TSYS01               |
+-------+----------------------+
| 2     | MCP9600              |
+-------+----------------------+
| 3     | MAX31865 2 or 4 wire |
+-------+----------------------+
| 4     | TSYS03               |
+-------+----------------------+
| 5     | Analog               |
+-------+----------------------+
| 6     | DroneCAN             |
+-------+----------------------+
| 7     | MLX90614             |
+-------+----------------------+
| 8     | SHT3x                |
+-------+----------------------+
| 9     | MAX31865 3 wire      |
+-------+----------------------+




.. _TEMP5_BUS:

TEMP5\_BUS: Temperature sensor bus
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor bus number\, typically used to select from multiple I2C buses


+--------+
| Range  |
+========+
| 0 to 3 |
+--------+




.. _TEMP5_ADDR:

TEMP5\_ADDR: Temperature sensor address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor address\, typically used for I2C address


+----------+
| Range    |
+==========+
| 0 to 127 |
+----------+




.. _TEMP5_SRC:

TEMP5\_SRC: Sensor Source
~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source is used to designate which device\'s temperature report will be replaced by this temperature sensor\'s data\. If 0 \(None\) then the data is only available via log\. In the future a new Motor temperature report will be created for returning data directly\.


+-------+---------------------------+
| Value | Meaning                   |
+=======+===========================+
| 0     | None                      |
+-------+---------------------------+
| 1     | ESC                       |
+-------+---------------------------+
| 2     | Motor                     |
+-------+---------------------------+
| 3     | Battery Index             |
+-------+---------------------------+
| 4     | Battery ID/SerialNumber   |
+-------+---------------------------+
| 5     | CAN based Pitot tube      |
+-------+---------------------------+
| 6     | DroneCAN-out on AP_Periph |
+-------+---------------------------+




.. _TEMP5_SRC_ID:

TEMP5\_SRC\_ID: Sensor Source Identification
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source Identification is used to replace a specific instance of a system component\'s temperature report with the temp sensor\'s\. Examples\: TEMP\_SRC \= 1 \(ESC\)\, TEMP\_SRC\_ID \= 1 will set the temp of ESC1\. TEMP\_SRC \= 3 \(BatteryIndex\)\,TEMP\_SRC\_ID \= 2 will set the temp of BATT2\. TEMP\_SRC \= 4 \(BatteryId\/SerialNum\)\,TEMP\_SRC\_ID\=42 will set the temp of all batteries that have param BATTn\_SERIAL \= 42\.


.. _TEMP5_PIN:

TEMP5\_PIN: Temperature sensor analog voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the analog input pin that should be used for temprature monitoring\. Values for some autopilots are given as examples\. Search wiki for \"Analog pins\"\.


+-------+--------------------------------------+
| Value | Meaning                              |
+=======+======================================+
| -1    | Disabled                             |
+-------+--------------------------------------+
| 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 |
+-------+--------------------------------------+
| 5     | Navigator                            |
+-------+--------------------------------------+
| 13    | Pixhawk2_PM2/CubeOrange_PM2          |
+-------+--------------------------------------+
| 14    | CubeOrange                           |
+-------+--------------------------------------+
| 16    | Durandal                             |
+-------+--------------------------------------+
| 100   | PX4-v1                               |
+-------+--------------------------------------+




.. _TEMP5_A0:

TEMP5\_A0: Temperature sensor analog 0th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a0 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP5_A1:

TEMP5\_A1: Temperature sensor analog 1st polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a1 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP5_A2:

TEMP5\_A2: Temperature sensor analog 2nd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a2 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP5_A3:

TEMP5\_A3: Temperature sensor analog 3rd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a3 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP5_A4:

TEMP5\_A4: Temperature sensor analog 4th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a4 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP5_A5:

TEMP5\_A5: Temperature sensor analog 5th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a5 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP5_MSG_ID:

TEMP5\_MSG\_ID: Temperature sensor DroneCAN message ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the message device ID this backend listens for


+------------+
| Range      |
+============+
| 0 to 65535 |
+------------+




.. _TEMP5_RTD_NOM:

TEMP5\_RTD\_NOM: Nominal RTD resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Nominal RTD resistance used to calculate temperature\, typically 100 or 1000 ohms\.


.. _TEMP5_RTD_REF:

TEMP5\_RTD\_REF: RTD reference resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reference resistance used to calculate temperature\, in ohms



.. _parameters_TEMP6_:

TEMP6\_ Parameters
------------------


.. _TEMP6_TYPE:

TEMP6\_TYPE: Temperature Sensor Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Enables temperature sensors


+-------+----------------------+
| Value | Meaning              |
+=======+======================+
| 0     | Disabled             |
+-------+----------------------+
| 1     | TSYS01               |
+-------+----------------------+
| 2     | MCP9600              |
+-------+----------------------+
| 3     | MAX31865 2 or 4 wire |
+-------+----------------------+
| 4     | TSYS03               |
+-------+----------------------+
| 5     | Analog               |
+-------+----------------------+
| 6     | DroneCAN             |
+-------+----------------------+
| 7     | MLX90614             |
+-------+----------------------+
| 8     | SHT3x                |
+-------+----------------------+
| 9     | MAX31865 3 wire      |
+-------+----------------------+




.. _TEMP6_BUS:

TEMP6\_BUS: Temperature sensor bus
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor bus number\, typically used to select from multiple I2C buses


+--------+
| Range  |
+========+
| 0 to 3 |
+--------+




.. _TEMP6_ADDR:

TEMP6\_ADDR: Temperature sensor address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor address\, typically used for I2C address


+----------+
| Range    |
+==========+
| 0 to 127 |
+----------+




.. _TEMP6_SRC:

TEMP6\_SRC: Sensor Source
~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source is used to designate which device\'s temperature report will be replaced by this temperature sensor\'s data\. If 0 \(None\) then the data is only available via log\. In the future a new Motor temperature report will be created for returning data directly\.


+-------+---------------------------+
| Value | Meaning                   |
+=======+===========================+
| 0     | None                      |
+-------+---------------------------+
| 1     | ESC                       |
+-------+---------------------------+
| 2     | Motor                     |
+-------+---------------------------+
| 3     | Battery Index             |
+-------+---------------------------+
| 4     | Battery ID/SerialNumber   |
+-------+---------------------------+
| 5     | CAN based Pitot tube      |
+-------+---------------------------+
| 6     | DroneCAN-out on AP_Periph |
+-------+---------------------------+




.. _TEMP6_SRC_ID:

TEMP6\_SRC\_ID: Sensor Source Identification
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source Identification is used to replace a specific instance of a system component\'s temperature report with the temp sensor\'s\. Examples\: TEMP\_SRC \= 1 \(ESC\)\, TEMP\_SRC\_ID \= 1 will set the temp of ESC1\. TEMP\_SRC \= 3 \(BatteryIndex\)\,TEMP\_SRC\_ID \= 2 will set the temp of BATT2\. TEMP\_SRC \= 4 \(BatteryId\/SerialNum\)\,TEMP\_SRC\_ID\=42 will set the temp of all batteries that have param BATTn\_SERIAL \= 42\.


.. _TEMP6_PIN:

TEMP6\_PIN: Temperature sensor analog voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the analog input pin that should be used for temprature monitoring\. Values for some autopilots are given as examples\. Search wiki for \"Analog pins\"\.


+-------+--------------------------------------+
| Value | Meaning                              |
+=======+======================================+
| -1    | Disabled                             |
+-------+--------------------------------------+
| 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 |
+-------+--------------------------------------+
| 5     | Navigator                            |
+-------+--------------------------------------+
| 13    | Pixhawk2_PM2/CubeOrange_PM2          |
+-------+--------------------------------------+
| 14    | CubeOrange                           |
+-------+--------------------------------------+
| 16    | Durandal                             |
+-------+--------------------------------------+
| 100   | PX4-v1                               |
+-------+--------------------------------------+




.. _TEMP6_A0:

TEMP6\_A0: Temperature sensor analog 0th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a0 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP6_A1:

TEMP6\_A1: Temperature sensor analog 1st polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a1 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP6_A2:

TEMP6\_A2: Temperature sensor analog 2nd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a2 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP6_A3:

TEMP6\_A3: Temperature sensor analog 3rd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a3 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP6_A4:

TEMP6\_A4: Temperature sensor analog 4th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a4 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP6_A5:

TEMP6\_A5: Temperature sensor analog 5th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a5 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP6_MSG_ID:

TEMP6\_MSG\_ID: Temperature sensor DroneCAN message ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the message device ID this backend listens for


+------------+
| Range      |
+============+
| 0 to 65535 |
+------------+




.. _TEMP6_RTD_NOM:

TEMP6\_RTD\_NOM: Nominal RTD resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Nominal RTD resistance used to calculate temperature\, typically 100 or 1000 ohms\.


.. _TEMP6_RTD_REF:

TEMP6\_RTD\_REF: RTD reference resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reference resistance used to calculate temperature\, in ohms



.. _parameters_TEMP7_:

TEMP7\_ Parameters
------------------


.. _TEMP7_TYPE:

TEMP7\_TYPE: Temperature Sensor Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Enables temperature sensors


+-------+----------------------+
| Value | Meaning              |
+=======+======================+
| 0     | Disabled             |
+-------+----------------------+
| 1     | TSYS01               |
+-------+----------------------+
| 2     | MCP9600              |
+-------+----------------------+
| 3     | MAX31865 2 or 4 wire |
+-------+----------------------+
| 4     | TSYS03               |
+-------+----------------------+
| 5     | Analog               |
+-------+----------------------+
| 6     | DroneCAN             |
+-------+----------------------+
| 7     | MLX90614             |
+-------+----------------------+
| 8     | SHT3x                |
+-------+----------------------+
| 9     | MAX31865 3 wire      |
+-------+----------------------+




.. _TEMP7_BUS:

TEMP7\_BUS: Temperature sensor bus
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor bus number\, typically used to select from multiple I2C buses


+--------+
| Range  |
+========+
| 0 to 3 |
+--------+




.. _TEMP7_ADDR:

TEMP7\_ADDR: Temperature sensor address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor address\, typically used for I2C address


+----------+
| Range    |
+==========+
| 0 to 127 |
+----------+




.. _TEMP7_SRC:

TEMP7\_SRC: Sensor Source
~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source is used to designate which device\'s temperature report will be replaced by this temperature sensor\'s data\. If 0 \(None\) then the data is only available via log\. In the future a new Motor temperature report will be created for returning data directly\.


+-------+---------------------------+
| Value | Meaning                   |
+=======+===========================+
| 0     | None                      |
+-------+---------------------------+
| 1     | ESC                       |
+-------+---------------------------+
| 2     | Motor                     |
+-------+---------------------------+
| 3     | Battery Index             |
+-------+---------------------------+
| 4     | Battery ID/SerialNumber   |
+-------+---------------------------+
| 5     | CAN based Pitot tube      |
+-------+---------------------------+
| 6     | DroneCAN-out on AP_Periph |
+-------+---------------------------+




.. _TEMP7_SRC_ID:

TEMP7\_SRC\_ID: Sensor Source Identification
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source Identification is used to replace a specific instance of a system component\'s temperature report with the temp sensor\'s\. Examples\: TEMP\_SRC \= 1 \(ESC\)\, TEMP\_SRC\_ID \= 1 will set the temp of ESC1\. TEMP\_SRC \= 3 \(BatteryIndex\)\,TEMP\_SRC\_ID \= 2 will set the temp of BATT2\. TEMP\_SRC \= 4 \(BatteryId\/SerialNum\)\,TEMP\_SRC\_ID\=42 will set the temp of all batteries that have param BATTn\_SERIAL \= 42\.


.. _TEMP7_PIN:

TEMP7\_PIN: Temperature sensor analog voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the analog input pin that should be used for temprature monitoring\. Values for some autopilots are given as examples\. Search wiki for \"Analog pins\"\.


+-------+--------------------------------------+
| Value | Meaning                              |
+=======+======================================+
| -1    | Disabled                             |
+-------+--------------------------------------+
| 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 |
+-------+--------------------------------------+
| 5     | Navigator                            |
+-------+--------------------------------------+
| 13    | Pixhawk2_PM2/CubeOrange_PM2          |
+-------+--------------------------------------+
| 14    | CubeOrange                           |
+-------+--------------------------------------+
| 16    | Durandal                             |
+-------+--------------------------------------+
| 100   | PX4-v1                               |
+-------+--------------------------------------+




.. _TEMP7_A0:

TEMP7\_A0: Temperature sensor analog 0th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a0 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP7_A1:

TEMP7\_A1: Temperature sensor analog 1st polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a1 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP7_A2:

TEMP7\_A2: Temperature sensor analog 2nd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a2 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP7_A3:

TEMP7\_A3: Temperature sensor analog 3rd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a3 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP7_A4:

TEMP7\_A4: Temperature sensor analog 4th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a4 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP7_A5:

TEMP7\_A5: Temperature sensor analog 5th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a5 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP7_MSG_ID:

TEMP7\_MSG\_ID: Temperature sensor DroneCAN message ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the message device ID this backend listens for


+------------+
| Range      |
+============+
| 0 to 65535 |
+------------+




.. _TEMP7_RTD_NOM:

TEMP7\_RTD\_NOM: Nominal RTD resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Nominal RTD resistance used to calculate temperature\, typically 100 or 1000 ohms\.


.. _TEMP7_RTD_REF:

TEMP7\_RTD\_REF: RTD reference resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reference resistance used to calculate temperature\, in ohms



.. _parameters_TEMP8_:

TEMP8\_ Parameters
------------------


.. _TEMP8_TYPE:

TEMP8\_TYPE: Temperature Sensor Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Enables temperature sensors


+-------+----------------------+
| Value | Meaning              |
+=======+======================+
| 0     | Disabled             |
+-------+----------------------+
| 1     | TSYS01               |
+-------+----------------------+
| 2     | MCP9600              |
+-------+----------------------+
| 3     | MAX31865 2 or 4 wire |
+-------+----------------------+
| 4     | TSYS03               |
+-------+----------------------+
| 5     | Analog               |
+-------+----------------------+
| 6     | DroneCAN             |
+-------+----------------------+
| 7     | MLX90614             |
+-------+----------------------+
| 8     | SHT3x                |
+-------+----------------------+
| 9     | MAX31865 3 wire      |
+-------+----------------------+




.. _TEMP8_BUS:

TEMP8\_BUS: Temperature sensor bus
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor bus number\, typically used to select from multiple I2C buses


+--------+
| Range  |
+========+
| 0 to 3 |
+--------+




.. _TEMP8_ADDR:

TEMP8\_ADDR: Temperature sensor address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor address\, typically used for I2C address


+----------+
| Range    |
+==========+
| 0 to 127 |
+----------+




.. _TEMP8_SRC:

TEMP8\_SRC: Sensor Source
~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source is used to designate which device\'s temperature report will be replaced by this temperature sensor\'s data\. If 0 \(None\) then the data is only available via log\. In the future a new Motor temperature report will be created for returning data directly\.


+-------+---------------------------+
| Value | Meaning                   |
+=======+===========================+
| 0     | None                      |
+-------+---------------------------+
| 1     | ESC                       |
+-------+---------------------------+
| 2     | Motor                     |
+-------+---------------------------+
| 3     | Battery Index             |
+-------+---------------------------+
| 4     | Battery ID/SerialNumber   |
+-------+---------------------------+
| 5     | CAN based Pitot tube      |
+-------+---------------------------+
| 6     | DroneCAN-out on AP_Periph |
+-------+---------------------------+




.. _TEMP8_SRC_ID:

TEMP8\_SRC\_ID: Sensor Source Identification
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source Identification is used to replace a specific instance of a system component\'s temperature report with the temp sensor\'s\. Examples\: TEMP\_SRC \= 1 \(ESC\)\, TEMP\_SRC\_ID \= 1 will set the temp of ESC1\. TEMP\_SRC \= 3 \(BatteryIndex\)\,TEMP\_SRC\_ID \= 2 will set the temp of BATT2\. TEMP\_SRC \= 4 \(BatteryId\/SerialNum\)\,TEMP\_SRC\_ID\=42 will set the temp of all batteries that have param BATTn\_SERIAL \= 42\.


.. _TEMP8_PIN:

TEMP8\_PIN: Temperature sensor analog voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the analog input pin that should be used for temprature monitoring\. Values for some autopilots are given as examples\. Search wiki for \"Analog pins\"\.


+-------+--------------------------------------+
| Value | Meaning                              |
+=======+======================================+
| -1    | Disabled                             |
+-------+--------------------------------------+
| 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 |
+-------+--------------------------------------+
| 5     | Navigator                            |
+-------+--------------------------------------+
| 13    | Pixhawk2_PM2/CubeOrange_PM2          |
+-------+--------------------------------------+
| 14    | CubeOrange                           |
+-------+--------------------------------------+
| 16    | Durandal                             |
+-------+--------------------------------------+
| 100   | PX4-v1                               |
+-------+--------------------------------------+




.. _TEMP8_A0:

TEMP8\_A0: Temperature sensor analog 0th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a0 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP8_A1:

TEMP8\_A1: Temperature sensor analog 1st polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a1 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP8_A2:

TEMP8\_A2: Temperature sensor analog 2nd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a2 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP8_A3:

TEMP8\_A3: Temperature sensor analog 3rd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a3 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP8_A4:

TEMP8\_A4: Temperature sensor analog 4th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a4 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP8_A5:

TEMP8\_A5: Temperature sensor analog 5th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a5 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP8_MSG_ID:

TEMP8\_MSG\_ID: Temperature sensor DroneCAN message ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the message device ID this backend listens for


+------------+
| Range      |
+============+
| 0 to 65535 |
+------------+




.. _TEMP8_RTD_NOM:

TEMP8\_RTD\_NOM: Nominal RTD resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Nominal RTD resistance used to calculate temperature\, typically 100 or 1000 ohms\.


.. _TEMP8_RTD_REF:

TEMP8\_RTD\_REF: RTD reference resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reference resistance used to calculate temperature\, in ohms



.. _parameters_TEMP9_:

TEMP9\_ Parameters
------------------


.. _TEMP9_TYPE:

TEMP9\_TYPE: Temperature Sensor Type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: Reboot required after change*

Enables temperature sensors


+-------+----------------------+
| Value | Meaning              |
+=======+======================+
| 0     | Disabled             |
+-------+----------------------+
| 1     | TSYS01               |
+-------+----------------------+
| 2     | MCP9600              |
+-------+----------------------+
| 3     | MAX31865 2 or 4 wire |
+-------+----------------------+
| 4     | TSYS03               |
+-------+----------------------+
| 5     | Analog               |
+-------+----------------------+
| 6     | DroneCAN             |
+-------+----------------------+
| 7     | MLX90614             |
+-------+----------------------+
| 8     | SHT3x                |
+-------+----------------------+
| 9     | MAX31865 3 wire      |
+-------+----------------------+




.. _TEMP9_BUS:

TEMP9\_BUS: Temperature sensor bus
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor bus number\, typically used to select from multiple I2C buses


+--------+
| Range  |
+========+
| 0 to 3 |
+--------+




.. _TEMP9_ADDR:

TEMP9\_ADDR: Temperature sensor address
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Temperature sensor address\, typically used for I2C address


+----------+
| Range    |
+==========+
| 0 to 127 |
+----------+




.. _TEMP9_SRC:

TEMP9\_SRC: Sensor Source
~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source is used to designate which device\'s temperature report will be replaced by this temperature sensor\'s data\. If 0 \(None\) then the data is only available via log\. In the future a new Motor temperature report will be created for returning data directly\.


+-------+---------------------------+
| Value | Meaning                   |
+=======+===========================+
| 0     | None                      |
+-------+---------------------------+
| 1     | ESC                       |
+-------+---------------------------+
| 2     | Motor                     |
+-------+---------------------------+
| 3     | Battery Index             |
+-------+---------------------------+
| 4     | Battery ID/SerialNumber   |
+-------+---------------------------+
| 5     | CAN based Pitot tube      |
+-------+---------------------------+
| 6     | DroneCAN-out on AP_Periph |
+-------+---------------------------+




.. _TEMP9_SRC_ID:

TEMP9\_SRC\_ID: Sensor Source Identification
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sensor Source Identification is used to replace a specific instance of a system component\'s temperature report with the temp sensor\'s\. Examples\: TEMP\_SRC \= 1 \(ESC\)\, TEMP\_SRC\_ID \= 1 will set the temp of ESC1\. TEMP\_SRC \= 3 \(BatteryIndex\)\,TEMP\_SRC\_ID \= 2 will set the temp of BATT2\. TEMP\_SRC \= 4 \(BatteryId\/SerialNum\)\,TEMP\_SRC\_ID\=42 will set the temp of all batteries that have param BATTn\_SERIAL \= 42\.


.. _TEMP9_PIN:

TEMP9\_PIN: Temperature sensor analog voltage sensing pin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the analog input pin that should be used for temprature monitoring\. Values for some autopilots are given as examples\. Search wiki for \"Analog pins\"\.


+-------+--------------------------------------+
| Value | Meaning                              |
+=======+======================================+
| -1    | Disabled                             |
+-------+--------------------------------------+
| 2     | Pixhawk/Pixracer/Navio2/Pixhawk2_PM1 |
+-------+--------------------------------------+
| 5     | Navigator                            |
+-------+--------------------------------------+
| 13    | Pixhawk2_PM2/CubeOrange_PM2          |
+-------+--------------------------------------+
| 14    | CubeOrange                           |
+-------+--------------------------------------+
| 16    | Durandal                             |
+-------+--------------------------------------+
| 100   | PX4-v1                               |
+-------+--------------------------------------+




.. _TEMP9_A0:

TEMP9\_A0: Temperature sensor analog 0th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a0 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP9_A1:

TEMP9\_A1: Temperature sensor analog 1st polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a1 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP9_A2:

TEMP9\_A2: Temperature sensor analog 2nd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a2 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP9_A3:

TEMP9\_A3: Temperature sensor analog 3rd polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a3 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP9_A4:

TEMP9\_A4: Temperature sensor analog 4th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a4 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP9_A5:

TEMP9\_A5: Temperature sensor analog 5th polynomial coefficient
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


a5 in polynomial of form temperature in deg \= a0 \+ a1\*voltage \+ a2\*voltage\^2 \+ a3\*voltage\^3 \+ a4\*voltage\^4 \+ a5\*voltage\^5


.. _TEMP9_MSG_ID:

TEMP9\_MSG\_ID: Temperature sensor DroneCAN message ID
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Sets the message device ID this backend listens for


+------------+
| Range      |
+============+
| 0 to 65535 |
+------------+




.. _TEMP9_RTD_NOM:

TEMP9\_RTD\_NOM: Nominal RTD resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Nominal RTD resistance used to calculate temperature\, typically 100 or 1000 ohms\.


.. _TEMP9_RTD_REF:

TEMP9\_RTD\_REF: RTD reference resistance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Reference resistance used to calculate temperature\, in ohms



.. _parameters_VISO:

VISO Parameters
---------------


.. _VISO_TYPE:

VISO\_TYPE: Visual odometry camera connection type
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*
| *Note: Reboot required after change*

Visual odometry camera connection type


+-------+---------------+
| Value | Meaning       |
+=======+===============+
| 0     | None          |
+-------+---------------+
| 1     | MAVLink       |
+-------+---------------+
| 2     | IntelT265     |
+-------+---------------+
| 3     | VOXL(ModalAI) |
+-------+---------------+




.. _VISO_POS_X:

VISO\_POS\_X: Visual odometry camera X position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

X position of the camera in body frame\. Positive X is forward of the origin\.


+-----------+---------+--------+
| Increment | Range   | Units  |
+===========+=========+========+
| 0.01      | -5 to 5 | meters |
+-----------+---------+--------+




.. _VISO_POS_Y:

VISO\_POS\_Y: Visual odometry camera Y position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Y position of the camera in body frame\. Positive Y is to the right of the origin\.


+-----------+---------+--------+
| Increment | Range   | Units  |
+===========+=========+========+
| 0.01      | -5 to 5 | meters |
+-----------+---------+--------+




.. _VISO_POS_Z:

VISO\_POS\_Z: Visual odometry camera Z position offset
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Z position of the camera in body frame\. Positive Z is down from the origin\.


+-----------+---------+--------+
| Increment | Range   | Units  |
+===========+=========+========+
| 0.01      | -5 to 5 | meters |
+-----------+---------+--------+




.. _VISO_ORIENT:

VISO\_ORIENT: Visual odometery camera orientation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Visual odometery camera orientation


+-------+---------+
| Value | Meaning |
+=======+=========+
| 0     | Forward |
+-------+---------+
| 2     | Right   |
+-------+---------+
| 4     | Back    |
+-------+---------+
| 6     | Left    |
+-------+---------+
| 24    | Up      |
+-------+---------+
| 25    | Down    |
+-------+---------+




.. _VISO_SCALE:

VISO\_SCALE: Visual odometry scaling factor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Visual odometry scaling factor applied to position estimates from sensor


.. _VISO_DELAY_MS:

VISO\_DELAY\_MS: Visual odometry sensor delay
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Visual odometry sensor delay relative to inertial measurements


+----------+--------------+
| Range    | Units        |
+==========+==============+
| 0 to 250 | milliseconds |
+----------+--------------+




.. _VISO_VEL_M_NSE:

VISO\_VEL\_M\_NSE: Visual odometry velocity measurement noise
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Visual odometry velocity measurement noise in m\/s


+-------------+-------------------+
| Range       | Units             |
+=============+===================+
| 0.05 to 5.0 | meters per second |
+-------------+-------------------+




.. _VISO_POS_M_NSE:

VISO\_POS\_M\_NSE: Visual odometry position measurement noise
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Visual odometry position measurement noise minimum \(meters\)\. This value will be used if the sensor provides a lower noise value \(or no noise value\)


+-------------+--------+
| Range       | Units  |
+=============+========+
| 0.1 to 10.0 | meters |
+-------------+--------+




.. _VISO_YAW_M_NSE:

VISO\_YAW\_M\_NSE: Visual odometry yaw measurement noise
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Visual odometry yaw measurement noise minimum \(radians\)\, This value will be used if the sensor provides a lower noise value \(or no noise value\)


+-------------+---------+
| Range       | Units   |
+=============+=========+
| 0.05 to 1.0 | radians |
+-------------+---------+




.. _VISO_QUAL_MIN:

VISO\_QUAL\_MIN: Visual odometry minimum quality
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Visual odometry will only be sent to EKF if over this value\. \-1 to always send \(even bad values\)\, 0 to send if good or unknown


+-----------+---------+
| Range     | Units   |
+===========+=========+
| -1 to 100 | percent |
+-----------+---------+





.. _parameters_VTX_:

VTX\_ Parameters
----------------


.. _VTX_ENABLE:

VTX\_ENABLE: Is the Video Transmitter enabled or not
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Toggles the Video Transmitter on and off


+-------+---------+
| Value | Meaning |
+=======+=========+
| 0     | Disable |
+-------+---------+
| 1     | Enable  |
+-------+---------+




.. _VTX_POWER:

VTX\_POWER: Video Transmitter Power Level
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video Transmitter Power Level\. Different VTXs support different power levels\, the power level chosen will be rounded down to the nearest supported power level


+-----------+
| Range     |
+===========+
| 1 to 1000 |
+-----------+




.. _VTX_CHANNEL:

VTX\_CHANNEL: Video Transmitter Channel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video Transmitter Channel


+--------+
| Range  |
+========+
| 0 to 7 |
+--------+




.. _VTX_BAND:

VTX\_BAND: Video Transmitter Band
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video Transmitter Band


+-------+--------------+
| Value | Meaning      |
+=======+==============+
| 0     | Band A       |
+-------+--------------+
| 1     | Band B       |
+-------+--------------+
| 2     | Band E       |
+-------+--------------+
| 3     | Airwave      |
+-------+--------------+
| 4     | RaceBand     |
+-------+--------------+
| 5     | Low RaceBand |
+-------+--------------+
| 6     | 1G3 Band A   |
+-------+--------------+
| 7     | 1G3 Band B   |
+-------+--------------+
| 8     | Band X       |
+-------+--------------+
| 9     | 3G3 Band A   |
+-------+--------------+
| 10    | 3G3 Band B   |
+-------+--------------+




.. _VTX_FREQ:

VTX\_FREQ: Video Transmitter Frequency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video Transmitter Frequency\. The frequency is derived from the setting of BAND and CHANNEL


+--------------+----------+
| Range        | ReadOnly |
+==============+==========+
| 1000 to 6000 | True     |
+--------------+----------+




.. _VTX_OPTIONS:

VTX\_OPTIONS: Video Transmitter Options
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

| *Note: This parameter is for advanced users*

Video Transmitter Options\. Pitmode puts the VTX in a low power state\. Unlocked enables certain restricted frequencies and power levels\. Do not enable the Unlocked option unless you have appropriate permissions in your jurisdiction to transmit at high power levels\. One stop\-bit may be required for VTXs that erroneously mimic iNav behaviour\.


+-----+-----------------------------------------------------------+
| Bit | Meaning                                                   |
+=====+===========================================================+
| 0   | Pitmode                                                   |
+-----+-----------------------------------------------------------+
| 1   | Pitmode until armed                                       |
+-----+-----------------------------------------------------------+
| 2   | Pitmode when disarmed                                     |
+-----+-----------------------------------------------------------+
| 3   | Unlocked                                                  |
+-----+-----------------------------------------------------------+
| 4   | Add leading zero byte to requests                         |
+-----+-----------------------------------------------------------+
| 5   | Use 1 stop-bit in SmartAudio                              |
+-----+-----------------------------------------------------------+
| 6   | Ignore CRC in SmartAudio                                  |
+-----+-----------------------------------------------------------+
| 7   | Ignore status updates in CRSF and blindly set VTX options |
+-----+-----------------------------------------------------------+




.. _VTX_MAX_POWER:

VTX\_MAX\_POWER: Video Transmitter Max Power Level
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Video Transmitter Maximum Power Level\. Different VTXs support different power levels\, this prevents the power aux switch from requesting too high a power level\. The switch supports 6 power levels and the selected power will be a subdivision between 0 and this setting\.


+------------+
| Range      |
+============+
| 25 to 1000 |
+------------+



