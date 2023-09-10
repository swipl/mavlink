/*  File:    mavlink.pl
    Author:  Roy Ratcliffe
    Created: Aug 20 2023
    Purpose: MAVLink
*/

:- module(mavlink,
          []).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

mavlink_definitions_r(all, A),
    forall((   member(B-C, A),
               mavlink_definitions:mavlink_definition(C, D)),
           assertz(mavlink:D)).

Predicate ordering matters.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

:- dynamic enum/2.

enum('ACCELCAL_VEHICLE_POS', []).
enum('HEADING_TYPE', []).
enum('SPEED_TYPE', []).
enum('MAV_CMD', []).
enum('SCRIPTING_CMD', []).
enum('LIMITS_STATE', []).
enum('LIMIT_MODULE', [bitmask=true]).
enum('RALLY_FLAGS', [bitmask=true]).
enum('CAMERA_STATUS_TYPES', []).
enum('CAMERA_FEEDBACK_FLAGS', []).
enum('MAV_MODE_GIMBAL', []).
enum('GIMBAL_AXIS', []).
enum('GIMBAL_AXIS_CALIBRATION_STATUS', []).
enum('GIMBAL_AXIS_CALIBRATION_REQUIRED', []).
enum('GOPRO_HEARTBEAT_STATUS', []).
enum('GOPRO_HEARTBEAT_FLAGS', [bitmask=true]).
enum('GOPRO_REQUEST_STATUS', []).
enum('GOPRO_COMMAND', []).
enum('GOPRO_CAPTURE_MODE', []).
enum('GOPRO_RESOLUTION', []).
enum('GOPRO_FRAME_RATE', []).
enum('GOPRO_FIELD_OF_VIEW', []).
enum('GOPRO_VIDEO_SETTINGS_FLAGS', [bitmask=true]).
enum('GOPRO_PHOTO_RESOLUTION', []).
enum('GOPRO_PROTUNE_WHITE_BALANCE', []).
enum('GOPRO_PROTUNE_COLOUR', []).
enum('GOPRO_PROTUNE_GAIN', []).
enum('GOPRO_PROTUNE_SHARPNESS', []).
enum('GOPRO_PROTUNE_EXPOSURE', []).
enum('GOPRO_CHARGING', []).
enum('GOPRO_MODEL', []).
enum('GOPRO_BURST_RATE', []).
enum('MAV_CMD_DO_AUX_FUNCTION_SWITCH_LEVEL', []).
enum('LED_CONTROL_PATTERN', []).
enum('EKF_STATUS_FLAGS', [bitmask=true]).
enum('PID_TUNING_AXIS', []).
enum('MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS', []).
enum('MAV_REMOTE_LOG_DATA_BLOCK_STATUSES', []).
enum('DEVICE_OP_BUSTYPE', []).
enum('DEEPSTALL_STAGE', []).
enum('PLANE_MODE', []).
enum('COPTER_MODE', []).
enum('SUB_MODE', []).
enum('ROVER_MODE', []).
enum('TRACKER_MODE', []).
enum('OSD_PARAM_CONFIG_TYPE', []).
enum('OSD_PARAM_CONFIG_ERROR', []).
enum('MAV_CMD', []).
enum('GSM_LINK_TYPE', []).
enum('GSM_MODEM_TYPE', []).
enum('FIRMWARE_VERSION_TYPE', []).
enum('HL_FAILURE_FLAG', [bitmask=true]).
enum('MAV_GOTO', []).
enum('MAV_MODE', []).
enum('MAV_SYS_STATUS_SENSOR', [bitmask=true]).
enum('MAV_SYS_STATUS_SENSOR_EXTENDED', [bitmask=true]).
enum('MAV_FRAME', []).
enum('MAVLINK_DATA_STREAM_TYPE', []).
enum('FENCE_ACTION', []).
enum('FENCE_BREACH', []).
enum('FENCE_MITIGATE', []).
enum('MAV_MOUNT_MODE', []).
enum('GIMBAL_DEVICE_CAP_FLAGS', [bitmask=true]).
enum('GIMBAL_MANAGER_CAP_FLAGS', [bitmask=true]).
enum('GIMBAL_DEVICE_FLAGS', [bitmask=true]).
enum('GIMBAL_MANAGER_FLAGS', [bitmask=true]).
enum('GIMBAL_DEVICE_ERROR_FLAGS', [bitmask=true]).
enum('GRIPPER_ACTIONS', []).
enum('WINCH_ACTIONS', []).
enum('UAVCAN_NODE_HEALTH', []).
enum('UAVCAN_NODE_MODE', []).
enum('ESC_CONNECTION_TYPE', []).
enum('ESC_FAILURE_FLAGS', [bitmask=true]).
enum('STORAGE_STATUS', []).
enum('STORAGE_TYPE', []).
enum('STORAGE_USAGE_FLAG', []).
enum('ORBIT_YAW_BEHAVIOUR', []).
enum('WIFI_CONFIG_AP_RESPONSE', []).
enum('CELLULAR_CONFIG_RESPONSE', []).
enum('WIFI_CONFIG_AP_MODE', []).
enum('COMP_METADATA_TYPE', []).
enum('ACTUATOR_CONFIGURATION', []).
enum('ACTUATOR_OUTPUT_FUNCTION', []).
enum('AUTOTUNE_AXIS', [bitmask=true]).
enum('PREFLIGHT_STORAGE_PARAMETER_ACTION', []).
enum('PREFLIGHT_STORAGE_MISSION_ACTION', []).
enum('MAV_CMD', []).
enum('MAV_DATA_STREAM', []).
enum('MAV_ROI', []).
enum('MAV_PARAM_TYPE', []).
enum('MAV_PARAM_EXT_TYPE', []).
enum('MAV_RESULT', []).
enum('MAV_MISSION_RESULT', []).
enum('MAV_SEVERITY', []).
enum('MAV_POWER_STATUS', [bitmask=true]).
enum('SERIAL_CONTROL_DEV', []).
enum('SERIAL_CONTROL_FLAG', [bitmask=true]).
enum('MAV_DISTANCE_SENSOR', []).
enum('MAV_SENSOR_ORIENTATION', []).
enum('MAV_PROTOCOL_CAPABILITY', [bitmask=true]).
enum('MAV_MISSION_TYPE', []).
enum('MAV_ESTIMATOR_TYPE', []).
enum('MAV_BATTERY_TYPE', []).
enum('MAV_BATTERY_FUNCTION', []).
enum('MAV_BATTERY_CHARGE_STATE', []).
enum('MAV_BATTERY_MODE', []).
enum('MAV_BATTERY_FAULT', [bitmask=true]).
enum('MAV_GENERATOR_STATUS_FLAG', [bitmask=true]).
enum('MAV_VTOL_STATE', []).
enum('MAV_LANDED_STATE', []).
enum('ADSB_ALTITUDE_TYPE', []).
enum('ADSB_EMITTER_TYPE', []).
enum('ADSB_FLAGS', [bitmask=true]).
enum('MAV_DO_REPOSITION_FLAGS', [bitmask=true]).
enum('ESTIMATOR_STATUS_FLAGS', [bitmask=true]).
enum('MOTOR_TEST_ORDER', []).
enum('MOTOR_TEST_THROTTLE_TYPE', []).
enum('GPS_INPUT_IGNORE_FLAGS', [bitmask=true]).
enum('MAV_COLLISION_ACTION', []).
enum('MAV_COLLISION_THREAT_LEVEL', []).
enum('MAV_COLLISION_SRC', []).
enum('GPS_FIX_TYPE', []).
enum('RTK_BASELINE_COORDINATE_SYSTEM', []).
enum('LANDING_TARGET_TYPE', []).
enum('VTOL_TRANSITION_HEADING', []).
enum('CAMERA_CAP_FLAGS', [bitmask=true]).
enum('VIDEO_STREAM_STATUS_FLAGS', [bitmask=true]).
enum('VIDEO_STREAM_TYPE', []).
enum('CAMERA_TRACKING_STATUS_FLAGS', []).
enum('CAMERA_TRACKING_MODE', []).
enum('CAMERA_TRACKING_TARGET_DATA', [bitmask=true]).
enum('CAMERA_ZOOM_TYPE', []).
enum('SET_FOCUS_TYPE', []).
enum('PARAM_ACK', []).
enum('CAMERA_MODE', []).
enum('MAV_ARM_AUTH_DENIED_REASON', []).
enum('RC_TYPE', []).
enum('POSITION_TARGET_TYPEMASK', [bitmask=true]).
enum('ATTITUDE_TARGET_TYPEMASK', [bitmask=true]).
enum('UTM_FLIGHT_STATE', []).
enum('UTM_DATA_AVAIL_FLAGS', [bitmask=true]).
enum('CELLULAR_STATUS_FLAG', []).
enum('CELLULAR_NETWORK_FAILED_REASON', []).
enum('CELLULAR_NETWORK_RADIO_TYPE', []).
enum('PRECISION_LAND_MODE', []).
enum('PARACHUTE_ACTION', []).
enum('MAV_TUNNEL_PAYLOAD_TYPE', []).
enum('MAV_ODID_ID_TYPE', []).
enum('MAV_ODID_UA_TYPE', []).
enum('MAV_ODID_STATUS', []).
enum('MAV_ODID_HEIGHT_REF', []).
enum('MAV_ODID_HOR_ACC', []).
enum('MAV_ODID_VER_ACC', []).
enum('MAV_ODID_SPEED_ACC', []).
enum('MAV_ODID_TIME_ACC', []).
enum('MAV_ODID_AUTH_TYPE', []).
enum('MAV_ODID_DESC_TYPE', []).
enum('MAV_ODID_OPERATOR_LOCATION_TYPE', []).
enum('MAV_ODID_CLASSIFICATION_TYPE', []).
enum('MAV_ODID_CATEGORY_EU', []).
enum('MAV_ODID_CLASS_EU', []).
enum('MAV_ODID_OPERATOR_ID_TYPE', []).
enum('MAV_ODID_ARM_STATUS', []).
enum('TUNE_FORMAT', []).
enum('AIS_TYPE', []).
enum('AIS_NAV_STATUS', []).
enum('AIS_FLAGS', [bitmask=true]).
enum('FAILURE_UNIT', []).
enum('FAILURE_TYPE', []).
enum('NAV_VTOL_LAND_OPTIONS', []).
enum('MAV_WINCH_STATUS_FLAG', [bitmask=true]).
enum('MAG_CAL_STATUS', []).
enum('MAV_EVENT_ERROR_REASON', []).
enum('MAV_EVENT_CURRENT_SEQUENCE_FLAGS', []).
enum('HIL_SENSOR_UPDATED_FLAGS', [bitmask=true]).
enum('HIGHRES_IMU_UPDATED_FLAGS', [bitmask=true]).
enum('CAN_FILTER_OP', []).
enum('MAV_FTP_ERR', []).
enum('MAV_FTP_OPCODE', []).
enum('MISSION_STATE', []).
enum('WIFI_NETWORK_SECURITY', []).
enum('AIRSPEED_SENSOR_FLAGS', [bitmask=true]).
enum('PARAM_TRANSACTION_TRANSPORT', []).
enum('PARAM_TRANSACTION_ACTION', []).
enum('MAV_STANDARD_MODE', []).
enum('MAV_MODE_PROPERTY', [bitmask=true]).
enum('MAV_CMD', []).
enum('MAV_BATTERY_STATUS_FLAGS', [bitmask=true]).
enum('MAV_CMD', []).
enum('TARGET_ABSOLUTE_SENSOR_CAPABILITY_FLAGS', [bitmask=true]).
enum('TARGET_OBS_FRAME', []).
enum('ICAROUS_TRACK_BAND_TYPES', []).
enum('ICAROUS_FMS_STATE', []).
enum('MAV_AUTOPILOT', []).
enum('MAV_TYPE', []).
enum('MAV_MODE_FLAG', [bitmask=true]).
enum('MAV_MODE_FLAG_DECODE_POSITION', [bitmask=true]).
enum('MAV_STATE', []).
enum('MAV_COMPONENT', []).
enum('UALBERTA_AUTOPILOT_MODE', []).
enum('UALBERTA_NAV_MODE', []).
enum('UALBERTA_PILOT_MODE', []).
enum('UAVIONIX_ADSB_OUT_DYNAMIC_STATE', [bitmask=true]).
enum('UAVIONIX_ADSB_OUT_RF_SELECT', [bitmask=true]).
enum('UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX', []).
enum('UAVIONIX_ADSB_RF_HEALTH', [bitmask=true]).
enum('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', []).
enum('UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT', []).
enum('UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON', []).
enum('UAVIONIX_ADSB_EMERGENCY_STATUS', []).
enum('MAV_STORM32_TUNNEL_PAYLOAD_TYPE', []).
enum('MAV_STORM32_GIMBAL_PREARM_FLAGS', [bitmask=true]).
enum('MAV_STORM32_CAMERA_PREARM_FLAGS', [bitmask=true]).
enum('MAV_STORM32_GIMBAL_MANAGER_CAP_FLAGS', [bitmask=true]).
enum('MAV_STORM32_GIMBAL_MANAGER_FLAGS', [bitmask=true]).
enum('MAV_STORM32_GIMBAL_MANAGER_CLIENT', []).
enum('MAV_STORM32_GIMBAL_MANAGER_PROFILE', []).
enum('MAV_QSHOT_MODE', []).
enum('MAV_CMD', []).
enum('RADIO_RC_CHANNELS_FLAGS', [bitmask=true]).
enum('RADIO_LINK_STATS_FLAGS', [bitmask=true]).
enum('MAV_CMD', []).
enum('MAV_AVSS_COMMAND_FAILURE_REASON', []).
enum('AVSS_M300_OPERATION_MODE', []).
enum('AVSS_HORSEFLY_OPERATION_MODE', []).
enum('AIRLINK_AUTH_RESPONSE_TYPE', []).

:- dynamic enum_deprecated/2.

enum_deprecated('MAV_MOUNT_MODE', [since='2020-01', replaced_by='GIMBAL_MANAGER_FLAGS']).
enum_deprecated('MAV_DATA_STREAM', [since='2015-06', replaced_by='MESSAGE_INTERVAL']).
enum_deprecated('MAV_ROI', [since='2018-01', replaced_by='MAV_CMD_DO_SET_ROI_*']).

:- dynamic enum_description/2.

enum_description('RALLY_FLAGS', 'Flags in RALLY_POINT message.').
enum_description('EKF_STATUS_FLAGS', 'Flags in EKF_STATUS message.').
enum_description('MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS', 'Special ACK block numbers control activation of dataflash log streaming.').
enum_description('MAV_REMOTE_LOG_DATA_BLOCK_STATUSES', 'Possible remote log data block statuses.').
enum_description('DEVICE_OP_BUSTYPE', 'Bus types for device operations.').
enum_description('DEEPSTALL_STAGE', 'Deepstall flight stage.').
enum_description('PLANE_MODE', 'A mapping of plane flight modes for custom_mode field of heartbeat.').
enum_description('COPTER_MODE', 'A mapping of copter flight modes for custom_mode field of heartbeat.').
enum_description('SUB_MODE', 'A mapping of sub flight modes for custom_mode field of heartbeat.').
enum_description('ROVER_MODE', 'A mapping of rover flight modes for custom_mode field of heartbeat.').
enum_description('TRACKER_MODE', 'A mapping of antenna tracker flight modes for custom_mode field of heartbeat.').
enum_description('OSD_PARAM_CONFIG_TYPE', 'The type of parameter for the OSD parameter editor.').
enum_description('OSD_PARAM_CONFIG_ERROR', 'The error type for the OSD parameter editor.').
enum_description('FIRMWARE_VERSION_TYPE', 'These values define the type of firmware release.  These values indicate the first version or release of this type.  For example the first alpha release would be 64, the second would be 65.').
enum_description('HL_FAILURE_FLAG', 'Flags to report failure cases over the high latency telemtry.').
enum_description('MAV_GOTO', 'Actions that may be specified in MAV_CMD_OVERRIDE_GOTO to override mission execution.').
enum_description('MAV_MODE', 'These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it\n               simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.').
enum_description('MAV_SYS_STATUS_SENSOR', 'These encode the sensors whose status is sent as part of the SYS_STATUS message.').
enum_description('MAV_SYS_STATUS_SENSOR_EXTENDED', 'These encode the sensors whose status is sent as part of the SYS_STATUS message in the extended fields.').
enum_description('MAV_FRAME', 'Coordinate frames used by MAVLink. Not all frames are supported by all commands, messages, or vehicles.\n\n      Global frames use the following naming conventions:\n      - "GLOBAL": Global coordinate frame with WGS84 latitude/longitude and altitude positive over mean sea level (MSL) by default.\n        The following modifiers may be used with "GLOBAL":\n        - "RELATIVE_ALT": Altitude is relative to the vehicle home position rather than MSL.\n        - "TERRAIN_ALT": Altitude is relative to ground level rather than MSL.\n        - "INT": Latitude/longitude (in degrees) are scaled by multiplying by 1E7.\n\n      Local frames use the following naming conventions:\n      - "LOCAL": Origin of local frame is fixed relative to earth. Unless otherwise specified this origin is the origin of the vehicle position-estimator ("EKF").\n      - "BODY": Origin of local frame travels with the vehicle. NOTE, "BODY" does NOT indicate alignment of frame axis with vehicle attitude.\n      - "OFFSET": Deprecated synonym for "BODY" (origin travels with the vehicle). Not to be used for new frames.\n\n      Some deprecated frames do not follow these conventions (e.g. MAV_FRAME_BODY_NED and MAV_FRAME_BODY_OFFSET_NED).\n ').
enum_description('FENCE_ACTION', 'Actions following geofence breach.').
enum_description('FENCE_MITIGATE', 'Actions being taken to mitigate/prevent fence breach').
enum_description('MAV_MOUNT_MODE', 'Enumeration of possible mount operation modes. This message is used by obsolete/deprecated gimbal messages.').
enum_description('GIMBAL_DEVICE_CAP_FLAGS', 'Gimbal device (low level) capability flags (bitmap).').
enum_description('GIMBAL_MANAGER_CAP_FLAGS', 'Gimbal manager high level capability flags (bitmap). The first 16 bits are identical to the GIMBAL_DEVICE_CAP_FLAGS. However, the gimbal manager does not need to copy the flags from the gimbal but can also enhance the capabilities and thus add flags.').
enum_description('GIMBAL_DEVICE_FLAGS', 'Flags for gimbal device (lower level) operation.').
enum_description('GIMBAL_MANAGER_FLAGS', 'Flags for high level gimbal manager operation The first 16 bits are identical to the GIMBAL_DEVICE_FLAGS.').
enum_description('GIMBAL_DEVICE_ERROR_FLAGS', 'Gimbal device (low level) error flags (bitmap, 0 means no error)').
enum_description('GRIPPER_ACTIONS', 'Gripper actions.').
enum_description('WINCH_ACTIONS', 'Winch actions.').
enum_description('UAVCAN_NODE_HEALTH', 'Generalized UAVCAN node health').
enum_description('UAVCAN_NODE_MODE', 'Generalized UAVCAN node mode').
enum_description('ESC_CONNECTION_TYPE', 'Indicates the ESC connection type.').
enum_description('ESC_FAILURE_FLAGS', 'Flags to report ESC failures.').
enum_description('STORAGE_STATUS', 'Flags to indicate the status of camera storage.').
enum_description('STORAGE_TYPE', 'Flags to indicate the type of storage.').
enum_description('STORAGE_USAGE_FLAG', 'Flags to indicate usage for a particular storage (see STORAGE_INFORMATION.storage_usage and MAV_CMD_SET_STORAGE_USAGE).').
enum_description('ORBIT_YAW_BEHAVIOUR', 'Yaw behaviour during orbit flight.').
enum_description('WIFI_CONFIG_AP_RESPONSE', 'Possible responses from a WIFI_CONFIG_AP message.').
enum_description('CELLULAR_CONFIG_RESPONSE', 'Possible responses from a CELLULAR_CONFIG message.').
enum_description('WIFI_CONFIG_AP_MODE', 'WiFi Mode.').
enum_description('COMP_METADATA_TYPE', 'Supported component metadata types. These are used in the "general" metadata file returned by COMPONENT_METADATA to provide information about supported metadata types. The types are not used directly in MAVLink messages.').
enum_description('ACTUATOR_CONFIGURATION', 'Actuator configuration, used to change a setting on an actuator. Component information metadata can be used to know which outputs support which commands.').
enum_description('ACTUATOR_OUTPUT_FUNCTION', 'Actuator output function. Values greater or equal to 1000 are autopilot-specific.').
enum_description('AUTOTUNE_AXIS', 'Enable axes that will be tuned via autotuning. Used in MAV_CMD_DO_AUTOTUNE_ENABLE.').
enum_description('PREFLIGHT_STORAGE_PARAMETER_ACTION', '\n        Actions for reading/writing parameters between persistent and volatile storage when using MAV_CMD_PREFLIGHT_STORAGE.\n        (Commonly parameters are loaded from persistent storage (flash/EEPROM) into volatile storage (RAM) on startup and written back when they are changed.)\n      ').
enum_description('PREFLIGHT_STORAGE_MISSION_ACTION', '\n        Actions for reading and writing plan information (mission, rally points, geofence) between persistent and volatile storage when using MAV_CMD_PREFLIGHT_STORAGE.\n        (Commonly missions are loaded from persistent storage (flash/EEPROM) into volatile storage (RAM) on startup and written back when they are changed.)\n      ').
enum_description('MAV_CMD', 'Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. NaN and INT32_MAX may be used in float/integer params (respectively) to indicate optional/default values (e.g. to use the component\'s current yaw or latitude rather than a specific value). See https://mavlink.io/en/guide/xml_schema.html#MAV_CMD for information about the structure of the MAV_CMD entries').
enum_description('MAV_DATA_STREAM', 'A data stream is not a fixed set of messages, but rather a\n     recommendation to the autopilot software. Individual autopilots may or may not obey\n     the recommended messages.').
enum_description('MAV_ROI', 'The ROI (region of interest) for the vehicle. This can be\n                be used by the vehicle for camera/vehicle attitude alignment (see\n                MAV_CMD_NAV_ROI).').
enum_description('MAV_PARAM_TYPE', 'Specifies the datatype of a MAVLink parameter.').
enum_description('MAV_PARAM_EXT_TYPE', 'Specifies the datatype of a MAVLink extended parameter.').
enum_description('MAV_RESULT', 'Result from a MAVLink command (MAV_CMD)').
enum_description('MAV_MISSION_RESULT', 'Result of mission operation (in a MISSION_ACK message).').
enum_description('MAV_SEVERITY', 'Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.').
enum_description('MAV_POWER_STATUS', 'Power supply status flags (bitmask)').
enum_description('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL device types').
enum_description('SERIAL_CONTROL_FLAG', 'SERIAL_CONTROL flags (bitmask)').
enum_description('MAV_DISTANCE_SENSOR', 'Enumeration of distance sensor types').
enum_description('MAV_SENSOR_ORIENTATION', 'Enumeration of sensor orientation, according to its rotations').
enum_description('MAV_PROTOCOL_CAPABILITY', 'Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability.').
enum_description('MAV_MISSION_TYPE', 'Type of mission items being requested/sent in mission protocol.').
enum_description('MAV_ESTIMATOR_TYPE', 'Enumeration of estimator types').
enum_description('MAV_BATTERY_TYPE', 'Enumeration of battery types').
enum_description('MAV_BATTERY_FUNCTION', 'Enumeration of battery functions').
enum_description('MAV_BATTERY_CHARGE_STATE', 'Enumeration for battery charge states.').
enum_description('MAV_BATTERY_MODE', 'Battery mode. Note, the normal operation mode (i.e. when flying) should be reported as MAV_BATTERY_MODE_UNKNOWN to allow message trimming in normal flight.').
enum_description('MAV_BATTERY_FAULT', 'Smart battery supply status/fault flags (bitmask) for health indication. The battery must also report either MAV_BATTERY_CHARGE_STATE_FAILED or MAV_BATTERY_CHARGE_STATE_UNHEALTHY if any of these are set.').
enum_description('MAV_GENERATOR_STATUS_FLAG', 'Flags to report status/failure cases for a power generator (used in GENERATOR_STATUS). Note that FAULTS are conditions that cause the generator to fail. Warnings are conditions that require attention before the next use (they indicate the system is not operating properly).').
enum_description('MAV_VTOL_STATE', 'Enumeration of VTOL states').
enum_description('MAV_LANDED_STATE', 'Enumeration of landed detector states').
enum_description('ADSB_ALTITUDE_TYPE', 'Enumeration of the ADSB altimeter types').
enum_description('ADSB_EMITTER_TYPE', 'ADSB classification for the type of vehicle emitting the transponder signal').
enum_description('ADSB_FLAGS', 'These flags indicate status such as data validity of each data source. Set = data valid').
enum_description('MAV_DO_REPOSITION_FLAGS', 'Bitmap of options for the MAV_CMD_DO_REPOSITION').
enum_description('ESTIMATOR_STATUS_FLAGS', 'Flags in ESTIMATOR_STATUS message').
enum_description('MOTOR_TEST_ORDER', 'Sequence that motors are tested when using MAV_CMD_DO_MOTOR_TEST.').
enum_description('MOTOR_TEST_THROTTLE_TYPE', 'Defines how throttle value is represented in MAV_CMD_DO_MOTOR_TEST.').
enum_description('MAV_COLLISION_ACTION', 'Possible actions an aircraft can take to avoid a collision.').
enum_description('MAV_COLLISION_THREAT_LEVEL', 'Aircraft-rated danger from this threat.').
enum_description('MAV_COLLISION_SRC', 'Source of information about this collision.').
enum_description('GPS_FIX_TYPE', 'Type of GPS fix').
enum_description('RTK_BASELINE_COORDINATE_SYSTEM', 'RTK GPS baseline coordinate system, used for RTK corrections').
enum_description('LANDING_TARGET_TYPE', 'Type of landing target').
enum_description('VTOL_TRANSITION_HEADING', 'Direction of VTOL transition').
enum_description('CAMERA_CAP_FLAGS', 'Camera capability flags (Bitmap)').
enum_description('VIDEO_STREAM_STATUS_FLAGS', 'Stream status flags (Bitmap)').
enum_description('VIDEO_STREAM_TYPE', 'Video stream types').
enum_description('CAMERA_TRACKING_STATUS_FLAGS', 'Camera tracking status flags').
enum_description('CAMERA_TRACKING_MODE', 'Camera tracking modes').
enum_description('CAMERA_TRACKING_TARGET_DATA', 'Camera tracking target data (shows where tracked target is within image)').
enum_description('CAMERA_ZOOM_TYPE', 'Zoom types for MAV_CMD_SET_CAMERA_ZOOM').
enum_description('SET_FOCUS_TYPE', 'Focus types for MAV_CMD_SET_CAMERA_FOCUS').
enum_description('PARAM_ACK', 'Result from PARAM_EXT_SET message (or a PARAM_SET within a transaction).').
enum_description('CAMERA_MODE', 'Camera Modes.').
enum_description('RC_TYPE', 'RC type').
enum_description('POSITION_TARGET_TYPEMASK', 'Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 9 is set the floats afx afy afz should be interpreted as force instead of acceleration.').
enum_description('ATTITUDE_TARGET_TYPEMASK', 'Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b00000000 indicates that none of the setpoint dimensions should be ignored.').
enum_description('UTM_FLIGHT_STATE', 'Airborne status of UAS.').
enum_description('UTM_DATA_AVAIL_FLAGS', 'Flags for the global position report.').
enum_description('CELLULAR_STATUS_FLAG', 'These flags encode the cellular network status').
enum_description('CELLULAR_NETWORK_FAILED_REASON', 'These flags are used to diagnose the failure state of CELLULAR_STATUS').
enum_description('CELLULAR_NETWORK_RADIO_TYPE', 'Cellular network radio type').
enum_description('PRECISION_LAND_MODE', 'Precision land modes (used in MAV_CMD_NAV_LAND).').
enum_description('PARACHUTE_ACTION', 'Parachute actions. Trigger release and enable/disable auto-release.').
enum_description('TUNE_FORMAT', 'Tune formats (used for vehicle buzzer/tone generation).').
enum_description('AIS_TYPE', 'Type of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html').
enum_description('AIS_NAV_STATUS', 'Navigational status of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html').
enum_description('AIS_FLAGS', 'These flags are used in the AIS_VESSEL.fields bitmask to indicate validity of data in the other message fields. When set, the data is valid.').
enum_description('FAILURE_UNIT', 'List of possible units where failures can be injected.').
enum_description('FAILURE_TYPE', 'List of possible failure type to inject.').
enum_description('MAV_WINCH_STATUS_FLAG', 'Winch status flags used in WINCH_STATUS').
enum_description('MAV_EVENT_ERROR_REASON', 'Reason for an event error response.').
enum_description('MAV_EVENT_CURRENT_SEQUENCE_FLAGS', 'Flags for CURRENT_EVENT_SEQUENCE.').
enum_description('HIL_SENSOR_UPDATED_FLAGS', 'Flags in the HIL_SENSOR message indicate which fields have updated since the last message').
enum_description('HIGHRES_IMU_UPDATED_FLAGS', 'Flags in the HIGHRES_IMU message indicate which fields have updated since the last message').
enum_description('MAV_FTP_ERR', 'MAV FTP error codes (https://mavlink.io/en/services/ftp.html)').
enum_description('MAV_FTP_OPCODE', 'MAV FTP opcodes: https://mavlink.io/en/services/ftp.html').
enum_description('MISSION_STATE', '\n        States of the mission state machine.\n        Note that these states are independent of whether the mission is in a mode that can execute mission items or not (is suspended).\n        They may not all be relevant on all vehicles.\n      ').
enum_description('WIFI_NETWORK_SECURITY', 'WiFi wireless security protocols.').
enum_description('AIRSPEED_SENSOR_FLAGS', 'Airspeed sensor flags').
enum_description('PARAM_TRANSACTION_TRANSPORT', 'Possible transport layers to set and get parameters via mavlink during a parameter transaction.').
enum_description('PARAM_TRANSACTION_ACTION', 'Possible parameter transaction actions.').
enum_description('MAV_STANDARD_MODE', 'Standard modes with a well understood meaning across flight stacks and vehicle types.\n        For example, most flight stack have the concept of a "return" or "RTL" mode that takes a vehicle to safety, even though the precise mechanics of this mode may differ.\n        Modes may be set using MAV_CMD_DO_SET_STANDARD_MODE.\n      ').
enum_description('MAV_MODE_PROPERTY', 'Mode properties.\n      ').
enum_description('MAV_CMD', 'Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. NaN and INT32_MAX may be used in float/integer params (respectively) to indicate optional/default values (e.g. to use the component\'s current yaw or latitude rather than a specific value). See https://mavlink.io/en/guide/xml_schema.html#MAV_CMD for information about the structure of the MAV_CMD entries').
enum_description('MAV_BATTERY_STATUS_FLAGS', 'Battery status flags for fault, health and state indication.').
enum_description('TARGET_ABSOLUTE_SENSOR_CAPABILITY_FLAGS', 'These flags indicate the sensor reporting capabilities for TARGET_ABSOLUTE.').
enum_description('TARGET_OBS_FRAME', 'The frame of a target observation from an onboard sensor.').
enum_description('MAV_AUTOPILOT', 'Micro air vehicle / autopilot classes. This identifies the individual model.').
enum_description('MAV_TYPE', 'MAVLINK component type reported in HEARTBEAT message. Flight controllers must report the type of the vehicle on which they are mounted (e.g. MAV_TYPE_OCTOROTOR). All other components must report a value appropriate for their type (e.g. a camera must use MAV_TYPE_CAMERA).').
enum_description('MAV_MODE_FLAG', 'These flags encode the MAV mode.').
enum_description('MAV_MODE_FLAG_DECODE_POSITION', 'These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not.').
enum_description('MAV_COMPONENT', 'Component ids (values) for the different types and instances of onboard hardware/software that might make up a MAVLink system (autopilot, cameras, servos, GPS systems, avoidance systems etc.).\n      Components must use the appropriate ID in their source address when sending messages. Components can also use IDs to determine if they are the intended recipient of an incoming message. The MAV_COMP_ID_ALL value is used to indicate messages that must be processed by all components.\n      When creating new entries, components that can have multiple instances (e.g. cameras, servos etc.) should be allocated sequential values. An appropriate number of values should be left free after these components to allow the number of instances to be expanded.').
enum_description('UALBERTA_AUTOPILOT_MODE', 'Available autopilot modes for ualberta uav').
enum_description('UALBERTA_NAV_MODE', 'Navigation filter mode').
enum_description('UALBERTA_PILOT_MODE', 'Mode currently commanded by pilot').
enum_description('UAVIONIX_ADSB_OUT_DYNAMIC_STATE', 'State flags for ADS-B transponder dynamic report').
enum_description('UAVIONIX_ADSB_OUT_RF_SELECT', 'Transceiver RF control flags for ADS-B transponder dynamic reports').
enum_description('UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX', 'Status for ADS-B transponder dynamic input').
enum_description('UAVIONIX_ADSB_RF_HEALTH', 'Status flags for ADS-B transponder dynamic output').
enum_description('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'Definitions for aircraft size').
enum_description('UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT', 'GPS lataral offset encoding').
enum_description('UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON', 'GPS longitudinal offset encoding').
enum_description('UAVIONIX_ADSB_EMERGENCY_STATUS', 'Emergency status encoding').
enum_description('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'STorM32 gimbal prearm check flags.').
enum_description('MAV_STORM32_CAMERA_PREARM_FLAGS', 'STorM32 camera prearm check flags.').
enum_description('MAV_STORM32_GIMBAL_MANAGER_CAP_FLAGS', 'Gimbal manager capability flags.').
enum_description('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'Flags for gimbal manager operation. Used for setting and reporting, unless specified otherwise. If a setting has been accepted by the gimbal manager is reported in the STORM32_GIMBAL_MANAGER_STATUS message.').
enum_description('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'Gimbal manager client ID. In a prioritizing profile, the priorities are determined by the implementation; they could e.g. be custom1 > onboard > GCS > autopilot/camera > GCS2 > custom2.').
enum_description('MAV_STORM32_GIMBAL_MANAGER_PROFILE', 'Gimbal manager profiles. Only standard profiles are defined. Any implementation can define its own profile(s) in addition, and should use enum values > 16.').
enum_description('MAV_QSHOT_MODE', 'Enumeration of possible shot modes.').
enum_description('RADIO_RC_CHANNELS_FLAGS', 'RADIO_RC_CHANNELS flags (bitmask).').
enum_description('RADIO_LINK_STATS_FLAGS', 'RADIO_LINK_STATS flags (bitmask).').

:- dynamic enum_entry/4.

enum_entry('ACCELCAL_VEHICLE_POS', 'ACCELCAL_VEHICLE_POS_LEVEL', 1, []).
enum_entry('ACCELCAL_VEHICLE_POS', 'ACCELCAL_VEHICLE_POS_LEFT', 2, []).
enum_entry('ACCELCAL_VEHICLE_POS', 'ACCELCAL_VEHICLE_POS_RIGHT', 3, []).
enum_entry('ACCELCAL_VEHICLE_POS', 'ACCELCAL_VEHICLE_POS_NOSEDOWN', 4, []).
enum_entry('ACCELCAL_VEHICLE_POS', 'ACCELCAL_VEHICLE_POS_NOSEUP', 5, []).
enum_entry('ACCELCAL_VEHICLE_POS', 'ACCELCAL_VEHICLE_POS_BACK', 6, []).
enum_entry('ACCELCAL_VEHICLE_POS', 'ACCELCAL_VEHICLE_POS_SUCCESS', 16777215, []).
enum_entry('ACCELCAL_VEHICLE_POS', 'ACCELCAL_VEHICLE_POS_FAILED', 16777216, []).
enum_entry('HEADING_TYPE', 'HEADING_TYPE_COURSE_OVER_GROUND', 0, []).
enum_entry('HEADING_TYPE', 'HEADING_TYPE_HEADING', 1, []).
enum_entry('SPEED_TYPE', 'SPEED_TYPE_AIRSPEED', 0, []).
enum_entry('SPEED_TYPE', 'SPEED_TYPE_GROUNDSPEED', 1, []).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_RESUME_REPEAT_DIST', 215, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SPRAYER', 216, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SEND_SCRIPT_MESSAGE', 217, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_AUX_FUNCTION', 218, []).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_ALTITUDE_WAIT', 83, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_POWER_OFF_INITIATED', 42000, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_CLICK', 42001, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_HOLD', 42002, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SOLO_BTN_PAUSE_CLICK', 42003, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL', 42004, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_FIELD', 42005, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SET_EKF_SOURCE_SET', 42007, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_START_MAG_CAL', 42424, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_ACCEPT_MAG_CAL', 42425, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_CANCEL_MAG_CAL', 42426, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_ACCELCAL_VEHICLE_POS', 42429, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SEND_BANNER', 42428, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SET_FACTORY_TEST_MODE', 42427, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_GIMBAL_RESET', 42501, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS', 42502, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION', 42503, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_GIMBAL_FULL_RESET', 42505, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_FLASH_BOOTLOADER', 42650, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_BATTERY_RESET', 42651, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DEBUG_TRAP', 42700, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SCRIPTING', 42701, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_SCRIPT_TIME', 42702, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_ATTITUDE_TIME', 42703, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_SPEED', 43000, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_ALTITUDE', 43001, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_HEADING', 43002, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_EXTERNAL_POSITION_ESTIMATE', 43003, [hasLocation=true, isDestination=false]).
enum_entry('SCRIPTING_CMD', 'SCRIPTING_CMD_REPL_START', 0, []).
enum_entry('SCRIPTING_CMD', 'SCRIPTING_CMD_REPL_STOP', 1, []).
enum_entry('SCRIPTING_CMD', 'SCRIPTING_CMD_STOP', 2, []).
enum_entry('SCRIPTING_CMD', 'SCRIPTING_CMD_STOP_AND_RESTART', 3, []).
enum_entry('LIMITS_STATE', 'LIMITS_INIT', 0, []).
enum_entry('LIMITS_STATE', 'LIMITS_DISABLED', 1, []).
enum_entry('LIMITS_STATE', 'LIMITS_ENABLED', 2, []).
enum_entry('LIMITS_STATE', 'LIMITS_TRIGGERED', 3, []).
enum_entry('LIMITS_STATE', 'LIMITS_RECOVERING', 4, []).
enum_entry('LIMITS_STATE', 'LIMITS_RECOVERED', 5, []).
enum_entry('LIMIT_MODULE', 'LIMIT_GPSLOCK', 1, []).
enum_entry('LIMIT_MODULE', 'LIMIT_GEOFENCE', 2, []).
enum_entry('LIMIT_MODULE', 'LIMIT_ALTITUDE', 4, []).
enum_entry('RALLY_FLAGS', 'FAVORABLE_WIND', 1, []).
enum_entry('RALLY_FLAGS', 'LAND_IMMEDIATELY', 2, []).
enum_entry('CAMERA_STATUS_TYPES', 'CAMERA_STATUS_TYPE_HEARTBEAT', 0, []).
enum_entry('CAMERA_STATUS_TYPES', 'CAMERA_STATUS_TYPE_TRIGGER', 1, []).
enum_entry('CAMERA_STATUS_TYPES', 'CAMERA_STATUS_TYPE_DISCONNECT', 2, []).
enum_entry('CAMERA_STATUS_TYPES', 'CAMERA_STATUS_TYPE_ERROR', 3, []).
enum_entry('CAMERA_STATUS_TYPES', 'CAMERA_STATUS_TYPE_LOWBATT', 4, []).
enum_entry('CAMERA_STATUS_TYPES', 'CAMERA_STATUS_TYPE_LOWSTORE', 5, []).
enum_entry('CAMERA_STATUS_TYPES', 'CAMERA_STATUS_TYPE_LOWSTOREV', 6, []).
enum_entry('CAMERA_FEEDBACK_FLAGS', 'CAMERA_FEEDBACK_PHOTO', 0, []).
enum_entry('CAMERA_FEEDBACK_FLAGS', 'CAMERA_FEEDBACK_VIDEO', 1, []).
enum_entry('CAMERA_FEEDBACK_FLAGS', 'CAMERA_FEEDBACK_BADEXPOSURE', 2, []).
enum_entry('CAMERA_FEEDBACK_FLAGS', 'CAMERA_FEEDBACK_CLOSEDLOOP', 3, []).
enum_entry('CAMERA_FEEDBACK_FLAGS', 'CAMERA_FEEDBACK_OPENLOOP', 4, []).
enum_entry('MAV_MODE_GIMBAL', 'MAV_MODE_GIMBAL_UNINITIALIZED', 0, []).
enum_entry('MAV_MODE_GIMBAL', 'MAV_MODE_GIMBAL_CALIBRATING_PITCH', 1, []).
enum_entry('MAV_MODE_GIMBAL', 'MAV_MODE_GIMBAL_CALIBRATING_ROLL', 2, []).
enum_entry('MAV_MODE_GIMBAL', 'MAV_MODE_GIMBAL_CALIBRATING_YAW', 3, []).
enum_entry('MAV_MODE_GIMBAL', 'MAV_MODE_GIMBAL_INITIALIZED', 4, []).
enum_entry('MAV_MODE_GIMBAL', 'MAV_MODE_GIMBAL_ACTIVE', 5, []).
enum_entry('MAV_MODE_GIMBAL', 'MAV_MODE_GIMBAL_RATE_CMD_TIMEOUT', 6, []).
enum_entry('GIMBAL_AXIS', 'GIMBAL_AXIS_YAW', 0, []).
enum_entry('GIMBAL_AXIS', 'GIMBAL_AXIS_PITCH', 1, []).
enum_entry('GIMBAL_AXIS', 'GIMBAL_AXIS_ROLL', 2, []).
enum_entry('GIMBAL_AXIS_CALIBRATION_STATUS', 'GIMBAL_AXIS_CALIBRATION_STATUS_IN_PROGRESS', 0, []).
enum_entry('GIMBAL_AXIS_CALIBRATION_STATUS', 'GIMBAL_AXIS_CALIBRATION_STATUS_SUCCEEDED', 1, []).
enum_entry('GIMBAL_AXIS_CALIBRATION_STATUS', 'GIMBAL_AXIS_CALIBRATION_STATUS_FAILED', 2, []).
enum_entry('GIMBAL_AXIS_CALIBRATION_REQUIRED', 'GIMBAL_AXIS_CALIBRATION_REQUIRED_UNKNOWN', 0, []).
enum_entry('GIMBAL_AXIS_CALIBRATION_REQUIRED', 'GIMBAL_AXIS_CALIBRATION_REQUIRED_TRUE', 1, []).
enum_entry('GIMBAL_AXIS_CALIBRATION_REQUIRED', 'GIMBAL_AXIS_CALIBRATION_REQUIRED_FALSE', 2, []).
enum_entry('GOPRO_HEARTBEAT_STATUS', 'GOPRO_HEARTBEAT_STATUS_DISCONNECTED', 0, []).
enum_entry('GOPRO_HEARTBEAT_STATUS', 'GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE', 1, []).
enum_entry('GOPRO_HEARTBEAT_STATUS', 'GOPRO_HEARTBEAT_STATUS_CONNECTED', 2, []).
enum_entry('GOPRO_HEARTBEAT_STATUS', 'GOPRO_HEARTBEAT_STATUS_ERROR', 3, []).
enum_entry('GOPRO_HEARTBEAT_FLAGS', 'GOPRO_FLAG_RECORDING', 1, []).
enum_entry('GOPRO_REQUEST_STATUS', 'GOPRO_REQUEST_SUCCESS', 0, []).
enum_entry('GOPRO_REQUEST_STATUS', 'GOPRO_REQUEST_FAILED', 1, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_POWER', 0, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_CAPTURE_MODE', 1, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_SHUTTER', 2, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_BATTERY', 3, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_MODEL', 4, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_VIDEO_SETTINGS', 5, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_LOW_LIGHT', 6, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_PHOTO_RESOLUTION', 7, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_PHOTO_BURST_RATE', 8, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_PROTUNE', 9, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_PROTUNE_WHITE_BALANCE', 10, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_PROTUNE_COLOUR', 11, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_PROTUNE_GAIN', 12, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_PROTUNE_SHARPNESS', 13, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_PROTUNE_EXPOSURE', 14, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_TIME', 15, []).
enum_entry('GOPRO_COMMAND', 'GOPRO_COMMAND_CHARGING', 16, []).
enum_entry('GOPRO_CAPTURE_MODE', 'GOPRO_CAPTURE_MODE_VIDEO', 0, []).
enum_entry('GOPRO_CAPTURE_MODE', 'GOPRO_CAPTURE_MODE_PHOTO', 1, []).
enum_entry('GOPRO_CAPTURE_MODE', 'GOPRO_CAPTURE_MODE_BURST', 2, []).
enum_entry('GOPRO_CAPTURE_MODE', 'GOPRO_CAPTURE_MODE_TIME_LAPSE', 3, []).
enum_entry('GOPRO_CAPTURE_MODE', 'GOPRO_CAPTURE_MODE_MULTI_SHOT', 4, []).
enum_entry('GOPRO_CAPTURE_MODE', 'GOPRO_CAPTURE_MODE_PLAYBACK', 5, []).
enum_entry('GOPRO_CAPTURE_MODE', 'GOPRO_CAPTURE_MODE_SETUP', 6, []).
enum_entry('GOPRO_CAPTURE_MODE', 'GOPRO_CAPTURE_MODE_UNKNOWN', 255, []).
enum_entry('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_480p', 0, []).
enum_entry('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_720p', 1, []).
enum_entry('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_960p', 2, []).
enum_entry('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_1080p', 3, []).
enum_entry('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_1440p', 4, []).
enum_entry('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_2_7k_17_9', 5, []).
enum_entry('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_2_7k_16_9', 6, []).
enum_entry('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_2_7k_4_3', 7, []).
enum_entry('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_4k_16_9', 8, []).
enum_entry('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_4k_17_9', 9, []).
enum_entry('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_720p_SUPERVIEW', 10, []).
enum_entry('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_1080p_SUPERVIEW', 11, []).
enum_entry('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_2_7k_SUPERVIEW', 12, []).
enum_entry('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_4k_SUPERVIEW', 13, []).
enum_entry('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_12', 0, []).
enum_entry('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_15', 1, []).
enum_entry('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_24', 2, []).
enum_entry('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_25', 3, []).
enum_entry('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_30', 4, []).
enum_entry('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_48', 5, []).
enum_entry('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_50', 6, []).
enum_entry('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_60', 7, []).
enum_entry('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_80', 8, []).
enum_entry('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_90', 9, []).
enum_entry('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_100', 10, []).
enum_entry('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_120', 11, []).
enum_entry('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_240', 12, []).
enum_entry('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_12_5', 13, []).
enum_entry('GOPRO_FIELD_OF_VIEW', 'GOPRO_FIELD_OF_VIEW_WIDE', 0, []).
enum_entry('GOPRO_FIELD_OF_VIEW', 'GOPRO_FIELD_OF_VIEW_MEDIUM', 1, []).
enum_entry('GOPRO_FIELD_OF_VIEW', 'GOPRO_FIELD_OF_VIEW_NARROW', 2, []).
enum_entry('GOPRO_VIDEO_SETTINGS_FLAGS', 'GOPRO_VIDEO_SETTINGS_TV_MODE', 1, []).
enum_entry('GOPRO_PHOTO_RESOLUTION', 'GOPRO_PHOTO_RESOLUTION_5MP_MEDIUM', 0, []).
enum_entry('GOPRO_PHOTO_RESOLUTION', 'GOPRO_PHOTO_RESOLUTION_7MP_MEDIUM', 1, []).
enum_entry('GOPRO_PHOTO_RESOLUTION', 'GOPRO_PHOTO_RESOLUTION_7MP_WIDE', 2, []).
enum_entry('GOPRO_PHOTO_RESOLUTION', 'GOPRO_PHOTO_RESOLUTION_10MP_WIDE', 3, []).
enum_entry('GOPRO_PHOTO_RESOLUTION', 'GOPRO_PHOTO_RESOLUTION_12MP_WIDE', 4, []).
enum_entry('GOPRO_PROTUNE_WHITE_BALANCE', 'GOPRO_PROTUNE_WHITE_BALANCE_AUTO', 0, []).
enum_entry('GOPRO_PROTUNE_WHITE_BALANCE', 'GOPRO_PROTUNE_WHITE_BALANCE_3000K', 1, []).
enum_entry('GOPRO_PROTUNE_WHITE_BALANCE', 'GOPRO_PROTUNE_WHITE_BALANCE_5500K', 2, []).
enum_entry('GOPRO_PROTUNE_WHITE_BALANCE', 'GOPRO_PROTUNE_WHITE_BALANCE_6500K', 3, []).
enum_entry('GOPRO_PROTUNE_WHITE_BALANCE', 'GOPRO_PROTUNE_WHITE_BALANCE_RAW', 4, []).
enum_entry('GOPRO_PROTUNE_COLOUR', 'GOPRO_PROTUNE_COLOUR_STANDARD', 0, []).
enum_entry('GOPRO_PROTUNE_COLOUR', 'GOPRO_PROTUNE_COLOUR_NEUTRAL', 1, []).
enum_entry('GOPRO_PROTUNE_GAIN', 'GOPRO_PROTUNE_GAIN_400', 0, []).
enum_entry('GOPRO_PROTUNE_GAIN', 'GOPRO_PROTUNE_GAIN_800', 1, []).
enum_entry('GOPRO_PROTUNE_GAIN', 'GOPRO_PROTUNE_GAIN_1600', 2, []).
enum_entry('GOPRO_PROTUNE_GAIN', 'GOPRO_PROTUNE_GAIN_3200', 3, []).
enum_entry('GOPRO_PROTUNE_GAIN', 'GOPRO_PROTUNE_GAIN_6400', 4, []).
enum_entry('GOPRO_PROTUNE_SHARPNESS', 'GOPRO_PROTUNE_SHARPNESS_LOW', 0, []).
enum_entry('GOPRO_PROTUNE_SHARPNESS', 'GOPRO_PROTUNE_SHARPNESS_MEDIUM', 1, []).
enum_entry('GOPRO_PROTUNE_SHARPNESS', 'GOPRO_PROTUNE_SHARPNESS_HIGH', 2, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_5_0', 0, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_4_5', 1, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_4_0', 2, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_3_5', 3, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_3_0', 4, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_2_5', 5, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_2_0', 6, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_1_5', 7, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_1_0', 8, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_0_5', 9, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_ZERO', 10, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_0_5', 11, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_1_0', 12, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_1_5', 13, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_2_0', 14, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_2_5', 15, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_3_0', 16, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_3_5', 17, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_4_0', 18, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_4_5', 19, []).
enum_entry('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_5_0', 20, []).
enum_entry('GOPRO_CHARGING', 'GOPRO_CHARGING_DISABLED', 0, []).
enum_entry('GOPRO_CHARGING', 'GOPRO_CHARGING_ENABLED', 1, []).
enum_entry('GOPRO_MODEL', 'GOPRO_MODEL_UNKNOWN', 0, []).
enum_entry('GOPRO_MODEL', 'GOPRO_MODEL_HERO_3_PLUS_SILVER', 1, []).
enum_entry('GOPRO_MODEL', 'GOPRO_MODEL_HERO_3_PLUS_BLACK', 2, []).
enum_entry('GOPRO_MODEL', 'GOPRO_MODEL_HERO_4_SILVER', 3, []).
enum_entry('GOPRO_MODEL', 'GOPRO_MODEL_HERO_4_BLACK', 4, []).
enum_entry('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_3_IN_1_SECOND', 0, []).
enum_entry('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_5_IN_1_SECOND', 1, []).
enum_entry('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_10_IN_1_SECOND', 2, []).
enum_entry('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_10_IN_2_SECOND', 3, []).
enum_entry('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_10_IN_3_SECOND', 4, []).
enum_entry('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_30_IN_1_SECOND', 5, []).
enum_entry('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_30_IN_2_SECOND', 6, []).
enum_entry('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_30_IN_3_SECOND', 7, []).
enum_entry('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_30_IN_6_SECOND', 8, []).
enum_entry('MAV_CMD_DO_AUX_FUNCTION_SWITCH_LEVEL', 'MAV_CMD_DO_AUX_FUNCTION_SWITCH_LEVEL_LOW', 0, []).
enum_entry('MAV_CMD_DO_AUX_FUNCTION_SWITCH_LEVEL', 'MAV_CMD_DO_AUX_FUNCTION_SWITCH_LEVEL_MIDDLE', 1, []).
enum_entry('MAV_CMD_DO_AUX_FUNCTION_SWITCH_LEVEL', 'MAV_CMD_DO_AUX_FUNCTION_SWITCH_LEVEL_HIGH', 2, []).
enum_entry('LED_CONTROL_PATTERN', 'LED_CONTROL_PATTERN_OFF', 0, []).
enum_entry('LED_CONTROL_PATTERN', 'LED_CONTROL_PATTERN_FIRMWAREUPDATE', 1, []).
enum_entry('LED_CONTROL_PATTERN', 'LED_CONTROL_PATTERN_CUSTOM', 255, []).
enum_entry('EKF_STATUS_FLAGS', 'EKF_ATTITUDE', 1, []).
enum_entry('EKF_STATUS_FLAGS', 'EKF_VELOCITY_HORIZ', 2, []).
enum_entry('EKF_STATUS_FLAGS', 'EKF_VELOCITY_VERT', 4, []).
enum_entry('EKF_STATUS_FLAGS', 'EKF_POS_HORIZ_REL', 8, []).
enum_entry('EKF_STATUS_FLAGS', 'EKF_POS_HORIZ_ABS', 16, []).
enum_entry('EKF_STATUS_FLAGS', 'EKF_POS_VERT_ABS', 32, []).
enum_entry('EKF_STATUS_FLAGS', 'EKF_POS_VERT_AGL', 64, []).
enum_entry('EKF_STATUS_FLAGS', 'EKF_CONST_POS_MODE', 128, []).
enum_entry('EKF_STATUS_FLAGS', 'EKF_PRED_POS_HORIZ_REL', 256, []).
enum_entry('EKF_STATUS_FLAGS', 'EKF_PRED_POS_HORIZ_ABS', 512, []).
enum_entry('EKF_STATUS_FLAGS', 'EKF_UNINITIALIZED', 1024, []).
enum_entry('PID_TUNING_AXIS', 'PID_TUNING_ROLL', 1, []).
enum_entry('PID_TUNING_AXIS', 'PID_TUNING_PITCH', 2, []).
enum_entry('PID_TUNING_AXIS', 'PID_TUNING_YAW', 3, []).
enum_entry('PID_TUNING_AXIS', 'PID_TUNING_ACCZ', 4, []).
enum_entry('PID_TUNING_AXIS', 'PID_TUNING_STEER', 5, []).
enum_entry('PID_TUNING_AXIS', 'PID_TUNING_LANDING', 6, []).
enum_entry('MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS', 'MAV_REMOTE_LOG_DATA_BLOCK_STOP', 2147483645, []).
enum_entry('MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS', 'MAV_REMOTE_LOG_DATA_BLOCK_START', 2147483646, []).
enum_entry('MAV_REMOTE_LOG_DATA_BLOCK_STATUSES', 'MAV_REMOTE_LOG_DATA_BLOCK_NACK', 0, []).
enum_entry('MAV_REMOTE_LOG_DATA_BLOCK_STATUSES', 'MAV_REMOTE_LOG_DATA_BLOCK_ACK', 1, []).
enum_entry('DEVICE_OP_BUSTYPE', 'DEVICE_OP_BUSTYPE_I2C', 0, []).
enum_entry('DEVICE_OP_BUSTYPE', 'DEVICE_OP_BUSTYPE_SPI', 1, []).
enum_entry('DEEPSTALL_STAGE', 'DEEPSTALL_STAGE_FLY_TO_LANDING', 0, []).
enum_entry('DEEPSTALL_STAGE', 'DEEPSTALL_STAGE_ESTIMATE_WIND', 1, []).
enum_entry('DEEPSTALL_STAGE', 'DEEPSTALL_STAGE_WAIT_FOR_BREAKOUT', 2, []).
enum_entry('DEEPSTALL_STAGE', 'DEEPSTALL_STAGE_FLY_TO_ARC', 3, []).
enum_entry('DEEPSTALL_STAGE', 'DEEPSTALL_STAGE_ARC', 4, []).
enum_entry('DEEPSTALL_STAGE', 'DEEPSTALL_STAGE_APPROACH', 5, []).
enum_entry('DEEPSTALL_STAGE', 'DEEPSTALL_STAGE_LAND', 6, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_MANUAL', 0, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_CIRCLE', 1, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_STABILIZE', 2, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_TRAINING', 3, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_ACRO', 4, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_FLY_BY_WIRE_A', 5, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_FLY_BY_WIRE_B', 6, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_CRUISE', 7, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_AUTOTUNE', 8, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_AUTO', 10, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_RTL', 11, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_LOITER', 12, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_TAKEOFF', 13, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_AVOID_ADSB', 14, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_GUIDED', 15, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_INITIALIZING', 16, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_QSTABILIZE', 17, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_QHOVER', 18, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_QLOITER', 19, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_QLAND', 20, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_QRTL', 21, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_QAUTOTUNE', 22, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_QACRO', 23, []).
enum_entry('PLANE_MODE', 'PLANE_MODE_THERMAL', 24, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_STABILIZE', 0, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_ACRO', 1, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_ALT_HOLD', 2, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_AUTO', 3, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_GUIDED', 4, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_LOITER', 5, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_RTL', 6, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_CIRCLE', 7, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_LAND', 9, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_DRIFT', 11, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_SPORT', 13, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_FLIP', 14, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_AUTOTUNE', 15, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_POSHOLD', 16, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_BRAKE', 17, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_THROW', 18, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_AVOID_ADSB', 19, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_GUIDED_NOGPS', 20, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_SMART_RTL', 21, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_FLOWHOLD', 22, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_FOLLOW', 23, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_ZIGZAG', 24, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_SYSTEMID', 25, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_AUTOROTATE', 26, []).
enum_entry('COPTER_MODE', 'COPTER_MODE_AUTO_RTL', 27, []).
enum_entry('SUB_MODE', 'SUB_MODE_STABILIZE', 0, []).
enum_entry('SUB_MODE', 'SUB_MODE_ACRO', 1, []).
enum_entry('SUB_MODE', 'SUB_MODE_ALT_HOLD', 2, []).
enum_entry('SUB_MODE', 'SUB_MODE_AUTO', 3, []).
enum_entry('SUB_MODE', 'SUB_MODE_GUIDED', 4, []).
enum_entry('SUB_MODE', 'SUB_MODE_CIRCLE', 7, []).
enum_entry('SUB_MODE', 'SUB_MODE_SURFACE', 9, []).
enum_entry('SUB_MODE', 'SUB_MODE_POSHOLD', 16, []).
enum_entry('SUB_MODE', 'SUB_MODE_MANUAL', 19, []).
enum_entry('ROVER_MODE', 'ROVER_MODE_MANUAL', 0, []).
enum_entry('ROVER_MODE', 'ROVER_MODE_ACRO', 1, []).
enum_entry('ROVER_MODE', 'ROVER_MODE_STEERING', 3, []).
enum_entry('ROVER_MODE', 'ROVER_MODE_HOLD', 4, []).
enum_entry('ROVER_MODE', 'ROVER_MODE_LOITER', 5, []).
enum_entry('ROVER_MODE', 'ROVER_MODE_FOLLOW', 6, []).
enum_entry('ROVER_MODE', 'ROVER_MODE_SIMPLE', 7, []).
enum_entry('ROVER_MODE', 'ROVER_MODE_AUTO', 10, []).
enum_entry('ROVER_MODE', 'ROVER_MODE_RTL', 11, []).
enum_entry('ROVER_MODE', 'ROVER_MODE_SMART_RTL', 12, []).
enum_entry('ROVER_MODE', 'ROVER_MODE_GUIDED', 15, []).
enum_entry('ROVER_MODE', 'ROVER_MODE_INITIALIZING', 16, []).
enum_entry('TRACKER_MODE', 'TRACKER_MODE_MANUAL', 0, []).
enum_entry('TRACKER_MODE', 'TRACKER_MODE_STOP', 1, []).
enum_entry('TRACKER_MODE', 'TRACKER_MODE_SCAN', 2, []).
enum_entry('TRACKER_MODE', 'TRACKER_MODE_SERVO_TEST', 3, []).
enum_entry('TRACKER_MODE', 'TRACKER_MODE_AUTO', 10, []).
enum_entry('TRACKER_MODE', 'TRACKER_MODE_INITIALIZING', 16, []).
enum_entry('OSD_PARAM_CONFIG_TYPE', 'OSD_PARAM_NONE', 0, []).
enum_entry('OSD_PARAM_CONFIG_TYPE', 'OSD_PARAM_SERIAL_PROTOCOL', 1, []).
enum_entry('OSD_PARAM_CONFIG_TYPE', 'OSD_PARAM_SERVO_FUNCTION', 2, []).
enum_entry('OSD_PARAM_CONFIG_TYPE', 'OSD_PARAM_AUX_FUNCTION', 3, []).
enum_entry('OSD_PARAM_CONFIG_TYPE', 'OSD_PARAM_FLIGHT_MODE', 4, []).
enum_entry('OSD_PARAM_CONFIG_TYPE', 'OSD_PARAM_FAILSAFE_ACTION', 5, []).
enum_entry('OSD_PARAM_CONFIG_TYPE', 'OSD_PARAM_FAILSAFE_ACTION_1', 6, []).
enum_entry('OSD_PARAM_CONFIG_TYPE', 'OSD_PARAM_FAILSAFE_ACTION_2', 7, []).
enum_entry('OSD_PARAM_CONFIG_TYPE', 'OSD_PARAM_NUM_TYPES', 8, []).
enum_entry('OSD_PARAM_CONFIG_ERROR', 'OSD_PARAM_SUCCESS', 0, []).
enum_entry('OSD_PARAM_CONFIG_ERROR', 'OSD_PARAM_INVALID_SCREEN', 1, []).
enum_entry('OSD_PARAM_CONFIG_ERROR', 'OSD_PARAM_INVALID_PARAMETER_INDEX', 2, []).
enum_entry('OSD_PARAM_CONFIG_ERROR', 'OSD_PARAM_INVALID_PARAMETER', 3, []).
enum_entry('MAV_CMD', 'MAV_CMD_RESET_MPPT', 40001, []).
enum_entry('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL', 40002, []).
enum_entry('GSM_LINK_TYPE', 'GSM_LINK_TYPE_NONE', 0, []).
enum_entry('GSM_LINK_TYPE', 'GSM_LINK_TYPE_UNKNOWN', 1, []).
enum_entry('GSM_LINK_TYPE', 'GSM_LINK_TYPE_2G', 2, []).
enum_entry('GSM_LINK_TYPE', 'GSM_LINK_TYPE_3G', 3, []).
enum_entry('GSM_LINK_TYPE', 'GSM_LINK_TYPE_4G', 4, []).
enum_entry('GSM_MODEM_TYPE', 'GSM_MODEM_TYPE_UNKNOWN', 0, []).
enum_entry('GSM_MODEM_TYPE', 'GSM_MODEM_TYPE_HUAWEI_E3372', 1, []).
enum_entry('FIRMWARE_VERSION_TYPE', 'FIRMWARE_VERSION_TYPE_DEV', 0, []).
enum_entry('FIRMWARE_VERSION_TYPE', 'FIRMWARE_VERSION_TYPE_ALPHA', 64, []).
enum_entry('FIRMWARE_VERSION_TYPE', 'FIRMWARE_VERSION_TYPE_BETA', 128, []).
enum_entry('FIRMWARE_VERSION_TYPE', 'FIRMWARE_VERSION_TYPE_RC', 192, []).
enum_entry('FIRMWARE_VERSION_TYPE', 'FIRMWARE_VERSION_TYPE_OFFICIAL', 255, []).
enum_entry('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_GPS', 1, []).
enum_entry('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE', 2, []).
enum_entry('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_ABSOLUTE_PRESSURE', 4, []).
enum_entry('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_3D_ACCEL', 8, []).
enum_entry('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_3D_GYRO', 16, []).
enum_entry('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_3D_MAG', 32, []).
enum_entry('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_TERRAIN', 64, []).
enum_entry('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_BATTERY', 128, []).
enum_entry('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_RC_RECEIVER', 256, []).
enum_entry('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_OFFBOARD_LINK', 512, []).
enum_entry('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_ENGINE', 1024, []).
enum_entry('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_GEOFENCE', 2048, []).
enum_entry('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_ESTIMATOR', 4096, []).
enum_entry('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_MISSION', 8192, []).
enum_entry('MAV_GOTO', 'MAV_GOTO_DO_HOLD', 0, []).
enum_entry('MAV_GOTO', 'MAV_GOTO_DO_CONTINUE', 1, []).
enum_entry('MAV_GOTO', 'MAV_GOTO_HOLD_AT_CURRENT_POSITION', 2, []).
enum_entry('MAV_GOTO', 'MAV_GOTO_HOLD_AT_SPECIFIED_POSITION', 3, []).
enum_entry('MAV_MODE', 'MAV_MODE_PREFLIGHT', 0, []).
enum_entry('MAV_MODE', 'MAV_MODE_STABILIZE_DISARMED', 80, []).
enum_entry('MAV_MODE', 'MAV_MODE_STABILIZE_ARMED', 208, []).
enum_entry('MAV_MODE', 'MAV_MODE_MANUAL_DISARMED', 64, []).
enum_entry('MAV_MODE', 'MAV_MODE_MANUAL_ARMED', 192, []).
enum_entry('MAV_MODE', 'MAV_MODE_GUIDED_DISARMED', 88, []).
enum_entry('MAV_MODE', 'MAV_MODE_GUIDED_ARMED', 216, []).
enum_entry('MAV_MODE', 'MAV_MODE_AUTO_DISARMED', 92, []).
enum_entry('MAV_MODE', 'MAV_MODE_AUTO_ARMED', 220, []).
enum_entry('MAV_MODE', 'MAV_MODE_TEST_DISARMED', 66, []).
enum_entry('MAV_MODE', 'MAV_MODE_TEST_ARMED', 194, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_3D_GYRO', 1, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_3D_ACCEL', 2, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_3D_MAG', 4, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE', 8, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE', 16, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_GPS', 32, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW', 64, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_VISION_POSITION', 128, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_LASER_POSITION', 256, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH', 512, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL', 1024, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION', 2048, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_YAW_POSITION', 4096, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL', 8192, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL', 16384, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS', 32768, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_RC_RECEIVER', 65536, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_3D_GYRO2', 131072, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_3D_ACCEL2', 262144, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_3D_MAG2', 524288, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_GEOFENCE', 1048576, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_AHRS', 2097152, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_TERRAIN', 4194304, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_REVERSE_MOTOR', 8388608, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_LOGGING', 16777216, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_BATTERY', 33554432, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_PROXIMITY', 67108864, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_SATCOM', 134217728, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_PREARM_CHECK', 268435456, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_OBSTACLE_AVOIDANCE', 536870912, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_PROPULSION', 1073741824, []).
enum_entry('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_EXTENSION_USED', 2147483648, []).
enum_entry('MAV_SYS_STATUS_SENSOR_EXTENDED', 'MAV_SYS_STATUS_RECOVERY_SYSTEM', 1, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_GLOBAL', 0, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_LOCAL_NED', 1, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_MISSION', 2, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_GLOBAL_RELATIVE_ALT', 3, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_LOCAL_ENU', 4, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_GLOBAL_INT', 5, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_GLOBAL_RELATIVE_ALT_INT', 6, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_LOCAL_OFFSET_NED', 7, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_BODY_NED', 8, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_BODY_OFFSET_NED', 9, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_GLOBAL_TERRAIN_ALT', 10, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_GLOBAL_TERRAIN_ALT_INT', 11, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_BODY_FRD', 12, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_RESERVED_13', 13, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_RESERVED_14', 14, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_RESERVED_15', 15, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_RESERVED_16', 16, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_RESERVED_17', 17, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_RESERVED_18', 18, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_RESERVED_19', 19, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_LOCAL_FRD', 20, []).
enum_entry('MAV_FRAME', 'MAV_FRAME_LOCAL_FLU', 21, []).
enum_entry('MAVLINK_DATA_STREAM_TYPE', 'MAVLINK_DATA_STREAM_IMG_JPEG', 0, []).
enum_entry('MAVLINK_DATA_STREAM_TYPE', 'MAVLINK_DATA_STREAM_IMG_BMP', 1, []).
enum_entry('MAVLINK_DATA_STREAM_TYPE', 'MAVLINK_DATA_STREAM_IMG_RAW8U', 2, []).
enum_entry('MAVLINK_DATA_STREAM_TYPE', 'MAVLINK_DATA_STREAM_IMG_RAW32U', 3, []).
enum_entry('MAVLINK_DATA_STREAM_TYPE', 'MAVLINK_DATA_STREAM_IMG_PGM', 4, []).
enum_entry('MAVLINK_DATA_STREAM_TYPE', 'MAVLINK_DATA_STREAM_IMG_PNG', 5, []).
enum_entry('FENCE_ACTION', 'FENCE_ACTION_NONE', 0, []).
enum_entry('FENCE_ACTION', 'FENCE_ACTION_GUIDED', 1, []).
enum_entry('FENCE_ACTION', 'FENCE_ACTION_REPORT', 2, []).
enum_entry('FENCE_ACTION', 'FENCE_ACTION_GUIDED_THR_PASS', 3, []).
enum_entry('FENCE_ACTION', 'FENCE_ACTION_RTL', 4, []).
enum_entry('FENCE_ACTION', 'FENCE_ACTION_HOLD', 5, []).
enum_entry('FENCE_ACTION', 'FENCE_ACTION_TERMINATE', 6, []).
enum_entry('FENCE_ACTION', 'FENCE_ACTION_LAND', 7, []).
enum_entry('FENCE_BREACH', 'FENCE_BREACH_NONE', 0, []).
enum_entry('FENCE_BREACH', 'FENCE_BREACH_MINALT', 1, []).
enum_entry('FENCE_BREACH', 'FENCE_BREACH_MAXALT', 2, []).
enum_entry('FENCE_BREACH', 'FENCE_BREACH_BOUNDARY', 3, []).
enum_entry('FENCE_MITIGATE', 'FENCE_MITIGATE_UNKNOWN', 0, []).
enum_entry('FENCE_MITIGATE', 'FENCE_MITIGATE_NONE', 1, []).
enum_entry('FENCE_MITIGATE', 'FENCE_MITIGATE_VEL_LIMIT', 2, []).
enum_entry('MAV_MOUNT_MODE', 'MAV_MOUNT_MODE_RETRACT', 0, []).
enum_entry('MAV_MOUNT_MODE', 'MAV_MOUNT_MODE_NEUTRAL', 1, []).
enum_entry('MAV_MOUNT_MODE', 'MAV_MOUNT_MODE_MAVLINK_TARGETING', 2, []).
enum_entry('MAV_MOUNT_MODE', 'MAV_MOUNT_MODE_RC_TARGETING', 3, []).
enum_entry('MAV_MOUNT_MODE', 'MAV_MOUNT_MODE_GPS_POINT', 4, []).
enum_entry('MAV_MOUNT_MODE', 'MAV_MOUNT_MODE_SYSID_TARGET', 5, []).
enum_entry('MAV_MOUNT_MODE', 'MAV_MOUNT_MODE_HOME_LOCATION', 6, []).
enum_entry('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT', 1, []).
enum_entry('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL', 2, []).
enum_entry('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS', 4, []).
enum_entry('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW', 8, []).
enum_entry('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK', 16, []).
enum_entry('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS', 32, []).
enum_entry('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW', 64, []).
enum_entry('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK', 128, []).
enum_entry('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS', 256, []).
enum_entry('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW', 512, []).
enum_entry('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK', 1024, []).
enum_entry('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW', 2048, []).
enum_entry('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME', 4096, []).
enum_entry('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_RC_INPUTS', 8192, []).
enum_entry('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT', 1, []).
enum_entry('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL', 2, []).
enum_entry('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS', 4, []).
enum_entry('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW', 8, []).
enum_entry('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK', 16, []).
enum_entry('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS', 32, []).
enum_entry('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW', 64, []).
enum_entry('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK', 128, []).
enum_entry('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS', 256, []).
enum_entry('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW', 512, []).
enum_entry('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK', 1024, []).
enum_entry('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW', 2048, []).
enum_entry('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME', 4096, []).
enum_entry('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_RC_INPUTS', 8192, []).
enum_entry('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL', 65536, []).
enum_entry('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL', 131072, []).
enum_entry('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_RETRACT', 1, []).
enum_entry('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_NEUTRAL', 2, []).
enum_entry('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_ROLL_LOCK', 4, []).
enum_entry('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_PITCH_LOCK', 8, []).
enum_entry('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_YAW_LOCK', 16, []).
enum_entry('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME', 32, []).
enum_entry('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME', 64, []).
enum_entry('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME', 128, []).
enum_entry('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE', 256, []).
enum_entry('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_RC_MIXED', 512, []).
enum_entry('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_RETRACT', 1, []).
enum_entry('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_NEUTRAL', 2, []).
enum_entry('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_ROLL_LOCK', 4, []).
enum_entry('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_PITCH_LOCK', 8, []).
enum_entry('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_YAW_LOCK', 16, []).
enum_entry('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME', 32, []).
enum_entry('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME', 64, []).
enum_entry('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME', 128, []).
enum_entry('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_RC_EXCLUSIVE', 256, []).
enum_entry('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_RC_MIXED', 512, []).
enum_entry('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT', 1, []).
enum_entry('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT', 2, []).
enum_entry('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT', 4, []).
enum_entry('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR', 8, []).
enum_entry('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR', 16, []).
enum_entry('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR', 32, []).
enum_entry('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR', 64, []).
enum_entry('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR', 128, []).
enum_entry('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING', 256, []).
enum_entry('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_NO_MANAGER', 512, []).
enum_entry('GRIPPER_ACTIONS', 'GRIPPER_ACTION_RELEASE', 0, []).
enum_entry('GRIPPER_ACTIONS', 'GRIPPER_ACTION_GRAB', 1, []).
enum_entry('WINCH_ACTIONS', 'WINCH_RELAXED', 0, []).
enum_entry('WINCH_ACTIONS', 'WINCH_RELATIVE_LENGTH_CONTROL', 1, []).
enum_entry('WINCH_ACTIONS', 'WINCH_RATE_CONTROL', 2, []).
enum_entry('WINCH_ACTIONS', 'WINCH_LOCK', 3, []).
enum_entry('WINCH_ACTIONS', 'WINCH_DELIVER', 4, []).
enum_entry('WINCH_ACTIONS', 'WINCH_HOLD', 5, []).
enum_entry('WINCH_ACTIONS', 'WINCH_RETRACT', 6, []).
enum_entry('WINCH_ACTIONS', 'WINCH_LOAD_LINE', 7, []).
enum_entry('WINCH_ACTIONS', 'WINCH_ABANDON_LINE', 8, []).
enum_entry('WINCH_ACTIONS', 'WINCH_LOAD_PAYLOAD', 9, []).
enum_entry('UAVCAN_NODE_HEALTH', 'UAVCAN_NODE_HEALTH_OK', 0, []).
enum_entry('UAVCAN_NODE_HEALTH', 'UAVCAN_NODE_HEALTH_WARNING', 1, []).
enum_entry('UAVCAN_NODE_HEALTH', 'UAVCAN_NODE_HEALTH_ERROR', 2, []).
enum_entry('UAVCAN_NODE_HEALTH', 'UAVCAN_NODE_HEALTH_CRITICAL', 3, []).
enum_entry('UAVCAN_NODE_MODE', 'UAVCAN_NODE_MODE_OPERATIONAL', 0, []).
enum_entry('UAVCAN_NODE_MODE', 'UAVCAN_NODE_MODE_INITIALIZATION', 1, []).
enum_entry('UAVCAN_NODE_MODE', 'UAVCAN_NODE_MODE_MAINTENANCE', 2, []).
enum_entry('UAVCAN_NODE_MODE', 'UAVCAN_NODE_MODE_SOFTWARE_UPDATE', 3, []).
enum_entry('UAVCAN_NODE_MODE', 'UAVCAN_NODE_MODE_OFFLINE', 7, []).
enum_entry('ESC_CONNECTION_TYPE', 'ESC_CONNECTION_TYPE_PPM', 0, []).
enum_entry('ESC_CONNECTION_TYPE', 'ESC_CONNECTION_TYPE_SERIAL', 1, []).
enum_entry('ESC_CONNECTION_TYPE', 'ESC_CONNECTION_TYPE_ONESHOT', 2, []).
enum_entry('ESC_CONNECTION_TYPE', 'ESC_CONNECTION_TYPE_I2C', 3, []).
enum_entry('ESC_CONNECTION_TYPE', 'ESC_CONNECTION_TYPE_CAN', 4, []).
enum_entry('ESC_CONNECTION_TYPE', 'ESC_CONNECTION_TYPE_DSHOT', 5, []).
enum_entry('ESC_FAILURE_FLAGS', 'ESC_FAILURE_NONE', 0, []).
enum_entry('ESC_FAILURE_FLAGS', 'ESC_FAILURE_OVER_CURRENT', 1, []).
enum_entry('ESC_FAILURE_FLAGS', 'ESC_FAILURE_OVER_VOLTAGE', 2, []).
enum_entry('ESC_FAILURE_FLAGS', 'ESC_FAILURE_OVER_TEMPERATURE', 4, []).
enum_entry('ESC_FAILURE_FLAGS', 'ESC_FAILURE_OVER_RPM', 8, []).
enum_entry('ESC_FAILURE_FLAGS', 'ESC_FAILURE_INCONSISTENT_CMD', 16, []).
enum_entry('ESC_FAILURE_FLAGS', 'ESC_FAILURE_MOTOR_STUCK', 32, []).
enum_entry('ESC_FAILURE_FLAGS', 'ESC_FAILURE_GENERIC', 64, []).
enum_entry('STORAGE_STATUS', 'STORAGE_STATUS_EMPTY', 0, []).
enum_entry('STORAGE_STATUS', 'STORAGE_STATUS_UNFORMATTED', 1, []).
enum_entry('STORAGE_STATUS', 'STORAGE_STATUS_READY', 2, []).
enum_entry('STORAGE_STATUS', 'STORAGE_STATUS_NOT_SUPPORTED', 3, []).
enum_entry('STORAGE_TYPE', 'STORAGE_TYPE_UNKNOWN', 0, []).
enum_entry('STORAGE_TYPE', 'STORAGE_TYPE_USB_STICK', 1, []).
enum_entry('STORAGE_TYPE', 'STORAGE_TYPE_SD', 2, []).
enum_entry('STORAGE_TYPE', 'STORAGE_TYPE_MICROSD', 3, []).
enum_entry('STORAGE_TYPE', 'STORAGE_TYPE_CF', 4, []).
enum_entry('STORAGE_TYPE', 'STORAGE_TYPE_CFE', 5, []).
enum_entry('STORAGE_TYPE', 'STORAGE_TYPE_XQD', 6, []).
enum_entry('STORAGE_TYPE', 'STORAGE_TYPE_HD', 7, []).
enum_entry('STORAGE_TYPE', 'STORAGE_TYPE_OTHER', 254, []).
enum_entry('STORAGE_USAGE_FLAG', 'STORAGE_USAGE_FLAG_SET', 1, []).
enum_entry('STORAGE_USAGE_FLAG', 'STORAGE_USAGE_FLAG_PHOTO', 2, []).
enum_entry('STORAGE_USAGE_FLAG', 'STORAGE_USAGE_FLAG_VIDEO', 4, []).
enum_entry('STORAGE_USAGE_FLAG', 'STORAGE_USAGE_FLAG_LOGS', 8, []).
enum_entry('ORBIT_YAW_BEHAVIOUR', 'ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER', 0, []).
enum_entry('ORBIT_YAW_BEHAVIOUR', 'ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING', 1, []).
enum_entry('ORBIT_YAW_BEHAVIOUR', 'ORBIT_YAW_BEHAVIOUR_UNCONTROLLED', 2, []).
enum_entry('ORBIT_YAW_BEHAVIOUR', 'ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE', 3, []).
enum_entry('ORBIT_YAW_BEHAVIOUR', 'ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED', 4, []).
enum_entry('WIFI_CONFIG_AP_RESPONSE', 'WIFI_CONFIG_AP_RESPONSE_UNDEFINED', 0, []).
enum_entry('WIFI_CONFIG_AP_RESPONSE', 'WIFI_CONFIG_AP_RESPONSE_ACCEPTED', 1, []).
enum_entry('WIFI_CONFIG_AP_RESPONSE', 'WIFI_CONFIG_AP_RESPONSE_REJECTED', 2, []).
enum_entry('WIFI_CONFIG_AP_RESPONSE', 'WIFI_CONFIG_AP_RESPONSE_MODE_ERROR', 3, []).
enum_entry('WIFI_CONFIG_AP_RESPONSE', 'WIFI_CONFIG_AP_RESPONSE_SSID_ERROR', 4, []).
enum_entry('WIFI_CONFIG_AP_RESPONSE', 'WIFI_CONFIG_AP_RESPONSE_PASSWORD_ERROR', 5, []).
enum_entry('CELLULAR_CONFIG_RESPONSE', 'CELLULAR_CONFIG_RESPONSE_ACCEPTED', 0, []).
enum_entry('CELLULAR_CONFIG_RESPONSE', 'CELLULAR_CONFIG_RESPONSE_APN_ERROR', 1, []).
enum_entry('CELLULAR_CONFIG_RESPONSE', 'CELLULAR_CONFIG_RESPONSE_PIN_ERROR', 2, []).
enum_entry('CELLULAR_CONFIG_RESPONSE', 'CELLULAR_CONFIG_RESPONSE_REJECTED', 3, []).
enum_entry('CELLULAR_CONFIG_RESPONSE', 'CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED', 4, []).
enum_entry('WIFI_CONFIG_AP_MODE', 'WIFI_CONFIG_AP_MODE_UNDEFINED', 0, []).
enum_entry('WIFI_CONFIG_AP_MODE', 'WIFI_CONFIG_AP_MODE_AP', 1, []).
enum_entry('WIFI_CONFIG_AP_MODE', 'WIFI_CONFIG_AP_MODE_STATION', 2, []).
enum_entry('WIFI_CONFIG_AP_MODE', 'WIFI_CONFIG_AP_MODE_DISABLED', 3, []).
enum_entry('COMP_METADATA_TYPE', 'COMP_METADATA_TYPE_GENERAL', 0, []).
enum_entry('COMP_METADATA_TYPE', 'COMP_METADATA_TYPE_PARAMETER', 1, []).
enum_entry('COMP_METADATA_TYPE', 'COMP_METADATA_TYPE_COMMANDS', 2, []).
enum_entry('COMP_METADATA_TYPE', 'COMP_METADATA_TYPE_PERIPHERALS', 3, []).
enum_entry('COMP_METADATA_TYPE', 'COMP_METADATA_TYPE_EVENTS', 4, []).
enum_entry('COMP_METADATA_TYPE', 'COMP_METADATA_TYPE_ACTUATORS', 5, []).
enum_entry('ACTUATOR_CONFIGURATION', 'ACTUATOR_CONFIGURATION_NONE', 0, []).
enum_entry('ACTUATOR_CONFIGURATION', 'ACTUATOR_CONFIGURATION_BEEP', 1, []).
enum_entry('ACTUATOR_CONFIGURATION', 'ACTUATOR_CONFIGURATION_3D_MODE_ON', 2, []).
enum_entry('ACTUATOR_CONFIGURATION', 'ACTUATOR_CONFIGURATION_3D_MODE_OFF', 3, []).
enum_entry('ACTUATOR_CONFIGURATION', 'ACTUATOR_CONFIGURATION_SPIN_DIRECTION1', 4, []).
enum_entry('ACTUATOR_CONFIGURATION', 'ACTUATOR_CONFIGURATION_SPIN_DIRECTION2', 5, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_NONE', 0, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR1', 1, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR2', 2, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR3', 3, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR4', 4, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR5', 5, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR6', 6, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR7', 7, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR8', 8, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR9', 9, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR10', 10, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR11', 11, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR12', 12, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR13', 13, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR14', 14, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR15', 15, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR16', 16, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO1', 33, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO2', 34, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO3', 35, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO4', 36, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO5', 37, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO6', 38, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO7', 39, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO8', 40, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO9', 41, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO10', 42, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO11', 43, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO12', 44, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO13', 45, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO14', 46, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO15', 47, []).
enum_entry('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO16', 48, []).
enum_entry('AUTOTUNE_AXIS', 'AUTOTUNE_AXIS_DEFAULT', 0, []).
enum_entry('AUTOTUNE_AXIS', 'AUTOTUNE_AXIS_ROLL', 1, []).
enum_entry('AUTOTUNE_AXIS', 'AUTOTUNE_AXIS_PITCH', 2, []).
enum_entry('AUTOTUNE_AXIS', 'AUTOTUNE_AXIS_YAW', 4, []).
enum_entry('PREFLIGHT_STORAGE_PARAMETER_ACTION', 'PARAM_READ_PERSISTENT', 0, []).
enum_entry('PREFLIGHT_STORAGE_PARAMETER_ACTION', 'PARAM_WRITE_PERSISTENT', 1, []).
enum_entry('PREFLIGHT_STORAGE_PARAMETER_ACTION', 'PARAM_RESET_CONFIG_DEFAULT', 2, []).
enum_entry('PREFLIGHT_STORAGE_PARAMETER_ACTION', 'PARAM_RESET_SENSOR_DEFAULT', 3, []).
enum_entry('PREFLIGHT_STORAGE_PARAMETER_ACTION', 'PARAM_RESET_ALL_DEFAULT', 4, []).
enum_entry('PREFLIGHT_STORAGE_MISSION_ACTION', 'MISSION_READ_PERSISTENT', 0, []).
enum_entry('PREFLIGHT_STORAGE_MISSION_ACTION', 'MISSION_WRITE_PERSISTENT', 1, []).
enum_entry('PREFLIGHT_STORAGE_MISSION_ACTION', 'MISSION_RESET_DEFAULT', 2, []).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_WAYPOINT', 16, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_LOITER_UNLIM', 17, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_LOITER_TURNS', 18, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_LOITER_TIME', 19, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_RETURN_TO_LAUNCH', 20, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_LAND', 21, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF', 22, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_LAND_LOCAL', 23, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF_LOCAL', 24, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_FOLLOW', 25, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT', 30, [hasLocation=false, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_LOITER_TO_ALT', 31, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_FOLLOW', 32, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_FOLLOW_REPOSITION', 33, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_ORBIT', 34, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_ROI', 80, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_PATHPLANNING', 81, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_SPLINE_WAYPOINT', 82, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_VTOL_TAKEOFF', 84, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_VTOL_LAND', 85, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_GUIDED_ENABLE', 92, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_DELAY', 93, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_PAYLOAD_PLACE', 94, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_LAST', 95, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_CONDITION_DELAY', 112, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_CONDITION_CHANGE_ALT', 113, [hasLocation=false, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_CONDITION_DISTANCE', 114, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_CONDITION_YAW', 115, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_CONDITION_LAST', 159, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_MODE', 176, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_JUMP', 177, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_CHANGE_SPEED', 178, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_HOME', 179, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_PARAMETER', 180, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_RELAY', 181, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_REPEAT_RELAY', 182, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_SERVO', 183, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_REPEAT_SERVO', 184, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_FLIGHTTERMINATION', 185, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_CHANGE_ALTITUDE', 186, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_ACTUATOR', 187, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_LAND_START', 189, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_RALLY_LAND', 190, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_GO_AROUND', 191, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_REPOSITION', 192, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_PAUSE_CONTINUE', 193, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_REVERSE', 194, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_ROI_LOCATION', 195, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET', 196, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_ROI_NONE', 197, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_ROI_SYSID', 198, []).
enum_entry('MAV_CMD', 'MAV_CMD_DO_CONTROL_VIDEO', 200, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_ROI', 201, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONFIGURE', 202, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONTROL', 203, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONFIGURE', 204, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL', 205, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_DIST', 206, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_FENCE_ENABLE', 207, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_PARACHUTE', 208, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_MOTOR_TEST', 209, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_INVERTED_FLIGHT', 210, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_GRIPPER', 211, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_AUTOTUNE_ENABLE', 212, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_SET_YAW_SPEED', 213, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL', 214, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL_QUAT', 220, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_GUIDED_MASTER', 221, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_GUIDED_LIMITS', 222, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_ENGINE_CONTROL', 223, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_MISSION_CURRENT', 224, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_LAST', 240, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_PREFLIGHT_CALIBRATION', 241, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS', 242, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_PREFLIGHT_UAVCAN', 243, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_PREFLIGHT_STORAGE', 245, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN', 246, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_OVERRIDE_GOTO', 252, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_OBLIQUE_SURVEY', 260, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_MISSION_START', 300, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_ACTUATOR_TEST', 310, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_CONFIGURE_ACTUATOR', 311, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_COMPONENT_ARM_DISARM', 400, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_RUN_PREARM_CHECKS', 401, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_ILLUMINATOR_ON_OFF', 405, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_GET_HOME_POSITION', 410, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_INJECT_FAILURE', 420, []).
enum_entry('MAV_CMD', 'MAV_CMD_START_RX_PAIR', 500, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_GET_MESSAGE_INTERVAL', 510, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SET_MESSAGE_INTERVAL', 511, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_REQUEST_MESSAGE', 512, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_REQUEST_PROTOCOL_VERSION', 519, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES', 520, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_INFORMATION', 521, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_SETTINGS', 522, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_REQUEST_STORAGE_INFORMATION', 525, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_STORAGE_FORMAT', 526, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS', 527, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_REQUEST_FLIGHT_INFORMATION', 528, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_RESET_CAMERA_SETTINGS', 529, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SET_CAMERA_MODE', 530, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SET_CAMERA_ZOOM', 531, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SET_CAMERA_FOCUS', 532, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SET_STORAGE_USAGE', 533, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_JUMP_TAG', 600, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_JUMP_TAG', 601, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW', 1000, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE', 1001, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_IMAGE_START_CAPTURE', 2000, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_IMAGE_STOP_CAPTURE', 2001, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE', 2002, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_TRIGGER_CONTROL', 2003, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_CAMERA_TRACK_POINT', 2004, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_CAMERA_TRACK_RECTANGLE', 2005, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_CAMERA_STOP_TRACKING', 2010, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_VIDEO_START_CAPTURE', 2500, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_VIDEO_STOP_CAPTURE', 2501, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_VIDEO_START_STREAMING', 2502, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_VIDEO_STOP_STREAMING', 2503, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION', 2504, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_REQUEST_VIDEO_STREAM_STATUS', 2505, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_LOGGING_START', 2510, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_LOGGING_STOP', 2511, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_AIRFRAME_CONFIGURATION', 2520, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_CONTROL_HIGH_LATENCY', 2600, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_PANORAMA_CREATE', 2800, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_VTOL_TRANSITION', 3000, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_ARM_AUTHORIZATION_REQUEST', 3001, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SET_GUIDED_SUBMODE_STANDARD', 4000, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE', 4001, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_CONDITION_GATE', 4501, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_FENCE_RETURN_POINT', 5000, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION', 5001, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION', 5002, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION', 5003, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION', 5004, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_NAV_RALLY_POINT', 5100, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_UAVCAN_GET_NODE_INFO', 5200, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_ADSB_OUT_IDENT', 10001, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_PAYLOAD_PREPARE_DEPLOY', 30001, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL_DEPLOY', 30002, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_YAW', 42006, []).
enum_entry('MAV_CMD', 'MAV_CMD_DO_WINCH', 42600, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_1', 31000, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_2', 31001, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_3', 31002, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_4', 31003, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_5', 31004, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_SPATIAL_USER_1', 31005, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SPATIAL_USER_2', 31006, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SPATIAL_USER_3', 31007, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SPATIAL_USER_4', 31008, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SPATIAL_USER_5', 31009, [hasLocation=true, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_USER_1', 31010, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_USER_2', 31011, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_USER_3', 31012, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_USER_4', 31013, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_USER_5', 31014, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_CAN_FORWARD', 32000, [hasLocation=false, isDestination=false]).
enum_entry('MAV_DATA_STREAM', 'MAV_DATA_STREAM_ALL', 0, []).
enum_entry('MAV_DATA_STREAM', 'MAV_DATA_STREAM_RAW_SENSORS', 1, []).
enum_entry('MAV_DATA_STREAM', 'MAV_DATA_STREAM_EXTENDED_STATUS', 2, []).
enum_entry('MAV_DATA_STREAM', 'MAV_DATA_STREAM_RC_CHANNELS', 3, []).
enum_entry('MAV_DATA_STREAM', 'MAV_DATA_STREAM_RAW_CONTROLLER', 4, []).
enum_entry('MAV_DATA_STREAM', 'MAV_DATA_STREAM_POSITION', 6, []).
enum_entry('MAV_DATA_STREAM', 'MAV_DATA_STREAM_EXTRA1', 10, []).
enum_entry('MAV_DATA_STREAM', 'MAV_DATA_STREAM_EXTRA2', 11, []).
enum_entry('MAV_DATA_STREAM', 'MAV_DATA_STREAM_EXTRA3', 12, []).
enum_entry('MAV_ROI', 'MAV_ROI_NONE', 0, []).
enum_entry('MAV_ROI', 'MAV_ROI_WPNEXT', 1, []).
enum_entry('MAV_ROI', 'MAV_ROI_WPINDEX', 2, []).
enum_entry('MAV_ROI', 'MAV_ROI_LOCATION', 3, []).
enum_entry('MAV_ROI', 'MAV_ROI_TARGET', 4, []).
enum_entry('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_UINT8', 1, []).
enum_entry('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_INT8', 2, []).
enum_entry('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_UINT16', 3, []).
enum_entry('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_INT16', 4, []).
enum_entry('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_UINT32', 5, []).
enum_entry('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_INT32', 6, []).
enum_entry('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_UINT64', 7, []).
enum_entry('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_INT64', 8, []).
enum_entry('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_REAL32', 9, []).
enum_entry('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_REAL64', 10, []).
enum_entry('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_UINT8', 1, []).
enum_entry('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_INT8', 2, []).
enum_entry('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_UINT16', 3, []).
enum_entry('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_INT16', 4, []).
enum_entry('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_UINT32', 5, []).
enum_entry('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_INT32', 6, []).
enum_entry('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_UINT64', 7, []).
enum_entry('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_INT64', 8, []).
enum_entry('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_REAL32', 9, []).
enum_entry('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_REAL64', 10, []).
enum_entry('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_CUSTOM', 11, []).
enum_entry('MAV_RESULT', 'MAV_RESULT_ACCEPTED', 0, []).
enum_entry('MAV_RESULT', 'MAV_RESULT_TEMPORARILY_REJECTED', 1, []).
enum_entry('MAV_RESULT', 'MAV_RESULT_DENIED', 2, []).
enum_entry('MAV_RESULT', 'MAV_RESULT_UNSUPPORTED', 3, []).
enum_entry('MAV_RESULT', 'MAV_RESULT_FAILED', 4, []).
enum_entry('MAV_RESULT', 'MAV_RESULT_IN_PROGRESS', 5, []).
enum_entry('MAV_RESULT', 'MAV_RESULT_CANCELLED', 6, []).
enum_entry('MAV_RESULT', 'MAV_RESULT_COMMAND_LONG_ONLY', 7, []).
enum_entry('MAV_RESULT', 'MAV_RESULT_COMMAND_INT_ONLY', 8, []).
enum_entry('MAV_RESULT', 'MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME', 9, []).
enum_entry('MAV_MISSION_RESULT', 'MAV_MISSION_ACCEPTED', 0, []).
enum_entry('MAV_MISSION_RESULT', 'MAV_MISSION_ERROR', 1, []).
enum_entry('MAV_MISSION_RESULT', 'MAV_MISSION_UNSUPPORTED_FRAME', 2, []).
enum_entry('MAV_MISSION_RESULT', 'MAV_MISSION_UNSUPPORTED', 3, []).
enum_entry('MAV_MISSION_RESULT', 'MAV_MISSION_NO_SPACE', 4, []).
enum_entry('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID', 5, []).
enum_entry('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID_PARAM1', 6, []).
enum_entry('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID_PARAM2', 7, []).
enum_entry('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID_PARAM3', 8, []).
enum_entry('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID_PARAM4', 9, []).
enum_entry('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID_PARAM5_X', 10, []).
enum_entry('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID_PARAM6_Y', 11, []).
enum_entry('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID_PARAM7', 12, []).
enum_entry('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID_SEQUENCE', 13, []).
enum_entry('MAV_MISSION_RESULT', 'MAV_MISSION_DENIED', 14, []).
enum_entry('MAV_MISSION_RESULT', 'MAV_MISSION_OPERATION_CANCELLED', 15, []).
enum_entry('MAV_SEVERITY', 'MAV_SEVERITY_EMERGENCY', 0, []).
enum_entry('MAV_SEVERITY', 'MAV_SEVERITY_ALERT', 1, []).
enum_entry('MAV_SEVERITY', 'MAV_SEVERITY_CRITICAL', 2, []).
enum_entry('MAV_SEVERITY', 'MAV_SEVERITY_ERROR', 3, []).
enum_entry('MAV_SEVERITY', 'MAV_SEVERITY_WARNING', 4, []).
enum_entry('MAV_SEVERITY', 'MAV_SEVERITY_NOTICE', 5, []).
enum_entry('MAV_SEVERITY', 'MAV_SEVERITY_INFO', 6, []).
enum_entry('MAV_SEVERITY', 'MAV_SEVERITY_DEBUG', 7, []).
enum_entry('MAV_POWER_STATUS', 'MAV_POWER_STATUS_BRICK_VALID', 1, []).
enum_entry('MAV_POWER_STATUS', 'MAV_POWER_STATUS_SERVO_VALID', 2, []).
enum_entry('MAV_POWER_STATUS', 'MAV_POWER_STATUS_USB_CONNECTED', 4, []).
enum_entry('MAV_POWER_STATUS', 'MAV_POWER_STATUS_PERIPH_OVERCURRENT', 8, []).
enum_entry('MAV_POWER_STATUS', 'MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT', 16, []).
enum_entry('MAV_POWER_STATUS', 'MAV_POWER_STATUS_CHANGED', 32, []).
enum_entry('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_DEV_TELEM1', 0, []).
enum_entry('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_DEV_TELEM2', 1, []).
enum_entry('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_DEV_GPS1', 2, []).
enum_entry('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_DEV_GPS2', 3, []).
enum_entry('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_DEV_SHELL', 10, []).
enum_entry('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL0', 100, []).
enum_entry('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL1', 101, []).
enum_entry('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL2', 102, []).
enum_entry('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL3', 103, []).
enum_entry('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL4', 104, []).
enum_entry('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL5', 105, []).
enum_entry('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL6', 106, []).
enum_entry('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL7', 107, []).
enum_entry('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL8', 108, []).
enum_entry('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL9', 109, []).
enum_entry('SERIAL_CONTROL_FLAG', 'SERIAL_CONTROL_FLAG_REPLY', 1, []).
enum_entry('SERIAL_CONTROL_FLAG', 'SERIAL_CONTROL_FLAG_RESPOND', 2, []).
enum_entry('SERIAL_CONTROL_FLAG', 'SERIAL_CONTROL_FLAG_EXCLUSIVE', 4, []).
enum_entry('SERIAL_CONTROL_FLAG', 'SERIAL_CONTROL_FLAG_BLOCKING', 8, []).
enum_entry('SERIAL_CONTROL_FLAG', 'SERIAL_CONTROL_FLAG_MULTI', 16, []).
enum_entry('MAV_DISTANCE_SENSOR', 'MAV_DISTANCE_SENSOR_LASER', 0, []).
enum_entry('MAV_DISTANCE_SENSOR', 'MAV_DISTANCE_SENSOR_ULTRASOUND', 1, []).
enum_entry('MAV_DISTANCE_SENSOR', 'MAV_DISTANCE_SENSOR_INFRARED', 2, []).
enum_entry('MAV_DISTANCE_SENSOR', 'MAV_DISTANCE_SENSOR_RADAR', 3, []).
enum_entry('MAV_DISTANCE_SENSOR', 'MAV_DISTANCE_SENSOR_UNKNOWN', 4, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_NONE', 0, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_YAW_45', 1, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_YAW_90', 2, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_YAW_135', 3, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_YAW_180', 4, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_YAW_225', 5, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_YAW_270', 6, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_YAW_315', 7, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180', 8, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180_YAW_45', 9, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180_YAW_90', 10, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180_YAW_135', 11, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_PITCH_180', 12, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180_YAW_225', 13, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180_YAW_270', 14, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180_YAW_315', 15, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90', 16, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_YAW_45', 17, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_YAW_90', 18, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_YAW_135', 19, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_270', 20, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_270_YAW_45', 21, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_270_YAW_90', 22, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_270_YAW_135', 23, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_PITCH_90', 24, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_PITCH_270', 25, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_PITCH_180_YAW_90', 26, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_PITCH_180_YAW_270', 27, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_90', 28, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180_PITCH_90', 29, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_270_PITCH_90', 30, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_180', 31, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_270_PITCH_180', 32, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_270', 33, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180_PITCH_270', 34, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_270_PITCH_270', 35, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90', 36, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_YAW_270', 37, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293', 38, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_PITCH_315', 39, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_315', 40, []).
enum_entry('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_CUSTOM', 100, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT', 1, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT', 2, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_MISSION_INT', 4, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_COMMAND_INT', 8, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE', 16, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_FTP', 32, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET', 64, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED', 128, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT', 256, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_TERRAIN', 512, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_RESERVED3', 1024, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION', 2048, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION', 4096, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_MAVLINK2', 8192, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_MISSION_FENCE', 16384, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_MISSION_RALLY', 32768, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_RESERVED2', 65536, []).
enum_entry('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST', 131072, []).
enum_entry('MAV_MISSION_TYPE', 'MAV_MISSION_TYPE_MISSION', 0, []).
enum_entry('MAV_MISSION_TYPE', 'MAV_MISSION_TYPE_FENCE', 1, []).
enum_entry('MAV_MISSION_TYPE', 'MAV_MISSION_TYPE_RALLY', 2, []).
enum_entry('MAV_MISSION_TYPE', 'MAV_MISSION_TYPE_ALL', 255, []).
enum_entry('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_UNKNOWN', 0, []).
enum_entry('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_NAIVE', 1, []).
enum_entry('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_VISION', 2, []).
enum_entry('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_VIO', 3, []).
enum_entry('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_GPS', 4, []).
enum_entry('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_GPS_INS', 5, []).
enum_entry('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_MOCAP', 6, []).
enum_entry('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_LIDAR', 7, []).
enum_entry('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_AUTOPILOT', 8, []).
enum_entry('MAV_BATTERY_TYPE', 'MAV_BATTERY_TYPE_UNKNOWN', 0, []).
enum_entry('MAV_BATTERY_TYPE', 'MAV_BATTERY_TYPE_LIPO', 1, []).
enum_entry('MAV_BATTERY_TYPE', 'MAV_BATTERY_TYPE_LIFE', 2, []).
enum_entry('MAV_BATTERY_TYPE', 'MAV_BATTERY_TYPE_LION', 3, []).
enum_entry('MAV_BATTERY_TYPE', 'MAV_BATTERY_TYPE_NIMH', 4, []).
enum_entry('MAV_BATTERY_FUNCTION', 'MAV_BATTERY_FUNCTION_UNKNOWN', 0, []).
enum_entry('MAV_BATTERY_FUNCTION', 'MAV_BATTERY_FUNCTION_ALL', 1, []).
enum_entry('MAV_BATTERY_FUNCTION', 'MAV_BATTERY_FUNCTION_PROPULSION', 2, []).
enum_entry('MAV_BATTERY_FUNCTION', 'MAV_BATTERY_FUNCTION_AVIONICS', 3, []).
enum_entry('MAV_BATTERY_FUNCTION', 'MAV_BATTERY_FUNCTION_PAYLOAD', 4, []).
enum_entry('MAV_BATTERY_CHARGE_STATE', 'MAV_BATTERY_CHARGE_STATE_UNDEFINED', 0, []).
enum_entry('MAV_BATTERY_CHARGE_STATE', 'MAV_BATTERY_CHARGE_STATE_OK', 1, []).
enum_entry('MAV_BATTERY_CHARGE_STATE', 'MAV_BATTERY_CHARGE_STATE_LOW', 2, []).
enum_entry('MAV_BATTERY_CHARGE_STATE', 'MAV_BATTERY_CHARGE_STATE_CRITICAL', 3, []).
enum_entry('MAV_BATTERY_CHARGE_STATE', 'MAV_BATTERY_CHARGE_STATE_EMERGENCY', 4, []).
enum_entry('MAV_BATTERY_CHARGE_STATE', 'MAV_BATTERY_CHARGE_STATE_FAILED', 5, []).
enum_entry('MAV_BATTERY_CHARGE_STATE', 'MAV_BATTERY_CHARGE_STATE_UNHEALTHY', 6, []).
enum_entry('MAV_BATTERY_CHARGE_STATE', 'MAV_BATTERY_CHARGE_STATE_CHARGING', 7, []).
enum_entry('MAV_BATTERY_MODE', 'MAV_BATTERY_MODE_UNKNOWN', 0, []).
enum_entry('MAV_BATTERY_MODE', 'MAV_BATTERY_MODE_AUTO_DISCHARGING', 1, []).
enum_entry('MAV_BATTERY_MODE', 'MAV_BATTERY_MODE_HOT_SWAP', 2, []).
enum_entry('MAV_BATTERY_FAULT', 'MAV_BATTERY_FAULT_DEEP_DISCHARGE', 1, []).
enum_entry('MAV_BATTERY_FAULT', 'MAV_BATTERY_FAULT_SPIKES', 2, []).
enum_entry('MAV_BATTERY_FAULT', 'MAV_BATTERY_FAULT_CELL_FAIL', 4, []).
enum_entry('MAV_BATTERY_FAULT', 'MAV_BATTERY_FAULT_OVER_CURRENT', 8, []).
enum_entry('MAV_BATTERY_FAULT', 'MAV_BATTERY_FAULT_OVER_TEMPERATURE', 16, []).
enum_entry('MAV_BATTERY_FAULT', 'MAV_BATTERY_FAULT_UNDER_TEMPERATURE', 32, []).
enum_entry('MAV_BATTERY_FAULT', 'MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE', 64, []).
enum_entry('MAV_BATTERY_FAULT', 'MAV_BATTERY_FAULT_INCOMPATIBLE_FIRMWARE', 128, []).
enum_entry('MAV_BATTERY_FAULT', 'BATTERY_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION', 256, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_OFF', 1, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_READY', 2, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_GENERATING', 4, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_CHARGING', 8, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER', 16, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_MAXPOWER', 32, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING', 64, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_OVERTEMP_FAULT', 128, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING', 256, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT', 512, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_FAULT', 1024, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_POWERSOURCE_FAULT', 2048, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_COMMUNICATION_WARNING', 4096, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_COOLING_WARNING', 8192, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_POWER_RAIL_FAULT', 16384, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT', 32768, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT', 65536, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_OVERVOLTAGE_FAULT', 131072, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_BATTERY_UNDERVOLT_FAULT', 262144, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_START_INHIBITED', 524288, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED', 1048576, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_WARMING_UP', 2097152, []).
enum_entry('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_IDLE', 4194304, []).
enum_entry('MAV_VTOL_STATE', 'MAV_VTOL_STATE_UNDEFINED', 0, []).
enum_entry('MAV_VTOL_STATE', 'MAV_VTOL_STATE_TRANSITION_TO_FW', 1, []).
enum_entry('MAV_VTOL_STATE', 'MAV_VTOL_STATE_TRANSITION_TO_MC', 2, []).
enum_entry('MAV_VTOL_STATE', 'MAV_VTOL_STATE_MC', 3, []).
enum_entry('MAV_VTOL_STATE', 'MAV_VTOL_STATE_FW', 4, []).
enum_entry('MAV_LANDED_STATE', 'MAV_LANDED_STATE_UNDEFINED', 0, []).
enum_entry('MAV_LANDED_STATE', 'MAV_LANDED_STATE_ON_GROUND', 1, []).
enum_entry('MAV_LANDED_STATE', 'MAV_LANDED_STATE_IN_AIR', 2, []).
enum_entry('MAV_LANDED_STATE', 'MAV_LANDED_STATE_TAKEOFF', 3, []).
enum_entry('MAV_LANDED_STATE', 'MAV_LANDED_STATE_LANDING', 4, []).
enum_entry('ADSB_ALTITUDE_TYPE', 'ADSB_ALTITUDE_TYPE_PRESSURE_QNH', 0, []).
enum_entry('ADSB_ALTITUDE_TYPE', 'ADSB_ALTITUDE_TYPE_GEOMETRIC', 1, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_NO_INFO', 0, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_LIGHT', 1, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_SMALL', 2, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_LARGE', 3, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE', 4, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_HEAVY', 5, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_HIGHLY_MANUV', 6, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_ROTOCRAFT', 7, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_UNASSIGNED', 8, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_GLIDER', 9, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_LIGHTER_AIR', 10, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_PARACHUTE', 11, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_ULTRA_LIGHT', 12, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_UNASSIGNED2', 13, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_UAV', 14, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_SPACE', 15, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_UNASSGINED3', 16, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_EMERGENCY_SURFACE', 17, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_SERVICE_SURFACE', 18, []).
enum_entry('ADSB_EMITTER_TYPE', 'ADSB_EMITTER_TYPE_POINT_OBSTACLE', 19, []).
enum_entry('ADSB_FLAGS', 'ADSB_FLAGS_VALID_COORDS', 1, []).
enum_entry('ADSB_FLAGS', 'ADSB_FLAGS_VALID_ALTITUDE', 2, []).
enum_entry('ADSB_FLAGS', 'ADSB_FLAGS_VALID_HEADING', 4, []).
enum_entry('ADSB_FLAGS', 'ADSB_FLAGS_VALID_VELOCITY', 8, []).
enum_entry('ADSB_FLAGS', 'ADSB_FLAGS_VALID_CALLSIGN', 16, []).
enum_entry('ADSB_FLAGS', 'ADSB_FLAGS_VALID_SQUAWK', 32, []).
enum_entry('ADSB_FLAGS', 'ADSB_FLAGS_SIMULATED', 64, []).
enum_entry('ADSB_FLAGS', 'ADSB_FLAGS_VERTICAL_VELOCITY_VALID', 128, []).
enum_entry('ADSB_FLAGS', 'ADSB_FLAGS_BARO_VALID', 256, []).
enum_entry('ADSB_FLAGS', 'ADSB_FLAGS_SOURCE_UAT', 32768, []).
enum_entry('MAV_DO_REPOSITION_FLAGS', 'MAV_DO_REPOSITION_FLAGS_CHANGE_MODE', 1, []).
enum_entry('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_ATTITUDE', 1, []).
enum_entry('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_VELOCITY_HORIZ', 2, []).
enum_entry('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_VELOCITY_VERT', 4, []).
enum_entry('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_POS_HORIZ_REL', 8, []).
enum_entry('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_POS_HORIZ_ABS', 16, []).
enum_entry('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_POS_VERT_ABS', 32, []).
enum_entry('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_POS_VERT_AGL', 64, []).
enum_entry('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_CONST_POS_MODE', 128, []).
enum_entry('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_PRED_POS_HORIZ_REL', 256, []).
enum_entry('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_PRED_POS_HORIZ_ABS', 512, []).
enum_entry('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_GPS_GLITCH', 1024, []).
enum_entry('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_ACCEL_ERROR', 2048, []).
enum_entry('MOTOR_TEST_ORDER', 'MOTOR_TEST_ORDER_DEFAULT', 0, []).
enum_entry('MOTOR_TEST_ORDER', 'MOTOR_TEST_ORDER_SEQUENCE', 1, []).
enum_entry('MOTOR_TEST_ORDER', 'MOTOR_TEST_ORDER_BOARD', 2, []).
enum_entry('MOTOR_TEST_THROTTLE_TYPE', 'MOTOR_TEST_THROTTLE_PERCENT', 0, []).
enum_entry('MOTOR_TEST_THROTTLE_TYPE', 'MOTOR_TEST_THROTTLE_PWM', 1, []).
enum_entry('MOTOR_TEST_THROTTLE_TYPE', 'MOTOR_TEST_THROTTLE_PILOT', 2, []).
enum_entry('MOTOR_TEST_THROTTLE_TYPE', 'MOTOR_TEST_COMPASS_CAL', 3, []).
enum_entry('GPS_INPUT_IGNORE_FLAGS', 'GPS_INPUT_IGNORE_FLAG_ALT', 1, []).
enum_entry('GPS_INPUT_IGNORE_FLAGS', 'GPS_INPUT_IGNORE_FLAG_HDOP', 2, []).
enum_entry('GPS_INPUT_IGNORE_FLAGS', 'GPS_INPUT_IGNORE_FLAG_VDOP', 4, []).
enum_entry('GPS_INPUT_IGNORE_FLAGS', 'GPS_INPUT_IGNORE_FLAG_VEL_HORIZ', 8, []).
enum_entry('GPS_INPUT_IGNORE_FLAGS', 'GPS_INPUT_IGNORE_FLAG_VEL_VERT', 16, []).
enum_entry('GPS_INPUT_IGNORE_FLAGS', 'GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY', 32, []).
enum_entry('GPS_INPUT_IGNORE_FLAGS', 'GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY', 64, []).
enum_entry('GPS_INPUT_IGNORE_FLAGS', 'GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY', 128, []).
enum_entry('MAV_COLLISION_ACTION', 'MAV_COLLISION_ACTION_NONE', 0, []).
enum_entry('MAV_COLLISION_ACTION', 'MAV_COLLISION_ACTION_REPORT', 1, []).
enum_entry('MAV_COLLISION_ACTION', 'MAV_COLLISION_ACTION_ASCEND_OR_DESCEND', 2, []).
enum_entry('MAV_COLLISION_ACTION', 'MAV_COLLISION_ACTION_MOVE_HORIZONTALLY', 3, []).
enum_entry('MAV_COLLISION_ACTION', 'MAV_COLLISION_ACTION_MOVE_PERPENDICULAR', 4, []).
enum_entry('MAV_COLLISION_ACTION', 'MAV_COLLISION_ACTION_RTL', 5, []).
enum_entry('MAV_COLLISION_ACTION', 'MAV_COLLISION_ACTION_HOVER', 6, []).
enum_entry('MAV_COLLISION_THREAT_LEVEL', 'MAV_COLLISION_THREAT_LEVEL_NONE', 0, []).
enum_entry('MAV_COLLISION_THREAT_LEVEL', 'MAV_COLLISION_THREAT_LEVEL_LOW', 1, []).
enum_entry('MAV_COLLISION_THREAT_LEVEL', 'MAV_COLLISION_THREAT_LEVEL_HIGH', 2, []).
enum_entry('MAV_COLLISION_SRC', 'MAV_COLLISION_SRC_ADSB', 0, []).
enum_entry('MAV_COLLISION_SRC', 'MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT', 1, []).
enum_entry('GPS_FIX_TYPE', 'GPS_FIX_TYPE_NO_GPS', 0, []).
enum_entry('GPS_FIX_TYPE', 'GPS_FIX_TYPE_NO_FIX', 1, []).
enum_entry('GPS_FIX_TYPE', 'GPS_FIX_TYPE_2D_FIX', 2, []).
enum_entry('GPS_FIX_TYPE', 'GPS_FIX_TYPE_3D_FIX', 3, []).
enum_entry('GPS_FIX_TYPE', 'GPS_FIX_TYPE_DGPS', 4, []).
enum_entry('GPS_FIX_TYPE', 'GPS_FIX_TYPE_RTK_FLOAT', 5, []).
enum_entry('GPS_FIX_TYPE', 'GPS_FIX_TYPE_RTK_FIXED', 6, []).
enum_entry('GPS_FIX_TYPE', 'GPS_FIX_TYPE_STATIC', 7, []).
enum_entry('GPS_FIX_TYPE', 'GPS_FIX_TYPE_PPP', 8, []).
enum_entry('RTK_BASELINE_COORDINATE_SYSTEM', 'RTK_BASELINE_COORDINATE_SYSTEM_ECEF', 0, []).
enum_entry('RTK_BASELINE_COORDINATE_SYSTEM', 'RTK_BASELINE_COORDINATE_SYSTEM_NED', 1, []).
enum_entry('LANDING_TARGET_TYPE', 'LANDING_TARGET_TYPE_LIGHT_BEACON', 0, []).
enum_entry('LANDING_TARGET_TYPE', 'LANDING_TARGET_TYPE_RADIO_BEACON', 1, []).
enum_entry('LANDING_TARGET_TYPE', 'LANDING_TARGET_TYPE_VISION_FIDUCIAL', 2, []).
enum_entry('LANDING_TARGET_TYPE', 'LANDING_TARGET_TYPE_VISION_OTHER', 3, []).
enum_entry('VTOL_TRANSITION_HEADING', 'VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT', 0, []).
enum_entry('VTOL_TRANSITION_HEADING', 'VTOL_TRANSITION_HEADING_NEXT_WAYPOINT', 1, []).
enum_entry('VTOL_TRANSITION_HEADING', 'VTOL_TRANSITION_HEADING_TAKEOFF', 2, []).
enum_entry('VTOL_TRANSITION_HEADING', 'VTOL_TRANSITION_HEADING_SPECIFIED', 3, []).
enum_entry('VTOL_TRANSITION_HEADING', 'VTOL_TRANSITION_HEADING_ANY', 4, []).
enum_entry('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_CAPTURE_VIDEO', 1, []).
enum_entry('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_CAPTURE_IMAGE', 2, []).
enum_entry('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_HAS_MODES', 4, []).
enum_entry('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE', 8, []).
enum_entry('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE', 16, []).
enum_entry('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE', 32, []).
enum_entry('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM', 64, []).
enum_entry('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS', 128, []).
enum_entry('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM', 256, []).
enum_entry('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_HAS_TRACKING_POINT', 512, []).
enum_entry('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE', 1024, []).
enum_entry('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS', 2048, []).
enum_entry('VIDEO_STREAM_STATUS_FLAGS', 'VIDEO_STREAM_STATUS_FLAGS_RUNNING', 1, []).
enum_entry('VIDEO_STREAM_STATUS_FLAGS', 'VIDEO_STREAM_STATUS_FLAGS_THERMAL', 2, []).
enum_entry('VIDEO_STREAM_TYPE', 'VIDEO_STREAM_TYPE_RTSP', 0, []).
enum_entry('VIDEO_STREAM_TYPE', 'VIDEO_STREAM_TYPE_RTPUDP', 1, []).
enum_entry('VIDEO_STREAM_TYPE', 'VIDEO_STREAM_TYPE_TCP_MPEG', 2, []).
enum_entry('VIDEO_STREAM_TYPE', 'VIDEO_STREAM_TYPE_MPEG_TS_H264', 3, []).
enum_entry('CAMERA_TRACKING_STATUS_FLAGS', 'CAMERA_TRACKING_STATUS_FLAGS_IDLE', 0, []).
enum_entry('CAMERA_TRACKING_STATUS_FLAGS', 'CAMERA_TRACKING_STATUS_FLAGS_ACTIVE', 1, []).
enum_entry('CAMERA_TRACKING_STATUS_FLAGS', 'CAMERA_TRACKING_STATUS_FLAGS_ERROR', 2, []).
enum_entry('CAMERA_TRACKING_MODE', 'CAMERA_TRACKING_MODE_NONE', 0, []).
enum_entry('CAMERA_TRACKING_MODE', 'CAMERA_TRACKING_MODE_POINT', 1, []).
enum_entry('CAMERA_TRACKING_MODE', 'CAMERA_TRACKING_MODE_RECTANGLE', 2, []).
enum_entry('CAMERA_TRACKING_TARGET_DATA', 'CAMERA_TRACKING_TARGET_DATA_NONE', 0, []).
enum_entry('CAMERA_TRACKING_TARGET_DATA', 'CAMERA_TRACKING_TARGET_DATA_EMBEDDED', 1, []).
enum_entry('CAMERA_TRACKING_TARGET_DATA', 'CAMERA_TRACKING_TARGET_DATA_RENDERED', 2, []).
enum_entry('CAMERA_TRACKING_TARGET_DATA', 'CAMERA_TRACKING_TARGET_DATA_IN_STATUS', 4, []).
enum_entry('CAMERA_ZOOM_TYPE', 'ZOOM_TYPE_STEP', 0, []).
enum_entry('CAMERA_ZOOM_TYPE', 'ZOOM_TYPE_CONTINUOUS', 1, []).
enum_entry('CAMERA_ZOOM_TYPE', 'ZOOM_TYPE_RANGE', 2, []).
enum_entry('CAMERA_ZOOM_TYPE', 'ZOOM_TYPE_FOCAL_LENGTH', 3, []).
enum_entry('CAMERA_ZOOM_TYPE', 'ZOOM_TYPE_HORIZONTAL_FOV', 4, []).
enum_entry('SET_FOCUS_TYPE', 'FOCUS_TYPE_STEP', 0, []).
enum_entry('SET_FOCUS_TYPE', 'FOCUS_TYPE_CONTINUOUS', 1, []).
enum_entry('SET_FOCUS_TYPE', 'FOCUS_TYPE_RANGE', 2, []).
enum_entry('SET_FOCUS_TYPE', 'FOCUS_TYPE_METERS', 3, []).
enum_entry('SET_FOCUS_TYPE', 'FOCUS_TYPE_AUTO', 4, []).
enum_entry('SET_FOCUS_TYPE', 'FOCUS_TYPE_AUTO_SINGLE', 5, []).
enum_entry('SET_FOCUS_TYPE', 'FOCUS_TYPE_AUTO_CONTINUOUS', 6, []).
enum_entry('PARAM_ACK', 'PARAM_ACK_ACCEPTED', 0, []).
enum_entry('PARAM_ACK', 'PARAM_ACK_VALUE_UNSUPPORTED', 1, []).
enum_entry('PARAM_ACK', 'PARAM_ACK_FAILED', 2, []).
enum_entry('PARAM_ACK', 'PARAM_ACK_IN_PROGRESS', 3, []).
enum_entry('CAMERA_MODE', 'CAMERA_MODE_IMAGE', 0, []).
enum_entry('CAMERA_MODE', 'CAMERA_MODE_VIDEO', 1, []).
enum_entry('CAMERA_MODE', 'CAMERA_MODE_IMAGE_SURVEY', 2, []).
enum_entry('MAV_ARM_AUTH_DENIED_REASON', 'MAV_ARM_AUTH_DENIED_REASON_GENERIC', 0, []).
enum_entry('MAV_ARM_AUTH_DENIED_REASON', 'MAV_ARM_AUTH_DENIED_REASON_NONE', 1, []).
enum_entry('MAV_ARM_AUTH_DENIED_REASON', 'MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT', 2, []).
enum_entry('MAV_ARM_AUTH_DENIED_REASON', 'MAV_ARM_AUTH_DENIED_REASON_TIMEOUT', 3, []).
enum_entry('MAV_ARM_AUTH_DENIED_REASON', 'MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE', 4, []).
enum_entry('MAV_ARM_AUTH_DENIED_REASON', 'MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER', 5, []).
enum_entry('RC_TYPE', 'RC_TYPE_SPEKTRUM_DSM2', 0, []).
enum_entry('RC_TYPE', 'RC_TYPE_SPEKTRUM_DSMX', 1, []).
enum_entry('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_X_IGNORE', 1, []).
enum_entry('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_Y_IGNORE', 2, []).
enum_entry('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_Z_IGNORE', 4, []).
enum_entry('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_VX_IGNORE', 8, []).
enum_entry('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_VY_IGNORE', 16, []).
enum_entry('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_VZ_IGNORE', 32, []).
enum_entry('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_AX_IGNORE', 64, []).
enum_entry('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_AY_IGNORE', 128, []).
enum_entry('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_AZ_IGNORE', 256, []).
enum_entry('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_FORCE_SET', 512, []).
enum_entry('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_YAW_IGNORE', 1024, []).
enum_entry('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE', 2048, []).
enum_entry('ATTITUDE_TARGET_TYPEMASK', 'ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE', 1, []).
enum_entry('ATTITUDE_TARGET_TYPEMASK', 'ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE', 2, []).
enum_entry('ATTITUDE_TARGET_TYPEMASK', 'ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE', 4, []).
enum_entry('ATTITUDE_TARGET_TYPEMASK', 'ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET', 32, []).
enum_entry('ATTITUDE_TARGET_TYPEMASK', 'ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE', 64, []).
enum_entry('ATTITUDE_TARGET_TYPEMASK', 'ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE', 128, []).
enum_entry('UTM_FLIGHT_STATE', 'UTM_FLIGHT_STATE_UNKNOWN', 1, []).
enum_entry('UTM_FLIGHT_STATE', 'UTM_FLIGHT_STATE_GROUND', 2, []).
enum_entry('UTM_FLIGHT_STATE', 'UTM_FLIGHT_STATE_AIRBORNE', 3, []).
enum_entry('UTM_FLIGHT_STATE', 'UTM_FLIGHT_STATE_EMERGENCY', 16, []).
enum_entry('UTM_FLIGHT_STATE', 'UTM_FLIGHT_STATE_NOCTRL', 32, []).
enum_entry('UTM_DATA_AVAIL_FLAGS', 'UTM_DATA_AVAIL_FLAGS_TIME_VALID', 1, []).
enum_entry('UTM_DATA_AVAIL_FLAGS', 'UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE', 2, []).
enum_entry('UTM_DATA_AVAIL_FLAGS', 'UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE', 4, []).
enum_entry('UTM_DATA_AVAIL_FLAGS', 'UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE', 8, []).
enum_entry('UTM_DATA_AVAIL_FLAGS', 'UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE', 16, []).
enum_entry('UTM_DATA_AVAIL_FLAGS', 'UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE', 32, []).
enum_entry('UTM_DATA_AVAIL_FLAGS', 'UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE', 64, []).
enum_entry('UTM_DATA_AVAIL_FLAGS', 'UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE', 128, []).
enum_entry('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_UNKNOWN', 0, []).
enum_entry('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_FAILED', 1, []).
enum_entry('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_INITIALIZING', 2, []).
enum_entry('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_LOCKED', 3, []).
enum_entry('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_DISABLED', 4, []).
enum_entry('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_DISABLING', 5, []).
enum_entry('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_ENABLING', 6, []).
enum_entry('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_ENABLED', 7, []).
enum_entry('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_SEARCHING', 8, []).
enum_entry('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_REGISTERED', 9, []).
enum_entry('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_DISCONNECTING', 10, []).
enum_entry('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_CONNECTING', 11, []).
enum_entry('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_CONNECTED', 12, []).
enum_entry('CELLULAR_NETWORK_FAILED_REASON', 'CELLULAR_NETWORK_FAILED_REASON_NONE', 0, []).
enum_entry('CELLULAR_NETWORK_FAILED_REASON', 'CELLULAR_NETWORK_FAILED_REASON_UNKNOWN', 1, []).
enum_entry('CELLULAR_NETWORK_FAILED_REASON', 'CELLULAR_NETWORK_FAILED_REASON_SIM_MISSING', 2, []).
enum_entry('CELLULAR_NETWORK_FAILED_REASON', 'CELLULAR_NETWORK_FAILED_REASON_SIM_ERROR', 3, []).
enum_entry('CELLULAR_NETWORK_RADIO_TYPE', 'CELLULAR_NETWORK_RADIO_TYPE_NONE', 0, []).
enum_entry('CELLULAR_NETWORK_RADIO_TYPE', 'CELLULAR_NETWORK_RADIO_TYPE_GSM', 1, []).
enum_entry('CELLULAR_NETWORK_RADIO_TYPE', 'CELLULAR_NETWORK_RADIO_TYPE_CDMA', 2, []).
enum_entry('CELLULAR_NETWORK_RADIO_TYPE', 'CELLULAR_NETWORK_RADIO_TYPE_WCDMA', 3, []).
enum_entry('CELLULAR_NETWORK_RADIO_TYPE', 'CELLULAR_NETWORK_RADIO_TYPE_LTE', 4, []).
enum_entry('PRECISION_LAND_MODE', 'PRECISION_LAND_MODE_DISABLED', 0, []).
enum_entry('PRECISION_LAND_MODE', 'PRECISION_LAND_MODE_OPPORTUNISTIC', 1, []).
enum_entry('PRECISION_LAND_MODE', 'PRECISION_LAND_MODE_REQUIRED', 2, []).
enum_entry('PARACHUTE_ACTION', 'PARACHUTE_DISABLE', 0, []).
enum_entry('PARACHUTE_ACTION', 'PARACHUTE_ENABLE', 1, []).
enum_entry('PARACHUTE_ACTION', 'PARACHUTE_RELEASE', 2, []).
enum_entry('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN', 0, []).
enum_entry('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0', 200, []).
enum_entry('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1', 201, []).
enum_entry('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2', 202, []).
enum_entry('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3', 203, []).
enum_entry('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4', 204, []).
enum_entry('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5', 205, []).
enum_entry('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6', 206, []).
enum_entry('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7', 207, []).
enum_entry('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8', 208, []).
enum_entry('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9', 209, []).
enum_entry('MAV_ODID_ID_TYPE', 'MAV_ODID_ID_TYPE_NONE', 0, []).
enum_entry('MAV_ODID_ID_TYPE', 'MAV_ODID_ID_TYPE_SERIAL_NUMBER', 1, []).
enum_entry('MAV_ODID_ID_TYPE', 'MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID', 2, []).
enum_entry('MAV_ODID_ID_TYPE', 'MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID', 3, []).
enum_entry('MAV_ODID_ID_TYPE', 'MAV_ODID_ID_TYPE_SPECIFIC_SESSION_ID', 4, []).
enum_entry('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_NONE', 0, []).
enum_entry('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_AEROPLANE', 1, []).
enum_entry('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR', 2, []).
enum_entry('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_GYROPLANE', 3, []).
enum_entry('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_HYBRID_LIFT', 4, []).
enum_entry('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_ORNITHOPTER', 5, []).
enum_entry('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_GLIDER', 6, []).
enum_entry('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_KITE', 7, []).
enum_entry('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_FREE_BALLOON', 8, []).
enum_entry('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_CAPTIVE_BALLOON', 9, []).
enum_entry('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_AIRSHIP', 10, []).
enum_entry('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE', 11, []).
enum_entry('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_ROCKET', 12, []).
enum_entry('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT', 13, []).
enum_entry('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_GROUND_OBSTACLE', 14, []).
enum_entry('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_OTHER', 15, []).
enum_entry('MAV_ODID_STATUS', 'MAV_ODID_STATUS_UNDECLARED', 0, []).
enum_entry('MAV_ODID_STATUS', 'MAV_ODID_STATUS_GROUND', 1, []).
enum_entry('MAV_ODID_STATUS', 'MAV_ODID_STATUS_AIRBORNE', 2, []).
enum_entry('MAV_ODID_STATUS', 'MAV_ODID_STATUS_EMERGENCY', 3, []).
enum_entry('MAV_ODID_STATUS', 'MAV_ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE', 4, []).
enum_entry('MAV_ODID_HEIGHT_REF', 'MAV_ODID_HEIGHT_REF_OVER_TAKEOFF', 0, []).
enum_entry('MAV_ODID_HEIGHT_REF', 'MAV_ODID_HEIGHT_REF_OVER_GROUND', 1, []).
enum_entry('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_UNKNOWN', 0, []).
enum_entry('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_10NM', 1, []).
enum_entry('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_4NM', 2, []).
enum_entry('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_2NM', 3, []).
enum_entry('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_1NM', 4, []).
enum_entry('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_0_5NM', 5, []).
enum_entry('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_0_3NM', 6, []).
enum_entry('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_0_1NM', 7, []).
enum_entry('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_0_05NM', 8, []).
enum_entry('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_30_METER', 9, []).
enum_entry('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_10_METER', 10, []).
enum_entry('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_3_METER', 11, []).
enum_entry('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_1_METER', 12, []).
enum_entry('MAV_ODID_VER_ACC', 'MAV_ODID_VER_ACC_UNKNOWN', 0, []).
enum_entry('MAV_ODID_VER_ACC', 'MAV_ODID_VER_ACC_150_METER', 1, []).
enum_entry('MAV_ODID_VER_ACC', 'MAV_ODID_VER_ACC_45_METER', 2, []).
enum_entry('MAV_ODID_VER_ACC', 'MAV_ODID_VER_ACC_25_METER', 3, []).
enum_entry('MAV_ODID_VER_ACC', 'MAV_ODID_VER_ACC_10_METER', 4, []).
enum_entry('MAV_ODID_VER_ACC', 'MAV_ODID_VER_ACC_3_METER', 5, []).
enum_entry('MAV_ODID_VER_ACC', 'MAV_ODID_VER_ACC_1_METER', 6, []).
enum_entry('MAV_ODID_SPEED_ACC', 'MAV_ODID_SPEED_ACC_UNKNOWN', 0, []).
enum_entry('MAV_ODID_SPEED_ACC', 'MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND', 1, []).
enum_entry('MAV_ODID_SPEED_ACC', 'MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND', 2, []).
enum_entry('MAV_ODID_SPEED_ACC', 'MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND', 3, []).
enum_entry('MAV_ODID_SPEED_ACC', 'MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND', 4, []).
enum_entry('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_UNKNOWN', 0, []).
enum_entry('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_1_SECOND', 1, []).
enum_entry('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_2_SECOND', 2, []).
enum_entry('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_3_SECOND', 3, []).
enum_entry('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_4_SECOND', 4, []).
enum_entry('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_5_SECOND', 5, []).
enum_entry('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_6_SECOND', 6, []).
enum_entry('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_7_SECOND', 7, []).
enum_entry('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_8_SECOND', 8, []).
enum_entry('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_9_SECOND', 9, []).
enum_entry('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_1_0_SECOND', 10, []).
enum_entry('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_1_1_SECOND', 11, []).
enum_entry('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_1_2_SECOND', 12, []).
enum_entry('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_1_3_SECOND', 13, []).
enum_entry('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_1_4_SECOND', 14, []).
enum_entry('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_1_5_SECOND', 15, []).
enum_entry('MAV_ODID_AUTH_TYPE', 'MAV_ODID_AUTH_TYPE_NONE', 0, []).
enum_entry('MAV_ODID_AUTH_TYPE', 'MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE', 1, []).
enum_entry('MAV_ODID_AUTH_TYPE', 'MAV_ODID_AUTH_TYPE_OPERATOR_ID_SIGNATURE', 2, []).
enum_entry('MAV_ODID_AUTH_TYPE', 'MAV_ODID_AUTH_TYPE_MESSAGE_SET_SIGNATURE', 3, []).
enum_entry('MAV_ODID_AUTH_TYPE', 'MAV_ODID_AUTH_TYPE_NETWORK_REMOTE_ID', 4, []).
enum_entry('MAV_ODID_AUTH_TYPE', 'MAV_ODID_AUTH_TYPE_SPECIFIC_AUTHENTICATION', 5, []).
enum_entry('MAV_ODID_DESC_TYPE', 'MAV_ODID_DESC_TYPE_TEXT', 0, []).
enum_entry('MAV_ODID_DESC_TYPE', 'MAV_ODID_DESC_TYPE_EMERGENCY', 1, []).
enum_entry('MAV_ODID_DESC_TYPE', 'MAV_ODID_DESC_TYPE_EXTENDED_STATUS', 2, []).
enum_entry('MAV_ODID_OPERATOR_LOCATION_TYPE', 'MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF', 0, []).
enum_entry('MAV_ODID_OPERATOR_LOCATION_TYPE', 'MAV_ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS', 1, []).
enum_entry('MAV_ODID_OPERATOR_LOCATION_TYPE', 'MAV_ODID_OPERATOR_LOCATION_TYPE_FIXED', 2, []).
enum_entry('MAV_ODID_CLASSIFICATION_TYPE', 'MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED', 0, []).
enum_entry('MAV_ODID_CLASSIFICATION_TYPE', 'MAV_ODID_CLASSIFICATION_TYPE_EU', 1, []).
enum_entry('MAV_ODID_CATEGORY_EU', 'MAV_ODID_CATEGORY_EU_UNDECLARED', 0, []).
enum_entry('MAV_ODID_CATEGORY_EU', 'MAV_ODID_CATEGORY_EU_OPEN', 1, []).
enum_entry('MAV_ODID_CATEGORY_EU', 'MAV_ODID_CATEGORY_EU_SPECIFIC', 2, []).
enum_entry('MAV_ODID_CATEGORY_EU', 'MAV_ODID_CATEGORY_EU_CERTIFIED', 3, []).
enum_entry('MAV_ODID_CLASS_EU', 'MAV_ODID_CLASS_EU_UNDECLARED', 0, []).
enum_entry('MAV_ODID_CLASS_EU', 'MAV_ODID_CLASS_EU_CLASS_0', 1, []).
enum_entry('MAV_ODID_CLASS_EU', 'MAV_ODID_CLASS_EU_CLASS_1', 2, []).
enum_entry('MAV_ODID_CLASS_EU', 'MAV_ODID_CLASS_EU_CLASS_2', 3, []).
enum_entry('MAV_ODID_CLASS_EU', 'MAV_ODID_CLASS_EU_CLASS_3', 4, []).
enum_entry('MAV_ODID_CLASS_EU', 'MAV_ODID_CLASS_EU_CLASS_4', 5, []).
enum_entry('MAV_ODID_CLASS_EU', 'MAV_ODID_CLASS_EU_CLASS_5', 6, []).
enum_entry('MAV_ODID_CLASS_EU', 'MAV_ODID_CLASS_EU_CLASS_6', 7, []).
enum_entry('MAV_ODID_OPERATOR_ID_TYPE', 'MAV_ODID_OPERATOR_ID_TYPE_CAA', 0, []).
enum_entry('MAV_ODID_ARM_STATUS', 'MAV_ODID_ARM_STATUS_GOOD_TO_ARM', 0, []).
enum_entry('MAV_ODID_ARM_STATUS', 'MAV_ODID_ARM_STATUS_PRE_ARM_FAIL_GENERIC', 1, []).
enum_entry('TUNE_FORMAT', 'TUNE_FORMAT_QBASIC1_1', 1, []).
enum_entry('TUNE_FORMAT', 'TUNE_FORMAT_MML_MODERN', 2, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_UNKNOWN', 0, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_1', 1, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_2', 2, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_3', 3, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_4', 4, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_5', 5, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_6', 6, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_7', 7, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_8', 8, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_9', 9, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_10', 10, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_11', 11, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_12', 12, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_13', 13, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_14', 14, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_15', 15, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_16', 16, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_17', 17, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_18', 18, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_19', 19, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_WIG', 20, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_WIG_HAZARDOUS_A', 21, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_WIG_HAZARDOUS_B', 22, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_WIG_HAZARDOUS_C', 23, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_WIG_HAZARDOUS_D', 24, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_WIG_RESERVED_1', 25, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_WIG_RESERVED_2', 26, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_WIG_RESERVED_3', 27, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_WIG_RESERVED_4', 28, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_WIG_RESERVED_5', 29, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_FISHING', 30, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_TOWING', 31, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_TOWING_LARGE', 32, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_DREDGING', 33, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_DIVING', 34, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_MILITARY', 35, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_SAILING', 36, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_PLEASURE', 37, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_20', 38, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_RESERVED_21', 39, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_HSC', 40, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_HSC_HAZARDOUS_A', 41, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_HSC_HAZARDOUS_B', 42, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_HSC_HAZARDOUS_C', 43, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_HSC_HAZARDOUS_D', 44, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_HSC_RESERVED_1', 45, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_HSC_RESERVED_2', 46, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_HSC_RESERVED_3', 47, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_HSC_RESERVED_4', 48, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_HSC_UNKNOWN', 49, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_PILOT', 50, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_SAR', 51, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_TUG', 52, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_PORT_TENDER', 53, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_ANTI_POLLUTION', 54, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_LAW_ENFORCEMENT', 55, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_SPARE_LOCAL_1', 56, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_SPARE_LOCAL_2', 57, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_MEDICAL_TRANSPORT', 58, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_NONECOMBATANT', 59, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_PASSENGER', 60, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_PASSENGER_HAZARDOUS_A', 61, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_PASSENGER_HAZARDOUS_B', 62, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_PASSENGER_HAZARDOUS_C', 63, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_PASSENGER_HAZARDOUS_D', 64, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_PASSENGER_RESERVED_1', 65, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_PASSENGER_RESERVED_2', 66, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_PASSENGER_RESERVED_3', 67, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_PASSENGER_RESERVED_4', 68, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_PASSENGER_UNKNOWN', 69, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_CARGO', 70, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_CARGO_HAZARDOUS_A', 71, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_CARGO_HAZARDOUS_B', 72, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_CARGO_HAZARDOUS_C', 73, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_CARGO_HAZARDOUS_D', 74, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_CARGO_RESERVED_1', 75, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_CARGO_RESERVED_2', 76, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_CARGO_RESERVED_3', 77, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_CARGO_RESERVED_4', 78, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_CARGO_UNKNOWN', 79, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_TANKER', 80, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_TANKER_HAZARDOUS_A', 81, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_TANKER_HAZARDOUS_B', 82, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_TANKER_HAZARDOUS_C', 83, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_TANKER_HAZARDOUS_D', 84, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_TANKER_RESERVED_1', 85, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_TANKER_RESERVED_2', 86, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_TANKER_RESERVED_3', 87, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_TANKER_RESERVED_4', 88, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_TANKER_UNKNOWN', 89, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_OTHER', 90, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_OTHER_HAZARDOUS_A', 91, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_OTHER_HAZARDOUS_B', 92, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_OTHER_HAZARDOUS_C', 93, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_OTHER_HAZARDOUS_D', 94, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_OTHER_RESERVED_1', 95, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_OTHER_RESERVED_2', 96, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_OTHER_RESERVED_3', 97, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_OTHER_RESERVED_4', 98, []).
enum_entry('AIS_TYPE', 'AIS_TYPE_OTHER_UNKNOWN', 99, []).
enum_entry('AIS_NAV_STATUS', 'UNDER_WAY', 0, []).
enum_entry('AIS_NAV_STATUS', 'AIS_NAV_ANCHORED', 1, []).
enum_entry('AIS_NAV_STATUS', 'AIS_NAV_UN_COMMANDED', 2, []).
enum_entry('AIS_NAV_STATUS', 'AIS_NAV_RESTRICTED_MANOEUVERABILITY', 3, []).
enum_entry('AIS_NAV_STATUS', 'AIS_NAV_DRAUGHT_CONSTRAINED', 4, []).
enum_entry('AIS_NAV_STATUS', 'AIS_NAV_MOORED', 5, []).
enum_entry('AIS_NAV_STATUS', 'AIS_NAV_AGROUND', 6, []).
enum_entry('AIS_NAV_STATUS', 'AIS_NAV_FISHING', 7, []).
enum_entry('AIS_NAV_STATUS', 'AIS_NAV_SAILING', 8, []).
enum_entry('AIS_NAV_STATUS', 'AIS_NAV_RESERVED_HSC', 9, []).
enum_entry('AIS_NAV_STATUS', 'AIS_NAV_RESERVED_WIG', 10, []).
enum_entry('AIS_NAV_STATUS', 'AIS_NAV_RESERVED_1', 11, []).
enum_entry('AIS_NAV_STATUS', 'AIS_NAV_RESERVED_2', 12, []).
enum_entry('AIS_NAV_STATUS', 'AIS_NAV_RESERVED_3', 13, []).
enum_entry('AIS_NAV_STATUS', 'AIS_NAV_AIS_SART', 14, []).
enum_entry('AIS_NAV_STATUS', 'AIS_NAV_UNKNOWN', 15, []).
enum_entry('AIS_FLAGS', 'AIS_FLAGS_POSITION_ACCURACY', 1, []).
enum_entry('AIS_FLAGS', 'AIS_FLAGS_VALID_COG', 2, []).
enum_entry('AIS_FLAGS', 'AIS_FLAGS_VALID_VELOCITY', 4, []).
enum_entry('AIS_FLAGS', 'AIS_FLAGS_HIGH_VELOCITY', 8, []).
enum_entry('AIS_FLAGS', 'AIS_FLAGS_VALID_TURN_RATE', 16, []).
enum_entry('AIS_FLAGS', 'AIS_FLAGS_TURN_RATE_SIGN_ONLY', 32, []).
enum_entry('AIS_FLAGS', 'AIS_FLAGS_VALID_DIMENSIONS', 64, []).
enum_entry('AIS_FLAGS', 'AIS_FLAGS_LARGE_BOW_DIMENSION', 128, []).
enum_entry('AIS_FLAGS', 'AIS_FLAGS_LARGE_STERN_DIMENSION', 256, []).
enum_entry('AIS_FLAGS', 'AIS_FLAGS_LARGE_PORT_DIMENSION', 512, []).
enum_entry('AIS_FLAGS', 'AIS_FLAGS_LARGE_STARBOARD_DIMENSION', 1024, []).
enum_entry('AIS_FLAGS', 'AIS_FLAGS_VALID_CALLSIGN', 2048, []).
enum_entry('AIS_FLAGS', 'AIS_FLAGS_VALID_NAME', 4096, []).
enum_entry('FAILURE_UNIT', 'FAILURE_UNIT_SENSOR_GYRO', 0, []).
enum_entry('FAILURE_UNIT', 'FAILURE_UNIT_SENSOR_ACCEL', 1, []).
enum_entry('FAILURE_UNIT', 'FAILURE_UNIT_SENSOR_MAG', 2, []).
enum_entry('FAILURE_UNIT', 'FAILURE_UNIT_SENSOR_BARO', 3, []).
enum_entry('FAILURE_UNIT', 'FAILURE_UNIT_SENSOR_GPS', 4, []).
enum_entry('FAILURE_UNIT', 'FAILURE_UNIT_SENSOR_OPTICAL_FLOW', 5, []).
enum_entry('FAILURE_UNIT', 'FAILURE_UNIT_SENSOR_VIO', 6, []).
enum_entry('FAILURE_UNIT', 'FAILURE_UNIT_SENSOR_DISTANCE_SENSOR', 7, []).
enum_entry('FAILURE_UNIT', 'FAILURE_UNIT_SENSOR_AIRSPEED', 8, []).
enum_entry('FAILURE_UNIT', 'FAILURE_UNIT_SYSTEM_BATTERY', 100, []).
enum_entry('FAILURE_UNIT', 'FAILURE_UNIT_SYSTEM_MOTOR', 101, []).
enum_entry('FAILURE_UNIT', 'FAILURE_UNIT_SYSTEM_SERVO', 102, []).
enum_entry('FAILURE_UNIT', 'FAILURE_UNIT_SYSTEM_AVOIDANCE', 103, []).
enum_entry('FAILURE_UNIT', 'FAILURE_UNIT_SYSTEM_RC_SIGNAL', 104, []).
enum_entry('FAILURE_UNIT', 'FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL', 105, []).
enum_entry('FAILURE_TYPE', 'FAILURE_TYPE_OK', 0, []).
enum_entry('FAILURE_TYPE', 'FAILURE_TYPE_OFF', 1, []).
enum_entry('FAILURE_TYPE', 'FAILURE_TYPE_STUCK', 2, []).
enum_entry('FAILURE_TYPE', 'FAILURE_TYPE_GARBAGE', 3, []).
enum_entry('FAILURE_TYPE', 'FAILURE_TYPE_WRONG', 4, []).
enum_entry('FAILURE_TYPE', 'FAILURE_TYPE_SLOW', 5, []).
enum_entry('FAILURE_TYPE', 'FAILURE_TYPE_DELAYED', 6, []).
enum_entry('FAILURE_TYPE', 'FAILURE_TYPE_INTERMITTENT', 7, []).
enum_entry('NAV_VTOL_LAND_OPTIONS', 'NAV_VTOL_LAND_OPTIONS_DEFAULT', 0, []).
enum_entry('NAV_VTOL_LAND_OPTIONS', 'NAV_VTOL_LAND_OPTIONS_FW_DESCENT', 1, []).
enum_entry('NAV_VTOL_LAND_OPTIONS', 'NAV_VTOL_LAND_OPTIONS_HOVER_DESCENT', 2, []).
enum_entry('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_HEALTHY', 1, []).
enum_entry('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_FULLY_RETRACTED', 2, []).
enum_entry('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_MOVING', 4, []).
enum_entry('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_CLUTCH_ENGAGED', 8, []).
enum_entry('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_LOCKED', 16, []).
enum_entry('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_DROPPING', 32, []).
enum_entry('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_ARRESTING', 64, []).
enum_entry('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_GROUND_SENSE', 128, []).
enum_entry('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_RETRACTING', 256, []).
enum_entry('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_REDELIVER', 512, []).
enum_entry('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_ABANDON_LINE', 1024, []).
enum_entry('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_LOCKING', 2048, []).
enum_entry('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_LOAD_LINE', 4096, []).
enum_entry('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_LOAD_PAYLOAD', 8192, []).
enum_entry('MAG_CAL_STATUS', 'MAG_CAL_NOT_STARTED', 0, []).
enum_entry('MAG_CAL_STATUS', 'MAG_CAL_WAITING_TO_START', 1, []).
enum_entry('MAG_CAL_STATUS', 'MAG_CAL_RUNNING_STEP_ONE', 2, []).
enum_entry('MAG_CAL_STATUS', 'MAG_CAL_RUNNING_STEP_TWO', 3, []).
enum_entry('MAG_CAL_STATUS', 'MAG_CAL_SUCCESS', 4, []).
enum_entry('MAG_CAL_STATUS', 'MAG_CAL_FAILED', 5, []).
enum_entry('MAG_CAL_STATUS', 'MAG_CAL_BAD_ORIENTATION', 6, []).
enum_entry('MAG_CAL_STATUS', 'MAG_CAL_BAD_RADIUS', 7, []).
enum_entry('MAV_EVENT_ERROR_REASON', 'MAV_EVENT_ERROR_REASON_UNAVAILABLE', 0, []).
enum_entry('MAV_EVENT_CURRENT_SEQUENCE_FLAGS', 'MAV_EVENT_CURRENT_SEQUENCE_FLAGS_RESET', 1, []).
enum_entry('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_NONE', 0, []).
enum_entry('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_XACC', 1, []).
enum_entry('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_YACC', 2, []).
enum_entry('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_ZACC', 4, []).
enum_entry('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_XGYRO', 8, []).
enum_entry('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_YGYRO', 16, []).
enum_entry('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_ZGYRO', 32, []).
enum_entry('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_XMAG', 64, []).
enum_entry('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_YMAG', 128, []).
enum_entry('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_ZMAG', 256, []).
enum_entry('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_ABS_PRESSURE', 512, []).
enum_entry('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_DIFF_PRESSURE', 1024, []).
enum_entry('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_PRESSURE_ALT', 2048, []).
enum_entry('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_TEMPERATURE', 4096, []).
enum_entry('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_RESET', 2147483648, []).
enum_entry('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_NONE', 0, []).
enum_entry('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_XACC', 1, []).
enum_entry('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_YACC', 2, []).
enum_entry('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_ZACC', 4, []).
enum_entry('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_XGYRO', 8, []).
enum_entry('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_YGYRO', 16, []).
enum_entry('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_ZGYRO', 32, []).
enum_entry('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_XMAG', 64, []).
enum_entry('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_YMAG', 128, []).
enum_entry('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_ZMAG', 256, []).
enum_entry('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_ABS_PRESSURE', 512, []).
enum_entry('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_DIFF_PRESSURE', 1024, []).
enum_entry('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_PRESSURE_ALT', 2048, []).
enum_entry('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_TEMPERATURE', 4096, []).
enum_entry('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_ALL', 65535, []).
enum_entry('CAN_FILTER_OP', 'CAN_FILTER_REPLACE', 0, []).
enum_entry('CAN_FILTER_OP', 'CAN_FILTER_ADD', 1, []).
enum_entry('CAN_FILTER_OP', 'CAN_FILTER_REMOVE', 2, []).
enum_entry('MAV_FTP_ERR', 'MAV_FTP_ERR_NONE', 0, []).
enum_entry('MAV_FTP_ERR', 'MAV_FTP_ERR_FAIL', 1, []).
enum_entry('MAV_FTP_ERR', 'MAV_FTP_ERR_FAILERRNO', 2, []).
enum_entry('MAV_FTP_ERR', 'MAV_FTP_ERR_INVALIDDATASIZE', 3, []).
enum_entry('MAV_FTP_ERR', 'MAV_FTP_ERR_INVALIDSESSION', 4, []).
enum_entry('MAV_FTP_ERR', 'MAV_FTP_ERR_NOSESSIONSAVAILABLE', 5, []).
enum_entry('MAV_FTP_ERR', 'MAV_FTP_ERR_EOF', 6, []).
enum_entry('MAV_FTP_ERR', 'MAV_FTP_ERR_UNKNOWNCOMMAND', 7, []).
enum_entry('MAV_FTP_ERR', 'MAV_FTP_ERR_FILEEXISTS', 8, []).
enum_entry('MAV_FTP_ERR', 'MAV_FTP_ERR_FILEPROTECTED', 9, []).
enum_entry('MAV_FTP_ERR', 'MAV_FTP_ERR_FILENOTFOUND', 10, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_NONE', 0, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_TERMINATESESSION', 1, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_RESETSESSION', 2, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_LISTDIRECTORY', 3, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_OPENFILERO', 4, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_READFILE', 5, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_CREATEFILE', 6, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_WRITEFILE', 7, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_REMOVEFILE', 8, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_CREATEDIRECTORY', 9, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_REMOVEDIRECTORY', 10, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_OPENFILEWO', 11, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_TRUNCATEFILE', 12, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_RENAME', 13, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_CALCFILECRC', 14, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_BURSTREADFILE', 15, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_ACK', 128, []).
enum_entry('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_NAK', 129, []).
enum_entry('MISSION_STATE', 'MISSION_STATE_UNKNOWN', 0, []).
enum_entry('MISSION_STATE', 'MISSION_STATE_NO_MISSION', 1, []).
enum_entry('MISSION_STATE', 'MISSION_STATE_NOT_STARTED', 2, []).
enum_entry('MISSION_STATE', 'MISSION_STATE_ACTIVE', 3, []).
enum_entry('MISSION_STATE', 'MISSION_STATE_PAUSED', 4, []).
enum_entry('MISSION_STATE', 'MISSION_STATE_COMPLETE', 5, []).
enum_entry('WIFI_NETWORK_SECURITY', 'WIFI_NETWORK_SECURITY_UNDEFINED', 0, []).
enum_entry('WIFI_NETWORK_SECURITY', 'WIFI_NETWORK_SECURITY_OPEN', 1, []).
enum_entry('WIFI_NETWORK_SECURITY', 'WIFI_NETWORK_SECURITY_WEP', 2, []).
enum_entry('WIFI_NETWORK_SECURITY', 'WIFI_NETWORK_SECURITY_WPA1', 3, []).
enum_entry('WIFI_NETWORK_SECURITY', 'WIFI_NETWORK_SECURITY_WPA2', 4, []).
enum_entry('WIFI_NETWORK_SECURITY', 'WIFI_NETWORK_SECURITY_WPA3', 5, []).
enum_entry('AIRSPEED_SENSOR_FLAGS', 'AIRSPEED_SENSOR_UNHEALTHY', 0, []).
enum_entry('AIRSPEED_SENSOR_FLAGS', 'AIRSPEED_SENSOR_USING', 1, []).
enum_entry('PARAM_TRANSACTION_TRANSPORT', 'PARAM_TRANSACTION_TRANSPORT_PARAM', 0, []).
enum_entry('PARAM_TRANSACTION_TRANSPORT', 'PARAM_TRANSACTION_TRANSPORT_PARAM_EXT', 1, []).
enum_entry('PARAM_TRANSACTION_ACTION', 'PARAM_TRANSACTION_ACTION_START', 0, []).
enum_entry('PARAM_TRANSACTION_ACTION', 'PARAM_TRANSACTION_ACTION_COMMIT', 1, []).
enum_entry('PARAM_TRANSACTION_ACTION', 'PARAM_TRANSACTION_ACTION_CANCEL', 2, []).
enum_entry('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_NON_STANDARD', 0, []).
enum_entry('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_POSITION_HOLD', 1, []).
enum_entry('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_ORBIT', 2, []).
enum_entry('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_CRUISE', 3, []).
enum_entry('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_ALTITUDE_HOLD', 4, []).
enum_entry('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_RETURN_HOME', 5, []).
enum_entry('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_SAFE_RECOVERY', 6, []).
enum_entry('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_MISSION', 7, []).
enum_entry('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_LAND', 8, []).
enum_entry('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_TAKEOFF', 9, []).
enum_entry('MAV_MODE_PROPERTY', 'MAV_MODE_PROPERTY_ADVANCED', 1, []).
enum_entry('MAV_MODE_PROPERTY', 'MAV_MODE_PROPERTY_NOT_USER_SELECTABLE', 2, []).
enum_entry('MAV_CMD', 'MAV_CMD_DO_FIGURE_EIGHT', 35, [hasLocation=true, isDestination=true]).
enum_entry('MAV_CMD', 'MAV_CMD_PARAM_TRANSACTION', 900, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SET_FENCE_BREACH_ACTION', 5010, [hasLocation=false, isDestination=false]).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_NOT_READY_TO_USE', 1, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_CHARGING', 2, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_CELL_BALANCING', 4, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_CELL_IMBALANCE', 8, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_AUTO_DISCHARGING', 16, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_REQUIRES_SERVICE', 32, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_BAD_BATTERY', 64, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_PROTECTIONS_ENABLED', 128, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_PROTECTION_SYSTEM', 256, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_OVER_VOLT', 512, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_UNDER_VOLT', 1024, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_OVER_TEMPERATURE', 2048, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_UNDER_TEMPERATURE', 4096, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_OVER_CURRENT', 8192, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_SHORT_CIRCUIT', 16384, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_INCOMPATIBLE_VOLTAGE', 32768, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_INCOMPATIBLE_FIRMWARE', 65536, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION', 131072, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_CAPACITY_RELATIVE_TO_FULL', 262144, []).
enum_entry('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_EXTENDED', 4294967295, []).
enum_entry('MAV_CMD', 'MAV_CMD_DO_UPGRADE', 247, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_GROUP_START', 301, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_GROUP_END', 302, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_DO_SET_STANDARD_MODE', 262, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_SET_AT_S_PARAM', 550, [hasLocation=false, isDestination=false]).
enum_entry('TARGET_ABSOLUTE_SENSOR_CAPABILITY_FLAGS', 'TARGET_ABSOLUTE_SENSOR_CAPABILITY_POSITION', 1, []).
enum_entry('TARGET_ABSOLUTE_SENSOR_CAPABILITY_FLAGS', 'TARGET_ABSOLUTE_SENSOR_CAPABILITY_VELOCITY', 2, []).
enum_entry('TARGET_ABSOLUTE_SENSOR_CAPABILITY_FLAGS', 'TARGET_ABSOLUTE_SENSOR_CAPABILITY_ACCELERATION', 4, []).
enum_entry('TARGET_ABSOLUTE_SENSOR_CAPABILITY_FLAGS', 'TARGET_ABSOLUTE_SENSOR_CAPABILITY_ATTITUDE', 8, []).
enum_entry('TARGET_ABSOLUTE_SENSOR_CAPABILITY_FLAGS', 'TARGET_ABSOLUTE_SENSOR_CAPABILITY_RATES', 16, []).
enum_entry('TARGET_OBS_FRAME', 'TARGET_OBS_FRAME_LOCAL_NED', 0, []).
enum_entry('TARGET_OBS_FRAME', 'TARGET_OBS_FRAME_BODY_FRD', 1, []).
enum_entry('TARGET_OBS_FRAME', 'TARGET_OBS_FRAME_LOCAL_OFFSET_NED', 2, []).
enum_entry('TARGET_OBS_FRAME', 'TARGET_OBS_FRAME_OTHER', 3, []).
enum_entry('ICAROUS_TRACK_BAND_TYPES', 'ICAROUS_TRACK_BAND_TYPE_NONE', 0, []).
enum_entry('ICAROUS_TRACK_BAND_TYPES', 'ICAROUS_TRACK_BAND_TYPE_NEAR', 1, []).
enum_entry('ICAROUS_TRACK_BAND_TYPES', 'ICAROUS_TRACK_BAND_TYPE_RECOVERY', 2, []).
enum_entry('ICAROUS_FMS_STATE', 'ICAROUS_FMS_STATE_IDLE', 0, []).
enum_entry('ICAROUS_FMS_STATE', 'ICAROUS_FMS_STATE_TAKEOFF', 1, []).
enum_entry('ICAROUS_FMS_STATE', 'ICAROUS_FMS_STATE_CLIMB', 2, []).
enum_entry('ICAROUS_FMS_STATE', 'ICAROUS_FMS_STATE_CRUISE', 3, []).
enum_entry('ICAROUS_FMS_STATE', 'ICAROUS_FMS_STATE_APPROACH', 4, []).
enum_entry('ICAROUS_FMS_STATE', 'ICAROUS_FMS_STATE_LAND', 5, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_GENERIC', 0, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_RESERVED', 1, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_SLUGS', 2, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_ARDUPILOTMEGA', 3, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_OPENPILOT', 4, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY', 5, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY', 6, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_GENERIC_MISSION_FULL', 7, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_INVALID', 8, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_PPZ', 9, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_UDB', 10, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_FP', 11, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_PX4', 12, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_SMACCMPILOT', 13, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_AUTOQUAD', 14, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_ARMAZILA', 15, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_AEROB', 16, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_ASLUAV', 17, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_SMARTAP', 18, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_AIRRAILS', 19, []).
enum_entry('MAV_AUTOPILOT', 'MAV_AUTOPILOT_REFLEX', 20, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_GENERIC', 0, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_FIXED_WING', 1, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_QUADROTOR', 2, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_COAXIAL', 3, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_HELICOPTER', 4, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_ANTENNA_TRACKER', 5, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_GCS', 6, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_AIRSHIP', 7, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_FREE_BALLOON', 8, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_ROCKET', 9, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_GROUND_ROVER', 10, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_SURFACE_BOAT', 11, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_SUBMARINE', 12, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_HEXAROTOR', 13, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_OCTOROTOR', 14, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_TRICOPTER', 15, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_FLAPPING_WING', 16, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_KITE', 17, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_ONBOARD_CONTROLLER', 18, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_VTOL_TAILSITTER_DUOROTOR', 19, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_VTOL_TAILSITTER_QUADROTOR', 20, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_VTOL_TILTROTOR', 21, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_VTOL_FIXEDROTOR', 22, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_VTOL_TAILSITTER', 23, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_VTOL_TILTWING', 24, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_VTOL_RESERVED5', 25, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_GIMBAL', 26, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_ADSB', 27, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_PARAFOIL', 28, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_DODECAROTOR', 29, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_CAMERA', 30, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_CHARGING_STATION', 31, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_FLARM', 32, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_SERVO', 33, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_ODID', 34, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_DECAROTOR', 35, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_BATTERY', 36, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_PARACHUTE', 37, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_LOG', 38, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_OSD', 39, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_IMU', 40, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_GPS', 41, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_WINCH', 42, []).
enum_entry('MAV_TYPE', 'MAV_TYPE_GENERIC_MULTIROTOR', 43, []).
enum_entry('MAV_MODE_FLAG', 'MAV_MODE_FLAG_SAFETY_ARMED', 128, []).
enum_entry('MAV_MODE_FLAG', 'MAV_MODE_FLAG_MANUAL_INPUT_ENABLED', 64, []).
enum_entry('MAV_MODE_FLAG', 'MAV_MODE_FLAG_HIL_ENABLED', 32, []).
enum_entry('MAV_MODE_FLAG', 'MAV_MODE_FLAG_STABILIZE_ENABLED', 16, []).
enum_entry('MAV_MODE_FLAG', 'MAV_MODE_FLAG_GUIDED_ENABLED', 8, []).
enum_entry('MAV_MODE_FLAG', 'MAV_MODE_FLAG_AUTO_ENABLED', 4, []).
enum_entry('MAV_MODE_FLAG', 'MAV_MODE_FLAG_TEST_ENABLED', 2, []).
enum_entry('MAV_MODE_FLAG', 'MAV_MODE_FLAG_CUSTOM_MODE_ENABLED', 1, []).
enum_entry('MAV_MODE_FLAG_DECODE_POSITION', 'MAV_MODE_FLAG_DECODE_POSITION_SAFETY', 128, []).
enum_entry('MAV_MODE_FLAG_DECODE_POSITION', 'MAV_MODE_FLAG_DECODE_POSITION_MANUAL', 64, []).
enum_entry('MAV_MODE_FLAG_DECODE_POSITION', 'MAV_MODE_FLAG_DECODE_POSITION_HIL', 32, []).
enum_entry('MAV_MODE_FLAG_DECODE_POSITION', 'MAV_MODE_FLAG_DECODE_POSITION_STABILIZE', 16, []).
enum_entry('MAV_MODE_FLAG_DECODE_POSITION', 'MAV_MODE_FLAG_DECODE_POSITION_GUIDED', 8, []).
enum_entry('MAV_MODE_FLAG_DECODE_POSITION', 'MAV_MODE_FLAG_DECODE_POSITION_AUTO', 4, []).
enum_entry('MAV_MODE_FLAG_DECODE_POSITION', 'MAV_MODE_FLAG_DECODE_POSITION_TEST', 2, []).
enum_entry('MAV_MODE_FLAG_DECODE_POSITION', 'MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE', 1, []).
enum_entry('MAV_STATE', 'MAV_STATE_UNINIT', 0, []).
enum_entry('MAV_STATE', 'MAV_STATE_BOOT', 1, []).
enum_entry('MAV_STATE', 'MAV_STATE_CALIBRATING', 2, []).
enum_entry('MAV_STATE', 'MAV_STATE_STANDBY', 3, []).
enum_entry('MAV_STATE', 'MAV_STATE_ACTIVE', 4, []).
enum_entry('MAV_STATE', 'MAV_STATE_CRITICAL', 5, []).
enum_entry('MAV_STATE', 'MAV_STATE_EMERGENCY', 6, []).
enum_entry('MAV_STATE', 'MAV_STATE_POWEROFF', 7, []).
enum_entry('MAV_STATE', 'MAV_STATE_FLIGHT_TERMINATION', 8, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_ALL', 0, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_AUTOPILOT1', 1, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER1', 25, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER2', 26, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER3', 27, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER4', 28, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER5', 29, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER6', 30, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER7', 31, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER8', 32, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER9', 33, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER10', 34, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER11', 35, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER12', 36, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER13', 37, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER14', 38, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER15', 39, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER16', 40, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER17', 41, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER18', 42, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER19', 43, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER20', 44, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER21', 45, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER22', 46, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER23', 47, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER24', 48, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER25', 49, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER26', 50, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER27', 51, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER28', 52, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER29', 53, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER30', 54, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER31', 55, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER32', 56, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER33', 57, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER34', 58, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER35', 59, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER36', 60, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER37', 61, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER38', 62, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER39', 63, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER40', 64, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER41', 65, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER42', 66, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER43', 67, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_TELEMETRY_RADIO', 68, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER45', 69, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER46', 70, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER47', 71, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER48', 72, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER49', 73, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER50', 74, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER51', 75, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER52', 76, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER53', 77, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER54', 78, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER55', 79, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER56', 80, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER57', 81, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER58', 82, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER59', 83, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER60', 84, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER61', 85, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER62', 86, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER63', 87, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER64', 88, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER65', 89, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER66', 90, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER67', 91, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER68', 92, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER69', 93, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER70', 94, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER71', 95, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER72', 96, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER73', 97, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER74', 98, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_USER75', 99, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_CAMERA', 100, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_CAMERA2', 101, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_CAMERA3', 102, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_CAMERA4', 103, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_CAMERA5', 104, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_CAMERA6', 105, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_SERVO1', 140, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_SERVO2', 141, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_SERVO3', 142, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_SERVO4', 143, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_SERVO5', 144, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_SERVO6', 145, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_SERVO7', 146, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_SERVO8', 147, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_SERVO9', 148, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_SERVO10', 149, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_SERVO11', 150, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_SERVO12', 151, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_SERVO13', 152, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_SERVO14', 153, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_GIMBAL', 154, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_LOG', 155, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_ADSB', 156, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_OSD', 157, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_PERIPHERAL', 158, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_QX1_GIMBAL', 159, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_FLARM', 160, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_PARACHUTE', 161, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_WINCH', 169, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_GIMBAL2', 171, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_GIMBAL3', 172, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_GIMBAL4', 173, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_GIMBAL5', 174, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_GIMBAL6', 175, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_BATTERY', 180, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_BATTERY2', 181, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_MAVCAN', 189, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_MISSIONPLANNER', 190, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_ONBOARD_COMPUTER', 191, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_ONBOARD_COMPUTER2', 192, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_ONBOARD_COMPUTER3', 193, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_ONBOARD_COMPUTER4', 194, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_PATHPLANNER', 195, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_OBSTACLE_AVOIDANCE', 196, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY', 197, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_PAIRING_MANAGER', 198, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_IMU', 200, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_IMU_2', 201, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_IMU_3', 202, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_GPS', 220, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_GPS2', 221, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_ODID_TXRX_1', 236, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_ODID_TXRX_2', 237, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_ODID_TXRX_3', 238, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_UDP_BRIDGE', 240, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_UART_BRIDGE', 241, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_TUNNEL_NODE', 242, []).
enum_entry('MAV_COMPONENT', 'MAV_COMP_ID_SYSTEM_CONTROL', 250, []).
enum_entry('UALBERTA_AUTOPILOT_MODE', 'MODE_MANUAL_DIRECT', 1, []).
enum_entry('UALBERTA_AUTOPILOT_MODE', 'MODE_MANUAL_SCALED', 2, []).
enum_entry('UALBERTA_AUTOPILOT_MODE', 'MODE_AUTO_PID_ATT', 3, []).
enum_entry('UALBERTA_AUTOPILOT_MODE', 'MODE_AUTO_PID_VEL', 4, []).
enum_entry('UALBERTA_AUTOPILOT_MODE', 'MODE_AUTO_PID_POS', 5, []).
enum_entry('UALBERTA_NAV_MODE', 'NAV_AHRS_INIT', 1, []).
enum_entry('UALBERTA_NAV_MODE', 'NAV_AHRS', 2, []).
enum_entry('UALBERTA_NAV_MODE', 'NAV_INS_GPS_INIT', 3, []).
enum_entry('UALBERTA_NAV_MODE', 'NAV_INS_GPS', 4, []).
enum_entry('UALBERTA_PILOT_MODE', 'PILOT_MANUAL', 1, []).
enum_entry('UALBERTA_PILOT_MODE', 'PILOT_AUTO', 2, []).
enum_entry('UALBERTA_PILOT_MODE', 'PILOT_ROTO', 3, []).
enum_entry('UAVIONIX_ADSB_OUT_DYNAMIC_STATE', 'UAVIONIX_ADSB_OUT_DYNAMIC_STATE_INTENT_CHANGE', 1, []).
enum_entry('UAVIONIX_ADSB_OUT_DYNAMIC_STATE', 'UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED', 2, []).
enum_entry('UAVIONIX_ADSB_OUT_DYNAMIC_STATE', 'UAVIONIX_ADSB_OUT_DYNAMIC_STATE_NICBARO_CROSSCHECKED', 4, []).
enum_entry('UAVIONIX_ADSB_OUT_DYNAMIC_STATE', 'UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND', 8, []).
enum_entry('UAVIONIX_ADSB_OUT_DYNAMIC_STATE', 'UAVIONIX_ADSB_OUT_DYNAMIC_STATE_IDENT', 16, []).
enum_entry('UAVIONIX_ADSB_OUT_RF_SELECT', 'UAVIONIX_ADSB_OUT_RF_SELECT_STANDBY', 0, []).
enum_entry('UAVIONIX_ADSB_OUT_RF_SELECT', 'UAVIONIX_ADSB_OUT_RF_SELECT_RX_ENABLED', 1, []).
enum_entry('UAVIONIX_ADSB_OUT_RF_SELECT', 'UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED', 2, []).
enum_entry('UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX', 'UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_0', 0, []).
enum_entry('UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX', 'UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_NONE_1', 1, []).
enum_entry('UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX', 'UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_2D', 2, []).
enum_entry('UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX', 'UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_3D', 3, []).
enum_entry('UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX', 'UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_DGPS', 4, []).
enum_entry('UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX', 'UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX_RTK', 5, []).
enum_entry('UAVIONIX_ADSB_RF_HEALTH', 'UAVIONIX_ADSB_RF_HEALTH_INITIALIZING', 0, []).
enum_entry('UAVIONIX_ADSB_RF_HEALTH', 'UAVIONIX_ADSB_RF_HEALTH_OK', 1, []).
enum_entry('UAVIONIX_ADSB_RF_HEALTH', 'UAVIONIX_ADSB_RF_HEALTH_FAIL_TX', 2, []).
enum_entry('UAVIONIX_ADSB_RF_HEALTH', 'UAVIONIX_ADSB_RF_HEALTH_FAIL_RX', 16, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_NO_DATA', 0, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L15M_W23M', 1, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25M_W28P5M', 2, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L25_34M', 3, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_33M', 4, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L35_38M', 5, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_39P5M', 6, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L45_45M', 7, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_45M', 8, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L55_52M', 9, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_59P5M', 10, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L65_67M', 11, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W72P5M', 12, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L75_W80M', 13, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W80M', 14, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', 'UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE_L85_W90M', 15, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT', 'UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_NO_DATA', 0, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT', 'UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_2M', 1, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT', 'UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_4M', 2, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT', 'UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_LEFT_6M', 3, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT', 'UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_0M', 4, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT', 'UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_2M', 5, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT', 'UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_4M', 6, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT', 'UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT_RIGHT_6M', 7, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON', 'UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_NO_DATA', 0, []).
enum_entry('UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON', 'UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON_APPLIED_BY_SENSOR', 1, []).
enum_entry('UAVIONIX_ADSB_EMERGENCY_STATUS', 'UAVIONIX_ADSB_OUT_NO_EMERGENCY', 0, []).
enum_entry('UAVIONIX_ADSB_EMERGENCY_STATUS', 'UAVIONIX_ADSB_OUT_GENERAL_EMERGENCY', 1, []).
enum_entry('UAVIONIX_ADSB_EMERGENCY_STATUS', 'UAVIONIX_ADSB_OUT_LIFEGUARD_EMERGENCY', 2, []).
enum_entry('UAVIONIX_ADSB_EMERGENCY_STATUS', 'UAVIONIX_ADSB_OUT_MINIMUM_FUEL_EMERGENCY', 3, []).
enum_entry('UAVIONIX_ADSB_EMERGENCY_STATUS', 'UAVIONIX_ADSB_OUT_NO_COMM_EMERGENCY', 4, []).
enum_entry('UAVIONIX_ADSB_EMERGENCY_STATUS', 'UAVIONIX_ADSB_OUT_UNLAWFUL_INTERFERANCE_EMERGENCY', 5, []).
enum_entry('UAVIONIX_ADSB_EMERGENCY_STATUS', 'UAVIONIX_ADSB_OUT_DOWNED_AIRCRAFT_EMERGENCY', 6, []).
enum_entry('UAVIONIX_ADSB_EMERGENCY_STATUS', 'UAVIONIX_ADSB_OUT_RESERVED', 7, []).
enum_entry('MAV_STORM32_TUNNEL_PAYLOAD_TYPE', 'MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH1_IN', 200, []).
enum_entry('MAV_STORM32_TUNNEL_PAYLOAD_TYPE', 'MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH1_OUT', 201, []).
enum_entry('MAV_STORM32_TUNNEL_PAYLOAD_TYPE', 'MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH2_IN', 202, []).
enum_entry('MAV_STORM32_TUNNEL_PAYLOAD_TYPE', 'MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH2_OUT', 203, []).
enum_entry('MAV_STORM32_TUNNEL_PAYLOAD_TYPE', 'MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH3_IN', 204, []).
enum_entry('MAV_STORM32_TUNNEL_PAYLOAD_TYPE', 'MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH3_OUT', 205, []).
enum_entry('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_IS_NORMAL', 1, []).
enum_entry('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_IMUS_WORKING', 2, []).
enum_entry('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_MOTORS_WORKING', 4, []).
enum_entry('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_ENCODERS_WORKING', 8, []).
enum_entry('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_VOLTAGE_OK', 16, []).
enum_entry('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_VIRTUALCHANNELS_RECEIVING', 32, []).
enum_entry('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_MAVLINK_RECEIVING', 64, []).
enum_entry('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_STORM32LINK_QFIX', 128, []).
enum_entry('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_STORM32LINK_WORKING', 256, []).
enum_entry('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_CAMERA_CONNECTED', 512, []).
enum_entry('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_AUX0_LOW', 1024, []).
enum_entry('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_AUX1_LOW', 2048, []).
enum_entry('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_NTLOGGER_WORKING', 4096, []).
enum_entry('MAV_STORM32_CAMERA_PREARM_FLAGS', 'MAV_STORM32_CAMERA_PREARM_FLAGS_CONNECTED', 1, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_CAP_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_CAP_FLAGS_HAS_PROFILES', 1, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_NONE', 0, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_RC_ACTIVE', 1, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_ONBOARD_ACTIVE', 2, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_AUTOPILOT_ACTIVE', 4, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_GCS_ACTIVE', 8, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_CAMERA_ACTIVE', 16, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_GCS2_ACTIVE', 32, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_CAMERA2_ACTIVE', 64, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_CUSTOM_ACTIVE', 128, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_CUSTOM2_ACTIVE', 256, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_SET_SUPERVISON', 512, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_SET_RELEASE', 1024, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_NONE', 0, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_ONBOARD', 1, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_AUTOPILOT', 2, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_GCS', 3, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_CAMERA', 4, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_GCS2', 5, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_CAMERA2', 6, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_CUSTOM', 7, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_CUSTOM2', 8, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_PROFILE', 'MAV_STORM32_GIMBAL_MANAGER_PROFILE_DEFAULT', 0, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_PROFILE', 'MAV_STORM32_GIMBAL_MANAGER_PROFILE_CUSTOM', 1, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_PROFILE', 'MAV_STORM32_GIMBAL_MANAGER_PROFILE_COOPERATIVE', 2, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_PROFILE', 'MAV_STORM32_GIMBAL_MANAGER_PROFILE_EXCLUSIVE', 3, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_PROFILE', 'MAV_STORM32_GIMBAL_MANAGER_PROFILE_PRIORITY_COOPERATIVE', 4, []).
enum_entry('MAV_STORM32_GIMBAL_MANAGER_PROFILE', 'MAV_STORM32_GIMBAL_MANAGER_PROFILE_PRIORITY_EXCLUSIVE', 5, []).
enum_entry('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_UNDEFINED', 0, []).
enum_entry('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_DEFAULT', 1, []).
enum_entry('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_GIMBAL_RETRACT', 2, []).
enum_entry('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_GIMBAL_NEUTRAL', 3, []).
enum_entry('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_GIMBAL_MISSION', 4, []).
enum_entry('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_GIMBAL_RC_CONTROL', 5, []).
enum_entry('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_POI_TARGETING', 6, []).
enum_entry('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_SYSID_TARGETING', 7, []).
enum_entry('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_CABLECAM_2POINT', 8, []).
enum_entry('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_HOME_TARGETING', 9, []).
enum_entry('MAV_CMD', 'MAV_CMD_STORM32_DO_GIMBAL_MANAGER_CONTROL_PITCHYAW', 60002, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_STORM32_DO_GIMBAL_MANAGER_SETUP', 60010, [hasLocation=false, isDestination=false]).
enum_entry('MAV_CMD', 'MAV_CMD_QSHOT_DO_CONFIGURE', 60020, [hasLocation=false, isDestination=false]).
enum_entry('RADIO_RC_CHANNELS_FLAGS', 'RADIO_RC_CHANNELS_FLAGS_FAILSAFE', 1, []).
enum_entry('RADIO_RC_CHANNELS_FLAGS', 'RADIO_RC_CHANNELS_FLAGS_FRAME_MISSED', 2, []).
enum_entry('RADIO_LINK_STATS_FLAGS', 'RADIO_LINK_STATS_FLAGS_RSSI_DBM', 1, []).
enum_entry('MAV_CMD', 'MAV_CMD_PRS_SET_ARM', 60050, [isDestination=false, hasLocation=false]).
enum_entry('MAV_CMD', 'MAV_CMD_PRS_GET_ARM', 60051, [isDestination=false, hasLocation=false]).
enum_entry('MAV_CMD', 'MAV_CMD_PRS_GET_BATTERY', 60052, [isDestination=false, hasLocation=false]).
enum_entry('MAV_CMD', 'MAV_CMD_PRS_GET_ERR', 60053, [isDestination=false, hasLocation=false]).
enum_entry('MAV_CMD', 'MAV_CMD_PRS_SET_ARM_ALTI', 60070, [isDestination=false, hasLocation=false]).
enum_entry('MAV_CMD', 'MAV_CMD_PRS_GET_ARM_ALTI', 60071, [isDestination=false, hasLocation=false]).
enum_entry('MAV_CMD', 'MAV_CMD_PRS_SHUTDOWN', 60072, [isDestination=false, hasLocation=false]).
enum_entry('MAV_AVSS_COMMAND_FAILURE_REASON', 'PRS_NOT_STEADY', 1, []).
enum_entry('MAV_AVSS_COMMAND_FAILURE_REASON', 'PRS_DTM_NOT_ARMED', 2, []).
enum_entry('MAV_AVSS_COMMAND_FAILURE_REASON', 'PRS_OTM_NOT_ARMED', 3, []).
enum_entry('AVSS_M300_OPERATION_MODE', 'MODE_M300_MANUAL_CTRL', 0, []).
enum_entry('AVSS_M300_OPERATION_MODE', 'MODE_M300_ATTITUDE', 1, []).
enum_entry('AVSS_M300_OPERATION_MODE', 'MODE_M300_P_GPS', 6, []).
enum_entry('AVSS_M300_OPERATION_MODE', 'MODE_M300_HOTPOINT_MODE', 9, []).
enum_entry('AVSS_M300_OPERATION_MODE', 'MODE_M300_ASSISTED_TAKEOFF', 10, []).
enum_entry('AVSS_M300_OPERATION_MODE', 'MODE_M300_AUTO_TAKEOFF', 11, []).
enum_entry('AVSS_M300_OPERATION_MODE', 'MODE_M300_AUTO_LANDING', 12, []).
enum_entry('AVSS_M300_OPERATION_MODE', 'MODE_M300_NAVI_GO_HOME', 15, []).
enum_entry('AVSS_M300_OPERATION_MODE', 'MODE_M300_NAVI_SDK_CTRL', 17, []).
enum_entry('AVSS_M300_OPERATION_MODE', 'MODE_M300_S_SPORT', 31, []).
enum_entry('AVSS_M300_OPERATION_MODE', 'MODE_M300_FORCE_AUTO_LANDING', 33, []).
enum_entry('AVSS_M300_OPERATION_MODE', 'MODE_M300_T_TRIPOD', 38, []).
enum_entry('AVSS_M300_OPERATION_MODE', 'MODE_M300_SEARCH_MODE', 40, []).
enum_entry('AVSS_M300_OPERATION_MODE', 'MODE_M300_ENGINE_START', 41, []).
enum_entry('AVSS_HORSEFLY_OPERATION_MODE', 'MODE_HORSEFLY_MANUAL_CTRL', 0, []).
enum_entry('AVSS_HORSEFLY_OPERATION_MODE', 'MODE_HORSEFLY_AUTO_TAKEOFF', 1, []).
enum_entry('AVSS_HORSEFLY_OPERATION_MODE', 'MODE_HORSEFLY_AUTO_LANDING', 2, []).
enum_entry('AVSS_HORSEFLY_OPERATION_MODE', 'MODE_HORSEFLY_NAVI_GO_HOME', 3, []).
enum_entry('AVSS_HORSEFLY_OPERATION_MODE', 'MODE_HORSEFLY_DROP', 4, []).
enum_entry('AIRLINK_AUTH_RESPONSE_TYPE', 'AIRLINK_ERROR_LOGIN_OR_PASS', 0, []).
enum_entry('AIRLINK_AUTH_RESPONSE_TYPE', 'AIRLINK_AUTH_OK', 1, []).

:- dynamic enum_entry_deprecated/3.

enum_entry_deprecated('MAV_FRAME', 'MAV_FRAME_BODY_NED', [since='2019-08', replaced_by='MAV_FRAME_BODY_FRD']).
enum_entry_deprecated('MAV_FRAME', 'MAV_FRAME_BODY_OFFSET_NED', [since='2019-08', replaced_by='MAV_FRAME_BODY_FRD']).
enum_entry_deprecated('MAV_FRAME', 'MAV_FRAME_RESERVED_13', [since='2019-04', replaced_by='']).
enum_entry_deprecated('MAV_FRAME', 'MAV_FRAME_RESERVED_14', [since='2019-04', replaced_by='MAV_FRAME_LOCAL_FRD']).
enum_entry_deprecated('MAV_FRAME', 'MAV_FRAME_RESERVED_15', [since='2019-04', replaced_by='MAV_FRAME_LOCAL_FLU']).
enum_entry_deprecated('MAV_FRAME', 'MAV_FRAME_RESERVED_16', [since='2019-04', replaced_by='MAV_FRAME_LOCAL_FRD']).
enum_entry_deprecated('MAV_FRAME', 'MAV_FRAME_RESERVED_17', [since='2019-04', replaced_by='MAV_FRAME_LOCAL_FLU']).
enum_entry_deprecated('MAV_FRAME', 'MAV_FRAME_RESERVED_18', [since='2019-04', replaced_by='MAV_FRAME_LOCAL_FRD']).
enum_entry_deprecated('MAV_FRAME', 'MAV_FRAME_RESERVED_19', [since='2019-04', replaced_by='MAV_FRAME_LOCAL_FLU']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_NAV_ROI', [since='2018-01', replaced_by='MAV_CMD_DO_SET_ROI_*']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_DO_SET_ROI', [since='2018-01', replaced_by='MAV_CMD_DO_SET_ROI_*']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONFIGURE', [since='2020-01', replaced_by='MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL', [since='2020-01', replaced_by='MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL_QUAT', [since='2020-01', replaced_by='MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_GET_HOME_POSITION', [since='2022-04', replaced_by='MAV_CMD_REQUEST_MESSAGE']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_GET_MESSAGE_INTERVAL', [since='2022-04', replaced_by='MAV_CMD_REQUEST_MESSAGE']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_REQUEST_PROTOCOL_VERSION', [since='2019-08', replaced_by='MAV_CMD_REQUEST_MESSAGE']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES', [since='2019-08', replaced_by='MAV_CMD_REQUEST_MESSAGE']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_INFORMATION', [since='2019-08', replaced_by='MAV_CMD_REQUEST_MESSAGE']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_SETTINGS', [since='2019-08', replaced_by='MAV_CMD_REQUEST_MESSAGE']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_REQUEST_STORAGE_INFORMATION', [since='2019-08', replaced_by='MAV_CMD_REQUEST_MESSAGE']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS', [since='2019-08', replaced_by='MAV_CMD_REQUEST_MESSAGE']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_REQUEST_FLIGHT_INFORMATION', [since='2019-08', replaced_by='MAV_CMD_REQUEST_MESSAGE']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE', [since='2019-08', replaced_by='MAV_CMD_REQUEST_MESSAGE']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION', [since='2019-08', replaced_by='MAV_CMD_REQUEST_MESSAGE']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_REQUEST_VIDEO_STREAM_STATUS', [since='2019-08', replaced_by='MAV_CMD_REQUEST_MESSAGE']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_PAYLOAD_PREPARE_DEPLOY', [since='2021-06', replaced_by='']).
enum_entry_deprecated('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL_DEPLOY', [since='2021-06', replaced_by='']).
enum_entry_deprecated('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT', [since='2022-03', replaced_by='MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST']).
enum_entry_deprecated('MAV_COMPONENT', 'MAV_COMP_ID_QX1_GIMBAL', [since='2018-11', replaced_by='MAV_COMP_ID_GIMBAL']).
enum_entry_deprecated('MAV_COMPONENT', 'MAV_COMP_ID_SYSTEM_CONTROL', [since='2018-11', replaced_by='MAV_COMP_ID_ALL']).

:- dynamic enum_entry_description/3.

enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_RESUME_REPEAT_DIST', 'Set the distance to be repeated on mission resume').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SPRAYER', 'Control attached liquid sprayer').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SEND_SCRIPT_MESSAGE', 'Pass instructions onto scripting, a script should be checking for a new command').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_AUX_FUNCTION', 'Execute auxiliary function').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_ALTITUDE_WAIT', 'Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up.').
enum_entry_description('MAV_CMD', 'MAV_CMD_POWER_OFF_INITIATED', 'A system wide power-off event has been initiated.').
enum_entry_description('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_CLICK', 'FLY button has been clicked.').
enum_entry_description('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_HOLD', 'FLY button has been held for 1.5 seconds.').
enum_entry_description('MAV_CMD', 'MAV_CMD_SOLO_BTN_PAUSE_CLICK', 'PAUSE button has been clicked.').
enum_entry_description('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL', 'Magnetometer calibration based on fixed position\n        in earth field given by inclination, declination and intensity.').
enum_entry_description('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_FIELD', 'Magnetometer calibration based on fixed expected field values.').
enum_entry_description('MAV_CMD', 'MAV_CMD_SET_EKF_SOURCE_SET', 'Set EKF sensor source set.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_START_MAG_CAL', 'Initiate a magnetometer calibration.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_ACCEPT_MAG_CAL', 'Accept a magnetometer calibration.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_CANCEL_MAG_CAL', 'Cancel a running magnetometer calibration.').
enum_entry_description('MAV_CMD', 'MAV_CMD_ACCELCAL_VEHICLE_POS', 'Used when doing accelerometer calibration. When sent to the GCS tells it what position to put the vehicle in. When sent to the vehicle says what position the vehicle is in.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SEND_BANNER', 'Reply with the version banner.').
enum_entry_description('MAV_CMD', 'MAV_CMD_SET_FACTORY_TEST_MODE', 'Command autopilot to get into factory test/diagnostic mode.').
enum_entry_description('MAV_CMD', 'MAV_CMD_GIMBAL_RESET', 'Causes the gimbal to reset and boot as if it was just powered on.').
enum_entry_description('MAV_CMD', 'MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS', 'Reports progress and success or failure of gimbal axis calibration procedure.').
enum_entry_description('MAV_CMD', 'MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION', 'Starts commutation calibration on the gimbal.').
enum_entry_description('MAV_CMD', 'MAV_CMD_GIMBAL_FULL_RESET', 'Erases gimbal application and parameters.').
enum_entry_description('MAV_CMD', 'MAV_CMD_FLASH_BOOTLOADER', 'Update the bootloader').
enum_entry_description('MAV_CMD', 'MAV_CMD_BATTERY_RESET', 'Reset battery capacity for batteries that accumulate consumed battery via integration.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DEBUG_TRAP', 'Issue a trap signal to the autopilot process, presumably to enter the debugger.').
enum_entry_description('MAV_CMD', 'MAV_CMD_SCRIPTING', 'Control onboard scripting.').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_SCRIPT_TIME', 'Scripting command as NAV command with wait for completion.').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_ATTITUDE_TIME', 'Maintain an attitude for a specified time.').
enum_entry_description('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_SPEED', 'Change flight speed at a given rate. This slews the vehicle at a controllable rate between it\'s previous speed and the new one. (affects GUIDED only. Outside GUIDED, aircraft ignores these commands. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.)').
enum_entry_description('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_ALTITUDE', 'Change target altitude at a given rate. This slews the vehicle at a controllable rate between it\'s previous altitude and the new one. (affects GUIDED only. Outside GUIDED, aircraft ignores these commands. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.)').
enum_entry_description('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_HEADING', 'Change to target heading at a given rate, overriding previous heading/s. This slews the vehicle at a controllable rate between it\'s previous heading and the new one. (affects GUIDED only. Exiting GUIDED returns aircraft to normal behaviour defined elsewhere. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.)').
enum_entry_description('MAV_CMD', 'MAV_CMD_EXTERNAL_POSITION_ESTIMATE', 'Provide an external position estimate for use when dead-reckoning. This is meant to be used for occasional position resets that may be provided by a external system such as a remote pilot using landmarks over a video link.').
enum_entry_description('SCRIPTING_CMD', 'SCRIPTING_CMD_REPL_START', 'Start a REPL session.').
enum_entry_description('SCRIPTING_CMD', 'SCRIPTING_CMD_REPL_STOP', 'End a REPL session.').
enum_entry_description('SCRIPTING_CMD', 'SCRIPTING_CMD_STOP', 'Stop execution of scripts.').
enum_entry_description('SCRIPTING_CMD', 'SCRIPTING_CMD_STOP_AND_RESTART', 'Stop execution of scripts and restart.').
enum_entry_description('LIMITS_STATE', 'LIMITS_INIT', 'Pre-initialization.').
enum_entry_description('LIMITS_STATE', 'LIMITS_DISABLED', 'Disabled.').
enum_entry_description('LIMITS_STATE', 'LIMITS_ENABLED', 'Checking limits.').
enum_entry_description('LIMITS_STATE', 'LIMITS_TRIGGERED', 'A limit has been breached.').
enum_entry_description('LIMITS_STATE', 'LIMITS_RECOVERING', 'Taking action e.g. Return/RTL.').
enum_entry_description('LIMITS_STATE', 'LIMITS_RECOVERED', 'We\'re no longer in breach of a limit.').
enum_entry_description('LIMIT_MODULE', 'LIMIT_GPSLOCK', 'Pre-initialization.').
enum_entry_description('LIMIT_MODULE', 'LIMIT_GEOFENCE', 'Disabled.').
enum_entry_description('LIMIT_MODULE', 'LIMIT_ALTITUDE', 'Checking limits.').
enum_entry_description('RALLY_FLAGS', 'FAVORABLE_WIND', 'Flag set when requiring favorable winds for landing.').
enum_entry_description('RALLY_FLAGS', 'LAND_IMMEDIATELY', 'Flag set when plane is to immediately descend to break altitude and land without GCS intervention. Flag not set when plane is to loiter at Rally point until commanded to land.').
enum_entry_description('CAMERA_STATUS_TYPES', 'CAMERA_STATUS_TYPE_HEARTBEAT', 'Camera heartbeat, announce camera component ID at 1Hz.').
enum_entry_description('CAMERA_STATUS_TYPES', 'CAMERA_STATUS_TYPE_TRIGGER', 'Camera image triggered.').
enum_entry_description('CAMERA_STATUS_TYPES', 'CAMERA_STATUS_TYPE_DISCONNECT', 'Camera connection lost.').
enum_entry_description('CAMERA_STATUS_TYPES', 'CAMERA_STATUS_TYPE_ERROR', 'Camera unknown error.').
enum_entry_description('CAMERA_STATUS_TYPES', 'CAMERA_STATUS_TYPE_LOWBATT', 'Camera battery low. Parameter p1 shows reported voltage.').
enum_entry_description('CAMERA_STATUS_TYPES', 'CAMERA_STATUS_TYPE_LOWSTORE', 'Camera storage low. Parameter p1 shows reported shots remaining.').
enum_entry_description('CAMERA_STATUS_TYPES', 'CAMERA_STATUS_TYPE_LOWSTOREV', 'Camera storage low. Parameter p1 shows reported video minutes remaining.').
enum_entry_description('CAMERA_FEEDBACK_FLAGS', 'CAMERA_FEEDBACK_PHOTO', 'Shooting photos, not video.').
enum_entry_description('CAMERA_FEEDBACK_FLAGS', 'CAMERA_FEEDBACK_VIDEO', 'Shooting video, not stills.').
enum_entry_description('CAMERA_FEEDBACK_FLAGS', 'CAMERA_FEEDBACK_BADEXPOSURE', 'Unable to achieve requested exposure (e.g. shutter speed too low).').
enum_entry_description('CAMERA_FEEDBACK_FLAGS', 'CAMERA_FEEDBACK_CLOSEDLOOP', 'Closed loop feedback from camera, we know for sure it has successfully taken a picture.').
enum_entry_description('CAMERA_FEEDBACK_FLAGS', 'CAMERA_FEEDBACK_OPENLOOP', 'Open loop camera, an image trigger has been requested but we can\'t know for sure it has successfully taken a picture.').
enum_entry_description('MAV_MODE_GIMBAL', 'MAV_MODE_GIMBAL_UNINITIALIZED', 'Gimbal is powered on but has not started initializing yet.').
enum_entry_description('MAV_MODE_GIMBAL', 'MAV_MODE_GIMBAL_CALIBRATING_PITCH', 'Gimbal is currently running calibration on the pitch axis.').
enum_entry_description('MAV_MODE_GIMBAL', 'MAV_MODE_GIMBAL_CALIBRATING_ROLL', 'Gimbal is currently running calibration on the roll axis.').
enum_entry_description('MAV_MODE_GIMBAL', 'MAV_MODE_GIMBAL_CALIBRATING_YAW', 'Gimbal is currently running calibration on the yaw axis.').
enum_entry_description('MAV_MODE_GIMBAL', 'MAV_MODE_GIMBAL_INITIALIZED', 'Gimbal has finished calibrating and initializing, but is relaxed pending reception of first rate command from copter.').
enum_entry_description('MAV_MODE_GIMBAL', 'MAV_MODE_GIMBAL_ACTIVE', 'Gimbal is actively stabilizing.').
enum_entry_description('MAV_MODE_GIMBAL', 'MAV_MODE_GIMBAL_RATE_CMD_TIMEOUT', 'Gimbal is relaxed because it missed more than 10 expected rate command messages in a row. Gimbal will move back to active mode when it receives a new rate command.').
enum_entry_description('GIMBAL_AXIS', 'GIMBAL_AXIS_YAW', 'Gimbal yaw axis.').
enum_entry_description('GIMBAL_AXIS', 'GIMBAL_AXIS_PITCH', 'Gimbal pitch axis.').
enum_entry_description('GIMBAL_AXIS', 'GIMBAL_AXIS_ROLL', 'Gimbal roll axis.').
enum_entry_description('GIMBAL_AXIS_CALIBRATION_STATUS', 'GIMBAL_AXIS_CALIBRATION_STATUS_IN_PROGRESS', 'Axis calibration is in progress.').
enum_entry_description('GIMBAL_AXIS_CALIBRATION_STATUS', 'GIMBAL_AXIS_CALIBRATION_STATUS_SUCCEEDED', 'Axis calibration succeeded.').
enum_entry_description('GIMBAL_AXIS_CALIBRATION_STATUS', 'GIMBAL_AXIS_CALIBRATION_STATUS_FAILED', 'Axis calibration failed.').
enum_entry_description('GIMBAL_AXIS_CALIBRATION_REQUIRED', 'GIMBAL_AXIS_CALIBRATION_REQUIRED_UNKNOWN', 'Whether or not this axis requires calibration is unknown at this time.').
enum_entry_description('GIMBAL_AXIS_CALIBRATION_REQUIRED', 'GIMBAL_AXIS_CALIBRATION_REQUIRED_TRUE', 'This axis requires calibration.').
enum_entry_description('GIMBAL_AXIS_CALIBRATION_REQUIRED', 'GIMBAL_AXIS_CALIBRATION_REQUIRED_FALSE', 'This axis does not require calibration.').
enum_entry_description('GOPRO_HEARTBEAT_STATUS', 'GOPRO_HEARTBEAT_STATUS_DISCONNECTED', 'No GoPro connected.').
enum_entry_description('GOPRO_HEARTBEAT_STATUS', 'GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE', 'The detected GoPro is not HeroBus compatible.').
enum_entry_description('GOPRO_HEARTBEAT_STATUS', 'GOPRO_HEARTBEAT_STATUS_CONNECTED', 'A HeroBus compatible GoPro is connected.').
enum_entry_description('GOPRO_HEARTBEAT_STATUS', 'GOPRO_HEARTBEAT_STATUS_ERROR', 'An unrecoverable error was encountered with the connected GoPro, it may require a power cycle.').
enum_entry_description('GOPRO_HEARTBEAT_FLAGS', 'GOPRO_FLAG_RECORDING', 'GoPro is currently recording.').
enum_entry_description('GOPRO_REQUEST_STATUS', 'GOPRO_REQUEST_SUCCESS', 'The write message with ID indicated succeeded.').
enum_entry_description('GOPRO_REQUEST_STATUS', 'GOPRO_REQUEST_FAILED', 'The write message with ID indicated failed.').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_POWER', '(Get/Set).').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_CAPTURE_MODE', '(Get/Set).').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_SHUTTER', '(___/Set).').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_BATTERY', '(Get/___).').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_MODEL', '(Get/___).').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_VIDEO_SETTINGS', '(Get/Set).').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_LOW_LIGHT', '(Get/Set).').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_PHOTO_RESOLUTION', '(Get/Set).').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_PHOTO_BURST_RATE', '(Get/Set).').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_PROTUNE', '(Get/Set).').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_PROTUNE_WHITE_BALANCE', '(Get/Set) Hero 3+ Only.').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_PROTUNE_COLOUR', '(Get/Set) Hero 3+ Only.').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_PROTUNE_GAIN', '(Get/Set) Hero 3+ Only.').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_PROTUNE_SHARPNESS', '(Get/Set) Hero 3+ Only.').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_PROTUNE_EXPOSURE', '(Get/Set) Hero 3+ Only.').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_TIME', '(Get/Set).').
enum_entry_description('GOPRO_COMMAND', 'GOPRO_COMMAND_CHARGING', '(Get/Set).').
enum_entry_description('GOPRO_CAPTURE_MODE', 'GOPRO_CAPTURE_MODE_VIDEO', 'Video mode.').
enum_entry_description('GOPRO_CAPTURE_MODE', 'GOPRO_CAPTURE_MODE_PHOTO', 'Photo mode.').
enum_entry_description('GOPRO_CAPTURE_MODE', 'GOPRO_CAPTURE_MODE_BURST', 'Burst mode, Hero 3+ only.').
enum_entry_description('GOPRO_CAPTURE_MODE', 'GOPRO_CAPTURE_MODE_TIME_LAPSE', 'Time lapse mode, Hero 3+ only.').
enum_entry_description('GOPRO_CAPTURE_MODE', 'GOPRO_CAPTURE_MODE_MULTI_SHOT', 'Multi shot mode, Hero 4 only.').
enum_entry_description('GOPRO_CAPTURE_MODE', 'GOPRO_CAPTURE_MODE_PLAYBACK', 'Playback mode, Hero 4 only, silver only except when LCD or HDMI is connected to black.').
enum_entry_description('GOPRO_CAPTURE_MODE', 'GOPRO_CAPTURE_MODE_SETUP', 'Playback mode, Hero 4 only.').
enum_entry_description('GOPRO_CAPTURE_MODE', 'GOPRO_CAPTURE_MODE_UNKNOWN', 'Mode not yet known.').
enum_entry_description('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_480p', '848 x 480 (480p).').
enum_entry_description('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_720p', '1280 x 720 (720p).').
enum_entry_description('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_960p', '1280 x 960 (960p).').
enum_entry_description('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_1080p', '1920 x 1080 (1080p).').
enum_entry_description('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_1440p', '1920 x 1440 (1440p).').
enum_entry_description('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_2_7k_17_9', '2704 x 1440 (2.7k-17:9).').
enum_entry_description('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_2_7k_16_9', '2704 x 1524 (2.7k-16:9).').
enum_entry_description('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_2_7k_4_3', '2704 x 2028 (2.7k-4:3).').
enum_entry_description('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_4k_16_9', '3840 x 2160 (4k-16:9).').
enum_entry_description('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_4k_17_9', '4096 x 2160 (4k-17:9).').
enum_entry_description('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_720p_SUPERVIEW', '1280 x 720 (720p-SuperView).').
enum_entry_description('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_1080p_SUPERVIEW', '1920 x 1080 (1080p-SuperView).').
enum_entry_description('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_2_7k_SUPERVIEW', '2704 x 1520 (2.7k-SuperView).').
enum_entry_description('GOPRO_RESOLUTION', 'GOPRO_RESOLUTION_4k_SUPERVIEW', '3840 x 2160 (4k-SuperView).').
enum_entry_description('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_12', '12 FPS.').
enum_entry_description('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_15', '15 FPS.').
enum_entry_description('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_24', '24 FPS.').
enum_entry_description('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_25', '25 FPS.').
enum_entry_description('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_30', '30 FPS.').
enum_entry_description('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_48', '48 FPS.').
enum_entry_description('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_50', '50 FPS.').
enum_entry_description('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_60', '60 FPS.').
enum_entry_description('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_80', '80 FPS.').
enum_entry_description('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_90', '90 FPS.').
enum_entry_description('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_100', '100 FPS.').
enum_entry_description('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_120', '120 FPS.').
enum_entry_description('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_240', '240 FPS.').
enum_entry_description('GOPRO_FRAME_RATE', 'GOPRO_FRAME_RATE_12_5', '12.5 FPS.').
enum_entry_description('GOPRO_FIELD_OF_VIEW', 'GOPRO_FIELD_OF_VIEW_WIDE', '0x00: Wide.').
enum_entry_description('GOPRO_FIELD_OF_VIEW', 'GOPRO_FIELD_OF_VIEW_MEDIUM', '0x01: Medium.').
enum_entry_description('GOPRO_FIELD_OF_VIEW', 'GOPRO_FIELD_OF_VIEW_NARROW', '0x02: Narrow.').
enum_entry_description('GOPRO_VIDEO_SETTINGS_FLAGS', 'GOPRO_VIDEO_SETTINGS_TV_MODE', '0=NTSC, 1=PAL.').
enum_entry_description('GOPRO_PHOTO_RESOLUTION', 'GOPRO_PHOTO_RESOLUTION_5MP_MEDIUM', '5MP Medium.').
enum_entry_description('GOPRO_PHOTO_RESOLUTION', 'GOPRO_PHOTO_RESOLUTION_7MP_MEDIUM', '7MP Medium.').
enum_entry_description('GOPRO_PHOTO_RESOLUTION', 'GOPRO_PHOTO_RESOLUTION_7MP_WIDE', '7MP Wide.').
enum_entry_description('GOPRO_PHOTO_RESOLUTION', 'GOPRO_PHOTO_RESOLUTION_10MP_WIDE', '10MP Wide.').
enum_entry_description('GOPRO_PHOTO_RESOLUTION', 'GOPRO_PHOTO_RESOLUTION_12MP_WIDE', '12MP Wide.').
enum_entry_description('GOPRO_PROTUNE_WHITE_BALANCE', 'GOPRO_PROTUNE_WHITE_BALANCE_AUTO', 'Auto.').
enum_entry_description('GOPRO_PROTUNE_WHITE_BALANCE', 'GOPRO_PROTUNE_WHITE_BALANCE_3000K', '3000K.').
enum_entry_description('GOPRO_PROTUNE_WHITE_BALANCE', 'GOPRO_PROTUNE_WHITE_BALANCE_5500K', '5500K.').
enum_entry_description('GOPRO_PROTUNE_WHITE_BALANCE', 'GOPRO_PROTUNE_WHITE_BALANCE_6500K', '6500K.').
enum_entry_description('GOPRO_PROTUNE_WHITE_BALANCE', 'GOPRO_PROTUNE_WHITE_BALANCE_RAW', 'Camera Raw.').
enum_entry_description('GOPRO_PROTUNE_COLOUR', 'GOPRO_PROTUNE_COLOUR_STANDARD', 'Auto.').
enum_entry_description('GOPRO_PROTUNE_COLOUR', 'GOPRO_PROTUNE_COLOUR_NEUTRAL', 'Neutral.').
enum_entry_description('GOPRO_PROTUNE_GAIN', 'GOPRO_PROTUNE_GAIN_400', 'ISO 400.').
enum_entry_description('GOPRO_PROTUNE_GAIN', 'GOPRO_PROTUNE_GAIN_800', 'ISO 800 (Only Hero 4).').
enum_entry_description('GOPRO_PROTUNE_GAIN', 'GOPRO_PROTUNE_GAIN_1600', 'ISO 1600.').
enum_entry_description('GOPRO_PROTUNE_GAIN', 'GOPRO_PROTUNE_GAIN_3200', 'ISO 3200 (Only Hero 4).').
enum_entry_description('GOPRO_PROTUNE_GAIN', 'GOPRO_PROTUNE_GAIN_6400', 'ISO 6400.').
enum_entry_description('GOPRO_PROTUNE_SHARPNESS', 'GOPRO_PROTUNE_SHARPNESS_LOW', 'Low Sharpness.').
enum_entry_description('GOPRO_PROTUNE_SHARPNESS', 'GOPRO_PROTUNE_SHARPNESS_MEDIUM', 'Medium Sharpness.').
enum_entry_description('GOPRO_PROTUNE_SHARPNESS', 'GOPRO_PROTUNE_SHARPNESS_HIGH', 'High Sharpness.').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_5_0', '-5.0 EV (Hero 3+ Only).').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_4_5', '-4.5 EV (Hero 3+ Only).').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_4_0', '-4.0 EV (Hero 3+ Only).').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_3_5', '-3.5 EV (Hero 3+ Only).').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_3_0', '-3.0 EV (Hero 3+ Only).').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_2_5', '-2.5 EV (Hero 3+ Only).').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_2_0', '-2.0 EV.').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_1_5', '-1.5 EV.').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_1_0', '-1.0 EV.').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_NEG_0_5', '-0.5 EV.').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_ZERO', '0.0 EV.').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_0_5', '+0.5 EV.').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_1_0', '+1.0 EV.').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_1_5', '+1.5 EV.').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_2_0', '+2.0 EV.').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_2_5', '+2.5 EV (Hero 3+ Only).').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_3_0', '+3.0 EV (Hero 3+ Only).').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_3_5', '+3.5 EV (Hero 3+ Only).').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_4_0', '+4.0 EV (Hero 3+ Only).').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_4_5', '+4.5 EV (Hero 3+ Only).').
enum_entry_description('GOPRO_PROTUNE_EXPOSURE', 'GOPRO_PROTUNE_EXPOSURE_POS_5_0', '+5.0 EV (Hero 3+ Only).').
enum_entry_description('GOPRO_CHARGING', 'GOPRO_CHARGING_DISABLED', 'Charging disabled.').
enum_entry_description('GOPRO_CHARGING', 'GOPRO_CHARGING_ENABLED', 'Charging enabled.').
enum_entry_description('GOPRO_MODEL', 'GOPRO_MODEL_UNKNOWN', 'Unknown gopro model.').
enum_entry_description('GOPRO_MODEL', 'GOPRO_MODEL_HERO_3_PLUS_SILVER', 'Hero 3+ Silver (HeroBus not supported by GoPro).').
enum_entry_description('GOPRO_MODEL', 'GOPRO_MODEL_HERO_3_PLUS_BLACK', 'Hero 3+ Black.').
enum_entry_description('GOPRO_MODEL', 'GOPRO_MODEL_HERO_4_SILVER', 'Hero 4 Silver.').
enum_entry_description('GOPRO_MODEL', 'GOPRO_MODEL_HERO_4_BLACK', 'Hero 4 Black.').
enum_entry_description('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_3_IN_1_SECOND', '3 Shots / 1 Second.').
enum_entry_description('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_5_IN_1_SECOND', '5 Shots / 1 Second.').
enum_entry_description('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_10_IN_1_SECOND', '10 Shots / 1 Second.').
enum_entry_description('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_10_IN_2_SECOND', '10 Shots / 2 Second.').
enum_entry_description('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_10_IN_3_SECOND', '10 Shots / 3 Second (Hero 4 Only).').
enum_entry_description('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_30_IN_1_SECOND', '30 Shots / 1 Second.').
enum_entry_description('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_30_IN_2_SECOND', '30 Shots / 2 Second.').
enum_entry_description('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_30_IN_3_SECOND', '30 Shots / 3 Second.').
enum_entry_description('GOPRO_BURST_RATE', 'GOPRO_BURST_RATE_30_IN_6_SECOND', '30 Shots / 6 Second.').
enum_entry_description('MAV_CMD_DO_AUX_FUNCTION_SWITCH_LEVEL', 'MAV_CMD_DO_AUX_FUNCTION_SWITCH_LEVEL_LOW', 'Switch Low.').
enum_entry_description('MAV_CMD_DO_AUX_FUNCTION_SWITCH_LEVEL', 'MAV_CMD_DO_AUX_FUNCTION_SWITCH_LEVEL_MIDDLE', 'Switch Middle.').
enum_entry_description('MAV_CMD_DO_AUX_FUNCTION_SWITCH_LEVEL', 'MAV_CMD_DO_AUX_FUNCTION_SWITCH_LEVEL_HIGH', 'Switch High.').
enum_entry_description('LED_CONTROL_PATTERN', 'LED_CONTROL_PATTERN_OFF', 'LED patterns off (return control to regular vehicle control).').
enum_entry_description('LED_CONTROL_PATTERN', 'LED_CONTROL_PATTERN_FIRMWAREUPDATE', 'LEDs show pattern during firmware update.').
enum_entry_description('LED_CONTROL_PATTERN', 'LED_CONTROL_PATTERN_CUSTOM', 'Custom Pattern using custom bytes fields.').
enum_entry_description('EKF_STATUS_FLAGS', 'EKF_ATTITUDE', 'Set if EKF\'s attitude estimate is good.').
enum_entry_description('EKF_STATUS_FLAGS', 'EKF_VELOCITY_HORIZ', 'Set if EKF\'s horizontal velocity estimate is good.').
enum_entry_description('EKF_STATUS_FLAGS', 'EKF_VELOCITY_VERT', 'Set if EKF\'s vertical velocity estimate is good.').
enum_entry_description('EKF_STATUS_FLAGS', 'EKF_POS_HORIZ_REL', 'Set if EKF\'s horizontal position (relative) estimate is good.').
enum_entry_description('EKF_STATUS_FLAGS', 'EKF_POS_HORIZ_ABS', 'Set if EKF\'s horizontal position (absolute) estimate is good.').
enum_entry_description('EKF_STATUS_FLAGS', 'EKF_POS_VERT_ABS', 'Set if EKF\'s vertical position (absolute) estimate is good.').
enum_entry_description('EKF_STATUS_FLAGS', 'EKF_POS_VERT_AGL', 'Set if EKF\'s vertical position (above ground) estimate is good.').
enum_entry_description('EKF_STATUS_FLAGS', 'EKF_CONST_POS_MODE', 'EKF is in constant position mode and does not know it\'s absolute or relative position.').
enum_entry_description('EKF_STATUS_FLAGS', 'EKF_PRED_POS_HORIZ_REL', 'Set if EKF\'s predicted horizontal position (relative) estimate is good.').
enum_entry_description('EKF_STATUS_FLAGS', 'EKF_PRED_POS_HORIZ_ABS', 'Set if EKF\'s predicted horizontal position (absolute) estimate is good.').
enum_entry_description('EKF_STATUS_FLAGS', 'EKF_UNINITIALIZED', 'Set if EKF has never been healthy.').
enum_entry_description('MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS', 'MAV_REMOTE_LOG_DATA_BLOCK_STOP', 'UAV to stop sending DataFlash blocks.').
enum_entry_description('MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS', 'MAV_REMOTE_LOG_DATA_BLOCK_START', 'UAV to start sending DataFlash blocks.').
enum_entry_description('MAV_REMOTE_LOG_DATA_BLOCK_STATUSES', 'MAV_REMOTE_LOG_DATA_BLOCK_NACK', 'This block has NOT been received.').
enum_entry_description('MAV_REMOTE_LOG_DATA_BLOCK_STATUSES', 'MAV_REMOTE_LOG_DATA_BLOCK_ACK', 'This block has been received.').
enum_entry_description('DEVICE_OP_BUSTYPE', 'DEVICE_OP_BUSTYPE_I2C', 'I2C Device operation.').
enum_entry_description('DEVICE_OP_BUSTYPE', 'DEVICE_OP_BUSTYPE_SPI', 'SPI Device operation.').
enum_entry_description('DEEPSTALL_STAGE', 'DEEPSTALL_STAGE_FLY_TO_LANDING', 'Flying to the landing point.').
enum_entry_description('DEEPSTALL_STAGE', 'DEEPSTALL_STAGE_ESTIMATE_WIND', 'Building an estimate of the wind.').
enum_entry_description('DEEPSTALL_STAGE', 'DEEPSTALL_STAGE_WAIT_FOR_BREAKOUT', 'Waiting to breakout of the loiter to fly the approach.').
enum_entry_description('DEEPSTALL_STAGE', 'DEEPSTALL_STAGE_FLY_TO_ARC', 'Flying to the first arc point to turn around to the landing point.').
enum_entry_description('DEEPSTALL_STAGE', 'DEEPSTALL_STAGE_ARC', 'Turning around back to the deepstall landing point.').
enum_entry_description('DEEPSTALL_STAGE', 'DEEPSTALL_STAGE_APPROACH', 'Approaching the landing point.').
enum_entry_description('DEEPSTALL_STAGE', 'DEEPSTALL_STAGE_LAND', 'Stalling and steering towards the land point.').
enum_entry_description('MAV_CMD', 'MAV_CMD_RESET_MPPT', 'Mission command to reset Maximum Power Point Tracker (MPPT)').
enum_entry_description('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL', 'Mission command to perform a power cycle on payload').
enum_entry_description('GSM_LINK_TYPE', 'GSM_LINK_TYPE_NONE', 'no service').
enum_entry_description('GSM_LINK_TYPE', 'GSM_LINK_TYPE_UNKNOWN', 'link type unknown').
enum_entry_description('GSM_LINK_TYPE', 'GSM_LINK_TYPE_2G', '2G (GSM/GRPS/EDGE) link').
enum_entry_description('GSM_LINK_TYPE', 'GSM_LINK_TYPE_3G', '3G link (WCDMA/HSDPA/HSPA) ').
enum_entry_description('GSM_LINK_TYPE', 'GSM_LINK_TYPE_4G', '4G link (LTE)').
enum_entry_description('GSM_MODEM_TYPE', 'GSM_MODEM_TYPE_UNKNOWN', 'not specified').
enum_entry_description('GSM_MODEM_TYPE', 'GSM_MODEM_TYPE_HUAWEI_E3372', 'HUAWEI LTE USB Stick E3372').
enum_entry_description('FIRMWARE_VERSION_TYPE', 'FIRMWARE_VERSION_TYPE_DEV', 'development release').
enum_entry_description('FIRMWARE_VERSION_TYPE', 'FIRMWARE_VERSION_TYPE_ALPHA', 'alpha release').
enum_entry_description('FIRMWARE_VERSION_TYPE', 'FIRMWARE_VERSION_TYPE_BETA', 'beta release').
enum_entry_description('FIRMWARE_VERSION_TYPE', 'FIRMWARE_VERSION_TYPE_RC', 'release candidate').
enum_entry_description('FIRMWARE_VERSION_TYPE', 'FIRMWARE_VERSION_TYPE_OFFICIAL', 'official stable release').
enum_entry_description('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_GPS', 'GPS failure.').
enum_entry_description('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE', 'Differential pressure sensor failure.').
enum_entry_description('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_ABSOLUTE_PRESSURE', 'Absolute pressure sensor failure.').
enum_entry_description('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_3D_ACCEL', 'Accelerometer sensor failure.').
enum_entry_description('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_3D_GYRO', 'Gyroscope sensor failure.').
enum_entry_description('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_3D_MAG', 'Magnetometer sensor failure.').
enum_entry_description('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_TERRAIN', 'Terrain subsystem failure.').
enum_entry_description('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_BATTERY', 'Battery failure/critical low battery.').
enum_entry_description('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_RC_RECEIVER', 'RC receiver failure/no rc connection.').
enum_entry_description('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_OFFBOARD_LINK', 'Offboard link failure.').
enum_entry_description('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_ENGINE', 'Engine failure.').
enum_entry_description('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_GEOFENCE', 'Geofence violation.').
enum_entry_description('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_ESTIMATOR', 'Estimator failure, for example measurement rejection or large variances.').
enum_entry_description('HL_FAILURE_FLAG', 'HL_FAILURE_FLAG_MISSION', 'Mission failure.').
enum_entry_description('MAV_GOTO', 'MAV_GOTO_DO_HOLD', 'Hold at the current position.').
enum_entry_description('MAV_GOTO', 'MAV_GOTO_DO_CONTINUE', 'Continue with the next item in mission execution.').
enum_entry_description('MAV_GOTO', 'MAV_GOTO_HOLD_AT_CURRENT_POSITION', 'Hold at the current position of the system').
enum_entry_description('MAV_GOTO', 'MAV_GOTO_HOLD_AT_SPECIFIED_POSITION', 'Hold at the position specified in the parameters of the DO_HOLD action').
enum_entry_description('MAV_MODE', 'MAV_MODE_PREFLIGHT', 'System is not ready to fly, booting, calibrating, etc. No flag is set.').
enum_entry_description('MAV_MODE', 'MAV_MODE_STABILIZE_DISARMED', 'System is allowed to be active, under assisted RC control.').
enum_entry_description('MAV_MODE', 'MAV_MODE_STABILIZE_ARMED', 'System is allowed to be active, under assisted RC control.').
enum_entry_description('MAV_MODE', 'MAV_MODE_MANUAL_DISARMED', 'System is allowed to be active, under manual (RC) control, no stabilization').
enum_entry_description('MAV_MODE', 'MAV_MODE_MANUAL_ARMED', 'System is allowed to be active, under manual (RC) control, no stabilization').
enum_entry_description('MAV_MODE', 'MAV_MODE_GUIDED_DISARMED', 'System is allowed to be active, under autonomous control, manual setpoint').
enum_entry_description('MAV_MODE', 'MAV_MODE_GUIDED_ARMED', 'System is allowed to be active, under autonomous control, manual setpoint').
enum_entry_description('MAV_MODE', 'MAV_MODE_AUTO_DISARMED', 'System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)').
enum_entry_description('MAV_MODE', 'MAV_MODE_AUTO_ARMED', 'System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)').
enum_entry_description('MAV_MODE', 'MAV_MODE_TEST_DISARMED', 'UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.').
enum_entry_description('MAV_MODE', 'MAV_MODE_TEST_ARMED', 'UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_3D_GYRO', '0x01 3D gyro').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_3D_ACCEL', '0x02 3D accelerometer').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_3D_MAG', '0x04 3D magnetometer').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE', '0x08 absolute pressure').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE', '0x10 differential pressure').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_GPS', '0x20 GPS').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW', '0x40 optical flow').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_VISION_POSITION', '0x80 computer vision position').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_LASER_POSITION', '0x100 laser based position').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH', '0x200 external ground truth (Vicon or Leica)').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL', '0x400 3D angular rate control').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION', '0x800 attitude stabilization').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_YAW_POSITION', '0x1000 yaw position').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL', '0x2000 z/altitude control').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL', '0x4000 x/y position control').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS', '0x8000 motor outputs / control').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_RC_RECEIVER', '0x10000 rc receiver').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_3D_GYRO2', '0x20000 2nd 3D gyro').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_3D_ACCEL2', '0x40000 2nd 3D accelerometer').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_3D_MAG2', '0x80000 2nd 3D magnetometer').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_GEOFENCE', '0x100000 geofence').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_AHRS', '0x200000 AHRS subsystem health').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_TERRAIN', '0x400000 Terrain subsystem health').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_REVERSE_MOTOR', '0x800000 Motors are reversed').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_LOGGING', '0x1000000 Logging').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_BATTERY', '0x2000000 Battery').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_PROXIMITY', '0x4000000 Proximity').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_SATCOM', '0x8000000 Satellite Communication ').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_PREARM_CHECK', '0x10000000 pre-arm check status. Always healthy when armed').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_OBSTACLE_AVOIDANCE', '0x20000000 Avoidance/collision prevention').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_SENSOR_PROPULSION', '0x40000000 propulsion (actuator, esc, motor or propellor)').
enum_entry_description('MAV_SYS_STATUS_SENSOR', 'MAV_SYS_STATUS_EXTENSION_USED', '0x80000000 Extended bit-field are used for further sensor status bits (needs to be set in onboard_control_sensors_present only)').
enum_entry_description('MAV_SYS_STATUS_SENSOR_EXTENDED', 'MAV_SYS_STATUS_RECOVERY_SYSTEM', '0x01 Recovery system (parachute, balloon, retracts etc)').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_GLOBAL', 'Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL).').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_LOCAL_NED', 'NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_MISSION', 'NOT a coordinate frame, indicates a mission command.').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_GLOBAL_RELATIVE_ALT', '\n          Global (WGS84) coordinate frame + altitude relative to the home position.\n          First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home position.\n        ').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_LOCAL_ENU', 'ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_GLOBAL_INT', 'Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude over mean sea level (MSL).').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_GLOBAL_RELATIVE_ALT_INT', '\n          Global (WGS84) coordinate frame (scaled) + altitude relative to the home position.\n          First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude with 0 being at the altitude of the home position.\n        ').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_LOCAL_OFFSET_NED', 'NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle.').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_BODY_NED', 'Same as MAV_FRAME_LOCAL_NED when used to represent position values. Same as MAV_FRAME_BODY_FRD when used with velocity/acceleration values.').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_BODY_OFFSET_NED', 'This is the same as MAV_FRAME_BODY_FRD.').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_GLOBAL_TERRAIN_ALT', 'Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_GLOBAL_TERRAIN_ALT_INT', 'Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees*1E7, second value / y: longitude in degrees*1E7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_BODY_FRD', 'FRD local frame aligned to the vehicle\'s attitude (x: Forward, y: Right, z: Down) with an origin that travels with vehicle.').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_RESERVED_13', 'MAV_FRAME_BODY_FLU - Body fixed frame of reference, Z-up (x: Forward, y: Left, z: Up).').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_RESERVED_14', 'MAV_FRAME_MOCAP_NED - Odometry local coordinate frame of data given by a motion capture system, Z-down (x: North, y: East, z: Down).').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_RESERVED_15', 'MAV_FRAME_MOCAP_ENU - Odometry local coordinate frame of data given by a motion capture system, Z-up (x: East, y: North, z: Up).').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_RESERVED_16', 'MAV_FRAME_VISION_NED - Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: North, y: East, z: Down).').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_RESERVED_17', 'MAV_FRAME_VISION_ENU - Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: East, y: North, z: Up).').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_RESERVED_18', 'MAV_FRAME_ESTIM_NED - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: North, y: East, z: Down).').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_RESERVED_19', 'MAV_FRAME_ESTIM_ENU - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: East, y: North, z: Up).').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_LOCAL_FRD', 'FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane.').
enum_entry_description('MAV_FRAME', 'MAV_FRAME_LOCAL_FLU', 'FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane.').
enum_entry_description('FENCE_ACTION', 'FENCE_ACTION_NONE', 'Disable fenced mode. If used in a plan this would mean the next fence is disabled.').
enum_entry_description('FENCE_ACTION', 'FENCE_ACTION_GUIDED', 'Fly to geofence MAV_CMD_NAV_FENCE_RETURN_POINT in GUIDED mode. Note: This action is only supported by ArduPlane, and may not be supported in all versions.').
enum_entry_description('FENCE_ACTION', 'FENCE_ACTION_REPORT', 'Report fence breach, but don\'t take action').
enum_entry_description('FENCE_ACTION', 'FENCE_ACTION_GUIDED_THR_PASS', 'Fly to geofence MAV_CMD_NAV_FENCE_RETURN_POINT with manual throttle control in GUIDED mode. Note: This action is only supported by ArduPlane, and may not be supported in all versions.').
enum_entry_description('FENCE_ACTION', 'FENCE_ACTION_RTL', 'Return/RTL mode.').
enum_entry_description('FENCE_ACTION', 'FENCE_ACTION_HOLD', 'Hold at current location.').
enum_entry_description('FENCE_ACTION', 'FENCE_ACTION_TERMINATE', 'Termination failsafe. Motors are shut down (some flight stacks may trigger other failsafe actions).').
enum_entry_description('FENCE_ACTION', 'FENCE_ACTION_LAND', 'Land at current location.').
enum_entry_description('FENCE_BREACH', 'FENCE_BREACH_NONE', 'No last fence breach').
enum_entry_description('FENCE_BREACH', 'FENCE_BREACH_MINALT', 'Breached minimum altitude').
enum_entry_description('FENCE_BREACH', 'FENCE_BREACH_MAXALT', 'Breached maximum altitude').
enum_entry_description('FENCE_BREACH', 'FENCE_BREACH_BOUNDARY', 'Breached fence boundary').
enum_entry_description('FENCE_MITIGATE', 'FENCE_MITIGATE_UNKNOWN', 'Unknown').
enum_entry_description('FENCE_MITIGATE', 'FENCE_MITIGATE_NONE', 'No actions being taken').
enum_entry_description('FENCE_MITIGATE', 'FENCE_MITIGATE_VEL_LIMIT', 'Velocity limiting active to prevent breach').
enum_entry_description('MAV_MOUNT_MODE', 'MAV_MOUNT_MODE_RETRACT', 'Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization').
enum_entry_description('MAV_MOUNT_MODE', 'MAV_MOUNT_MODE_NEUTRAL', 'Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.').
enum_entry_description('MAV_MOUNT_MODE', 'MAV_MOUNT_MODE_MAVLINK_TARGETING', 'Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization').
enum_entry_description('MAV_MOUNT_MODE', 'MAV_MOUNT_MODE_RC_TARGETING', 'Load neutral position and start RC Roll,Pitch,Yaw control with stabilization').
enum_entry_description('MAV_MOUNT_MODE', 'MAV_MOUNT_MODE_GPS_POINT', 'Load neutral position and start to point to Lat,Lon,Alt').
enum_entry_description('MAV_MOUNT_MODE', 'MAV_MOUNT_MODE_SYSID_TARGET', 'Gimbal tracks system with specified system ID').
enum_entry_description('MAV_MOUNT_MODE', 'MAV_MOUNT_MODE_HOME_LOCATION', 'Gimbal tracks home position').
enum_entry_description('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT', 'Gimbal device supports a retracted position.').
enum_entry_description('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL', 'Gimbal device supports a horizontal, forward looking position, stabilized.').
enum_entry_description('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS', 'Gimbal device supports rotating around roll axis.').
enum_entry_description('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW', 'Gimbal device supports to follow a roll angle relative to the vehicle.').
enum_entry_description('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK', 'Gimbal device supports locking to a roll angle (generally that\'s the default with roll stabilized).').
enum_entry_description('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS', 'Gimbal device supports rotating around pitch axis.').
enum_entry_description('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW', 'Gimbal device supports to follow a pitch angle relative to the vehicle.').
enum_entry_description('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK', 'Gimbal device supports locking to a pitch angle (generally that\'s the default with pitch stabilized).').
enum_entry_description('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS', 'Gimbal device supports rotating around yaw axis.').
enum_entry_description('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW', 'Gimbal device supports to follow a yaw angle relative to the vehicle (generally that\'s the default).').
enum_entry_description('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK', 'Gimbal device supports locking to an absolute heading, i.e., yaw angle relative to North (earth frame, often this is an option available).').
enum_entry_description('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW', 'Gimbal device supports yawing/panning infinetely (e.g. using slip disk).').
enum_entry_description('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME', 'Gimbal device supports yaw angles and angular velocities relative to North (earth frame). This usually requires support by an autopilot via AUTOPILOT_STATE_FOR_GIMBAL_DEVICE. Support can go on and off during runtime, which is reported by the flag GIMBAL_DEVICE_FLAGS_CAN_ACCEPT_YAW_IN_EARTH_FRAME.').
enum_entry_description('GIMBAL_DEVICE_CAP_FLAGS', 'GIMBAL_DEVICE_CAP_FLAGS_HAS_RC_INPUTS', 'Gimbal device supports radio control inputs as an alternative input for controlling the gimbal orientation.').
enum_entry_description('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT', 'Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT.').
enum_entry_description('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL', 'Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL.').
enum_entry_description('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS', 'Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS.').
enum_entry_description('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW', 'Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW.').
enum_entry_description('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK', 'Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK.').
enum_entry_description('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS', 'Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS.').
enum_entry_description('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW', 'Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW.').
enum_entry_description('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK', 'Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK.').
enum_entry_description('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS', 'Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS.').
enum_entry_description('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW', 'Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW.').
enum_entry_description('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK', 'Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK.').
enum_entry_description('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW', 'Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW.').
enum_entry_description('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME', 'Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME.').
enum_entry_description('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_HAS_RC_INPUTS', 'Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RC_INPUTS.').
enum_entry_description('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL', 'Gimbal manager supports to point to a local position.').
enum_entry_description('GIMBAL_MANAGER_CAP_FLAGS', 'GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL', 'Gimbal manager supports to point to a global latitude, longitude, altitude position.').
enum_entry_description('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_RETRACT', 'Set to retracted safe position (no stabilization), takes presedence over all other flags.').
enum_entry_description('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_NEUTRAL', 'Set to neutral/default position, taking precedence over all other flags except RETRACT. Neutral is commonly forward-facing and horizontal (roll=pitch=yaw=0) but may be any orientation.').
enum_entry_description('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_ROLL_LOCK', 'Lock roll angle to absolute angle relative to horizon (not relative to vehicle). This is generally the default with a stabilizing gimbal.').
enum_entry_description('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_PITCH_LOCK', 'Lock pitch angle to absolute angle relative to horizon (not relative to vehicle). This is generally the default with a stabilizing gimbal.').
enum_entry_description('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_YAW_LOCK', 'Lock yaw angle to absolute angle relative to North (not relative to vehicle). If this flag is set, the yaw angle and z component of angular velocity are relative to North (earth frame, x-axis pointing North), else they are relative to the vehicle heading (vehicle frame, earth frame rotated so that the x-axis is pointing forward).').
enum_entry_description('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME', 'Yaw angle and z component of angular velocity are relative to the vehicle heading (vehicle frame, earth frame rotated such that the x-axis is pointing forward).').
enum_entry_description('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME', 'Yaw angle and z component of angular velocity are relative to North (earth frame, x-axis is pointing North).').
enum_entry_description('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME', 'Gimbal device can accept yaw angle inputs relative to North (earth frame). This flag is only for reporting (attempts to set this flag are ignored).').
enum_entry_description('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE', 'The gimbal orientation is set exclusively by the RC signals feed to the gimbal\'s radio control inputs. MAVLink messages for setting the gimbal orientation (GIMBAL_DEVICE_SET_ATTITUDE) are ignored.').
enum_entry_description('GIMBAL_DEVICE_FLAGS', 'GIMBAL_DEVICE_FLAGS_RC_MIXED', 'The gimbal orientation is determined by combining/mixing the RC signals feed to the gimbal\'s radio control inputs and the MAVLink messages for setting the gimbal orientation (GIMBAL_DEVICE_SET_ATTITUDE). How these two controls are combined or mixed is not defined by the protocol but is up to the implementation.').
enum_entry_description('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_RETRACT', 'Based on GIMBAL_DEVICE_FLAGS_RETRACT.').
enum_entry_description('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_NEUTRAL', 'Based on GIMBAL_DEVICE_FLAGS_NEUTRAL.').
enum_entry_description('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_ROLL_LOCK', 'Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK.').
enum_entry_description('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_PITCH_LOCK', 'Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK.').
enum_entry_description('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_YAW_LOCK', 'Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK.').
enum_entry_description('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME', 'Based on GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME.').
enum_entry_description('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME', 'Based on GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME.').
enum_entry_description('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME', 'Based on GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME.').
enum_entry_description('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_RC_EXCLUSIVE', 'Based on GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE.').
enum_entry_description('GIMBAL_MANAGER_FLAGS', 'GIMBAL_MANAGER_FLAGS_RC_MIXED', 'Based on GIMBAL_DEVICE_FLAGS_RC_MIXED.').
enum_entry_description('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT', 'Gimbal device is limited by hardware roll limit.').
enum_entry_description('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT', 'Gimbal device is limited by hardware pitch limit.').
enum_entry_description('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT', 'Gimbal device is limited by hardware yaw limit.').
enum_entry_description('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR', 'There is an error with the gimbal encoders.').
enum_entry_description('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR', 'There is an error with the gimbal power source.').
enum_entry_description('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR', 'There is an error with the gimbal motors.').
enum_entry_description('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR', 'There is an error with the gimbal\'s software.').
enum_entry_description('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR', 'There is an error with the gimbal\'s communication.').
enum_entry_description('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING', 'Gimbal device is currently calibrating.').
enum_entry_description('GIMBAL_DEVICE_ERROR_FLAGS', 'GIMBAL_DEVICE_ERROR_FLAGS_NO_MANAGER', 'Gimbal device is not assigned to a gimbal manager.').
enum_entry_description('GRIPPER_ACTIONS', 'GRIPPER_ACTION_RELEASE', 'Gripper release cargo.').
enum_entry_description('GRIPPER_ACTIONS', 'GRIPPER_ACTION_GRAB', 'Gripper grab onto cargo.').
enum_entry_description('WINCH_ACTIONS', 'WINCH_RELAXED', 'Allow motor to freewheel.').
enum_entry_description('WINCH_ACTIONS', 'WINCH_RELATIVE_LENGTH_CONTROL', 'Wind or unwind specified length of line, optionally using specified rate.').
enum_entry_description('WINCH_ACTIONS', 'WINCH_RATE_CONTROL', 'Wind or unwind line at specified rate.').
enum_entry_description('WINCH_ACTIONS', 'WINCH_LOCK', 'Perform the locking sequence to relieve motor while in the fully retracted position. Only action and instance command parameters are used, others are ignored.').
enum_entry_description('WINCH_ACTIONS', 'WINCH_DELIVER', 'Sequence of drop, slow down, touch down, reel up, lock. Only action and instance command parameters are used, others are ignored.').
enum_entry_description('WINCH_ACTIONS', 'WINCH_HOLD', 'Engage motor and hold current position. Only action and instance command parameters are used, others are ignored.').
enum_entry_description('WINCH_ACTIONS', 'WINCH_RETRACT', 'Return the reel to the fully retracted position. Only action and instance command parameters are used, others are ignored.').
enum_entry_description('WINCH_ACTIONS', 'WINCH_LOAD_LINE', 'Load the reel with line. The winch will calculate the total loaded length and stop when the tension exceeds a threshold. Only action and instance command parameters are used, others are ignored.').
enum_entry_description('WINCH_ACTIONS', 'WINCH_ABANDON_LINE', 'Spool out the entire length of the line. Only action and instance command parameters are used, others are ignored.').
enum_entry_description('WINCH_ACTIONS', 'WINCH_LOAD_PAYLOAD', 'Spools out just enough to present the hook to the user to load the payload. Only action and instance command parameters are used, others are ignored').
enum_entry_description('UAVCAN_NODE_HEALTH', 'UAVCAN_NODE_HEALTH_OK', 'The node is functioning properly.').
enum_entry_description('UAVCAN_NODE_HEALTH', 'UAVCAN_NODE_HEALTH_WARNING', 'A critical parameter went out of range or the node has encountered a minor failure.').
enum_entry_description('UAVCAN_NODE_HEALTH', 'UAVCAN_NODE_HEALTH_ERROR', 'The node has encountered a major failure.').
enum_entry_description('UAVCAN_NODE_HEALTH', 'UAVCAN_NODE_HEALTH_CRITICAL', 'The node has suffered a fatal malfunction.').
enum_entry_description('UAVCAN_NODE_MODE', 'UAVCAN_NODE_MODE_OPERATIONAL', 'The node is performing its primary functions.').
enum_entry_description('UAVCAN_NODE_MODE', 'UAVCAN_NODE_MODE_INITIALIZATION', 'The node is initializing; this mode is entered immediately after startup.').
enum_entry_description('UAVCAN_NODE_MODE', 'UAVCAN_NODE_MODE_MAINTENANCE', 'The node is under maintenance.').
enum_entry_description('UAVCAN_NODE_MODE', 'UAVCAN_NODE_MODE_SOFTWARE_UPDATE', 'The node is in the process of updating its software.').
enum_entry_description('UAVCAN_NODE_MODE', 'UAVCAN_NODE_MODE_OFFLINE', 'The node is no longer available online.').
enum_entry_description('ESC_CONNECTION_TYPE', 'ESC_CONNECTION_TYPE_PPM', 'Traditional PPM ESC.').
enum_entry_description('ESC_CONNECTION_TYPE', 'ESC_CONNECTION_TYPE_SERIAL', 'Serial Bus connected ESC.').
enum_entry_description('ESC_CONNECTION_TYPE', 'ESC_CONNECTION_TYPE_ONESHOT', 'One Shot PPM ESC.').
enum_entry_description('ESC_CONNECTION_TYPE', 'ESC_CONNECTION_TYPE_I2C', 'I2C ESC.').
enum_entry_description('ESC_CONNECTION_TYPE', 'ESC_CONNECTION_TYPE_CAN', 'CAN-Bus ESC.').
enum_entry_description('ESC_CONNECTION_TYPE', 'ESC_CONNECTION_TYPE_DSHOT', 'DShot ESC.').
enum_entry_description('ESC_FAILURE_FLAGS', 'ESC_FAILURE_NONE', 'No ESC failure.').
enum_entry_description('ESC_FAILURE_FLAGS', 'ESC_FAILURE_OVER_CURRENT', 'Over current failure.').
enum_entry_description('ESC_FAILURE_FLAGS', 'ESC_FAILURE_OVER_VOLTAGE', 'Over voltage failure.').
enum_entry_description('ESC_FAILURE_FLAGS', 'ESC_FAILURE_OVER_TEMPERATURE', 'Over temperature failure.').
enum_entry_description('ESC_FAILURE_FLAGS', 'ESC_FAILURE_OVER_RPM', 'Over RPM failure.').
enum_entry_description('ESC_FAILURE_FLAGS', 'ESC_FAILURE_INCONSISTENT_CMD', 'Inconsistent command failure i.e. out of bounds.').
enum_entry_description('ESC_FAILURE_FLAGS', 'ESC_FAILURE_MOTOR_STUCK', 'Motor stuck failure.').
enum_entry_description('ESC_FAILURE_FLAGS', 'ESC_FAILURE_GENERIC', 'Generic ESC failure.').
enum_entry_description('STORAGE_STATUS', 'STORAGE_STATUS_EMPTY', 'Storage is missing (no microSD card loaded for example.)').
enum_entry_description('STORAGE_STATUS', 'STORAGE_STATUS_UNFORMATTED', 'Storage present but unformatted.').
enum_entry_description('STORAGE_STATUS', 'STORAGE_STATUS_READY', 'Storage present and ready.').
enum_entry_description('STORAGE_STATUS', 'STORAGE_STATUS_NOT_SUPPORTED', 'Camera does not supply storage status information. Capacity information in STORAGE_INFORMATION fields will be ignored.').
enum_entry_description('STORAGE_TYPE', 'STORAGE_TYPE_UNKNOWN', 'Storage type is not known.').
enum_entry_description('STORAGE_TYPE', 'STORAGE_TYPE_USB_STICK', 'Storage type is USB device.').
enum_entry_description('STORAGE_TYPE', 'STORAGE_TYPE_SD', 'Storage type is SD card.').
enum_entry_description('STORAGE_TYPE', 'STORAGE_TYPE_MICROSD', 'Storage type is microSD card.').
enum_entry_description('STORAGE_TYPE', 'STORAGE_TYPE_CF', 'Storage type is CFast.').
enum_entry_description('STORAGE_TYPE', 'STORAGE_TYPE_CFE', 'Storage type is CFexpress.').
enum_entry_description('STORAGE_TYPE', 'STORAGE_TYPE_XQD', 'Storage type is XQD.').
enum_entry_description('STORAGE_TYPE', 'STORAGE_TYPE_HD', 'Storage type is HD mass storage type.').
enum_entry_description('STORAGE_TYPE', 'STORAGE_TYPE_OTHER', 'Storage type is other, not listed type.').
enum_entry_description('STORAGE_USAGE_FLAG', 'STORAGE_USAGE_FLAG_SET', 'Always set to 1 (indicates STORAGE_INFORMATION.storage_usage is supported).').
enum_entry_description('STORAGE_USAGE_FLAG', 'STORAGE_USAGE_FLAG_PHOTO', 'Storage for saving photos.').
enum_entry_description('STORAGE_USAGE_FLAG', 'STORAGE_USAGE_FLAG_VIDEO', 'Storage for saving videos.').
enum_entry_description('STORAGE_USAGE_FLAG', 'STORAGE_USAGE_FLAG_LOGS', 'Storage for saving logs.').
enum_entry_description('ORBIT_YAW_BEHAVIOUR', 'ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER', 'Vehicle front points to the center (default).').
enum_entry_description('ORBIT_YAW_BEHAVIOUR', 'ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING', 'Vehicle front holds heading when message received.').
enum_entry_description('ORBIT_YAW_BEHAVIOUR', 'ORBIT_YAW_BEHAVIOUR_UNCONTROLLED', 'Yaw uncontrolled.').
enum_entry_description('ORBIT_YAW_BEHAVIOUR', 'ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE', 'Vehicle front follows flight path (tangential to circle).').
enum_entry_description('ORBIT_YAW_BEHAVIOUR', 'ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED', 'Yaw controlled by RC input.').
enum_entry_description('WIFI_CONFIG_AP_RESPONSE', 'WIFI_CONFIG_AP_RESPONSE_UNDEFINED', 'Undefined response. Likely an indicative of a system that doesn\'t support this request.').
enum_entry_description('WIFI_CONFIG_AP_RESPONSE', 'WIFI_CONFIG_AP_RESPONSE_ACCEPTED', 'Changes accepted.').
enum_entry_description('WIFI_CONFIG_AP_RESPONSE', 'WIFI_CONFIG_AP_RESPONSE_REJECTED', 'Changes rejected.').
enum_entry_description('WIFI_CONFIG_AP_RESPONSE', 'WIFI_CONFIG_AP_RESPONSE_MODE_ERROR', 'Invalid Mode.').
enum_entry_description('WIFI_CONFIG_AP_RESPONSE', 'WIFI_CONFIG_AP_RESPONSE_SSID_ERROR', 'Invalid SSID.').
enum_entry_description('WIFI_CONFIG_AP_RESPONSE', 'WIFI_CONFIG_AP_RESPONSE_PASSWORD_ERROR', 'Invalid Password.').
enum_entry_description('CELLULAR_CONFIG_RESPONSE', 'CELLULAR_CONFIG_RESPONSE_ACCEPTED', 'Changes accepted.').
enum_entry_description('CELLULAR_CONFIG_RESPONSE', 'CELLULAR_CONFIG_RESPONSE_APN_ERROR', 'Invalid APN.').
enum_entry_description('CELLULAR_CONFIG_RESPONSE', 'CELLULAR_CONFIG_RESPONSE_PIN_ERROR', 'Invalid PIN.').
enum_entry_description('CELLULAR_CONFIG_RESPONSE', 'CELLULAR_CONFIG_RESPONSE_REJECTED', 'Changes rejected.').
enum_entry_description('CELLULAR_CONFIG_RESPONSE', 'CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED', 'PUK is required to unblock SIM card.').
enum_entry_description('WIFI_CONFIG_AP_MODE', 'WIFI_CONFIG_AP_MODE_UNDEFINED', 'WiFi mode is undefined.').
enum_entry_description('WIFI_CONFIG_AP_MODE', 'WIFI_CONFIG_AP_MODE_AP', 'WiFi configured as an access point.').
enum_entry_description('WIFI_CONFIG_AP_MODE', 'WIFI_CONFIG_AP_MODE_STATION', 'WiFi configured as a station connected to an existing local WiFi network.').
enum_entry_description('WIFI_CONFIG_AP_MODE', 'WIFI_CONFIG_AP_MODE_DISABLED', 'WiFi disabled.').
enum_entry_description('COMP_METADATA_TYPE', 'COMP_METADATA_TYPE_GENERAL', 'General information about the component. General metadata includes information about other metadata types supported by the component. Files of this type must be supported, and must be downloadable from vehicle using a MAVLink FTP URI.').
enum_entry_description('COMP_METADATA_TYPE', 'COMP_METADATA_TYPE_PARAMETER', 'Parameter meta data.').
enum_entry_description('COMP_METADATA_TYPE', 'COMP_METADATA_TYPE_COMMANDS', 'Meta data that specifies which commands and command parameters the vehicle supports. (WIP)').
enum_entry_description('COMP_METADATA_TYPE', 'COMP_METADATA_TYPE_PERIPHERALS', 'Meta data that specifies external non-MAVLink peripherals.').
enum_entry_description('COMP_METADATA_TYPE', 'COMP_METADATA_TYPE_EVENTS', 'Meta data for the events interface.').
enum_entry_description('COMP_METADATA_TYPE', 'COMP_METADATA_TYPE_ACTUATORS', 'Meta data for actuator configuration (motors, servos and vehicle geometry) and testing.').
enum_entry_description('ACTUATOR_CONFIGURATION', 'ACTUATOR_CONFIGURATION_NONE', 'Do nothing.').
enum_entry_description('ACTUATOR_CONFIGURATION', 'ACTUATOR_CONFIGURATION_BEEP', 'Command the actuator to beep now.').
enum_entry_description('ACTUATOR_CONFIGURATION', 'ACTUATOR_CONFIGURATION_3D_MODE_ON', 'Permanently set the actuator (ESC) to 3D mode (reversible thrust).').
enum_entry_description('ACTUATOR_CONFIGURATION', 'ACTUATOR_CONFIGURATION_3D_MODE_OFF', 'Permanently set the actuator (ESC) to non 3D mode (non-reversible thrust).').
enum_entry_description('ACTUATOR_CONFIGURATION', 'ACTUATOR_CONFIGURATION_SPIN_DIRECTION1', 'Permanently set the actuator (ESC) to spin direction 1 (which can be clockwise or counter-clockwise).').
enum_entry_description('ACTUATOR_CONFIGURATION', 'ACTUATOR_CONFIGURATION_SPIN_DIRECTION2', 'Permanently set the actuator (ESC) to spin direction 2 (opposite of direction 1).').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_NONE', 'No function (disabled).').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR1', 'Motor 1').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR2', 'Motor 2').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR3', 'Motor 3').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR4', 'Motor 4').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR5', 'Motor 5').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR6', 'Motor 6').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR7', 'Motor 7').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR8', 'Motor 8').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR9', 'Motor 9').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR10', 'Motor 10').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR11', 'Motor 11').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR12', 'Motor 12').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR13', 'Motor 13').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR14', 'Motor 14').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR15', 'Motor 15').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_MOTOR16', 'Motor 16').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO1', 'Servo 1').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO2', 'Servo 2').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO3', 'Servo 3').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO4', 'Servo 4').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO5', 'Servo 5').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO6', 'Servo 6').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO7', 'Servo 7').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO8', 'Servo 8').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO9', 'Servo 9').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO10', 'Servo 10').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO11', 'Servo 11').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO12', 'Servo 12').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO13', 'Servo 13').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO14', 'Servo 14').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO15', 'Servo 15').
enum_entry_description('ACTUATOR_OUTPUT_FUNCTION', 'ACTUATOR_OUTPUT_FUNCTION_SERVO16', 'Servo 16').
enum_entry_description('AUTOTUNE_AXIS', 'AUTOTUNE_AXIS_DEFAULT', 'Flight stack tunes axis according to its default settings.').
enum_entry_description('AUTOTUNE_AXIS', 'AUTOTUNE_AXIS_ROLL', 'Autotune roll axis.').
enum_entry_description('AUTOTUNE_AXIS', 'AUTOTUNE_AXIS_PITCH', 'Autotune pitch axis.').
enum_entry_description('AUTOTUNE_AXIS', 'AUTOTUNE_AXIS_YAW', 'Autotune yaw axis.').
enum_entry_description('PREFLIGHT_STORAGE_PARAMETER_ACTION', 'PARAM_READ_PERSISTENT', 'Read all parameters from persistent storage. Replaces values in volatile storage.').
enum_entry_description('PREFLIGHT_STORAGE_PARAMETER_ACTION', 'PARAM_WRITE_PERSISTENT', 'Write all parameter values to persistent storage (flash/EEPROM)').
enum_entry_description('PREFLIGHT_STORAGE_PARAMETER_ACTION', 'PARAM_RESET_CONFIG_DEFAULT', 'Reset all user configurable parameters to their default value (including airframe selection, sensor calibration data, safety settings, and so on). Does not reset values that contain operation counters and vehicle computed statistics.').
enum_entry_description('PREFLIGHT_STORAGE_PARAMETER_ACTION', 'PARAM_RESET_SENSOR_DEFAULT', 'Reset only sensor calibration parameters to factory defaults (or firmware default if not available)').
enum_entry_description('PREFLIGHT_STORAGE_PARAMETER_ACTION', 'PARAM_RESET_ALL_DEFAULT', 'Reset all parameters, including operation counters, to default values').
enum_entry_description('PREFLIGHT_STORAGE_MISSION_ACTION', 'MISSION_READ_PERSISTENT', 'Read current mission data from persistent storage').
enum_entry_description('PREFLIGHT_STORAGE_MISSION_ACTION', 'MISSION_WRITE_PERSISTENT', 'Write current mission data to persistent storage').
enum_entry_description('PREFLIGHT_STORAGE_MISSION_ACTION', 'MISSION_RESET_DEFAULT', 'Erase all mission data stored on the vehicle (both persistent and volatile storage)').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_WAYPOINT', 'Navigate to waypoint. This is intended for use in missions (for guided commands outside of missions use MAV_CMD_DO_REPOSITION).').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_LOITER_UNLIM', 'Loiter around this waypoint an unlimited amount of time').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_LOITER_TURNS', 'Loiter around this waypoint for X turns').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_LOITER_TIME', 'Loiter at the specified latitude, longitude and altitude for a certain amount of time. Multicopter vehicles stop at the point (within a vehicle-specific acceptance radius). Forward-only moving vehicles (e.g. fixed-wing) circle the point with the specified radius/direction. If the Heading Required parameter (2) is non-zero forward moving aircraft will only leave the loiter circle once heading towards the next waypoint.').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_RETURN_TO_LAUNCH', 'Return to launch location').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_LAND', 'Land at location.').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF', 'Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode.').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_LAND_LOCAL', 'Land at local position (local frame only)').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF_LOCAL', 'Takeoff from local position (local frame only)').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_FOLLOW', 'Vehicle following, i.e. this waypoint represents the position of a moving vehicle').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT', 'Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don\'t proceed to the next command until the desired altitude is reached.').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_LOITER_TO_ALT', 'Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don\'t consider the navigation command complete (don\'t leave loiter) until the altitude has been reached. Additionally, if the Heading Required parameter is non-zero the aircraft will not leave the loiter until heading toward the next waypoint.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_FOLLOW', 'Begin following a target').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_FOLLOW_REPOSITION', 'Reposition the MAV after a follow target command has been sent').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_ORBIT', 'Start orbiting on the circumference of a circle defined by the parameters. Setting values to NaN/INT32_MAX (as appropriate) results in using defaults.').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_ROI', 'Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle\'s control system to control the vehicle attitude and the attitude of various sensors such as cameras.').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_PATHPLANNING', 'Control autonomous path planning on the MAV.').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_SPLINE_WAYPOINT', 'Navigate to waypoint using a spline path.').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_VTOL_TAKEOFF', 'Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The command should be ignored by vehicles that dont support both VTOL and fixed-wing flight (multicopters, boats,etc.).').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_VTOL_LAND', 'Land using VTOL mode').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_GUIDED_ENABLE', 'hand control over to an external controller').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_DELAY', 'Delay the next navigation command a number of seconds or until a specified time').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_PAYLOAD_PLACE', 'Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload.').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_LAST', 'NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration').
enum_entry_description('MAV_CMD', 'MAV_CMD_CONDITION_DELAY', 'Delay mission state machine.').
enum_entry_description('MAV_CMD', 'MAV_CMD_CONDITION_CHANGE_ALT', 'Ascend/descend to target altitude at specified rate. Delay mission state machine until desired altitude reached.').
enum_entry_description('MAV_CMD', 'MAV_CMD_CONDITION_DISTANCE', 'Delay mission state machine until within desired distance of next NAV point.').
enum_entry_description('MAV_CMD', 'MAV_CMD_CONDITION_YAW', 'Reach a certain target angle.').
enum_entry_description('MAV_CMD', 'MAV_CMD_CONDITION_LAST', 'NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_MODE', 'Set system mode.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_JUMP', 'Jump to the desired command in the mission list.  Repeat this action only the specified number of times').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_CHANGE_SPEED', 'Change speed and/or throttle set points. The value persists until it is overridden or there is a mode change.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_HOME', '\n          Sets the home position to either to the current position or a specified position.\n          The home position is the default position that the system will return to and land on.\n          The position is set automatically by the system during the takeoff (and may also be set using this command).\n          Note: the current home position may be emitted in a HOME_POSITION message on request (using MAV_CMD_REQUEST_MESSAGE with param1=242).\n        ').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_PARAMETER', 'Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_RELAY', 'Set a relay to a condition.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_REPEAT_RELAY', 'Cycle a relay on and off for a desired number of cycles with a desired period.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_SERVO', 'Set a servo to a desired PWM value.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_REPEAT_SERVO', 'Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_FLIGHTTERMINATION', 'Terminate flight immediately.\n          Flight termination immediately and irreversably terminates the current flight, returning the vehicle to ground.\n          The vehicle will ignore RC or other input until it has been power-cycled.\n          Termination may trigger safety measures, including: disabling motors and deployment of parachute on multicopters, and setting flight surfaces to initiate a landing pattern on fixed-wing).\n          On multicopters without a parachute it may trigger a crash landing.\n          Support for this command can be tested using the protocol bit: MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION.\n          Support for this command can also be tested by sending the command with param1=0 (< 0.5); the ACK should be either MAV_RESULT_FAILED or MAV_RESULT_UNSUPPORTED.\n        ').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_CHANGE_ALTITUDE', 'Change altitude set point.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_ACTUATOR', 'Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs (e.g. on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter).').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_LAND_START', 'Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts.\n\t  It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used.\n\t  The Latitude/Longitude/Altitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence.\n\t').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_RALLY_LAND', 'Mission command to perform a landing from a rally point.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_GO_AROUND', 'Mission command to safely abort an autonomous landing.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_REPOSITION', 'Reposition the vehicle to a specific WGS84 global position. This command is intended for guided commands (for missions use MAV_CMD_NAV_WAYPOINT instead).').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_PAUSE_CONTINUE', 'If in a GPS controlled position mode, hold the current position or continue.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_REVERSE', 'Set moving direction to forward or reverse.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_ROI_LOCATION', 'Sets the region of interest (ROI) to a location. This can then be used by the vehicle\'s control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal is not to react to this message.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET', 'Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicle\'s control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_ROI_NONE', 'Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicle\'s control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. After this command the gimbal manager should go back to manual input if available, and otherwise assume a neutral position.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_ROI_SYSID', 'Mount tracks system with specified system ID. Determination of target vehicle position may be done with GLOBAL_POSITION_INT or any other means. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_CONTROL_VIDEO', 'Control onboard camera system.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_ROI', 'Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle\'s control system to control the vehicle attitude and the attitude of various sensors such as cameras.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONFIGURE', 'Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONTROL', 'Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONFIGURE', 'Mission command to configure a camera or antenna mount').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL', 'Mission command to control a camera or antenna mount').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_DIST', 'Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_FENCE_ENABLE', 'Mission command to enable the geofence').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_PARACHUTE', 'Mission item/command to release a parachute or enable/disable auto release.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_MOTOR_TEST', 'Command to perform motor test.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_INVERTED_FLIGHT', 'Change to/from inverted flight.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_GRIPPER', 'Mission command to operate a gripper.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_AUTOTUNE_ENABLE', 'Enable/disable autotune.').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_SET_YAW_SPEED', 'Sets a desired vehicle turn angle and speed change.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL', 'Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL_QUAT', 'Mission command to control a camera or antenna mount, using a quaternion as reference.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_GUIDED_MASTER', 'set id of master controller').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_GUIDED_LIMITS', 'Set limits for external control').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_ENGINE_CONTROL', 'Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_MISSION_CURRENT', '\n          Set the mission item with sequence number seq as the current item and emit MISSION_CURRENT (whether or not the mission number changed).\n          If a mission is currently being executed, the system will continue to this new mission item on the shortest path, skipping any intermediate mission items.\n\t  Note that mission jump repeat counters are not reset unless param2 is set (see MAV_CMD_DO_JUMP param2).\n\n          This command may trigger a mission state-machine change on some systems: for example from MISSION_STATE_NOT_STARTED or MISSION_STATE_PAUSED to MISSION_STATE_ACTIVE.\n          If the system is in mission mode, on those systems this command might therefore start, restart or resume the mission.\n          If the system is not in mission mode this command must not trigger a switch to mission mode.\n\n          The mission may be "reset" using param2.\n          Resetting sets jump counters to initial values (to reset counters without changing the current mission item set the param1 to `-1`).\n          Resetting also explicitly changes a mission state of MISSION_STATE_COMPLETE to MISSION_STATE_PAUSED or MISSION_STATE_ACTIVE, potentially allowing it to resume when it is (next) in a mission mode.\n\n\t  The command will ACK with MAV_RESULT_FAILED if the sequence number is out of range (including if there is no mission item).\n        ').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_LAST', 'NOP - This command is only used to mark the upper limit of the DO commands in the enumeration').
enum_entry_description('MAV_CMD', 'MAV_CMD_PREFLIGHT_CALIBRATION', 'Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero.').
enum_entry_description('MAV_CMD', 'MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS', 'Set sensor offsets. This command will be only accepted if in pre-flight mode.').
enum_entry_description('MAV_CMD', 'MAV_CMD_PREFLIGHT_UAVCAN', 'Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to the legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during initial vehicle configuration (it is not a normal pre-flight command and has been poorly named).').
enum_entry_description('MAV_CMD', 'MAV_CMD_PREFLIGHT_STORAGE', 'Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.').
enum_entry_description('MAV_CMD', 'MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN', 'Request the reboot or shutdown of system components.').
enum_entry_description('MAV_CMD', 'MAV_CMD_OVERRIDE_GOTO', 'Override current mission with command to pause mission, pause mission and move to position, continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether it holds in place or moves to another position.').
enum_entry_description('MAV_CMD', 'MAV_CMD_OBLIQUE_SURVEY', 'Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this purpose). The camera is triggered each time this distance is exceeded, then the mount moves to the next position. Params 4~6 set-up the angle limits and number of positions for oblique survey, where mount-enabled vehicles automatically roll the camera between shots to emulate an oblique camera setup (providing an increased HFOV). This command can also be used to set the shutter integration time for the camera.').
enum_entry_description('MAV_CMD', 'MAV_CMD_MISSION_START', 'start running a mission').
enum_entry_description('MAV_CMD', 'MAV_CMD_ACTUATOR_TEST', 'Actuator testing command. This is similar to MAV_CMD_DO_MOTOR_TEST but operates on the level of output functions, i.e. it is possible to test Motor1 independent from which output it is configured on. Autopilots typically refuse this command while armed.').
enum_entry_description('MAV_CMD', 'MAV_CMD_CONFIGURE_ACTUATOR', 'Actuator configuration command.').
enum_entry_description('MAV_CMD', 'MAV_CMD_COMPONENT_ARM_DISARM', 'Arms / Disarms a component').
enum_entry_description('MAV_CMD', 'MAV_CMD_RUN_PREARM_CHECKS', 'Instructs a target system to run pre-arm checks.\n          This allows preflight checks to be run on demand, which may be useful on systems that normally run them at low rate, or which do not trigger checks when the armable state might have changed.\n          This command should return MAV_RESULT_ACCEPTED if it will run the checks.\n          The results of the checks are usually then reported in SYS_STATUS messages (this is system-specific).\n          The command should return MAV_RESULT_TEMPORARILY_REJECTED if the system is already armed.\n        ').
enum_entry_description('MAV_CMD', 'MAV_CMD_ILLUMINATOR_ON_OFF', 'Turns illuminators ON/OFF. An illuminator is a light source that is used for lighting up dark areas external to the sytstem: e.g. a torch or searchlight (as opposed to a light source for illuminating the system itself, e.g. an indicator light).').
enum_entry_description('MAV_CMD', 'MAV_CMD_GET_HOME_POSITION', 'Request the home position from the vehicle.\n\t  The vehicle will ACK the command and then emit the HOME_POSITION message.').
enum_entry_description('MAV_CMD', 'MAV_CMD_INJECT_FAILURE', 'Inject artificial failure for testing purposes. Note that autopilots should implement an additional protection before accepting this command such as a specific param setting.').
enum_entry_description('MAV_CMD', 'MAV_CMD_START_RX_PAIR', 'Starts receiver pairing.').
enum_entry_description('MAV_CMD', 'MAV_CMD_GET_MESSAGE_INTERVAL', '\n          Request the interval between messages for a particular MAVLink message ID.\n          The receiver should ACK the command and then emit its response in a MESSAGE_INTERVAL message.\n        ').
enum_entry_description('MAV_CMD', 'MAV_CMD_SET_MESSAGE_INTERVAL', 'Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM.').
enum_entry_description('MAV_CMD', 'MAV_CMD_REQUEST_MESSAGE', 'Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version of MAV_CMD_SET_MESSAGE_INTERVAL).').
enum_entry_description('MAV_CMD', 'MAV_CMD_REQUEST_PROTOCOL_VERSION', 'Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit their capabilities in an PROTOCOL_VERSION message').
enum_entry_description('MAV_CMD', 'MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES', 'Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in an AUTOPILOT_VERSION message').
enum_entry_description('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_INFORMATION', 'Request camera information (CAMERA_INFORMATION).').
enum_entry_description('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_SETTINGS', 'Request camera settings (CAMERA_SETTINGS).').
enum_entry_description('MAV_CMD', 'MAV_CMD_REQUEST_STORAGE_INFORMATION', 'Request storage information (STORAGE_INFORMATION). Use the command\'s target_component to target a specific component\'s storage.').
enum_entry_description('MAV_CMD', 'MAV_CMD_STORAGE_FORMAT', 'Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command\'s target_component to target a specific component\'s storage.').
enum_entry_description('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS', 'Request camera capture status (CAMERA_CAPTURE_STATUS)').
enum_entry_description('MAV_CMD', 'MAV_CMD_REQUEST_FLIGHT_INFORMATION', 'Request flight information (FLIGHT_INFORMATION)').
enum_entry_description('MAV_CMD', 'MAV_CMD_RESET_CAMERA_SETTINGS', 'Reset all camera settings to Factory Default').
enum_entry_description('MAV_CMD', 'MAV_CMD_SET_CAMERA_MODE', 'Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming.').
enum_entry_description('MAV_CMD', 'MAV_CMD_SET_CAMERA_ZOOM', 'Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success).').
enum_entry_description('MAV_CMD', 'MAV_CMD_SET_CAMERA_FOCUS', 'Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success).').
enum_entry_description('MAV_CMD', 'MAV_CMD_SET_STORAGE_USAGE', 'Set that a particular storage is the preferred location for saving photos, videos, and/or other media (e.g. to set that an SD card is used for storing videos).\n          There can only be one preferred save location for each particular media type: setting a media usage flag will clear/reset that same flag if set on any other storage.\n          If no flag is set the system should use its default storage.\n          A target system can choose to always use default storage, in which case it should ACK the command with MAV_RESULT_UNSUPPORTED.\n          A target system can choose to not allow a particular storage to be set as preferred storage, in which case it should ACK the command with MAV_RESULT_DENIED.').
enum_entry_description('MAV_CMD', 'MAV_CMD_JUMP_TAG', 'Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_JUMP_TAG', 'Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW', 'Set gimbal manager pitch/yaw setpoints (low rate command). It is possible to set combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used to signal unset. Note: only the gimbal manager will react to this command - it will be ignored by a gimbal device. Use GIMBAL_MANAGER_SET_PITCHYAW if you need to stream pitch/yaw setpoints at higher rate. ').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE', 'Gimbal configuration to set which sysid/compid is in primary and secondary control.').
enum_entry_description('MAV_CMD', 'MAV_CMD_IMAGE_START_CAPTURE', 'Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values.').
enum_entry_description('MAV_CMD', 'MAV_CMD_IMAGE_STOP_CAPTURE', 'Stop image capture sequence Use NaN for reserved values.').
enum_entry_description('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE', 'Re-request a CAMERA_IMAGE_CAPTURED message.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_TRIGGER_CONTROL', 'Enable or disable on-board camera triggering system.').
enum_entry_description('MAV_CMD', 'MAV_CMD_CAMERA_TRACK_POINT', 'If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking.').
enum_entry_description('MAV_CMD', 'MAV_CMD_CAMERA_TRACK_RECTANGLE', 'If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking.').
enum_entry_description('MAV_CMD', 'MAV_CMD_CAMERA_STOP_TRACKING', 'Stops ongoing tracking.').
enum_entry_description('MAV_CMD', 'MAV_CMD_VIDEO_START_CAPTURE', 'Starts video capture (recording).').
enum_entry_description('MAV_CMD', 'MAV_CMD_VIDEO_STOP_CAPTURE', 'Stop the current video capture (recording).').
enum_entry_description('MAV_CMD', 'MAV_CMD_VIDEO_START_STREAMING', 'Start video streaming').
enum_entry_description('MAV_CMD', 'MAV_CMD_VIDEO_STOP_STREAMING', 'Stop the given video stream').
enum_entry_description('MAV_CMD', 'MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION', 'Request video stream information (VIDEO_STREAM_INFORMATION)').
enum_entry_description('MAV_CMD', 'MAV_CMD_REQUEST_VIDEO_STREAM_STATUS', 'Request video stream status (VIDEO_STREAM_STATUS)').
enum_entry_description('MAV_CMD', 'MAV_CMD_LOGGING_START', 'Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)').
enum_entry_description('MAV_CMD', 'MAV_CMD_LOGGING_STOP', 'Request to stop streaming log data over MAVLink').
enum_entry_description('MAV_CMD', 'MAV_CMD_CONTROL_HIGH_LATENCY', 'Request to start/stop transmitting over the high latency telemetry').
enum_entry_description('MAV_CMD', 'MAV_CMD_PANORAMA_CREATE', 'Create a panorama at the current position').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_VTOL_TRANSITION', 'Request VTOL transition').
enum_entry_description('MAV_CMD', 'MAV_CMD_ARM_AUTHORIZATION_REQUEST', 'Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request.\n\t\tIf approved the COMMAND_ACK message progress field should be set with period of time that this authorization is valid in seconds.\n\t\tIf the authorization is denied COMMAND_ACK.result_param2 should be set with one of the reasons in ARM_AUTH_DENIED_REASON.\n        ').
enum_entry_description('MAV_CMD', 'MAV_CMD_SET_GUIDED_SUBMODE_STANDARD', 'This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes.\n                  ').
enum_entry_description('MAV_CMD', 'MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE', 'This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.\n                  ').
enum_entry_description('MAV_CMD', 'MAV_CMD_CONDITION_GATE', 'Delay mission state machine until gate has been reached.').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_FENCE_RETURN_POINT', 'Fence return point (there can only be one such point in a geofence definition). If rally points are supported they should be used instead.').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION', 'Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.\n        ').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION', 'Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.\n        ').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION', 'Circular fence area. The vehicle must stay inside this area.\n        ').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION', 'Circular fence area. The vehicle must stay outside this area.\n        ').
enum_entry_description('MAV_CMD', 'MAV_CMD_NAV_RALLY_POINT', 'Rally point. You can have multiple rally points defined.\n        ').
enum_entry_description('MAV_CMD', 'MAV_CMD_UAVCAN_GET_NODE_INFO', 'Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_ADSB_OUT_IDENT', 'Trigger the start of an ADSB-out IDENT. This should only be used when requested to do so by an Air Traffic Controller in controlled airspace. This starts the IDENT which is then typically held for 18 seconds by the hardware per the Mode A, C, and S transponder spec.').
enum_entry_description('MAV_CMD', 'MAV_CMD_PAYLOAD_PREPARE_DEPLOY', 'Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity.').
enum_entry_description('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL_DEPLOY', 'Control the payload deployment.').
enum_entry_description('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_YAW', 'Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero then use the current vehicle location.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_WINCH', 'Command to operate winch.').
enum_entry_description('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_1', 'User defined waypoint item. Ground Station will show the Vehicle as flying through this item.').
enum_entry_description('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_2', 'User defined waypoint item. Ground Station will show the Vehicle as flying through this item.').
enum_entry_description('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_3', 'User defined waypoint item. Ground Station will show the Vehicle as flying through this item.').
enum_entry_description('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_4', 'User defined waypoint item. Ground Station will show the Vehicle as flying through this item.').
enum_entry_description('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_5', 'User defined waypoint item. Ground Station will show the Vehicle as flying through this item.').
enum_entry_description('MAV_CMD', 'MAV_CMD_SPATIAL_USER_1', 'User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.').
enum_entry_description('MAV_CMD', 'MAV_CMD_SPATIAL_USER_2', 'User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.').
enum_entry_description('MAV_CMD', 'MAV_CMD_SPATIAL_USER_3', 'User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.').
enum_entry_description('MAV_CMD', 'MAV_CMD_SPATIAL_USER_4', 'User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.').
enum_entry_description('MAV_CMD', 'MAV_CMD_SPATIAL_USER_5', 'User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.').
enum_entry_description('MAV_CMD', 'MAV_CMD_USER_1', 'User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.').
enum_entry_description('MAV_CMD', 'MAV_CMD_USER_2', 'User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.').
enum_entry_description('MAV_CMD', 'MAV_CMD_USER_3', 'User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.').
enum_entry_description('MAV_CMD', 'MAV_CMD_USER_4', 'User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.').
enum_entry_description('MAV_CMD', 'MAV_CMD_USER_5', 'User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.').
enum_entry_description('MAV_CMD', 'MAV_CMD_CAN_FORWARD', 'Request forwarding of CAN packets from the given CAN bus to this component. CAN Frames are sent using CAN_FRAME and CANFD_FRAME messages').
enum_entry_description('MAV_DATA_STREAM', 'MAV_DATA_STREAM_ALL', 'Enable all data streams').
enum_entry_description('MAV_DATA_STREAM', 'MAV_DATA_STREAM_RAW_SENSORS', 'Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.').
enum_entry_description('MAV_DATA_STREAM', 'MAV_DATA_STREAM_EXTENDED_STATUS', 'Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS').
enum_entry_description('MAV_DATA_STREAM', 'MAV_DATA_STREAM_RC_CHANNELS', 'Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW').
enum_entry_description('MAV_DATA_STREAM', 'MAV_DATA_STREAM_RAW_CONTROLLER', 'Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.').
enum_entry_description('MAV_DATA_STREAM', 'MAV_DATA_STREAM_POSITION', 'Enable LOCAL_POSITION, GLOBAL_POSITION_INT messages.').
enum_entry_description('MAV_DATA_STREAM', 'MAV_DATA_STREAM_EXTRA1', 'Dependent on the autopilot').
enum_entry_description('MAV_DATA_STREAM', 'MAV_DATA_STREAM_EXTRA2', 'Dependent on the autopilot').
enum_entry_description('MAV_DATA_STREAM', 'MAV_DATA_STREAM_EXTRA3', 'Dependent on the autopilot').
enum_entry_description('MAV_ROI', 'MAV_ROI_NONE', 'No region of interest.').
enum_entry_description('MAV_ROI', 'MAV_ROI_WPNEXT', 'Point toward next waypoint, with optional pitch/roll/yaw offset.').
enum_entry_description('MAV_ROI', 'MAV_ROI_WPINDEX', 'Point toward given waypoint.').
enum_entry_description('MAV_ROI', 'MAV_ROI_LOCATION', 'Point toward fixed location.').
enum_entry_description('MAV_ROI', 'MAV_ROI_TARGET', 'Point toward of given id.').
enum_entry_description('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_UINT8', '8-bit unsigned integer').
enum_entry_description('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_INT8', '8-bit signed integer').
enum_entry_description('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_UINT16', '16-bit unsigned integer').
enum_entry_description('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_INT16', '16-bit signed integer').
enum_entry_description('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_UINT32', '32-bit unsigned integer').
enum_entry_description('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_INT32', '32-bit signed integer').
enum_entry_description('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_UINT64', '64-bit unsigned integer').
enum_entry_description('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_INT64', '64-bit signed integer').
enum_entry_description('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_REAL32', '32-bit floating-point').
enum_entry_description('MAV_PARAM_TYPE', 'MAV_PARAM_TYPE_REAL64', '64-bit floating-point').
enum_entry_description('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_UINT8', '8-bit unsigned integer').
enum_entry_description('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_INT8', '8-bit signed integer').
enum_entry_description('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_UINT16', '16-bit unsigned integer').
enum_entry_description('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_INT16', '16-bit signed integer').
enum_entry_description('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_UINT32', '32-bit unsigned integer').
enum_entry_description('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_INT32', '32-bit signed integer').
enum_entry_description('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_UINT64', '64-bit unsigned integer').
enum_entry_description('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_INT64', '64-bit signed integer').
enum_entry_description('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_REAL32', '32-bit floating-point').
enum_entry_description('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_REAL64', '64-bit floating-point').
enum_entry_description('MAV_PARAM_EXT_TYPE', 'MAV_PARAM_EXT_TYPE_CUSTOM', 'Custom Type').
enum_entry_description('MAV_RESULT', 'MAV_RESULT_ACCEPTED', 'Command is valid (is supported and has valid parameters), and was executed.').
enum_entry_description('MAV_RESULT', 'MAV_RESULT_TEMPORARILY_REJECTED', 'Command is valid, but cannot be executed at this time. This is used to indicate a problem that should be fixed just by waiting (e.g. a state machine is busy, can\'t arm because have not got GPS lock, etc.). Retrying later should work.').
enum_entry_description('MAV_RESULT', 'MAV_RESULT_DENIED', 'Command is invalid (is supported but has invalid parameters). Retrying same command and parameters will not work.').
enum_entry_description('MAV_RESULT', 'MAV_RESULT_UNSUPPORTED', 'Command is not supported (unknown).').
enum_entry_description('MAV_RESULT', 'MAV_RESULT_FAILED', 'Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected problem, i.e. any problem that must be fixed before the command can succeed/be retried. For example, attempting to write a file when out of memory, attempting to arm when sensors are not calibrated, etc.').
enum_entry_description('MAV_RESULT', 'MAV_RESULT_IN_PROGRESS', 'Command is valid and is being executed. This will be followed by further progress updates, i.e. the component may send further COMMAND_ACK messages with result MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation), and must terminate by sending a COMMAND_ACK message with final result of the operation. The COMMAND_ACK.progress field can be used to indicate the progress of the operation.').
enum_entry_description('MAV_RESULT', 'MAV_RESULT_CANCELLED', 'Command has been cancelled (as a result of receiving a COMMAND_CANCEL message).').
enum_entry_description('MAV_RESULT', 'MAV_RESULT_COMMAND_LONG_ONLY', 'Command is valid, but it is only accepted when sent as a COMMAND_LONG (as it has float values for params 5 and 6).').
enum_entry_description('MAV_RESULT', 'MAV_RESULT_COMMAND_INT_ONLY', 'Command is valid, but it is only accepted when sent as a COMMAND_INT (as it encodes a location in params 5, 6 and 7, and hence requires a reference MAV_FRAME).').
enum_entry_description('MAV_RESULT', 'MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME', 'Command is invalid because a frame is required and the specified frame is not supported.').
enum_entry_description('MAV_MISSION_RESULT', 'MAV_MISSION_ACCEPTED', 'mission accepted OK').
enum_entry_description('MAV_MISSION_RESULT', 'MAV_MISSION_ERROR', 'Generic error / not accepting mission commands at all right now.').
enum_entry_description('MAV_MISSION_RESULT', 'MAV_MISSION_UNSUPPORTED_FRAME', 'Coordinate frame is not supported.').
enum_entry_description('MAV_MISSION_RESULT', 'MAV_MISSION_UNSUPPORTED', 'Command is not supported.').
enum_entry_description('MAV_MISSION_RESULT', 'MAV_MISSION_NO_SPACE', 'Mission items exceed storage space.').
enum_entry_description('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID', 'One of the parameters has an invalid value.').
enum_entry_description('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID_PARAM1', 'param1 has an invalid value.').
enum_entry_description('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID_PARAM2', 'param2 has an invalid value.').
enum_entry_description('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID_PARAM3', 'param3 has an invalid value.').
enum_entry_description('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID_PARAM4', 'param4 has an invalid value.').
enum_entry_description('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID_PARAM5_X', 'x / param5 has an invalid value.').
enum_entry_description('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID_PARAM6_Y', 'y / param6 has an invalid value.').
enum_entry_description('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID_PARAM7', 'z / param7 has an invalid value.').
enum_entry_description('MAV_MISSION_RESULT', 'MAV_MISSION_INVALID_SEQUENCE', 'Mission item received out of sequence').
enum_entry_description('MAV_MISSION_RESULT', 'MAV_MISSION_DENIED', 'Not accepting any mission commands from this communication partner.').
enum_entry_description('MAV_MISSION_RESULT', 'MAV_MISSION_OPERATION_CANCELLED', 'Current mission operation cancelled (e.g. mission upload, mission download).').
enum_entry_description('MAV_SEVERITY', 'MAV_SEVERITY_EMERGENCY', 'System is unusable. This is a "panic" condition.').
enum_entry_description('MAV_SEVERITY', 'MAV_SEVERITY_ALERT', 'Action should be taken immediately. Indicates error in non-critical systems.').
enum_entry_description('MAV_SEVERITY', 'MAV_SEVERITY_CRITICAL', 'Action must be taken immediately. Indicates failure in a primary system.').
enum_entry_description('MAV_SEVERITY', 'MAV_SEVERITY_ERROR', 'Indicates an error in secondary/redundant systems.').
enum_entry_description('MAV_SEVERITY', 'MAV_SEVERITY_WARNING', 'Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.').
enum_entry_description('MAV_SEVERITY', 'MAV_SEVERITY_NOTICE', 'An unusual event has occurred, though not an error condition. This should be investigated for the root cause.').
enum_entry_description('MAV_SEVERITY', 'MAV_SEVERITY_INFO', 'Normal operational messages. Useful for logging. No action is required for these messages.').
enum_entry_description('MAV_SEVERITY', 'MAV_SEVERITY_DEBUG', 'Useful non-operational messages that can assist in debugging. These should not occur during normal operation.').
enum_entry_description('MAV_POWER_STATUS', 'MAV_POWER_STATUS_BRICK_VALID', 'main brick power supply valid').
enum_entry_description('MAV_POWER_STATUS', 'MAV_POWER_STATUS_SERVO_VALID', 'main servo power supply valid for FMU').
enum_entry_description('MAV_POWER_STATUS', 'MAV_POWER_STATUS_USB_CONNECTED', 'USB power is connected').
enum_entry_description('MAV_POWER_STATUS', 'MAV_POWER_STATUS_PERIPH_OVERCURRENT', 'peripheral supply is in over-current state').
enum_entry_description('MAV_POWER_STATUS', 'MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT', 'hi-power peripheral supply is in over-current state').
enum_entry_description('MAV_POWER_STATUS', 'MAV_POWER_STATUS_CHANGED', 'Power status has changed since boot').
enum_entry_description('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_DEV_TELEM1', 'First telemetry port').
enum_entry_description('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_DEV_TELEM2', 'Second telemetry port').
enum_entry_description('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_DEV_GPS1', 'First GPS port').
enum_entry_description('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_DEV_GPS2', 'Second GPS port').
enum_entry_description('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_DEV_SHELL', 'system shell').
enum_entry_description('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL0', 'SERIAL0').
enum_entry_description('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL1', 'SERIAL1').
enum_entry_description('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL2', 'SERIAL2').
enum_entry_description('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL3', 'SERIAL3').
enum_entry_description('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL4', 'SERIAL4').
enum_entry_description('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL5', 'SERIAL5').
enum_entry_description('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL6', 'SERIAL6').
enum_entry_description('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL7', 'SERIAL7').
enum_entry_description('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL8', 'SERIAL8').
enum_entry_description('SERIAL_CONTROL_DEV', 'SERIAL_CONTROL_SERIAL9', 'SERIAL9').
enum_entry_description('SERIAL_CONTROL_FLAG', 'SERIAL_CONTROL_FLAG_REPLY', 'Set if this is a reply').
enum_entry_description('SERIAL_CONTROL_FLAG', 'SERIAL_CONTROL_FLAG_RESPOND', 'Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message').
enum_entry_description('SERIAL_CONTROL_FLAG', 'SERIAL_CONTROL_FLAG_EXCLUSIVE', 'Set if access to the serial port should be removed from whatever driver is currently using it, giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without this flag set').
enum_entry_description('SERIAL_CONTROL_FLAG', 'SERIAL_CONTROL_FLAG_BLOCKING', 'Block on writes to the serial port').
enum_entry_description('SERIAL_CONTROL_FLAG', 'SERIAL_CONTROL_FLAG_MULTI', 'Send multiple replies until port is drained').
enum_entry_description('MAV_DISTANCE_SENSOR', 'MAV_DISTANCE_SENSOR_LASER', 'Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units').
enum_entry_description('MAV_DISTANCE_SENSOR', 'MAV_DISTANCE_SENSOR_ULTRASOUND', 'Ultrasound rangefinder, e.g. MaxBotix units').
enum_entry_description('MAV_DISTANCE_SENSOR', 'MAV_DISTANCE_SENSOR_INFRARED', 'Infrared rangefinder, e.g. Sharp units').
enum_entry_description('MAV_DISTANCE_SENSOR', 'MAV_DISTANCE_SENSOR_RADAR', 'Radar type, e.g. uLanding units').
enum_entry_description('MAV_DISTANCE_SENSOR', 'MAV_DISTANCE_SENSOR_UNKNOWN', 'Broken or unknown type, e.g. analog units').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_NONE', 'Roll: 0, Pitch: 0, Yaw: 0').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_YAW_45', 'Roll: 0, Pitch: 0, Yaw: 45').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_YAW_90', 'Roll: 0, Pitch: 0, Yaw: 90').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_YAW_135', 'Roll: 0, Pitch: 0, Yaw: 135').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_YAW_180', 'Roll: 0, Pitch: 0, Yaw: 180').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_YAW_225', 'Roll: 0, Pitch: 0, Yaw: 225').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_YAW_270', 'Roll: 0, Pitch: 0, Yaw: 270').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_YAW_315', 'Roll: 0, Pitch: 0, Yaw: 315').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180', 'Roll: 180, Pitch: 0, Yaw: 0').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180_YAW_45', 'Roll: 180, Pitch: 0, Yaw: 45').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180_YAW_90', 'Roll: 180, Pitch: 0, Yaw: 90').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180_YAW_135', 'Roll: 180, Pitch: 0, Yaw: 135').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_PITCH_180', 'Roll: 0, Pitch: 180, Yaw: 0').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180_YAW_225', 'Roll: 180, Pitch: 0, Yaw: 225').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180_YAW_270', 'Roll: 180, Pitch: 0, Yaw: 270').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180_YAW_315', 'Roll: 180, Pitch: 0, Yaw: 315').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90', 'Roll: 90, Pitch: 0, Yaw: 0').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_YAW_45', 'Roll: 90, Pitch: 0, Yaw: 45').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_YAW_90', 'Roll: 90, Pitch: 0, Yaw: 90').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_YAW_135', 'Roll: 90, Pitch: 0, Yaw: 135').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_270', 'Roll: 270, Pitch: 0, Yaw: 0').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_270_YAW_45', 'Roll: 270, Pitch: 0, Yaw: 45').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_270_YAW_90', 'Roll: 270, Pitch: 0, Yaw: 90').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_270_YAW_135', 'Roll: 270, Pitch: 0, Yaw: 135').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_PITCH_90', 'Roll: 0, Pitch: 90, Yaw: 0').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_PITCH_270', 'Roll: 0, Pitch: 270, Yaw: 0').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_PITCH_180_YAW_90', 'Roll: 0, Pitch: 180, Yaw: 90').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_PITCH_180_YAW_270', 'Roll: 0, Pitch: 180, Yaw: 270').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_90', 'Roll: 90, Pitch: 90, Yaw: 0').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180_PITCH_90', 'Roll: 180, Pitch: 90, Yaw: 0').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_270_PITCH_90', 'Roll: 270, Pitch: 90, Yaw: 0').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_180', 'Roll: 90, Pitch: 180, Yaw: 0').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_270_PITCH_180', 'Roll: 270, Pitch: 180, Yaw: 0').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_270', 'Roll: 90, Pitch: 270, Yaw: 0').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_180_PITCH_270', 'Roll: 180, Pitch: 270, Yaw: 0').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_270_PITCH_270', 'Roll: 270, Pitch: 270, Yaw: 0').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90', 'Roll: 90, Pitch: 180, Yaw: 90').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_YAW_270', 'Roll: 90, Pitch: 0, Yaw: 270').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293', 'Roll: 90, Pitch: 68, Yaw: 293').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_PITCH_315', 'Pitch: 315').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_ROLL_90_PITCH_315', 'Roll: 90, Pitch: 315').
enum_entry_description('MAV_SENSOR_ORIENTATION', 'MAV_SENSOR_ROTATION_CUSTOM', 'Custom orientation').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT', 'Autopilot supports the MISSION_ITEM float message type.\n          Note that MISSION_ITEM is deprecated, and autopilots should use MISSION_INT instead.\n        ').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT', 'Autopilot supports the new param float message type.').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_MISSION_INT', 'Autopilot supports MISSION_ITEM_INT scaled integer message type.\n          Note that this flag must always be set if missions are supported, because missions must always use MISSION_ITEM_INT (rather than MISSION_ITEM, which is deprecated).\n        ').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_COMMAND_INT', 'Autopilot supports COMMAND_INT scaled integer message type.').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE', 'Parameter protocol uses byte-wise encoding of parameter values into param_value (float) fields: https://mavlink.io/en/services/parameter.html#parameter-encoding.\n          Note that either this flag or MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST should be set if the parameter protocol is supported.\n        ').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_FTP', 'Autopilot supports the File Transfer Protocol v1: https://mavlink.io/en/services/ftp.html.').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET', 'Autopilot supports commanding attitude offboard.').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED', 'Autopilot supports commanding position and velocity targets in local NED frame.').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT', 'Autopilot supports commanding position and velocity targets in global scaled integers.').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_TERRAIN', 'Autopilot supports terrain protocol / data handling.').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_RESERVED3', 'Reserved for future use.').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION', 'Autopilot supports the MAV_CMD_DO_FLIGHTTERMINATION command (flight termination).').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION', 'Autopilot supports onboard compass calibration.').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_MAVLINK2', 'Autopilot supports MAVLink version 2.').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_MISSION_FENCE', 'Autopilot supports mission fence protocol.').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_MISSION_RALLY', 'Autopilot supports mission rally point protocol.').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_RESERVED2', 'Reserved for future use.').
enum_entry_description('MAV_PROTOCOL_CAPABILITY', 'MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST', 'Parameter protocol uses C-cast of parameter values to set the param_value (float) fields: https://mavlink.io/en/services/parameter.html#parameter-encoding.\n          Note that either this flag or MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE should be set if the parameter protocol is supported.\n        ').
enum_entry_description('MAV_MISSION_TYPE', 'MAV_MISSION_TYPE_MISSION', 'Items are mission commands for main mission.').
enum_entry_description('MAV_MISSION_TYPE', 'MAV_MISSION_TYPE_FENCE', 'Specifies GeoFence area(s). Items are MAV_CMD_NAV_FENCE_ GeoFence items.').
enum_entry_description('MAV_MISSION_TYPE', 'MAV_MISSION_TYPE_RALLY', 'Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_NAV_RALLY_POINT rally point items.').
enum_entry_description('MAV_MISSION_TYPE', 'MAV_MISSION_TYPE_ALL', 'Only used in MISSION_CLEAR_ALL to clear all mission types.').
enum_entry_description('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_UNKNOWN', 'Unknown type of the estimator.').
enum_entry_description('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_NAIVE', 'This is a naive estimator without any real covariance feedback.').
enum_entry_description('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_VISION', 'Computer vision based estimate. Might be up to scale.').
enum_entry_description('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_VIO', 'Visual-inertial estimate.').
enum_entry_description('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_GPS', 'Plain GPS estimate.').
enum_entry_description('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_GPS_INS', 'Estimator integrating GPS and inertial sensing.').
enum_entry_description('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_MOCAP', 'Estimate from external motion capturing system.').
enum_entry_description('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_LIDAR', 'Estimator based on lidar sensor input.').
enum_entry_description('MAV_ESTIMATOR_TYPE', 'MAV_ESTIMATOR_TYPE_AUTOPILOT', 'Estimator on autopilot.').
enum_entry_description('MAV_BATTERY_TYPE', 'MAV_BATTERY_TYPE_UNKNOWN', 'Not specified.').
enum_entry_description('MAV_BATTERY_TYPE', 'MAV_BATTERY_TYPE_LIPO', 'Lithium polymer battery').
enum_entry_description('MAV_BATTERY_TYPE', 'MAV_BATTERY_TYPE_LIFE', 'Lithium-iron-phosphate battery').
enum_entry_description('MAV_BATTERY_TYPE', 'MAV_BATTERY_TYPE_LION', 'Lithium-ION battery').
enum_entry_description('MAV_BATTERY_TYPE', 'MAV_BATTERY_TYPE_NIMH', 'Nickel metal hydride battery').
enum_entry_description('MAV_BATTERY_FUNCTION', 'MAV_BATTERY_FUNCTION_UNKNOWN', 'Battery function is unknown').
enum_entry_description('MAV_BATTERY_FUNCTION', 'MAV_BATTERY_FUNCTION_ALL', 'Battery supports all flight systems').
enum_entry_description('MAV_BATTERY_FUNCTION', 'MAV_BATTERY_FUNCTION_PROPULSION', 'Battery for the propulsion system').
enum_entry_description('MAV_BATTERY_FUNCTION', 'MAV_BATTERY_FUNCTION_AVIONICS', 'Avionics battery').
enum_entry_description('MAV_BATTERY_FUNCTION', 'MAV_BATTERY_FUNCTION_PAYLOAD', 'Payload battery').
enum_entry_description('MAV_BATTERY_CHARGE_STATE', 'MAV_BATTERY_CHARGE_STATE_UNDEFINED', 'Low battery state is not provided').
enum_entry_description('MAV_BATTERY_CHARGE_STATE', 'MAV_BATTERY_CHARGE_STATE_OK', 'Battery is not in low state. Normal operation.').
enum_entry_description('MAV_BATTERY_CHARGE_STATE', 'MAV_BATTERY_CHARGE_STATE_LOW', 'Battery state is low, warn and monitor close.').
enum_entry_description('MAV_BATTERY_CHARGE_STATE', 'MAV_BATTERY_CHARGE_STATE_CRITICAL', 'Battery state is critical, return or abort immediately.').
enum_entry_description('MAV_BATTERY_CHARGE_STATE', 'MAV_BATTERY_CHARGE_STATE_EMERGENCY', 'Battery state is too low for ordinary abort sequence. Perform fastest possible emergency stop to prevent damage.').
enum_entry_description('MAV_BATTERY_CHARGE_STATE', 'MAV_BATTERY_CHARGE_STATE_FAILED', 'Battery failed, damage unavoidable. Possible causes (faults) are listed in MAV_BATTERY_FAULT.').
enum_entry_description('MAV_BATTERY_CHARGE_STATE', 'MAV_BATTERY_CHARGE_STATE_UNHEALTHY', 'Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible causes (faults) are listed in MAV_BATTERY_FAULT.').
enum_entry_description('MAV_BATTERY_CHARGE_STATE', 'MAV_BATTERY_CHARGE_STATE_CHARGING', 'Battery is charging.').
enum_entry_description('MAV_BATTERY_MODE', 'MAV_BATTERY_MODE_UNKNOWN', 'Battery mode not supported/unknown battery mode/normal operation.').
enum_entry_description('MAV_BATTERY_MODE', 'MAV_BATTERY_MODE_AUTO_DISCHARGING', 'Battery is auto discharging (towards storage level).').
enum_entry_description('MAV_BATTERY_MODE', 'MAV_BATTERY_MODE_HOT_SWAP', 'Battery in hot-swap mode (current limited to prevent spikes that might damage sensitive electrical circuits).').
enum_entry_description('MAV_BATTERY_FAULT', 'MAV_BATTERY_FAULT_DEEP_DISCHARGE', 'Battery has deep discharged.').
enum_entry_description('MAV_BATTERY_FAULT', 'MAV_BATTERY_FAULT_SPIKES', 'Voltage spikes.').
enum_entry_description('MAV_BATTERY_FAULT', 'MAV_BATTERY_FAULT_CELL_FAIL', 'One or more cells have failed. Battery should also report MAV_BATTERY_CHARGE_STATE_FAILE (and should not be used).').
enum_entry_description('MAV_BATTERY_FAULT', 'MAV_BATTERY_FAULT_OVER_CURRENT', 'Over-current fault.').
enum_entry_description('MAV_BATTERY_FAULT', 'MAV_BATTERY_FAULT_OVER_TEMPERATURE', 'Over-temperature fault.').
enum_entry_description('MAV_BATTERY_FAULT', 'MAV_BATTERY_FAULT_UNDER_TEMPERATURE', 'Under-temperature fault.').
enum_entry_description('MAV_BATTERY_FAULT', 'MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE', 'Vehicle voltage is not compatible with this battery (batteries on same power rail should have similar voltage).').
enum_entry_description('MAV_BATTERY_FAULT', 'MAV_BATTERY_FAULT_INCOMPATIBLE_FIRMWARE', 'Battery firmware is not compatible with current autopilot firmware.').
enum_entry_description('MAV_BATTERY_FAULT', 'BATTERY_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION', 'Battery is not compatible due to cell configuration (e.g. 5s1p when vehicle requires 6s).').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_OFF', 'Generator is off.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_READY', 'Generator is ready to start generating power.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_GENERATING', 'Generator is generating power.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_CHARGING', 'Generator is charging the batteries (generating enough power to charge and provide the load).').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER', 'Generator is operating at a reduced maximum power.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_MAXPOWER', 'Generator is providing the maximum output.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING', 'Generator is near the maximum operating temperature, cooling is insufficient.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_OVERTEMP_FAULT', 'Generator hit the maximum operating temperature and shutdown.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING', 'Power electronics are near the maximum operating temperature, cooling is insufficient.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT', 'Power electronics hit the maximum operating temperature and shutdown.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_FAULT', 'Power electronics experienced a fault and shutdown.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_POWERSOURCE_FAULT', 'The power source supplying the generator failed e.g. mechanical generator stopped, tether is no longer providing power, solar cell is in shade, hydrogen reaction no longer happening.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_COMMUNICATION_WARNING', 'Generator controller having communication problems.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_COOLING_WARNING', 'Power electronic or generator cooling system error.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_POWER_RAIL_FAULT', 'Generator controller power rail experienced a fault.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT', 'Generator controller exceeded the overcurrent threshold and shutdown to prevent damage.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT', 'Generator controller detected a high current going into the batteries and shutdown to prevent battery damage.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_OVERVOLTAGE_FAULT', 'Generator controller exceeded it\'s overvoltage threshold and shutdown to prevent it exceeding the voltage rating.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_BATTERY_UNDERVOLT_FAULT', 'Batteries are under voltage (generator will not start).').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_START_INHIBITED', 'Generator start is inhibited by e.g. a safety switch.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED', 'Generator requires maintenance.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_WARMING_UP', 'Generator is not ready to generate yet.').
enum_entry_description('MAV_GENERATOR_STATUS_FLAG', 'MAV_GENERATOR_STATUS_FLAG_IDLE', 'Generator is idle.').
enum_entry_description('MAV_VTOL_STATE', 'MAV_VTOL_STATE_UNDEFINED', 'MAV is not configured as VTOL').
enum_entry_description('MAV_VTOL_STATE', 'MAV_VTOL_STATE_TRANSITION_TO_FW', 'VTOL is in transition from multicopter to fixed-wing').
enum_entry_description('MAV_VTOL_STATE', 'MAV_VTOL_STATE_TRANSITION_TO_MC', 'VTOL is in transition from fixed-wing to multicopter').
enum_entry_description('MAV_VTOL_STATE', 'MAV_VTOL_STATE_MC', 'VTOL is in multicopter state').
enum_entry_description('MAV_VTOL_STATE', 'MAV_VTOL_STATE_FW', 'VTOL is in fixed-wing state').
enum_entry_description('MAV_LANDED_STATE', 'MAV_LANDED_STATE_UNDEFINED', 'MAV landed state is unknown').
enum_entry_description('MAV_LANDED_STATE', 'MAV_LANDED_STATE_ON_GROUND', 'MAV is landed (on ground)').
enum_entry_description('MAV_LANDED_STATE', 'MAV_LANDED_STATE_IN_AIR', 'MAV is in air').
enum_entry_description('MAV_LANDED_STATE', 'MAV_LANDED_STATE_TAKEOFF', 'MAV currently taking off').
enum_entry_description('MAV_LANDED_STATE', 'MAV_LANDED_STATE_LANDING', 'MAV currently landing').
enum_entry_description('ADSB_ALTITUDE_TYPE', 'ADSB_ALTITUDE_TYPE_PRESSURE_QNH', 'Altitude reported from a Baro source using QNH reference').
enum_entry_description('ADSB_ALTITUDE_TYPE', 'ADSB_ALTITUDE_TYPE_GEOMETRIC', 'Altitude reported from a GNSS source').
enum_entry_description('MAV_DO_REPOSITION_FLAGS', 'MAV_DO_REPOSITION_FLAGS_CHANGE_MODE', 'The aircraft should immediately transition into guided. This should not be set for follow me applications').
enum_entry_description('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_ATTITUDE', 'True if the attitude estimate is good').
enum_entry_description('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_VELOCITY_HORIZ', 'True if the horizontal velocity estimate is good').
enum_entry_description('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_VELOCITY_VERT', 'True if the  vertical velocity estimate is good').
enum_entry_description('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_POS_HORIZ_REL', 'True if the horizontal position (relative) estimate is good').
enum_entry_description('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_POS_HORIZ_ABS', 'True if the horizontal position (absolute) estimate is good').
enum_entry_description('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_POS_VERT_ABS', 'True if the vertical position (absolute) estimate is good').
enum_entry_description('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_POS_VERT_AGL', 'True if the vertical position (above ground) estimate is good').
enum_entry_description('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_CONST_POS_MODE', 'True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)').
enum_entry_description('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_PRED_POS_HORIZ_REL', 'True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate').
enum_entry_description('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_PRED_POS_HORIZ_ABS', 'True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate').
enum_entry_description('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_GPS_GLITCH', 'True if the EKF has detected a GPS glitch').
enum_entry_description('ESTIMATOR_STATUS_FLAGS', 'ESTIMATOR_ACCEL_ERROR', 'True if the EKF has detected bad accelerometer data').
enum_entry_description('MOTOR_TEST_ORDER', 'MOTOR_TEST_ORDER_DEFAULT', 'Default autopilot motor test method.').
enum_entry_description('MOTOR_TEST_ORDER', 'MOTOR_TEST_ORDER_SEQUENCE', 'Motor numbers are specified as their index in a predefined vehicle-specific sequence.').
enum_entry_description('MOTOR_TEST_ORDER', 'MOTOR_TEST_ORDER_BOARD', 'Motor numbers are specified as the output as labeled on the board.').
enum_entry_description('MOTOR_TEST_THROTTLE_TYPE', 'MOTOR_TEST_THROTTLE_PERCENT', 'Throttle as a percentage (0 ~ 100)').
enum_entry_description('MOTOR_TEST_THROTTLE_TYPE', 'MOTOR_TEST_THROTTLE_PWM', 'Throttle as an absolute PWM value (normally in range of 1000~2000).').
enum_entry_description('MOTOR_TEST_THROTTLE_TYPE', 'MOTOR_TEST_THROTTLE_PILOT', 'Throttle pass-through from pilot\'s transmitter.').
enum_entry_description('MOTOR_TEST_THROTTLE_TYPE', 'MOTOR_TEST_COMPASS_CAL', 'Per-motor compass calibration test.').
enum_entry_description('GPS_INPUT_IGNORE_FLAGS', 'GPS_INPUT_IGNORE_FLAG_ALT', 'ignore altitude field').
enum_entry_description('GPS_INPUT_IGNORE_FLAGS', 'GPS_INPUT_IGNORE_FLAG_HDOP', 'ignore hdop field').
enum_entry_description('GPS_INPUT_IGNORE_FLAGS', 'GPS_INPUT_IGNORE_FLAG_VDOP', 'ignore vdop field').
enum_entry_description('GPS_INPUT_IGNORE_FLAGS', 'GPS_INPUT_IGNORE_FLAG_VEL_HORIZ', 'ignore horizontal velocity field (vn and ve)').
enum_entry_description('GPS_INPUT_IGNORE_FLAGS', 'GPS_INPUT_IGNORE_FLAG_VEL_VERT', 'ignore vertical velocity field (vd)').
enum_entry_description('GPS_INPUT_IGNORE_FLAGS', 'GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY', 'ignore speed accuracy field').
enum_entry_description('GPS_INPUT_IGNORE_FLAGS', 'GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY', 'ignore horizontal accuracy field').
enum_entry_description('GPS_INPUT_IGNORE_FLAGS', 'GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY', 'ignore vertical accuracy field').
enum_entry_description('MAV_COLLISION_ACTION', 'MAV_COLLISION_ACTION_NONE', 'Ignore any potential collisions').
enum_entry_description('MAV_COLLISION_ACTION', 'MAV_COLLISION_ACTION_REPORT', 'Report potential collision').
enum_entry_description('MAV_COLLISION_ACTION', 'MAV_COLLISION_ACTION_ASCEND_OR_DESCEND', 'Ascend or Descend to avoid threat').
enum_entry_description('MAV_COLLISION_ACTION', 'MAV_COLLISION_ACTION_MOVE_HORIZONTALLY', 'Move horizontally to avoid threat').
enum_entry_description('MAV_COLLISION_ACTION', 'MAV_COLLISION_ACTION_MOVE_PERPENDICULAR', 'Aircraft to move perpendicular to the collision\'s velocity vector').
enum_entry_description('MAV_COLLISION_ACTION', 'MAV_COLLISION_ACTION_RTL', 'Aircraft to fly directly back to its launch point').
enum_entry_description('MAV_COLLISION_ACTION', 'MAV_COLLISION_ACTION_HOVER', 'Aircraft to stop in place').
enum_entry_description('MAV_COLLISION_THREAT_LEVEL', 'MAV_COLLISION_THREAT_LEVEL_NONE', 'Not a threat').
enum_entry_description('MAV_COLLISION_THREAT_LEVEL', 'MAV_COLLISION_THREAT_LEVEL_LOW', 'Craft is mildly concerned about this threat').
enum_entry_description('MAV_COLLISION_THREAT_LEVEL', 'MAV_COLLISION_THREAT_LEVEL_HIGH', 'Craft is panicking, and may take actions to avoid threat').
enum_entry_description('MAV_COLLISION_SRC', 'MAV_COLLISION_SRC_ADSB', 'ID field references ADSB_VEHICLE packets').
enum_entry_description('MAV_COLLISION_SRC', 'MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT', 'ID field references MAVLink SRC ID').
enum_entry_description('GPS_FIX_TYPE', 'GPS_FIX_TYPE_NO_GPS', 'No GPS connected').
enum_entry_description('GPS_FIX_TYPE', 'GPS_FIX_TYPE_NO_FIX', 'No position information, GPS is connected').
enum_entry_description('GPS_FIX_TYPE', 'GPS_FIX_TYPE_2D_FIX', '2D position').
enum_entry_description('GPS_FIX_TYPE', 'GPS_FIX_TYPE_3D_FIX', '3D position').
enum_entry_description('GPS_FIX_TYPE', 'GPS_FIX_TYPE_DGPS', 'DGPS/SBAS aided 3D position').
enum_entry_description('GPS_FIX_TYPE', 'GPS_FIX_TYPE_RTK_FLOAT', 'RTK float, 3D position').
enum_entry_description('GPS_FIX_TYPE', 'GPS_FIX_TYPE_RTK_FIXED', 'RTK Fixed, 3D position').
enum_entry_description('GPS_FIX_TYPE', 'GPS_FIX_TYPE_STATIC', 'Static fixed, typically used for base stations').
enum_entry_description('GPS_FIX_TYPE', 'GPS_FIX_TYPE_PPP', 'PPP, 3D position.').
enum_entry_description('RTK_BASELINE_COORDINATE_SYSTEM', 'RTK_BASELINE_COORDINATE_SYSTEM_ECEF', 'Earth-centered, Earth-fixed').
enum_entry_description('RTK_BASELINE_COORDINATE_SYSTEM', 'RTK_BASELINE_COORDINATE_SYSTEM_NED', 'RTK basestation centered, north, east, down').
enum_entry_description('LANDING_TARGET_TYPE', 'LANDING_TARGET_TYPE_LIGHT_BEACON', 'Landing target signaled by light beacon (ex: IR-LOCK)').
enum_entry_description('LANDING_TARGET_TYPE', 'LANDING_TARGET_TYPE_RADIO_BEACON', 'Landing target signaled by radio beacon (ex: ILS, NDB)').
enum_entry_description('LANDING_TARGET_TYPE', 'LANDING_TARGET_TYPE_VISION_FIDUCIAL', 'Landing target represented by a fiducial marker (ex: ARTag)').
enum_entry_description('LANDING_TARGET_TYPE', 'LANDING_TARGET_TYPE_VISION_OTHER', 'Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)').
enum_entry_description('VTOL_TRANSITION_HEADING', 'VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT', 'Respect the heading configuration of the vehicle.').
enum_entry_description('VTOL_TRANSITION_HEADING', 'VTOL_TRANSITION_HEADING_NEXT_WAYPOINT', 'Use the heading pointing towards the next waypoint.').
enum_entry_description('VTOL_TRANSITION_HEADING', 'VTOL_TRANSITION_HEADING_TAKEOFF', 'Use the heading on takeoff (while sitting on the ground).').
enum_entry_description('VTOL_TRANSITION_HEADING', 'VTOL_TRANSITION_HEADING_SPECIFIED', 'Use the specified heading in parameter 4.').
enum_entry_description('VTOL_TRANSITION_HEADING', 'VTOL_TRANSITION_HEADING_ANY', 'Use the current heading when reaching takeoff altitude (potentially facing the wind when weather-vaning is active).').
enum_entry_description('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_CAPTURE_VIDEO', 'Camera is able to record video').
enum_entry_description('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_CAPTURE_IMAGE', 'Camera is able to capture images').
enum_entry_description('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_HAS_MODES', 'Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)').
enum_entry_description('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE', 'Camera can capture images while in video mode').
enum_entry_description('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE', 'Camera can capture videos while in Photo/Image mode').
enum_entry_description('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE', 'Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)').
enum_entry_description('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM', 'Camera has basic zoom control (MAV_CMD_SET_CAMERA_ZOOM)').
enum_entry_description('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS', 'Camera has basic focus control (MAV_CMD_SET_CAMERA_FOCUS)').
enum_entry_description('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM', 'Camera has video streaming capabilities (request VIDEO_STREAM_INFORMATION with MAV_CMD_REQUEST_MESSAGE for video streaming info)').
enum_entry_description('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_HAS_TRACKING_POINT', 'Camera supports tracking of a point on the camera view.').
enum_entry_description('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE', 'Camera supports tracking of a selection rectangle on the camera view.').
enum_entry_description('CAMERA_CAP_FLAGS', 'CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS', 'Camera supports tracking geo status (CAMERA_TRACKING_GEO_STATUS).').
enum_entry_description('VIDEO_STREAM_STATUS_FLAGS', 'VIDEO_STREAM_STATUS_FLAGS_RUNNING', 'Stream is active (running)').
enum_entry_description('VIDEO_STREAM_STATUS_FLAGS', 'VIDEO_STREAM_STATUS_FLAGS_THERMAL', 'Stream is thermal imaging').
enum_entry_description('VIDEO_STREAM_TYPE', 'VIDEO_STREAM_TYPE_RTSP', 'Stream is RTSP').
enum_entry_description('VIDEO_STREAM_TYPE', 'VIDEO_STREAM_TYPE_RTPUDP', 'Stream is RTP UDP (URI gives the port number)').
enum_entry_description('VIDEO_STREAM_TYPE', 'VIDEO_STREAM_TYPE_TCP_MPEG', 'Stream is MPEG on TCP').
enum_entry_description('VIDEO_STREAM_TYPE', 'VIDEO_STREAM_TYPE_MPEG_TS_H264', 'Stream is h.264 on MPEG TS (URI gives the port number)').
enum_entry_description('CAMERA_TRACKING_STATUS_FLAGS', 'CAMERA_TRACKING_STATUS_FLAGS_IDLE', 'Camera is not tracking').
enum_entry_description('CAMERA_TRACKING_STATUS_FLAGS', 'CAMERA_TRACKING_STATUS_FLAGS_ACTIVE', 'Camera is tracking').
enum_entry_description('CAMERA_TRACKING_STATUS_FLAGS', 'CAMERA_TRACKING_STATUS_FLAGS_ERROR', 'Camera tracking in error state').
enum_entry_description('CAMERA_TRACKING_MODE', 'CAMERA_TRACKING_MODE_NONE', 'Not tracking').
enum_entry_description('CAMERA_TRACKING_MODE', 'CAMERA_TRACKING_MODE_POINT', 'Target is a point').
enum_entry_description('CAMERA_TRACKING_MODE', 'CAMERA_TRACKING_MODE_RECTANGLE', 'Target is a rectangle').
enum_entry_description('CAMERA_TRACKING_TARGET_DATA', 'CAMERA_TRACKING_TARGET_DATA_NONE', 'No target data').
enum_entry_description('CAMERA_TRACKING_TARGET_DATA', 'CAMERA_TRACKING_TARGET_DATA_EMBEDDED', 'Target data embedded in image data (proprietary)').
enum_entry_description('CAMERA_TRACKING_TARGET_DATA', 'CAMERA_TRACKING_TARGET_DATA_RENDERED', 'Target data rendered in image').
enum_entry_description('CAMERA_TRACKING_TARGET_DATA', 'CAMERA_TRACKING_TARGET_DATA_IN_STATUS', 'Target data within status message (Point or Rectangle)').
enum_entry_description('CAMERA_ZOOM_TYPE', 'ZOOM_TYPE_STEP', 'Zoom one step increment (-1 for wide, 1 for tele)').
enum_entry_description('CAMERA_ZOOM_TYPE', 'ZOOM_TYPE_CONTINUOUS', 'Continuous zoom up/down until stopped (-1 for wide, 1 for tele, 0 to stop zooming)').
enum_entry_description('CAMERA_ZOOM_TYPE', 'ZOOM_TYPE_RANGE', 'Zoom value as proportion of full camera range (a percentage value between 0.0 and 100.0)').
enum_entry_description('CAMERA_ZOOM_TYPE', 'ZOOM_TYPE_FOCAL_LENGTH', 'Zoom value/variable focal length in millimetres. Note that there is no message to get the valid zoom range of the camera, so this can type can only be used for cameras where the zoom range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera)').
enum_entry_description('CAMERA_ZOOM_TYPE', 'ZOOM_TYPE_HORIZONTAL_FOV', 'Zoom value as horizontal field of view in degrees.').
enum_entry_description('SET_FOCUS_TYPE', 'FOCUS_TYPE_STEP', 'Focus one step increment (-1 for focusing in, 1 for focusing out towards infinity).').
enum_entry_description('SET_FOCUS_TYPE', 'FOCUS_TYPE_CONTINUOUS', 'Continuous focus up/down until stopped (-1 for focusing in, 1 for focusing out towards infinity, 0 to stop focusing)').
enum_entry_description('SET_FOCUS_TYPE', 'FOCUS_TYPE_RANGE', 'Focus value as proportion of full camera focus range (a value between 0.0 and 100.0)').
enum_entry_description('SET_FOCUS_TYPE', 'FOCUS_TYPE_METERS', 'Focus value in metres. Note that there is no message to get the valid focus range of the camera, so this can type can only be used for cameras where the range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera).').
enum_entry_description('SET_FOCUS_TYPE', 'FOCUS_TYPE_AUTO', 'Focus automatically.').
enum_entry_description('SET_FOCUS_TYPE', 'FOCUS_TYPE_AUTO_SINGLE', 'Single auto focus. Mainly used for still pictures. Usually abbreviated as AF-S.').
enum_entry_description('SET_FOCUS_TYPE', 'FOCUS_TYPE_AUTO_CONTINUOUS', 'Continuous auto focus. Mainly used for dynamic scenes. Abbreviated as AF-C.').
enum_entry_description('PARAM_ACK', 'PARAM_ACK_ACCEPTED', 'Parameter value ACCEPTED and SET').
enum_entry_description('PARAM_ACK', 'PARAM_ACK_VALUE_UNSUPPORTED', 'Parameter value UNKNOWN/UNSUPPORTED').
enum_entry_description('PARAM_ACK', 'PARAM_ACK_FAILED', 'Parameter failed to set').
enum_entry_description('PARAM_ACK', 'PARAM_ACK_IN_PROGRESS', 'Parameter value received but not yet set/accepted. A subsequent PARAM_ACK_TRANSACTION or PARAM_EXT_ACK with the final result will follow once operation is completed. This is returned immediately for parameters that take longer to set, indicating that the the parameter was received and does not need to be resent.').
enum_entry_description('CAMERA_MODE', 'CAMERA_MODE_IMAGE', 'Camera is in image/photo capture mode.').
enum_entry_description('CAMERA_MODE', 'CAMERA_MODE_VIDEO', 'Camera is in video capture mode.').
enum_entry_description('CAMERA_MODE', 'CAMERA_MODE_IMAGE_SURVEY', 'Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys.').
enum_entry_description('MAV_ARM_AUTH_DENIED_REASON', 'MAV_ARM_AUTH_DENIED_REASON_GENERIC', 'Not a specific reason').
enum_entry_description('MAV_ARM_AUTH_DENIED_REASON', 'MAV_ARM_AUTH_DENIED_REASON_NONE', 'Authorizer will send the error as string to GCS').
enum_entry_description('MAV_ARM_AUTH_DENIED_REASON', 'MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT', 'At least one waypoint have a invalid value').
enum_entry_description('MAV_ARM_AUTH_DENIED_REASON', 'MAV_ARM_AUTH_DENIED_REASON_TIMEOUT', 'Timeout in the authorizer process(in case it depends on network)').
enum_entry_description('MAV_ARM_AUTH_DENIED_REASON', 'MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE', 'Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id that caused it to be denied.').
enum_entry_description('MAV_ARM_AUTH_DENIED_REASON', 'MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER', 'Weather is not good to fly').
enum_entry_description('RC_TYPE', 'RC_TYPE_SPEKTRUM_DSM2', 'Spektrum DSM2').
enum_entry_description('RC_TYPE', 'RC_TYPE_SPEKTRUM_DSMX', 'Spektrum DSMX').
enum_entry_description('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_X_IGNORE', 'Ignore position x').
enum_entry_description('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_Y_IGNORE', 'Ignore position y').
enum_entry_description('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_Z_IGNORE', 'Ignore position z').
enum_entry_description('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_VX_IGNORE', 'Ignore velocity x').
enum_entry_description('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_VY_IGNORE', 'Ignore velocity y').
enum_entry_description('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_VZ_IGNORE', 'Ignore velocity z').
enum_entry_description('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_AX_IGNORE', 'Ignore acceleration x').
enum_entry_description('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_AY_IGNORE', 'Ignore acceleration y').
enum_entry_description('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_AZ_IGNORE', 'Ignore acceleration z').
enum_entry_description('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_FORCE_SET', 'Use force instead of acceleration').
enum_entry_description('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_YAW_IGNORE', 'Ignore yaw').
enum_entry_description('POSITION_TARGET_TYPEMASK', 'POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE', 'Ignore yaw rate').
enum_entry_description('ATTITUDE_TARGET_TYPEMASK', 'ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE', 'Ignore body roll rate').
enum_entry_description('ATTITUDE_TARGET_TYPEMASK', 'ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE', 'Ignore body pitch rate').
enum_entry_description('ATTITUDE_TARGET_TYPEMASK', 'ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE', 'Ignore body yaw rate').
enum_entry_description('ATTITUDE_TARGET_TYPEMASK', 'ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET', 'Use 3D body thrust setpoint instead of throttle').
enum_entry_description('ATTITUDE_TARGET_TYPEMASK', 'ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE', 'Ignore throttle').
enum_entry_description('ATTITUDE_TARGET_TYPEMASK', 'ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE', 'Ignore attitude').
enum_entry_description('UTM_FLIGHT_STATE', 'UTM_FLIGHT_STATE_UNKNOWN', 'The flight state can\'t be determined.').
enum_entry_description('UTM_FLIGHT_STATE', 'UTM_FLIGHT_STATE_GROUND', 'UAS on ground.').
enum_entry_description('UTM_FLIGHT_STATE', 'UTM_FLIGHT_STATE_AIRBORNE', 'UAS airborne.').
enum_entry_description('UTM_FLIGHT_STATE', 'UTM_FLIGHT_STATE_EMERGENCY', 'UAS is in an emergency flight state.').
enum_entry_description('UTM_FLIGHT_STATE', 'UTM_FLIGHT_STATE_NOCTRL', 'UAS has no active controls.').
enum_entry_description('UTM_DATA_AVAIL_FLAGS', 'UTM_DATA_AVAIL_FLAGS_TIME_VALID', 'The field time contains valid data.').
enum_entry_description('UTM_DATA_AVAIL_FLAGS', 'UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE', 'The field uas_id contains valid data.').
enum_entry_description('UTM_DATA_AVAIL_FLAGS', 'UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE', 'The fields lat, lon and h_acc contain valid data.').
enum_entry_description('UTM_DATA_AVAIL_FLAGS', 'UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE', 'The fields alt and v_acc contain valid data.').
enum_entry_description('UTM_DATA_AVAIL_FLAGS', 'UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE', 'The field relative_alt contains valid data.').
enum_entry_description('UTM_DATA_AVAIL_FLAGS', 'UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE', 'The fields vx and vy contain valid data.').
enum_entry_description('UTM_DATA_AVAIL_FLAGS', 'UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE', 'The field vz contains valid data.').
enum_entry_description('UTM_DATA_AVAIL_FLAGS', 'UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE', 'The fields next_lat, next_lon and next_alt contain valid data.').
enum_entry_description('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_UNKNOWN', 'State unknown or not reportable.').
enum_entry_description('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_FAILED', 'Modem is unusable').
enum_entry_description('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_INITIALIZING', 'Modem is being initialized').
enum_entry_description('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_LOCKED', 'Modem is locked').
enum_entry_description('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_DISABLED', 'Modem is not enabled and is powered down').
enum_entry_description('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_DISABLING', 'Modem is currently transitioning to the CELLULAR_STATUS_FLAG_DISABLED state').
enum_entry_description('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_ENABLING', 'Modem is currently transitioning to the CELLULAR_STATUS_FLAG_ENABLED state').
enum_entry_description('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_ENABLED', 'Modem is enabled and powered on but not registered with a network provider and not available for data connections').
enum_entry_description('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_SEARCHING', 'Modem is searching for a network provider to register').
enum_entry_description('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_REGISTERED', 'Modem is registered with a network provider, and data connections and messaging may be available for use').
enum_entry_description('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_DISCONNECTING', 'Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered if more than one packet data bearer is active and one of the active bearers is deactivated').
enum_entry_description('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_CONNECTING', 'Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another bearer is already active do not cause this state to be entered').
enum_entry_description('CELLULAR_STATUS_FLAG', 'CELLULAR_STATUS_FLAG_CONNECTED', 'One or more packet data bearers is active and connected').
enum_entry_description('CELLULAR_NETWORK_FAILED_REASON', 'CELLULAR_NETWORK_FAILED_REASON_NONE', 'No error').
enum_entry_description('CELLULAR_NETWORK_FAILED_REASON', 'CELLULAR_NETWORK_FAILED_REASON_UNKNOWN', 'Error state is unknown').
enum_entry_description('CELLULAR_NETWORK_FAILED_REASON', 'CELLULAR_NETWORK_FAILED_REASON_SIM_MISSING', 'SIM is required for the modem but missing').
enum_entry_description('CELLULAR_NETWORK_FAILED_REASON', 'CELLULAR_NETWORK_FAILED_REASON_SIM_ERROR', 'SIM is available, but not usable for connection').
enum_entry_description('PRECISION_LAND_MODE', 'PRECISION_LAND_MODE_DISABLED', 'Normal (non-precision) landing.').
enum_entry_description('PRECISION_LAND_MODE', 'PRECISION_LAND_MODE_OPPORTUNISTIC', 'Use precision landing if beacon detected when land command accepted, otherwise land normally.').
enum_entry_description('PRECISION_LAND_MODE', 'PRECISION_LAND_MODE_REQUIRED', 'Use precision landing, searching for beacon if not found when land command accepted (land normally if beacon cannot be found).').
enum_entry_description('PARACHUTE_ACTION', 'PARACHUTE_DISABLE', 'Disable auto-release of parachute (i.e. release triggered by crash detectors).').
enum_entry_description('PARACHUTE_ACTION', 'PARACHUTE_ENABLE', 'Enable auto-release of parachute.').
enum_entry_description('PARACHUTE_ACTION', 'PARACHUTE_RELEASE', 'Release parachute and kill motors.').
enum_entry_description('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN', 'Encoding of payload unknown.').
enum_entry_description('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0', 'Registered for STorM32 gimbal controller.').
enum_entry_description('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1', 'Registered for STorM32 gimbal controller.').
enum_entry_description('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2', 'Registered for STorM32 gimbal controller.').
enum_entry_description('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3', 'Registered for STorM32 gimbal controller.').
enum_entry_description('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4', 'Registered for STorM32 gimbal controller.').
enum_entry_description('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5', 'Registered for STorM32 gimbal controller.').
enum_entry_description('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6', 'Registered for STorM32 gimbal controller.').
enum_entry_description('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7', 'Registered for STorM32 gimbal controller.').
enum_entry_description('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8', 'Registered for STorM32 gimbal controller.').
enum_entry_description('MAV_TUNNEL_PAYLOAD_TYPE', 'MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9', 'Registered for STorM32 gimbal controller.').
enum_entry_description('MAV_ODID_ID_TYPE', 'MAV_ODID_ID_TYPE_NONE', 'No type defined.').
enum_entry_description('MAV_ODID_ID_TYPE', 'MAV_ODID_ID_TYPE_SERIAL_NUMBER', 'Manufacturer Serial Number (ANSI/CTA-2063 format).').
enum_entry_description('MAV_ODID_ID_TYPE', 'MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID', 'CAA (Civil Aviation Authority) registered ID. Format: [ICAO Country Code].[CAA Assigned ID].').
enum_entry_description('MAV_ODID_ID_TYPE', 'MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID', 'UTM (Unmanned Traffic Management) assigned UUID (RFC4122).').
enum_entry_description('MAV_ODID_ID_TYPE', 'MAV_ODID_ID_TYPE_SPECIFIC_SESSION_ID', 'A 20 byte ID for a specific flight/session. The exact ID type is indicated by the first byte of uas_id and these type values are managed by ICAO.').
enum_entry_description('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_NONE', 'No UA (Unmanned Aircraft) type defined.').
enum_entry_description('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_AEROPLANE', 'Aeroplane/Airplane. Fixed wing.').
enum_entry_description('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR', 'Helicopter or multirotor.').
enum_entry_description('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_GYROPLANE', 'Gyroplane.').
enum_entry_description('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_HYBRID_LIFT', 'VTOL (Vertical Take-Off and Landing). Fixed wing aircraft that can take off vertically.').
enum_entry_description('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_ORNITHOPTER', 'Ornithopter.').
enum_entry_description('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_GLIDER', 'Glider.').
enum_entry_description('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_KITE', 'Kite.').
enum_entry_description('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_FREE_BALLOON', 'Free Balloon.').
enum_entry_description('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_CAPTIVE_BALLOON', 'Captive Balloon.').
enum_entry_description('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_AIRSHIP', 'Airship. E.g. a blimp.').
enum_entry_description('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE', 'Free Fall/Parachute (unpowered).').
enum_entry_description('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_ROCKET', 'Rocket.').
enum_entry_description('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT', 'Tethered powered aircraft.').
enum_entry_description('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_GROUND_OBSTACLE', 'Ground Obstacle.').
enum_entry_description('MAV_ODID_UA_TYPE', 'MAV_ODID_UA_TYPE_OTHER', 'Other type of aircraft not listed earlier.').
enum_entry_description('MAV_ODID_STATUS', 'MAV_ODID_STATUS_UNDECLARED', 'The status of the (UA) Unmanned Aircraft is undefined.').
enum_entry_description('MAV_ODID_STATUS', 'MAV_ODID_STATUS_GROUND', 'The UA is on the ground.').
enum_entry_description('MAV_ODID_STATUS', 'MAV_ODID_STATUS_AIRBORNE', 'The UA is in the air.').
enum_entry_description('MAV_ODID_STATUS', 'MAV_ODID_STATUS_EMERGENCY', 'The UA is having an emergency.').
enum_entry_description('MAV_ODID_STATUS', 'MAV_ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE', 'The remote ID system is failing or unreliable in some way.').
enum_entry_description('MAV_ODID_HEIGHT_REF', 'MAV_ODID_HEIGHT_REF_OVER_TAKEOFF', 'The height field is relative to the take-off location.').
enum_entry_description('MAV_ODID_HEIGHT_REF', 'MAV_ODID_HEIGHT_REF_OVER_GROUND', 'The height field is relative to ground.').
enum_entry_description('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_UNKNOWN', 'The horizontal accuracy is unknown.').
enum_entry_description('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_10NM', 'The horizontal accuracy is smaller than 10 Nautical Miles. 18.52 km.').
enum_entry_description('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_4NM', 'The horizontal accuracy is smaller than 4 Nautical Miles. 7.408 km.').
enum_entry_description('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_2NM', 'The horizontal accuracy is smaller than 2 Nautical Miles. 3.704 km.').
enum_entry_description('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_1NM', 'The horizontal accuracy is smaller than 1 Nautical Miles. 1.852 km.').
enum_entry_description('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_0_5NM', 'The horizontal accuracy is smaller than 0.5 Nautical Miles. 926 m.').
enum_entry_description('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_0_3NM', 'The horizontal accuracy is smaller than 0.3 Nautical Miles. 555.6 m.').
enum_entry_description('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_0_1NM', 'The horizontal accuracy is smaller than 0.1 Nautical Miles. 185.2 m.').
enum_entry_description('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_0_05NM', 'The horizontal accuracy is smaller than 0.05 Nautical Miles. 92.6 m.').
enum_entry_description('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_30_METER', 'The horizontal accuracy is smaller than 30 meter.').
enum_entry_description('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_10_METER', 'The horizontal accuracy is smaller than 10 meter.').
enum_entry_description('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_3_METER', 'The horizontal accuracy is smaller than 3 meter.').
enum_entry_description('MAV_ODID_HOR_ACC', 'MAV_ODID_HOR_ACC_1_METER', 'The horizontal accuracy is smaller than 1 meter.').
enum_entry_description('MAV_ODID_VER_ACC', 'MAV_ODID_VER_ACC_UNKNOWN', 'The vertical accuracy is unknown.').
enum_entry_description('MAV_ODID_VER_ACC', 'MAV_ODID_VER_ACC_150_METER', 'The vertical accuracy is smaller than 150 meter.').
enum_entry_description('MAV_ODID_VER_ACC', 'MAV_ODID_VER_ACC_45_METER', 'The vertical accuracy is smaller than 45 meter.').
enum_entry_description('MAV_ODID_VER_ACC', 'MAV_ODID_VER_ACC_25_METER', 'The vertical accuracy is smaller than 25 meter.').
enum_entry_description('MAV_ODID_VER_ACC', 'MAV_ODID_VER_ACC_10_METER', 'The vertical accuracy is smaller than 10 meter.').
enum_entry_description('MAV_ODID_VER_ACC', 'MAV_ODID_VER_ACC_3_METER', 'The vertical accuracy is smaller than 3 meter.').
enum_entry_description('MAV_ODID_VER_ACC', 'MAV_ODID_VER_ACC_1_METER', 'The vertical accuracy is smaller than 1 meter.').
enum_entry_description('MAV_ODID_SPEED_ACC', 'MAV_ODID_SPEED_ACC_UNKNOWN', 'The speed accuracy is unknown.').
enum_entry_description('MAV_ODID_SPEED_ACC', 'MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND', 'The speed accuracy is smaller than 10 meters per second.').
enum_entry_description('MAV_ODID_SPEED_ACC', 'MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND', 'The speed accuracy is smaller than 3 meters per second.').
enum_entry_description('MAV_ODID_SPEED_ACC', 'MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND', 'The speed accuracy is smaller than 1 meters per second.').
enum_entry_description('MAV_ODID_SPEED_ACC', 'MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND', 'The speed accuracy is smaller than 0.3 meters per second.').
enum_entry_description('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_UNKNOWN', 'The timestamp accuracy is unknown.').
enum_entry_description('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_1_SECOND', 'The timestamp accuracy is smaller than or equal to 0.1 second.').
enum_entry_description('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_2_SECOND', 'The timestamp accuracy is smaller than or equal to 0.2 second.').
enum_entry_description('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_3_SECOND', 'The timestamp accuracy is smaller than or equal to 0.3 second.').
enum_entry_description('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_4_SECOND', 'The timestamp accuracy is smaller than or equal to 0.4 second.').
enum_entry_description('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_5_SECOND', 'The timestamp accuracy is smaller than or equal to 0.5 second.').
enum_entry_description('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_6_SECOND', 'The timestamp accuracy is smaller than or equal to 0.6 second.').
enum_entry_description('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_7_SECOND', 'The timestamp accuracy is smaller than or equal to 0.7 second.').
enum_entry_description('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_8_SECOND', 'The timestamp accuracy is smaller than or equal to 0.8 second.').
enum_entry_description('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_0_9_SECOND', 'The timestamp accuracy is smaller than or equal to 0.9 second.').
enum_entry_description('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_1_0_SECOND', 'The timestamp accuracy is smaller than or equal to 1.0 second.').
enum_entry_description('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_1_1_SECOND', 'The timestamp accuracy is smaller than or equal to 1.1 second.').
enum_entry_description('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_1_2_SECOND', 'The timestamp accuracy is smaller than or equal to 1.2 second.').
enum_entry_description('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_1_3_SECOND', 'The timestamp accuracy is smaller than or equal to 1.3 second.').
enum_entry_description('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_1_4_SECOND', 'The timestamp accuracy is smaller than or equal to 1.4 second.').
enum_entry_description('MAV_ODID_TIME_ACC', 'MAV_ODID_TIME_ACC_1_5_SECOND', 'The timestamp accuracy is smaller than or equal to 1.5 second.').
enum_entry_description('MAV_ODID_AUTH_TYPE', 'MAV_ODID_AUTH_TYPE_NONE', 'No authentication type is specified.').
enum_entry_description('MAV_ODID_AUTH_TYPE', 'MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE', 'Signature for the UAS (Unmanned Aircraft System) ID.').
enum_entry_description('MAV_ODID_AUTH_TYPE', 'MAV_ODID_AUTH_TYPE_OPERATOR_ID_SIGNATURE', 'Signature for the Operator ID.').
enum_entry_description('MAV_ODID_AUTH_TYPE', 'MAV_ODID_AUTH_TYPE_MESSAGE_SET_SIGNATURE', 'Signature for the entire message set.').
enum_entry_description('MAV_ODID_AUTH_TYPE', 'MAV_ODID_AUTH_TYPE_NETWORK_REMOTE_ID', 'Authentication is provided by Network Remote ID.').
enum_entry_description('MAV_ODID_AUTH_TYPE', 'MAV_ODID_AUTH_TYPE_SPECIFIC_AUTHENTICATION', 'The exact authentication type is indicated by the first byte of authentication_data and these type values are managed by ICAO.').
enum_entry_description('MAV_ODID_DESC_TYPE', 'MAV_ODID_DESC_TYPE_TEXT', 'Optional free-form text description of the purpose of the flight.').
enum_entry_description('MAV_ODID_DESC_TYPE', 'MAV_ODID_DESC_TYPE_EMERGENCY', 'Optional additional clarification when status == MAV_ODID_STATUS_EMERGENCY.').
enum_entry_description('MAV_ODID_DESC_TYPE', 'MAV_ODID_DESC_TYPE_EXTENDED_STATUS', 'Optional additional clarification when status != MAV_ODID_STATUS_EMERGENCY.').
enum_entry_description('MAV_ODID_OPERATOR_LOCATION_TYPE', 'MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF', 'The location/altitude of the operator is the same as the take-off location.').
enum_entry_description('MAV_ODID_OPERATOR_LOCATION_TYPE', 'MAV_ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS', 'The location/altitude of the operator is dynamic. E.g. based on live GNSS data.').
enum_entry_description('MAV_ODID_OPERATOR_LOCATION_TYPE', 'MAV_ODID_OPERATOR_LOCATION_TYPE_FIXED', 'The location/altitude of the operator are fixed values.').
enum_entry_description('MAV_ODID_CLASSIFICATION_TYPE', 'MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED', 'The classification type for the UA is undeclared.').
enum_entry_description('MAV_ODID_CLASSIFICATION_TYPE', 'MAV_ODID_CLASSIFICATION_TYPE_EU', 'The classification type for the UA follows EU (European Union) specifications.').
enum_entry_description('MAV_ODID_CATEGORY_EU', 'MAV_ODID_CATEGORY_EU_UNDECLARED', 'The category for the UA, according to the EU specification, is undeclared.').
enum_entry_description('MAV_ODID_CATEGORY_EU', 'MAV_ODID_CATEGORY_EU_OPEN', 'The category for the UA, according to the EU specification, is the Open category.').
enum_entry_description('MAV_ODID_CATEGORY_EU', 'MAV_ODID_CATEGORY_EU_SPECIFIC', 'The category for the UA, according to the EU specification, is the Specific category.').
enum_entry_description('MAV_ODID_CATEGORY_EU', 'MAV_ODID_CATEGORY_EU_CERTIFIED', 'The category for the UA, according to the EU specification, is the Certified category.').
enum_entry_description('MAV_ODID_CLASS_EU', 'MAV_ODID_CLASS_EU_UNDECLARED', 'The class for the UA, according to the EU specification, is undeclared.').
enum_entry_description('MAV_ODID_CLASS_EU', 'MAV_ODID_CLASS_EU_CLASS_0', 'The class for the UA, according to the EU specification, is Class 0.').
enum_entry_description('MAV_ODID_CLASS_EU', 'MAV_ODID_CLASS_EU_CLASS_1', 'The class for the UA, according to the EU specification, is Class 1.').
enum_entry_description('MAV_ODID_CLASS_EU', 'MAV_ODID_CLASS_EU_CLASS_2', 'The class for the UA, according to the EU specification, is Class 2.').
enum_entry_description('MAV_ODID_CLASS_EU', 'MAV_ODID_CLASS_EU_CLASS_3', 'The class for the UA, according to the EU specification, is Class 3.').
enum_entry_description('MAV_ODID_CLASS_EU', 'MAV_ODID_CLASS_EU_CLASS_4', 'The class for the UA, according to the EU specification, is Class 4.').
enum_entry_description('MAV_ODID_CLASS_EU', 'MAV_ODID_CLASS_EU_CLASS_5', 'The class for the UA, according to the EU specification, is Class 5.').
enum_entry_description('MAV_ODID_CLASS_EU', 'MAV_ODID_CLASS_EU_CLASS_6', 'The class for the UA, according to the EU specification, is Class 6.').
enum_entry_description('MAV_ODID_OPERATOR_ID_TYPE', 'MAV_ODID_OPERATOR_ID_TYPE_CAA', 'CAA (Civil Aviation Authority) registered operator ID.').
enum_entry_description('MAV_ODID_ARM_STATUS', 'MAV_ODID_ARM_STATUS_GOOD_TO_ARM', 'Passing arming checks.').
enum_entry_description('MAV_ODID_ARM_STATUS', 'MAV_ODID_ARM_STATUS_PRE_ARM_FAIL_GENERIC', 'Generic arming failure, see error string for details.').
enum_entry_description('TUNE_FORMAT', 'TUNE_FORMAT_QBASIC1_1', 'Format is QBasic 1.1 Play: https://www.qbasic.net/en/reference/qb11/Statement/PLAY-006.htm.').
enum_entry_description('TUNE_FORMAT', 'TUNE_FORMAT_MML_MODERN', 'Format is Modern Music Markup Language (MML): https://en.wikipedia.org/wiki/Music_Macro_Language#Modern_MML.').
enum_entry_description('AIS_TYPE', 'AIS_TYPE_UNKNOWN', 'Not available (default).').
enum_entry_description('AIS_TYPE', 'AIS_TYPE_WIG', 'Wing In Ground effect.').
enum_entry_description('AIS_TYPE', 'AIS_TYPE_TOWING_LARGE', 'Towing: length exceeds 200m or breadth exceeds 25m.').
enum_entry_description('AIS_TYPE', 'AIS_TYPE_DREDGING', 'Dredging or other underwater ops.').
enum_entry_description('AIS_TYPE', 'AIS_TYPE_HSC', 'High Speed Craft.').
enum_entry_description('AIS_TYPE', 'AIS_TYPE_SAR', 'Search And Rescue vessel.').
enum_entry_description('AIS_TYPE', 'AIS_TYPE_ANTI_POLLUTION', 'Anti-pollution equipment.').
enum_entry_description('AIS_TYPE', 'AIS_TYPE_NONECOMBATANT', 'Noncombatant ship according to RR Resolution No. 18.').
enum_entry_description('AIS_NAV_STATUS', 'UNDER_WAY', 'Under way using engine.').
enum_entry_description('AIS_NAV_STATUS', 'AIS_NAV_AIS_SART', 'Search And Rescue Transponder.').
enum_entry_description('AIS_NAV_STATUS', 'AIS_NAV_UNKNOWN', 'Not available (default).').
enum_entry_description('AIS_FLAGS', 'AIS_FLAGS_POSITION_ACCURACY', '1 = Position accuracy less than 10m, 0 = position accuracy greater than 10m.').
enum_entry_description('AIS_FLAGS', 'AIS_FLAGS_HIGH_VELOCITY', '1 = Velocity over 52.5765m/s (102.2 knots)').
enum_entry_description('AIS_FLAGS', 'AIS_FLAGS_TURN_RATE_SIGN_ONLY', 'Only the sign of the returned turn rate value is valid, either greater than 5deg/30s or less than -5deg/30s').
enum_entry_description('AIS_FLAGS', 'AIS_FLAGS_LARGE_BOW_DIMENSION', 'Distance to bow is larger than 511m').
enum_entry_description('AIS_FLAGS', 'AIS_FLAGS_LARGE_STERN_DIMENSION', 'Distance to stern is larger than 511m').
enum_entry_description('AIS_FLAGS', 'AIS_FLAGS_LARGE_PORT_DIMENSION', 'Distance to port side is larger than 63m').
enum_entry_description('AIS_FLAGS', 'AIS_FLAGS_LARGE_STARBOARD_DIMENSION', 'Distance to starboard side is larger than 63m').
enum_entry_description('FAILURE_TYPE', 'FAILURE_TYPE_OK', 'No failure injected, used to reset a previous failure.').
enum_entry_description('FAILURE_TYPE', 'FAILURE_TYPE_OFF', 'Sets unit off, so completely non-responsive.').
enum_entry_description('FAILURE_TYPE', 'FAILURE_TYPE_STUCK', 'Unit is stuck e.g. keeps reporting the same value.').
enum_entry_description('FAILURE_TYPE', 'FAILURE_TYPE_GARBAGE', 'Unit is reporting complete garbage.').
enum_entry_description('FAILURE_TYPE', 'FAILURE_TYPE_WRONG', 'Unit is consistently wrong.').
enum_entry_description('FAILURE_TYPE', 'FAILURE_TYPE_SLOW', 'Unit is slow, so e.g. reporting at slower than expected rate.').
enum_entry_description('FAILURE_TYPE', 'FAILURE_TYPE_DELAYED', 'Data of unit is delayed in time.').
enum_entry_description('FAILURE_TYPE', 'FAILURE_TYPE_INTERMITTENT', 'Unit is sometimes working, sometimes not.').
enum_entry_description('NAV_VTOL_LAND_OPTIONS', 'NAV_VTOL_LAND_OPTIONS_DEFAULT', 'Default autopilot landing behaviour.').
enum_entry_description('NAV_VTOL_LAND_OPTIONS', 'NAV_VTOL_LAND_OPTIONS_FW_DESCENT', 'Descend in fixed wing mode, transitioning to multicopter mode for vertical landing when close to the ground.\n          The fixed wing descent pattern is at the discretion of the vehicle (e.g. transition altitude, loiter direction, radius, and speed, etc.).\n        ').
enum_entry_description('NAV_VTOL_LAND_OPTIONS', 'NAV_VTOL_LAND_OPTIONS_HOVER_DESCENT', 'Land in multicopter mode on reaching the landing coordinates (the whole landing is by "hover descent").').
enum_entry_description('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_HEALTHY', 'Winch is healthy').
enum_entry_description('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_FULLY_RETRACTED', 'Winch line is fully retracted').
enum_entry_description('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_MOVING', 'Winch motor is moving').
enum_entry_description('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_CLUTCH_ENGAGED', 'Winch clutch is engaged allowing motor to move freely.').
enum_entry_description('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_LOCKED', 'Winch is locked by locking mechanism.').
enum_entry_description('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_DROPPING', 'Winch is gravity dropping payload.').
enum_entry_description('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_ARRESTING', 'Winch is arresting payload descent.').
enum_entry_description('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_GROUND_SENSE', 'Winch is using torque measurements to sense the ground.').
enum_entry_description('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_RETRACTING', 'Winch is returning to the fully retracted position.').
enum_entry_description('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_REDELIVER', 'Winch is redelivering the payload. This is a failover state if the line tension goes above a threshold during RETRACTING.').
enum_entry_description('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_ABANDON_LINE', 'Winch is abandoning the line and possibly payload. Winch unspools the entire calculated line length. This is a failover state from REDELIVER if the number of attempts exceeds a threshold.').
enum_entry_description('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_LOCKING', 'Winch is engaging the locking mechanism.').
enum_entry_description('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_LOAD_LINE', 'Winch is spooling on line.').
enum_entry_description('MAV_WINCH_STATUS_FLAG', 'MAV_WINCH_STATUS_LOAD_PAYLOAD', 'Winch is loading a payload.').
enum_entry_description('MAV_EVENT_ERROR_REASON', 'MAV_EVENT_ERROR_REASON_UNAVAILABLE', 'The requested event is not available (anymore).').
enum_entry_description('MAV_EVENT_CURRENT_SEQUENCE_FLAGS', 'MAV_EVENT_CURRENT_SEQUENCE_FLAGS_RESET', 'A sequence reset has happened (e.g. vehicle reboot).').
enum_entry_description('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_NONE', 'None of the fields in HIL_SENSOR have been updated').
enum_entry_description('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_XACC', 'The value in the xacc field has been updated').
enum_entry_description('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_YACC', 'The value in the yacc field has been updated').
enum_entry_description('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_ZACC', 'The value in the zacc field has been updated').
enum_entry_description('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_XGYRO', 'The value in the xgyro field has been updated').
enum_entry_description('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_YGYRO', 'The value in the ygyro field has been updated').
enum_entry_description('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_ZGYRO', 'The value in the zgyro field has been updated').
enum_entry_description('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_XMAG', 'The value in the xmag field has been updated').
enum_entry_description('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_YMAG', 'The value in the ymag field has been updated').
enum_entry_description('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_ZMAG', 'The value in the zmag field has been updated').
enum_entry_description('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_ABS_PRESSURE', 'The value in the abs_pressure field has been updated').
enum_entry_description('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_DIFF_PRESSURE', 'The value in the diff_pressure field has been updated').
enum_entry_description('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_PRESSURE_ALT', 'The value in the pressure_alt field has been updated').
enum_entry_description('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_TEMPERATURE', 'The value in the temperature field has been updated').
enum_entry_description('HIL_SENSOR_UPDATED_FLAGS', 'HIL_SENSOR_UPDATED_RESET', 'Full reset of attitude/position/velocities/etc was performed in sim (Bit 31).').
enum_entry_description('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_NONE', 'None of the fields in HIGHRES_IMU have been updated').
enum_entry_description('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_XACC', 'The value in the xacc field has been updated').
enum_entry_description('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_YACC', 'The value in the yacc field has been updated').
enum_entry_description('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_ZACC', 'The value in the zacc field has been updated since').
enum_entry_description('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_XGYRO', 'The value in the xgyro field has been updated').
enum_entry_description('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_YGYRO', 'The value in the ygyro field has been updated').
enum_entry_description('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_ZGYRO', 'The value in the zgyro field has been updated').
enum_entry_description('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_XMAG', 'The value in the xmag field has been updated').
enum_entry_description('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_YMAG', 'The value in the ymag field has been updated').
enum_entry_description('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_ZMAG', 'The value in the zmag field has been updated').
enum_entry_description('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_ABS_PRESSURE', 'The value in the abs_pressure field has been updated').
enum_entry_description('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_DIFF_PRESSURE', 'The value in the diff_pressure field has been updated').
enum_entry_description('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_PRESSURE_ALT', 'The value in the pressure_alt field has been updated').
enum_entry_description('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_TEMPERATURE', 'The value in the temperature field has been updated').
enum_entry_description('HIGHRES_IMU_UPDATED_FLAGS', 'HIGHRES_IMU_UPDATED_ALL', 'All fields in HIGHRES_IMU have been updated.').
enum_entry_description('MAV_FTP_ERR', 'MAV_FTP_ERR_NONE', 'None: No error').
enum_entry_description('MAV_FTP_ERR', 'MAV_FTP_ERR_FAIL', 'Fail: Unknown failure').
enum_entry_description('MAV_FTP_ERR', 'MAV_FTP_ERR_FAILERRNO', 'FailErrno: Command failed, Err number sent back in PayloadHeader.data[1].\n\t\tThis is a file-system error number understood by the server operating system.').
enum_entry_description('MAV_FTP_ERR', 'MAV_FTP_ERR_INVALIDDATASIZE', 'InvalidDataSize: Payload size is invalid').
enum_entry_description('MAV_FTP_ERR', 'MAV_FTP_ERR_INVALIDSESSION', 'InvalidSession: Session is not currently open').
enum_entry_description('MAV_FTP_ERR', 'MAV_FTP_ERR_NOSESSIONSAVAILABLE', 'NoSessionsAvailable: All available sessions are already in use').
enum_entry_description('MAV_FTP_ERR', 'MAV_FTP_ERR_EOF', 'EOF: Offset past end of file for ListDirectory and ReadFile commands').
enum_entry_description('MAV_FTP_ERR', 'MAV_FTP_ERR_UNKNOWNCOMMAND', 'UnknownCommand: Unknown command / opcode').
enum_entry_description('MAV_FTP_ERR', 'MAV_FTP_ERR_FILEEXISTS', 'FileExists: File/directory already exists').
enum_entry_description('MAV_FTP_ERR', 'MAV_FTP_ERR_FILEPROTECTED', 'FileProtected: File/directory is write protected').
enum_entry_description('MAV_FTP_ERR', 'MAV_FTP_ERR_FILENOTFOUND', 'FileNotFound: File/directory not found').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_NONE', 'None. Ignored, always ACKed').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_TERMINATESESSION', 'TerminateSession: Terminates open Read session').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_RESETSESSION', 'ResetSessions: Terminates all open read sessions').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_LISTDIRECTORY', 'ListDirectory. List files and directories in path from offset').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_OPENFILERO', 'OpenFileRO: Opens file at path for reading, returns session').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_READFILE', 'ReadFile: Reads size bytes from offset in session').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_CREATEFILE', 'CreateFile: Creates file at path for writing, returns session').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_WRITEFILE', 'WriteFile: Writes size bytes to offset in session').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_REMOVEFILE', 'RemoveFile: Remove file at path').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_CREATEDIRECTORY', 'CreateDirectory: Creates directory at path').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_REMOVEDIRECTORY', 'RemoveDirectory: Removes directory at path. The directory must be empty.').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_OPENFILEWO', 'OpenFileWO: Opens file at path for writing, returns session').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_TRUNCATEFILE', 'TruncateFile: Truncate file at path to offset length').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_RENAME', 'Rename: Rename path1 to path2').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_CALCFILECRC', 'CalcFileCRC32: Calculate CRC32 for file at path').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_BURSTREADFILE', 'BurstReadFile: Burst download session file').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_ACK', 'ACK: ACK response').
enum_entry_description('MAV_FTP_OPCODE', 'MAV_FTP_OPCODE_NAK', 'NAK: NAK response').
enum_entry_description('MISSION_STATE', 'MISSION_STATE_UNKNOWN', 'The mission status reporting is not supported.').
enum_entry_description('MISSION_STATE', 'MISSION_STATE_NO_MISSION', 'No mission on the vehicle.').
enum_entry_description('MISSION_STATE', 'MISSION_STATE_NOT_STARTED', 'Mission has not started. This is the case after a mission has uploaded but not yet started executing.').
enum_entry_description('MISSION_STATE', 'MISSION_STATE_ACTIVE', 'Mission is active, and will execute mission items when in auto mode.').
enum_entry_description('MISSION_STATE', 'MISSION_STATE_PAUSED', 'Mission is paused when in auto mode.').
enum_entry_description('MISSION_STATE', 'MISSION_STATE_COMPLETE', 'Mission has executed all mission items.').
enum_entry_description('WIFI_NETWORK_SECURITY', 'WIFI_NETWORK_SECURITY_UNDEFINED', 'Undefined or unknown security protocol.').
enum_entry_description('WIFI_NETWORK_SECURITY', 'WIFI_NETWORK_SECURITY_OPEN', 'Open network, no security.').
enum_entry_description('WIFI_NETWORK_SECURITY', 'WIFI_NETWORK_SECURITY_WEP', 'WEP.').
enum_entry_description('WIFI_NETWORK_SECURITY', 'WIFI_NETWORK_SECURITY_WPA1', 'WPA1.').
enum_entry_description('WIFI_NETWORK_SECURITY', 'WIFI_NETWORK_SECURITY_WPA2', 'WPA2.').
enum_entry_description('WIFI_NETWORK_SECURITY', 'WIFI_NETWORK_SECURITY_WPA3', 'WPA3.').
enum_entry_description('AIRSPEED_SENSOR_FLAGS', 'AIRSPEED_SENSOR_UNHEALTHY', 'Airspeed sensor is unhealthy').
enum_entry_description('AIRSPEED_SENSOR_FLAGS', 'AIRSPEED_SENSOR_USING', 'True if the data from this sensor is being actively used by the flight controller for guidance, navigation or control.').
enum_entry_description('PARAM_TRANSACTION_TRANSPORT', 'PARAM_TRANSACTION_TRANSPORT_PARAM', 'Transaction over param transport.').
enum_entry_description('PARAM_TRANSACTION_TRANSPORT', 'PARAM_TRANSACTION_TRANSPORT_PARAM_EXT', 'Transaction over param_ext transport.').
enum_entry_description('PARAM_TRANSACTION_ACTION', 'PARAM_TRANSACTION_ACTION_START', 'Commit the current parameter transaction.').
enum_entry_description('PARAM_TRANSACTION_ACTION', 'PARAM_TRANSACTION_ACTION_COMMIT', 'Commit the current parameter transaction.').
enum_entry_description('PARAM_TRANSACTION_ACTION', 'PARAM_TRANSACTION_ACTION_CANCEL', 'Cancel the current parameter transaction.').
enum_entry_description('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_NON_STANDARD', 'Non standard mode.\n          This may be used when reporting the mode if the current flight mode is not a standard mode.\n        ').
enum_entry_description('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_POSITION_HOLD', 'Position mode (manual).\n          Position-controlled and stabilized manual mode.\n          When sticks are released vehicles return to their level-flight orientation and hold both position and altitude against wind and external forces.\n          This mode can only be set by vehicles that can hold a fixed position.\n          Multicopter (MC) vehicles actively brake and hold both position and altitude against wind and external forces.\n          Hybrid MC/FW ("VTOL") vehicles first transition to multicopter mode (if needed) but otherwise behave in the same way as MC vehicles.\n          Fixed-wing (FW) vehicles must not support this mode.\n          Other vehicle types must not support this mode (this may be revisited through the PR process).\n        ').
enum_entry_description('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_ORBIT', 'Orbit (manual).\n          Position-controlled and stabilized manual mode.\n          The vehicle circles around a fixed setpoint in the horizontal plane at a particular radius, altitude, and direction.\n          Flight stacks may further allow manual control over the setpoint position, radius, direction, speed, and/or altitude of the circle, but this is not mandated.\n          Flight stacks may support the [MAV_CMD_DO_ORBIT](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_ORBIT) for changing the orbit parameters.\n          MC and FW vehicles may support this mode.\n          Hybrid MC/FW ("VTOL") vehicles may support this mode in MC/FW or both modes; if the mode is not supported by the current configuration the vehicle should transition to the supported configuration.\n          Other vehicle types must not support this mode (this may be revisited through the PR process).\n        ').
enum_entry_description('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_CRUISE', 'Cruise mode (manual).\n          Position-controlled and stabilized manual mode.\n          When sticks are released vehicles return to their level-flight orientation and hold their original track against wind and external forces.\n          Fixed-wing (FW) vehicles level orientation and maintain current track and altitude against wind and external forces.\n          Hybrid MC/FW ("VTOL") vehicles first transition to FW mode (if needed) but otherwise behave in the same way as MC vehicles.\n          Multicopter (MC) vehicles must not support this mode.\n          Other vehicle types must not support this mode (this may be revisited through the PR process).\n        ').
enum_entry_description('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_ALTITUDE_HOLD', 'Altitude hold (manual).\n          Altitude-controlled and stabilized manual mode.\n          When sticks are released vehicles return to their level-flight orientation and hold their altitude.\n          MC vehicles continue with existing momentum and may move with wind (or other external forces).\n          FW vehicles continue with current heading, but may be moved off-track by wind.\n          Hybrid MC/FW ("VTOL") vehicles behave according to their current configuration/mode (FW or MC).\n          Other vehicle types must not support this mode (this may be revisited through the PR process).\n        ').
enum_entry_description('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_RETURN_HOME', 'Return home mode (auto).\n          Automatic mode that returns vehicle to home via a safe flight path.\n          It may also automatically land the vehicle (i.e. RTL).\n          The precise flight path and landing behaviour depend on vehicle configuration and type.\n        ').
enum_entry_description('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_SAFE_RECOVERY', 'Safe recovery mode (auto).\n          Automatic mode that takes vehicle to a predefined safe location via a safe flight path (rally point or mission defined landing) .\n          It may also automatically land the vehicle.\n          The precise return location, flight path, and landing behaviour depend on vehicle configuration and type.\n        ').
enum_entry_description('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_MISSION', 'Mission mode (automatic).\n          Automatic mode that executes MAVLink missions.\n          Missions are executed from the current waypoint as soon as the mode is enabled.\n        ').
enum_entry_description('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_LAND', 'Land mode (auto).\n          Automatic mode that lands the vehicle at the current location.\n          The precise landing behaviour depends on vehicle configuration and type.\n        ').
enum_entry_description('MAV_STANDARD_MODE', 'MAV_STANDARD_MODE_TAKEOFF', 'Takeoff mode (auto).\n          Automatic takeoff mode.\n          The precise takeoff behaviour depends on vehicle configuration and type.\n        ').
enum_entry_description('MAV_MODE_PROPERTY', 'MAV_MODE_PROPERTY_ADVANCED', 'If set, this mode is an advanced mode.\n          For example a rate-controlled manual mode might be advanced, whereas a position-controlled manual mode is not.\n          A GCS can optionally use this flag to configure the UI for its intended users.\n        ').
enum_entry_description('MAV_MODE_PROPERTY', 'MAV_MODE_PROPERTY_NOT_USER_SELECTABLE', 'If set, this mode should not be added to the list of selectable modes.\n          The mode might still be selected by the FC directly (for example as part of a failsafe).\n        ').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_FIGURE_EIGHT', 'Fly a figure eight path as defined by the parameters.\n          Set parameters to NaN/INT32_MAX (as appropriate) to use system-default values.\n          The command is intended for fixed wing vehicles (and VTOL hybrids flying in fixed-wing mode), allowing POI tracking for gimbals that don\'t support infinite rotation.\n          This command only defines the flight path. Speed should be set independently (use e.g. MAV_CMD_DO_CHANGE_SPEED).\n          Yaw and other degrees of freedom are not specified, and will be flight-stack specific (on vehicles where they can be controlled independent of the heading).\n        ').
enum_entry_description('MAV_CMD', 'MAV_CMD_PARAM_TRANSACTION', 'Request to start or end a parameter transaction. Multiple kinds of transport layers can be used to exchange parameters in the transaction (param, param_ext and mavftp). The command response can either be a success/failure or an in progress in case the receiving side takes some time to apply the parameters.').
enum_entry_description('MAV_CMD', 'MAV_CMD_SET_FENCE_BREACH_ACTION', 'Sets the action on geofence breach.\n          If sent using the command protocol this sets the system-default geofence action.\n          As part of a mission protocol plan it sets the fence action for the next complete geofence definition *after* the command.\n          Note: A fence action defined in a plan will override the default system setting (even if the system-default is `FENCE_ACTION_NONE`).\n          Note: Every geofence in a plan can have its own action; if no fence action is defined for a particular fence the system-default will be used.\n          Note: The flight stack should reject a plan or command that uses a geofence action that it does not support and send a STATUSTEXT with the reason.\n        ').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_NOT_READY_TO_USE', '\n          The battery is not ready to use (fly).\n          Set if the battery has faults or other conditions that make it unsafe to fly with.\n          Note: It will be the logical OR of other status bits (chosen by the manufacturer/integrator).\n        ').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_CHARGING', '\n          Battery is charging.\n        ').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_CELL_BALANCING', '\n          Battery is cell balancing (during charging).\n          Not ready to use (MAV_BATTERY_STATUS_FLAGS_NOT_READY_TO_USE may be set).\n        ').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_CELL_IMBALANCE', '\n          Battery cells are not balanced.\n          Not ready to use.\n        ').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_AUTO_DISCHARGING', '\n          Battery is auto discharging (towards storage level).\n          Not ready to use (MAV_BATTERY_STATUS_FLAGS_NOT_READY_TO_USE would be set).\n        ').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_REQUIRES_SERVICE', '\n          Battery requires service (not safe to fly). \n          This is set at vendor discretion.\n          It is likely to be set for most faults, and may also be set according to a maintenance schedule (such as age, or number of recharge cycles, etc.).\n        ').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_BAD_BATTERY', '\n          Battery is faulty and cannot be repaired (not safe to fly). \n          This is set at vendor discretion.\n          The battery should be disposed of safely.\n        ').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_PROTECTIONS_ENABLED', '\n          Automatic battery protection monitoring is enabled.\n          When enabled, the system will monitor for certain kinds of faults, such as cells being over-voltage.\n          If a fault is triggered then and protections are enabled then a safety fault (MAV_BATTERY_STATUS_FLAGS_FAULT_PROTECTION_SYSTEM) will be set and power from the battery will be stopped.\n          Note that battery protection monitoring should only be enabled when the vehicle is landed. Once the vehicle is armed, or starts moving, the protections should be disabled to prevent false positives from disabling the output.\n        ').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_PROTECTION_SYSTEM', '\n          The battery fault protection system had detected a fault and cut all power from the battery.\n          This will only trigger if MAV_BATTERY_STATUS_FLAGS_PROTECTIONS_ENABLED is set.\n          Other faults like MAV_BATTERY_STATUS_FLAGS_FAULT_OVER_VOLT may also be set, indicating the cause of the protection fault.\n        ').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_OVER_VOLT', 'One or more cells are above their maximum voltage rating.').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_UNDER_VOLT', '\n          One or more cells are below their minimum voltage rating.\n          A battery that had deep-discharged might be irrepairably damaged, and set both MAV_BATTERY_STATUS_FLAGS_FAULT_UNDER_VOLT and MAV_BATTERY_STATUS_FLAGS_BAD_BATTERY.\n        ').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_OVER_TEMPERATURE', 'Over-temperature fault.').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_UNDER_TEMPERATURE', 'Under-temperature fault.').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_OVER_CURRENT', 'Over-current fault.').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_SHORT_CIRCUIT', '\n          Short circuit event detected.\n          The battery may or may not be safe to use (check other flags).\n        ').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_INCOMPATIBLE_VOLTAGE', 'Voltage not compatible with power rail voltage (batteries on same power rail should have similar voltage).').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_INCOMPATIBLE_FIRMWARE', 'Battery firmware is not compatible with current autopilot firmware.').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION', 'Battery is not compatible due to cell configuration (e.g. 5s1p when vehicle requires 6s).').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_CAPACITY_RELATIVE_TO_FULL', '\n          Battery capacity_consumed and capacity_remaining values are relative to a full battery (they sum to the total capacity of the battery).\n          This flag would be set for a smart battery that can accurately determine its remaining charge across vehicle reboots and discharge/recharge cycles.\n          If unset the capacity_consumed indicates the consumption since vehicle power-on, as measured using a power monitor. The capacity_remaining, if provided, indicates the estimated remaining capacity on the assumption that the battery was full on vehicle boot.\n          If unset a GCS is recommended to advise that users fully charge the battery on power on.\n        ').
enum_entry_description('MAV_BATTERY_STATUS_FLAGS', 'MAV_BATTERY_STATUS_FLAGS_EXTENDED', 'Reserved (not used). If set, this will indicate that an additional status field exists for higher status values.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_UPGRADE', 'Request a target system to start an upgrade of one (or all) of its components.\n          For example, the command might be sent to a companion computer to cause it to upgrade a connected flight controller.\n          The system doing the upgrade will report progress using the normal command protocol sequence for a long running operation.\n          Command protocol information: https://mavlink.io/en/services/command.html.').
enum_entry_description('MAV_CMD', 'MAV_CMD_GROUP_START', 'Define start of a group of mission items. When control reaches this command a GROUP_START message is emitted.\n          The end of a group is marked using MAV_CMD_GROUP_END with the same group id.\n          Group ids are expected, but not required, to iterate sequentially.\n          Groups can be nested.').
enum_entry_description('MAV_CMD', 'MAV_CMD_GROUP_END', 'Define end of a group of mission items. When control reaches this command a GROUP_END message is emitted.\n          The start of the group is marked is marked using MAV_CMD_GROUP_START with the same group id.\n          Group ids are expected, but not required, to iterate sequentially.\n          Groups can be nested.').
enum_entry_description('MAV_CMD', 'MAV_CMD_DO_SET_STANDARD_MODE', 'Enable the specified standard MAVLink mode.\n          If the mode is not supported the vehicle should ACK with MAV_RESULT_FAILED.\n        ').
enum_entry_description('MAV_CMD', 'MAV_CMD_SET_AT_S_PARAM', 'Allows setting an AT S command of an SiK radio.\n        ').
enum_entry_description('TARGET_OBS_FRAME', 'TARGET_OBS_FRAME_LOCAL_NED', 'NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.').
enum_entry_description('TARGET_OBS_FRAME', 'TARGET_OBS_FRAME_BODY_FRD', 'FRD local frame aligned to the vehicle\'s attitude (x: Forward, y: Right, z: Down) with an origin that travels with vehicle.').
enum_entry_description('TARGET_OBS_FRAME', 'TARGET_OBS_FRAME_LOCAL_OFFSET_NED', 'NED local tangent frame (x: North, y: East, z: Down) with an origin that travels with vehicle.').
enum_entry_description('TARGET_OBS_FRAME', 'TARGET_OBS_FRAME_OTHER', 'Other sensor frame for target observations neither in local NED nor in body FRD.').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_GENERIC', 'Generic autopilot, full support for everything').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_RESERVED', 'Reserved for future use.').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_SLUGS', 'SLUGS autopilot, http://slugsuav.soe.ucsc.edu').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_ARDUPILOTMEGA', 'ArduPilot - Plane/Copter/Rover/Sub/Tracker, https://ardupilot.org').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_OPENPILOT', 'OpenPilot, http://openpilot.org').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY', 'Generic autopilot only supporting simple waypoints').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY', 'Generic autopilot supporting waypoints and other simple navigation commands').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_GENERIC_MISSION_FULL', 'Generic autopilot supporting the full mission command set').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_INVALID', 'No valid autopilot, e.g. a GCS or other MAVLink component').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_PPZ', 'PPZ UAV - http://nongnu.org/paparazzi').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_UDB', 'UAV Dev Board').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_FP', 'FlexiPilot').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_PX4', 'PX4 Autopilot - http://px4.io/').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_SMACCMPILOT', 'SMACCMPilot - http://smaccmpilot.org').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_AUTOQUAD', 'AutoQuad -- http://autoquad.org').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_ARMAZILA', 'Armazila -- http://armazila.com').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_AEROB', 'Aerob -- http://aerob.ru').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_ASLUAV', 'ASLUAV autopilot -- http://www.asl.ethz.ch').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_SMARTAP', 'SmartAP Autopilot - http://sky-drones.com').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_AIRRAILS', 'AirRails - http://uaventure.com').
enum_entry_description('MAV_AUTOPILOT', 'MAV_AUTOPILOT_REFLEX', 'Fusion Reflex - https://fusion.engineering').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_GENERIC', 'Generic micro air vehicle').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_FIXED_WING', 'Fixed wing aircraft.').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_QUADROTOR', 'Quadrotor').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_COAXIAL', 'Coaxial helicopter').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_HELICOPTER', 'Normal helicopter with tail rotor.').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_ANTENNA_TRACKER', 'Ground installation').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_GCS', 'Operator control unit / ground control station').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_AIRSHIP', 'Airship, controlled').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_FREE_BALLOON', 'Free balloon, uncontrolled').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_ROCKET', 'Rocket').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_GROUND_ROVER', 'Ground rover').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_SURFACE_BOAT', 'Surface vessel, boat, ship').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_SUBMARINE', 'Submarine').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_HEXAROTOR', 'Hexarotor').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_OCTOROTOR', 'Octorotor').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_TRICOPTER', 'Tricopter').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_FLAPPING_WING', 'Flapping wing').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_KITE', 'Kite').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_ONBOARD_CONTROLLER', 'Onboard companion controller').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_VTOL_TAILSITTER_DUOROTOR', 'Two-rotor Tailsitter VTOL that additionally uses control surfaces in vertical operation. Note, value previously named MAV_TYPE_VTOL_DUOROTOR.').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_VTOL_TAILSITTER_QUADROTOR', 'Quad-rotor Tailsitter VTOL using a V-shaped quad config in vertical operation. Note: value previously named MAV_TYPE_VTOL_QUADROTOR.').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_VTOL_TILTROTOR', 'Tiltrotor VTOL. Fuselage and wings stay (nominally) horizontal in all flight phases. It able to tilt (some) rotors to provide thrust in cruise flight.').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_VTOL_FIXEDROTOR', 'VTOL with separate fixed rotors for hover and cruise flight. Fuselage and wings stay (nominally) horizontal in all flight phases.').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_VTOL_TAILSITTER', 'Tailsitter VTOL. Fuselage and wings orientation changes depending on flight phase: vertical for hover, horizontal for cruise. Use more specific VTOL MAV_TYPE_VTOL_DUOROTOR or MAV_TYPE_VTOL_QUADROTOR if appropriate.').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_VTOL_TILTWING', 'Tiltwing VTOL. Fuselage stays horizontal in all flight phases. The whole wing, along with any attached engine, can tilt between vertical and horizontal mode.').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_VTOL_RESERVED5', 'VTOL reserved 5').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_GIMBAL', 'Gimbal').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_ADSB', 'ADSB system').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_PARAFOIL', 'Steerable, nonrigid airfoil').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_DODECAROTOR', 'Dodecarotor').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_CAMERA', 'Camera').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_CHARGING_STATION', 'Charging station').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_FLARM', 'FLARM collision avoidance system').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_SERVO', 'Servo').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_ODID', 'Open Drone ID. See https://mavlink.io/en/services/opendroneid.html.').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_DECAROTOR', 'Decarotor').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_BATTERY', 'Battery').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_PARACHUTE', 'Parachute').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_LOG', 'Log').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_OSD', 'OSD').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_IMU', 'IMU').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_GPS', 'GPS').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_WINCH', 'Winch').
enum_entry_description('MAV_TYPE', 'MAV_TYPE_GENERIC_MULTIROTOR', 'Generic multirotor that does not fit into a specific type or whose type is unknown').
enum_entry_description('MAV_MODE_FLAG', 'MAV_MODE_FLAG_SAFETY_ARMED', '0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state.').
enum_entry_description('MAV_MODE_FLAG', 'MAV_MODE_FLAG_MANUAL_INPUT_ENABLED', '0b01000000 remote control input is enabled.').
enum_entry_description('MAV_MODE_FLAG', 'MAV_MODE_FLAG_HIL_ENABLED', '0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.').
enum_entry_description('MAV_MODE_FLAG', 'MAV_MODE_FLAG_STABILIZE_ENABLED', '0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.').
enum_entry_description('MAV_MODE_FLAG', 'MAV_MODE_FLAG_GUIDED_ENABLED', '0b00001000 guided mode enabled, system flies waypoints / mission items.').
enum_entry_description('MAV_MODE_FLAG', 'MAV_MODE_FLAG_AUTO_ENABLED', '0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.').
enum_entry_description('MAV_MODE_FLAG', 'MAV_MODE_FLAG_TEST_ENABLED', '0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.').
enum_entry_description('MAV_MODE_FLAG', 'MAV_MODE_FLAG_CUSTOM_MODE_ENABLED', '0b00000001 Reserved for future use.').
enum_entry_description('MAV_MODE_FLAG_DECODE_POSITION', 'MAV_MODE_FLAG_DECODE_POSITION_SAFETY', 'First bit:  10000000').
enum_entry_description('MAV_MODE_FLAG_DECODE_POSITION', 'MAV_MODE_FLAG_DECODE_POSITION_MANUAL', 'Second bit: 01000000').
enum_entry_description('MAV_MODE_FLAG_DECODE_POSITION', 'MAV_MODE_FLAG_DECODE_POSITION_HIL', 'Third bit:  00100000').
enum_entry_description('MAV_MODE_FLAG_DECODE_POSITION', 'MAV_MODE_FLAG_DECODE_POSITION_STABILIZE', 'Fourth bit: 00010000').
enum_entry_description('MAV_MODE_FLAG_DECODE_POSITION', 'MAV_MODE_FLAG_DECODE_POSITION_GUIDED', 'Fifth bit:  00001000').
enum_entry_description('MAV_MODE_FLAG_DECODE_POSITION', 'MAV_MODE_FLAG_DECODE_POSITION_AUTO', 'Sixth bit:   00000100').
enum_entry_description('MAV_MODE_FLAG_DECODE_POSITION', 'MAV_MODE_FLAG_DECODE_POSITION_TEST', 'Seventh bit: 00000010').
enum_entry_description('MAV_MODE_FLAG_DECODE_POSITION', 'MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE', 'Eighth bit: 00000001').
enum_entry_description('MAV_STATE', 'MAV_STATE_UNINIT', 'Uninitialized system, state is unknown.').
enum_entry_description('MAV_STATE', 'MAV_STATE_BOOT', 'System is booting up.').
enum_entry_description('MAV_STATE', 'MAV_STATE_CALIBRATING', 'System is calibrating and not flight-ready.').
enum_entry_description('MAV_STATE', 'MAV_STATE_STANDBY', 'System is grounded and on standby. It can be launched any time.').
enum_entry_description('MAV_STATE', 'MAV_STATE_ACTIVE', 'System is active and might be already airborne. Motors are engaged.').
enum_entry_description('MAV_STATE', 'MAV_STATE_CRITICAL', 'System is in a non-normal flight mode (failsafe). It can however still navigate.').
enum_entry_description('MAV_STATE', 'MAV_STATE_EMERGENCY', 'System is in a non-normal flight mode (failsafe). It lost control over parts or over the whole airframe. It is in mayday and going down.').
enum_entry_description('MAV_STATE', 'MAV_STATE_POWEROFF', 'System just initialized its power-down sequence, will shut down now.').
enum_entry_description('MAV_STATE', 'MAV_STATE_FLIGHT_TERMINATION', 'System is terminating itself (failsafe or commanded).').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_ALL', 'Target id (target_component) used to broadcast messages to all components of the receiving system. Components should attempt to process messages with this component ID and forward to components on any other interfaces. Note: This is not a valid *source* component id for a message.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_AUTOPILOT1', 'System flight controller component ("autopilot"). Only one autopilot is expected in a particular system.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER1', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER2', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER3', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER4', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER5', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER6', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER7', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER8', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER9', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER10', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER11', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER12', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER13', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER14', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER15', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER16', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER17', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER18', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER19', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER20', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER21', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER22', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER23', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER24', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER25', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER26', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER27', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER28', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER29', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER30', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER31', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER32', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER33', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER34', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER35', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER36', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER37', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER38', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER39', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER40', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER41', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER42', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER43', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_TELEMETRY_RADIO', 'Telemetry radio (e.g. SiK radio, or other component that emits RADIO_STATUS messages).').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER45', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER46', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER47', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER48', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER49', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER50', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER51', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER52', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER53', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER54', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER55', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER56', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER57', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER58', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER59', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER60', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER61', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER62', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER63', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER64', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER65', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER66', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER67', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER68', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER69', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER70', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER71', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER72', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER73', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER74', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_USER75', 'Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_CAMERA', 'Camera #1.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_CAMERA2', 'Camera #2.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_CAMERA3', 'Camera #3.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_CAMERA4', 'Camera #4.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_CAMERA5', 'Camera #5.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_CAMERA6', 'Camera #6.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_SERVO1', 'Servo #1.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_SERVO2', 'Servo #2.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_SERVO3', 'Servo #3.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_SERVO4', 'Servo #4.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_SERVO5', 'Servo #5.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_SERVO6', 'Servo #6.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_SERVO7', 'Servo #7.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_SERVO8', 'Servo #8.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_SERVO9', 'Servo #9.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_SERVO10', 'Servo #10.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_SERVO11', 'Servo #11.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_SERVO12', 'Servo #12.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_SERVO13', 'Servo #13.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_SERVO14', 'Servo #14.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_GIMBAL', 'Gimbal #1.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_LOG', 'Logging component.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_ADSB', 'Automatic Dependent Surveillance-Broadcast (ADS-B) component.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_OSD', 'On Screen Display (OSD) devices for video links.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_PERIPHERAL', 'Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter microservice.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_QX1_GIMBAL', 'Gimbal ID for QX1.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_FLARM', 'FLARM collision alert component.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_PARACHUTE', 'Parachute component.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_WINCH', 'Winch component.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_GIMBAL2', 'Gimbal #2.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_GIMBAL3', 'Gimbal #3.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_GIMBAL4', 'Gimbal #4').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_GIMBAL5', 'Gimbal #5.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_GIMBAL6', 'Gimbal #6.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_BATTERY', 'Battery #1.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_BATTERY2', 'Battery #2.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_MAVCAN', 'CAN over MAVLink client.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_MISSIONPLANNER', 'Component that can generate/supply a mission flight plan (e.g. GCS or developer API).').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_ONBOARD_COMPUTER', 'Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don\'t directly speak mavlink and so on.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_ONBOARD_COMPUTER2', 'Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don\'t directly speak mavlink and so on.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_ONBOARD_COMPUTER3', 'Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don\'t directly speak mavlink and so on.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_ONBOARD_COMPUTER4', 'Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don\'t directly speak mavlink and so on.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_PATHPLANNER', 'Component that finds an optimal path between points based on a certain constraint (e.g. minimum snap, shortest path, cost, etc.).').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_OBSTACLE_AVOIDANCE', 'Component that plans a collision free path between two points.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY', 'Component that provides position estimates using VIO techniques.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_PAIRING_MANAGER', 'Component that manages pairing of vehicle and GCS.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_IMU', 'Inertial Measurement Unit (IMU) #1.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_IMU_2', 'Inertial Measurement Unit (IMU) #2.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_IMU_3', 'Inertial Measurement Unit (IMU) #3.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_GPS', 'GPS #1.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_GPS2', 'GPS #2.').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_ODID_TXRX_1', 'Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_ODID_TXRX_2', 'Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_ODID_TXRX_3', 'Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_UDP_BRIDGE', 'Component to bridge MAVLink to UDP (i.e. from a UART).').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_UART_BRIDGE', 'Component to bridge to UART (i.e. from UDP).').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_TUNNEL_NODE', 'Component handling TUNNEL messages (e.g. vendor specific GUI of a component).').
enum_entry_description('MAV_COMPONENT', 'MAV_COMP_ID_SYSTEM_CONTROL', 'Deprecated, don\'t use. Component for handling system messages (e.g. to ARM, takeoff, etc.).').
enum_entry_description('UALBERTA_AUTOPILOT_MODE', 'MODE_MANUAL_DIRECT', 'Raw input pulse widts sent to output').
enum_entry_description('UALBERTA_AUTOPILOT_MODE', 'MODE_MANUAL_SCALED', 'Inputs are normalized using calibration, the converted back to raw pulse widths for output').
enum_entry_description('UALBERTA_NAV_MODE', 'NAV_AHRS', 'AHRS mode').
enum_entry_description('UALBERTA_NAV_MODE', 'NAV_INS_GPS_INIT', 'INS/GPS initialization mode').
enum_entry_description('UALBERTA_NAV_MODE', 'NAV_INS_GPS', 'INS/GPS mode').
enum_entry_description('UALBERTA_PILOT_MODE', 'PILOT_ROTO', ' Rotomotion mode ').
enum_entry_description('MAV_STORM32_TUNNEL_PAYLOAD_TYPE', 'MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH1_IN', 'Registered for STorM32 gimbal controller. For communication with gimbal or camera.').
enum_entry_description('MAV_STORM32_TUNNEL_PAYLOAD_TYPE', 'MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH1_OUT', 'Registered for STorM32 gimbal controller. For communication with gimbal or camera.').
enum_entry_description('MAV_STORM32_TUNNEL_PAYLOAD_TYPE', 'MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH2_IN', 'Registered for STorM32 gimbal controller. For communication with gimbal.').
enum_entry_description('MAV_STORM32_TUNNEL_PAYLOAD_TYPE', 'MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH2_OUT', 'Registered for STorM32 gimbal controller. For communication with gimbal.').
enum_entry_description('MAV_STORM32_TUNNEL_PAYLOAD_TYPE', 'MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH3_IN', 'Registered for STorM32 gimbal controller. For communication with camera.').
enum_entry_description('MAV_STORM32_TUNNEL_PAYLOAD_TYPE', 'MAV_STORM32_TUNNEL_PAYLOAD_TYPE_STORM32_CH3_OUT', 'Registered for STorM32 gimbal controller. For communication with camera.').
enum_entry_description('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_IS_NORMAL', 'STorM32 gimbal is in normal state.').
enum_entry_description('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_IMUS_WORKING', 'The IMUs are healthy and working normally.').
enum_entry_description('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_MOTORS_WORKING', 'The motors are active and working normally.').
enum_entry_description('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_ENCODERS_WORKING', 'The encoders are healthy and working normally.').
enum_entry_description('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_VOLTAGE_OK', 'A battery voltage is applied and is in range.').
enum_entry_description('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_VIRTUALCHANNELS_RECEIVING', 'Virtual input channels are receiving data.').
enum_entry_description('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_MAVLINK_RECEIVING', 'Mavlink messages are being received.').
enum_entry_description('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_STORM32LINK_QFIX', 'The STorM32Link data indicates QFix.').
enum_entry_description('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_STORM32LINK_WORKING', 'The STorM32Link is working.').
enum_entry_description('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_CAMERA_CONNECTED', 'The camera has been found and is connected.').
enum_entry_description('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_AUX0_LOW', 'The signal on the AUX0 input pin is low.').
enum_entry_description('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_AUX1_LOW', 'The signal on the AUX1 input pin is low.').
enum_entry_description('MAV_STORM32_GIMBAL_PREARM_FLAGS', 'MAV_STORM32_GIMBAL_PREARM_FLAGS_NTLOGGER_WORKING', 'The NTLogger is working normally.').
enum_entry_description('MAV_STORM32_CAMERA_PREARM_FLAGS', 'MAV_STORM32_CAMERA_PREARM_FLAGS_CONNECTED', 'The camera has been found and is connected.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_CAP_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_CAP_FLAGS_HAS_PROFILES', 'The gimbal manager supports several profiles.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_NONE', '0 = ignore.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_RC_ACTIVE', 'Request to set RC input to active, or report RC input is active. Implies RC mixed. RC exclusive is achieved by setting all clients to inactive.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_ONBOARD_ACTIVE', 'Request to set onboard/companion computer client to active, or report this client is active.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_AUTOPILOT_ACTIVE', 'Request to set autopliot client to active, or report this client is active.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_GCS_ACTIVE', 'Request to set GCS client to active, or report this client is active.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_CAMERA_ACTIVE', 'Request to set camera client to active, or report this client is active.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_GCS2_ACTIVE', 'Request to set GCS2 client to active, or report this client is active.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_CAMERA2_ACTIVE', 'Request to set camera2 client to active, or report this client is active.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_CUSTOM_ACTIVE', 'Request to set custom client to active, or report this client is active.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_CLIENT_CUSTOM2_ACTIVE', 'Request to set custom2 client to active, or report this client is active.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_SET_SUPERVISON', 'Request supervision. This flag is only for setting, it is not reported.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_FLAGS', 'MAV_STORM32_GIMBAL_MANAGER_FLAGS_SET_RELEASE', 'Release supervision. This flag is only for setting, it is not reported.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_NONE', 'For convenience.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_ONBOARD', 'This is the onboard/companion computer client.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_AUTOPILOT', 'This is the autopilot client.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_GCS', 'This is the GCS client.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_CAMERA', 'This is the camera client.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_GCS2', 'This is the GCS2 client.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_CAMERA2', 'This is the camera2 client.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_CUSTOM', 'This is the custom client.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_CLIENT', 'MAV_STORM32_GIMBAL_MANAGER_CLIENT_CUSTOM2', 'This is the custom2 client.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_PROFILE', 'MAV_STORM32_GIMBAL_MANAGER_PROFILE_DEFAULT', 'Default profile. Implementation specific.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_PROFILE', 'MAV_STORM32_GIMBAL_MANAGER_PROFILE_CUSTOM', 'Not supported/deprecated.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_PROFILE', 'MAV_STORM32_GIMBAL_MANAGER_PROFILE_COOPERATIVE', 'Profile with cooperative behavior.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_PROFILE', 'MAV_STORM32_GIMBAL_MANAGER_PROFILE_EXCLUSIVE', 'Profile with exclusive behavior.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_PROFILE', 'MAV_STORM32_GIMBAL_MANAGER_PROFILE_PRIORITY_COOPERATIVE', 'Profile with priority and cooperative behavior for equal priority.').
enum_entry_description('MAV_STORM32_GIMBAL_MANAGER_PROFILE', 'MAV_STORM32_GIMBAL_MANAGER_PROFILE_PRIORITY_EXCLUSIVE', 'Profile with priority and exclusive behavior for equal priority.').
enum_entry_description('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_UNDEFINED', 'Undefined shot mode. Can be used to determine if qshots should be used or not.').
enum_entry_description('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_DEFAULT', 'Start normal gimbal operation. Is usually used to return back from a shot.').
enum_entry_description('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_GIMBAL_RETRACT', 'Load and keep safe gimbal position and stop stabilization.').
enum_entry_description('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_GIMBAL_NEUTRAL', 'Load neutral gimbal position and keep it while stabilizing.').
enum_entry_description('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_GIMBAL_MISSION', 'Start mission with gimbal control.').
enum_entry_description('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_GIMBAL_RC_CONTROL', 'Start RC gimbal control.').
enum_entry_description('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_POI_TARGETING', 'Start gimbal tracking the point specified by Lat, Lon, Alt.').
enum_entry_description('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_SYSID_TARGETING', 'Start gimbal tracking the system with specified system ID.').
enum_entry_description('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_CABLECAM_2POINT', 'Start 2-point cable cam quick shot.').
enum_entry_description('MAV_QSHOT_MODE', 'MAV_QSHOT_MODE_HOME_TARGETING', 'Start gimbal tracking the home location.').
enum_entry_description('MAV_CMD', 'MAV_CMD_STORM32_DO_GIMBAL_MANAGER_CONTROL_PITCHYAW', 'Command to a gimbal manager to control the gimbal tilt and pan angles. It is possible to set combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used to signal unset. A gimbal device is never to react to this command.').
enum_entry_description('MAV_CMD', 'MAV_CMD_STORM32_DO_GIMBAL_MANAGER_SETUP', 'Command to configure a gimbal manager. A gimbal device is never to react to this command. The selected profile is reported in the STORM32_GIMBAL_MANAGER_STATUS message.').
enum_entry_description('MAV_CMD', 'MAV_CMD_QSHOT_DO_CONFIGURE', 'Command to set the shot manager mode.').
enum_entry_description('RADIO_RC_CHANNELS_FLAGS', 'RADIO_RC_CHANNELS_FLAGS_FAILSAFE', 'Failsafe is active.').
enum_entry_description('RADIO_RC_CHANNELS_FLAGS', 'RADIO_RC_CHANNELS_FLAGS_FRAME_MISSED', 'Indicates that the current frame has not been received. Channel values are frozen.').
enum_entry_description('RADIO_LINK_STATS_FLAGS', 'RADIO_LINK_STATS_FLAGS_RSSI_DBM', 'Rssi are in negative dBm. Values 0..254 corresponds to 0..-254 dBm.').
enum_entry_description('MAV_CMD', 'MAV_CMD_PRS_SET_ARM', 'AVSS defined command. Set PRS arm statuses.').
enum_entry_description('MAV_CMD', 'MAV_CMD_PRS_GET_ARM', 'AVSS defined command. Gets PRS arm statuses').
enum_entry_description('MAV_CMD', 'MAV_CMD_PRS_GET_BATTERY', 'AVSS defined command.  Get the PRS battery voltage in millivolts').
enum_entry_description('MAV_CMD', 'MAV_CMD_PRS_GET_ERR', 'AVSS defined command. Get the PRS error statuses.').
enum_entry_description('MAV_CMD', 'MAV_CMD_PRS_SET_ARM_ALTI', 'AVSS defined command. Set the ATS arming altitude in meters.').
enum_entry_description('MAV_CMD', 'MAV_CMD_PRS_GET_ARM_ALTI', 'AVSS defined command. Get the ATS arming altitude in meters.').
enum_entry_description('MAV_CMD', 'MAV_CMD_PRS_SHUTDOWN', 'AVSS defined command. Shuts down the PRS system.').
enum_entry_description('MAV_AVSS_COMMAND_FAILURE_REASON', 'PRS_NOT_STEADY', 'AVSS defined command failure reason. PRS not steady.').
enum_entry_description('MAV_AVSS_COMMAND_FAILURE_REASON', 'PRS_DTM_NOT_ARMED', 'AVSS defined command failure reason. PRS DTM not armed.').
enum_entry_description('MAV_AVSS_COMMAND_FAILURE_REASON', 'PRS_OTM_NOT_ARMED', 'AVSS defined command failure reason. PRS OTM not armed.').
enum_entry_description('AVSS_M300_OPERATION_MODE', 'MODE_M300_MANUAL_CTRL', 'In manual control mode').
enum_entry_description('AVSS_M300_OPERATION_MODE', 'MODE_M300_ATTITUDE', 'In attitude mode ').
enum_entry_description('AVSS_M300_OPERATION_MODE', 'MODE_M300_P_GPS', 'In GPS mode').
enum_entry_description('AVSS_M300_OPERATION_MODE', 'MODE_M300_HOTPOINT_MODE', 'In hotpoint mode ').
enum_entry_description('AVSS_M300_OPERATION_MODE', 'MODE_M300_ASSISTED_TAKEOFF', 'In assisted takeoff mode').
enum_entry_description('AVSS_M300_OPERATION_MODE', 'MODE_M300_AUTO_TAKEOFF', 'In auto takeoff mode').
enum_entry_description('AVSS_M300_OPERATION_MODE', 'MODE_M300_AUTO_LANDING', 'In auto landing mode').
enum_entry_description('AVSS_M300_OPERATION_MODE', 'MODE_M300_NAVI_GO_HOME', 'In go home mode').
enum_entry_description('AVSS_M300_OPERATION_MODE', 'MODE_M300_NAVI_SDK_CTRL', 'In sdk control mode').
enum_entry_description('AVSS_M300_OPERATION_MODE', 'MODE_M300_S_SPORT', 'In sport mode').
enum_entry_description('AVSS_M300_OPERATION_MODE', 'MODE_M300_FORCE_AUTO_LANDING', 'In force auto landing mode').
enum_entry_description('AVSS_M300_OPERATION_MODE', 'MODE_M300_T_TRIPOD', 'In tripod mode').
enum_entry_description('AVSS_M300_OPERATION_MODE', 'MODE_M300_SEARCH_MODE', 'In search mode').
enum_entry_description('AVSS_M300_OPERATION_MODE', 'MODE_M300_ENGINE_START', 'In engine mode').
enum_entry_description('AVSS_HORSEFLY_OPERATION_MODE', 'MODE_HORSEFLY_MANUAL_CTRL', 'In manual control mode').
enum_entry_description('AVSS_HORSEFLY_OPERATION_MODE', 'MODE_HORSEFLY_AUTO_TAKEOFF', 'In auto takeoff mode').
enum_entry_description('AVSS_HORSEFLY_OPERATION_MODE', 'MODE_HORSEFLY_AUTO_LANDING', 'In auto landing mode').
enum_entry_description('AVSS_HORSEFLY_OPERATION_MODE', 'MODE_HORSEFLY_NAVI_GO_HOME', 'In go home mode').
enum_entry_description('AVSS_HORSEFLY_OPERATION_MODE', 'MODE_HORSEFLY_DROP', 'In drop mode').
enum_entry_description('AIRLINK_AUTH_RESPONSE_TYPE', 'AIRLINK_ERROR_LOGIN_OR_PASS', 'Login or password error').
enum_entry_description('AIRLINK_AUTH_RESPONSE_TYPE', 'AIRLINK_AUTH_OK', 'Auth successful').

:- dynamic enum_entry_param/4.

enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_RESUME_REPEAT_DIST', 1, [label='Distance', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_RESUME_REPEAT_DIST', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_RESUME_REPEAT_DIST', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_RESUME_REPEAT_DIST', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_RESUME_REPEAT_DIST', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_RESUME_REPEAT_DIST', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_RESUME_REPEAT_DIST', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SPRAYER', 1, [label='Sprayer Enable', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SPRAYER', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SPRAYER', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SPRAYER', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SPRAYER', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SPRAYER', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SPRAYER', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SEND_SCRIPT_MESSAGE', 1, [label='ID', minValue='0', maxValue='65535', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SEND_SCRIPT_MESSAGE', 2, [label='param 1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SEND_SCRIPT_MESSAGE', 3, [label='param 2']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SEND_SCRIPT_MESSAGE', 4, [label='param 3']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SEND_SCRIPT_MESSAGE', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SEND_SCRIPT_MESSAGE', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SEND_SCRIPT_MESSAGE', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_AUX_FUNCTION', 1, [label='AuxiliaryFunction']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_AUX_FUNCTION', 2, [label='SwitchPosition', enum='MAV_CMD_DO_AUX_FUNCTION_SWITCH_LEVEL']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_AUX_FUNCTION', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_AUX_FUNCTION', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_AUX_FUNCTION', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_AUX_FUNCTION', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_AUX_FUNCTION', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ALTITUDE_WAIT', 1, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ALTITUDE_WAIT', 2, [label='Descent Speed', units='m/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ALTITUDE_WAIT', 3, [label='Wiggle Time', units=s]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ALTITUDE_WAIT', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ALTITUDE_WAIT', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ALTITUDE_WAIT', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ALTITUDE_WAIT', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_POWER_OFF_INITIATED', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_POWER_OFF_INITIATED', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_POWER_OFF_INITIATED', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_POWER_OFF_INITIATED', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_POWER_OFF_INITIATED', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_POWER_OFF_INITIATED', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_POWER_OFF_INITIATED', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_CLICK', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_CLICK', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_CLICK', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_CLICK', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_CLICK', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_CLICK', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_CLICK', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_HOLD', 1, [label='Takeoff Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_HOLD', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_HOLD', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_HOLD', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_HOLD', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_HOLD', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_FLY_HOLD', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_PAUSE_CLICK', 1, [label='Shot Mode', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_PAUSE_CLICK', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_PAUSE_CLICK', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_PAUSE_CLICK', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_PAUSE_CLICK', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_PAUSE_CLICK', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SOLO_BTN_PAUSE_CLICK', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL', 1, [label='Declination', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL', 2, [label='Inclination', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL', 3, [label='Intensity', units=mgauss]).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL', 4, [label='Yaw', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_FIELD', 1, [label='Field X', units=mgauss]).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_FIELD', 2, [label='Field Y', units=mgauss]).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_FIELD', 3, [label='Field Z', units=mgauss]).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_FIELD', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_FIELD', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_FIELD', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_FIELD', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_EKF_SOURCE_SET', 1, [label='SourceSetId', minValue='1', maxValue='3', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_EKF_SOURCE_SET', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_EKF_SOURCE_SET', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_EKF_SOURCE_SET', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_EKF_SOURCE_SET', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_EKF_SOURCE_SET', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_EKF_SOURCE_SET', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_START_MAG_CAL', 1, [label='Magnetometers Bitmask', minValue='0', maxValue='255', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_START_MAG_CAL', 2, [label='Retry on Failure', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_START_MAG_CAL', 3, [label='Autosave', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_START_MAG_CAL', 4, [label='Delay', units=s]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_START_MAG_CAL', 5, [label='Autoreboot', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_START_MAG_CAL', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_START_MAG_CAL', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ACCEPT_MAG_CAL', 1, [label='Magnetometers Bitmask', minValue='0', maxValue='255', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ACCEPT_MAG_CAL', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ACCEPT_MAG_CAL', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ACCEPT_MAG_CAL', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ACCEPT_MAG_CAL', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ACCEPT_MAG_CAL', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ACCEPT_MAG_CAL', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CANCEL_MAG_CAL', 1, [label='Magnetometers Bitmask', minValue='0', maxValue='255', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CANCEL_MAG_CAL', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CANCEL_MAG_CAL', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CANCEL_MAG_CAL', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CANCEL_MAG_CAL', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CANCEL_MAG_CAL', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CANCEL_MAG_CAL', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_ACCELCAL_VEHICLE_POS', 1, [label='Position', enum='ACCELCAL_VEHICLE_POS']).
enum_entry_param('MAV_CMD', 'MAV_CMD_ACCELCAL_VEHICLE_POS', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_ACCELCAL_VEHICLE_POS', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_ACCELCAL_VEHICLE_POS', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_ACCELCAL_VEHICLE_POS', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_ACCELCAL_VEHICLE_POS', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_ACCELCAL_VEHICLE_POS', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SEND_BANNER', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SEND_BANNER', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SEND_BANNER', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SEND_BANNER', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SEND_BANNER', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SEND_BANNER', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SEND_BANNER', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_FACTORY_TEST_MODE', 1, [label='Test Mode', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_FACTORY_TEST_MODE', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_FACTORY_TEST_MODE', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_FACTORY_TEST_MODE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_FACTORY_TEST_MODE', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_FACTORY_TEST_MODE', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_FACTORY_TEST_MODE', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_RESET', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_RESET', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_RESET', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_RESET', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_RESET', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_RESET', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_RESET', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS', 1, [label='Axis', enum='GIMBAL_AXIS']).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS', 2, [label='Progress', units='%', minValue='0', maxValue='100']).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS', 3, [label='Status', enum='GIMBAL_AXIS_CALIBRATION_STATUS']).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_FULL_RESET', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_FULL_RESET', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_FULL_RESET', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_FULL_RESET', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_FULL_RESET', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_FULL_RESET', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GIMBAL_FULL_RESET', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_FLASH_BOOTLOADER', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_FLASH_BOOTLOADER', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_FLASH_BOOTLOADER', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_FLASH_BOOTLOADER', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_FLASH_BOOTLOADER', 5, [label='Magic Number', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_FLASH_BOOTLOADER', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_FLASH_BOOTLOADER', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_BATTERY_RESET', 1, [label='battery mask']).
enum_entry_param('MAV_CMD', 'MAV_CMD_BATTERY_RESET', 2, [label=percentage, minValue='0', maxValue='100', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DEBUG_TRAP', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DEBUG_TRAP', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DEBUG_TRAP', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DEBUG_TRAP', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DEBUG_TRAP', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DEBUG_TRAP', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DEBUG_TRAP', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SCRIPTING', 1, [enum='SCRIPTING_CMD']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SCRIPT_TIME', 1, [label=command]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SCRIPT_TIME', 2, [label=timeout, units=s]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SCRIPT_TIME', 3, [label=arg1]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SCRIPT_TIME', 4, [label=arg2]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SCRIPT_TIME', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SCRIPT_TIME', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SCRIPT_TIME', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ATTITUDE_TIME', 1, [label=time, units=s]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ATTITUDE_TIME', 2, [label=roll, units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ATTITUDE_TIME', 3, [label=pitch, units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ATTITUDE_TIME', 4, [label=yaw, units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ATTITUDE_TIME', 5, [label=climb_rate, units='m/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ATTITUDE_TIME', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ATTITUDE_TIME', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_SPEED', 1, [label='speed type', enum='SPEED_TYPE']).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_SPEED', 2, [label='speed target', units='m/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_SPEED', 3, [label='speed rate-of-change', units='m/s/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_SPEED', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_SPEED', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_SPEED', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_SPEED', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_ALTITUDE', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_ALTITUDE', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_ALTITUDE', 3, [label='alt rate-of-change', units='m/s/s', minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_ALTITUDE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_ALTITUDE', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_ALTITUDE', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_ALTITUDE', 7, [label='target alt', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_HEADING', 1, [label='heading type', enum='HEADING_TYPE']).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_HEADING', 2, [label='heading target', units=deg, minValue='0', maxValue='359.99']).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_HEADING', 3, [label='heading rate-of-change', units='m/s/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_HEADING', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_HEADING', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_HEADING', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GUIDED_CHANGE_HEADING', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_EXTERNAL_POSITION_ESTIMATE', 1, [label=transmission_time, units=s]).
enum_entry_param('MAV_CMD', 'MAV_CMD_EXTERNAL_POSITION_ESTIMATE', 2, [label=processing_time, units=s]).
enum_entry_param('MAV_CMD', 'MAV_CMD_EXTERNAL_POSITION_ESTIMATE', 3, [label=accuracy]).
enum_entry_param('MAV_CMD', 'MAV_CMD_EXTERNAL_POSITION_ESTIMATE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_EXTERNAL_POSITION_ESTIMATE', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_EXTERNAL_POSITION_ESTIMATE', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_EXTERNAL_POSITION_ESTIMATE', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_RESET_MPPT', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_RESET_MPPT', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_RESET_MPPT', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_RESET_MPPT', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_RESET_MPPT', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_RESET_MPPT', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_RESET_MPPT', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_WAYPOINT', 1, [label='Hold', units=s, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_WAYPOINT', 2, [label='Accept Radius', units=m, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_WAYPOINT', 3, [label='Pass Radius', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_WAYPOINT', 4, [label='Yaw', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_WAYPOINT', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_WAYPOINT', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_WAYPOINT', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_UNLIM', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_UNLIM', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_UNLIM', 3, [label='Radius', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_UNLIM', 4, [label='Yaw', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_UNLIM', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_UNLIM', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_UNLIM', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TURNS', 1, [label='Turns', minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TURNS', 2, [label='Heading Required', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TURNS', 3, [label='Radius', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TURNS', 4, [label='Xtrack Location']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TURNS', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TURNS', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TURNS', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TIME', 1, [label='Time', units=s, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TIME', 2, [label='Heading Required', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TIME', 3, [label='Radius', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TIME', 4, [label='Xtrack Location']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TIME', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TIME', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TIME', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_RETURN_TO_LAUNCH', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_RETURN_TO_LAUNCH', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_RETURN_TO_LAUNCH', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_RETURN_TO_LAUNCH', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_RETURN_TO_LAUNCH', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_RETURN_TO_LAUNCH', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_RETURN_TO_LAUNCH', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAND', 1, [label='Abort Alt', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAND', 2, [label='Land Mode', enum='PRECISION_LAND_MODE']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAND', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAND', 4, [label='Yaw Angle', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAND', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAND', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAND', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF', 1, [label='Pitch', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF', 4, [label='Yaw', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAND_LOCAL', 1, [label='Target', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAND_LOCAL', 2, [label='Offset', units=m, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAND_LOCAL', 3, [label='Descend Rate', units='m/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAND_LOCAL', 4, [label='Yaw', units=rad]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAND_LOCAL', 5, [label='Y Position', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAND_LOCAL', 6, [label='X Position', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAND_LOCAL', 7, [label='Z Position', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF_LOCAL', 1, [label='Pitch', units=rad]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF_LOCAL', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF_LOCAL', 3, [label='Ascend Rate', units='m/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF_LOCAL', 4, [label='Yaw', units=rad]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF_LOCAL', 5, [label='Y Position', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF_LOCAL', 6, [label='X Position', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_TAKEOFF_LOCAL', 7, [label='Z Position', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FOLLOW', 1, [label='Following', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FOLLOW', 2, [label='Ground Speed', units='m/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FOLLOW', 3, [label='Radius', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FOLLOW', 4, [label='Yaw', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FOLLOW', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FOLLOW', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FOLLOW', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT', 1, [label='Action', minValue='0', maxValue='2', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TO_ALT', 1, [label='Heading Required', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TO_ALT', 2, [label='Radius', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TO_ALT', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TO_ALT', 4, [label='Xtrack Location', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TO_ALT', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TO_ALT', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LOITER_TO_ALT', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FOLLOW', 1, [label='System ID', minValue='0', maxValue='255', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FOLLOW', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FOLLOW', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FOLLOW', 4, [label='Altitude Mode', minValue='0', maxValue='2', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FOLLOW', 5, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FOLLOW', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FOLLOW', 7, [label='Time to Land', units=s, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FOLLOW_REPOSITION', 1, [label='Camera Q1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FOLLOW_REPOSITION', 2, [label='Camera Q2']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FOLLOW_REPOSITION', 3, [label='Camera Q3']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FOLLOW_REPOSITION', 4, [label='Camera Q4']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FOLLOW_REPOSITION', 5, [label='Altitude Offset', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FOLLOW_REPOSITION', 6, [label='X Offset', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FOLLOW_REPOSITION', 7, [label='Y Offset', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ORBIT', 1, [label='Radius', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ORBIT', 2, [label='Velocity', units='m/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ORBIT', 3, [label='Yaw Behavior', enum='ORBIT_YAW_BEHAVIOUR']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ORBIT', 4, [label='Orbits', units=rad, minValue='0', default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ORBIT', 5, [label='Latitude/X']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ORBIT', 6, [label='Longitude/Y']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ORBIT', 7, [label='Altitude/Z']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ROI', 1, [label='ROI Mode', enum='MAV_ROI']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ROI', 2, [label='WP Index', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ROI', 3, [label='ROI Index', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ROI', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ROI', 5, [label='X']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ROI', 6, [label='Y']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_ROI', 7, [label='Z']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_PATHPLANNING', 1, [label='Local Ctrl', minValue='0', maxValue='2', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_PATHPLANNING', 2, [label='Global Ctrl', minValue='0', maxValue='3', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_PATHPLANNING', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_PATHPLANNING', 4, [label='Yaw', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_PATHPLANNING', 5, [label='Latitude/X']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_PATHPLANNING', 6, [label='Longitude/Y']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_PATHPLANNING', 7, [label='Altitude/Z']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SPLINE_WAYPOINT', 1, [label='Hold', units=s, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SPLINE_WAYPOINT', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SPLINE_WAYPOINT', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SPLINE_WAYPOINT', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SPLINE_WAYPOINT', 5, [label='Latitude/X']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SPLINE_WAYPOINT', 6, [label='Longitude/Y']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SPLINE_WAYPOINT', 7, [label='Altitude/Z']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_VTOL_TAKEOFF', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_VTOL_TAKEOFF', 2, [label='Transition Heading', enum='VTOL_TRANSITION_HEADING']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_VTOL_TAKEOFF', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_VTOL_TAKEOFF', 4, [label='Yaw Angle', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_VTOL_TAKEOFF', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_VTOL_TAKEOFF', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_VTOL_TAKEOFF', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_VTOL_LAND', 1, [label='Land Options', enum='NAV_VTOL_LAND_OPTIONS']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_VTOL_LAND', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_VTOL_LAND', 3, [label='Approach Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_VTOL_LAND', 4, [label='Yaw', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_VTOL_LAND', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_VTOL_LAND', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_VTOL_LAND', 7, [label='Ground Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_GUIDED_ENABLE', 1, [label='Enable', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_GUIDED_ENABLE', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_GUIDED_ENABLE', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_GUIDED_ENABLE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_GUIDED_ENABLE', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_GUIDED_ENABLE', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_GUIDED_ENABLE', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_DELAY', 1, [label='Delay', units=s, minValue='-1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_DELAY', 2, [label='Hour', minValue='-1', maxValue='23', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_DELAY', 3, [label='Minute', minValue='-1', maxValue='59', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_DELAY', 4, [label='Second', minValue='-1', maxValue='59', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_DELAY', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_DELAY', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_DELAY', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_PAYLOAD_PLACE', 1, [label='Max Descent', units=m, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_PAYLOAD_PLACE', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_PAYLOAD_PLACE', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_PAYLOAD_PLACE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_PAYLOAD_PLACE', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_PAYLOAD_PLACE', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_PAYLOAD_PLACE', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAST', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAST', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAST', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAST', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAST', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAST', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_LAST', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_DELAY', 1, [label='Delay', units=s, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_DELAY', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_DELAY', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_DELAY', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_DELAY', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_DELAY', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_DELAY', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_CHANGE_ALT', 1, [label='Rate', units='m/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_CHANGE_ALT', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_CHANGE_ALT', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_CHANGE_ALT', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_CHANGE_ALT', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_CHANGE_ALT', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_CHANGE_ALT', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_DISTANCE', 1, [label='Distance', units=m, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_DISTANCE', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_DISTANCE', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_DISTANCE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_DISTANCE', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_DISTANCE', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_DISTANCE', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_YAW', 1, [label='Angle', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_YAW', 2, [label='Angular Speed', units='deg/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_YAW', 3, [label='Direction', minValue='-1', maxValue='1', increment='2']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_YAW', 4, [label='Relative', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_YAW', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_YAW', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_YAW', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_LAST', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_LAST', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_LAST', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_LAST', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_LAST', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_LAST', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_LAST', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_MODE', 1, [label='Mode', enum='MAV_MODE']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_MODE', 2, [label='Custom Mode']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_MODE', 3, [label='Custom Submode']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_MODE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_MODE', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_MODE', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_MODE', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_JUMP', 1, [label='Number', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_JUMP', 2, [label='Repeat', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_JUMP', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_JUMP', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_JUMP', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_JUMP', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_JUMP', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CHANGE_SPEED', 1, [label='Speed Type', minValue='0', maxValue='3', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CHANGE_SPEED', 2, [label='Speed', units='m/s', minValue='-2']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CHANGE_SPEED', 3, [label='Throttle', units='%', minValue='-2']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CHANGE_SPEED', 4, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CHANGE_SPEED', 5, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CHANGE_SPEED', 6, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CHANGE_SPEED', 7, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_HOME', 1, [label='Use Current', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_HOME', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_HOME', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_HOME', 4, [label='Yaw', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_HOME', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_HOME', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_HOME', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_PARAMETER', 1, [label='Number', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_PARAMETER', 2, [label='Value']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_PARAMETER', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_PARAMETER', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_PARAMETER', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_PARAMETER', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_PARAMETER', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_RELAY', 1, [label='Instance', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_RELAY', 2, [label='Setting', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_RELAY', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_RELAY', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_RELAY', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_RELAY', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_RELAY', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPEAT_RELAY', 1, [label='Instance', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPEAT_RELAY', 2, [label='Count', minValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPEAT_RELAY', 3, [label='Time', units=s, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPEAT_RELAY', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPEAT_RELAY', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPEAT_RELAY', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPEAT_RELAY', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_SERVO', 1, [label='Instance', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_SERVO', 2, [label='PWM', units=us, minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_SERVO', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_SERVO', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_SERVO', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_SERVO', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_SERVO', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPEAT_SERVO', 1, [label='Instance', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPEAT_SERVO', 2, [label='PWM', units=us, minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPEAT_SERVO', 3, [label='Count', minValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPEAT_SERVO', 4, [label='Time', units=s, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPEAT_SERVO', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPEAT_SERVO', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPEAT_SERVO', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FLIGHTTERMINATION', 1, [label='Terminate', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FLIGHTTERMINATION', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FLIGHTTERMINATION', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FLIGHTTERMINATION', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FLIGHTTERMINATION', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FLIGHTTERMINATION', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FLIGHTTERMINATION', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CHANGE_ALTITUDE', 1, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CHANGE_ALTITUDE', 2, [label='Frame', enum='MAV_FRAME']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CHANGE_ALTITUDE', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CHANGE_ALTITUDE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CHANGE_ALTITUDE', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CHANGE_ALTITUDE', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CHANGE_ALTITUDE', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ACTUATOR', 1, [label='Actuator 1', minValue='-1', maxValue='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ACTUATOR', 2, [label='Actuator 2', minValue='-1', maxValue='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ACTUATOR', 3, [label='Actuator 3', minValue='-1', maxValue='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ACTUATOR', 4, [label='Actuator 4', minValue='-1', maxValue='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ACTUATOR', 5, [label='Actuator 5', minValue='-1', maxValue='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ACTUATOR', 6, [label='Actuator 6', minValue='-1', maxValue='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ACTUATOR', 7, [label='Index', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_LAND_START', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_LAND_START', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_LAND_START', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_LAND_START', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_LAND_START', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_LAND_START', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_LAND_START', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_RALLY_LAND', 1, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_RALLY_LAND', 2, [label='Speed', units='m/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_RALLY_LAND', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_RALLY_LAND', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_RALLY_LAND', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_RALLY_LAND', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_RALLY_LAND', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GO_AROUND', 1, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GO_AROUND', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GO_AROUND', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GO_AROUND', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GO_AROUND', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GO_AROUND', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GO_AROUND', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPOSITION', 1, [label='Speed', units='m/s', minValue='-1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPOSITION', 2, [label='Bitmask', enum='MAV_DO_REPOSITION_FLAGS']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPOSITION', 3, [label='Radius', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPOSITION', 4, [label='Yaw', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPOSITION', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPOSITION', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_REPOSITION', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_PAUSE_CONTINUE', 1, [label='Continue', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_PAUSE_CONTINUE', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_PAUSE_CONTINUE', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_PAUSE_CONTINUE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_PAUSE_CONTINUE', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_PAUSE_CONTINUE', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_PAUSE_CONTINUE', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_REVERSE', 1, [label='Reverse', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_REVERSE', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_REVERSE', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_REVERSE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_REVERSE', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_REVERSE', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_REVERSE', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_LOCATION', 1, [label='Gimbal device ID']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_LOCATION', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_LOCATION', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_LOCATION', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_LOCATION', 5, [label='Latitude', units=degE7]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_LOCATION', 6, [label='Longitude', units=degE7]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_LOCATION', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET', 1, [label='Gimbal device ID']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET', 5, [label='Pitch Offset']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET', 6, [label='Roll Offset']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET', 7, [label='Yaw Offset']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_NONE', 1, [label='Gimbal device ID']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_NONE', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_NONE', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_NONE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_NONE', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_NONE', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_NONE', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_SYSID', 1, [label='System ID', minValue='1', maxValue='255', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI_SYSID', 2, [label='Gimbal device ID']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CONTROL_VIDEO', 1, [label='ID', minValue='-1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CONTROL_VIDEO', 2, [label='Transmission', minValue='0', maxValue='2', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CONTROL_VIDEO', 3, [label='Interval', units=s, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CONTROL_VIDEO', 4, [label='Recording', minValue='0', maxValue='2', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CONTROL_VIDEO', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CONTROL_VIDEO', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_CONTROL_VIDEO', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI', 1, [label='ROI Mode', enum='MAV_ROI']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI', 2, [label='WP Index', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI', 3, [label='ROI Index', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_ROI', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONFIGURE', 1, [label='Mode', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONFIGURE', 2, [label='Shutter Speed', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONFIGURE', 3, [label='Aperture', minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONFIGURE', 4, [label='ISO', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONFIGURE', 5, [label='Exposure']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONFIGURE', 6, [label='Command Identity']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONFIGURE', 7, [label='Engine Cut-off', units=ds, minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONTROL', 1, [label='Session Control']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONTROL', 2, [label='Zoom Absolute']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONTROL', 3, [label='Zoom Relative']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONTROL', 4, [label='Focus']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONTROL', 5, [label='Shoot Command']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONTROL', 6, [label='Command Identity']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_DIGICAM_CONTROL', 7, [label='Shot ID']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONFIGURE', 1, [label='Mode', enum='MAV_MOUNT_MODE']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONFIGURE', 2, [label='Stabilize Roll', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONFIGURE', 3, [label='Stabilize Pitch', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONFIGURE', 4, [label='Stabilize Yaw', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONFIGURE', 5, [label='Roll Input Mode']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONFIGURE', 6, [label='Pitch Input Mode']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONFIGURE', 7, [label='Yaw Input Mode']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL', 1, [label='Pitch']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL', 2, [label='Roll']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL', 3, [label='Yaw']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL', 4, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL', 7, [label='Mode', enum='MAV_MOUNT_MODE']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_DIST', 1, [label='Distance', units=m, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_DIST', 2, [label='Shutter', units=ms, minValue='-1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_DIST', 3, [label='Trigger', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_DIST', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_DIST', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_DIST', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_DIST', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FENCE_ENABLE', 1, [label='Enable', minValue='0', maxValue='2', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FENCE_ENABLE', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FENCE_ENABLE', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FENCE_ENABLE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FENCE_ENABLE', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FENCE_ENABLE', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FENCE_ENABLE', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_PARACHUTE', 1, [label='Action', enum='PARACHUTE_ACTION']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_PARACHUTE', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_PARACHUTE', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_PARACHUTE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_PARACHUTE', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_PARACHUTE', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_PARACHUTE', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOTOR_TEST', 1, [label='Instance', minValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOTOR_TEST', 2, [label='Throttle Type', enum='MOTOR_TEST_THROTTLE_TYPE']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOTOR_TEST', 3, [label='Throttle']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOTOR_TEST', 4, [label='Timeout', units=s, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOTOR_TEST', 5, [label='Motor Count', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOTOR_TEST', 6, [label='Test Order', enum='MOTOR_TEST_ORDER']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOTOR_TEST', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_INVERTED_FLIGHT', 1, [label='Inverted', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_INVERTED_FLIGHT', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_INVERTED_FLIGHT', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_INVERTED_FLIGHT', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_INVERTED_FLIGHT', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_INVERTED_FLIGHT', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_INVERTED_FLIGHT', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GRIPPER', 1, [label='Instance', minValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GRIPPER', 2, [label='Action', enum='GRIPPER_ACTIONS']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GRIPPER', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GRIPPER', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GRIPPER', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GRIPPER', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GRIPPER', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_AUTOTUNE_ENABLE', 1, [label='Enable', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_AUTOTUNE_ENABLE', 2, [label='Axis', enum='AUTOTUNE_AXIS']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_AUTOTUNE_ENABLE', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_AUTOTUNE_ENABLE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_AUTOTUNE_ENABLE', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_AUTOTUNE_ENABLE', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_AUTOTUNE_ENABLE', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SET_YAW_SPEED', 1, [label='Yaw', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SET_YAW_SPEED', 2, [label='Speed', units='m/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SET_YAW_SPEED', 3, [label='Angle', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SET_YAW_SPEED', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SET_YAW_SPEED', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SET_YAW_SPEED', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_SET_YAW_SPEED', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL', 1, [label='Trigger Cycle', units=ms, minValue='-1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL', 2, [label='Shutter Integration', units=ms, minValue='-1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL_QUAT', 1, [label='Q1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL_QUAT', 2, [label='Q2']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL_QUAT', 3, [label='Q3']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL_QUAT', 4, [label='Q4']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL_QUAT', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL_QUAT', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_MOUNT_CONTROL_QUAT', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GUIDED_MASTER', 1, [label='System ID', minValue='0', maxValue='255', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GUIDED_MASTER', 2, [label='Component ID', minValue='0', maxValue='255', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GUIDED_MASTER', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GUIDED_MASTER', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GUIDED_MASTER', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GUIDED_MASTER', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GUIDED_MASTER', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GUIDED_LIMITS', 1, [label='Timeout', units=s, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GUIDED_LIMITS', 2, [label='Min Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GUIDED_LIMITS', 3, [label='Max Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GUIDED_LIMITS', 4, [label='Horiz. Move Limit', units=m, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GUIDED_LIMITS', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GUIDED_LIMITS', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GUIDED_LIMITS', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ENGINE_CONTROL', 1, [label='Start Engine', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ENGINE_CONTROL', 2, [label='Cold Start', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ENGINE_CONTROL', 3, [label='Height Delay', units=m, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ENGINE_CONTROL', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ENGINE_CONTROL', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ENGINE_CONTROL', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ENGINE_CONTROL', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_MISSION_CURRENT', 1, [label='Number', minValue='-1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_MISSION_CURRENT', 2, [label='Reset Mission', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_MISSION_CURRENT', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_MISSION_CURRENT', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_MISSION_CURRENT', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_MISSION_CURRENT', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_MISSION_CURRENT', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_LAST', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_LAST', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_LAST', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_LAST', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_LAST', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_LAST', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_LAST', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_CALIBRATION', 1, [label='Gyro Temperature', minValue='0', maxValue='3', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_CALIBRATION', 2, [label='Magnetometer', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_CALIBRATION', 3, [label='Ground Pressure', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_CALIBRATION', 4, [label='Remote Control', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_CALIBRATION', 5, [label='Accelerometer', minValue='0', maxValue='4', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_CALIBRATION', 6, [label='Compmot or Airspeed', minValue='0', maxValue='2', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_CALIBRATION', 7, [label='ESC or Baro', minValue='0', maxValue='3', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS', 1, [label='Sensor Type', minValue='0', maxValue='6', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS', 2, [label='X Offset']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS', 3, [label='Y Offset']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS', 4, [label='Z Offset']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS', 5, [label='4th Dimension']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS', 6, [label='5th Dimension']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS', 7, [label='6th Dimension']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_UAVCAN', 1, [label='Actuator ID']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_UAVCAN', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_UAVCAN', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_UAVCAN', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_UAVCAN', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_UAVCAN', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_UAVCAN', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_STORAGE', 1, [label='Parameter Storage', enum='PREFLIGHT_STORAGE_PARAMETER_ACTION']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_STORAGE', 2, [label='Mission Storage', enum='PREFLIGHT_STORAGE_MISSION_ACTION']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_STORAGE', 3, [label='Logging Rate', units='Hz', minValue='-1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_STORAGE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_STORAGE', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_STORAGE', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_STORAGE', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN', 1, [label='Autopilot', minValue='0', maxValue='3', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN', 2, [label='Companion', minValue='0', maxValue='3', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN', 3, [label='Component action', minValue='0', maxValue='3', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN', 4, [label='Component ID', minValue='0', maxValue='255', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_OVERRIDE_GOTO', 1, [label='Continue', enum='MAV_GOTO']).
enum_entry_param('MAV_CMD', 'MAV_CMD_OVERRIDE_GOTO', 2, [label='Position', enum='MAV_GOTO']).
enum_entry_param('MAV_CMD', 'MAV_CMD_OVERRIDE_GOTO', 3, [label='Frame', enum='MAV_FRAME']).
enum_entry_param('MAV_CMD', 'MAV_CMD_OVERRIDE_GOTO', 4, [label='Yaw', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_OVERRIDE_GOTO', 5, [label='Latitude/X']).
enum_entry_param('MAV_CMD', 'MAV_CMD_OVERRIDE_GOTO', 6, [label='Longitude/Y']).
enum_entry_param('MAV_CMD', 'MAV_CMD_OVERRIDE_GOTO', 7, [label='Altitude/Z']).
enum_entry_param('MAV_CMD', 'MAV_CMD_OBLIQUE_SURVEY', 1, [label='Distance', units=m, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_OBLIQUE_SURVEY', 2, [label='Shutter', units=ms, minValue='0', increment='1', default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_OBLIQUE_SURVEY', 3, [label='Min Interval', units=ms, minValue='0', maxValue='10000', increment='1', default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_OBLIQUE_SURVEY', 4, [label='Positions', minValue='2', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_OBLIQUE_SURVEY', 5, [label='Roll Angle', units=deg, minValue='0', default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_OBLIQUE_SURVEY', 6, [label='Pitch Angle', units=deg, minValue='-180', maxValue='180', default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_OBLIQUE_SURVEY', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_MISSION_START', 1, [label='First Item', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_MISSION_START', 2, [label='Last Item', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_ACTUATOR_TEST', 1, [label='Value', minValue='-1', maxValue='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_ACTUATOR_TEST', 2, [label='Timeout', units=s, minValue='0', maxValue='3']).
enum_entry_param('MAV_CMD', 'MAV_CMD_ACTUATOR_TEST', 3, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_ACTUATOR_TEST', 4, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_ACTUATOR_TEST', 5, [label='Output Function', enum='ACTUATOR_OUTPUT_FUNCTION']).
enum_entry_param('MAV_CMD', 'MAV_CMD_ACTUATOR_TEST', 6, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_ACTUATOR_TEST', 7, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONFIGURE_ACTUATOR', 1, [label='Configuration', enum='ACTUATOR_CONFIGURATION']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONFIGURE_ACTUATOR', 2, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONFIGURE_ACTUATOR', 3, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONFIGURE_ACTUATOR', 4, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONFIGURE_ACTUATOR', 5, [label='Output Function', enum='ACTUATOR_OUTPUT_FUNCTION']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONFIGURE_ACTUATOR', 6, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONFIGURE_ACTUATOR', 7, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_COMPONENT_ARM_DISARM', 1, [label='Arm', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_COMPONENT_ARM_DISARM', 2, [label='Force', minValue='0', maxValue='21196', increment='21196']).
enum_entry_param('MAV_CMD', 'MAV_CMD_ILLUMINATOR_ON_OFF', 1, [label='Enable', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_GET_HOME_POSITION', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GET_HOME_POSITION', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GET_HOME_POSITION', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GET_HOME_POSITION', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GET_HOME_POSITION', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GET_HOME_POSITION', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GET_HOME_POSITION', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_INJECT_FAILURE', 1, [label='Failure unit', enum='FAILURE_UNIT']).
enum_entry_param('MAV_CMD', 'MAV_CMD_INJECT_FAILURE', 2, [label='Failure type', enum='FAILURE_TYPE']).
enum_entry_param('MAV_CMD', 'MAV_CMD_INJECT_FAILURE', 3, [label='Instance']).
enum_entry_param('MAV_CMD', 'MAV_CMD_START_RX_PAIR', 1, [label='Spektrum']).
enum_entry_param('MAV_CMD', 'MAV_CMD_START_RX_PAIR', 2, [label='RC Type', enum='RC_TYPE']).
enum_entry_param('MAV_CMD', 'MAV_CMD_GET_MESSAGE_INTERVAL', 1, [label='Message ID', minValue='0', maxValue='16777215', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_MESSAGE_INTERVAL', 1, [label='Message ID', minValue='0', maxValue='16777215', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_MESSAGE_INTERVAL', 2, [label='Interval', units=us, minValue='-1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_MESSAGE_INTERVAL', 7, [label='Response Target', minValue='0', maxValue='2', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_MESSAGE', 1, [label='Message ID', minValue='0', maxValue='16777215', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_MESSAGE', 2, [label='Req Param 1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_MESSAGE', 3, [label='Req Param 2']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_MESSAGE', 4, [label='Req Param 3']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_MESSAGE', 5, [label='Req Param 4']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_MESSAGE', 6, [label='Req Param 5']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_MESSAGE', 7, [label='Response Target', minValue='0', maxValue='2', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_PROTOCOL_VERSION', 1, [label='Protocol', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_PROTOCOL_VERSION', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES', 1, [label='Version', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_INFORMATION', 1, [label='Capabilities', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_INFORMATION', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_SETTINGS', 1, [label='Settings', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_SETTINGS', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_STORAGE_INFORMATION', 1, [label='Storage ID', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_STORAGE_INFORMATION', 2, [label='Information', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_STORAGE_INFORMATION', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_STORAGE_FORMAT', 1, [label='Storage ID', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_STORAGE_FORMAT', 2, [label='Format', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_STORAGE_FORMAT', 3, [label='Reset Image Log', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_STORAGE_FORMAT', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS', 1, [label='Capture Status', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_FLIGHT_INFORMATION', 1, [label='Flight Information', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_FLIGHT_INFORMATION', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_RESET_CAMERA_SETTINGS', 1, [label='Reset', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_RESET_CAMERA_SETTINGS', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_CAMERA_MODE', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_CAMERA_MODE', 2, [label='Camera Mode', enum='CAMERA_MODE']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_CAMERA_MODE', 3, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_CAMERA_MODE', 4, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_CAMERA_MODE', 7, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_CAMERA_ZOOM', 1, [label='Zoom Type', enum='CAMERA_ZOOM_TYPE']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_CAMERA_ZOOM', 2, [label='Zoom Value']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_CAMERA_ZOOM', 3, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_CAMERA_ZOOM', 4, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_CAMERA_ZOOM', 7, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_CAMERA_FOCUS', 1, [label='Focus Type', enum='SET_FOCUS_TYPE']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_CAMERA_FOCUS', 2, [label='Focus Value']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_CAMERA_FOCUS', 3, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_CAMERA_FOCUS', 4, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_CAMERA_FOCUS', 7, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_STORAGE_USAGE', 1, [label='Storage ID', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_STORAGE_USAGE', 2, [label='Usage', enum='STORAGE_USAGE_FLAG']).
enum_entry_param('MAV_CMD', 'MAV_CMD_JUMP_TAG', 1, [label='Tag', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_JUMP_TAG', 1, [label='Tag', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_JUMP_TAG', 2, [label='Repeat', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW', 1, [label='Pitch angle', units=deg, minValue='-180', maxValue='180']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW', 2, [label='Yaw angle', units=deg, minValue='-180', maxValue='180']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW', 3, [label='Pitch rate', units='deg/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW', 4, [label='Yaw rate', units='deg/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW', 5, [label='Gimbal manager flags', enum='GIMBAL_MANAGER_FLAGS']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW', 7, [label='Gimbal device ID']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE', 1, [label='sysid primary control']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE', 2, [label='compid primary control']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE', 3, [label='sysid secondary control']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE', 4, [label='compid secondary control']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE', 7, [label='Gimbal device ID']).
enum_entry_param('MAV_CMD', 'MAV_CMD_IMAGE_START_CAPTURE', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_IMAGE_START_CAPTURE', 2, [label='Interval', units=s, minValue='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_IMAGE_START_CAPTURE', 3, [label='Total Images', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_IMAGE_START_CAPTURE', 4, [label='Sequence Number', minValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_IMAGE_START_CAPTURE', 5, [reserved=true]).
enum_entry_param('MAV_CMD', 'MAV_CMD_IMAGE_START_CAPTURE', 6, [reserved=true]).
enum_entry_param('MAV_CMD', 'MAV_CMD_IMAGE_START_CAPTURE', 7, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_IMAGE_STOP_CAPTURE', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_IMAGE_STOP_CAPTURE', 2, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_IMAGE_STOP_CAPTURE', 3, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_IMAGE_STOP_CAPTURE', 4, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_IMAGE_STOP_CAPTURE', 5, [reserved=true]).
enum_entry_param('MAV_CMD', 'MAV_CMD_IMAGE_STOP_CAPTURE', 6, [reserved=true]).
enum_entry_param('MAV_CMD', 'MAV_CMD_IMAGE_STOP_CAPTURE', 7, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE', 1, [label='Number', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE', 2, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE', 3, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE', 4, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE', 5, [reserved=true]).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE', 6, [reserved=true]).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE', 7, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_TRIGGER_CONTROL', 1, [label='Enable', minValue='-1', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_TRIGGER_CONTROL', 2, [label='Reset', minValue='-1', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_TRIGGER_CONTROL', 3, [label='Pause', minValue='-1', maxValue='1', increment='2']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CAMERA_TRACK_POINT', 1, [label='Point x', minValue='0', maxValue='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CAMERA_TRACK_POINT', 2, [label='Point y', minValue='0', maxValue='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CAMERA_TRACK_POINT', 3, [label='Radius', minValue='0', maxValue='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CAMERA_TRACK_RECTANGLE', 1, [label='Top left corner x', minValue='0', maxValue='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CAMERA_TRACK_RECTANGLE', 2, [label='Top left corner y', minValue='0', maxValue='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CAMERA_TRACK_RECTANGLE', 3, [label='Bottom right corner x', minValue='0', maxValue='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CAMERA_TRACK_RECTANGLE', 4, [label='Bottom right corner y', minValue='0', maxValue='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_VIDEO_START_CAPTURE', 1, [label='Stream ID', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_VIDEO_START_CAPTURE', 2, [label='Status Frequency', minValue='0', units='Hz']).
enum_entry_param('MAV_CMD', 'MAV_CMD_VIDEO_START_CAPTURE', 3, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_VIDEO_START_CAPTURE', 4, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_VIDEO_START_CAPTURE', 5, [reserved=true]).
enum_entry_param('MAV_CMD', 'MAV_CMD_VIDEO_START_CAPTURE', 6, [reserved=true]).
enum_entry_param('MAV_CMD', 'MAV_CMD_VIDEO_START_CAPTURE', 7, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_VIDEO_STOP_CAPTURE', 1, [label='Stream ID', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_VIDEO_STOP_CAPTURE', 2, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_VIDEO_STOP_CAPTURE', 3, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_VIDEO_STOP_CAPTURE', 4, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_VIDEO_STOP_CAPTURE', 5, [reserved=true]).
enum_entry_param('MAV_CMD', 'MAV_CMD_VIDEO_STOP_CAPTURE', 6, [reserved=true]).
enum_entry_param('MAV_CMD', 'MAV_CMD_VIDEO_STOP_CAPTURE', 7, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_VIDEO_START_STREAMING', 1, [label='Stream ID', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_VIDEO_STOP_STREAMING', 1, [label='Stream ID', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION', 1, [label='Stream ID', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_REQUEST_VIDEO_STREAM_STATUS', 1, [label='Stream ID', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_LOGGING_START', 1, [label='Format', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_LOGGING_START', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_LOGGING_START', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_LOGGING_START', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_LOGGING_START', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_LOGGING_START', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_LOGGING_START', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_LOGGING_STOP', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_LOGGING_STOP', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_LOGGING_STOP', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_LOGGING_STOP', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_LOGGING_STOP', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_LOGGING_STOP', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_LOGGING_STOP', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_AIRFRAME_CONFIGURATION', 1, [label='Landing Gear ID', minValue='-1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_AIRFRAME_CONFIGURATION', 2, [label='Landing Gear Position']).
enum_entry_param('MAV_CMD', 'MAV_CMD_AIRFRAME_CONFIGURATION', 3, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_AIRFRAME_CONFIGURATION', 4, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_AIRFRAME_CONFIGURATION', 5, [reserved=true]).
enum_entry_param('MAV_CMD', 'MAV_CMD_AIRFRAME_CONFIGURATION', 6, [reserved=true]).
enum_entry_param('MAV_CMD', 'MAV_CMD_AIRFRAME_CONFIGURATION', 7, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONTROL_HIGH_LATENCY', 1, [label='Enable', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONTROL_HIGH_LATENCY', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONTROL_HIGH_LATENCY', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONTROL_HIGH_LATENCY', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONTROL_HIGH_LATENCY', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONTROL_HIGH_LATENCY', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONTROL_HIGH_LATENCY', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PANORAMA_CREATE', 1, [label='Horizontal Angle', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_PANORAMA_CREATE', 2, [label='Vertical Angle', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_PANORAMA_CREATE', 3, [label='Horizontal Speed', units='deg/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PANORAMA_CREATE', 4, [label='Vertical Speed', units='deg/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_VTOL_TRANSITION', 1, [label='State', enum='MAV_VTOL_STATE']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_VTOL_TRANSITION', 2, [label='Immediate']).
enum_entry_param('MAV_CMD', 'MAV_CMD_ARM_AUTHORIZATION_REQUEST', 1, [label='System ID', minValue='0', maxValue='255', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE', 1, [label='Radius', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE', 5, [label='Latitude', units=degE7]).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE', 6, [label='Longitude', units=degE7]).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_GATE', 1, [label='Geometry', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_GATE', 2, [label='UseAltitude', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_GATE', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_GATE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_GATE', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_GATE', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_CONDITION_GATE', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_RETURN_POINT', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_RETURN_POINT', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_RETURN_POINT', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_RETURN_POINT', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_RETURN_POINT', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_RETURN_POINT', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_RETURN_POINT', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION', 1, [label='Vertex Count', minValue='3', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION', 2, [label='Inclusion Group', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION', 1, [label='Vertex Count', minValue='3', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION', 1, [label='Radius', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION', 2, [label='Inclusion Group', minValue='0', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION', 1, [label='Radius', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_RALLY_POINT', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_RALLY_POINT', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_RALLY_POINT', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_RALLY_POINT', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_RALLY_POINT', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_RALLY_POINT', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_NAV_RALLY_POINT', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_UAVCAN_GET_NODE_INFO', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_UAVCAN_GET_NODE_INFO', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_UAVCAN_GET_NODE_INFO', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_UAVCAN_GET_NODE_INFO', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_UAVCAN_GET_NODE_INFO', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_UAVCAN_GET_NODE_INFO', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_UAVCAN_GET_NODE_INFO', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ADSB_OUT_IDENT', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ADSB_OUT_IDENT', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ADSB_OUT_IDENT', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ADSB_OUT_IDENT', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ADSB_OUT_IDENT', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ADSB_OUT_IDENT', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_ADSB_OUT_IDENT', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_PREPARE_DEPLOY', 1, [label='Operation Mode', minValue='0', maxValue='2', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_PREPARE_DEPLOY', 2, [label='Approach Vector', units=deg, minValue='-1', maxValue='360']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_PREPARE_DEPLOY', 3, [label='Ground Speed', minValue='-1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_PREPARE_DEPLOY', 4, [label='Altitude Clearance', units=m, minValue='-1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_PREPARE_DEPLOY', 5, [label='Latitude', units=degE7]).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_PREPARE_DEPLOY', 6, [label='Longitude', units=degE7]).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_PREPARE_DEPLOY', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL_DEPLOY', 1, [label='Operation Mode', minValue='0', maxValue='101', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL_DEPLOY', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL_DEPLOY', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL_DEPLOY', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL_DEPLOY', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL_DEPLOY', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PAYLOAD_CONTROL_DEPLOY', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_YAW', 1, [label='Yaw', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_YAW', 2, [label='CompassMask']).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_YAW', 3, [label='Latitude', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_YAW', 4, [label='Longitude', units=deg]).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_YAW', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_YAW', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_FIXED_MAG_CAL_YAW', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_WINCH', 1, [label='Instance', minValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_WINCH', 2, [label='Action', enum='WINCH_ACTIONS']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_WINCH', 3, [label='Length', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_WINCH', 4, [label='Rate', units='m/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_WINCH', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_WINCH', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_WINCH', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_1', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_1', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_1', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_1', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_1', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_1', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_1', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_2', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_2', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_2', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_2', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_2', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_2', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_2', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_3', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_3', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_3', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_3', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_3', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_3', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_3', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_4', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_4', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_4', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_4', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_4', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_4', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_4', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_5', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_5', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_5', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_5', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_5', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_5', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_WAYPOINT_USER_5', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_1', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_1', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_1', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_1', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_1', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_1', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_1', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_2', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_2', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_2', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_2', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_2', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_2', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_2', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_3', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_3', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_3', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_3', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_3', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_3', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_3', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_4', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_4', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_4', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_4', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_4', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_4', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_4', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_5', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_5', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_5', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_5', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_5', 5, [label='Latitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_5', 6, [label='Longitude']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SPATIAL_USER_5', 7, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_1', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_1', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_1', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_1', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_1', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_1', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_1', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_2', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_2', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_2', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_2', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_2', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_2', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_2', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_3', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_3', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_3', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_3', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_3', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_3', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_3', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_4', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_4', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_4', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_4', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_4', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_4', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_4', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_5', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_5', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_5', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_5', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_5', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_5', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_USER_5', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CAN_FORWARD', 1, [label=bus]).
enum_entry_param('MAV_CMD', 'MAV_CMD_CAN_FORWARD', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CAN_FORWARD', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CAN_FORWARD', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CAN_FORWARD', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CAN_FORWARD', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_CAN_FORWARD', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FIGURE_EIGHT', 1, [label='Major Radius', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FIGURE_EIGHT', 2, [label='Minor Radius', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FIGURE_EIGHT', 3, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FIGURE_EIGHT', 4, [label='Orientation', units=rad]).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FIGURE_EIGHT', 5, [label='Latitude/X']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FIGURE_EIGHT', 6, [label='Longitude/Y']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_FIGURE_EIGHT', 7, [label='Altitude/Z']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PARAM_TRANSACTION', 1, [label='Action', enum='PARAM_TRANSACTION_ACTION']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PARAM_TRANSACTION', 2, [label='Transport', enum='PARAM_TRANSACTION_TRANSPORT']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PARAM_TRANSACTION', 3, [label='Transaction ID']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_FENCE_BREACH_ACTION', 1, [label='Action', enum='FENCE_ACTION']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_UPGRADE', 1, [label='Component ID', enum='MAV_COMPONENT']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_UPGRADE', 2, [label='Reboot', minValue='0', maxValue='1', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_UPGRADE', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_UPGRADE', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_UPGRADE', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_UPGRADE', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_UPGRADE', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_GROUP_START', 1, [label='Group ID', minValue='0', maxValue='16777216', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_GROUP_END', 1, [label='Group ID', minValue='0', maxValue='16777216', increment='1']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_STANDARD_MODE', 1, [label='Standard Mode', enum='MAV_STANDARD_MODE']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_STANDARD_MODE', 2, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_STANDARD_MODE', 3, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_STANDARD_MODE', 4, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_STANDARD_MODE', 5, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_STANDARD_MODE', 6, [reserved=true, default='0']).
enum_entry_param('MAV_CMD', 'MAV_CMD_DO_SET_STANDARD_MODE', 7, [reserved=true, default='NaN']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_AT_S_PARAM', 1, [label='Radio instance']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_AT_S_PARAM', 2, [label='Index']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_AT_S_PARAM', 3, [label='Value']).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_AT_S_PARAM', 4, [reserved=true]).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_AT_S_PARAM', 5, [reserved=true]).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_AT_S_PARAM', 6, [reserved=true]).
enum_entry_param('MAV_CMD', 'MAV_CMD_SET_AT_S_PARAM', 7, [reserved=true]).
enum_entry_param('MAV_CMD', 'MAV_CMD_STORM32_DO_GIMBAL_MANAGER_CONTROL_PITCHYAW', 1, [label='Pitch angle', units=deg, minValue='-180', maxValue='180']).
enum_entry_param('MAV_CMD', 'MAV_CMD_STORM32_DO_GIMBAL_MANAGER_CONTROL_PITCHYAW', 2, [label='Yaw angle', units=deg, minValue='-180', maxValue='180']).
enum_entry_param('MAV_CMD', 'MAV_CMD_STORM32_DO_GIMBAL_MANAGER_CONTROL_PITCHYAW', 3, [label='Pitch rate', units='deg/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_STORM32_DO_GIMBAL_MANAGER_CONTROL_PITCHYAW', 4, [label='Yaw rate', units='deg/s']).
enum_entry_param('MAV_CMD', 'MAV_CMD_STORM32_DO_GIMBAL_MANAGER_CONTROL_PITCHYAW', 5, [label='Gimbal device flags', enum='GIMBAL_DEVICE_FLAGS']).
enum_entry_param('MAV_CMD', 'MAV_CMD_STORM32_DO_GIMBAL_MANAGER_CONTROL_PITCHYAW', 6, [label='Gimbal manager flags', enum='MAV_STORM32_GIMBAL_MANAGER_FLAGS']).
enum_entry_param('MAV_CMD', 'MAV_CMD_STORM32_DO_GIMBAL_MANAGER_CONTROL_PITCHYAW', 7, [label='Gimbal ID and client']).
enum_entry_param('MAV_CMD', 'MAV_CMD_STORM32_DO_GIMBAL_MANAGER_SETUP', 1, [label='Profile', enum='MAV_STORM32_GIMBAL_MANAGER_PROFILE']).
enum_entry_param('MAV_CMD', 'MAV_CMD_STORM32_DO_GIMBAL_MANAGER_SETUP', 7, [label='Gimbal ID']).
enum_entry_param('MAV_CMD', 'MAV_CMD_QSHOT_DO_CONFIGURE', 1, [label='Mode', enum='MAV_QSHOT_MODE']).
enum_entry_param('MAV_CMD', 'MAV_CMD_QSHOT_DO_CONFIGURE', 2, [label='Shot state or command']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SET_ARM', 1, [label='ARM status']).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SET_ARM', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SET_ARM', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SET_ARM', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SET_ARM', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SET_ARM', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SET_ARM', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ARM', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ARM', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ARM', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ARM', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ARM', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ARM', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ARM', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_BATTERY', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_BATTERY', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_BATTERY', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_BATTERY', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_BATTERY', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_BATTERY', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_BATTERY', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ERR', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ERR', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ERR', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ERR', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ERR', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ERR', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ERR', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SET_ARM_ALTI', 1, [label='Altitude', units=m]).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SET_ARM_ALTI', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SET_ARM_ALTI', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SET_ARM_ALTI', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SET_ARM_ALTI', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SET_ARM_ALTI', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SET_ARM_ALTI', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ARM_ALTI', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ARM_ALTI', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ARM_ALTI', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ARM_ALTI', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ARM_ALTI', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ARM_ALTI', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_GET_ARM_ALTI', 7, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SHUTDOWN', 1, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SHUTDOWN', 2, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SHUTDOWN', 3, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SHUTDOWN', 4, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SHUTDOWN', 5, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SHUTDOWN', 6, []).
enum_entry_param('MAV_CMD', 'MAV_CMD_PRS_SHUTDOWN', 7, []).

:- dynamic enum_include/2.

enum_include('ACCELCAL_VEHICLE_POS', ardupilotmega).
enum_include('HEADING_TYPE', ardupilotmega).
enum_include('SPEED_TYPE', ardupilotmega).
enum_include('MAV_CMD', ardupilotmega).
enum_include('SCRIPTING_CMD', ardupilotmega).
enum_include('LIMITS_STATE', ardupilotmega).
enum_include('LIMIT_MODULE', ardupilotmega).
enum_include('RALLY_FLAGS', ardupilotmega).
enum_include('CAMERA_STATUS_TYPES', ardupilotmega).
enum_include('CAMERA_FEEDBACK_FLAGS', ardupilotmega).
enum_include('MAV_MODE_GIMBAL', ardupilotmega).
enum_include('GIMBAL_AXIS', ardupilotmega).
enum_include('GIMBAL_AXIS_CALIBRATION_STATUS', ardupilotmega).
enum_include('GIMBAL_AXIS_CALIBRATION_REQUIRED', ardupilotmega).
enum_include('GOPRO_HEARTBEAT_STATUS', ardupilotmega).
enum_include('GOPRO_HEARTBEAT_FLAGS', ardupilotmega).
enum_include('GOPRO_REQUEST_STATUS', ardupilotmega).
enum_include('GOPRO_COMMAND', ardupilotmega).
enum_include('GOPRO_CAPTURE_MODE', ardupilotmega).
enum_include('GOPRO_RESOLUTION', ardupilotmega).
enum_include('GOPRO_FRAME_RATE', ardupilotmega).
enum_include('GOPRO_FIELD_OF_VIEW', ardupilotmega).
enum_include('GOPRO_VIDEO_SETTINGS_FLAGS', ardupilotmega).
enum_include('GOPRO_PHOTO_RESOLUTION', ardupilotmega).
enum_include('GOPRO_PROTUNE_WHITE_BALANCE', ardupilotmega).
enum_include('GOPRO_PROTUNE_COLOUR', ardupilotmega).
enum_include('GOPRO_PROTUNE_GAIN', ardupilotmega).
enum_include('GOPRO_PROTUNE_SHARPNESS', ardupilotmega).
enum_include('GOPRO_PROTUNE_EXPOSURE', ardupilotmega).
enum_include('GOPRO_CHARGING', ardupilotmega).
enum_include('GOPRO_MODEL', ardupilotmega).
enum_include('GOPRO_BURST_RATE', ardupilotmega).
enum_include('MAV_CMD_DO_AUX_FUNCTION_SWITCH_LEVEL', ardupilotmega).
enum_include('LED_CONTROL_PATTERN', ardupilotmega).
enum_include('EKF_STATUS_FLAGS', ardupilotmega).
enum_include('PID_TUNING_AXIS', ardupilotmega).
enum_include('MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS', ardupilotmega).
enum_include('MAV_REMOTE_LOG_DATA_BLOCK_STATUSES', ardupilotmega).
enum_include('DEVICE_OP_BUSTYPE', ardupilotmega).
enum_include('DEEPSTALL_STAGE', ardupilotmega).
enum_include('PLANE_MODE', ardupilotmega).
enum_include('COPTER_MODE', ardupilotmega).
enum_include('SUB_MODE', ardupilotmega).
enum_include('ROVER_MODE', ardupilotmega).
enum_include('TRACKER_MODE', ardupilotmega).
enum_include('OSD_PARAM_CONFIG_TYPE', ardupilotmega).
enum_include('OSD_PARAM_CONFIG_ERROR', ardupilotmega).
enum_include('MAV_CMD', 'ASLUAV').
enum_include('GSM_LINK_TYPE', 'ASLUAV').
enum_include('GSM_MODEM_TYPE', 'ASLUAV').
enum_include('FIRMWARE_VERSION_TYPE', common).
enum_include('HL_FAILURE_FLAG', common).
enum_include('MAV_GOTO', common).
enum_include('MAV_MODE', common).
enum_include('MAV_SYS_STATUS_SENSOR', common).
enum_include('MAV_SYS_STATUS_SENSOR_EXTENDED', common).
enum_include('MAV_FRAME', common).
enum_include('MAVLINK_DATA_STREAM_TYPE', common).
enum_include('FENCE_ACTION', common).
enum_include('FENCE_BREACH', common).
enum_include('FENCE_MITIGATE', common).
enum_include('MAV_MOUNT_MODE', common).
enum_include('GIMBAL_DEVICE_CAP_FLAGS', common).
enum_include('GIMBAL_MANAGER_CAP_FLAGS', common).
enum_include('GIMBAL_DEVICE_FLAGS', common).
enum_include('GIMBAL_MANAGER_FLAGS', common).
enum_include('GIMBAL_DEVICE_ERROR_FLAGS', common).
enum_include('GRIPPER_ACTIONS', common).
enum_include('WINCH_ACTIONS', common).
enum_include('UAVCAN_NODE_HEALTH', common).
enum_include('UAVCAN_NODE_MODE', common).
enum_include('ESC_CONNECTION_TYPE', common).
enum_include('ESC_FAILURE_FLAGS', common).
enum_include('STORAGE_STATUS', common).
enum_include('STORAGE_TYPE', common).
enum_include('STORAGE_USAGE_FLAG', common).
enum_include('ORBIT_YAW_BEHAVIOUR', common).
enum_include('WIFI_CONFIG_AP_RESPONSE', common).
enum_include('CELLULAR_CONFIG_RESPONSE', common).
enum_include('WIFI_CONFIG_AP_MODE', common).
enum_include('COMP_METADATA_TYPE', common).
enum_include('ACTUATOR_CONFIGURATION', common).
enum_include('ACTUATOR_OUTPUT_FUNCTION', common).
enum_include('AUTOTUNE_AXIS', common).
enum_include('PREFLIGHT_STORAGE_PARAMETER_ACTION', common).
enum_include('PREFLIGHT_STORAGE_MISSION_ACTION', common).
enum_include('MAV_CMD', common).
enum_include('MAV_DATA_STREAM', common).
enum_include('MAV_ROI', common).
enum_include('MAV_PARAM_TYPE', common).
enum_include('MAV_PARAM_EXT_TYPE', common).
enum_include('MAV_RESULT', common).
enum_include('MAV_MISSION_RESULT', common).
enum_include('MAV_SEVERITY', common).
enum_include('MAV_POWER_STATUS', common).
enum_include('SERIAL_CONTROL_DEV', common).
enum_include('SERIAL_CONTROL_FLAG', common).
enum_include('MAV_DISTANCE_SENSOR', common).
enum_include('MAV_SENSOR_ORIENTATION', common).
enum_include('MAV_PROTOCOL_CAPABILITY', common).
enum_include('MAV_MISSION_TYPE', common).
enum_include('MAV_ESTIMATOR_TYPE', common).
enum_include('MAV_BATTERY_TYPE', common).
enum_include('MAV_BATTERY_FUNCTION', common).
enum_include('MAV_BATTERY_CHARGE_STATE', common).
enum_include('MAV_BATTERY_MODE', common).
enum_include('MAV_BATTERY_FAULT', common).
enum_include('MAV_GENERATOR_STATUS_FLAG', common).
enum_include('MAV_VTOL_STATE', common).
enum_include('MAV_LANDED_STATE', common).
enum_include('ADSB_ALTITUDE_TYPE', common).
enum_include('ADSB_EMITTER_TYPE', common).
enum_include('ADSB_FLAGS', common).
enum_include('MAV_DO_REPOSITION_FLAGS', common).
enum_include('ESTIMATOR_STATUS_FLAGS', common).
enum_include('MOTOR_TEST_ORDER', common).
enum_include('MOTOR_TEST_THROTTLE_TYPE', common).
enum_include('GPS_INPUT_IGNORE_FLAGS', common).
enum_include('MAV_COLLISION_ACTION', common).
enum_include('MAV_COLLISION_THREAT_LEVEL', common).
enum_include('MAV_COLLISION_SRC', common).
enum_include('GPS_FIX_TYPE', common).
enum_include('RTK_BASELINE_COORDINATE_SYSTEM', common).
enum_include('LANDING_TARGET_TYPE', common).
enum_include('VTOL_TRANSITION_HEADING', common).
enum_include('CAMERA_CAP_FLAGS', common).
enum_include('VIDEO_STREAM_STATUS_FLAGS', common).
enum_include('VIDEO_STREAM_TYPE', common).
enum_include('CAMERA_TRACKING_STATUS_FLAGS', common).
enum_include('CAMERA_TRACKING_MODE', common).
enum_include('CAMERA_TRACKING_TARGET_DATA', common).
enum_include('CAMERA_ZOOM_TYPE', common).
enum_include('SET_FOCUS_TYPE', common).
enum_include('PARAM_ACK', common).
enum_include('CAMERA_MODE', common).
enum_include('MAV_ARM_AUTH_DENIED_REASON', common).
enum_include('RC_TYPE', common).
enum_include('POSITION_TARGET_TYPEMASK', common).
enum_include('ATTITUDE_TARGET_TYPEMASK', common).
enum_include('UTM_FLIGHT_STATE', common).
enum_include('UTM_DATA_AVAIL_FLAGS', common).
enum_include('CELLULAR_STATUS_FLAG', common).
enum_include('CELLULAR_NETWORK_FAILED_REASON', common).
enum_include('CELLULAR_NETWORK_RADIO_TYPE', common).
enum_include('PRECISION_LAND_MODE', common).
enum_include('PARACHUTE_ACTION', common).
enum_include('MAV_TUNNEL_PAYLOAD_TYPE', common).
enum_include('MAV_ODID_ID_TYPE', common).
enum_include('MAV_ODID_UA_TYPE', common).
enum_include('MAV_ODID_STATUS', common).
enum_include('MAV_ODID_HEIGHT_REF', common).
enum_include('MAV_ODID_HOR_ACC', common).
enum_include('MAV_ODID_VER_ACC', common).
enum_include('MAV_ODID_SPEED_ACC', common).
enum_include('MAV_ODID_TIME_ACC', common).
enum_include('MAV_ODID_AUTH_TYPE', common).
enum_include('MAV_ODID_DESC_TYPE', common).
enum_include('MAV_ODID_OPERATOR_LOCATION_TYPE', common).
enum_include('MAV_ODID_CLASSIFICATION_TYPE', common).
enum_include('MAV_ODID_CATEGORY_EU', common).
enum_include('MAV_ODID_CLASS_EU', common).
enum_include('MAV_ODID_OPERATOR_ID_TYPE', common).
enum_include('MAV_ODID_ARM_STATUS', common).
enum_include('TUNE_FORMAT', common).
enum_include('AIS_TYPE', common).
enum_include('AIS_NAV_STATUS', common).
enum_include('AIS_FLAGS', common).
enum_include('FAILURE_UNIT', common).
enum_include('FAILURE_TYPE', common).
enum_include('NAV_VTOL_LAND_OPTIONS', common).
enum_include('MAV_WINCH_STATUS_FLAG', common).
enum_include('MAG_CAL_STATUS', common).
enum_include('MAV_EVENT_ERROR_REASON', common).
enum_include('MAV_EVENT_CURRENT_SEQUENCE_FLAGS', common).
enum_include('HIL_SENSOR_UPDATED_FLAGS', common).
enum_include('HIGHRES_IMU_UPDATED_FLAGS', common).
enum_include('CAN_FILTER_OP', common).
enum_include('MAV_FTP_ERR', common).
enum_include('MAV_FTP_OPCODE', common).
enum_include('MISSION_STATE', common).
enum_include('WIFI_NETWORK_SECURITY', development).
enum_include('AIRSPEED_SENSOR_FLAGS', development).
enum_include('PARAM_TRANSACTION_TRANSPORT', development).
enum_include('PARAM_TRANSACTION_ACTION', development).
enum_include('MAV_STANDARD_MODE', development).
enum_include('MAV_MODE_PROPERTY', development).
enum_include('MAV_CMD', development).
enum_include('MAV_BATTERY_STATUS_FLAGS', development).
enum_include('MAV_CMD', development).
enum_include('TARGET_ABSOLUTE_SENSOR_CAPABILITY_FLAGS', development).
enum_include('TARGET_OBS_FRAME', development).
enum_include('ICAROUS_TRACK_BAND_TYPES', icarous).
enum_include('ICAROUS_FMS_STATE', icarous).
enum_include('MAV_AUTOPILOT', minimal).
enum_include('MAV_TYPE', minimal).
enum_include('MAV_MODE_FLAG', minimal).
enum_include('MAV_MODE_FLAG_DECODE_POSITION', minimal).
enum_include('MAV_STATE', minimal).
enum_include('MAV_COMPONENT', minimal).
enum_include('UALBERTA_AUTOPILOT_MODE', ualberta).
enum_include('UALBERTA_NAV_MODE', ualberta).
enum_include('UALBERTA_PILOT_MODE', ualberta).
enum_include('UAVIONIX_ADSB_OUT_DYNAMIC_STATE', uAvionix).
enum_include('UAVIONIX_ADSB_OUT_RF_SELECT', uAvionix).
enum_include('UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX', uAvionix).
enum_include('UAVIONIX_ADSB_RF_HEALTH', uAvionix).
enum_include('UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE', uAvionix).
enum_include('UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT', uAvionix).
enum_include('UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON', uAvionix).
enum_include('UAVIONIX_ADSB_EMERGENCY_STATUS', uAvionix).
enum_include('MAV_STORM32_TUNNEL_PAYLOAD_TYPE', storm32).
enum_include('MAV_STORM32_GIMBAL_PREARM_FLAGS', storm32).
enum_include('MAV_STORM32_CAMERA_PREARM_FLAGS', storm32).
enum_include('MAV_STORM32_GIMBAL_MANAGER_CAP_FLAGS', storm32).
enum_include('MAV_STORM32_GIMBAL_MANAGER_FLAGS', storm32).
enum_include('MAV_STORM32_GIMBAL_MANAGER_CLIENT', storm32).
enum_include('MAV_STORM32_GIMBAL_MANAGER_PROFILE', storm32).
enum_include('MAV_QSHOT_MODE', storm32).
enum_include('MAV_CMD', storm32).
enum_include('RADIO_RC_CHANNELS_FLAGS', storm32).
enum_include('RADIO_LINK_STATS_FLAGS', storm32).
enum_include('MAV_CMD', 'AVSSUAS').
enum_include('MAV_AVSS_COMMAND_FAILURE_REASON', 'AVSSUAS').
enum_include('AVSS_M300_OPERATION_MODE', 'AVSSUAS').
enum_include('AVSS_HORSEFLY_OPERATION_MODE', 'AVSSUAS').
enum_include('AIRLINK_AUTH_RESPONSE_TYPE', csAirLink).

:- dynamic message/3.

message('SENSOR_OFFSETS', 150, []).
message('SET_MAG_OFFSETS', 151, []).
message('MEMINFO', 152, []).
message('AP_ADC', 153, []).
message('DIGICAM_CONFIGURE', 154, []).
message('DIGICAM_CONTROL', 155, []).
message('MOUNT_CONFIGURE', 156, []).
message('MOUNT_CONTROL', 157, []).
message('MOUNT_STATUS', 158, []).
message('FENCE_POINT', 160, []).
message('FENCE_FETCH_POINT', 161, []).
message('AHRS', 163, []).
message('SIMSTATE', 164, []).
message('HWSTATUS', 165, []).
message('RADIO', 166, []).
message('LIMITS_STATUS', 167, []).
message('WIND', 168, []).
message('DATA16', 169, []).
message('DATA32', 170, []).
message('DATA64', 171, []).
message('DATA96', 172, []).
message('RANGEFINDER', 173, []).
message('AIRSPEED_AUTOCAL', 174, []).
message('RALLY_POINT', 175, []).
message('RALLY_FETCH_POINT', 176, []).
message('COMPASSMOT_STATUS', 177, []).
message('AHRS2', 178, []).
message('CAMERA_STATUS', 179, []).
message('CAMERA_FEEDBACK', 180, []).
message('BATTERY2', 181, []).
message('AHRS3', 182, []).
message('AUTOPILOT_VERSION_REQUEST', 183, []).
message('REMOTE_LOG_DATA_BLOCK', 184, []).
message('REMOTE_LOG_BLOCK_STATUS', 185, []).
message('LED_CONTROL', 186, []).
message('MAG_CAL_PROGRESS', 191, []).
message('EKF_STATUS_REPORT', 193, []).
message('PID_TUNING', 194, []).
message('DEEPSTALL', 195, []).
message('GIMBAL_REPORT', 200, []).
message('GIMBAL_CONTROL', 201, []).
message('GIMBAL_TORQUE_CMD_REPORT', 214, []).
message('GOPRO_HEARTBEAT', 215, []).
message('GOPRO_GET_REQUEST', 216, []).
message('GOPRO_GET_RESPONSE', 217, []).
message('GOPRO_SET_REQUEST', 218, []).
message('GOPRO_SET_RESPONSE', 219, []).
message('RPM', 226, []).
message('DEVICE_OP_READ', 11000, []).
message('DEVICE_OP_READ_REPLY', 11001, []).
message('DEVICE_OP_WRITE', 11002, []).
message('DEVICE_OP_WRITE_REPLY', 11003, []).
message('ADAP_TUNING', 11010, []).
message('VISION_POSITION_DELTA', 11011, []).
message('AOA_SSA', 11020, []).
message('ESC_TELEMETRY_1_TO_4', 11030, []).
message('ESC_TELEMETRY_5_TO_8', 11031, []).
message('ESC_TELEMETRY_9_TO_12', 11032, []).
message('OSD_PARAM_CONFIG', 11033, []).
message('OSD_PARAM_CONFIG_REPLY', 11034, []).
message('OSD_PARAM_SHOW_CONFIG', 11035, []).
message('OSD_PARAM_SHOW_CONFIG_REPLY', 11036, []).
message('OBSTACLE_DISTANCE_3D', 11037, []).
message('WATER_DEPTH', 11038, []).
message('MCU_STATUS', 11039, []).
message('COMMAND_INT_STAMPED', 223, []).
message('COMMAND_LONG_STAMPED', 224, []).
message('SENS_POWER', 8002, []).
message('SENS_MPPT', 8003, []).
message('ASLCTRL_DATA', 8004, []).
message('ASLCTRL_DEBUG', 8005, []).
message('ASLUAV_STATUS', 8006, []).
message('EKF_EXT', 8007, []).
message('ASL_OBCTRL', 8008, []).
message('SENS_ATMOS', 8009, []).
message('SENS_BATMON', 8010, []).
message('FW_SOARING_DATA', 8011, []).
message('SENSORPOD_STATUS', 8012, []).
message('SENS_POWER_BOARD', 8013, []).
message('GSM_LINK_STATUS', 8014, []).
message('SATCOM_LINK_STATUS', 8015, []).
message('SENSOR_AIRFLOW_ANGLES', 8016, []).
message('SYS_STATUS', 1, []).
message('SYSTEM_TIME', 2, []).
message('PING', 4, []).
message('CHANGE_OPERATOR_CONTROL', 5, []).
message('CHANGE_OPERATOR_CONTROL_ACK', 6, []).
message('AUTH_KEY', 7, []).
message('LINK_NODE_STATUS', 8, []).
message('SET_MODE', 11, []).
message('PARAM_REQUEST_READ', 20, []).
message('PARAM_REQUEST_LIST', 21, []).
message('PARAM_VALUE', 22, []).
message('PARAM_SET', 23, []).
message('GPS_RAW_INT', 24, []).
message('GPS_STATUS', 25, []).
message('SCALED_IMU', 26, []).
message('RAW_IMU', 27, []).
message('RAW_PRESSURE', 28, []).
message('SCALED_PRESSURE', 29, []).
message('ATTITUDE', 30, []).
message('ATTITUDE_QUATERNION', 31, []).
message('LOCAL_POSITION_NED', 32, []).
message('GLOBAL_POSITION_INT', 33, []).
message('RC_CHANNELS_SCALED', 34, []).
message('RC_CHANNELS_RAW', 35, []).
message('SERVO_OUTPUT_RAW', 36, []).
message('MISSION_REQUEST_PARTIAL_LIST', 37, []).
message('MISSION_WRITE_PARTIAL_LIST', 38, []).
message('MISSION_ITEM', 39, []).
message('MISSION_REQUEST', 40, []).
message('MISSION_SET_CURRENT', 41, []).
message('MISSION_CURRENT', 42, []).
message('MISSION_REQUEST_LIST', 43, []).
message('MISSION_COUNT', 44, []).
message('MISSION_CLEAR_ALL', 45, []).
message('MISSION_ITEM_REACHED', 46, []).
message('MISSION_ACK', 47, []).
message('SET_GPS_GLOBAL_ORIGIN', 48, []).
message('GPS_GLOBAL_ORIGIN', 49, []).
message('PARAM_MAP_RC', 50, []).
message('MISSION_REQUEST_INT', 51, []).
message('SAFETY_SET_ALLOWED_AREA', 54, []).
message('SAFETY_ALLOWED_AREA', 55, []).
message('ATTITUDE_QUATERNION_COV', 61, []).
message('NAV_CONTROLLER_OUTPUT', 62, []).
message('GLOBAL_POSITION_INT_COV', 63, []).
message('LOCAL_POSITION_NED_COV', 64, []).
message('RC_CHANNELS', 65, []).
message('REQUEST_DATA_STREAM', 66, []).
message('DATA_STREAM', 67, []).
message('MANUAL_CONTROL', 69, []).
message('RC_CHANNELS_OVERRIDE', 70, []).
message('MISSION_ITEM_INT', 73, []).
message('VFR_HUD', 74, []).
message('COMMAND_INT', 75, []).
message('COMMAND_LONG', 76, []).
message('COMMAND_ACK', 77, []).
message('COMMAND_CANCEL', 80, []).
message('MANUAL_SETPOINT', 81, []).
message('SET_ATTITUDE_TARGET', 82, []).
message('ATTITUDE_TARGET', 83, []).
message('SET_POSITION_TARGET_LOCAL_NED', 84, []).
message('POSITION_TARGET_LOCAL_NED', 85, []).
message('SET_POSITION_TARGET_GLOBAL_INT', 86, []).
message('POSITION_TARGET_GLOBAL_INT', 87, []).
message('LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET', 89, []).
message('HIL_STATE', 90, []).
message('HIL_CONTROLS', 91, []).
message('HIL_RC_INPUTS_RAW', 92, []).
message('HIL_ACTUATOR_CONTROLS', 93, []).
message('OPTICAL_FLOW', 100, []).
message('GLOBAL_VISION_POSITION_ESTIMATE', 101, []).
message('VISION_POSITION_ESTIMATE', 102, []).
message('VISION_SPEED_ESTIMATE', 103, []).
message('VICON_POSITION_ESTIMATE', 104, []).
message('HIGHRES_IMU', 105, []).
message('OPTICAL_FLOW_RAD', 106, []).
message('HIL_SENSOR', 107, []).
message('SIM_STATE', 108, []).
message('RADIO_STATUS', 109, []).
message('FILE_TRANSFER_PROTOCOL', 110, []).
message('TIMESYNC', 111, []).
message('CAMERA_TRIGGER', 112, []).
message('HIL_GPS', 113, []).
message('HIL_OPTICAL_FLOW', 114, []).
message('HIL_STATE_QUATERNION', 115, []).
message('SCALED_IMU2', 116, []).
message('LOG_REQUEST_LIST', 117, []).
message('LOG_ENTRY', 118, []).
message('LOG_REQUEST_DATA', 119, []).
message('LOG_DATA', 120, []).
message('LOG_ERASE', 121, []).
message('LOG_REQUEST_END', 122, []).
message('GPS_INJECT_DATA', 123, []).
message('GPS2_RAW', 124, []).
message('POWER_STATUS', 125, []).
message('SERIAL_CONTROL', 126, []).
message('GPS_RTK', 127, []).
message('GPS2_RTK', 128, []).
message('SCALED_IMU3', 129, []).
message('DATA_TRANSMISSION_HANDSHAKE', 130, []).
message('ENCAPSULATED_DATA', 131, []).
message('DISTANCE_SENSOR', 132, []).
message('TERRAIN_REQUEST', 133, []).
message('TERRAIN_DATA', 134, []).
message('TERRAIN_CHECK', 135, []).
message('TERRAIN_REPORT', 136, []).
message('SCALED_PRESSURE2', 137, []).
message('ATT_POS_MOCAP', 138, []).
message('SET_ACTUATOR_CONTROL_TARGET', 139, []).
message('ACTUATOR_CONTROL_TARGET', 140, []).
message('ALTITUDE', 141, []).
message('RESOURCE_REQUEST', 142, []).
message('SCALED_PRESSURE3', 143, []).
message('FOLLOW_TARGET', 144, []).
message('CONTROL_SYSTEM_STATE', 146, []).
message('BATTERY_STATUS', 147, []).
message('AUTOPILOT_VERSION', 148, []).
message('LANDING_TARGET', 149, []).
message('FENCE_STATUS', 162, []).
message('MAG_CAL_REPORT', 192, []).
message('EFI_STATUS', 225, []).
message('ESTIMATOR_STATUS', 230, []).
message('WIND_COV', 231, []).
message('GPS_INPUT', 232, []).
message('GPS_RTCM_DATA', 233, []).
message('HIGH_LATENCY', 234, []).
message('HIGH_LATENCY2', 235, []).
message('VIBRATION', 241, []).
message('HOME_POSITION', 242, []).
message('SET_HOME_POSITION', 243, []).
message('MESSAGE_INTERVAL', 244, []).
message('EXTENDED_SYS_STATE', 245, []).
message('ADSB_VEHICLE', 246, []).
message('COLLISION', 247, []).
message('V2_EXTENSION', 248, []).
message('MEMORY_VECT', 249, []).
message('DEBUG_VECT', 250, []).
message('NAMED_VALUE_FLOAT', 251, []).
message('NAMED_VALUE_INT', 252, []).
message('STATUSTEXT', 253, []).
message('DEBUG', 254, []).
message('SETUP_SIGNING', 256, []).
message('BUTTON_CHANGE', 257, []).
message('PLAY_TUNE', 258, []).
message('CAMERA_INFORMATION', 259, []).
message('CAMERA_SETTINGS', 260, []).
message('STORAGE_INFORMATION', 261, []).
message('CAMERA_CAPTURE_STATUS', 262, []).
message('CAMERA_IMAGE_CAPTURED', 263, []).
message('FLIGHT_INFORMATION', 264, []).
message('MOUNT_ORIENTATION', 265, []).
message('LOGGING_DATA', 266, []).
message('LOGGING_DATA_ACKED', 267, []).
message('LOGGING_ACK', 268, []).
message('VIDEO_STREAM_INFORMATION', 269, []).
message('VIDEO_STREAM_STATUS', 270, []).
message('CAMERA_FOV_STATUS', 271, []).
message('CAMERA_TRACKING_IMAGE_STATUS', 275, []).
message('CAMERA_TRACKING_GEO_STATUS', 276, []).
message('GIMBAL_MANAGER_INFORMATION', 280, []).
message('GIMBAL_MANAGER_STATUS', 281, []).
message('GIMBAL_MANAGER_SET_ATTITUDE', 282, []).
message('GIMBAL_DEVICE_INFORMATION', 283, []).
message('GIMBAL_DEVICE_SET_ATTITUDE', 284, []).
message('GIMBAL_DEVICE_ATTITUDE_STATUS', 285, []).
message('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', 286, []).
message('GIMBAL_MANAGER_SET_PITCHYAW', 287, []).
message('GIMBAL_MANAGER_SET_MANUAL_CONTROL', 288, []).
message('ESC_INFO', 290, []).
message('ESC_STATUS', 291, []).
message('WIFI_CONFIG_AP', 299, []).
message('AIS_VESSEL', 301, []).
message('UAVCAN_NODE_STATUS', 310, []).
message('UAVCAN_NODE_INFO', 311, []).
message('PARAM_EXT_REQUEST_READ', 320, []).
message('PARAM_EXT_REQUEST_LIST', 321, []).
message('PARAM_EXT_VALUE', 322, []).
message('PARAM_EXT_SET', 323, []).
message('PARAM_EXT_ACK', 324, []).
message('OBSTACLE_DISTANCE', 330, []).
message('ODOMETRY', 331, []).
message('TRAJECTORY_REPRESENTATION_WAYPOINTS', 332, []).
message('TRAJECTORY_REPRESENTATION_BEZIER', 333, []).
message('CELLULAR_STATUS', 334, []).
message('ISBD_LINK_STATUS', 335, []).
message('CELLULAR_CONFIG', 336, []).
message('RAW_RPM', 339, []).
message('UTM_GLOBAL_POSITION', 340, []).
message('DEBUG_FLOAT_ARRAY', 350, []).
message('ORBIT_EXECUTION_STATUS', 360, []).
message('SMART_BATTERY_INFO', 370, []).
message('GENERATOR_STATUS', 373, []).
message('ACTUATOR_OUTPUT_STATUS', 375, []).
message('TIME_ESTIMATE_TO_TARGET', 380, []).
message('TUNNEL', 385, []).
message('CAN_FRAME', 386, []).
message('ONBOARD_COMPUTER_STATUS', 390, []).
message('COMPONENT_INFORMATION', 395, []).
message('COMPONENT_METADATA', 397, []).
message('PLAY_TUNE_V2', 400, []).
message('SUPPORTED_TUNES', 401, []).
message('EVENT', 410, []).
message('CURRENT_EVENT_SEQUENCE', 411, []).
message('REQUEST_EVENT', 412, []).
message('RESPONSE_EVENT_ERROR', 413, []).
message('CANFD_FRAME', 387, []).
message('CAN_FILTER_MODIFY', 388, []).
message('WHEEL_DISTANCE', 9000, []).
message('WINCH_STATUS', 9005, []).
message('OPEN_DRONE_ID_BASIC_ID', 12900, []).
message('OPEN_DRONE_ID_LOCATION', 12901, []).
message('OPEN_DRONE_ID_AUTHENTICATION', 12902, []).
message('OPEN_DRONE_ID_SELF_ID', 12903, []).
message('OPEN_DRONE_ID_SYSTEM', 12904, []).
message('OPEN_DRONE_ID_OPERATOR_ID', 12905, []).
message('OPEN_DRONE_ID_MESSAGE_PACK', 12915, []).
message('OPEN_DRONE_ID_ARM_STATUS', 12918, []).
message('OPEN_DRONE_ID_SYSTEM_UPDATE', 12919, []).
message('HYGROMETER_SENSOR', 12920, []).
message('PARAM_ACK_TRANSACTION', 19, []).
message('AIRSPEED', 295, []).
message('WIFI_NETWORK_INFO', 298, []).
message('FIGURE_EIGHT_EXECUTION_STATUS', 361, []).
message('BATTERY_STATUS_V2', 369, []).
message('COMPONENT_INFORMATION_BASIC', 396, []).
message('GROUP_START', 414, []).
message('GROUP_END', 415, []).
message('AVAILABLE_MODES', 435, []).
message('CURRENT_MODE', 436, []).
message('AVAILABLE_MODES_MONITOR', 437, []).
message('TARGET_ABSOLUTE', 510, []).
message('TARGET_RELATIVE', 511, []).
message('ICAROUS_HEARTBEAT', 42000, []).
message('ICAROUS_KINEMATIC_BANDS', 42001, []).
message('HEARTBEAT', 0, []).
message('PROTOCOL_VERSION', 300, []).
message('ARRAY_TEST_0', 17150, []).
message('ARRAY_TEST_1', 17151, []).
message('ARRAY_TEST_3', 17153, []).
message('ARRAY_TEST_4', 17154, []).
message('ARRAY_TEST_5', 17155, []).
message('ARRAY_TEST_6', 17156, []).
message('ARRAY_TEST_7', 17157, []).
message('ARRAY_TEST_8', 17158, []).
message('TEST_TYPES', 17000, []).
message('NAV_FILTER_BIAS', 220, []).
message('RADIO_CALIBRATION', 221, []).
message('UALBERTA_SYS_STATUS', 222, []).
message('UAVIONIX_ADSB_OUT_CFG', 10001, []).
message('UAVIONIX_ADSB_OUT_DYNAMIC', 10002, []).
message('UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT', 10003, []).
message('STORM32_GIMBAL_MANAGER_INFORMATION', 60010, []).
message('STORM32_GIMBAL_MANAGER_STATUS', 60011, []).
message('STORM32_GIMBAL_MANAGER_CONTROL', 60012, []).
message('STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW', 60013, []).
message('STORM32_GIMBAL_MANAGER_CORRECT_ROLL', 60014, []).
message('QSHOT_STATUS', 60020, []).
message('RADIO_RC_CHANNELS', 60045, []).
message('RADIO_LINK_STATS', 60046, []).
message('FRSKY_PASSTHROUGH_ARRAY', 60040, []).
message('PARAM_VALUE_ARRAY', 60041, []).
message('AVSS_PRS_SYS_STATUS', 60050, []).
message('AVSS_DRONE_POSITION', 60051, []).
message('AVSS_DRONE_IMU', 60052, []).
message('AVSS_DRONE_OPERATION_MODE', 60053, []).
message('CUBEPILOT_RAW_RC', 50001, []).
message('HERELINK_VIDEO_STREAM_INFORMATION', 50002, []).
message('HERELINK_TELEM', 50003, []).
message('CUBEPILOT_FIRMWARE_UPDATE_START', 50004, []).
message('CUBEPILOT_FIRMWARE_UPDATE_RESP', 50005, []).
message('AIRLINK_AUTH', 52000, []).
message('AIRLINK_AUTH_RESPONSE', 52001, []).

:- dynamic message_deprecated/2.

message_deprecated('SET_MAG_OFFSETS', [since='2014-07', replaced_by='MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS']).
message_deprecated('BATTERY2', [since='2017-04', replaced_by='BATTERY_STATUS']).
message_deprecated('PING', [since='2011-08', replaced_by='SYSTEM_TIME']).
message_deprecated('SET_MODE', [since='2015-12', replaced_by='MAV_CMD_DO_SET_MODE']).
message_deprecated('MISSION_ITEM', [since='2020-06', replaced_by='MISSION_ITEM_INT']).
message_deprecated('MISSION_REQUEST', [since='2020-06', replaced_by='MISSION_REQUEST_INT']).
message_deprecated('MISSION_SET_CURRENT', [since='2022-08', replaced_by='MAV_CMD_DO_SET_MISSION_CURRENT']).
message_deprecated('REQUEST_DATA_STREAM', [since='2015-08', replaced_by='SET_MESSAGE_INTERVAL']).
message_deprecated('DATA_STREAM', [since='2015-08', replaced_by='MESSAGE_INTERVAL']).
message_deprecated('HIL_STATE', [since='2013-07', replaced_by='HIL_STATE_QUATERNION']).
message_deprecated('GPS_INJECT_DATA', [since='2022-05', replaced_by='GPS_RTCM_DATA']).
message_deprecated('HIGH_LATENCY', [since='2020-10', replaced_by='HIGH_LATENCY2']).
message_deprecated('SET_HOME_POSITION', [since='2022-02', replaced_by='MAV_CMD_DO_SET_HOME']).
message_deprecated('PLAY_TUNE', [since='2019-10', replaced_by='PLAY_TUNE_V2']).
message_deprecated('MOUNT_ORIENTATION', [since='2020-01', replaced_by='MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW']).
message_deprecated('COMPONENT_INFORMATION', [since='2022-04', replaced_by='COMPONENT_METADATA']).

:- dynamic message_description/2.

message_description('SENSOR_OFFSETS', 'Offsets and calibrations values for hardware sensors. This makes it easier to debug the calibration process.').
message_description('SET_MAG_OFFSETS', 'Set the magnetometer offsets').
message_description('MEMINFO', 'State of autopilot RAM.').
message_description('AP_ADC', 'Raw ADC output.').
message_description('DIGICAM_CONFIGURE', 'Configure on-board Camera Control System.').
message_description('DIGICAM_CONTROL', 'Control on-board Camera Control System to take shots.').
message_description('MOUNT_CONFIGURE', 'Message to configure a camera mount, directional antenna, etc.').
message_description('MOUNT_CONTROL', 'Message to control a camera mount, directional antenna, etc.').
message_description('MOUNT_STATUS', 'Message with some status from autopilot to GCS about camera or antenna mount.').
message_description('FENCE_POINT', 'A fence point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS.').
message_description('FENCE_FETCH_POINT', 'Request a current fence point from MAV.').
message_description('AHRS', 'Status of DCM attitude estimator.').
message_description('SIMSTATE', 'Status of simulation environment, if used.').
message_description('HWSTATUS', 'Status of key hardware.').
message_description('RADIO', 'Status generated by radio.').
message_description('LIMITS_STATUS', 'Status of AP_Limits. Sent in extended status stream when AP_Limits is enabled.').
message_description('WIND', 'Wind estimation.').
message_description('DATA16', 'Data packet, size 16.').
message_description('DATA32', 'Data packet, size 32.').
message_description('DATA64', 'Data packet, size 64.').
message_description('DATA96', 'Data packet, size 96.').
message_description('RANGEFINDER', 'Rangefinder reporting.').
message_description('AIRSPEED_AUTOCAL', 'Airspeed auto-calibration.').
message_description('RALLY_POINT', 'A rally point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS.').
message_description('RALLY_FETCH_POINT', 'Request a current rally point from MAV. MAV should respond with a RALLY_POINT message. MAV should not respond if the request is invalid.').
message_description('COMPASSMOT_STATUS', 'Status of compassmot calibration.').
message_description('AHRS2', 'Status of secondary AHRS filter if available.').
message_description('CAMERA_STATUS', 'Camera Event.').
message_description('CAMERA_FEEDBACK', 'Camera Capture Feedback.').
message_description('BATTERY2', '2nd Battery status').
message_description('AHRS3', 'Status of third AHRS filter if available. This is for ANU research group (Ali and Sean).').
message_description('AUTOPILOT_VERSION_REQUEST', 'Request the autopilot version from the system/component.').
message_description('REMOTE_LOG_DATA_BLOCK', 'Send a block of log data to remote location.').
message_description('REMOTE_LOG_BLOCK_STATUS', 'Send Status of each log block that autopilot board might have sent.').
message_description('LED_CONTROL', 'Control vehicle LEDs.').
message_description('MAG_CAL_PROGRESS', 'Reports progress of compass calibration.').
message_description('EKF_STATUS_REPORT', 'EKF Status message including flags and variances.').
message_description('PID_TUNING', 'PID tuning information.').
message_description('DEEPSTALL', 'Deepstall path planning.').
message_description('GIMBAL_REPORT', '3 axis gimbal measurements.').
message_description('GIMBAL_CONTROL', 'Control message for rate gimbal.').
message_description('GIMBAL_TORQUE_CMD_REPORT', '100 Hz gimbal torque command telemetry.').
message_description('GOPRO_HEARTBEAT', 'Heartbeat from a HeroBus attached GoPro.').
message_description('GOPRO_GET_REQUEST', 'Request a GOPRO_COMMAND response from the GoPro.').
message_description('GOPRO_GET_RESPONSE', 'Response from a GOPRO_COMMAND get request.').
message_description('GOPRO_SET_REQUEST', 'Request to set a GOPRO_COMMAND with a desired.').
message_description('GOPRO_SET_RESPONSE', 'Response from a GOPRO_COMMAND set request.').
message_description('RPM', 'RPM sensor output.').
message_description('DEVICE_OP_READ', 'Read registers for a device.').
message_description('DEVICE_OP_READ_REPLY', 'Read registers reply.').
message_description('DEVICE_OP_WRITE', 'Write registers for a device.').
message_description('DEVICE_OP_WRITE_REPLY', 'Write registers reply.').
message_description('ADAP_TUNING', 'Adaptive Controller tuning information.').
message_description('VISION_POSITION_DELTA', 'Camera vision based attitude and position deltas.').
message_description('AOA_SSA', 'Angle of Attack and Side Slip Angle.').
message_description('ESC_TELEMETRY_1_TO_4', 'ESC Telemetry Data for ESCs 1 to 4, matching data sent by BLHeli ESCs.').
message_description('ESC_TELEMETRY_5_TO_8', 'ESC Telemetry Data for ESCs 5 to 8, matching data sent by BLHeli ESCs.').
message_description('ESC_TELEMETRY_9_TO_12', 'ESC Telemetry Data for ESCs 9 to 12, matching data sent by BLHeli ESCs.').
message_description('OSD_PARAM_CONFIG', 'Configure an OSD parameter slot.').
message_description('OSD_PARAM_CONFIG_REPLY', 'Configure OSD parameter reply.').
message_description('OSD_PARAM_SHOW_CONFIG', 'Read a configured an OSD parameter slot.').
message_description('OSD_PARAM_SHOW_CONFIG_REPLY', 'Read configured OSD parameter reply.').
message_description('OBSTACLE_DISTANCE_3D', 'Obstacle located as a 3D vector.').
message_description('WATER_DEPTH', 'Water depth').
message_description('MCU_STATUS', 'The MCU status, giving MCU temperature and voltage. The min and max voltages are to allow for detecting power supply instability.').
message_description('COMMAND_INT_STAMPED', 'Message encoding a command with parameters as scaled integers and additional metadata. Scaling depends on the actual command value.').
message_description('COMMAND_LONG_STAMPED', 'Send a command with up to seven parameters to the MAV and additional metadata').
message_description('SENS_POWER', 'Voltage and current sensor data').
message_description('SENS_MPPT', 'Maximum Power Point Tracker (MPPT) sensor data for solar module power performance tracking').
message_description('ASLCTRL_DATA', 'ASL-fixed-wing controller data').
message_description('ASLCTRL_DEBUG', 'ASL-fixed-wing controller debug data').
message_description('ASLUAV_STATUS', 'Extended state information for ASLUAVs').
message_description('EKF_EXT', 'Extended EKF state estimates for ASLUAVs').
message_description('ASL_OBCTRL', 'Off-board controls/commands for ASLUAVs').
message_description('SENS_ATMOS', 'Atmospheric sensors (temperature, humidity, ...) ').
message_description('SENS_BATMON', 'Battery pack monitoring data for Li-Ion batteries').
message_description('FW_SOARING_DATA', 'Fixed-wing soaring (i.e. thermal seeking) data').
message_description('SENSORPOD_STATUS', 'Monitoring of sensorpod status').
message_description('SENS_POWER_BOARD', 'Monitoring of power board status').
message_description('GSM_LINK_STATUS', 'Status of GSM modem (connected to onboard computer)').
message_description('SATCOM_LINK_STATUS', 'Status of the SatCom link').
message_description('SENSOR_AIRFLOW_ANGLES', 'Calibrated airflow angle measurements').
message_description('SYS_STATUS', 'The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows whether the system is currently active or not and if an emergency occurred. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occurred it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.').
message_description('SYSTEM_TIME', 'The system time is the time of the master clock, typically the computer clock of the main onboard computer.').
message_description('PING', 'A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections. The ping microservice is documented at https://mavlink.io/en/services/ping.html').
message_description('CHANGE_OPERATOR_CONTROL', 'Request to control this MAV').
message_description('CHANGE_OPERATOR_CONTROL_ACK', 'Accept / deny control of this MAV').
message_description('AUTH_KEY', 'Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety.').
message_description('LINK_NODE_STATUS', 'Status generated in each node in the communication chain and injected into MAVLink stream.').
message_description('SET_MODE', 'Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component.').
message_description('PARAM_REQUEST_READ', 'Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also https://mavlink.io/en/services/parameter.html for a full documentation of QGroundControl and IMU code.').
message_description('PARAM_REQUEST_LIST', 'Request all parameters of this component. After this request, all parameters are emitted. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html').
message_description('PARAM_VALUE', 'Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html').
message_description('PARAM_SET', 'Set a parameter value (write new value to permanent storage).\n        The receiving component should acknowledge the new parameter value by broadcasting a PARAM_VALUE message (broadcasting ensures that multiple GCS all have an up-to-date list of all parameters). If the sending GCS did not receive a PARAM_VALUE within its timeout time, it should re-send the PARAM_SET message. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html.\n        PARAM_SET may also be called within the context of a transaction (started with MAV_CMD_PARAM_TRANSACTION). Within a transaction the receiving component should respond with PARAM_ACK_TRANSACTION to the setter component (instead of broadcasting PARAM_VALUE), and PARAM_SET should be re-sent if this is ACK not received.').
message_description('GPS_RAW_INT', 'The global position, as returned by the Global Positioning System (GPS). This is\n                NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION_INT for the global position estimate.').
message_description('GPS_STATUS', 'The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION_INT for the global position estimate. This message can contain information for up to 20 satellites.').
message_description('SCALED_IMU', 'The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units').
message_description('RAW_IMU', 'The RAW IMU readings for a 9DOF sensor, which is identified by the id (default IMU1). This message should always contain the true raw values without any scaling to allow data capture and system debugging.').
message_description('RAW_PRESSURE', 'The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.').
message_description('SCALED_PRESSURE', 'The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field.').
message_description('ATTITUDE', 'The attitude in the aeronautical frame (right-handed, Z-down, Y-right, X-front, ZYX, intrinsic).').
message_description('ATTITUDE_QUATERNION', 'The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).').
message_description('LOCAL_POSITION_NED', 'The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)').
message_description('GLOBAL_POSITION_INT', 'The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It\n               is designed as scaled integer message since the resolution of float is not sufficient.').
message_description('RC_CHANNELS_SCALED', 'The scaled values of the RC channels received: (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to INT16_MAX.').
message_description('RC_CHANNELS_RAW', 'The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. A value of UINT16_MAX implies the channel is unused. Individual receivers/transmitters might violate this specification.').
message_description('SERVO_OUTPUT_RAW', 'Superseded by ACTUATOR_OUTPUT_STATUS. The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.').
message_description('MISSION_REQUEST_PARTIAL_LIST', 'Request a partial list of mission items from the system/component. https://mavlink.io/en/services/mission.html. If start and end index are the same, just send one waypoint.').
message_description('MISSION_WRITE_PARTIAL_LIST', 'This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!').
message_description('MISSION_ITEM', 'Message encoding a mission item. This message is emitted to announce\n                the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). NaN may be used to indicate an optional/default value (e.g. to use the system\'s current latitude or yaw rather than a specific value). See also https://mavlink.io/en/services/mission.html.').
message_description('MISSION_REQUEST', 'Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. https://mavlink.io/en/services/mission.html').
message_description('MISSION_SET_CURRENT', '\n        Set the mission item with sequence number seq as the current item and emit MISSION_CURRENT (whether or not the mission number changed).\n        If a mission is currently being executed, the system will continue to this new mission item on the shortest path, skipping any intermediate mission items.\n        Note that mission jump repeat counters are not reset (see MAV_CMD_DO_JUMP param2).\n\n        This message may trigger a mission state-machine change on some systems: for example from MISSION_STATE_NOT_STARTED or MISSION_STATE_PAUSED to MISSION_STATE_ACTIVE.\n        If the system is in mission mode, on those systems this command might therefore start, restart or resume the mission.\n        If the system is not in mission mode this message must not trigger a switch to mission mode.\n      ').
message_description('MISSION_CURRENT', '\n        Message that announces the sequence number of the current target mission item (that the system will fly towards/execute when the mission is running).\n        This message should be streamed all the time (nominally at 1Hz).\n        This message should be emitted following a call to MAV_CMD_DO_SET_MISSION_CURRENT or SET_MISSION_CURRENT.\n      ').
message_description('MISSION_REQUEST_LIST', 'Request the overall list of mission items from the system/component.').
message_description('MISSION_COUNT', 'This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of waypoints.').
message_description('MISSION_CLEAR_ALL', 'Delete all mission items at once.').
message_description('MISSION_ITEM_REACHED', 'A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next waypoint.').
message_description('MISSION_ACK', 'Acknowledgment message during waypoint handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).').
message_description('SET_GPS_GLOBAL_ORIGIN', 'Sets the GPS coordinates of the vehicle local origin (0,0,0) position. Vehicle should emit GPS_GLOBAL_ORIGIN irrespective of whether the origin is changed. This enables transform between the local coordinate frame and the global (GPS) coordinate frame, which may be necessary when (for example) indoor and outdoor settings are connected and the MAV should move from in- to outdoor.').
message_description('GPS_GLOBAL_ORIGIN', 'Publishes the GPS coordinates of the vehicle local origin (0,0,0) position. Emitted whenever a new GPS-Local position mapping is requested or set - e.g. following SET_GPS_GLOBAL_ORIGIN message.').
message_description('PARAM_MAP_RC', 'Bind a RC channel to a parameter. The parameter should change according to the RC channel value.').
message_description('MISSION_REQUEST_INT', 'Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM_INT message. https://mavlink.io/en/services/mission.html').
message_description('SAFETY_SET_ALLOWED_AREA', 'Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national or competition regulations.').
message_description('SAFETY_ALLOWED_AREA', 'Read out the safety zone the MAV currently assumes.').
message_description('ATTITUDE_QUATERNION_COV', 'The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).').
message_description('NAV_CONTROLLER_OUTPUT', 'The state of the navigation and position controller.').
message_description('GLOBAL_POSITION_INT_COV', 'The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE: This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset.').
message_description('LOCAL_POSITION_NED_COV', 'The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)').
message_description('RC_CHANNELS', 'The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.  A value of UINT16_MAX implies the channel is unused. Individual receivers/transmitters might violate this specification.').
message_description('REQUEST_DATA_STREAM', 'Request a data stream.').
message_description('DATA_STREAM', 'Data stream status information.').
message_description('MANUAL_CONTROL', 'This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled and buttons states are transmitted as individual on/off bits of a bitmask').
message_description('RC_CHANNELS_OVERRIDE', 'The RAW values of the RC channels sent to the MAV to override info received from the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.  Note carefully the semantic differences between the first 8 channels and the subsequent channels').
message_description('MISSION_ITEM_INT', 'Message encoding a mission item. This message is emitted to announce\n                the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). NaN or INT32_MAX may be used in float/integer params (respectively) to indicate optional/default values (e.g. to use the component\'s current latitude, yaw rather than a specific value). See also https://mavlink.io/en/services/mission.html.').
message_description('VFR_HUD', 'Metrics typically displayed on a HUD for fixed wing aircraft.').
message_description('COMMAND_INT', 'Send a command with up to seven parameters to the MAV, where params 5 and 6 are integers and the other values are floats. This is preferred over COMMAND_LONG as it allows the MAV_FRAME to be specified for interpreting positional information, such as altitude. COMMAND_INT is also preferred when sending latitude and longitude data in params 5 and 6, as it allows for greater precision. Param 5 and 6 encode positional data as scaled integers, where the scaling depends on the actual command value. NaN or INT32_MAX may be used in float/integer params (respectively) to indicate optional/default values (e.g. to use the component\'s current latitude, yaw rather than a specific value). The command microservice is documented at https://mavlink.io/en/services/command.html').
message_description('COMMAND_LONG', 'Send a command with up to seven parameters to the MAV. COMMAND_INT is generally preferred when sending MAV_CMD commands that include positional information; it offers higher precision and allows the MAV_FRAME to be specified (which may otherwise be ambiguous, particularly for altitude). The command microservice is documented at https://mavlink.io/en/services/command.html').
message_description('COMMAND_ACK', 'Report status of a command. Includes feedback whether the command was executed. The command microservice is documented at https://mavlink.io/en/services/command.html').
message_description('COMMAND_CANCEL', 'Cancel a long running command. The target system should respond with a COMMAND_ACK to the original command with result=MAV_RESULT_CANCELLED if the long running process was cancelled. If it has already completed, the cancel action can be ignored. The cancel action can be retried until some sort of acknowledgement to the original command has been received. The command microservice is documented at https://mavlink.io/en/services/command.html').
message_description('MANUAL_SETPOINT', 'Setpoint in roll, pitch, yaw and thrust from the operator').
message_description('SET_ATTITUDE_TARGET', 'Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller or other system).').
message_description('ATTITUDE_TARGET', 'Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way.').
message_description('SET_POSITION_TARGET_LOCAL_NED', 'Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller to command the vehicle (manual controller or other system).').
message_description('POSITION_TARGET_LOCAL_NED', 'Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled this way.').
message_description('SET_POSITION_TARGET_GLOBAL_INT', 'Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84). Used by an external controller to command the vehicle (manual controller or other system).').
message_description('POSITION_TARGET_GLOBAL_INT', 'Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled this way.').
message_description('LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET', 'The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)').
message_description('HIL_STATE', 'Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations.').
message_description('HIL_CONTROLS', 'Sent from autopilot to simulation. Hardware in the loop control outputs').
message_description('HIL_RC_INPUTS_RAW', 'Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.').
message_description('HIL_ACTUATOR_CONTROLS', 'Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS)').
message_description('OPTICAL_FLOW', 'Optical flow from a flow sensor (e.g. optical mouse sensor)').
message_description('GLOBAL_VISION_POSITION_ESTIMATE', 'Global position/attitude estimate from a vision source.').
message_description('VISION_POSITION_ESTIMATE', 'Local position/attitude estimate from a vision source.').
message_description('VISION_SPEED_ESTIMATE', 'Speed estimate from a vision source.').
message_description('VICON_POSITION_ESTIMATE', 'Global position estimate from a Vicon motion system source.').
message_description('HIGHRES_IMU', 'The IMU readings in SI units in NED body frame').
message_description('OPTICAL_FLOW_RAD', 'Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)').
message_description('HIL_SENSOR', 'The IMU readings in SI units in NED body frame').
message_description('SIM_STATE', 'Status of simulation environment, if used').
message_description('RADIO_STATUS', 'Status generated by radio and injected into MAVLink stream.').
message_description('FILE_TRANSFER_PROTOCOL', 'File transfer protocol message: https://mavlink.io/en/services/ftp.html.').
message_description('TIMESYNC', '\n        Time synchronization message.\n        The message is used for both timesync requests and responses.\n        The request is sent with `ts1=syncing component timestamp` and `tc1=0`, and may be broadcast or targeted to a specific system/component.\n        The response is sent with `ts1=syncing component timestamp` (mirror back unchanged), and `tc1=responding component timestamp`, with the `target_system` and `target_component` set to ids of the original request.\n        Systems can determine if they are receiving a request or response based on the value of `tc`.\n        If the response has `target_system==target_component==0` the remote system has not been updated to use the component IDs and cannot reliably timesync; the requestor may report an error.\n        Timestamps are UNIX Epoch time or time since system boot in nanoseconds (the timestamp format can be inferred by checking for the magnitude of the number; generally it doesn\'t matter as only the offset is used).\n        The message sequence is repeated numerous times with results being filtered/averaged to estimate the offset.\n      ').
message_description('CAMERA_TRIGGER', 'Camera-IMU triggering and synchronisation message.').
message_description('HIL_GPS', 'The global position, as returned by the Global Positioning System (GPS). This is\n                 NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION_INT for the global position estimate.').
message_description('HIL_OPTICAL_FLOW', 'Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)').
message_description('HIL_STATE_QUATERNION', 'Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations.').
message_description('SCALED_IMU2', 'The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to the described units').
message_description('LOG_REQUEST_LIST', 'Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END is called. If there are no log files available this request shall be answered with one LOG_ENTRY message with id = 0 and num_logs = 0.').
message_description('LOG_ENTRY', 'Reply to LOG_REQUEST_LIST').
message_description('LOG_REQUEST_DATA', 'Request a chunk of a log').
message_description('LOG_DATA', 'Reply to LOG_REQUEST_DATA').
message_description('LOG_ERASE', 'Erase all logs').
message_description('LOG_REQUEST_END', 'Stop log transfer and resume normal logging').
message_description('GPS_INJECT_DATA', 'Data for injecting into the onboard GPS (used for DGPS)').
message_description('GPS2_RAW', 'Second GPS data.').
message_description('POWER_STATUS', 'Power supply status').
message_description('SERIAL_CONTROL', 'Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages or change the devices settings. A message with zero bytes can be used to change just the baudrate.').
message_description('GPS_RTK', 'RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting').
message_description('GPS2_RTK', 'RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting').
message_description('SCALED_IMU3', 'The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described units').
message_description('DATA_TRANSMISSION_HANDSHAKE', 'Handshake message to initiate, control and stop image streaming when using the Image Transmission Protocol: https://mavlink.io/en/services/image_transmission.html.').
message_description('ENCAPSULATED_DATA', 'Data packet for images sent using the Image Transmission Protocol: https://mavlink.io/en/services/image_transmission.html.').
message_description('DISTANCE_SENSOR', 'Distance sensor information for an onboard rangefinder.').
message_description('TERRAIN_REQUEST', 'Request for terrain data and terrain status. See terrain protocol docs: https://mavlink.io/en/services/terrain.html').
message_description('TERRAIN_DATA', 'Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST. See terrain protocol docs: https://mavlink.io/en/services/terrain.html').
message_description('TERRAIN_CHECK', 'Request that the vehicle report terrain height at the given location (expected response is a TERRAIN_REPORT). Used by GCS to check if vehicle has all terrain data needed for a mission.').
message_description('TERRAIN_REPORT', 'Streamed from drone to report progress of terrain map download (initiated by TERRAIN_REQUEST), or sent as a response to a TERRAIN_CHECK request. See terrain protocol docs: https://mavlink.io/en/services/terrain.html').
message_description('SCALED_PRESSURE2', 'Barometer readings for 2nd barometer').
message_description('ATT_POS_MOCAP', 'Motion capture attitude and position').
message_description('SET_ACTUATOR_CONTROL_TARGET', 'Set the vehicle attitude and body angular rates.').
message_description('ACTUATOR_CONTROL_TARGET', 'Set the vehicle attitude and body angular rates.').
message_description('ALTITUDE', 'The current system altitude.').
message_description('RESOURCE_REQUEST', 'The autopilot is requesting a resource (file, binary, other type of data)').
message_description('SCALED_PRESSURE3', 'Barometer readings for 3rd barometer').
message_description('FOLLOW_TARGET', 'Current motion information from a designated system').
message_description('CONTROL_SYSTEM_STATE', 'The smoothed, monotonic system state used to feed the control loops of the system.').
message_description('BATTERY_STATUS', 'Battery information. Updates GCS with flight controller battery status. Smart batteries also use this message, but may additionally send SMART_BATTERY_INFO.').
message_description('AUTOPILOT_VERSION', 'Version and capability of autopilot software. This should be emitted in response to a request with MAV_CMD_REQUEST_MESSAGE.').
message_description('LANDING_TARGET', 'The location of a landing target. See: https://mavlink.io/en/services/landing_target.html').
message_description('FENCE_STATUS', 'Status of geo-fencing. Sent in extended status stream when fencing enabled.').
message_description('MAG_CAL_REPORT', 'Reports results of completed compass calibration. Sent until MAG_CAL_ACK received.').
message_description('EFI_STATUS', 'EFI status output').
message_description('ESTIMATOR_STATUS', 'Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS enum definition for further information. The innovation test ratios show the magnitude of the sensor innovation divided by the innovation check threshold. Under normal operation the innovation test ratios should be below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should be optional and controllable by the user.').
message_description('WIND_COV', 'Wind estimate from vehicle. Note that despite the name, this message does not actually contain any covariances but instead variability and accuracy fields in terms of standard deviation (1-STD).').
message_description('GPS_INPUT', 'GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position estimate of the system.').
message_description('GPS_RTCM_DATA', 'RTCM message for injecting into the onboard GPS (used for DGPS)').
message_description('HIGH_LATENCY', 'Message appropriate for high latency connections like Iridium').
message_description('HIGH_LATENCY2', 'Message appropriate for high latency connections like Iridium (version 2)').
message_description('VIBRATION', 'Vibration levels and accelerometer clipping').
message_description('HOME_POSITION', '\n\tContains the home position.\n\tThe home position is the default position that the system will return to and land on.\n\tThe position must be set automatically by the system during the takeoff, and may also be explicitly set using MAV_CMD_DO_SET_HOME.\n\tThe global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface.\n\tUnder normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach.\n\tThe approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.\n        Note: this message can be requested by sending the MAV_CMD_REQUEST_MESSAGE with param1=242 (or the deprecated MAV_CMD_GET_HOME_POSITION command).\n      ').
message_description('SET_HOME_POSITION', '\n        Sets the home position.\n\tThe home position is the default position that the system will return to and land on.\n        The position is set automatically by the system during the takeoff (and may also be set using this message).\n        The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface.\n        Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach.\n        The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.\n        Note: the current home position may be emitted in a HOME_POSITION message on request (using MAV_CMD_REQUEST_MESSAGE with param1=242).\n      ').
message_description('MESSAGE_INTERVAL', '\n        The interval between messages for a particular MAVLink message ID.\n        This message is sent in response to the MAV_CMD_REQUEST_MESSAGE command with param1=244 (this message) and param2=message_id (the id of the message for which the interval is required).\n\tIt may also be sent in response to MAV_CMD_GET_MESSAGE_INTERVAL.\n\tThis interface replaces DATA_STREAM.').
message_description('EXTENDED_SYS_STATE', 'Provides state for additional features').
message_description('ADSB_VEHICLE', 'The location and information of an ADSB vehicle').
message_description('COLLISION', 'Information about a potential collision').
message_description('V2_EXTENSION', 'Message implementing parts of the V2 payload specs in V1 frames for transitional support.').
message_description('MEMORY_VECT', 'Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.').
message_description('DEBUG_VECT', 'To debug something using a named 3D vector.').
message_description('NAMED_VALUE_FLOAT', 'Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.').
message_description('NAMED_VALUE_INT', 'Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.').
message_description('STATUSTEXT', 'Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).').
message_description('DEBUG', 'Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.').
message_description('SETUP_SIGNING', 'Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will disable signing').
message_description('BUTTON_CHANGE', 'Report button state change.').
message_description('PLAY_TUNE', 'Control vehicle tone generation (buzzer).').
message_description('CAMERA_INFORMATION', 'Information about a camera. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.').
message_description('CAMERA_SETTINGS', 'Settings of a camera. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.').
message_description('STORAGE_INFORMATION', 'Information about a storage medium. This message is sent in response to a request with MAV_CMD_REQUEST_MESSAGE and whenever the status of the storage changes (STORAGE_STATUS). Use MAV_CMD_REQUEST_MESSAGE.param2 to indicate the index/id of requested storage: 0 for all, 1 for first, 2 for second, etc.').
message_description('CAMERA_CAPTURE_STATUS', 'Information about the status of a capture. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.').
message_description('CAMERA_IMAGE_CAPTURED', 'Information about a captured image. This is emitted every time a message is captured.\n        MAV_CMD_REQUEST_MESSAGE can be used to (re)request this message for a specific sequence number or range of sequence numbers:\n        MAV_CMD_REQUEST_MESSAGE.param2 indicates the sequence number the first image to send, or set to -1 to send the message for all sequence numbers.\n        MAV_CMD_REQUEST_MESSAGE.param3 is used to specify a range of messages to send:\n        set to 0 (default) to send just the the message for the sequence number in param 2,\n        set to -1 to send the message for the sequence number in param 2 and all the following sequence numbers,\n        set to the sequence number of the final message in the range.').
message_description('FLIGHT_INFORMATION', 'Information about flight since last arming.\n        This can be requested using MAV_CMD_REQUEST_MESSAGE.\n      ').
message_description('MOUNT_ORIENTATION', 'Orientation of a mount').
message_description('LOGGING_DATA', 'A message containing logged data (see also MAV_CMD_LOGGING_START)').
message_description('LOGGING_DATA_ACKED', 'A message containing logged data which requires a LOGGING_ACK to be sent back').
message_description('LOGGING_ACK', 'An ack for a LOGGING_DATA_ACKED message').
message_description('VIDEO_STREAM_INFORMATION', 'Information about video stream. It may be requested using MAV_CMD_REQUEST_MESSAGE, where param2 indicates the video stream id: 0 for all streams, 1 for first, 2 for second, etc.').
message_description('VIDEO_STREAM_STATUS', 'Information about the status of a video stream. It may be requested using MAV_CMD_REQUEST_MESSAGE.').
message_description('CAMERA_FOV_STATUS', 'Information about the field of view of a camera. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.').
message_description('CAMERA_TRACKING_IMAGE_STATUS', 'Camera tracking status, sent while in active tracking. Use MAV_CMD_SET_MESSAGE_INTERVAL to define message interval.').
message_description('CAMERA_TRACKING_GEO_STATUS', 'Camera tracking status, sent while in active tracking. Use MAV_CMD_SET_MESSAGE_INTERVAL to define message interval.').
message_description('GIMBAL_MANAGER_INFORMATION', 'Information about a high level gimbal manager. This message should be requested by a ground station using MAV_CMD_REQUEST_MESSAGE.').
message_description('GIMBAL_MANAGER_STATUS', 'Current status about a high level gimbal manager. This message should be broadcast at a low regular rate (e.g. 5Hz).').
message_description('GIMBAL_MANAGER_SET_ATTITUDE', 'High level message to control a gimbal\'s attitude. This message is to be sent to the gimbal manager (e.g. from a ground station). Angles and rates can be set to NaN according to use case.').
message_description('GIMBAL_DEVICE_INFORMATION', 'Information about a low level gimbal. This message should be requested by the gimbal manager or a ground station using MAV_CMD_REQUEST_MESSAGE. The maximum angles and rates are the limits by hardware. However, the limits by software used are likely different/smaller and dependent on mode/settings/etc..').
message_description('GIMBAL_DEVICE_SET_ATTITUDE', 'Low level message to control a gimbal device\'s attitude.\n\t  This message is to be sent from the gimbal manager to the gimbal device component.\n\t  The quaternion and angular velocities can be set to NaN according to use case.\n\t  For the angles encoded in the quaternion and the angular velocities holds:\n\t  If the flag GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME is set, then they are relative to the vehicle heading (vehicle frame).\n\t  If the flag GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME is set, then they are relative to absolute North (earth frame).\n\t  If neither of these flags are set, then (for backwards compatibility) it holds:\n\t  If the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set, then they are relative to absolute North (earth frame),\n\t  else they are relative to the vehicle heading (vehicle frame).\n\t  Setting both GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME and GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME is not allowed.\n\t  These rules are to ensure backwards compatibility.\n\t  New implementations should always set either GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME or GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME.').
message_description('GIMBAL_DEVICE_ATTITUDE_STATUS', 'Message reporting the status of a gimbal device.\n\t  This message should be broadcast by a gimbal device component at a low regular rate (e.g. 5 Hz).\n\t  For the angles encoded in the quaternion and the angular velocities holds:\n\t  If the flag GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME is set, then they are relative to the vehicle heading (vehicle frame).\n\t  If the flag GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME is set, then they are relative to absolute North (earth frame).\n\t  If neither of these flags are set, then (for backwards compatibility) it holds:\n\t  If the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set, then they are relative to absolute North (earth frame),\n\t  else they are relative to the vehicle heading (vehicle frame).\n\t  Other conditions of the flags are not allowed.\n\t  The quaternion and angular velocities in the other frame can be calculated from delta_yaw and delta_yaw_velocity as\n\t  q_earth = q_delta_yaw * q_vehicle and w_earth = w_delta_yaw_velocity + w_vehicle (if not NaN).\n\t  If neither the GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME nor the GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME flag is set,\n\t  then (for backwards compatibility) the data in the delta_yaw and delta_yaw_velocity fields are to be ignored.\n\t  New implementations should always set either GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME or GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME,\n\t  and always should set delta_yaw and delta_yaw_velocity either to the proper value or NaN.').
message_description('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', 'Low level message containing autopilot state relevant for a gimbal device. This message is to be sent from the autopilot to the gimbal device component. The data of this message are for the gimbal device\'s estimator corrections, in particular horizon compensation, as well as indicates autopilot control intentions, e.g. feed forward angular control in the z-axis.').
message_description('GIMBAL_MANAGER_SET_PITCHYAW', 'Set gimbal manager pitch and yaw angles (high rate message). This message is to be sent to the gimbal manager (e.g. from a ground station) and will be ignored by gimbal devices. Angles and rates can be set to NaN according to use case. Use MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW for low-rate adjustments that require confirmation.').
message_description('GIMBAL_MANAGER_SET_MANUAL_CONTROL', 'High level message to control a gimbal manually. The angles or angular rates are unitless; the actual rates will depend on internal gimbal manager settings/configuration (e.g. set by parameters). This message is to be sent to the gimbal manager (e.g. from a ground station). Angles and rates can be set to NaN according to use case.').
message_description('ESC_INFO', 'ESC information for lower rate streaming. Recommended streaming rate 1Hz. See ESC_STATUS for higher-rate ESC data.').
message_description('ESC_STATUS', 'ESC information for higher rate streaming. Recommended streaming rate is ~10 Hz. Information that changes more slowly is sent in ESC_INFO. It should typically only be streamed on high-bandwidth links (i.e. to a companion computer).').
message_description('WIFI_CONFIG_AP', 'Configure WiFi AP SSID, password, and mode. This message is re-emitted as an acknowledgement by the AP. The message may also be explicitly requested using MAV_CMD_REQUEST_MESSAGE').
message_description('AIS_VESSEL', 'The location and information of an AIS vessel').
message_description('UAVCAN_NODE_STATUS', 'General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message "uavcan.protocol.NodeStatus" for the background information. The UAVCAN specification is available at http://uavcan.org.').
message_description('UAVCAN_NODE_INFO', 'General information describing a particular UAVCAN node. Please refer to the definition of the UAVCAN service "uavcan.protocol.GetNodeInfo" for the background information. This message should be emitted by the system whenever a new node appears online, or an existing node reboots. Additionally, it can be emitted upon request from the other end of the MAVLink channel (see MAV_CMD_UAVCAN_GET_NODE_INFO). It is also not prohibited to emit this message unconditionally at a low frequency. The UAVCAN specification is available at http://uavcan.org.').
message_description('PARAM_EXT_REQUEST_READ', 'Request to read the value of a parameter with either the param_id string id or param_index. PARAM_EXT_VALUE should be emitted in response.').
message_description('PARAM_EXT_REQUEST_LIST', 'Request all parameters of this component. All parameters should be emitted in response as PARAM_EXT_VALUE.').
message_description('PARAM_EXT_VALUE', 'Emit the value of a parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows them to re-request missing parameters after a loss or timeout.').
message_description('PARAM_EXT_SET', 'Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET), when setting a parameter value and the new value is the same as the current value, you will immediately get a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you will accordingly receive a PARAM_ACK_IN_PROGRESS in response.').
message_description('PARAM_EXT_ACK', 'Response from a PARAM_EXT_SET message.').
message_description('OBSTACLE_DISTANCE', 'Obstacle distances in front of the sensor, starting from the left in increment degrees to the right').
message_description('ODOMETRY', 'Odometry message to communicate odometry information with an external interface. Fits ROS REP 147 standard for aerial vehicles (http://www.ros.org/reps/rep-0147.html).').
message_description('TRAJECTORY_REPRESENTATION_WAYPOINTS', 'Describe a trajectory using an array of up-to 5 waypoints in the local frame (MAV_FRAME_LOCAL_NED).').
message_description('TRAJECTORY_REPRESENTATION_BEZIER', 'Describe a trajectory using an array of up-to 5 bezier control points in the local frame (MAV_FRAME_LOCAL_NED).').
message_description('CELLULAR_STATUS', 'Report current used cellular network status').
message_description('ISBD_LINK_STATUS', 'Status of the Iridium SBD link.').
message_description('CELLULAR_CONFIG', 'Configure cellular modems.\n        This message is re-emitted as an acknowledgement by the modem.\n        The message may also be explicitly requested using MAV_CMD_REQUEST_MESSAGE.').
message_description('RAW_RPM', 'RPM sensor data message.').
message_description('UTM_GLOBAL_POSITION', 'The global position resulting from GPS and sensor fusion.').
message_description('DEBUG_FLOAT_ARRAY', 'Large debug/prototyping array. The message uses the maximum available payload for data. The array_id and name fields are used to discriminate between messages in code and in user interfaces (respectively). Do not use in production code.').
message_description('ORBIT_EXECUTION_STATUS', 'Vehicle status report that is sent out while orbit execution is in progress (see MAV_CMD_DO_ORBIT).').
message_description('SMART_BATTERY_INFO', 'Smart Battery information (static/infrequent update). Use for updates from: smart battery to flight stack, flight stack to GCS. Use BATTERY_STATUS for smart battery frequent updates.').
message_description('GENERATOR_STATUS', 'Telemetry of power generation system. Alternator or mechanical generator.').
message_description('ACTUATOR_OUTPUT_STATUS', 'The raw values of the actuator outputs (e.g. on Pixhawk, from MAIN, AUX ports). This message supersedes SERVO_OUTPUT_RAW.').
message_description('TIME_ESTIMATE_TO_TARGET', 'Time/duration estimates for various events and actions given the current vehicle state and position.').
message_description('TUNNEL', 'Message for transporting "arbitrary" variable-length data from one component to another (broadcast is not forbidden, but discouraged). The encoding of the data is usually extension specific, i.e. determined by the source, and is usually not documented as part of the MAVLink specification.').
message_description('CAN_FRAME', 'A forwarded CAN frame as requested by MAV_CMD_CAN_FORWARD.').
message_description('ONBOARD_COMPUTER_STATUS', 'Hardware status sent by an onboard computer.').
message_description('COMPONENT_INFORMATION', '\n        Component information message, which may be requested using MAV_CMD_REQUEST_MESSAGE.\n      ').
message_description('COMPONENT_METADATA', '\n        Component metadata message, which may be requested using MAV_CMD_REQUEST_MESSAGE.\n\n        This contains the MAVLink FTP URI and CRC for the component\'s general metadata file.\n        The file must be hosted on the component, and may be xz compressed.\n        The file CRC can be used for file caching.\n\n        The general metadata file can be read to get the locations of other metadata files (COMP_METADATA_TYPE) and translations, which may be hosted either on the vehicle or the internet.\n        For more information see: https://mavlink.io/en/services/component_information.html.\n\n        Note: Camera components should use CAMERA_INFORMATION instead, and autopilots may use both this message and AUTOPILOT_VERSION.\n      ').
message_description('PLAY_TUNE_V2', 'Play vehicle tone/tune (buzzer). Supersedes message PLAY_TUNE.').
message_description('SUPPORTED_TUNES', 'Tune formats supported by vehicle. This should be emitted as response to MAV_CMD_REQUEST_MESSAGE.').
message_description('EVENT', 'Event message. Each new event from a particular component gets a new sequence number. The same message might be sent multiple times if (re-)requested. Most events are broadcast, some can be specific to a target component (as receivers keep track of the sequence for missed events, all events need to be broadcast. Thus we use destination_component instead of target_component).').
message_description('CURRENT_EVENT_SEQUENCE', 'Regular broadcast for the current latest event sequence number for a component. This is used to check for dropped events.').
message_description('REQUEST_EVENT', 'Request one or more events to be (re-)sent. If first_sequence==last_sequence, only a single event is requested. Note that first_sequence can be larger than last_sequence (because the sequence number can wrap). Each sequence will trigger an EVENT or EVENT_ERROR response.').
message_description('RESPONSE_EVENT_ERROR', 'Response to a REQUEST_EVENT in case of an error (e.g. the event is not available anymore).').
message_description('CANFD_FRAME', 'A forwarded CANFD frame as requested by MAV_CMD_CAN_FORWARD. These are separated from CAN_FRAME as they need different handling (eg. TAO handling)').
message_description('CAN_FILTER_MODIFY', 'Modify the filter of what CAN messages to forward over the mavlink. This can be used to make CAN forwarding work well on low bandwidth links. The filtering is applied on bits 8 to 24 of the CAN id (2nd and 3rd bytes) which corresponds to the DroneCAN message ID for DroneCAN. Filters with more than 16 IDs can be constructed by sending multiple CAN_FILTER_MODIFY messages.').
message_description('WHEEL_DISTANCE', 'Cumulative distance traveled for each reported wheel.').
message_description('WINCH_STATUS', 'Winch status.').
message_description('OPEN_DRONE_ID_BASIC_ID', 'Data for filling the OpenDroneID Basic ID message. This and the below messages are primarily meant for feeding data to/from an OpenDroneID implementation. E.g. https://github.com/opendroneid/opendroneid-core-c. These messages are compatible with the ASTM F3411 Remote ID standard and the ASD-STAN prEN 4709-002 Direct Remote ID standard. Additional information and usage of these messages is documented at https://mavlink.io/en/services/opendroneid.html.').
message_description('OPEN_DRONE_ID_LOCATION', 'Data for filling the OpenDroneID Location message. The float data types are 32-bit IEEE 754. The Location message provides the location, altitude, direction and speed of the aircraft.').
message_description('OPEN_DRONE_ID_AUTHENTICATION', 'Data for filling the OpenDroneID Authentication message. The Authentication Message defines a field that can provide a means of authenticity for the identity of the UAS (Unmanned Aircraft System). The Authentication message can have two different formats. For data page 0, the fields PageCount, Length and TimeStamp are present and AuthData is only 17 bytes. For data page 1 through 15, PageCount, Length and TimeStamp are not present and the size of AuthData is 23 bytes.').
message_description('OPEN_DRONE_ID_SELF_ID', 'Data for filling the OpenDroneID Self ID message. The Self ID Message is an opportunity for the operator to (optionally) declare their identity and purpose of the flight. This message can provide additional information that could reduce the threat profile of a UA (Unmanned Aircraft) flying in a particular area or manner. This message can also be used to provide optional additional clarification in an emergency/remote ID system failure situation.').
message_description('OPEN_DRONE_ID_SYSTEM', 'Data for filling the OpenDroneID System message. The System Message contains general system information including the operator location/altitude and possible aircraft group and/or category/class information.').
message_description('OPEN_DRONE_ID_OPERATOR_ID', 'Data for filling the OpenDroneID Operator ID message, which contains the CAA (Civil Aviation Authority) issued operator ID.').
message_description('OPEN_DRONE_ID_MESSAGE_PACK', 'An OpenDroneID message pack is a container for multiple encoded OpenDroneID messages (i.e. not in the format given for the above message descriptions but after encoding into the compressed OpenDroneID byte format). Used e.g. when transmitting on Bluetooth 5.0 Long Range/Extended Advertising or on WiFi Neighbor Aware Networking or on WiFi Beacon.').
message_description('OPEN_DRONE_ID_ARM_STATUS', 'Transmitter (remote ID system) is enabled and ready to start sending location and other required information. This is streamed by transmitter. A flight controller uses it as a condition to arm.').
message_description('OPEN_DRONE_ID_SYSTEM_UPDATE', 'Update the data in the OPEN_DRONE_ID_SYSTEM message with new location information. This can be sent to update the location information for the operator when no other information in the SYSTEM message has changed. This message allows for efficient operation on radio links which have limited uplink bandwidth while meeting requirements for update frequency of the operator location.').
message_description('HYGROMETER_SENSOR', 'Temperature and humidity from hygrometer.').
message_description('PARAM_ACK_TRANSACTION', 'Response from a PARAM_SET message when it is used in a transaction.').
message_description('AIRSPEED', 'Airspeed information from a sensor.').
message_description('WIFI_NETWORK_INFO', 'Detected WiFi network status information. This message is sent per each WiFi network detected in range with known SSID and general status parameters.').
message_description('FIGURE_EIGHT_EXECUTION_STATUS', '\n        Vehicle status report that is sent out while figure eight execution is in progress (see MAV_CMD_DO_FIGURE_EIGHT).\n        This may typically send at low rates: of the order of 2Hz.\n      ').
message_description('BATTERY_STATUS_V2', 'Battery dynamic information.\n        This should be streamed (nominally at 1Hz).\n        Static/invariant battery information is sent in SMART_BATTERY_INFO.\n        Note that smart batteries should set the MAV_BATTERY_STATUS_FLAGS_CAPACITY_RELATIVE_TO_FULL bit to indicate that supplied capacity values are relative to a battery that is known to be full.\n        Power monitors would not set this bit, indicating that capacity_consumed is relative to drone power-on, and that other values are estimated based on the assumption that the battery was full on power-on.\n      ').
message_description('COMPONENT_INFORMATION_BASIC', 'Basic component information data. Should be requested using MAV_CMD_REQUEST_MESSAGE on startup, or when required.').
message_description('GROUP_START', 'Emitted during mission execution when control reaches MAV_CMD_GROUP_START.').
message_description('GROUP_END', 'Emitted during mission execution when control reaches MAV_CMD_GROUP_END.').
message_description('AVAILABLE_MODES', 'Get information about a particular flight modes.\n        The message can be enumerated or requested for a particular mode using MAV_CMD_REQUEST_MESSAGE.\n        Specify 0 in param2 to request that the message is emitted for all available modes or the specific index for just one mode.\n        The modes must be available/settable for the current vehicle/frame type.\n        Each modes should only be emitted once (even if it is both standard and custom).\n      ').
message_description('CURRENT_MODE', 'Get the current mode.\n        This should be emitted on any mode change, and broadcast at low rate (nominally 0.5 Hz).\n        It may be requested using MAV_CMD_REQUEST_MESSAGE.\n      ').
message_description('AVAILABLE_MODES_MONITOR', 'A change to the sequence number indicates that the set of AVAILABLE_MODES has changed.\n        A receiver must re-request all available modes whenever the sequence number changes.\n        This is only emitted after the first change and should then be broadcast at low rate (nominally 0.3 Hz) and on change.\n      ').
message_description('TARGET_ABSOLUTE', 'Current motion information from sensors on a target').
message_description('TARGET_RELATIVE', 'The location of a target measured by MAV\'s onboard sensors. ').
message_description('ICAROUS_HEARTBEAT', 'ICAROUS heartbeat').
message_description('ICAROUS_KINEMATIC_BANDS', 'Kinematic multi bands (track) output from Daidalus').
message_description('HEARTBEAT', 'The heartbeat message shows that a system or component is present and responding. The type and autopilot fields (along with the message component id), allow the receiving system to treat further messages from this system appropriately (e.g. by laying out the user interface based on the autopilot). This microservice is documented at https://mavlink.io/en/services/heartbeat.html').
message_description('PROTOCOL_VERSION', 'Version and capability of protocol version. This message can be requested with MAV_CMD_REQUEST_MESSAGE and is used as part of the handshaking to establish which MAVLink version should be used on the network. Every node should respond to a request for PROTOCOL_VERSION to enable the handshaking. Library implementers should consider adding this into the default decoding state machine to allow the protocol core to respond directly.').
message_description('ARRAY_TEST_0', 'Array test #0.').
message_description('ARRAY_TEST_1', 'Array test #1.').
message_description('ARRAY_TEST_3', 'Array test #3.').
message_description('ARRAY_TEST_4', 'Array test #4.').
message_description('ARRAY_TEST_5', 'Array test #5.').
message_description('ARRAY_TEST_6', 'Array test #6.').
message_description('ARRAY_TEST_7', 'Array test #7.').
message_description('ARRAY_TEST_8', 'Array test #8.').
message_description('TEST_TYPES', 'Test all field types').
message_description('NAV_FILTER_BIAS', 'Accelerometer and Gyro biases from the navigation filter').
message_description('RADIO_CALIBRATION', 'Complete set of calibration parameters for the radio').
message_description('UALBERTA_SYS_STATUS', 'System status specific to ualberta uav').
message_description('UAVIONIX_ADSB_OUT_CFG', 'Static data to configure the ADS-B transponder (send within 10 sec of a POR and every 10 sec thereafter)').
message_description('UAVIONIX_ADSB_OUT_DYNAMIC', 'Dynamic data used to generate ADS-B out transponder data (send at 5Hz)').
message_description('UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT', 'Transceiver heartbeat with health report (updated every 10s)').
message_description('STORM32_GIMBAL_MANAGER_INFORMATION', 'Information about a gimbal manager. This message should be requested by a ground station using MAV_CMD_REQUEST_MESSAGE. It mirrors some fields of the GIMBAL_DEVICE_INFORMATION message, but not all. If the additional information is desired, also GIMBAL_DEVICE_INFORMATION should be requested.').
message_description('STORM32_GIMBAL_MANAGER_STATUS', 'Message reporting the current status of a gimbal manager. This message should be broadcast at a low regular rate (e.g. 1 Hz, may be increase momentarily to e.g. 5 Hz for a period of 1 sec after a change).').
message_description('STORM32_GIMBAL_MANAGER_CONTROL', 'Message to a gimbal manager to control the gimbal attitude. Angles and rates can be set to NaN according to use case. A gimbal device is never to react to this message.').
message_description('STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW', 'Message to a gimbal manager to control the gimbal tilt and pan angles. Angles and rates can be set to NaN according to use case. A gimbal device is never to react to this message.').
message_description('STORM32_GIMBAL_MANAGER_CORRECT_ROLL', 'Message to a gimbal manager to correct the gimbal roll angle. This message is typically used to manually correct for a tilted horizon in operation. A gimbal device is never to react to this message.').
message_description('QSHOT_STATUS', 'Information about the shot operation.').
message_description('RADIO_RC_CHANNELS', 'Radio channels. Supports up to 24 channels. Channel values are in centerd 13 bit format. Range is [-4096,4096], center is 0. Conversion to PWM is x * 5/32 + 1500. Should be emitted only by components with component id MAV_COMP_ID_TELEMETRY_RADIO.').
message_description('RADIO_LINK_STATS', 'Radio link statistics. Should be emitted only by components with component id MAV_COMP_ID_TELEMETRY_RADIO. Per default, rssi values are in MAVLink units: 0 represents weakest signal, 254 represents maximum signal; can be changed to dBm with the flag RADIO_LINK_STATS_FLAGS_RSSI_DBM.').
message_description('FRSKY_PASSTHROUGH_ARRAY', 'Frsky SPort passthrough multi packet container.').
message_description('PARAM_VALUE_ARRAY', 'Parameter multi param value container.').
message_description('AVSS_PRS_SYS_STATUS', ' AVSS PRS system status.').
message_description('AVSS_DRONE_POSITION', ' Drone position.').
message_description('AVSS_DRONE_IMU', ' Drone IMU data. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).').
message_description('AVSS_DRONE_OPERATION_MODE', ' Drone operation mode.').
message_description('CUBEPILOT_RAW_RC', 'Raw RC Data').
message_description('HERELINK_VIDEO_STREAM_INFORMATION', 'Information about video stream').
message_description('HERELINK_TELEM', 'Herelink Telemetry').
message_description('CUBEPILOT_FIRMWARE_UPDATE_START', 'Start firmware update with encapsulated data.').
message_description('CUBEPILOT_FIRMWARE_UPDATE_RESP', 'offset response to encapsulated data.').
message_description('AIRLINK_AUTH', 'Authorization package').
message_description('AIRLINK_AUTH_RESPONSE', 'Response to the authorization request').

:- dynamic message_extensions/2.

message_extensions('MEMINFO', [freemem32]).
message_extensions('MOUNT_STATUS', [mount_mode]).
message_extensions('CAMERA_FEEDBACK', [completed_captures]).
message_extensions('EKF_STATUS_REPORT', [airspeed_variance]).
message_extensions('PID_TUNING', ['PDmod', 'SRate']).
message_extensions('DEVICE_OP_READ', [bank]).
message_extensions('DEVICE_OP_READ_REPLY', [bank]).
message_extensions('DEVICE_OP_WRITE', [bank]).
message_extensions('SYS_STATUS', [onboard_control_sensors_health_extended, onboard_control_sensors_enabled_extended, onboard_control_sensors_present_extended]).
message_extensions('GPS_RAW_INT', [yaw, hdg_acc, vel_acc, v_acc, h_acc, alt_ellipsoid]).
message_extensions('SCALED_IMU', [temperature]).
message_extensions('RAW_IMU', [temperature, id]).
message_extensions('SCALED_PRESSURE', [temperature_press_diff]).
message_extensions('ATTITUDE_QUATERNION', [repr_offset_q]).
message_extensions('SERVO_OUTPUT_RAW', [servo16_raw, servo15_raw, servo14_raw, servo13_raw, servo12_raw, servo11_raw, servo10_raw, servo9_raw]).
message_extensions('MISSION_REQUEST_PARTIAL_LIST', [mission_type]).
message_extensions('MISSION_WRITE_PARTIAL_LIST', [mission_type]).
message_extensions('MISSION_ITEM', [mission_type]).
message_extensions('MISSION_REQUEST', [mission_type]).
message_extensions('MISSION_CURRENT', [mission_mode, mission_state, total]).
message_extensions('MISSION_REQUEST_LIST', [mission_type]).
message_extensions('MISSION_COUNT', [mission_type]).
message_extensions('MISSION_CLEAR_ALL', [mission_type]).
message_extensions('MISSION_ACK', [mission_type]).
message_extensions('SET_GPS_GLOBAL_ORIGIN', [time_usec]).
message_extensions('GPS_GLOBAL_ORIGIN', [time_usec]).
message_extensions('MISSION_REQUEST_INT', [mission_type]).
message_extensions('MANUAL_CONTROL', [aux6, aux5, aux4, aux3, aux2, aux1, t, s, enabled_extensions, buttons2]).
message_extensions('RC_CHANNELS_OVERRIDE', [chan18_raw, chan17_raw, chan16_raw, chan15_raw, chan14_raw, chan13_raw, chan12_raw, chan11_raw, chan10_raw, chan9_raw]).
message_extensions('MISSION_ITEM_INT', [mission_type]).
message_extensions('COMMAND_ACK', [target_component, target_system, result_param2, progress]).
message_extensions('SET_ATTITUDE_TARGET', [thrust_body]).
message_extensions('OPTICAL_FLOW', [flow_rate_y, flow_rate_x]).
message_extensions('GLOBAL_VISION_POSITION_ESTIMATE', [reset_counter, covariance]).
message_extensions('VISION_POSITION_ESTIMATE', [reset_counter, covariance]).
message_extensions('VISION_SPEED_ESTIMATE', [reset_counter, covariance]).
message_extensions('VICON_POSITION_ESTIMATE', [covariance]).
message_extensions('HIGHRES_IMU', [id]).
message_extensions('HIL_SENSOR', [id]).
message_extensions('SIM_STATE', [lon_int, lat_int]).
message_extensions('TIMESYNC', [target_component, target_system]).
message_extensions('HIL_GPS', [yaw, id]).
message_extensions('SCALED_IMU2', [temperature]).
message_extensions('GPS2_RAW', [hdg_acc, vel_acc, v_acc, h_acc, alt_ellipsoid, yaw]).
message_extensions('SERIAL_CONTROL', [target_component, target_system]).
message_extensions('SCALED_IMU3', [temperature]).
message_extensions('DISTANCE_SENSOR', [signal_quality, quaternion, vertical_fov, horizontal_fov]).
message_extensions('SCALED_PRESSURE2', [temperature_press_diff]).
message_extensions('ATT_POS_MOCAP', [covariance]).
message_extensions('SCALED_PRESSURE3', [temperature_press_diff]).
message_extensions('BATTERY_STATUS', [fault_bitmask, mode, voltages_ext, charge_state, time_remaining]).
message_extensions('AUTOPILOT_VERSION', [uid2]).
message_extensions('LANDING_TARGET', [position_valid, type, q, z, y, x]).
message_extensions('FENCE_STATUS', [breach_mitigation]).
message_extensions('MAG_CAL_REPORT', [scale_factor, new_orientation, old_orientation, orientation_confidence]).
message_extensions('EFI_STATUS', [fuel_pressure, ignition_voltage]).
message_extensions('GPS_INPUT', [yaw]).
message_extensions('HOME_POSITION', [time_usec]).
message_extensions('SET_HOME_POSITION', [time_usec]).
message_extensions('STATUSTEXT', [chunk_seq, id]).
message_extensions('PLAY_TUNE', [tune2]).
message_extensions('CAMERA_INFORMATION', [gimbal_device_id]).
message_extensions('CAMERA_SETTINGS', [focusLevel, zoomLevel]).
message_extensions('STORAGE_INFORMATION', [storage_usage, name, type]).
message_extensions('CAMERA_CAPTURE_STATUS', [image_count]).
message_extensions('MOUNT_ORIENTATION', [yaw_absolute]).
message_extensions('GIMBAL_DEVICE_INFORMATION', [gimbal_device_id]).
message_extensions('GIMBAL_DEVICE_ATTITUDE_STATUS', [gimbal_device_id, delta_yaw_velocity, delta_yaw]).
message_extensions('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', [angular_velocity_z]).
message_extensions('WIFI_CONFIG_AP', [response, mode]).
message_extensions('OBSTACLE_DISTANCE', [frame, angle_offset, increment_f]).
message_extensions('ODOMETRY', [quality, estimator_type, reset_counter]).
message_extensions('DEBUG_FLOAT_ARRAY', [data]).
message_extensions('SMART_BATTERY_INFO', [manufacture_date, discharge_maximum_burst_current, discharge_maximum_current, cells_in_series, charging_maximum_voltage]).
message_extensions('RADIO_RC_CHANNELS', [channels]).

:- dynamic message_field/4.

message_field('SENSOR_OFFSETS', mag_ofs_x, int16_t, []).
message_field('SENSOR_OFFSETS', mag_ofs_y, int16_t, []).
message_field('SENSOR_OFFSETS', mag_ofs_z, int16_t, []).
message_field('SENSOR_OFFSETS', mag_declination, float, [units=rad]).
message_field('SENSOR_OFFSETS', raw_press, int32_t, []).
message_field('SENSOR_OFFSETS', raw_temp, int32_t, []).
message_field('SENSOR_OFFSETS', gyro_cal_x, float, []).
message_field('SENSOR_OFFSETS', gyro_cal_y, float, []).
message_field('SENSOR_OFFSETS', gyro_cal_z, float, []).
message_field('SENSOR_OFFSETS', accel_cal_x, float, []).
message_field('SENSOR_OFFSETS', accel_cal_y, float, []).
message_field('SENSOR_OFFSETS', accel_cal_z, float, []).
message_field('SET_MAG_OFFSETS', target_system, uint8_t, []).
message_field('SET_MAG_OFFSETS', target_component, uint8_t, []).
message_field('SET_MAG_OFFSETS', mag_ofs_x, int16_t, []).
message_field('SET_MAG_OFFSETS', mag_ofs_y, int16_t, []).
message_field('SET_MAG_OFFSETS', mag_ofs_z, int16_t, []).
message_field('MEMINFO', brkval, uint16_t, []).
message_field('MEMINFO', freemem, uint16_t, [units=bytes]).
message_field('MEMINFO', freemem32, uint32_t, [units=bytes]).
message_field('AP_ADC', adc1, uint16_t, []).
message_field('AP_ADC', adc2, uint16_t, []).
message_field('AP_ADC', adc3, uint16_t, []).
message_field('AP_ADC', adc4, uint16_t, []).
message_field('AP_ADC', adc5, uint16_t, []).
message_field('AP_ADC', adc6, uint16_t, []).
message_field('DIGICAM_CONFIGURE', target_system, uint8_t, []).
message_field('DIGICAM_CONFIGURE', target_component, uint8_t, []).
message_field('DIGICAM_CONFIGURE', mode, uint8_t, []).
message_field('DIGICAM_CONFIGURE', shutter_speed, uint16_t, []).
message_field('DIGICAM_CONFIGURE', aperture, uint8_t, []).
message_field('DIGICAM_CONFIGURE', iso, uint8_t, []).
message_field('DIGICAM_CONFIGURE', exposure_type, uint8_t, []).
message_field('DIGICAM_CONFIGURE', command_id, uint8_t, []).
message_field('DIGICAM_CONFIGURE', engine_cut_off, uint8_t, [units=ds]).
message_field('DIGICAM_CONFIGURE', extra_param, uint8_t, []).
message_field('DIGICAM_CONFIGURE', extra_value, float, []).
message_field('DIGICAM_CONTROL', target_system, uint8_t, []).
message_field('DIGICAM_CONTROL', target_component, uint8_t, []).
message_field('DIGICAM_CONTROL', session, uint8_t, []).
message_field('DIGICAM_CONTROL', zoom_pos, uint8_t, []).
message_field('DIGICAM_CONTROL', zoom_step, int8_t, []).
message_field('DIGICAM_CONTROL', focus_lock, uint8_t, []).
message_field('DIGICAM_CONTROL', shot, uint8_t, []).
message_field('DIGICAM_CONTROL', command_id, uint8_t, []).
message_field('DIGICAM_CONTROL', extra_param, uint8_t, []).
message_field('DIGICAM_CONTROL', extra_value, float, []).
message_field('MOUNT_CONFIGURE', target_system, uint8_t, []).
message_field('MOUNT_CONFIGURE', target_component, uint8_t, []).
message_field('MOUNT_CONFIGURE', mount_mode, uint8_t, [enum='MAV_MOUNT_MODE']).
message_field('MOUNT_CONFIGURE', stab_roll, uint8_t, []).
message_field('MOUNT_CONFIGURE', stab_pitch, uint8_t, []).
message_field('MOUNT_CONFIGURE', stab_yaw, uint8_t, []).
message_field('MOUNT_CONTROL', target_system, uint8_t, []).
message_field('MOUNT_CONTROL', target_component, uint8_t, []).
message_field('MOUNT_CONTROL', input_a, int32_t, []).
message_field('MOUNT_CONTROL', input_b, int32_t, []).
message_field('MOUNT_CONTROL', input_c, int32_t, []).
message_field('MOUNT_CONTROL', save_position, uint8_t, []).
message_field('MOUNT_STATUS', target_system, uint8_t, []).
message_field('MOUNT_STATUS', target_component, uint8_t, []).
message_field('MOUNT_STATUS', pointing_a, int32_t, [units=cdeg]).
message_field('MOUNT_STATUS', pointing_b, int32_t, [units=cdeg]).
message_field('MOUNT_STATUS', pointing_c, int32_t, [units=cdeg]).
message_field('MOUNT_STATUS', mount_mode, uint8_t, [enum='MAV_MOUNT_MODE']).
message_field('FENCE_POINT', target_system, uint8_t, []).
message_field('FENCE_POINT', target_component, uint8_t, []).
message_field('FENCE_POINT', idx, uint8_t, []).
message_field('FENCE_POINT', count, uint8_t, []).
message_field('FENCE_POINT', lat, float, [units=deg]).
message_field('FENCE_POINT', lng, float, [units=deg]).
message_field('FENCE_FETCH_POINT', target_system, uint8_t, []).
message_field('FENCE_FETCH_POINT', target_component, uint8_t, []).
message_field('FENCE_FETCH_POINT', idx, uint8_t, []).
message_field('AHRS', omegaIx, float, [units='rad/s']).
message_field('AHRS', omegaIy, float, [units='rad/s']).
message_field('AHRS', omegaIz, float, [units='rad/s']).
message_field('AHRS', accel_weight, float, []).
message_field('AHRS', renorm_val, float, []).
message_field('AHRS', error_rp, float, []).
message_field('AHRS', error_yaw, float, []).
message_field('SIMSTATE', roll, float, [units=rad]).
message_field('SIMSTATE', pitch, float, [units=rad]).
message_field('SIMSTATE', yaw, float, [units=rad]).
message_field('SIMSTATE', xacc, float, [units='m/s/s']).
message_field('SIMSTATE', yacc, float, [units='m/s/s']).
message_field('SIMSTATE', zacc, float, [units='m/s/s']).
message_field('SIMSTATE', xgyro, float, [units='rad/s']).
message_field('SIMSTATE', ygyro, float, [units='rad/s']).
message_field('SIMSTATE', zgyro, float, [units='rad/s']).
message_field('SIMSTATE', lat, int32_t, [units=degE7]).
message_field('SIMSTATE', lng, int32_t, [units=degE7]).
message_field('HWSTATUS', 'Vcc', uint16_t, [units=mV]).
message_field('HWSTATUS', 'I2Cerr', uint8_t, []).
message_field('RADIO', rssi, uint8_t, []).
message_field('RADIO', remrssi, uint8_t, []).
message_field('RADIO', txbuf, uint8_t, [units='%']).
message_field('RADIO', noise, uint8_t, []).
message_field('RADIO', remnoise, uint8_t, []).
message_field('RADIO', rxerrors, uint16_t, []).
message_field('RADIO', fixed, uint16_t, []).
message_field('LIMITS_STATUS', limits_state, uint8_t, [enum='LIMITS_STATE']).
message_field('LIMITS_STATUS', last_trigger, uint32_t, [units=ms]).
message_field('LIMITS_STATUS', last_action, uint32_t, [units=ms]).
message_field('LIMITS_STATUS', last_recovery, uint32_t, [units=ms]).
message_field('LIMITS_STATUS', last_clear, uint32_t, [units=ms]).
message_field('LIMITS_STATUS', breach_count, uint16_t, []).
message_field('LIMITS_STATUS', mods_enabled, uint8_t, [enum='LIMIT_MODULE', display=bitmask]).
message_field('LIMITS_STATUS', mods_required, uint8_t, [enum='LIMIT_MODULE', display=bitmask]).
message_field('LIMITS_STATUS', mods_triggered, uint8_t, [enum='LIMIT_MODULE', display=bitmask]).
message_field('WIND', direction, float, [units=deg]).
message_field('WIND', speed, float, [units='m/s']).
message_field('WIND', speed_z, float, [units='m/s']).
message_field('DATA16', type, uint8_t, []).
message_field('DATA16', len, uint8_t, [units=bytes]).
message_field('DATA16', data, 'uint8_t[16]', []).
message_field('DATA32', type, uint8_t, []).
message_field('DATA32', len, uint8_t, [units=bytes]).
message_field('DATA32', data, 'uint8_t[32]', []).
message_field('DATA64', type, uint8_t, []).
message_field('DATA64', len, uint8_t, [units=bytes]).
message_field('DATA64', data, 'uint8_t[64]', []).
message_field('DATA96', type, uint8_t, []).
message_field('DATA96', len, uint8_t, [units=bytes]).
message_field('DATA96', data, 'uint8_t[96]', []).
message_field('RANGEFINDER', distance, float, [units=m]).
message_field('RANGEFINDER', voltage, float, [units='V']).
message_field('AIRSPEED_AUTOCAL', vx, float, [units='m/s']).
message_field('AIRSPEED_AUTOCAL', vy, float, [units='m/s']).
message_field('AIRSPEED_AUTOCAL', vz, float, [units='m/s']).
message_field('AIRSPEED_AUTOCAL', diff_pressure, float, [units='Pa']).
message_field('AIRSPEED_AUTOCAL', 'EAS2TAS', float, []).
message_field('AIRSPEED_AUTOCAL', ratio, float, []).
message_field('AIRSPEED_AUTOCAL', state_x, float, []).
message_field('AIRSPEED_AUTOCAL', state_y, float, []).
message_field('AIRSPEED_AUTOCAL', state_z, float, []).
message_field('AIRSPEED_AUTOCAL', 'Pax', float, []).
message_field('AIRSPEED_AUTOCAL', 'Pby', float, []).
message_field('AIRSPEED_AUTOCAL', 'Pcz', float, []).
message_field('RALLY_POINT', target_system, uint8_t, []).
message_field('RALLY_POINT', target_component, uint8_t, []).
message_field('RALLY_POINT', idx, uint8_t, []).
message_field('RALLY_POINT', count, uint8_t, []).
message_field('RALLY_POINT', lat, int32_t, [units=degE7]).
message_field('RALLY_POINT', lng, int32_t, [units=degE7]).
message_field('RALLY_POINT', alt, int16_t, [units=m]).
message_field('RALLY_POINT', break_alt, int16_t, [units=m]).
message_field('RALLY_POINT', land_dir, uint16_t, [units=cdeg]).
message_field('RALLY_POINT', flags, uint8_t, [enum='RALLY_FLAGS', display=bitmask]).
message_field('RALLY_FETCH_POINT', target_system, uint8_t, []).
message_field('RALLY_FETCH_POINT', target_component, uint8_t, []).
message_field('RALLY_FETCH_POINT', idx, uint8_t, []).
message_field('COMPASSMOT_STATUS', throttle, uint16_t, [units='d%']).
message_field('COMPASSMOT_STATUS', current, float, [units='A']).
message_field('COMPASSMOT_STATUS', interference, uint16_t, [units='%']).
message_field('COMPASSMOT_STATUS', 'CompensationX', float, []).
message_field('COMPASSMOT_STATUS', 'CompensationY', float, []).
message_field('COMPASSMOT_STATUS', 'CompensationZ', float, []).
message_field('AHRS2', roll, float, [units=rad]).
message_field('AHRS2', pitch, float, [units=rad]).
message_field('AHRS2', yaw, float, [units=rad]).
message_field('AHRS2', altitude, float, [units=m]).
message_field('AHRS2', lat, int32_t, [units=degE7]).
message_field('AHRS2', lng, int32_t, [units=degE7]).
message_field('CAMERA_STATUS', time_usec, uint64_t, [units=us]).
message_field('CAMERA_STATUS', target_system, uint8_t, []).
message_field('CAMERA_STATUS', cam_idx, uint8_t, []).
message_field('CAMERA_STATUS', img_idx, uint16_t, []).
message_field('CAMERA_STATUS', event_id, uint8_t, [enum='CAMERA_STATUS_TYPES']).
message_field('CAMERA_STATUS', p1, float, []).
message_field('CAMERA_STATUS', p2, float, []).
message_field('CAMERA_STATUS', p3, float, []).
message_field('CAMERA_STATUS', p4, float, []).
message_field('CAMERA_FEEDBACK', time_usec, uint64_t, [units=us]).
message_field('CAMERA_FEEDBACK', target_system, uint8_t, []).
message_field('CAMERA_FEEDBACK', cam_idx, uint8_t, []).
message_field('CAMERA_FEEDBACK', img_idx, uint16_t, []).
message_field('CAMERA_FEEDBACK', lat, int32_t, [units=degE7]).
message_field('CAMERA_FEEDBACK', lng, int32_t, [units=degE7]).
message_field('CAMERA_FEEDBACK', alt_msl, float, [units=m]).
message_field('CAMERA_FEEDBACK', alt_rel, float, [units=m]).
message_field('CAMERA_FEEDBACK', roll, float, [units=deg]).
message_field('CAMERA_FEEDBACK', pitch, float, [units=deg]).
message_field('CAMERA_FEEDBACK', yaw, float, [units=deg]).
message_field('CAMERA_FEEDBACK', foc_len, float, [units=mm]).
message_field('CAMERA_FEEDBACK', flags, uint8_t, [enum='CAMERA_FEEDBACK_FLAGS']).
message_field('CAMERA_FEEDBACK', completed_captures, uint16_t, []).
message_field('BATTERY2', voltage, uint16_t, [units=mV]).
message_field('BATTERY2', current_battery, int16_t, [units=cA]).
message_field('AHRS3', roll, float, [units=rad]).
message_field('AHRS3', pitch, float, [units=rad]).
message_field('AHRS3', yaw, float, [units=rad]).
message_field('AHRS3', altitude, float, [units=m]).
message_field('AHRS3', lat, int32_t, [units=degE7]).
message_field('AHRS3', lng, int32_t, [units=degE7]).
message_field('AHRS3', v1, float, []).
message_field('AHRS3', v2, float, []).
message_field('AHRS3', v3, float, []).
message_field('AHRS3', v4, float, []).
message_field('AUTOPILOT_VERSION_REQUEST', target_system, uint8_t, []).
message_field('AUTOPILOT_VERSION_REQUEST', target_component, uint8_t, []).
message_field('REMOTE_LOG_DATA_BLOCK', target_system, uint8_t, []).
message_field('REMOTE_LOG_DATA_BLOCK', target_component, uint8_t, []).
message_field('REMOTE_LOG_DATA_BLOCK', seqno, uint32_t, [enum='MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS']).
message_field('REMOTE_LOG_DATA_BLOCK', data, 'uint8_t[200]', []).
message_field('REMOTE_LOG_BLOCK_STATUS', target_system, uint8_t, []).
message_field('REMOTE_LOG_BLOCK_STATUS', target_component, uint8_t, []).
message_field('REMOTE_LOG_BLOCK_STATUS', seqno, uint32_t, []).
message_field('REMOTE_LOG_BLOCK_STATUS', status, uint8_t, [enum='MAV_REMOTE_LOG_DATA_BLOCK_STATUSES']).
message_field('LED_CONTROL', target_system, uint8_t, []).
message_field('LED_CONTROL', target_component, uint8_t, []).
message_field('LED_CONTROL', instance, uint8_t, []).
message_field('LED_CONTROL', pattern, uint8_t, []).
message_field('LED_CONTROL', custom_len, uint8_t, []).
message_field('LED_CONTROL', custom_bytes, 'uint8_t[24]', []).
message_field('MAG_CAL_PROGRESS', compass_id, uint8_t, [instance=true]).
message_field('MAG_CAL_PROGRESS', cal_mask, uint8_t, [display=bitmask]).
message_field('MAG_CAL_PROGRESS', cal_status, uint8_t, [enum='MAG_CAL_STATUS']).
message_field('MAG_CAL_PROGRESS', attempt, uint8_t, []).
message_field('MAG_CAL_PROGRESS', completion_pct, uint8_t, [units='%']).
message_field('MAG_CAL_PROGRESS', completion_mask, 'uint8_t[10]', []).
message_field('MAG_CAL_PROGRESS', direction_x, float, []).
message_field('MAG_CAL_PROGRESS', direction_y, float, []).
message_field('MAG_CAL_PROGRESS', direction_z, float, []).
message_field('EKF_STATUS_REPORT', flags, uint16_t, [enum='EKF_STATUS_FLAGS', display=bitmask]).
message_field('EKF_STATUS_REPORT', velocity_variance, float, []).
message_field('EKF_STATUS_REPORT', pos_horiz_variance, float, []).
message_field('EKF_STATUS_REPORT', pos_vert_variance, float, []).
message_field('EKF_STATUS_REPORT', compass_variance, float, []).
message_field('EKF_STATUS_REPORT', terrain_alt_variance, float, []).
message_field('EKF_STATUS_REPORT', airspeed_variance, float, []).
message_field('PID_TUNING', axis, uint8_t, [enum='PID_TUNING_AXIS', instance=true]).
message_field('PID_TUNING', desired, float, []).
message_field('PID_TUNING', achieved, float, []).
message_field('PID_TUNING', 'FF', float, []).
message_field('PID_TUNING', 'P', float, []).
message_field('PID_TUNING', 'I', float, []).
message_field('PID_TUNING', 'D', float, []).
message_field('PID_TUNING', 'SRate', float, []).
message_field('PID_TUNING', 'PDmod', float, []).
message_field('DEEPSTALL', landing_lat, int32_t, [units=degE7]).
message_field('DEEPSTALL', landing_lon, int32_t, [units=degE7]).
message_field('DEEPSTALL', path_lat, int32_t, [units=degE7]).
message_field('DEEPSTALL', path_lon, int32_t, [units=degE7]).
message_field('DEEPSTALL', arc_entry_lat, int32_t, [units=degE7]).
message_field('DEEPSTALL', arc_entry_lon, int32_t, [units=degE7]).
message_field('DEEPSTALL', altitude, float, [units=m]).
message_field('DEEPSTALL', expected_travel_distance, float, [units=m]).
message_field('DEEPSTALL', cross_track_error, float, [units=m]).
message_field('DEEPSTALL', stage, uint8_t, [enum='DEEPSTALL_STAGE']).
message_field('GIMBAL_REPORT', target_system, uint8_t, []).
message_field('GIMBAL_REPORT', target_component, uint8_t, []).
message_field('GIMBAL_REPORT', delta_time, float, [units=s]).
message_field('GIMBAL_REPORT', delta_angle_x, float, [units=rad]).
message_field('GIMBAL_REPORT', delta_angle_y, float, [units=rad]).
message_field('GIMBAL_REPORT', delta_angle_z, float, [units=rad]).
message_field('GIMBAL_REPORT', delta_velocity_x, float, [units='m/s']).
message_field('GIMBAL_REPORT', delta_velocity_y, float, [units='m/s']).
message_field('GIMBAL_REPORT', delta_velocity_z, float, [units='m/s']).
message_field('GIMBAL_REPORT', joint_roll, float, [units=rad]).
message_field('GIMBAL_REPORT', joint_el, float, [units=rad]).
message_field('GIMBAL_REPORT', joint_az, float, [units=rad]).
message_field('GIMBAL_CONTROL', target_system, uint8_t, []).
message_field('GIMBAL_CONTROL', target_component, uint8_t, []).
message_field('GIMBAL_CONTROL', demanded_rate_x, float, [units='rad/s']).
message_field('GIMBAL_CONTROL', demanded_rate_y, float, [units='rad/s']).
message_field('GIMBAL_CONTROL', demanded_rate_z, float, [units='rad/s']).
message_field('GIMBAL_TORQUE_CMD_REPORT', target_system, uint8_t, []).
message_field('GIMBAL_TORQUE_CMD_REPORT', target_component, uint8_t, []).
message_field('GIMBAL_TORQUE_CMD_REPORT', rl_torque_cmd, int16_t, []).
message_field('GIMBAL_TORQUE_CMD_REPORT', el_torque_cmd, int16_t, []).
message_field('GIMBAL_TORQUE_CMD_REPORT', az_torque_cmd, int16_t, []).
message_field('GOPRO_HEARTBEAT', status, uint8_t, [enum='GOPRO_HEARTBEAT_STATUS']).
message_field('GOPRO_HEARTBEAT', capture_mode, uint8_t, [enum='GOPRO_CAPTURE_MODE']).
message_field('GOPRO_HEARTBEAT', flags, uint8_t, [enum='GOPRO_HEARTBEAT_FLAGS', display=bitmask]).
message_field('GOPRO_GET_REQUEST', target_system, uint8_t, []).
message_field('GOPRO_GET_REQUEST', target_component, uint8_t, []).
message_field('GOPRO_GET_REQUEST', cmd_id, uint8_t, [enum='GOPRO_COMMAND']).
message_field('GOPRO_GET_RESPONSE', cmd_id, uint8_t, [enum='GOPRO_COMMAND']).
message_field('GOPRO_GET_RESPONSE', status, uint8_t, [enum='GOPRO_REQUEST_STATUS']).
message_field('GOPRO_GET_RESPONSE', value, 'uint8_t[4]', []).
message_field('GOPRO_SET_REQUEST', target_system, uint8_t, []).
message_field('GOPRO_SET_REQUEST', target_component, uint8_t, []).
message_field('GOPRO_SET_REQUEST', cmd_id, uint8_t, [enum='GOPRO_COMMAND']).
message_field('GOPRO_SET_REQUEST', value, 'uint8_t[4]', []).
message_field('GOPRO_SET_RESPONSE', cmd_id, uint8_t, [enum='GOPRO_COMMAND']).
message_field('GOPRO_SET_RESPONSE', status, uint8_t, [enum='GOPRO_REQUEST_STATUS']).
message_field('RPM', rpm1, float, []).
message_field('RPM', rpm2, float, []).
message_field('DEVICE_OP_READ', target_system, uint8_t, []).
message_field('DEVICE_OP_READ', target_component, uint8_t, []).
message_field('DEVICE_OP_READ', request_id, uint32_t, []).
message_field('DEVICE_OP_READ', bustype, uint8_t, [enum='DEVICE_OP_BUSTYPE']).
message_field('DEVICE_OP_READ', bus, uint8_t, []).
message_field('DEVICE_OP_READ', address, uint8_t, []).
message_field('DEVICE_OP_READ', busname, 'char[40]', []).
message_field('DEVICE_OP_READ', regstart, uint8_t, []).
message_field('DEVICE_OP_READ', count, uint8_t, []).
message_field('DEVICE_OP_READ', bank, uint8_t, []).
message_field('DEVICE_OP_READ_REPLY', request_id, uint32_t, []).
message_field('DEVICE_OP_READ_REPLY', result, uint8_t, []).
message_field('DEVICE_OP_READ_REPLY', regstart, uint8_t, []).
message_field('DEVICE_OP_READ_REPLY', count, uint8_t, []).
message_field('DEVICE_OP_READ_REPLY', data, 'uint8_t[128]', []).
message_field('DEVICE_OP_READ_REPLY', bank, uint8_t, []).
message_field('DEVICE_OP_WRITE', target_system, uint8_t, []).
message_field('DEVICE_OP_WRITE', target_component, uint8_t, []).
message_field('DEVICE_OP_WRITE', request_id, uint32_t, []).
message_field('DEVICE_OP_WRITE', bustype, uint8_t, [enum='DEVICE_OP_BUSTYPE']).
message_field('DEVICE_OP_WRITE', bus, uint8_t, []).
message_field('DEVICE_OP_WRITE', address, uint8_t, []).
message_field('DEVICE_OP_WRITE', busname, 'char[40]', []).
message_field('DEVICE_OP_WRITE', regstart, uint8_t, []).
message_field('DEVICE_OP_WRITE', count, uint8_t, []).
message_field('DEVICE_OP_WRITE', data, 'uint8_t[128]', []).
message_field('DEVICE_OP_WRITE', bank, uint8_t, []).
message_field('DEVICE_OP_WRITE_REPLY', request_id, uint32_t, []).
message_field('DEVICE_OP_WRITE_REPLY', result, uint8_t, []).
message_field('ADAP_TUNING', axis, uint8_t, [enum='PID_TUNING_AXIS', instance=true]).
message_field('ADAP_TUNING', desired, float, [units='deg/s']).
message_field('ADAP_TUNING', achieved, float, [units='deg/s']).
message_field('ADAP_TUNING', error, float, []).
message_field('ADAP_TUNING', theta, float, []).
message_field('ADAP_TUNING', omega, float, []).
message_field('ADAP_TUNING', sigma, float, []).
message_field('ADAP_TUNING', theta_dot, float, []).
message_field('ADAP_TUNING', omega_dot, float, []).
message_field('ADAP_TUNING', sigma_dot, float, []).
message_field('ADAP_TUNING', f, float, []).
message_field('ADAP_TUNING', f_dot, float, []).
message_field('ADAP_TUNING', u, float, []).
message_field('VISION_POSITION_DELTA', time_usec, uint64_t, [units=us]).
message_field('VISION_POSITION_DELTA', time_delta_usec, uint64_t, [units=us]).
message_field('VISION_POSITION_DELTA', angle_delta, 'float[3]', [units=rad]).
message_field('VISION_POSITION_DELTA', position_delta, 'float[3]', [units=m]).
message_field('VISION_POSITION_DELTA', confidence, float, [units='%']).
message_field('AOA_SSA', time_usec, uint64_t, [units=us]).
message_field('AOA_SSA', 'AOA', float, [units=deg]).
message_field('AOA_SSA', 'SSA', float, [units=deg]).
message_field('ESC_TELEMETRY_1_TO_4', temperature, 'uint8_t[4]', [units=degC]).
message_field('ESC_TELEMETRY_1_TO_4', voltage, 'uint16_t[4]', [units=cV]).
message_field('ESC_TELEMETRY_1_TO_4', current, 'uint16_t[4]', [units=cA]).
message_field('ESC_TELEMETRY_1_TO_4', totalcurrent, 'uint16_t[4]', [units=mAh]).
message_field('ESC_TELEMETRY_1_TO_4', rpm, 'uint16_t[4]', [units=rpm]).
message_field('ESC_TELEMETRY_1_TO_4', count, 'uint16_t[4]', []).
message_field('ESC_TELEMETRY_5_TO_8', temperature, 'uint8_t[4]', [units=degC]).
message_field('ESC_TELEMETRY_5_TO_8', voltage, 'uint16_t[4]', [units=cV]).
message_field('ESC_TELEMETRY_5_TO_8', current, 'uint16_t[4]', [units=cA]).
message_field('ESC_TELEMETRY_5_TO_8', totalcurrent, 'uint16_t[4]', [units=mAh]).
message_field('ESC_TELEMETRY_5_TO_8', rpm, 'uint16_t[4]', [units=rpm]).
message_field('ESC_TELEMETRY_5_TO_8', count, 'uint16_t[4]', []).
message_field('ESC_TELEMETRY_9_TO_12', temperature, 'uint8_t[4]', [units=degC]).
message_field('ESC_TELEMETRY_9_TO_12', voltage, 'uint16_t[4]', [units=cV]).
message_field('ESC_TELEMETRY_9_TO_12', current, 'uint16_t[4]', [units=cA]).
message_field('ESC_TELEMETRY_9_TO_12', totalcurrent, 'uint16_t[4]', [units=mAh]).
message_field('ESC_TELEMETRY_9_TO_12', rpm, 'uint16_t[4]', [units=rpm]).
message_field('ESC_TELEMETRY_9_TO_12', count, 'uint16_t[4]', []).
message_field('OSD_PARAM_CONFIG', target_system, uint8_t, []).
message_field('OSD_PARAM_CONFIG', target_component, uint8_t, []).
message_field('OSD_PARAM_CONFIG', request_id, uint32_t, []).
message_field('OSD_PARAM_CONFIG', osd_screen, uint8_t, []).
message_field('OSD_PARAM_CONFIG', osd_index, uint8_t, []).
message_field('OSD_PARAM_CONFIG', param_id, 'char[16]', []).
message_field('OSD_PARAM_CONFIG', config_type, uint8_t, [enum='OSD_PARAM_CONFIG_TYPE']).
message_field('OSD_PARAM_CONFIG', min_value, float, []).
message_field('OSD_PARAM_CONFIG', max_value, float, []).
message_field('OSD_PARAM_CONFIG', increment, float, []).
message_field('OSD_PARAM_CONFIG_REPLY', request_id, uint32_t, []).
message_field('OSD_PARAM_CONFIG_REPLY', result, uint8_t, [enum='OSD_PARAM_CONFIG_ERROR']).
message_field('OSD_PARAM_SHOW_CONFIG', target_system, uint8_t, []).
message_field('OSD_PARAM_SHOW_CONFIG', target_component, uint8_t, []).
message_field('OSD_PARAM_SHOW_CONFIG', request_id, uint32_t, []).
message_field('OSD_PARAM_SHOW_CONFIG', osd_screen, uint8_t, []).
message_field('OSD_PARAM_SHOW_CONFIG', osd_index, uint8_t, []).
message_field('OSD_PARAM_SHOW_CONFIG_REPLY', request_id, uint32_t, []).
message_field('OSD_PARAM_SHOW_CONFIG_REPLY', result, uint8_t, [enum='OSD_PARAM_CONFIG_ERROR']).
message_field('OSD_PARAM_SHOW_CONFIG_REPLY', param_id, 'char[16]', []).
message_field('OSD_PARAM_SHOW_CONFIG_REPLY', config_type, uint8_t, [enum='OSD_PARAM_CONFIG_TYPE']).
message_field('OSD_PARAM_SHOW_CONFIG_REPLY', min_value, float, []).
message_field('OSD_PARAM_SHOW_CONFIG_REPLY', max_value, float, []).
message_field('OSD_PARAM_SHOW_CONFIG_REPLY', increment, float, []).
message_field('OBSTACLE_DISTANCE_3D', time_boot_ms, uint32_t, [units=ms]).
message_field('OBSTACLE_DISTANCE_3D', sensor_type, uint8_t, [enum='MAV_DISTANCE_SENSOR']).
message_field('OBSTACLE_DISTANCE_3D', frame, uint8_t, [enum='MAV_FRAME']).
message_field('OBSTACLE_DISTANCE_3D', obstacle_id, uint16_t, [instance=true]).
message_field('OBSTACLE_DISTANCE_3D', x, float, [units=m]).
message_field('OBSTACLE_DISTANCE_3D', y, float, [units=m]).
message_field('OBSTACLE_DISTANCE_3D', z, float, [units=m]).
message_field('OBSTACLE_DISTANCE_3D', min_distance, float, [units=m]).
message_field('OBSTACLE_DISTANCE_3D', max_distance, float, [units=m]).
message_field('WATER_DEPTH', time_boot_ms, uint32_t, [units=ms]).
message_field('WATER_DEPTH', id, uint8_t, [instance=true]).
message_field('WATER_DEPTH', healthy, uint8_t, []).
message_field('WATER_DEPTH', lat, int32_t, [units=degE7]).
message_field('WATER_DEPTH', lng, int32_t, [units=degE7]).
message_field('WATER_DEPTH', alt, float, [units=m]).
message_field('WATER_DEPTH', roll, float, [units=rad]).
message_field('WATER_DEPTH', pitch, float, [units=rad]).
message_field('WATER_DEPTH', yaw, float, [units=rad]).
message_field('WATER_DEPTH', distance, float, [units=m]).
message_field('WATER_DEPTH', temperature, float, [units=degC]).
message_field('MCU_STATUS', id, uint8_t, [instance=true]).
message_field('MCU_STATUS', 'MCU_temperature', int16_t, [units=cdegC]).
message_field('MCU_STATUS', 'MCU_voltage', uint16_t, [units=mV]).
message_field('MCU_STATUS', 'MCU_voltage_min', uint16_t, [units=mV]).
message_field('MCU_STATUS', 'MCU_voltage_max', uint16_t, [units=mV]).
message_field('COMMAND_INT_STAMPED', utc_time, uint32_t, []).
message_field('COMMAND_INT_STAMPED', vehicle_timestamp, uint64_t, []).
message_field('COMMAND_INT_STAMPED', target_system, uint8_t, []).
message_field('COMMAND_INT_STAMPED', target_component, uint8_t, []).
message_field('COMMAND_INT_STAMPED', frame, uint8_t, [enum='MAV_FRAME']).
message_field('COMMAND_INT_STAMPED', command, uint16_t, [enum='MAV_CMD']).
message_field('COMMAND_INT_STAMPED', current, uint8_t, []).
message_field('COMMAND_INT_STAMPED', autocontinue, uint8_t, []).
message_field('COMMAND_INT_STAMPED', param1, float, []).
message_field('COMMAND_INT_STAMPED', param2, float, []).
message_field('COMMAND_INT_STAMPED', param3, float, []).
message_field('COMMAND_INT_STAMPED', param4, float, []).
message_field('COMMAND_INT_STAMPED', x, int32_t, []).
message_field('COMMAND_INT_STAMPED', y, int32_t, []).
message_field('COMMAND_INT_STAMPED', z, float, []).
message_field('COMMAND_LONG_STAMPED', utc_time, uint32_t, []).
message_field('COMMAND_LONG_STAMPED', vehicle_timestamp, uint64_t, []).
message_field('COMMAND_LONG_STAMPED', target_system, uint8_t, []).
message_field('COMMAND_LONG_STAMPED', target_component, uint8_t, []).
message_field('COMMAND_LONG_STAMPED', command, uint16_t, [enum='MAV_CMD']).
message_field('COMMAND_LONG_STAMPED', confirmation, uint8_t, []).
message_field('COMMAND_LONG_STAMPED', param1, float, []).
message_field('COMMAND_LONG_STAMPED', param2, float, []).
message_field('COMMAND_LONG_STAMPED', param3, float, []).
message_field('COMMAND_LONG_STAMPED', param4, float, []).
message_field('COMMAND_LONG_STAMPED', param5, float, []).
message_field('COMMAND_LONG_STAMPED', param6, float, []).
message_field('COMMAND_LONG_STAMPED', param7, float, []).
message_field('SENS_POWER', adc121_vspb_volt, float, [units='V']).
message_field('SENS_POWER', adc121_cspb_amp, float, [units='A']).
message_field('SENS_POWER', adc121_cs1_amp, float, [units='A']).
message_field('SENS_POWER', adc121_cs2_amp, float, [units='A']).
message_field('SENS_MPPT', mppt_timestamp, uint64_t, [units=us]).
message_field('SENS_MPPT', mppt1_volt, float, [units='V']).
message_field('SENS_MPPT', mppt1_amp, float, [units='A']).
message_field('SENS_MPPT', mppt1_pwm, uint16_t, [units=us]).
message_field('SENS_MPPT', mppt1_status, uint8_t, []).
message_field('SENS_MPPT', mppt2_volt, float, [units='V']).
message_field('SENS_MPPT', mppt2_amp, float, [units='A']).
message_field('SENS_MPPT', mppt2_pwm, uint16_t, [units=us]).
message_field('SENS_MPPT', mppt2_status, uint8_t, []).
message_field('SENS_MPPT', mppt3_volt, float, [units='V']).
message_field('SENS_MPPT', mppt3_amp, float, [units='A']).
message_field('SENS_MPPT', mppt3_pwm, uint16_t, [units=us]).
message_field('SENS_MPPT', mppt3_status, uint8_t, []).
message_field('ASLCTRL_DATA', timestamp, uint64_t, [units=us]).
message_field('ASLCTRL_DATA', aslctrl_mode, uint8_t, []).
message_field('ASLCTRL_DATA', h, float, []).
message_field('ASLCTRL_DATA', hRef, float, []).
message_field('ASLCTRL_DATA', hRef_t, float, []).
message_field('ASLCTRL_DATA', 'PitchAngle', float, [units=deg]).
message_field('ASLCTRL_DATA', 'PitchAngleRef', float, [units=deg]).
message_field('ASLCTRL_DATA', q, float, []).
message_field('ASLCTRL_DATA', qRef, float, []).
message_field('ASLCTRL_DATA', uElev, float, []).
message_field('ASLCTRL_DATA', uThrot, float, []).
message_field('ASLCTRL_DATA', uThrot2, float, []).
message_field('ASLCTRL_DATA', nZ, float, []).
message_field('ASLCTRL_DATA', 'AirspeedRef', float, [units='m/s']).
message_field('ASLCTRL_DATA', 'SpoilersEngaged', uint8_t, []).
message_field('ASLCTRL_DATA', 'YawAngle', float, [units=deg]).
message_field('ASLCTRL_DATA', 'YawAngleRef', float, [units=deg]).
message_field('ASLCTRL_DATA', 'RollAngle', float, [units=deg]).
message_field('ASLCTRL_DATA', 'RollAngleRef', float, [units=deg]).
message_field('ASLCTRL_DATA', p, float, []).
message_field('ASLCTRL_DATA', pRef, float, []).
message_field('ASLCTRL_DATA', r, float, []).
message_field('ASLCTRL_DATA', rRef, float, []).
message_field('ASLCTRL_DATA', uAil, float, []).
message_field('ASLCTRL_DATA', uRud, float, []).
message_field('ASLCTRL_DEBUG', i32_1, uint32_t, []).
message_field('ASLCTRL_DEBUG', i8_1, uint8_t, []).
message_field('ASLCTRL_DEBUG', i8_2, uint8_t, []).
message_field('ASLCTRL_DEBUG', f_1, float, []).
message_field('ASLCTRL_DEBUG', f_2, float, []).
message_field('ASLCTRL_DEBUG', f_3, float, []).
message_field('ASLCTRL_DEBUG', f_4, float, []).
message_field('ASLCTRL_DEBUG', f_5, float, []).
message_field('ASLCTRL_DEBUG', f_6, float, []).
message_field('ASLCTRL_DEBUG', f_7, float, []).
message_field('ASLCTRL_DEBUG', f_8, float, []).
message_field('ASLUAV_STATUS', 'LED_status', uint8_t, []).
message_field('ASLUAV_STATUS', 'SATCOM_status', uint8_t, []).
message_field('ASLUAV_STATUS', 'Servo_status', 'uint8_t[8]', []).
message_field('ASLUAV_STATUS', 'Motor_rpm', float, []).
message_field('EKF_EXT', timestamp, uint64_t, [units=us]).
message_field('EKF_EXT', 'Windspeed', float, [units='m/s']).
message_field('EKF_EXT', 'WindDir', float, [units=rad]).
message_field('EKF_EXT', 'WindZ', float, [units='m/s']).
message_field('EKF_EXT', 'Airspeed', float, [units='m/s']).
message_field('EKF_EXT', beta, float, [units=rad]).
message_field('EKF_EXT', alpha, float, [units=rad]).
message_field('ASL_OBCTRL', timestamp, uint64_t, [units=us]).
message_field('ASL_OBCTRL', uElev, float, []).
message_field('ASL_OBCTRL', uThrot, float, []).
message_field('ASL_OBCTRL', uThrot2, float, []).
message_field('ASL_OBCTRL', uAilL, float, []).
message_field('ASL_OBCTRL', uAilR, float, []).
message_field('ASL_OBCTRL', uRud, float, []).
message_field('ASL_OBCTRL', obctrl_status, uint8_t, []).
message_field('SENS_ATMOS', timestamp, uint64_t, [units=us]).
message_field('SENS_ATMOS', 'TempAmbient', float, [units=degC]).
message_field('SENS_ATMOS', 'Humidity', float, [units='%']).
message_field('SENS_BATMON', batmon_timestamp, uint64_t, [units=us]).
message_field('SENS_BATMON', temperature, float, [units=degC]).
message_field('SENS_BATMON', voltage, uint16_t, [units=mV]).
message_field('SENS_BATMON', current, int16_t, [units=mA]).
message_field('SENS_BATMON', 'SoC', uint8_t, []).
message_field('SENS_BATMON', batterystatus, uint16_t, []).
message_field('SENS_BATMON', serialnumber, uint16_t, []).
message_field('SENS_BATMON', safetystatus, uint32_t, []).
message_field('SENS_BATMON', operationstatus, uint32_t, []).
message_field('SENS_BATMON', cellvoltage1, uint16_t, [units=mV]).
message_field('SENS_BATMON', cellvoltage2, uint16_t, [units=mV]).
message_field('SENS_BATMON', cellvoltage3, uint16_t, [units=mV]).
message_field('SENS_BATMON', cellvoltage4, uint16_t, [units=mV]).
message_field('SENS_BATMON', cellvoltage5, uint16_t, [units=mV]).
message_field('SENS_BATMON', cellvoltage6, uint16_t, [units=mV]).
message_field('FW_SOARING_DATA', timestamp, uint64_t, [units=ms]).
message_field('FW_SOARING_DATA', timestampModeChanged, uint64_t, [units=ms]).
message_field('FW_SOARING_DATA', xW, float, [units='m/s']).
message_field('FW_SOARING_DATA', xR, float, [units=m]).
message_field('FW_SOARING_DATA', xLat, float, [units=deg]).
message_field('FW_SOARING_DATA', xLon, float, [units=deg]).
message_field('FW_SOARING_DATA', 'VarW', float, []).
message_field('FW_SOARING_DATA', 'VarR', float, []).
message_field('FW_SOARING_DATA', 'VarLat', float, []).
message_field('FW_SOARING_DATA', 'VarLon', float, []).
message_field('FW_SOARING_DATA', 'LoiterRadius', float, [units=m]).
message_field('FW_SOARING_DATA', 'LoiterDirection', float, []).
message_field('FW_SOARING_DATA', 'DistToSoarPoint', float, [units=m]).
message_field('FW_SOARING_DATA', vSinkExp, float, [units='m/s']).
message_field('FW_SOARING_DATA', z1_LocalUpdraftSpeed, float, [units='m/s']).
message_field('FW_SOARING_DATA', z2_DeltaRoll, float, [units=deg]).
message_field('FW_SOARING_DATA', z1_exp, float, []).
message_field('FW_SOARING_DATA', z2_exp, float, []).
message_field('FW_SOARING_DATA', 'ThermalGSNorth', float, [units='m/s']).
message_field('FW_SOARING_DATA', 'ThermalGSEast', float, [units='m/s']).
message_field('FW_SOARING_DATA', 'TSE_dot', float, [units='m/s']).
message_field('FW_SOARING_DATA', 'DebugVar1', float, []).
message_field('FW_SOARING_DATA', 'DebugVar2', float, []).
message_field('FW_SOARING_DATA', 'ControlMode', uint8_t, []).
message_field('FW_SOARING_DATA', valid, uint8_t, []).
message_field('SENSORPOD_STATUS', timestamp, uint64_t, [units=ms]).
message_field('SENSORPOD_STATUS', visensor_rate_1, uint8_t, []).
message_field('SENSORPOD_STATUS', visensor_rate_2, uint8_t, []).
message_field('SENSORPOD_STATUS', visensor_rate_3, uint8_t, []).
message_field('SENSORPOD_STATUS', visensor_rate_4, uint8_t, []).
message_field('SENSORPOD_STATUS', recording_nodes_count, uint8_t, []).
message_field('SENSORPOD_STATUS', cpu_temp, uint8_t, [units=degC]).
message_field('SENSORPOD_STATUS', free_space, uint16_t, []).
message_field('SENS_POWER_BOARD', timestamp, uint64_t, [units=us]).
message_field('SENS_POWER_BOARD', pwr_brd_status, uint8_t, []).
message_field('SENS_POWER_BOARD', pwr_brd_led_status, uint8_t, []).
message_field('SENS_POWER_BOARD', pwr_brd_system_volt, float, [units='V']).
message_field('SENS_POWER_BOARD', pwr_brd_servo_volt, float, [units='V']).
message_field('SENS_POWER_BOARD', pwr_brd_digital_volt, float, [units='V']).
message_field('SENS_POWER_BOARD', pwr_brd_mot_l_amp, float, [units='A']).
message_field('SENS_POWER_BOARD', pwr_brd_mot_r_amp, float, [units='A']).
message_field('SENS_POWER_BOARD', pwr_brd_analog_amp, float, [units='A']).
message_field('SENS_POWER_BOARD', pwr_brd_digital_amp, float, [units='A']).
message_field('SENS_POWER_BOARD', pwr_brd_ext_amp, float, [units='A']).
message_field('SENS_POWER_BOARD', pwr_brd_aux_amp, float, [units='A']).
message_field('GSM_LINK_STATUS', timestamp, uint64_t, [units=us]).
message_field('GSM_LINK_STATUS', gsm_modem_type, uint8_t, [enum='GSM_MODEM_TYPE']).
message_field('GSM_LINK_STATUS', gsm_link_type, uint8_t, [enum='GSM_LINK_TYPE']).
message_field('GSM_LINK_STATUS', rssi, uint8_t, []).
message_field('GSM_LINK_STATUS', rsrp_rscp, uint8_t, []).
message_field('GSM_LINK_STATUS', sinr_ecio, uint8_t, []).
message_field('GSM_LINK_STATUS', rsrq, uint8_t, []).
message_field('SATCOM_LINK_STATUS', timestamp, uint64_t, [units=us]).
message_field('SATCOM_LINK_STATUS', last_heartbeat, uint64_t, [units=us]).
message_field('SATCOM_LINK_STATUS', failed_sessions, uint16_t, []).
message_field('SATCOM_LINK_STATUS', successful_sessions, uint16_t, []).
message_field('SATCOM_LINK_STATUS', signal_quality, uint8_t, []).
message_field('SATCOM_LINK_STATUS', ring_pending, uint8_t, []).
message_field('SATCOM_LINK_STATUS', tx_session_pending, uint8_t, []).
message_field('SATCOM_LINK_STATUS', rx_session_pending, uint8_t, []).
message_field('SENSOR_AIRFLOW_ANGLES', timestamp, uint64_t, [units=us]).
message_field('SENSOR_AIRFLOW_ANGLES', angleofattack, float, [units=deg]).
message_field('SENSOR_AIRFLOW_ANGLES', angleofattack_valid, uint8_t, []).
message_field('SENSOR_AIRFLOW_ANGLES', sideslip, float, [units=deg]).
message_field('SENSOR_AIRFLOW_ANGLES', sideslip_valid, uint8_t, []).
message_field('SYS_STATUS', onboard_control_sensors_present, uint32_t, [enum='MAV_SYS_STATUS_SENSOR', display=bitmask, print_format='0x%04x']).
message_field('SYS_STATUS', onboard_control_sensors_enabled, uint32_t, [enum='MAV_SYS_STATUS_SENSOR', display=bitmask, print_format='0x%04x']).
message_field('SYS_STATUS', onboard_control_sensors_health, uint32_t, [enum='MAV_SYS_STATUS_SENSOR', display=bitmask, print_format='0x%04x']).
message_field('SYS_STATUS', load, uint16_t, [units='d%']).
message_field('SYS_STATUS', voltage_battery, uint16_t, [units=mV, invalid='UINT16_MAX']).
message_field('SYS_STATUS', current_battery, int16_t, [units=cA, invalid='-1']).
message_field('SYS_STATUS', battery_remaining, int8_t, [units='%', invalid='-1']).
message_field('SYS_STATUS', drop_rate_comm, uint16_t, [units='c%']).
message_field('SYS_STATUS', errors_comm, uint16_t, []).
message_field('SYS_STATUS', errors_count1, uint16_t, []).
message_field('SYS_STATUS', errors_count2, uint16_t, []).
message_field('SYS_STATUS', errors_count3, uint16_t, []).
message_field('SYS_STATUS', errors_count4, uint16_t, []).
message_field('SYS_STATUS', onboard_control_sensors_present_extended, uint32_t, [enum='MAV_SYS_STATUS_SENSOR_EXTENDED', display=bitmask, print_format='0x%04x']).
message_field('SYS_STATUS', onboard_control_sensors_enabled_extended, uint32_t, [enum='MAV_SYS_STATUS_SENSOR_EXTENDED', display=bitmask, print_format='0x%04x']).
message_field('SYS_STATUS', onboard_control_sensors_health_extended, uint32_t, [enum='MAV_SYS_STATUS_SENSOR_EXTENDED', display=bitmask, print_format='0x%04x']).
message_field('SYSTEM_TIME', time_unix_usec, uint64_t, [units=us]).
message_field('SYSTEM_TIME', time_boot_ms, uint32_t, [units=ms]).
message_field('PING', time_usec, uint64_t, [units=us]).
message_field('PING', seq, uint32_t, []).
message_field('PING', target_system, uint8_t, []).
message_field('PING', target_component, uint8_t, []).
message_field('CHANGE_OPERATOR_CONTROL', target_system, uint8_t, []).
message_field('CHANGE_OPERATOR_CONTROL', control_request, uint8_t, []).
message_field('CHANGE_OPERATOR_CONTROL', version, uint8_t, [units=rad]).
message_field('CHANGE_OPERATOR_CONTROL', passkey, 'char[25]', []).
message_field('CHANGE_OPERATOR_CONTROL_ACK', gcs_system_id, uint8_t, []).
message_field('CHANGE_OPERATOR_CONTROL_ACK', control_request, uint8_t, []).
message_field('CHANGE_OPERATOR_CONTROL_ACK', ack, uint8_t, []).
message_field('AUTH_KEY', key, 'char[32]', []).
message_field('LINK_NODE_STATUS', timestamp, uint64_t, [units=ms]).
message_field('LINK_NODE_STATUS', tx_buf, uint8_t, [units='%']).
message_field('LINK_NODE_STATUS', rx_buf, uint8_t, [units='%']).
message_field('LINK_NODE_STATUS', tx_rate, uint32_t, [units='bytes/s']).
message_field('LINK_NODE_STATUS', rx_rate, uint32_t, [units='bytes/s']).
message_field('LINK_NODE_STATUS', rx_parse_err, uint16_t, [units=bytes]).
message_field('LINK_NODE_STATUS', tx_overflows, uint16_t, [units=bytes]).
message_field('LINK_NODE_STATUS', rx_overflows, uint16_t, [units=bytes]).
message_field('LINK_NODE_STATUS', messages_sent, uint32_t, []).
message_field('LINK_NODE_STATUS', messages_received, uint32_t, []).
message_field('LINK_NODE_STATUS', messages_lost, uint32_t, []).
message_field('SET_MODE', target_system, uint8_t, []).
message_field('SET_MODE', base_mode, uint8_t, [enum='MAV_MODE']).
message_field('SET_MODE', custom_mode, uint32_t, []).
message_field('PARAM_REQUEST_READ', target_system, uint8_t, []).
message_field('PARAM_REQUEST_READ', target_component, uint8_t, []).
message_field('PARAM_REQUEST_READ', param_id, 'char[16]', []).
message_field('PARAM_REQUEST_READ', param_index, int16_t, [invalid='-1']).
message_field('PARAM_REQUEST_LIST', target_system, uint8_t, []).
message_field('PARAM_REQUEST_LIST', target_component, uint8_t, []).
message_field('PARAM_VALUE', param_id, 'char[16]', []).
message_field('PARAM_VALUE', param_value, float, []).
message_field('PARAM_VALUE', param_type, uint8_t, [enum='MAV_PARAM_TYPE']).
message_field('PARAM_VALUE', param_count, uint16_t, []).
message_field('PARAM_VALUE', param_index, uint16_t, []).
message_field('PARAM_SET', target_system, uint8_t, []).
message_field('PARAM_SET', target_component, uint8_t, []).
message_field('PARAM_SET', param_id, 'char[16]', []).
message_field('PARAM_SET', param_value, float, []).
message_field('PARAM_SET', param_type, uint8_t, [enum='MAV_PARAM_TYPE']).
message_field('GPS_RAW_INT', time_usec, uint64_t, [units=us]).
message_field('GPS_RAW_INT', fix_type, uint8_t, [enum='GPS_FIX_TYPE']).
message_field('GPS_RAW_INT', lat, int32_t, [units=degE7]).
message_field('GPS_RAW_INT', lon, int32_t, [units=degE7]).
message_field('GPS_RAW_INT', alt, int32_t, [units=mm]).
message_field('GPS_RAW_INT', eph, uint16_t, [invalid='UINT16_MAX']).
message_field('GPS_RAW_INT', epv, uint16_t, [invalid='UINT16_MAX']).
message_field('GPS_RAW_INT', vel, uint16_t, [units='cm/s', invalid='UINT16_MAX']).
message_field('GPS_RAW_INT', cog, uint16_t, [units=cdeg, invalid='UINT16_MAX']).
message_field('GPS_RAW_INT', satellites_visible, uint8_t, [invalid='UINT8_MAX']).
message_field('GPS_RAW_INT', alt_ellipsoid, int32_t, [units=mm]).
message_field('GPS_RAW_INT', h_acc, uint32_t, [units=mm]).
message_field('GPS_RAW_INT', v_acc, uint32_t, [units=mm]).
message_field('GPS_RAW_INT', vel_acc, uint32_t, [units=mm]).
message_field('GPS_RAW_INT', hdg_acc, uint32_t, [units=degE5]).
message_field('GPS_RAW_INT', yaw, uint16_t, [units=cdeg, invalid='0']).
message_field('GPS_STATUS', satellites_visible, uint8_t, []).
message_field('GPS_STATUS', satellite_prn, 'uint8_t[20]', []).
message_field('GPS_STATUS', satellite_used, 'uint8_t[20]', []).
message_field('GPS_STATUS', satellite_elevation, 'uint8_t[20]', [units=deg]).
message_field('GPS_STATUS', satellite_azimuth, 'uint8_t[20]', [units=deg]).
message_field('GPS_STATUS', satellite_snr, 'uint8_t[20]', [units=dB]).
message_field('SCALED_IMU', time_boot_ms, uint32_t, [units=ms]).
message_field('SCALED_IMU', xacc, int16_t, [units=mG]).
message_field('SCALED_IMU', yacc, int16_t, [units=mG]).
message_field('SCALED_IMU', zacc, int16_t, [units=mG]).
message_field('SCALED_IMU', xgyro, int16_t, [units='mrad/s']).
message_field('SCALED_IMU', ygyro, int16_t, [units='mrad/s']).
message_field('SCALED_IMU', zgyro, int16_t, [units='mrad/s']).
message_field('SCALED_IMU', xmag, int16_t, [units=mgauss]).
message_field('SCALED_IMU', ymag, int16_t, [units=mgauss]).
message_field('SCALED_IMU', zmag, int16_t, [units=mgauss]).
message_field('SCALED_IMU', temperature, int16_t, [units=cdegC]).
message_field('RAW_IMU', time_usec, uint64_t, [units=us]).
message_field('RAW_IMU', xacc, int16_t, []).
message_field('RAW_IMU', yacc, int16_t, []).
message_field('RAW_IMU', zacc, int16_t, []).
message_field('RAW_IMU', xgyro, int16_t, []).
message_field('RAW_IMU', ygyro, int16_t, []).
message_field('RAW_IMU', zgyro, int16_t, []).
message_field('RAW_IMU', xmag, int16_t, []).
message_field('RAW_IMU', ymag, int16_t, []).
message_field('RAW_IMU', zmag, int16_t, []).
message_field('RAW_IMU', id, uint8_t, [instance=true]).
message_field('RAW_IMU', temperature, int16_t, [units=cdegC]).
message_field('RAW_PRESSURE', time_usec, uint64_t, [units=us]).
message_field('RAW_PRESSURE', press_abs, int16_t, []).
message_field('RAW_PRESSURE', press_diff1, int16_t, [invalid='0']).
message_field('RAW_PRESSURE', press_diff2, int16_t, [invalid='0']).
message_field('RAW_PRESSURE', temperature, int16_t, []).
message_field('SCALED_PRESSURE', time_boot_ms, uint32_t, [units=ms]).
message_field('SCALED_PRESSURE', press_abs, float, [units=hPa]).
message_field('SCALED_PRESSURE', press_diff, float, [units=hPa]).
message_field('SCALED_PRESSURE', temperature, int16_t, [units=cdegC]).
message_field('SCALED_PRESSURE', temperature_press_diff, int16_t, [units=cdegC, invalid='0']).
message_field('ATTITUDE', time_boot_ms, uint32_t, [units=ms]).
message_field('ATTITUDE', roll, float, [units=rad]).
message_field('ATTITUDE', pitch, float, [units=rad]).
message_field('ATTITUDE', yaw, float, [units=rad]).
message_field('ATTITUDE', rollspeed, float, [units='rad/s']).
message_field('ATTITUDE', pitchspeed, float, [units='rad/s']).
message_field('ATTITUDE', yawspeed, float, [units='rad/s']).
message_field('ATTITUDE_QUATERNION', time_boot_ms, uint32_t, [units=ms]).
message_field('ATTITUDE_QUATERNION', q1, float, []).
message_field('ATTITUDE_QUATERNION', q2, float, []).
message_field('ATTITUDE_QUATERNION', q3, float, []).
message_field('ATTITUDE_QUATERNION', q4, float, []).
message_field('ATTITUDE_QUATERNION', rollspeed, float, [units='rad/s']).
message_field('ATTITUDE_QUATERNION', pitchspeed, float, [units='rad/s']).
message_field('ATTITUDE_QUATERNION', yawspeed, float, [units='rad/s']).
message_field('ATTITUDE_QUATERNION', repr_offset_q, 'float[4]', [invalid='[0]']).
message_field('LOCAL_POSITION_NED', time_boot_ms, uint32_t, [units=ms]).
message_field('LOCAL_POSITION_NED', x, float, [units=m]).
message_field('LOCAL_POSITION_NED', y, float, [units=m]).
message_field('LOCAL_POSITION_NED', z, float, [units=m]).
message_field('LOCAL_POSITION_NED', vx, float, [units='m/s']).
message_field('LOCAL_POSITION_NED', vy, float, [units='m/s']).
message_field('LOCAL_POSITION_NED', vz, float, [units='m/s']).
message_field('GLOBAL_POSITION_INT', time_boot_ms, uint32_t, [units=ms]).
message_field('GLOBAL_POSITION_INT', lat, int32_t, [units=degE7]).
message_field('GLOBAL_POSITION_INT', lon, int32_t, [units=degE7]).
message_field('GLOBAL_POSITION_INT', alt, int32_t, [units=mm]).
message_field('GLOBAL_POSITION_INT', relative_alt, int32_t, [units=mm]).
message_field('GLOBAL_POSITION_INT', vx, int16_t, [units='cm/s']).
message_field('GLOBAL_POSITION_INT', vy, int16_t, [units='cm/s']).
message_field('GLOBAL_POSITION_INT', vz, int16_t, [units='cm/s']).
message_field('GLOBAL_POSITION_INT', hdg, uint16_t, [units=cdeg, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_SCALED', time_boot_ms, uint32_t, [units=ms]).
message_field('RC_CHANNELS_SCALED', port, uint8_t, []).
message_field('RC_CHANNELS_SCALED', chan1_scaled, int16_t, [invalid='INT16_MAX']).
message_field('RC_CHANNELS_SCALED', chan2_scaled, int16_t, [invalid='INT16_MAX']).
message_field('RC_CHANNELS_SCALED', chan3_scaled, int16_t, [invalid='INT16_MAX']).
message_field('RC_CHANNELS_SCALED', chan4_scaled, int16_t, [invalid='INT16_MAX']).
message_field('RC_CHANNELS_SCALED', chan5_scaled, int16_t, [invalid='INT16_MAX']).
message_field('RC_CHANNELS_SCALED', chan6_scaled, int16_t, [invalid='INT16_MAX']).
message_field('RC_CHANNELS_SCALED', chan7_scaled, int16_t, [invalid='INT16_MAX']).
message_field('RC_CHANNELS_SCALED', chan8_scaled, int16_t, [invalid='INT16_MAX']).
message_field('RC_CHANNELS_SCALED', rssi, uint8_t, [invalid='UINT8_MAX']).
message_field('RC_CHANNELS_RAW', time_boot_ms, uint32_t, [units=ms]).
message_field('RC_CHANNELS_RAW', port, uint8_t, []).
message_field('RC_CHANNELS_RAW', chan1_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_RAW', chan2_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_RAW', chan3_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_RAW', chan4_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_RAW', chan5_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_RAW', chan6_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_RAW', chan7_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_RAW', chan8_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_RAW', rssi, uint8_t, [invalid='UINT8_MAX']).
message_field('SERVO_OUTPUT_RAW', time_usec, uint32_t, [units=us]).
message_field('SERVO_OUTPUT_RAW', port, uint8_t, []).
message_field('SERVO_OUTPUT_RAW', servo1_raw, uint16_t, [units=us]).
message_field('SERVO_OUTPUT_RAW', servo2_raw, uint16_t, [units=us]).
message_field('SERVO_OUTPUT_RAW', servo3_raw, uint16_t, [units=us]).
message_field('SERVO_OUTPUT_RAW', servo4_raw, uint16_t, [units=us]).
message_field('SERVO_OUTPUT_RAW', servo5_raw, uint16_t, [units=us]).
message_field('SERVO_OUTPUT_RAW', servo6_raw, uint16_t, [units=us]).
message_field('SERVO_OUTPUT_RAW', servo7_raw, uint16_t, [units=us]).
message_field('SERVO_OUTPUT_RAW', servo8_raw, uint16_t, [units=us]).
message_field('SERVO_OUTPUT_RAW', servo9_raw, uint16_t, [units=us]).
message_field('SERVO_OUTPUT_RAW', servo10_raw, uint16_t, [units=us]).
message_field('SERVO_OUTPUT_RAW', servo11_raw, uint16_t, [units=us]).
message_field('SERVO_OUTPUT_RAW', servo12_raw, uint16_t, [units=us]).
message_field('SERVO_OUTPUT_RAW', servo13_raw, uint16_t, [units=us]).
message_field('SERVO_OUTPUT_RAW', servo14_raw, uint16_t, [units=us]).
message_field('SERVO_OUTPUT_RAW', servo15_raw, uint16_t, [units=us]).
message_field('SERVO_OUTPUT_RAW', servo16_raw, uint16_t, [units=us]).
message_field('MISSION_REQUEST_PARTIAL_LIST', target_system, uint8_t, []).
message_field('MISSION_REQUEST_PARTIAL_LIST', target_component, uint8_t, []).
message_field('MISSION_REQUEST_PARTIAL_LIST', start_index, int16_t, []).
message_field('MISSION_REQUEST_PARTIAL_LIST', end_index, int16_t, []).
message_field('MISSION_REQUEST_PARTIAL_LIST', mission_type, uint8_t, [enum='MAV_MISSION_TYPE']).
message_field('MISSION_WRITE_PARTIAL_LIST', target_system, uint8_t, []).
message_field('MISSION_WRITE_PARTIAL_LIST', target_component, uint8_t, []).
message_field('MISSION_WRITE_PARTIAL_LIST', start_index, int16_t, []).
message_field('MISSION_WRITE_PARTIAL_LIST', end_index, int16_t, []).
message_field('MISSION_WRITE_PARTIAL_LIST', mission_type, uint8_t, [enum='MAV_MISSION_TYPE']).
message_field('MISSION_ITEM', target_system, uint8_t, []).
message_field('MISSION_ITEM', target_component, uint8_t, []).
message_field('MISSION_ITEM', seq, uint16_t, []).
message_field('MISSION_ITEM', frame, uint8_t, [enum='MAV_FRAME']).
message_field('MISSION_ITEM', command, uint16_t, [enum='MAV_CMD']).
message_field('MISSION_ITEM', current, uint8_t, []).
message_field('MISSION_ITEM', autocontinue, uint8_t, []).
message_field('MISSION_ITEM', param1, float, []).
message_field('MISSION_ITEM', param2, float, []).
message_field('MISSION_ITEM', param3, float, []).
message_field('MISSION_ITEM', param4, float, []).
message_field('MISSION_ITEM', x, float, []).
message_field('MISSION_ITEM', y, float, []).
message_field('MISSION_ITEM', z, float, []).
message_field('MISSION_ITEM', mission_type, uint8_t, [enum='MAV_MISSION_TYPE']).
message_field('MISSION_REQUEST', target_system, uint8_t, []).
message_field('MISSION_REQUEST', target_component, uint8_t, []).
message_field('MISSION_REQUEST', seq, uint16_t, []).
message_field('MISSION_REQUEST', mission_type, uint8_t, [enum='MAV_MISSION_TYPE']).
message_field('MISSION_SET_CURRENT', target_system, uint8_t, []).
message_field('MISSION_SET_CURRENT', target_component, uint8_t, []).
message_field('MISSION_SET_CURRENT', seq, uint16_t, []).
message_field('MISSION_CURRENT', seq, uint16_t, []).
message_field('MISSION_CURRENT', total, uint16_t, [invalid='UINT16_MAX']).
message_field('MISSION_CURRENT', mission_state, uint8_t, [enum='MISSION_STATE', invalid='0']).
message_field('MISSION_CURRENT', mission_mode, uint8_t, [invalid='0']).
message_field('MISSION_REQUEST_LIST', target_system, uint8_t, []).
message_field('MISSION_REQUEST_LIST', target_component, uint8_t, []).
message_field('MISSION_REQUEST_LIST', mission_type, uint8_t, [enum='MAV_MISSION_TYPE']).
message_field('MISSION_COUNT', target_system, uint8_t, []).
message_field('MISSION_COUNT', target_component, uint8_t, []).
message_field('MISSION_COUNT', count, uint16_t, []).
message_field('MISSION_COUNT', mission_type, uint8_t, [enum='MAV_MISSION_TYPE']).
message_field('MISSION_CLEAR_ALL', target_system, uint8_t, []).
message_field('MISSION_CLEAR_ALL', target_component, uint8_t, []).
message_field('MISSION_CLEAR_ALL', mission_type, uint8_t, [enum='MAV_MISSION_TYPE']).
message_field('MISSION_ITEM_REACHED', seq, uint16_t, []).
message_field('MISSION_ACK', target_system, uint8_t, []).
message_field('MISSION_ACK', target_component, uint8_t, []).
message_field('MISSION_ACK', type, uint8_t, [enum='MAV_MISSION_RESULT']).
message_field('MISSION_ACK', mission_type, uint8_t, [enum='MAV_MISSION_TYPE']).
message_field('SET_GPS_GLOBAL_ORIGIN', target_system, uint8_t, []).
message_field('SET_GPS_GLOBAL_ORIGIN', latitude, int32_t, [units=degE7]).
message_field('SET_GPS_GLOBAL_ORIGIN', longitude, int32_t, [units=degE7]).
message_field('SET_GPS_GLOBAL_ORIGIN', altitude, int32_t, [units=mm]).
message_field('SET_GPS_GLOBAL_ORIGIN', time_usec, uint64_t, [units=us]).
message_field('GPS_GLOBAL_ORIGIN', latitude, int32_t, [units=degE7]).
message_field('GPS_GLOBAL_ORIGIN', longitude, int32_t, [units=degE7]).
message_field('GPS_GLOBAL_ORIGIN', altitude, int32_t, [units=mm]).
message_field('GPS_GLOBAL_ORIGIN', time_usec, uint64_t, [units=us]).
message_field('PARAM_MAP_RC', target_system, uint8_t, []).
message_field('PARAM_MAP_RC', target_component, uint8_t, []).
message_field('PARAM_MAP_RC', param_id, 'char[16]', []).
message_field('PARAM_MAP_RC', param_index, int16_t, []).
message_field('PARAM_MAP_RC', parameter_rc_channel_index, uint8_t, []).
message_field('PARAM_MAP_RC', param_value0, float, []).
message_field('PARAM_MAP_RC', scale, float, []).
message_field('PARAM_MAP_RC', param_value_min, float, []).
message_field('PARAM_MAP_RC', param_value_max, float, []).
message_field('MISSION_REQUEST_INT', target_system, uint8_t, []).
message_field('MISSION_REQUEST_INT', target_component, uint8_t, []).
message_field('MISSION_REQUEST_INT', seq, uint16_t, []).
message_field('MISSION_REQUEST_INT', mission_type, uint8_t, [enum='MAV_MISSION_TYPE']).
message_field('SAFETY_SET_ALLOWED_AREA', target_system, uint8_t, []).
message_field('SAFETY_SET_ALLOWED_AREA', target_component, uint8_t, []).
message_field('SAFETY_SET_ALLOWED_AREA', frame, uint8_t, [enum='MAV_FRAME']).
message_field('SAFETY_SET_ALLOWED_AREA', p1x, float, [units=m]).
message_field('SAFETY_SET_ALLOWED_AREA', p1y, float, [units=m]).
message_field('SAFETY_SET_ALLOWED_AREA', p1z, float, [units=m]).
message_field('SAFETY_SET_ALLOWED_AREA', p2x, float, [units=m]).
message_field('SAFETY_SET_ALLOWED_AREA', p2y, float, [units=m]).
message_field('SAFETY_SET_ALLOWED_AREA', p2z, float, [units=m]).
message_field('SAFETY_ALLOWED_AREA', frame, uint8_t, [enum='MAV_FRAME']).
message_field('SAFETY_ALLOWED_AREA', p1x, float, [units=m]).
message_field('SAFETY_ALLOWED_AREA', p1y, float, [units=m]).
message_field('SAFETY_ALLOWED_AREA', p1z, float, [units=m]).
message_field('SAFETY_ALLOWED_AREA', p2x, float, [units=m]).
message_field('SAFETY_ALLOWED_AREA', p2y, float, [units=m]).
message_field('SAFETY_ALLOWED_AREA', p2z, float, [units=m]).
message_field('ATTITUDE_QUATERNION_COV', time_usec, uint64_t, [units=us]).
message_field('ATTITUDE_QUATERNION_COV', q, 'float[4]', []).
message_field('ATTITUDE_QUATERNION_COV', rollspeed, float, [units='rad/s']).
message_field('ATTITUDE_QUATERNION_COV', pitchspeed, float, [units='rad/s']).
message_field('ATTITUDE_QUATERNION_COV', yawspeed, float, [units='rad/s']).
message_field('ATTITUDE_QUATERNION_COV', covariance, 'float[9]', [invalid='[NaN:]']).
message_field('NAV_CONTROLLER_OUTPUT', nav_roll, float, [units=deg]).
message_field('NAV_CONTROLLER_OUTPUT', nav_pitch, float, [units=deg]).
message_field('NAV_CONTROLLER_OUTPUT', nav_bearing, int16_t, [units=deg]).
message_field('NAV_CONTROLLER_OUTPUT', target_bearing, int16_t, [units=deg]).
message_field('NAV_CONTROLLER_OUTPUT', wp_dist, uint16_t, [units=m]).
message_field('NAV_CONTROLLER_OUTPUT', alt_error, float, [units=m]).
message_field('NAV_CONTROLLER_OUTPUT', aspd_error, float, [units='m/s']).
message_field('NAV_CONTROLLER_OUTPUT', xtrack_error, float, [units=m]).
message_field('GLOBAL_POSITION_INT_COV', time_usec, uint64_t, [units=us]).
message_field('GLOBAL_POSITION_INT_COV', estimator_type, uint8_t, [enum='MAV_ESTIMATOR_TYPE']).
message_field('GLOBAL_POSITION_INT_COV', lat, int32_t, [units=degE7]).
message_field('GLOBAL_POSITION_INT_COV', lon, int32_t, [units=degE7]).
message_field('GLOBAL_POSITION_INT_COV', alt, int32_t, [units=mm]).
message_field('GLOBAL_POSITION_INT_COV', relative_alt, int32_t, [units=mm]).
message_field('GLOBAL_POSITION_INT_COV', vx, float, [units='m/s']).
message_field('GLOBAL_POSITION_INT_COV', vy, float, [units='m/s']).
message_field('GLOBAL_POSITION_INT_COV', vz, float, [units='m/s']).
message_field('GLOBAL_POSITION_INT_COV', covariance, 'float[36]', [invalid='[NaN:]']).
message_field('LOCAL_POSITION_NED_COV', time_usec, uint64_t, [units=us]).
message_field('LOCAL_POSITION_NED_COV', estimator_type, uint8_t, [enum='MAV_ESTIMATOR_TYPE']).
message_field('LOCAL_POSITION_NED_COV', x, float, [units=m]).
message_field('LOCAL_POSITION_NED_COV', y, float, [units=m]).
message_field('LOCAL_POSITION_NED_COV', z, float, [units=m]).
message_field('LOCAL_POSITION_NED_COV', vx, float, [units='m/s']).
message_field('LOCAL_POSITION_NED_COV', vy, float, [units='m/s']).
message_field('LOCAL_POSITION_NED_COV', vz, float, [units='m/s']).
message_field('LOCAL_POSITION_NED_COV', ax, float, [units='m/s/s']).
message_field('LOCAL_POSITION_NED_COV', ay, float, [units='m/s/s']).
message_field('LOCAL_POSITION_NED_COV', az, float, [units='m/s/s']).
message_field('LOCAL_POSITION_NED_COV', covariance, 'float[45]', [invalid='[NaN:]']).
message_field('RC_CHANNELS', time_boot_ms, uint32_t, [units=ms]).
message_field('RC_CHANNELS', chancount, uint8_t, []).
message_field('RC_CHANNELS', chan1_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan2_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan3_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan4_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan5_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan6_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan7_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan8_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan9_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan10_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan11_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan12_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan13_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan14_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan15_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan16_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan17_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', chan18_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS', rssi, uint8_t, [invalid='UINT8_MAX']).
message_field('REQUEST_DATA_STREAM', target_system, uint8_t, []).
message_field('REQUEST_DATA_STREAM', target_component, uint8_t, []).
message_field('REQUEST_DATA_STREAM', req_stream_id, uint8_t, []).
message_field('REQUEST_DATA_STREAM', req_message_rate, uint16_t, [units='Hz']).
message_field('REQUEST_DATA_STREAM', start_stop, uint8_t, []).
message_field('DATA_STREAM', stream_id, uint8_t, []).
message_field('DATA_STREAM', message_rate, uint16_t, [units='Hz']).
message_field('DATA_STREAM', on_off, uint8_t, []).
message_field('MANUAL_CONTROL', target, uint8_t, []).
message_field('MANUAL_CONTROL', x, int16_t, [invalid='INT16_MAX']).
message_field('MANUAL_CONTROL', y, int16_t, [invalid='INT16_MAX']).
message_field('MANUAL_CONTROL', z, int16_t, [invalid='INT16_MAX']).
message_field('MANUAL_CONTROL', r, int16_t, [invalid='INT16_MAX']).
message_field('MANUAL_CONTROL', buttons, uint16_t, []).
message_field('MANUAL_CONTROL', buttons2, uint16_t, []).
message_field('MANUAL_CONTROL', enabled_extensions, uint8_t, []).
message_field('MANUAL_CONTROL', s, int16_t, []).
message_field('MANUAL_CONTROL', t, int16_t, []).
message_field('MANUAL_CONTROL', aux1, int16_t, []).
message_field('MANUAL_CONTROL', aux2, int16_t, []).
message_field('MANUAL_CONTROL', aux3, int16_t, []).
message_field('MANUAL_CONTROL', aux4, int16_t, []).
message_field('MANUAL_CONTROL', aux5, int16_t, []).
message_field('MANUAL_CONTROL', aux6, int16_t, []).
message_field('RC_CHANNELS_OVERRIDE', target_system, uint8_t, []).
message_field('RC_CHANNELS_OVERRIDE', target_component, uint8_t, []).
message_field('RC_CHANNELS_OVERRIDE', chan1_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_OVERRIDE', chan2_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_OVERRIDE', chan3_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_OVERRIDE', chan4_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_OVERRIDE', chan5_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_OVERRIDE', chan6_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_OVERRIDE', chan7_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_OVERRIDE', chan8_raw, uint16_t, [units=us, invalid='UINT16_MAX']).
message_field('RC_CHANNELS_OVERRIDE', chan9_raw, uint16_t, [units=us, invalid='0']).
message_field('RC_CHANNELS_OVERRIDE', chan10_raw, uint16_t, [units=us, invalid='0']).
message_field('RC_CHANNELS_OVERRIDE', chan11_raw, uint16_t, [units=us, invalid='0']).
message_field('RC_CHANNELS_OVERRIDE', chan12_raw, uint16_t, [units=us, invalid='0']).
message_field('RC_CHANNELS_OVERRIDE', chan13_raw, uint16_t, [units=us, invalid='0']).
message_field('RC_CHANNELS_OVERRIDE', chan14_raw, uint16_t, [units=us, invalid='0']).
message_field('RC_CHANNELS_OVERRIDE', chan15_raw, uint16_t, [units=us, invalid='0']).
message_field('RC_CHANNELS_OVERRIDE', chan16_raw, uint16_t, [units=us, invalid='0']).
message_field('RC_CHANNELS_OVERRIDE', chan17_raw, uint16_t, [units=us, invalid='0']).
message_field('RC_CHANNELS_OVERRIDE', chan18_raw, uint16_t, [units=us, invalid='0']).
message_field('MISSION_ITEM_INT', target_system, uint8_t, []).
message_field('MISSION_ITEM_INT', target_component, uint8_t, []).
message_field('MISSION_ITEM_INT', seq, uint16_t, []).
message_field('MISSION_ITEM_INT', frame, uint8_t, [enum='MAV_FRAME']).
message_field('MISSION_ITEM_INT', command, uint16_t, [enum='MAV_CMD']).
message_field('MISSION_ITEM_INT', current, uint8_t, []).
message_field('MISSION_ITEM_INT', autocontinue, uint8_t, []).
message_field('MISSION_ITEM_INT', param1, float, []).
message_field('MISSION_ITEM_INT', param2, float, []).
message_field('MISSION_ITEM_INT', param3, float, []).
message_field('MISSION_ITEM_INT', param4, float, []).
message_field('MISSION_ITEM_INT', x, int32_t, []).
message_field('MISSION_ITEM_INT', y, int32_t, []).
message_field('MISSION_ITEM_INT', z, float, []).
message_field('MISSION_ITEM_INT', mission_type, uint8_t, [enum='MAV_MISSION_TYPE']).
message_field('VFR_HUD', airspeed, float, [units='m/s']).
message_field('VFR_HUD', groundspeed, float, [units='m/s']).
message_field('VFR_HUD', heading, int16_t, [units=deg]).
message_field('VFR_HUD', throttle, uint16_t, [units='%']).
message_field('VFR_HUD', alt, float, [units=m]).
message_field('VFR_HUD', climb, float, [units='m/s']).
message_field('COMMAND_INT', target_system, uint8_t, []).
message_field('COMMAND_INT', target_component, uint8_t, []).
message_field('COMMAND_INT', frame, uint8_t, [enum='MAV_FRAME']).
message_field('COMMAND_INT', command, uint16_t, [enum='MAV_CMD']).
message_field('COMMAND_INT', current, uint8_t, []).
message_field('COMMAND_INT', autocontinue, uint8_t, []).
message_field('COMMAND_INT', param1, float, [invalid='NaN']).
message_field('COMMAND_INT', param2, float, [invalid='NaN']).
message_field('COMMAND_INT', param3, float, [invalid='NaN']).
message_field('COMMAND_INT', param4, float, [invalid='NaN']).
message_field('COMMAND_INT', x, int32_t, [invalid='INT32_MAX']).
message_field('COMMAND_INT', y, int32_t, [invalid='INT32_MAX']).
message_field('COMMAND_INT', z, float, [invalid='NaN']).
message_field('COMMAND_LONG', target_system, uint8_t, []).
message_field('COMMAND_LONG', target_component, uint8_t, []).
message_field('COMMAND_LONG', command, uint16_t, [enum='MAV_CMD']).
message_field('COMMAND_LONG', confirmation, uint8_t, []).
message_field('COMMAND_LONG', param1, float, [invalid='NaN']).
message_field('COMMAND_LONG', param2, float, [invalid='NaN']).
message_field('COMMAND_LONG', param3, float, [invalid='NaN']).
message_field('COMMAND_LONG', param4, float, [invalid='NaN']).
message_field('COMMAND_LONG', param5, float, [invalid='NaN']).
message_field('COMMAND_LONG', param6, float, [invalid='NaN']).
message_field('COMMAND_LONG', param7, float, [invalid='NaN']).
message_field('COMMAND_ACK', command, uint16_t, [enum='MAV_CMD']).
message_field('COMMAND_ACK', result, uint8_t, [enum='MAV_RESULT']).
message_field('COMMAND_ACK', progress, uint8_t, [invalid='UINT8_MAX', units='%']).
message_field('COMMAND_ACK', result_param2, int32_t, []).
message_field('COMMAND_ACK', target_system, uint8_t, []).
message_field('COMMAND_ACK', target_component, uint8_t, []).
message_field('COMMAND_CANCEL', target_system, uint8_t, []).
message_field('COMMAND_CANCEL', target_component, uint8_t, []).
message_field('COMMAND_CANCEL', command, uint16_t, [enum='MAV_CMD']).
message_field('MANUAL_SETPOINT', time_boot_ms, uint32_t, [units=ms]).
message_field('MANUAL_SETPOINT', roll, float, [units='rad/s']).
message_field('MANUAL_SETPOINT', pitch, float, [units='rad/s']).
message_field('MANUAL_SETPOINT', yaw, float, [units='rad/s']).
message_field('MANUAL_SETPOINT', thrust, float, []).
message_field('MANUAL_SETPOINT', mode_switch, uint8_t, []).
message_field('MANUAL_SETPOINT', manual_override_switch, uint8_t, []).
message_field('SET_ATTITUDE_TARGET', time_boot_ms, uint32_t, [units=ms]).
message_field('SET_ATTITUDE_TARGET', target_system, uint8_t, []).
message_field('SET_ATTITUDE_TARGET', target_component, uint8_t, []).
message_field('SET_ATTITUDE_TARGET', type_mask, uint8_t, [enum='ATTITUDE_TARGET_TYPEMASK', display=bitmask]).
message_field('SET_ATTITUDE_TARGET', q, 'float[4]', []).
message_field('SET_ATTITUDE_TARGET', body_roll_rate, float, [units='rad/s']).
message_field('SET_ATTITUDE_TARGET', body_pitch_rate, float, [units='rad/s']).
message_field('SET_ATTITUDE_TARGET', body_yaw_rate, float, [units='rad/s']).
message_field('SET_ATTITUDE_TARGET', thrust, float, []).
message_field('SET_ATTITUDE_TARGET', thrust_body, 'float[3]', []).
message_field('ATTITUDE_TARGET', time_boot_ms, uint32_t, [units=ms]).
message_field('ATTITUDE_TARGET', type_mask, uint8_t, [enum='ATTITUDE_TARGET_TYPEMASK', display=bitmask]).
message_field('ATTITUDE_TARGET', q, 'float[4]', []).
message_field('ATTITUDE_TARGET', body_roll_rate, float, [units='rad/s']).
message_field('ATTITUDE_TARGET', body_pitch_rate, float, [units='rad/s']).
message_field('ATTITUDE_TARGET', body_yaw_rate, float, [units='rad/s']).
message_field('ATTITUDE_TARGET', thrust, float, []).
message_field('SET_POSITION_TARGET_LOCAL_NED', time_boot_ms, uint32_t, [units=ms]).
message_field('SET_POSITION_TARGET_LOCAL_NED', target_system, uint8_t, []).
message_field('SET_POSITION_TARGET_LOCAL_NED', target_component, uint8_t, []).
message_field('SET_POSITION_TARGET_LOCAL_NED', coordinate_frame, uint8_t, [enum='MAV_FRAME']).
message_field('SET_POSITION_TARGET_LOCAL_NED', type_mask, uint16_t, [enum='POSITION_TARGET_TYPEMASK', display=bitmask]).
message_field('SET_POSITION_TARGET_LOCAL_NED', x, float, [units=m]).
message_field('SET_POSITION_TARGET_LOCAL_NED', y, float, [units=m]).
message_field('SET_POSITION_TARGET_LOCAL_NED', z, float, [units=m]).
message_field('SET_POSITION_TARGET_LOCAL_NED', vx, float, [units='m/s']).
message_field('SET_POSITION_TARGET_LOCAL_NED', vy, float, [units='m/s']).
message_field('SET_POSITION_TARGET_LOCAL_NED', vz, float, [units='m/s']).
message_field('SET_POSITION_TARGET_LOCAL_NED', afx, float, [units='m/s/s']).
message_field('SET_POSITION_TARGET_LOCAL_NED', afy, float, [units='m/s/s']).
message_field('SET_POSITION_TARGET_LOCAL_NED', afz, float, [units='m/s/s']).
message_field('SET_POSITION_TARGET_LOCAL_NED', yaw, float, [units=rad]).
message_field('SET_POSITION_TARGET_LOCAL_NED', yaw_rate, float, [units='rad/s']).
message_field('POSITION_TARGET_LOCAL_NED', time_boot_ms, uint32_t, [units=ms]).
message_field('POSITION_TARGET_LOCAL_NED', coordinate_frame, uint8_t, [enum='MAV_FRAME']).
message_field('POSITION_TARGET_LOCAL_NED', type_mask, uint16_t, [enum='POSITION_TARGET_TYPEMASK', display=bitmask]).
message_field('POSITION_TARGET_LOCAL_NED', x, float, [units=m]).
message_field('POSITION_TARGET_LOCAL_NED', y, float, [units=m]).
message_field('POSITION_TARGET_LOCAL_NED', z, float, [units=m]).
message_field('POSITION_TARGET_LOCAL_NED', vx, float, [units='m/s']).
message_field('POSITION_TARGET_LOCAL_NED', vy, float, [units='m/s']).
message_field('POSITION_TARGET_LOCAL_NED', vz, float, [units='m/s']).
message_field('POSITION_TARGET_LOCAL_NED', afx, float, [units='m/s/s']).
message_field('POSITION_TARGET_LOCAL_NED', afy, float, [units='m/s/s']).
message_field('POSITION_TARGET_LOCAL_NED', afz, float, [units='m/s/s']).
message_field('POSITION_TARGET_LOCAL_NED', yaw, float, [units=rad]).
message_field('POSITION_TARGET_LOCAL_NED', yaw_rate, float, [units='rad/s']).
message_field('SET_POSITION_TARGET_GLOBAL_INT', time_boot_ms, uint32_t, [units=ms]).
message_field('SET_POSITION_TARGET_GLOBAL_INT', target_system, uint8_t, []).
message_field('SET_POSITION_TARGET_GLOBAL_INT', target_component, uint8_t, []).
message_field('SET_POSITION_TARGET_GLOBAL_INT', coordinate_frame, uint8_t, [enum='MAV_FRAME']).
message_field('SET_POSITION_TARGET_GLOBAL_INT', type_mask, uint16_t, [enum='POSITION_TARGET_TYPEMASK', display=bitmask]).
message_field('SET_POSITION_TARGET_GLOBAL_INT', lat_int, int32_t, [units=degE7]).
message_field('SET_POSITION_TARGET_GLOBAL_INT', lon_int, int32_t, [units=degE7]).
message_field('SET_POSITION_TARGET_GLOBAL_INT', alt, float, [units=m]).
message_field('SET_POSITION_TARGET_GLOBAL_INT', vx, float, [units='m/s']).
message_field('SET_POSITION_TARGET_GLOBAL_INT', vy, float, [units='m/s']).
message_field('SET_POSITION_TARGET_GLOBAL_INT', vz, float, [units='m/s']).
message_field('SET_POSITION_TARGET_GLOBAL_INT', afx, float, [units='m/s/s']).
message_field('SET_POSITION_TARGET_GLOBAL_INT', afy, float, [units='m/s/s']).
message_field('SET_POSITION_TARGET_GLOBAL_INT', afz, float, [units='m/s/s']).
message_field('SET_POSITION_TARGET_GLOBAL_INT', yaw, float, [units=rad]).
message_field('SET_POSITION_TARGET_GLOBAL_INT', yaw_rate, float, [units='rad/s']).
message_field('POSITION_TARGET_GLOBAL_INT', time_boot_ms, uint32_t, [units=ms]).
message_field('POSITION_TARGET_GLOBAL_INT', coordinate_frame, uint8_t, [enum='MAV_FRAME']).
message_field('POSITION_TARGET_GLOBAL_INT', type_mask, uint16_t, [enum='POSITION_TARGET_TYPEMASK', display=bitmask]).
message_field('POSITION_TARGET_GLOBAL_INT', lat_int, int32_t, [units=degE7]).
message_field('POSITION_TARGET_GLOBAL_INT', lon_int, int32_t, [units=degE7]).
message_field('POSITION_TARGET_GLOBAL_INT', alt, float, [units=m]).
message_field('POSITION_TARGET_GLOBAL_INT', vx, float, [units='m/s']).
message_field('POSITION_TARGET_GLOBAL_INT', vy, float, [units='m/s']).
message_field('POSITION_TARGET_GLOBAL_INT', vz, float, [units='m/s']).
message_field('POSITION_TARGET_GLOBAL_INT', afx, float, [units='m/s/s']).
message_field('POSITION_TARGET_GLOBAL_INT', afy, float, [units='m/s/s']).
message_field('POSITION_TARGET_GLOBAL_INT', afz, float, [units='m/s/s']).
message_field('POSITION_TARGET_GLOBAL_INT', yaw, float, [units=rad]).
message_field('POSITION_TARGET_GLOBAL_INT', yaw_rate, float, [units='rad/s']).
message_field('LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET', time_boot_ms, uint32_t, [units=ms]).
message_field('LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET', x, float, [units=m]).
message_field('LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET', y, float, [units=m]).
message_field('LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET', z, float, [units=m]).
message_field('LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET', roll, float, [units=rad]).
message_field('LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET', pitch, float, [units=rad]).
message_field('LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET', yaw, float, [units=rad]).
message_field('HIL_STATE', time_usec, uint64_t, [units=us]).
message_field('HIL_STATE', roll, float, [units=rad]).
message_field('HIL_STATE', pitch, float, [units=rad]).
message_field('HIL_STATE', yaw, float, [units=rad]).
message_field('HIL_STATE', rollspeed, float, [units='rad/s']).
message_field('HIL_STATE', pitchspeed, float, [units='rad/s']).
message_field('HIL_STATE', yawspeed, float, [units='rad/s']).
message_field('HIL_STATE', lat, int32_t, [units=degE7]).
message_field('HIL_STATE', lon, int32_t, [units=degE7]).
message_field('HIL_STATE', alt, int32_t, [units=mm]).
message_field('HIL_STATE', vx, int16_t, [units='cm/s']).
message_field('HIL_STATE', vy, int16_t, [units='cm/s']).
message_field('HIL_STATE', vz, int16_t, [units='cm/s']).
message_field('HIL_STATE', xacc, int16_t, [units=mG]).
message_field('HIL_STATE', yacc, int16_t, [units=mG]).
message_field('HIL_STATE', zacc, int16_t, [units=mG]).
message_field('HIL_CONTROLS', time_usec, uint64_t, [units=us]).
message_field('HIL_CONTROLS', roll_ailerons, float, []).
message_field('HIL_CONTROLS', pitch_elevator, float, []).
message_field('HIL_CONTROLS', yaw_rudder, float, []).
message_field('HIL_CONTROLS', throttle, float, []).
message_field('HIL_CONTROLS', aux1, float, []).
message_field('HIL_CONTROLS', aux2, float, []).
message_field('HIL_CONTROLS', aux3, float, []).
message_field('HIL_CONTROLS', aux4, float, []).
message_field('HIL_CONTROLS', mode, uint8_t, [enum='MAV_MODE']).
message_field('HIL_CONTROLS', nav_mode, uint8_t, []).
message_field('HIL_RC_INPUTS_RAW', time_usec, uint64_t, [units=us]).
message_field('HIL_RC_INPUTS_RAW', chan1_raw, uint16_t, [units=us]).
message_field('HIL_RC_INPUTS_RAW', chan2_raw, uint16_t, [units=us]).
message_field('HIL_RC_INPUTS_RAW', chan3_raw, uint16_t, [units=us]).
message_field('HIL_RC_INPUTS_RAW', chan4_raw, uint16_t, [units=us]).
message_field('HIL_RC_INPUTS_RAW', chan5_raw, uint16_t, [units=us]).
message_field('HIL_RC_INPUTS_RAW', chan6_raw, uint16_t, [units=us]).
message_field('HIL_RC_INPUTS_RAW', chan7_raw, uint16_t, [units=us]).
message_field('HIL_RC_INPUTS_RAW', chan8_raw, uint16_t, [units=us]).
message_field('HIL_RC_INPUTS_RAW', chan9_raw, uint16_t, [units=us]).
message_field('HIL_RC_INPUTS_RAW', chan10_raw, uint16_t, [units=us]).
message_field('HIL_RC_INPUTS_RAW', chan11_raw, uint16_t, [units=us]).
message_field('HIL_RC_INPUTS_RAW', chan12_raw, uint16_t, [units=us]).
message_field('HIL_RC_INPUTS_RAW', rssi, uint8_t, [invalid='UINT8_MAX']).
message_field('HIL_ACTUATOR_CONTROLS', time_usec, uint64_t, [units=us]).
message_field('HIL_ACTUATOR_CONTROLS', controls, 'float[16]', []).
message_field('HIL_ACTUATOR_CONTROLS', mode, uint8_t, [enum='MAV_MODE_FLAG', display=bitmask]).
message_field('HIL_ACTUATOR_CONTROLS', flags, uint64_t, [display=bitmask]).
message_field('OPTICAL_FLOW', time_usec, uint64_t, [units=us]).
message_field('OPTICAL_FLOW', sensor_id, uint8_t, []).
message_field('OPTICAL_FLOW', flow_x, int16_t, [units=dpix]).
message_field('OPTICAL_FLOW', flow_y, int16_t, [units=dpix]).
message_field('OPTICAL_FLOW', flow_comp_m_x, float, [units='m/s']).
message_field('OPTICAL_FLOW', flow_comp_m_y, float, [units='m/s']).
message_field('OPTICAL_FLOW', quality, uint8_t, []).
message_field('OPTICAL_FLOW', ground_distance, float, [units=m]).
message_field('OPTICAL_FLOW', flow_rate_x, float, [units='rad/s']).
message_field('OPTICAL_FLOW', flow_rate_y, float, [units='rad/s']).
message_field('GLOBAL_VISION_POSITION_ESTIMATE', usec, uint64_t, [units=us]).
message_field('GLOBAL_VISION_POSITION_ESTIMATE', x, float, [units=m]).
message_field('GLOBAL_VISION_POSITION_ESTIMATE', y, float, [units=m]).
message_field('GLOBAL_VISION_POSITION_ESTIMATE', z, float, [units=m]).
message_field('GLOBAL_VISION_POSITION_ESTIMATE', roll, float, [units=rad]).
message_field('GLOBAL_VISION_POSITION_ESTIMATE', pitch, float, [units=rad]).
message_field('GLOBAL_VISION_POSITION_ESTIMATE', yaw, float, [units=rad]).
message_field('GLOBAL_VISION_POSITION_ESTIMATE', covariance, 'float[21]', [invalid='[NaN:]']).
message_field('GLOBAL_VISION_POSITION_ESTIMATE', reset_counter, uint8_t, []).
message_field('VISION_POSITION_ESTIMATE', usec, uint64_t, [units=us]).
message_field('VISION_POSITION_ESTIMATE', x, float, [units=m]).
message_field('VISION_POSITION_ESTIMATE', y, float, [units=m]).
message_field('VISION_POSITION_ESTIMATE', z, float, [units=m]).
message_field('VISION_POSITION_ESTIMATE', roll, float, [units=rad]).
message_field('VISION_POSITION_ESTIMATE', pitch, float, [units=rad]).
message_field('VISION_POSITION_ESTIMATE', yaw, float, [units=rad]).
message_field('VISION_POSITION_ESTIMATE', covariance, 'float[21]', [invalid='[NaN:]']).
message_field('VISION_POSITION_ESTIMATE', reset_counter, uint8_t, []).
message_field('VISION_SPEED_ESTIMATE', usec, uint64_t, [units=us]).
message_field('VISION_SPEED_ESTIMATE', x, float, [units='m/s']).
message_field('VISION_SPEED_ESTIMATE', y, float, [units='m/s']).
message_field('VISION_SPEED_ESTIMATE', z, float, [units='m/s']).
message_field('VISION_SPEED_ESTIMATE', covariance, 'float[9]', [invalid='[NaN:]']).
message_field('VISION_SPEED_ESTIMATE', reset_counter, uint8_t, []).
message_field('VICON_POSITION_ESTIMATE', usec, uint64_t, [units=us]).
message_field('VICON_POSITION_ESTIMATE', x, float, [units=m]).
message_field('VICON_POSITION_ESTIMATE', y, float, [units=m]).
message_field('VICON_POSITION_ESTIMATE', z, float, [units=m]).
message_field('VICON_POSITION_ESTIMATE', roll, float, [units=rad]).
message_field('VICON_POSITION_ESTIMATE', pitch, float, [units=rad]).
message_field('VICON_POSITION_ESTIMATE', yaw, float, [units=rad]).
message_field('VICON_POSITION_ESTIMATE', covariance, 'float[21]', [invalid='[NaN:]']).
message_field('HIGHRES_IMU', time_usec, uint64_t, [units=us]).
message_field('HIGHRES_IMU', xacc, float, [units='m/s/s']).
message_field('HIGHRES_IMU', yacc, float, [units='m/s/s']).
message_field('HIGHRES_IMU', zacc, float, [units='m/s/s']).
message_field('HIGHRES_IMU', xgyro, float, [units='rad/s']).
message_field('HIGHRES_IMU', ygyro, float, [units='rad/s']).
message_field('HIGHRES_IMU', zgyro, float, [units='rad/s']).
message_field('HIGHRES_IMU', xmag, float, [units=gauss]).
message_field('HIGHRES_IMU', ymag, float, [units=gauss]).
message_field('HIGHRES_IMU', zmag, float, [units=gauss]).
message_field('HIGHRES_IMU', abs_pressure, float, [units=hPa]).
message_field('HIGHRES_IMU', diff_pressure, float, [units=hPa]).
message_field('HIGHRES_IMU', pressure_alt, float, []).
message_field('HIGHRES_IMU', temperature, float, [units=degC]).
message_field('HIGHRES_IMU', fields_updated, uint16_t, [enum='HIGHRES_IMU_UPDATED_FLAGS', display=bitmask]).
message_field('HIGHRES_IMU', id, uint8_t, [instance=true]).
message_field('OPTICAL_FLOW_RAD', time_usec, uint64_t, [units=us]).
message_field('OPTICAL_FLOW_RAD', sensor_id, uint8_t, [instance=true]).
message_field('OPTICAL_FLOW_RAD', integration_time_us, uint32_t, [units=us]).
message_field('OPTICAL_FLOW_RAD', integrated_x, float, [units=rad]).
message_field('OPTICAL_FLOW_RAD', integrated_y, float, [units=rad]).
message_field('OPTICAL_FLOW_RAD', integrated_xgyro, float, [units=rad]).
message_field('OPTICAL_FLOW_RAD', integrated_ygyro, float, [units=rad]).
message_field('OPTICAL_FLOW_RAD', integrated_zgyro, float, [units=rad]).
message_field('OPTICAL_FLOW_RAD', temperature, int16_t, [units=cdegC]).
message_field('OPTICAL_FLOW_RAD', quality, uint8_t, []).
message_field('OPTICAL_FLOW_RAD', time_delta_distance_us, uint32_t, [units=us]).
message_field('OPTICAL_FLOW_RAD', distance, float, [units=m]).
message_field('HIL_SENSOR', time_usec, uint64_t, [units=us]).
message_field('HIL_SENSOR', xacc, float, [units='m/s/s']).
message_field('HIL_SENSOR', yacc, float, [units='m/s/s']).
message_field('HIL_SENSOR', zacc, float, [units='m/s/s']).
message_field('HIL_SENSOR', xgyro, float, [units='rad/s']).
message_field('HIL_SENSOR', ygyro, float, [units='rad/s']).
message_field('HIL_SENSOR', zgyro, float, [units='rad/s']).
message_field('HIL_SENSOR', xmag, float, [units=gauss]).
message_field('HIL_SENSOR', ymag, float, [units=gauss]).
message_field('HIL_SENSOR', zmag, float, [units=gauss]).
message_field('HIL_SENSOR', abs_pressure, float, [units=hPa]).
message_field('HIL_SENSOR', diff_pressure, float, [units=hPa]).
message_field('HIL_SENSOR', pressure_alt, float, []).
message_field('HIL_SENSOR', temperature, float, [units=degC]).
message_field('HIL_SENSOR', fields_updated, uint32_t, [enum='HIL_SENSOR_UPDATED_FLAGS', display=bitmask]).
message_field('HIL_SENSOR', id, uint8_t, []).
message_field('SIM_STATE', q1, float, []).
message_field('SIM_STATE', q2, float, []).
message_field('SIM_STATE', q3, float, []).
message_field('SIM_STATE', q4, float, []).
message_field('SIM_STATE', roll, float, []).
message_field('SIM_STATE', pitch, float, []).
message_field('SIM_STATE', yaw, float, []).
message_field('SIM_STATE', xacc, float, [units='m/s/s']).
message_field('SIM_STATE', yacc, float, [units='m/s/s']).
message_field('SIM_STATE', zacc, float, [units='m/s/s']).
message_field('SIM_STATE', xgyro, float, [units='rad/s']).
message_field('SIM_STATE', ygyro, float, [units='rad/s']).
message_field('SIM_STATE', zgyro, float, [units='rad/s']).
message_field('SIM_STATE', lat, float, [units=deg]).
message_field('SIM_STATE', lon, float, [units=deg]).
message_field('SIM_STATE', alt, float, [units=m]).
message_field('SIM_STATE', std_dev_horz, float, []).
message_field('SIM_STATE', std_dev_vert, float, []).
message_field('SIM_STATE', vn, float, [units='m/s']).
message_field('SIM_STATE', ve, float, [units='m/s']).
message_field('SIM_STATE', vd, float, [units='m/s']).
message_field('SIM_STATE', lat_int, int32_t, [units=degE7, invalid='0']).
message_field('SIM_STATE', lon_int, int32_t, [units=degE7, invalid='0']).
message_field('RADIO_STATUS', rssi, uint8_t, [invalid='UINT8_MAX']).
message_field('RADIO_STATUS', remrssi, uint8_t, [invalid='UINT8_MAX']).
message_field('RADIO_STATUS', txbuf, uint8_t, [units='%']).
message_field('RADIO_STATUS', noise, uint8_t, [invalid='UINT8_MAX']).
message_field('RADIO_STATUS', remnoise, uint8_t, [invalid='UINT8_MAX']).
message_field('RADIO_STATUS', rxerrors, uint16_t, []).
message_field('RADIO_STATUS', fixed, uint16_t, []).
message_field('FILE_TRANSFER_PROTOCOL', target_network, uint8_t, []).
message_field('FILE_TRANSFER_PROTOCOL', target_system, uint8_t, []).
message_field('FILE_TRANSFER_PROTOCOL', target_component, uint8_t, []).
message_field('FILE_TRANSFER_PROTOCOL', payload, 'uint8_t[251]', []).
message_field('TIMESYNC', tc1, int64_t, [units=ns]).
message_field('TIMESYNC', ts1, int64_t, [units=ns]).
message_field('TIMESYNC', target_system, uint8_t, []).
message_field('TIMESYNC', target_component, uint8_t, []).
message_field('CAMERA_TRIGGER', time_usec, uint64_t, [units=us]).
message_field('CAMERA_TRIGGER', seq, uint32_t, []).
message_field('HIL_GPS', time_usec, uint64_t, [units=us]).
message_field('HIL_GPS', fix_type, uint8_t, []).
message_field('HIL_GPS', lat, int32_t, [units=degE7]).
message_field('HIL_GPS', lon, int32_t, [units=degE7]).
message_field('HIL_GPS', alt, int32_t, [units=mm]).
message_field('HIL_GPS', eph, uint16_t, [invalid='UINT16_MAX']).
message_field('HIL_GPS', epv, uint16_t, [invalid='UINT16_MAX']).
message_field('HIL_GPS', vel, uint16_t, [units='cm/s', invalid='UINT16_MAX']).
message_field('HIL_GPS', vn, int16_t, [units='cm/s']).
message_field('HIL_GPS', ve, int16_t, [units='cm/s']).
message_field('HIL_GPS', vd, int16_t, [units='cm/s']).
message_field('HIL_GPS', cog, uint16_t, [units=cdeg, invalid='UINT16_MAX']).
message_field('HIL_GPS', satellites_visible, uint8_t, [invalid='UINT8_MAX']).
message_field('HIL_GPS', id, uint8_t, []).
message_field('HIL_GPS', yaw, uint16_t, [units=cdeg]).
message_field('HIL_OPTICAL_FLOW', time_usec, uint64_t, [units=us]).
message_field('HIL_OPTICAL_FLOW', sensor_id, uint8_t, []).
message_field('HIL_OPTICAL_FLOW', integration_time_us, uint32_t, [units=us]).
message_field('HIL_OPTICAL_FLOW', integrated_x, float, [units=rad]).
message_field('HIL_OPTICAL_FLOW', integrated_y, float, [units=rad]).
message_field('HIL_OPTICAL_FLOW', integrated_xgyro, float, [units=rad]).
message_field('HIL_OPTICAL_FLOW', integrated_ygyro, float, [units=rad]).
message_field('HIL_OPTICAL_FLOW', integrated_zgyro, float, [units=rad]).
message_field('HIL_OPTICAL_FLOW', temperature, int16_t, [units=cdegC]).
message_field('HIL_OPTICAL_FLOW', quality, uint8_t, []).
message_field('HIL_OPTICAL_FLOW', time_delta_distance_us, uint32_t, [units=us]).
message_field('HIL_OPTICAL_FLOW', distance, float, [units=m, invalid='-1.0']).
message_field('HIL_STATE_QUATERNION', time_usec, uint64_t, [units=us]).
message_field('HIL_STATE_QUATERNION', attitude_quaternion, 'float[4]', []).
message_field('HIL_STATE_QUATERNION', rollspeed, float, [units='rad/s']).
message_field('HIL_STATE_QUATERNION', pitchspeed, float, [units='rad/s']).
message_field('HIL_STATE_QUATERNION', yawspeed, float, [units='rad/s']).
message_field('HIL_STATE_QUATERNION', lat, int32_t, [units=degE7]).
message_field('HIL_STATE_QUATERNION', lon, int32_t, [units=degE7]).
message_field('HIL_STATE_QUATERNION', alt, int32_t, [units=mm]).
message_field('HIL_STATE_QUATERNION', vx, int16_t, [units='cm/s']).
message_field('HIL_STATE_QUATERNION', vy, int16_t, [units='cm/s']).
message_field('HIL_STATE_QUATERNION', vz, int16_t, [units='cm/s']).
message_field('HIL_STATE_QUATERNION', ind_airspeed, uint16_t, [units='cm/s']).
message_field('HIL_STATE_QUATERNION', true_airspeed, uint16_t, [units='cm/s']).
message_field('HIL_STATE_QUATERNION', xacc, int16_t, [units=mG]).
message_field('HIL_STATE_QUATERNION', yacc, int16_t, [units=mG]).
message_field('HIL_STATE_QUATERNION', zacc, int16_t, [units=mG]).
message_field('SCALED_IMU2', time_boot_ms, uint32_t, [units=ms]).
message_field('SCALED_IMU2', xacc, int16_t, [units=mG]).
message_field('SCALED_IMU2', yacc, int16_t, [units=mG]).
message_field('SCALED_IMU2', zacc, int16_t, [units=mG]).
message_field('SCALED_IMU2', xgyro, int16_t, [units='mrad/s']).
message_field('SCALED_IMU2', ygyro, int16_t, [units='mrad/s']).
message_field('SCALED_IMU2', zgyro, int16_t, [units='mrad/s']).
message_field('SCALED_IMU2', xmag, int16_t, [units=mgauss]).
message_field('SCALED_IMU2', ymag, int16_t, [units=mgauss]).
message_field('SCALED_IMU2', zmag, int16_t, [units=mgauss]).
message_field('SCALED_IMU2', temperature, int16_t, [units=cdegC, invalid='0']).
message_field('LOG_REQUEST_LIST', target_system, uint8_t, []).
message_field('LOG_REQUEST_LIST', target_component, uint8_t, []).
message_field('LOG_REQUEST_LIST', start, uint16_t, []).
message_field('LOG_REQUEST_LIST', end, uint16_t, []).
message_field('LOG_ENTRY', id, uint16_t, []).
message_field('LOG_ENTRY', num_logs, uint16_t, []).
message_field('LOG_ENTRY', last_log_num, uint16_t, []).
message_field('LOG_ENTRY', time_utc, uint32_t, [units=s, invalid='0']).
message_field('LOG_ENTRY', size, uint32_t, [units=bytes]).
message_field('LOG_REQUEST_DATA', target_system, uint8_t, []).
message_field('LOG_REQUEST_DATA', target_component, uint8_t, []).
message_field('LOG_REQUEST_DATA', id, uint16_t, []).
message_field('LOG_REQUEST_DATA', ofs, uint32_t, []).
message_field('LOG_REQUEST_DATA', count, uint32_t, [units=bytes]).
message_field('LOG_DATA', id, uint16_t, []).
message_field('LOG_DATA', ofs, uint32_t, []).
message_field('LOG_DATA', count, uint8_t, [units=bytes]).
message_field('LOG_DATA', data, 'uint8_t[90]', []).
message_field('LOG_ERASE', target_system, uint8_t, []).
message_field('LOG_ERASE', target_component, uint8_t, []).
message_field('LOG_REQUEST_END', target_system, uint8_t, []).
message_field('LOG_REQUEST_END', target_component, uint8_t, []).
message_field('GPS_INJECT_DATA', target_system, uint8_t, []).
message_field('GPS_INJECT_DATA', target_component, uint8_t, []).
message_field('GPS_INJECT_DATA', len, uint8_t, [units=bytes]).
message_field('GPS_INJECT_DATA', data, 'uint8_t[110]', []).
message_field('GPS2_RAW', time_usec, uint64_t, [units=us]).
message_field('GPS2_RAW', fix_type, uint8_t, [enum='GPS_FIX_TYPE']).
message_field('GPS2_RAW', lat, int32_t, [units=degE7]).
message_field('GPS2_RAW', lon, int32_t, [units=degE7]).
message_field('GPS2_RAW', alt, int32_t, [units=mm]).
message_field('GPS2_RAW', eph, uint16_t, [invalid='UINT16_MAX']).
message_field('GPS2_RAW', epv, uint16_t, [invalid='UINT16_MAX']).
message_field('GPS2_RAW', vel, uint16_t, [units='cm/s', invalid='UINT16_MAX']).
message_field('GPS2_RAW', cog, uint16_t, [units=cdeg, invalid='UINT16_MAX']).
message_field('GPS2_RAW', satellites_visible, uint8_t, [invalid='UINT8_MAX']).
message_field('GPS2_RAW', dgps_numch, uint8_t, []).
message_field('GPS2_RAW', dgps_age, uint32_t, [units=ms]).
message_field('GPS2_RAW', yaw, uint16_t, [units=cdeg, invalid='0']).
message_field('GPS2_RAW', alt_ellipsoid, int32_t, [units=mm]).
message_field('GPS2_RAW', h_acc, uint32_t, [units=mm]).
message_field('GPS2_RAW', v_acc, uint32_t, [units=mm]).
message_field('GPS2_RAW', vel_acc, uint32_t, [units=mm]).
message_field('GPS2_RAW', hdg_acc, uint32_t, [units=degE5]).
message_field('POWER_STATUS', 'Vcc', uint16_t, [units=mV]).
message_field('POWER_STATUS', 'Vservo', uint16_t, [units=mV]).
message_field('POWER_STATUS', flags, uint16_t, [enum='MAV_POWER_STATUS', display=bitmask]).
message_field('SERIAL_CONTROL', device, uint8_t, [enum='SERIAL_CONTROL_DEV']).
message_field('SERIAL_CONTROL', flags, uint8_t, [enum='SERIAL_CONTROL_FLAG', display=bitmask]).
message_field('SERIAL_CONTROL', timeout, uint16_t, [units=ms]).
message_field('SERIAL_CONTROL', baudrate, uint32_t, [units='bits/s']).
message_field('SERIAL_CONTROL', count, uint8_t, [units=bytes]).
message_field('SERIAL_CONTROL', data, 'uint8_t[70]', []).
message_field('SERIAL_CONTROL', target_system, uint8_t, []).
message_field('SERIAL_CONTROL', target_component, uint8_t, []).
message_field('GPS_RTK', time_last_baseline_ms, uint32_t, [units=ms]).
message_field('GPS_RTK', rtk_receiver_id, uint8_t, []).
message_field('GPS_RTK', wn, uint16_t, []).
message_field('GPS_RTK', tow, uint32_t, [units=ms]).
message_field('GPS_RTK', rtk_health, uint8_t, []).
message_field('GPS_RTK', rtk_rate, uint8_t, [units='Hz']).
message_field('GPS_RTK', nsats, uint8_t, []).
message_field('GPS_RTK', baseline_coords_type, uint8_t, [enum='RTK_BASELINE_COORDINATE_SYSTEM']).
message_field('GPS_RTK', baseline_a_mm, int32_t, [units=mm]).
message_field('GPS_RTK', baseline_b_mm, int32_t, [units=mm]).
message_field('GPS_RTK', baseline_c_mm, int32_t, [units=mm]).
message_field('GPS_RTK', accuracy, uint32_t, []).
message_field('GPS_RTK', iar_num_hypotheses, int32_t, []).
message_field('GPS2_RTK', time_last_baseline_ms, uint32_t, [units=ms]).
message_field('GPS2_RTK', rtk_receiver_id, uint8_t, []).
message_field('GPS2_RTK', wn, uint16_t, []).
message_field('GPS2_RTK', tow, uint32_t, [units=ms]).
message_field('GPS2_RTK', rtk_health, uint8_t, []).
message_field('GPS2_RTK', rtk_rate, uint8_t, [units='Hz']).
message_field('GPS2_RTK', nsats, uint8_t, []).
message_field('GPS2_RTK', baseline_coords_type, uint8_t, [enum='RTK_BASELINE_COORDINATE_SYSTEM']).
message_field('GPS2_RTK', baseline_a_mm, int32_t, [units=mm]).
message_field('GPS2_RTK', baseline_b_mm, int32_t, [units=mm]).
message_field('GPS2_RTK', baseline_c_mm, int32_t, [units=mm]).
message_field('GPS2_RTK', accuracy, uint32_t, []).
message_field('GPS2_RTK', iar_num_hypotheses, int32_t, []).
message_field('SCALED_IMU3', time_boot_ms, uint32_t, [units=ms]).
message_field('SCALED_IMU3', xacc, int16_t, [units=mG]).
message_field('SCALED_IMU3', yacc, int16_t, [units=mG]).
message_field('SCALED_IMU3', zacc, int16_t, [units=mG]).
message_field('SCALED_IMU3', xgyro, int16_t, [units='mrad/s']).
message_field('SCALED_IMU3', ygyro, int16_t, [units='mrad/s']).
message_field('SCALED_IMU3', zgyro, int16_t, [units='mrad/s']).
message_field('SCALED_IMU3', xmag, int16_t, [units=mgauss]).
message_field('SCALED_IMU3', ymag, int16_t, [units=mgauss]).
message_field('SCALED_IMU3', zmag, int16_t, [units=mgauss]).
message_field('SCALED_IMU3', temperature, int16_t, [units=cdegC, invalid='0']).
message_field('DATA_TRANSMISSION_HANDSHAKE', type, uint8_t, [enum='MAVLINK_DATA_STREAM_TYPE']).
message_field('DATA_TRANSMISSION_HANDSHAKE', size, uint32_t, [units=bytes]).
message_field('DATA_TRANSMISSION_HANDSHAKE', width, uint16_t, []).
message_field('DATA_TRANSMISSION_HANDSHAKE', height, uint16_t, []).
message_field('DATA_TRANSMISSION_HANDSHAKE', packets, uint16_t, []).
message_field('DATA_TRANSMISSION_HANDSHAKE', payload, uint8_t, [units=bytes]).
message_field('DATA_TRANSMISSION_HANDSHAKE', jpg_quality, uint8_t, [units='%']).
message_field('ENCAPSULATED_DATA', seqnr, uint16_t, []).
message_field('ENCAPSULATED_DATA', data, 'uint8_t[253]', []).
message_field('DISTANCE_SENSOR', time_boot_ms, uint32_t, [units=ms]).
message_field('DISTANCE_SENSOR', min_distance, uint16_t, [units=cm]).
message_field('DISTANCE_SENSOR', max_distance, uint16_t, [units=cm]).
message_field('DISTANCE_SENSOR', current_distance, uint16_t, [units=cm]).
message_field('DISTANCE_SENSOR', type, uint8_t, [enum='MAV_DISTANCE_SENSOR']).
message_field('DISTANCE_SENSOR', id, uint8_t, [instance=true]).
message_field('DISTANCE_SENSOR', orientation, uint8_t, [enum='MAV_SENSOR_ORIENTATION']).
message_field('DISTANCE_SENSOR', covariance, uint8_t, [units='cm^2', invalid='UINT8_MAX']).
message_field('DISTANCE_SENSOR', horizontal_fov, float, [units=rad, invalid='0']).
message_field('DISTANCE_SENSOR', vertical_fov, float, [units=rad, invalid='0']).
message_field('DISTANCE_SENSOR', quaternion, 'float[4]', [invalid='[0]']).
message_field('DISTANCE_SENSOR', signal_quality, uint8_t, [units='%', invalid='0']).
message_field('TERRAIN_REQUEST', lat, int32_t, [units=degE7]).
message_field('TERRAIN_REQUEST', lon, int32_t, [units=degE7]).
message_field('TERRAIN_REQUEST', grid_spacing, uint16_t, [units=m]).
message_field('TERRAIN_REQUEST', mask, uint64_t, [display=bitmask, print_format='0x%07x']).
message_field('TERRAIN_DATA', lat, int32_t, [units=degE7]).
message_field('TERRAIN_DATA', lon, int32_t, [units=degE7]).
message_field('TERRAIN_DATA', grid_spacing, uint16_t, [units=m]).
message_field('TERRAIN_DATA', gridbit, uint8_t, []).
message_field('TERRAIN_DATA', data, 'int16_t[16]', [units=m]).
message_field('TERRAIN_CHECK', lat, int32_t, [units=degE7]).
message_field('TERRAIN_CHECK', lon, int32_t, [units=degE7]).
message_field('TERRAIN_REPORT', lat, int32_t, [units=degE7]).
message_field('TERRAIN_REPORT', lon, int32_t, [units=degE7]).
message_field('TERRAIN_REPORT', spacing, uint16_t, []).
message_field('TERRAIN_REPORT', terrain_height, float, [units=m]).
message_field('TERRAIN_REPORT', current_height, float, [units=m]).
message_field('TERRAIN_REPORT', pending, uint16_t, []).
message_field('TERRAIN_REPORT', loaded, uint16_t, []).
message_field('SCALED_PRESSURE2', time_boot_ms, uint32_t, [units=ms]).
message_field('SCALED_PRESSURE2', press_abs, float, [units=hPa]).
message_field('SCALED_PRESSURE2', press_diff, float, [units=hPa]).
message_field('SCALED_PRESSURE2', temperature, int16_t, [units=cdegC]).
message_field('SCALED_PRESSURE2', temperature_press_diff, int16_t, [units=cdegC, invalid='0']).
message_field('ATT_POS_MOCAP', time_usec, uint64_t, [units=us]).
message_field('ATT_POS_MOCAP', q, 'float[4]', []).
message_field('ATT_POS_MOCAP', x, float, [units=m]).
message_field('ATT_POS_MOCAP', y, float, [units=m]).
message_field('ATT_POS_MOCAP', z, float, [units=m]).
message_field('ATT_POS_MOCAP', covariance, 'float[21]', [invalid='[NaN:]']).
message_field('SET_ACTUATOR_CONTROL_TARGET', time_usec, uint64_t, [units=us]).
message_field('SET_ACTUATOR_CONTROL_TARGET', group_mlx, uint8_t, []).
message_field('SET_ACTUATOR_CONTROL_TARGET', target_system, uint8_t, []).
message_field('SET_ACTUATOR_CONTROL_TARGET', target_component, uint8_t, []).
message_field('SET_ACTUATOR_CONTROL_TARGET', controls, 'float[8]', []).
message_field('ACTUATOR_CONTROL_TARGET', time_usec, uint64_t, [units=us]).
message_field('ACTUATOR_CONTROL_TARGET', group_mlx, uint8_t, []).
message_field('ACTUATOR_CONTROL_TARGET', controls, 'float[8]', []).
message_field('ALTITUDE', time_usec, uint64_t, [units=us]).
message_field('ALTITUDE', altitude_monotonic, float, [units=m]).
message_field('ALTITUDE', altitude_amsl, float, [units=m]).
message_field('ALTITUDE', altitude_local, float, [units=m]).
message_field('ALTITUDE', altitude_relative, float, [units=m]).
message_field('ALTITUDE', altitude_terrain, float, [units=m]).
message_field('ALTITUDE', bottom_clearance, float, [units=m]).
message_field('RESOURCE_REQUEST', request_id, uint8_t, []).
message_field('RESOURCE_REQUEST', uri_type, uint8_t, []).
message_field('RESOURCE_REQUEST', uri, 'uint8_t[120]', []).
message_field('RESOURCE_REQUEST', transfer_type, uint8_t, []).
message_field('RESOURCE_REQUEST', storage, 'uint8_t[120]', []).
message_field('SCALED_PRESSURE3', time_boot_ms, uint32_t, [units=ms]).
message_field('SCALED_PRESSURE3', press_abs, float, [units=hPa]).
message_field('SCALED_PRESSURE3', press_diff, float, [units=hPa]).
message_field('SCALED_PRESSURE3', temperature, int16_t, [units=cdegC]).
message_field('SCALED_PRESSURE3', temperature_press_diff, int16_t, [units=cdegC, invalid='0']).
message_field('FOLLOW_TARGET', timestamp, uint64_t, [units=ms]).
message_field('FOLLOW_TARGET', est_capabilities, uint8_t, []).
message_field('FOLLOW_TARGET', lat, int32_t, [units=degE7]).
message_field('FOLLOW_TARGET', lon, int32_t, [units=degE7]).
message_field('FOLLOW_TARGET', alt, float, [units=m]).
message_field('FOLLOW_TARGET', vel, 'float[3]', [units='m/s', invalid='[0]']).
message_field('FOLLOW_TARGET', acc, 'float[3]', [units='m/s/s', invalid='[0]']).
message_field('FOLLOW_TARGET', attitude_q, 'float[4]', [invalid='[0]']).
message_field('FOLLOW_TARGET', rates, 'float[3]', [invalid='[0]']).
message_field('FOLLOW_TARGET', position_cov, 'float[3]', []).
message_field('FOLLOW_TARGET', custom_state, uint64_t, []).
message_field('CONTROL_SYSTEM_STATE', time_usec, uint64_t, [units=us]).
message_field('CONTROL_SYSTEM_STATE', x_acc, float, [units='m/s/s']).
message_field('CONTROL_SYSTEM_STATE', y_acc, float, [units='m/s/s']).
message_field('CONTROL_SYSTEM_STATE', z_acc, float, [units='m/s/s']).
message_field('CONTROL_SYSTEM_STATE', x_vel, float, [units='m/s']).
message_field('CONTROL_SYSTEM_STATE', y_vel, float, [units='m/s']).
message_field('CONTROL_SYSTEM_STATE', z_vel, float, [units='m/s']).
message_field('CONTROL_SYSTEM_STATE', x_pos, float, [units=m]).
message_field('CONTROL_SYSTEM_STATE', y_pos, float, [units=m]).
message_field('CONTROL_SYSTEM_STATE', z_pos, float, [units=m]).
message_field('CONTROL_SYSTEM_STATE', airspeed, float, [units='m/s', invalid='-1']).
message_field('CONTROL_SYSTEM_STATE', vel_variance, 'float[3]', []).
message_field('CONTROL_SYSTEM_STATE', pos_variance, 'float[3]', []).
message_field('CONTROL_SYSTEM_STATE', q, 'float[4]', []).
message_field('CONTROL_SYSTEM_STATE', roll_rate, float, [units='rad/s']).
message_field('CONTROL_SYSTEM_STATE', pitch_rate, float, [units='rad/s']).
message_field('CONTROL_SYSTEM_STATE', yaw_rate, float, [units='rad/s']).
message_field('BATTERY_STATUS', id, uint8_t, [instance=true]).
message_field('BATTERY_STATUS', battery_function, uint8_t, [enum='MAV_BATTERY_FUNCTION']).
message_field('BATTERY_STATUS', type, uint8_t, [enum='MAV_BATTERY_TYPE']).
message_field('BATTERY_STATUS', temperature, int16_t, [units=cdegC, invalid='INT16_MAX']).
message_field('BATTERY_STATUS', voltages, 'uint16_t[10]', [units=mV, invalid='[UINT16_MAX]']).
message_field('BATTERY_STATUS', current_battery, int16_t, [units=cA, invalid='-1']).
message_field('BATTERY_STATUS', current_consumed, int32_t, [units=mAh, invalid='-1']).
message_field('BATTERY_STATUS', energy_consumed, int32_t, [units=hJ, invalid='-1']).
message_field('BATTERY_STATUS', battery_remaining, int8_t, [units='%', invalid='-1']).
message_field('BATTERY_STATUS', time_remaining, int32_t, [units=s, invalid='0']).
message_field('BATTERY_STATUS', charge_state, uint8_t, [enum='MAV_BATTERY_CHARGE_STATE']).
message_field('BATTERY_STATUS', voltages_ext, 'uint16_t[4]', [units=mV, invalid='[0]']).
message_field('BATTERY_STATUS', mode, uint8_t, [enum='MAV_BATTERY_MODE']).
message_field('BATTERY_STATUS', fault_bitmask, uint32_t, [display=bitmask, enum='MAV_BATTERY_FAULT']).
message_field('AUTOPILOT_VERSION', capabilities, uint64_t, [enum='MAV_PROTOCOL_CAPABILITY', display=bitmask]).
message_field('AUTOPILOT_VERSION', flight_sw_version, uint32_t, []).
message_field('AUTOPILOT_VERSION', middleware_sw_version, uint32_t, []).
message_field('AUTOPILOT_VERSION', os_sw_version, uint32_t, []).
message_field('AUTOPILOT_VERSION', board_version, uint32_t, []).
message_field('AUTOPILOT_VERSION', flight_custom_version, 'uint8_t[8]', []).
message_field('AUTOPILOT_VERSION', middleware_custom_version, 'uint8_t[8]', []).
message_field('AUTOPILOT_VERSION', os_custom_version, 'uint8_t[8]', []).
message_field('AUTOPILOT_VERSION', vendor_id, uint16_t, []).
message_field('AUTOPILOT_VERSION', product_id, uint16_t, []).
message_field('AUTOPILOT_VERSION', uid, uint64_t, []).
message_field('AUTOPILOT_VERSION', uid2, 'uint8_t[18]', []).
message_field('LANDING_TARGET', time_usec, uint64_t, [units=us]).
message_field('LANDING_TARGET', target_num, uint8_t, []).
message_field('LANDING_TARGET', frame, uint8_t, [enum='MAV_FRAME']).
message_field('LANDING_TARGET', angle_x, float, [units=rad]).
message_field('LANDING_TARGET', angle_y, float, [units=rad]).
message_field('LANDING_TARGET', distance, float, [units=m]).
message_field('LANDING_TARGET', size_x, float, [units=rad]).
message_field('LANDING_TARGET', size_y, float, [units=rad]).
message_field('LANDING_TARGET', x, float, [units=m]).
message_field('LANDING_TARGET', y, float, [units=m]).
message_field('LANDING_TARGET', z, float, [units=m]).
message_field('LANDING_TARGET', q, 'float[4]', []).
message_field('LANDING_TARGET', type, uint8_t, [enum='LANDING_TARGET_TYPE']).
message_field('LANDING_TARGET', position_valid, uint8_t, [invalid='0']).
message_field('FENCE_STATUS', breach_status, uint8_t, []).
message_field('FENCE_STATUS', breach_count, uint16_t, []).
message_field('FENCE_STATUS', breach_type, uint8_t, [enum='FENCE_BREACH']).
message_field('FENCE_STATUS', breach_time, uint32_t, [units=ms]).
message_field('FENCE_STATUS', breach_mitigation, uint8_t, [enum='FENCE_MITIGATE']).
message_field('MAG_CAL_REPORT', compass_id, uint8_t, [instance=true]).
message_field('MAG_CAL_REPORT', cal_mask, uint8_t, [display=bitmask]).
message_field('MAG_CAL_REPORT', cal_status, uint8_t, [enum='MAG_CAL_STATUS']).
message_field('MAG_CAL_REPORT', autosaved, uint8_t, []).
message_field('MAG_CAL_REPORT', fitness, float, [units=mgauss]).
message_field('MAG_CAL_REPORT', ofs_x, float, []).
message_field('MAG_CAL_REPORT', ofs_y, float, []).
message_field('MAG_CAL_REPORT', ofs_z, float, []).
message_field('MAG_CAL_REPORT', diag_x, float, []).
message_field('MAG_CAL_REPORT', diag_y, float, []).
message_field('MAG_CAL_REPORT', diag_z, float, []).
message_field('MAG_CAL_REPORT', offdiag_x, float, []).
message_field('MAG_CAL_REPORT', offdiag_y, float, []).
message_field('MAG_CAL_REPORT', offdiag_z, float, []).
message_field('MAG_CAL_REPORT', orientation_confidence, float, []).
message_field('MAG_CAL_REPORT', old_orientation, uint8_t, [enum='MAV_SENSOR_ORIENTATION']).
message_field('MAG_CAL_REPORT', new_orientation, uint8_t, [enum='MAV_SENSOR_ORIENTATION']).
message_field('MAG_CAL_REPORT', scale_factor, float, []).
message_field('EFI_STATUS', health, uint8_t, []).
message_field('EFI_STATUS', ecu_index, float, []).
message_field('EFI_STATUS', rpm, float, []).
message_field('EFI_STATUS', fuel_consumed, float, [units='cm^3']).
message_field('EFI_STATUS', fuel_flow, float, [units='cm^3/min']).
message_field('EFI_STATUS', engine_load, float, [units='%']).
message_field('EFI_STATUS', throttle_position, float, [units='%']).
message_field('EFI_STATUS', spark_dwell_time, float, [units=ms]).
message_field('EFI_STATUS', barometric_pressure, float, [units=kPa]).
message_field('EFI_STATUS', intake_manifold_pressure, float, [units=kPa]).
message_field('EFI_STATUS', intake_manifold_temperature, float, [units=degC]).
message_field('EFI_STATUS', cylinder_head_temperature, float, [units=degC]).
message_field('EFI_STATUS', ignition_timing, float, [units=deg]).
message_field('EFI_STATUS', injection_time, float, [units=ms]).
message_field('EFI_STATUS', exhaust_gas_temperature, float, [units=degC]).
message_field('EFI_STATUS', throttle_out, float, [units='%']).
message_field('EFI_STATUS', pt_compensation, float, []).
message_field('EFI_STATUS', ignition_voltage, float, [units='V']).
message_field('EFI_STATUS', fuel_pressure, float, [units=kPa]).
message_field('ESTIMATOR_STATUS', time_usec, uint64_t, [units=us]).
message_field('ESTIMATOR_STATUS', flags, uint16_t, [enum='ESTIMATOR_STATUS_FLAGS', display=bitmask]).
message_field('ESTIMATOR_STATUS', vel_ratio, float, []).
message_field('ESTIMATOR_STATUS', pos_horiz_ratio, float, []).
message_field('ESTIMATOR_STATUS', pos_vert_ratio, float, []).
message_field('ESTIMATOR_STATUS', mag_ratio, float, []).
message_field('ESTIMATOR_STATUS', hagl_ratio, float, []).
message_field('ESTIMATOR_STATUS', tas_ratio, float, []).
message_field('ESTIMATOR_STATUS', pos_horiz_accuracy, float, [units=m]).
message_field('ESTIMATOR_STATUS', pos_vert_accuracy, float, [units=m]).
message_field('WIND_COV', time_usec, uint64_t, [units=us]).
message_field('WIND_COV', wind_x, float, [units='m/s', invalid='NaN']).
message_field('WIND_COV', wind_y, float, [units='m/s', invalid='NaN']).
message_field('WIND_COV', wind_z, float, [units='m/s', invalid='NaN']).
message_field('WIND_COV', var_horiz, float, [units='m/s', invalid='NaN']).
message_field('WIND_COV', var_vert, float, [units='m/s', invalid='NaN']).
message_field('WIND_COV', wind_alt, float, [units=m, invalid='NaN']).
message_field('WIND_COV', horiz_accuracy, float, [units='m/s', invalid='0']).
message_field('WIND_COV', vert_accuracy, float, [units='m/s', invalid='0']).
message_field('GPS_INPUT', time_usec, uint64_t, [units=us]).
message_field('GPS_INPUT', gps_id, uint8_t, [instance=true]).
message_field('GPS_INPUT', ignore_flags, uint16_t, [enum='GPS_INPUT_IGNORE_FLAGS', display=bitmask]).
message_field('GPS_INPUT', time_week_ms, uint32_t, [units=ms]).
message_field('GPS_INPUT', time_week, uint16_t, []).
message_field('GPS_INPUT', fix_type, uint8_t, []).
message_field('GPS_INPUT', lat, int32_t, [units=degE7]).
message_field('GPS_INPUT', lon, int32_t, [units=degE7]).
message_field('GPS_INPUT', alt, float, [units=m]).
message_field('GPS_INPUT', hdop, float, [invalid='UINT16_MAX']).
message_field('GPS_INPUT', vdop, float, [invalid='UINT16_MAX']).
message_field('GPS_INPUT', vn, float, [units='m/s']).
message_field('GPS_INPUT', ve, float, [units='m/s']).
message_field('GPS_INPUT', vd, float, [units='m/s']).
message_field('GPS_INPUT', speed_accuracy, float, [units='m/s']).
message_field('GPS_INPUT', horiz_accuracy, float, [units=m]).
message_field('GPS_INPUT', vert_accuracy, float, [units=m]).
message_field('GPS_INPUT', satellites_visible, uint8_t, []).
message_field('GPS_INPUT', yaw, uint16_t, [units=cdeg]).
message_field('GPS_RTCM_DATA', flags, uint8_t, []).
message_field('GPS_RTCM_DATA', len, uint8_t, [units=bytes]).
message_field('GPS_RTCM_DATA', data, 'uint8_t[180]', []).
message_field('HIGH_LATENCY', base_mode, uint8_t, [enum='MAV_MODE_FLAG', display=bitmask]).
message_field('HIGH_LATENCY', custom_mode, uint32_t, [display=bitmask]).
message_field('HIGH_LATENCY', landed_state, uint8_t, [enum='MAV_LANDED_STATE']).
message_field('HIGH_LATENCY', roll, int16_t, [units=cdeg]).
message_field('HIGH_LATENCY', pitch, int16_t, [units=cdeg]).
message_field('HIGH_LATENCY', heading, uint16_t, [units=cdeg]).
message_field('HIGH_LATENCY', throttle, int8_t, [units='%']).
message_field('HIGH_LATENCY', heading_sp, int16_t, [units=cdeg]).
message_field('HIGH_LATENCY', latitude, int32_t, [units=degE7]).
message_field('HIGH_LATENCY', longitude, int32_t, [units=degE7]).
message_field('HIGH_LATENCY', altitude_amsl, int16_t, [units=m]).
message_field('HIGH_LATENCY', altitude_sp, int16_t, [units=m]).
message_field('HIGH_LATENCY', airspeed, uint8_t, [units='m/s']).
message_field('HIGH_LATENCY', airspeed_sp, uint8_t, [units='m/s']).
message_field('HIGH_LATENCY', groundspeed, uint8_t, [units='m/s']).
message_field('HIGH_LATENCY', climb_rate, int8_t, [units='m/s']).
message_field('HIGH_LATENCY', gps_nsat, uint8_t, [invalid='UINT8_MAX']).
message_field('HIGH_LATENCY', gps_fix_type, uint8_t, [enum='GPS_FIX_TYPE']).
message_field('HIGH_LATENCY', battery_remaining, uint8_t, [units='%']).
message_field('HIGH_LATENCY', temperature, int8_t, [units=degC]).
message_field('HIGH_LATENCY', temperature_air, int8_t, [units=degC]).
message_field('HIGH_LATENCY', failsafe, uint8_t, []).
message_field('HIGH_LATENCY', wp_num, uint8_t, []).
message_field('HIGH_LATENCY', wp_distance, uint16_t, [units=m]).
message_field('HIGH_LATENCY2', timestamp, uint32_t, [units=ms]).
message_field('HIGH_LATENCY2', type, uint8_t, [enum='MAV_TYPE']).
message_field('HIGH_LATENCY2', autopilot, uint8_t, [enum='MAV_AUTOPILOT']).
message_field('HIGH_LATENCY2', custom_mode, uint16_t, [display=bitmask]).
message_field('HIGH_LATENCY2', latitude, int32_t, [units=degE7]).
message_field('HIGH_LATENCY2', longitude, int32_t, [units=degE7]).
message_field('HIGH_LATENCY2', altitude, int16_t, [units=m]).
message_field('HIGH_LATENCY2', target_altitude, int16_t, [units=m]).
message_field('HIGH_LATENCY2', heading, uint8_t, [units='deg/2']).
message_field('HIGH_LATENCY2', target_heading, uint8_t, [units='deg/2']).
message_field('HIGH_LATENCY2', target_distance, uint16_t, [units=dam]).
message_field('HIGH_LATENCY2', throttle, uint8_t, [units='%']).
message_field('HIGH_LATENCY2', airspeed, uint8_t, [units='m/s*5']).
message_field('HIGH_LATENCY2', airspeed_sp, uint8_t, [units='m/s*5']).
message_field('HIGH_LATENCY2', groundspeed, uint8_t, [units='m/s*5']).
message_field('HIGH_LATENCY2', windspeed, uint8_t, [units='m/s*5']).
message_field('HIGH_LATENCY2', wind_heading, uint8_t, [units='deg/2']).
message_field('HIGH_LATENCY2', eph, uint8_t, [units=dm]).
message_field('HIGH_LATENCY2', epv, uint8_t, [units=dm]).
message_field('HIGH_LATENCY2', temperature_air, int8_t, [units=degC]).
message_field('HIGH_LATENCY2', climb_rate, int8_t, [units='dm/s']).
message_field('HIGH_LATENCY2', battery, int8_t, [units='%', invalid='-1']).
message_field('HIGH_LATENCY2', wp_num, uint16_t, []).
message_field('HIGH_LATENCY2', failure_flags, uint16_t, [enum='HL_FAILURE_FLAG', display=bitmask]).
message_field('HIGH_LATENCY2', custom0, int8_t, []).
message_field('HIGH_LATENCY2', custom1, int8_t, []).
message_field('HIGH_LATENCY2', custom2, int8_t, []).
message_field('VIBRATION', time_usec, uint64_t, [units=us]).
message_field('VIBRATION', vibration_x, float, []).
message_field('VIBRATION', vibration_y, float, []).
message_field('VIBRATION', vibration_z, float, []).
message_field('VIBRATION', clipping_0, uint32_t, []).
message_field('VIBRATION', clipping_1, uint32_t, []).
message_field('VIBRATION', clipping_2, uint32_t, []).
message_field('HOME_POSITION', latitude, int32_t, [units=degE7]).
message_field('HOME_POSITION', longitude, int32_t, [units=degE7]).
message_field('HOME_POSITION', altitude, int32_t, [units=mm]).
message_field('HOME_POSITION', x, float, [units=m]).
message_field('HOME_POSITION', y, float, [units=m]).
message_field('HOME_POSITION', z, float, [units=m]).
message_field('HOME_POSITION', q, 'float[4]', [invalid='[NaN]']).
message_field('HOME_POSITION', approach_x, float, [units=m]).
message_field('HOME_POSITION', approach_y, float, [units=m]).
message_field('HOME_POSITION', approach_z, float, [units=m]).
message_field('HOME_POSITION', time_usec, uint64_t, [units=us]).
message_field('SET_HOME_POSITION', target_system, uint8_t, []).
message_field('SET_HOME_POSITION', latitude, int32_t, [units=degE7]).
message_field('SET_HOME_POSITION', longitude, int32_t, [units=degE7]).
message_field('SET_HOME_POSITION', altitude, int32_t, [units=mm]).
message_field('SET_HOME_POSITION', x, float, [units=m]).
message_field('SET_HOME_POSITION', y, float, [units=m]).
message_field('SET_HOME_POSITION', z, float, [units=m]).
message_field('SET_HOME_POSITION', q, 'float[4]', []).
message_field('SET_HOME_POSITION', approach_x, float, [units=m]).
message_field('SET_HOME_POSITION', approach_y, float, [units=m]).
message_field('SET_HOME_POSITION', approach_z, float, [units=m]).
message_field('SET_HOME_POSITION', time_usec, uint64_t, [units=us]).
message_field('MESSAGE_INTERVAL', message_id, uint16_t, []).
message_field('MESSAGE_INTERVAL', interval_us, int32_t, [units=us]).
message_field('EXTENDED_SYS_STATE', vtol_state, uint8_t, [enum='MAV_VTOL_STATE']).
message_field('EXTENDED_SYS_STATE', landed_state, uint8_t, [enum='MAV_LANDED_STATE']).
message_field('ADSB_VEHICLE', 'ICAO_address', uint32_t, []).
message_field('ADSB_VEHICLE', lat, int32_t, [units=degE7]).
message_field('ADSB_VEHICLE', lon, int32_t, [units=degE7]).
message_field('ADSB_VEHICLE', altitude_type, uint8_t, [enum='ADSB_ALTITUDE_TYPE']).
message_field('ADSB_VEHICLE', altitude, int32_t, [units=mm]).
message_field('ADSB_VEHICLE', heading, uint16_t, [units=cdeg]).
message_field('ADSB_VEHICLE', hor_velocity, uint16_t, [units='cm/s']).
message_field('ADSB_VEHICLE', ver_velocity, int16_t, [units='cm/s']).
message_field('ADSB_VEHICLE', callsign, 'char[9]', []).
message_field('ADSB_VEHICLE', emitter_type, uint8_t, [enum='ADSB_EMITTER_TYPE']).
message_field('ADSB_VEHICLE', tslc, uint8_t, [units=s]).
message_field('ADSB_VEHICLE', flags, uint16_t, [enum='ADSB_FLAGS', display=bitmask]).
message_field('ADSB_VEHICLE', squawk, uint16_t, []).
message_field('COLLISION', src, uint8_t, [enum='MAV_COLLISION_SRC']).
message_field('COLLISION', id, uint32_t, []).
message_field('COLLISION', action, uint8_t, [enum='MAV_COLLISION_ACTION']).
message_field('COLLISION', threat_level, uint8_t, [enum='MAV_COLLISION_THREAT_LEVEL']).
message_field('COLLISION', time_to_minimum_delta, float, [units=s]).
message_field('COLLISION', altitude_minimum_delta, float, [units=m]).
message_field('COLLISION', horizontal_minimum_delta, float, [units=m]).
message_field('V2_EXTENSION', target_network, uint8_t, []).
message_field('V2_EXTENSION', target_system, uint8_t, []).
message_field('V2_EXTENSION', target_component, uint8_t, []).
message_field('V2_EXTENSION', message_type, uint16_t, []).
message_field('V2_EXTENSION', payload, 'uint8_t[249]', []).
message_field('MEMORY_VECT', address, uint16_t, []).
message_field('MEMORY_VECT', ver, uint8_t, [invalid='0']).
message_field('MEMORY_VECT', type, uint8_t, []).
message_field('MEMORY_VECT', value, 'int8_t[32]', []).
message_field('DEBUG_VECT', name, 'char[10]', [instance=true]).
message_field('DEBUG_VECT', time_usec, uint64_t, [units=us]).
message_field('DEBUG_VECT', x, float, []).
message_field('DEBUG_VECT', y, float, []).
message_field('DEBUG_VECT', z, float, []).
message_field('NAMED_VALUE_FLOAT', time_boot_ms, uint32_t, [units=ms]).
message_field('NAMED_VALUE_FLOAT', name, 'char[10]', [instance=true]).
message_field('NAMED_VALUE_FLOAT', value, float, []).
message_field('NAMED_VALUE_INT', time_boot_ms, uint32_t, [units=ms]).
message_field('NAMED_VALUE_INT', name, 'char[10]', [instance=true]).
message_field('NAMED_VALUE_INT', value, int32_t, []).
message_field('STATUSTEXT', severity, uint8_t, [enum='MAV_SEVERITY']).
message_field('STATUSTEXT', text, 'char[50]', []).
message_field('STATUSTEXT', id, uint16_t, []).
message_field('STATUSTEXT', chunk_seq, uint8_t, []).
message_field('DEBUG', time_boot_ms, uint32_t, [units=ms]).
message_field('DEBUG', ind, uint8_t, []).
message_field('DEBUG', value, float, []).
message_field('SETUP_SIGNING', target_system, uint8_t, []).
message_field('SETUP_SIGNING', target_component, uint8_t, []).
message_field('SETUP_SIGNING', secret_key, 'uint8_t[32]', []).
message_field('SETUP_SIGNING', initial_timestamp, uint64_t, []).
message_field('BUTTON_CHANGE', time_boot_ms, uint32_t, [units=ms]).
message_field('BUTTON_CHANGE', last_change_ms, uint32_t, [units=ms]).
message_field('BUTTON_CHANGE', state, uint8_t, [display=bitmask]).
message_field('PLAY_TUNE', target_system, uint8_t, []).
message_field('PLAY_TUNE', target_component, uint8_t, []).
message_field('PLAY_TUNE', tune, 'char[30]', []).
message_field('PLAY_TUNE', tune2, 'char[200]', []).
message_field('CAMERA_INFORMATION', time_boot_ms, uint32_t, [units=ms]).
message_field('CAMERA_INFORMATION', vendor_name, 'uint8_t[32]', []).
message_field('CAMERA_INFORMATION', model_name, 'uint8_t[32]', []).
message_field('CAMERA_INFORMATION', firmware_version, uint32_t, [invalid='0']).
message_field('CAMERA_INFORMATION', focal_length, float, [units=mm, invalid='NaN']).
message_field('CAMERA_INFORMATION', sensor_size_h, float, [units=mm, invalid='NaN']).
message_field('CAMERA_INFORMATION', sensor_size_v, float, [units=mm, invalid='NaN']).
message_field('CAMERA_INFORMATION', resolution_h, uint16_t, [units=pix, invalid='0']).
message_field('CAMERA_INFORMATION', resolution_v, uint16_t, [units=pix, invalid='0']).
message_field('CAMERA_INFORMATION', lens_id, uint8_t, [invalid='0']).
message_field('CAMERA_INFORMATION', flags, uint32_t, [enum='CAMERA_CAP_FLAGS', display=bitmask]).
message_field('CAMERA_INFORMATION', cam_definition_version, uint16_t, []).
message_field('CAMERA_INFORMATION', cam_definition_uri, 'char[140]', []).
message_field('CAMERA_INFORMATION', gimbal_device_id, uint8_t, [invalid='0']).
message_field('CAMERA_SETTINGS', time_boot_ms, uint32_t, [units=ms]).
message_field('CAMERA_SETTINGS', mode_id, uint8_t, [enum='CAMERA_MODE']).
message_field('CAMERA_SETTINGS', zoomLevel, float, [invalid='NaN']).
message_field('CAMERA_SETTINGS', focusLevel, float, [invalid='NaN']).
message_field('STORAGE_INFORMATION', time_boot_ms, uint32_t, [units=ms]).
message_field('STORAGE_INFORMATION', storage_id, uint8_t, [instance=true]).
message_field('STORAGE_INFORMATION', storage_count, uint8_t, []).
message_field('STORAGE_INFORMATION', status, uint8_t, [enum='STORAGE_STATUS']).
message_field('STORAGE_INFORMATION', total_capacity, float, [units='MiB']).
message_field('STORAGE_INFORMATION', used_capacity, float, [units='MiB']).
message_field('STORAGE_INFORMATION', available_capacity, float, [units='MiB']).
message_field('STORAGE_INFORMATION', read_speed, float, [units='MiB/s']).
message_field('STORAGE_INFORMATION', write_speed, float, [units='MiB/s']).
message_field('STORAGE_INFORMATION', type, uint8_t, [enum='STORAGE_TYPE']).
message_field('STORAGE_INFORMATION', name, 'char[32]', []).
message_field('STORAGE_INFORMATION', storage_usage, uint8_t, [enum='STORAGE_USAGE_FLAG']).
message_field('CAMERA_CAPTURE_STATUS', time_boot_ms, uint32_t, [units=ms]).
message_field('CAMERA_CAPTURE_STATUS', image_status, uint8_t, []).
message_field('CAMERA_CAPTURE_STATUS', video_status, uint8_t, []).
message_field('CAMERA_CAPTURE_STATUS', image_interval, float, [units=s]).
message_field('CAMERA_CAPTURE_STATUS', recording_time_ms, uint32_t, [units=ms]).
message_field('CAMERA_CAPTURE_STATUS', available_capacity, float, [units='MiB']).
message_field('CAMERA_CAPTURE_STATUS', image_count, int32_t, []).
message_field('CAMERA_IMAGE_CAPTURED', time_boot_ms, uint32_t, [units=ms]).
message_field('CAMERA_IMAGE_CAPTURED', time_utc, uint64_t, [units=us, invalid='0']).
message_field('CAMERA_IMAGE_CAPTURED', camera_id, uint8_t, []).
message_field('CAMERA_IMAGE_CAPTURED', lat, int32_t, [units=degE7]).
message_field('CAMERA_IMAGE_CAPTURED', lon, int32_t, [units=degE7]).
message_field('CAMERA_IMAGE_CAPTURED', alt, int32_t, [units=mm]).
message_field('CAMERA_IMAGE_CAPTURED', relative_alt, int32_t, [units=mm]).
message_field('CAMERA_IMAGE_CAPTURED', q, 'float[4]', []).
message_field('CAMERA_IMAGE_CAPTURED', image_index, int32_t, []).
message_field('CAMERA_IMAGE_CAPTURED', capture_result, int8_t, []).
message_field('CAMERA_IMAGE_CAPTURED', file_url, 'char[205]', []).
message_field('FLIGHT_INFORMATION', time_boot_ms, uint32_t, [units=ms]).
message_field('FLIGHT_INFORMATION', arming_time_utc, uint64_t, [units=us, invalid='0']).
message_field('FLIGHT_INFORMATION', takeoff_time_utc, uint64_t, [units=us, invalid='0']).
message_field('FLIGHT_INFORMATION', flight_uuid, uint64_t, []).
message_field('MOUNT_ORIENTATION', time_boot_ms, uint32_t, [units=ms]).
message_field('MOUNT_ORIENTATION', roll, float, [units=deg, invalid='NaN']).
message_field('MOUNT_ORIENTATION', pitch, float, [units=deg, invalid='NaN']).
message_field('MOUNT_ORIENTATION', yaw, float, [units=deg, invalid='NaN']).
message_field('MOUNT_ORIENTATION', yaw_absolute, float, [units=deg, invalid='NaN']).
message_field('LOGGING_DATA', target_system, uint8_t, []).
message_field('LOGGING_DATA', target_component, uint8_t, []).
message_field('LOGGING_DATA', sequence, uint16_t, []).
message_field('LOGGING_DATA', length, uint8_t, [units=bytes]).
message_field('LOGGING_DATA', first_message_offset, uint8_t, [units=bytes, invalid='UINT8_MAX']).
message_field('LOGGING_DATA', data, 'uint8_t[249]', []).
message_field('LOGGING_DATA_ACKED', target_system, uint8_t, []).
message_field('LOGGING_DATA_ACKED', target_component, uint8_t, []).
message_field('LOGGING_DATA_ACKED', sequence, uint16_t, []).
message_field('LOGGING_DATA_ACKED', length, uint8_t, [units=bytes]).
message_field('LOGGING_DATA_ACKED', first_message_offset, uint8_t, [units=bytes, invalid='UINT8_MAX']).
message_field('LOGGING_DATA_ACKED', data, 'uint8_t[249]', []).
message_field('LOGGING_ACK', target_system, uint8_t, []).
message_field('LOGGING_ACK', target_component, uint8_t, []).
message_field('LOGGING_ACK', sequence, uint16_t, []).
message_field('VIDEO_STREAM_INFORMATION', stream_id, uint8_t, [instance=true]).
message_field('VIDEO_STREAM_INFORMATION', count, uint8_t, []).
message_field('VIDEO_STREAM_INFORMATION', type, uint8_t, [enum='VIDEO_STREAM_TYPE']).
message_field('VIDEO_STREAM_INFORMATION', flags, uint16_t, [enum='VIDEO_STREAM_STATUS_FLAGS']).
message_field('VIDEO_STREAM_INFORMATION', framerate, float, [units='Hz']).
message_field('VIDEO_STREAM_INFORMATION', resolution_h, uint16_t, [units=pix]).
message_field('VIDEO_STREAM_INFORMATION', resolution_v, uint16_t, [units=pix]).
message_field('VIDEO_STREAM_INFORMATION', bitrate, uint32_t, [units='bits/s']).
message_field('VIDEO_STREAM_INFORMATION', rotation, uint16_t, [units=deg]).
message_field('VIDEO_STREAM_INFORMATION', hfov, uint16_t, [units=deg]).
message_field('VIDEO_STREAM_INFORMATION', name, 'char[32]', []).
message_field('VIDEO_STREAM_INFORMATION', uri, 'char[160]', []).
message_field('VIDEO_STREAM_STATUS', stream_id, uint8_t, [instance=true]).
message_field('VIDEO_STREAM_STATUS', flags, uint16_t, [enum='VIDEO_STREAM_STATUS_FLAGS']).
message_field('VIDEO_STREAM_STATUS', framerate, float, [units='Hz']).
message_field('VIDEO_STREAM_STATUS', resolution_h, uint16_t, [units=pix]).
message_field('VIDEO_STREAM_STATUS', resolution_v, uint16_t, [units=pix]).
message_field('VIDEO_STREAM_STATUS', bitrate, uint32_t, [units='bits/s']).
message_field('VIDEO_STREAM_STATUS', rotation, uint16_t, [units=deg]).
message_field('VIDEO_STREAM_STATUS', hfov, uint16_t, [units=deg]).
message_field('CAMERA_FOV_STATUS', time_boot_ms, uint32_t, [units=ms]).
message_field('CAMERA_FOV_STATUS', lat_camera, int32_t, [units=degE7, invalid='INT32_MAX']).
message_field('CAMERA_FOV_STATUS', lon_camera, int32_t, [units=degE7, invalid='INT32_MAX']).
message_field('CAMERA_FOV_STATUS', alt_camera, int32_t, [units=mm, invalid='INT32_MAX']).
message_field('CAMERA_FOV_STATUS', lat_image, int32_t, [units=degE7, invalid='INT32_MAX']).
message_field('CAMERA_FOV_STATUS', lon_image, int32_t, [units=degE7, invalid='INT32_MAX']).
message_field('CAMERA_FOV_STATUS', alt_image, int32_t, [units=mm, invalid='INT32_MAX']).
message_field('CAMERA_FOV_STATUS', q, 'float[4]', []).
message_field('CAMERA_FOV_STATUS', hfov, float, [units=deg, invalid='NaN']).
message_field('CAMERA_FOV_STATUS', vfov, float, [units=deg, invalid='NaN']).
message_field('CAMERA_TRACKING_IMAGE_STATUS', tracking_status, uint8_t, [enum='CAMERA_TRACKING_STATUS_FLAGS']).
message_field('CAMERA_TRACKING_IMAGE_STATUS', tracking_mode, uint8_t, [enum='CAMERA_TRACKING_MODE']).
message_field('CAMERA_TRACKING_IMAGE_STATUS', target_data, uint8_t, [enum='CAMERA_TRACKING_TARGET_DATA']).
message_field('CAMERA_TRACKING_IMAGE_STATUS', point_x, float, [invalid='NaN']).
message_field('CAMERA_TRACKING_IMAGE_STATUS', point_y, float, [invalid='NaN']).
message_field('CAMERA_TRACKING_IMAGE_STATUS', radius, float, [invalid='NaN']).
message_field('CAMERA_TRACKING_IMAGE_STATUS', rec_top_x, float, [invalid='NaN']).
message_field('CAMERA_TRACKING_IMAGE_STATUS', rec_top_y, float, [invalid='NaN']).
message_field('CAMERA_TRACKING_IMAGE_STATUS', rec_bottom_x, float, [invalid='NaN']).
message_field('CAMERA_TRACKING_IMAGE_STATUS', rec_bottom_y, float, [invalid='NaN']).
message_field('CAMERA_TRACKING_GEO_STATUS', tracking_status, uint8_t, [enum='CAMERA_TRACKING_STATUS_FLAGS']).
message_field('CAMERA_TRACKING_GEO_STATUS', lat, int32_t, [units=degE7]).
message_field('CAMERA_TRACKING_GEO_STATUS', lon, int32_t, [units=degE7]).
message_field('CAMERA_TRACKING_GEO_STATUS', alt, float, [units=m]).
message_field('CAMERA_TRACKING_GEO_STATUS', h_acc, float, [units=m, invalid='NaN']).
message_field('CAMERA_TRACKING_GEO_STATUS', v_acc, float, [units=m, invalid='NaN']).
message_field('CAMERA_TRACKING_GEO_STATUS', vel_n, float, [units='m/s', invalid='NaN']).
message_field('CAMERA_TRACKING_GEO_STATUS', vel_e, float, [units='m/s', invalid='NaN']).
message_field('CAMERA_TRACKING_GEO_STATUS', vel_d, float, [units='m/s', invalid='NaN']).
message_field('CAMERA_TRACKING_GEO_STATUS', vel_acc, float, [units='m/s', invalid='NaN']).
message_field('CAMERA_TRACKING_GEO_STATUS', dist, float, [units=m, invalid='NaN']).
message_field('CAMERA_TRACKING_GEO_STATUS', hdg, float, [units=rad, invalid='NaN']).
message_field('CAMERA_TRACKING_GEO_STATUS', hdg_acc, float, [units=rad, invalid='NaN']).
message_field('GIMBAL_MANAGER_INFORMATION', time_boot_ms, uint32_t, [units=ms]).
message_field('GIMBAL_MANAGER_INFORMATION', cap_flags, uint32_t, [enum='GIMBAL_MANAGER_CAP_FLAGS', display=bitmask]).
message_field('GIMBAL_MANAGER_INFORMATION', gimbal_device_id, uint8_t, [instance=true]).
message_field('GIMBAL_MANAGER_INFORMATION', roll_min, float, [units=rad]).
message_field('GIMBAL_MANAGER_INFORMATION', roll_max, float, [units=rad]).
message_field('GIMBAL_MANAGER_INFORMATION', pitch_min, float, [units=rad]).
message_field('GIMBAL_MANAGER_INFORMATION', pitch_max, float, [units=rad]).
message_field('GIMBAL_MANAGER_INFORMATION', yaw_min, float, [units=rad]).
message_field('GIMBAL_MANAGER_INFORMATION', yaw_max, float, [units=rad]).
message_field('GIMBAL_MANAGER_STATUS', time_boot_ms, uint32_t, [units=ms]).
message_field('GIMBAL_MANAGER_STATUS', flags, uint32_t, [enum='GIMBAL_MANAGER_FLAGS', display=bitmask]).
message_field('GIMBAL_MANAGER_STATUS', gimbal_device_id, uint8_t, [instance=true]).
message_field('GIMBAL_MANAGER_STATUS', primary_control_sysid, uint8_t, []).
message_field('GIMBAL_MANAGER_STATUS', primary_control_compid, uint8_t, []).
message_field('GIMBAL_MANAGER_STATUS', secondary_control_sysid, uint8_t, []).
message_field('GIMBAL_MANAGER_STATUS', secondary_control_compid, uint8_t, []).
message_field('GIMBAL_MANAGER_SET_ATTITUDE', target_system, uint8_t, []).
message_field('GIMBAL_MANAGER_SET_ATTITUDE', target_component, uint8_t, []).
message_field('GIMBAL_MANAGER_SET_ATTITUDE', flags, uint32_t, [enum='GIMBAL_MANAGER_FLAGS']).
message_field('GIMBAL_MANAGER_SET_ATTITUDE', gimbal_device_id, uint8_t, [instance=true]).
message_field('GIMBAL_MANAGER_SET_ATTITUDE', q, 'float[4]', []).
message_field('GIMBAL_MANAGER_SET_ATTITUDE', angular_velocity_x, float, [units='rad/s', invalid='NaN']).
message_field('GIMBAL_MANAGER_SET_ATTITUDE', angular_velocity_y, float, [units='rad/s', invalid='NaN']).
message_field('GIMBAL_MANAGER_SET_ATTITUDE', angular_velocity_z, float, [units='rad/s', invalid='NaN']).
message_field('GIMBAL_DEVICE_INFORMATION', time_boot_ms, uint32_t, [units=ms]).
message_field('GIMBAL_DEVICE_INFORMATION', vendor_name, 'char[32]', []).
message_field('GIMBAL_DEVICE_INFORMATION', model_name, 'char[32]', []).
message_field('GIMBAL_DEVICE_INFORMATION', custom_name, 'char[32]', []).
message_field('GIMBAL_DEVICE_INFORMATION', firmware_version, uint32_t, []).
message_field('GIMBAL_DEVICE_INFORMATION', hardware_version, uint32_t, []).
message_field('GIMBAL_DEVICE_INFORMATION', uid, uint64_t, [invalid='0']).
message_field('GIMBAL_DEVICE_INFORMATION', cap_flags, uint16_t, [enum='GIMBAL_DEVICE_CAP_FLAGS', display=bitmask]).
message_field('GIMBAL_DEVICE_INFORMATION', custom_cap_flags, uint16_t, [display=bitmask]).
message_field('GIMBAL_DEVICE_INFORMATION', roll_min, float, [units=rad, invalid='NaN']).
message_field('GIMBAL_DEVICE_INFORMATION', roll_max, float, [units=rad, invalid='NaN']).
message_field('GIMBAL_DEVICE_INFORMATION', pitch_min, float, [units=rad, invalid='NaN']).
message_field('GIMBAL_DEVICE_INFORMATION', pitch_max, float, [units=rad, invalid='NaN']).
message_field('GIMBAL_DEVICE_INFORMATION', yaw_min, float, [units=rad, invalid='NaN']).
message_field('GIMBAL_DEVICE_INFORMATION', yaw_max, float, [units=rad, invalid='NaN']).
message_field('GIMBAL_DEVICE_INFORMATION', gimbal_device_id, uint8_t, [invalid='0']).
message_field('GIMBAL_DEVICE_SET_ATTITUDE', target_system, uint8_t, []).
message_field('GIMBAL_DEVICE_SET_ATTITUDE', target_component, uint8_t, []).
message_field('GIMBAL_DEVICE_SET_ATTITUDE', flags, uint16_t, [enum='GIMBAL_DEVICE_FLAGS', display=bitmask]).
message_field('GIMBAL_DEVICE_SET_ATTITUDE', q, 'float[4]', [invalid='[NaN]']).
message_field('GIMBAL_DEVICE_SET_ATTITUDE', angular_velocity_x, float, [units='rad/s', invalid='NaN']).
message_field('GIMBAL_DEVICE_SET_ATTITUDE', angular_velocity_y, float, [units='rad/s', invalid='NaN']).
message_field('GIMBAL_DEVICE_SET_ATTITUDE', angular_velocity_z, float, [units='rad/s', invalid='NaN']).
message_field('GIMBAL_DEVICE_ATTITUDE_STATUS', target_system, uint8_t, []).
message_field('GIMBAL_DEVICE_ATTITUDE_STATUS', target_component, uint8_t, []).
message_field('GIMBAL_DEVICE_ATTITUDE_STATUS', time_boot_ms, uint32_t, [units=ms]).
message_field('GIMBAL_DEVICE_ATTITUDE_STATUS', flags, uint16_t, [enum='GIMBAL_DEVICE_FLAGS', display=bitmask]).
message_field('GIMBAL_DEVICE_ATTITUDE_STATUS', q, 'float[4]', []).
message_field('GIMBAL_DEVICE_ATTITUDE_STATUS', angular_velocity_x, float, [units='rad/s', invalid='NaN']).
message_field('GIMBAL_DEVICE_ATTITUDE_STATUS', angular_velocity_y, float, [units='rad/s', invalid='NaN']).
message_field('GIMBAL_DEVICE_ATTITUDE_STATUS', angular_velocity_z, float, [units='rad/s', invalid='NaN']).
message_field('GIMBAL_DEVICE_ATTITUDE_STATUS', failure_flags, uint32_t, [enum='GIMBAL_DEVICE_ERROR_FLAGS', display=bitmask]).
message_field('GIMBAL_DEVICE_ATTITUDE_STATUS', delta_yaw, float, [units=rad, invalid='NAN']).
message_field('GIMBAL_DEVICE_ATTITUDE_STATUS', delta_yaw_velocity, float, [units='rad/s', invalid='NAN']).
message_field('GIMBAL_DEVICE_ATTITUDE_STATUS', gimbal_device_id, uint8_t, [invalid='0']).
message_field('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', target_system, uint8_t, []).
message_field('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', target_component, uint8_t, []).
message_field('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', time_boot_us, uint64_t, [units=us]).
message_field('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', q, 'float[4]', []).
message_field('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', q_estimated_delay_us, uint32_t, [units=us, invalid='0']).
message_field('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', vx, float, [units='m/s', invalid='NaN']).
message_field('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', vy, float, [units='m/s', invalid='NaN']).
message_field('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', vz, float, [units='m/s', invalid='NaN']).
message_field('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', v_estimated_delay_us, uint32_t, [units=us, invalid='0']).
message_field('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', feed_forward_angular_velocity_z, float, [units='rad/s', invalid='NaN']).
message_field('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', estimator_status, uint16_t, [enum='ESTIMATOR_STATUS_FLAGS', display=bitmask]).
message_field('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', landed_state, uint8_t, [enum='MAV_LANDED_STATE', invalid='MAV_LANDED_STATE_UNDEFINED']).
message_field('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', angular_velocity_z, float, [units='rad/s', invalid='NaN']).
message_field('GIMBAL_MANAGER_SET_PITCHYAW', target_system, uint8_t, []).
message_field('GIMBAL_MANAGER_SET_PITCHYAW', target_component, uint8_t, []).
message_field('GIMBAL_MANAGER_SET_PITCHYAW', flags, uint32_t, [enum='GIMBAL_MANAGER_FLAGS']).
message_field('GIMBAL_MANAGER_SET_PITCHYAW', gimbal_device_id, uint8_t, [instance=true]).
message_field('GIMBAL_MANAGER_SET_PITCHYAW', pitch, float, [units=rad, invalid='NaN']).
message_field('GIMBAL_MANAGER_SET_PITCHYAW', yaw, float, [units=rad, invalid='NaN']).
message_field('GIMBAL_MANAGER_SET_PITCHYAW', pitch_rate, float, [units='rad/s', invalid='NaN']).
message_field('GIMBAL_MANAGER_SET_PITCHYAW', yaw_rate, float, [units='rad/s', invalid='NaN']).
message_field('GIMBAL_MANAGER_SET_MANUAL_CONTROL', target_system, uint8_t, []).
message_field('GIMBAL_MANAGER_SET_MANUAL_CONTROL', target_component, uint8_t, []).
message_field('GIMBAL_MANAGER_SET_MANUAL_CONTROL', flags, uint32_t, [enum='GIMBAL_MANAGER_FLAGS']).
message_field('GIMBAL_MANAGER_SET_MANUAL_CONTROL', gimbal_device_id, uint8_t, [instance=true]).
message_field('GIMBAL_MANAGER_SET_MANUAL_CONTROL', pitch, float, [invalid='NaN']).
message_field('GIMBAL_MANAGER_SET_MANUAL_CONTROL', yaw, float, [invalid='NaN']).
message_field('GIMBAL_MANAGER_SET_MANUAL_CONTROL', pitch_rate, float, [invalid='NaN']).
message_field('GIMBAL_MANAGER_SET_MANUAL_CONTROL', yaw_rate, float, [invalid='NaN']).
message_field('ESC_INFO', index, uint8_t, [instance=true]).
message_field('ESC_INFO', time_usec, uint64_t, [units=us]).
message_field('ESC_INFO', counter, uint16_t, []).
message_field('ESC_INFO', count, uint8_t, []).
message_field('ESC_INFO', connection_type, uint8_t, [enum='ESC_CONNECTION_TYPE']).
message_field('ESC_INFO', info, uint8_t, [display=bitmask]).
message_field('ESC_INFO', failure_flags, 'uint16_t[4]', [enum='ESC_FAILURE_FLAGS', display=bitmask]).
message_field('ESC_INFO', error_count, 'uint32_t[4]', []).
message_field('ESC_INFO', temperature, 'int16_t[4]', [units=cdegC, invalid='[INT16_MAX]']).
message_field('ESC_STATUS', index, uint8_t, [instance=true]).
message_field('ESC_STATUS', time_usec, uint64_t, [units=us]).
message_field('ESC_STATUS', rpm, 'int32_t[4]', [units=rpm]).
message_field('ESC_STATUS', voltage, 'float[4]', [units='V']).
message_field('ESC_STATUS', current, 'float[4]', [units='A']).
message_field('WIFI_CONFIG_AP', ssid, 'char[32]', []).
message_field('WIFI_CONFIG_AP', password, 'char[64]', []).
message_field('WIFI_CONFIG_AP', mode, int8_t, [enum='WIFI_CONFIG_AP_MODE']).
message_field('WIFI_CONFIG_AP', response, int8_t, [enum='WIFI_CONFIG_AP_RESPONSE']).
message_field('AIS_VESSEL', 'MMSI', uint32_t, []).
message_field('AIS_VESSEL', lat, int32_t, [units=degE7]).
message_field('AIS_VESSEL', lon, int32_t, [units=degE7]).
message_field('AIS_VESSEL', 'COG', uint16_t, [units=cdeg]).
message_field('AIS_VESSEL', heading, uint16_t, [units=cdeg]).
message_field('AIS_VESSEL', velocity, uint16_t, [units='cm/s']).
message_field('AIS_VESSEL', turn_rate, int8_t, [units='cdeg/s']).
message_field('AIS_VESSEL', navigational_status, uint8_t, [enum='AIS_NAV_STATUS']).
message_field('AIS_VESSEL', type, uint8_t, [enum='AIS_TYPE']).
message_field('AIS_VESSEL', dimension_bow, uint16_t, [units=m]).
message_field('AIS_VESSEL', dimension_stern, uint16_t, [units=m]).
message_field('AIS_VESSEL', dimension_port, uint8_t, [units=m]).
message_field('AIS_VESSEL', dimension_starboard, uint8_t, [units=m]).
message_field('AIS_VESSEL', callsign, 'char[7]', []).
message_field('AIS_VESSEL', name, 'char[20]', []).
message_field('AIS_VESSEL', tslc, uint16_t, [units=s]).
message_field('AIS_VESSEL', flags, uint16_t, [enum='AIS_FLAGS', display=bitmask]).
message_field('UAVCAN_NODE_STATUS', time_usec, uint64_t, [units=us]).
message_field('UAVCAN_NODE_STATUS', uptime_sec, uint32_t, [units=s]).
message_field('UAVCAN_NODE_STATUS', health, uint8_t, [enum='UAVCAN_NODE_HEALTH']).
message_field('UAVCAN_NODE_STATUS', mode, uint8_t, [enum='UAVCAN_NODE_MODE']).
message_field('UAVCAN_NODE_STATUS', sub_mode, uint8_t, []).
message_field('UAVCAN_NODE_STATUS', vendor_specific_status_code, uint16_t, []).
message_field('UAVCAN_NODE_INFO', time_usec, uint64_t, [units=us]).
message_field('UAVCAN_NODE_INFO', uptime_sec, uint32_t, [units=s]).
message_field('UAVCAN_NODE_INFO', name, 'char[80]', []).
message_field('UAVCAN_NODE_INFO', hw_version_major, uint8_t, []).
message_field('UAVCAN_NODE_INFO', hw_version_minor, uint8_t, []).
message_field('UAVCAN_NODE_INFO', hw_unique_id, 'uint8_t[16]', []).
message_field('UAVCAN_NODE_INFO', sw_version_major, uint8_t, []).
message_field('UAVCAN_NODE_INFO', sw_version_minor, uint8_t, []).
message_field('UAVCAN_NODE_INFO', sw_vcs_commit, uint32_t, [invalid='0']).
message_field('PARAM_EXT_REQUEST_READ', target_system, uint8_t, []).
message_field('PARAM_EXT_REQUEST_READ', target_component, uint8_t, []).
message_field('PARAM_EXT_REQUEST_READ', param_id, 'char[16]', []).
message_field('PARAM_EXT_REQUEST_READ', param_index, int16_t, [invalid='-1']).
message_field('PARAM_EXT_REQUEST_LIST', target_system, uint8_t, []).
message_field('PARAM_EXT_REQUEST_LIST', target_component, uint8_t, []).
message_field('PARAM_EXT_VALUE', param_id, 'char[16]', []).
message_field('PARAM_EXT_VALUE', param_value, 'char[128]', []).
message_field('PARAM_EXT_VALUE', param_type, uint8_t, [enum='MAV_PARAM_EXT_TYPE']).
message_field('PARAM_EXT_VALUE', param_count, uint16_t, []).
message_field('PARAM_EXT_VALUE', param_index, uint16_t, []).
message_field('PARAM_EXT_SET', target_system, uint8_t, []).
message_field('PARAM_EXT_SET', target_component, uint8_t, []).
message_field('PARAM_EXT_SET', param_id, 'char[16]', []).
message_field('PARAM_EXT_SET', param_value, 'char[128]', []).
message_field('PARAM_EXT_SET', param_type, uint8_t, [enum='MAV_PARAM_EXT_TYPE']).
message_field('PARAM_EXT_ACK', param_id, 'char[16]', []).
message_field('PARAM_EXT_ACK', param_value, 'char[128]', []).
message_field('PARAM_EXT_ACK', param_type, uint8_t, [enum='MAV_PARAM_EXT_TYPE']).
message_field('PARAM_EXT_ACK', param_result, uint8_t, [enum='PARAM_ACK']).
message_field('OBSTACLE_DISTANCE', time_usec, uint64_t, [units=us]).
message_field('OBSTACLE_DISTANCE', sensor_type, uint8_t, [enum='MAV_DISTANCE_SENSOR']).
message_field('OBSTACLE_DISTANCE', distances, 'uint16_t[72]', [units=cm, invalid='[UINT16_MAX]']).
message_field('OBSTACLE_DISTANCE', increment, uint8_t, [units=deg]).
message_field('OBSTACLE_DISTANCE', min_distance, uint16_t, [units=cm]).
message_field('OBSTACLE_DISTANCE', max_distance, uint16_t, [units=cm]).
message_field('OBSTACLE_DISTANCE', increment_f, float, [units=deg]).
message_field('OBSTACLE_DISTANCE', angle_offset, float, [units=deg]).
message_field('OBSTACLE_DISTANCE', frame, uint8_t, [enum='MAV_FRAME']).
message_field('ODOMETRY', time_usec, uint64_t, [units=us]).
message_field('ODOMETRY', frame_id, uint8_t, [enum='MAV_FRAME']).
message_field('ODOMETRY', child_frame_id, uint8_t, [enum='MAV_FRAME']).
message_field('ODOMETRY', x, float, [units=m]).
message_field('ODOMETRY', y, float, [units=m]).
message_field('ODOMETRY', z, float, [units=m]).
message_field('ODOMETRY', q, 'float[4]', []).
message_field('ODOMETRY', vx, float, [units='m/s']).
message_field('ODOMETRY', vy, float, [units='m/s']).
message_field('ODOMETRY', vz, float, [units='m/s']).
message_field('ODOMETRY', rollspeed, float, [units='rad/s']).
message_field('ODOMETRY', pitchspeed, float, [units='rad/s']).
message_field('ODOMETRY', yawspeed, float, [units='rad/s']).
message_field('ODOMETRY', pose_covariance, 'float[21]', [invalid='[NaN:]']).
message_field('ODOMETRY', velocity_covariance, 'float[21]', [invalid='[NaN:]']).
message_field('ODOMETRY', reset_counter, uint8_t, []).
message_field('ODOMETRY', estimator_type, uint8_t, [enum='MAV_ESTIMATOR_TYPE']).
message_field('ODOMETRY', quality, int8_t, [units='%', invalid='0']).
message_field('TRAJECTORY_REPRESENTATION_WAYPOINTS', time_usec, uint64_t, [units=us]).
message_field('TRAJECTORY_REPRESENTATION_WAYPOINTS', valid_points, uint8_t, []).
message_field('TRAJECTORY_REPRESENTATION_WAYPOINTS', pos_x, 'float[5]', [units=m, invalid='[NaN]']).
message_field('TRAJECTORY_REPRESENTATION_WAYPOINTS', pos_y, 'float[5]', [units=m, invalid='[NaN]']).
message_field('TRAJECTORY_REPRESENTATION_WAYPOINTS', pos_z, 'float[5]', [units=m, invalid='[NaN]']).
message_field('TRAJECTORY_REPRESENTATION_WAYPOINTS', vel_x, 'float[5]', [units='m/s', invalid='[NaN]']).
message_field('TRAJECTORY_REPRESENTATION_WAYPOINTS', vel_y, 'float[5]', [units='m/s', invalid='[NaN]']).
message_field('TRAJECTORY_REPRESENTATION_WAYPOINTS', vel_z, 'float[5]', [units='m/s', invalid='[NaN]']).
message_field('TRAJECTORY_REPRESENTATION_WAYPOINTS', acc_x, 'float[5]', [units='m/s/s', invalid='[NaN]']).
message_field('TRAJECTORY_REPRESENTATION_WAYPOINTS', acc_y, 'float[5]', [units='m/s/s', invalid='[NaN]']).
message_field('TRAJECTORY_REPRESENTATION_WAYPOINTS', acc_z, 'float[5]', [units='m/s/s', invalid='[NaN]']).
message_field('TRAJECTORY_REPRESENTATION_WAYPOINTS', pos_yaw, 'float[5]', [units=rad, invalid='[NaN]']).
message_field('TRAJECTORY_REPRESENTATION_WAYPOINTS', vel_yaw, 'float[5]', [units='rad/s', invalid='[NaN]']).
message_field('TRAJECTORY_REPRESENTATION_WAYPOINTS', command, 'uint16_t[5]', [enum='MAV_CMD', invalid='[UINT16_MAX]']).
message_field('TRAJECTORY_REPRESENTATION_BEZIER', time_usec, uint64_t, [units=us]).
message_field('TRAJECTORY_REPRESENTATION_BEZIER', valid_points, uint8_t, []).
message_field('TRAJECTORY_REPRESENTATION_BEZIER', pos_x, 'float[5]', [units=m, invalid='[NaN]']).
message_field('TRAJECTORY_REPRESENTATION_BEZIER', pos_y, 'float[5]', [units=m, invalid='[NaN]']).
message_field('TRAJECTORY_REPRESENTATION_BEZIER', pos_z, 'float[5]', [units=m, invalid='[NaN]']).
message_field('TRAJECTORY_REPRESENTATION_BEZIER', delta, 'float[5]', [units=s, invalid='[NaN]']).
message_field('TRAJECTORY_REPRESENTATION_BEZIER', pos_yaw, 'float[5]', [units=rad, invalid='[NaN]']).
message_field('CELLULAR_STATUS', status, uint8_t, [enum='CELLULAR_STATUS_FLAG']).
message_field('CELLULAR_STATUS', failure_reason, uint8_t, [enum='CELLULAR_NETWORK_FAILED_REASON']).
message_field('CELLULAR_STATUS', type, uint8_t, [enum='CELLULAR_NETWORK_RADIO_TYPE']).
message_field('CELLULAR_STATUS', quality, uint8_t, [invalid='UINT8_MAX']).
message_field('CELLULAR_STATUS', mcc, uint16_t, [invalid='UINT16_MAX']).
message_field('CELLULAR_STATUS', mnc, uint16_t, [invalid='UINT16_MAX']).
message_field('CELLULAR_STATUS', lac, uint16_t, [invalid='0']).
message_field('ISBD_LINK_STATUS', timestamp, uint64_t, [units=us]).
message_field('ISBD_LINK_STATUS', last_heartbeat, uint64_t, [units=us]).
message_field('ISBD_LINK_STATUS', failed_sessions, uint16_t, []).
message_field('ISBD_LINK_STATUS', successful_sessions, uint16_t, []).
message_field('ISBD_LINK_STATUS', signal_quality, uint8_t, []).
message_field('ISBD_LINK_STATUS', ring_pending, uint8_t, []).
message_field('ISBD_LINK_STATUS', tx_session_pending, uint8_t, []).
message_field('ISBD_LINK_STATUS', rx_session_pending, uint8_t, []).
message_field('CELLULAR_CONFIG', enable_lte, uint8_t, []).
message_field('CELLULAR_CONFIG', enable_pin, uint8_t, []).
message_field('CELLULAR_CONFIG', pin, 'char[16]', []).
message_field('CELLULAR_CONFIG', new_pin, 'char[16]', []).
message_field('CELLULAR_CONFIG', apn, 'char[32]', []).
message_field('CELLULAR_CONFIG', puk, 'char[16]', []).
message_field('CELLULAR_CONFIG', roaming, uint8_t, []).
message_field('CELLULAR_CONFIG', response, uint8_t, [enum='CELLULAR_CONFIG_RESPONSE']).
message_field('RAW_RPM', index, uint8_t, []).
message_field('RAW_RPM', frequency, float, [units=rpm]).
message_field('UTM_GLOBAL_POSITION', time, uint64_t, [units=us]).
message_field('UTM_GLOBAL_POSITION', uas_id, 'uint8_t[18]', []).
message_field('UTM_GLOBAL_POSITION', lat, int32_t, [units=degE7]).
message_field('UTM_GLOBAL_POSITION', lon, int32_t, [units=degE7]).
message_field('UTM_GLOBAL_POSITION', alt, int32_t, [units=mm]).
message_field('UTM_GLOBAL_POSITION', relative_alt, int32_t, [units=mm]).
message_field('UTM_GLOBAL_POSITION', vx, int16_t, [units='cm/s']).
message_field('UTM_GLOBAL_POSITION', vy, int16_t, [units='cm/s']).
message_field('UTM_GLOBAL_POSITION', vz, int16_t, [units='cm/s']).
message_field('UTM_GLOBAL_POSITION', h_acc, uint16_t, [units=mm]).
message_field('UTM_GLOBAL_POSITION', v_acc, uint16_t, [units=mm]).
message_field('UTM_GLOBAL_POSITION', vel_acc, uint16_t, [units='cm/s']).
message_field('UTM_GLOBAL_POSITION', next_lat, int32_t, [units=degE7]).
message_field('UTM_GLOBAL_POSITION', next_lon, int32_t, [units=degE7]).
message_field('UTM_GLOBAL_POSITION', next_alt, int32_t, [units=mm]).
message_field('UTM_GLOBAL_POSITION', update_rate, uint16_t, [units=cs, invalid='0']).
message_field('UTM_GLOBAL_POSITION', flight_state, uint8_t, [enum='UTM_FLIGHT_STATE']).
message_field('UTM_GLOBAL_POSITION', flags, uint8_t, [enum='UTM_DATA_AVAIL_FLAGS', display=bitmask]).
message_field('DEBUG_FLOAT_ARRAY', time_usec, uint64_t, [units=us]).
message_field('DEBUG_FLOAT_ARRAY', name, 'char[10]', []).
message_field('DEBUG_FLOAT_ARRAY', array_id, uint16_t, [instance=true]).
message_field('DEBUG_FLOAT_ARRAY', data, 'float[58]', []).
message_field('ORBIT_EXECUTION_STATUS', time_usec, uint64_t, [units=us]).
message_field('ORBIT_EXECUTION_STATUS', radius, float, [units=m]).
message_field('ORBIT_EXECUTION_STATUS', frame, uint8_t, [enum='MAV_FRAME']).
message_field('ORBIT_EXECUTION_STATUS', x, int32_t, []).
message_field('ORBIT_EXECUTION_STATUS', y, int32_t, []).
message_field('ORBIT_EXECUTION_STATUS', z, float, [units=m]).
message_field('SMART_BATTERY_INFO', id, uint8_t, [instance=true]).
message_field('SMART_BATTERY_INFO', battery_function, uint8_t, [enum='MAV_BATTERY_FUNCTION']).
message_field('SMART_BATTERY_INFO', type, uint8_t, [enum='MAV_BATTERY_TYPE']).
message_field('SMART_BATTERY_INFO', capacity_full_specification, int32_t, [units=mAh, invalid='-1']).
message_field('SMART_BATTERY_INFO', capacity_full, int32_t, [units=mAh, invalid='-1']).
message_field('SMART_BATTERY_INFO', cycle_count, uint16_t, [invalid='UINT16_MAX']).
message_field('SMART_BATTERY_INFO', serial_number, 'char[16]', [invalid='[0]']).
message_field('SMART_BATTERY_INFO', device_name, 'char[50]', [invalid='[0]']).
message_field('SMART_BATTERY_INFO', weight, uint16_t, [units=g, invalid='0']).
message_field('SMART_BATTERY_INFO', discharge_minimum_voltage, uint16_t, [units=mV, invalid='UINT16_MAX']).
message_field('SMART_BATTERY_INFO', charging_minimum_voltage, uint16_t, [units=mV, invalid='UINT16_MAX']).
message_field('SMART_BATTERY_INFO', resting_minimum_voltage, uint16_t, [units=mV, invalid='UINT16_MAX']).
message_field('SMART_BATTERY_INFO', charging_maximum_voltage, uint16_t, [units=mV, invalid='0']).
message_field('SMART_BATTERY_INFO', cells_in_series, uint8_t, [invalid='0']).
message_field('SMART_BATTERY_INFO', discharge_maximum_current, uint32_t, [units=mA, invalid='0']).
message_field('SMART_BATTERY_INFO', discharge_maximum_burst_current, uint32_t, [units=mA, invalid='0']).
message_field('SMART_BATTERY_INFO', manufacture_date, 'char[11]', [invalid='[0]']).
message_field('GENERATOR_STATUS', status, uint64_t, [display=bitmask, enum='MAV_GENERATOR_STATUS_FLAG']).
message_field('GENERATOR_STATUS', generator_speed, uint16_t, [units=rpm, invalid='UINT16_MAX']).
message_field('GENERATOR_STATUS', battery_current, float, [units='A', invalid='NaN']).
message_field('GENERATOR_STATUS', load_current, float, [units='A', invalid='NaN']).
message_field('GENERATOR_STATUS', power_generated, float, [units='W', invalid='NaN']).
message_field('GENERATOR_STATUS', bus_voltage, float, [units='V']).
message_field('GENERATOR_STATUS', rectifier_temperature, int16_t, [units=degC, invalid='INT16_MAX']).
message_field('GENERATOR_STATUS', bat_current_setpoint, float, [units='A', invalid='NaN']).
message_field('GENERATOR_STATUS', generator_temperature, int16_t, [units=degC, invalid='INT16_MAX']).
message_field('GENERATOR_STATUS', runtime, uint32_t, [units=s, invalid='UINT32_MAX']).
message_field('GENERATOR_STATUS', time_until_maintenance, int32_t, [units=s, invalid='INT32_MAX']).
message_field('ACTUATOR_OUTPUT_STATUS', time_usec, uint64_t, [units=us]).
message_field('ACTUATOR_OUTPUT_STATUS', active, uint32_t, [display=bitmask]).
message_field('ACTUATOR_OUTPUT_STATUS', actuator, 'float[32]', []).
message_field('TIME_ESTIMATE_TO_TARGET', safe_return, int32_t, [units=s]).
message_field('TIME_ESTIMATE_TO_TARGET', land, int32_t, [units=s]).
message_field('TIME_ESTIMATE_TO_TARGET', mission_next_item, int32_t, [units=s, invalid='-1']).
message_field('TIME_ESTIMATE_TO_TARGET', mission_end, int32_t, [units=s, invalid='-1']).
message_field('TIME_ESTIMATE_TO_TARGET', commanded_action, int32_t, [units=s, invalid='-1']).
message_field('TUNNEL', target_system, uint8_t, []).
message_field('TUNNEL', target_component, uint8_t, []).
message_field('TUNNEL', payload_type, uint16_t, [enum='MAV_TUNNEL_PAYLOAD_TYPE']).
message_field('TUNNEL', payload_length, uint8_t, []).
message_field('TUNNEL', payload, 'uint8_t[128]', []).
message_field('CAN_FRAME', target_system, uint8_t, []).
message_field('CAN_FRAME', target_component, uint8_t, []).
message_field('CAN_FRAME', bus, uint8_t, []).
message_field('CAN_FRAME', len, uint8_t, []).
message_field('CAN_FRAME', id, uint32_t, []).
message_field('CAN_FRAME', data, 'uint8_t[8]', []).
message_field('ONBOARD_COMPUTER_STATUS', time_usec, uint64_t, [units=us]).
message_field('ONBOARD_COMPUTER_STATUS', uptime, uint32_t, [units=ms]).
message_field('ONBOARD_COMPUTER_STATUS', type, uint8_t, []).
message_field('ONBOARD_COMPUTER_STATUS', cpu_cores, 'uint8_t[8]', [invalid='[UINT8_MAX]']).
message_field('ONBOARD_COMPUTER_STATUS', cpu_combined, 'uint8_t[10]', [invalid='[UINT8_MAX]']).
message_field('ONBOARD_COMPUTER_STATUS', gpu_cores, 'uint8_t[4]', [invalid='[UINT8_MAX]']).
message_field('ONBOARD_COMPUTER_STATUS', gpu_combined, 'uint8_t[10]', [invalid='[UINT8_MAX]']).
message_field('ONBOARD_COMPUTER_STATUS', temperature_board, int8_t, [units=degC, invalid='INT8_MAX']).
message_field('ONBOARD_COMPUTER_STATUS', temperature_core, 'int8_t[8]', [units=degC, invalid='[INT8_MAX]']).
message_field('ONBOARD_COMPUTER_STATUS', fan_speed, 'int16_t[4]', [units=rpm, invalid='[INT16_MAX]']).
message_field('ONBOARD_COMPUTER_STATUS', ram_usage, uint32_t, [units='MiB', invalid='UINT32_MAX']).
message_field('ONBOARD_COMPUTER_STATUS', ram_total, uint32_t, [units='MiB', invalid='UINT32_MAX']).
message_field('ONBOARD_COMPUTER_STATUS', storage_type, 'uint32_t[4]', [invalid='[UINT32_MAX]']).
message_field('ONBOARD_COMPUTER_STATUS', storage_usage, 'uint32_t[4]', [units='MiB', invalid='[UINT32_MAX]']).
message_field('ONBOARD_COMPUTER_STATUS', storage_total, 'uint32_t[4]', [units='MiB', invalid='[UINT32_MAX]']).
message_field('ONBOARD_COMPUTER_STATUS', link_type, 'uint32_t[6]', []).
message_field('ONBOARD_COMPUTER_STATUS', link_tx_rate, 'uint32_t[6]', [units='KiB/s', invalid='[UINT32_MAX]']).
message_field('ONBOARD_COMPUTER_STATUS', link_rx_rate, 'uint32_t[6]', [units='KiB/s', invalid='[UINT32_MAX]']).
message_field('ONBOARD_COMPUTER_STATUS', link_tx_max, 'uint32_t[6]', [units='KiB/s', invalid='[UINT32_MAX]']).
message_field('ONBOARD_COMPUTER_STATUS', link_rx_max, 'uint32_t[6]', [units='KiB/s', invalid='[UINT32_MAX]']).
message_field('COMPONENT_INFORMATION', time_boot_ms, uint32_t, [units=ms]).
message_field('COMPONENT_INFORMATION', general_metadata_file_crc, uint32_t, []).
message_field('COMPONENT_INFORMATION', general_metadata_uri, 'char[100]', []).
message_field('COMPONENT_INFORMATION', peripherals_metadata_file_crc, uint32_t, []).
message_field('COMPONENT_INFORMATION', peripherals_metadata_uri, 'char[100]', []).
message_field('COMPONENT_METADATA', time_boot_ms, uint32_t, [units=ms]).
message_field('COMPONENT_METADATA', file_crc, uint32_t, []).
message_field('COMPONENT_METADATA', uri, 'char[100]', []).
message_field('PLAY_TUNE_V2', target_system, uint8_t, []).
message_field('PLAY_TUNE_V2', target_component, uint8_t, []).
message_field('PLAY_TUNE_V2', format, uint32_t, [enum='TUNE_FORMAT', display=bitmask]).
message_field('PLAY_TUNE_V2', tune, 'char[248]', []).
message_field('SUPPORTED_TUNES', target_system, uint8_t, []).
message_field('SUPPORTED_TUNES', target_component, uint8_t, []).
message_field('SUPPORTED_TUNES', format, uint32_t, [enum='TUNE_FORMAT', display=bitmask]).
message_field('EVENT', destination_component, uint8_t, []).
message_field('EVENT', destination_system, uint8_t, []).
message_field('EVENT', id, uint32_t, []).
message_field('EVENT', event_time_boot_ms, uint32_t, [units=ms]).
message_field('EVENT', sequence, uint16_t, []).
message_field('EVENT', log_levels, uint8_t, []).
message_field('EVENT', arguments, 'uint8_t[40]', []).
message_field('CURRENT_EVENT_SEQUENCE', sequence, uint16_t, []).
message_field('CURRENT_EVENT_SEQUENCE', flags, uint8_t, [enum='MAV_EVENT_CURRENT_SEQUENCE_FLAGS', display=bitmask]).
message_field('REQUEST_EVENT', target_system, uint8_t, []).
message_field('REQUEST_EVENT', target_component, uint8_t, []).
message_field('REQUEST_EVENT', first_sequence, uint16_t, []).
message_field('REQUEST_EVENT', last_sequence, uint16_t, []).
message_field('RESPONSE_EVENT_ERROR', target_system, uint8_t, []).
message_field('RESPONSE_EVENT_ERROR', target_component, uint8_t, []).
message_field('RESPONSE_EVENT_ERROR', sequence, uint16_t, []).
message_field('RESPONSE_EVENT_ERROR', sequence_oldest_available, uint16_t, []).
message_field('RESPONSE_EVENT_ERROR', reason, uint8_t, [enum='MAV_EVENT_ERROR_REASON']).
message_field('CANFD_FRAME', target_system, uint8_t, []).
message_field('CANFD_FRAME', target_component, uint8_t, []).
message_field('CANFD_FRAME', bus, uint8_t, []).
message_field('CANFD_FRAME', len, uint8_t, []).
message_field('CANFD_FRAME', id, uint32_t, []).
message_field('CANFD_FRAME', data, 'uint8_t[64]', []).
message_field('CAN_FILTER_MODIFY', target_system, uint8_t, []).
message_field('CAN_FILTER_MODIFY', target_component, uint8_t, []).
message_field('CAN_FILTER_MODIFY', bus, uint8_t, []).
message_field('CAN_FILTER_MODIFY', operation, uint8_t, [enum='CAN_FILTER_OP']).
message_field('CAN_FILTER_MODIFY', num_ids, uint8_t, []).
message_field('CAN_FILTER_MODIFY', ids, 'uint16_t[16]', []).
message_field('WHEEL_DISTANCE', time_usec, uint64_t, [units=us]).
message_field('WHEEL_DISTANCE', count, uint8_t, []).
message_field('WHEEL_DISTANCE', distance, 'double[16]', [units=m]).
message_field('WINCH_STATUS', time_usec, uint64_t, [units=us]).
message_field('WINCH_STATUS', line_length, float, [units=m, invalid='NaN']).
message_field('WINCH_STATUS', speed, float, [units='m/s', invalid='NaN']).
message_field('WINCH_STATUS', tension, float, [units=kg, invalid='NaN']).
message_field('WINCH_STATUS', voltage, float, [units='V', invalid='NaN']).
message_field('WINCH_STATUS', current, float, [units='A', invalid='NaN']).
message_field('WINCH_STATUS', temperature, int16_t, [units=degC, invalid='INT16_MAX']).
message_field('WINCH_STATUS', status, uint32_t, [display=bitmask, enum='MAV_WINCH_STATUS_FLAG']).
message_field('OPEN_DRONE_ID_BASIC_ID', target_system, uint8_t, []).
message_field('OPEN_DRONE_ID_BASIC_ID', target_component, uint8_t, []).
message_field('OPEN_DRONE_ID_BASIC_ID', id_or_mac, 'uint8_t[20]', []).
message_field('OPEN_DRONE_ID_BASIC_ID', id_type, uint8_t, [enum='MAV_ODID_ID_TYPE']).
message_field('OPEN_DRONE_ID_BASIC_ID', ua_type, uint8_t, [enum='MAV_ODID_UA_TYPE']).
message_field('OPEN_DRONE_ID_BASIC_ID', uas_id, 'uint8_t[20]', []).
message_field('OPEN_DRONE_ID_LOCATION', target_system, uint8_t, []).
message_field('OPEN_DRONE_ID_LOCATION', target_component, uint8_t, []).
message_field('OPEN_DRONE_ID_LOCATION', id_or_mac, 'uint8_t[20]', []).
message_field('OPEN_DRONE_ID_LOCATION', status, uint8_t, [enum='MAV_ODID_STATUS']).
message_field('OPEN_DRONE_ID_LOCATION', direction, uint16_t, [units=cdeg, invalid='36100']).
message_field('OPEN_DRONE_ID_LOCATION', speed_horizontal, uint16_t, [units='cm/s']).
message_field('OPEN_DRONE_ID_LOCATION', speed_vertical, int16_t, [units='cm/s']).
message_field('OPEN_DRONE_ID_LOCATION', latitude, int32_t, [units=degE7, invalid='0']).
message_field('OPEN_DRONE_ID_LOCATION', longitude, int32_t, [units=degE7, invalid='0']).
message_field('OPEN_DRONE_ID_LOCATION', altitude_barometric, float, [units=m, invalid='-1000']).
message_field('OPEN_DRONE_ID_LOCATION', altitude_geodetic, float, [units=m, invalid='-1000']).
message_field('OPEN_DRONE_ID_LOCATION', height_reference, uint8_t, [enum='MAV_ODID_HEIGHT_REF']).
message_field('OPEN_DRONE_ID_LOCATION', height, float, [units=m, invalid='-1000']).
message_field('OPEN_DRONE_ID_LOCATION', horizontal_accuracy, uint8_t, [enum='MAV_ODID_HOR_ACC']).
message_field('OPEN_DRONE_ID_LOCATION', vertical_accuracy, uint8_t, [enum='MAV_ODID_VER_ACC']).
message_field('OPEN_DRONE_ID_LOCATION', barometer_accuracy, uint8_t, [enum='MAV_ODID_VER_ACC']).
message_field('OPEN_DRONE_ID_LOCATION', speed_accuracy, uint8_t, [enum='MAV_ODID_SPEED_ACC']).
message_field('OPEN_DRONE_ID_LOCATION', timestamp, float, [units=s, invalid='0xFFFF']).
message_field('OPEN_DRONE_ID_LOCATION', timestamp_accuracy, uint8_t, [enum='MAV_ODID_TIME_ACC']).
message_field('OPEN_DRONE_ID_AUTHENTICATION', target_system, uint8_t, []).
message_field('OPEN_DRONE_ID_AUTHENTICATION', target_component, uint8_t, []).
message_field('OPEN_DRONE_ID_AUTHENTICATION', id_or_mac, 'uint8_t[20]', []).
message_field('OPEN_DRONE_ID_AUTHENTICATION', authentication_type, uint8_t, [enum='MAV_ODID_AUTH_TYPE']).
message_field('OPEN_DRONE_ID_AUTHENTICATION', data_page, uint8_t, []).
message_field('OPEN_DRONE_ID_AUTHENTICATION', last_page_index, uint8_t, []).
message_field('OPEN_DRONE_ID_AUTHENTICATION', length, uint8_t, [units=bytes]).
message_field('OPEN_DRONE_ID_AUTHENTICATION', timestamp, uint32_t, [units=s]).
message_field('OPEN_DRONE_ID_AUTHENTICATION', authentication_data, 'uint8_t[23]', []).
message_field('OPEN_DRONE_ID_SELF_ID', target_system, uint8_t, []).
message_field('OPEN_DRONE_ID_SELF_ID', target_component, uint8_t, []).
message_field('OPEN_DRONE_ID_SELF_ID', id_or_mac, 'uint8_t[20]', []).
message_field('OPEN_DRONE_ID_SELF_ID', description_type, uint8_t, [enum='MAV_ODID_DESC_TYPE']).
message_field('OPEN_DRONE_ID_SELF_ID', description, 'char[23]', []).
message_field('OPEN_DRONE_ID_SYSTEM', target_system, uint8_t, []).
message_field('OPEN_DRONE_ID_SYSTEM', target_component, uint8_t, []).
message_field('OPEN_DRONE_ID_SYSTEM', id_or_mac, 'uint8_t[20]', []).
message_field('OPEN_DRONE_ID_SYSTEM', operator_location_type, uint8_t, [enum='MAV_ODID_OPERATOR_LOCATION_TYPE']).
message_field('OPEN_DRONE_ID_SYSTEM', classification_type, uint8_t, [enum='MAV_ODID_CLASSIFICATION_TYPE']).
message_field('OPEN_DRONE_ID_SYSTEM', operator_latitude, int32_t, [units=degE7, invalid='0']).
message_field('OPEN_DRONE_ID_SYSTEM', operator_longitude, int32_t, [units=degE7, invalid='0']).
message_field('OPEN_DRONE_ID_SYSTEM', area_count, uint16_t, []).
message_field('OPEN_DRONE_ID_SYSTEM', area_radius, uint16_t, [units=m]).
message_field('OPEN_DRONE_ID_SYSTEM', area_ceiling, float, [units=m, invalid='-1000']).
message_field('OPEN_DRONE_ID_SYSTEM', area_floor, float, [units=m, invalid='-1000']).
message_field('OPEN_DRONE_ID_SYSTEM', category_eu, uint8_t, [enum='MAV_ODID_CATEGORY_EU']).
message_field('OPEN_DRONE_ID_SYSTEM', class_eu, uint8_t, [enum='MAV_ODID_CLASS_EU']).
message_field('OPEN_DRONE_ID_SYSTEM', operator_altitude_geo, float, [units=m, invalid='-1000']).
message_field('OPEN_DRONE_ID_SYSTEM', timestamp, uint32_t, [units=s]).
message_field('OPEN_DRONE_ID_OPERATOR_ID', target_system, uint8_t, []).
message_field('OPEN_DRONE_ID_OPERATOR_ID', target_component, uint8_t, []).
message_field('OPEN_DRONE_ID_OPERATOR_ID', id_or_mac, 'uint8_t[20]', []).
message_field('OPEN_DRONE_ID_OPERATOR_ID', operator_id_type, uint8_t, [enum='MAV_ODID_OPERATOR_ID_TYPE']).
message_field('OPEN_DRONE_ID_OPERATOR_ID', operator_id, 'char[20]', []).
message_field('OPEN_DRONE_ID_MESSAGE_PACK', target_system, uint8_t, []).
message_field('OPEN_DRONE_ID_MESSAGE_PACK', target_component, uint8_t, []).
message_field('OPEN_DRONE_ID_MESSAGE_PACK', id_or_mac, 'uint8_t[20]', []).
message_field('OPEN_DRONE_ID_MESSAGE_PACK', single_message_size, uint8_t, [units=bytes]).
message_field('OPEN_DRONE_ID_MESSAGE_PACK', msg_pack_size, uint8_t, []).
message_field('OPEN_DRONE_ID_MESSAGE_PACK', messages, 'uint8_t[225]', []).
message_field('OPEN_DRONE_ID_ARM_STATUS', status, uint8_t, [enum='MAV_ODID_ARM_STATUS']).
message_field('OPEN_DRONE_ID_ARM_STATUS', error, 'char[50]', []).
message_field('OPEN_DRONE_ID_SYSTEM_UPDATE', target_system, uint8_t, []).
message_field('OPEN_DRONE_ID_SYSTEM_UPDATE', target_component, uint8_t, []).
message_field('OPEN_DRONE_ID_SYSTEM_UPDATE', operator_latitude, int32_t, [units=degE7, invalid='0']).
message_field('OPEN_DRONE_ID_SYSTEM_UPDATE', operator_longitude, int32_t, [units=degE7, invalid='0']).
message_field('OPEN_DRONE_ID_SYSTEM_UPDATE', operator_altitude_geo, float, [units=m, invalid='-1000']).
message_field('OPEN_DRONE_ID_SYSTEM_UPDATE', timestamp, uint32_t, [units=s]).
message_field('HYGROMETER_SENSOR', id, uint8_t, [instance=true]).
message_field('HYGROMETER_SENSOR', temperature, int16_t, [units=cdegC]).
message_field('HYGROMETER_SENSOR', humidity, uint16_t, [units='c%']).
message_field('PARAM_ACK_TRANSACTION', target_system, uint8_t, []).
message_field('PARAM_ACK_TRANSACTION', target_component, uint8_t, []).
message_field('PARAM_ACK_TRANSACTION', param_id, 'char[16]', []).
message_field('PARAM_ACK_TRANSACTION', param_value, float, []).
message_field('PARAM_ACK_TRANSACTION', param_type, uint8_t, [enum='MAV_PARAM_TYPE']).
message_field('PARAM_ACK_TRANSACTION', param_result, uint8_t, [enum='PARAM_ACK']).
message_field('AIRSPEED', id, uint8_t, [instance=true]).
message_field('AIRSPEED', airspeed, float, [units='m/s']).
message_field('AIRSPEED', temperature, int16_t, [units=cdegC]).
message_field('AIRSPEED', raw_press, float, [units=hPa]).
message_field('AIRSPEED', flags, uint8_t, [enum='AIRSPEED_SENSOR_FLAGS']).
message_field('WIFI_NETWORK_INFO', ssid, 'char[32]', []).
message_field('WIFI_NETWORK_INFO', channel_id, uint8_t, []).
message_field('WIFI_NETWORK_INFO', signal_quality, uint8_t, [units='%']).
message_field('WIFI_NETWORK_INFO', data_rate, uint16_t, [units='MiB/s']).
message_field('WIFI_NETWORK_INFO', security, uint8_t, [enum='WIFI_NETWORK_SECURITY']).
message_field('FIGURE_EIGHT_EXECUTION_STATUS', time_usec, uint64_t, [units=us]).
message_field('FIGURE_EIGHT_EXECUTION_STATUS', major_radius, float, [units=m]).
message_field('FIGURE_EIGHT_EXECUTION_STATUS', minor_radius, float, [units=m]).
message_field('FIGURE_EIGHT_EXECUTION_STATUS', orientation, float, [units=rad]).
message_field('FIGURE_EIGHT_EXECUTION_STATUS', frame, uint8_t, [enum='MAV_FRAME']).
message_field('FIGURE_EIGHT_EXECUTION_STATUS', x, int32_t, []).
message_field('FIGURE_EIGHT_EXECUTION_STATUS', y, int32_t, []).
message_field('FIGURE_EIGHT_EXECUTION_STATUS', z, float, [units=m]).
message_field('BATTERY_STATUS_V2', id, uint8_t, [instance=true]).
message_field('BATTERY_STATUS_V2', temperature, int16_t, [units=cdegC, invalid='INT16_MAX']).
message_field('BATTERY_STATUS_V2', voltage, float, [units='V', invalid='NaN']).
message_field('BATTERY_STATUS_V2', current, float, [units='A', invalid='NaN']).
message_field('BATTERY_STATUS_V2', capacity_consumed, float, [units='Ah', invalid='NaN']).
message_field('BATTERY_STATUS_V2', capacity_remaining, float, [units='Ah', invalid='NaN']).
message_field('BATTERY_STATUS_V2', percent_remaining, uint8_t, [units='%', invalid='UINT8_MAX']).
message_field('BATTERY_STATUS_V2', status_flags, uint32_t, [display=bitmask, enum='MAV_BATTERY_STATUS_FLAGS']).
message_field('COMPONENT_INFORMATION_BASIC', time_boot_ms, uint32_t, [units=ms]).
message_field('COMPONENT_INFORMATION_BASIC', capabilities, uint64_t, [enum='MAV_PROTOCOL_CAPABILITY', display=bitmask]).
message_field('COMPONENT_INFORMATION_BASIC', vendor_name, 'char[32]', []).
message_field('COMPONENT_INFORMATION_BASIC', model_name, 'char[32]', []).
message_field('COMPONENT_INFORMATION_BASIC', software_version, 'char[24]', []).
message_field('COMPONENT_INFORMATION_BASIC', hardware_version, 'char[24]', []).
message_field('COMPONENT_INFORMATION_BASIC', serial_number, 'char[32]', []).
message_field('GROUP_START', group_id, uint32_t, []).
message_field('GROUP_START', mission_checksum, uint32_t, []).
message_field('GROUP_START', time_usec, uint64_t, [units=us]).
message_field('GROUP_END', group_id, uint32_t, []).
message_field('GROUP_END', mission_checksum, uint32_t, []).
message_field('GROUP_END', time_usec, uint64_t, [units=us]).
message_field('AVAILABLE_MODES', number_modes, uint8_t, []).
message_field('AVAILABLE_MODES', mode_index, uint8_t, []).
message_field('AVAILABLE_MODES', standard_mode, uint8_t, [enum='MAV_STANDARD_MODE']).
message_field('AVAILABLE_MODES', custom_mode, uint32_t, []).
message_field('AVAILABLE_MODES', properties, uint32_t, [enum='MAV_MODE_PROPERTY']).
message_field('AVAILABLE_MODES', mode_name, 'char[35]', []).
message_field('CURRENT_MODE', standard_mode, uint8_t, [enum='MAV_STANDARD_MODE']).
message_field('CURRENT_MODE', custom_mode, uint32_t, []).
message_field('CURRENT_MODE', intended_custom_mode, uint32_t, [invalid='0']).
message_field('AVAILABLE_MODES_MONITOR', seq, uint8_t, []).
message_field('TARGET_ABSOLUTE', timestamp, uint64_t, [units=us]).
message_field('TARGET_ABSOLUTE', id, uint8_t, []).
message_field('TARGET_ABSOLUTE', sensor_capabilities, uint8_t, [enum='TARGET_ABSOLUTE_SENSOR_CAPABILITY_FLAGS', display=bitmask]).
message_field('TARGET_ABSOLUTE', lat, int32_t, [units=degE7]).
message_field('TARGET_ABSOLUTE', lon, int32_t, [units=degE7]).
message_field('TARGET_ABSOLUTE', alt, float, [units=m]).
message_field('TARGET_ABSOLUTE', vel, 'float[3]', [units='m/s', invalid='[0]']).
message_field('TARGET_ABSOLUTE', acc, 'float[3]', [units='m/s/s', invalid='[0]']).
message_field('TARGET_ABSOLUTE', q_target, 'float[4]', [invalid='[0]']).
message_field('TARGET_ABSOLUTE', rates, 'float[3]', [units='rad/s', invalid='[0]']).
message_field('TARGET_ABSOLUTE', position_std, 'float[2]', [units=m]).
message_field('TARGET_ABSOLUTE', vel_std, 'float[3]', [units='m/s']).
message_field('TARGET_ABSOLUTE', acc_std, 'float[3]', [units='m/s/s']).
message_field('TARGET_RELATIVE', timestamp, uint64_t, [units=us]).
message_field('TARGET_RELATIVE', id, uint8_t, [instance=true]).
message_field('TARGET_RELATIVE', frame, uint8_t, [enum='TARGET_OBS_FRAME']).
message_field('TARGET_RELATIVE', x, float, [units=m]).
message_field('TARGET_RELATIVE', y, float, [units=m]).
message_field('TARGET_RELATIVE', z, float, [units=m]).
message_field('TARGET_RELATIVE', pos_std, 'float[3]', [units=m]).
message_field('TARGET_RELATIVE', yaw_std, float, [units=rad]).
message_field('TARGET_RELATIVE', q_target, 'float[4]', []).
message_field('TARGET_RELATIVE', q_sensor, 'float[4]', []).
message_field('TARGET_RELATIVE', type, uint8_t, [enum='LANDING_TARGET_TYPE']).
message_field('ICAROUS_HEARTBEAT', status, uint8_t, [enum='ICAROUS_FMS_STATE']).
message_field('ICAROUS_KINEMATIC_BANDS', numBands, int8_t, []).
message_field('ICAROUS_KINEMATIC_BANDS', type1, uint8_t, [enum='ICAROUS_TRACK_BAND_TYPES']).
message_field('ICAROUS_KINEMATIC_BANDS', min1, float, [units=deg]).
message_field('ICAROUS_KINEMATIC_BANDS', max1, float, [units=deg]).
message_field('ICAROUS_KINEMATIC_BANDS', type2, uint8_t, [enum='ICAROUS_TRACK_BAND_TYPES']).
message_field('ICAROUS_KINEMATIC_BANDS', min2, float, [units=deg]).
message_field('ICAROUS_KINEMATIC_BANDS', max2, float, [units=deg]).
message_field('ICAROUS_KINEMATIC_BANDS', type3, uint8_t, [enum='ICAROUS_TRACK_BAND_TYPES']).
message_field('ICAROUS_KINEMATIC_BANDS', min3, float, [units=deg]).
message_field('ICAROUS_KINEMATIC_BANDS', max3, float, [units=deg]).
message_field('ICAROUS_KINEMATIC_BANDS', type4, uint8_t, [enum='ICAROUS_TRACK_BAND_TYPES']).
message_field('ICAROUS_KINEMATIC_BANDS', min4, float, [units=deg]).
message_field('ICAROUS_KINEMATIC_BANDS', max4, float, [units=deg]).
message_field('ICAROUS_KINEMATIC_BANDS', type5, uint8_t, [enum='ICAROUS_TRACK_BAND_TYPES']).
message_field('ICAROUS_KINEMATIC_BANDS', min5, float, [units=deg]).
message_field('ICAROUS_KINEMATIC_BANDS', max5, float, [units=deg]).
message_field('HEARTBEAT', type, uint8_t, [enum='MAV_TYPE']).
message_field('HEARTBEAT', autopilot, uint8_t, [enum='MAV_AUTOPILOT']).
message_field('HEARTBEAT', base_mode, uint8_t, [enum='MAV_MODE_FLAG', display=bitmask]).
message_field('HEARTBEAT', custom_mode, uint32_t, []).
message_field('HEARTBEAT', system_status, uint8_t, [enum='MAV_STATE']).
message_field('HEARTBEAT', mavlink_version, uint8_t_mavlink_version, []).
message_field('PROTOCOL_VERSION', version, uint16_t, []).
message_field('PROTOCOL_VERSION', min_version, uint16_t, []).
message_field('PROTOCOL_VERSION', max_version, uint16_t, []).
message_field('PROTOCOL_VERSION', spec_version_hash, 'uint8_t[8]', []).
message_field('PROTOCOL_VERSION', library_version_hash, 'uint8_t[8]', []).
message_field('ARRAY_TEST_0', v1, uint8_t, []).
message_field('ARRAY_TEST_0', ar_i8, 'int8_t[4]', []).
message_field('ARRAY_TEST_0', ar_u8, 'uint8_t[4]', []).
message_field('ARRAY_TEST_0', ar_u16, 'uint16_t[4]', []).
message_field('ARRAY_TEST_0', ar_u32, 'uint32_t[4]', []).
message_field('ARRAY_TEST_1', ar_u32, 'uint32_t[4]', []).
message_field('ARRAY_TEST_3', v, uint8_t, []).
message_field('ARRAY_TEST_3', ar_u32, 'uint32_t[4]', []).
message_field('ARRAY_TEST_4', ar_u32, 'uint32_t[4]', []).
message_field('ARRAY_TEST_4', v, uint8_t, []).
message_field('ARRAY_TEST_5', c1, 'char[5]', []).
message_field('ARRAY_TEST_5', c2, 'char[5]', []).
message_field('ARRAY_TEST_6', v1, uint8_t, []).
message_field('ARRAY_TEST_6', v2, uint16_t, []).
message_field('ARRAY_TEST_6', v3, uint32_t, []).
message_field('ARRAY_TEST_6', ar_u32, 'uint32_t[2]', []).
message_field('ARRAY_TEST_6', ar_i32, 'int32_t[2]', []).
message_field('ARRAY_TEST_6', ar_u16, 'uint16_t[2]', []).
message_field('ARRAY_TEST_6', ar_i16, 'int16_t[2]', []).
message_field('ARRAY_TEST_6', ar_u8, 'uint8_t[2]', []).
message_field('ARRAY_TEST_6', ar_i8, 'int8_t[2]', []).
message_field('ARRAY_TEST_6', ar_c, 'char[32]', []).
message_field('ARRAY_TEST_6', ar_d, 'double[2]', []).
message_field('ARRAY_TEST_6', ar_f, 'float[2]', []).
message_field('ARRAY_TEST_7', ar_d, 'double[2]', []).
message_field('ARRAY_TEST_7', ar_f, 'float[2]', []).
message_field('ARRAY_TEST_7', ar_u32, 'uint32_t[2]', []).
message_field('ARRAY_TEST_7', ar_i32, 'int32_t[2]', []).
message_field('ARRAY_TEST_7', ar_u16, 'uint16_t[2]', []).
message_field('ARRAY_TEST_7', ar_i16, 'int16_t[2]', []).
message_field('ARRAY_TEST_7', ar_u8, 'uint8_t[2]', []).
message_field('ARRAY_TEST_7', ar_i8, 'int8_t[2]', []).
message_field('ARRAY_TEST_7', ar_c, 'char[32]', []).
message_field('ARRAY_TEST_8', v3, uint32_t, []).
message_field('ARRAY_TEST_8', ar_d, 'double[2]', []).
message_field('ARRAY_TEST_8', ar_u16, 'uint16_t[2]', []).
message_field('TEST_TYPES', c, char, []).
message_field('TEST_TYPES', s, 'char[10]', []).
message_field('TEST_TYPES', u8, uint8_t, []).
message_field('TEST_TYPES', u16, uint16_t, []).
message_field('TEST_TYPES', u32, uint32_t, [print_format='0x%08x']).
message_field('TEST_TYPES', u64, uint64_t, []).
message_field('TEST_TYPES', s8, int8_t, []).
message_field('TEST_TYPES', s16, int16_t, []).
message_field('TEST_TYPES', s32, int32_t, []).
message_field('TEST_TYPES', s64, int64_t, []).
message_field('TEST_TYPES', f, float, []).
message_field('TEST_TYPES', d, double, []).
message_field('TEST_TYPES', u8_array, 'uint8_t[3]', []).
message_field('TEST_TYPES', u16_array, 'uint16_t[3]', []).
message_field('TEST_TYPES', u32_array, 'uint32_t[3]', []).
message_field('TEST_TYPES', u64_array, 'uint64_t[3]', []).
message_field('TEST_TYPES', s8_array, 'int8_t[3]', []).
message_field('TEST_TYPES', s16_array, 'int16_t[3]', []).
message_field('TEST_TYPES', s32_array, 'int32_t[3]', []).
message_field('TEST_TYPES', s64_array, 'int64_t[3]', []).
message_field('TEST_TYPES', f_array, 'float[3]', []).
message_field('TEST_TYPES', d_array, 'double[3]', []).
message_field('NAV_FILTER_BIAS', usec, uint64_t, []).
message_field('NAV_FILTER_BIAS', accel_0, float, []).
message_field('NAV_FILTER_BIAS', accel_1, float, []).
message_field('NAV_FILTER_BIAS', accel_2, float, []).
message_field('NAV_FILTER_BIAS', gyro_0, float, []).
message_field('NAV_FILTER_BIAS', gyro_1, float, []).
message_field('NAV_FILTER_BIAS', gyro_2, float, []).
message_field('RADIO_CALIBRATION', aileron, 'uint16_t[3]', []).
message_field('RADIO_CALIBRATION', elevator, 'uint16_t[3]', []).
message_field('RADIO_CALIBRATION', rudder, 'uint16_t[3]', []).
message_field('RADIO_CALIBRATION', gyro, 'uint16_t[2]', []).
message_field('RADIO_CALIBRATION', pitch, 'uint16_t[5]', []).
message_field('RADIO_CALIBRATION', throttle, 'uint16_t[5]', []).
message_field('UALBERTA_SYS_STATUS', mode, uint8_t, []).
message_field('UALBERTA_SYS_STATUS', nav_mode, uint8_t, []).
message_field('UALBERTA_SYS_STATUS', pilot, uint8_t, []).
message_field('UAVIONIX_ADSB_OUT_CFG', 'ICAO', uint32_t, []).
message_field('UAVIONIX_ADSB_OUT_CFG', callsign, 'char[9]', []).
message_field('UAVIONIX_ADSB_OUT_CFG', emitterType, uint8_t, [enum='ADSB_EMITTER_TYPE']).
message_field('UAVIONIX_ADSB_OUT_CFG', aircraftSize, uint8_t, [enum='UAVIONIX_ADSB_OUT_CFG_AIRCRAFT_SIZE']).
message_field('UAVIONIX_ADSB_OUT_CFG', gpsOffsetLat, uint8_t, [enum='UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LAT']).
message_field('UAVIONIX_ADSB_OUT_CFG', gpsOffsetLon, uint8_t, [enum='UAVIONIX_ADSB_OUT_CFG_GPS_OFFSET_LON']).
message_field('UAVIONIX_ADSB_OUT_CFG', stallSpeed, uint16_t, [units='cm/s']).
message_field('UAVIONIX_ADSB_OUT_CFG', rfSelect, uint8_t, [enum='UAVIONIX_ADSB_OUT_RF_SELECT', display=bitmask]).
message_field('UAVIONIX_ADSB_OUT_DYNAMIC', utcTime, uint32_t, [units=s]).
message_field('UAVIONIX_ADSB_OUT_DYNAMIC', gpsLat, int32_t, [units=degE7]).
message_field('UAVIONIX_ADSB_OUT_DYNAMIC', gpsLon, int32_t, [units=degE7]).
message_field('UAVIONIX_ADSB_OUT_DYNAMIC', gpsAlt, int32_t, [units=mm]).
message_field('UAVIONIX_ADSB_OUT_DYNAMIC', gpsFix, uint8_t, [enum='UAVIONIX_ADSB_OUT_DYNAMIC_GPS_FIX']).
message_field('UAVIONIX_ADSB_OUT_DYNAMIC', numSats, uint8_t, []).
message_field('UAVIONIX_ADSB_OUT_DYNAMIC', baroAltMSL, int32_t, [units=mbar]).
message_field('UAVIONIX_ADSB_OUT_DYNAMIC', accuracyHor, uint32_t, [units=mm]).
message_field('UAVIONIX_ADSB_OUT_DYNAMIC', accuracyVert, uint16_t, [units=cm]).
message_field('UAVIONIX_ADSB_OUT_DYNAMIC', accuracyVel, uint16_t, [units='mm/s']).
message_field('UAVIONIX_ADSB_OUT_DYNAMIC', velVert, int16_t, [units='cm/s']).
message_field('UAVIONIX_ADSB_OUT_DYNAMIC', velNS, int16_t, [units='cm/s']).
message_field('UAVIONIX_ADSB_OUT_DYNAMIC', 'VelEW', int16_t, [units='cm/s']).
message_field('UAVIONIX_ADSB_OUT_DYNAMIC', emergencyStatus, uint8_t, [enum='UAVIONIX_ADSB_EMERGENCY_STATUS']).
message_field('UAVIONIX_ADSB_OUT_DYNAMIC', state, uint16_t, [enum='UAVIONIX_ADSB_OUT_DYNAMIC_STATE', display=bitmask]).
message_field('UAVIONIX_ADSB_OUT_DYNAMIC', squawk, uint16_t, []).
message_field('UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT', rfHealth, uint8_t, [enum='UAVIONIX_ADSB_RF_HEALTH', display=bitmask]).
message_field('STORM32_GIMBAL_MANAGER_INFORMATION', gimbal_id, uint8_t, [instance=true]).
message_field('STORM32_GIMBAL_MANAGER_INFORMATION', device_cap_flags, uint32_t, [enum='GIMBAL_DEVICE_CAP_FLAGS', display=bitmask]).
message_field('STORM32_GIMBAL_MANAGER_INFORMATION', manager_cap_flags, uint32_t, [enum='MAV_STORM32_GIMBAL_MANAGER_CAP_FLAGS', display=bitmask]).
message_field('STORM32_GIMBAL_MANAGER_INFORMATION', roll_min, float, [units=rad, invalid='NaN']).
message_field('STORM32_GIMBAL_MANAGER_INFORMATION', roll_max, float, [units=rad, invalid='NaN']).
message_field('STORM32_GIMBAL_MANAGER_INFORMATION', pitch_min, float, [units=rad, invalid='NaN']).
message_field('STORM32_GIMBAL_MANAGER_INFORMATION', pitch_max, float, [units=rad, invalid='NaN']).
message_field('STORM32_GIMBAL_MANAGER_INFORMATION', yaw_min, float, [units=rad, invalid='NaN']).
message_field('STORM32_GIMBAL_MANAGER_INFORMATION', yaw_max, float, [units=rad, invalid='NaN']).
message_field('STORM32_GIMBAL_MANAGER_STATUS', gimbal_id, uint8_t, [instance=true]).
message_field('STORM32_GIMBAL_MANAGER_STATUS', supervisor, uint8_t, [enum='MAV_STORM32_GIMBAL_MANAGER_CLIENT']).
message_field('STORM32_GIMBAL_MANAGER_STATUS', device_flags, uint16_t, [enum='GIMBAL_DEVICE_FLAGS']).
message_field('STORM32_GIMBAL_MANAGER_STATUS', manager_flags, uint16_t, [enum='MAV_STORM32_GIMBAL_MANAGER_FLAGS']).
message_field('STORM32_GIMBAL_MANAGER_STATUS', profile, uint8_t, [enum='MAV_STORM32_GIMBAL_MANAGER_PROFILE']).
message_field('STORM32_GIMBAL_MANAGER_CONTROL', target_system, uint8_t, []).
message_field('STORM32_GIMBAL_MANAGER_CONTROL', target_component, uint8_t, []).
message_field('STORM32_GIMBAL_MANAGER_CONTROL', gimbal_id, uint8_t, [instance=true]).
message_field('STORM32_GIMBAL_MANAGER_CONTROL', client, uint8_t, [enum='MAV_STORM32_GIMBAL_MANAGER_CLIENT']).
message_field('STORM32_GIMBAL_MANAGER_CONTROL', device_flags, uint16_t, [enum='GIMBAL_DEVICE_FLAGS', invalid='UINT16_MAX']).
message_field('STORM32_GIMBAL_MANAGER_CONTROL', manager_flags, uint16_t, [enum='MAV_STORM32_GIMBAL_MANAGER_FLAGS', invalid='0']).
message_field('STORM32_GIMBAL_MANAGER_CONTROL', q, 'float[4]', [invalid='[NaN:]']).
message_field('STORM32_GIMBAL_MANAGER_CONTROL', angular_velocity_x, float, [units='rad/s', invalid='NaN']).
message_field('STORM32_GIMBAL_MANAGER_CONTROL', angular_velocity_y, float, [units='rad/s', invalid='NaN']).
message_field('STORM32_GIMBAL_MANAGER_CONTROL', angular_velocity_z, float, [units='rad/s', invalid='NaN']).
message_field('STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW', target_system, uint8_t, []).
message_field('STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW', target_component, uint8_t, []).
message_field('STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW', gimbal_id, uint8_t, [instance=true]).
message_field('STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW', client, uint8_t, [enum='MAV_STORM32_GIMBAL_MANAGER_CLIENT']).
message_field('STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW', device_flags, uint16_t, [enum='GIMBAL_DEVICE_FLAGS', invalid='UINT16_MAX']).
message_field('STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW', manager_flags, uint16_t, [enum='MAV_STORM32_GIMBAL_MANAGER_FLAGS', invalid='0']).
message_field('STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW', pitch, float, [units=rad, invalid='NaN']).
message_field('STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW', yaw, float, [units=rad, invalid='NaN']).
message_field('STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW', pitch_rate, float, [units='rad/s', invalid='NaN']).
message_field('STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW', yaw_rate, float, [units='rad/s', invalid='NaN']).
message_field('STORM32_GIMBAL_MANAGER_CORRECT_ROLL', target_system, uint8_t, []).
message_field('STORM32_GIMBAL_MANAGER_CORRECT_ROLL', target_component, uint8_t, []).
message_field('STORM32_GIMBAL_MANAGER_CORRECT_ROLL', gimbal_id, uint8_t, [instance=true]).
message_field('STORM32_GIMBAL_MANAGER_CORRECT_ROLL', client, uint8_t, [enum='MAV_STORM32_GIMBAL_MANAGER_CLIENT']).
message_field('STORM32_GIMBAL_MANAGER_CORRECT_ROLL', roll, float, [units=rad]).
message_field('QSHOT_STATUS', mode, uint16_t, [enum='MAV_QSHOT_MODE']).
message_field('QSHOT_STATUS', shot_state, uint16_t, []).
message_field('RADIO_RC_CHANNELS', count, uint8_t, []).
message_field('RADIO_RC_CHANNELS', flags, uint8_t, [enum='RADIO_RC_CHANNELS_FLAGS', display=bitmask]).
message_field('RADIO_RC_CHANNELS', channels, 'int16_t[24]', []).
message_field('RADIO_LINK_STATS', flags, uint8_t, [enum='RADIO_LINK_STATS_FLAGS', display=bitmask]).
message_field('RADIO_LINK_STATS', rx_LQ, uint8_t, [units='c%', invalid='UINT8_MAX']).
message_field('RADIO_LINK_STATS', rx_rssi1, uint8_t, [invalid='UINT8_MAX']).
message_field('RADIO_LINK_STATS', rx_snr1, int8_t, [invalid='INT8_MAX']).
message_field('RADIO_LINK_STATS', rx_rssi2, uint8_t, [invalid='UINT8_MAX']).
message_field('RADIO_LINK_STATS', rx_snr2, int8_t, [invalid='INT8_MAX']).
message_field('RADIO_LINK_STATS', rx_receive_antenna, uint8_t, [invalid='UINT8_MAX']).
message_field('RADIO_LINK_STATS', rx_transmit_antenna, uint8_t, [invalid='UINT8_MAX']).
message_field('RADIO_LINK_STATS', tx_LQ, uint8_t, [units='c%', invalid='UINT8_MAX']).
message_field('RADIO_LINK_STATS', tx_rssi1, uint8_t, [invalid='UINT8_MAX']).
message_field('RADIO_LINK_STATS', tx_snr1, int8_t, [invalid='INT8_MAX']).
message_field('RADIO_LINK_STATS', tx_rssi2, uint8_t, [invalid='UINT8_MAX']).
message_field('RADIO_LINK_STATS', tx_snr2, int8_t, [invalid='INT8_MAX']).
message_field('RADIO_LINK_STATS', tx_receive_antenna, uint8_t, [invalid='UINT8_MAX']).
message_field('RADIO_LINK_STATS', tx_transmit_antenna, uint8_t, [invalid='UINT8_MAX']).
message_field('FRSKY_PASSTHROUGH_ARRAY', time_boot_ms, uint32_t, [units=ms]).
message_field('FRSKY_PASSTHROUGH_ARRAY', count, uint8_t, []).
message_field('FRSKY_PASSTHROUGH_ARRAY', packet_buf, 'uint8_t[240]', []).
message_field('PARAM_VALUE_ARRAY', param_count, uint16_t, []).
message_field('PARAM_VALUE_ARRAY', param_index_first, uint16_t, []).
message_field('PARAM_VALUE_ARRAY', param_array_len, uint8_t, []).
message_field('PARAM_VALUE_ARRAY', flags, uint16_t, []).
message_field('PARAM_VALUE_ARRAY', packet_buf, 'uint8_t[248]', []).
message_field('AVSS_PRS_SYS_STATUS', time_boot_ms, uint32_t, [units=ms]).
message_field('AVSS_PRS_SYS_STATUS', error_status, uint32_t, []).
message_field('AVSS_PRS_SYS_STATUS', battery_status, uint32_t, []).
message_field('AVSS_PRS_SYS_STATUS', arm_status, uint8_t, []).
message_field('AVSS_PRS_SYS_STATUS', charge_status, uint8_t, []).
message_field('AVSS_DRONE_POSITION', time_boot_ms, uint32_t, [units=ms]).
message_field('AVSS_DRONE_POSITION', lat, int32_t, [units=degE7]).
message_field('AVSS_DRONE_POSITION', lon, int32_t, [units=degE7]).
message_field('AVSS_DRONE_POSITION', alt, int32_t, [units=mm]).
message_field('AVSS_DRONE_POSITION', ground_alt, float, [units=m]).
message_field('AVSS_DRONE_POSITION', barometer_alt, float, [units=m]).
message_field('AVSS_DRONE_IMU', time_boot_ms, uint32_t, [units=ms]).
message_field('AVSS_DRONE_IMU', q1, float, []).
message_field('AVSS_DRONE_IMU', q2, float, []).
message_field('AVSS_DRONE_IMU', q3, float, []).
message_field('AVSS_DRONE_IMU', q4, float, []).
message_field('AVSS_DRONE_IMU', xacc, float, [units='m/s/s']).
message_field('AVSS_DRONE_IMU', yacc, float, [units='m/s/s']).
message_field('AVSS_DRONE_IMU', zacc, float, [units='m/s/s']).
message_field('AVSS_DRONE_IMU', xgyro, float, [units='rad/s']).
message_field('AVSS_DRONE_IMU', ygyro, float, [units='rad/s']).
message_field('AVSS_DRONE_IMU', zgyro, float, [units='rad/s']).
message_field('AVSS_DRONE_OPERATION_MODE', time_boot_ms, uint32_t, [units=ms]).
message_field('AVSS_DRONE_OPERATION_MODE', 'M300_operation_mode', uint8_t, []).
message_field('AVSS_DRONE_OPERATION_MODE', horsefly_operation_mode, uint8_t, []).
message_field('CUBEPILOT_RAW_RC', rc_raw, 'uint8_t[32]', []).
message_field('HERELINK_VIDEO_STREAM_INFORMATION', camera_id, uint8_t, []).
message_field('HERELINK_VIDEO_STREAM_INFORMATION', status, uint8_t, []).
message_field('HERELINK_VIDEO_STREAM_INFORMATION', framerate, float, [units='Hz']).
message_field('HERELINK_VIDEO_STREAM_INFORMATION', resolution_h, uint16_t, [units=pix]).
message_field('HERELINK_VIDEO_STREAM_INFORMATION', resolution_v, uint16_t, [units=pix]).
message_field('HERELINK_VIDEO_STREAM_INFORMATION', bitrate, uint32_t, [units='bits/s']).
message_field('HERELINK_VIDEO_STREAM_INFORMATION', rotation, uint16_t, [units=deg]).
message_field('HERELINK_VIDEO_STREAM_INFORMATION', uri, 'char[230]', []).
message_field('HERELINK_TELEM', rssi, uint8_t, []).
message_field('HERELINK_TELEM', snr, int16_t, []).
message_field('HERELINK_TELEM', rf_freq, uint32_t, []).
message_field('HERELINK_TELEM', link_bw, uint32_t, []).
message_field('HERELINK_TELEM', link_rate, uint32_t, []).
message_field('HERELINK_TELEM', cpu_temp, int16_t, []).
message_field('HERELINK_TELEM', board_temp, int16_t, []).
message_field('CUBEPILOT_FIRMWARE_UPDATE_START', target_system, uint8_t, []).
message_field('CUBEPILOT_FIRMWARE_UPDATE_START', target_component, uint8_t, []).
message_field('CUBEPILOT_FIRMWARE_UPDATE_START', size, uint32_t, [units=bytes]).
message_field('CUBEPILOT_FIRMWARE_UPDATE_START', crc, uint32_t, []).
message_field('CUBEPILOT_FIRMWARE_UPDATE_RESP', target_system, uint8_t, []).
message_field('CUBEPILOT_FIRMWARE_UPDATE_RESP', target_component, uint8_t, []).
message_field('CUBEPILOT_FIRMWARE_UPDATE_RESP', offset, uint32_t, [units=bytes]).
message_field('AIRLINK_AUTH', login, 'char[50]', []).
message_field('AIRLINK_AUTH', password, 'char[50]', []).
message_field('AIRLINK_AUTH_RESPONSE', resp_type, uint8_t, [enum='AIRLINK_AUTH_RESPONSE_TYPE']).

:- dynamic message_field_len/3.

message_field_len('DATA16', data, 16).
message_field_len('DATA32', data, 32).
message_field_len('DATA64', data, 64).
message_field_len('DATA96', data, 96).
message_field_len('REMOTE_LOG_DATA_BLOCK', data, 200).
message_field_len('LED_CONTROL', custom_bytes, 24).
message_field_len('MAG_CAL_PROGRESS', completion_mask, 10).
message_field_len('GOPRO_GET_RESPONSE', value, 4).
message_field_len('GOPRO_SET_REQUEST', value, 4).
message_field_len('DEVICE_OP_READ', busname, 40).
message_field_len('DEVICE_OP_READ_REPLY', data, 128).
message_field_len('DEVICE_OP_WRITE', busname, 40).
message_field_len('DEVICE_OP_WRITE', data, 128).
message_field_len('ESC_TELEMETRY_1_TO_4', count, 4).
message_field_len('ESC_TELEMETRY_5_TO_8', count, 4).
message_field_len('ESC_TELEMETRY_9_TO_12', count, 4).
message_field_len('OSD_PARAM_CONFIG', param_id, 16).
message_field_len('OSD_PARAM_SHOW_CONFIG_REPLY', param_id, 16).
message_field_len('ASLUAV_STATUS', 'Servo_status', 8).
message_field_len('CHANGE_OPERATOR_CONTROL', passkey, 25).
message_field_len('AUTH_KEY', key, 32).
message_field_len('PARAM_REQUEST_READ', param_id, 16).
message_field_len('PARAM_VALUE', param_id, 16).
message_field_len('PARAM_SET', param_id, 16).
message_field_len('GPS_STATUS', satellite_prn, 20).
message_field_len('GPS_STATUS', satellite_used, 20).
message_field_len('PARAM_MAP_RC', param_id, 16).
message_field_len('ATTITUDE_QUATERNION_COV', q, 4).
message_field_len('SET_ATTITUDE_TARGET', q, 4).
message_field_len('SET_ATTITUDE_TARGET', thrust_body, 3).
message_field_len('ATTITUDE_TARGET', q, 4).
message_field_len('HIL_ACTUATOR_CONTROLS', controls, 16).
message_field_len('FILE_TRANSFER_PROTOCOL', payload, 251).
message_field_len('HIL_STATE_QUATERNION', attitude_quaternion, 4).
message_field_len('LOG_DATA', data, 90).
message_field_len('GPS_INJECT_DATA', data, 110).
message_field_len('SERIAL_CONTROL', data, 70).
message_field_len('ENCAPSULATED_DATA', data, 253).
message_field_len('ATT_POS_MOCAP', q, 4).
message_field_len('SET_ACTUATOR_CONTROL_TARGET', controls, 8).
message_field_len('ACTUATOR_CONTROL_TARGET', controls, 8).
message_field_len('RESOURCE_REQUEST', uri, 120).
message_field_len('RESOURCE_REQUEST', storage, 120).
message_field_len('FOLLOW_TARGET', position_cov, 3).
message_field_len('CONTROL_SYSTEM_STATE', vel_variance, 3).
message_field_len('CONTROL_SYSTEM_STATE', pos_variance, 3).
message_field_len('CONTROL_SYSTEM_STATE', q, 4).
message_field_len('AUTOPILOT_VERSION', flight_custom_version, 8).
message_field_len('AUTOPILOT_VERSION', middleware_custom_version, 8).
message_field_len('AUTOPILOT_VERSION', os_custom_version, 8).
message_field_len('AUTOPILOT_VERSION', uid2, 18).
message_field_len('LANDING_TARGET', q, 4).
message_field_len('GPS_RTCM_DATA', data, 180).
message_field_len('SET_HOME_POSITION', q, 4).
message_field_len('ADSB_VEHICLE', callsign, 9).
message_field_len('V2_EXTENSION', payload, 249).
message_field_len('MEMORY_VECT', value, 32).
message_field_len('STATUSTEXT', text, 50).
message_field_len('SETUP_SIGNING', secret_key, 32).
message_field_len('PLAY_TUNE', tune, 30).
message_field_len('PLAY_TUNE', tune2, 200).
message_field_len('CAMERA_INFORMATION', vendor_name, 32).
message_field_len('CAMERA_INFORMATION', model_name, 32).
message_field_len('CAMERA_INFORMATION', cam_definition_uri, 140).
message_field_len('STORAGE_INFORMATION', name, 32).
message_field_len('CAMERA_IMAGE_CAPTURED', q, 4).
message_field_len('CAMERA_IMAGE_CAPTURED', file_url, 205).
message_field_len('LOGGING_DATA', data, 249).
message_field_len('LOGGING_DATA_ACKED', data, 249).
message_field_len('VIDEO_STREAM_INFORMATION', name, 32).
message_field_len('VIDEO_STREAM_INFORMATION', uri, 160).
message_field_len('CAMERA_FOV_STATUS', q, 4).
message_field_len('GIMBAL_MANAGER_SET_ATTITUDE', q, 4).
message_field_len('GIMBAL_DEVICE_INFORMATION', vendor_name, 32).
message_field_len('GIMBAL_DEVICE_INFORMATION', model_name, 32).
message_field_len('GIMBAL_DEVICE_INFORMATION', custom_name, 32).
message_field_len('GIMBAL_DEVICE_ATTITUDE_STATUS', q, 4).
message_field_len('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', q, 4).
message_field_len('ESC_INFO', error_count, 4).
message_field_len('WIFI_CONFIG_AP', ssid, 32).
message_field_len('WIFI_CONFIG_AP', password, 64).
message_field_len('AIS_VESSEL', callsign, 7).
message_field_len('AIS_VESSEL', name, 20).
message_field_len('UAVCAN_NODE_INFO', name, 80).
message_field_len('UAVCAN_NODE_INFO', hw_unique_id, 16).
message_field_len('PARAM_EXT_REQUEST_READ', param_id, 16).
message_field_len('PARAM_EXT_VALUE', param_id, 16).
message_field_len('PARAM_EXT_VALUE', param_value, 128).
message_field_len('PARAM_EXT_SET', param_id, 16).
message_field_len('PARAM_EXT_SET', param_value, 128).
message_field_len('PARAM_EXT_ACK', param_id, 16).
message_field_len('PARAM_EXT_ACK', param_value, 128).
message_field_len('ODOMETRY', q, 4).
message_field_len('CELLULAR_CONFIG', pin, 16).
message_field_len('CELLULAR_CONFIG', new_pin, 16).
message_field_len('CELLULAR_CONFIG', apn, 32).
message_field_len('CELLULAR_CONFIG', puk, 16).
message_field_len('UTM_GLOBAL_POSITION', uas_id, 18).
message_field_len('DEBUG_FLOAT_ARRAY', name, 10).
message_field_len('DEBUG_FLOAT_ARRAY', data, 58).
message_field_len('ACTUATOR_OUTPUT_STATUS', actuator, 32).
message_field_len('TUNNEL', payload, 128).
message_field_len('CAN_FRAME', data, 8).
message_field_len('ONBOARD_COMPUTER_STATUS', link_type, 6).
message_field_len('COMPONENT_INFORMATION', general_metadata_uri, 100).
message_field_len('COMPONENT_INFORMATION', peripherals_metadata_uri, 100).
message_field_len('COMPONENT_METADATA', uri, 100).
message_field_len('PLAY_TUNE_V2', tune, 248).
message_field_len('EVENT', arguments, 40).
message_field_len('CANFD_FRAME', data, 64).
message_field_len('CAN_FILTER_MODIFY', ids, 16).
message_field_len('OPEN_DRONE_ID_BASIC_ID', id_or_mac, 20).
message_field_len('OPEN_DRONE_ID_BASIC_ID', uas_id, 20).
message_field_len('OPEN_DRONE_ID_LOCATION', id_or_mac, 20).
message_field_len('OPEN_DRONE_ID_AUTHENTICATION', id_or_mac, 20).
message_field_len('OPEN_DRONE_ID_AUTHENTICATION', authentication_data, 23).
message_field_len('OPEN_DRONE_ID_SELF_ID', id_or_mac, 20).
message_field_len('OPEN_DRONE_ID_SELF_ID', description, 23).
message_field_len('OPEN_DRONE_ID_SYSTEM', id_or_mac, 20).
message_field_len('OPEN_DRONE_ID_OPERATOR_ID', id_or_mac, 20).
message_field_len('OPEN_DRONE_ID_OPERATOR_ID', operator_id, 20).
message_field_len('OPEN_DRONE_ID_MESSAGE_PACK', id_or_mac, 20).
message_field_len('OPEN_DRONE_ID_MESSAGE_PACK', messages, 225).
message_field_len('OPEN_DRONE_ID_ARM_STATUS', error, 50).
message_field_len('PARAM_ACK_TRANSACTION', param_id, 16).
message_field_len('WIFI_NETWORK_INFO', ssid, 32).
message_field_len('COMPONENT_INFORMATION_BASIC', vendor_name, 32).
message_field_len('COMPONENT_INFORMATION_BASIC', model_name, 32).
message_field_len('COMPONENT_INFORMATION_BASIC', software_version, 24).
message_field_len('COMPONENT_INFORMATION_BASIC', hardware_version, 24).
message_field_len('COMPONENT_INFORMATION_BASIC', serial_number, 32).
message_field_len('AVAILABLE_MODES', mode_name, 35).
message_field_len('TARGET_RELATIVE', q_target, 4).
message_field_len('TARGET_RELATIVE', q_sensor, 4).
message_field_len('PROTOCOL_VERSION', spec_version_hash, 8).
message_field_len('PROTOCOL_VERSION', library_version_hash, 8).
message_field_len('ARRAY_TEST_0', ar_i8, 4).
message_field_len('ARRAY_TEST_0', ar_u8, 4).
message_field_len('ARRAY_TEST_0', ar_u16, 4).
message_field_len('ARRAY_TEST_0', ar_u32, 4).
message_field_len('ARRAY_TEST_1', ar_u32, 4).
message_field_len('ARRAY_TEST_3', ar_u32, 4).
message_field_len('ARRAY_TEST_4', ar_u32, 4).
message_field_len('ARRAY_TEST_5', c1, 5).
message_field_len('ARRAY_TEST_5', c2, 5).
message_field_len('ARRAY_TEST_6', ar_u32, 2).
message_field_len('ARRAY_TEST_6', ar_i32, 2).
message_field_len('ARRAY_TEST_6', ar_u16, 2).
message_field_len('ARRAY_TEST_6', ar_i16, 2).
message_field_len('ARRAY_TEST_6', ar_u8, 2).
message_field_len('ARRAY_TEST_6', ar_i8, 2).
message_field_len('ARRAY_TEST_6', ar_c, 32).
message_field_len('ARRAY_TEST_6', ar_d, 2).
message_field_len('ARRAY_TEST_6', ar_f, 2).
message_field_len('ARRAY_TEST_7', ar_d, 2).
message_field_len('ARRAY_TEST_7', ar_f, 2).
message_field_len('ARRAY_TEST_7', ar_u32, 2).
message_field_len('ARRAY_TEST_7', ar_i32, 2).
message_field_len('ARRAY_TEST_7', ar_u16, 2).
message_field_len('ARRAY_TEST_7', ar_i16, 2).
message_field_len('ARRAY_TEST_7', ar_u8, 2).
message_field_len('ARRAY_TEST_7', ar_i8, 2).
message_field_len('ARRAY_TEST_7', ar_c, 32).
message_field_len('ARRAY_TEST_8', ar_d, 2).
message_field_len('ARRAY_TEST_8', ar_u16, 2).
message_field_len('TEST_TYPES', s, 10).
message_field_len('TEST_TYPES', u8_array, 3).
message_field_len('TEST_TYPES', u16_array, 3).
message_field_len('TEST_TYPES', u32_array, 3).
message_field_len('TEST_TYPES', u64_array, 3).
message_field_len('TEST_TYPES', s8_array, 3).
message_field_len('TEST_TYPES', s16_array, 3).
message_field_len('TEST_TYPES', s32_array, 3).
message_field_len('TEST_TYPES', s64_array, 3).
message_field_len('TEST_TYPES', f_array, 3).
message_field_len('TEST_TYPES', d_array, 3).
message_field_len('RADIO_CALIBRATION', aileron, 3).
message_field_len('RADIO_CALIBRATION', elevator, 3).
message_field_len('RADIO_CALIBRATION', rudder, 3).
message_field_len('RADIO_CALIBRATION', gyro, 2).
message_field_len('RADIO_CALIBRATION', pitch, 5).
message_field_len('RADIO_CALIBRATION', throttle, 5).
message_field_len('UAVIONIX_ADSB_OUT_CFG', callsign, 9).
message_field_len('RADIO_RC_CHANNELS', channels, 24).
message_field_len('FRSKY_PASSTHROUGH_ARRAY', packet_buf, 240).
message_field_len('PARAM_VALUE_ARRAY', packet_buf, 248).
message_field_len('CUBEPILOT_RAW_RC', rc_raw, 32).
message_field_len('HERELINK_VIDEO_STREAM_INFORMATION', uri, 230).
message_field_len('AIRLINK_AUTH', login, 50).
message_field_len('AIRLINK_AUTH', password, 50).

:- dynamic message_field_type/3.

message_field_type('SENSOR_OFFSETS', mag_ofs_x, int(16)).
message_field_type('SENSOR_OFFSETS', mag_ofs_y, int(16)).
message_field_type('SENSOR_OFFSETS', mag_ofs_z, int(16)).
message_field_type('SENSOR_OFFSETS', raw_press, int(32)).
message_field_type('SENSOR_OFFSETS', raw_temp, int(32)).
message_field_type('SENSOR_OFFSETS', gyro_cal_x, float).
message_field_type('SENSOR_OFFSETS', gyro_cal_y, float).
message_field_type('SENSOR_OFFSETS', gyro_cal_z, float).
message_field_type('SENSOR_OFFSETS', accel_cal_x, float).
message_field_type('SENSOR_OFFSETS', accel_cal_y, float).
message_field_type('SENSOR_OFFSETS', accel_cal_z, float).
message_field_type('SET_MAG_OFFSETS', target_system, uint(8)).
message_field_type('SET_MAG_OFFSETS', target_component, uint(8)).
message_field_type('SET_MAG_OFFSETS', mag_ofs_x, int(16)).
message_field_type('SET_MAG_OFFSETS', mag_ofs_y, int(16)).
message_field_type('SET_MAG_OFFSETS', mag_ofs_z, int(16)).
message_field_type('MEMINFO', brkval, uint(16)).
message_field_type('AP_ADC', adc1, uint(16)).
message_field_type('AP_ADC', adc2, uint(16)).
message_field_type('AP_ADC', adc3, uint(16)).
message_field_type('AP_ADC', adc4, uint(16)).
message_field_type('AP_ADC', adc5, uint(16)).
message_field_type('AP_ADC', adc6, uint(16)).
message_field_type('DIGICAM_CONFIGURE', target_system, uint(8)).
message_field_type('DIGICAM_CONFIGURE', target_component, uint(8)).
message_field_type('DIGICAM_CONFIGURE', mode, uint(8)).
message_field_type('DIGICAM_CONFIGURE', shutter_speed, uint(16)).
message_field_type('DIGICAM_CONFIGURE', aperture, uint(8)).
message_field_type('DIGICAM_CONFIGURE', iso, uint(8)).
message_field_type('DIGICAM_CONFIGURE', exposure_type, uint(8)).
message_field_type('DIGICAM_CONFIGURE', command_id, uint(8)).
message_field_type('DIGICAM_CONFIGURE', extra_param, uint(8)).
message_field_type('DIGICAM_CONFIGURE', extra_value, float).
message_field_type('DIGICAM_CONTROL', target_system, uint(8)).
message_field_type('DIGICAM_CONTROL', target_component, uint(8)).
message_field_type('DIGICAM_CONTROL', session, uint(8)).
message_field_type('DIGICAM_CONTROL', zoom_pos, uint(8)).
message_field_type('DIGICAM_CONTROL', zoom_step, int(8)).
message_field_type('DIGICAM_CONTROL', focus_lock, uint(8)).
message_field_type('DIGICAM_CONTROL', shot, uint(8)).
message_field_type('DIGICAM_CONTROL', command_id, uint(8)).
message_field_type('DIGICAM_CONTROL', extra_param, uint(8)).
message_field_type('DIGICAM_CONTROL', extra_value, float).
message_field_type('MOUNT_CONFIGURE', target_system, uint(8)).
message_field_type('MOUNT_CONFIGURE', target_component, uint(8)).
message_field_type('MOUNT_CONFIGURE', stab_roll, uint(8)).
message_field_type('MOUNT_CONFIGURE', stab_pitch, uint(8)).
message_field_type('MOUNT_CONFIGURE', stab_yaw, uint(8)).
message_field_type('MOUNT_CONTROL', target_system, uint(8)).
message_field_type('MOUNT_CONTROL', target_component, uint(8)).
message_field_type('MOUNT_CONTROL', input_a, int(32)).
message_field_type('MOUNT_CONTROL', input_b, int(32)).
message_field_type('MOUNT_CONTROL', input_c, int(32)).
message_field_type('MOUNT_CONTROL', save_position, uint(8)).
message_field_type('MOUNT_STATUS', target_system, uint(8)).
message_field_type('MOUNT_STATUS', target_component, uint(8)).
message_field_type('FENCE_POINT', target_system, uint(8)).
message_field_type('FENCE_POINT', target_component, uint(8)).
message_field_type('FENCE_POINT', idx, uint(8)).
message_field_type('FENCE_POINT', count, uint(8)).
message_field_type('FENCE_FETCH_POINT', target_system, uint(8)).
message_field_type('FENCE_FETCH_POINT', target_component, uint(8)).
message_field_type('FENCE_FETCH_POINT', idx, uint(8)).
message_field_type('AHRS', accel_weight, float).
message_field_type('AHRS', renorm_val, float).
message_field_type('AHRS', error_rp, float).
message_field_type('AHRS', error_yaw, float).
message_field_type('HWSTATUS', 'I2Cerr', uint(8)).
message_field_type('RADIO', rssi, uint(8)).
message_field_type('RADIO', remrssi, uint(8)).
message_field_type('RADIO', noise, uint(8)).
message_field_type('RADIO', remnoise, uint(8)).
message_field_type('RADIO', rxerrors, uint(16)).
message_field_type('RADIO', fixed, uint(16)).
message_field_type('LIMITS_STATUS', breach_count, uint(16)).
message_field_type('DATA16', type, uint(8)).
message_field_type('DATA16', data, uint(8)).
message_field_type('DATA32', type, uint(8)).
message_field_type('DATA32', data, uint(8)).
message_field_type('DATA64', type, uint(8)).
message_field_type('DATA64', data, uint(8)).
message_field_type('DATA96', type, uint(8)).
message_field_type('DATA96', data, uint(8)).
message_field_type('AIRSPEED_AUTOCAL', 'EAS2TAS', float).
message_field_type('AIRSPEED_AUTOCAL', ratio, float).
message_field_type('AIRSPEED_AUTOCAL', state_x, float).
message_field_type('AIRSPEED_AUTOCAL', state_y, float).
message_field_type('AIRSPEED_AUTOCAL', state_z, float).
message_field_type('AIRSPEED_AUTOCAL', 'Pax', float).
message_field_type('AIRSPEED_AUTOCAL', 'Pby', float).
message_field_type('AIRSPEED_AUTOCAL', 'Pcz', float).
message_field_type('RALLY_POINT', target_system, uint(8)).
message_field_type('RALLY_POINT', target_component, uint(8)).
message_field_type('RALLY_POINT', idx, uint(8)).
message_field_type('RALLY_POINT', count, uint(8)).
message_field_type('RALLY_FETCH_POINT', target_system, uint(8)).
message_field_type('RALLY_FETCH_POINT', target_component, uint(8)).
message_field_type('RALLY_FETCH_POINT', idx, uint(8)).
message_field_type('COMPASSMOT_STATUS', 'CompensationX', float).
message_field_type('COMPASSMOT_STATUS', 'CompensationY', float).
message_field_type('COMPASSMOT_STATUS', 'CompensationZ', float).
message_field_type('CAMERA_STATUS', target_system, uint(8)).
message_field_type('CAMERA_STATUS', cam_idx, uint(8)).
message_field_type('CAMERA_STATUS', img_idx, uint(16)).
message_field_type('CAMERA_STATUS', p1, float).
message_field_type('CAMERA_STATUS', p2, float).
message_field_type('CAMERA_STATUS', p3, float).
message_field_type('CAMERA_STATUS', p4, float).
message_field_type('CAMERA_FEEDBACK', target_system, uint(8)).
message_field_type('CAMERA_FEEDBACK', cam_idx, uint(8)).
message_field_type('CAMERA_FEEDBACK', img_idx, uint(16)).
message_field_type('CAMERA_FEEDBACK', completed_captures, uint(16)).
message_field_type('AHRS3', v1, float).
message_field_type('AHRS3', v2, float).
message_field_type('AHRS3', v3, float).
message_field_type('AHRS3', v4, float).
message_field_type('AUTOPILOT_VERSION_REQUEST', target_system, uint(8)).
message_field_type('AUTOPILOT_VERSION_REQUEST', target_component, uint(8)).
message_field_type('REMOTE_LOG_DATA_BLOCK', target_system, uint(8)).
message_field_type('REMOTE_LOG_DATA_BLOCK', target_component, uint(8)).
message_field_type('REMOTE_LOG_DATA_BLOCK', data, uint(8)).
message_field_type('REMOTE_LOG_BLOCK_STATUS', target_system, uint(8)).
message_field_type('REMOTE_LOG_BLOCK_STATUS', target_component, uint(8)).
message_field_type('REMOTE_LOG_BLOCK_STATUS', seqno, uint(32)).
message_field_type('LED_CONTROL', target_system, uint(8)).
message_field_type('LED_CONTROL', target_component, uint(8)).
message_field_type('LED_CONTROL', instance, uint(8)).
message_field_type('LED_CONTROL', pattern, uint(8)).
message_field_type('LED_CONTROL', custom_len, uint(8)).
message_field_type('LED_CONTROL', custom_bytes, uint(8)).
message_field_type('MAG_CAL_PROGRESS', attempt, uint(8)).
message_field_type('MAG_CAL_PROGRESS', completion_mask, uint(8)).
message_field_type('MAG_CAL_PROGRESS', direction_x, float).
message_field_type('MAG_CAL_PROGRESS', direction_y, float).
message_field_type('MAG_CAL_PROGRESS', direction_z, float).
message_field_type('EKF_STATUS_REPORT', velocity_variance, float).
message_field_type('EKF_STATUS_REPORT', pos_horiz_variance, float).
message_field_type('EKF_STATUS_REPORT', pos_vert_variance, float).
message_field_type('EKF_STATUS_REPORT', compass_variance, float).
message_field_type('EKF_STATUS_REPORT', terrain_alt_variance, float).
message_field_type('EKF_STATUS_REPORT', airspeed_variance, float).
message_field_type('PID_TUNING', desired, float).
message_field_type('PID_TUNING', achieved, float).
message_field_type('PID_TUNING', 'FF', float).
message_field_type('PID_TUNING', 'P', float).
message_field_type('PID_TUNING', 'I', float).
message_field_type('PID_TUNING', 'D', float).
message_field_type('PID_TUNING', 'SRate', float).
message_field_type('PID_TUNING', 'PDmod', float).
message_field_type('GIMBAL_REPORT', target_system, uint(8)).
message_field_type('GIMBAL_REPORT', target_component, uint(8)).
message_field_type('GIMBAL_CONTROL', target_system, uint(8)).
message_field_type('GIMBAL_CONTROL', target_component, uint(8)).
message_field_type('GIMBAL_TORQUE_CMD_REPORT', target_system, uint(8)).
message_field_type('GIMBAL_TORQUE_CMD_REPORT', target_component, uint(8)).
message_field_type('GIMBAL_TORQUE_CMD_REPORT', rl_torque_cmd, int(16)).
message_field_type('GIMBAL_TORQUE_CMD_REPORT', el_torque_cmd, int(16)).
message_field_type('GIMBAL_TORQUE_CMD_REPORT', az_torque_cmd, int(16)).
message_field_type('GOPRO_GET_REQUEST', target_system, uint(8)).
message_field_type('GOPRO_GET_REQUEST', target_component, uint(8)).
message_field_type('GOPRO_GET_RESPONSE', value, uint(8)).
message_field_type('GOPRO_SET_REQUEST', target_system, uint(8)).
message_field_type('GOPRO_SET_REQUEST', target_component, uint(8)).
message_field_type('GOPRO_SET_REQUEST', value, uint(8)).
message_field_type('RPM', rpm1, float).
message_field_type('RPM', rpm2, float).
message_field_type('DEVICE_OP_READ', target_system, uint(8)).
message_field_type('DEVICE_OP_READ', target_component, uint(8)).
message_field_type('DEVICE_OP_READ', request_id, uint(32)).
message_field_type('DEVICE_OP_READ', bus, uint(8)).
message_field_type('DEVICE_OP_READ', address, uint(8)).
message_field_type('DEVICE_OP_READ', busname, char).
message_field_type('DEVICE_OP_READ', regstart, uint(8)).
message_field_type('DEVICE_OP_READ', count, uint(8)).
message_field_type('DEVICE_OP_READ', bank, uint(8)).
message_field_type('DEVICE_OP_READ_REPLY', request_id, uint(32)).
message_field_type('DEVICE_OP_READ_REPLY', result, uint(8)).
message_field_type('DEVICE_OP_READ_REPLY', regstart, uint(8)).
message_field_type('DEVICE_OP_READ_REPLY', count, uint(8)).
message_field_type('DEVICE_OP_READ_REPLY', data, uint(8)).
message_field_type('DEVICE_OP_READ_REPLY', bank, uint(8)).
message_field_type('DEVICE_OP_WRITE', target_system, uint(8)).
message_field_type('DEVICE_OP_WRITE', target_component, uint(8)).
message_field_type('DEVICE_OP_WRITE', request_id, uint(32)).
message_field_type('DEVICE_OP_WRITE', bus, uint(8)).
message_field_type('DEVICE_OP_WRITE', address, uint(8)).
message_field_type('DEVICE_OP_WRITE', busname, char).
message_field_type('DEVICE_OP_WRITE', regstart, uint(8)).
message_field_type('DEVICE_OP_WRITE', count, uint(8)).
message_field_type('DEVICE_OP_WRITE', data, uint(8)).
message_field_type('DEVICE_OP_WRITE', bank, uint(8)).
message_field_type('DEVICE_OP_WRITE_REPLY', request_id, uint(32)).
message_field_type('DEVICE_OP_WRITE_REPLY', result, uint(8)).
message_field_type('ADAP_TUNING', error, float).
message_field_type('ADAP_TUNING', theta, float).
message_field_type('ADAP_TUNING', omega, float).
message_field_type('ADAP_TUNING', sigma, float).
message_field_type('ADAP_TUNING', theta_dot, float).
message_field_type('ADAP_TUNING', omega_dot, float).
message_field_type('ADAP_TUNING', sigma_dot, float).
message_field_type('ADAP_TUNING', f, float).
message_field_type('ADAP_TUNING', f_dot, float).
message_field_type('ADAP_TUNING', u, float).
message_field_type('ESC_TELEMETRY_1_TO_4', count, uint(16)).
message_field_type('ESC_TELEMETRY_5_TO_8', count, uint(16)).
message_field_type('ESC_TELEMETRY_9_TO_12', count, uint(16)).
message_field_type('OSD_PARAM_CONFIG', target_system, uint(8)).
message_field_type('OSD_PARAM_CONFIG', target_component, uint(8)).
message_field_type('OSD_PARAM_CONFIG', request_id, uint(32)).
message_field_type('OSD_PARAM_CONFIG', osd_screen, uint(8)).
message_field_type('OSD_PARAM_CONFIG', osd_index, uint(8)).
message_field_type('OSD_PARAM_CONFIG', param_id, char).
message_field_type('OSD_PARAM_CONFIG', min_value, float).
message_field_type('OSD_PARAM_CONFIG', max_value, float).
message_field_type('OSD_PARAM_CONFIG', increment, float).
message_field_type('OSD_PARAM_CONFIG_REPLY', request_id, uint(32)).
message_field_type('OSD_PARAM_SHOW_CONFIG', target_system, uint(8)).
message_field_type('OSD_PARAM_SHOW_CONFIG', target_component, uint(8)).
message_field_type('OSD_PARAM_SHOW_CONFIG', request_id, uint(32)).
message_field_type('OSD_PARAM_SHOW_CONFIG', osd_screen, uint(8)).
message_field_type('OSD_PARAM_SHOW_CONFIG', osd_index, uint(8)).
message_field_type('OSD_PARAM_SHOW_CONFIG_REPLY', request_id, uint(32)).
message_field_type('OSD_PARAM_SHOW_CONFIG_REPLY', param_id, char).
message_field_type('OSD_PARAM_SHOW_CONFIG_REPLY', min_value, float).
message_field_type('OSD_PARAM_SHOW_CONFIG_REPLY', max_value, float).
message_field_type('OSD_PARAM_SHOW_CONFIG_REPLY', increment, float).
message_field_type('WATER_DEPTH', healthy, uint(8)).
message_field_type('COMMAND_INT_STAMPED', utc_time, uint(32)).
message_field_type('COMMAND_INT_STAMPED', vehicle_timestamp, uint(64)).
message_field_type('COMMAND_INT_STAMPED', target_system, uint(8)).
message_field_type('COMMAND_INT_STAMPED', target_component, uint(8)).
message_field_type('COMMAND_INT_STAMPED', current, uint(8)).
message_field_type('COMMAND_INT_STAMPED', autocontinue, uint(8)).
message_field_type('COMMAND_INT_STAMPED', param1, float).
message_field_type('COMMAND_INT_STAMPED', param2, float).
message_field_type('COMMAND_INT_STAMPED', param3, float).
message_field_type('COMMAND_INT_STAMPED', param4, float).
message_field_type('COMMAND_INT_STAMPED', x, int(32)).
message_field_type('COMMAND_INT_STAMPED', y, int(32)).
message_field_type('COMMAND_INT_STAMPED', z, float).
message_field_type('COMMAND_LONG_STAMPED', utc_time, uint(32)).
message_field_type('COMMAND_LONG_STAMPED', vehicle_timestamp, uint(64)).
message_field_type('COMMAND_LONG_STAMPED', target_system, uint(8)).
message_field_type('COMMAND_LONG_STAMPED', target_component, uint(8)).
message_field_type('COMMAND_LONG_STAMPED', confirmation, uint(8)).
message_field_type('COMMAND_LONG_STAMPED', param1, float).
message_field_type('COMMAND_LONG_STAMPED', param2, float).
message_field_type('COMMAND_LONG_STAMPED', param3, float).
message_field_type('COMMAND_LONG_STAMPED', param4, float).
message_field_type('COMMAND_LONG_STAMPED', param5, float).
message_field_type('COMMAND_LONG_STAMPED', param6, float).
message_field_type('COMMAND_LONG_STAMPED', param7, float).
message_field_type('SENS_MPPT', mppt1_status, uint(8)).
message_field_type('SENS_MPPT', mppt2_status, uint(8)).
message_field_type('SENS_MPPT', mppt3_status, uint(8)).
message_field_type('ASLCTRL_DATA', aslctrl_mode, uint(8)).
message_field_type('ASLCTRL_DATA', h, float).
message_field_type('ASLCTRL_DATA', hRef, float).
message_field_type('ASLCTRL_DATA', hRef_t, float).
message_field_type('ASLCTRL_DATA', q, float).
message_field_type('ASLCTRL_DATA', qRef, float).
message_field_type('ASLCTRL_DATA', uElev, float).
message_field_type('ASLCTRL_DATA', uThrot, float).
message_field_type('ASLCTRL_DATA', uThrot2, float).
message_field_type('ASLCTRL_DATA', nZ, float).
message_field_type('ASLCTRL_DATA', 'SpoilersEngaged', uint(8)).
message_field_type('ASLCTRL_DATA', p, float).
message_field_type('ASLCTRL_DATA', pRef, float).
message_field_type('ASLCTRL_DATA', r, float).
message_field_type('ASLCTRL_DATA', rRef, float).
message_field_type('ASLCTRL_DATA', uAil, float).
message_field_type('ASLCTRL_DATA', uRud, float).
message_field_type('ASLCTRL_DEBUG', i32_1, uint(32)).
message_field_type('ASLCTRL_DEBUG', i8_1, uint(8)).
message_field_type('ASLCTRL_DEBUG', i8_2, uint(8)).
message_field_type('ASLCTRL_DEBUG', f_1, float).
message_field_type('ASLCTRL_DEBUG', f_2, float).
message_field_type('ASLCTRL_DEBUG', f_3, float).
message_field_type('ASLCTRL_DEBUG', f_4, float).
message_field_type('ASLCTRL_DEBUG', f_5, float).
message_field_type('ASLCTRL_DEBUG', f_6, float).
message_field_type('ASLCTRL_DEBUG', f_7, float).
message_field_type('ASLCTRL_DEBUG', f_8, float).
message_field_type('ASLUAV_STATUS', 'LED_status', uint(8)).
message_field_type('ASLUAV_STATUS', 'SATCOM_status', uint(8)).
message_field_type('ASLUAV_STATUS', 'Servo_status', uint(8)).
message_field_type('ASLUAV_STATUS', 'Motor_rpm', float).
message_field_type('ASL_OBCTRL', uElev, float).
message_field_type('ASL_OBCTRL', uThrot, float).
message_field_type('ASL_OBCTRL', uThrot2, float).
message_field_type('ASL_OBCTRL', uAilL, float).
message_field_type('ASL_OBCTRL', uAilR, float).
message_field_type('ASL_OBCTRL', uRud, float).
message_field_type('ASL_OBCTRL', obctrl_status, uint(8)).
message_field_type('SENS_BATMON', 'SoC', uint(8)).
message_field_type('SENS_BATMON', batterystatus, uint(16)).
message_field_type('SENS_BATMON', serialnumber, uint(16)).
message_field_type('SENS_BATMON', safetystatus, uint(32)).
message_field_type('SENS_BATMON', operationstatus, uint(32)).
message_field_type('FW_SOARING_DATA', 'VarW', float).
message_field_type('FW_SOARING_DATA', 'VarR', float).
message_field_type('FW_SOARING_DATA', 'VarLat', float).
message_field_type('FW_SOARING_DATA', 'VarLon', float).
message_field_type('FW_SOARING_DATA', 'LoiterDirection', float).
message_field_type('FW_SOARING_DATA', z1_exp, float).
message_field_type('FW_SOARING_DATA', z2_exp, float).
message_field_type('FW_SOARING_DATA', 'DebugVar1', float).
message_field_type('FW_SOARING_DATA', 'DebugVar2', float).
message_field_type('FW_SOARING_DATA', 'ControlMode', uint(8)).
message_field_type('FW_SOARING_DATA', valid, uint(8)).
message_field_type('SENSORPOD_STATUS', visensor_rate_1, uint(8)).
message_field_type('SENSORPOD_STATUS', visensor_rate_2, uint(8)).
message_field_type('SENSORPOD_STATUS', visensor_rate_3, uint(8)).
message_field_type('SENSORPOD_STATUS', visensor_rate_4, uint(8)).
message_field_type('SENSORPOD_STATUS', recording_nodes_count, uint(8)).
message_field_type('SENSORPOD_STATUS', free_space, uint(16)).
message_field_type('SENS_POWER_BOARD', pwr_brd_status, uint(8)).
message_field_type('SENS_POWER_BOARD', pwr_brd_led_status, uint(8)).
message_field_type('GSM_LINK_STATUS', rssi, uint(8)).
message_field_type('GSM_LINK_STATUS', rsrp_rscp, uint(8)).
message_field_type('GSM_LINK_STATUS', sinr_ecio, uint(8)).
message_field_type('GSM_LINK_STATUS', rsrq, uint(8)).
message_field_type('SATCOM_LINK_STATUS', failed_sessions, uint(16)).
message_field_type('SATCOM_LINK_STATUS', successful_sessions, uint(16)).
message_field_type('SATCOM_LINK_STATUS', signal_quality, uint(8)).
message_field_type('SATCOM_LINK_STATUS', ring_pending, uint(8)).
message_field_type('SATCOM_LINK_STATUS', tx_session_pending, uint(8)).
message_field_type('SATCOM_LINK_STATUS', rx_session_pending, uint(8)).
message_field_type('SENSOR_AIRFLOW_ANGLES', angleofattack_valid, uint(8)).
message_field_type('SENSOR_AIRFLOW_ANGLES', sideslip_valid, uint(8)).
message_field_type('SYS_STATUS', errors_comm, uint(16)).
message_field_type('SYS_STATUS', errors_count1, uint(16)).
message_field_type('SYS_STATUS', errors_count2, uint(16)).
message_field_type('SYS_STATUS', errors_count3, uint(16)).
message_field_type('SYS_STATUS', errors_count4, uint(16)).
message_field_type('PING', seq, uint(32)).
message_field_type('PING', target_system, uint(8)).
message_field_type('PING', target_component, uint(8)).
message_field_type('CHANGE_OPERATOR_CONTROL', target_system, uint(8)).
message_field_type('CHANGE_OPERATOR_CONTROL', control_request, uint(8)).
message_field_type('CHANGE_OPERATOR_CONTROL', passkey, char).
message_field_type('CHANGE_OPERATOR_CONTROL_ACK', gcs_system_id, uint(8)).
message_field_type('CHANGE_OPERATOR_CONTROL_ACK', control_request, uint(8)).
message_field_type('CHANGE_OPERATOR_CONTROL_ACK', ack, uint(8)).
message_field_type('AUTH_KEY', key, char).
message_field_type('LINK_NODE_STATUS', messages_sent, uint(32)).
message_field_type('LINK_NODE_STATUS', messages_received, uint(32)).
message_field_type('LINK_NODE_STATUS', messages_lost, uint(32)).
message_field_type('SET_MODE', target_system, uint(8)).
message_field_type('SET_MODE', custom_mode, uint(32)).
message_field_type('PARAM_REQUEST_READ', target_system, uint(8)).
message_field_type('PARAM_REQUEST_READ', target_component, uint(8)).
message_field_type('PARAM_REQUEST_READ', param_id, char).
message_field_type('PARAM_REQUEST_LIST', target_system, uint(8)).
message_field_type('PARAM_REQUEST_LIST', target_component, uint(8)).
message_field_type('PARAM_VALUE', param_id, char).
message_field_type('PARAM_VALUE', param_value, float).
message_field_type('PARAM_VALUE', param_count, uint(16)).
message_field_type('PARAM_VALUE', param_index, uint(16)).
message_field_type('PARAM_SET', target_system, uint(8)).
message_field_type('PARAM_SET', target_component, uint(8)).
message_field_type('PARAM_SET', param_id, char).
message_field_type('PARAM_SET', param_value, float).
message_field_type('GPS_STATUS', satellites_visible, uint(8)).
message_field_type('GPS_STATUS', satellite_prn, uint(8)).
message_field_type('GPS_STATUS', satellite_used, uint(8)).
message_field_type('RAW_IMU', xacc, int(16)).
message_field_type('RAW_IMU', yacc, int(16)).
message_field_type('RAW_IMU', zacc, int(16)).
message_field_type('RAW_IMU', xgyro, int(16)).
message_field_type('RAW_IMU', ygyro, int(16)).
message_field_type('RAW_IMU', zgyro, int(16)).
message_field_type('RAW_IMU', xmag, int(16)).
message_field_type('RAW_IMU', ymag, int(16)).
message_field_type('RAW_IMU', zmag, int(16)).
message_field_type('RAW_PRESSURE', press_abs, int(16)).
message_field_type('RAW_PRESSURE', temperature, int(16)).
message_field_type('ATTITUDE_QUATERNION', q1, float).
message_field_type('ATTITUDE_QUATERNION', q2, float).
message_field_type('ATTITUDE_QUATERNION', q3, float).
message_field_type('ATTITUDE_QUATERNION', q4, float).
message_field_type('RC_CHANNELS_SCALED', port, uint(8)).
message_field_type('RC_CHANNELS_RAW', port, uint(8)).
message_field_type('SERVO_OUTPUT_RAW', port, uint(8)).
message_field_type('MISSION_REQUEST_PARTIAL_LIST', target_system, uint(8)).
message_field_type('MISSION_REQUEST_PARTIAL_LIST', target_component, uint(8)).
message_field_type('MISSION_REQUEST_PARTIAL_LIST', start_index, int(16)).
message_field_type('MISSION_REQUEST_PARTIAL_LIST', end_index, int(16)).
message_field_type('MISSION_WRITE_PARTIAL_LIST', target_system, uint(8)).
message_field_type('MISSION_WRITE_PARTIAL_LIST', target_component, uint(8)).
message_field_type('MISSION_WRITE_PARTIAL_LIST', start_index, int(16)).
message_field_type('MISSION_WRITE_PARTIAL_LIST', end_index, int(16)).
message_field_type('MISSION_ITEM', target_system, uint(8)).
message_field_type('MISSION_ITEM', target_component, uint(8)).
message_field_type('MISSION_ITEM', seq, uint(16)).
message_field_type('MISSION_ITEM', current, uint(8)).
message_field_type('MISSION_ITEM', autocontinue, uint(8)).
message_field_type('MISSION_ITEM', param1, float).
message_field_type('MISSION_ITEM', param2, float).
message_field_type('MISSION_ITEM', param3, float).
message_field_type('MISSION_ITEM', param4, float).
message_field_type('MISSION_ITEM', x, float).
message_field_type('MISSION_ITEM', y, float).
message_field_type('MISSION_ITEM', z, float).
message_field_type('MISSION_REQUEST', target_system, uint(8)).
message_field_type('MISSION_REQUEST', target_component, uint(8)).
message_field_type('MISSION_REQUEST', seq, uint(16)).
message_field_type('MISSION_SET_CURRENT', target_system, uint(8)).
message_field_type('MISSION_SET_CURRENT', target_component, uint(8)).
message_field_type('MISSION_SET_CURRENT', seq, uint(16)).
message_field_type('MISSION_CURRENT', seq, uint(16)).
message_field_type('MISSION_REQUEST_LIST', target_system, uint(8)).
message_field_type('MISSION_REQUEST_LIST', target_component, uint(8)).
message_field_type('MISSION_COUNT', target_system, uint(8)).
message_field_type('MISSION_COUNT', target_component, uint(8)).
message_field_type('MISSION_COUNT', count, uint(16)).
message_field_type('MISSION_CLEAR_ALL', target_system, uint(8)).
message_field_type('MISSION_CLEAR_ALL', target_component, uint(8)).
message_field_type('MISSION_ITEM_REACHED', seq, uint(16)).
message_field_type('MISSION_ACK', target_system, uint(8)).
message_field_type('MISSION_ACK', target_component, uint(8)).
message_field_type('SET_GPS_GLOBAL_ORIGIN', target_system, uint(8)).
message_field_type('PARAM_MAP_RC', target_system, uint(8)).
message_field_type('PARAM_MAP_RC', target_component, uint(8)).
message_field_type('PARAM_MAP_RC', param_id, char).
message_field_type('PARAM_MAP_RC', param_index, int(16)).
message_field_type('PARAM_MAP_RC', parameter_rc_channel_index, uint(8)).
message_field_type('PARAM_MAP_RC', param_value0, float).
message_field_type('PARAM_MAP_RC', scale, float).
message_field_type('PARAM_MAP_RC', param_value_min, float).
message_field_type('PARAM_MAP_RC', param_value_max, float).
message_field_type('MISSION_REQUEST_INT', target_system, uint(8)).
message_field_type('MISSION_REQUEST_INT', target_component, uint(8)).
message_field_type('MISSION_REQUEST_INT', seq, uint(16)).
message_field_type('SAFETY_SET_ALLOWED_AREA', target_system, uint(8)).
message_field_type('SAFETY_SET_ALLOWED_AREA', target_component, uint(8)).
message_field_type('ATTITUDE_QUATERNION_COV', q, float).
message_field_type('RC_CHANNELS', chancount, uint(8)).
message_field_type('REQUEST_DATA_STREAM', target_system, uint(8)).
message_field_type('REQUEST_DATA_STREAM', target_component, uint(8)).
message_field_type('REQUEST_DATA_STREAM', req_stream_id, uint(8)).
message_field_type('REQUEST_DATA_STREAM', start_stop, uint(8)).
message_field_type('DATA_STREAM', stream_id, uint(8)).
message_field_type('DATA_STREAM', on_off, uint(8)).
message_field_type('MANUAL_CONTROL', target, uint(8)).
message_field_type('MANUAL_CONTROL', buttons, uint(16)).
message_field_type('MANUAL_CONTROL', buttons2, uint(16)).
message_field_type('MANUAL_CONTROL', enabled_extensions, uint(8)).
message_field_type('MANUAL_CONTROL', s, int(16)).
message_field_type('MANUAL_CONTROL', t, int(16)).
message_field_type('MANUAL_CONTROL', aux1, int(16)).
message_field_type('MANUAL_CONTROL', aux2, int(16)).
message_field_type('MANUAL_CONTROL', aux3, int(16)).
message_field_type('MANUAL_CONTROL', aux4, int(16)).
message_field_type('MANUAL_CONTROL', aux5, int(16)).
message_field_type('MANUAL_CONTROL', aux6, int(16)).
message_field_type('RC_CHANNELS_OVERRIDE', target_system, uint(8)).
message_field_type('RC_CHANNELS_OVERRIDE', target_component, uint(8)).
message_field_type('MISSION_ITEM_INT', target_system, uint(8)).
message_field_type('MISSION_ITEM_INT', target_component, uint(8)).
message_field_type('MISSION_ITEM_INT', seq, uint(16)).
message_field_type('MISSION_ITEM_INT', current, uint(8)).
message_field_type('MISSION_ITEM_INT', autocontinue, uint(8)).
message_field_type('MISSION_ITEM_INT', param1, float).
message_field_type('MISSION_ITEM_INT', param2, float).
message_field_type('MISSION_ITEM_INT', param3, float).
message_field_type('MISSION_ITEM_INT', param4, float).
message_field_type('MISSION_ITEM_INT', x, int(32)).
message_field_type('MISSION_ITEM_INT', y, int(32)).
message_field_type('MISSION_ITEM_INT', z, float).
message_field_type('COMMAND_INT', target_system, uint(8)).
message_field_type('COMMAND_INT', target_component, uint(8)).
message_field_type('COMMAND_INT', current, uint(8)).
message_field_type('COMMAND_INT', autocontinue, uint(8)).
message_field_type('COMMAND_LONG', target_system, uint(8)).
message_field_type('COMMAND_LONG', target_component, uint(8)).
message_field_type('COMMAND_LONG', confirmation, uint(8)).
message_field_type('COMMAND_ACK', result_param2, int(32)).
message_field_type('COMMAND_ACK', target_system, uint(8)).
message_field_type('COMMAND_ACK', target_component, uint(8)).
message_field_type('COMMAND_CANCEL', target_system, uint(8)).
message_field_type('COMMAND_CANCEL', target_component, uint(8)).
message_field_type('MANUAL_SETPOINT', thrust, float).
message_field_type('MANUAL_SETPOINT', mode_switch, uint(8)).
message_field_type('MANUAL_SETPOINT', manual_override_switch, uint(8)).
message_field_type('SET_ATTITUDE_TARGET', target_system, uint(8)).
message_field_type('SET_ATTITUDE_TARGET', target_component, uint(8)).
message_field_type('SET_ATTITUDE_TARGET', q, float).
message_field_type('SET_ATTITUDE_TARGET', thrust, float).
message_field_type('SET_ATTITUDE_TARGET', thrust_body, float).
message_field_type('ATTITUDE_TARGET', q, float).
message_field_type('ATTITUDE_TARGET', thrust, float).
message_field_type('SET_POSITION_TARGET_LOCAL_NED', target_system, uint(8)).
message_field_type('SET_POSITION_TARGET_LOCAL_NED', target_component, uint(8)).
message_field_type('SET_POSITION_TARGET_GLOBAL_INT', target_system, uint(8)).
message_field_type('SET_POSITION_TARGET_GLOBAL_INT', target_component, uint(8)).
message_field_type('HIL_CONTROLS', roll_ailerons, float).
message_field_type('HIL_CONTROLS', pitch_elevator, float).
message_field_type('HIL_CONTROLS', yaw_rudder, float).
message_field_type('HIL_CONTROLS', throttle, float).
message_field_type('HIL_CONTROLS', aux1, float).
message_field_type('HIL_CONTROLS', aux2, float).
message_field_type('HIL_CONTROLS', aux3, float).
message_field_type('HIL_CONTROLS', aux4, float).
message_field_type('HIL_CONTROLS', nav_mode, uint(8)).
message_field_type('HIL_ACTUATOR_CONTROLS', controls, float).
message_field_type('OPTICAL_FLOW', sensor_id, uint(8)).
message_field_type('OPTICAL_FLOW', quality, uint(8)).
message_field_type('GLOBAL_VISION_POSITION_ESTIMATE', reset_counter, uint(8)).
message_field_type('VISION_POSITION_ESTIMATE', reset_counter, uint(8)).
message_field_type('VISION_SPEED_ESTIMATE', reset_counter, uint(8)).
message_field_type('HIGHRES_IMU', pressure_alt, float).
message_field_type('OPTICAL_FLOW_RAD', quality, uint(8)).
message_field_type('HIL_SENSOR', pressure_alt, float).
message_field_type('HIL_SENSOR', id, uint(8)).
message_field_type('SIM_STATE', q1, float).
message_field_type('SIM_STATE', q2, float).
message_field_type('SIM_STATE', q3, float).
message_field_type('SIM_STATE', q4, float).
message_field_type('SIM_STATE', roll, float).
message_field_type('SIM_STATE', pitch, float).
message_field_type('SIM_STATE', yaw, float).
message_field_type('SIM_STATE', std_dev_horz, float).
message_field_type('SIM_STATE', std_dev_vert, float).
message_field_type('RADIO_STATUS', rxerrors, uint(16)).
message_field_type('RADIO_STATUS', fixed, uint(16)).
message_field_type('FILE_TRANSFER_PROTOCOL', target_network, uint(8)).
message_field_type('FILE_TRANSFER_PROTOCOL', target_system, uint(8)).
message_field_type('FILE_TRANSFER_PROTOCOL', target_component, uint(8)).
message_field_type('FILE_TRANSFER_PROTOCOL', payload, uint(8)).
message_field_type('TIMESYNC', target_system, uint(8)).
message_field_type('TIMESYNC', target_component, uint(8)).
message_field_type('CAMERA_TRIGGER', seq, uint(32)).
message_field_type('HIL_GPS', fix_type, uint(8)).
message_field_type('HIL_GPS', id, uint(8)).
message_field_type('HIL_OPTICAL_FLOW', sensor_id, uint(8)).
message_field_type('HIL_OPTICAL_FLOW', quality, uint(8)).
message_field_type('HIL_STATE_QUATERNION', attitude_quaternion, float).
message_field_type('LOG_REQUEST_LIST', target_system, uint(8)).
message_field_type('LOG_REQUEST_LIST', target_component, uint(8)).
message_field_type('LOG_REQUEST_LIST', start, uint(16)).
message_field_type('LOG_REQUEST_LIST', end, uint(16)).
message_field_type('LOG_ENTRY', id, uint(16)).
message_field_type('LOG_ENTRY', num_logs, uint(16)).
message_field_type('LOG_ENTRY', last_log_num, uint(16)).
message_field_type('LOG_REQUEST_DATA', target_system, uint(8)).
message_field_type('LOG_REQUEST_DATA', target_component, uint(8)).
message_field_type('LOG_REQUEST_DATA', id, uint(16)).
message_field_type('LOG_REQUEST_DATA', ofs, uint(32)).
message_field_type('LOG_DATA', id, uint(16)).
message_field_type('LOG_DATA', ofs, uint(32)).
message_field_type('LOG_DATA', data, uint(8)).
message_field_type('LOG_ERASE', target_system, uint(8)).
message_field_type('LOG_ERASE', target_component, uint(8)).
message_field_type('LOG_REQUEST_END', target_system, uint(8)).
message_field_type('LOG_REQUEST_END', target_component, uint(8)).
message_field_type('GPS_INJECT_DATA', target_system, uint(8)).
message_field_type('GPS_INJECT_DATA', target_component, uint(8)).
message_field_type('GPS_INJECT_DATA', data, uint(8)).
message_field_type('GPS2_RAW', dgps_numch, uint(8)).
message_field_type('SERIAL_CONTROL', data, uint(8)).
message_field_type('SERIAL_CONTROL', target_system, uint(8)).
message_field_type('SERIAL_CONTROL', target_component, uint(8)).
message_field_type('GPS_RTK', rtk_receiver_id, uint(8)).
message_field_type('GPS_RTK', wn, uint(16)).
message_field_type('GPS_RTK', rtk_health, uint(8)).
message_field_type('GPS_RTK', nsats, uint(8)).
message_field_type('GPS_RTK', accuracy, uint(32)).
message_field_type('GPS_RTK', iar_num_hypotheses, int(32)).
message_field_type('GPS2_RTK', rtk_receiver_id, uint(8)).
message_field_type('GPS2_RTK', wn, uint(16)).
message_field_type('GPS2_RTK', rtk_health, uint(8)).
message_field_type('GPS2_RTK', nsats, uint(8)).
message_field_type('GPS2_RTK', accuracy, uint(32)).
message_field_type('GPS2_RTK', iar_num_hypotheses, int(32)).
message_field_type('DATA_TRANSMISSION_HANDSHAKE', width, uint(16)).
message_field_type('DATA_TRANSMISSION_HANDSHAKE', height, uint(16)).
message_field_type('DATA_TRANSMISSION_HANDSHAKE', packets, uint(16)).
message_field_type('ENCAPSULATED_DATA', seqnr, uint(16)).
message_field_type('ENCAPSULATED_DATA', data, uint(8)).
message_field_type('TERRAIN_DATA', gridbit, uint(8)).
message_field_type('TERRAIN_REPORT', spacing, uint(16)).
message_field_type('TERRAIN_REPORT', pending, uint(16)).
message_field_type('TERRAIN_REPORT', loaded, uint(16)).
message_field_type('ATT_POS_MOCAP', q, float).
message_field_type('SET_ACTUATOR_CONTROL_TARGET', group_mlx, uint(8)).
message_field_type('SET_ACTUATOR_CONTROL_TARGET', target_system, uint(8)).
message_field_type('SET_ACTUATOR_CONTROL_TARGET', target_component, uint(8)).
message_field_type('SET_ACTUATOR_CONTROL_TARGET', controls, float).
message_field_type('ACTUATOR_CONTROL_TARGET', group_mlx, uint(8)).
message_field_type('ACTUATOR_CONTROL_TARGET', controls, float).
message_field_type('RESOURCE_REQUEST', request_id, uint(8)).
message_field_type('RESOURCE_REQUEST', uri_type, uint(8)).
message_field_type('RESOURCE_REQUEST', uri, uint(8)).
message_field_type('RESOURCE_REQUEST', transfer_type, uint(8)).
message_field_type('RESOURCE_REQUEST', storage, uint(8)).
message_field_type('FOLLOW_TARGET', est_capabilities, uint(8)).
message_field_type('FOLLOW_TARGET', position_cov, float).
message_field_type('FOLLOW_TARGET', custom_state, uint(64)).
message_field_type('CONTROL_SYSTEM_STATE', vel_variance, float).
message_field_type('CONTROL_SYSTEM_STATE', pos_variance, float).
message_field_type('CONTROL_SYSTEM_STATE', q, float).
message_field_type('AUTOPILOT_VERSION', flight_sw_version, uint(32)).
message_field_type('AUTOPILOT_VERSION', middleware_sw_version, uint(32)).
message_field_type('AUTOPILOT_VERSION', os_sw_version, uint(32)).
message_field_type('AUTOPILOT_VERSION', board_version, uint(32)).
message_field_type('AUTOPILOT_VERSION', flight_custom_version, uint(8)).
message_field_type('AUTOPILOT_VERSION', middleware_custom_version, uint(8)).
message_field_type('AUTOPILOT_VERSION', os_custom_version, uint(8)).
message_field_type('AUTOPILOT_VERSION', vendor_id, uint(16)).
message_field_type('AUTOPILOT_VERSION', product_id, uint(16)).
message_field_type('AUTOPILOT_VERSION', uid, uint(64)).
message_field_type('AUTOPILOT_VERSION', uid2, uint(8)).
message_field_type('LANDING_TARGET', target_num, uint(8)).
message_field_type('LANDING_TARGET', q, float).
message_field_type('FENCE_STATUS', breach_status, uint(8)).
message_field_type('FENCE_STATUS', breach_count, uint(16)).
message_field_type('MAG_CAL_REPORT', autosaved, uint(8)).
message_field_type('MAG_CAL_REPORT', ofs_x, float).
message_field_type('MAG_CAL_REPORT', ofs_y, float).
message_field_type('MAG_CAL_REPORT', ofs_z, float).
message_field_type('MAG_CAL_REPORT', diag_x, float).
message_field_type('MAG_CAL_REPORT', diag_y, float).
message_field_type('MAG_CAL_REPORT', diag_z, float).
message_field_type('MAG_CAL_REPORT', offdiag_x, float).
message_field_type('MAG_CAL_REPORT', offdiag_y, float).
message_field_type('MAG_CAL_REPORT', offdiag_z, float).
message_field_type('MAG_CAL_REPORT', orientation_confidence, float).
message_field_type('MAG_CAL_REPORT', scale_factor, float).
message_field_type('EFI_STATUS', health, uint(8)).
message_field_type('EFI_STATUS', ecu_index, float).
message_field_type('EFI_STATUS', rpm, float).
message_field_type('EFI_STATUS', pt_compensation, float).
message_field_type('ESTIMATOR_STATUS', vel_ratio, float).
message_field_type('ESTIMATOR_STATUS', pos_horiz_ratio, float).
message_field_type('ESTIMATOR_STATUS', pos_vert_ratio, float).
message_field_type('ESTIMATOR_STATUS', mag_ratio, float).
message_field_type('ESTIMATOR_STATUS', hagl_ratio, float).
message_field_type('ESTIMATOR_STATUS', tas_ratio, float).
message_field_type('GPS_INPUT', time_week, uint(16)).
message_field_type('GPS_INPUT', fix_type, uint(8)).
message_field_type('GPS_INPUT', satellites_visible, uint(8)).
message_field_type('GPS_RTCM_DATA', flags, uint(8)).
message_field_type('GPS_RTCM_DATA', data, uint(8)).
message_field_type('HIGH_LATENCY', failsafe, uint(8)).
message_field_type('HIGH_LATENCY', wp_num, uint(8)).
message_field_type('HIGH_LATENCY2', wp_num, uint(16)).
message_field_type('HIGH_LATENCY2', custom0, int(8)).
message_field_type('HIGH_LATENCY2', custom1, int(8)).
message_field_type('HIGH_LATENCY2', custom2, int(8)).
message_field_type('VIBRATION', vibration_x, float).
message_field_type('VIBRATION', vibration_y, float).
message_field_type('VIBRATION', vibration_z, float).
message_field_type('VIBRATION', clipping_0, uint(32)).
message_field_type('VIBRATION', clipping_1, uint(32)).
message_field_type('VIBRATION', clipping_2, uint(32)).
message_field_type('SET_HOME_POSITION', target_system, uint(8)).
message_field_type('SET_HOME_POSITION', q, float).
message_field_type('MESSAGE_INTERVAL', message_id, uint(16)).
message_field_type('ADSB_VEHICLE', 'ICAO_address', uint(32)).
message_field_type('ADSB_VEHICLE', callsign, char).
message_field_type('ADSB_VEHICLE', squawk, uint(16)).
message_field_type('COLLISION', id, uint(32)).
message_field_type('V2_EXTENSION', target_network, uint(8)).
message_field_type('V2_EXTENSION', target_system, uint(8)).
message_field_type('V2_EXTENSION', target_component, uint(8)).
message_field_type('V2_EXTENSION', message_type, uint(16)).
message_field_type('V2_EXTENSION', payload, uint(8)).
message_field_type('MEMORY_VECT', address, uint(16)).
message_field_type('MEMORY_VECT', type, uint(8)).
message_field_type('MEMORY_VECT', value, int(8)).
message_field_type('DEBUG_VECT', x, float).
message_field_type('DEBUG_VECT', y, float).
message_field_type('DEBUG_VECT', z, float).
message_field_type('NAMED_VALUE_FLOAT', value, float).
message_field_type('NAMED_VALUE_INT', value, int(32)).
message_field_type('STATUSTEXT', text, char).
message_field_type('STATUSTEXT', id, uint(16)).
message_field_type('STATUSTEXT', chunk_seq, uint(8)).
message_field_type('DEBUG', ind, uint(8)).
message_field_type('DEBUG', value, float).
message_field_type('SETUP_SIGNING', target_system, uint(8)).
message_field_type('SETUP_SIGNING', target_component, uint(8)).
message_field_type('SETUP_SIGNING', secret_key, uint(8)).
message_field_type('SETUP_SIGNING', initial_timestamp, uint(64)).
message_field_type('PLAY_TUNE', target_system, uint(8)).
message_field_type('PLAY_TUNE', target_component, uint(8)).
message_field_type('PLAY_TUNE', tune, char).
message_field_type('PLAY_TUNE', tune2, char).
message_field_type('CAMERA_INFORMATION', vendor_name, uint(8)).
message_field_type('CAMERA_INFORMATION', model_name, uint(8)).
message_field_type('CAMERA_INFORMATION', cam_definition_version, uint(16)).
message_field_type('CAMERA_INFORMATION', cam_definition_uri, char).
message_field_type('STORAGE_INFORMATION', storage_count, uint(8)).
message_field_type('STORAGE_INFORMATION', name, char).
message_field_type('CAMERA_CAPTURE_STATUS', image_status, uint(8)).
message_field_type('CAMERA_CAPTURE_STATUS', video_status, uint(8)).
message_field_type('CAMERA_CAPTURE_STATUS', image_count, int(32)).
message_field_type('CAMERA_IMAGE_CAPTURED', camera_id, uint(8)).
message_field_type('CAMERA_IMAGE_CAPTURED', q, float).
message_field_type('CAMERA_IMAGE_CAPTURED', image_index, int(32)).
message_field_type('CAMERA_IMAGE_CAPTURED', capture_result, int(8)).
message_field_type('CAMERA_IMAGE_CAPTURED', file_url, char).
message_field_type('FLIGHT_INFORMATION', flight_uuid, uint(64)).
message_field_type('LOGGING_DATA', target_system, uint(8)).
message_field_type('LOGGING_DATA', target_component, uint(8)).
message_field_type('LOGGING_DATA', sequence, uint(16)).
message_field_type('LOGGING_DATA', data, uint(8)).
message_field_type('LOGGING_DATA_ACKED', target_system, uint(8)).
message_field_type('LOGGING_DATA_ACKED', target_component, uint(8)).
message_field_type('LOGGING_DATA_ACKED', sequence, uint(16)).
message_field_type('LOGGING_DATA_ACKED', data, uint(8)).
message_field_type('LOGGING_ACK', target_system, uint(8)).
message_field_type('LOGGING_ACK', target_component, uint(8)).
message_field_type('LOGGING_ACK', sequence, uint(16)).
message_field_type('VIDEO_STREAM_INFORMATION', count, uint(8)).
message_field_type('VIDEO_STREAM_INFORMATION', name, char).
message_field_type('VIDEO_STREAM_INFORMATION', uri, char).
message_field_type('CAMERA_FOV_STATUS', q, float).
message_field_type('GIMBAL_MANAGER_STATUS', primary_control_sysid, uint(8)).
message_field_type('GIMBAL_MANAGER_STATUS', primary_control_compid, uint(8)).
message_field_type('GIMBAL_MANAGER_STATUS', secondary_control_sysid, uint(8)).
message_field_type('GIMBAL_MANAGER_STATUS', secondary_control_compid, uint(8)).
message_field_type('GIMBAL_MANAGER_SET_ATTITUDE', target_system, uint(8)).
message_field_type('GIMBAL_MANAGER_SET_ATTITUDE', target_component, uint(8)).
message_field_type('GIMBAL_MANAGER_SET_ATTITUDE', q, float).
message_field_type('GIMBAL_DEVICE_INFORMATION', vendor_name, char).
message_field_type('GIMBAL_DEVICE_INFORMATION', model_name, char).
message_field_type('GIMBAL_DEVICE_INFORMATION', custom_name, char).
message_field_type('GIMBAL_DEVICE_INFORMATION', firmware_version, uint(32)).
message_field_type('GIMBAL_DEVICE_INFORMATION', hardware_version, uint(32)).
message_field_type('GIMBAL_DEVICE_SET_ATTITUDE', target_system, uint(8)).
message_field_type('GIMBAL_DEVICE_SET_ATTITUDE', target_component, uint(8)).
message_field_type('GIMBAL_DEVICE_ATTITUDE_STATUS', target_system, uint(8)).
message_field_type('GIMBAL_DEVICE_ATTITUDE_STATUS', target_component, uint(8)).
message_field_type('GIMBAL_DEVICE_ATTITUDE_STATUS', q, float).
message_field_type('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', target_system, uint(8)).
message_field_type('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', target_component, uint(8)).
message_field_type('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', q, float).
message_field_type('GIMBAL_MANAGER_SET_PITCHYAW', target_system, uint(8)).
message_field_type('GIMBAL_MANAGER_SET_PITCHYAW', target_component, uint(8)).
message_field_type('GIMBAL_MANAGER_SET_MANUAL_CONTROL', target_system, uint(8)).
message_field_type('GIMBAL_MANAGER_SET_MANUAL_CONTROL', target_component, uint(8)).
message_field_type('ESC_INFO', counter, uint(16)).
message_field_type('ESC_INFO', count, uint(8)).
message_field_type('ESC_INFO', error_count, uint(32)).
message_field_type('WIFI_CONFIG_AP', ssid, char).
message_field_type('WIFI_CONFIG_AP', password, char).
message_field_type('AIS_VESSEL', 'MMSI', uint(32)).
message_field_type('AIS_VESSEL', callsign, char).
message_field_type('AIS_VESSEL', name, char).
message_field_type('UAVCAN_NODE_STATUS', sub_mode, uint(8)).
message_field_type('UAVCAN_NODE_STATUS', vendor_specific_status_code, uint(16)).
message_field_type('UAVCAN_NODE_INFO', name, char).
message_field_type('UAVCAN_NODE_INFO', hw_version_major, uint(8)).
message_field_type('UAVCAN_NODE_INFO', hw_version_minor, uint(8)).
message_field_type('UAVCAN_NODE_INFO', hw_unique_id, uint(8)).
message_field_type('UAVCAN_NODE_INFO', sw_version_major, uint(8)).
message_field_type('UAVCAN_NODE_INFO', sw_version_minor, uint(8)).
message_field_type('PARAM_EXT_REQUEST_READ', target_system, uint(8)).
message_field_type('PARAM_EXT_REQUEST_READ', target_component, uint(8)).
message_field_type('PARAM_EXT_REQUEST_READ', param_id, char).
message_field_type('PARAM_EXT_REQUEST_LIST', target_system, uint(8)).
message_field_type('PARAM_EXT_REQUEST_LIST', target_component, uint(8)).
message_field_type('PARAM_EXT_VALUE', param_id, char).
message_field_type('PARAM_EXT_VALUE', param_value, char).
message_field_type('PARAM_EXT_VALUE', param_count, uint(16)).
message_field_type('PARAM_EXT_VALUE', param_index, uint(16)).
message_field_type('PARAM_EXT_SET', target_system, uint(8)).
message_field_type('PARAM_EXT_SET', target_component, uint(8)).
message_field_type('PARAM_EXT_SET', param_id, char).
message_field_type('PARAM_EXT_SET', param_value, char).
message_field_type('PARAM_EXT_ACK', param_id, char).
message_field_type('PARAM_EXT_ACK', param_value, char).
message_field_type('ODOMETRY', q, float).
message_field_type('ODOMETRY', reset_counter, uint(8)).
message_field_type('TRAJECTORY_REPRESENTATION_WAYPOINTS', valid_points, uint(8)).
message_field_type('TRAJECTORY_REPRESENTATION_BEZIER', valid_points, uint(8)).
message_field_type('ISBD_LINK_STATUS', failed_sessions, uint(16)).
message_field_type('ISBD_LINK_STATUS', successful_sessions, uint(16)).
message_field_type('ISBD_LINK_STATUS', signal_quality, uint(8)).
message_field_type('ISBD_LINK_STATUS', ring_pending, uint(8)).
message_field_type('ISBD_LINK_STATUS', tx_session_pending, uint(8)).
message_field_type('ISBD_LINK_STATUS', rx_session_pending, uint(8)).
message_field_type('CELLULAR_CONFIG', enable_lte, uint(8)).
message_field_type('CELLULAR_CONFIG', enable_pin, uint(8)).
message_field_type('CELLULAR_CONFIG', pin, char).
message_field_type('CELLULAR_CONFIG', new_pin, char).
message_field_type('CELLULAR_CONFIG', apn, char).
message_field_type('CELLULAR_CONFIG', puk, char).
message_field_type('CELLULAR_CONFIG', roaming, uint(8)).
message_field_type('RAW_RPM', index, uint(8)).
message_field_type('UTM_GLOBAL_POSITION', uas_id, uint(8)).
message_field_type('DEBUG_FLOAT_ARRAY', name, char).
message_field_type('DEBUG_FLOAT_ARRAY', data, float).
message_field_type('ORBIT_EXECUTION_STATUS', x, int(32)).
message_field_type('ORBIT_EXECUTION_STATUS', y, int(32)).
message_field_type('ACTUATOR_OUTPUT_STATUS', actuator, float).
message_field_type('TUNNEL', target_system, uint(8)).
message_field_type('TUNNEL', target_component, uint(8)).
message_field_type('TUNNEL', payload_length, uint(8)).
message_field_type('TUNNEL', payload, uint(8)).
message_field_type('CAN_FRAME', target_system, uint(8)).
message_field_type('CAN_FRAME', target_component, uint(8)).
message_field_type('CAN_FRAME', bus, uint(8)).
message_field_type('CAN_FRAME', len, uint(8)).
message_field_type('CAN_FRAME', id, uint(32)).
message_field_type('CAN_FRAME', data, uint(8)).
message_field_type('ONBOARD_COMPUTER_STATUS', type, uint(8)).
message_field_type('ONBOARD_COMPUTER_STATUS', link_type, uint(32)).
message_field_type('COMPONENT_INFORMATION', general_metadata_file_crc, uint(32)).
message_field_type('COMPONENT_INFORMATION', general_metadata_uri, char).
message_field_type('COMPONENT_INFORMATION', peripherals_metadata_file_crc, uint(32)).
message_field_type('COMPONENT_INFORMATION', peripherals_metadata_uri, char).
message_field_type('COMPONENT_METADATA', file_crc, uint(32)).
message_field_type('COMPONENT_METADATA', uri, char).
message_field_type('PLAY_TUNE_V2', target_system, uint(8)).
message_field_type('PLAY_TUNE_V2', target_component, uint(8)).
message_field_type('PLAY_TUNE_V2', tune, char).
message_field_type('SUPPORTED_TUNES', target_system, uint(8)).
message_field_type('SUPPORTED_TUNES', target_component, uint(8)).
message_field_type('EVENT', destination_component, uint(8)).
message_field_type('EVENT', destination_system, uint(8)).
message_field_type('EVENT', id, uint(32)).
message_field_type('EVENT', sequence, uint(16)).
message_field_type('EVENT', log_levels, uint(8)).
message_field_type('EVENT', arguments, uint(8)).
message_field_type('CURRENT_EVENT_SEQUENCE', sequence, uint(16)).
message_field_type('REQUEST_EVENT', target_system, uint(8)).
message_field_type('REQUEST_EVENT', target_component, uint(8)).
message_field_type('REQUEST_EVENT', first_sequence, uint(16)).
message_field_type('REQUEST_EVENT', last_sequence, uint(16)).
message_field_type('RESPONSE_EVENT_ERROR', target_system, uint(8)).
message_field_type('RESPONSE_EVENT_ERROR', target_component, uint(8)).
message_field_type('RESPONSE_EVENT_ERROR', sequence, uint(16)).
message_field_type('RESPONSE_EVENT_ERROR', sequence_oldest_available, uint(16)).
message_field_type('CANFD_FRAME', target_system, uint(8)).
message_field_type('CANFD_FRAME', target_component, uint(8)).
message_field_type('CANFD_FRAME', bus, uint(8)).
message_field_type('CANFD_FRAME', len, uint(8)).
message_field_type('CANFD_FRAME', id, uint(32)).
message_field_type('CANFD_FRAME', data, uint(8)).
message_field_type('CAN_FILTER_MODIFY', target_system, uint(8)).
message_field_type('CAN_FILTER_MODIFY', target_component, uint(8)).
message_field_type('CAN_FILTER_MODIFY', bus, uint(8)).
message_field_type('CAN_FILTER_MODIFY', num_ids, uint(8)).
message_field_type('CAN_FILTER_MODIFY', ids, uint(16)).
message_field_type('WHEEL_DISTANCE', count, uint(8)).
message_field_type('OPEN_DRONE_ID_BASIC_ID', target_system, uint(8)).
message_field_type('OPEN_DRONE_ID_BASIC_ID', target_component, uint(8)).
message_field_type('OPEN_DRONE_ID_BASIC_ID', id_or_mac, uint(8)).
message_field_type('OPEN_DRONE_ID_BASIC_ID', uas_id, uint(8)).
message_field_type('OPEN_DRONE_ID_LOCATION', target_system, uint(8)).
message_field_type('OPEN_DRONE_ID_LOCATION', target_component, uint(8)).
message_field_type('OPEN_DRONE_ID_LOCATION', id_or_mac, uint(8)).
message_field_type('OPEN_DRONE_ID_AUTHENTICATION', target_system, uint(8)).
message_field_type('OPEN_DRONE_ID_AUTHENTICATION', target_component, uint(8)).
message_field_type('OPEN_DRONE_ID_AUTHENTICATION', id_or_mac, uint(8)).
message_field_type('OPEN_DRONE_ID_AUTHENTICATION', data_page, uint(8)).
message_field_type('OPEN_DRONE_ID_AUTHENTICATION', last_page_index, uint(8)).
message_field_type('OPEN_DRONE_ID_AUTHENTICATION', authentication_data, uint(8)).
message_field_type('OPEN_DRONE_ID_SELF_ID', target_system, uint(8)).
message_field_type('OPEN_DRONE_ID_SELF_ID', target_component, uint(8)).
message_field_type('OPEN_DRONE_ID_SELF_ID', id_or_mac, uint(8)).
message_field_type('OPEN_DRONE_ID_SELF_ID', description, char).
message_field_type('OPEN_DRONE_ID_SYSTEM', target_system, uint(8)).
message_field_type('OPEN_DRONE_ID_SYSTEM', target_component, uint(8)).
message_field_type('OPEN_DRONE_ID_SYSTEM', id_or_mac, uint(8)).
message_field_type('OPEN_DRONE_ID_SYSTEM', area_count, uint(16)).
message_field_type('OPEN_DRONE_ID_OPERATOR_ID', target_system, uint(8)).
message_field_type('OPEN_DRONE_ID_OPERATOR_ID', target_component, uint(8)).
message_field_type('OPEN_DRONE_ID_OPERATOR_ID', id_or_mac, uint(8)).
message_field_type('OPEN_DRONE_ID_OPERATOR_ID', operator_id, char).
message_field_type('OPEN_DRONE_ID_MESSAGE_PACK', target_system, uint(8)).
message_field_type('OPEN_DRONE_ID_MESSAGE_PACK', target_component, uint(8)).
message_field_type('OPEN_DRONE_ID_MESSAGE_PACK', id_or_mac, uint(8)).
message_field_type('OPEN_DRONE_ID_MESSAGE_PACK', msg_pack_size, uint(8)).
message_field_type('OPEN_DRONE_ID_MESSAGE_PACK', messages, uint(8)).
message_field_type('OPEN_DRONE_ID_ARM_STATUS', error, char).
message_field_type('OPEN_DRONE_ID_SYSTEM_UPDATE', target_system, uint(8)).
message_field_type('OPEN_DRONE_ID_SYSTEM_UPDATE', target_component, uint(8)).
message_field_type('PARAM_ACK_TRANSACTION', target_system, uint(8)).
message_field_type('PARAM_ACK_TRANSACTION', target_component, uint(8)).
message_field_type('PARAM_ACK_TRANSACTION', param_id, char).
message_field_type('PARAM_ACK_TRANSACTION', param_value, float).
message_field_type('WIFI_NETWORK_INFO', ssid, char).
message_field_type('WIFI_NETWORK_INFO', channel_id, uint(8)).
message_field_type('FIGURE_EIGHT_EXECUTION_STATUS', x, int(32)).
message_field_type('FIGURE_EIGHT_EXECUTION_STATUS', y, int(32)).
message_field_type('COMPONENT_INFORMATION_BASIC', vendor_name, char).
message_field_type('COMPONENT_INFORMATION_BASIC', model_name, char).
message_field_type('COMPONENT_INFORMATION_BASIC', software_version, char).
message_field_type('COMPONENT_INFORMATION_BASIC', hardware_version, char).
message_field_type('COMPONENT_INFORMATION_BASIC', serial_number, char).
message_field_type('GROUP_START', group_id, uint(32)).
message_field_type('GROUP_START', mission_checksum, uint(32)).
message_field_type('GROUP_END', group_id, uint(32)).
message_field_type('GROUP_END', mission_checksum, uint(32)).
message_field_type('AVAILABLE_MODES', number_modes, uint(8)).
message_field_type('AVAILABLE_MODES', mode_index, uint(8)).
message_field_type('AVAILABLE_MODES', custom_mode, uint(32)).
message_field_type('AVAILABLE_MODES', mode_name, char).
message_field_type('CURRENT_MODE', custom_mode, uint(32)).
message_field_type('AVAILABLE_MODES_MONITOR', seq, uint(8)).
message_field_type('TARGET_ABSOLUTE', id, uint(8)).
message_field_type('TARGET_RELATIVE', q_target, float).
message_field_type('TARGET_RELATIVE', q_sensor, float).
message_field_type('ICAROUS_KINEMATIC_BANDS', numBands, int(8)).
message_field_type('HEARTBEAT', custom_mode, uint(32)).
message_field_type('HEARTBEAT', mavlink_version, uint(8)).
message_field_type('PROTOCOL_VERSION', version, uint(16)).
message_field_type('PROTOCOL_VERSION', min_version, uint(16)).
message_field_type('PROTOCOL_VERSION', max_version, uint(16)).
message_field_type('PROTOCOL_VERSION', spec_version_hash, uint(8)).
message_field_type('PROTOCOL_VERSION', library_version_hash, uint(8)).
message_field_type('ARRAY_TEST_0', v1, uint(8)).
message_field_type('ARRAY_TEST_0', ar_i8, int(8)).
message_field_type('ARRAY_TEST_0', ar_u8, uint(8)).
message_field_type('ARRAY_TEST_0', ar_u16, uint(16)).
message_field_type('ARRAY_TEST_0', ar_u32, uint(32)).
message_field_type('ARRAY_TEST_1', ar_u32, uint(32)).
message_field_type('ARRAY_TEST_3', v, uint(8)).
message_field_type('ARRAY_TEST_3', ar_u32, uint(32)).
message_field_type('ARRAY_TEST_4', ar_u32, uint(32)).
message_field_type('ARRAY_TEST_4', v, uint(8)).
message_field_type('ARRAY_TEST_5', c1, char).
message_field_type('ARRAY_TEST_5', c2, char).
message_field_type('ARRAY_TEST_6', v1, uint(8)).
message_field_type('ARRAY_TEST_6', v2, uint(16)).
message_field_type('ARRAY_TEST_6', v3, uint(32)).
message_field_type('ARRAY_TEST_6', ar_u32, uint(32)).
message_field_type('ARRAY_TEST_6', ar_i32, int(32)).
message_field_type('ARRAY_TEST_6', ar_u16, uint(16)).
message_field_type('ARRAY_TEST_6', ar_i16, int(16)).
message_field_type('ARRAY_TEST_6', ar_u8, uint(8)).
message_field_type('ARRAY_TEST_6', ar_i8, int(8)).
message_field_type('ARRAY_TEST_6', ar_c, char).
message_field_type('ARRAY_TEST_6', ar_d, double).
message_field_type('ARRAY_TEST_6', ar_f, float).
message_field_type('ARRAY_TEST_7', ar_d, double).
message_field_type('ARRAY_TEST_7', ar_f, float).
message_field_type('ARRAY_TEST_7', ar_u32, uint(32)).
message_field_type('ARRAY_TEST_7', ar_i32, int(32)).
message_field_type('ARRAY_TEST_7', ar_u16, uint(16)).
message_field_type('ARRAY_TEST_7', ar_i16, int(16)).
message_field_type('ARRAY_TEST_7', ar_u8, uint(8)).
message_field_type('ARRAY_TEST_7', ar_i8, int(8)).
message_field_type('ARRAY_TEST_7', ar_c, char).
message_field_type('ARRAY_TEST_8', v3, uint(32)).
message_field_type('ARRAY_TEST_8', ar_d, double).
message_field_type('ARRAY_TEST_8', ar_u16, uint(16)).
message_field_type('TEST_TYPES', c, char).
message_field_type('TEST_TYPES', s, char).
message_field_type('TEST_TYPES', u8, uint(8)).
message_field_type('TEST_TYPES', u16, uint(16)).
message_field_type('TEST_TYPES', u64, uint(64)).
message_field_type('TEST_TYPES', s8, int(8)).
message_field_type('TEST_TYPES', s16, int(16)).
message_field_type('TEST_TYPES', s32, int(32)).
message_field_type('TEST_TYPES', s64, int(64)).
message_field_type('TEST_TYPES', f, float).
message_field_type('TEST_TYPES', d, double).
message_field_type('TEST_TYPES', u8_array, uint(8)).
message_field_type('TEST_TYPES', u16_array, uint(16)).
message_field_type('TEST_TYPES', u32_array, uint(32)).
message_field_type('TEST_TYPES', u64_array, uint(64)).
message_field_type('TEST_TYPES', s8_array, int(8)).
message_field_type('TEST_TYPES', s16_array, int(16)).
message_field_type('TEST_TYPES', s32_array, int(32)).
message_field_type('TEST_TYPES', s64_array, int(64)).
message_field_type('TEST_TYPES', f_array, float).
message_field_type('TEST_TYPES', d_array, double).
message_field_type('NAV_FILTER_BIAS', usec, uint(64)).
message_field_type('NAV_FILTER_BIAS', accel_0, float).
message_field_type('NAV_FILTER_BIAS', accel_1, float).
message_field_type('NAV_FILTER_BIAS', accel_2, float).
message_field_type('NAV_FILTER_BIAS', gyro_0, float).
message_field_type('NAV_FILTER_BIAS', gyro_1, float).
message_field_type('NAV_FILTER_BIAS', gyro_2, float).
message_field_type('RADIO_CALIBRATION', aileron, uint(16)).
message_field_type('RADIO_CALIBRATION', elevator, uint(16)).
message_field_type('RADIO_CALIBRATION', rudder, uint(16)).
message_field_type('RADIO_CALIBRATION', gyro, uint(16)).
message_field_type('RADIO_CALIBRATION', pitch, uint(16)).
message_field_type('RADIO_CALIBRATION', throttle, uint(16)).
message_field_type('UALBERTA_SYS_STATUS', mode, uint(8)).
message_field_type('UALBERTA_SYS_STATUS', nav_mode, uint(8)).
message_field_type('UALBERTA_SYS_STATUS', pilot, uint(8)).
message_field_type('UAVIONIX_ADSB_OUT_CFG', 'ICAO', uint(32)).
message_field_type('UAVIONIX_ADSB_OUT_CFG', callsign, char).
message_field_type('UAVIONIX_ADSB_OUT_DYNAMIC', numSats, uint(8)).
message_field_type('UAVIONIX_ADSB_OUT_DYNAMIC', squawk, uint(16)).
message_field_type('STORM32_GIMBAL_MANAGER_CONTROL', target_system, uint(8)).
message_field_type('STORM32_GIMBAL_MANAGER_CONTROL', target_component, uint(8)).
message_field_type('STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW', target_system, uint(8)).
message_field_type('STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW', target_component, uint(8)).
message_field_type('STORM32_GIMBAL_MANAGER_CORRECT_ROLL', target_system, uint(8)).
message_field_type('STORM32_GIMBAL_MANAGER_CORRECT_ROLL', target_component, uint(8)).
message_field_type('QSHOT_STATUS', shot_state, uint(16)).
message_field_type('RADIO_RC_CHANNELS', count, uint(8)).
message_field_type('RADIO_RC_CHANNELS', channels, int(16)).
message_field_type('FRSKY_PASSTHROUGH_ARRAY', count, uint(8)).
message_field_type('FRSKY_PASSTHROUGH_ARRAY', packet_buf, uint(8)).
message_field_type('PARAM_VALUE_ARRAY', param_count, uint(16)).
message_field_type('PARAM_VALUE_ARRAY', param_index_first, uint(16)).
message_field_type('PARAM_VALUE_ARRAY', param_array_len, uint(8)).
message_field_type('PARAM_VALUE_ARRAY', flags, uint(16)).
message_field_type('PARAM_VALUE_ARRAY', packet_buf, uint(8)).
message_field_type('AVSS_PRS_SYS_STATUS', error_status, uint(32)).
message_field_type('AVSS_PRS_SYS_STATUS', battery_status, uint(32)).
message_field_type('AVSS_PRS_SYS_STATUS', arm_status, uint(8)).
message_field_type('AVSS_PRS_SYS_STATUS', charge_status, uint(8)).
message_field_type('AVSS_DRONE_IMU', q1, float).
message_field_type('AVSS_DRONE_IMU', q2, float).
message_field_type('AVSS_DRONE_IMU', q3, float).
message_field_type('AVSS_DRONE_IMU', q4, float).
message_field_type('AVSS_DRONE_OPERATION_MODE', 'M300_operation_mode', uint(8)).
message_field_type('AVSS_DRONE_OPERATION_MODE', horsefly_operation_mode, uint(8)).
message_field_type('CUBEPILOT_RAW_RC', rc_raw, uint(8)).
message_field_type('HERELINK_VIDEO_STREAM_INFORMATION', camera_id, uint(8)).
message_field_type('HERELINK_VIDEO_STREAM_INFORMATION', status, uint(8)).
message_field_type('HERELINK_VIDEO_STREAM_INFORMATION', uri, char).
message_field_type('HERELINK_TELEM', rssi, uint(8)).
message_field_type('HERELINK_TELEM', snr, int(16)).
message_field_type('HERELINK_TELEM', rf_freq, uint(32)).
message_field_type('HERELINK_TELEM', link_bw, uint(32)).
message_field_type('HERELINK_TELEM', link_rate, uint(32)).
message_field_type('HERELINK_TELEM', cpu_temp, int(16)).
message_field_type('HERELINK_TELEM', board_temp, int(16)).
message_field_type('CUBEPILOT_FIRMWARE_UPDATE_START', target_system, uint(8)).
message_field_type('CUBEPILOT_FIRMWARE_UPDATE_START', target_component, uint(8)).
message_field_type('CUBEPILOT_FIRMWARE_UPDATE_START', crc, uint(32)).
message_field_type('CUBEPILOT_FIRMWARE_UPDATE_RESP', target_system, uint(8)).
message_field_type('CUBEPILOT_FIRMWARE_UPDATE_RESP', target_component, uint(8)).
message_field_type('AIRLINK_AUTH', login, char).
message_field_type('AIRLINK_AUTH', password, char).

:- dynamic message_include/2.

message_include('SENSOR_OFFSETS', ardupilotmega).
message_include('SET_MAG_OFFSETS', ardupilotmega).
message_include('MEMINFO', ardupilotmega).
message_include('AP_ADC', ardupilotmega).
message_include('DIGICAM_CONFIGURE', ardupilotmega).
message_include('DIGICAM_CONTROL', ardupilotmega).
message_include('MOUNT_CONFIGURE', ardupilotmega).
message_include('MOUNT_CONTROL', ardupilotmega).
message_include('MOUNT_STATUS', ardupilotmega).
message_include('FENCE_POINT', ardupilotmega).
message_include('FENCE_FETCH_POINT', ardupilotmega).
message_include('AHRS', ardupilotmega).
message_include('SIMSTATE', ardupilotmega).
message_include('HWSTATUS', ardupilotmega).
message_include('RADIO', ardupilotmega).
message_include('LIMITS_STATUS', ardupilotmega).
message_include('WIND', ardupilotmega).
message_include('DATA16', ardupilotmega).
message_include('DATA32', ardupilotmega).
message_include('DATA64', ardupilotmega).
message_include('DATA96', ardupilotmega).
message_include('RANGEFINDER', ardupilotmega).
message_include('AIRSPEED_AUTOCAL', ardupilotmega).
message_include('RALLY_POINT', ardupilotmega).
message_include('RALLY_FETCH_POINT', ardupilotmega).
message_include('COMPASSMOT_STATUS', ardupilotmega).
message_include('AHRS2', ardupilotmega).
message_include('CAMERA_STATUS', ardupilotmega).
message_include('CAMERA_FEEDBACK', ardupilotmega).
message_include('BATTERY2', ardupilotmega).
message_include('AHRS3', ardupilotmega).
message_include('AUTOPILOT_VERSION_REQUEST', ardupilotmega).
message_include('REMOTE_LOG_DATA_BLOCK', ardupilotmega).
message_include('REMOTE_LOG_BLOCK_STATUS', ardupilotmega).
message_include('LED_CONTROL', ardupilotmega).
message_include('MAG_CAL_PROGRESS', ardupilotmega).
message_include('EKF_STATUS_REPORT', ardupilotmega).
message_include('PID_TUNING', ardupilotmega).
message_include('DEEPSTALL', ardupilotmega).
message_include('GIMBAL_REPORT', ardupilotmega).
message_include('GIMBAL_CONTROL', ardupilotmega).
message_include('GIMBAL_TORQUE_CMD_REPORT', ardupilotmega).
message_include('GOPRO_HEARTBEAT', ardupilotmega).
message_include('GOPRO_GET_REQUEST', ardupilotmega).
message_include('GOPRO_GET_RESPONSE', ardupilotmega).
message_include('GOPRO_SET_REQUEST', ardupilotmega).
message_include('GOPRO_SET_RESPONSE', ardupilotmega).
message_include('RPM', ardupilotmega).
message_include('DEVICE_OP_READ', ardupilotmega).
message_include('DEVICE_OP_READ_REPLY', ardupilotmega).
message_include('DEVICE_OP_WRITE', ardupilotmega).
message_include('DEVICE_OP_WRITE_REPLY', ardupilotmega).
message_include('ADAP_TUNING', ardupilotmega).
message_include('VISION_POSITION_DELTA', ardupilotmega).
message_include('AOA_SSA', ardupilotmega).
message_include('ESC_TELEMETRY_1_TO_4', ardupilotmega).
message_include('ESC_TELEMETRY_5_TO_8', ardupilotmega).
message_include('ESC_TELEMETRY_9_TO_12', ardupilotmega).
message_include('OSD_PARAM_CONFIG', ardupilotmega).
message_include('OSD_PARAM_CONFIG_REPLY', ardupilotmega).
message_include('OSD_PARAM_SHOW_CONFIG', ardupilotmega).
message_include('OSD_PARAM_SHOW_CONFIG_REPLY', ardupilotmega).
message_include('OBSTACLE_DISTANCE_3D', ardupilotmega).
message_include('WATER_DEPTH', ardupilotmega).
message_include('MCU_STATUS', ardupilotmega).
message_include('COMMAND_INT_STAMPED', 'ASLUAV').
message_include('COMMAND_LONG_STAMPED', 'ASLUAV').
message_include('SENS_POWER', 'ASLUAV').
message_include('SENS_MPPT', 'ASLUAV').
message_include('ASLCTRL_DATA', 'ASLUAV').
message_include('ASLCTRL_DEBUG', 'ASLUAV').
message_include('ASLUAV_STATUS', 'ASLUAV').
message_include('EKF_EXT', 'ASLUAV').
message_include('ASL_OBCTRL', 'ASLUAV').
message_include('SENS_ATMOS', 'ASLUAV').
message_include('SENS_BATMON', 'ASLUAV').
message_include('FW_SOARING_DATA', 'ASLUAV').
message_include('SENSORPOD_STATUS', 'ASLUAV').
message_include('SENS_POWER_BOARD', 'ASLUAV').
message_include('GSM_LINK_STATUS', 'ASLUAV').
message_include('SATCOM_LINK_STATUS', 'ASLUAV').
message_include('SENSOR_AIRFLOW_ANGLES', 'ASLUAV').
message_include('SYS_STATUS', common).
message_include('SYSTEM_TIME', common).
message_include('PING', common).
message_include('CHANGE_OPERATOR_CONTROL', common).
message_include('CHANGE_OPERATOR_CONTROL_ACK', common).
message_include('AUTH_KEY', common).
message_include('LINK_NODE_STATUS', common).
message_include('SET_MODE', common).
message_include('PARAM_REQUEST_READ', common).
message_include('PARAM_REQUEST_LIST', common).
message_include('PARAM_VALUE', common).
message_include('PARAM_SET', common).
message_include('GPS_RAW_INT', common).
message_include('GPS_STATUS', common).
message_include('SCALED_IMU', common).
message_include('RAW_IMU', common).
message_include('RAW_PRESSURE', common).
message_include('SCALED_PRESSURE', common).
message_include('ATTITUDE', common).
message_include('ATTITUDE_QUATERNION', common).
message_include('LOCAL_POSITION_NED', common).
message_include('GLOBAL_POSITION_INT', common).
message_include('RC_CHANNELS_SCALED', common).
message_include('RC_CHANNELS_RAW', common).
message_include('SERVO_OUTPUT_RAW', common).
message_include('MISSION_REQUEST_PARTIAL_LIST', common).
message_include('MISSION_WRITE_PARTIAL_LIST', common).
message_include('MISSION_ITEM', common).
message_include('MISSION_REQUEST', common).
message_include('MISSION_SET_CURRENT', common).
message_include('MISSION_CURRENT', common).
message_include('MISSION_REQUEST_LIST', common).
message_include('MISSION_COUNT', common).
message_include('MISSION_CLEAR_ALL', common).
message_include('MISSION_ITEM_REACHED', common).
message_include('MISSION_ACK', common).
message_include('SET_GPS_GLOBAL_ORIGIN', common).
message_include('GPS_GLOBAL_ORIGIN', common).
message_include('PARAM_MAP_RC', common).
message_include('MISSION_REQUEST_INT', common).
message_include('SAFETY_SET_ALLOWED_AREA', common).
message_include('SAFETY_ALLOWED_AREA', common).
message_include('ATTITUDE_QUATERNION_COV', common).
message_include('NAV_CONTROLLER_OUTPUT', common).
message_include('GLOBAL_POSITION_INT_COV', common).
message_include('LOCAL_POSITION_NED_COV', common).
message_include('RC_CHANNELS', common).
message_include('REQUEST_DATA_STREAM', common).
message_include('DATA_STREAM', common).
message_include('MANUAL_CONTROL', common).
message_include('RC_CHANNELS_OVERRIDE', common).
message_include('MISSION_ITEM_INT', common).
message_include('VFR_HUD', common).
message_include('COMMAND_INT', common).
message_include('COMMAND_LONG', common).
message_include('COMMAND_ACK', common).
message_include('COMMAND_CANCEL', common).
message_include('MANUAL_SETPOINT', common).
message_include('SET_ATTITUDE_TARGET', common).
message_include('ATTITUDE_TARGET', common).
message_include('SET_POSITION_TARGET_LOCAL_NED', common).
message_include('POSITION_TARGET_LOCAL_NED', common).
message_include('SET_POSITION_TARGET_GLOBAL_INT', common).
message_include('POSITION_TARGET_GLOBAL_INT', common).
message_include('LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET', common).
message_include('HIL_STATE', common).
message_include('HIL_CONTROLS', common).
message_include('HIL_RC_INPUTS_RAW', common).
message_include('HIL_ACTUATOR_CONTROLS', common).
message_include('OPTICAL_FLOW', common).
message_include('GLOBAL_VISION_POSITION_ESTIMATE', common).
message_include('VISION_POSITION_ESTIMATE', common).
message_include('VISION_SPEED_ESTIMATE', common).
message_include('VICON_POSITION_ESTIMATE', common).
message_include('HIGHRES_IMU', common).
message_include('OPTICAL_FLOW_RAD', common).
message_include('HIL_SENSOR', common).
message_include('SIM_STATE', common).
message_include('RADIO_STATUS', common).
message_include('FILE_TRANSFER_PROTOCOL', common).
message_include('TIMESYNC', common).
message_include('CAMERA_TRIGGER', common).
message_include('HIL_GPS', common).
message_include('HIL_OPTICAL_FLOW', common).
message_include('HIL_STATE_QUATERNION', common).
message_include('SCALED_IMU2', common).
message_include('LOG_REQUEST_LIST', common).
message_include('LOG_ENTRY', common).
message_include('LOG_REQUEST_DATA', common).
message_include('LOG_DATA', common).
message_include('LOG_ERASE', common).
message_include('LOG_REQUEST_END', common).
message_include('GPS_INJECT_DATA', common).
message_include('GPS2_RAW', common).
message_include('POWER_STATUS', common).
message_include('SERIAL_CONTROL', common).
message_include('GPS_RTK', common).
message_include('GPS2_RTK', common).
message_include('SCALED_IMU3', common).
message_include('DATA_TRANSMISSION_HANDSHAKE', common).
message_include('ENCAPSULATED_DATA', common).
message_include('DISTANCE_SENSOR', common).
message_include('TERRAIN_REQUEST', common).
message_include('TERRAIN_DATA', common).
message_include('TERRAIN_CHECK', common).
message_include('TERRAIN_REPORT', common).
message_include('SCALED_PRESSURE2', common).
message_include('ATT_POS_MOCAP', common).
message_include('SET_ACTUATOR_CONTROL_TARGET', common).
message_include('ACTUATOR_CONTROL_TARGET', common).
message_include('ALTITUDE', common).
message_include('RESOURCE_REQUEST', common).
message_include('SCALED_PRESSURE3', common).
message_include('FOLLOW_TARGET', common).
message_include('CONTROL_SYSTEM_STATE', common).
message_include('BATTERY_STATUS', common).
message_include('AUTOPILOT_VERSION', common).
message_include('LANDING_TARGET', common).
message_include('FENCE_STATUS', common).
message_include('MAG_CAL_REPORT', common).
message_include('EFI_STATUS', common).
message_include('ESTIMATOR_STATUS', common).
message_include('WIND_COV', common).
message_include('GPS_INPUT', common).
message_include('GPS_RTCM_DATA', common).
message_include('HIGH_LATENCY', common).
message_include('HIGH_LATENCY2', common).
message_include('VIBRATION', common).
message_include('HOME_POSITION', common).
message_include('SET_HOME_POSITION', common).
message_include('MESSAGE_INTERVAL', common).
message_include('EXTENDED_SYS_STATE', common).
message_include('ADSB_VEHICLE', common).
message_include('COLLISION', common).
message_include('V2_EXTENSION', common).
message_include('MEMORY_VECT', common).
message_include('DEBUG_VECT', common).
message_include('NAMED_VALUE_FLOAT', common).
message_include('NAMED_VALUE_INT', common).
message_include('STATUSTEXT', common).
message_include('DEBUG', common).
message_include('SETUP_SIGNING', common).
message_include('BUTTON_CHANGE', common).
message_include('PLAY_TUNE', common).
message_include('CAMERA_INFORMATION', common).
message_include('CAMERA_SETTINGS', common).
message_include('STORAGE_INFORMATION', common).
message_include('CAMERA_CAPTURE_STATUS', common).
message_include('CAMERA_IMAGE_CAPTURED', common).
message_include('FLIGHT_INFORMATION', common).
message_include('MOUNT_ORIENTATION', common).
message_include('LOGGING_DATA', common).
message_include('LOGGING_DATA_ACKED', common).
message_include('LOGGING_ACK', common).
message_include('VIDEO_STREAM_INFORMATION', common).
message_include('VIDEO_STREAM_STATUS', common).
message_include('CAMERA_FOV_STATUS', common).
message_include('CAMERA_TRACKING_IMAGE_STATUS', common).
message_include('CAMERA_TRACKING_GEO_STATUS', common).
message_include('GIMBAL_MANAGER_INFORMATION', common).
message_include('GIMBAL_MANAGER_STATUS', common).
message_include('GIMBAL_MANAGER_SET_ATTITUDE', common).
message_include('GIMBAL_DEVICE_INFORMATION', common).
message_include('GIMBAL_DEVICE_SET_ATTITUDE', common).
message_include('GIMBAL_DEVICE_ATTITUDE_STATUS', common).
message_include('AUTOPILOT_STATE_FOR_GIMBAL_DEVICE', common).
message_include('GIMBAL_MANAGER_SET_PITCHYAW', common).
message_include('GIMBAL_MANAGER_SET_MANUAL_CONTROL', common).
message_include('ESC_INFO', common).
message_include('ESC_STATUS', common).
message_include('WIFI_CONFIG_AP', common).
message_include('AIS_VESSEL', common).
message_include('UAVCAN_NODE_STATUS', common).
message_include('UAVCAN_NODE_INFO', common).
message_include('PARAM_EXT_REQUEST_READ', common).
message_include('PARAM_EXT_REQUEST_LIST', common).
message_include('PARAM_EXT_VALUE', common).
message_include('PARAM_EXT_SET', common).
message_include('PARAM_EXT_ACK', common).
message_include('OBSTACLE_DISTANCE', common).
message_include('ODOMETRY', common).
message_include('TRAJECTORY_REPRESENTATION_WAYPOINTS', common).
message_include('TRAJECTORY_REPRESENTATION_BEZIER', common).
message_include('CELLULAR_STATUS', common).
message_include('ISBD_LINK_STATUS', common).
message_include('CELLULAR_CONFIG', common).
message_include('RAW_RPM', common).
message_include('UTM_GLOBAL_POSITION', common).
message_include('DEBUG_FLOAT_ARRAY', common).
message_include('ORBIT_EXECUTION_STATUS', common).
message_include('SMART_BATTERY_INFO', common).
message_include('GENERATOR_STATUS', common).
message_include('ACTUATOR_OUTPUT_STATUS', common).
message_include('TIME_ESTIMATE_TO_TARGET', common).
message_include('TUNNEL', common).
message_include('CAN_FRAME', common).
message_include('ONBOARD_COMPUTER_STATUS', common).
message_include('COMPONENT_INFORMATION', common).
message_include('COMPONENT_METADATA', common).
message_include('PLAY_TUNE_V2', common).
message_include('SUPPORTED_TUNES', common).
message_include('EVENT', common).
message_include('CURRENT_EVENT_SEQUENCE', common).
message_include('REQUEST_EVENT', common).
message_include('RESPONSE_EVENT_ERROR', common).
message_include('CANFD_FRAME', common).
message_include('CAN_FILTER_MODIFY', common).
message_include('WHEEL_DISTANCE', common).
message_include('WINCH_STATUS', common).
message_include('OPEN_DRONE_ID_BASIC_ID', common).
message_include('OPEN_DRONE_ID_LOCATION', common).
message_include('OPEN_DRONE_ID_AUTHENTICATION', common).
message_include('OPEN_DRONE_ID_SELF_ID', common).
message_include('OPEN_DRONE_ID_SYSTEM', common).
message_include('OPEN_DRONE_ID_OPERATOR_ID', common).
message_include('OPEN_DRONE_ID_MESSAGE_PACK', common).
message_include('OPEN_DRONE_ID_ARM_STATUS', common).
message_include('OPEN_DRONE_ID_SYSTEM_UPDATE', common).
message_include('HYGROMETER_SENSOR', common).
message_include('PARAM_ACK_TRANSACTION', development).
message_include('AIRSPEED', development).
message_include('WIFI_NETWORK_INFO', development).
message_include('FIGURE_EIGHT_EXECUTION_STATUS', development).
message_include('BATTERY_STATUS_V2', development).
message_include('COMPONENT_INFORMATION_BASIC', development).
message_include('GROUP_START', development).
message_include('GROUP_END', development).
message_include('AVAILABLE_MODES', development).
message_include('CURRENT_MODE', development).
message_include('AVAILABLE_MODES_MONITOR', development).
message_include('TARGET_ABSOLUTE', development).
message_include('TARGET_RELATIVE', development).
message_include('ICAROUS_HEARTBEAT', icarous).
message_include('ICAROUS_KINEMATIC_BANDS', icarous).
message_include('HEARTBEAT', minimal).
message_include('PROTOCOL_VERSION', minimal).
message_include('ARRAY_TEST_0', python_array_test).
message_include('ARRAY_TEST_1', python_array_test).
message_include('ARRAY_TEST_3', python_array_test).
message_include('ARRAY_TEST_4', python_array_test).
message_include('ARRAY_TEST_5', python_array_test).
message_include('ARRAY_TEST_6', python_array_test).
message_include('ARRAY_TEST_7', python_array_test).
message_include('ARRAY_TEST_8', python_array_test).
message_include('TEST_TYPES', test).
message_include('NAV_FILTER_BIAS', ualberta).
message_include('RADIO_CALIBRATION', ualberta).
message_include('UALBERTA_SYS_STATUS', ualberta).
message_include('UAVIONIX_ADSB_OUT_CFG', uAvionix).
message_include('UAVIONIX_ADSB_OUT_DYNAMIC', uAvionix).
message_include('UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT', uAvionix).
message_include('STORM32_GIMBAL_MANAGER_INFORMATION', storm32).
message_include('STORM32_GIMBAL_MANAGER_STATUS', storm32).
message_include('STORM32_GIMBAL_MANAGER_CONTROL', storm32).
message_include('STORM32_GIMBAL_MANAGER_CONTROL_PITCHYAW', storm32).
message_include('STORM32_GIMBAL_MANAGER_CORRECT_ROLL', storm32).
message_include('QSHOT_STATUS', storm32).
message_include('RADIO_RC_CHANNELS', storm32).
message_include('RADIO_LINK_STATS', storm32).
message_include('FRSKY_PASSTHROUGH_ARRAY', storm32).
message_include('PARAM_VALUE_ARRAY', storm32).
message_include('AVSS_PRS_SYS_STATUS', 'AVSSUAS').
message_include('AVSS_DRONE_POSITION', 'AVSSUAS').
message_include('AVSS_DRONE_IMU', 'AVSSUAS').
message_include('AVSS_DRONE_OPERATION_MODE', 'AVSSUAS').
message_include('CUBEPILOT_RAW_RC', cubepilot).
message_include('HERELINK_VIDEO_STREAM_INFORMATION', cubepilot).
message_include('HERELINK_TELEM', cubepilot).
message_include('CUBEPILOT_FIRMWARE_UPDATE_START', cubepilot).
message_include('CUBEPILOT_FIRMWARE_UPDATE_RESP', cubepilot).
message_include('AIRLINK_AUTH', csAirLink).
message_include('AIRLINK_AUTH_RESPONSE', csAirLink).

:- dynamic message_wip/1.

message_wip('OBSTACLE_DISTANCE_3D').
message_wip('LINK_NODE_STATUS').
message_wip('COMMAND_CANCEL').
message_wip('ESC_INFO').
message_wip('ESC_STATUS').
message_wip('ORBIT_EXECUTION_STATUS').
message_wip('TIME_ESTIMATE_TO_TARGET').
message_wip('ONBOARD_COMPUTER_STATUS').
message_wip('COMPONENT_METADATA').
message_wip('EVENT').
message_wip('CURRENT_EVENT_SEQUENCE').
message_wip('REQUEST_EVENT').
message_wip('RESPONSE_EVENT_ERROR').
message_wip('FIGURE_EIGHT_EXECUTION_STATUS').
message_wip('PROTOCOL_VERSION').
message_wip('QSHOT_STATUS').
