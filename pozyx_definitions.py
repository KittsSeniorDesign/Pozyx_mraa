_POZYX_LIB_VERSION = 1
_POZYX_FEET_PER_METER = 3.2808399
_POZYX_INCH_PER_METER = 39.3700787
MAX_ANCHORS_IN_LIST = 20
POZYX_I2C_ADDRESS = 0x4B
POZYX_WHO_AM_I = 0x0
POZYX_FIRMWARE_VER = 0x1
POZYX_HARDWARE_VER = 0x2
POZYX_ST_RESULT = 0x3
POZYX_ERRORCODE = 0x4
POZYX_INT_STATUS = 0x5
POZYX_CALIB_STATUS = 0x6
POZYX_INT_MASK = 0x10
POZYX_INT_CONFIG = 0x11
POZYX_CONFIG_LEDS = 0x15
POZYX_POS_ALG = 0x16
POZYX_POS_NUM_ANCHORS = 0x17
POZYX_POS_INTERVAL = 0x18
POZYX_NETWORK_ID = 0x1A
POZYX_UWB_CHANNEL = 0x1C
POZYX_UWB_RATES = 0x1D
POZYX_UWB_PLEN = 0x1E
POZYX_UWB_GAIN = 0x1F
POZYX_UWB_XTALTRIM = 0x20
POZYX_OPERATION_MODE = 0x22
POZYX_SENSORS_MODE = 0x23
POZYX_CONFIG_GPIO1 = 0x27
POZYX_CONFIG_GPIO2 = 0x28
POZYX_CONFIG_GPIO3 = 0x29
POZYX_CONFIG_GPIO4 = 0x2A
POZYX_POS_X = 0x30
POZYX_POS_Y = 0x34
POZYX_POS_Z = 0x38
POZYX_POS_ERR_X = 0x3C
POZYX_POS_ERR_Y = 0x3E
POZYX_POS_ERR_Z = 0x40
POZYX_POS_ERR_XY = 0x42
POZYX_POS_ERR_XZ = 0x44
POZYX_POS_ERR_YZ = 0x46
POZYX_PRESSURE = 0x50
POZYX_ACCEL_X = 0x54
POZYX_ACCEL_Y = 0x56
POZYX_ACCEL_Z = 0x58
POZYX_MAGN_X = 0x5A
POZYX_MAGN_Y = 0x5C
POZYX_MAGN_Z = 0x5E
POZYX_GYRO_X = 0x60
POZYX_GYRO_Y = 0x62
POZYX_GYRO_Z = 0x64
POZYX_EUL_HEADING = 0x66
POZYX_EUL_ROLL = 0x68
POZYX_EUL_PITCH = 0x6A
POZYX_QUAT_W = 0x6C
POZYX_QUAT_X = 0x6E
POZYX_QUAT_Y = 0x70
POZYX_QUAT_Z = 0x72
POZYX_LIA_X = 0x74
POZYX_LIA_Y = 0x76
POZYX_LIA_Z = 0x78
POZYX_GRAV_X = 0x7A
POZYX_GRAV_Y = 0x7C
POZYX_GRAV_Z = 0x7E
POZYX_TEMPERATURE = 0x80
POZYX_DEVICE_LIST_SIZE = 0x81
POZYX_RX_NETWORK_ID = 0x82
POZYX_RX_DATA_LEN = 0x84
POZYX_GPIO1 = 0x85
POZYX_GPIO2 = 0x86
POZYX_GPIO3 = 0x87
POZYX_GPIO4 = 0x88
POZYX_RESET_SYS = 0xB0
POZYX_LED_CTRL = 0xB1
POZYX_TX_DATA = 0xB2
POZYX_TX_SEND = 0xB3
POZYX_RX_DATA = 0xB4
POZYX_DO_RANGING = 0xB5
POZYX_DO_POSITIONING = 0xB6
POZYX_POS_SET_ANCHOR_IDS = 0xB7
POZYX_POS_GET_ANCHOR_IDS = 0xB8
POZYX_FLASH_RESET = 0xB9
POZYX_FLASH_SAVE = 0xBA
POZYX_FLASH_DETAILS = 0xBB
POZYX_DEVICES_GETIDS = 0xC0
POZYX_DEVICES_DISCOVER = 0xC1
POZYX_DEVICES_CALIBRATE = 0xC2
POZYX_DEVICES_CLEAR = 0xC3
POZYX_DEVICE_ADD = 0xC4
POZYX_DEVICE_GETINFO = 0xC5
POZYX_DEVICE_GETCOORDS = 0xC6
POZYX_DEVICE_GETRANGEINFO = 0xC7
POZYX_CIR_DATA = 0xC8
POZYX_ST_RESULT_ACC = 0x01
POZYX_ST_RESULT_MAGN = 0x02
POZYX_ST_RESULT_GYR = 0x04
POZYX_ST_RESULT_MCU = 0x08
POZYX_ST_RESULT_PRES = 0x10
POZYX_ST_RESULT_UWB = 0x20
POZYX_INT_STATUS_ERR = 0x01
POZYX_INT_STATUS_POS = 0x02
POZYX_INT_STATUS_IMU = 0x04
POZYX_INT_STATUS_RX_DATA = 0x08
POZYX_INT_STATUS_FUNC = 0x10
POZYX_INT_MASK_ERR = 0x01
POZYX_INT_MASK_POS = 0x02
POZYX_INT_MASK_IMU = 0x04
POZYX_INT_MASK_RX_DATA = 0x08
POZYX_INT_MASK_FUNC = 0x10
POZYX_INT_MASK_TDMA = 0x40
POZYX_INT_MASK_PIN = 0x80
POZYX_POS_ALG_UWB_ONLY = 0x00
POZYX_POS_ALG_TRACKING = 0x01
POZYX_POS_ALG_LS = 0x02
POZYX_RANGE_PROTOCOL_SDS_TWR = 0x00
POZYX_RANGE_PROTOCOL_TWR = 0x01
POZYX_RANGE_PROTOCOL_TEST = 0x02
POZYX_LED_CTRL_LED1 = 0x01
POZYX_LED_CTRL_LED2 = 0x02
POZYX_LED_CTRL_LED3 = 0x04
POZYX_LED_CTRL_LED4 = 0x08
POZYX_TYPE = 0xE0
POZYX_ANCHOR = 0x00
POZYX_TAG = 0x20
MAX_BUF_SIZE = 100
POZYX_INT_MASK_ALL = 0x1F
POZYX_DELAY_LOCAL_WRITE = 1
POZYX_DELAY_LOCAL_FUNCTION = 5
POZYX_DELAY_REMOTE_WRITE = 5
POZYX_DELAY_REMOTE_FUNCTION = 10
POZYX_DELAY_INTERRUPT = 100
POZYX_DELAY_CALIBRATION = 1000
POZYX_FAILURE = 0x0
POZYX_SUCCESS = 0x1
POZYX_TIMEOUT = 0x8
POZYX_3D = 3
POZYX_2D = 2
POZYX_2_5D = 1
POZYX_INT_PIN0 = 0x0
POZYX_INT_PIN1 = 0x1
POZYX_LED_CTRL_LEDRX = 0x10
POZYX_LED_CTRL_LEDTX = 0x20
POZYX_ANCHOR_MODE = 0
POZYX_TAG_MODE = 1
POZYX_GPIO_DIGITAL_INPUT = 0
POZYX_GPIO_PUSHPULL = 1
POZYX_GPIO_OPENDRAIN = 1
POZYX_GPIO_NOPULL = 0
POZYX_GPIO_PULLUP = 1
POZYX_GPIO_PULLDOWN = 2
POZYX_ANCHOR_SEL_MANUAL = 0
POZYX_ANCHOR_SEL_AUTO = 1
POZYX_DISCOVERY_ANCHORS_ONLY = 0
POZYX_DISCOVERY_TAGS_ONLY = 1
POZYX_DISCOVERY_ALL_DEVICES = 2
MODE_POLLING = 0
MODE_INTERRUPT = 1
POZYX_POS_DIV_MM = 1.0
POZYX_PRESS_DIV_PA = 1000.0
POZYX_ACCEL_DIV_MG = 1.0
POZYX_GYRO_DIV_DPS = 16.0
POZYX_MAG_DIV_UT = 16.0
POZYX_EULER_DIV_DEG = 16.0
POZYX_QUAT_DIV = 16384.0
POZYX_TEMP_DIV_CELSIUS = 1.0
POZYX_ERROR_NONE = 0x00
POZYX_ERROR_I2C_WRITE = 0x01
POZYX_ERROR_I2C_CMDFULL = 0x02
POZYX_ERROR_ANCHOR_ADD = 0x03
POZYX_ERROR_COMM_QUEUE_FULL = 0x04
POZYX_ERROR_I2C_READ = 0x05
POZYX_ERROR_UWB_CONFIG = 0x06
POZYX_ERROR_OPERATION_QUEUE_FULL = 0x07
POZYX_ERROR_TDMA = 0xA0
POZYX_ERROR_STARTUP_BUSFAULT = 0x08
POZYX_ERROR_FLASH_INVALID = 0x09
POZYX_ERROR_NOT_ENOUGH_ANCHORS = 0x0A
POZYX_ERROR_DISCOVERY = 0X0B
POZYX_ERROR_CALIBRATION = 0x0C
POZYX_ERROR_FUNC_PARAM = 0x0D
POZYX_ERROR_ANCHOR_NOT_FOUND = 0x0E
POZYX_ERROR_GENERAL = 0xFF
POZYX_FLASH_REGS = 1
POZYX_FLASH_ANCHOR_IDS = 2
POZYX_FLASH_NETWORK = 3
PIN_MODE_PUSHPULL = 0
PIN_ACTIVE_LOW = 0
PIN_ACTIVE_HIGH = 1
