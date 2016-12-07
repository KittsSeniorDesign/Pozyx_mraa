#https://pgi-jcns.fz-juelich.de/portal/pages/using-c-from-python.html
# to make so files https://docs.python.org/3/extending/building.html
from ctypes import *
from pozyx_definitions import *
import os

'''
/** 
* The UWB settings type defines all attributes needed to set the UWB (communication) parameters
*/
'''
class UWB_settings_t(Structure):
	'''
	/** The UWB channel number. Possible values are 1, 2, 3, 4, 5, 7. See the reg:POZYX_UWB_CHANNEL register for more information. */
    uint8_t channel;  
    /** The bitrate. Possible values are 
    *
    * - 0: 110kbits/s
    * - 1: 850kbits/s
    * - 2: 6.8Mbits/s. 
    *
    * See the reg:POZYX_UWB_RATES register for more information */              
    uint8_t bitrate; 
    /** The UWB pulse repetition frequency (PRF). Possible values are 
    * 
    * - 1: 16MHz
    * - 2: 64MHz 
    *
    * See the reg:POZYX_UWB_RATES register for more information */                    
    uint8_t prf;                 
    /** The preabmle length. Possible values are:
    *
    * - 0x0C : 4096 symbols.
    * - 0x28 : 2048 symbols. 
    * - 0x18 : 1536 symbols. 
    * - 0x08 : 1024 symbols.
    * - 0x34 : 512 symbols. 
    * - 0x24 : 256 symbols. 
    * - 0x14 : 128 symbols. 
    * - 0x04 : 64 symbols.  
    *
    * See the reg:POZYX_UWB_PLEN register for more information.
    */ 
    uint8_t plen;                   
    /** The transmission gain in dB. Possible values are between 0dB and 33.5dB, with a resolution of 0.5dB. See the reg:POZYX_UWB_GAIN register for more information.*/
    float gain_db;             
	'''
	_fields_ = [
		('channel', c_uint8),
		('bitrate', c_uint8),
		('prf', c_uint8),
		('plen', c_uint8),
		('gain_db', c_float)
	]

'''
/**
* The coordinates type defines the coordinates of position result or anchor location
*/
'''
class coordinates_t(Structure):
	'''
	/** The x-coordinate in mm */
    int32_t x;                      
    /** The y-coordinate in mm */
    int32_t y;                      
    /** The z-coordinate in mm */
    int32_t z;
	'''
	_pack_ = 1
	_fields_ = [ 
		('x', c_int32),
		('y', c_int32),
		('z', c_int32)
	]

'''
/** 
 * A structure representing a 3D vector with floating points. 
 * This type is used to represent most of the sensor values that have components in 3 dimensions.
 */
'''
class v3D_float32_t(Structure):
	'''
	float32 is the same as float. If you look at Pozyx.h on line 48, 
	you'll see that float is typedefed to float32
	/** The x-coordinate of the vector */
    float32_t x;                      
    /** The y-coordinate of the vector */
    float32_t y;                      
    /** The z-coordinate of the vector */
    float32_t z;  
	'''
	_pack_ = 1
	_fields_ = [
		('x', c_float),
		('y', c_float),
		('z', c_float)
	]
# some supporting types for specific sensors
acceleration_t = v3D_float32_t
magnetic_t = v3D_float32_t
angular_vel_t = v3D_float32_t
linear_acceleration_t = v3D_float32_t
gravity_vector_t = v3D_float32_t

'''
/**
* The position error type gives the resulting error covariance for a given position result
*/
'''
class pos_error_t(Structure):
	'''
	/** The variance in the x-coordinate */
    int16_t x;
    /** The variance in the y-coordinate */
    int16_t y;
    /** The variance in the z-coordinate */
    int16_t z;
    /** The covariance of xy */
    int16_t xy;
    /** The covariance of xz */
    int16_t xz;
    /** The covariance of yz */
    int16_t yz;
	'''
	_pack_ = 1
	_fields_ = [
		('x', c_int16),
		('y', c_int16),
		('z', c_int16),
		('xy', c_int16),
		('xz', c_int16),
		('yz', c_int16)
	]

'''
/**
* The euler angles type holds the absolute orientation of the pozyx device using the Euler angles (yaw, pitch, roll) representation
*/
'''
class euler_angles_t(Structure):
	'''
	float32 is the same as float. If you look at Pozyx.h on line 48, 
	you'll see that float is typedefed to float32
	/** The heading (yaw) in degrees. */
    float32_t heading;
    /** The roll in degrees. */
    float32_t roll;
    /** The pitch in degrees. */
    float32_t pitch;
	'''
	_pack_ = 1
	_fields_ = [
		('heading', c_float),
		('roll', c_float),
		('pitch', c_float)
	]

'''
/**
* The quaternion_t type holds the absolute orientation of the pozyx device using the a quaternion representation
*/
'''
class quaternion_t(Structure):
	'''
	float32 is the same as float. If you look at Pozyx.h on line 48, 
	you'll see that float is typedefed to float32
	/** weight of the quaterion. */
    float32_t weight;
    /** x-coordinate of the quaterion. */
    float32_t x;
    /** y-coordinate of the quaterion. */
    float32_t y;
    /** z-coordinate of the quaterion. */
    float32_t z;
	'''
	_pack_ = 1
	_fields_ = [
		('weight', c_float),
		('x', c_float),
		('y', c_float),
		('z', c_float)
	]

'''
/**
* raw sensor data. This follows the ordering of the pozyx registers
*/
'''
class sensor_raw_t(Structure):
	_pack_ = 1
	_fields_ = [
		('pressure', c_uint32),
		('acceleration', (c_int16 * 3)),
		('magnetic', (c_int16 * 3)),
		('angular_vel', (c_int16 * 3)),
		('euler_angles', (c_int16 * 3)),
		('quaternion', (c_int16 * 4)),
		('linear_acceleration', (c_int16 * 3)),
		('gravity_vector', (c_int16 * 3)),
		('temperature', c_uint8)
	]

'''
/**
* The sensor data type allows to read the whole sensor data in one datastructure with one call
*/
'''
class sensor_data_t(Structure):
	_pack_ = 1
	_fields_ = [
		('pressure', c_float),
		('acceleration', acceleration_t),
		('magnetic', magnetic_t),
		('angular_vel', angular_vel_t),
		('euler_angles', euler_angles_t),
		('quaternion', quaternion_t),
		('linear_acceleration', linear_acceleration_t),
		('gravity_vector', gravity_vector_t),
		('temperature', c_float)
	]

'''
/**
* The device_coordinates_t type is used to describe a pozyx device required for the device list
*/
'''
class device_coordinates_t(Structure):
	'''
	/** the unique 16-bit network id (by default this is the same as on the label of the device) */
    uint16_t network_id;
    /** a flag indicating some aspects of the device such as anchor or tag. 
     * Possible values are:
     *
     * - 1 : anchor
     * - 2 : tag
     */
    uint8_t flag;
    /** The coordinates of the device */
    coordinates_t pos;
	'''
	_pack_ = 1
	_fields_ = [
		('network_id', c_uint16),
		('flag', c_uint8),
		('pos', coordinates_t)
	]

'''
/**
* The device range type stores all the attributes linked to a range measurement
*/
'''
class device_range_t(Structure):
	'''
	/** The timestamp in ms of the range measurement. */ 
    uint32_t timestamp;
    /** The distance in mm. */
    uint32_t distance;
    /** The received signal strength in dBm. */
    int16_t RSS;
	'''
	_pack_ = 1
	_fields_ = [
		('timestamp', c_uint32),
		('distance', c_uint32),
		('RSS', c_int16)
	]

_pozyx = CDLL(os.path.abspath('Pozyx.so'))
_pozyx.waitForFlag_safe.argtypes = [c_uint8, c_int, POINTER(c_uint8)]
_pozyx.waitForFlag_safe.restype = c_bool
_pozyx.waitForFlag.argtypes = [c_uint8, c_int, POINTER(c_uint8)]
_pozyx.waitForFlag.restype = c_bool
_pozyx.begin.argtypes = [c_bool, c_int, c_int, c_int]
_pozyx.regRead.argtypes = [c_uint8, POINTER(c_uint8), c_int]
_pozyx.regFunction.argtypes = [c_uint8, POINTER(c_uint8)]
_pozyx.remoteRegWrite.argtypes = [c_uint16, c_uint8, POINTER(c_uint8), c_int]
_pozyx.remoteRegRead.argtypes = [c_uint16, c_uint8, POINTER(c_uint8), c_int]
_pozyx.remoteRegFunction.argtypes = [c_uint16, c_uint8, POINTER(c_uint8), c_int, POINTER(c_uint8), c_int]
_pozyx.sendData.argtypes = [c_uint16, POINTER(c_uint8), c_int]
_pozyx.writeTXBufferData.argtypes = [POINTER(c_uint8), c_int, c_int]
_pozyx.sendTXBufferData.argtypes = [c_uint16]
_pozyx.readRXBufferData.argtypes = [POINTER(c_uint8), c_int]
_pozyx.getLastNetworkId.argtypes = [POINTER(c_uint16), c_uint16]
_pozyx.getLastDataLength.argtypes = [POINTER(c_uint8), c_uint16]
_pozyx.getNetworkId.argtypes = [POINTER(c_uint16)]
_pozyx.setNetworkId.argtypes = [c_uint16, c_uint16]
_pozyx.getUWBSettings.argtypes = [POINTER(UWB_settings_t), c_uint16]
_pozyx.setUWBSettings.argtypes = [POINTER(UWB_settings_t), c_uint16]
_pozyx.setUWBChannel.argtypes = [c_int, c_uint16]
_pozyx.getUWBChannel.argtypes = [POINTER(c_int), c_uint16]
_pozyx.setTxPower.argtypes = [c_float, c_uint16]
_pozyx.getTxPower.argtypes = [POINTER(c_float), c_uint16]
_pozyx.getWhoAmI.argtypes = [POINTER(c_uint8), c_uint16]
_pozyx.getFirmwareVersion.argtypes = [POINTER(c_uint8), c_uint16]
_pozyx.getHardwareVersion.argtypes = [POINTER(c_uint8), c_uint16]
_pozyx.getSelftest.argtypes = [POINTER(c_uint8), c_uint16]
_pozyx.getErrorCode.argtypes = [POINTER(c_uint8), c_uint16]
_pozyx.getInterruptStatus.argtypes = [POINTER(c_uint8), c_uint16]
_pozyx.getCalibrationStatus.argtypes = [POINTER(c_uint8), c_uint16]
_pozyx.getGPIO.argtypes = [c_int, POINTER(c_uint8), c_uint16]
_pozyx.setGPIO.argtypes = [c_int, c_uint8, c_uint16]
_pozyx.resetSystem.argtypes = [c_uint16]
_pozyx.resetSystem.restype = None
_pozyx.setLed.argtypes = [c_int, c_bool, c_uint16]
_pozyx.getInterruptMask.argtypes = [POINTER(c_uint8), c_uint16]
_pozyx.setInterruptMask.argtypes = [c_uint8, c_uint16]
_pozyx.getConfigModeGPIO.argtypes = [c_int, POINTER(c_uint8), c_uint16]
_pozyx.getConfigPullGPIO.argtypes = [c_int, POINTER(c_uint8), c_uint16]
_pozyx.setConfigGPIO.argtypes = [c_int, c_int, c_int, c_uint16]
_pozyx.setLedConfig.argtypes = [c_uint8, c_uint16]
_pozyx.configInterruptPin.argtypes = [c_int, c_int, c_int, c_int, c_uint16]
_pozyx.saveConfiguration.argtypes = [c_int, POINTER(c_uint8), c_int, c_uint16]
_pozyx.clearConfiguration.argtypes = [c_uint16]
_pozyx.isRegisterSaved.argtypes = [c_uint8, c_uint16]
_pozyx.isRegisterSaved.restype = c_bool
_pozyx.getNumRegistersSaved.argtypes = [c_uint16]
_pozyx.getCoordinates.argtypes = [POINTER(coordinates_t), c_uint16]
_pozyx.setCoordinates.argtypes = [coordinates_t, c_uint16]
_pozyx.getPositionError.argtypes = [POINTER(pos_error_t), c_uint16]
_pozyx.setPositioningAnchorIds.argtypes = [POINTER(c_uint16), c_int, c_uint16]
_pozyx.getPositioningAnchorIds.argtypes = [POINTER(c_uint16), c_int, c_uint16]
_pozyx.getUpdateInterval.argtypes = [POINTER(c_uint16), c_uint16]
_pozyx.setUpdateInterval.argtypes = [c_uint16, c_uint16]
_pozyx.getPositionAlgorithm.argtypes = [POINTER(c_uint8), c_uint16]
_pozyx.getPositionDimension.argtypes = [POINTER(c_uint8), c_uint16]
_pozyx.setPositionAlgorithm.argtypes = [c_int, c_int, c_uint16]
_pozyx.getAnchorSelectionMode.argtypes = [POINTER(c_uint8), c_uint16]
_pozyx.getNumberOfAnchors.argtypes = [POINTER(c_uint8), c_uint16]
_pozyx.setSelectionOfAnchors.argtypes = [c_int, c_int, c_uint16]
_pozyx.getOperationMode.argtypes = [POINTER(c_uint8), c_uint16]
_pozyx.setOperationMode.argtypes = [c_uint8, c_uint16]
_pozyx.getSystemError.argtypes = [c_uint16]
_pozyx.getSystemError.restype = c_char_p
_pozyx.getSensorMode.argtypes = [POINTER(c_uint8), c_uint16]
_pozyx.setSensorMode.argtypes = [c_uint8, c_uint16]
_pozyx.getAllSensorData.argtypes = [POINTER(sensor_data_t), c_uint16]
_pozyx.getPressure_Pa.argtypes = [POINTER(c_float), c_uint16]
_pozyx.getAcceleration_mg.argtypes = [POINTER(acceleration_t), c_uint16]
_pozyx.getMagnetic_uT.argtypes = [POINTER(magnetic_t), c_uint16]
_pozyx.getAngularVelocity_dps.argtypes = [POINTER(angular_vel_t), c_uint16]
_pozyx.getEulerAngles_deg.argtypes = [POINTER(euler_angles_t), c_uint16]
_pozyx.getQuaternion.argtypes = [POINTER(quaternion_t), c_uint16]
_pozyx.getLinearAcceleration_mg.argtypes = [POINTER(linear_acceleration_t), c_uint16]
_pozyx.getGravityVector_mg.argtypes = [POINTER(gravity_vector_t), c_uint16]
_pozyx.getTemperature_c.argtypes = [POINTER(c_float), c_uint16]
_pozyx.doPositioning.argtypes = [POINTER(coordinates_t), c_uint8, c_int32, c_uint8]
_pozyx.doRemotePositioning.argtypes = [c_uint16, POINTER(coordinates_t), c_uint8, c_int32, c_uint8]
_pozyx.doRanging.argtypes = [c_uint16, POINTER(device_range_t)]
_pozyx.doRemoteRanging.argtypes = [c_uint16, c_uint16, POINTER(device_range_t)]
_pozyx.getDeviceRangeInfo.argtypes = [c_uint16, POINTER(device_range_t), c_uint16]
_pozyx.getDeviceListSize.argtypes = [POINTER(c_uint8), c_uint16]
_pozyx.getDeviceIds.argtypes = [POINTER(c_uint16), c_int, c_uint16]
_pozyx.getAnchorIds.argtypes = [POINTER(c_uint16), c_int, c_uint16]
_pozyx.getTagIds.argtypes = [POINTER(c_uint16), c_int, c_uint16]
_pozyx.doDiscovery.argtypes = [c_int, c_int, c_int]
_pozyx.doAnchorCalibration.argtypes = [c_int, c_int, c_int, POINTER(c_uint16), POINTER(c_int32)]
_pozyx.clearDevices.argtypes = [c_uint16]
_pozyx.addDevice.argtypes = [device_coordinates_t, c_uint16]
_pozyx.getDeviceCoordinates = [c_uint16, POINTER(coordinates_t), c_uint16]


# does the same thing as waitForFlag, but is 'safe'
# I would be more descriptive, but there is no documentation on it. Probably means thread safe
def waitForFlag_safe(interrupt_flag, timeout_ms, interrupt = None):
	global _pozyx
	if interrupt:
		i = pointer(c_uint8(interrupt))
	else:
		i = POINTER(c_uint8)()
	return _pozyx.waitForFlag_safe(c_uint8(interrupt_flag), c_int(timeout_ms), i)

''' 
/**    
* Wait until the Pozyx shields has raised a specfic event flag or until timeout.
* This functions halts the process flow until a specific event flag was raised by the Pozyx
* shield. The event flag is checked by reading the contents of the reg:POZYX_INT_STATUS register.
* This function can work in both polled and interupt mode
* 
*   @param interrupt_flag the exepected Pozyx interrupt. Possible values are #POZYX_INT_STATUS_ERR, 
*   #POZYX_INT_STATUS_POS, #POZYX_INT_STATUS_IMU, #POZYX_INT_STATUS_RX_DATA, #POZYX_INT_STATUS_FUNC, or combinations.  
*   @param timeout_ms maximum waiting time in milliseconds for flag to occur
*   @param interrupt a pointer that will contain the value of the interrupt status register
*
* @retval #true event occured.
* @retval #false event did not occur, this function timed out.
*/
'''
def waitForFlag(interrupt_flag, timeout_ms, interrupt = None):
	global _pozyx
	if interrupt:
		i = pointer(c_uint8(interrupt))
	else:
		i = POINTER(c_uint8)()
	return _pozyx.waitForFlag_safe(c_uint8(interrupt_flag), c_int(timeout_ms), i)

'''
/**
* Initiates the Pozyx shield. This function initializes the pozyx device. 
* It will verify that the device is functioning correctly by means of the self-test, and it will configure the interrupts. 
* See the register reg:POZYX_INT_MASK for more details about the interrupts.
* 
* @param print_result outputs the result of the function to the Serial output
* @param mode The modus of the system: #MODE_POLLING or #MODE_INTERRUPT
* @param interrupts defines which events trigger interrupts. This field is only required for #MODE_INTERRUPT. Possible 
* values are bit-wise combinations of #POZYX_INT_MASK_ERR, #POZYX_INT_MASK_POS, #POZYX_INT_MASK_IMU, #POZYX_INT_MASK_RX_DATA and #POZYX_INT_MASK_FUNC. Use #POZYX_INT_MASK_ALL to trigger on all events.   
* @param interrupt_pin Pozyx interrupt pin: #POZYX_INT_PIN0 or #POZYX_INT_PIN1. This field is only required for #MODE_INTERRUPT. 
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*/
'''
def begin(print_result = False, mode = MODE_INTERRUPT, interrupts = POZYX_INT_MASK_ALL, interrupt_pin = POZYX_INT_PIN0):
	global _pozyx
	return int(_pozyx.begin(c_bool(print_result), c_int(mode), c_int(interrupts), c_int(interrupt_pin)))

'''
reg_address is the specific register address to start reading from
size is the number of bytes to read

This function actually returns a tuple where 
the first element is the status returned by the c function
the second element is what the c function did to pData (what it read from the register)
below is the original comment from the c function
/**
* Read from the registers of the connected Pozyx shield.
* 
*   @param reg_address: the specific register address to start reading from
*   @param pData: a pointer to the data thas will be read
*   @param size: the number of bytes to read
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*/
'''
def regRead(reg_address, size):
	global _pozyx
	pData = (c_uint8 * size)()
	status = int(_pozyx.regRead(c_uint8(reg_address), pData, c_int(size)))
	return (status, [pData[i] for i in range(size)])


'''
reg_address is the specific register address to start writing to	
pData is a string of the data to be written to the address
below is the original comment from the c function
/**
* Write to the registers of the connected Pozyx shield.
* 
*   @param reg_address: the specific register address to start writing to
*   @param pData: a pointer to the data thas will be written
*   @param size: the number of bytes to write
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*/
size is calculated from the length of pData
'''
def regWrite(reg_address, pData):
	global _pozyx
	size = len(pData)
	ptr = (c_uint8 * size)(*pData)
	return int(_pozyx.regWrite(c_uint8(reg_address), ptr, c_int(size)))

'''
reg_address is the specific register address of the function
params is a list or tuple (maybe even a string) of parameters to send to the pozyx device's function being called
size is the number of bytes to be read (see size in c comments below)

this function will return a tuple where
	the first element is the status returned by the c function
	the second element is what the c function did to pData (what it read from the register)
below is the comment from the c code
/**
* Call a register funcion on the connected Pozyx shield.
* 
*   @param reg_address: the specific register address of the function
*   @param params: this is the pointer to a parameter array required for the specific function that is called
*   @param param_size: the number of bytes in the params array
*   @param pData: a pointer to the data thas will be read
*   @param size: the number of bytes that will be stored in pData
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*/
'''
def regFunction(reg_address, params = None, size = 0):
	global _pozyx
	if params:
		paramSize = len(params)
		paramPtr = (c_uint8 * paramSize)(*params)
	else:
		paramSize = 0
		paramPtr = POINTER(c_uint8)()
	if size != 0:
		pData = (c_uint8 * size)()
	else:
		pData = POINTER(c_uint8)()
	status = int(_pozyx.regFunction(c_uint8(reg_address), paramPtr, paramSize, pData, c_uint(size)))
	return (status, [pData[i] for i in range(size)])

'''
/**
* Write to the registers on a remote Pozyx device (anchor or tag).
* 
*   @param destination: this is the network id of the receiving Pozyx tag
*   @param reg_address: the specific register address to start writing to
*   @param pData: a pointer to the data thas will be written
*   @param size: the number of bytes to write
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*/ 
'''
def remoteRegWrite(destination, reg_address, pData):
	global _pozyx 
	size = len(pData)
	ptr = (c_uint8 * size)(*pData)
	return int(_pozyx.remoteRegWrite(c_uint16(destination), c_uint8(reg_address), ptr, c_int(size)))

'''
destination is the network id of the receiving Pozyx tag
reg_address is the specific register address to start reading from
size is the number of bytes to read

This function actually returns a tuple where 
	the first element is the status returned by the c function
	the second element is what the c function did to pData (what it read from the register)
below is the original comment from the c function 
/**
* Read from the registers on a remote Pozyx device (anchor or tag).
* 
*   @param destination: this is the network id of the receiving Pozyx tag
*   @param reg_address: the specific register address to start reading from
*   @param pData: a pointer to the data thas will be read
*   @param size: the number of bytes to read
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def remoteRegRead(destination, reg_address, size):
	global _pozyx
	pData = (c_uint8 * size)()
	status = int(_pozyx.remoteRegRead(c_uint16(destination), c_uint8(reg_address), pData, c_int(size)))
	return (status, [pData[i] for i in range(size)])

'''
destination is the network id of the receiving Pozyx tag
reg_address is the specific register address of the function
params is a list or tuple (maybe even a string) of parameters to send to the pozyx device's function being called
size is the number of bytes to be read (see size in c comments below)

this function will return
	an int representing the status returned by the c function
	a string representing the data read from the register if there is any

below is the comment from the c code
/**
* Call a register funcion on a remote Pozyx device (anchor or tag).
* 
*   @param destination: this is the network id of the receiving Pozyx tag
*   @param reg_address: the specific register address of the function
*   @param params: this is the pointer to a parameter array required for the specific function that is called
*   @param param_size: the number of bytes in the params array
*   @param pData: a pointer to the data thas will be read
*   @param size: the number of bytes that will be stored in pData
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def remoteRegFunction(destination, reg_address, params = None, size = 0):
	global _pozyx
	if params:
		paramSize = len(params)
		paramPtr = (c_uint8 * paramSize)(*params)
	else:
		paramSize = 0
		paramPtr = POINTER(c_uint8)()
	if size != 0:
		pData = (c_uint8 * size)()
	else:
		pData = POINTER(c_uint8)()
	status = int(_pozyx.remoteRegFunction(c_uint16(destination), c_uint8(reg_address), paramPtr, c_int(paramSize), pData, c_int(size)))
	return (status, [pData[i] for i in range(size)])

'''
/** @}*/ 

/** \addtogroup communication_functions 
*  @{
*/  

/**
* Wirelessly transmit data to a remote pozyx device.
* This function combines writeTXBufferData and sendTXBufferData to write data to the transmit buffer and immediately transmit it.
*
*   @param destination the network id of the device that should receive the data. A value of 0 will result in a broadcast
*   @param pData pointer to the data that should be transmitted 
*   @param size number of bytes to transmit
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*/
'''
def sendData(destination, pData):
	global _pozyx
	size = len(pData)
	pDataPtr = (c_uint8 * size)(*pData)
	return int(_pozyx.sendData(c_uint16(destination), pDataPtr, c_int(size)))

'''
/**
* Write data bytes in the transmit buffer.
* This function writes bytes in the transmit buffer without sending it yet.
*
*   @param data[] the array with data bytes
*   @param size size of the data array
*   @param offset The offset in the memory
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*
* @see sendTXBufferData
*/
'''
def writeTXBufferData(data, offset = 0):
	global _pozyx
	size = len(data)
	dataPtr = (c_uint8 * size)(*data)
	return int(_pozyx.writeTXBufferData(dataPtr, c_int(size), c_int(offset)))

'''
/**
* Wirelessly transmit data.
* Wirelessly transmit the contents of the transmit buffer over UWB.
* 
*   @param destination the network id of the device that should receive the data. A value of 0 will result in a broadcast.
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*
* @see writeTXBufferData
*/
'''
def sendTXBufferData(destination = 0x0):
	global _pozyx
	return int(_pozyx.sendTXBufferData(c_uint16(destination)))

'''
/**
* Read data bytes from receive buffer.
* This function reads the bytes from the receive buffer from the last received message.
*
*   @param pData pointer to where the data will be stored
*   @param size the number of bytes to read from the receive buffer.
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*
* @see getLastDataLength getLastNetworkId
*/
'''
def readRXBufferData(size):
	global _pozyx
	dataPtr = (c_uint8 * size)()
	status = int(_pozyx.readRXBufferData(dataPtr, c_int(size)))
	return (status, [dataPtr[i] for i in range(size)])

'''
/**
* Obtain the network id of the last message.
* This function identifies the source of the last message that was wirelessly received.
*
*   @param network_id: the pointer that stores the network_id
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getLastNetworkId(remote_id = -1):
	global _pozyx
	network_idPtr = pointer(c_uint16())
	status = int(_pozyx.getLastNetworkId(network_idPtr, c_uint16(remote_id)))
	return (status, network_idPtr.contents.value)

'''
/**
* Obtain the number of bytes received.
* This function gives the number of bytes in the last message that was wirelessly received.
*
*   @param device_list_size: the pointer that stores the device list size
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getLastDataLength(remote_id = -1):
	global _pozyx
	data_length = pointer(c_uint8())
	status = int(_pozyx.getLastDataLength(data_length, c_uint16(remote_id)))
	return (status, data_length.contents.value)

'''
/**
* Obtain the network id of the connected Pozyx device.
* The network id is a unique 16bit identifier determined based on the hardware components. When the system is reset the orignal value is restored
*
*   @param network_id: reference to the network_id pointer to store the data
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*/
'''
def getNetworkId():
	global _pozyx
	network_idPtr = pointer(c_uint16)
	status = int(_pozyx.getNetworkId(network_idPtr))
	return (status, network_idPtr.contents.value)

'''
/**
* Overwrite the network id.
* This function overwrites the network id of the pozyx device either locally or remotely. The network id must be unique within a network.
* When the system is reset the orignal network id is restored.
*
*   @param network_id: the new network id
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def setNetworkId(network_id, remote_id = -1):
	global _pozyx
	return int(_pozyx.setNetworkId(c_uint16(network_id), c_uint8(remote_id)))

'''
/**
* Obtain the current UWB settings.
* Functions to retrieve the current UWB settings. In order to communicate, two pozyx devices must have exactly the same UWB settings.
*
*   @param UWB_settings reference to the UWB settings object to store the data
*   @param remote_id optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getUWBSettings(remote_id = -1):
	global _pozyx
	UWB_settings = pointer(UWB_settings_t())
	status = int(_pozyx.getUWBSettings(UWB_settings, c_uint16(remote_id)))
	return (status, UWB_settings.contents)

'''
/**
* Overwrite the UWB settings.
* This function overwrites the UWB settings such as UWB channel, gain, PRF, etc.
* In order to communicate, two pozyx devices must have exactly the same UWB settings.
* Upon reset, the default UWB settings will be loaded.
*
*   @param UWB_settings reference to the new UWB settings. If the gain_db is set to 0, it will automatically load the default gain value for the given uwb paramters.
*   @param remote_id optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def setUWBSettings(UWB_settings, remote_id = -1):
	global _pozyx
	return int(_pozyx.setUWBSettings(pointer(UWB_settings), c_uint16(remote_id)))

'''
/**
* Set the Ultra-wideband frequency channel.
* This function sets the ultra-wideband (UWB) frequency channel used for transmission and reception.
* For wireless communication, both the receiver and transmitter must be on the same UWB channel.
* More information can be found in the register reg:POZYX_UWB_CHANNEL.
*
*   @param channel_num the channel number, possible values are 1, 2, 3, 4, 5 and 7.
*   @param remote_id optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*/
'''
def setUWBChannel(channel_num, remote_id = -1):
	global _pozyx
	return int(_pozyx.setUWBChannel(c_int(channel_num), c_uint16(remote_id)))

'''
/**
* Get the Ultra-wideband frequency channel.
* This function reads the ultra-wideband (UWB) frequency channel used for transmission and reception.
* For wireless communication, both the receiver and transmitter must be on the same UWB channel.
* More information can be found in the register reg:POZYX_UWB_CHANNEL.
*
*   @param channel_num the channel number currently set, possible values are 1, 2, 3, 4, 5 and 7.
*   @param remote_id optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getUWBChannel(remote_id):
	global _pozyx
	channel_num = pointer(c_int())
	status = int(_pozyx.getUWBChannel(channel_num, c_uint16(remote_id)))
	return (status, channel_num.contents.value)

'''
/**
* configure the UWB transmission power.
*
* @note setting a large TX gain may cause the system to fall out of regulation. Applications that require regulations must set an appropriate TX gain to meet the spectral mask of your region. This can be verified with a spectrum analyzer.
*   
* This function configures the UWB transmission power gain. The default value is different for each UWB channel.
* Setting a larger transmit power will result in a larger range. For increased communication range (which is two-way), both the 
* transmitter and the receiver must set the appropriate transmit power.
* Changing the UWB channel will reset the power to the default value.
*
*   @param txgain_dB the transmission gain in dB. Possible values are between 0dB and 33.5dB, with a resolution of 0.5dB.
*   @param remote_id optional parameter that determines the remote device to be used.
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def setTxPower(txgain_dB, remote_id = -1):
	global _pozyx
	return int(_pozyx.setTxPower(c_float(txgain_dB), c_uint16(remote_id)))

'''
/**
* Obtain the UWB transmission power.
*  
* This function obtains the configured UWB transmission power gain. The default value is different for each UWB channel.
* Changing the UWB channel will reset the TX power to the default value.
*
*   @param txgain_dB the configured transmission gain in dB. Possible values are between 0dB and 33.5dB, with a resolution of 0.5dB.
*   @param remote_id optional parameter that determines the remote device to be used.
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getTxPower(remote_id = -1):
	global _pozyx
	txgain_dB = pointer(c_float())
	status = int(_pozyx.getTxPower(txgain_dB, c_uint16(remote_id)))
	return (status, txgain_dB.contents.value)

'''
/** \addtogroup system_functions 
 *  @{
 */
    
/**
* Obtain the who_am_i value.
* This function reads the reg:POZYX_WHO_AM_I register. 
*
*   @param whoami: reference to the pointer where the read data should be stored e.g. whoami
*   @param remote_id: optional parameter that determines the remote device to be read
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getWhoAmI(remote_id = -1):
	global _pozyx
	whoami = pointer(c_uint8())
	status = int(_pozyx.getWhoAmI(whoami, c_uint16(remote_id)))
	return (status, whoami.contents.value)

'''
/**
* Obtain the firmware version.
* This function reads the reg:POZYX_FIRMWARE_VER register. 
*
*   @param firmware: reference to the pointer where the read data should be stored e.g. the firmware version
*   @param remote_id: optional parameter that determines the remote device to be read
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getFirmwareVersion(remote_id = -1):
	global _pozyx
	firmware = pointer(c_uint8())
	status = int(_pozyx.getFirmwareVersion(firmware, c_uint16(remote_id)))
	return (status, firmware.contents.value)

'''
/**
* Obtain hte hardware version.
* This function reads the reg:POZYX_HARDWARE_VER register. 
*
*   @param data: reference to the pointer where the read data should be stored e.g. hardware version
*   @param remote_id: optional parameter that determines the remote device to be read
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getHardwareVersion(remote_id = -1):
	global _pozyx
	hardware = pointer(c_uint8())
	status = int(_pozyx.getHardwareVersion(hardware, c_uint16(remote_id)))
	return (status, hardware.contents.value)

'''
/**
* Obtain the selftest result.
* This function reads the reg:POZYX_ST_RESULT register. 
*
*   @param data: reference to the pointer where the read data should be stored e.g. the selftest result
*   @param remote_id: optional parameter that determines the remote device to be read
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getSelftest(remote_id = -1):
	global _pozyx
	selftest = pointer(c_uint8())
	status = int(_pozyx.getSelftest(selftest, c_uint16(remote_id)))
	return (status, selftest.contents.value)

'''
/**
* Obtain the error code.
* This function reads the reg:POZYX_ERRORCODE register. 
*
*   @param data: reference to the pointer where the read data should be stored e.g. the error code
*   @param remote_id: optional parameter that determines the remote device to be read
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getErrorCode(remote_id = -1):
	global _pozyx
	error_code = pointer(c_uint8())
	status = int(_pozyx.getErrorCode(error_code, c_uint16(remote_id)))
	return (status, error_code.contents.value)

'''
/**
* Obtain the interrupt status.
* This function reads the reg:POZYX_INT_STATUS register. 
*
*   @param data: reference to the pointer where the read data should be stored e.g. the interrupt status
*   @param remote_id: optional parameter that determines the remote device to be read
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*
* @see waitForFlag
*/
'''
def getInterruptStatus(remote_id = -1):
	global _pozyx
	interrupts = pointer(c_uint8())
	status = int(_pozyx.getInterruptStatus(interrupts, c_uint16(remote_id)))
	return (status, interrupts.contents.value)

'''
/**
* Obtain the calibration status.
* This function reads the reg:POZYX_CALIB_STATUS register. 
*
*   @param data: reference to the pointer where the read data should be stored e.g. calibration status
*   @param remote_id: optional parameter that determines the remote device to be read
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getCalibrationStatus(remote_id = -1):
	global _pozyx
	calibration_status = pointer(c_uint8())
	status = int(_pozyx.getCalibrationStatus(calibration_status, c_uint16(remote_id)))
	return (status, calibration_status.contents.value)

'''
/**
* Obtain the digital value on one of the GPIO pins.
* Function to retieve the value of the given General Purpose Input/output pin (GPIO) on the target device 
*
*   @param gpio_num: the gpio pin to be set or retrieved. Possible values are 1, 2, 3 or 4.
*   @param value: the pointer that stores the value for the GPIO. 
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*
* @note In firmware version v0.9. The GPIO state cannot be read remotely.
*/
'''
def getGPIO(gpio_num, remote_id = -1):
	global _pozyx
	value = pointer(c_uint8())
	status = int(_pozyx.getGPIO(c_int(gpio_num), value, c_uint16(remote_id)))
	return (status, value.contents.value)

'''
/**
* Set the digital value on one of the GPIO pins.
* Function to set or set the value of the given GPIO on the target device 
*
*   @param gpio_num: the gpio pin to be set or retrieved. Possible values are 1, 2, 3 or 4.
*   @param value: the value for the GPIO. Can be O (LOW) or 1 (HIGH).
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def setGPIO(gpio_num, value, remote_id = -1):
	global _pozyx
	return int(_pozyx.setGPIO(c_int(gpio_num), c_uint8(value), c_uint16(remote_id)))

'''
/**
* Trigger a software reset of the Pozyx device.
* Function that will trigger the reset of the system. 
* This will reload all configurations from flash memory, or to their default values.
*
*   @param remote_id: optional parameter that determines the remote device to be used.
*
* @see clearConfiguration, saveConfiguration
*/
'''
def resetSystem(remote_id = -1):
	global _pozyx
	_pozyx.resetSystem(c_uint16(remote_id))

'''
/**
* Function for turning the leds on and off.
* This function allows you to turn one of the 4 LEDs on the Pozyx board on or off. 
* By default, the LEDs are controlled by the Pozyx system to give status information. 
* This can be changed using the function setLedConfig.
*
*   @param led_num: the led number to be controlled, value between 1, 2, 3 or 4.
*   @param state: the state to set the selected led to. Can be 0 (led is off) or 1 (led is on)
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*
* @see setLedConfig
*/
'''
def setLed(led_num, state, remote_id = -1):
	global _pozyx
	return int(_pozyx.setLed(c_int(led_num), c_bool(state), c_uint16(remote_id)))

'''
/**
* Function to obtain the interrupt configuration.
* This functions obtains the interrupt mask from the register reg:POZYX_INT_MASK. 
* The interrupt mask is used to determine which event trigger an interrupt.
*
*   @param mask: the configured interrupt mask
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getInterruptMask(remote_id = -1):
	global _pozyx
	mask = pointer(c_uint8())
	status = int(_pozyx.getInterruptMask(mask, c_uint16(remote_id)))
	return (status, mask.contents.value)

'''
/**
* Function to configure the interrupts.
* Function to configure the interrupts by means of the interrupt mask from the register reg:POZYX_INT_MASK. 
* The interrupt mask is used to determine which event trigger an interrupt.
*
*   @param mask: reference to the interrupt mask to be written. Bit-wise combinations of the following flags are allowed: #POZYX_INT_MASK_ERR, 
*   #POZYX_INT_MASK_POS, #POZYX_INT_MASK_IMU, #POZYX_INT_MASK_RX_DATA, #POZYX_INT_MASK_FUNC.
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def setInterruptMask(mask, remote_id = -1):
	global _pozyx
	return int(_pozyx.setInterruptMask(c_uint8(mask), c_uint16(remote_id)))

'''
/**
* Obtain the pull configuration of a GPIO pin.
* Function to obtain the configured pin mode of the GPIO for the given pin number. This is performed by reading from 
* the reg:POZYX_CONFIG_GPIO1 up to reg:POZYX_CONFIG_GPIO4 register.
*
*   @param gpio_num: the pin to update
*   @param mode: pointer to the mode of GPIO. Possible values #POZYX_GPIO_DIGITAL_INPUT, #POZYX_GPIO_PUSHPULL, #POZYX_GPIO_OPENDRAIN
*   @param remote_id: optional parameter that determines the remote device to be used.
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getConfigModeGPIO(gpio_num, remote_id = -1):
	global _pozyx
	mode = pointer(c_uint8())
	status = int(_pozyx.getConfigModeGPIO(c_int(gpio_num), mode, c_uint16(remote_id)))
	return (status, mode.contents.value)

'''
/**
* Obtain the pull configuration of a GPIO pin.
* Function to obtain the configured pull resistor of the GPIO for the given pin number. This is performed by reading from 
* the reg:POZYX_CONFIG_GPIO1 up to reg:POZYX_CONFIG_GPIO4 register.
*
*   @param gpio_num: the pin to update
*   @param pull: pull configuration of GPIO. Possible values are #POZYX_GPIO_NOPULL, #POZYX_GPIO_PULLUP, #POZYX_GPIO_PULLDOWN. 
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getConfigPullGPIO(gpio_num, remote_id = -1):
	global _pozyx
	pull = pointer(c_uint8())
	status = int(_pozyx.getConfigPullGPIO(c_int(gpio_num), pull, c_uint16(remote_id)))
	return (status, pull.contents.value)

'''
/**
* Configure the GPIOs.
* Function to set the configuration mode of the GPIO for the given pin number. This is performed by writing to 
* the reg:POZYX_CONFIG_GPIO1 up to reg:POZYX_CONFIG_GPIO4 register.
*
*   @param gpio_num: the GPIO pin to update. Possible values are 1, 2, 3 or 4.
*   @param mode: the mode of GPIO. Possible values #POZYX_GPIO_DIGITAL_INPUT, #POZYX_GPIO_PUSHPULL, #POZYX_GPIO_OPENDRAIN
*   @param pull: pull configuration of GPIO. Possible values are #POZYX_GPIO_NOPULL, #POZYX_GPIO_PULLUP, #POZYX_GPIO_PULLDOWN. 
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def setConfigGPIO(gpio_num, mode, pull, remote_id = -1):
	global _pozyx
	return int(_pozyx.setConfigGPIO(c_int(gpio_num), c_int(mode), c_int(pull), c_uint16(remote_id)))

'''
/**
* Configure the LEDs.
* This function configures the 6 LEDs on the pozyx device by writing to the register reg:POZYX_LED_CTRL. 
* More specifically, the function configures which LEDs give system information. By default all the LEDs
* will give system information.
*
*   @param config default: the configuration to be set. See POZYX_LED_CTRL for details.
*   @param remote_id: optional parameter that determines the remote device to be used.
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*
* @see setLed
*/
'''
def setLedConfig(config = 0x0, remote_id = -1):
	global _pozyx
	return int(_pozyx.setLedConfig(c_uint8(config), c_uint16(remote_id)))

'''
/**
 * Configure the interrupt pin.
 * 
 * @param  pin          pin id on the pozyx device, can be 1,2,3,4 (or 5 or 6 on the pozyx tag)
 * @param  mode         push-pull or pull-mode
 * @param  bActiveHigh  is the interrupt active level HIGH (i.e. 3.3V)
 * @param  bLatch       should the interrupt be a short pulse or should it latch until the interrupt status register is read
 *
 * @retval #POZYX_SUCCESS success.
 * @retval #POZYX_FAIL function failed.   
 */
'''
def configInterruptPin(pin, mode, bActiveHigh, bLatch, remote_id = -1):
	global _pozyx
	return int(_pozyx.configInterruptPin(c_int(pin), c_int(mode), c_int(bActiveHigh), c_int(bLatch), c_uint16(remote_id)))

'''
/**
* Save (part of) the configuration to Flash memory.
* @version Requires firmware version v1.0
*
* This functions stores (parts of) the configurable Pozyx settings in the non-volatile flash memory of the Pozyx device.
* This function will save the current settings and on the next startup of the Pozyx device these saved settings will be loaded automatically.
* settings from the flash memory. All registers that are writable, the anchor ids for positioning and the device list (which contains the anchor coordinates) can be saved.
*
*   @param type this specifies what should be saved. Possible options are #POZYX_FLASH_REGS, #POZYX_FLASH_ANCHOR_IDS or #POZYX_FLASH_NETWORK.
*   @param registers an option array that holds all the register addresses for which the value must be saved. All registers that are writable are allowed.
*   @param num_registers optional parameter that determines the length of the registers array.
*   @param remote_id optional parameter that determines the remote device to be used.
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAIL function failed.
*
* @see clearConfiguration
*/
'''
def saveConfiguration(type, registers = None, remote_id = -1):
	global _pozyx
	if registers:
		num_registers = len(registers)
		registersPtr = (c_uint8 * num_registers)(*registers)
	else:
		num_registers = 0
		registersPtr = POINTER(c_uint8)()
	return int(_pozyx.saveConfiguration(c_int(type), registersPtr, c_int(num_registers), c_uint16(remote_id)))

'''
/**
* Clears the configuration.
* @version Requires firmware version v1.0
*
* This function clears (part of) the configuration that was previously stored in the non-volatile Flash memory.
* The next startup of the Pozyx device will load the default configuration values for the registers, anchor_ids and network list.
*
*   @param remote_id optional parameter that determines the remote device to be used.
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAIL function failed.
*
* @see saveConfiguration
*/
'''
def clearConfiguration(remote_id = -1):
	global _pozyx
	return int(_pozyx.clearConfiguration(c_uint16(remote_id)))

'''
/**
 * Verify if a register content is saved in the flash memory.
 * @version Requires firmware version v1.0
 *
 * This function verifies if a given register variable, specified by its address, is saved in flash memory.
 * @param  regAddress the register address to check
 * @param  remote_id optional parameter that determines the remote device to be used.
 * 
 * @retval true(1) if the register variable is saved
 * @retval false(0) if the register variable is not saved
 */
'''
def isRegisterSaved(regAddress, remote_id = -1):
	global _pozyx
	return int(_pozyx.isRegisterSaved(c_uint8(regAddress), c_uint16(remote_id))) == 1

'''
/**
 * Return the number of register variables saved in flash memory.
 * 
 * @param  remote_id optional parameter that determines the remote device to be used.
 * 
 * @return           the number of register variables saved in flash memory.
 */
'''
def getNumRegistersSaved(remote_id = -1):
	global _pozyx
	return int(_pozyx.getNumRegistersSaved(c_uint16(remote_id)))

'''
/** \addtogroup positioning_functions 
*  @{
*/


/**
* Obtain the last coordinates of the device.
* Retrieve the last coordinates of the device or the remote device. Note that this function does not
* trigger positioning.
*
*   @param coordinates reference to the coordinates pointer object.
*   @param remote_id optional parameter that determines the remote device to be used.
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*
* @see doPositioning, doRemotePositioning
*/
'''
def getCoordinates(remote_id = -1):
	global _pozyx
	coordinates = pointer(coordinates_t())
	status = int(_pozyx.getCoordinates(coordinates, c_uint16(remote_id)))
	return (status, coordinates.contents)

'''
/**
* Set the coordinates of the device. 
* Manually set the coordinates of the device or the remote device.
*
*   @param coordinates coordinates to be set
*   @param remote_id optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*
* @see getCoordinates
*/
'''
def setCoordinates(coordinates, remote_id = -1):
	global _pozyx
	return int(_pozyx.setCoordinates(coordinates, c_uint16(remote_id)))

'''
/**
* Obtain the last estimated position error covariance information. 
* Retrieve the last error covariance of the position for the device or the remote device. Note that this function does not
* trigger positioning.
*
*   @param pos_error reference to the pos error object
*   @param remote_id optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getPositionError(remote_id = -1):
	global _pozyx
	pos_error = pointer(pos_error_t())
	status = int(_pozyx.getPositionError(pos_error, c_uint16(remote_id)))
	return (status, pos_error.contents)

'''
/**
* Manually set which anchors to use for positioning.
* Function to manually set which anchors to use for positioning by calling the register function reg:POZYX_POS_SET_ANCHOR_IDS. 
* Notice that the anchors specified are only used if this is configured with setSelectionOfAnchors. Furthermore, the anchors 
* specified in this functions must be available in the device list with their coordinates.
*
*   @param anchors[] an array with network id's of anchors to be used
*   @param anchor_num the number of anchors write
*   @param remote_id optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*
* @see setSelectionOfAnchors, getPositioningAnchorIds
*/
'''
def setPositioningAnchorIds(anchors, remote_id = -1):
	global _pozyx
	anchor_num = len(anchors)
	anchorPtr = (c_uint16 * anchor_num)(*anchors)
	return int(_pozyx.setPositioningAnchorIds(anchorPtr, c_int(anchor_num), c_uint16(remote_id)))

'''
/**
* Obtain which anchors used for positioning.
* Function to retrieve the anchors that used for positioning by calling the register function reg:POZYX_POS_GET_ANCHOR_IDS. 
* When in automatic anchor selection mode, the chosen anchors are listed here.
*
*   @param anchors[] an array with network id's of anchors manually set
*   @param anchor_num the number of anchors to read.
*   @param remote_id optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*
* @see setSelectionOfAnchors, setPositioningAnchorIds
*/
'''
def getPositioningAnchorIds(anchor_num, remote_id = -1):
	global _pozyx
	anchors = (c_uint16 * anchor_num)()
	status = int(_pozyx.getPositioningAnchorIds(anchors, c_int(anchor_num), c_uint16(remote_id)))
	return (status, [anchors[i] for i in range(anchor_num)])

'''
/**
* Read the update interval continuous positioning.
* This function reads the configured update interval for continuous positioning from the register reg:POZYX_POS_INTERVAL.
*
*   @param ms pointer to the update interval in milliseconds. A value of 0 means that continuous positioning is disabled.
*   @param remote_id optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*
* @see setUpdateInterval
*/
'''
def getUpdateInterval(remote_id = -1):
	global _pozyx
	ms = pointer(c_uint16())
	status = int(_pozyx.getUpdateInterval(ms, c_uint16(remote_id)))
	return (status, ms.contents.value)

'''
/**
* Configure the udpate interval for continuous positioning.
* This function configures the update interval by writing to the register reg:POZYX_POS_INTERVAL.
* By writing a valid value, the system will start continuous positioning which will generate a position estimate
* on regular intervals. This will generate a #POZYX_INT_STATUS_POS interrupt when interrupts are enabled.
*
*   @param ms update interval in milliseconds. The update interval must be larger or equal to 100ms.
*   @param remote_id optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*
* @see getUpdateInterval
*/
'''
def setUpdateInterval(ms, remote_id = -1):
	global _pozyx
	return int(_pozyx.setUpdateInterval(c_uint16(ms), c_uint16(remote_id)))

'''
/**
* Obtain the configured positioning algorithm.
* This function obtains the configured positioning algorithm by reading from the reg:POZYX_POS_ALG register.
* 
*   @param algorithm pointer to the variable holding the algorithm used to determine position. 
*   Possible values for the positioning algorithm are #POZYX_POS_ALG_UWB_ONLY and #POZYX_POS_ALG_LS.
*   @param remote_id optional parameter that determines the remote device to be used
* 
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*
* @see getPositionDimension, setPositionAlgorithm
*/
'''
def getPositionAlgorithm(remote_id = -1):
	global _pozyx
	algorithm = pointer(c_uint8())
	status = int(_pozyx.getPositionAlgorithm(algorithm, c_uint16(remote_id)))
	return (status, algorithm.contents.value)

'''
/**
* Obtain the configured positioning dimension.
* This function obtains the configuration of the physical dimension by reading from the reg:POZYX_POS_ALG register.
* 
*   @param dimension pointer to physical dimension used for the algorithm. Possible values for the dimension are #POZYX_3D, #POZYX_2D or #POZYX_2_5D.
*   @param remote_id optional parameter that determines the remote device to be used
* 
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*
* @see getPositionAlgorithm, setPositionAlgorithm
*/
'''
def getPositionDimension(remote_id = -1):
	global _pozyx
	dimension = pointer(c_uint8())
	status = int(_pozyx.getPositionDimension(dimension, c_uint16(remote_id)))
	return (status, dimension.contents.value)

'''
/**
* Configure the positioning algorithm.
* This function configures the positioning algorithm and the desired physical dimension by writing to the
* register reg:POZYX_POS_ALG.
*
*   @param algorithm algorithm used to determine the position. Possible values are #POZYX_POS_ALG_UWB_ONLY and #POZYX_POS_ALG_LS.  
*   @param dimension physical dimension used for the algorithm. Possible values are #POZYX_3D, #POZYX_2D or #POZYX_2_5D.
*   @param remote_id optional parameter that determines the remote device to be used
* 
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*
* @see getPositionAlgorithm, getPositionDimension
*/
'''
def setPositionAlgorithm(algorithm = POZYX_POS_ALG_UWB_ONLY, dimension = 0x0, remote_id = -1):
	global _pozyx
	return int(_pozyx.setPositionAlgorithm(c_int(algorithm), c_int(dimension), c_uint16(remote_id)))

'''
/**
* Obtain the anchor selection mode.
* This function reads the anchor selection mode bit from the register reg:POZYX_POS_NUM_ANCHORS.
* This bit describes how the anchors are selected for positioning, either manually or automatically.
*
*   @param mode reference to the anchor selection mode. Possible results are #POZYX_ANCHOR_SEL_MANUAL for manual anchor selection or #POZYX_ANCHOR_SEL_AUTO for automatic anchor selection.
*   @param remote_id optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getAnchorSelectionMode(remote_id = -1):
	global _pozyx
	mode = pointer(c_uint8())
	status = int(_pozyx.getAnchorSelectionMode(mode, c_uint16(remote_id)))
	return (status, mode.contents.value)

'''
/**
* Obtain the configured number of anchors used for positioning.
* This functions reads out the reg:POZYX_POS_NUM_ANCHORS register to obtain the
* number of anchors that are being used for positioning.
*
*   @param nr_anchors reference to the number of anchors
*   @param remote_id optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/  
'''
def getNumberOfAnchors(remote_id = -1):
	global _pozyx
	nr_anchors = pointer(c_uint8())
	status = int(_pozyx.getNumberOfAnchors(nr_anchors, c_uint16(remote_id)))
	return (status, nr_anchors.contents.value)

'''
/**
* Configure how many anchors are used for positioning and how they are selected.
* This function configures the number of anchors used for positioning. Theoretically, a larger
* number of anchors leads to better positioning performance. However, in practice this is not always the case.
* The more anchors used for positioning, the longer the positioning process will take. Furthermore, 
* it can be chosen which anchors are to be used: either a fixed set given by the user, or an automatic selection
* between all the anchors in the internal device list.
*
*   @param mode describes how the anchors are selected. Possible values are #POZYX_ANCHOR_SEL_MANUAL for manual anchor selection or #POZYX_ANCHOR_SEL_AUTO for automatic anchor selection.
*   @param nr_anchors the number of anchors to use for positioning. Must be larger than 2 and smaller than 16.
*   @param remote_id optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*
* @see setPositioningAnchorIds to set the anchor IDs in manual anchor selection mode.
*/  
'''
def setSelectionOfAnchors(mode, nr_anchors, remote_id = -1):
	global _pozyx
	return int(_pozyx.setSelectionOfAnchors(c_int(mode), c_int(nr_anchors), c_uint16(remote_id)))

'''
/**
* Obtain the operation mode of the device.
* This function obtains the operation mode (anchor or tag) by reading from the register reg:POZYX_OPERATION_MODE.
* This operation mode is independent of the hardware it is on and will have it's effect when performing discovery
* or auto calibration.
*
*   @param mode The mode of operations #POZYX_ANCHOR_MODE or #POZYX_TAG_MODE
*   @param remote_id optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*
* @see setOperationMode
*/
'''
def getOperationMode(remote_id = -1):
	global _pozyx
	mode = pointer(c_uint8())
	status = int(_pozyx.getOperationMode(mode, c_uint16(remote_id)))
	return (status, mode.contents.value)

'''
/**
* Define if the device operates as a tag or an anchor.
* This function defines how the device should operate (as an anchor or tag) by writing to the register reg:POZYX_OPERATION_MODE.
* This operation mode is independent of the hardware it is on and will have it's effect when performing discovery
* or auto calibration. This function overrules the jumper that is present on the board.
*
*   @param mode The mode of operations #POZYX_ANCHOR_MODE or #POZYX_TAG_MODE
*   @param remote_id optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*
* @see getOperationMode
*/ 
'''
def setOperationMode(mode, remote_id = -1):
	global _pozyx
	return int(_pozyx.setOperationMode(c_uint8(mode), c_uint16(remote_id)))

'''
/**
* Get the textual system error.
* This function reads out the reg:POZYX_ERRORCODE register and converts the error code to a textual message.
*
*   @param remote_id optional parameter that determines the remote device to be used
*
* @retval std::string the textual error
*
*/
'''
def getSystemError(remote_id = -1):
	global _pozyx
	sysErr = _pozyx.getSystemError(c_uint16(remote_id))
	return sysErr

'''
/**
* Retrieve the configured sensor mode.
* This function reads out the register reg:POZYX_SENSORS_MODE which describes the configured sensor mode.
*
*   @param sensor_mode: reference to the sensor mode
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getSensorMode(remote_id = -1):
	global _pozyx
	sensor_mode = pointer(c_uint8())
	status = int(_pozyx.getSensorMode(sensor_mode, c_uint16(remote_id)))
	return (status, sensor_mode.contents.value)

'''
/**
* Configure the sensor mode.
* This function reads out the register reg:POZYX_SENSORS_MODE which describes the configured sensor mode.
*
*   @param sensor_mode: the desired sensor mode.
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*/
'''
def setSensorMode(sensor_mode, remote_id = -1):
	global _pozyx
	return int(_pozyx.setSensorMode(c_uint8(sensor_mode), c_uint16(remote_id)))

'''
/**
* Obtain all sensor data at once.
* This functions reads out the pressure, acceleration, magnetic field strength, angular velocity, orientation in Euler angles, the orientation as a quaternion, 
* the linear acceleration, the gravity vector and temperature.
*
*   @param sensor_data: reference to the sensor_data object
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getAllSensorData(remote_id = -1):
	global _pozyx
	sensor_data = pointer(sensor_data_t())
	status = int(_pozyx.getAllSensorData(sensor_data, c_uint16(remote_id)))
	return (status, sensor_data.contents)

'''
/**
* Obtain the atmospheric pressure in Pascal. 
* This function reads out the pressure starting from the register POZYX_PRESSURE. The maximal update rate is 10Hz. The units are Pa.
*
*   @param pressure: reference to the pressure variable
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getPressure_Pa(remote_id = -1):
	global _pozyx
	pressure = pointer(c_float())		
	status = int(_pozyx.getPressure_Pa(pressure, c_uint16(remote_id)))
	return (status, pressure.contents.value)

'''
/**
* Obtain the 3D acceleration vector in mg. 
* This function reads out the acceleration data starting from the register reg:POZYX_ACCEL_X. The maximal update rate is 100Hz. The units are mg.
* The vector is expressed in body coordinates (i.e., along axes of the device).
*
*   @param acceleration: reference to the acceleration object
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getAcceleration_mg(remote_id = -1):
	global _pozyx
	acceleration = pointer(acceleration_t())
	status = int(_pozyx.getAcceleration_mg(acceleration, c_uint16(remote_id)))
	return (status, acceleration.contents)

'''
/**
* Obtain the 3D magnetic field strength vector in = Tesla.
* This function reads out the magnetic field strength data starting from the register reg:POZYX_MAGN_X. The maximal update rate is 100Hz.
* The vector is expressed in body coordinates (i.e., along axes of the device).
*
*   @param magnetic: reference to the magnetic object
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getMagnetic_uT(remote_id = -1):
	global _pozyx
	magnetic = pointer(magnetic_t())
	status = int(_pozyx.getMagnetic_uT(magnetic, c_uint16(remote_id)))
	return (status, magnetic.contents)

'''
/**
* Obtain the 3D angular velocity vector degrees per second.
* This function reads out the angular velocity from the gyroscope using the register reg:POZYX_GYRO_X. The maximal update rate is 100Hz.
* The rotations are expressed in body coordinates (i.e., the rotations around the axes of the device).
*
*   @param angular_vel: reference to the angular velocity object
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getAngularVelocity_dps(remote_id = -1):
	global _pozyx
	angular_vel = pointer(angular_vel_t())
	status = int(_pozyx.getAngularVelocity_dps(angular_vel, c_uint16(remote_id)))
	return (status, angular_vel.contents)

'''
/**
* Obtain the orientation in Euler angles in degrees.
* This function reads out the Euleur angles: Yaw, Pitch and Roll that represents the 3D orientation of the device
*
*   @param euler_angles: reference to the euler_angles object
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getEulerAngles_deg(remote_id = -1):
	global _pozyx
	euler_angles = pointer(euler_angles_t())
	status = int(_pozyx.getEulerAngles_deg(euler_angles, c_uint16(remote_id)))
	return (status, euler_angles.contents)

'''
/**
* Obtain the orientation in quaternions.
* This function reads out the 4 coordinates of the quaternion that represents the 3D orientation of the device.
* The quaternions are unitless and normalized.
*
*   @param quaternion: reference to the quaternion object
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getQuaternion(remote_id = -1):
	global _pozyx
	quaternion = pointer(quaternion_t())
	status = int(_pozyx.getQuaternion(quaternion, c_uint16(remote_id)))
	return (status, quaternion.contents)

'''
/**
* Obtain the 3D linear acceleration in mg.
* This function reads out the linear acceleration data starting from the register reg:POZYX_LIA_X. 
* The Linear acceleration is the acceleration compensated for gravity.
* The maximal update rate is 100Hz. The units are mg.
* The vector is expressed in body coordinates (i.e., along axes of the device).
*
*   @param linear_acceleration: reference to the acceleration object
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getLinearAcceleration_mg(remote_id = -1):
	global _pozyx
	linear_acceleration = pointer(linear_acceleration_t())
	status = int(_pozyx.getLinearAcceleration_mg(linear_acceleration, c_uint16(remote_id)))
	return (status, linear_acceleration.contents)

'''
/**
* Obtain the 3D gravity vector in mg.
* This function reads out the gravity vector coordinates starting from the register reg:POZYX_GRAV_X. The maximal update rate is 100Hz. The units are mg.
* The vector is expressed in body coordinates (i.e., along axes of the device). This vector always points to the ground, regardless of the orientation.
*
*   @param gravity_vector: reference to the gravity_vector object
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getGravityVector_mg(remote_id = -1):
	global _pozyx
	gravity_vector = pointer(gravity_vector_t())
	status = int(_pozyx.getGravityVector_mg(gravity_vector, c_uint16(remote_id)))
	return (status, gravity_vector.contents)

'''
/**
* Obtain the temperature in degrees Celcius.
* This function reads out the temperature from the register reg:POZYX_TEMPERATURE. 
* This function is unsupported in firmware version v0.9.
*
*   @param temperature: reference to the temperature variable
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
* @retval #POZYX_TIMEOUT function timed out, no response received.
*/
'''
def getTemperature_c(remote_id = -1):
	global _pozyx
	temperature = pointer(c_float())
	status = int(_pozyx.getTemperature_c(temperature, c_uint16(remote_id)))
	return (status, temperature.contents)

'''
/**
* Obtain the coordinates.
* This function triggers the positioning algorithm to perform positioning with the given parameters.
* By default it will automatically select 4 anchors from the internal device list. It will then perform
* ranging with these anchors and use the results to compute the coordinates. 
* This function requires that the coordinates for the anchors are stored in the internal device list.
*
* Please read the tutorial ready to localize to get started with positioning.
*
*   @param position: data object to store the result
*   @param dimension: optional flag to specify the positioning dimension, possible options are #POZYX_2D, #POZYX_2_5D, and #POZYX_3D
*   @param height: optional parameter that is used for #POZYX_2_5D to give the height in mm of the tag 
*   @param algorithm: optional flag to specifiy the positioning algorithm to be used. Possible options are #POZYX_POS_ALG_UWB_ONLY and #POZYX_POS_ALG_LS
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*
* @see doRemotePositioning, doAnchorCalibration, addDevice, setSelectionOfAnchors
*/
'''
def doPositioning(dimension = POZYX_2D, height = 0, algorithm = POZYX_POS_ALG_UWB_ONLY):
	global _pozyx
	coordinates = pointer(coordinates_t())
	status = int(_pozyx.doPositioning(coordinates, c_uint8(dimension), c_int32(height), c_uint8(algorithm)))
	return (status, coordinates.contents)

'''
/**
* Obtain the coordinates of a remote device.
*
* @note This function is subject to major change upon the next firmware update.
*
* This function triggers the positioning algorithm on a remote pozyx device to perform positioning with the given parameters.
* By default it will automatically select 4 anchors from the internal device list on the remote device. The device will perform
* ranging with the anchors and use the results to compute the coordinates. 
* This function requires that the coordinates for the anchors are stored in the internal device list on the remote device.
* After positioning is completed, the remote device will automatically transmit the result back.
*
*   @param remote_id: the remote device that will do the positioning
*   @param position: data object to store the result
*   @param dimension: optional flag to specify the positioning dimension, possible options are #POZYX_2D, #POZYX_2_5D, and #POZYX_3D
*   @param height: optional parameter that is used for #POZYX_2_5D to give the height in mm of the tag 
*   @param algorithm: optional flag to specifiy the positioning algorithm to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*
* @see doPositioning, addDevice, setSelectionOfAnchors
*/
'''
def doRemotePositioning(remote_id, dimension = POZYX_2D, height = 0, algorithm = POZYX_POS_ALG_UWB_ONLY):
	global _pozyx
	coordinates = pointer(coordinates_t())
	status = int(_pozyx.doRemotePositioning(c_uint16(remote_id), coordinates, c_uint8(dimension), c_int32(height), c_uint8(algorithm)))
	return (status, coordinates.contents)

'''
/**
* Trigger ranging with a remote device.
* This function performs ranging with a remote device using the UWB signals.
*
*   @param destination: the target device to do ranging with 
*   @param range: the pointer to where the resulting data will be stored
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*
* @see doRemoteRanging, getDeviceRangeInfo
*/
'''
def doRanging(destination):
	global _pozyx 
	rangePtr = pointer(device_range_t())
	status = int(_pozyx.doRanging(c_uint16(destination), rangePtr))
	return (status, rangePtr.contents)

'''
/**
* Trigger ranging between two remote devivces.  
* Function allowing to trigger ranging between two remote devices A and B. The ranging data is collected by
* device A and transmitted back to the local device.
*
*   @param device_from: device A that will initiate the range request.
*   @param device_to: device B that will respond to the range request.
*   @param range: the pointer to where the resulting data will be stored
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*
* @see doRanging, getDeviceRangeInfo
*/
'''
def doRemoteRanging(device_from, device_to):
	global _pozyx
	rangePtr = pointer(device_range_t())
	status = int(_pozyx.doRemoteRanging(c_uint16(device_from), c_uint16(device_to), rangePtr))
	return (status, rangePtr.contents)

'''
/**
* Retrieve stored ranging information.
* Functions to retrieve the latest ranging information (i.e., the distance, signal strength and timestamp) with
* respect to a remote device. This function does not trigger ranging.
*
*   @param device_id: network id of the device for which range information is requested
*   @param device_range: data object to store the information
*   @param remote_id: optional parameter that determines the remote device where this function is called.
*
* @see doRanging, doRemoteRanging
*/
'''
def getDeviceRangeInfo(device_id, remote_id = -1):
	global _pozyx
	device_range = pointer(device_range_t())
	status = int(_pozyx.getDeviceRangeInfo(c_uint16(device_id), device_range, c_uint16(remote_id)))
	return (status, device_range.contents)

'''
/**
* Obtain the number of devices stored internally.
* The following function retrieves the number of devices stored in the device list.
*
*   @param device_list_size: the pointer that stores the device list size
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*
* @see doDiscovery, doAnchorCalibration
*/
'''
def getDeviceListSize(remote_id = -1):
	global _pozyx
	device_list_size = pointer(c_uint8())
	status = int(_pozyx.getDeviceListSize(device_list_size, c_uint16(remote_id)))
	return (status, device_list_size.contents)

'''
/**
* Obtain the network IDs from all the devices in the device list.
* Function to get all the network ids of the devices in the device list
*
*   @param devices[]: array that will be filled with the network ids
*   @param size the number of network IDs to read.
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*/
'''
def getDeviceIds(size, remote_id = -1):
	global _pozyx
	devices = (c_uint16 * size)()
	status = int(_pozyx.getDeviceIds(devices, c_int(size), c_uint16(remote_id)))
	return (status, [devices[i] for i in range(size)])

'''
/**
* Obtain the network IDs from all the anchors in the device list.
* Function to get all the network ids of the anchors in the device list
*
*   @param devices[]: array that will be filled with the network ids
*   @param size the number of network IDs to read.
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*/
'''
def getAnchorIds(size, remote_id = -1):
	global _pozyx
	anchors = (c_uint16 * size)()
	status = int(_pozyx.getAnchorIds(anchors, c_int(size), c_uint16(remote_id)))
	return (status, [anchors[i] for i in range(size)])

'''
/**
* Obtain the network IDs from all the tags in the device list.
* Function to get all the network ids of the tags in the device list
*
*   @param tags[]: array that will be filled with the network ids
*   @param size the number of network IDs to read.
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*/
'''
def getTagIds(size, remote_id = -1):
	global _pozyx
	tags = (c_uint16 * size)()
	status = int(_pozyx.getTagIds(tags, c_int(size), c_uint16(remote_id)))
	return (status, tags)

'''
/**
* Discover Pozyx devices in range.
* Function to wirelessly discover anchors/tags/all Pozyx devices in range. The discovered devices are added
* to the internal device list.
*
*   @param type which type of device to discover. Possible values are #POZYX_DISCOVERY_ANCHORS_ONLY: anchors only, #POZYX_DISCOVERY_TAGS_ONLY: tags only or #POZYX_DISCOVERY_ALL_DEVICES: anchors and tags
*   @param slots The number of slots to wait for a response of an undiscovered device
*   @param slot_duration Time duration of an idle slot
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*
* @see getDeviceListSize, getDeviceIds
*/
'''
def doDiscovery(type = 0x0, slots = 3, slot_duration = 10):
	global _pozyx
	return int(_pozyx.doDiscovery(c_int(type), c_int(slots), c_int(slot_duration)))

'''
/**
* Automatically obtain the relative anchor positions.
* This function triggers the automatic anchor calibration to obtain the relative coordinates of up to 6
* pozyx devices in range. This function can be used for quickly setting up the positioning system. 
* The procedure may take several hundres of milliseconds depending on the number of devices in range and 
* the number of range measurements requested. During the calibration proces LED 2 will be turned on. 
* At the end of calibration the corresponding bit in the reg:POZYX_CALIB_STATUS register will be set. 
* The resulting coordinates are stored in the internal device list.
* \n\n
* Please read the tutorial Ready to Localize to learn how to use this function.
*
*   @param type dimension of the calibration, can be #POZYX_2D or #POZYX_2_5D
*   @param measurements: The number of measurements per link. Recommended 10. Theoretically, a larger number should result in better calibration accuracy.
*   @param anchor_num The number of anchors in the anchors[] array
*   @param anchors[] The anchors that determine the axis (see datasheet)       
*   @param heights The heights in mm of the anchors in the anchors[] array (only used for #POZYX_2_5D)
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*
* @see Please read the Ready to Localize tutorial to get started with this function.
*/
'''
def doAnchorCalibration(dimension = POZYX_2D, num_measurements = 10, anchors = None, heights = None):
	global _pozyx
	if anchors:
		num_anchors = len(anchors)
		anchorsPtr = (c_uint16 * num_anchors)(*anchors)
		heightsPtr = (c_int32 * num_anchors)(*heights)
	else:
		num_anchors = 0
		anchorsPtr = POINTER(c_uint16)()
		heightsPtr = POINTER(c_int32)()
	return int(_pozyx.doAnchorCalibration(c_int(dimension), c_int(num_measurements), c_int(num_anchors), anchors, heights))

'''
/**
* Empty the internal list of devices.
* This function empties the internal list of devices.
*
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*/
'''
def clearDevices(remote_id = -1):
	global _pozyx
	return int(_pozyx.clearDevices(c_uint16(remote_id)))

'''
/**
* Manualy adds a device to the device list.
* This function can be used to manually add a device and its coordinates to the device list. 
* Once the device is added, it can be used for positioning.
*
*   @param device_coordinates: the device information to be added
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*/
'''
def addDevice(device_coordinates, remote_id = -1):
	global _pozyx
	return int(_pozyx.addDevice(device_coordinates, c_uint16(remote_id)))

'''
/**
* Retrieve the stored coordinates of a device.
* This function retrieves the device coordinates stored in the internal device list.
*
*   @param device_id: device from which the information needs to be retrieved
*   @param device_coordinates: data object to store the information
*   @param remote_id: optional parameter that determines the remote device to be used
*
* @retval #POZYX_SUCCESS success.
* @retval #POZYX_FAILURE function failed.
*/
'''
def getDeviceCoordinates(device_id, remote_id = -1):
	global _pozyx
	coordinates = pointer(coordinates_t())
	status = int(_pozyx.getDeviceCoordinates(c_uint16(device_id), coordinates, c_uint16(remote_id)))
	return (status, coordinates.contents)
