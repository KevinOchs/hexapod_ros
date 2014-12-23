#ifndef PHIDGET_H
#define PHIDGET_H 

#ifdef __cplusplus
extern "C" {
#endif
 
#if defined(__stdcall)  
 #define CCONV __stdcall  	
#else 
 #if defined(__BORLANDC__) || defined(_MSC_VER) 
  #define CCONV __stdcall  
 #else 
  #define CCONV 
 #endif 
#endif 
 
#if !defined(__int64) 
#if !defined(__BORLANDC__) && !defined(_MSC_VER) 
typedef long long __int64; 
#endif 
#endif

 
typedef struct _CPhidget *CPhidgetHandle;
typedef struct _CPhidget_Timestamp {
 int seconds;
 int microseconds;
} CPhidget_Timestamp, *CPhidget_TimestampHandle;
typedef enum {
 PHIDCLASS_ACCELEROMETER = 2,
 PHIDCLASS_ADVANCEDSERVO = 3,
 PHIDCLASS_ANALOG = 22,
 PHIDCLASS_BRIDGE = 23,
 PHIDCLASS_ENCODER = 4,
 PHIDCLASS_FREQUENCYCOUNTER = 21,
 PHIDCLASS_GPS = 5,
 PHIDCLASS_INTERFACEKIT = 7,
 PHIDCLASS_IR = 19,
 PHIDCLASS_LED = 8,
 PHIDCLASS_MOTORCONTROL = 9,
 PHIDCLASS_PHSENSOR = 10,
 PHIDCLASS_RFID = 11,
 PHIDCLASS_SERVO = 12,
 PHIDCLASS_SPATIAL = 20,
 PHIDCLASS_STEPPER = 13,
 PHIDCLASS_TEMPERATURESENSOR = 14,
 PHIDCLASS_TEXTLCD = 15,
 PHIDCLASS_TEXTLED = 16,
 PHIDCLASS_WEIGHTSENSOR = 17,
} CPhidget_DeviceClass;
typedef enum {
 PHIDID_ACCELEROMETER_3AXIS = 0x07E,
 PHIDID_ADVANCEDSERVO_1MOTOR = 0x082,
 PHIDID_ADVANCEDSERVO_8MOTOR = 0x03A,
 PHIDID_ANALOG_4OUTPUT = 0x037,
 PHIDID_BIPOLAR_STEPPER_1MOTOR = 0x07B,
 PHIDID_BRIDGE_4INPUT = 0x03B,
 PHIDID_ENCODER_1ENCODER_1INPUT = 0x04B,
 PHIDID_ENCODER_HS_1ENCODER = 0x080,
 PHIDID_ENCODER_HS_4ENCODER_4INPUT = 0x04F,
 PHIDID_FREQUENCYCOUNTER_2INPUT = 0x035,
 PHIDID_GPS = 0x079,
 PHIDID_INTERFACEKIT_0_0_4 = 0x040,
 PHIDID_INTERFACEKIT_0_0_8 = 0x081,
 PHIDID_INTERFACEKIT_0_16_16 = 0x044,
 PHIDID_INTERFACEKIT_2_2_2 = 0x036,
 PHIDID_INTERFACEKIT_8_8_8 = 0x045,
 PHIDID_INTERFACEKIT_8_8_8_w_LCD = 0x07D,
 PHIDID_IR = 0x04D,
 PHIDID_LED_64_ADV = 0x04C,
 PHIDID_LINEAR_TOUCH = 0x076,
 PHIDID_MOTORCONTROL_1MOTOR = 0x03E,
 PHIDID_MOTORCONTROL_HC_2MOTOR = 0x059,
 PHIDID_RFID_2OUTPUT = 0x031,
 PHIDID_RFID_2OUTPUT_READ_WRITE = 0x034,
 PHIDID_ROTARY_TOUCH = 0x077,
 PHIDID_SPATIAL_ACCEL_3AXIS = 0x07F,
 PHIDID_SPATIAL_ACCEL_GYRO_COMPASS = 0x033,
 PHIDID_TEMPERATURESENSOR = 0x070,
 PHIDID_TEMPERATURESENSOR_4 = 0x032,
 PHIDID_TEMPERATURESENSOR_IR = 0x03C,
 PHIDID_TEXTLCD_2x20_w_8_8_8 = 0x17D,
 PHIDID_TEXTLCD_ADAPTER = 0x03D,
 PHIDID_UNIPOLAR_STEPPER_4MOTOR = 0x07A,
 PHIDID_ACCELEROMETER_2AXIS = 0x071,
 PHIDID_INTERFACEKIT_0_8_8_w_LCD = 0x053,
 PHIDID_INTERFACEKIT_4_8_8 = 4,
 PHIDID_LED_64 = 0x04A,
 PHIDID_MOTORCONTROL_LV_2MOTOR_4INPUT = 0x058,
 PHIDID_PHSENSOR = 0x074,
 PHIDID_RFID = 0x030,
 PHIDID_SERVO_1MOTOR = 0x039,
 PHIDID_SERVO_1MOTOR_OLD = 2,
 PHIDID_SERVO_4MOTOR = 0x038,
 PHIDID_SERVO_4MOTOR_OLD = 3,
 PHIDID_TEXTLCD_2x20 = 0x052,
 PHIDID_TEXTLCD_2x20_w_0_8_8 = 0x153,
 PHIDID_TEXTLED_1x8 = 0x049,
 PHIDID_TEXTLED_4x8 = 0x048,
 PHIDID_WEIGHTSENSOR = 0x072,
 PHIDID_FIRMWARE_UPGRADE = 0x098,
} CPhidget_DeviceID;
typedef enum {
 PHIDUID_NOTHING =1,
 PHIDUID_ACCELEROMETER_2AXIS_2G,
 PHIDUID_ACCELEROMETER_2AXIS_10G,
 PHIDUID_ACCELEROMETER_2AXIS_5G,
 PHIDUID_ACCELEROMETER_3AXIS_3G,
 PHIDUID_ADVANCEDSERVO_1MOTOR,
 PHIDUID_ADVANCEDSERVO_8MOTOR,
 PHIDUID_ADVANCEDSERVO_8MOTOR_PGOOD_FLAG,
 PHIDUID_ADVANCEDSERVO_8MOTOR_CURSENSE_FIX,
 PHIDUID_ANALOG_4OUTPUT,
 PHIDUID_BRIDGE_4INPUT,
 PHIDUID_ENCODER_1ENCODER_1INPUT_OLD,
 PHIDUID_ENCODER_1ENCODER_1INPUT_v1,
 PHIDUID_ENCODER_1ENCODER_1INPUT_v2,
 PHIDUID_ENCODER_HS_1ENCODER,
 PHIDUID_ENCODER_HS_4ENCODER_4INPUT,
 PHIDUID_FREQUENCYCOUNTER_2INPUT,
 PHIDUID_GPS,
 PHIDUID_INTERFACEKIT_0_0_4_NO_ECHO,
 PHIDUID_INTERFACEKIT_0_0_4,
 PHIDUID_INTERFACEKIT_0_0_8,
 PHIDUID_INTERFACEKIT_0_5_7,
 PHIDUID_INTERFACEKIT_0_8_8_w_LCD,
 PHIDUID_INTERFACEKIT_0_16_16_NO_ECHO,
 PHIDUID_INTERFACEKIT_0_16_16_BITBUG,
 PHIDUID_INTERFACEKIT_0_16_16,
 PHIDUID_INTERFACEKIT_2_2_2,
 PHIDUID_INTERFACEKIT_2_8_8,
 PHIDUID_INTERFACEKIT_4_8_8,
 PHIDUID_INTERFACEKIT_8_8_8_NO_ECHO,
 PHIDUID_INTERFACEKIT_8_8_8,
 PHIDUID_INTERFACEKIT_8_8_8_FAST,
 PHIDUID_INTERFACEKIT_8_8_8_w_LCD_NO_ECHO,
 PHIDUID_INTERFACEKIT_8_8_8_w_LCD,
 PHIDUID_INTERFACEKIT_8_8_8_w_LCD_FAST,
 PHIDUID_INTERFACEKIT_TOUCH_SLIDER,
 PHIDUID_INTERFACEKIT_TOUCH_ROTARY,
 PHIDUID_IR,
 PHIDUID_LED_64,
 PHIDUID_LED_64_ADV,
 PHIDUID_LED_64_ADV_M3,
 PHIDUID_MOTORCONTROL_1MOTOR,
 PHIDUID_MOTORCONTROL_HC_2MOTOR,
 PHIDUID_MOTORCONTROL_LV_2MOTOR_4INPUT,
 PHIDUID_PHSENSOR,
 PHIDUID_RFID_OLD,
 PHIDUID_RFID,
 PHIDUID_RFID_2OUTPUT_NO_ECHO,
 PHIDUID_RFID_2OUTPUT,
 PHIDUID_RFID_2OUTPUT_READ_WRITE,
 PHIDUID_SERVO_1MOTOR_OLD,
 PHIDUID_SERVO_4MOTOR_OLD,
 PHIDUID_SERVO_1MOTOR_NO_ECHO,
 PHIDUID_SERVO_1MOTOR,
 PHIDUID_SERVO_4MOTOR_NO_ECHO,
 PHIDUID_SERVO_4MOTOR,
 PHIDUID_SPATIAL_ACCEL_3AXIS_1049,
 PHIDUID_SPATIAL_ACCEL_3AXIS_1041,
 PHIDUID_SPATIAL_ACCEL_3AXIS_1043,
 PHIDUID_SPATIAL_ACCEL_GYRO_COMPASS_1056,
 PHIDUID_SPATIAL_ACCEL_GYRO_COMPASS_1056_NEG_GAIN,
 PHIDUID_SPATIAL_ACCEL_GYRO_COMPASS_1042,
 PHIDUID_SPATIAL_ACCEL_GYRO_COMPASS_1044,
 PHIDUID_STEPPER_BIPOLAR_1MOTOR,
 PHIDUID_STEPPER_BIPOLAR_1MOTOR_M3,
 PHIDUID_STEPPER_UNIPOLAR_4MOTOR,
 PHIDUID_TEMPERATURESENSOR_OLD,
 PHIDUID_TEMPERATURESENSOR,
 PHIDUID_TEMPERATURESENSOR_AD22100,
 PHIDUID_TEMPERATURESENSOR_TERMINAL_BLOCKS,
 PHIDUID_TEMPERATURESENSOR_4,
 PHIDUID_TEMPERATURESENSOR_IR,
 PHIDUID_TEXTLCD_2x20,
 PHIDUID_TEXTLCD_2x20_w_8_8_8,
 PHIDUID_TEXTLCD_2x20_w_8_8_8_BRIGHTNESS,
 PHIDUID_TEXTLCD_ADAPTER,
 PHIDUID_TEXTLED_1x8,
 PHIDUID_TEXTLED_4x8,
 PHIDUID_WEIGHTSENSOR,
 PHIDUID_GENERIC,
 PHIDUID_FIRMWARE_UPGRADE
} CPhidget_DeviceUID;
 int CPhidget_open(CPhidgetHandle phid, int serialNumber);
 int CPhidget_openLabel(CPhidgetHandle phid, const char *label);
 int CPhidget_close(CPhidgetHandle phid);
 int CPhidget_delete(CPhidgetHandle phid);
 int CPhidget_set_OnDetach_Handler(CPhidgetHandle phid, int( *fptr)(CPhidgetHandle phid, void *userPtr), void *userPtr);
 int CPhidget_set_OnAttach_Handler(CPhidgetHandle phid, int( *fptr)(CPhidgetHandle phid, void *userPtr), void *userPtr);
 int CPhidget_set_OnServerConnect_Handler(CPhidgetHandle phid, int ( *fptr)(CPhidgetHandle phid, void *userPtr), void *userPtr);
 int CPhidget_set_OnServerDisconnect_Handler(CPhidgetHandle phid, int ( *fptr)(CPhidgetHandle phid, void *userPtr), void *userPtr);
 int CPhidget_set_OnError_Handler(CPhidgetHandle phid, int( *fptr)(CPhidgetHandle phid, void *userPtr, int errorCode, const char *errorString), void *userPtr);
 int CPhidget_getDeviceName(CPhidgetHandle phid, const char **deviceName);
 int CPhidget_getSerialNumber(CPhidgetHandle phid, int *serialNumber);
 int CPhidget_getDeviceVersion(CPhidgetHandle phid, int *deviceVersion);
 int CPhidget_getDeviceStatus(CPhidgetHandle phid, int *deviceStatus);
 int CPhidget_getLibraryVersion(const char **libraryVersion);
 int CPhidget_getDeviceType(CPhidgetHandle phid, const char **deviceType);
 int CPhidget_getDeviceLabel(CPhidgetHandle phid, const char **deviceLabel);
 int CPhidget_setDeviceLabel(CPhidgetHandle phid, const char *deviceLabel);
 int CPhidget_getErrorDescription(int errorCode, const char **errorString);
 int CPhidget_waitForAttachment(CPhidgetHandle phid, int milliseconds);
 int CPhidget_getServerID(CPhidgetHandle phid, const char **serverID);
 int CPhidget_getServerAddress(CPhidgetHandle phid, const char **address, int *port);
 int CPhidget_getServerStatus(CPhidgetHandle phid, int *serverStatus);
 int CPhidget_getDeviceID(CPhidgetHandle phid, CPhidget_DeviceID *deviceID);
 int CPhidget_getDeviceClass(CPhidgetHandle phid, CPhidget_DeviceClass *deviceClass);
typedef enum {
 PHIDGET_DICTIONARY_VALUE_CHANGED = 1,
 PHIDGET_DICTIONARY_ENTRY_ADDED,
 PHIDGET_DICTIONARY_ENTRY_REMOVING,
 PHIDGET_DICTIONARY_CURRENT_VALUE
} CPhidgetDictionary_keyChangeReason;
typedef struct _CPhidgetDictionary *CPhidgetDictionaryHandle;
typedef struct _CPhidgetDictionaryListener *CPhidgetDictionaryListenerHandle;
 int CPhidgetDictionary_create(CPhidgetDictionaryHandle *dict);
 int CPhidgetDictionary_close(CPhidgetDictionaryHandle dict);
 int CPhidgetDictionary_delete(CPhidgetDictionaryHandle dict);
 int CPhidgetDictionary_set_OnError_Handler(CPhidgetDictionaryHandle dict,
    int( *fptr)(CPhidgetDictionaryHandle, void *userPtr, int errorCode, const char *errorString), void *userPtr);
 int CPhidgetDictionary_addKey(CPhidgetDictionaryHandle dict, const char *key, const char *value, int persistent);
 int CPhidgetDictionary_removeKey(CPhidgetDictionaryHandle dict, const char *pattern);
typedef int( *CPhidgetDictionary_OnKeyChange_Function)(CPhidgetDictionaryHandle dict, void *userPtr,
 const char *key, const char *value, CPhidgetDictionary_keyChangeReason reason);
 int CPhidgetDictionary_set_OnKeyChange_Handler(CPhidgetDictionaryHandle dict, CPhidgetDictionaryListenerHandle *dictlistener, const char *pattern,
 CPhidgetDictionary_OnKeyChange_Function fptr, void *userPtr);
 int CPhidgetDictionary_remove_OnKeyChange_Handler(CPhidgetDictionaryListenerHandle dictlistener);
 int CPhidgetDictionary_getKey(CPhidgetDictionaryHandle dict, const char *key, char *value, int valuelen);
 int CPhidgetDictionary_set_OnServerConnect_Handler(CPhidgetDictionaryHandle dict, int ( *fptr)(CPhidgetDictionaryHandle dict, void *userPtr), void *userPtr);
 int CPhidgetDictionary_set_OnServerDisconnect_Handler(CPhidgetDictionaryHandle dict, int ( *fptr)(CPhidgetDictionaryHandle dict, void *userPtr), void *userPtr);
 int CPhidgetDictionary_getServerID(CPhidgetDictionaryHandle dict, const char **serverID);
 int CPhidgetDictionary_getServerAddress(CPhidgetDictionaryHandle dict, const char **address, int *port);
 int CPhidgetDictionary_getServerStatus(CPhidgetDictionaryHandle dict, int *serverStatus);
typedef struct _CPhidgetManager *CPhidgetManagerHandle;
 int CPhidgetManager_create(CPhidgetManagerHandle *phidm);
 int CPhidgetManager_open(CPhidgetManagerHandle phidm);
 int CPhidgetManager_close(CPhidgetManagerHandle phidm);
 int CPhidgetManager_delete(CPhidgetManagerHandle phidm);
 int CPhidgetManager_set_OnAttach_Handler(CPhidgetManagerHandle phidm, int ( *fptr)(CPhidgetHandle phid, void *userPtr), void *userPtr);
 int CPhidgetManager_set_OnDetach_Handler(CPhidgetManagerHandle phidm, int ( *fptr)(CPhidgetHandle phid, void *userPtr), void *userPtr);
 int CPhidgetManager_getAttachedDevices(CPhidgetManagerHandle phidm, CPhidgetHandle *phidArray[], int *count);
 int CPhidgetManager_freeAttachedDevicesArray(CPhidgetHandle phidArray[]);
 int CPhidgetManager_set_OnError_Handler(CPhidgetManagerHandle phidm, int( *fptr)(CPhidgetManagerHandle phidm, void *userPtr, int errorCode, const char *errorString), void *userPtr);
 int CPhidgetManager_set_OnServerConnect_Handler(CPhidgetManagerHandle phidm, int ( *fptr)(CPhidgetManagerHandle phidm, void *userPtr), void *userPtr);
 int CPhidgetManager_set_OnServerDisconnect_Handler(CPhidgetManagerHandle phidm, int ( *fptr)(CPhidgetManagerHandle phidm, void *userPtr), void *userPtr);
 int CPhidgetManager_getServerID(CPhidgetManagerHandle phidm, const char **serverID);
 int CPhidgetManager_getServerAddress(CPhidgetManagerHandle phidm, const char **address, int *port);
 int CPhidgetManager_getServerStatus(CPhidgetManagerHandle phidm, int *serverStatus);
 int CPhidget_openRemote(CPhidgetHandle phid, int serial, const char *serverID, const char *password);
 int CPhidget_openLabelRemote(CPhidgetHandle phid, const char *label, const char *serverID, const char *password);
 int CPhidget_openRemoteIP(CPhidgetHandle phid, int serial, const char *address, int port, const char *password);
 int CPhidget_openLabelRemoteIP(CPhidgetHandle phid, const char *label, const char *address, int port, const char *password);
 int CPhidgetManager_openRemote(CPhidgetManagerHandle phidm, const char *serverID, const char *password);
 int CPhidgetManager_openRemoteIP(CPhidgetManagerHandle phidm, const char *address, int port, const char *password);
 int CPhidgetDictionary_openRemote(CPhidgetDictionaryHandle dict, const char *serverID, const char *password);
 int CPhidgetDictionary_openRemoteIP(CPhidgetDictionaryHandle dict, const char *address, int port, const char *password);
typedef enum {
 PHIDGET_LOG_CRITICAL = 1,
 PHIDGET_LOG_ERROR,
 PHIDGET_LOG_WARNING,
 PHIDGET_LOG_DEBUG,
 PHIDGET_LOG_INFO,
 PHIDGET_LOG_VERBOSE
} CPhidgetLog_level;
 int CPhidget_enableLogging(CPhidgetLog_level level, const char *outputFile);
 int CPhidget_disableLogging();
 int CPhidget_log(CPhidgetLog_level level, const char *id, const char *message, ...);
typedef struct _CPhidgetAccelerometer *CPhidgetAccelerometerHandle;
 int CPhidgetAccelerometer_create(CPhidgetAccelerometerHandle *phid);
 int CPhidgetAccelerometer_getAxisCount(CPhidgetAccelerometerHandle phid, int *count);
 int CPhidgetAccelerometer_getAcceleration(CPhidgetAccelerometerHandle phid, int index, double *acceleration);
 int CPhidgetAccelerometer_getAccelerationMax(CPhidgetAccelerometerHandle phid, int index, double *max);
 int CPhidgetAccelerometer_getAccelerationMin(CPhidgetAccelerometerHandle phid, int index, double *min);
 int CPhidgetAccelerometer_set_OnAccelerationChange_Handler(CPhidgetAccelerometerHandle phid, int ( *fptr)(CPhidgetAccelerometerHandle phid, void *userPtr, int index, double acceleration), void *userPtr);
 int CPhidgetAccelerometer_getAccelerationChangeTrigger(CPhidgetAccelerometerHandle phid, int index, double *trigger);
 int CPhidgetAccelerometer_setAccelerationChangeTrigger(CPhidgetAccelerometerHandle phid, int index, double trigger);
typedef struct _CPhidgetAdvancedServo *CPhidgetAdvancedServoHandle;
 int CPhidgetAdvancedServo_create(CPhidgetAdvancedServoHandle *phid);
typedef enum {
 PHIDGET_SERVO_DEFAULT = 1,
 PHIDGET_SERVO_RAW_us_MODE,
 PHIDGET_SERVO_HITEC_HS322HD,
 PHIDGET_SERVO_HITEC_HS5245MG,
 PHIDGET_SERVO_HITEC_805BB,
 PHIDGET_SERVO_HITEC_HS422,
 PHIDGET_SERVO_TOWERPRO_MG90,
 PHIDGET_SERVO_HITEC_HSR1425CR,
 PHIDGET_SERVO_HITEC_HS785HB,
 PHIDGET_SERVO_HITEC_HS485HB,
 PHIDGET_SERVO_HITEC_HS645MG,
 PHIDGET_SERVO_HITEC_815BB,
 PHIDGET_SERVO_FIRGELLI_L12_30_50_06_R,
 PHIDGET_SERVO_FIRGELLI_L12_50_100_06_R,
 PHIDGET_SERVO_FIRGELLI_L12_50_210_06_R,
 PHIDGET_SERVO_FIRGELLI_L12_100_50_06_R,
 PHIDGET_SERVO_FIRGELLI_L12_100_100_06_R,
 PHIDGET_SERVO_SPRINGRC_SM_S2313M,
 PHIDGET_SERVO_SPRINGRC_SM_S3317M,
 PHIDGET_SERVO_SPRINGRC_SM_S3317SR,
 PHIDGET_SERVO_SPRINGRC_SM_S4303R,
 PHIDGET_SERVO_SPRINGRC_SM_S4315M,
 PHIDGET_SERVO_SPRINGRC_SM_S4315R,
 PHIDGET_SERVO_SPRINGRC_SM_S4505B,
 PHIDGET_SERVO_USER_DEFINED
} CPhidget_ServoType;
 int CPhidgetAdvancedServo_getMotorCount(CPhidgetAdvancedServoHandle phid, int *count);
 int CPhidgetAdvancedServo_getAcceleration(CPhidgetAdvancedServoHandle phid, int index, double *acceleration);
 int CPhidgetAdvancedServo_setAcceleration(CPhidgetAdvancedServoHandle phid, int index, double acceleration);
 int CPhidgetAdvancedServo_getAccelerationMax(CPhidgetAdvancedServoHandle phid, int index, double *max);
 int CPhidgetAdvancedServo_getAccelerationMin(CPhidgetAdvancedServoHandle phid, int index, double *min);
 int CPhidgetAdvancedServo_getVelocityLimit(CPhidgetAdvancedServoHandle phid, int index, double *limit);
 int CPhidgetAdvancedServo_setVelocityLimit(CPhidgetAdvancedServoHandle phid, int index, double limit);
 int CPhidgetAdvancedServo_getVelocity(CPhidgetAdvancedServoHandle phid, int index, double *velocity);
 int CPhidgetAdvancedServo_getVelocityMax(CPhidgetAdvancedServoHandle phid, int index, double *max);
 int CPhidgetAdvancedServo_getVelocityMin(CPhidgetAdvancedServoHandle phid, int index, double *min);
 int CPhidgetAdvancedServo_set_OnVelocityChange_Handler(CPhidgetAdvancedServoHandle phid, int ( *fptr)(CPhidgetAdvancedServoHandle phid, void *userPtr, int index, double velocity), void *userPtr);
 int CPhidgetAdvancedServo_getPosition(CPhidgetAdvancedServoHandle phid, int index, double *position);
 int CPhidgetAdvancedServo_setPosition(CPhidgetAdvancedServoHandle phid, int index, double position);
 int CPhidgetAdvancedServo_getPositionMax(CPhidgetAdvancedServoHandle phid, int index, double *max);
 int CPhidgetAdvancedServo_setPositionMax(CPhidgetAdvancedServoHandle phid, int index, double max);
 int CPhidgetAdvancedServo_getPositionMin(CPhidgetAdvancedServoHandle phid, int index, double *min);
 int CPhidgetAdvancedServo_setPositionMin(CPhidgetAdvancedServoHandle phid, int index, double min);
 int CPhidgetAdvancedServo_set_OnPositionChange_Handler(CPhidgetAdvancedServoHandle phid, int ( *fptr)(CPhidgetAdvancedServoHandle phid, void *userPtr, int index, double position), void *userPtr);
 int CPhidgetAdvancedServo_getCurrent(CPhidgetAdvancedServoHandle phid, int index, double *current);
 int CPhidgetAdvancedServo_set_OnCurrentChange_Handler(CPhidgetAdvancedServoHandle phid, int ( *fptr)(CPhidgetAdvancedServoHandle phid, void *userPtr, int index, double current), void *userPtr);
 int CPhidgetAdvancedServo_getSpeedRampingOn(CPhidgetAdvancedServoHandle phid, int index, int *rampingState);
 int CPhidgetAdvancedServo_setSpeedRampingOn(CPhidgetAdvancedServoHandle phid, int index, int rampingState);
 int CPhidgetAdvancedServo_getEngaged(CPhidgetAdvancedServoHandle phid, int index, int *engagedState);
 int CPhidgetAdvancedServo_setEngaged(CPhidgetAdvancedServoHandle phid, int index, int engagedState);
 int CPhidgetAdvancedServo_getStopped(CPhidgetAdvancedServoHandle phid, int index, int *stoppedState);
 int CPhidgetAdvancedServo_getServoType(CPhidgetAdvancedServoHandle phid, int index, CPhidget_ServoType *servoType);
 int CPhidgetAdvancedServo_setServoType(CPhidgetAdvancedServoHandle phid, int index, CPhidget_ServoType servoType);
 int CPhidgetAdvancedServo_setServoParameters(CPhidgetAdvancedServoHandle phid, int index, double min_us,double max_us,double degrees,double velocity_max);
typedef struct _CPhidgetAnalog *CPhidgetAnalogHandle;
 int CPhidgetAnalog_create(CPhidgetAnalogHandle *phid);
 int CPhidgetAnalog_getOutputCount(CPhidgetAnalogHandle phid, int *count);
 int CPhidgetAnalog_getVoltage(CPhidgetAnalogHandle phid, int index, double *voltage);
 int CPhidgetAnalog_setVoltage(CPhidgetAnalogHandle phid, int index, double voltage);
 int CPhidgetAnalog_getVoltageMax(CPhidgetAnalogHandle phid, int index, double *max);
 int CPhidgetAnalog_getVoltageMin(CPhidgetAnalogHandle phid, int index, double *min);
 int CPhidgetAnalog_setEnabled(CPhidgetAnalogHandle phid, int index, int enabledState);
 int CPhidgetAnalog_getEnabled(CPhidgetAnalogHandle phid, int index, int *enabledState);
typedef enum {
 PHIDGET_BRIDGE_GAIN_1 = 1,
 PHIDGET_BRIDGE_GAIN_8,
 PHIDGET_BRIDGE_GAIN_16,
 PHIDGET_BRIDGE_GAIN_32,
 PHIDGET_BRIDGE_GAIN_64,
 PHIDGET_BRIDGE_GAIN_128,
 PHIDGET_BRIDGE_GAIN_UNKNOWN
} CPhidgetBridge_Gain;
typedef struct _CPhidgetBridge *CPhidgetBridgeHandle;
 int CPhidgetBridge_create(CPhidgetBridgeHandle *phid);
 int CPhidgetBridge_getInputCount(CPhidgetBridgeHandle phid, int *count);
 int CPhidgetBridge_getBridgeValue(CPhidgetBridgeHandle phid, int index, double *value);
 int CPhidgetBridge_getBridgeMax(CPhidgetBridgeHandle phid, int index, double *max);
 int CPhidgetBridge_getBridgeMin(CPhidgetBridgeHandle phid, int index, double *min);
 int CPhidgetBridge_setEnabled(CPhidgetBridgeHandle phid, int index, int enabledState);
 int CPhidgetBridge_getEnabled(CPhidgetBridgeHandle phid, int index, int *enabledState);
 int CPhidgetBridge_getGain(CPhidgetBridgeHandle phid, int index, CPhidgetBridge_Gain *gain);
 int CPhidgetBridge_setGain(CPhidgetBridgeHandle phid, int index, CPhidgetBridge_Gain gain);
 int CPhidgetBridge_getDataRate(CPhidgetBridgeHandle phid, int *milliseconds);
 int CPhidgetBridge_setDataRate(CPhidgetBridgeHandle phid, int milliseconds);
 int CPhidgetBridge_getDataRateMax(CPhidgetBridgeHandle phid, int *max);
 int CPhidgetBridge_getDataRateMin(CPhidgetBridgeHandle phid, int *min);
 int CPhidgetBridge_set_OnBridgeData_Handler(CPhidgetBridgeHandle phid, int ( *fptr)(CPhidgetBridgeHandle phid, void *userPtr, int index, double value), void *userPtr);
typedef struct _CPhidgetEncoder *CPhidgetEncoderHandle;
 int CPhidgetEncoder_create(CPhidgetEncoderHandle *phid);
 int CPhidgetEncoder_getInputCount(CPhidgetEncoderHandle phid, int *count);
 int CPhidgetEncoder_getInputState(CPhidgetEncoderHandle phid, int index, int *inputState);
 int CPhidgetEncoder_set_OnInputChange_Handler(CPhidgetEncoderHandle phid, int ( *fptr)(CPhidgetEncoderHandle phid, void *userPtr, int index, int inputState), void *userPtr);
 int CPhidgetEncoder_getEncoderCount(CPhidgetEncoderHandle phid, int *count);
 int CPhidgetEncoder_getPosition(CPhidgetEncoderHandle phid, int index, int *position);
 int CPhidgetEncoder_setPosition(CPhidgetEncoderHandle phid, int index, int position);
 int CPhidgetEncoder_set_OnPositionChange_Handler(CPhidgetEncoderHandle phid, int ( *fptr)(CPhidgetEncoderHandle phid, void *userPtr, int index, int time,int positionChange), void *userPtr);
 int CPhidgetEncoder_set_OnIndex_Handler(CPhidgetEncoderHandle phid, int ( *fptr)(CPhidgetEncoderHandle phid, void *userPtr, int index, int indexPosition), void *userPtr);
 int CPhidgetEncoder_getIndexPosition(CPhidgetEncoderHandle phid, int index, int *position);
 int CPhidgetEncoder_getEnabled(CPhidgetEncoderHandle phid, int index, int *enabledState);
 int CPhidgetEncoder_setEnabled(CPhidgetEncoderHandle phid, int index, int enabledState);
typedef enum {
 PHIDGET_FREQUENCYCOUNTER_FILTERTYPE_ZERO_CROSSING = 1,
 PHIDGET_FREQUENCYCOUNTER_FILTERTYPE_LOGIC_LEVEL,
 PHIDGET_FREQUENCYCOUNTER_FILTERTYPE_UNKNOWN
} CPhidgetFrequencyCounter_FilterType;
typedef struct _CPhidgetFrequencyCounter *CPhidgetFrequencyCounterHandle;
 int CPhidgetFrequencyCounter_create(CPhidgetFrequencyCounterHandle *phid);
 int CPhidgetFrequencyCounter_getFrequencyInputCount(CPhidgetFrequencyCounterHandle phid, int *count);
 int CPhidgetFrequencyCounter_getFrequency(CPhidgetFrequencyCounterHandle phid, int index, double *frequency);
 int CPhidgetFrequencyCounter_getTotalTime(CPhidgetFrequencyCounterHandle phid, int index, __int64 *time);
 int CPhidgetFrequencyCounter_getTotalCount(CPhidgetFrequencyCounterHandle phid, int index, __int64 *count);
 int CPhidgetFrequencyCounter_setTimeout(CPhidgetFrequencyCounterHandle phid, int index, int timeout);
 int CPhidgetFrequencyCounter_getTimeout(CPhidgetFrequencyCounterHandle phid, int index, int *timeout);
 int CPhidgetFrequencyCounter_setEnabled(CPhidgetFrequencyCounterHandle phid, int index, int enabledState);
 int CPhidgetFrequencyCounter_getEnabled(CPhidgetFrequencyCounterHandle phid, int index, int *enabledState);
 int CPhidgetFrequencyCounter_setFilter(CPhidgetFrequencyCounterHandle phid, int index, CPhidgetFrequencyCounter_FilterType filter);
 int CPhidgetFrequencyCounter_getFilter(CPhidgetFrequencyCounterHandle phid, int index, CPhidgetFrequencyCounter_FilterType *filter);
 int CPhidgetFrequencyCounter_reset(CPhidgetFrequencyCounterHandle phid, int index);
 int CPhidgetFrequencyCounter_set_OnCount_Handler(CPhidgetFrequencyCounterHandle phid, int ( *fptr)(CPhidgetFrequencyCounterHandle phid, void *userPtr, int index, int time,int counts), void *userPtr);
struct __GPSTime
{
 short tm_ms;
 short tm_sec;
 short tm_min;
 short tm_hour;
} typedef GPSTime;
struct __GPSDate
{
 short tm_mday;
 short tm_mon;
 short tm_year;
} typedef GPSDate;
struct __GPSSatInfo
{
 short ID;
 short elevation;
 int azimuth;
 short SNR;
} typedef GPSSatInfo;
struct __GPGGA
{
 GPSTime time;
 double latitude;
 double longitude;
 short fixQuality;
 short numSatellites;
 double horizontalDilution;
 double altitude;
 double heightOfGeoid;
} typedef GPGGA;
struct __GPGSA
{
 char mode;
 short fixType;
 short satUsed[12];
 double posnDilution;
 double horizDilution;
 double vertDilution;
} typedef GPGSA;
struct __GPGSV
{
 short satsInView;
 GPSSatInfo satInfo[12];
} typedef GPGSV;
struct __GPRMC
{
 GPSTime time;
 char status;
 double latitude;
 double longitude;
 double speedKnots;
 double heading;
 GPSDate date;
 double magneticVariation;
 char mode;
} typedef GPRMC;
struct __GPVTG
{
 double trueHeading;
 double magneticHeading;
 double speedKnots;
 double speed;
 char mode;
} typedef GPVTG;
struct __NMEAData
{
 GPGGA GGA;
 GPGSA GSA;
 GPGSV GSV;
 GPRMC RMC;
 GPVTG VTG;
} typedef NMEAData;
typedef struct _CPhidgetGPS *CPhidgetGPSHandle;
 int CPhidgetGPS_create(CPhidgetGPSHandle *phid);
 int CPhidgetGPS_getLatitude(CPhidgetGPSHandle phid, double *latitude);
 int CPhidgetGPS_getLongitude(CPhidgetGPSHandle phid, double *longitude);
 int CPhidgetGPS_getAltitude(CPhidgetGPSHandle phid, double *altitude);
 int CPhidgetGPS_getHeading(CPhidgetGPSHandle phid, double *heading);
 int CPhidgetGPS_getVelocity(CPhidgetGPSHandle phid, double *velocity);
 int CPhidgetGPS_getTime(CPhidgetGPSHandle phid, GPSTime *time);
 int CPhidgetGPS_getDate(CPhidgetGPSHandle phid, GPSDate *date);
 int CPhidgetGPS_getPositionFixStatus(CPhidgetGPSHandle phid, int *fixStatus);
 int CPhidgetGPS_getNMEAData(CPhidgetGPSHandle phid, NMEAData *data);
 int CPhidgetGPS_set_OnPositionChange_Handler(CPhidgetGPSHandle phid, int ( *fptr)(CPhidgetGPSHandle phid, void *userPtr, double latitude,double longitude,double altitude), void *userPtr);
 int CPhidgetGPS_set_OnPositionFixStatusChange_Handler(CPhidgetGPSHandle phid, int ( *fptr)(CPhidgetGPSHandle phid, void *userPtr, int status), void *userPtr);
typedef struct _CPhidgetInterfaceKit *CPhidgetInterfaceKitHandle;
 int CPhidgetInterfaceKit_create(CPhidgetInterfaceKitHandle *phid);
 int CPhidgetInterfaceKit_getInputCount(CPhidgetInterfaceKitHandle phid, int *count);
 int CPhidgetInterfaceKit_getInputState(CPhidgetInterfaceKitHandle phid, int index, int *inputState);
 int CPhidgetInterfaceKit_set_OnInputChange_Handler(CPhidgetInterfaceKitHandle phid, int ( *fptr)(CPhidgetInterfaceKitHandle phid, void *userPtr, int index, int inputState), void *userPtr);
 int CPhidgetInterfaceKit_getOutputCount(CPhidgetInterfaceKitHandle phid, int *count);
 int CPhidgetInterfaceKit_getOutputState(CPhidgetInterfaceKitHandle phid, int index, int *outputState);
 int CPhidgetInterfaceKit_setOutputState(CPhidgetInterfaceKitHandle phid, int index, int outputState);
 int CPhidgetInterfaceKit_set_OnOutputChange_Handler(CPhidgetInterfaceKitHandle phid, int ( *fptr)(CPhidgetInterfaceKitHandle phid, void *userPtr, int index, int outputState), void *userPtr);
 int CPhidgetInterfaceKit_getSensorCount(CPhidgetInterfaceKitHandle phid, int *count);
 int CPhidgetInterfaceKit_getSensorValue(CPhidgetInterfaceKitHandle phid, int index, int *sensorValue);
 int CPhidgetInterfaceKit_getSensorRawValue(CPhidgetInterfaceKitHandle phid, int index, int *sensorRawValue);
 int CPhidgetInterfaceKit_set_OnSensorChange_Handler(CPhidgetInterfaceKitHandle phid, int ( *fptr)(CPhidgetInterfaceKitHandle phid, void *userPtr, int index, int sensorValue), void *userPtr);
 int CPhidgetInterfaceKit_getSensorChangeTrigger(CPhidgetInterfaceKitHandle phid, int index, int *trigger);
 int CPhidgetInterfaceKit_setSensorChangeTrigger(CPhidgetInterfaceKitHandle phid, int index, int trigger);
 int CPhidgetInterfaceKit_getRatiometric(CPhidgetInterfaceKitHandle phid, int *ratiometric);
 int CPhidgetInterfaceKit_setRatiometric(CPhidgetInterfaceKitHandle phid, int ratiometric);
 int CPhidgetInterfaceKit_getDataRate(CPhidgetInterfaceKitHandle phid, int index, int *milliseconds);
 int CPhidgetInterfaceKit_setDataRate(CPhidgetInterfaceKitHandle phid, int index, int milliseconds);
 int CPhidgetInterfaceKit_getDataRateMax(CPhidgetInterfaceKitHandle phid, int index, int *max);
 int CPhidgetInterfaceKit_getDataRateMin(CPhidgetInterfaceKitHandle phid, int index, int *min);
typedef struct _CPhidgetIR *CPhidgetIRHandle;
 int CPhidgetIR_create(CPhidgetIRHandle *phid);
typedef enum {
 PHIDGET_IR_ENCODING_UNKNOWN = 1,
 PHIDGET_IR_ENCODING_SPACE,
 PHIDGET_IR_ENCODING_PULSE,
 PHIDGET_IR_ENCODING_BIPHASE,
 PHIDGET_IR_ENCODING_RC5,
 PHIDGET_IR_ENCODING_RC6
} CPhidgetIR_Encoding;
typedef enum {
 PHIDGET_IR_LENGTH_UNKNOWN = 1,
 PHIDGET_IR_LENGTH_CONSTANT,
 PHIDGET_IR_LENGTH_VARIABLE
} CPhidgetIR_Length;
typedef struct _CPhidgetIR_CodeInfo
{
 int bitCount;
 CPhidgetIR_Encoding encoding;
 CPhidgetIR_Length length;
 int gap;
 int trail;
 int header[2];
 int one[2];
 int zero[2];
 int repeat[26];
 int min_repeat;
 unsigned char toggle_mask[(128 / 8)];
 int carrierFrequency;
 int dutyCycle;
} CPhidgetIR_CodeInfo, *CPhidgetIR_CodeInfoHandle;
 int CPhidgetIR_Transmit(CPhidgetIRHandle phid, unsigned char *data, CPhidgetIR_CodeInfoHandle codeInfo);
 int CPhidgetIR_TransmitRepeat(CPhidgetIRHandle phid);
 int CPhidgetIR_TransmitRaw(CPhidgetIRHandle phid, int *data, int length, int carrierFrequency, int dutyCycle, int gap);
 int CPhidgetIR_getRawData(CPhidgetIRHandle phid, int *data, int *dataLength);
 int CPhidgetIR_getLastCode(CPhidgetIRHandle phid, unsigned char *data, int *dataLength, int *bitCount);
 int CPhidgetIR_getLastLearnedCode(CPhidgetIRHandle phid, unsigned char *data, int *dataLength, CPhidgetIR_CodeInfo *codeInfo);
 int CPhidgetIR_set_OnCode_Handler(CPhidgetIRHandle phid, int ( *fptr)(CPhidgetIRHandle phid, void *userPtr, unsigned char *data, int dataLength, int bitCount, int repeat), void *userPtr);
 int CPhidgetIR_set_OnLearn_Handler(CPhidgetIRHandle phid, int ( *fptr)(CPhidgetIRHandle phid, void *userPtr, unsigned char *data, int dataLength, CPhidgetIR_CodeInfoHandle codeInfo), void *userPtr);
 int CPhidgetIR_set_OnRawData_Handler(CPhidgetIRHandle phid, int ( *fptr)(CPhidgetIRHandle phid, void *userPtr, int *data, int dataLength), void *userPtr);
typedef struct _CPhidgetLED *CPhidgetLEDHandle;
 int CPhidgetLED_create(CPhidgetLEDHandle *phid);
typedef enum {
 PHIDGET_LED_CURRENT_LIMIT_20mA = 1,
 PHIDGET_LED_CURRENT_LIMIT_40mA,
 PHIDGET_LED_CURRENT_LIMIT_60mA,
 PHIDGET_LED_CURRENT_LIMIT_80mA
} CPhidgetLED_CurrentLimit;
typedef enum {
 PHIDGET_LED_VOLTAGE_1_7V = 1,
 PHIDGET_LED_VOLTAGE_2_75V,
 PHIDGET_LED_VOLTAGE_3_9V,
 PHIDGET_LED_VOLTAGE_5_0V
} CPhidgetLED_Voltage;
 int CPhidgetLED_getLEDCount(CPhidgetLEDHandle phid, int *count);
 int CPhidgetLED_getCurrentLimit(CPhidgetLEDHandle phid, CPhidgetLED_CurrentLimit *currentLimit);
 int CPhidgetLED_setCurrentLimit(CPhidgetLEDHandle phid, CPhidgetLED_CurrentLimit currentLimit);
 int CPhidgetLED_getVoltage(CPhidgetLEDHandle phid, CPhidgetLED_Voltage *voltage);
 int CPhidgetLED_setVoltage(CPhidgetLEDHandle phid, CPhidgetLED_Voltage voltage);
 int CPhidgetLED_getBrightness(CPhidgetLEDHandle phid, int index, double *brightness);
 int CPhidgetLED_setBrightness(CPhidgetLEDHandle phid, int index, double brightness);
 int CPhidgetLED_getCurrentLimitIndexed(CPhidgetLEDHandle phid, int index, double *limit);
 int CPhidgetLED_setCurrentLimitIndexed(CPhidgetLEDHandle phid, int index, double limit);
typedef struct _CPhidgetMotorControl *CPhidgetMotorControlHandle;
 int CPhidgetMotorControl_create(CPhidgetMotorControlHandle *phid);
 int CPhidgetMotorControl_getMotorCount(CPhidgetMotorControlHandle phid, int *count);
 int CPhidgetMotorControl_getVelocity(CPhidgetMotorControlHandle phid, int index, double *velocity);
 int CPhidgetMotorControl_setVelocity(CPhidgetMotorControlHandle phid, int index, double velocity);
 int CPhidgetMotorControl_set_OnVelocityChange_Handler(CPhidgetMotorControlHandle phid, int ( *fptr)(CPhidgetMotorControlHandle phid, void *userPtr, int index, double velocity), void *userPtr);
 int CPhidgetMotorControl_getAcceleration(CPhidgetMotorControlHandle phid, int index, double *acceleration);
 int CPhidgetMotorControl_setAcceleration(CPhidgetMotorControlHandle phid, int index, double acceleration);
 int CPhidgetMotorControl_getAccelerationMax(CPhidgetMotorControlHandle phid, int index, double *max);
 int CPhidgetMotorControl_getAccelerationMin(CPhidgetMotorControlHandle phid, int index, double *min);
 int CPhidgetMotorControl_getCurrent(CPhidgetMotorControlHandle phid, int index, double *current);
 int CPhidgetMotorControl_set_OnCurrentChange_Handler(CPhidgetMotorControlHandle phid, int ( *fptr)(CPhidgetMotorControlHandle phid, void *userPtr, int index, double current), void *userPtr);
 int CPhidgetMotorControl_getInputCount(CPhidgetMotorControlHandle phid, int *count);
 int CPhidgetMotorControl_getInputState(CPhidgetMotorControlHandle phid, int index, int *inputState);
 int CPhidgetMotorControl_set_OnInputChange_Handler(CPhidgetMotorControlHandle phid, int ( *fptr)(CPhidgetMotorControlHandle phid, void *userPtr, int index, int inputState), void *userPtr);
 int CPhidgetMotorControl_getEncoderCount(CPhidgetMotorControlHandle phid, int *count);
 int CPhidgetMotorControl_getEncoderPosition(CPhidgetMotorControlHandle phid, int index, int *position);
 int CPhidgetMotorControl_setEncoderPosition(CPhidgetMotorControlHandle phid, int index, int position);
 int CPhidgetMotorControl_set_OnEncoderPositionChange_Handler(CPhidgetMotorControlHandle phid, int ( *fptr)(CPhidgetMotorControlHandle phid, void *userPtr, int index, int time,int positionChange), void *userPtr);
 int CPhidgetMotorControl_set_OnEncoderPositionUpdate_Handler(CPhidgetMotorControlHandle phid, int ( *fptr)(CPhidgetMotorControlHandle phid, void *userPtr, int index, int positionChange), void *userPtr);
 int CPhidgetMotorControl_getBackEMFSensingState(CPhidgetMotorControlHandle phid, int index, int *bEMFState);
 int CPhidgetMotorControl_setBackEMFSensingState(CPhidgetMotorControlHandle phid, int index, int bEMFState);
 int CPhidgetMotorControl_getBackEMF(CPhidgetMotorControlHandle phid, int index, double *voltage);
 int CPhidgetMotorControl_set_OnBackEMFUpdate_Handler(CPhidgetMotorControlHandle phid, int ( *fptr)(CPhidgetMotorControlHandle phid, void *userPtr, int index, double voltage), void *userPtr);
 int CPhidgetMotorControl_getSupplyVoltage(CPhidgetMotorControlHandle phid, double *supplyVoltage);
 int CPhidgetMotorControl_getBraking(CPhidgetMotorControlHandle phid, int index, double *braking);
 int CPhidgetMotorControl_setBraking(CPhidgetMotorControlHandle phid, int index, double braking);
 int CPhidgetMotorControl_getSensorCount(CPhidgetMotorControlHandle phid, int *count);
 int CPhidgetMotorControl_getSensorValue(CPhidgetMotorControlHandle phid, int index, int *sensorValue);
 int CPhidgetMotorControl_getSensorRawValue(CPhidgetMotorControlHandle phid, int index, int *sensorRawValue);
 int CPhidgetMotorControl_set_OnSensorUpdate_Handler(CPhidgetMotorControlHandle phid, int ( *fptr)(CPhidgetMotorControlHandle phid, void *userPtr, int index, int sensorValue), void *userPtr);
 int CPhidgetMotorControl_getRatiometric(CPhidgetMotorControlHandle phid, int *ratiometric);
 int CPhidgetMotorControl_setRatiometric(CPhidgetMotorControlHandle phid, int ratiometric);
 int CPhidgetMotorControl_set_OnCurrentUpdate_Handler(CPhidgetMotorControlHandle phid, int ( *fptr)(CPhidgetMotorControlHandle phid, void *userPtr, int index, double current), void *userPtr);
typedef struct _CPhidgetPHSensor *CPhidgetPHSensorHandle;
 int CPhidgetPHSensor_create(CPhidgetPHSensorHandle *phid);
 int CPhidgetPHSensor_getPH(CPhidgetPHSensorHandle phid, double *ph);
 int CPhidgetPHSensor_getPHMax(CPhidgetPHSensorHandle phid, double *max);
 int CPhidgetPHSensor_getPHMin(CPhidgetPHSensorHandle phid, double *min);
 int CPhidgetPHSensor_set_OnPHChange_Handler(CPhidgetPHSensorHandle phid, int ( *fptr)(CPhidgetPHSensorHandle phid, void *userPtr, double ph), void *userPtr);
 int CPhidgetPHSensor_getPHChangeTrigger(CPhidgetPHSensorHandle phid, double *trigger);
 int CPhidgetPHSensor_setPHChangeTrigger(CPhidgetPHSensorHandle phid, double trigger);
 int CPhidgetPHSensor_getPotential(CPhidgetPHSensorHandle phid, double *potential);
 int CPhidgetPHSensor_getPotentialMax(CPhidgetPHSensorHandle phid, double *max);
 int CPhidgetPHSensor_getPotentialMin(CPhidgetPHSensorHandle phid, double *min);
 int CPhidgetPHSensor_setTemperature(CPhidgetPHSensorHandle phid, double temperature);
typedef struct _CPhidgetRFID *CPhidgetRFIDHandle;
 int CPhidgetRFID_create(CPhidgetRFIDHandle *phid);
typedef enum {
 PHIDGET_RFID_PROTOCOL_EM4100 = 1,
 PHIDGET_RFID_PROTOCOL_ISO11785_FDX_B,
 PHIDGET_RFID_PROTOCOL_PHIDGETS,
} CPhidgetRFID_Protocol;
 int CPhidgetRFID_getOutputCount(CPhidgetRFIDHandle phid, int *count);
 int CPhidgetRFID_getOutputState(CPhidgetRFIDHandle phid, int index, int *outputState);
 int CPhidgetRFID_setOutputState(CPhidgetRFIDHandle phid, int index, int outputState);
 int CPhidgetRFID_set_OnOutputChange_Handler(CPhidgetRFIDHandle phid, int ( *fptr)(CPhidgetRFIDHandle phid, void *userPtr, int index, int outputState), void *userPtr);
 int CPhidgetRFID_getAntennaOn(CPhidgetRFIDHandle phid, int *antennaState);
 int CPhidgetRFID_setAntennaOn(CPhidgetRFIDHandle phid, int antennaState);
 int CPhidgetRFID_getLEDOn(CPhidgetRFIDHandle phid, int *LEDState);
 int CPhidgetRFID_setLEDOn(CPhidgetRFIDHandle phid, int LEDState);
 int CPhidgetRFID_getLastTag2(CPhidgetRFIDHandle phid, char **tagString, CPhidgetRFID_Protocol *protocol);
 int CPhidgetRFID_getTagStatus(CPhidgetRFIDHandle phid, int *status);
 int CPhidgetRFID_write(CPhidgetRFIDHandle phid, char *tagString, CPhidgetRFID_Protocol protocol, int lock);
 int CPhidgetRFID_set_OnTag2_Handler(CPhidgetRFIDHandle phid, int ( *fptr)(CPhidgetRFIDHandle phid, void *userPtr, char *tagString, CPhidgetRFID_Protocol protocol), void *userPtr);
 int CPhidgetRFID_set_OnTagLost2_Handler(CPhidgetRFIDHandle phid, int ( *fptr)(CPhidgetRFIDHandle phid, void *userPtr, char *tagString, CPhidgetRFID_Protocol protocol), void *userPtr);
typedef struct _CPhidgetServo *CPhidgetServoHandle;
 int CPhidgetServo_create(CPhidgetServoHandle *phid);
 int CPhidgetServo_getMotorCount(CPhidgetServoHandle phid, int *count);
 int CPhidgetServo_getPosition(CPhidgetServoHandle phid, int index, double *position);
 int CPhidgetServo_setPosition(CPhidgetServoHandle phid, int index, double position);
 int CPhidgetServo_getPositionMax(CPhidgetServoHandle phid, int index, double *max);
 int CPhidgetServo_getPositionMin(CPhidgetServoHandle phid, int index, double *min);
 int CPhidgetServo_set_OnPositionChange_Handler(CPhidgetServoHandle phid, int ( *fptr)(CPhidgetServoHandle phid, void *userPtr, int index, double position), void *userPtr);
 int CPhidgetServo_getEngaged(CPhidgetServoHandle phid, int index, int *engagedState);
 int CPhidgetServo_setEngaged(CPhidgetServoHandle phid, int index, int engagedState);
 int CPhidgetServo_getServoType(CPhidgetServoHandle phid, int index, CPhidget_ServoType *servoType);
 int CPhidgetServo_setServoType(CPhidgetServoHandle phid, int index, CPhidget_ServoType servoType);
 int CPhidgetServo_setServoParameters(CPhidgetServoHandle phid, int index, double min_us,double max_us,double degrees);
typedef struct _CPhidgetSpatial *CPhidgetSpatialHandle;
 int CPhidgetSpatial_create(CPhidgetSpatialHandle *phid);
typedef struct _CPhidgetSpatial_SpatialEventData
{
 double acceleration[3];
 double angularRate[3];
 double magneticField[3];
 CPhidget_Timestamp timestamp;
} CPhidgetSpatial_SpatialEventData, *CPhidgetSpatial_SpatialEventDataHandle;
 int CPhidgetSpatial_getAccelerationAxisCount(CPhidgetSpatialHandle phid, int *count);
 int CPhidgetSpatial_getGyroAxisCount(CPhidgetSpatialHandle phid, int *count);
 int CPhidgetSpatial_getCompassAxisCount(CPhidgetSpatialHandle phid, int *count);
 int CPhidgetSpatial_getAcceleration(CPhidgetSpatialHandle phid, int index, double *acceleration);
 int CPhidgetSpatial_getAccelerationMax(CPhidgetSpatialHandle phid, int index, double *max);
 int CPhidgetSpatial_getAccelerationMin(CPhidgetSpatialHandle phid, int index, double *min);
 int CPhidgetSpatial_getAngularRate(CPhidgetSpatialHandle phid, int index, double *angularRate);
 int CPhidgetSpatial_getAngularRateMax(CPhidgetSpatialHandle phid, int index, double *max);
 int CPhidgetSpatial_getAngularRateMin(CPhidgetSpatialHandle phid, int index, double *min);
 int CPhidgetSpatial_getMagneticField(CPhidgetSpatialHandle phid, int index, double *magneticField);
 int CPhidgetSpatial_getMagneticFieldMax(CPhidgetSpatialHandle phid, int index, double *max);
 int CPhidgetSpatial_getMagneticFieldMin(CPhidgetSpatialHandle phid, int index, double *min);
 int CPhidgetSpatial_zeroGyro(CPhidgetSpatialHandle phid);
 int CPhidgetSpatial_getDataRate(CPhidgetSpatialHandle phid, int *milliseconds);
 int CPhidgetSpatial_setDataRate(CPhidgetSpatialHandle phid, int milliseconds);
 int CPhidgetSpatial_getDataRateMax(CPhidgetSpatialHandle phid, int *max);
 int CPhidgetSpatial_getDataRateMin(CPhidgetSpatialHandle phid, int *min);
 int CPhidgetSpatial_setCompassCorrectionParameters(CPhidgetSpatialHandle phid, double magField, double offset0, double offset1, double offset2, double gain0, double gain1, double gain2, double T0, double T1, double T2, double T3, double T4, double T5);
 int CPhidgetSpatial_resetCompassCorrectionParameters(CPhidgetSpatialHandle phid);
 int CPhidgetSpatial_set_OnSpatialData_Handler(CPhidgetSpatialHandle phid, int ( *fptr)(CPhidgetSpatialHandle phid, void *userPtr, CPhidgetSpatial_SpatialEventDataHandle *data, int dataCount), void *userPtr);
typedef struct _CPhidgetStepper *CPhidgetStepperHandle;
 int CPhidgetStepper_create(CPhidgetStepperHandle *phid);
 int CPhidgetStepper_getInputCount(CPhidgetStepperHandle phid, int *count);
 int CPhidgetStepper_getInputState(CPhidgetStepperHandle phid, int index, int *inputState);
 int CPhidgetStepper_set_OnInputChange_Handler(CPhidgetStepperHandle phid, int ( *fptr)(CPhidgetStepperHandle phid, void *userPtr, int index, int inputState), void *userPtr);
 int CPhidgetStepper_getMotorCount(CPhidgetStepperHandle phid, int *count);
 int CPhidgetStepper_getAcceleration(CPhidgetStepperHandle phid, int index, double *acceleration);
 int CPhidgetStepper_setAcceleration(CPhidgetStepperHandle phid, int index, double acceleration);
 int CPhidgetStepper_getAccelerationMax(CPhidgetStepperHandle phid, int index, double *max);
 int CPhidgetStepper_getAccelerationMin(CPhidgetStepperHandle phid, int index, double *min);
 int CPhidgetStepper_getVelocityLimit(CPhidgetStepperHandle phid, int index, double *limit);
 int CPhidgetStepper_setVelocityLimit(CPhidgetStepperHandle phid, int index, double limit);
 int CPhidgetStepper_getVelocity(CPhidgetStepperHandle phid, int index, double *velocity);
 int CPhidgetStepper_getVelocityMax(CPhidgetStepperHandle phid, int index, double *max);
 int CPhidgetStepper_getVelocityMin(CPhidgetStepperHandle phid, int index, double *min);
 int CPhidgetStepper_set_OnVelocityChange_Handler(CPhidgetStepperHandle phid, int ( *fptr)(CPhidgetStepperHandle phid, void *userPtr, int index, double velocity), void *userPtr);
 int CPhidgetStepper_getTargetPosition(CPhidgetStepperHandle phid, int index, __int64 *position);
 int CPhidgetStepper_setTargetPosition(CPhidgetStepperHandle phid, int index, __int64 position);
 int CPhidgetStepper_getCurrentPosition(CPhidgetStepperHandle phid, int index, __int64 *position);
 int CPhidgetStepper_setCurrentPosition(CPhidgetStepperHandle phid, int index, __int64 position);
 int CPhidgetStepper_getPositionMax(CPhidgetStepperHandle phid, int index, __int64 *max);
 int CPhidgetStepper_getPositionMin(CPhidgetStepperHandle phid, int index, __int64 *min);
 int CPhidgetStepper_set_OnPositionChange_Handler(CPhidgetStepperHandle phid, int ( *fptr)(CPhidgetStepperHandle phid, void *userPtr, int index, __int64 position), void *userPtr);
 int CPhidgetStepper_getCurrentLimit(CPhidgetStepperHandle phid, int index, double *limit);
 int CPhidgetStepper_setCurrentLimit(CPhidgetStepperHandle phid, int index, double limit);
 int CPhidgetStepper_getCurrent(CPhidgetStepperHandle phid, int index, double *current);
 int CPhidgetStepper_getCurrentMax(CPhidgetStepperHandle phid, int index, double *max);
 int CPhidgetStepper_getCurrentMin(CPhidgetStepperHandle phid, int index, double *min);
 int CPhidgetStepper_set_OnCurrentChange_Handler(CPhidgetStepperHandle phid, int ( *fptr)(CPhidgetStepperHandle phid, void *userPtr, int index, double current), void *userPtr);
 int CPhidgetStepper_getEngaged(CPhidgetStepperHandle phid, int index, int *engagedState);
 int CPhidgetStepper_setEngaged(CPhidgetStepperHandle phid, int index, int engagedState);
 int CPhidgetStepper_getStopped(CPhidgetStepperHandle phid, int index, int *stoppedState);
typedef struct _CPhidgetTemperatureSensor *CPhidgetTemperatureSensorHandle;
 int CPhidgetTemperatureSensor_create(CPhidgetTemperatureSensorHandle *phid);
typedef enum {
 PHIDGET_TEMPERATURE_SENSOR_K_TYPE = 1,
 PHIDGET_TEMPERATURE_SENSOR_J_TYPE,
 PHIDGET_TEMPERATURE_SENSOR_E_TYPE,
 PHIDGET_TEMPERATURE_SENSOR_T_TYPE
} CPhidgetTemperatureSensor_ThermocoupleType;
 int CPhidgetTemperatureSensor_getTemperatureInputCount(CPhidgetTemperatureSensorHandle phid, int *count);
 int CPhidgetTemperatureSensor_getTemperature(CPhidgetTemperatureSensorHandle phid, int index, double *temperature);
 int CPhidgetTemperatureSensor_getTemperatureMax(CPhidgetTemperatureSensorHandle phid, int index, double *max);
 int CPhidgetTemperatureSensor_getTemperatureMin(CPhidgetTemperatureSensorHandle phid, int index, double *min);
 int CPhidgetTemperatureSensor_set_OnTemperatureChange_Handler(CPhidgetTemperatureSensorHandle phid, int ( *fptr)(CPhidgetTemperatureSensorHandle phid, void *userPtr, int index, double temperature), void *userPtr);
 int CPhidgetTemperatureSensor_getTemperatureChangeTrigger(CPhidgetTemperatureSensorHandle phid, int index, double *trigger);
 int CPhidgetTemperatureSensor_setTemperatureChangeTrigger(CPhidgetTemperatureSensorHandle phid, int index, double trigger);
 int CPhidgetTemperatureSensor_getPotential(CPhidgetTemperatureSensorHandle phid, int index, double *potential);
 int CPhidgetTemperatureSensor_getPotentialMax(CPhidgetTemperatureSensorHandle phid, int index, double *max);
 int CPhidgetTemperatureSensor_getPotentialMin(CPhidgetTemperatureSensorHandle phid, int index, double *min);
 int CPhidgetTemperatureSensor_getAmbientTemperature(CPhidgetTemperatureSensorHandle phid, double *ambient);
 int CPhidgetTemperatureSensor_getAmbientTemperatureMax(CPhidgetTemperatureSensorHandle phid, double *max);
 int CPhidgetTemperatureSensor_getAmbientTemperatureMin(CPhidgetTemperatureSensorHandle phid, double *min);
 int CPhidgetTemperatureSensor_getThermocoupleType(CPhidgetTemperatureSensorHandle phid, int index, CPhidgetTemperatureSensor_ThermocoupleType *type);
 int CPhidgetTemperatureSensor_setThermocoupleType(CPhidgetTemperatureSensorHandle phid, int index, CPhidgetTemperatureSensor_ThermocoupleType type);
typedef struct _CPhidgetTextLCD *CPhidgetTextLCDHandle;
 int CPhidgetTextLCD_create(CPhidgetTextLCDHandle *phid);
 int CPhidgetTextLCD_getRowCount(CPhidgetTextLCDHandle phid, int *count);
 int CPhidgetTextLCD_getColumnCount(CPhidgetTextLCDHandle phid, int *count);
 int CPhidgetTextLCD_getBacklight(CPhidgetTextLCDHandle phid, int *backlightState);
 int CPhidgetTextLCD_setBacklight(CPhidgetTextLCDHandle phid, int backlightState);
 int CPhidgetTextLCD_getBrightness(CPhidgetTextLCDHandle phid, int *brightness);
 int CPhidgetTextLCD_setBrightness(CPhidgetTextLCDHandle phid, int brightness);
 int CPhidgetTextLCD_getContrast(CPhidgetTextLCDHandle phid, int *contrast);
 int CPhidgetTextLCD_setContrast(CPhidgetTextLCDHandle phid, int contrast);
 int CPhidgetTextLCD_getCursorOn(CPhidgetTextLCDHandle phid, int *cursorState);
 int CPhidgetTextLCD_setCursorOn(CPhidgetTextLCDHandle phid, int cursorState);
 int CPhidgetTextLCD_getCursorBlink(CPhidgetTextLCDHandle phid, int *cursorBlinkState);
 int CPhidgetTextLCD_setCursorBlink(CPhidgetTextLCDHandle phid, int cursorBlinkState);
 int CPhidgetTextLCD_setCustomCharacter(CPhidgetTextLCDHandle phid, int index, int var1,int var2);
 int CPhidgetTextLCD_setDisplayCharacter(CPhidgetTextLCDHandle phid, int index, int column,unsigned char character);
 int CPhidgetTextLCD_setDisplayString(CPhidgetTextLCDHandle phid, int index, char *displayString);
typedef enum {
 PHIDGET_TEXTLCD_SCREEN_NONE = 1,
 PHIDGET_TEXTLCD_SCREEN_1x8,
 PHIDGET_TEXTLCD_SCREEN_2x8,
 PHIDGET_TEXTLCD_SCREEN_1x16,
 PHIDGET_TEXTLCD_SCREEN_2x16,
 PHIDGET_TEXTLCD_SCREEN_4x16,
 PHIDGET_TEXTLCD_SCREEN_2x20,
 PHIDGET_TEXTLCD_SCREEN_4x20,
 PHIDGET_TEXTLCD_SCREEN_2x24,
 PHIDGET_TEXTLCD_SCREEN_1x40,
 PHIDGET_TEXTLCD_SCREEN_2x40,
 PHIDGET_TEXTLCD_SCREEN_4x40,
 PHIDGET_TEXTLCD_SCREEN_UNKNOWN
} CPhidgetTextLCD_ScreenSize;
 int CPhidgetTextLCD_getScreenCount(CPhidgetTextLCDHandle phid, int *count);
 int CPhidgetTextLCD_getScreen(CPhidgetTextLCDHandle phid, int *screenIndex);
 int CPhidgetTextLCD_setScreen(CPhidgetTextLCDHandle phid, int screenIndex);
 int CPhidgetTextLCD_getScreenSize(CPhidgetTextLCDHandle phid, CPhidgetTextLCD_ScreenSize *screenSize);
 int CPhidgetTextLCD_setScreenSize(CPhidgetTextLCDHandle phid, CPhidgetTextLCD_ScreenSize screenSize);
 int CPhidgetTextLCD_initialize(CPhidgetTextLCDHandle phid);
typedef struct _CPhidgetTextLED *CPhidgetTextLEDHandle;
 int CPhidgetTextLED_create(CPhidgetTextLEDHandle *phid);
 int CPhidgetTextLED_getRowCount(CPhidgetTextLEDHandle phid, int *count);
 int CPhidgetTextLED_getColumnCount(CPhidgetTextLEDHandle phid, int *count);
 int CPhidgetTextLED_getBrightness(CPhidgetTextLEDHandle phid, int *brightness);
 int CPhidgetTextLED_setBrightness(CPhidgetTextLEDHandle phid, int brightness);
 int CPhidgetTextLED_setDisplayString(CPhidgetTextLEDHandle phid, int index, char *displayString);
typedef struct _CPhidgetWeightSensor *CPhidgetWeightSensorHandle;
 int CPhidgetWeightSensor_create(CPhidgetWeightSensorHandle *phid);
 int CPhidgetWeightSensor_getWeight(CPhidgetWeightSensorHandle phid, double *weight);
 int CPhidgetWeightSensor_set_OnWeightChange_Handler(CPhidgetWeightSensorHandle phid, int ( *fptr)(CPhidgetWeightSensorHandle phid, void *userPtr, double weight), void *userPtr);
 int CPhidgetWeightSensor_getWeightChangeTrigger(CPhidgetWeightSensorHandle phid, double *trigger);
 int CPhidgetWeightSensor_setWeightChangeTrigger(CPhidgetWeightSensorHandle phid, double trigger);
#ifndef CPHIDGET_CONSTANTS
#define CPHIDGET_CONSTANTS

/** \defgroup phidconst Phidget Constants 
 * Various constants used throughout the library.
 * @{
 */

/** \name Phidget States
 * Returned by getStatus() functions
 * @{
 */
#define PHIDGET_ATTACHED				0x1 /**< Phidget attached */
#define PHIDGET_NOTATTACHED				0x0 /**< Phidget not attached */
/** @} */

//Adding error codes: Update .NET, COM, Python, Java
/** \name Phidget Error Codes
 * Returned by all C API calls
 * @{
 */
#define	PHIDGET_ERROR_CODE_COUNT		20
#define EPHIDGET_OK						0	/**< Function completed successfully. */
#define EPHIDGET_NOTFOUND				1	/**< Phidget not found. "A Phidget matching the type and or serial number could not be found." */
#define EPHIDGET_NOMEMORY				2	/**< No memory. "Memory could not be allocated." */
#define EPHIDGET_UNEXPECTED				3	/**< Unexpected. "Unexpected Error. Contact Phidgets Inc. for support." */
#define EPHIDGET_INVALIDARG				4	/**< Invalid argument. "Invalid argument passed to function." */
#define EPHIDGET_NOTATTACHED			5	/**< Phidget not attached. "Phidget not physically attached." */
#define EPHIDGET_INTERRUPTED			6	/**< Interrupted. "Read/Write operation was interrupted." This code is not currently used. */
#define EPHIDGET_INVALID				7	/**< Invalid error code. "The Error Code is not defined." */
#define EPHIDGET_NETWORK				8	/**< Network. "Network Error." */
#define EPHIDGET_UNKNOWNVAL				9	/**< Value unknown. "Value is Unknown (State not yet received from device, or not yet set by user)." */
#define EPHIDGET_BADPASSWORD			10	/**< Authorization exception. "No longer used. Replaced by EEPHIDGET_BADPASSWORD" */
#define EPHIDGET_UNSUPPORTED			11	/**< Unsupported. "Not Supported." */
#define EPHIDGET_DUPLICATE				12	/**< Duplicate request. "Duplicated request." */
#define EPHIDGET_TIMEOUT				13	/**< Timeout. "Given timeout has been exceeded." */
#define EPHIDGET_OUTOFBOUNDS			14	/**< Out of bounds. "Index out of Bounds." */
#define EPHIDGET_EVENT					15	/**< Event. "A non-null error code was returned from an event handler." This code is not currently used. */
#define EPHIDGET_NETWORK_NOTCONNECTED	16	/**< Network not connected. "A connection to the server does not exist." */
#define EPHIDGET_WRONGDEVICE			17	/**< Wrong device. "Function is not applicable for this device." */
#define EPHIDGET_CLOSED					18	/**< Phidget Closed. "Phidget handle was closed." */
#define EPHIDGET_BADVERSION				19	/**< Version Mismatch. "No longer used. Replaced by EEPHIDGET_BADVERSION" */
/** @} */

//Adding error codes: Update .NET, COM, Python, Java
/** \name Phidget Error Event Codes
 * Returned in the Phidget error event
 * @{
 */
#define EEPHIDGET_EVENT_ERROR(code) (0x8000 + code)


//Library errors
#define EEPHIDGET_NETWORK		EEPHIDGET_EVENT_ERROR(0x0001)	/**< Network Error (asynchronous). */
#define EEPHIDGET_BADPASSWORD	EEPHIDGET_EVENT_ERROR(0x0002)	/**< Authorization Failed. */
#define EEPHIDGET_BADVERSION	EEPHIDGET_EVENT_ERROR(0x0003)	/**< Webservice and Client protocol versions don't match. Update to newest release. */

//Errors streamed back from firmware
#define EEPHIDGET_OK			EEPHIDGET_EVENT_ERROR(0x1000)	/**< An error state has ended - see description for details. */
#define EEPHIDGET_OVERRUN		EEPHIDGET_EVENT_ERROR(0x1002)	/**< A sampling overrun happend in firmware. */
#define EEPHIDGET_PACKETLOST	EEPHIDGET_EVENT_ERROR(0x1003)	/**< One or more packets were lost. */
#define EEPHIDGET_WRAP			EEPHIDGET_EVENT_ERROR(0x1004)	/**< A variable has wrapped around. */
#define EEPHIDGET_OVERTEMP		EEPHIDGET_EVENT_ERROR(0x1005)	/**< Overtemperature condition detected. */
#define EEPHIDGET_OVERCURRENT	EEPHIDGET_EVENT_ERROR(0x1006)	/**< Overcurrent condition detected. */
#define EEPHIDGET_OUTOFRANGE	EEPHIDGET_EVENT_ERROR(0x1007)	/**< Out of range condition detected. */
#define EEPHIDGET_BADPOWER		EEPHIDGET_EVENT_ERROR(0x1008)	/**< Power supply problem detected. */

/** @} */

/** \name Phidget Unknown Constants
 * Data values will be set to these constants when a call fails with \ref EPHIDGET_UNKNOWNVAL.
 * @{
 */
#define PUNK_BOOL	0x02					/**< Unknown Boolean (unsigned char) */
#define PUNK_SHRT	0x7FFF					/**< Unknown Short	 (16-bit) */
#define PUNK_INT	0x7FFFFFFF				/**< Unknown Integer (32-bit) */
#define PUNK_INT64	0x7FFFFFFFFFFFFFFFLL	/**< Unknown Integer (64-bit) */
#define PUNK_DBL	1e300					/**< Unknown Double */
#define PUNK_FLT	1e30					/**< Unknown Float */
/** @} */

#define PFALSE		0x00	/**< False. Used for boolean values. */
#define PTRUE		0x01	/**< True. Used for boolean values. */

/** @} */

#endif

#ifdef __cplusplus
}
#endif
#endif