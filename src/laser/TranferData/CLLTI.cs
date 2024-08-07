/*
 * scanCONTROL C# SDK - C# wrapper for LLT.dll
 *
 * MIT License
 *
 * Copyright © 2017-2018 Micro-Epsilon Messtechnik GmbH & Co. KG
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

using System;
using System.Text;
using System.Runtime.InteropServices;

namespace MEScanControl
{
    #region "Enums"
    //specifies the interface type for CreateLLTDevice and IsInterfaceType
    public enum TInterfaceType
    {
        INTF_TYPE_UNKNOWN = 0,
        INTF_TYPE_SERIAL = 1,
        INTF_TYPE_FIREWIRE = 2, // Firewire not supported by LLT.dll 3.7.x.x
        INTF_TYPE_ETHERNET = 3
    }
    //specify the callback type
    //if you programming language don't support enums, you can use a signed int
    public enum TCallbackType
    {
        STD_CALL = 0,
        C_DECL = 1,
    }

    //specify the type of the scanner
    //if you programming language don't support enums, you can use a signed int
    public enum TScannerType
    {

        StandardType = -1,                   //can't decode scanCONTROL name use standard measurement range
        LLT25 = 0,                           //scanCONTROL28xx with 25mm measurmentrange
        LLT100 = 1,                          //scanCONTROL28xx with 100mm measurmentrange

        scanCONTROL28xx_25 = 0,              //scanCONTROL28xx with 25mm measurmentrange
        scanCONTROL28xx_100 = 1,             //scanCONTROL28xx with 100mm measurmentrange
        scanCONTROL28xx_10 = 2,              //scanCONTROL28xx with 10mm measurmentrange
        scanCONTROL28xx_xxx = 999,           //scanCONTROL28xx with no measurmentrange -> use standard measurement range

        scanCONTROL27xx_25 = 1000,           //scanCONTROL27xx with 25mm measurmentrange
        scanCONTROL27xx_100 = 1001,          //scanCONTROL27xx with 100mm measurmentrange
        scanCONTROL27xx_50 = 1002,           //scanCONTROL27xx with 50mm measurmentrange
        scanCONTROL2751_25 = 1020,           //scanCONTROL27xx with 25mm measurmentrange
        scanCONTROL2751_100 = 1021,          //scanCONTROL27xx with 100mm measurmentrange
        scanCONTROL2702_50 = 1032,           //scanCONTROL2702 with 50mm measurement range
        scanCONTROL27xx_xxx = 1999,          //scanCONTROL27xx with no measurmentrange -> use standard measurement range

        scanCONTROL26xx_25 = 2000,           //scanCONTROL26xx with 25mm measurmentrange
        scanCONTROL26xx_100 = 2001,          //scanCONTROL26xx with 100mm measurmentrange
        scanCONTROL26xx_50 = 2002,           //scanCONTROL26xx with 50mm measurmentrange 
        scanCONTROL26xx_10 = 2003,           // scanCONTROL26xx with 10mm measurement range         
        scanCONTROL2651_25 = 2020,           //scanCONTROL26xx with 25mm measurmentrange
        scanCONTROL2651_100 = 2021,          //scanCONTROL26xx with 100mm measurmentrange
        scanCONTROL2602_50 = 2032,           //scanCONTROL2602 with 50mm measurement range
        scanCONTROL26xx_xxx = 2999,          //scanCONTROL26xx with no measurmentrange -> use standard measurement range

        scanCONTROL29xx_25 = 3000,           //scanCONTROL29xx with 25mm measurmentrange
        scanCONTROL29xx_100 = 3001,          //scanCONTROL29xx with 100mm measurmentrange
        scanCONTROL29xx_50 = 3002,           // scanCONTROL29xx with 50mm measurement range
        scanCONTROL29xx_10 = 3003,           //scanCONTROL29xx with 10mm measurmentrange
        scanCONTROL2905_50 = 3010,           // scanCONTROL2905 with 50mm measurement range
        scanCONTROL2905_100 = 3011,          // scanCONTROL2905 with 100mm measurement range      
        scanCONTROL2951_25 = 3020,           //scanCONTROL29xx with 25mm measurmentrange
        scanCONTROL2951_100 = 3021,          //scanCONTROL29xx with 100mm measurmentrange
        scanCONTROL2902_50 = 3032,           //scanCONTROL2902 with 50mm measurement range
        scanCONTROL2953_30 = 3033,           //scanCONTROL2953 with 30mm measurement range
        scanCONTROL2954_10 = 3034,           //scanCONTROL2954 with 10mm measurement range
        scanCONTROL2954_25 = 3035,           //scanCONTROL2954 with 25mm measurement range
        scanCONTROL2954_50 = 3036,           //scanCONTROL2954 with 50mm measurement range
        scanCONTROL2954_100 = 3037,          //scanCONTROL2954 with 100mm measurement range
        scanCONTROL2954_xxx = 3038,          //scanCONTROL2954 with 30mm measurement range
        scanCONTROL2968_10 = 3040,           // scanCONTROL2968 with 10mm measurement range
        scanCONTROL2968_25 = 3041,           // scanCONTROL2968 with 25mm measurement range
        scanCONTROL2968_50 = 3042,           // scanCONTROL2968 with 50mm measurement range
        scanCONTROL2968_100 = 3043,          // scanCONTROL2968 with 100mm measurement range
        scanCONTROL2968_xxx = 3044,          // scanCONTROL2968 with unknown measurement range
        scanCONTROL29xx_xxx = 3999,          //scanCONTROL29xx with no measurmentrange -> use standard measurement range

        scanCONTROL30xx_25 = 4000,           // scanCONTROL30xx with 25mm measurement range
        scanCONTROL30xx_50 = 4001,           // scanCONTROL30xx with 50mm measurement range
        scanCONTROL30xx_200 = 4002,         // scanCONTROL30xx with 200mm measurement range
        scanCONTROL30xx_100 = 4003,             // scanCONTROL30xx with 100mm measurement range
        scanCONTROL30xx_xxx = 4999,          // scanCONTROL29xx with no measurement range

        scanCONTROL25xx_25 = 5000,           // scanCONTROL25xx with 25mm measurement range
        scanCONTROL25xx_50 = 5002,           // scanCONTROL25xx with 50mm measurement range
        scanCONTROL25xx_100 = 5001,           // scanCONTROL25xx with 100mm measurement range
        scanCONTROL25xx_10 = 5003,           // scanCONTROL25xx with 10mm measurement range
        scanCONTROL25xx_xxx = 5999,          // scanCONTROL25xx with no measurement range

        scanCONTROL2xxx = 6000,              //dummy
    }
    //specify the profile configuration
    //if you programming language don't support enums, you can use a signed int
    public enum TProfileConfig
    {
        NONE = 0,
        PROFILE = 1,
        CONTAINER = 1,
        VIDEO_IMAGE = 1,
        PURE_PROFILE = 2,
        QUARTER_PROFILE = 3,
        CSV_PROFILE = 4,
        PARTIAL_PROFILE = 5,
    }

    public enum TTransferProfileType
    {
        NORMAL_TRANSFER = 0,
        SHOT_TRANSFER = 1,
        NORMAL_CONTAINER_MODE = 2,
        SHOT_CONTAINER_MODE = 3,
        NONE_TRANSFER = 4,
    }
    public struct TPartialProfile
    {
        public uint nStartPoint;
        public uint nStartPointData;
        public uint nPointCount;
        public uint nPointDataWidth;
    }
    [StructLayout(LayoutKind.Sequential, Pack = 0)]
    public struct TConvertContainerParameter
    {
        public IntPtr Container;
        public uint profileRearrangement;
        public uint numberOfProfilesToExtract;
        public uint scanner;
        public uint reflectionNumber;
        public int ConvertToMM;
        public IntPtr ReflectionWidth;
        public IntPtr MaxIntensity;
        public IntPtr Threshold;
        public IntPtr Moment0;
        public IntPtr Moment1;
        public IntPtr X;
        public IntPtr Z;
    }
    public enum TTransferVideoType
    {
        VIDEO_MODE_0 = 0,
        VIDEO_MODE_1 = 1,
        NONE_VIDEOMODE = 2,
    }
    public enum TFileType
    {
        AVI = 0,
        LLT = 1,
        CSV = 2,
        BMP = 3,
        CSV_NEG = 4,
    }
    public unsafe delegate void ProfileReceiveMethod(byte* data, uint iSize, uint pUserData);

    #endregion
    public class CLLTI
    {
        // C# wrapper for LLT.dll interface

        #region "Constant declarations"

        

        public const uint CONVERT_X = 0x00000800;
        public const uint CONVERT_Z = 0x00001000;
        public const uint CONVERT_WIDTH = 0x00000100;
        public const uint CONVERT_MAXIMUM = 0x00000200;
        public const uint CONVERT_THRESHOLD = 0x00000400;
        public const uint CONVERT_M0 = 0x00002000;
        public const uint CONVERT_M1 = 0x00004000;

        // specify the type for the RS422-multiplexer (27xx)
        public const int RS422_INTERFACE_FUNCTION_AUTO_27xx = 0;
        public const int RS422_INTERFACE_FUNCTION_SERIALPORT_115200_27xx = 1;
        public const int RS422_INTERFACE_FUNCTION_TRIGGER_27xx = 2;
        public const int RS422_INTERFACE_FUNCTION_CMM_TRIGGER_27xx = 3;
        public const int RS422_INTERFACE_FUNCTION_ENCODER_27xx = 4;
        public const int RS422_INTERFACE_FUNCTION_DIGITAL_OUTPUT_27xx = 6;
        public const int RS422_INTERFACE_FUNCTION_TRIGGER_LASER_PULSE_27xx = 7;
        public const int RS422_INTERFACE_FUNCTION_SERIALPORT_57600_27xx = 8;
        public const int RS422_INTERFACE_FUNCTION_SERIALPORT_38400_27xx = 9;
        public const int RS422_INTERFACE_FUNCTION_SERIALPORT_19200_27xx = 10;
        public const int RS422_INTERFACE_FUNCTION_SERIALPORT_9600_27xx = 11;

        // specify the type for the RS422-multiplexer (26xx/29xx)
        public const int RS422_INTERFACE_FUNCTION_SERIALPORT_115200 = 0;
        public const int RS422_INTERFACE_FUNCTION_SERIALPORT_57600 = 1;
        public const int RS422_INTERFACE_FUNCTION_SERIALPORT_38400 = 2;
        public const int RS422_INTERFACE_FUNCTION_SERIALPORT_19200 = 3;
        public const int RS422_INTERFACE_FUNCTION_SERIALPORT_9600 = 4;
        public const int RS422_INTERFACE_FUNCTION_TRIGGER_INPUT = 5;
        public const int RS422_INTERFACE_FUNCTION_TRIGGER_OUTPUT = 6;
        public const int RS422_INTERFACE_FUNCTION_CMM_TRIGGER = 7;

        // processing flags
        public const uint PROC_HIGH_RESOLUTION = 1;
        public const uint PROC_CALIBRATION = (1 << 1);
        public const uint PROC_MULTIREFL_ALL = (0 << 2);
        public const uint PROC_MULITREFL_FIRST = (1 << 2);
        public const uint PROC_MULITREFL_LAST = (2 << 2);
        public const uint PROC_MULITREFL_LARGESTAREA = (3 << 2);
        public const uint PROC_MULITREFL_MAXINTENS = (4 << 2);
        public const uint PROC_MULITREFL_SINGLE = (5 << 2);
        public const uint PROC_POSTPROCESSING_ON = (1 << 5);
        public const uint PROC_FLIP_DISTANCE = (1 << 6);
        public const uint PROC_FLIP_POSITION = (1 << 7);
        public const uint PROC_AUTOSHUTTER_DELAY = (1 << 8);
        public const uint PROC_SHUTTERALIGN_CENTRE = (0 << 9);
        public const uint PROC_SHUTTERALIGN_RIGHT = (1 << 9);
        public const uint PROC_SHUTTERALIGN_LEFT = (2 << 9);
        public const uint PROC_SHUTTERALIGN_OFF = (3 << 9);
        public const uint PROC_AUTOSHUTTER_ADVANCED = (1 << 11);

        // threshold flags
        public const uint THRESHOLD_AUTOMATIC = (1 << 24);
        public const uint THRESHOLD_VIDEO_FILTER = (1 << 11);
        public const uint THRESHOLD_AMBIENT_LIGHT_SUPPRESSION = (1 << 10);
        // threshold flags (deprecated names)
        public const uint THRESHOLD_DYNAMIC = (1 << 24);
        public const uint THRESHOLD_BACKGROUND_FILTER = (1 << 10);

        // exposure flags
        public const uint EXPOSURE_AUTOMATIC = (1 << 24);
        // shutter flags (deprecated names)
        public const uint SHUTTER_AUTOMATIC = (1 << 24);

        // laser power flags
        public const uint LASER_OFF = 0;
        public const uint LASER_REDUCED_POWER = 1;
        public const uint LASER_FULL_POWER = 2;
        public const uint LASER_PULSEMODE = (1 << 11);

        // temperature register
        public const uint TEMP_PREPARE_VALUE = 0x86000000;

        // trigger flags
        public const uint TRIG_MODE_EDGE = (0 << 16);
        public const uint TRIG_MODE_PULSE = (1 << 16);
        public const uint TRIG_MODE_GATE = (2 << 16);
        public const uint TRIG_MODE_ENCODER = (3 << 16);
        public const uint TRIG_INPUT_RS422 = (0 << 21);
        public const uint TRIG_INPUT_DIGIN = (1 << 21);
        public const uint TRIG_POLARITY_LOW = (0 << 24);
        public const uint TRIG_POLARITY_HIGH = (1 << 24);
        public const uint TRIG_EXT_ACTIVE = (1 << 25);
        public const uint TRIG_INTERNAL = (0 << 25);

        // region Of Interest 1 flags
        public const uint ROI1_FREE_REGION = (1 << 11);
        // measuring field flags
        public const uint MEASFIELD_ACTIVATE_FREE = (1 << 11);

        // Image Features flags
        public const uint ROI2_ENABLE = (1 << 0);
        public const uint RONI_ENABLE = (1 << 1);
        public const uint EA_REF_REGION_ENABLE = (1 << 2);
        public const uint FAST_MODE_ENABLE = (1 << 30);
        public const uint HDR_ENABLE = (uint)1 << 31;
        public const uint SENSITIVITY_MINIMUM = (0 << 3);
        public const uint SENSITIVITY_LOW = (1 << 3);
        public const uint SENSITIVITY_MEDIUM = (2 << 3);
        public const uint SENSITIVITY_HIGH = (3 << 3);
        public const uint SENSITIVITY_MAXIMUM = (4 << 3);

        // maintenance flags
        public const uint MAINTENANCE_COMPRESS_DATA = 1;
        public const uint MAINTENANCE_LOOPBACK = (1 << 1);
        public const uint MAINTENANCE_ENCODER_ACTIVE = (1 << 3);
        public const uint MAINTENANCE_SUPPRESS_REGISTER_RESET = (1 << 5);
        public const uint MAINTENANCE_UM_LOAD_VIA_DIGIN = (0 << 6);
        public const uint MAINTENANCE_UM_SUPPRESS_UNTIL_REBOOT = (1 << 6);
        public const uint MAINTENANCE_UM_SUPPRESS_UNTIL_GVCP_CLOSE = (2 << 6);
        public const uint MAINTENANCE_UM_SUPPRESS_UNTIL_REBOOT_GVCP_CLOSE = (3 << 6);

        // multiport flags
        public const uint MULTI_LEVEL_5V = (0 << 11);
        public const uint MULTI_LEVEL_24V = (1 << 11);
        public const uint MULTI_RS422_TERM_ON = (0 << 10);
        public const uint MULTI_RS422_TERM_OFF = (1 << 10);
        public const uint MULTI_ENCODER_BIDIRECT = (0 << 9);
        public const uint MULTI_ENCODER_UNIDIRECT = (1 << 9);
        public const uint MULTI_INPUT_PULLUP = (0 << 8);
        public const uint MULTI_INPUT_PULLDOWN = (1 << 8);
        public const uint MULTI_DIGIN_ENC_INDEX = (0 << 4);
        public const uint MULTI_DIGIN_ENC_TRIG = (1 << 4);
        public const uint MULTI_DIGIN_TRIG_ONLY = (2 << 4);
        public const uint MULTI_DIGIN_TRIG_UM = (3 << 4);
        public const uint MULTI_DIGIN_UM = (4 << 4);
        public const uint MULTI_DIGIN_TS = (5 << 4);
        public const uint MULTI_DIGIN_FRAMETRIG_BI = (6 << 4);
        public const uint MULTI_DIGIN_FRAMETRIG_UNI = (7 << 4);
        public const uint MULTI_DIGIN_GATED_ENCODER = (8 << 4);
        public const uint MULTI_DIGIN_TRIG_UM2_TS1 = (9 << 4);
        public const uint MULTI_DIGIN_UM3_TS1 = (10 << 4);
        public const uint MULTI_DIGIN_UM2_TS2 = (11 << 4);
        public const uint MULTI_RS422_115200 = 0;
        public const uint MULTI_RS422_57600 = 1;
        public const uint MULTI_RS422_38400 = 2;
        public const uint MULTI_RS422_19200 = 3;
        public const uint MULTI_RS422_9600 = 4;
        public const uint MULTI_RS422_TRIG_IN = 5;
        public const uint MULTI_RS422_TRIG_OUT = 6;
        public const uint MULTI_RS422_CMM = 7;

        // profile filter flags
        public const uint FILTER_RESAMPLE_EXTRAPOLATE_POINTS = (1 << 11);
        public const uint FILTER_RESAMPLE_ALL_INFO = (1 << 10);
        public const uint FILTER_RESAMPLE_DISABLED = (0 << 4);
        public const uint FILTER_RESAMPLE_TINY = (1 << 4);
        public const uint FILTER_RESAMPLE_VERYSMALL = (2 << 4);
        public const uint FILTER_RESAMPLE_SMALL = (3 << 4);
        public const uint FILTER_RESAMPLE_MEDIUM = (4 << 4);
        public const uint FILTER_RESAMPLE_LARGE = (5 << 4);
        public const uint FILTER_RESAMPLE_VERYLARGE = (6 << 4);
        public const uint FILTER_RESAMPLE_HUGE = (7 << 4);
        public const uint FILTER_MEDIAN_DISABLED = (0 << 2);
        public const uint FILTER_MEDIAN_3 = (1 << 2);
        public const uint FILTER_MEDIAN_5 = (2 << 2);
        public const uint FILTER_MEDIAN_7 = (3 << 2);
        public const uint FILTER_AVG_DISABLED = 0;
        public const uint FILTER_AVG_3 = 1;
        public const uint FILTER_AVG_5 = 2;
        public const uint FILTER_AVG_7 = 3;

        // container flags
        public const uint CONTAINER_STRIPE_4 = (1 << 23);
        public const uint CONTAINER_STRIPE_3 = (1 << 22);
        public const uint CONTAINER_STRIPE_2 = (1 << 21);
        public const uint CONTAINER_STRIPE_1 = (1 << 20);
        public const uint CONTAINER_JOIN = (1 << 19);
        public const uint CONTAINER_DATA_SIGNED = (1 << 18);
        public const uint CONTAINER_DATA_LSBF = (1 << 17);
        public const uint CONTAINER_DATA_TS = (1 << 11);
        public const uint CONTAINER_DATA_EMPTYFIELD4TS = (1 << 10);
        public const uint CONTAINER_DATA_LOOPBACK = (1 << 9);
        public const uint CONTAINER_DATA_MOM1U = (1 << 8);
        public const uint CONTAINER_DATA_MOM1L = (1 << 7);
        public const uint CONTAINER_DATA_MOM0U = (1 << 6);
        public const uint CONTAINER_DATA_MOM0L = (1 << 5);
        public const uint CONTAINER_DATA_WIDTH = (1 << 4);
        public const uint CONTAINER_DATA_INTENS = (1 << 3);
        public const uint CONTAINER_DATA_THRES = (1 << 2);
        public const uint CONTAINER_DATA_X = (1 << 1);
        public const uint CONTAINER_DATA_Z = (1 << 0);

        #endregion

        #region "Return Values"

        //Message-Values
        public const int ERROR_OK = 0;
        public const int ERROR_SERIAL_COMM = 1;
        public const int ERROR_SERIAL_LLT = 7;
        public const int ERROR_CONNECTIONLOST = 10;
        public const int ERROR_STOPSAVING = 100;

        //Return-Values for the is-functions
        public const int IS_FUNC_NO = 0;
        public const int IS_FUNC_YES = 1;

        //General return-values for all functions
        public const int GENERAL_FUNCTION_DEVICE_NAME_NOT_SUPPORTED = 4;
        public const int GENERAL_FUNCTION_PACKET_SIZE_CHANGED = 3;
        public const int GENERAL_FUNCTION_CONTAINER_MODE_HEIGHT_CHANGED = 2;
        public const int GENERAL_FUNCTION_OK = 1;
        public const int GENERAL_FUNCTION_NOT_AVAILABLE = 0;

        public const int ERROR_GENERAL_WHILE_LOAD_PROFILE = -1000;
        public const int ERROR_GENERAL_NOT_CONNECTED = -1001;
        public const int ERROR_GENERAL_DEVICE_BUSY = -1002;
        public const int ERROR_GENERAL_WHILE_LOAD_PROFILE_OR_GET_PROFILES = -1003;
        public const int ERROR_GENERAL_WHILE_GET_PROFILES = -1004;
        public const int ERROR_GENERAL_GET_SET_ADDRESS = -1005;
        public const int ERROR_GENERAL_POINTER_MISSING = -1006;
        public const int ERROR_GENERAL_WHILE_SAVE_PROFILES = -1007;
        public const int ERROR_GENERAL_SECOND_CONNECTION_TO_LLT = -1008;

        //Return-Values for GetDeviceName
        public const int ERROR_GETDEVICENAME_SIZE_TOO_LOW = -1;
        public const int ERROR_GETDEVICENAME_NO_BUFFER = -2;

        //Return-Values for Load/SaveProfiles
        public const int ERROR_LOADSAVE_WRITING_LAST_BUFFER = -50;
        public const int ERROR_LOADSAVE_WHILE_SAVE_PROFILE = -51;
        public const int ERROR_LOADSAVE_NO_PROFILELENGTH_POINTER = -52;
        public const int ERROR_LOADSAVE_NO_LOAD_PROFILE = -53;
        public const int ERROR_LOADSAVE_STOP_ALREADY_LOAD = -54;
        public const int ERROR_LOADSAVE_CANT_OPEN_FILE = -55;
        public const int ERROR_LOADSAVE_FILE_POSITION_TOO_HIGH = -57;
        public const int ERROR_LOADSAVE_AVI_NOT_SUPPORTED = -58;
        public const int ERROR_LOADSAVE_NO_REARRANGEMENT_POINTER = -59;
        public const int ERROR_LOADSAVE_WRONG_PROFILE_CONFIG = -60;
        public const int ERROR_LOADSAVE_NOT_TRANSFERING = -61;

        //Return-Values for profile transfer functions
        public const int ERROR_PROFTRANS_SHOTS_NOT_ACTIVE = -100;
        public const int ERROR_PROFTRANS_SHOTS_COUNT_TOO_HIGH = -101;
        public const int ERROR_PROFTRANS_WRONG_PROFILE_CONFIG = -102;
        public const int ERROR_PROFTRANS_FILE_EOF = -103;
        public const int ERROR_PROFTRANS_NO_NEW_PROFILE = -104;
        public const int ERROR_PROFTRANS_BUFFER_SIZE_TOO_LOW = -105;
        public const int ERROR_PROFTRANS_NO_PROFILE_TRANSFER = -106;
        public const int ERROR_PROFTRANS_PACKET_SIZE_TOO_HIGH = -107;
        public const int ERROR_PROFTRANS_CREATE_BUFFERS = -108;
        public const int ERROR_PROFTRANS_WRONG_PACKET_SIZE_FOR_CONTAINER = -109;
        public const int ERROR_PROFTRANS_REFLECTION_NUMBER_TOO_HIGH = -110;
        public const int ERROR_PROFTRANS_MULTIPLE_SHOTS_ACTIVE = -111;
        public const int ERROR_PROFTRANS_BUFFER_HANDOUT = -112;
        public const int ERROR_PROFTRANS_WRONG_BUFFER_POINTER = -113;
        public const int ERROR_PROFTRANS_WRONG_TRANSFER_CONFIG = -114;

        //Return-Values for Set/GetFunctions
        public const int ERROR_SETGETFUNCTIONS_WRONG_BUFFER_COUNT = -150;
        public const int ERROR_SETGETFUNCTIONS_PACKET_SIZE = -151;
        public const int ERROR_SETGETFUNCTIONS_WRONG_PROFILE_CONFIG = -152;
        public const int ERROR_SETGETFUNCTIONS_NOT_SUPPORTED_RESOLUTION = -153;
        public const int ERROR_SETGETFUNCTIONS_REFLECTION_NUMBER_TOO_HIGH = -154;
        public const int ERROR_SETGETFUNCTIONS_WRONG_FEATURE_ADRESS = -155;
        public const int ERROR_SETGETFUNCTIONS_SIZE_TOO_LOW = -156;
        public const int ERROR_SETGETFUNCTIONS_WRONG_PROFILE_SIZE = -157;
        public const int ERROR_SETGETFUNCTIONS_MOD_4 = -158;
        public const int ERROR_SETGETFUNCTIONS_REARRANGEMENT_PROFILE = -159;
        public const int ERROR_SETGETFUNCTIONS_USER_MODE_TOO_HIGH = -160;
        public const int ERROR_SETGETFUNCTIONS_USER_MODE_FACTORY_DEFAULT = -161;
        public const int ERROR_SETGETFUNCTIONS_HEARTBEAT_TOO_HIGH = -162;


        //Return-Values PostProcessingParameter
        public const int ERROR_POSTPROCESSING_NO_PROF_BUFFER = -200;
        public const int ERROR_POSTPROCESSING_MOD_4 = -201;
        public const int ERROR_POSTPROCESSING_NO_RESULT = -202;
        public const int ERROR_POSTPROCESSING_LOW_BUFFERSIZE = -203;
        public const int ERROR_POSTPROCESSING_WRONG_RESULT_SIZE = -204;
        public const int ERROR_POSTPROCESSING_REFLECTION_NUMBER_TOO_HIGH = -205;
        public const int ERROR_POSTPROCESSING_REFLECTION_NUMBER_NOT_AVAILABLE = -206;
        public const int ERROR_POSTPROCESSING_INVALID_CONTAINER_RESOLUTION = -207;
        public const int ERROR_POSTPROCESSING_INCOMPLETE_M0_OR_M1 = -208;


        //Return-Values for GetDeviceInterfaces
        public const int ERROR_GETDEVINTERFACES_WIN_NOT_SUPPORTED = -250;
        public const int ERROR_GETDEVINTERFACES_REQUEST_COUNT = -251;
        public const int ERROR_GETDEVINTERFACES_CONNECTED = -252;
        public const int ERROR_GETDEVINTERFACES_INTERNAL = -253;

        //Return-Values for Connect
        public const int ERROR_CONNECT_LLT_COUNT = -300;
        public const int ERROR_CONNECT_SELECTED_LLT = -301;
        public const int ERROR_CONNECT_ALREADY_CONNECTED = -302;
        public const int ERROR_CONNECT_LLT_NUMBER_ALREADY_USED = -303;
        public const int ERROR_CONNECT_SERIAL_CONNECTION = -304;
        public const int ERROR_CONNECT_INVALID_IP = -305;

        //Return-Values for SetPartialProfile
        public const int ERROR_PARTPROFILE_NO_PART_PROF = -350;
        public const int ERROR_PARTPROFILE_TOO_MUCH_BYTES = -351;
        public const int ERROR_PARTPROFILE_TOO_MUCH_POINTS = -352;
        public const int ERROR_PARTPROFILE_NO_POINT_COUNT = -353;
        public const int ERROR_PARTPROFILE_NOT_MOD_UNITSIZE_POINT = -354;
        public const int ERROR_PARTPROFILE_NOT_MOD_UNITSIZE_DATA = -355;

        //Return-Values for Start/StopTransmissionAndCmmTrigger
        public const int ERROR_CMMTRIGGER_NO_DIVISOR = -400;
        public const int ERROR_CMMTRIGGER_TIMEOUT_AFTER_TRANSFERPROFILES = -401;
        public const int ERROR_CMMTRIGGER_TIMEOUT_AFTER_SETCMMTRIGGER = -402;

        //Return-Values for TranslateErrorValue
        public const int ERROR_TRANSERRORVALUE_WRONG_ERROR_VALUE = -450;
        public const int ERROR_TRANSERRORVALUE_BUFFER_SIZE_TOO_LOW = -451;

        //Read/write config functions
        public const int ERROR_READWRITECONFIG_CANT_CREATE_FILE = -500;
        public const int ERROR_READWRITECONFIG_CANT_OPEN_FILE = -501;
        public const int ERROR_READWRITECONFIG_QUEUE_TO_SMALL = -502;
        public const int ERROR_READWRITECONFIG_FILE_EMPTY = -503;
        public const int ERROR_READWRITECONFIG_UNKNOWN_FILE = -504;

        #endregion

        #region "Registers"
        //Function defines for the Get/SetFeature function
        public const uint FEATURE_FUNCTION_SERIAL_NUMBER = 0xf0000410;
        public const uint FEATURE_FUNCTION_CALIBRATION_SCALE = 0xf0a00000;
        public const uint FEATURE_FUNCTION_CALIBRATION_OFFSET = 0xf0a00004;
        public const uint FEATURE_FUNCTION_PEAKFILTER_WIDTH = 0xf0b02000;
        public const uint FEATURE_FUNCTION_PEAKFILTER_HEIGHT = 0xf0b02004;
        public const uint FEATURE_FUNCTION_ROI1_DISTANCE = 0xf0b02008;
        public const uint FEATURE_FUNCTION_ROI1_POSITION = 0xf0b0200c;
        public const uint FEATURE_FUNCTION_ROI1_TRACKING_DIVISOR = 0xf0b02010;
        public const uint FEATURE_FUNCTION_ROI1_TRACKING_FACTOR = 0xf0b02014;
        public const uint FEATURE_FUNCTION_CALIBRATION_0 = 0xf0b02020;
        public const uint FEATURE_FUNCTION_CALIBRATION_1 = 0xf0b02024;
        public const uint FEATURE_FUNCTION_CALIBRATION_2 = 0xf0b02028;
        public const uint FEATURE_FUNCTION_CALIBRATION_3 = 0xf0b0202c;
        public const uint FEATURE_FUNCTION_CALIBRATION_4 = 0xf0b02030;
        public const uint FEATURE_FUNCTION_CALIBRATION_5 = 0xf0b02034;
        public const uint FEATURE_FUNCTION_CALIBRATION_6 = 0xf0b02038;
        public const uint FEATURE_FUNCTION_CALIBRATION_7 = 0xf0b0203c;
        public const uint FEATURE_FUNCTION_IMAGE_FEATURES = 0xf0b02100;
        public const uint FEATURE_FUNCTION_ROI2_DISTANCE = 0xf0b02104;
        public const uint FEATURE_FUNCTION_ROI2_POSITION = 0xf0b02108;
        public const uint FEATURE_FUNCTION_RONI_DISTANCE = 0xf0b0210c;
        public const uint FEATURE_FUNCTION_RONI_POSITION = 0xf0b02110;
        public const uint FEATURE_FUNCTION_EA_REFERENCE_REGION_DISTANCE = 0xf0b02114;
        public const uint FEATURE_FUNCTION_EA_REFERENCE_REGION_POSITION = 0xf0b02118;
        public const uint FEATURE_FUNCTION_LASER = 0xf0f00824;
        public const uint INQUIRY_FUNCTION_LASER = 0xf0f00524;
        public const uint FEATURE_FUNCTION_ROI1_PRESET = 0xf0f00880;
        public const uint INQUIRY_FUNCTION_ROI1_PRESET = 0xf0f00580;
        public const uint FEATURE_FUNCTION_TRIGGER = 0xf0f00830;
        public const uint INQUIRY_FUNCTION_TRIGGER = 0xf0f00530;
        public const uint FEATURE_FUNCTION_EXPOSURE_AUTOMATIC_LIMITS = 0xf0f00834;
        public const uint INQUIRY_FUNCTION_EXPOSURE_AUTOMATIC_LIMITS = 0xf0f00534;
        public const uint FEATURE_FUNCTION_EXPOSURE_TIME = 0xf0f0081c;
        public const uint INQUIRY_FUNCTION_EXPOSURE_TIME = 0xf0f0051c;
        public const uint FEATURE_FUNCTION_IDLE_TIME = 0xf0f00800;
        public const uint INQUIRY_FUNCTION_IDLE_TIME = 0xf0f00500;
        public const uint FEATURE_FUNCTION_PROFILE_PROCESSING = 0xf0f00804;
        public const uint INQUIRY_FUNCTION_PROFILE_PROCESSING = 0xf0f00504;
        public const uint FEATURE_FUNCTION_THRESHOLD = 0xf0f00810;
        public const uint INQUIRY_FUNCTION_THRESHOLD = 0xf0f00510;
        public const uint FEATURE_FUNCTION_MAINTENANCE = 0xf0f0088c;
        public const uint INQUIRY_FUNCTION_MAINTENANCE = 0xf0f0058c;
        public const uint FEATURE_FUNCTION_CMM_TRIGGER = 0xf0f00888;
        public const uint INQUIRY_FUNCTION_CMM_TRIGGER = 0xf0f00588;
        public const uint FEATURE_FUNCTION_PROFILE_REARRANGEMENT = 0xf0f0080c;
        public const uint INQUIRY_FUNCTION_PROFILE_REARRANGEMENT = 0xf0f0050c;
        public const uint FEATURE_FUNCTION_PROFILE_FILTER = 0xf0f00818;
        public const uint INQUIRY_FUNCTION_PROFILE_FILTER = 0xf0f00518;
        public const uint FEATURE_FUNCTION_DIGITAL_IO = 0xf0f008c0;
        public const uint INQUIRY_FUNCTION_DIGITAL_IO = 0xf0f005c0;
        public const uint FEATURE_FUNCTION_TEMPERATURE = 0xf0f0082c;
        public const uint INQUIRY_FUNCTION_TEMPERATURE = 0xf0f0052c;
        public const uint FEATURE_FUNCTION_EXTRA_PARAMETER = 0xf0f00808;
        public const uint INQUIRY_FUNCTION_EXTRA_PARAMETER = 0xf0f00508;

        public const uint FEATURE_FUNCTION_PACKET_DELAY = 0x00000d08;
        public const uint FEATURE_FUNCTION_CONNECTION_SPEED = 0x00000670;

        // function Defines for the Get/SetFeature function (deprecated names)
        public const uint FEATURE_FUNCTION_SERIAL = 0xf0000410;
        public const uint FEATURE_FUNCTION_FREE_MEASURINGFIELD_Z = 0xf0b02008;
        public const uint FEATURE_FUNCTION_FREE_MEASURINGFIELD_X = 0xf0b0200c;
        public const uint FEATURE_FUNCTION_DYNAMIC_TRACK_DIVISOR = 0xf0b02010;
        public const uint FEATURE_FUNCTION_DYNAMIC_TRACK_FACTOR = 0xf0b02014;
        public const uint FEATURE_FUNCTION_LASERPOWER = 0xf0f00824;
        public const uint INQUIRY_FUNCTION_LASERPOWER = 0xf0f00524;
        public const uint FEATURE_FUNCTION_MEASURINGFIELD = 0xf0f00880;
        public const uint INQUIRY_FUNCTION_MEASURINGFIELD = 0xf0f00580;
        public const uint FEATURE_FUNCTION_SHUTTERTIME = 0xf0f0081c;
        public const uint INQUIRY_FUNCTION_SHUTTERTIME = 0xf0f0051c;
        public const uint FEATURE_FUNCTION_IDLETIME = 0xf0f00800;
        public const uint INQUIRY_FUNCTION_IDLETIME = 0xf0f00500;
        public const uint FEATURE_FUNCTION_PROCESSING_PROFILEDATA = 0xf0f00804;
        public const uint INQUIRY_FUNCTION_PROCESSING_PROFILEDATA = 0xf0f00504;
        public const uint FEATURE_FUNCTION_MAINTENANCEFUNCTIONS = 0xf0f0088c;
        public const uint INQUIRY_FUNCTION_MAINTENANCEFUNCTIONS = 0xf0f0058c;
        public const uint FEATURE_FUNCTION_ANALOGFREQUENCY = 0xf0f00828;
        public const uint INQUIRY_FUNCTION_ANALOGFREQUENCY = 0xf0f00528;
        public const uint FEATURE_FUNCTION_ANALOGOUTPUTMODES = 0xf0f00820;
        public const uint INQUIRY_FUNCTION_ANALOGOUTPUTMODES = 0xf0f00520;
        public const uint FEATURE_FUNCTION_CMMTRIGGER = 0xf0f00888;
        public const uint INQUIRY_FUNCTION_CMMTRIGGER = 0xf0f00588;
        public const uint FEATURE_FUNCTION_REARRANGEMENT_PROFILE = 0xf0f0080c;
        public const uint INQUIRY_FUNCTION_REARRANGEMENT_PROFILE = 0xf0f0050c;
        public const uint FEATURE_FUNCTION_RS422_INTERFACE_FUNCTION = 0xf0f008c0;
        public const uint INQUIRY_FUNCTION_RS422_INTERFACE_FUNCTION = 0xf0f005c0;
        public const uint FEATURE_FUNCTION_SATURATION = 0xf0f00814;
        public const uint INQUIRY_FUNCTION_SATURATION = 0xf0f00514;
        public const uint FEATURE_FUNCTION_CAPTURE_QUALITY = 0xf0f008c4;
        public const uint INQUIRY_FUNCTION_CAPTURE_QUALITY = 0xf0f005c4;
        public const uint FEATURE_FUNCTION_SHARPNESS = 0xf0f00808;
        public const uint INQUIRY_FUNCTION_SHARPNESS = 0xf0f00508;

        #endregion

        #region "dll functions"
        public static uint CreateLLTDevice(TInterfaceType iInterfaceType)
        {
            return NativeMethods.CreateLLTDevice(iInterfaceType);
        }

        public static int GetInterfaceType(uint pLLT)
        {
            return NativeMethods.GetInterfaceType(pLLT);
        }

        public static int DelDevice(uint pLLT)
        {
            return NativeMethods.DelDevice(pLLT);
        }

        // Connect functions
        public static int Connect(uint pLLT)
        {
            return NativeMethods.Connect(pLLT);
        }

        public static int Disconnect(uint pLLT)
        {
            return NativeMethods.Disconnect(pLLT);
        }

        // Write config functions
        public static int ExportLLTConfig(uint pLLT, StringBuilder pFileName)
        {
            return NativeMethods.ExportLLTConfig(pLLT, pFileName);
        }

        // Write config functions
        public static int ExportLLTConfig(uint pLLT, StringBuilder pFileName, int fileSize)
        {
            return NativeMethods.ExportLLTConfigString(pLLT, pFileName, fileSize);
        }

        // Write config functions
        public static int ImportLLTConfig(uint pLLT, StringBuilder pFileName, bool ignoreCalibration)
        {
            return NativeMethods.ImportLLTConfig(pLLT, pFileName, ignoreCalibration);
        }

        // Write config functions
        public static int ImportLLTConfigString(uint pLLT, StringBuilder pFileName, int fileSize, bool ignoreCalibration)
        {
            return NativeMethods.ImportLLTConfigString(pLLT, pFileName, fileSize, ignoreCalibration);
        }

        // Device interface functions
        public static int GetDeviceInterfaces(uint pLLT, uint[] pInterfaces, int nSize)
        {
            return NativeMethods.GetDeviceInterfaces(pLLT, pInterfaces, nSize);
        }

        public static int GetDeviceInterfacesFast(uint pLLT, uint[] pInterfaces, int nSize)
        {
            return NativeMethods.GetDeviceInterfacesFast(pLLT, pInterfaces, nSize);
        }

        public static int SetDeviceInterface(uint pLLT, uint nInterface, int nAdditional)
        {
            return NativeMethods.SetDeviceInterface(pLLT, nInterface, nAdditional);
        }

        public static uint GetDiscoveryBroadcastTarget(uint pLLT)
        {
            return NativeMethods.GetDiscoveryBroadcastTarget(pLLT);
        }

        public static int SetDiscoveryBroadcastTarget(uint pLLT, uint nNetworkAddress, uint nSubnetMask)
        {
            return NativeMethods.SetDiscoveryBroadcastTarget(pLLT, nNetworkAddress, nSubnetMask);
        }

        // scanControl identification functions
        public static int GetDeviceName(uint pLLT, StringBuilder sbDevName, int nDevNameSize, StringBuilder sbVenName, int nVenNameSize)
        {
            return NativeMethods.GetDeviceName(pLLT, sbDevName, nDevNameSize, sbVenName, nVenNameSize);
        }

        public static int GetLLTVersions(uint pLLT, ref uint pDSP, ref uint pFPGA1, ref uint pFPGA2)
        {
            return NativeMethods.GetLLTVersions(pLLT, ref pDSP, ref pFPGA1, ref pFPGA2);
        }

        public static int GetLLTType(uint pLLT, ref TScannerType ScannerType)
        {
            return NativeMethods.GetLLTType(pLLT, ref ScannerType);
        }

        //Get functions
        public static int GetMinMaxPacketSize(uint pLLT, ref ulong pMinPacketSize, ref ulong pMaxPacketSize)
        {
            return NativeMethods.GetMinMaxPacketSize(pLLT, ref pMinPacketSize, ref pMaxPacketSize);
        }

        public static int GetResolutions(uint pLLT, uint[] pValue, int nSize)
        {
            return NativeMethods.GetResolutions(pLLT, pValue, nSize);
        }

        public static int GetFeature(uint pLLT, uint Function, ref uint pValue)
        {
            return NativeMethods.GetFeature(pLLT, Function, ref pValue);
        }

        public static int GetBufferCount(uint pLLT, ref uint pValue)
        {
            return NativeMethods.GetBufferCount(pLLT, ref pValue);
        }

        public static int GetMainReflection(uint pLLT, ref uint pValue)
        {
            return NativeMethods.GetMainReflection(pLLT, ref pValue);
        }

        public static int GetMaxFileSize(uint pLLT, ref uint pValue)
        {
            return NativeMethods.GetMaxFileSize(pLLT, ref pValue);
        }
        public static int GetPacketSize(uint pLLT, ref uint pValue)
        {
            return NativeMethods.GetPacketSize(pLLT, ref pValue);
        }

        public static int GetProfileConfig(uint pLLT, ref TProfileConfig pValue)
        {
            return NativeMethods.GetProfileConfig(pLLT, ref pValue);
        }

        public static int GetResolution(uint pLLT, ref uint pValue)
        {
            return NativeMethods.GetResolution(pLLT, ref pValue);
        }

        public static int GetProfileContainerSize(uint pLLT, ref uint pWidth, ref uint pHeight)
        {
            return NativeMethods.GetProfileContainerSize(pLLT, ref pWidth, ref pHeight);
        }

        public static int GetMaxProfileContainerSize(uint pLLT, ref uint pMaxWidth, ref uint pMaxHeight)
        {
            return NativeMethods.GetMaxProfileContainerSize(pLLT, ref pMaxWidth, ref pMaxHeight);
        }

        public static int GetEthernetHeartbeatTimeout(uint pLLT, ref uint pValue)
        {
            return NativeMethods.GetEthernetHeartbeatTimeout(pLLT, ref pValue);
        }

        //Set functions
        public static int SetFeature(uint pLLT, uint Function, uint Value)
        {
            return NativeMethods.SetFeature(pLLT, Function, Value);
        }

        public static int SetResolution(uint pLLT, uint Value)
        {
            return NativeMethods.SetResolution(pLLT, Value);
        }

        public static int SetProfileConfig(uint pLLT, TProfileConfig Value)
        {
            return NativeMethods.SetProfileConfig(pLLT, Value);
        }

        public static int SetBufferCount(uint pLLT, uint pValue)
        {
            return NativeMethods.SetBufferCount(pLLT, pValue);
        }

        public static int SetMainReflection(uint pLLT, uint pValue)
        {
            return NativeMethods.SetMainReflection(pLLT, pValue);
        }

        public static int SetMaxFileSize(uint pLLT, uint pValue)
        {
            return NativeMethods.SetMaxFileSize(pLLT, pValue);
        }
        public static int SetPacketSize(uint pLLT, uint pValue)
        {
            return NativeMethods.SetPacketSize(pLLT, pValue);
        }

        public static int SetProfileContainerSize(uint pLLT, uint nWidth, uint nHeight)
        {
            return NativeMethods.SetProfileContainerSize(pLLT, nWidth, nHeight);
        }
        public static int SetEthernetHeartbeatTimeout(uint pLLT, uint pValue)
        {
            return NativeMethods.SetEthernetHeartbeatTimeout(pLLT, pValue);
        }

        //Register functions
        public static int RegisterCallback(uint pLLT, TCallbackType tCallbackType, ProfileReceiveMethod tReceiveProfiles, uint pUserData)
        {
            return NativeMethods.RegisterCallback(pLLT, tCallbackType, tReceiveProfiles, pUserData);
        }
        public static int RegisterErrorMsg(uint pLLT, uint Msg, IntPtr hWnd, UIntPtr WParam)
        {
            return NativeMethods.RegisterErrorMsg(pLLT, Msg, hWnd, WParam);
        }

        // Profile transfer functions
        public static int TransferProfiles(uint pLLT, TTransferProfileType TransferProfileType, int nEnable)
        {
            return NativeMethods.TransferProfiles(pLLT, TransferProfileType, nEnable);
        }
        public static int GetProfile(uint pLLT)
        {
            return NativeMethods.GetProfile(pLLT);
        }

        public static int TransferVideoStream(uint pLLT, TTransferVideoType TransferVideoType, int nEnable, ref uint pWidth, ref uint pHeight)
        {
            return NativeMethods.TransferVideoStream(pLLT, TransferVideoType, nEnable, ref pWidth, ref pHeight);
        }

        public static int MultiShot(uint pLLT, uint nCount)
        {
            return NativeMethods.MultiShot(pLLT, nCount);
        }

        public static int GetActualProfile(uint pLLT, byte[] pBuffer, int nBuffersize,
          TProfileConfig ProfileConfig, ref uint pLostProfiles)
        {
            return NativeMethods.GetActualProfile(pLLT, pBuffer, nBuffersize, ProfileConfig, ref pLostProfiles);
        }

        public static int ConvertProfile2Values(uint pLLT, byte[] pProfile, uint nResolution,
          TProfileConfig ProfileConfig, TScannerType ScannerType, uint nReflection, int nConvertToMM,
          ushort[] pWidth, ushort[] pMaximum, ushort[] pThreshold, double[] pX, double[] pZ,
          uint[] pM0, uint[] pM1)
        {
            return NativeMethods.ConvertProfile2Values(pLLT, pProfile, nResolution, ProfileConfig, ScannerType, nReflection, nConvertToMM,
                pWidth, pMaximum, pThreshold, pX, pZ, pM0, pM1);
        }

        public static int ConvertPartProfile2Values(uint pLLT, byte[] pProfile, ref TPartialProfile ProfileConfig, TScannerType ScannerType, uint nReflection, int nConvertToMM,
          ushort[] pWidth, ushort[] pMaximum, ushort[] pThreshold, double[] pX, double[] pZ,
          uint[] pM0, uint[] pM1)
        {
            return NativeMethods.ConvertPartProfile2Values(pLLT, pProfile, ref ProfileConfig, ScannerType, nReflection, nConvertToMM,
                pWidth, pMaximum, pThreshold, pX, pZ, pM0, pM1);
        }
        public static int ConvertContainer2Values(uint pLLT, TConvertContainerParameter parameter)
        {
            return NativeMethods.ConvertContainer2Values(pLLT, parameter);
        }

        public static int SetHoldBuffersForPolling(uint pLLT, uint uiHoldBuffersForPolling)
        {
            return NativeMethods.SetHoldBuffersForPolling(pLLT, uiHoldBuffersForPolling);
        }

        public static int GetHoldBuffersForPolling(uint pLLT, ref uint puiHoldBuffersForPolling)
        {
            return NativeMethods.GetHoldBuffersForPolling(pLLT, ref puiHoldBuffersForPolling);
        }

        //Is functions
        public static int IsInterfaceType(uint pLLT, int iInterfaceType)
        {
            return NativeMethods.IsInterfaceType(pLLT, iInterfaceType);
        }

        public static int IsSerial(uint pLLT)
        {
            return NativeMethods.IsSerial(pLLT);
        }

        public static int IsTransferingProfiles(uint pLLT)
        {
            return NativeMethods.IsTransferingProfiles(pLLT);
        }

        //PartialProfile functions
        public static int GetPartialProfileUnitSize(uint pLLT, ref uint pUnitSizePoint, ref uint pUnitSizePointData)
        {
            return NativeMethods.GetPartialProfileUnitSize(pLLT, ref pUnitSizePoint, ref pUnitSizePointData);
        }

        public static int GetPartialProfile(uint pLLT, ref TPartialProfile pPartialProfile)
        {
            return NativeMethods.GetPartialProfile(pLLT, ref pPartialProfile);
        }

        public static int SetPartialProfile(uint pLLT, ref TPartialProfile pPartialProfile)
        {
            return NativeMethods.SetPartialProfile(pLLT, ref pPartialProfile);
        }

        //Timestamp convert functions,
        public static int Timestamp2CmmTriggerAndInCounter(byte[] pBuffer, ref uint pInCounter, ref int pCmmTrigger, ref int pCmmActive, ref uint pCmmCount)
        {
            return NativeMethods.Timestamp2CmmTriggerAndInCounter(pBuffer, ref pInCounter, ref pCmmTrigger, ref pCmmActive, ref pCmmCount);
        }

        public static int Timestamp2TimeAndCount(byte[] pBuffer, ref double dTimeShutterOpen, ref double dTimeShutterClose, ref uint uiProfileCount)
        {
            return NativeMethods.Timestamp2TimeAndCount(pBuffer, ref dTimeShutterOpen, ref dTimeShutterClose, ref uiProfileCount);
        }

        //PostProcessing functions
        public static int ReadPostProcessingParameter(uint pLLT, ref uint pParameter, uint nSize)
        {
            return NativeMethods.ReadPostProcessingParameter(pLLT, ref pParameter, nSize);
        }
        public static int WritePostProcessingParameter(uint pLLT, ref uint pParameter, uint nSize)
        {
            return NativeMethods.WritePostProcessingParameter(pLLT, ref pParameter, nSize);
        }

        public static int ConvertProfile2ModuleResult(uint pLLT, byte[] pProfileBuffer, uint nProfileBufferSize, byte[] pModuleResultBuffer, uint nResultBufferSize, ref TPartialProfile pPartialProfile /* = NULL*/)
        {
            return NativeMethods.ConvertProfile2ModuleResult(pLLT, pProfileBuffer, nProfileBufferSize, pModuleResultBuffer, nResultBufferSize, ref pPartialProfile);
        }

        //Load/Save functions
        public static int LoadProfiles(uint pLLT, StringBuilder pFilename, ref TPartialProfile pPartialProfile, ref TProfileConfig pProfileConfig, ref TScannerType pScannerType, ref uint pRearrengementProfile)
        {
            return NativeMethods.LoadProfiles(pLLT, pFilename, ref pPartialProfile, ref pProfileConfig, ref pScannerType, ref pRearrengementProfile);
        }

        public static int SaveProfiles(uint pLLT, StringBuilder pFilename, TFileType FileType)
        {
            return NativeMethods.SaveProfiles(pLLT, pFilename, FileType);
        }

        public static int LoadProfilesGetPos(uint pLLT, ref uint pActualPosition, ref uint pMaxPosition)
        {
            return NativeMethods.LoadProfilesGetPos(pLLT, ref pActualPosition, ref pMaxPosition);
        }

        public static int LoadProfilesSetPos(uint pLLT, uint nNewPosition)
        {
            return NativeMethods.LoadProfilesSetPos(pLLT, nNewPosition);
        }

        //Special CMM trigger functions
        public static int StartTransmissionAndCmmTrigger(uint pLLT, uint nCmmTrigger, TTransferProfileType TransferProfileType, uint nProfilesForerun, StringBuilder pFilename, TFileType FileType, uint Timeout)
        {
            return NativeMethods.StartTransmissionAndCmmTrigger(pLLT, nCmmTrigger, TransferProfileType, nProfilesForerun, pFilename, FileType, Timeout);
        }


        public static int StopTransmissionAndCmmTrigger(uint pLLT, int nCmmTriggerPolarity, uint nTimeout)
        {
            return NativeMethods.StopTransmissionAndCmmTrigger(pLLT, nCmmTriggerPolarity, nTimeout);
        }

        // Converts a error-value in a string 
        public static int TranslateErrorValue(uint pLLT, int ErrorValue, byte[] pString, int nStringSize)
        {
            return NativeMethods.TranslateErrorValue(pLLT, ErrorValue, pString, nStringSize);
        }

        //User mode

        public static int GetActualUserMode(uint pLLT, ref uint pActualUserMode, ref uint pUserModeCount)
        {
            return NativeMethods.GetActualUserMode(pLLT, ref pActualUserMode, ref pUserModeCount);
        }

        public static int ReadWriteUserModes(uint pLLT, int nWrite, uint nUserMode)
        {
            return NativeMethods.ReadWriteUserModes(pLLT, nWrite, nUserMode);
        }

        public static int TriggerProfile(uint pLLT)
        {
            return NativeMethods.TriggerProfile(pLLT);
        }

        public static int TriggerContainer(uint pLLT)
        {
            return NativeMethods.TriggerContainer(pLLT);
        }

        public static int ContainerTriggerEnable(uint pLLT)
        {
            return NativeMethods.ContainerTriggerEnable(pLLT);
        }

        public static int ContainerTriggerDisable(uint pLLT)
        {
            return NativeMethods.ContainerTriggerDisable(pLLT);
        }

        public static int SaveGlobalParameter(uint pLLT)
        {
            return NativeMethods.SaveGlobalParameter(pLLT);
        }
        #endregion



    }
    internal static class NativeMethods {

        #region "DLL references"
        public const string DRIVER_DLL_NAME = "LLT.dll";

        // Instance functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_CreateLLTDevice",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern uint CreateLLTDevice(TInterfaceType iInterfaceType);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetInterfaceType",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetInterfaceType(uint pLLT);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_DelDevice",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int DelDevice(uint pLLT);

        // Connect functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_Connect",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int Connect(uint pLLT);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_Disconnect",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int Disconnect(uint pLLT);

        // Write config functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_ExportLLTConfig",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int ExportLLTConfig(uint pLLT, StringBuilder pFileName);

        // Write config functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_ExportLLTConfigString",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int ExportLLTConfigString(uint pLLT, StringBuilder pFileName, int fileSize);

        // Write config functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_ImportLLTConfig",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int ImportLLTConfig(uint pLLT, StringBuilder pFileName, bool ignoreCalibration);

        // Write config functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_ImportLLTConfigString",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int ImportLLTConfigString(uint pLLT, StringBuilder pFileName, int fileSize, bool ignoreCalibration);

        // Device interface functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetDeviceInterfaces",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetDeviceInterfaces(uint pLLT, uint[] pInterfaces, int nSize);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetDeviceInterfacesFast",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetDeviceInterfacesFast(uint pLLT, uint[] pInterfaces, int nSize);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_SetDeviceInterface",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int SetDeviceInterface(uint pLLT, uint nInterface, int nAdditional);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetDiscoveryBroadcastTarget",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern uint GetDiscoveryBroadcastTarget(uint pLLT);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_SetDiscoveryBroadcastTarget",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int SetDiscoveryBroadcastTarget(uint pLLT, uint nNetworkAddress, uint nSubnetMask);

        // scanControl identification functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetDeviceName",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetDeviceName(uint pLLT, StringBuilder sbDevName, int nDevNameSize, StringBuilder sbVenName, int nVenNameSize);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetLLTVersions",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetLLTVersions(uint pLLT, ref uint pDSP, ref uint pFPGA1, ref uint pFPGA2);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetLLTType",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetLLTType(uint pLLT, ref TScannerType ScannerType);

        //Get functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetMinMaxPacketSize",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetMinMaxPacketSize(uint pLLT, ref ulong pMinPacketSize, ref ulong pMaxPacketSize);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetResolutions",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetResolutions(uint pLLT, uint[] pValue, int nSize);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetFeature",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetFeature(uint pLLT, uint Function, ref uint pValue);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetBufferCount",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetBufferCount(uint pLLT, ref uint pValue);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetMainReflection",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetMainReflection(uint pLLT, ref uint pValue);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetMaxFileSize",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetMaxFileSize(uint pLLT, ref uint pValue);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetPacketSize",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetPacketSize(uint pLLT, ref uint pValue);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetProfileConfig",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetProfileConfig(uint pLLT, ref TProfileConfig pValue);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetResolution",
             CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetResolution(uint pLLT, ref uint pValue);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetProfileContainerSize",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetProfileContainerSize(uint pLLT, ref uint pWidth, ref uint pHeight);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetMaxProfileContainerSize",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetMaxProfileContainerSize(uint pLLT, ref uint pMaxWidth, ref uint pMaxHeight);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetEthernetHeartbeatTimeout",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetEthernetHeartbeatTimeout(uint pLLT, ref uint pValue);

        //Set functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_SetFeature",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int SetFeature(uint pLLT, uint Function, uint Value);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_SetResolution",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int SetResolution(uint pLLT, uint Value);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_SetProfileConfig",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int SetProfileConfig(uint pLLT, TProfileConfig Value);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_SetBufferCount",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int SetBufferCount(uint pLLT, uint pValue);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_SetMainReflection",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int SetMainReflection(uint pLLT, uint pValue);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_SetMaxFileSize",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int SetMaxFileSize(uint pLLT, uint pValue);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_SetPacketSize",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int SetPacketSize(uint pLLT, uint pValue);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_SetProfileContainerSize",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int SetProfileContainerSize(uint pLLT, uint nWidth, uint nHeight);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_SetEthernetHeartbeatTimeout",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int SetEthernetHeartbeatTimeout(uint pLLT, uint pValue);

        //Register functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_RegisterCallback",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int RegisterCallback(uint pLLT, TCallbackType tCallbackType, ProfileReceiveMethod tReceiveProfiles, uint pUserData);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_RegisterErrorMsg",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int RegisterErrorMsg(uint pLLT, uint Msg, IntPtr hWnd, UIntPtr WParam);

        // Profile transfer functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_TransferProfiles",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int TransferProfiles(uint pLLT, TTransferProfileType TransferProfileType, int nEnable);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetProfile",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetProfile(uint pLLT);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_TransferVideoStream",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int TransferVideoStream(uint pLLT, TTransferVideoType TransferVideoType, int nEnable, ref uint pWidth, ref uint pHeight);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_MultiShot",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int MultiShot(uint pLLT, uint nCount);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetActualProfile",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetActualProfile(uint pLLT, byte[] pBuffer, int nBuffersize,
          TProfileConfig ProfileConfig, ref uint pLostProfiles);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_ConvertProfile2Values",
           CallingConvention = CallingConvention.StdCall)]
        
        internal static extern int ConvertProfile2Values(uint pLLT, byte[] pProfile, uint nResolution,
          TProfileConfig ProfileConfig, TScannerType ScannerType, uint nReflection, int nConvertToMM,
          ushort[] pWidth, ushort[] pMaximum, ushort[] pThreshold, double[] pX, double[] pZ,
          uint[] pM0, uint[] pM1);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_ConvertPartProfile2Values",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int ConvertPartProfile2Values(uint pLLT, byte[] pProfile, ref TPartialProfile ProfileConfig, TScannerType ScannerType, uint nReflection, int nConvertToMM,
          ushort[] pWidth, ushort[] pMaximum, ushort[] pThreshold, double[] pX, double[] pZ,
          uint[] pM0, uint[] pM1);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_ConvertContainer2Values",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int ConvertContainer2Values(uint pLLT,  TConvertContainerParameter parameter);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_SetHoldBuffersForPolling",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int SetHoldBuffersForPolling(uint pLLT, uint uiHoldBuffersForPolling);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetHoldBuffersForPolling",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetHoldBuffersForPolling(uint pLLT, ref uint puiHoldBuffersForPolling);

        //Is functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_IsInterfaceType",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int IsInterfaceType(uint pLLT, int iInterfaceType);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_IsSerial",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int IsSerial(uint pLLT);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_IsTransferingProfiles",
            CallingConvention = CallingConvention.StdCall)]
        internal static extern int IsTransferingProfiles(uint pLLT);

        //PartialProfile functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetPartialProfileUnitSize",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetPartialProfileUnitSize(uint pLLT, ref uint pUnitSizePoint, ref uint pUnitSizePointData);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetPartialProfile",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetPartialProfile(uint pLLT, ref TPartialProfile pPartialProfile);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_SetPartialProfile",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int SetPartialProfile(uint pLLT, ref TPartialProfile pPartialProfile);

        //Timestamp convert functions,
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_Timestamp2CmmTriggerAndInCounter",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int Timestamp2CmmTriggerAndInCounter(byte[] pBuffer, ref uint pInCounter, ref int pCmmTrigger, ref int pCmmActive, ref uint pCmmCount);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_Timestamp2TimeAndCount",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int Timestamp2TimeAndCount(byte[] pBuffer, ref double dTimeShutterOpen, ref double dTimeShutterClose, ref uint uiProfileCount);

        //PostProcessing functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_ReadPostProcessingParameter",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int ReadPostProcessingParameter(uint pLLT, ref uint pParameter, uint nSize);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_WritePostProcessingParameter",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int WritePostProcessingParameter(uint pLLT, ref uint pParameter, uint nSize);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_ConvertProfile2ModuleResult",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int ConvertProfile2ModuleResult(uint pLLT, byte[] pProfileBuffer, uint nProfileBufferSize, byte[] pModuleResultBuffer, uint nResultBufferSize, ref TPartialProfile pPartialProfile /* = NULL*/);

        //Load/Save functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_LoadProfiles",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int LoadProfiles(uint pLLT, StringBuilder pFilename, ref TPartialProfile pPartialProfile, ref TProfileConfig pProfileConfig, ref TScannerType pScannerType, ref uint pRearrengementProfile);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_SaveProfiles",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int SaveProfiles(uint pLLT, StringBuilder pFilename, TFileType FileType);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_LoadProfilesGetPos",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int LoadProfilesGetPos(uint pLLT, ref uint pActualPosition, ref uint pMaxPosition);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_LoadProfilesSetPos",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int LoadProfilesSetPos(uint pLLT, uint nNewPosition);

        //Special CMM trigger functions
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_StartTransmissionAndCmmTrigger",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int StartTransmissionAndCmmTrigger(uint pLLT, uint nCmmTrigger, TTransferProfileType TransferProfileType, uint nProfilesForerun, StringBuilder pFilename, TFileType FileType, uint Timeout);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_StopTransmissionAndCmmTrigger",
           CallingConvention = CallingConvention.StdCall)]
        internal static extern int StopTransmissionAndCmmTrigger(uint pLLT, int nCmmTriggerPolarity, uint nTimeout);

        // Converts a error-value in a string 
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_TranslateErrorValue",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int TranslateErrorValue(uint pLLT, int ErrorValue, byte[] pString, int nStringSize);

        //User mode
        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_GetActualUserMode",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int GetActualUserMode(uint pLLT, ref uint pActualUserMode, ref uint pUserModeCount);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_ReadWriteUserModes",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int ReadWriteUserModes(uint pLLT, int nWrite, uint nUserMode);
		
		[DllImport(DRIVER_DLL_NAME, EntryPoint = "s_TriggerProfile",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int TriggerProfile(uint pLLT);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_TriggerContainer",
        CallingConvention = CallingConvention.StdCall)]
        internal static extern int TriggerContainer(uint pLLT);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_ContainerTriggerEnable",
        CallingConvention = CallingConvention.StdCall)]
        internal static extern int ContainerTriggerEnable(uint pLLT);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_ContainerTriggerDisable",
        CallingConvention = CallingConvention.StdCall)]
        internal static extern int ContainerTriggerDisable(uint pLLT);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_FlushContainer",
        CallingConvention = CallingConvention.StdCall)]
        internal static extern int FlushContainer(uint pLLT);

        [DllImport(DRIVER_DLL_NAME, EntryPoint = "s_SaveGlobalParameter",
          CallingConvention = CallingConvention.StdCall)]
        internal static extern int SaveGlobalParameter(uint pLLT);

        #endregion

 
    }
}
