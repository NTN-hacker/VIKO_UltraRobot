###### c# ########
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using System.Threading;
using DevExpress.Mvvm;
using SciChart.Charting.Model.DataSeries;

namespace WeldLaserScanner.LLT
{

    public class ScanControl    
    {
        public const int MAX_INTERFACE_COUNT = 5;
        public const int MAX_RESOULUTIONS = 6;

        static public uint uiResolution = 1024;
        static public uint hLLT = 0;
        static public TScannerType tscanCONTROLType;
        static public TConvertContainerParameter convertContainerParameter;

        static public uint uiExposureTime = 100;
        static public uint uiIdleTime = 100;

        int iRetValue;
        uint uiFieldCount = 4;    // Number of fields transmitted (TS is one field)
        uint uiProfileCount = 32;  // Number of profiles in one container
        uint uiProfileCounter = 0;
        uint uiInquiry = 0;
        uint uiLostProfiles = 0;
        ushort usValue = 0;
        double dTimeShutterOpen = 0.0;
        double dTimeShutterClose = 0.0;
        bool noContainerReceived = true;



        static public double raster_x = 0;
        static public double raster_z = 0;




        public ScanControl()
        {
            uint[] auiInterfaces = new uint[MAX_INTERFACE_COUNT];
            uint[] auiResolutions = new uint[MAX_RESOULUTIONS];

            StringBuilder sbDevName = new StringBuilder(100);
            StringBuilder sbVenName = new StringBuilder(100);

            uint uiBufferCount = 50, uiPacketSize = 320;

            int iInterfaceCount = 0;

            int iRetValue;
            bool bOK = true;
            bool bConnected = false;
            ConsoleKeyInfo cki;

            hLLT = 0;


            hLLT = CLLTI.CreateLLTDevice(TInterfaceType.INTF_TYPE_ETHERNET);
            if (hLLT != 0)
                Console.WriteLine("CreateLLTDevice OK");
            else
                Console.WriteLine("Error during CreateLLTDevice\n");


            iInterfaceCount = CLLTI.GetDeviceInterfacesFast(hLLT, auiInterfaces, auiInterfaces.GetLength(0));
            if (iInterfaceCount <= 0)
                Console.WriteLine("FAST: There is no scanCONTROL connected");
            else if (iInterfaceCount == 1)
                Console.WriteLine("FAST: There is 1 scanCONTROL connected ");
            else
                Console.WriteLine("FAST: There are " + iInterfaceCount + " scanCONTROL's connected");


            if (iInterfaceCount >= 1)
            {
                uint target4 = auiInterfaces[0] & 0x000000FF;
                uint target3 = (auiInterfaces[0] & 0x0000FF00) >> 8;
                uint target2 = (auiInterfaces[0] & 0x00FF0000) >> 16;
                uint target1 = (auiInterfaces[0] & 0xFF000000) >> 24;

                // Set the first IP address detected by GetDeviceInterfacesFast to handle
                Console.WriteLine("Select the device interface: " + target1 + "." + target2 + "." + target3 + "." + target4);
                if ((iRetValue = CLLTI.SetDeviceInterface(hLLT, auiInterfaces[0], 0)) < CLLTI.GENERAL_FUNCTION_OK)
                    return;
                if ((iRetValue = CLLTI.Connect(hLLT)) < CLLTI.GENERAL_FUNCTION_OK)
                    return;
                if ((iRetValue = CLLTI.GetDeviceName(hLLT, sbDevName, sbDevName.Capacity, sbVenName, sbVenName.Capacity)) < CLLTI.GENERAL_FUNCTION_OK)
                    return;

                Console.WriteLine(" - Devname: " + sbDevName + "\n - Venname: " + sbVenName);
                if ((iRetValue = CLLTI.GetLLTType(hLLT, ref tscanCONTROLType)) < CLLTI.GENERAL_FUNCTION_OK)
                        return;
                if ((iRetValue = CLLTI.GetResolutions(hLLT, auiResolutions, auiResolutions.GetLength(0))) < CLLTI.GENERAL_FUNCTION_OK)
                        return;

                uiResolution = (uint) Store.Ins.SensorConfig.PointPerProfile;
                CLLTI.SetResolution(hLLT, uiResolution);

                uint uiWorkingUserMode = 2;
                if ((iRetValue = CLLTI.ReadWriteUserModes(hLLT, 0, uiWorkingUserMode)) < CLLTI.GENERAL_FUNCTION_OK)
                {
                    OnError("Error during loading UM 4", iRetValue);
                    bOK = false;
                }
            }
        }

        private void SetRoi()
        {
            int iRetValue;
            ushort col_start;
            ushort col_size;
            ushort row_start;
            ushort row_size;

            // Percentage X/Z of ROI
            double start_z = 70;
            double end_z = 90;

            double start_x = 25;
            double end_x = 75;

            if (tscanCONTROLType >= TScannerType.scanCONTROL26xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL26xx_xxx)
            {
                col_start = (ushort)(65535 - ((Math.Round(end_x / raster_x) * raster_x) / 100 * 65535));
                col_size = (ushort)(Math.Round((end_x - start_x) / raster_x) * raster_x / 100 * 65535);
                row_start = (ushort)(Math.Round(start_z / raster_z) * raster_z / 100 * 65536);
                row_size = (ushort)(Math.Round((end_z - start_z) / raster_z) * raster_z / 100 * 65535);

            }
            else if (tscanCONTROLType >= TScannerType.scanCONTROL25xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL25xx_xxx)
            {
                col_start = (ushort)(65535 - ((Math.Round(end_x / raster_x) * raster_x) / 100 * 65535));
                col_size = (ushort)(Math.Round((end_x - start_x) / raster_x) * raster_x / 100 * 65535);
                row_start = (ushort)(65535 - ((Math.Round(end_z / raster_z) * raster_z) / 100 * 65535));
                row_size = (ushort)(Math.Round((end_z - start_z) / raster_z) * raster_z / 100 * 65535);
            }
            else if (tscanCONTROLType >= TScannerType.scanCONTROL27xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL27xx_xxx)
            {
                col_start = (ushort)(65535 - ((Math.Round(end_x / raster_x) * raster_x) / 100 * 65535));
                col_size = (ushort)(Math.Round((end_x - start_x) / raster_x) * raster_x / 100 * 65535);
                row_start = (ushort)(65535 - ((Math.Round(end_z / raster_z) * raster_z) / 100 * 65535));
                row_size = (ushort)(Math.Round((end_z - start_z) / raster_z) * raster_z / 100 * 65535);
            }
            else if (tscanCONTROLType >= TScannerType.scanCONTROL29xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL29xx_xxx)
            {
                col_start = (ushort)(65535 - ((Math.Round(end_x / raster_x) * raster_x) / 100 * 65535));
                col_size = (ushort)(Math.Round((end_x - start_x) / raster_x) * raster_x / 100 * 65535);
                row_start = (ushort)(65535 - ((Math.Round(end_z / raster_z) * raster_z) / 100 * 65535));
                row_size = (ushort)(Math.Round((end_z - start_z) / raster_z) * raster_z / 100 * 65535);
            }
            else if (tscanCONTROLType >= TScannerType.scanCONTROL30xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL30xx_xxx)
            {
                col_start = (ushort)(Math.Round(start_x / raster_x) * raster_x / 100 * 65536);
                col_size = (ushort)(Math.Round((end_x - start_x) / raster_x) * raster_x / 100 * 65535);
                row_start = (ushort)(Math.Round(start_z / raster_z) * raster_z / 100 * 65536);
                row_size = (ushort)(Math.Round((end_z - start_z) / raster_z) * raster_z / 100 * 65535);
            }
            else
            {
                Console.WriteLine("The scanCONTROL is a undefined type\nPlease contact Micro-Epsilon for a newer SDK\n\n");
                return;
            }

            Console.WriteLine("Enable ROI1 free region");
            if ((iRetValue = CLLTI.SetFeature(hLLT, CLLTI.FEATURE_FUNCTION_ROI1_PRESET, 0x800)) < CLLTI.GENERAL_FUNCTION_OK)
            {
                OnError("Error during SetFeature(FEATURE_FUNCTION_ROI1_PRESET)", iRetValue);
            }

            Console.WriteLine("Set ROI1_Position parameter");
            if ((iRetValue = CLLTI.SetFeature(hLLT, CLLTI.FEATURE_FUNCTION_ROI1_POSITION, (uint)(col_start << 16) + col_size)) < CLLTI.GENERAL_FUNCTION_OK)
            {
                OnError("Error during SetFeature(FEATURE_FUNCTION_ROI1_POSITION)", iRetValue);
            }

            Console.WriteLine("Set ROI1_Distance parameter");
            if ((iRetValue = CLLTI.SetFeature(hLLT, CLLTI.FEATURE_FUNCTION_ROI1_DISTANCE, (uint)(row_start << 16) + row_size)) < CLLTI.GENERAL_FUNCTION_OK)
            {
                OnError("Error during SetFeature(FEATURE_FUNCTION_ROI1_DISTANCE)", iRetValue);
            }

            Console.WriteLine("Activate ROI1 free region\n\n");
            if ((iRetValue = CLLTI.SetFeature(hLLT, CLLTI.FEATURE_FUNCTION_EXTRA_PARAMETER, 0)) < CLLTI.GENERAL_FUNCTION_OK)
            {
                OnError("Error during SetFeature(FEATURE_FUNCTION_EXTRA_PARAMETER)", iRetValue);
            }

          
        }

        static void OnError(string strErrorTxt, int iErrorValue)
        {
            byte[] acErrorString = new byte[200];

            Console.WriteLine(strErrorTxt);
            if (CLLTI.TranslateErrorValue(hLLT, iErrorValue, acErrorString, acErrorString.GetLength(0))
                                            >= CLLTI.GENERAL_FUNCTION_OK)
                Console.WriteLine(System.Text.Encoding.ASCII.GetString(acErrorString, 0, acErrorString.GetLength(0)));
        }


        private void GetRasterResolution()
        {
            if (tscanCONTROLType >= TScannerType.scanCONTROL27xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL27xx_xxx)
            {
                Console.WriteLine(" - The scanCONTROL is a scanCONTROL27xx");
                raster_x = 1.25;
                raster_z = 100.0 / 480.0;
            }
            else if (tscanCONTROLType >= TScannerType.scanCONTROL25xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL25xx_xxx)
            {
                Console.WriteLine(" - The scanCONTROL is a scanCONTROL25xx");
                raster_x = 2.5;
                raster_z = 100.0 / 1024.0;
            }
            else if (tscanCONTROLType >= TScannerType.scanCONTROL26xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL26xx_xxx)
            {
                Console.WriteLine(" - The scanCONTROL is a scanCONTROL26xx");
                raster_x = 1.25;
                raster_z = 100.0 / 480.0;
            }
            else if (tscanCONTROLType >= TScannerType.scanCONTROL29xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL29xx_xxx)
            {
                Console.WriteLine(" - The scanCONTROL is a scanCONTROL29xx");
                raster_x = 2.5;
                raster_z = 100.0 / 1024.0;
            }
            else if (tscanCONTROLType >= TScannerType.scanCONTROL30xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL30xx_xxx)
            {
                Console.WriteLine(" - The scanCONTROL is a scanCONTROL30xx");
                raster_x = 1.5625;
                raster_z = 200.0 / 1088.0;
            }
            else
            {
                Console.WriteLine(" - The scanCONTROL is a undefined type\nPlease contact Micro-Epsilon for a newer SDK");
            }
        }


        public void Config()
        {
          

        }

        public void TransferData ()
        {
            Thread thread = new Thread(TransferDataThreading);
            thread.Priority = ThreadPriority.Highest;
            thread.Start();
        }

        private bool countinueTransfer = false;

        public void StopTransfer()
        {
            countinueTransfer = false;
        }

        public void LoadProfile(uint uiWorkingUserMode)
        {
         
            if ((iRetValue = CLLTI.ReadWriteUserModes(hLLT, 0, uiWorkingUserMode)) < CLLTI.GENERAL_FUNCTION_OK)
            {
                OnError("Error during loading UM 4", iRetValue);
             
            }
        }

        public void TransferDataThreading()
        {
           
            double dTempLog = 1.0 / Math.Log(2.0);
            uint uiResolutionBitField = (uint)Math.Floor((Math.Log((double)uiResolution) * dTempLog) + 0.5);

            if ((iRetValue = CLLTI.SetFeature(hLLT, CLLTI.FEATURE_FUNCTION_PROFILE_REARRANGEMENT,
                (CLLTI.CONTAINER_STRIPE_1 | CLLTI.CONTAINER_DATA_Z | CLLTI.CONTAINER_DATA_X |
                CLLTI.CONTAINER_DATA_INTENS | CLLTI.CONTAINER_DATA_TS | CLLTI.CONTAINER_DATA_EMPTYFIELD4TS
                | (uiResolutionBitField << 12)))) < CLLTI.GENERAL_FUNCTION_OK)
            {
                return;
            }
            // Read out the rearrangement parameter
            if ((iRetValue = CLLTI.GetFeature(hLLT, CLLTI.FEATURE_FUNCTION_PROFILE_REARRANGEMENT, ref uiInquiry)) < CLLTI.GENERAL_FUNCTION_OK)
            {
                return;
            }
            // Set the profile container size according to the given profile count
            if ((iRetValue = CLLTI.SetProfileContainerSize(hLLT, 0, uiProfileCount)) < CLLTI.GENERAL_FUNCTION_OK)
            {
                return;
            }
            //if ((iRetValue = CLLTI.SetFeature(hLLT, CLLTI.FEATURE_FUNCTION_TRIGGER, CLLTI.TRIG_INTERNAL)) < CLLTI.GENERAL_FUNCTION_OK)
            //{
            //    return;
            //}

            System.Threading.Thread.Sleep(500);

            countinueTransfer = true;

            //Declare array
            double[] adValueX = new double[uiResolution * uiProfileCount];
            double[] adValueZ = new double[uiResolution * uiProfileCount];
            ushort[] intens = new ushort[uiResolution * uiProfileCount];
            double[] DisplayX = new double[uiResolution];
            double[] DisplayZ = new double[uiResolution];



            // Start continous profile transmission
            if ((iRetValue = CLLTI.TransferProfiles(hLLT, TTransferProfileType.NORMAL_CONTAINER_MODE, 1)) < CLLTI.GENERAL_FUNCTION_OK)
            {
                return;
            }

            // Allocate buffersize according to transmitted data
          
            byte[] abyTimestamp = new byte[16];
            Store store = Store.Ins;

            int counter = 0;

            Console.WriteLine("Start Transfer Data");

            while(countinueTransfer)
            {
                noContainerReceived = true;
                byte[] abyContainerBuffer = new byte[uiResolution * 2 * uiFieldCount * uiProfileCount]; // 2* because 1 value has 2 bytes

                while (noContainerReceived && countinueTransfer)
                {
                    if ((iRetValue = CLLTI.GetActualProfile(hLLT, abyContainerBuffer, abyContainerBuffer.GetLength(0), TProfileConfig.CONTAINER, ref uiLostProfiles)) != abyContainerBuffer.GetLength(0))
                    {
                        if (iRetValue == CLLTI.ERROR_PROFTRANS_NO_NEW_PROFILE)
                        {
                            //System.Threading.Thread.Sleep((int)(uiIdleTime + uiExposureTime) / 100);

                            System.Threading.Thread.Sleep((int)(uiIdleTime + uiExposureTime) / 100);
                            noContainerReceived = true;
                        }
                        else
                        {
                            return;
                        }
                    }
                    else
                    {
                        noContainerReceived = false;
                    }

                    //Console.WriteLine(string.Format("Lost profile: {0}", uiLostProfiles));
                }             


                ProfileData profile = new ProfileData();
                profile.abyContainerBuffer = abyContainerBuffer;
                profile.uiInquiry = uiInquiry;
                Store.Ins.ScanDataBuffer.Enqueue(profile);
             
                //Console.WriteLine("Enqueue");
            }

            if ((iRetValue = CLLTI.TransferProfiles(hLLT, TTransferProfileType.NORMAL_CONTAINER_MODE, 0)) < CLLTI.GENERAL_FUNCTION_OK)
            {
                return;
            }
        }
    }














    //public class ScanControl

    //{  /* Global variables */
    //    public const int MAX_INTERFACE_COUNT = 5;
    //    public const int MAX_RESOULUTIONS = 6;
    //    static public uint uiResolution = 0;
    //    static public uint hLLT = 0;
    //    static public TScannerType tscanCONTROLType;

    //    static public byte[] abyProfileBuffer;

    //    static public uint bProfileReceived = 0;
    //    static public uint uiNeededProfileCount = 100;
    //    static public uint uiProfileDataSize = 0;

    //    // Define an array with two AutoResetEvent WaitHandles. 
    //    static AutoResetEvent hProfileEvent = new AutoResetEvent(false);

    //    static ProfileReceiveMethod fnProfileReceiveMethod = null;

    //    public ScanControl()
    //    {
            
    //    }

    //    public void Config()
    //    {
     

    //        uint[] auiInterfaces = new uint[MAX_INTERFACE_COUNT];
    //        uint[] auiResolutions = new uint[MAX_RESOULUTIONS];

    //        StringBuilder sbDevName = new StringBuilder(100);
    //        StringBuilder sbVenName = new StringBuilder(100);

          

    //        uint uiBufferCount = 200, uiMainReflection = 0, uiPacketSize = 1024;

    //        int iInterfaceCount = 0;
    //        uint uiExposureTime = 1000;
    //        uint uiIdleTime = 3900;
    //        int iRetValue;
    //        bool bOK = true;
    //        bool bConnected = false;
    //        ConsoleKeyInfo cki;

    //        hLLT = 0;
    //        uiResolution = 0;

    //        //Create a Ethernet Device -> returns handle to LLT device
    //        hLLT = CLLTI.CreateLLTDevice(TInterfaceType.INTF_TYPE_ETHERNET);
    //        if (hLLT != 0)
    //            Console.WriteLine("CreateLLTDevice OK");
    //        else
    //            Console.WriteLine("Error during CreateLLTDevice\n");



    //        //Gets the available interfaces from the scanCONTROL-device
    //        iInterfaceCount = CLLTI.GetDeviceInterfacesFast(hLLT, auiInterfaces, auiInterfaces.GetLength(0));
    //        if (iInterfaceCount <= 0)
    //            Console.WriteLine("FAST: There is no scanCONTROL connected");
    //        else if (iInterfaceCount == 1)
    //            Console.WriteLine("FAST: There is 1 scanCONTROL connected ");
    //        else
    //            Console.WriteLine("FAST: There are " + iInterfaceCount + " scanCONTROL's connected");


    //        if (iInterfaceCount >= 1)
    //        {
    //            uint target4 = auiInterfaces[0] & 0x000000FF;
    //            uint target3 = (auiInterfaces[0] & 0x0000FF00) >> 8;
    //            uint target2 = (auiInterfaces[0] & 0x00FF0000) >> 16;
    //            uint target1 = (auiInterfaces[0] & 0xFF000000) >> 24;

    //            if ((iRetValue = CLLTI.SetDeviceInterface(hLLT, auiInterfaces[0], 0)) < CLLTI.GENERAL_FUNCTION_OK)
    //            {
    //                OnError("Error during SetDeviceInterface", iRetValue);
    //                bOK = false;
    //            }


    //            if (bOK)
    //            {
    //                // Connect to sensor with the device interface set before                
    //                if ((iRetValue = CLLTI.Connect(hLLT)) < CLLTI.GENERAL_FUNCTION_OK)
    //                {
    //                    OnError("Error during Connect", iRetValue);
    //                    bOK = false;
    //                }
    //                else
    //                    bConnected = true;
    //            }

    //            if (bOK)
    //            {
    //                if ((iRetValue = CLLTI.GetDeviceName(hLLT, sbDevName, sbDevName.Capacity, sbVenName, sbVenName.Capacity)) < CLLTI.GENERAL_FUNCTION_OK)
    //                {
    //                    OnError("Error during GetDevName", iRetValue);
    //                    bOK = false;
    //                }
    //                else
    //                {
    //                    Console.WriteLine(" - Devname: " + sbDevName + "\n - Venname: " + sbVenName);
    //                }
    //            }

    //            if (bOK)
    //            {
    //                // Get the scanCONTROL type and check if it is valid
    //                Console.WriteLine("Get scanCONTROL type");
    //                if ((iRetValue = CLLTI.GetLLTType(hLLT, ref tscanCONTROLType)) < CLLTI.GENERAL_FUNCTION_OK)
    //                {
    //                    OnError("Error during GetLLTType", iRetValue);
    //                    bOK = false;
    //                }

    //                if (iRetValue == CLLTI.GENERAL_FUNCTION_DEVICE_NAME_NOT_SUPPORTED)
    //                {
    //                    Console.WriteLine(" - Can't decode scanCONTROL type. Please contact Micro-Epsilon for a newer version of the LLT.dll.");
    //                }

    //                if (tscanCONTROLType >= TScannerType.scanCONTROL27xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL27xx_xxx)
    //                {
    //                    Console.WriteLine(" - The scanCONTROL is a scanCONTROL27xx");
    //                }
    //                else if (tscanCONTROLType >= TScannerType.scanCONTROL25xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL25xx_xxx)
    //                {
    //                    Console.WriteLine(" - The scanCONTROL is a scanCONTROL25xx");
    //                }
    //                else if (tscanCONTROLType >= TScannerType.scanCONTROL26xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL26xx_xxx)
    //                {
    //                    Console.WriteLine(" - The scanCONTROL is a scanCONTROL26xx");
    //                }
    //                else if (tscanCONTROLType >= TScannerType.scanCONTROL29xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL29xx_xxx)
    //                {
    //                    Console.WriteLine(" - The scanCONTROL is a scanCONTROL29xx");
    //                }
    //                else if (tscanCONTROLType >= TScannerType.scanCONTROL30xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL30xx_xxx)
    //                {
    //                    Console.WriteLine(" - The scanCONTROL is a scanCONTROL30xx");
    //                }
    //                else
    //                {
    //                    Console.WriteLine(" - The scanCONTROL is a undefined type\nPlease contact Micro-Epsilon for a newer SDK");
    //                }

    //                // Get all possible resolutions for connected sensor and save them in array 
    //                Console.WriteLine("Get all possible resolutions");
    //                if ((iRetValue = CLLTI.GetResolutions(hLLT, auiResolutions, auiResolutions.GetLength(0))) < CLLTI.GENERAL_FUNCTION_OK)
    //                {
    //                    OnError("Error during GetResolutions", iRetValue);
    //                    bOK = false;
    //                }

    //                // Set the max. possible resolution
    //                uiResolution = auiResolutions[0];
    //            }

    //            if (bOK)
    //            {
    //                Console.WriteLine("\n----- Set scanCONTROL Parameters -----\n");

    //                Console.WriteLine("Set resolution to " + uiResolution);
    //                if ((iRetValue = CLLTI.SetResolution(hLLT, uiResolution)) < CLLTI.GENERAL_FUNCTION_OK)
    //                {
    //                    OnError("Error during SetResolution", iRetValue);
    //                    bOK = false;
    //                }
    //            }

    //            if (bOK)
    //            {
    //                Console.WriteLine("Set BufferCount to " + uiBufferCount);
    //                if ((iRetValue = CLLTI.SetBufferCount(hLLT, uiBufferCount)) < CLLTI.GENERAL_FUNCTION_OK)
    //                {
    //                    OnError("Error during SetBufferCount", iRetValue);
    //                    bOK = false;
    //                }
    //            }

    //            if (bOK)
    //            {
    //                Console.WriteLine("Set MainReflection to " + uiMainReflection);
    //                if ((iRetValue = CLLTI.SetMainReflection(hLLT, uiMainReflection)) < CLLTI.GENERAL_FUNCTION_OK)
    //                {
    //                    OnError("Error during SetMainReflection", iRetValue);
    //                    bOK = false;
    //                }
    //            }

    //            if (bOK)
    //            {
    //                Console.WriteLine("Set Packetsize to " + uiPacketSize);
    //                if ((iRetValue = CLLTI.SetPacketSize(hLLT, uiPacketSize)) < CLLTI.GENERAL_FUNCTION_OK)
    //                {
    //                    OnError("Error during SetPacketSize", iRetValue);
    //                    bOK = false;
    //                }
    //            }

    //            if (bOK)
    //            {
    //                Console.WriteLine("Set Profile config to PROFILE");
    //                if ((iRetValue = CLLTI.SetProfileConfig(hLLT, TProfileConfig.PROFILE)) < CLLTI.GENERAL_FUNCTION_OK)
    //                {
    //                    OnError("Error during SetProfileConfig", iRetValue);
    //                    bOK = false;
    //                }
    //            }

    //            if (bOK)
    //            {
    //                Console.WriteLine("Set trigger to internal");
    //                if ((iRetValue = CLLTI.SetFeature(hLLT, CLLTI.FEATURE_FUNCTION_TRIGGER, CLLTI.TRIG_INTERNAL)) < CLLTI.GENERAL_FUNCTION_OK)
    //                {
    //                    OnError("Error during SetFeature(FEATURE_FUNCTION_TRIGGER)", iRetValue);
    //                    bOK = false;
    //                }
    //            }

    //            if (bOK)
    //            {
    //                Console.WriteLine("Set exposure time to " + uiExposureTime);
    //                if ((iRetValue = CLLTI.SetFeature(hLLT, CLLTI.FEATURE_FUNCTION_EXPOSURE_TIME, uiExposureTime)) < CLLTI.GENERAL_FUNCTION_OK)
    //                {
    //                    OnError("Error during SetFeature(FEATURE_FUNCTION_EXPOSURE_TIME)", iRetValue);
    //                    bOK = false;
    //                }
    //            }

    //            if (bOK)
    //            {
    //                Console.WriteLine("Set idle time to " + uiIdleTime);
    //                if ((iRetValue = CLLTI.SetFeature(hLLT, CLLTI.FEATURE_FUNCTION_IDLE_TIME, uiIdleTime)) < CLLTI.GENERAL_FUNCTION_OK)
    //                {
    //                    OnError("Error during SetFeature(FEATURE_FUNCTION_IDLE_TIME)", iRetValue);
    //                    bOK = false;
    //                }
    //            }

    //            // Setup callback
    //            if (bOK)
    //            {
    //                Console.WriteLine("\n----- Setup Callback function and event -----\n");

    //                Console.WriteLine("Register the callback");
    //                // Set the callback function
    //                unsafe
    //                {
    //                    fnProfileReceiveMethod = new ProfileReceiveMethod(ProfileEvent);
    //                }


    //                // Register the callback
    //                if ((iRetValue = CLLTI.RegisterCallback(hLLT, TCallbackType.STD_CALL, fnProfileReceiveMethod, hLLT)) < CLLTI.GENERAL_FUNCTION_OK)
    //                {
    //                    OnError("Error during RegisterCallback", iRetValue);
    //                    bOK = false;
    //                }
    //            }

    //            // Main tasks in this example
    //            if (bOK)
    //            {
    //                Console.WriteLine("\n----- Get profiles with Callback from scanCONTROL -----\n");

    //                GetProfiles_Callback();
    //            }

    //        }
    //    }

    //    static void GetProfiles_Callback()
    //    {
    //        int iRetValue;
    //        double[] adValueX = new double[uiResolution];
    //        double[] adValueZ = new double[uiResolution];

    //        // Allocate the profile buffer to the maximal profile size times the profile count
    //        abyProfileBuffer = new byte[uiResolution * 64 * uiNeededProfileCount];
    //        byte[] abyTimestamp = new byte[16];

    //        // Start continous profile transmission
    //        Console.WriteLine("Enable the measurement");
    //        if ((iRetValue = CLLTI.TransferProfiles(hLLT, TTransferProfileType.NORMAL_TRANSFER, 1)) < CLLTI.GENERAL_FUNCTION_OK)
    //        {
    //            OnError("Error during TransferProfiles", iRetValue);
    //            return;
    //        }
                 
    //    }

    //    static unsafe void ProfileEvent(byte* data, uint uiSize, uint uiUserData)
    //    {
    //        if (uiSize > 0)
    //        {

    //            //If the needed profile count not arrived: copy the new Profile in the buffer and increase the recived buffer count
    //            uiProfileDataSize = uiSize;

    //            //byte[] abyProfileBuffer = new byte[uiSize];
    //            //fixed (byte* dst = &abyProfileBuffer[0])
    //            //{
    //            //    memcpy(dst, data, uiSize);
    //            //}
    //            bProfileReceived++;

         
    //            //double[] adValueX = new double[uiResolution];
    //            //double[] adValueZ = new double[uiResolution];
    //            //int iRetValue = CLLTI.ConvertProfile2Values(hLLT, abyProfileBuffer, uiResolution, TProfileConfig.PROFILE, tscanCONTROLType,
    //            //                        0, 1, null, null, null, adValueX, adValueZ, null, null);

    //            //if (((iRetValue & CLLTI.CONVERT_X) == 0) || ((iRetValue & CLLTI.CONVERT_Z) == 0))
    //            //{
    //            //    OnError("Error during Converting of profile data", iRetValue);
    //            //    return;
    //            //}

    //            //List<double[]> profile = new List<double[]>();
    //            //profile.Add(adValueX);
    //            //profile.Add(adValueZ);

    //            //Store.Ins.ProfileDatas.Enqueue(new ProfileData { ValueX = adValueX, ValueZ = adValueZ });

    //            Console.WriteLine(bProfileReceived);

    //        }
    //    }


    //    [DllImport("msvcrt.dll", EntryPoint = "memcpy", CallingConvention = CallingConvention.Cdecl, SetLastError = false)]
    //    static unsafe extern IntPtr memcpy(byte* dest, byte* src, uint count);

    //    static void OnError(string strErrorTxt, int iErrorValue)
    //    {
    //        byte[] acErrorString = new byte[200];

    //        Console.WriteLine(strErrorTxt);
    //        if (CLLTI.TranslateErrorValue(hLLT, iErrorValue, acErrorString, acErrorString.GetLength(0))
    //                                        >= CLLTI.GENERAL_FUNCTION_OK)
    //            Console.WriteLine(System.Text.Encoding.ASCII.GetString(acErrorString, 0, acErrorString.GetLength(0)));
    //    }
    //}
}
