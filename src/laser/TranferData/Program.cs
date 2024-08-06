//###### c# ########
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Runtime.InteropServices;
using System.Threading;
using System.IO.MemoryMappedFiles;
using System.Text.Json;
using System.Threading.Tasks.Dataflow;

// using DevExpress.Mvvm;
// using SciChart.Charting.Model.DataSeries;

namespace MEScanControl
{
        public class ProfileData
    {
        public byte[] ContainerBuffer { get; set; }
        public uint Inquiry { get; set; }
    }

        public class ScanControl    
    {
        public double dLengthWeld {get; set;}
        public double dResolutionY  {get; set;}



        
        public const int MAX_INTERFACE_COUNT = 5;
        public const int MAX_RESOULUTIONS = 6;

        static public uint uiResolution = 1024;
        static public uint hLLT = 0;
        static public TScannerType tscanCONTROLType;
        static public TConvertContainerParameter convertContainerParameter;

        static public uint uiExposureTime = 100;
        static public uint uiIdleTime = 100;

        int iRetValue;
        uint uiFieldCount = 4;    // Number of fields transmitted (TS is one field), is field to send to buffer (X, Y, Z, Timestamp)
        uint uiProfileCount = 32;  // Number of profiles in one container

        
        uint uiProfileCounter = 0;

        double dShutterOpen = 0, dShutterClose = 0;
        uint uiInquiry = 0;
        uint uiLostProfiles = 0;
        ushort usValue = 0;
        double dTimeShutterOpen = 0.0;
        double dTimeShutterClose = 0.0;
        bool noContainerReceived = true;

        uint uiProfilecountStart = 0;
        double dShutteropenStart = 0.0;
    
        private static System.Timers.Timer transferTimer;


        static public double raster_x = 0;
        static public double raster_z = 0;

        public long count_data = 0;

        private bool continueTransfer;
        private Queue<ProfileData> dataQueue = new Queue<ProfileData>();
        private Thread dataProcessingThread;
        private ManualResetEvent dataAvailable = new ManualResetEvent(false);
        static public List<List<Double>> LIDARdat = new List<List<double>>();

        private string outputFilePath = null;
        static public void AllSettings(double glenth)
        {
            double resolutionY = 0.1; // 100 micromet = 1 profile -> convert to mm or 50 micromet = 1 profile and resY * len(weld) = total profiles
            double lengthWeld = glenth;
            ScanControl scancontrol = new ScanControl(resolutionY, lengthWeld);
            scancontrol.SetRoi();
            scancontrol.GetRasterResolution();
            // scancontrol.Config();
            // scancontrol.LoadProfile(3);
            scancontrol.TransferData();
            scancontrol.StopTransfer();
        }
        static void Main(string[] args)
        {

            // AllSettings();
            // IPC Setting with keyword: "LIDAR_LAEER_RESULT"
            // IPC Getting trigger "LIDAR_LASER_START"

            const string mapName = "LIDAR_LASER_START";
            const string mapRes = "LIDAR_LAEER_RESULT";
            const long mapSizeRes = 1024 * 2016 * sizeof(double)*10;
            const uint mapSize = 4;
            DateTime startTime = DateTime.Now;
            using (var mmf = MemoryMappedFile.CreateOrOpen(mapName, mapSize))
            {
                while (true)
                {
                    int getLength = CheckLidarLaserStart(mmf);
                    DateTime currentTime = DateTime.Now;
                    TimeSpan durTime = currentTime - startTime;
                    var mmres = MemoryMappedFile.CreateOrOpen(mapRes, mapSizeRes);
                    if (getLength != 0 && durTime.TotalSeconds > 2)
                    {
                        LIDARdat.Clear();
                        AllSettings(getLength);
                        startTime = DateTime.Now;
                    }
                    // check if Lidardat size = 1024*2016
                    if (LIDARdat.Count > 1024*2014)
                    {
                        Console.WriteLine("Start sending");
                        SendMatrixData(mmres);

                    }
                    Thread.Sleep(33); // nghỉ 33ms ~ 30 lần một giây
                }
            }

        }
        static int CheckLidarLaserStart(MemoryMappedFile mmf)
        {
            using (var accessor = mmf.CreateViewAccessor(0, 4))
            {
                // Read the integer value from the buffer
                byte[] buffer = new byte[4];
                accessor.ReadArray(0, buffer, 0, 4);
                
                // Convert the byte array to an integer
                int tlength = BitConverter.ToInt32(buffer, 0);
                
                return tlength;
            }
        }

        static void SendMatrixData(MemoryMappedFile mmf)
        {
            byte[] data = SerializeLIDARData(LIDARdat);
            const long mapSize = 1024 * 2016 * sizeof(double) * 10; // Kích thước bộ nhớ được tạo

            using (var accessor = mmf.CreateViewAccessor(0, mapSize, MemoryMappedFileAccess.Write))
            {
                for (int i = 0; i < 50; i++)  // Gửi dữ liệu lên IPC liên tục trong vòng 5 giây
                {
                    accessor.WriteArray(0, data, 0, data.Length);
                    System.Threading.Thread.Sleep(100); // Nghỉ 0,1 giây

                    // Kiểm tra một phần dữ liệu được gửi
                    if (i == 0)  // In ra dữ liệu chỉ một lần ở lần đầu tiên
                    {
                        string content = " ";
                        for (int j = 40; j < 50; j++)
                        {
                            content += data[j].ToString() + " ";
                        }
                        Console.WriteLine(content);
                    }
                }
            }

            // Clear LIDARdat and reset its size to 1
            LIDARdat.Clear();
            LIDARdat.Add(new List<double>());
        }

       static byte[] SerializeLIDARData(List<List<double>> data)
        {
            var options = new JsonSerializerOptions
            {
                WriteIndented = false
            };
            return JsonSerializer.SerializeToUtf8Bytes(data, options);
        }


        public ScanControl(double LengthWeld, double ResolutionY)
        {
            dLengthWeld = LengthWeld;
            dResolutionY = ResolutionY;
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

            Console.WriteLine("----- Connect to scanCONTROL -----\n");

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
                if (bOK)
                {
                    // Get the scanCONTROL type and check if it is valid
                    Console.WriteLine("Get scanCONTROL type");
                    if ((iRetValue = CLLTI.GetLLTType(hLLT, ref tscanCONTROLType)) < CLLTI.GENERAL_FUNCTION_OK)
                    {
                        OnError("Error during GetLLTType", iRetValue);
                        bOK = false;
                    }

                    if (iRetValue == CLLTI.GENERAL_FUNCTION_DEVICE_NAME_NOT_SUPPORTED)
                    {
                        Console.WriteLine(" - Can't decode scanCONTROL type. Please contact Micro-Epsilon for a newer version of the LLT.dll.");
                    }
                    else if (tscanCONTROLType >= TScannerType.scanCONTROL30xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL30xx_xxx)
                    {
                        Console.WriteLine(" - The scanCONTROL is a scanCONTROL30xx");
                    }
                    else
                    {
                        Console.WriteLine(" -The scanCONTROL is not a scanCONTROL30xx");
                        bOK = false;
                    }

                    uiResolution = 1024;
                    CLLTI.SetResolution(hLLT, uiResolution);

                    if (bOK)
                    {
                        Console.WriteLine("Set trigger to internal");
                        if ((iRetValue = CLLTI.SetFeature(hLLT, CLLTI.FEATURE_FUNCTION_TRIGGER, CLLTI.TRIG_INTERNAL)) < CLLTI.GENERAL_FUNCTION_OK)
                        {
                            OnError("Error during SetFeature(FEATURE_FUNCTION_TRIGGER)", iRetValue);
                            bOK = false;
                        }
                    }

                    if (bOK)
                    {

                        Console.WriteLine("Load Usermode 0 to reset the sensor to factory settings");
                        if ((iRetValue = CLLTI.ReadWriteUserModes(hLLT,0,0)) < CLLTI.GENERAL_FUNCTION_OK)
                        {
                            OnError("Error during Loading Usermode", iRetValue);
                            bOK = false;
                        }
                        // test: 6 - 1000mm/s - 100 us; 5-800mm/s - 500 us
                        uint uiWorkingUserMode = 3;
                        if ((iRetValue = CLLTI.ReadWriteUserModes(hLLT, 0, uiWorkingUserMode)) < CLLTI.GENERAL_FUNCTION_OK)
                        {
                            OnError("Error during loading UM 4", iRetValue);
                            bOK = false;
                        }
                    }           
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

            Console.WriteLine("Start Z (%):" + start_z);
            Console.WriteLine("End Z (%):" + end_z);
            Console.WriteLine("Start X (%):" + start_x);
            Console.WriteLine("End X (%):" + end_x + "\n");
            
            // No need review conditional because the device is using is 30xx_50
            if (tscanCONTROLType >= TScannerType.scanCONTROL30xx_25 && tscanCONTROLType <= TScannerType.scanCONTROL30xx_xxx)
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
            // Exposure Time / Idle Time in µs
            Console.WriteLine("Set idle time to " + uiIdleTime);
            if ((iRetValue = CLLTI.SetFeature(hLLT, CLLTI.FEATURE_FUNCTION_IDLE_TIME, (((uiIdleTime % 10) << 12) & 0xF000) + ((uiIdleTime / 10) & 0xFFF))) < CLLTI.GENERAL_FUNCTION_OK)
            {
                OnError("Error during SetFeature(FEATURE_FUNCTION_IDLE_TIME)", iRetValue);
            }
            Console.WriteLine("Set exposure time to " + uiExposureTime);
            if ((iRetValue = CLLTI.SetFeature(hLLT, CLLTI.FEATURE_FUNCTION_EXPOSURE_TIME, (((uiExposureTime % 10) << 12) & 0xF000) + ((uiExposureTime / 10) & 0xFFF))) < CLLTI.GENERAL_FUNCTION_OK)
            {
                OnError("Error during SetFeature(FEATURE_FUNCTION_EXPOSURE_TIME)", iRetValue);
            }

            // Set resolution to 1
            Console.WriteLine("Get all possible resolutions");
            // Get all possible resolutions for connected sensor and save them in array 
            // if ((iRetValue = CLLTI.GetResolutions(hLLT, auiResolutions, auiResolutions.GetLength(0))) < CLLTI.GENERAL_FUNCTION_OK)
            // {
            //     OnError("Error during GetResolutions", iRetValue);
            // }
            uiResolution = 1024;
            Console.WriteLine("Set resolution to " + uiResolution);
            if ((iRetValue = CLLTI.SetResolution(hLLT, uiResolution)) < CLLTI.GENERAL_FUNCTION_OK)
            {
                OnError("Error during SetResolution", iRetValue);
            }
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
            // Disconnect_Laser();
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
            long count = 0;
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

            Thread.Sleep(500);

            continueTransfer = true;

            //Declare array
            double[] adValueX = new double[uiResolution * uiProfileCount];
            double[] adValueZ = new double[uiResolution * uiProfileCount];
            ushort[] intens = new ushort[uiResolution * uiProfileCount];
            double[] DisplayX = new double[uiResolution];
            double[] DisplayZ = new double[uiResolution];

            // Start continuous profile transmission
            if ((iRetValue = CLLTI.TransferProfiles(hLLT, TTransferProfileType.NORMAL_CONTAINER_MODE, 1)) < CLLTI.GENERAL_FUNCTION_OK)
            {
                return;
            }

            // Start the data processing thread
            dataProcessingThread = new Thread(ProcessData);
            dataProcessingThread.Start();

            int counter = 0;
            Console.WriteLine("Start Transfer Data");

            while (continueTransfer && count_data <= (dResolutionY / dLengthWeld / (double)uiProfileCount))
            {
                noContainerReceived = true;
                byte[] abyContainerBuffer = new byte[uiResolution * 2 * uiFieldCount * uiProfileCount]; // 2* because 1 value has 2 bytes
                byte[] abyTimestamp = new byte[16];
                uint size_container_buffer = uiResolution * 2 * uiFieldCount * uiProfileCount;
                // Console.WriteLine("Length container Buffer {0}", abyContainerBuffer.Length);
                count_data += 1;

                while (noContainerReceived && continueTransfer)
                {
                    // //biến bắt đầu thời gian
                    // string timestamp_follow = DateTime.Now.ToString("yyyy_MM_dd_HH_mm_ss_fff");
                    // Console.WriteLine($"Start {timestamp_follow}");
                   
                    if ((iRetValue = CLLTI.GetActualProfile(hLLT, abyContainerBuffer, abyContainerBuffer.Length, TProfileConfig.CONTAINER, ref uiLostProfiles)) != abyContainerBuffer.Length)
                    {
                        if (iRetValue == CLLTI.ERROR_PROFTRANS_NO_NEW_PROFILE)
                        {
                            Thread.Sleep((int)(uiIdleTime + uiExposureTime) / 100);
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
                    // count += 1;
                    // Console.WriteLine(string.Format("Lost profile: {0}", uiLostProfiles));
                    // timestamp_follow = DateTime.Now.ToString("yyyy_MM_dd_HH_mm_ss_fff");
                    // Console.WriteLine($"Stop {timestamp_follow}");
                }

                ProfileData profile = new ProfileData
                {
                    ContainerBuffer = abyContainerBuffer,
                    Inquiry = uiInquiry
                };

                lock (dataQueue)
                {
                    dataQueue.Enqueue(profile);
                    dataAvailable.Set();
                }

                // Console.WriteLine("Enqueue");
                // Console.WriteLine("Count {0}", count);
            }

            if ((iRetValue = CLLTI.TransferProfiles(hLLT, TTransferProfileType.NORMAL_CONTAINER_MODE, 0)) < CLLTI.GENERAL_FUNCTION_OK)
            {
                return;
            }

            // Signal the data processing thread to stop
            continueTransfer = false;
            dataAvailable.Set();
            dataProcessingThread.Join();
        }
        private void ProcessData()
        {
            string timestamp = DateTime.Now.ToString("yyyy_MM_dd_HH_mm");
            outputFilePath = $"data/data_{timestamp}.txt";

            
            
            using (StreamWriter writer = new StreamWriter(outputFilePath))
            {
                while (continueTransfer || dataQueue.Count > 0)
                {
                    ProfileData profile = null;
                    lock (dataQueue)
                    {
                        if (dataQueue.Count == 0)
                        {
                            dataAvailable.Reset();
                        }
                        else
                        {
                            profile = dataQueue.Dequeue();
                        }
                    }

                    if (profile != null)
                    {
                        GCHandle pinnArray = GCHandle.Alloc(profile.ContainerBuffer, GCHandleType.Pinned);
                        double[] adValueX = new double[uiResolution * uiProfileCount];
                        double[] adValueZ = new double[uiResolution * uiProfileCount];
                        ushort[] intens = new ushort[uiResolution * uiProfileCount];
                        double[] DisplayX = new double[uiResolution];
                        double[] DisplayZ = new double[uiResolution];
                        byte[] abyTimestamp = new byte[16];

                        GCHandle pinnX = GCHandle.Alloc(adValueX, GCHandleType.Pinned);
                        GCHandle pinnZ = GCHandle.Alloc(adValueZ, GCHandleType.Pinned);
                        GCHandle pinnIntens = GCHandle.Alloc(intens, GCHandleType.Pinned);
                        IntPtr ptr_to_buf = pinnArray.AddrOfPinnedObject();
                        IntPtr ptr_to_X = pinnX.AddrOfPinnedObject();
                        IntPtr ptr_to_Z = pinnZ.AddrOfPinnedObject();
                        IntPtr ptr_to_Intens = pinnIntens.AddrOfPinnedObject();

                        convertContainerParameter.Container = ptr_to_buf;
                        convertContainerParameter.profileRearrangement = profile.Inquiry;
                        convertContainerParameter.numberOfProfilesToExtract = uiProfileCount;
                        convertContainerParameter.scanner = (uint)tscanCONTROLType;
                        convertContainerParameter.reflectionNumber = 0;
                        convertContainerParameter.ConvertToMM = 1;
                        convertContainerParameter.ReflectionWidth = IntPtr.Zero;
                        convertContainerParameter.MaxIntensity = ptr_to_Intens;
                        convertContainerParameter.Threshold = IntPtr.Zero;
                        convertContainerParameter.Moment0 = IntPtr.Zero;
                        convertContainerParameter.Moment1 = IntPtr.Zero;
                        convertContainerParameter.X = ptr_to_X;
                        convertContainerParameter.Z = ptr_to_Z;

                        if ((iRetValue = CLLTI.ConvertContainer2Values(hLLT, convertContainerParameter)) < CLLTI.GENERAL_FUNCTION_OK)
                        {
                            OnError("Error during ConvertContainer2Values", iRetValue);
                            return;
                        }
                        
                        // Console.WriteLine("No Profiles {0}", count_data);

                        // Console.WriteLine("----Extract the X/Z data and Timestamp information from container ----");
                        // Console.WriteLine("Start to write: ");

                        for (int iProfile = 0; iProfile < uiProfileCount; iProfile++)
                        {
                            Buffer.BlockCopy(profile.ContainerBuffer, (int)(2 * (iProfile + 1) * uiResolution * uiFieldCount - 16), abyTimestamp, 0, 16);
                            Buffer.BlockCopy(adValueX, (int)(uiResolution * iProfile * 8), DisplayX, 0, DisplayX.Length * 8);
                            Buffer.BlockCopy(adValueZ, (int)(uiResolution * iProfile * 8), DisplayZ, 0, DisplayZ.Length * 8);
                            CLLTI.Timestamp2TimeAndCount(abyTimestamp, ref dTimeShutterOpen, ref dTimeShutterClose, ref uiProfileCounter);
                            // DisplayProfile(DisplayX, DisplayZ, 1, dTimeShutterOpen, dTimeShutterClose, uiProfileCounter);
                            for (int i = 0; i < DisplayX.Length; i++)
                            {
                                List<double> pair = new List<double> { DisplayX[i], DisplayZ[i] };
                                LIDARdat.Add(pair);
                            }
                             // In ra số lượng phần tử để kiểm tra
                        }
                        
                        
                        pinnArray.Free();
                        pinnX.Free();
                        pinnZ.Free();
                        pinnIntens.Free();
                    }

                    dataAvailable.WaitOne();
                }
            }
        }

        private void SaveProfileData(StreamWriter writer, double[] x, double[] z, uint counter, double timeOpen, double timeClose)
        {
            
            // writer.WriteLine($"Profile Counter: {counter}, Time Open: {timeOpen}, Time Close: {timeClose}");

            for (int i = 0; i < x.Length; i++)
            {
                writer.WriteLine($"{x[i]},    {z[i]}"); //4 spacing
            }
            writer.WriteLine(); // Add a blank line between profiles
            // Console.WriteLine("\n----- SAVE PROFILE -----" + x.Length + "\n");

        }

        private void DisplayProfile(double[] x, double[] z, int resolution, double timeOpen, double timeClose, uint counter)
        {
            Console.WriteLine($"Profile Counter: {counter}, Time Open: {timeOpen}, Time Close: {timeClose}");
            for (int i = 0; i < resolution; i++)
            {
                Console.WriteLine($"X: {x[i]}, Z: {z[i]}");
            }
        }
       

        public void Disconnect_Laser()
        {
            Console.WriteLine("\n----- Disconnect from scanCONTROL -----\n");

            // Disconnect from the sensor
            Console.WriteLine("Disconnect the scanCONTROL");
            if ((iRetValue = CLLTI.Disconnect(hLLT)) < CLLTI.GENERAL_FUNCTION_OK)
            {
                OnError("Error during Disconnect", iRetValue);
            }

            // Free ressources
            Console.WriteLine("Delete the scanCONTROL instance");
            if ((iRetValue = CLLTI.DelDevice(hLLT)) < CLLTI.GENERAL_FUNCTION_OK)
            {
                OnError("Error during Delete", iRetValue);
            }
                    
        }
    }
}