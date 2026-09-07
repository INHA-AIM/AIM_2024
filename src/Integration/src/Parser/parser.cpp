#include "parser.h"

#define TAB1 "  "
#define TAB2 "    "
#define TIMEOUT 10000
#define PIXEL_FORMAT BGR8

extern SENSOR_DATA_t st_SensorData;
extern bool b_Running;
extern DEAD_RECKONING_DATA_t st_DeadReckoningData;
extern LOGIC_HZ_t st_LogicHz;
extern pthread_mutex_t pth_GpsImuThread;
extern pthread_cond_t pth_GpsImuCond;
extern int32_t b_Start;

extern bool b_RefLLA;
extern int32_t s32_MapMake;
extern float32_t f32_PathGap;
extern std::string s_OriginAdress;
extern std::string s_DirectoryName;
extern std::string s_PathName;
extern serial::Serial GPSSerial;
extern string s_LiDARHzMode;

int32_t s32_FileCopy = 0;
float32_t f32_BeforeE = 0, f32_BeforeN = 0;

extern CAN_DATA_t st_CANData;
extern int32_t s32_CanMode, s32_LabCount;
extern float32_t f32_LabCheckX1, f32_LabCheckX2, f32_LabCheckY1, f32_LabCheckY2;
bool b_TimeCheckStart = true, b_LabCheck = false;
uint64_t u64_StartLabTime = 0, u64_LabTime = 0;
uint64_t u64_BestLapTime = 18446744073709551615ULL;

float32_t roll, pitch, yaw = 0;

size_t WriteCallback(void *pv_Contents, size_t u64_Size, size_t u64_NMEMB, string *ps_Response)
{
    size_t u64_TotalSize = u64_Size * u64_NMEMB;
    ps_Response->append(static_cast<char *>(pv_Contents), u64_TotalSize);
    return u64_TotalSize;
}

void LIDARVelodyne128Parser_UDP(const char *pc_Address, int32_t s32_Port)
{
    int32_t s32_Sock = 0;
    struct sockaddr_in st_ServerAddr = {0};
    uint16_t u16_PrevAzimuth = 0;
    uint16_t u16_Azimuth = 0;
    int32_t s32_Recv = 0;
    int32_t s32_Current = 0;
    int32_t s32_PacketSize = 0;
    char arc_Buffer[c_LIDAR_BUFFER_SIZE] = {0};
    char arc_TempBuffer[1500] = {0};

    s32_Sock = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&st_ServerAddr, 0, sizeof(st_ServerAddr));

    st_ServerAddr.sin_family = AF_INET;
    st_ServerAddr.sin_addr.s_addr = inet_addr(pc_Address);
    st_ServerAddr.sin_port = htons(s32_Port);

    if (bind(s32_Sock, reinterpret_cast<struct sockaddr *>(&st_ServerAddr), sizeof(st_ServerAddr)) == -1)
    {
        // perror("LIDAR - UDP Bind failed");
        close(s32_Sock);
        return;
    }

    while (b_Running)
    {
        s32_Recv = recvfrom(s32_Sock, arc_TempBuffer, 1500, 0, nullptr, nullptr);

        if (s32_Recv != 1206)
        {
            // perror("LIDAR - UDP Error in receiving Data");
            continue;
        }

        u16_Azimuth = ((uint8_t)arc_TempBuffer[3]) << 8 | ((uint8_t)arc_TempBuffer[2]);
        memcpy(arc_Buffer + s32_PacketSize, arc_TempBuffer, 1200 * sizeof(char));
        s32_PacketSize += 1200;

        if (u16_Azimuth > 9000 && u16_PrevAzimuth <= 9000)
        {
            pthread_mutex_lock(&st_SensorData.st_MutexLIDAR);
            st_SensorData.st_RawLIDAR.u64_Timestamp = getMillisecond();
            st_SensorData.st_RawLIDAR.s32_Num = s32_PacketSize;
            memcpy(st_SensorData.st_RawLIDAR.arc_Buffer, &roll, sizeof(float32_t));
            memcpy(st_SensorData.st_RawLIDAR.arc_Buffer + 4, &pitch, sizeof(float32_t));
            memcpy(st_SensorData.st_RawLIDAR.arc_Buffer + 8, &yaw, sizeof(float32_t));
            memcpy(st_SensorData.st_RawLIDAR.arc_Buffer + 12, &st_SensorData.st_RawLIDAR.s32_Num, sizeof(int32_t));
            memcpy(st_SensorData.st_RawLIDAR.arc_Buffer + 16, arc_Buffer, st_SensorData.st_RawLIDAR.s32_Num * sizeof(char));
            pthread_mutex_unlock(&st_SensorData.st_MutexLIDAR);
            s32_PacketSize = 0;
        }

        u16_PrevAzimuth = u16_Azimuth;
    }
}

void LIDARVelodyne128Parser_ROS()
{
}

void LIDAROuster128Parser_UDP(const char *pc_Address, int32_t s32_Port)
{
    int32_t s32_Sock = 0;
    struct sockaddr_in st_ServerAddr = {0};
    uint16_t u16_MeasurementId = 0;
    uint16_t u16_Header = 32;
    int32_t s32_Recv = 0;
    uint16_t u16_Azimuth = 0;
    uint16_t u16_PrevAzimuth = 0;
    int32_t s32_PacketSize = 0;
    char arc_Buffer[c_LIDAR_BUFFER_SIZE] = {0};
    char arc_TempBuffer[24832] = {0};

    s32_Sock = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&st_ServerAddr, 0, sizeof(st_ServerAddr));

    st_ServerAddr.sin_family = AF_INET;
    st_ServerAddr.sin_addr.s_addr = inet_addr(pc_Address);
    st_ServerAddr.sin_port = htons(s32_Port);

    if (bind(s32_Sock, reinterpret_cast<struct sockaddr *>(&st_ServerAddr), sizeof(st_ServerAddr)) == -1)
    {
        // perror("LIDAR - UDP Bind failed");
        close(s32_Sock);
        return;
    }

    while (b_Running)
    {
        s32_Recv = recvfrom(s32_Sock, arc_TempBuffer, 24832, 0, nullptr, nullptr);

        if (s32_Recv != 24832)
        {
            continue;
        }

        for (int i = 0; i < 16; i++)
        {
            u16_Azimuth = (uint8_t)arc_TempBuffer[u16_Header + 1548 * i + 9] << 8 | (uint8_t)arc_TempBuffer[u16_Header + 1548 * i + 8];

            if (u16_Azimuth < u16_PrevAzimuth)
            {
                pthread_mutex_lock(&st_SensorData.st_MutexLIDAR);
                st_SensorData.st_RawLIDAR.u64_Timestamp = getMillisecond();
                st_SensorData.st_RawLIDAR.s32_Num = s32_PacketSize + 4;
                memcpy(st_SensorData.st_RawLIDAR.arc_Buffer, &roll, sizeof(float32_t));
                memcpy(st_SensorData.st_RawLIDAR.arc_Buffer + 4, &pitch, sizeof(float32_t));
                memcpy(st_SensorData.st_RawLIDAR.arc_Buffer + 8, &yaw, sizeof(float32_t));
                memcpy(st_SensorData.st_RawLIDAR.arc_Buffer + 12, &st_SensorData.st_RawLIDAR.s32_Num, sizeof(int32_t));
                memcpy(st_SensorData.st_RawLIDAR.arc_Buffer + 16, arc_Buffer, st_SensorData.st_RawLIDAR.s32_Num * sizeof(char));
                pthread_mutex_unlock(&st_SensorData.st_MutexLIDAR);
                s32_PacketSize = 0;
            }

            memcpy(arc_Buffer + s32_PacketSize, arc_TempBuffer + u16_Header + (1548 * i), 1548);
            s32_PacketSize += 1548;

            u16_PrevAzimuth = u16_Azimuth;
        }
    }
}

Arena::DeviceInfo SelectDevice(std::vector<Arena::DeviceInfo> &deviceInfos)
{

    cout << "deviceInfos.size(): " << deviceInfos.size() << endl;

    if (deviceInfos.size() == 1)
    {
        std::cout << "\n"
                  << TAB1 << "Only one device detected: " << deviceInfos[0].ModelName() << TAB1 << deviceInfos[0].SerialNumber() << TAB1 << deviceInfos[0].IpAddressStr() << ".\n";
        std::cout << TAB1 << "Automatically selecting this device.\n";
        return deviceInfos[0];
    }

    std::cout << TAB1 << "\nSelect device:\n";
    for (size_t i = 0; i < deviceInfos.size(); i++)
    {
        std::cout << TAB1 << i + 1 << ". " << deviceInfos[i].ModelName() << TAB1 << deviceInfos[i].SerialNumber() << TAB1 << deviceInfos[i].IpAddressStr() << "\n";
    }
    size_t selection = 0;

    do
    {
        std::cout << TAB1 << "Make selection (1-" << deviceInfos.size() << "): ";
        std::cin >> selection;

        if (std::cin.fail())
        {
            std::cin.clear();
            while (std::cin.get() != '\n')
                ;
            std::cout << TAB1 << "Invalid input. Please enter a number.\n";
        }
        else if (selection <= 0 || selection > deviceInfos.size())
        {
            std::cout << TAB1 << "Invalid device selected. Please select a device in the range (1-" << deviceInfos.size() << ").\n";
        }

    } while (selection <= 0 || selection > deviceInfos.size());

    return deviceInfos[selection - 1];
}

void CameraParser_UDP(const char *pc_Address, int32_t s32_Port)
{
    cout << "Camera UDP Mode - Connecting LUCID Camera" << endl;

    // prepare example
    Arena::ISystem *pSystem = Arena::OpenSystem();
    pSystem->UpdateDevices(100);
    std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
    if (deviceInfos.size() == 0)
    {
        std::cout << "\nNo camera connected\nPress enter to complete\n";
        std::getchar();
    }

    Arena::DeviceInfo selectedDeviceInfo = SelectDevice(deviceInfos);
    Arena::IDevice *pDevice = pSystem->CreateDevice(selectedDeviceInfo);
    printf("Camera Debuging: 0\n");
    GenApi::INodeMap *pNodeMap = pDevice->GetNodeMap();
    printf("Camera Debuging: 1\n");

    GenApi::CFloatPtr pExposureTimeNode = pNodeMap->GetNode("ExposureTime");
    printf("Camera Debuging: 2\n");
    const int64_t getImageTimeout_ms = static_cast<int64_t>(pExposureTimeNode->GetMax() / 1000 * 2);

    int32_t s32_Height, s32_Width;

    pDevice->StartStream();
    cv::Mat img;
    Arena::IImage *pImage;

    printf("Camera Debuging: 3\n");
    while (b_Running)
    {
        pImage = pDevice->GetImage(TIMEOUT);
        if (pImage)
        {
            int32_t s32_Height = static_cast<int32_t>(pImage->GetHeight());
            int32_t s32_Width = static_cast<int32_t>(pImage->GetWidth());

            cv::Mat img(s32_Height, s32_Width, CV_8UC3, (void *)pImage->GetData());

            cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

            std_msgs::Header header;         // empty header
            header.stamp = ros::Time::now(); // time

            sensor_msgs::CompressedImage compressed_image;
            compressed_image.header = header;
            compressed_image.format = "jpeg";

            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
            cv::imencode(".jpg", img, compressed_image.data, params);

            std::vector<char> st_Buffer(compressed_image.data.begin(), compressed_image.data.end());

            pthread_mutex_lock(&st_SensorData.st_MutexCamera);
            st_SensorData.st_RawCamera.u64_Timestamp = getMillisecond();
            st_SensorData.st_RawCamera.s32_Num = st_Buffer.size();
            memcpy(st_SensorData.st_RawCamera.arc_Buffer, st_Buffer.data(), st_Buffer.size() * sizeof(char));
            pthread_mutex_unlock(&st_SensorData.st_MutexCamera);

            st_Buffer.clear();
            pDevice->RequeueBuffer(pImage);
        }
        else
        {
            printf("Camera Debuging: 4\n");
        }
    }

    pDevice->StopStream();
    pSystem->DestroyDevice(pDevice);
    Arena::CloseSystem(pSystem);
}

void CameraParser_Callback(const sensor_msgs::CompressedImageConstPtr &st_Msg)
{
    std::vector<char> st_Buffer(st_Msg->data.begin(), st_Msg->data.end());

    pthread_mutex_lock(&st_SensorData.st_MutexCamera);
    st_SensorData.st_RawCamera.u64_Timestamp = getMillisecond();
    st_SensorData.st_RawCamera.s32_Num = st_Buffer.size();
    memcpy(st_SensorData.st_RawCamera.arc_Buffer, st_Buffer.data(), st_Buffer.size() * sizeof(char));
    pthread_mutex_unlock(&st_SensorData.st_MutexCamera);

    st_Buffer.clear();
}

{
    std::string lidar_init = lidar_Mode;
    cout << lidar_init << endl;
    std::string lidar_mode = "{\"lidar_mode\": \"" + lidar_init + "\"}";
    string status;
    string mode;
    int64_t s64_temp;
    uint64_t prev_time = 0;

    while (b_Running)
    {
        CURL *curl_status = curl_easy_init();
        CURL *curl_mode = curl_easy_init();
        CURL *curl_temp = curl_easy_init();
        struct curl_slist *list = NULL;
        CURLcode res_status;
        CURLcode res_mode;
        CURLcode res_temp;

        const char *url_post = "http://os-992310000063.local/api/v1/sensor/config";
        const char *url_status = "http://os-992310000063.local/api/v1/sensor/metadata/sensor_info";
        const char *url_mode = "http://os-992310000063.local/api/v1/sensor/config";
        const char *url_temp = "http://os-992310000063.local/api/v1/sensor/telemetry";

        if (curl_status)
        {
            std::string response_status;
            std::string response_mode;
            std::string response_temp;

            curl_easy_setopt(curl_status, CURLOPT_URL, url_status);
            curl_easy_setopt(curl_status, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl_status, CURLOPT_WRITEDATA, &response_status);

            curl_easy_setopt(curl_mode, CURLOPT_URL, url_mode);
            curl_easy_setopt(curl_mode, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl_mode, CURLOPT_WRITEDATA, &response_mode);

            curl_easy_setopt(curl_temp, CURLOPT_URL, url_temp);
            curl_easy_setopt(curl_temp, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl_temp, CURLOPT_WRITEDATA, &response_temp);

            res_status = curl_easy_perform(curl_status);
            res_mode = curl_easy_perform(curl_mode);
            res_temp = curl_easy_perform(curl_temp);

            if (res_status == CURLE_OK && res_mode == CURLE_OK && res_temp == CURLE_OK)
            {
                rapidjson::Document document_status;
                document_status.Parse(response_status.c_str());
                if (document_status.HasParseError())
                {
                    std::cerr << "JSON Parse Error in response_status: " << rapidjson::GetParseError_En(document_status.GetParseError()) << std::endl;
                }
                else if (!document_status.IsObject())
                {
                    std::cerr << "JSON is not an object as expected in response_status." << std::endl;
                }

                rapidjson::Document document_mode;
                document_mode.Parse(response_mode.c_str());
                if (document_mode.HasParseError())
                {
                    std::cerr << "JSON Parse Error in response_mode: " << rapidjson::GetParseError_En(document_mode.GetParseError()) << std::endl;
                }
                else if (!document_mode.IsObject())
                {
                    std::cerr << "JSON is not an object as expected in response_mode." << std::endl;
                }

                rapidjson::Document document_temp;
                document_temp.Parse(response_temp.c_str());
                if (document_temp.HasParseError())
                {
                    std::cerr << "JSON Parse Error in response_temp: " << rapidjson::GetParseError_En(document_temp.GetParseError()) << std::endl;
                }
                else if (!document_temp.IsObject())
                {
                    std::cerr << "JSON is not an object as expected in response_temp." << std::endl;
                }

                if (document_status.HasMember("status"))
                {
                    std::cout << "Status exists and is a " << (document_status["status"].IsString() ? "string" : "not string") << std::endl;
                }
                else
                {
                    std::cout << "Status key does not exist" << std::endl;
                }

                if (document_mode.HasMember("lidar_mode"))
                {
                    std::cout << "LiDAR_HzMode exists and is a " << (document_mode["lidar_mode"].IsString() ? "string" : "not string") << std::endl;
                }
                else
                {
                    std::cout << "LiDAR_HzMode key does not exist" << std::endl;
                }

                // Temp에 대한 로깅
                if (document_temp.HasMember("internal_temperature_deg_c"))
                {
                    std::cout << "internal_temperature_deg_c exists and is a " << (document_temp["internal_temperature_deg_c"].IsInt64() ? "integer" : "not integer") << std::endl;
                }
                else
                {
                    std::cout << "internal_temperature_deg_c key does not exist" << std::endl;
                }

                if (!document_status.HasParseError() && document_status.IsObject() && !document_mode.HasParseError() && document_mode.IsObject() && !document_temp.HasParseError() && document_temp.IsObject())
                {
                    // if (document_status.HasMember("status") && document_status["status"].IsString())
                    if (document_status.HasMember("status") && document_status["status"].IsString() && document_mode.HasMember("lidar_mode") && document_mode["lidar_mode"].IsString() && document_temp.HasMember("internal_temperature_deg_c") && document_temp["internal_temperature_deg_c"].IsInt64())
                    {

                        status = document_status["status"].GetString();
                        mode = document_mode["lidar_mode"].GetString();
                        s64_temp = document_temp["internal_temperature_deg_c"].GetInt64();

                        pthread_mutex_lock(&st_SensorData.st_MutexLIDAR);
                        st_SensorData.st_RawLIDAR.s_LIDAR_Status = status;
                        st_SensorData.st_RawLIDAR.s_LIDAR_Mode = mode;
                        st_SensorData.st_RawLIDAR.s64_LIDAR_Temp = s64_temp;
                        pthread_mutex_unlock(&st_SensorData.st_MutexLIDAR);

                        // INITIALIZING, UPDATING, RUNNING, ERROR, UNCONFIGURED
                        if (status == "ERROR" || status == "UNCONFIGURED")
                        {
                            if (curl_status)
                            {
                                curl_status = curl_easy_init();

                                list = NULL;

                                curl_easy_setopt(curl_status, CURLOPT_URL, url_post);
                                curl_easy_setopt(curl_status, CURLOPT_POST, 1L);

                                list = curl_slist_append(list, "Content-Type: application/json");
                                curl_easy_setopt(curl_status, CURLOPT_HTTPHEADER, list);
                                curl_easy_setopt(curl_status, CURLOPT_SSL_VERIFYPEER, 1L);
                                curl_easy_setopt(curl_status, CURLOPT_POSTFIELDS, lidar_mode.c_str());

                                res_status = curl_easy_perform(curl_status);
                                curl_easy_cleanup(curl_status);

                                printf("=============  Restarting LiDAR because of Error Status  =============\n");
                            }
                        }
                        usleep(500000000);
                    }
                    else
                    {
                        std::cerr << "Invalid JSON format.. \n"
                                  << std::endl;
                    }
                }
                else
                {
                    std::cerr << "Failed to parse JSON.. \n"
                              << std::endl;
                }
            }
            else
            {
                // std::cerr << "Failed to make a request.. \n" << curl_easy_strerror(res_status) << std::endl;
            }

            curl_easy_cleanup(curl_status);
            curl_easy_cleanup(curl_mode);
            curl_easy_cleanup(curl_temp);
        }
        else
        {
            std::cerr << "Failed to initialize curl.. \n"
                      << std::endl;
        }
    }
}
void CameraParser_ROS(ros::NodeHandle *pst_NodeHandle)
{
    ros::Subscriber st_SubscribeImage = pst_NodeHandle->subscribe<sensor_msgs::CompressedImage>("/image_jpeg/compressed", 1, CameraParser_Callback);

    while (b_Running)
    {
        ros::spinOnce();
        usleep(10);
    }
}

void CreateDirectory(std::string &s_DirectoryName)
{
    std::string s_MakeDirectory;
    std::string s_MakeDirectoryAdress = s_OriginAdress;
    std::istringstream StreamDirectory(s_DirectoryName);
    while (std::getline(StreamDirectory, s_MakeDirectory, '/'))
    {
        if (!s_MakeDirectory.empty())
        {
            s_MakeDirectoryAdress += "/" + s_MakeDirectory;
            struct stat st = {0};
            if (stat(s_MakeDirectoryAdress.c_str(), &st) == -1)
            {
                mkdir(s_MakeDirectoryAdress.c_str(), 0700);
            }
            else
            {
                std::cout << "Aready have Directory" << std::endl;
            }
        }
    }
}

void MakeMap(GPS_DATA_t &st_GPSData, std::string &s_DirectoryName, std::string &s_PathName)
{
    float32_t f32_Dist = 0;
    std::string s_MakeDirectoryAdress = "";
    std::string s_MakeFileAdress = "";

    if (b_RefLLA)
    {
        s_MakeDirectoryAdress = s_DirectoryName + "/Path";
        s_MakeFileAdress = s_OriginAdress + "/" + s_DirectoryName;

        CreateDirectory(s_MakeDirectoryAdress);
        std::ofstream RefLLAFile(s_MakeFileAdress + "/ref.txt");

        RefLLAFile << fixed;
        RefLLAFile.precision(10);
        c_ORIGIN_LATITUDE_DEG = st_GPSData.f64_Latitude;
        c_ORIGIN_LONGITUDE_DEG = st_GPSData.f64_Longitude;
        c_ORIGIN_ALTITUDE = 0.0;

        c_ORIGIN_LATITUDE_RAD = deg2rad(c_ORIGIN_LATITUDE_DEG);
        c_ORIGIN_LONGITUDE_RAD = deg2rad(c_ORIGIN_LONGITUDE_DEG);

        float64_t f64_Lat_rad = c_ORIGIN_LATITUDE_RAD;
        float64_t f64_Lon_rad = c_ORIGIN_LONGITUDE_RAD;
        float64_t f64_Alt = c_ORIGIN_ALTITUDE;

        float64_t f64_Chi = sqrt(1 - c_LLA2ENU_N_2 * pow(sin(f64_Lat_rad), 2));
        float64_t f64_Q = (c_LLA2ENU_A / f64_Chi + f64_Alt) * cos(f64_Lat_rad);

        c_ORIGIN_REFERENCE_X = f64_Q * cos(f64_Lon_rad);
        c_ORIGIN_REFERENCE_Y = f64_Q * sin(f64_Lon_rad);
        c_ORIGIN_REFERENCE_Z = ((c_LLA2ENU_A * (1 - c_LLA2ENU_N_2) / f64_Chi) + f64_Alt) * sin(f64_Lat_rad);
        RefLLAFile << c_ORIGIN_LATITUDE_DEG << "   " << c_ORIGIN_LONGITUDE_DEG << "    " << c_ORIGIN_ALTITUDE;
        RefLLAFile.close();
        b_RefLLA = false;
        s32_FileCopy = 1;
    }

    float32_t f32_E, f32_N, f32_U;
    lla2enu(st_GPSData.f64_Latitude, st_GPSData.f64_Longitude, 0, f32_E, f32_N, f32_U);

    f32_Dist = getDistance2d(f32_BeforeE, f32_BeforeN, f32_E, f32_N);
    if (f32_Dist > f32_PathGap)
    {
        f32_BeforeE = f32_E;
        f32_BeforeN = f32_N;
        // ENUData.precision(4);
        // ENUData << f32_BeforeE << "   " << f32_BeforeN << endl;
    }
}

void MakeMapMove(std::string &s_DirectoryName, std::string &s_PathName)
{
    try
    {
        std::string s_ENUFileAdress = s_OriginAdress + "/" + s_DirectoryName + "/Path/" + s_PathName + ".txt";
        std::string s_ENUDataAdress = "../AIM_2024/enu.txt";
        std::filesystem::rename(s_ENUDataAdress.c_str(), s_ENUFileAdress.c_str());
    }
    catch (const std::filesystem::filesystem_error &e)
    {
        return;
    }
}

void LabCheck(GPS_DATA_t &st_GPS)
{
    if (s32_CanMode != 0)
    {
        uint64_t u64_CurrentTime = getMillisecond();
        uint64_t u64_TotalTime = 0, u64_CurrentLapTime = 0;
        float32_t f32_LabCheckDist1 = getDistance2d(f32_LabCheckX1, f32_LabCheckY1, st_GPS.f32_E, st_GPS.f32_N);
        float32_t f32_LabCheckDist2 = getDistance2d(f32_LabCheckX2, f32_LabCheckY2, st_GPS.f32_E, st_GPS.f32_N);

        st_GPS.s32_LabCount = s32_LabCount;

        if (b_TimeCheckStart)
        {
            if (s32_CanMode == 1)
            {
                u64_StartLabTime = u64_CurrentTime;
                b_TimeCheckStart = false;
            }
            else if ((s32_CanMode == 3 || s32_CanMode == 4) && st_CANData.u8_EPS_Control_Status == 2)
            {
                u64_StartLabTime = u64_CurrentTime;
                b_TimeCheckStart = false;
            }
        }

        if (s32_LabCount == 0)
        {
            u64_LabTime = u64_StartLabTime;
        }
        u64_TotalTime = u64_CurrentTime - u64_StartLabTime; // total lab time
        u64_CurrentLapTime = u64_CurrentTime - u64_LabTime; // current lab time

        if (f32_LabCheckDist1 < 5)
        {
            b_LabCheck = true;
        }
        if (b_LabCheck && f32_LabCheckDist2 < 5)
        {
            s32_LabCount++;
            b_LabCheck = false;
            if (u64_BestLapTime > u64_CurrentLapTime && s32_LabCount > 1)
            {
                u64_BestLapTime = u64_CurrentLapTime;
            }
            u64_LabTime = getMillisecond();
        }
        if (st_CANData.u8_EPS_Control_Status == 2 || st_CANData.u8_EPS_Control_Status == 3)
        {
            ms2hms(u64_TotalTime, st_GPS.ars32_TotalTime);
            ms2hms(u64_CurrentLapTime, st_GPS.ars32_CurrentLapTime);
            if (s32_LabCount > 1)
            {
                ms2hms(u64_BestLapTime, st_GPS.ars32_BestLapTime);
            }
        }
    }
}

void GPS_IMU_Parser_Serial_AVANTE()
{
    GPS_DATA_t st_GPS;
    IMU_DATA_t st_IMU;

    float32_t f32_U;
    uint64_t u64_StartTime_ms = 0;
    int32_t StartIdx = 0;
    int32_t s32_BufferSize = 0;

    float32_t arf32_DCM[3][3] = {0};

    uint8_t aru8_Buffer[5000] = {0};
    while (b_Running)
    {
        if (10 < GPSSerial.available())
        {
            u64_StartTime_ms = getMillisecond();
            s32_BufferSize = GPSSerial.available();
            GPSSerial.read(aru8_Buffer, s32_BufferSize);
            for (int i = 0; i < s32_BufferSize - 6; ++i)
            {
                if (aru8_Buffer[i] == 0xB5 && aru8_Buffer[i + 1] == 0x62)
                {
                    uint16_t u16_Length = *(uint16_t *)&aru8_Buffer[i + 4];
                    if (aru8_Buffer[i + 2] == 0x01 && aru8_Buffer[i + 3] == 0x07 && u16_Length == 92 && (i + 6 + u16_Length) <= s32_BufferSize)
                    {
                        StartIdx = i + 6;
                        st_GPS.s32_mode = aru8_Buffer[StartIdx + 20];
                        for (int32_t s32_j = 0; s32_j < 3; s32_j++)
                        {
                            st_IMU.arf32_VelocityNED_m_s[s32_j] = *(int32_t *)&aru8_Buffer[StartIdx + 48 + 4 * (s32_j)] / 1000.f;
                        }
                        for (int32_t s32_j = 0; s32_j < 3; s32_j++)
                        {
                            st_IMU.arf32_VelocityXYZ_m_s[s32_j] = arf32_DCM[s32_j][0] * st_IMU.arf32_VelocityNED_m_s[0] + arf32_DCM[s32_j][1] * st_IMU.arf32_VelocityNED_m_s[1] + arf32_DCM[s32_j][2] * st_IMU.arf32_VelocityNED_m_s[2];
                        }
                        // f32_GroundVelocity_m_s = *(int32_t*)&aru8_Buffer[StartIdx + 60]/1000.f;

                        st_GPS.f64_Latitude = *(int32_t *)&aru8_Buffer[StartIdx + 28] / 1e7;
                        st_GPS.f64_Longitude = *(int32_t *)&aru8_Buffer[StartIdx + 24] / 1e7;
                        st_GPS.f64_Altitude = 0.0;
                        // st_GPS.f64_Altitude = *(int32_t*)&aru8_Buffer[StartIdx + 32] / 1e3;
                        st_IMU.f32_Yaw_deg = *(int32_t *)&aru8_Buffer[StartIdx + 64] / 1e5;
                        if (st_IMU.f32_Yaw_deg > 180)
                        {
                            st_IMU.f32_Yaw_deg -= 360;
                        }
                        st_IMU.f32_Yaw_rad = deg2rad(st_IMU.f32_Yaw_deg);
                        lla2enu(st_GPS.f64_Latitude, st_GPS.f64_Longitude, 0, st_GPS.f32_E, st_GPS.f32_N, f32_U);
                    }
                    else if (aru8_Buffer[i + 2] == 0x01 && aru8_Buffer[i + 3] == 0x05 && u16_Length == 32 && (i + 6 + u16_Length) <= s32_BufferSize)
                    {
                        StartIdx = i + 6;
                        st_IMU.f32_Roll_deg = *(int32_t *)&aru8_Buffer[StartIdx + 8] / 1e5;
                        st_IMU.f32_Pitch_deg = *(int32_t *)&aru8_Buffer[StartIdx + 12] / 1e5;

                        st_IMU.f32_Roll_rad = deg2rad(st_IMU.f32_Roll_deg);
                        st_IMU.f32_Pitch_rad = deg2rad(st_IMU.f32_Pitch_deg);

                        arf32_DCM[0][0] = cos(st_IMU.f32_Pitch_rad) * cos(st_IMU.f32_Yaw_rad);
                        arf32_DCM[0][1] = cos(st_IMU.f32_Pitch_rad) * sin(st_IMU.f32_Yaw_rad);
                        arf32_DCM[0][2] = -sin(st_IMU.f32_Pitch_rad);

                        arf32_DCM[1][0] = sin(st_IMU.f32_Roll_rad) * sin(st_IMU.f32_Pitch_rad) * cos(st_IMU.f32_Yaw_rad) - cos(st_IMU.f32_Roll_rad) * sin(st_IMU.f32_Yaw_rad);
                        arf32_DCM[1][1] = sin(st_IMU.f32_Roll_rad) * sin(st_IMU.f32_Pitch_rad) * sin(st_IMU.f32_Yaw_rad) + cos(st_IMU.f32_Roll_rad) * cos(st_IMU.f32_Yaw_rad);
                        arf32_DCM[1][2] = sin(st_IMU.f32_Roll_rad) * cos(st_IMU.f32_Pitch_rad);

                        arf32_DCM[2][0] = cos(st_IMU.f32_Roll_rad) * sin(st_IMU.f32_Pitch_rad) * cos(st_IMU.f32_Yaw_rad) + sin(st_IMU.f32_Roll_rad) * sin(st_IMU.f32_Yaw_rad);
                        arf32_DCM[2][1] = cos(st_IMU.f32_Roll_rad) * sin(st_IMU.f32_Pitch_rad) * sin(st_IMU.f32_Yaw_rad) - sin(st_IMU.f32_Roll_rad) * cos(st_IMU.f32_Yaw_rad);
                        arf32_DCM[2][2] = cos(st_IMU.f32_Roll_rad) * cos(st_IMU.f32_Pitch_rad);
                    }
                    else if (aru8_Buffer[i + 2] == 0x10 && aru8_Buffer[i + 3] == 0x15 && u16_Length == 36 && (i + 6 + u16_Length) <= s32_BufferSize)
                    {
                        StartIdx = i + 6;
                        st_IMU.f32_GyroX = *(int32_t *)&aru8_Buffer[StartIdx + 4] / 1e3;
                        st_IMU.f32_GyroY = *(int32_t *)&aru8_Buffer[StartIdx + 8] / 1e3;
                        st_IMU.f32_GyroZ = *(int32_t *)&aru8_Buffer[StartIdx + 12] / 1e3;
                        st_IMU.f32_AccelX = *(int32_t *)&aru8_Buffer[StartIdx + 16] / 1e2;
                        st_IMU.f32_AccelY = *(int32_t *)&aru8_Buffer[StartIdx + 20] / 1e2;
                        st_IMU.f32_AccelZ = *(int32_t *)&aru8_Buffer[StartIdx + 24] / 1e2;
                    }
                }
            }

            LabCheck(st_GPS);

            pthread_mutex_lock(&st_SensorData.st_MutexGPSINS);
            st_SensorData.st_RawGPSINS.u64_Timestamp = getMillisecond();
            st_SensorData.st_RawGPSINS.s32_Num = sizeof(GPS_DATA_t);
            memcpy(st_SensorData.st_RawGPSINS.arc_Buffer, &st_GPS, sizeof(GPS_DATA_t));
            pthread_mutex_unlock(&st_SensorData.st_MutexGPSINS);

            pthread_mutex_lock(&st_SensorData.st_MutexIMU);
            st_SensorData.st_RawIMU.u64_Timestamp = getMillisecond();
            st_SensorData.st_RawIMU.s32_Num = sizeof(IMU_DATA_t);
            memcpy(st_SensorData.st_RawIMU.arc_Buffer, &st_IMU, sizeof(IMU_DATA_t));
            pthread_mutex_unlock(&st_SensorData.st_MutexIMU);

            if (b_Start)
            {
                MakeMap(st_GPS, s_DirectoryName, s_PathName);
            }
            else if (s32_FileCopy == 1)
            {
                MakeMapMove(s_DirectoryName, s_PathName);
                s32_FileCopy = 2;
            }
            st_LogicHz.u64_DeadReckoningHz = getMillisecond() - u64_StartTime_ms;
        }
    }
}

void GPSParser_MORAI_Callback(const morai_msgs::GPSMessage::ConstPtr &st_Msg)
{
    GPS_DATA_t st_GPS = {st_Msg->latitude, st_Msg->longitude, st_Msg->altitude};
    float32_t f32_U = 0;
    lla2enu(st_GPS.f64_Latitude, st_GPS.f64_Longitude, 0, st_GPS.f32_E, st_GPS.f32_N, f32_U);
    LabCheck(st_GPS);

    pthread_mutex_lock(&st_SensorData.st_MutexGPSINS);
    st_SensorData.st_RawGPSINS.u64_Timestamp = getMillisecond();
    st_SensorData.st_RawGPSINS.s32_Num = sizeof(GPS_DATA_t);
    memcpy(st_SensorData.st_RawGPSINS.arc_Buffer, &st_GPS, sizeof(GPS_DATA_t));

    pthread_mutex_unlock(&st_SensorData.st_MutexGPSINS);

    if (b_Start)
    {
        MakeMap(st_GPS, s_DirectoryName, s_PathName);
    }
    else if (s32_FileCopy == 1)
    {
        MakeMapMove(s_DirectoryName, s_PathName);
        s32_FileCopy = 2;
    }
}

void GPSParser_MORAI(ros::NodeHandle *pst_NodeHandle)
{
    ros::Subscriber st_SubscribeGPS = pst_NodeHandle->subscribe<morai_msgs::GPSMessage>("/gps", 1, GPSParser_MORAI_Callback);
    while (b_Running)
    {
        ros::spinOnce();
        usleep(10);
    }
}

void IMUParser_MORAI_Callback(const sensor_msgs::Imu::ConstPtr &st_Msg)
{
    IMU_DATA_t st_IMU;

    float32_t f32_SquaredQW;
    float32_t f32_SquaredQX;
    float32_t f32_SquaredQY;
    float32_t f32_SquaredQZ;

    st_IMU.f32_AccelX = st_Msg->linear_acceleration.x;
    st_IMU.f32_AccelY = st_Msg->linear_acceleration.y;
    st_IMU.f32_AccelZ = st_Msg->linear_acceleration.z;

    st_IMU.f32_GyroX = st_Msg->angular_velocity.x;
    st_IMU.f32_GyroY = st_Msg->angular_velocity.y;
    st_IMU.f32_GyroZ = st_Msg->angular_velocity.z;

    st_IMU.f32_QuaternionX = st_Msg->orientation.x;
    st_IMU.f32_QuaternionY = st_Msg->orientation.y;
    st_IMU.f32_QuaternionZ = st_Msg->orientation.z;
    st_IMU.f32_QuaternionW = st_Msg->orientation.w;

    f32_SquaredQX = pow(st_IMU.f32_QuaternionX, 2);
    f32_SquaredQY = pow(st_IMU.f32_QuaternionY, 2);
    f32_SquaredQZ = pow(st_IMU.f32_QuaternionZ, 2);
    f32_SquaredQW = pow(st_IMU.f32_QuaternionW, 2);

    st_IMU.f32_Roll_rad = atan2f(2.0f * (st_IMU.f32_QuaternionW * st_IMU.f32_QuaternionX + st_IMU.f32_QuaternionY * st_IMU.f32_QuaternionZ), 1.0f - 2.0f * (f32_SquaredQX + f32_SquaredQY));
    st_IMU.f32_Pitch_rad = asinf(2.0f * (st_IMU.f32_QuaternionW * st_IMU.f32_QuaternionY - st_IMU.f32_QuaternionZ * st_IMU.f32_QuaternionX));
    st_IMU.f32_Yaw_rad = atan2f(2.0f * (st_IMU.f32_QuaternionW * st_IMU.f32_QuaternionZ + st_IMU.f32_QuaternionX * st_IMU.f32_QuaternionY), 1.0f - 2.0f * (f32_SquaredQY + f32_SquaredQZ));

    st_IMU.f32_Roll_deg = rad2deg(st_IMU.f32_Roll_rad);
    st_IMU.f32_Pitch_deg = rad2deg(st_IMU.f32_Pitch_rad);
    st_IMU.f32_Yaw_deg = rad2deg(st_IMU.f32_Yaw_rad);

    pthread_mutex_lock(&st_SensorData.st_MutexIMU);
    st_SensorData.st_RawIMU.u64_Timestamp = getMillisecond();
    st_SensorData.st_RawIMU.s32_Num = sizeof(IMU_DATA_t);
    memcpy(st_SensorData.st_RawIMU.arc_Buffer, &st_IMU, sizeof(IMU_DATA_t));
    pthread_mutex_unlock(&st_SensorData.st_MutexIMU);
}

void IMUParser_MORAI(ros::NodeHandle *pst_NodeHandle)
{
    ros::Subscriber st_SubscribeIMU = pst_NodeHandle->subscribe<sensor_msgs::Imu>("/imu", 1, IMUParser_MORAI_Callback);
    while (b_Running)
    {
        ros::spinOnce();
        usleep(10);
    }
}

void *LIDARParserWrapper(void *p_Arg)
{
    ros::NodeHandle *pst_NodeHandle = (ros::NodeHandle *)p_Arg;
    std::string s_LIDARName = "";
    int32_t s32_LIDARMode = 0;
    int32_t s32_LIDARPort = 0;
    std::string s_LIDARAddress = "";

    YAML::Node st_Config = YAML::LoadFile("configuration.yaml");

    s_LIDARName = st_Config["LIDAR_Name"].as<std::string>();
    s32_LIDARMode = st_Config["LIDAR_Mode"].as<int32_t>();
    s_LIDARAddress = st_Config["LIDAR_Address"].as<std::string>();
    s32_LIDARPort = st_Config["LIDAR_Port"].as<int32_t>();

    if (s32_LIDARMode == c_PARSING_UDP)
    {
        if (s_LIDARName == "VLS")
        {
            LIDARVelodyne128Parser_UDP(s_LIDARAddress.c_str(), s32_LIDARPort);
        }
        else if (s_LIDARName == "OS2")
        {
            LIDAROuster128Parser_UDP(s_LIDARAddress.c_str(), s32_LIDARPort);
        }
    }
    else if (s32_LIDARMode == c_PARSING_ROS)
    {
        ;
    }
    else if (s32_LIDARMode == c_PARSING_SERIAL)
    {
        ;
    }
}

void *CameraParserWrapper(void *p_Arg)
{
    ros::NodeHandle *pst_NodeHandle = (ros::NodeHandle *)p_Arg;
    int32_t s32_CameraPort = 0;
    int32_t s32_CameraMode = 0;
    std::string s_CameraAddress = "";

    YAML::Node st_Config = YAML::LoadFile("configuration.yaml");

    s32_CameraMode = st_Config["Camera_Mode"].as<int32_t>();
    s_CameraAddress = st_Config["Camera_Address"].as<std::string>();
    s32_CameraPort = st_Config["Camera_Port"].as<int32_t>();

    if (s32_CameraMode == c_PARSING_UDP)
    {
        // cout<<"here"<<endl;
        CameraParser_UDP(s_CameraAddress.c_str(), s32_CameraPort);
    }
    else if (s32_CameraMode == c_PARSING_ROS)
    {
        CameraParser_ROS(pst_NodeHandle);
    }
    else if (s32_CameraMode == c_PARSING_SERIAL)
    {
        ;
    }
}

void *GPSParserWrapper(void *p_Arg)
{
    ros::NodeHandle *pst_NodeHandle = (ros::NodeHandle *)p_Arg;
    int32_t s32_GPSMode = 0;
    int32_t s32_Boudrate = 0;
    uint64_t u64_StartTime_ms = 0;
    std::string s_GPSPort = "";

    YAML::Node st_Config = YAML::LoadFile("configuration.yaml");

    s32_GPSMode = st_Config["GPS_Mode"].as<int32_t>();

    if (s32_GPSMode == c_PARSING_MORAI)
    {
        GPSParser_MORAI(pst_NodeHandle);
    }
    else if (s32_GPSMode == c_PARSING_AVANTE)
    {
        GPS_IMU_Parser_Serial_AVANTE();
    }
}

void *IMUParserWrapper(void *p_Arg)
{
    ros::NodeHandle *pst_NodeHandle = (ros::NodeHandle *)p_Arg;
    int32_t s32_IMUMode = 0;
    uint64_t u64_StartTime_ms = 0;

    YAML::Node st_Config = YAML::LoadFile("configuration.yaml");

    s32_IMUMode = st_Config["IMU_Mode"].as<int32_t>();

    if (s32_IMUMode == c_PARSING_MORAI)
    {
        IMUParser_MORAI(pst_NodeHandle);
    }
    else if (s32_IMUMode == c_PARSING_AVANTE)
    {
        ;
    }
}

double getAngle(st_KF &rst_KF, float64_t newAngle, float64_t newRate, float64_t f64_dT)
{
    rst_KF.rate = newRate - rst_KF.bias;
    rst_KF.angle += f64_dT * rst_KF.rate;

    rst_KF.P[0][0] += f64_dT * (f64_dT * rst_KF.P[1][1] - rst_KF.P[0][1] - rst_KF.P[1][0] + rst_KF.Q_angle);
    rst_KF.P[0][1] -= f64_dT * rst_KF.P[1][1];
    rst_KF.P[1][0] -= f64_dT * rst_KF.P[1][1];
    rst_KF.P[1][1] += rst_KF.Q_bias * f64_dT;

    float64_t S = rst_KF.P[0][0] + rst_KF.R_measure;
    float64_t K[2];
    K[0] = rst_KF.P[0][0] / S;
    K[1] = rst_KF.P[1][0] / S;

    float64_t y = newAngle - rst_KF.angle;
    rst_KF.angle += K[0] * y;
    rst_KF.bias += K[1] * y;

    float64_t P00_temp = rst_KF.P[0][0];
    float64_t P01_temp = rst_KF.P[0][1];

    rst_KF.P[0][0] -= K[0] * P00_temp;
    rst_KF.P[0][1] -= K[0] * P01_temp;
    rst_KF.P[1][0] -= K[1] * P00_temp;
    rst_KF.P[1][1] -= K[1] * P01_temp;

    return rst_KF.angle;
}

void initKalmanFilter(st_KF &kf)
{
    kf.angle = 0;
    kf.bias = 0;
    kf.rate = 0;
    kf.P[0][0] = 1;
    kf.P[0][1] = 0;
    kf.P[1][0] = 0;
    kf.P[1][1] = 1;
    kf.Q_angle = 0.001;
    kf.Q_bias = 0.003;
    kf.R_measure = 0.03;
}

void *OusterIMUParserWrapper(void *p_Arg)
{

    st_KF rst_KFRoll;
    st_KF rst_KFPitch;

    initKalmanFilter(rst_KFRoll);
    initKalmanFilter(rst_KFPitch);

    int32_t s32_Sock = 0;
    struct sockaddr_in st_ServerAddr = {0};
    uint16_t u16_MeasurementId = 0;
    int32_t s32_Recv = 0;
    int32_t s32_Current = 0;
    char arc_Buffer[48] = {0};
    char arc_TempBuffer[48] = {0};
    int32_t s32_Port = 2369;

    float32_t f32_AccelX = 0;
    float32_t f32_AccelY = 0;
    float32_t f32_AccelZ = 0;

    float32_t f32_GyroX = 0;
    float32_t f32_GyroY = 0;
    float32_t f32_GyroZ = 0;
    float32_t f32_TempRoll = 0;

    s32_Sock = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&st_ServerAddr, 0, sizeof(st_ServerAddr));

    st_ServerAddr.sin_family = AF_INET;
    st_ServerAddr.sin_addr.s_addr = inet_addr("192.168.1.101");
    st_ServerAddr.sin_port = htons(s32_Port);

    if (bind(s32_Sock, reinterpret_cast<struct sockaddr *>(&st_ServerAddr), sizeof(st_ServerAddr)) == -1)
    {
        perror("LIDAR - UDP Bind failed");
        close(s32_Sock);
    }

    while (b_Running)
    {
        s32_Recv = recvfrom(s32_Sock, arc_TempBuffer, 48, 0, nullptr, nullptr);
        memcpy(&f32_AccelX, arc_TempBuffer + 24, sizeof(float32_t));
        memcpy(&f32_AccelY, arc_TempBuffer + 28, sizeof(float32_t));
        memcpy(&f32_AccelZ, arc_TempBuffer + 32, sizeof(float32_t));
        memcpy(&f32_GyroX, arc_TempBuffer + 36, sizeof(float32_t));
        memcpy(&f32_GyroY, arc_TempBuffer + 40, sizeof(float32_t));
        memcpy(&f32_GyroZ, arc_TempBuffer + 44, sizeof(float32_t));

        float64_t rollAcc = atan2(f32_AccelY, f32_AccelZ) * 180 / M_PI;
        float64_t pitchAcc = atan2(-f32_AccelX, sqrt(f32_AccelY * f32_AccelY + f32_AccelZ * f32_AccelZ)) * 180 / M_PI;

        roll = getAngle(rst_KFRoll, rollAcc, f32_GyroX, 0.01);
        pitch = getAngle(rst_KFPitch, pitchAcc, f32_GyroY, 0.01);
        f32_TempRoll = roll;

        // roll = -pitch;

        // pitch = -f32_TempRoll;

        // // printf("roll : %f\n", roll);
        // // printf(" Roll : ")
        // float32_t f32_matrix[3][3];
        // rotationMatrix(-f32_Pitch_deg, -f32_Roll_deg, -90, f32_matrix);
        yaw += f32_GyroZ * 0.01;

        // if(s32_Recv < 0)
        // {
        //     printf("lidar UDP Failed\n");
        //     continue;
        // }
    }
}

void *LiDARStatusWrapper(void *arg)
{
    YAML::Node st_Config = YAML::LoadFile("configuration.yaml");
    s_LiDARHzMode = st_Config["lidar_mode"].as<std ::string>();
    LIDARFailSafe(s_LiDARHzMode);
}
