#include <kernel.cuh>

#include "cukd/builder.h"
#include "cukd/fcp.h"

Octree::Octree(OCTREE_POINT_t st_Center, float32_t f32_HalfLength, ICP_POINT_CLOUD_t* pst_PointCloud)
{
    int32_t s32_I = 0, s32_J = 0;

    for (s32_I = 0; s32_I < pst_PointCloud->s32_PointNum; s32_I++)
    {

        st_Coords.push_back({ pst_PointCloud->arf32_X[s32_I], 
                              pst_PointCloud->arf32_Y[s32_I], 
                              pst_PointCloud->arf32_Z[s32_I]});
    }

    pst_RootNode = new OCTREE_NODE_t();
    pst_RootNode->s32_FirstChildIdx = 0;
    pst_RootNode->st_Children = std::vector<OCTREE_NODE_t*>();
    pst_RootNode->st_Data = std::vector<OCTREE_POINT_t>();
    pst_RootNode->st_DataIdx = std::vector<int32_t>();
    pst_RootNode->st_Center = st_Center;
    pst_RootNode->f32_HalfLength = f32_HalfLength;
    pst_RootNode->b_IsLeaf = true;
    st_NodePool.push_back(pst_RootNode);
}


Octree::~Octree()
{
    for (auto st_Node : st_NodePool)
    {
        free(st_Node);
    }
}

void Octree::Create()
{
    uint64_t u64_Root = 0;
    int32_t s32_I = 0;

    for (s32_I = 0; s32_I < st_Coords.size(); s32_I++) {
        Insert(u64_Root, st_Coords[s32_I], s32_I);
    }
}

void Octree::Insert(uint64_t u64_CurKey, OCTREE_POINT_t st_Data, int32_t s32_DataIdx)
{
    OCTREE_NODE_t* pst_CurOctreeNode = st_NodePool[u64_CurKey];
    int32_t s32_I;
    int32_t s32_X, s32_Y, s32_Z = 0;
    uint64_t u64_BaseKey;
    uint64_t u64_NewKey;
    float32_t f32_NewHalfLength;

    //Option 1:Check if we're at a leaf, then add the point
    if (pst_CurOctreeNode->b_IsLeaf) 
    {
    
        bool b_HasMaxData = pst_CurOctreeNode->st_Data.size() >= c_MAX_PTS_PER_OCTANT;
        
        if (!b_HasMaxData) 
        { //If we haven't surpassed MAX_PTS_PER_OCTANT, just append the data
            pst_CurOctreeNode->st_Data.push_back(st_Data);
            pst_CurOctreeNode->st_DataIdx.push_back(s32_DataIdx);
        }
        else 
        { //We have surpassed MAX_PTS_PER_OCTANT
            //1:Subdivide
            u64_BaseKey = st_NodePool.size();
            f32_NewHalfLength = pst_CurOctreeNode->f32_HalfLength / 2.f;
            pst_CurOctreeNode->s32_FirstChildIdx = u64_BaseKey;
            pst_CurOctreeNode->b_IsLeaf = false;

            for (s32_Z = 0; s32_Z < 2; s32_Z++) 
            {
                for (s32_Y = 0; s32_Y < 2; s32_Y++) 
                {
                    for (s32_X = 0; s32_X < 2; s32_X++) 
                    {
                        //Update the code
                        u64_NewKey = u64_BaseKey + (s32_X + s32_Y * 2 + s32_Z * 4);

                        //Update the center
                        OCTREE_POINT_t st_Center = pst_CurOctreeNode->st_Center;
                        OCTREE_POINT_t st_NewCenter = { 0.f, 0.f, 0.f };
                        st_NewCenter.f32_X = st_Center.f32_X + f32_NewHalfLength * (s32_X ? 1 : -1);
                        st_NewCenter.f32_Y = st_Center.f32_Y + f32_NewHalfLength * (s32_Y ? 1 : -1);
                        st_NewCenter.f32_Z = st_Center.f32_Z + f32_NewHalfLength * (s32_Z ? 1 : -1);

                        //Update new entry in octNodePool
                        OCTREE_NODE_t* pst_NewNode = new OCTREE_NODE_t();
                        pst_NewNode->s32_FirstChildIdx = 0;
                        pst_NewNode->st_Center = st_NewCenter;
                        pst_NewNode->f32_HalfLength = f32_NewHalfLength;
                        pst_NewNode->b_IsLeaf = true;
                        st_NodePool.push_back(pst_NewNode);

                        //Add child to parent
                        pst_CurOctreeNode->st_Children.push_back(pst_NewNode);
                    }
                }
            }
            //2:Redistribute Points
            for (s32_I = 0; s32_I < pst_CurOctreeNode->st_Data.size(); s32_I++) 
            {
                Insert(u64_CurKey, pst_CurOctreeNode->st_Data[s32_I], pst_CurOctreeNode->st_DataIdx[s32_I]);
            }
            Insert(u64_CurKey, st_Data, s32_DataIdx);
        }
    }
    else 
    {
        OCTREE_POINT_t st_Center = pst_CurOctreeNode->st_Center;
        
        //Determine which octant the point lies in (0 is bottom-back-left)
        s32_X = st_Data.f32_X > st_Center.f32_X;
        s32_Y = st_Data.f32_Y > st_Center.f32_Y;
        s32_Z = st_Data.f32_Z > st_Center.f32_Z;

        //Update the code
        u64_NewKey = pst_CurOctreeNode->s32_FirstChildIdx + (s32_X + s32_Y * 2 + s32_Z * 4);

        Insert(u64_NewKey, st_Data, s32_DataIdx);
    }

}

void Octree::Compact()
{
    int32_t s32_I = 0, s32_J = 0;

    for (s32_I = 0; s32_I < st_NodePool.size(); s32_I++) 
    {
        OCTREE_NODE_t* pst_CurNode = st_NodePool[s32_I];
        OCTREE_NODE_GPU_t st_GpuNode;

        //copy currNode into gpuNode
        st_GpuNode.s32_FirstChildIdx = pst_CurNode->s32_FirstChildIdx;
        st_GpuNode.b_IsLeaf = pst_CurNode->b_IsLeaf;
        st_GpuNode.st_Center = pst_CurNode->st_Center;

        if (st_GpuNode.b_IsLeaf && pst_CurNode->st_Data.size() > 0) 
        {
            
            st_GpuNode.s32_Count = pst_CurNode->st_Data.size();
            st_GpuNode.s32_DataStartIdx = st_GpuCoords.size();

            for (s32_J = 0; s32_J < st_GpuNode.s32_Count; s32_J++) 
            {
                st_GpuCoords.push_back(pst_CurNode->st_DataIdx[s32_J]);
            }
        }
        else 
        {
            st_GpuNode.s32_Count = 0;
            st_GpuNode.s32_DataStartIdx = -1;
        }
        st_GpuNodePool.push_back(st_GpuNode);
    }
}


__device__
float32_t CUDA_rad2deg(float32_t f32_A)
{
    f32_A = f32_A >= 0.0f ? f32_A : (f32_A + (2 * M_PI));
    return f32_A * 180.0f / M_PI;
}

__device__
float32_t CUDA_deg2rad(float32_t f32_A)
{
    f32_A = f32_A >= 0.0f ? f32_A : (f32_A + 360.f);
    return f32_A * M_PI / 180.0f;
}

__device__
float32_t CUDA_getDistance2d(float32_t f32_X1, float32_t f32_Y1, float32_t f32_X2, float32_t f32_Y2)
{
    return sqrtf(powf(f32_X1 - f32_X2, 2.f) + powf(f32_Y1 - f32_Y2, 2.f));
}

__device__
void rotationMatrix(float32_t f32_Roll, float32_t f32_Pitch, float32_t f32_Yaw, float32_t f32_matrix[3][3]) 
{
    float32_t f32_cos_roll = cos(CUDA_deg2rad(f32_Roll));
    float32_t f32_sin_roll = sin(CUDA_deg2rad(f32_Roll));
    float32_t f32_cos_pitch = cos(CUDA_deg2rad(f32_Pitch));
    float32_t f32_sin_pitch = sin(CUDA_deg2rad(f32_Pitch));
    float32_t f32_cos_yaw = cos(CUDA_deg2rad(f32_Yaw));
    float32_t f32_sin_yaw = sin(CUDA_deg2rad(f32_Yaw));

    f32_matrix[0][0] = f32_cos_yaw * f32_cos_pitch;
    f32_matrix[0][1] = f32_cos_yaw * f32_sin_pitch * f32_sin_roll - f32_sin_yaw * f32_cos_roll;
    f32_matrix[0][2] = f32_cos_yaw * f32_sin_pitch * f32_cos_roll + f32_sin_yaw * f32_sin_roll;
    f32_matrix[1][0] = f32_sin_yaw * f32_cos_pitch;
    f32_matrix[1][1] = f32_sin_yaw * f32_sin_pitch * f32_sin_roll + f32_cos_yaw * f32_cos_roll;
    f32_matrix[1][2] = f32_sin_yaw * f32_sin_pitch * f32_cos_roll - f32_cos_yaw * f32_sin_roll;
    f32_matrix[2][0] = -f32_sin_pitch;
    f32_matrix[2][1] = f32_cos_pitch * f32_sin_roll;
    f32_matrix[2][2] = f32_cos_pitch * f32_cos_roll;
}

// Translate Cur to DB
__global__
void CUDA_MakeTranslationInfo(OMNI_FEATURE_t* pst_A, OMNI_FEATURE_t* pst_B, float32_t* pf32_Cos, float32_t* pf32_Sin, bool* pb_Available)
{
    int32_t s32_Index = blockIdx.x * blockDim.x + threadIdx.x;

    int32_t s32_I = s32_Index / 360;
    int32_t s32_J = s32_Index % 360;
    int32_t s32_NextI = (s32_I + 2) % 360;
    int32_t s32_NextJ = (s32_J + 2) % 360;

    float32_t f32_AzimuthA = 0.f;
    float32_t f32_AzimuthB = 0.f;

    pb_Available[s32_Index] = false;

    if (!(pst_A->arb_Valid[s32_I] && pst_A->arb_Valid[s32_NextI]))
    {
        return;
    }

    if (!(pst_B->arb_Valid[s32_J] && pst_B->arb_Valid[s32_NextJ]))
    {
        return;
    }

    f32_AzimuthA = atan2f(pst_A->arf32_Y[s32_NextI] - pst_A->arf32_Y[s32_I], pst_A->arf32_X[s32_NextI] - pst_A->arf32_X[s32_I]);
    f32_AzimuthB = atan2f(pst_B->arf32_Y[s32_NextJ] - pst_B->arf32_Y[s32_J], pst_B->arf32_X[s32_NextJ] - pst_B->arf32_X[s32_J]);

    pf32_Cos[s32_Index] = cos(f32_AzimuthA - f32_AzimuthB);
    pf32_Sin[s32_Index] = sin(f32_AzimuthA - f32_AzimuthB);
    pb_Available[s32_Index] = true;
}


//// FeatureA : DB, FeatureB : Cur Scan
__global__
void CUDA_BestMatcingCalculator(OMNI_FEATURE_t* pst_A, OMNI_FEATURE_t* pst_B, float32_t* pf32_Cos, float32_t* pf32_Sin, bool* pb_Available, int32_t* ps32_Result, float32_t* pf32_Result)
{
    int32_t s32_Index = blockIdx.x * blockDim.x + threadIdx.x;
    int32_t s32_I = s32_Index / 360;
    int32_t s32_J = s32_Index % 360;
    int32_t s32_K = 0;
    int32_t s32_L = 0;
    int32_t s32_Result = 0;
    float32_t f32_Result = 0.f;

    float32_t f32_X = 0.f;
    float32_t f32_Y = 0.f;
    float32_t f32_TempX = 0.f;
    float32_t f32_TempY = 0.f;

    float32_t f32_Azimuth = 0.f;
    float32_t f32_Distance = 0.f;
    float32_t f32_DistaceA = 0.f;

    ps32_Result[s32_Index] = 0;
    pf32_Result[s32_Index] = 0;
    
    if (!pb_Available[s32_Index])
    {
        return;
    }


    for (s32_K = 0; s32_K < 360; s32_K++)
    {
        if (!pst_B->arb_Valid[s32_K])
        {
            continue;
        }

        f32_TempX = pst_B->arf32_X[s32_K] - pst_B->arf32_X[s32_J];
        f32_TempY = pst_B->arf32_Y[s32_K] - pst_B->arf32_Y[s32_J];

        f32_X = (f32_TempX * pf32_Cos[s32_Index] - f32_TempY * pf32_Sin[s32_Index]) + pst_A->arf32_X[s32_I];
        f32_Y = (f32_TempX * pf32_Sin[s32_Index] + f32_TempY * pf32_Cos[s32_Index]) + pst_A->arf32_Y[s32_I];

        f32_Azimuth = CUDA_rad2deg(atan2f(f32_Y, f32_X));
        f32_Distance = sqrtf(powf(f32_X, 2.f) + powf(f32_Y, 2.f));
        s32_L = (int32_t)roundf(f32_Azimuth);
        f32_DistaceA = sqrtf(powf(pst_A->arf32_X[s32_L], 2.f) + powf(pst_A->arf32_Y[s32_L], 2.f));

        //if (f32_Distance >= 200.f)
        //{
        //    continue;
        //}


        //if (!pst_A->arb_Valid[s32_L])
        //{
        //    continue;
        //}

        if (pst_A->arb_Valid[s32_L] && fabsf(f32_DistaceA - f32_Distance) < 0.2f)
        {
            s32_Result += 1;
            //f32_Result += fabsf(pst_A->arf32_Distance[s32_L] - f32_Distance);
        }
    }

    ps32_Result[s32_Index] = s32_Result;
    pf32_Result[s32_Index] = f32_Result;
}

// __global__ void myKernel() {
//     int idx = threadIdx.x + blockIdx.x * blockDim.x;
//     printf("Hello from thread %d\n", idx);
// }

void checkCudaError(cudaError_t err, const char* message) {
    if (err != cudaSuccess) {
        std::cerr << message << ": " << cudaGetErrorString(err) << std::endl;
        exit(EXIT_FAILURE);
    }
}


//// FeatureA : DB, FeatureB : Cur Scan
void CUDA_BestMatching(OMNI_FEATURE_t* pst_FeatureA, OMNI_FEATURE_t* pst_FeatureB, int32_t* ps32_Result, float32_t* pf32_Result)
{
    OMNI_FEATURE_t* pst_FeatureA_device;
    OMNI_FEATURE_t* pst_FeatureB_device;
    int32_t* ps32_Result_device;
    float32_t* pf32_Result_device;

    float32_t* pf32_Cos_device;
    float32_t* pf32_Sin_device;
    bool* pb_Available_device;
    cudaError_t st_Err;

    cudaMalloc((void**)&pst_FeatureA_device, sizeof(OMNI_FEATURE_t));
    cudaMalloc((void**)&pst_FeatureB_device, sizeof(OMNI_FEATURE_t));
    cudaMalloc((void**)&ps32_Result_device, sizeof(int32_t) * 360 * 360);
    cudaMalloc((void**)&pf32_Result_device, sizeof(float32_t) * 360 * 360);

    cudaMalloc((void**)&pf32_Cos_device, sizeof(float32_t) * 360 * 360);
    cudaMalloc((void**)&pf32_Sin_device, sizeof(float32_t) * 360 * 360);
    cudaMalloc((void**)&pb_Available_device, sizeof(bool) * 360 * 360);

    st_Err = cudaGetLastError();
    checkCudaError(st_Err, "CUDA_BestMatching Malloc Error");


    cudaMemcpy(pst_FeatureA_device, pst_FeatureA, sizeof(OMNI_FEATURE_t), cudaMemcpyHostToDevice);
    cudaMemcpy(pst_FeatureB_device, pst_FeatureB, sizeof(OMNI_FEATURE_t), cudaMemcpyHostToDevice);

    st_Err = cudaGetLastError();
    checkCudaError(st_Err, "CUDA_BestMatching Memcpy Error");

    dim3 st_Grid(2025, 1, 1);
    dim3 st_Block(64, 1, 1);

    CUDA_MakeTranslationInfo << < st_Grid, st_Block >> > (pst_FeatureA_device, pst_FeatureB_device, pf32_Cos_device, pf32_Sin_device, pb_Available_device);
    cudaDeviceSynchronize();
    st_Err = cudaGetLastError();
    checkCudaError(st_Err, "CUDA_MakeTranslationInfo Error");

    CUDA_BestMatcingCalculator << < st_Grid, st_Block >> > (pst_FeatureA_device, pst_FeatureB_device, pf32_Cos_device, pf32_Sin_device, pb_Available_device, ps32_Result_device, pf32_Result_device);
    cudaDeviceSynchronize();
    st_Err = cudaGetLastError();
    checkCudaError(st_Err, "CUDA_BestMatcingCalculator Error");


    cudaMemcpy(ps32_Result, ps32_Result_device, sizeof(int32_t) * 360 * 360, cudaMemcpyDeviceToHost);
    cudaMemcpy(pf32_Result, pf32_Result_device, sizeof(float32_t) * 360 * 360, cudaMemcpyDeviceToHost);

    cudaFree(pf32_Cos_device);
    cudaFree(pf32_Sin_device);
    cudaFree(pb_Available_device);

    cudaFree(pst_FeatureA_device);
    cudaFree(pst_FeatureB_device);
    cudaFree(ps32_Result_device);
    cudaFree(pf32_Result_device);
}









__global__
void CUDA_LidarParsing(char* pc_Buffer, HMC_LIDAR_DATA_t* pst_LidarPointData, float32_t f32_Roll, float32_t f32_Pitch)
{
    // blockIdx.x = 0~1024
    // threadIdx.x = 0~128

    const float32_t f32_RefRange_mm = 27.396999999999998f;

    int32_t s32_PointNum;
    int32_t s32_Index;
    int32_t s32_Box;
    int32_t s32_Cur;
    int32_t s32_Column;
    int32_t s32_PointIndex;
    int32_t s32_X, s32_Y, s32_Z;
    uint64_t u64_Hash = 0;

    float32_t f32_MeasurementID;
    float32_t f32_Encoder;
    float32_t f32_Azimuth_deg;
    float32_t f32_Azimuth_rad;
    float32_t f32_Theta_deg;
    float32_t f32_Theta_rad;
    float32_t f32_Range_mm;
    float32_t f32_X, f32_Y, f32_Z;
    float32_t f32_RX, f32_RY, f32_RZ;
    float32_t f32_Distance;
    float32_t f32_Intensity;
    float32_t f32_Matrix[3][3];
    rotationMatrix(f32_Roll, -f32_Pitch, 0.f, f32_Matrix);
    s32_Box = blockIdx.x / 16; // 0~1024column / 16column = which box
    // s32_Cur = 24800 * s32_Box + 32; // ( (12byte*128point + 12byte) * 16column ) * 64box = which column 
    s32_Cur = 24768 * s32_Box; // ( (12byte*128point + 12byte) * 16column ) * 64box = which column 

    s32_Column = blockIdx.x % 16; // 0~1024column % 16column = which column(max16) in box
    s32_Index = threadIdx.x; // 0~128point
    s32_PointNum = blockIdx.x * 128; // 0~1024column * 128point = which point num

    f32_MeasurementID = (float32_t)(((uint8_t)pc_Buffer[s32_Cur + 1548 * s32_Column + 9] << 8 | (uint8_t)pc_Buffer[s32_Cur + 1548 * s32_Column + 8]));
    f32_Encoder = 2.f * M_PI * (1.f - (f32_MeasurementID / 1024.f));

    f32_Azimuth_deg = -arf32_BeamAzimuthAngles_device[s32_Index];
    f32_Azimuth_rad = CUDA_deg2rad(f32_Azimuth_deg) + f32_Encoder + M_PI;

    if (f32_Azimuth_rad >= M_PI * 2)
    {
        f32_Azimuth_rad -= 2 * M_PI;
    }
    else if (f32_Azimuth_rad < 0.f)
    {
        f32_Azimuth_rad += 2 * M_PI;
    }

    f32_Azimuth_deg = CUDA_rad2deg(f32_Azimuth_rad);
    f32_Theta_deg = arf32_BeamAltitudeAngles_device[s32_Index];
    f32_Theta_rad = CUDA_deg2rad(f32_Theta_deg);

    f32_Range_mm = (float32_t)(((uint8_t)pc_Buffer[s32_Cur + 1548 * s32_Column + 14 + (12 * s32_Index)] & 7) << 16 |
        (uint8_t)pc_Buffer[s32_Cur + 1548 * s32_Column + 13 + (12 * s32_Index)] << 8 |
        (uint8_t)pc_Buffer[s32_Cur + 1548 * s32_Column + 12 + (12 * s32_Index)]);

    f32_X = (f32_Range_mm - f32_RefRange_mm) * cosf(f32_Azimuth_rad) * cosf(f32_Theta_rad) + (arf32_BeamToLidar_device[0 * 4 + 3] * cosf(f32_Encoder));
    f32_Y = (f32_Range_mm - f32_RefRange_mm) * sinf(f32_Azimuth_rad) * cosf(f32_Theta_rad) + (arf32_BeamToLidar_device[0 * 4 + 3] * sinf(f32_Encoder));
    f32_Z = (f32_Range_mm - f32_RefRange_mm) * sinf(f32_Theta_rad) + (arf32_BeamToLidar_device[2 * 4 + 3]);
    f32_X /= 1000.f;
    f32_Y /= 1000.f;
    f32_Z /= 1000.f;
    // f32_X = f32_X - 2.f;
    // f32_Y = f32_Y;
    // f32_Z = f32_Z - 1.5f;
    // f32_RX = f32_Matrix[0][0] * f32_X + f32_Matrix[0][1] * f32_Y + f32_Matrix[0][2] * f32_Z;
    // f32_RY = f32_Matrix[1][0] * f32_X + f32_Matrix[1][1] * f32_Y + f32_Matrix[1][2] * f32_Z;
    // f32_RZ = f32_Matrix[2][0] * f32_X + f32_Matrix[2][1] * f32_Y + f32_Matrix[2][2] * f32_Z;

    f32_Distance = sqrtf(powf(f32_X, 2.f) + powf(f32_Y, 2.f));
    f32_Intensity = (float32_t)((uint8_t)pc_Buffer[s32_Cur + 1548 * s32_Column + 16 + (12 * s32_Index)]);
    s32_PointIndex = (s32_PointNum + ars32_PointIndex_device[s32_Index] + 131072) % 131072;

    pst_LidarPointData->arst_Point[s32_PointIndex].f32_Azimuth_rad = f32_Azimuth_rad;
    pst_LidarPointData->arst_Point[s32_PointIndex].f32_Azimuth_deg = f32_Azimuth_deg;

    pst_LidarPointData->arst_Point[s32_PointIndex].f32_Theta_rad = f32_Theta_rad;
    pst_LidarPointData->arst_Point[s32_PointIndex].f32_Theta_deg = f32_Theta_deg;

    pst_LidarPointData->arst_Point[s32_PointIndex].f32_R = (f32_Range_mm - f32_RefRange_mm) / 1000.f;
    pst_LidarPointData->arst_Point[s32_PointIndex].f32_Distance = f32_Distance;
    pst_LidarPointData->arst_Point[s32_PointIndex].u8_Intensity = f32_Intensity;
    pst_LidarPointData->arst_Point[s32_PointIndex].u8_Layer = 127 - s32_Index;

    // f32_X = (f32_RX + 2.f) / 1000.f;
    // f32_Y = f32_RY / 1000.f;
    // f32_Z = (f32_RZ + 1.5f) / 1000.f;


    pst_LidarPointData->arst_Point[s32_PointIndex].f32_X = f32_X;
    pst_LidarPointData->arst_Point[s32_PointIndex].f32_Y = f32_Y;
    pst_LidarPointData->arst_Point[s32_PointIndex].f32_Z = f32_Z;

    pst_LidarPointData->arst_Point[s32_PointIndex].s32_VoxelIndex = -1;

    // if (pst_LidarPointData->arst_Point[s32_PointIndex].f32_R < 0.5f)
    // {
    //     pst_LidarPointData->arst_Point[s32_PointIndex].u8_Flag = c_POINT_INVALID;
    // }
    // else
    // {
    // }
    pst_LidarPointData->arst_Point[s32_PointIndex].u8_Flag = c_POINT_GROUND;

    s32_X = (int32_t)(f32_X / c_ICP_DOWNSAMPLING_SIZE_XY);
    s32_Y = (int32_t)(f32_Y / c_ICP_DOWNSAMPLING_SIZE_XY);
    s32_Z = (int32_t)(f32_Z / c_ICP_DOWNSAMPLING_SIZE_Z);

    u64_Hash ^= ((uint64_t)s32_X + 0x9e3779b9 + (u64_Hash << 6) + (u64_Hash >> 2));
    u64_Hash ^= ((uint64_t)s32_Y + 0x9e3779b9 + (u64_Hash << 6) + (u64_Hash >> 2));
    u64_Hash ^= ((uint64_t)s32_Z + 0x9e3779b9 + (u64_Hash << 6) + (u64_Hash >> 2));
    u64_Hash %= c_DOWNSAMPLING_VOXEL_CAPACITY;

    pst_LidarPointData->arst_Point[s32_PointIndex].u64_Hash = u64_Hash;

    pst_LidarPointData->s32_PointNum = 131072;
    pst_LidarPointData->s32_LayerNum = 128;
}

__global__
void CUDA_LidarParsing_VLS128(char* pc_Buffer, HMC_LIDAR_DATA_t* pst_LidarPointData, float32_t f32_Roll, float32_t f32_Pitch)
{
    // blockIdx.x = 0~599
    // threadIdx.x = 0~383

    int32_t s32_PacketId = blockIdx.x;             // 0~599, Packet ID in Whole Buffer
    int32_t s32_PointId = threadIdx.x;             // 0~383, Point ID in Which Packet

    int32_t s32_BlockNum = s32_PointId / 32;       // 0~11, Block Number in Which Packet
    int32_t s32_LocalPointId = s32_PointId & 31;   // 0~31, Point ID in Which Data Block

    int32_t s32_I = s32_PacketId * 1200;
    int32_t s32_J = s32_BlockNum * 100;
    int32_t s32_K = s32_LocalPointId * 3 + 4;
    int32_t s32_PointNum = s32_PacketId * 384 + s32_PointId;
    int32_t s32_PointOffset = (s32_PointNum / 128) * 128;

    int32_t s32_AzimuthBufferOffset = s32_I + s32_J;
    float32_t f32_Azimuth = (float32_t)(((uint8_t)pc_Buffer[s32_AzimuthBufferOffset + 3] << 8) |
                                (uint8_t)pc_Buffer[s32_AzimuthBufferOffset + 2]) * 0.01f;

    int32_t s32_DistanceBufferOffset = s32_I + s32_J + s32_K;
    float32_t f32_Distance = (((uint8_t)pc_Buffer[s32_DistanceBufferOffset + 1] << 8) |
                          (uint8_t)pc_Buffer[s32_DistanceBufferOffset]) * 0.004f;

    uint8_t u8_Intensity = (uint8_t)pc_Buffer[s32_DistanceBufferOffset + 2];

    uint8_t u8_LaserOrder = (s32_PointNum & 127);

    float32_t f32_Azimuth_deg = f32_Azimuth - arf32_AzimuthOffset_VLS128[u8_LaserOrder] + 90;
    
    if (f32_Azimuth_deg < 0.f) f32_Azimuth_deg += 360.f;
    else if (f32_Azimuth_deg > 360.f) f32_Azimuth_deg -= 360.f;

    float32_t f32_Elevation_deg = arf32_ElevationDeg_VLS128[u8_LaserOrder];
    uint8_t u8_Channel = aru8_ChannelMappingTable_VLS128[u8_LaserOrder];
    float32_t f32_Omega = arf32_ElevationRad_VLS128[u8_Channel] * 0.000001f;

    float32_t f32_Alpha = f32_Azimuth_deg * 0.01745329251f;   // deg to rad

    float32_t f32_X = f32_Distance * arf32_CosineOmega[u8_Channel] * sinf(f32_Alpha);
    float32_t f32_Y = f32_Distance * arf32_CosineOmega[u8_Channel] * cosf(f32_Alpha);
    float32_t f32_Z = f32_Distance * arf32_SineOmega[u8_Channel];

    int32_t s32_OrderedPointNum = ((s32_PointOffset - ars32_AzimuthIndexOffset_VLS128[u8_LaserOrder] + u8_Channel) + 230400) % 230400;
    
    pst_LidarPointData->arst_Point[s32_OrderedPointNum].f32_X = f32_X;
    pst_LidarPointData->arst_Point[s32_OrderedPointNum].f32_Y = f32_Y;
    pst_LidarPointData->arst_Point[s32_OrderedPointNum].f32_Z = f32_Z;
    pst_LidarPointData->arst_Point[s32_OrderedPointNum].f32_R = f32_Distance;
    pst_LidarPointData->arst_Point[s32_OrderedPointNum].f32_Distance = sqrtf(f32_X * f32_X + f32_Y * f32_Y);
    pst_LidarPointData->arst_Point[s32_OrderedPointNum].f32_Azimuth_deg = f32_Azimuth_deg;
    pst_LidarPointData->arst_Point[s32_OrderedPointNum].f32_Azimuth_rad = f32_Alpha;
    pst_LidarPointData->arst_Point[s32_OrderedPointNum].f32_Theta_deg = f32_Elevation_deg;
    pst_LidarPointData->arst_Point[s32_OrderedPointNum].f32_Theta_rad = f32_Omega;
    pst_LidarPointData->arst_Point[s32_OrderedPointNum].u8_Intensity = u8_Intensity;
    pst_LidarPointData->arst_Point[s32_OrderedPointNum].u8_Layer = u8_Channel;
    pst_LidarPointData->arst_Point[s32_OrderedPointNum].s32_VoxelIndex = -1;

    pst_LidarPointData->s32_PointNum = 230400;
    pst_LidarPointData->s32_LayerNum = 128;

    if (f32_Distance < 0.5f) 
    {
        pst_LidarPointData->arst_Point[s32_OrderedPointNum].u8_Flag = c_POINT_INVALID;
    } 
    else {
        pst_LidarPointData->arst_Point[s32_OrderedPointNum].u8_Flag = c_POINT_GROUND;
    }
}

__global__
void CUDA_LidarGroundFiltering(HMC_LIDAR_DATA_t *pst_LidarPointData)
{
    // blockldx.x : 0~1024
    // threadldx.x : 0~127

    // Threshold
    float32_t f32_CriteriaDist = 0.5f;
    float32_t f32_ThresholdAngleVer = 0.3f;
    float32_t f32_ThresholdZDiffVer = 0.2f;

    float32_t f32_ThresholdAngleDiffHor = 0.1f;
    float32_t f32_ThresholdZDiffHor = 0.1f;

    // Variables
    int32_t s32_CurrentIndex, s32_NextIndex;
    int32_t s32_AzimuthIndex;
    int32_t s32_PointIndex;
    int32_t u8_Flag, u8_NextFlag;
    int32_t s32_SearchIndex;
    int8_t u8_CurrentIntensity, u8_NextIntensity;

    int32_t s32_PrevAzimuthPointIdx, s32_NextAzimuthPointIdx;
    float32_t f32_X1, f32_X2, f32_X3, f32_X4, f32_X5, f32_Y1, f32_Y2, f32_Y3, f32_Y4, f32_Y5, f32_Z1, f32_Z2, f32_Z3, f32_Z4, f32_Z5;
    float32_t f32_DistXY1, f32_DistXY2, f32_DistZ1, f32_DistZ2, f32_SlopeAngleVer, f32_SlopeAngleHor1, f32_SlopeAngleHor2;
    float32_t f32_DistX1, f32_DistX2, f32_DistY1, f32_DistY2;
    int32_t s32_PointNum = pst_LidarPointData->s32_PointNum;
    int32_t s32_PointCompNum = s32_PointNum - 128;
    
    HMC_POINT_t* pst_CurPoint;
    HMC_POINT_t* pst_NextPoint;
    HMC_POINT_t* pst_PrevAzimuthPoint;
    HMC_POINT_t* pst_NextAzimuthPoint;
    HMC_POINT_t* pst_LastGNDPoint;
    
    s32_AzimuthIndex = 128 * blockIdx.x;
    s32_PointIndex = threadIdx.x;
    
    s32_CurrentIndex = s32_AzimuthIndex + s32_PointIndex;
    s32_NextIndex = s32_CurrentIndex + 1;
    pst_CurPoint = &pst_LidarPointData->arst_Point[s32_CurrentIndex];
    pst_NextPoint = &pst_LidarPointData->arst_Point[s32_NextIndex];

    u8_Flag = pst_CurPoint->u8_Flag;
    u8_NextFlag = pst_NextPoint->u8_Flag;

    if (pst_LidarPointData->s32_PointNum != 0)
    {
        s32_CurrentIndex = s32_AzimuthIndex + s32_PointIndex;
        s32_NextIndex = s32_CurrentIndex + 1;

        pst_CurPoint = &pst_LidarPointData->arst_Point[s32_CurrentIndex];
        pst_NextPoint = &pst_LidarPointData->arst_Point[s32_NextIndex];

        u8_Flag = pst_CurPoint->u8_Flag;
        u8_NextFlag = pst_NextPoint->u8_Flag;

        if(s32_PointIndex == 0)
        {
            pst_CurPoint->u8_Flag = c_POINT_GROUND;
        }
        else if (
            (u8_Flag != c_POINT_INVALID && u8_NextFlag != c_POINT_INVALID) &&
            (s32_CurrentIndex < (s32_AzimuthIndex + 128)) && (s32_NextIndex < (s32_AzimuthIndex + 128))    
        )
        {
            f32_X1 = pst_CurPoint->f32_X;
            f32_Y1 = pst_CurPoint->f32_Y;
            f32_Z1 = pst_CurPoint->f32_Z;

            f32_X2 = pst_NextPoint->f32_X;
            f32_Y2 = pst_NextPoint->f32_Y;
            f32_Z2 = pst_NextPoint->f32_Z;

            f32_DistX1 = f32_X2 - f32_X1;
            f32_DistY1 = f32_Y2 - f32_Y1;
            
            f32_DistXY1 = sqrtf((f32_DistX1 * f32_DistX1) + (f32_DistY1 * f32_DistY1));
            f32_DistZ1 = fabsf(f32_Z2 - f32_Z1);
            f32_SlopeAngleVer = atan2f(f32_DistZ1, f32_DistXY1);

            u8_CurrentIntensity = pst_CurPoint->u8_Intensity;
            u8_NextIntensity = pst_NextPoint->u8_Intensity;

            // 높이 기반 1차 제거
            if(f32_Z1 < -1.5f)
            {
                pst_CurPoint->u8_Flag = c_POINT_GROUND;
            }

            // Slope 기반 2차 제거
            // Channel 간 비교
            
            if(fabsf(u8_CurrentIntensity - u8_NextIntensity) < 4 && u8_CurrentIntensity < 10)
            {
                pst_CurPoint->u8_Flag = c_POINT_GROUND;
            }
            else if(f32_DistXY1 < f32_CriteriaDist)
            {
                f32_DistZ1 = fabsf(f32_Z2 - f32_Z1);
                f32_SlopeAngleVer = atan2f(f32_DistZ1, f32_DistXY1);
                
                if
                (
                    (f32_SlopeAngleVer < f32_ThresholdAngleVer) &&
                    (pst_CurPoint->f32_Distance < pst_NextPoint->f32_Distance) &&
                    (f32_DistZ1 < f32_ThresholdZDiffVer)
                )
                {   
                    pst_CurPoint->u8_Flag = c_POINT_GROUND;
                    pst_NextPoint->u8_Flag = c_POINT_GROUND;
                    pst_CurPoint->u8_GroundFilteringFlag = 0; //초 
                }
                else
                {
                    pst_CurPoint->u8_Flag = c_POINT_VALID;
                }
            }
            // Channel 내 비교
            else
            {
                s32_PrevAzimuthPointIdx = (s32_CurrentIndex + s32_PointCompNum) % s32_PointNum;
                s32_NextAzimuthPointIdx = (s32_CurrentIndex + 128) % s32_PointNum;

                pst_PrevAzimuthPoint = &pst_LidarPointData->arst_Point[s32_PrevAzimuthPointIdx];
                pst_NextAzimuthPoint = &pst_LidarPointData->arst_Point[s32_NextAzimuthPointIdx];

                f32_X3 = pst_PrevAzimuthPoint->f32_X;
                f32_Y3 = pst_PrevAzimuthPoint->f32_Y;
                f32_Z3 = pst_PrevAzimuthPoint->f32_Z;

                f32_X4 = pst_NextAzimuthPoint->f32_X;
                f32_Y4 = pst_NextAzimuthPoint->f32_Y;
                f32_Z4 = pst_NextAzimuthPoint->f32_Z;

                f32_DistX1 = f32_X3 - f32_X1;
                f32_DistY1 = f32_Y3 - f32_Y1;

                f32_DistX2 = f32_X4 - f32_X1;
                f32_DistY2 = f32_Y4 - f32_Y1;

                f32_DistXY1 = sqrtf((f32_DistX1 * f32_DistX1) + (f32_DistY1 * f32_DistY1));
                f32_DistZ1 = fabsf(f32_Z3 - f32_Z1);
                f32_SlopeAngleHor1 = atan2f(f32_DistZ1, f32_DistXY1);

                f32_DistXY2 = sqrtf((f32_DistX2 * f32_DistX2) + (f32_DistY2 * f32_DistY2));
                f32_DistZ2 = fabsf(f32_Z4 - f32_Z1);
                f32_SlopeAngleHor2 = atan2f(f32_DistZ2, f32_DistXY2);

                if(
                    (f32_SlopeAngleHor1 + f32_SlopeAngleHor2) < f32_ThresholdAngleDiffHor &&
                    (f32_DistZ1 + f32_DistZ2) < f32_ThresholdZDiffHor
                )
                {
                    pst_CurPoint->u8_Flag = c_POINT_GROUND;
                    pst_CurPoint->u8_GroundFilteringFlag = 1; //하늘색

                }

                if(pst_CurPoint->u8_Flag == c_POINT_VALID)
                {
                    s32_SearchIndex = s32_CurrentIndex - 1;
                    while (s32_SearchIndex >= s32_AzimuthIndex && (s32_SearchIndex & 127) != 0)
                    {
                        if (pst_LidarPointData->arst_Point[s32_SearchIndex].u8_Flag == c_POINT_GROUND)
                        {
                            pst_LastGNDPoint = &pst_LidarPointData->arst_Point[s32_SearchIndex];

                            f32_X5 = pst_LastGNDPoint->f32_X;
                            f32_Y5 = pst_LastGNDPoint->f32_Y;
                            f32_Z5 = pst_LastGNDPoint->f32_Z;

                            f32_DistX1 = f32_X5 - f32_X1;
                            f32_DistY1 = f32_Y5 - f32_Y1;
                            f32_DistXY1 = sqrtf(f32_DistX1 * f32_DistX1 + f32_DistY1 * f32_DistY1);
                            f32_DistZ1 = fabsf(f32_Z5 - f32_Z1);

                            f32_SlopeAngleVer = atan2f(f32_DistZ1, f32_DistXY1);

                            if (f32_SlopeAngleVer < (f32_ThresholdAngleVer + 1.f))
                            {
                                pst_CurPoint->u8_Flag = c_POINT_GROUND;
                                pst_CurPoint->u8_GroundFilteringFlag = 2; // 빨강 
                            }
                            break;
                        }

                        s32_SearchIndex--;
                    }
                }
            }
        }
    }
}

__global__
void CUDA_LidarGroundFilteringV2(HMC_LIDAR_DATA_t *pst_LidarPointData, int32_t s32_LIDARMode)
{
    // blockldx.x : 0~1023 - OS2, 0~1799 - VLS
    // threadldx.x : 0~127

    // Variables
    bool b_ChkFirst = false;
    int32_t s32_I = 0;
    int32_t s32_J = 0;
    int32_t s32_Index = 128 * blockIdx.x;
    int32_t s32_CurrentIndex = 0;
    int32_t s32_NextIndex = 0;
    uint8_t u8_PrevFlag = 0;
    int32_t s32_PointNum = pst_LidarPointData->s32_PointNum;

    float32_t f32_X0, f32_Y0, f32_Z0;
    float32_t f32_X1, f32_Y1, f32_Z1;
    float32_t f32_X2, f32_Y2, f32_Z2;
    float32_t f32_DistX, f32_DistY;
    float32_t f32_Dist = 0.f;
    float32_t f32_DistXY;
    float32_t f32_DistZ;
    float32_t f32_SumZ;
    float32_t f32_Inclination;

    HMC_POINT_t* pst_CurPoint;
    HMC_POINT_t* pst_NextPoint;

    int32_t ars32_Buffer[128] = {0};
    int32_t s32_BufferNum = 0;

    while (s32_I < 128)
    {
        // INVALID는 넘어가면서 128 넘어가지 않도록 방지
        while (pst_LidarPointData->arst_Point[s32_Index + s32_I].u8_Flag != c_POINT_GROUND && s32_I < 128)
        {
            s32_I += 1;
        }

        // 127ch 점이면 break
        if (s32_I >= 127)
        {
            break;
        }
        // 그 이하의 Channel이면 현재 Index 받기
        else
        {
            s32_CurrentIndex = s32_Index + s32_I;
        }

        pst_CurPoint = &pst_LidarPointData->arst_Point[s32_CurrentIndex];

        f32_X1 = pst_CurPoint->f32_X;
        f32_Y1 = pst_CurPoint->f32_Y;
        f32_Z1 = pst_CurPoint->f32_Z;
        s32_BufferNum = 0;
        f32_SumZ = 0.f;

        for (s32_J = s32_I + 1;s32_J < 128;s32_J++)
        {
            s32_NextIndex = s32_Index + s32_J;
            pst_NextPoint = &pst_LidarPointData->arst_Point[s32_NextIndex];

            // 다음 점이 INVALID면 넘어간다.
            if (pst_NextPoint->u8_Flag != c_POINT_GROUND)
            {
                continue;
            }

            f32_X2 = pst_NextPoint->f32_X;
            f32_Y2 = pst_NextPoint->f32_Y;
            f32_Z2 = pst_NextPoint->f32_Z;

            f32_DistX = f32_X2 - f32_X1;
            f32_DistY = f32_Y2 - f32_Y1;
            
            f32_DistXY = sqrtf((f32_DistX * f32_DistX) + (f32_DistY * f32_DistY));
            f32_DistZ = f32_Z2 - f32_Z1;
    
            f32_Inclination = f32_DistZ / f32_DistXY;

            // 다음 점과의 거리가 0.5 이하(섹터 별로 지면제거 가능), 현재 점 보다 다음 점의 높이가 더 높고 현재점~다음점의 높이차가 0.1 이상
            // 이때는 Buffer에 그 다음 점들의 Index를 모두 담고 높이차를 누적합한다.


        //     if(s32_LIDARMode == c_HMC_MODE_OS2)
        //     {
        //         if (
        //             // (f32_DistXY < 0.2f) &&
        //             // (f32_DistZ > 0.05f) &&
        //             // (f32_DistZ < 0.5f) &&
        //             (f32_Inclination > 0.1f)
        //              // Bank Removal Rule
        //         )
        //         {
        //             ars32_Buffer[s32_BufferNum++] = s32_NextIndex;
        //             f32_SumZ += f32_DistZ;
        //         }
        //         // else if(
        //         //     (f32_Inclination < -0.01f) &&
        //         //     (f32_DistXY < 0.5f) &&
        //         //     (f32_DistZ > 0.08f) &&
        //         //     (f32_DistZ < 0.5f) 
        //         // )
        //         // {
        //         //     ars32_Buffer[s32_BufferNum++] = s32_NextIndex;
        //         //     f32_SumZ += f32_DistZ;
        //         // }
        //         else if(
        //             pst_CurPoint->f32_Distance > 60.f &&
        //             f32_DistXY < 1.5f &&
        //             (f32_Inclination > 1.f)
        //         )
        //         {
        //             ars32_Buffer[s32_BufferNum++] = s32_NextIndex;
        //             f32_SumZ += f32_DistZ;
        //         }
        //     }
        //     else
        //     {
        //         if (
        //             (f32_DistXY < 0.5f) &&
        //             (f32_Inclination > 1.2f) // Bank Removal Rule
        //         )
        //         {
        //             ars32_Buffer[s32_BufferNum++] = s32_NextIndex;
        //             f32_SumZ += f32_DistZ;
        //         }
        //         else if(
        //             pst_CurPoint->f32_Distance > 60.f &&
        //             f32_DistXY < 1.5f &&
        //             (f32_Inclination > 1.f)
        //         )
        //         {
        //             ars32_Buffer[s32_BufferNum++] = s32_NextIndex;
        //             f32_SumZ += f32_DistZ;
        //         }
        //     }
        // }

        // // Buffer 내 점의 갯수가 3 이상이고 높이차의 누적합이 0.3 이상이면 그 점들은 모두 살린다.
        // if ((s32_BufferNum >= 3) && (f32_SumZ > 0.2f))
        // {
        //     for (s32_J = 0;s32_J < s32_BufferNum;s32_J++)
        //     {
        //         pst_LidarPointData->arst_Point[ars32_Buffer[s32_J]].u8_Flag = c_POINT_VALID;
        //         pst_LidarPointData->arst_Point[ars32_Buffer[s32_J]].u8_GroundFilteringFlag = 1;
        //     }
        // }
        // // 현재 점까지의 거리가 60m 이상, Buffer 내 점의 갯수가 2 이상, 누적합이 0.2 이상이면 그 점들은 모두 살린다.
        // else if ((pst_CurPoint->f32_Distance > 60.f) && (s32_BufferNum >= 2) && (f32_SumZ > 0.2f))
        // {
        //     for (s32_J = 0;s32_J < s32_BufferNum;s32_J++)
        //     {
        //         pst_LidarPointData->arst_Point[ars32_Buffer[s32_J]].u8_Flag = c_POINT_VALID;
        //         pst_LidarPointData->arst_Point[ars32_Buffer[s32_J]].u8_GroundFilteringFlag = 2;
        //     }
        // }











            if(s32_LIDARMode == c_HMC_MODE_OS2)
            {
                if (
                    (f32_DistXY < 0.5f) &&
                    (f32_DistZ > 0.1f) &&
                    (f32_Inclination > 1.2f) // Bank Removal Rule
                )
                {
                    ars32_Buffer[s32_BufferNum++] = s32_NextIndex;
                    f32_SumZ += f32_DistZ;
                }
                else if(
                    pst_CurPoint->f32_Distance > 60.f &&
                    f32_DistXY < 1.5f &&
                    (f32_Inclination > 1.f)
                )
                {
                    ars32_Buffer[s32_BufferNum++] = s32_NextIndex;
                    f32_SumZ += f32_DistZ;
                }
            }
            else
            {
                if (
                    (f32_DistXY < 0.5f) &&
                    (f32_Inclination > 1.2f) // Bank Removal Rule
                )
                {
                    ars32_Buffer[s32_BufferNum++] = s32_NextIndex;
                    f32_SumZ += f32_DistZ;
                }
                else if(
                    pst_CurPoint->f32_Distance > 60.f &&
                    f32_DistXY < 1.5f &&
                    (f32_Inclination > 1.f)
                )
                {
                    ars32_Buffer[s32_BufferNum++] = s32_NextIndex;
                    f32_SumZ += f32_DistZ;
                }
            }
        }

        // Buffer 내 점의 갯수가 3 이상이고 높이차의 누적합이 0.3 이상이면 그 점들은 모두 살린다.
        if ((s32_BufferNum >= 3) && (f32_SumZ > 0.3f))
        {
            for (s32_J = 0;s32_J < s32_BufferNum;s32_J++)
            {
                pst_LidarPointData->arst_Point[ars32_Buffer[s32_J]].u8_Flag = c_POINT_VALID;
                pst_LidarPointData->arst_Point[ars32_Buffer[s32_J]].u8_GroundFilteringFlag = 1;
            }
        }
        // 현재 점까지의 거리가 60m 이상, Buffer 내 점의 갯수가 2 이상, 누적합이 0.2 이상이면 그 점들은 모두 살린다.
        else if ((pst_CurPoint->f32_Distance > 60.f) && (s32_BufferNum >= 2) && (f32_SumZ > 0.2f))
        {
            for (s32_J = 0;s32_J < s32_BufferNum;s32_J++)
            {
                pst_LidarPointData->arst_Point[ars32_Buffer[s32_J]].u8_Flag = c_POINT_VALID;
                pst_LidarPointData->arst_Point[ars32_Buffer[s32_J]].u8_GroundFilteringFlag = 2;
            }
        }
        s32_I += 1;
    }
}

__global__
void CUDA_LidarGroundFilteringV3(HMC_LIDAR_DATA_t *pst_LidarPointData, int32_t s32_LIDARMode)
{
    // blockldx.x : 0~1023 - OS2, 0~1799 - VLS
    // threadldx.x : 0~127

    // Variables
    int32_t s32_J = 0;
    int32_t s32_BlockIndex = 128 * blockIdx.x;
    int32_t s32_PointIndex = 128 * blockIdx.x + threadIdx.x;
    int32_t s32_NextIndex = 0;

    float32_t f32_X2 = 0.f;
    float32_t f32_Y2 = 0.f;
    float32_t f32_Z2 = 0.f;
    float32_t f32_DistX = 0.f;
    float32_t f32_DistY = 0.f;
    float32_t f32_DistXY = 0.f;
    float32_t f32_DistZ = 0.f;
    float32_t f32_Inclination = 0.f;
    int32_t s32_BufferIndex;

    HMC_POINT_t* pst_Point = pst_LidarPointData->arst_Point;
    HMC_POINT_t* pst_NextPoint;

    __shared__ HMC_POINT_t* pst_CurPoint;
    __shared__ float32_t f32_X1, f32_Y1, f32_Z1;
    __shared__ int32_t ars32_Buffer[128];
    __shared__ int32_t s32_BufferNum;
    __shared__ float32_t f32_SumZ;
    __shared__ int32_t s32_RefPointIndex;

    ars32_Buffer[threadIdx.x] = 0;
    s32_BufferNum = 0;
    f32_SumZ = 0;
    s32_RefPointIndex = 128 * blockIdx.x;
    
    __syncthreads();

    // VALID/INVALID는 넘어가면서 128 넘어가지 않도록 방지
    
    while(s32_RefPointIndex < s32_BlockIndex + 128)
    {
        if(threadIdx.x == 0)
        {
            while(pst_Point[s32_RefPointIndex].u8_Flag == c_POINT_INVALID && s32_RefPointIndex < s32_BlockIndex + 128)
            {
                s32_RefPointIndex++;
            }

            pst_CurPoint = &pst_LidarPointData->arst_Point[s32_RefPointIndex];

            f32_X1 = pst_CurPoint->f32_X;
            f32_Y1 = pst_CurPoint->f32_Y;
            f32_Z1 = pst_CurPoint->f32_Z;
            s32_BufferNum = 0;
            f32_SumZ = 0.f;
        }

        __syncthreads();

        if(
            (s32_PointIndex > s32_RefPointIndex) &&
            (s32_PointIndex < s32_BlockIndex + 128) &&
            (pst_Point[s32_PointIndex].u8_Flag == c_POINT_GROUND)
        )
        {
            pst_NextPoint = &pst_LidarPointData->arst_Point[s32_PointIndex];

            f32_X2 = pst_NextPoint->f32_X;
            f32_Y2 = pst_NextPoint->f32_Y;
            f32_Z2 = pst_NextPoint->f32_Z;

            f32_DistX = f32_X2 - f32_X1;
            f32_DistY = f32_Y2 - f32_Y1;
            
            f32_DistXY = sqrtf((f32_DistX * f32_DistX) + (f32_DistY * f32_DistY));
            f32_DistZ = f32_Z2 - f32_Z1;
            f32_Inclination = f32_DistZ / f32_DistXY;
            
            // 다음 점과의 거리가 0.5 이하(섹터 별로 지면제거 가능), 현재 점 보다 다음 점의 높이가 더 높고 현재점~다음점의 높이차가 0.1 이상
            // 이때는 Buffer에 그 다음 점들의 Index를 모두 담고 높이차를 누적합한다.
            if(s32_LIDARMode == c_HMC_MODE_OS2)
            {
                if (
                    (f32_DistXY < 0.5f) &&
                    (f32_DistZ > 0.1f) &&
                    (f32_Inclination > 1.2f) // Bank Removal Rule
                )
                {
                    s32_BufferIndex = atomicAdd(&s32_BufferNum, 1);
                    ars32_Buffer[s32_BufferIndex] = s32_PointIndex;
                    atomicAdd(&f32_SumZ, f32_DistZ);
                }
                else if(
                    pst_CurPoint->f32_Distance > 60.f &&
                    f32_DistXY < 1.5f &&
                    (f32_Inclination > 1.f)
                )
                {
                    s32_BufferIndex = atomicAdd(&s32_BufferNum, 1);
                    ars32_Buffer[s32_BufferIndex] = s32_PointIndex;
                    atomicAdd(&f32_SumZ, f32_DistZ);
                }
            }
            else
            {
                if (
                    (f32_DistXY < 0.5f) &&
                    (f32_Inclination > 1.2f) // Bank Removal Rule
                )
                {
                    s32_BufferIndex = atomicAdd(&s32_BufferNum, 1);
                    ars32_Buffer[s32_BufferIndex] = s32_PointIndex;
                    atomicAdd(&f32_SumZ, f32_DistZ);
                }
                else if(
                    pst_CurPoint->f32_Distance > 60.f &&
                    f32_DistXY < 1.5f &&
                    (f32_Inclination > 1.f)
                )
                {
                    s32_BufferIndex = atomicAdd(&s32_BufferNum, 1);
                    ars32_Buffer[s32_BufferIndex] = s32_PointIndex;
                    atomicAdd(&f32_SumZ, f32_DistZ);
                }
            }
        }

        __syncthreads();

        // Buffer 내 점의 갯수가 3 이상이고 높이차의 누적합이 0.3 이상이면 그 점들은 모두 살린다.

        if(threadIdx.x < s32_BufferNum)
        {
            // printf("%d %d After\n", threadIdx.x, s32_BufferNum);
            if ((s32_BufferNum >= 3) && (f32_SumZ > 0.3f))
            {
                pst_Point[ars32_Buffer[threadIdx.x]].u8_Flag = c_POINT_VALID;
            }
            // 현재 점까지의 거리가 60m 이상, Buffer 내 점의 갯수가 2 이상, 누적합이 0.2 이상이면 그 점들은 모두 살린다.
            else if ((pst_CurPoint->f32_Distance > 60.f) && (s32_BufferNum >= 2) && (f32_SumZ > 0.2f))
            {
                pst_Point[ars32_Buffer[threadIdx.x]].u8_Flag = c_POINT_VALID;
            }
        }

        if(threadIdx.x == 0) s32_RefPointIndex++;

        __syncthreads();
    }
}

__device__
int32_t CUDA_GetVoxelIndex(HMC_LIDAR_DATA_t* pst_LidarPointData, int32_t s32_Index)
{
    int32_t s32_VoxelAzimuthIndex = 0;
    int32_t s32_VoxelDistanceIndex = 0;
    int32_t s32_VoxelZIndex = 0;
    int32_t s32_VoxelIndex = 0;

    float32_t f32_Z = 0.f;
    float32_t f32_Distance = 0.f;
    float32_t f32_Azimuth = 0.f;

    f32_Z = pst_LidarPointData->arst_Point[s32_Index].f32_Z;
    f32_Distance = pst_LidarPointData->arst_Point[s32_Index].f32_Distance;
    f32_Azimuth = pst_LidarPointData->arst_Point[s32_Index].f32_Azimuth_deg;

    if (f32_Distance > 100.f || f32_Distance < 0.5f || f32_Z > 1.f || f32_Z < -3.f)
    {
        return -1;
    }

    s32_VoxelAzimuthIndex = (int32_t)(f32_Azimuth / c_HMC_VOXEL_STEP_AZIMUTH);
    s32_VoxelZIndex = (int32_t)((f32_Z + 3.f) / c_HMC_VOXEL_STEP_Z);
    s32_VoxelDistanceIndex = (int32_t)(f32_Distance / c_HMC_VOXEL_STEP_R);

    s32_VoxelIndex = (s32_VoxelDistanceIndex * c_HMC_VOXEL_SIZE_AZIMUTH + s32_VoxelAzimuthIndex) * c_HMC_VOXEL_SIZE_Z + s32_VoxelZIndex;

    if (s32_VoxelIndex >= c_HMC_VOXEL_SIZE)
    {
        return -1;
    }
    return s32_VoxelIndex;
}

__global__
void CUDA_LidarVoxelInit(HMC_LIDAR_DATA_t* pst_LidarPointData, int32_t s32_LIDARMode)
{
    // dim3 st_VoxelBlock(c_HMC_VOXEL_SIZE_AZIMUTH, c_HMC_VOXEL_SIZE_R);
    // dim3 st_VoxelThread(8, c_HMC_VOXEL_SIZE_Z);
    // blockIdx.x : 0~150, blockIdx.y : 0~100
    // threadIdx.x : 0~8, threadIdx.y : 0~8

    int32_t s32_VoxelIndex = 0;
    int32_t s32_VoxelBaseIndex = 0;
    int32_t s32_DistanceIndex = 0;

    float32_t f32_Azimuth = 0;
    float32_t f32_Distance = 0;
    float32_t f32_X = 0;
    float32_t f32_Y = 0;
    float32_t f32_Z = 0;

    s32_VoxelIndex = (blockIdx.y * c_HMC_VOXEL_SIZE_AZIMUTH + blockIdx.x) * c_HMC_VOXEL_SIZE_Z + threadIdx.y;
    s32_VoxelBaseIndex = threadIdx.x;

    s32_DistanceIndex = (int32_t)((int32_t)(s32_VoxelIndex / c_HMC_VOXEL_SIZE_Z) / c_HMC_VOXEL_SIZE_AZIMUTH);
    f32_Z = (s32_VoxelIndex % c_HMC_VOXEL_SIZE_Z) * c_HMC_VOXEL_STEP_Z;
    f32_Azimuth = ((int32_t)(s32_VoxelIndex / c_HMC_VOXEL_SIZE_Z) % c_HMC_VOXEL_SIZE_AZIMUTH) * c_HMC_VOXEL_STEP_AZIMUTH;

    if (s32_VoxelBaseIndex % 2 == 1)
    {
        f32_Distance = (float32_t)s32_DistanceIndex + c_HMC_VOXEL_STEP_R;
    }
    else
    {
        f32_Distance = (float32_t)s32_DistanceIndex;
    }

    if (s32_VoxelBaseIndex >= 4)
    {
        f32_Z += c_HMC_VOXEL_STEP_Z;
    }

    if (s32_VoxelBaseIndex & 2)
    {
        f32_Azimuth += c_HMC_VOXEL_STEP_AZIMUTH;
    }

    if(s32_LIDARMode == c_HMC_MODE_OS2)
    {
        f32_X = f32_Distance * cos(CUDA_deg2rad(f32_Azimuth));
        f32_Y = f32_Distance * sin(CUDA_deg2rad(f32_Azimuth));
    }
    else
    {
        f32_X = f32_Distance * sin(CUDA_deg2rad(f32_Azimuth));
        f32_Y = f32_Distance * cos(CUDA_deg2rad(f32_Azimuth));
    }

    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].arf32_VoxelBaseX[s32_VoxelBaseIndex] = f32_X;
    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].arf32_VoxelBaseY[s32_VoxelBaseIndex] = f32_Y;
    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].arf32_VoxelBaseZ[s32_VoxelBaseIndex] = f32_Z - 3.f;
    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].arb_PointUpdate[s32_VoxelBaseIndex] = false;

    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].b_HasPoint = false;
    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].s32_ClusterID = -1;
    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].s32_PointNum = 0;
    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].s32_Parent = s32_VoxelIndex;
    

}

__device__ bool isPointInsideVoxel(float32_t f32_X, float32_t f32_Y,  HMC_VOXEL_t* pst_Voxel)
{
    float32_t f32_MinX = fminf(fminf(pst_Voxel->arf32_VoxelBaseX[0], pst_Voxel->arf32_VoxelBaseX[1]),
                       fminf(pst_Voxel->arf32_VoxelBaseX[2], pst_Voxel->arf32_VoxelBaseX[3]));
    float32_t f32_MaxX = fmaxf(fmaxf(pst_Voxel->arf32_VoxelBaseX[0], pst_Voxel->arf32_VoxelBaseX[1]),
                       fmaxf(pst_Voxel->arf32_VoxelBaseX[2], pst_Voxel->arf32_VoxelBaseX[3]));

    float32_t f32_MinY = fminf(fminf(pst_Voxel->arf32_VoxelBaseY[0], pst_Voxel->arf32_VoxelBaseY[1]),
                       fminf(pst_Voxel->arf32_VoxelBaseY[2], pst_Voxel->arf32_VoxelBaseY[3]));
    float32_t f32_MaxY = fmaxf(fmaxf(pst_Voxel->arf32_VoxelBaseY[0], pst_Voxel->arf32_VoxelBaseY[1]),
                       fmaxf(pst_Voxel->arf32_VoxelBaseY[2], pst_Voxel->arf32_VoxelBaseY[3]));



    if (f32_X >= f32_MinX && f32_X <= f32_MaxX &&
        f32_Y >= f32_MinY && f32_Y <= f32_MaxY )
    {
        return true;  
    }
    return false; 
}
__global__
void CUDA_LidarVoxelization(HMC_LIDAR_DATA_t* pst_LidarPointData)
{
    // blockIdx.x : 0~1023 - OS2, 0~1799 - VLS 
    // threadIdx.x : 0~127

    int32_t s32_PointIndex = 0;
    int32_t s32_VoxelIndex = 0;

    float32_t f32_X = 0;
    float32_t f32_Y = 0;
    float32_t f32_Z = 0;
    float32_t f32_Distance = 0;

    s32_PointIndex = 128 * blockIdx.x + threadIdx.x;

    f32_X = pst_LidarPointData->arst_Point[s32_PointIndex].f32_X;
    f32_Y = pst_LidarPointData->arst_Point[s32_PointIndex].f32_Y;
    f32_Z = pst_LidarPointData->arst_Point[s32_PointIndex].f32_Z;
    f32_Distance = pst_LidarPointData->arst_Point[s32_PointIndex].f32_Distance;

    if (pst_LidarPointData->arst_Point[s32_PointIndex].u8_Flag == c_POINT_VALID)
    {
        s32_VoxelIndex = CUDA_GetVoxelIndex(pst_LidarPointData, s32_PointIndex);
        if (s32_VoxelIndex >= 0)
        {
            pst_LidarPointData->arst_Point[s32_PointIndex].s32_VoxelIndex = s32_VoxelIndex;

            // 복셀 내에 포인트가 포함되어 있는지 검사
            if (isPointInsideVoxel(f32_X, f32_Y, &pst_LidarPointData->arst_Voxel[s32_VoxelIndex]))
            {
                atomicAdd(&pst_LidarPointData->arst_Voxel[s32_VoxelIndex].s32_PointNum, 1);

                if (f32_Distance <= 10.f && pst_LidarPointData->arst_Voxel[s32_VoxelIndex].s32_PointNum >= 10)
                {
                    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].b_HasPoint = true;
                    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].u8_VoxelColor = 0;
                }
                else if (f32_Distance > 10.f && f32_Distance <= 20.f && pst_LidarPointData->arst_Voxel[s32_VoxelIndex].s32_PointNum >= 6)
                {
                    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].b_HasPoint = true;
                    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].u8_VoxelColor = 1;
                }
                else if (f32_Distance > 20.f && f32_Distance <= 50.f && pst_LidarPointData->arst_Voxel[s32_VoxelIndex].s32_PointNum >= 5)
                {
                    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].b_HasPoint = true;
                    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].u8_VoxelColor = 2;
                }
                else if (f32_Distance > 50.f && f32_Distance <= 80.f && pst_LidarPointData->arst_Voxel[s32_VoxelIndex].s32_PointNum >= 2)
                {
                    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].b_HasPoint = true;
                    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].u8_VoxelColor = 3;
                }
                else if (f32_Distance > 80.f && f32_Distance <= 100.f && pst_LidarPointData->arst_Voxel[s32_VoxelIndex].s32_PointNum >= 1)
                {
                    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].b_HasPoint = true;
                    pst_LidarPointData->arst_Voxel[s32_VoxelIndex].u8_VoxelColor = 4;
                }
            }
            // }
            // else
            // {
            //     pst_LidarPointData->arst_Voxel[s32_VoxelIndex].b_HasPoint = false;
            // }

            // atomicAdd(&pst_LidarPointData->arst_Voxel[s32_VoxelIndex].s32_PointNum, 1);
            // // 포함된 경우, 복셀에 포인트가 있다고 표시
            // pst_LidarPointData->arst_Voxel[s32_VoxelIndex].b_HasPoint = true;
            
            // // f32_Distance에 따라 VoxelColor 설정
            // if (f32_Distance <= 10.f && pst_LidarPointData->arst_Voxel[s32_VoxelIndex].s32_PointNum >= 13)
            // {
            //     pst_LidarPointData->arst_Voxel[s32_VoxelIndex].u8_VoxelColor = 0;
            // }
            // else if (f32_Distance > 10.f && f32_Distance <= 20.f && pst_LidarPointData->arst_Voxel[s32_VoxelIndex].s32_PointNum >= 8)
            // {
            //     pst_LidarPointData->arst_Voxel[s32_VoxelIndex].u8_VoxelColor = 1;
            // }
            // else if (f32_Distance > 20.f && f32_Distance <= 50.f && pst_LidarPointData->arst_Voxel[s32_VoxelIndex].s32_PointNum >= 5)
            // {
            //     pst_LidarPointData->arst_Voxel[s32_VoxelIndex].u8_VoxelColor = 2;
            // }
            // else if (f32_Distance > 50.f && f32_Distance <= 80.f && pst_LidarPointData->arst_Voxel[s32_VoxelIndex].s32_PointNum >= 2)
            // {
            //     pst_LidarPointData->arst_Voxel[s32_VoxelIndex].u8_VoxelColor = 3;
            // }
            // else if (f32_Distance > 80.f && f32_Distance <= 120.f && pst_LidarPointData->arst_Voxel[s32_VoxelIndex].s32_PointNum >= 1)
            // {
            //     pst_LidarPointData->arst_Voxel[s32_VoxelIndex].u8_VoxelColor = 4;
            // }
        }
    }
}

__global__
void CUDA_LidarClusteringInit(HMC_LIDAR_DATA_t* pst_LidarPointData)
{
    int32_t s32_ClusterIdx = 128 * blockIdx.x + threadIdx.x;
    if(s32_ClusterIdx >= 500) return;

    pst_LidarPointData->s32_ClusterNum = 0;

    pst_LidarPointData->arst_Cluster[s32_ClusterIdx].f32_X1 = -9999.f;
    pst_LidarPointData->arst_Cluster[s32_ClusterIdx].f32_Y1 = -9999.f;
    pst_LidarPointData->arst_Cluster[s32_ClusterIdx].f32_Z1 = -9999.f;
    pst_LidarPointData->arst_Cluster[s32_ClusterIdx].f32_X2 = 9999.f;
    pst_LidarPointData->arst_Cluster[s32_ClusterIdx].f32_Y2 = 9999.f;
    pst_LidarPointData->arst_Cluster[s32_ClusterIdx].f32_Z2 = 9999.f;

    pst_LidarPointData->arst_Cluster[s32_ClusterIdx].f32_CenterX = 0.f;
    pst_LidarPointData->arst_Cluster[s32_ClusterIdx].f32_CenterY = 0.f;
    pst_LidarPointData->arst_Cluster[s32_ClusterIdx].f32_Length = 0.f;
    pst_LidarPointData->arst_Cluster[s32_ClusterIdx].f32_Size = 0.f;

    pst_LidarPointData->arst_Cluster[s32_ClusterIdx].s32_PointNum = 0;
    pst_LidarPointData->arst_Cluster[s32_ClusterIdx].s32_VoxelNum = 0;
    pst_LidarPointData->arst_Cluster[s32_ClusterIdx].u8_Flag = 0;
    pst_LidarPointData->arst_Cluster[s32_ClusterIdx].b_TrackingFlag = false;
}

__device__
int32_t CUDA_FindRootVoxel(HMC_VOXEL_t* pst_Voxel, int32_t s32_VoxelIndex, int32_t* ps32_VoxelRootNode)
{
    int32_t s32_NewRoot;

    if (pst_Voxel[s32_VoxelIndex].s32_Parent != s32_VoxelIndex)
    {
        s32_NewRoot = CUDA_FindRootVoxel(pst_Voxel, pst_Voxel[s32_VoxelIndex].s32_Parent, ps32_VoxelRootNode);
        pst_Voxel[s32_VoxelIndex].s32_Parent = s32_NewRoot;
        ps32_VoxelRootNode[s32_VoxelIndex] = s32_NewRoot;
    }

    return pst_Voxel[s32_VoxelIndex].s32_Parent;
}

__device__
void CUDA_UniteVoxel(HMC_VOXEL_t* pst_Voxel, int32_t s32_VoxelIndex1, int32_t s32_VoxelIndex2, int32_t* ps32_VoxelRootNode)
{
    int32_t s32_Root1 = CUDA_FindRootVoxel(pst_Voxel, s32_VoxelIndex1, ps32_VoxelRootNode);
    int32_t s32_Root2 = CUDA_FindRootVoxel(pst_Voxel, s32_VoxelIndex2, ps32_VoxelRootNode);

    if (s32_Root1 != s32_Root2)
    {
        if(s32_Root1 < s32_Root2)
        {
            pst_Voxel[s32_Root2].s32_Parent = s32_Root1;
            ps32_VoxelRootNode[s32_Root2] = s32_Root1;
        }
        else
        {
            pst_Voxel[s32_Root1].s32_Parent = s32_Root2;
            ps32_VoxelRootNode[s32_Root1] = s32_Root2;
        }
    }
}

__global__
void CUDA_LidarVoxelMerge(HMC_LIDAR_DATA_t* pst_LidarPointData, int32_t* ps32_VoxelRootNode)
{
    int32_t s32_I, s32_J, s32_K;
    int32_t s32_DR, s32_DAzi, s32_DZ;
    int32_t s32_PointIndex = 128 * blockIdx.x + threadIdx.x;
    int32_t s32_CurVoxelIndex;
    int32_t s32_NextVoxelIndex;
    int32_t ars32_MoveR[] = { -c_HMC_VOXEL_SIZE_AZIMUTH * c_HMC_VOXEL_SIZE_Z, 0, c_HMC_VOXEL_SIZE_AZIMUTH * c_HMC_VOXEL_SIZE_Z };
    int32_t ars32_MoveAzi[] = { -c_HMC_VOXEL_SIZE_Z, 0, c_HMC_VOXEL_SIZE_Z };
    int32_t ars32_MoveZ[] = { -1, 0, 1 };
    HMC_POINT_t* pst_Point = pst_LidarPointData->arst_Point;
    HMC_VOXEL_t* pst_Voxel = pst_LidarPointData->arst_Voxel;
    HMC_CLUSTER_t* pst_Cluster = pst_LidarPointData->arst_Cluster;

    
    if(pst_Point[s32_PointIndex].u8_Flag != c_POINT_VALID) return;
    
    s32_CurVoxelIndex = pst_Point[s32_PointIndex].s32_VoxelIndex;
    
    if (!(pst_Voxel[s32_CurVoxelIndex].b_HasPoint) || s32_CurVoxelIndex == -1)
    {
        ps32_VoxelRootNode[s32_CurVoxelIndex] = -1;
        pst_Voxel[s32_CurVoxelIndex].s32_Parent = -1;
        return;
    }

    for (s32_I = 0; s32_I < 3; s32_I++)
    {
        s32_DR = ars32_MoveR[s32_I];

        for (s32_J = 0; s32_J < 3; s32_J++)
        {
            s32_DAzi = ars32_MoveAzi[s32_J];

            for (s32_K = 0; s32_K < 3; s32_K++)
            {
                s32_DZ = ars32_MoveZ[s32_K];

                s32_NextVoxelIndex = s32_CurVoxelIndex + s32_DR + s32_DAzi + s32_DZ;

                if (s32_NextVoxelIndex < 0 || s32_NextVoxelIndex >= c_HMC_VOXEL_SIZE || s32_NextVoxelIndex == s32_CurVoxelIndex)
                {
                    continue;
                }

                if (pst_Voxel[s32_NextVoxelIndex].b_HasPoint)
                {
                    CUDA_UniteVoxel(pst_Voxel, s32_CurVoxelIndex, s32_NextVoxelIndex, ps32_VoxelRootNode);
                }
                else
                {
                    ps32_VoxelRootNode[s32_NextVoxelIndex] = -1;
                    pst_Voxel[s32_NextVoxelIndex].s32_Parent = -1;
                }
            }
        }
    }

    __syncthreads();

    // Make Sure To Execute Twice For Properly Merging
    for (s32_I = 0; s32_I < 3; s32_I++)
    {
        s32_DR = ars32_MoveR[s32_I];

        for (s32_J = 0; s32_J < 3; s32_J++)
        {
            s32_DAzi = ars32_MoveAzi[s32_J];

            for (s32_K = 0; s32_K < 3; s32_K++)
            {
                s32_DZ = ars32_MoveZ[s32_K];

                s32_NextVoxelIndex = s32_CurVoxelIndex + s32_DR + s32_DAzi + s32_DZ;

                if (s32_NextVoxelIndex < 0 || s32_NextVoxelIndex >= c_HMC_VOXEL_SIZE || s32_NextVoxelIndex == s32_CurVoxelIndex)
                {
                    continue;
                }

                if (pst_Voxel[s32_NextVoxelIndex].b_HasPoint)
                {
                    CUDA_UniteVoxel(pst_Voxel, s32_CurVoxelIndex, s32_NextVoxelIndex, ps32_VoxelRootNode);
                }
                else
                {
                    ps32_VoxelRootNode[s32_NextVoxelIndex] = -1;
                    pst_Voxel[s32_NextVoxelIndex].s32_Parent = -1;
                }
            }
        }
    }
}

__global__
void CUDA_LidarVoxelAlloc(HMC_LIDAR_DATA_t* pst_LidarPointData, int32_t* ps32_VoxelToCluster)
{
    HMC_POINT_t* pst_Point = pst_LidarPointData->arst_Point;
    HMC_VOXEL_t* pst_Voxel = pst_LidarPointData->arst_Voxel;
    HMC_CLUSTER_t* pst_Cluster = pst_LidarPointData->arst_Cluster;

    int32_t s32_PointIndex = 128 * blockIdx.x + threadIdx.x;
    if(pst_Point[s32_PointIndex].u8_Flag != c_POINT_VALID) return;
    
    int32_t s32_VoxelIndex = pst_Point[s32_PointIndex].s32_VoxelIndex;
    if(s32_VoxelIndex == -1) return;
    else if(!pst_Voxel[s32_VoxelIndex].b_HasPoint) return;

    int32_t s32_ClusterIndex = ps32_VoxelToCluster[s32_VoxelIndex];
    if(s32_ClusterIndex == -1) return;

    pst_Voxel[s32_VoxelIndex].s32_ClusterID = s32_ClusterIndex;
    atomicAdd(&pst_Cluster[s32_ClusterIndex].s32_VoxelNum, 1);
}

__device__
float32_t CUDA_AtomicMaxFloat(float32_t* pf32_DstValue, float32_t f32_SrcValue)
{
    int32_t* ps32_DstValueAsInt = (int*) pf32_DstValue;
    int32_t s32_OldValue = *ps32_DstValueAsInt;
    int32_t s32_AssumedValue;

    do {
        s32_AssumedValue = s32_OldValue;
        s32_OldValue = atomicCAS(ps32_DstValueAsInt, s32_AssumedValue, __float_as_int(fmaxf(f32_SrcValue, __int_as_float(s32_AssumedValue))));
    } while (s32_AssumedValue != s32_OldValue);

    return __int_as_float(s32_OldValue);
}

__device__
float32_t CUDA_AtomicMinFloat(float32_t* pf32_DstValue, float32_t f32_SrcValue)
{
    int32_t* ps32_DstValueAsInt = (int*) pf32_DstValue;
    int32_t s32_OldValue = *ps32_DstValueAsInt;
    int32_t s32_AssumedValue;

    do {
        s32_AssumedValue = s32_OldValue;
        s32_OldValue = atomicCAS(ps32_DstValueAsInt, s32_AssumedValue, __float_as_int(fminf(f32_SrcValue, __int_as_float(s32_AssumedValue))));
    } while (s32_AssumedValue != s32_OldValue);

    return __int_as_float(s32_OldValue);
}

__global__
void CUDA_LidarGetClusterInfo(HMC_LIDAR_DATA_t* pst_LidarPointData)
{
    HMC_POINT_t* pst_Point = pst_LidarPointData->arst_Point;
    HMC_VOXEL_t* pst_Voxel = pst_LidarPointData->arst_Voxel;
    HMC_CLUSTER_t* pst_Cluster = pst_LidarPointData->arst_Cluster;

    int32_t s32_PointIdx = 128 * blockIdx.x + threadIdx.x;
    int32_t s32_VoxelIdx = pst_LidarPointData->arst_Point[s32_PointIdx].s32_VoxelIndex;
    int32_t s32_ClusterIdx;
    int32_t s32_ClusterPos;

    float32_t f32_X = 0.f;
    float32_t f32_Y = 0.f;
    float32_t f32_Z = 0.f;

    if (
        pst_Point[s32_PointIdx].u8_Flag != c_POINT_VALID ||
        s32_VoxelIdx == -1
    )
    {
        return;
    }

    s32_ClusterIdx = pst_Voxel[s32_VoxelIdx].s32_ClusterID;
    
    if(s32_ClusterIdx == -1) return;

    f32_X = pst_Point[s32_PointIdx].f32_X;
    f32_Y = pst_Point[s32_PointIdx].f32_Y;
    f32_Z = pst_Point[s32_PointIdx].f32_Z;
    
    if(pst_Cluster[s32_ClusterIdx].s32_PointNum < c_HMC_CLUSTER_MAX_POINT_NUM)
    {
        atomicAdd(&(pst_Cluster[s32_ClusterIdx].s32_PointNum), 1);
    }

    CUDA_AtomicMaxFloat(&(pst_Cluster[s32_ClusterIdx].f32_X1), f32_X);
    CUDA_AtomicMinFloat(&(pst_Cluster[s32_ClusterIdx].f32_X2), f32_X);

    CUDA_AtomicMaxFloat(&(pst_Cluster[s32_ClusterIdx].f32_Y1), f32_Y);
    CUDA_AtomicMinFloat(&(pst_Cluster[s32_ClusterIdx].f32_Y2), f32_Y);

    CUDA_AtomicMaxFloat(&(pst_Cluster[s32_ClusterIdx].f32_Z1), f32_Z);
    CUDA_AtomicMinFloat(&(pst_Cluster[s32_ClusterIdx].f32_Z2), f32_Z);

    __syncthreads();

    pst_Cluster[s32_ClusterIdx].f32_CenterX = (pst_Cluster[s32_ClusterIdx].f32_X1 + pst_Cluster[s32_ClusterIdx].f32_X2) / 2.f;
    pst_Cluster[s32_ClusterIdx].f32_CenterY = (pst_Cluster[s32_ClusterIdx].f32_Y1 + pst_Cluster[s32_ClusterIdx].f32_Y2) / 2.f;

    pst_Cluster[s32_ClusterIdx].f32_Length = CUDA_getDistance2d(pst_Cluster[s32_ClusterIdx].f32_X1, pst_Cluster[s32_ClusterIdx].f32_Y1, pst_Cluster[s32_ClusterIdx].f32_X2, pst_Cluster[s32_ClusterIdx].f32_Y2);
    pst_Cluster[s32_ClusterIdx].f32_Size = fabsf(pst_Cluster[s32_ClusterIdx].f32_X1 - pst_Cluster[s32_ClusterIdx].f32_X2) *
                                            fabsf(pst_Cluster[s32_ClusterIdx].f32_Y1 - pst_Cluster[s32_ClusterIdx].f32_Y2) *
                                            fabsf(pst_Cluster[s32_ClusterIdx].f32_Z1 - pst_Cluster[s32_ClusterIdx].f32_Z2);

    if (pst_Cluster[s32_ClusterIdx].s32_PointNum < 5 ||
        pst_Cluster[s32_ClusterIdx].f32_Length > 10.f)
    {
        pst_Cluster[s32_ClusterIdx].u8_Flag = c_HMC_CLUSTER_NONE;
        pst_Cluster[s32_ClusterIdx].b_TrackingFlag = false;
    }
    else
    {
        pst_Cluster[s32_ClusterIdx].u8_Flag = c_HMC_CLUSTER_VEHICLE;
        pst_Cluster[s32_ClusterIdx].b_TrackingFlag = true;
    }
}

void CUDA_LidarClustering(HMC_LIDAR_DATA_t* pst_LidarPointData_device, int32_t s32_LIDARMode)
{
    dim3 st_ParseBlock, st_ParseThread, st_Block, st_Thread;
    dim3 st_ClusterInitBlock(4);
    dim3 st_ClusterInitThread(128);
    int32_t s32_I, s32_J;
    cudaError_t st_Error;
    
    int32_t ars32_VoxelRootNode_host[c_HMC_VOXEL_SIZE];
    int32_t ars32_VoxelToCluster_host[c_HMC_VOXEL_SIZE];
    int32_t* ps32_VoxelRootNode_device = nullptr;
    int32_t* ps32_VoxelToCluster_device = nullptr;
    std::set<int32_t> st_VoxelRootSet;
    int32_t s32_ClusterNum;
    
    if(s32_LIDARMode == c_HMC_MODE_OS2)
    {
        st_ParseBlock = dim3(c_HMC_CUDA_BLOCK_OS2);
        st_ParseThread = dim3(c_HMC_CUDA_THREAD_OS2);
        st_Block = dim3(c_HMC_CUDA_BLOCK_OS2);
        st_Thread = dim3(c_HMC_CUDA_THREAD_OS2);
    }
    else // VLS128
    {
        st_ParseBlock = dim3(c_HMC_CUDA_PARSE_BLOCK_VLS);
        st_ParseThread = dim3(c_HMC_CUDA_PARSE_THREAD_VLS);
        st_Block = dim3(c_HMC_CUDA_BLOCK_VLS);
        st_Thread = dim3(c_HMC_CUDA_THREAD_VLS);
    }

    cudaMalloc((void**)&ps32_VoxelRootNode_device, sizeof(int32_t) * c_HMC_VOXEL_SIZE);
    cudaMalloc((void**)&ps32_VoxelToCluster_device, sizeof(int32_t) * c_HMC_VOXEL_SIZE);
    cudaMemset(ps32_VoxelRootNode_device, -1, sizeof(int32_t) * c_HMC_VOXEL_SIZE);
    cudaMemset(ps32_VoxelToCluster_device, -1, sizeof(int32_t) * c_HMC_VOXEL_SIZE);

    CUDA_LidarClusteringInit << < st_ClusterInitBlock, st_ClusterInitThread >> > (pst_LidarPointData_device);
    cudaDeviceSynchronize();

    CUDA_LidarVoxelMerge << < st_Block, st_Thread >> > (pst_LidarPointData_device, ps32_VoxelRootNode_device);
    cudaDeviceSynchronize();

    cudaMemcpy(ars32_VoxelRootNode_host, ps32_VoxelRootNode_device, c_HMC_VOXEL_SIZE * sizeof(int32_t), cudaMemcpyDeviceToHost);
    
    for (s32_I = 0; s32_I < c_HMC_VOXEL_SIZE; s32_I++)
    {
        if (ars32_VoxelRootNode_host[s32_I] != -1)
        {
            st_VoxelRootSet.insert({ars32_VoxelRootNode_host[s32_I]});
        }
    }

    s32_ClusterNum = st_VoxelRootSet.size();

    for (s32_I = 0; s32_I < c_HMC_VOXEL_SIZE; s32_I++)
    {
        ars32_VoxelToCluster_host[s32_I] = -1;
        if (ars32_VoxelRootNode_host[s32_I] != -1)
        {
            ars32_VoxelToCluster_host[s32_I] = std::distance(st_VoxelRootSet.begin(), st_VoxelRootSet.find(ars32_VoxelRootNode_host[s32_I]));
        }
    }

    cudaMalloc((void**)&ps32_VoxelToCluster_device, c_HMC_VOXEL_SIZE * sizeof(int32_t));
    cudaMemcpy(ps32_VoxelToCluster_device, ars32_VoxelToCluster_host, c_HMC_VOXEL_SIZE * sizeof(int32_t), cudaMemcpyHostToDevice);
    cudaMemcpy(&(pst_LidarPointData_device->s32_ClusterNum), &s32_ClusterNum, sizeof(int32_t), cudaMemcpyHostToDevice);

    CUDA_LidarVoxelAlloc << < st_Block, st_Thread >> > (pst_LidarPointData_device, ps32_VoxelToCluster_device);
    cudaDeviceSynchronize();


    CUDA_LidarGetClusterInfo << < st_Block, st_Thread >> > (pst_LidarPointData_device);
    cudaDeviceSynchronize();

    cudaFree(ps32_VoxelRootNode_device);
    cudaFree(ps32_VoxelToCluster_device);
}

char* pc_Buffer_device = nullptr;
HMC_LIDAR_DATA_t* pst_LidarPointData_device = nullptr;
uint8_t u8_CudaInit = 1;

void CUDA_LidarPreprocessing(char* pc_Buffer, HMC_LIDAR_DATA_t* pst_LidarPointData, float32_t f32_Roll, float32_t f32_Pitch, int32_t s32_LIDARMode)
{
    float32_t f32_Time;
    cudaEvent_t a, b;
    cudaError_t st_Err;

    dim3 st_ParseBlock, st_ParseThread, st_Block, st_Thread;
    dim3 st_VoxelBlock(c_HMC_VOXEL_SIZE_AZIMUTH, c_HMC_VOXEL_SIZE_R);
    dim3 st_VoxelThread(8, c_HMC_VOXEL_SIZE_Z);

    int32_t s32_PacketSize;
    memcpy(&s32_PacketSize, pc_Buffer, sizeof(int32_t));

    if(s32_LIDARMode == c_HMC_MODE_OS2)
    {
        st_ParseBlock = dim3(c_HMC_CUDA_BLOCK_OS2);
        st_ParseThread = dim3(c_HMC_CUDA_THREAD_OS2);
        st_Block = dim3(c_HMC_CUDA_BLOCK_OS2);
        st_Thread = dim3(c_HMC_CUDA_THREAD_OS2);
    }
    else // VLS128
    {
        st_ParseBlock = dim3(c_HMC_CUDA_PARSE_BLOCK_VLS);
        st_ParseThread = dim3(c_HMC_CUDA_PARSE_THREAD_VLS);
        st_Block = dim3(c_HMC_CUDA_BLOCK_VLS);
        st_Thread = dim3(c_HMC_CUDA_THREAD_VLS);
    }

    if (s32_PacketSize == 0)
    {
        return;
    }
    
    // MORAI Hot Fix
    // 1. If문만 주석처리하고 내부 cudaMalloc 구문 그대로 두기
    //if (u8_CudaInit)
    //{
        cudaMalloc((void**)&pc_Buffer_device, sizeof(char) * s32_PacketSize * 2);
        cudaMalloc((void**)&pst_LidarPointData_device, sizeof(HMC_LIDAR_DATA_t));
        u8_CudaInit = 0;
    //}

    cudaMemcpy(pc_Buffer_device, pc_Buffer + 4, sizeof(char) * s32_PacketSize, cudaMemcpyHostToDevice);


    cudaMemcpy(pc_Buffer_device, pc_Buffer + 4, sizeof(char) * s32_PacketSize, cudaMemcpyHostToDevice);
    
    if(s32_LIDARMode == c_HMC_MODE_OS2)
    {
        CUDA_LidarParsing << < st_ParseBlock, st_ParseThread >> > (pc_Buffer_device, pst_LidarPointData_device, f32_Roll, f32_Pitch);
        cudaDeviceSynchronize();
    }
    else
    {
        CUDA_LidarParsing_VLS128 << < st_ParseBlock, st_ParseThread >> > (pc_Buffer_device, pst_LidarPointData_device, f32_Roll, f32_Pitch);
        cudaDeviceSynchronize();
    }
   
    // cudaEventCreate(&a);    
    // cudaEventCreate(&b);

    // cudaEventRecord(a);
    CUDA_LidarGroundFilteringV2 << < st_Block, st_Thread >> > (pst_LidarPointData_device, s32_LIDARMode);
    cudaDeviceSynchronize();
    // cudaEventRecord(b);
    // cudaEventSynchronize(b);
    // cudaEventElapsedTime(&f32_Time, a, b);
    // printf("Ground : %f\n", f32_Time);

    // cudaEventRecord(a);
    CUDA_LidarVoxelInit << < st_VoxelBlock, st_VoxelThread >> > (pst_LidarPointData_device, s32_LIDARMode);
    cudaDeviceSynchronize();

    CUDA_LidarVoxelization << < st_Block, st_Thread >> > (pst_LidarPointData_device);
    cudaDeviceSynchronize();
    // cudaEventRecord(b);
    // cudaEventSynchronize(b);
    // cudaEventElapsedTime(&f32_Time, a, b);
    // printf("Voxel : %f\n", f32_Time);

    // cudaEventRecord(a);
    // CUDA_LidarClustering(pst_LidarPointData_device, s32_LIDARMode);
    // cudaEventRecord(b);
    // cudaEventSynchronize(b);
    // cudaEventElapsedTime(&f32_Time, a, b);
    // printf("Clustering : %f\n", f32_Time);

    // cudaEventRecord(a);
    cudaMemcpy(pst_LidarPointData, pst_LidarPointData_device, sizeof(HMC_LIDAR_DATA_t), cudaMemcpyDeviceToHost);
    // cudaEventRecord(b);
    // cudaEventSynchronize(b);
    // cudaEventElapsedTime(&f32_Time, a, b);
    // printf("Device To Host : %f\n", f32_Time);

    // MORAI Hot Fix
    // 2. 아래 cudaFree구문 주석 해제
     cudaFree(pc_Buffer_device);
     cudaFree(pst_LidarPointData_device);

}



__global__ void CUDA_LaneRotation(float32_t f32_X, float32_t f32_Y, 
                                float32_t *pf32_CandX, float32_t *pf32_CandY, 
                                float32_t *pf32_LaneX, float32_t *pf32_LaneY, 
                                int32_t *ps32_Result, float32_t *pf32_Result)
{
    int32_t s32_Cand = threadIdx.x;
    int32_t s32_Lane = blockIdx.x;
    int32_t s32_I = 0;
    float32_t f32_TX = pf32_LaneX[s32_Lane] - f32_X;
    float32_t f32_TY = pf32_LaneY[s32_Lane] - f32_Y;
    float32_t f32_RX, f32_RY;
    float32_t f32_Dist = 0.f;

    for (s32_I = 0;s32_I < 720;s32_I++)
    {
        f32_RX = arf32_Cos[s32_I] * f32_TX + arf32_Sin[s32_I] * f32_TY;
        f32_RY = -arf32_Sin[s32_I] * f32_TX + arf32_Cos[s32_I] * f32_TY;

        f32_Dist = sqrtf(powf(f32_RX - pf32_CandX[s32_Cand], 2.f) + powf(f32_RY - pf32_CandY[s32_Cand], 2.f));

        if (f32_Dist < 1.f)
        {
            atomicAdd(&ps32_Result[s32_I], 1);
            atomicAdd(&pf32_Result[s32_I], f32_Dist);
        }
    }
}



void CUDA_LaneICP(float32_t f32_X, float32_t f32_Y, 
                    float32_t *pf32_CandX, float32_t *pf32_CandY, int32_t s32_CandNum, 
                    float32_t *pf32_LaneX, float32_t *pf32_LaneY, int32_t s32_LaneNum, 
                    float32_t &f32_Result_deg)
{
    int32_t s32_I;
    int32_t s32_Result;
    int32_t s32_Max = -1;
    int32_t f32_Min = 99999999.f;
    float32_t* pf32_CandX_device;
    float32_t* pf32_CandY_device;
    float32_t* pf32_LaneX_device;
    float32_t* pf32_LaneY_device;
    int32_t*   ps32_Result_device;
    float32_t* pf32_Result_device;

    int32_t   ars32_Result[720];
    float32_t arf32_Result[720];
    cudaError_t st_Err;

    cudaMalloc((void**)&pf32_CandX_device, sizeof(float32_t) * s32_CandNum);
    cudaMalloc((void**)&pf32_CandY_device, sizeof(float32_t) * s32_CandNum);
    cudaMalloc((void**)&pf32_LaneX_device, sizeof(float32_t) * s32_LaneNum);
    cudaMalloc((void**)&pf32_LaneY_device, sizeof(float32_t) * s32_LaneNum);
    cudaMalloc((void**)&ps32_Result_device, sizeof(int32_t) * 720);
    cudaMalloc((void**)&pf32_Result_device, sizeof(float32_t) * 720);
    st_Err = cudaGetLastError();
    checkCudaError(st_Err, "Lane Rotation failed for CUDA_LidarParsing");

    cudaMemcpy(pf32_CandX_device, pf32_CandX, sizeof(float32_t) * s32_CandNum, cudaMemcpyHostToDevice);
    cudaMemcpy(pf32_CandY_device, pf32_CandY, sizeof(float32_t) * s32_CandNum, cudaMemcpyHostToDevice);
    cudaMemcpy(pf32_LaneX_device, pf32_LaneX, sizeof(float32_t) * s32_LaneNum, cudaMemcpyHostToDevice);
    cudaMemcpy(pf32_LaneY_device, pf32_LaneY, sizeof(float32_t) * s32_LaneNum, cudaMemcpyHostToDevice);
    st_Err = cudaGetLastError();
    checkCudaError(st_Err, "Lane Rotation failed for CUDA_LidarParsing");

    CUDA_LaneRotation << < s32_LaneNum, s32_CandNum >> > (f32_X, f32_Y, 
                                                            pf32_CandX_device, pf32_CandY_device,
                                                            pf32_LaneX_device, pf32_LaneY_device,
                                                            ps32_Result_device, pf32_Result_device);
    cudaDeviceSynchronize();
    st_Err = cudaGetLastError();
    checkCudaError(st_Err, "Lane Rotation failed for CUDA_LidarParsing");

    cudaMemcpy(ars32_Result, ps32_Result_device, sizeof(int32_t) * 720, cudaMemcpyDeviceToHost);
    cudaMemcpy(arf32_Result, pf32_Result_device, sizeof(float32_t) * 720, cudaMemcpyDeviceToHost);

    for (s32_I = 0;s32_I < 720;s32_I++)
    {
        if (ars32_Result[s32_I] > s32_Max)
        {
            s32_Max = ars32_Result[s32_I];
            f32_Min = arf32_Result[s32_I];
            s32_Result = s32_I;
        }
        else if (ars32_Result[s32_I] == s32_Max)
        {
            if (arf32_Result[s32_I] < f32_Min)
            {
                f32_Min = arf32_Result[s32_I];
                s32_Result = s32_I;
            }

        }

        // printf("%d %d %f\n", s32_I, ars32_Result[s32_I], arf32_Result[s32_I]);
    }

    f32_Result_deg = (float32_t)(s32_Result) / 2.f;
    // printf("%f %d %f\n", f32_Result_deg, s32_Max, f32_Min);

    cudaFree(pf32_CandX_device);
    cudaFree(pf32_CandY_device);
    cudaFree(pf32_LaneX_device);
    cudaFree(pf32_LaneY_device);
    cudaFree(ps32_Result_device);
    cudaFree(pf32_Result_device);
}



__global__ void CUDA_FindNearestNeighbors(ICP_POINT_CLOUD_t *pst_Source, ICP_POINT_CLOUD_t *pst_Target, int32_t *ps32_NearestIdx, float32_t f32_Threshold) 
{
    
    int32_t s32_I;
    int32_t s32_Idx = blockIdx.x * blockDim.x + threadIdx.x;
    int32_t s32_NearestIdx = -1;
    float32_t f32_Dist;
    float32_t f32_MinDist = f32_Threshold;
    float32_t f32_X, f32_Y, f32_Z;
    float32_t f32_TX, f32_TY, f32_TZ;

    if (s32_Idx >= pst_Source->s32_PointNum) 
    {
        return;
    }

    f32_X = pst_Source->arf32_X[s32_Idx];
    f32_Y = pst_Source->arf32_Y[s32_Idx];
    f32_Z = pst_Source->arf32_Z[s32_Idx];

    for (s32_I = 0; s32_I < pst_Target->s32_PointNum;s32_I++) 
    {
        f32_TX = pst_Target->arf32_X[s32_I];
        f32_TY = pst_Target->arf32_Y[s32_I];
        f32_TZ = pst_Target->arf32_Z[s32_I];

        f32_Dist = sqrtf(powf(f32_X - f32_TX, 2.f) + powf(f32_Y - f32_TY, 2.f) + powf(f32_Z - f32_TZ, 2.f));

        if (f32_MinDist > f32_Dist) {
            f32_MinDist = f32_Dist;
            s32_NearestIdx = s32_I;
        }
    }

    ps32_NearestIdx[s32_Idx] = s32_NearestIdx;
}

__global__ void CUDA_ComputeCovarianceMatrix(ICP_POINT_CLOUD_t *pst_Source, ICP_POINT_CLOUD_t *pst_Target, int32_t *ps32_NearestIdx, 
                                        float32_t* pf32_H, float32_t *pf32_SourceCentroid, float32_t *pf32_TargetCentroid) 
{
    __shared__ float32_t arf32_Shared_H[9];

    // Initialize shared memory for covariance matrix
    if (threadIdx.x < 9) 
    {
        arf32_Shared_H[threadIdx.x] = 0.f;
    }
    __syncthreads();

    int32_t s32_I = blockIdx.x * blockDim.x + threadIdx.x;
    int32_t s32_J = ps32_NearestIdx[s32_I];

    if (s32_I < pst_Source->s32_PointNum && s32_J != -1) 
    {
        float32_t f32_AX = pst_Source->arf32_X[s32_I] - pf32_SourceCentroid[0];
        float32_t f32_AY = pst_Source->arf32_Y[s32_I] - pf32_SourceCentroid[1];
        float32_t f32_AZ = pst_Source->arf32_Z[s32_I] - pf32_SourceCentroid[2];
        float32_t f32_BX = pst_Target->arf32_X[s32_J] - pf32_TargetCentroid[0];
        float32_t f32_BY = pst_Target->arf32_Y[s32_J] - pf32_TargetCentroid[1];
        float32_t f32_BZ = pst_Target->arf32_Z[s32_J] - pf32_TargetCentroid[2];

        atomicAdd(&arf32_Shared_H[0], f32_AX * f32_BX);
        atomicAdd(&arf32_Shared_H[1], f32_AX * f32_BY);
        atomicAdd(&arf32_Shared_H[2], f32_AX * f32_BZ);
        atomicAdd(&arf32_Shared_H[3], f32_AY * f32_BX);
        atomicAdd(&arf32_Shared_H[4], f32_AY * f32_BY);
        atomicAdd(&arf32_Shared_H[5], f32_AY * f32_BZ);
        atomicAdd(&arf32_Shared_H[6], f32_AZ * f32_BX);
        atomicAdd(&arf32_Shared_H[7], f32_AZ * f32_BY);
        atomicAdd(&arf32_Shared_H[8], f32_AZ * f32_BZ);
    }
    __syncthreads();

    // Write results to global memory
    if (threadIdx.x < 9) 
    {
        atomicAdd(&pf32_H[threadIdx.x], arf32_Shared_H[threadIdx.x]);
    }
}


__global__ void CUDA_ComputeCovarianceMatrix(ICP_POINT_CLOUD_t *pst_Source, float3 *pst_Target, int32_t *ps32_NearestIdx, 
                                        float32_t* pf32_H, float32_t *pf32_SourceCentroid, float32_t *pf32_TargetCentroid) 
{
    __shared__ float32_t arf32_Shared_H[9];

    // Initialize shared memory for covariance matrix
    if (threadIdx.x < 9) 
    {
        arf32_Shared_H[threadIdx.x] = 0.f;
    }
    __syncthreads();

    int32_t s32_I = blockIdx.x * blockDim.x + threadIdx.x;
    int32_t s32_J = ps32_NearestIdx[s32_I];

    if (s32_I < pst_Source->s32_PointNum && s32_J != -1) 
    {
        float32_t f32_AX = pst_Source->arf32_X[s32_I] - pf32_SourceCentroid[0];
        float32_t f32_AY = pst_Source->arf32_Y[s32_I] - pf32_SourceCentroid[1];
        float32_t f32_AZ = pst_Source->arf32_Z[s32_I] - pf32_SourceCentroid[2];
        float32_t f32_BX = pst_Target[s32_J].x - pf32_TargetCentroid[0];
        float32_t f32_BY = pst_Target[s32_J].y - pf32_TargetCentroid[1];
        float32_t f32_BZ = pst_Target[s32_J].z - pf32_TargetCentroid[2];

        atomicAdd(&arf32_Shared_H[0], f32_AX * f32_BX);
        atomicAdd(&arf32_Shared_H[1], f32_AX * f32_BY);
        atomicAdd(&arf32_Shared_H[2], f32_AX * f32_BZ);
        atomicAdd(&arf32_Shared_H[3], f32_AY * f32_BX);
        atomicAdd(&arf32_Shared_H[4], f32_AY * f32_BY);
        atomicAdd(&arf32_Shared_H[5], f32_AY * f32_BZ);
        atomicAdd(&arf32_Shared_H[6], f32_AZ * f32_BX);
        atomicAdd(&arf32_Shared_H[7], f32_AZ * f32_BY);
        atomicAdd(&arf32_Shared_H[8], f32_AZ * f32_BZ);
    }
    __syncthreads();

    // Write results to global memory
    if (threadIdx.x < 9) 
    {
        atomicAdd(&pf32_H[threadIdx.x], arf32_Shared_H[threadIdx.x]);
    }
}



__global__ void CUDA_TransformPoints(ICP_POINT_CLOUD_t* pst_Source, float32_t* pf32_Transform) 
{
    int32_t s32_I = blockIdx.x * blockDim.x + threadIdx.x;

    if (s32_I >= pst_Source->s32_PointNum)
    {
        return;
    }

    float32_t f32_X = pst_Source->arf32_X[s32_I];
    float32_t f32_Y = pst_Source->arf32_Y[s32_I];
    float32_t f32_Z = pst_Source->arf32_Z[s32_I];

    pst_Source->arf32_X[s32_I] = pf32_Transform[0] * f32_X + pf32_Transform[1] * f32_Y + pf32_Transform[2] * f32_Z + pf32_Transform[3];
    pst_Source->arf32_Y[s32_I] = pf32_Transform[4] * f32_X + pf32_Transform[5] * f32_Y + pf32_Transform[6] * f32_Z + pf32_Transform[7];
    pst_Source->arf32_Z[s32_I] = pf32_Transform[8] * f32_X + pf32_Transform[9] * f32_Y + pf32_Transform[10] * f32_Z + pf32_Transform[11];
}





void CUDA_ICP(ICP_POINT_CLOUD_t *pst_Source, ICP_POINT_CLOUD_t *pst_Target, int32_t s32_Iteration, int32_t *ps32_NearestIdx, float32_t *pf32_Result)
{
    int32_t s32_I = 0, s32_J = 0, s32_K = 0, s32_Idx = 0;
    float32_t f32_X1, f32_Y1, f32_Z1;
    float32_t f32_X2, f32_Y2, f32_Z2;
    ICP_POINT_CLOUD_t *pst_Source_pinned;
    ICP_POINT_CLOUD_t *pst_Target_pinned;
    int32_t *ps32_NearestIdx_pinned;
    float32_t f32_Threshold = 30.f;
    float32_t *pf32_Transform_device;
    int32_t s32_CorresNum;
    int32_t s32_SourceNum;
    int32_t s32_TargetNum;
    float32_t *pf32_SourceCentroid_pinned;
    float32_t *pf32_TargetCentroid_pinned;
    float32_t *pf32_H_pinned;
    float32_t f32_Distance = 0.f;

    Eigen::Matrix4f st_ResultT = Eigen::Matrix4f::Identity();

    Eigen::MatrixXf st_H;
    cudaError_t st_Err;

    cudaHostAlloc((void**)&pst_Source_pinned, sizeof(ICP_POINT_CLOUD_t), cudaHostAllocDefault);
    cudaHostAlloc((void**)&pst_Target_pinned, sizeof(ICP_POINT_CLOUD_t), cudaHostAllocDefault);
    cudaHostAlloc((void**)&ps32_NearestIdx_pinned, sizeof(int32_t) * pst_Source->s32_PointNum, cudaHostAllocDefault);
    cudaMalloc((void**)&pf32_Transform_device, sizeof(float32_t) * 16);

    cudaHostAlloc((void**)&pf32_SourceCentroid_pinned, sizeof(float32_t) * 3, cudaHostAllocDefault);
    cudaHostAlloc((void**)&pf32_TargetCentroid_pinned, sizeof(float32_t) * 3, cudaHostAllocDefault);
    cudaHostAlloc((void**)&pf32_H_pinned, sizeof(float32_t) * 9, cudaHostAllocDefault);

    cudaMemcpy(pst_Source_pinned, pst_Source, sizeof(ICP_POINT_CLOUD_t), cudaMemcpyHostToDevice);
    cudaMemcpy(pst_Target_pinned, pst_Target, sizeof(ICP_POINT_CLOUD_t), cudaMemcpyHostToDevice);

    for (s32_I = 0;s32_I < s32_Iteration;s32_I++)
    {
        CUDA_FindNearestNeighbors<<<128, 256>>>(pst_Source_pinned, pst_Target_pinned, ps32_NearestIdx_pinned, f32_Threshold);
        cudaDeviceSynchronize();
        st_Err = cudaGetLastError();
        checkCudaError(st_Err, "Nearest Neighbor Error");

        s32_SourceNum = 0; s32_TargetNum = 0; s32_CorresNum = 0;
        pf32_SourceCentroid_pinned[0] = 0.f; pf32_SourceCentroid_pinned[1] = 0.f; pf32_SourceCentroid_pinned[2] = 0.f;
        pf32_TargetCentroid_pinned[0] = 0.f; pf32_TargetCentroid_pinned[1] = 0.f; pf32_TargetCentroid_pinned[2] = 0.f;
        pf32_H_pinned[0] = 0.f; pf32_H_pinned[1] = 0.f; pf32_H_pinned[2] = 0.f;
        pf32_H_pinned[3] = 0.f; pf32_H_pinned[4] = 0.f; pf32_H_pinned[5] = 0.f;
        pf32_H_pinned[6] = 0.f; pf32_H_pinned[7] = 0.f; pf32_H_pinned[8] = 0.f;
        f32_Distance = 0.f;
        Eigen::MatrixXf st_Source(0, 3);
        Eigen::MatrixXf st_Target(0, 3);

        for (s32_J = 0;s32_J < pst_Source->s32_PointNum;s32_J++)
        {

            if (ps32_NearestIdx_pinned[s32_J] == -1)
            {
                continue;
            }

            pf32_SourceCentroid_pinned[0] += pst_Source_pinned->arf32_X[s32_J];
            pf32_SourceCentroid_pinned[1] += pst_Source_pinned->arf32_Y[s32_J];
            pf32_SourceCentroid_pinned[2] += pst_Source_pinned->arf32_Z[s32_J];
            s32_SourceNum += 1;

            pf32_TargetCentroid_pinned[0] += pst_Target_pinned->arf32_X[ps32_NearestIdx_pinned[s32_J]];
            pf32_TargetCentroid_pinned[1] += pst_Target_pinned->arf32_Y[ps32_NearestIdx_pinned[s32_J]];
            pf32_TargetCentroid_pinned[2] += pst_Target_pinned->arf32_Z[ps32_NearestIdx_pinned[s32_J]];
            s32_TargetNum += 1;

            // st_Source.conservativeResize(st_Source.rows() + 1, Eigen::NoChange);
            // st_Source(st_Source.rows() - 1, 0) = pst_Source_pinned->arf32_X[s32_J];
            // st_Source(st_Source.rows() - 1, 1) = pst_Source_pinned->arf32_Y[s32_J];
            // st_Source(st_Source.rows() - 1, 2) = pst_Source_pinned->arf32_Z[s32_J];


            // st_Target.conservativeResize(st_Target.rows() + 1, Eigen::NoChange);
            // st_Target(st_Target.rows() - 1, 0) = pst_Target_pinned->arf32_X[ps32_NearestIdx_pinned[s32_J]];
            // st_Target(st_Target.rows() - 1, 1) = pst_Target_pinned->arf32_Y[ps32_NearestIdx_pinned[s32_J]];
            // st_Target(st_Target.rows() - 1, 2) = pst_Target_pinned->arf32_Z[ps32_NearestIdx_pinned[s32_J]];

            // f32_X1 = pst_Source_pinned->arf32_X[s32_J];
            // f32_Y1 = pst_Source_pinned->arf32_Y[s32_J];
            // f32_Z1 = pst_Source_pinned->arf32_Z[s32_J];


            // f32_X2 = pst_Target_pinned->arf32_X[ps32_NearestIdx_pinned[s32_J]];
            // f32_Y2 = pst_Target_pinned->arf32_Y[ps32_NearestIdx_pinned[s32_J]];
            // f32_Z2 = pst_Target_pinned->arf32_Z[ps32_NearestIdx_pinned[s32_J]];
            // // f32_Distance += sqrtf(powf(pf32_SourceCentroid_pinned[0] - pf32_TargetCentroid_pinned[0], 2.f) + 
            // //                         powf(pf32_SourceCentroid_pinned[1] - pf32_TargetCentroid_pinned[1], 2.f) + 
            // //                         powf(pf32_SourceCentroid_pinned[2] - pf32_TargetCentroid_pinned[2], 2.f));
            // // s32_CorresNum += 1;

            // f32_Distance = sqrtf(powf(f32_X1 - f32_X2, 2.f) + powf(f32_Y1 - f32_Y2, 2.f) + powf(f32_Z1 - f32_Z2, 2.f));

            // if (f32_Distance > f32_Threshold)
            // {
            //     printf("%.3f, %.3f, %.3f - %.3f, %.3f, %.3f (%.2f)\n", f32_X1, f32_Y1, f32_Z1, f32_X2, f32_Y2, f32_Z2, f32_Distance);
            // }
        }

        if (s32_SourceNum > 0)
        {
            pf32_SourceCentroid_pinned[0] /= s32_SourceNum;
            pf32_SourceCentroid_pinned[1] /= s32_SourceNum;
            pf32_SourceCentroid_pinned[2] /= s32_SourceNum;
        }
        else
        {
            break;
        }

        if (s32_TargetNum > 0)
        {
            pf32_TargetCentroid_pinned[0] /= s32_TargetNum;
            pf32_TargetCentroid_pinned[1] /= s32_TargetNum;
            pf32_TargetCentroid_pinned[2] /= s32_TargetNum;
        }
        else
        {
            break;
        }



        CUDA_ComputeCovarianceMatrix<<<128, 256>>>(pst_Source_pinned, pst_Target_pinned, ps32_NearestIdx_pinned,
                                                    pf32_H_pinned, pf32_SourceCentroid_pinned, pf32_TargetCentroid_pinned); 
        cudaDeviceSynchronize();
        st_Err = cudaGetLastError();
        checkCudaError(st_Err, "Compute Covariance Failed");

        Eigen::RowVector3f st_Source_Centroid(pf32_SourceCentroid_pinned[0], pf32_SourceCentroid_pinned[1], pf32_SourceCentroid_pinned[2]);
        Eigen::RowVector3f st_Target_Centroid(pf32_TargetCentroid_pinned[0], pf32_TargetCentroid_pinned[1], pf32_TargetCentroid_pinned[2]);
        // Eigen::Matrix3f st_H = st_Target.transpose() * st_Source;
        Eigen::Map<Eigen::Matrix3f> st_H(pf32_H_pinned);

        Eigen::JacobiSVD<Eigen::MatrixXf> st_SVD(st_H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::MatrixXf st_U = st_SVD.matrixU();
        Eigen::MatrixXf st_V = st_SVD.matrixV();
        Eigen::Matrix3f st_R = st_U * st_V.transpose();
        Eigen::Vector3f st_t = st_Target_Centroid.transpose() - (st_R * st_Source_Centroid.transpose());
        Eigen::Matrix4f st_T = Eigen::Matrix4f::Identity();
        st_T.block(0, 0, 3, 3) = st_R;
        st_T.block(0, 3, 3, 1) = st_t;
        st_ResultT = st_T * st_ResultT;

        float32_t pf32_Transform[] = {st_T(0, 0), st_T(0, 1), st_T(0, 2), st_T(0, 3),
                                      st_T(1, 0), st_T(1, 1), st_T(1, 2), st_T(1, 3),
                                      st_T(2, 0), st_T(2, 1), st_T(2, 2), st_T(2, 3),
                                      st_T(3, 0), st_T(3, 1), st_T(3, 2), st_T(3, 3)};

        cudaMemcpy(pf32_Transform_device, pf32_Transform, 16 * sizeof(float32_t), cudaMemcpyHostToDevice);
        CUDA_TransformPoints<<<128, 256>>>(pst_Source_pinned, pf32_Transform_device);
        cudaDeviceSynchronize();

        printf("Iter: %d Threshold: %.2f - %d\n", s32_I, f32_Threshold, s32_TargetNum);

        f32_Threshold *= 0.95f;
        f32_Threshold = max(0.3f, f32_Threshold);
    }

    CUDA_FindNearestNeighbors<<<128, 256>>>(pst_Source_pinned, pst_Target_pinned, ps32_NearestIdx_pinned, f32_Threshold);
    cudaDeviceSynchronize();

    for (s32_I = 0;s32_I < pst_Source->s32_PointNum;s32_I++)
    {
        ps32_NearestIdx[s32_I] = ps32_NearestIdx_pinned[s32_I];
    }

    cudaFreeHost(pst_Source_pinned);
    cudaFreeHost(pst_Target_pinned);
    cudaFreeHost(ps32_NearestIdx_pinned);
    cudaFreeHost(pf32_SourceCentroid_pinned);
    cudaFreeHost(pf32_TargetCentroid_pinned);
    cudaFreeHost(pf32_H_pinned);

    cudaFree(pf32_Transform_device);

    for (s32_I = 0;s32_I < 3;s32_I++)
    {
        for (s32_J = 0;s32_J < 4;s32_J++)
        {
            pf32_Result[s32_I * 4 + s32_J] = st_ResultT(s32_I, s32_J);
            // printf("%f ", pf32_Result[s32_I * 4 + s32_J]);
        }
        // printf("\n");
    }
}





__device__ OCTREE_NODE_GPU_t CUDA_FindLeaf(float32_t f32_X, float32_t f32_Y, float32_t f32_Z, OCTREE_NODE_GPU_t* pst_OctreeNodes) 
{
    uint64_t u64_Cur = 0;
    uint8_t u8_X = 0, u8_Y = 0, u8_Z = 0;

    OCTREE_NODE_GPU_t st_CurNode = pst_OctreeNodes[u64_Cur];
    OCTREE_NODE_GPU_t st_ParentNode = st_CurNode;
    
    while (!st_CurNode.b_IsLeaf) 
    {
        OCTREE_POINT_t st_Center = st_CurNode.st_Center;

        u8_X = f32_X > st_Center.f32_X;
        u8_Y = f32_Y > st_Center.f32_Y;
        u8_Z = f32_Z > st_Center.f32_Z;

        u64_Cur = st_CurNode.s32_FirstChildIdx + (u8_X + 2 * u8_Y + 4 * u8_Z);
        st_ParentNode = st_CurNode;
        st_CurNode = pst_OctreeNodes[u64_Cur];
    }
    
    return st_CurNode;
}





// Source Point에 근접한 Point를 Target Point에서 찾음
__global__ void CUDA_NearestNeighborOctree(ICP_POINT_CLOUD_t *pst_Source, 
                                           ICP_POINT_CLOUD_t* pst_Target, 
                                           OCTREE_NODE_GPU_t* pst_OctreeNodes,
                                           int32_t *ps32_OctreeIndex,
                                           float32_t f32_Threshold,
                                           int32_t* ps32_NearestIdx) 
{
    int32_t s32_Idx = (blockIdx.x * blockDim.x) + threadIdx.x;
    int32_t s32_NearestIdx = -1;
    int32_t s32_Start, s32_End;
    int32_t s32_Cur;
    int32_t s32_J;

    float32_t f32_Distance;
    float32_t f32_MinDist = f32_Threshold;
    float32_t f32_X, f32_Y, f32_Z;

    if (s32_Idx < pst_Source->s32_PointNum) 
    {
        f32_X = pst_Source->arf32_X[s32_Idx];
        f32_Y = pst_Source->arf32_Y[s32_Idx];
        f32_Z = pst_Source->arf32_Z[s32_Idx];

        //Find our leaf node and extract tgt_start and tgt_end from it
        OCTREE_NODE_GPU_t st_CurNode = CUDA_FindLeaf(f32_X, f32_Y, f32_Z, pst_OctreeNodes);
        int32_t s32_Start = st_CurNode.s32_DataStartIdx;
        int32_t s32_End = st_CurNode.s32_DataStartIdx + st_CurNode.s32_Count;

        for (s32_Cur = s32_Start; s32_Cur < s32_End; s32_Cur++) 
        { //Iterate through each tgt & find closest
            s32_J = ps32_OctreeIndex[s32_Cur];
            f32_Distance = sqrtf(powf((pst_Target->arf32_X[s32_J] - f32_X), 2.f) + 
                                 powf((pst_Target->arf32_Y[s32_J] - f32_Y), 2.f));

            // f32_Distance = sqrtf(powf((pst_Target->arf32_X[s32_J] - f32_X), 2.f) + 
            //                      powf((pst_Target->arf32_Y[s32_J] - f32_Y), 2.f) + 
            //                      powf((pst_Target->arf32_Z[s32_J] - f32_Z), 2.f));

            if (f32_MinDist > f32_Distance)
            {
                f32_MinDist = f32_Distance;
                s32_NearestIdx = s32_J;
            }
        }

        ps32_NearestIdx[s32_Idx] = s32_NearestIdx;
    }
}




__device__ int32_t CUDA_FindNearestRecursive(float32_t f32_X, float32_t f32_Y, float32_t f32_Z,
                                           ICP_POINT_CLOUD_t* pst_Target, 
                                           OCTREE_NODE_GPU_t* pst_OctreeNodes, 
                                           int32_t *ps32_OctreeIndex,
                                           uint64_t u64_Root, 
                                           float32_t f32_HalfLength, 
                                           float32_t f32_Threshold) 
{
    int32_t s32_Result = -1;
    int32_t s32_Cand = 0;
    float32_t f32_MinDist = f32_Threshold;
    int32_t s32_I = 0;
    uint8_t u8_X = 0, u8_Y = 0, u8_Z = 0;
    uint64_t u64_Child = 0;
    int32_t s32_Start, s32_End, s32_Cur;
    float32_t f32_Distance;
    float32_t f32_CandDistance;
    float32_t f32_LengthThreshold = 0.f;
    float32_t f32_CenterDistance = 0.f;

    OCTREE_NODE_GPU_t st_CurNode = pst_OctreeNodes[u64_Root];
    OCTREE_NODE_GPU_t st_ChildNode;
    OCTREE_POINT_t st_Center;

    if (st_CurNode.b_IsLeaf)
    {
        int32_t s32_Start = st_CurNode.s32_DataStartIdx;
        int32_t s32_End = st_CurNode.s32_DataStartIdx + st_CurNode.s32_Count;

        for (s32_Cur = s32_Start; s32_Cur < s32_End; s32_Cur++) 
        {
            s32_I = ps32_OctreeIndex[s32_Cur];
            f32_Distance = sqrtf(powf((pst_Target->arf32_X[s32_I] - f32_X), 2.f) + 
                                 powf((pst_Target->arf32_Y[s32_I] - f32_Y), 2.f) + 
                                 powf((pst_Target->arf32_Z[s32_I] - f32_Z), 2.f));

            if (f32_MinDist > f32_Distance)
            {
                f32_MinDist = f32_Distance;
                s32_Result = s32_I;
            }
        }
    }
    else
    {
        f32_HalfLength /= 2.f;
        f32_LengthThreshold = f32_HalfLength * 1.75f + f32_Threshold;

        for (s32_I = 0;s32_I < 8;s32_I++)
        {
            u64_Child = st_CurNode.s32_FirstChildIdx + s32_I;
            st_ChildNode = pst_OctreeNodes[u64_Child];
            st_Center = st_ChildNode.st_Center;

            f32_CenterDistance = sqrtf(powf(st_Center.f32_X - f32_X, 2.f) + 
                                       powf(st_Center.f32_Y - f32_Y, 2.f) +
                                       powf(st_Center.f32_Z - f32_Z, 2.f));


            if (f32_CenterDistance <= f32_LengthThreshold)
            {
                s32_Cand = CUDA_FindNearestRecursive(f32_X, f32_Y, f32_Z,
                                                      pst_Target,
                                                      pst_OctreeNodes, 
                                                      ps32_OctreeIndex,
                                                      u64_Child, 
                                                      f32_HalfLength, 
                                                      f32_Threshold);

                if (s32_Result == -1)
                {
                    s32_Result = s32_Cand;
                    f32_MinDist = sqrtf(powf((pst_Target->arf32_X[s32_Result] - f32_X), 2.f) + 
                                         powf((pst_Target->arf32_Y[s32_Result] - f32_Y), 2.f) + 
                                         powf((pst_Target->arf32_Z[s32_Result] - f32_Z), 2.f));
                }
                else
                {
                    f32_CandDistance = sqrtf(powf((pst_Target->arf32_X[s32_Cand] - f32_X), 2.f) + 
                                         powf((pst_Target->arf32_Y[s32_Cand] - f32_Y), 2.f) + 
                                         powf((pst_Target->arf32_Z[s32_Cand] - f32_Z), 2.f));

                    if (f32_CandDistance < f32_MinDist)
                    {
                        f32_MinDist = f32_CandDistance;
                        s32_Result = s32_Cand;
                    }
                }
            }
        }
    }

    return s32_Result;
}








// Source Point에 근접한 Point를 Target Point에서 찾음
__global__ void CUDA_NearestNeighborOctreeV2(ICP_POINT_CLOUD_t *pst_Source, 
                                           ICP_POINT_CLOUD_t* pst_Target, 
                                           OCTREE_NODE_GPU_t* pst_OctreeNodes,
                                           int32_t *ps32_OctreeIndex,
                                           float32_t f32_HalfLength,
                                           float32_t f32_Threshold,
                                           int32_t* ps32_NearestIdx) 
{
    int32_t s32_Idx = (blockIdx.x * blockDim.x) + threadIdx.x;
    int32_t s32_NearestIdx = -1;
    float32_t f32_MinDist = f32_Threshold;
    float32_t f32_X, f32_Y, f32_Z;

    if (s32_Idx >= pst_Source->s32_PointNum)
    {
        return;
    }

    f32_X = pst_Source->arf32_X[s32_Idx];
    f32_Y = pst_Source->arf32_Y[s32_Idx];
    f32_Z = pst_Source->arf32_Z[s32_Idx];

    s32_NearestIdx = CUDA_FindNearestRecursive(f32_X, f32_Y, f32_Z, 
                                                pst_Target, 
                                                pst_OctreeNodes, 
                                                ps32_OctreeIndex,
                                                0, 
                                                f32_HalfLength, 
                                                f32_Threshold); 

    ps32_NearestIdx[s32_Idx] = s32_NearestIdx;
}








void CUDA_ICP_OCTREE(ICP_POINT_CLOUD_t *pst_Source, ICP_POINT_CLOUD_t *pst_Target, int32_t s32_Iteration, int32_t *ps32_NearestIdx, float32_t *pf32_Result)
{
    int32_t s32_I = 0, s32_J = 0, s32_K = 0, s32_Idx = 0;
    float32_t f32_X1, f32_Y1, f32_Z1;
    float32_t f32_X2, f32_Y2, f32_Z2;
    int32_t *ps32_NearestIdx_pinned;
    float32_t f32_Threshold = 10.f;
    float32_t *pf32_Transform_device;

    int32_t s32_CorresNum;
    int32_t s32_SourceNum;

    float32_t *pf32_SourceCentroid_pinned;
    float32_t *pf32_TargetCentroid_pinned;
    float32_t *pf32_H_pinned;
    float32_t f32_Distance = 0.f;

    Eigen::Matrix4f st_ResultT = Eigen::Matrix4f::Identity();

    Eigen::MatrixXf st_H;
    cudaError_t st_Err;

    // Source & Target Point Cloud
    ICP_POINT_CLOUD_t *pst_Source_pinned;
    ICP_POINT_CLOUD_t *pst_Target_pinned;

    cudaHostAlloc((void**)&pst_Source_pinned, sizeof(ICP_POINT_CLOUD_t), cudaHostAllocDefault);
    cudaMemcpy(pst_Source_pinned, pst_Source, sizeof(ICP_POINT_CLOUD_t), cudaMemcpyHostToDevice);

    cudaHostAlloc((void**)&pst_Target_pinned, sizeof(ICP_POINT_CLOUD_t), cudaHostAllocDefault);
    cudaMemcpy(pst_Target_pinned, pst_Target, sizeof(ICP_POINT_CLOUD_t), cudaMemcpyHostToDevice);


    // Octree
    Octree st_Octree({ 0.f, 0.f, 0.f }, 300, pst_Target);
    st_Octree.Create();
    st_Octree.Compact();

    int32_t s32_NumNodes = st_Octree.st_GpuNodePool.size();
    int32_t s32_TargetIndexNum = st_Octree.st_GpuCoords.size();;

    OCTREE_NODE_GPU_t* pst_OctreeNode_device;
    int32_t *ps32_OctreeIndex_device;

    cudaMalloc((void**)&pst_OctreeNode_device, s32_NumNodes * sizeof(OCTREE_NODE_GPU_t));
    cudaMemcpy(pst_OctreeNode_device, st_Octree.st_GpuNodePool.data(), s32_NumNodes * sizeof(OCTREE_NODE_GPU_t), cudaMemcpyHostToDevice);

    cudaMalloc((void**)&ps32_OctreeIndex_device, s32_TargetIndexNum * sizeof(int32_t));
    cudaMemcpy(ps32_OctreeIndex_device, st_Octree.st_GpuCoords.data(), s32_TargetIndexNum * sizeof(int32_t), cudaMemcpyHostToDevice);



    // Etc..
    cudaHostAlloc((void**)&ps32_NearestIdx_pinned, sizeof(int32_t) * pst_Source->s32_PointNum, cudaHostAllocDefault);
    cudaMalloc((void**)&pf32_Transform_device, sizeof(float32_t) * 16);

    cudaHostAlloc((void**)&pf32_SourceCentroid_pinned, sizeof(float32_t) * 3, cudaHostAllocDefault);
    cudaHostAlloc((void**)&pf32_TargetCentroid_pinned, sizeof(float32_t) * 3, cudaHostAllocDefault);
    cudaHostAlloc((void**)&pf32_H_pinned, sizeof(float32_t) * 9, cudaHostAllocDefault);

    float32_t f32_TempMatrix[3] = {0.f};


    for (s32_I = 0;s32_I < s32_Iteration;s32_I++)
    {
        CUDA_NearestNeighborOctreeV2<<<256, 512>>>(pst_Source_pinned, pst_Target_pinned, pst_OctreeNode_device, ps32_OctreeIndex_device, 300.f, f32_Threshold, ps32_NearestIdx_pinned);
        // CUDA_NearestNeighborOctree<<<128, 256>>>(pst_Source_pinned, pst_Target_pinned, pst_OctreeNode_device, ps32_OctreeIndex_device, f32_Threshold, ps32_NearestIdx_pinned);
        cudaDeviceSynchronize();

        st_Err = cudaGetLastError();
        checkCudaError(st_Err, "Nearest Neighbor Error");

        s32_CorresNum = 0;
        pf32_SourceCentroid_pinned[0] = 0.f; pf32_SourceCentroid_pinned[1] = 0.f; pf32_SourceCentroid_pinned[2] = 0.f;
        pf32_TargetCentroid_pinned[0] = 0.f; pf32_TargetCentroid_pinned[1] = 0.f; pf32_TargetCentroid_pinned[2] = 0.f;
        pf32_H_pinned[0] = 0.f; pf32_H_pinned[1] = 0.f; pf32_H_pinned[2] = 0.f;
        pf32_H_pinned[3] = 0.f; pf32_H_pinned[4] = 0.f; pf32_H_pinned[5] = 0.f;
        pf32_H_pinned[6] = 0.f; pf32_H_pinned[7] = 0.f; pf32_H_pinned[8] = 0.f;

        for (s32_J = 0;s32_J < pst_Source->s32_PointNum;s32_J++)
        {
            s32_K = ps32_NearestIdx_pinned[s32_J];

            if (s32_K == -1)
            {
                continue;
            }

            pf32_SourceCentroid_pinned[0] += pst_Source_pinned->arf32_X[s32_J];
            pf32_SourceCentroid_pinned[1] += pst_Source_pinned->arf32_Y[s32_J];
            pf32_SourceCentroid_pinned[2] += pst_Source_pinned->arf32_Z[s32_J];

            pf32_TargetCentroid_pinned[0] += pst_Target_pinned->arf32_X[s32_K];
            pf32_TargetCentroid_pinned[1] += pst_Target_pinned->arf32_Y[s32_K];
            pf32_TargetCentroid_pinned[2] += pst_Target_pinned->arf32_Z[s32_K];

            s32_CorresNum += 1;
        }

        if (s32_CorresNum > 0)
        {
            pf32_SourceCentroid_pinned[0] /= s32_CorresNum;
            pf32_SourceCentroid_pinned[1] /= s32_CorresNum;
            pf32_SourceCentroid_pinned[2] /= s32_CorresNum;
            pf32_TargetCentroid_pinned[0] /= s32_CorresNum;
            pf32_TargetCentroid_pinned[1] /= s32_CorresNum;
            pf32_TargetCentroid_pinned[2] /= s32_CorresNum;
        }
        else
        {
            break;
        }



        CUDA_ComputeCovarianceMatrix<<<256, 512>>>(pst_Source_pinned, pst_Target_pinned, ps32_NearestIdx_pinned,
                                                    pf32_H_pinned, pf32_SourceCentroid_pinned, pf32_TargetCentroid_pinned); 
        cudaDeviceSynchronize();
        st_Err = cudaGetLastError();
        checkCudaError(st_Err, "Compute Covariance Failed");

        Eigen::RowVector3f st_Source_Centroid(pf32_SourceCentroid_pinned[0], pf32_SourceCentroid_pinned[1], pf32_SourceCentroid_pinned[2]);
        Eigen::RowVector3f st_Target_Centroid(pf32_TargetCentroid_pinned[0], pf32_TargetCentroid_pinned[1], pf32_TargetCentroid_pinned[2]);
        Eigen::Map<Eigen::Matrix3f> st_H(pf32_H_pinned);

        Eigen::JacobiSVD<Eigen::MatrixXf> st_SVD(st_H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::MatrixXf st_U = st_SVD.matrixU();
        Eigen::MatrixXf st_V = st_SVD.matrixV();
        Eigen::Matrix3f st_R = st_U * st_V.transpose();
        Eigen::Vector3f st_t = st_Target_Centroid.transpose() - (st_R * st_Source_Centroid.transpose());
        Eigen::Matrix4f st_T = Eigen::Matrix4f::Identity();
        st_T.block(0, 0, 3, 3) = st_R;
        st_T.block(0, 3, 3, 1) = st_t;
        st_ResultT = st_T * st_ResultT;

        float32_t pf32_Transform[] = {st_T(0, 0), st_T(0, 1), st_T(0, 2), st_T(0, 3),
                                      st_T(1, 0), st_T(1, 1), st_T(1, 2), st_T(1, 3),
                                      st_T(2, 0), st_T(2, 1), st_T(2, 2), st_T(2, 3),
                                      st_T(3, 0), st_T(3, 1), st_T(3, 2), st_T(3, 3)};

        cudaMemcpy(pf32_Transform_device, pf32_Transform, 16 * sizeof(float32_t), cudaMemcpyHostToDevice);
        CUDA_TransformPoints<<<256, 512>>>(pst_Source_pinned, pf32_Transform_device);
        cudaDeviceSynchronize();

        // printf("Iter: %d Threshold: %.2f - %d\n", s32_I, f32_Threshold, s32_CorresNum);
        f32_Threshold -= 0.5f;
        f32_Threshold = max(2.f, f32_Threshold);
    }

    CUDA_NearestNeighborOctreeV2<<<256, 512>>>(pst_Source_pinned, pst_Target_pinned, pst_OctreeNode_device, ps32_OctreeIndex_device, 300.f, f32_Threshold, ps32_NearestIdx_pinned);
    // CUDA_NearestNeighborOctree<<<128, 256>>>(pst_Source_pinned, pst_Target_pinned, pst_OctreeNode_device, ps32_OctreeIndex_device, f32_Threshold, ps32_NearestIdx_pinned);
    cudaDeviceSynchronize();

    for (s32_I = 0;s32_I < pst_Source->s32_PointNum;s32_I++)
    {
        ps32_NearestIdx[s32_I] = ps32_NearestIdx_pinned[s32_I];
    }

    for (s32_I = 0;s32_I < 4;s32_I++)
    {
        for (s32_J = 0;s32_J < 4;s32_J++)
        {
            pf32_Result[s32_I * 4 + s32_J] = st_ResultT(s32_I, s32_J);
            // printf("%.4f ", pf32_Result[s32_I * 4 + s32_J]);
        }
        // printf("\n");
    }

    cudaFreeHost(pst_Target_pinned);
    cudaFreeHost(pst_Source_pinned);
    cudaFreeHost(ps32_NearestIdx_pinned);
    cudaFreeHost(pf32_SourceCentroid_pinned);
    cudaFreeHost(pf32_TargetCentroid_pinned);
    cudaFreeHost(pf32_H_pinned);

    cudaFree(pst_OctreeNode_device);
    cudaFree(ps32_OctreeIndex_device);
    cudaFree(pf32_Transform_device);
}




// Source Point에 근접한 Point를 Target Point에서 찾음
__global__ void CUDA_NearestNeighborKDTree(ICP_POINT_CLOUD_t *pst_Source, 
                                           float3* pst_Target, 
                                           int32_t s32_TargetNum,
                                           float32_t f32_Threshold,
                                           int32_t* ps32_NearestIdx,
                                           float32_t* pf32_FitnessScore) 
{
    int32_t s32_Idx = (blockIdx.x * blockDim.x) + threadIdx.x;
    int32_t s32_NearestIdx;
    float32_t f32_X, f32_Y, f32_Z;
    float32_t f32_Distance;

    if (s32_Idx >= pst_Source->s32_PointNum)
    {
        return;
    }

    f32_X = pst_Source->arf32_X[s32_Idx];
    f32_Y = pst_Source->arf32_Y[s32_Idx];
    f32_Z = pst_Source->arf32_Z[s32_Idx];

    float3 st_Query = {f32_X, f32_Y, f32_Z};
    s32_NearestIdx = cukd::stackBased::fcp(st_Query, pst_Target, s32_TargetNum);
    f32_Distance = sqrtf(powf(f32_X - pst_Target[s32_NearestIdx].x, 2.f) +
                         powf(f32_Y - pst_Target[s32_NearestIdx].y, 2.f) +
                         powf(f32_Z - pst_Target[s32_NearestIdx].z, 2.f));

    if (f32_Distance <= f32_Threshold)
    {
        ps32_NearestIdx[s32_Idx] = s32_NearestIdx;
        pf32_FitnessScore[s32_Idx] = f32_Distance;
    }
    else
    {
        ps32_NearestIdx[s32_Idx] = -1;
    }
}



void CUDA_BUILD_KDTREE(ICP_POINT_CLOUD_t *pst_PointCloud)
{
    int32_t s32_I;
    float3 *pst_PointCloud_pinned;
    cudaHostAlloc((void**)&pst_PointCloud_pinned, sizeof(float3) * pst_PointCloud->s32_PointNum, cudaHostAllocDefault);
    
    for (s32_I = 0;s32_I < pst_PointCloud->s32_PointNum;s32_I++)
    {
        pst_PointCloud_pinned[s32_I].x = pst_PointCloud->arf32_X[s32_I];
        pst_PointCloud_pinned[s32_I].y = pst_PointCloud->arf32_Y[s32_I];
        pst_PointCloud_pinned[s32_I].z = pst_PointCloud->arf32_Z[s32_I];
    }
    
    cukd::buildTree(pst_PointCloud_pinned, pst_PointCloud->s32_PointNum);

    for (s32_I = 0;s32_I < pst_PointCloud->s32_PointNum;s32_I++)
    {
        pst_PointCloud->arf32_X[s32_I] = pst_PointCloud_pinned[s32_I].x;
        pst_PointCloud->arf32_Y[s32_I] = pst_PointCloud_pinned[s32_I].y;
        pst_PointCloud->arf32_Z[s32_I] = pst_PointCloud_pinned[s32_I].z;
    }
    cudaFreeHost(pst_PointCloud_pinned);
}

float32_t CUDA_ICP_KDTREE(ICP_POINT_CLOUD_t *pst_Source, ICP_POINT_CLOUD_t *pst_Target, int32_t s32_Iteration, int32_t *ps32_NearestIdx, float32_t *pf32_Result)
{
    int32_t s32_I = 0, s32_J = 0, s32_K = 0, s32_Idx = 0;
    float32_t f32_X1, f32_Y1, f32_Z1;
    float32_t f32_X2, f32_Y2, f32_Z2;
    int32_t *ps32_NearestIdx_pinned;
    float32_t f32_Threshold = 20.f;
    float32_t *pf32_Transform_device;
    float32_t *pf32_FitnessScore_pinned;
    float32_t f32_PrevFitnessScore = 99999.f;
    float32_t f32_FitnessScore = 99999.f;

    int32_t s32_CorresNum;
    int32_t s32_SourceNum;

    float32_t *pf32_SourceCentroid_pinned;
    float32_t *pf32_TargetCentroid_pinned;
    float32_t arf32_H[9];
    float32_t f32_Distance = 0.f;

    Eigen::Matrix4f st_ResultT = Eigen::Matrix4f::Identity();

    Eigen::MatrixXf st_H;
    cudaError_t st_Err;
    cudaEvent_t a, b;
    cudaEventCreate(&a);
    cudaEventCreate(&b);

    float32_t f32_Time;

    float3 *pst_Target_pinned;

    // Source & Target Point Cloud
    ICP_POINT_CLOUD_t *pst_Source_pinned;

    cudaHostAlloc((void**)&pst_Source_pinned, sizeof(ICP_POINT_CLOUD_t), cudaHostAllocDefault);
    cudaMemcpy(pst_Source_pinned, pst_Source, sizeof(ICP_POINT_CLOUD_t), cudaMemcpyHostToDevice);

    cudaHostAlloc((void**)&pst_Target_pinned, sizeof(float3) * pst_Target->s32_PointNum, cudaHostAllocDefault);

    for (s32_I = 0;s32_I < pst_Target->s32_PointNum;s32_I++)
    {
        pst_Target_pinned[s32_I].x = pst_Target->arf32_X[s32_I];
        pst_Target_pinned[s32_I].y = pst_Target->arf32_Y[s32_I];
        pst_Target_pinned[s32_I].z = pst_Target->arf32_Z[s32_I];
    }


    cudaHostAlloc((void**)&pf32_FitnessScore_pinned, sizeof(float32_t) * pst_Source->s32_PointNum, cudaHostAllocDefault);
    cudaHostAlloc((void**)&ps32_NearestIdx_pinned, sizeof(int32_t) * pst_Source->s32_PointNum, cudaHostAllocDefault);
    cudaHostAlloc((void**)&pf32_SourceCentroid_pinned, sizeof(float32_t) * 3, cudaHostAllocDefault);
    cudaHostAlloc((void**)&pf32_TargetCentroid_pinned, sizeof(float32_t) * 3, cudaHostAllocDefault);

    cudaMalloc((void**)&pf32_Transform_device, sizeof(float32_t) * 16);

    for (s32_I = 0;s32_I < s32_Iteration;s32_I++)
    {
        // cudaEventRecord(a);
        CUDA_NearestNeighborKDTree<<<512, 256>>>(pst_Source_pinned, pst_Target_pinned, pst_Target->s32_PointNum, f32_Threshold, ps32_NearestIdx_pinned, pf32_FitnessScore_pinned);
        cudaDeviceSynchronize();
        st_Err = cudaGetLastError();
        checkCudaError(st_Err, "CUDA_KNN Error1");

        // cudaEventRecord(b);
        // cudaEventSynchronize(b);
        // cudaEventElapsedTime(&f32_Time, a, b);
        // printf("NN : %f\n", f32_Time);


        s32_CorresNum = 0;
        pf32_SourceCentroid_pinned[0] = 0.f; pf32_SourceCentroid_pinned[1] = 0.f; pf32_SourceCentroid_pinned[2] = 0.f;
        pf32_TargetCentroid_pinned[0] = 0.f; pf32_TargetCentroid_pinned[1] = 0.f; pf32_TargetCentroid_pinned[2] = 0.f;
        f32_Distance = 0.f;
        memset(arf32_H, 0, sizeof(float32_t) * 9);

        for (s32_J = 0;s32_J < pst_Source->s32_PointNum;s32_J++)
        {
            s32_K = ps32_NearestIdx_pinned[s32_J];

            if (s32_K == -1)
            {
                continue;
            }

            pf32_SourceCentroid_pinned[0] += pst_Source_pinned->arf32_X[s32_J];
            pf32_SourceCentroid_pinned[1] += pst_Source_pinned->arf32_Y[s32_J];
            pf32_SourceCentroid_pinned[2] += pst_Source_pinned->arf32_Z[s32_J];

            pf32_TargetCentroid_pinned[0] += pst_Target_pinned[s32_K].x;
            pf32_TargetCentroid_pinned[1] += pst_Target_pinned[s32_K].y;
            pf32_TargetCentroid_pinned[2] += pst_Target_pinned[s32_K].z;

            f32_Distance += pf32_FitnessScore_pinned[s32_J];
            s32_CorresNum += 1;
        }

        if (s32_CorresNum > 0)
        {
            pf32_SourceCentroid_pinned[0] /= s32_CorresNum;
            pf32_SourceCentroid_pinned[1] /= s32_CorresNum;
            pf32_SourceCentroid_pinned[2] /= s32_CorresNum;
            pf32_TargetCentroid_pinned[0] /= s32_CorresNum;
            pf32_TargetCentroid_pinned[1] /= s32_CorresNum;
            pf32_TargetCentroid_pinned[2] /= s32_CorresNum;
        }
        else
        {
            break;
        }

        // cudaEventRecord(a);

        for (s32_J = 0;s32_J < pst_Source->s32_PointNum;s32_J++)
        {
            s32_K = ps32_NearestIdx_pinned[s32_J];

            if (s32_K == -1)
            {
                continue;
            }

            f32_X1 = pst_Source_pinned->arf32_X[s32_J] - pf32_SourceCentroid_pinned[0];
            f32_Y1 = pst_Source_pinned->arf32_Y[s32_J] - pf32_SourceCentroid_pinned[1];
            f32_Z1 = pst_Source_pinned->arf32_Z[s32_J] - pf32_SourceCentroid_pinned[2];
            f32_X2 = pst_Target_pinned[s32_K].x - pf32_TargetCentroid_pinned[0];
            f32_Y2 = pst_Target_pinned[s32_K].y - pf32_TargetCentroid_pinned[1];
            f32_Z2 = pst_Target_pinned[s32_K].z - pf32_TargetCentroid_pinned[2];

            arf32_H[0] += f32_X1 * f32_X2;
            arf32_H[1] += f32_X1 * f32_Y2;
            arf32_H[2] += f32_X1 * f32_Z2;
            arf32_H[3] += f32_Y1 * f32_X2;
            arf32_H[4] += f32_Y1 * f32_Y2;
            arf32_H[5] += f32_Y1 * f32_Z2;
            arf32_H[6] += f32_Z1 * f32_X2;
            arf32_H[7] += f32_Z1 * f32_Y2;
            arf32_H[8] += f32_Z1 * f32_Z2;
        }

        // cudaEventRecord(b);
        // cudaEventSynchronize(b);
        // cudaEventElapsedTime(&f32_Time, a, b);

        // printf("Cov : %f\n", f32_Time);


        Eigen::RowVector3f st_Source_Centroid(pf32_SourceCentroid_pinned[0], pf32_SourceCentroid_pinned[1], pf32_SourceCentroid_pinned[2]);
        Eigen::RowVector3f st_Target_Centroid(pf32_TargetCentroid_pinned[0], pf32_TargetCentroid_pinned[1], pf32_TargetCentroid_pinned[2]);
        Eigen::Map<Eigen::Matrix3f> st_H(arf32_H);
        // Eigen::Map<Eigen::Matrix3f> st_H(pf32_H_pinned);

        Eigen::JacobiSVD<Eigen::MatrixXf> st_SVD(st_H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::MatrixXf st_U = st_SVD.matrixU();
        Eigen::MatrixXf st_V = st_SVD.matrixV();
        Eigen::Matrix3f st_R = st_U * st_V.transpose();
        Eigen::Vector3f st_t = st_Target_Centroid.transpose() - (st_R * st_Source_Centroid.transpose());
        Eigen::Matrix4f st_T = Eigen::Matrix4f::Identity();
        st_T.block(0, 0, 3, 3) = st_R;
        st_T.block(0, 3, 3, 1) = st_t;
        st_ResultT = st_T * st_ResultT;

        float32_t pf32_Transform[] = {st_T(0, 0), st_T(0, 1), st_T(0, 2), st_T(0, 3),
                                      st_T(1, 0), st_T(1, 1), st_T(1, 2), st_T(1, 3),
                                      st_T(2, 0), st_T(2, 1), st_T(2, 2), st_T(2, 3),
                                      st_T(3, 0), st_T(3, 1), st_T(3, 2), st_T(3, 3)};

        cudaMemcpy(pf32_Transform_device, pf32_Transform, 16 * sizeof(float32_t), cudaMemcpyHostToDevice);

        CUDA_TransformPoints<<<512, 256>>>(pst_Source_pinned, pf32_Transform_device);
        cudaDeviceSynchronize();

        f32_FitnessScore = f32_Distance / s32_CorresNum;
        // printf("Iter: %d / Fitness Score: %.2f\n", s32_I, f32_FitnessScore);

        if ((f32_FitnessScore + 0.001f) > f32_PrevFitnessScore || f32_FitnessScore < 0.39f)
        {
            break;
        }

        f32_PrevFitnessScore = f32_FitnessScore;
        f32_Threshold -= 0.7f;
        f32_Threshold = max(2.f, f32_Threshold);
    }

    CUDA_NearestNeighborKDTree<<<512, 256>>>(pst_Source_pinned, pst_Target_pinned, pst_Target->s32_PointNum, f32_Threshold, ps32_NearestIdx_pinned, pf32_FitnessScore_pinned);
    cudaDeviceSynchronize();
    st_Err = cudaGetLastError();
    checkCudaError(st_Err, "CUDA_KNN Error2");

    // s32_CorresNum = 0;
    // f32_Distance = 0.f;

    for (s32_I = 0;s32_I < pst_Source->s32_PointNum;s32_I++)
    {
        ps32_NearestIdx[s32_I] = ps32_NearestIdx_pinned[s32_I];
        // if (ps32_NearestIdx[s32_I] != -1)
        // {
        //     s32_CorresNum += 1;
        //     f32_Distance += pf32_FitnessScore_pinned[s32_I];
        // }
    }

    for (s32_I = 0;s32_I < 4;s32_I++)
    {
        for (s32_J = 0;s32_J < 4;s32_J++)
        {
            pf32_Result[s32_I * 4 + s32_J] = st_ResultT(s32_I, s32_J);
        }
    }

    cudaFreeHost(pst_Target_pinned);
    cudaFreeHost(pst_Source_pinned);
    cudaFreeHost(ps32_NearestIdx_pinned);
    cudaFreeHost(pf32_SourceCentroid_pinned);
    cudaFreeHost(pf32_TargetCentroid_pinned);
    cudaFreeHost(pf32_FitnessScore_pinned);
    cudaFree(pf32_Transform_device);

    if (s32_CorresNum > 0)
    {
       return f32_FitnessScore;
       // return f32_Distance / s32_CorresNum;
    }
    else
    {
        return 999999.f;
    }
        
}





__device__ float32_t CUDA_CalcDotProduct(HMC_POINT_t v1, HMC_POINT_t v2)
{
    return v1.f32_X * v2.f32_X + v1.f32_Y * v2.f32_Y;
}


__device__ void CUDA_getGlobalCoord(float32_t f32_X, float32_t f32_Y, float32_t f32_Yaw_rad,
                                    float32_t f32_LocalX, float32_t f32_LocalY,
                                    float32_t &f32_ResultX, float32_t &f32_ResultY)
{
   f32_ResultX = (f32_LocalX * cosf(f32_Yaw_rad) - f32_LocalY * sinf(f32_Yaw_rad)) + f32_X;
   f32_ResultY = (f32_LocalX * sinf(f32_Yaw_rad) + f32_LocalY * cosf(f32_Yaw_rad)) + f32_Y;
}


__device__ void CUDA_CheckMapBoundary(PATH_t *pst_Boundary, float32_t f32_GlobalX, float32_t f32_GlobalY, int32_t &s32_Cross, float32_t &f32_MinDistance)
{
    int32_t s32_I;

    float32_t f32_StartX, f32_StartY, f32_EndX, f32_EndY;
    float32_t f32_IntersectX;
    float32_t f32_UnitProjection, f32_Distance, f32_ProjectionX, f32_ProjectionY;

    HMC_POINT_t st_LineVector, st_PointVector;

    for(s32_I = 0; s32_I < pst_Boundary->s32_Num; s32_I++)
    {
        f32_StartX = pst_Boundary->arf32_X[s32_I];
        f32_StartY = pst_Boundary->arf32_Y[s32_I];
        f32_EndX = pst_Boundary->arf32_X[(s32_I + 1) % pst_Boundary->s32_Num];
        f32_EndY = pst_Boundary->arf32_Y[(s32_I + 1) % pst_Boundary->s32_Num];

        if((f32_StartY > f32_GlobalY) != (f32_EndY > f32_GlobalY))
        {
            f32_IntersectX = (f32_EndX - f32_StartX) * (f32_GlobalY - f32_StartY) / (f32_EndY - f32_StartY) + f32_StartX;

            if(f32_GlobalX < f32_IntersectX)
            {
                s32_Cross++;
            }
        }

        st_LineVector.f32_X = f32_EndX - f32_StartX;
        st_LineVector.f32_Y = f32_EndY - f32_StartY;
        st_PointVector.f32_X = f32_GlobalX - f32_StartX;
        st_PointVector.f32_Y = f32_GlobalY - f32_StartY;

        f32_UnitProjection = CUDA_CalcDotProduct(st_PointVector, st_LineVector) / CUDA_CalcDotProduct(st_LineVector, st_LineVector);

        if(f32_UnitProjection < 0.f)
        {
            f32_Distance = CUDA_getDistance2d(f32_StartX, f32_StartY, f32_GlobalX, f32_GlobalY);
        }
        else if(f32_UnitProjection > 1.f)
        {
            f32_Distance = CUDA_getDistance2d(f32_EndX, f32_EndY, f32_GlobalX, f32_GlobalY);
        }
        else
        {
            f32_ProjectionX = f32_StartX + f32_UnitProjection * st_LineVector.f32_X;
            f32_ProjectionY = f32_StartY + f32_UnitProjection * st_LineVector.f32_Y;

            f32_Distance = CUDA_getDistance2d(f32_ProjectionX, f32_ProjectionY, f32_GlobalX, f32_GlobalY);
        }

        if(f32_Distance < f32_MinDistance)
        {
            f32_MinDistance = f32_Distance;
        }
    }
}


__global__ void CUDA_MapBoundaryFilterCluster(HMC_LIDAR_DATA_t *pst_LidarData, PATH_t *pst_RefBoundary, PATH_t *pst_PitBoundary,
                                              float32_t f32_MapDistanceOffset, VEHICLE_DATA_t *pst_EgoGlobalPosition)
{
    int32_t s32_I = blockIdx.x * blockDim.x + threadIdx.x;

    if(s32_I >= pst_LidarData->s32_ClusterNum)
    {
        return;
    }

    int32_t s32_Cross = 0;

    float32_t f32_ClusterGlobalX, f32_ClusterGlobalY;
    float32_t f32_MinDistance = FLT_MAX;

    HMC_CLUSTER_t *pst_Cluster = pst_LidarData->arst_Cluster;

    CUDA_getGlobalCoord(pst_EgoGlobalPosition->f32_X_global, pst_EgoGlobalPosition->f32_Y_global, pst_EgoGlobalPosition->f32_Yaw_rad_ENU,
                        pst_Cluster[s32_I].f32_CenterX, pst_Cluster[s32_I].f32_CenterY, f32_ClusterGlobalX, f32_ClusterGlobalY);

    CUDA_CheckMapBoundary(pst_RefBoundary, f32_ClusterGlobalX, f32_ClusterGlobalY, s32_Cross, f32_MinDistance);

    if (s32_Cross % 2 == 0)
    {
        pst_Cluster[s32_I].u8_Flag = c_HMC_CLUSTER_NONE;
    }
    else if (f32_MinDistance < f32_MapDistanceOffset)
    {
        pst_Cluster[s32_I].u8_Flag = c_HMC_CLUSTER_NONE;
    }
    else
    {
        s32_Cross = 0;

        CUDA_CheckMapBoundary(pst_PitBoundary, f32_ClusterGlobalX, f32_ClusterGlobalY, s32_Cross, f32_MinDistance);

        if (s32_Cross % 2 != 0)
        {
            pst_Cluster[s32_I].u8_Flag = c_HMC_CLUSTER_NONE;
        }
        else if (f32_MinDistance < f32_MapDistanceOffset)
        {
            pst_Cluster[s32_I].u8_Flag = c_HMC_CLUSTER_NONE;
        }
    }
}


void CUDA_MapBoundaryFilter(HMC_LIDAR_DATA_t *pst_LidarData, PATH_t *pst_RefBoundary, PATH_t *pst_PitBoundary,
                            float32_t f32_MapDistanceOffset, VEHICLE_DATA_t *pst_EgoGlobalPosition, int32_t s32_Target)
{
    if(pst_LidarData->s32_ClusterNum <= 0)
    {
        return;
    }

    int32_t s32_Block;

    HMC_LIDAR_DATA_t *pst_LidarData_device;
    PATH_t *pst_RefBoundary_device;
    PATH_t *pst_PitBoundary_device;
    VEHICLE_DATA_t *pst_EgoGlobalPosition_device;

    cudaMalloc((void**)&pst_LidarData_device, sizeof(HMC_LIDAR_DATA_t));
    cudaMalloc((void**)&pst_RefBoundary_device, sizeof(PATH_t));
    cudaMalloc((void**)&pst_PitBoundary_device, sizeof(PATH_t));
    cudaMalloc((void**)&pst_EgoGlobalPosition_device, sizeof(VEHICLE_DATA_t));

    cudaMemcpy(pst_LidarData_device->arst_Cluster, pst_LidarData->arst_Cluster, pst_LidarData->s32_ClusterNum * sizeof(HMC_CLUSTER_t), cudaMemcpyHostToDevice);
    cudaMemcpy(&pst_LidarData_device->s32_ClusterNum, &pst_LidarData->s32_ClusterNum, sizeof(int32_t), cudaMemcpyHostToDevice);
    cudaMemcpy(pst_RefBoundary_device, pst_RefBoundary, sizeof(PATH_t), cudaMemcpyHostToDevice);
    cudaMemcpy(pst_PitBoundary_device, pst_PitBoundary, sizeof(PATH_t), cudaMemcpyHostToDevice);
    cudaMemcpy(pst_EgoGlobalPosition_device, pst_EgoGlobalPosition, sizeof(VEHICLE_DATA_t), cudaMemcpyHostToDevice);

    if(s32_Target == 0) // Cluster
    {
        s32_Block = (pst_LidarData->s32_ClusterNum + 256 - 1) / 256;

        CUDA_MapBoundaryFilterCluster<<<s32_Block, 256>>>(pst_LidarData_device, pst_RefBoundary_device, pst_PitBoundary_device,
                                                          f32_MapDistanceOffset, pst_EgoGlobalPosition_device);
        cudaDeviceSynchronize();
    }
    else // Tracking
    {
        ;
    }

    cudaMemcpy(pst_LidarData->arst_Cluster, pst_LidarData_device->arst_Cluster, pst_LidarData->s32_ClusterNum * sizeof(HMC_CLUSTER_t), cudaMemcpyDeviceToHost);

    cudaFree(pst_LidarData_device);
    cudaFree(pst_RefBoundary_device);
    cudaFree(pst_PitBoundary_device);
    cudaFree(pst_EgoGlobalPosition_device);
}


__global__ void CUDA_InitialAlignment(ICP_POINT_CLOUD_t *pst_Source)
{
    int32_t s32_I = blockIdx.x * blockDim.x + threadIdx.x;

    if(s32_I >= pst_Source->s32_PointNum)
    {
        return;
    }

    float32_t f32_X, f32_Y, f32_Z, f32_Yaw_rad;

    f32_X = pst_Source->arf32_X[s32_I];
    f32_Y = pst_Source->arf32_Y[s32_I];
    f32_Z = pst_Source->arf32_Z[s32_I];
    f32_Yaw_rad = (-pst_Source->f32_Yaw_deg); // rad

    f32_X -= pst_Source->f32_X;
    f32_Y -= pst_Source->f32_Y;
    f32_Z -= pst_Source->f32_Z;

    pst_Source->arf32_X[s32_I] = cos(f32_Yaw_rad) * f32_X - sin(f32_Yaw_rad) * f32_Y;
    pst_Source->arf32_Y[s32_I] = sin(f32_Yaw_rad) * f32_X + cos(f32_Yaw_rad) * f32_Y;
    pst_Source->arf32_Z[s32_I] = f32_Z;
}


void CUDA_ComputeCovarianceMatrix(ICP_POINT_CLOUD_t *pst_Source, float3 *pst_Target, int32_t *ps32_NearestIdx, 
                                             float32_t* pf32_H, Eigen::Vector3f &st_SourceCentroid, Eigen::Vector3f &st_TargetCentroid) 
{
    int32_t s32_I, s32_J;
    float32_t f32_X1, f32_Y1, f32_Z1;
    float32_t f32_X2, f32_Y2, f32_Z2;

    for (s32_I = 0;s32_I < pst_Source->s32_PointNum;s32_I++)
    {
        s32_J = ps32_NearestIdx[s32_I];

        if (s32_J == -1)
        {
            continue;
        }

        f32_X1 = pst_Source->arf32_X[s32_I] - st_SourceCentroid[0];
        f32_Y1 = pst_Source->arf32_Y[s32_I] - st_SourceCentroid[1];
        f32_Z1 = pst_Source->arf32_Z[s32_I] - st_SourceCentroid[2];
        f32_X2 = pst_Target[s32_J].x - st_TargetCentroid[0];
        f32_Y2 = pst_Target[s32_J].y - st_TargetCentroid[1];
        f32_Z2 = pst_Target[s32_J].z - st_TargetCentroid[2];

        pf32_H[0] += f32_X1 * f32_X2;
        pf32_H[1] += f32_X1 * f32_Y2;
        pf32_H[2] += f32_X1 * f32_Z2;
        pf32_H[3] += f32_Y1 * f32_X2;
        pf32_H[4] += f32_Y1 * f32_Y2;
        pf32_H[5] += f32_Y1 * f32_Z2;
        pf32_H[6] += f32_Z1 * f32_X2;
        pf32_H[7] += f32_Z1 * f32_Y2;
        pf32_H[8] += f32_Z1 * f32_Z2;
    }
}


void CUDA_TransformPoints(ICP_POINT_CLOUD_t* pst_Source, Eigen::Matrix3f &st_R, Eigen::Vector3f &st_t) 
{
    int32_t s32_I;
    float32_t f32_X, f32_Y, f32_Z;

    for(s32_I = 0; s32_I < pst_Source->s32_PointNum; s32_I++)
    {
        f32_X = pst_Source->arf32_X[s32_I];
        f32_Y = pst_Source->arf32_Y[s32_I];
        f32_Z = pst_Source->arf32_Z[s32_I];

        pst_Source->arf32_X[s32_I] = st_R(0, 0) * f32_X + st_R(0, 1) * f32_Y + st_R(0, 2) * f32_Z + st_t(0);
        pst_Source->arf32_Y[s32_I] = st_R(1, 0) * f32_X + st_R(1, 1) * f32_Y + st_R(1, 2) * f32_Z + st_t(1);
        pst_Source->arf32_Z[s32_I] = st_R(2, 0) * f32_X + st_R(2, 1) * f32_Y + st_R(2, 2) * f32_Z + st_t(2);
    }
}


void CUDA_GetEulerAngles(Eigen::Matrix3f& st_R, float32_t& f32_Roll_rad, float32_t& f32_Pitch_rad, float32_t& f32_Yaw_rad)
{
    if(st_R(2, 0) < 1.f)
    {
        if(st_R(2, 0) > -1.f)
        {
            f32_Pitch_rad = asin(-st_R(2, 0));
            f32_Roll_rad = atan2(st_R(2, 1) / cos(f32_Pitch_rad), st_R(2, 2) / cos(f32_Pitch_rad));
            f32_Yaw_rad = atan2(st_R(1, 0) / cos(f32_Pitch_rad), st_R(0, 0) / cos(f32_Pitch_rad));
        }
        else
        {
            f32_Pitch_rad = M_PI / 2.f;
            f32_Roll_rad = -atan2(-st_R(1, 2), st_R(1, 1));
            f32_Yaw_rad = 0.f;
        }
    }
    else
    {
        f32_Pitch_rad = -M_PI / 2.f;
        f32_Roll_rad = atan2(-st_R(1, 2), st_R(1, 1));
        f32_Yaw_rad = 0.f;
    }
}


float32_t CUDA_VehicleICP(ICP_POINT_CLOUD_t *pst_Source, ICP_POINT_CLOUD_t *pst_Target, int32_t s32_Iteration)
{
    int32_t s32_Block;
    int32_t s32_I, s32_J, s32_K;
    cudaError_t st_Err;

    ICP_POINT_CLOUD_t *pst_Source_pinned;
    float3 *pst_Target_pinned;

    float32_t f32_Threshold = 10.f;
    int32_t *ps32_NearestIdx_pinned;
    float32_t *pf32_FitnessScore_pinned;
    float32_t f32_FitnessScore = 99999.f;

    int32_t s32_CorresNum;
    Eigen::Vector3f st_SourceCentroid;
    Eigen::Vector3f st_TargetCentroid;

    float32_t arf32_H[9];
    Eigen::Matrix3f st_H;
    Eigen::JacobiSVD<Eigen::MatrixXf> st_SVD;
    Eigen::Matrix3f st_R;
    Eigen::Vector3f st_t;

    float32_t f32_Roll_rad, f32_Pitch_rad, f32_Yaw_rad;
    float32_t f32_TotalRoll_rad = 0, f32_TotalPitch_rad = 0, f32_TotalYaw_rad = 0;
    float32_t arf32_TotalTranslate[3] = {0};

    cudaHostAlloc((void**)&pst_Source_pinned, sizeof(ICP_POINT_CLOUD_t), cudaHostAllocDefault);
    cudaMemcpy(pst_Source_pinned, pst_Source, sizeof(ICP_POINT_CLOUD_t), cudaMemcpyHostToDevice);

    cudaHostAlloc((void**)&pst_Target_pinned, sizeof(float3) * pst_Target->s32_PointNum, cudaHostAllocDefault);

    for (s32_I = 0;s32_I < pst_Target->s32_PointNum;s32_I++)
    {
        pst_Target_pinned[s32_I].x = pst_Target->arf32_X[s32_I];
        pst_Target_pinned[s32_I].y = pst_Target->arf32_Y[s32_I];
        pst_Target_pinned[s32_I].z = pst_Target->arf32_Z[s32_I];
    }

    cudaHostAlloc((void**)&ps32_NearestIdx_pinned, sizeof(int32_t) * pst_Source->s32_PointNum, cudaHostAllocDefault);
    cudaHostAlloc((void**)&pf32_FitnessScore_pinned, sizeof(float32_t) * pst_Source->s32_PointNum, cudaHostAllocDefault);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    s32_Block = (pst_Source->s32_PointNum + 256 - 1) / 256;

    CUDA_InitialAlignment<<<s32_Block, 256>>>(pst_Source_pinned);
    cudaDeviceSynchronize();
    st_Err = cudaGetLastError();
    checkCudaError(st_Err, "CUDA_VehicleICP Error1");

    for(s32_I = 0; s32_I < s32_Iteration; s32_I++)
    {
        CUDA_NearestNeighborKDTree<<<512, 256>>>(pst_Source_pinned, pst_Target_pinned, pst_Target->s32_PointNum, f32_Threshold, ps32_NearestIdx_pinned, pf32_FitnessScore_pinned);
        cudaDeviceSynchronize();
        st_Err = cudaGetLastError();
        checkCudaError(st_Err, "CUDA_VehicleICP Error2");

        s32_CorresNum = 0;
        f32_FitnessScore = 0.f;

        memset(&st_SourceCentroid, 0, sizeof(Eigen::Vector3f));
        memset(&st_TargetCentroid, 0, sizeof(Eigen::Vector3f));
        memset(arf32_H, 0, sizeof(float32_t) * 9);

        for(s32_J = 0; s32_J < pst_Source->s32_PointNum; s32_J++)
        {
            s32_K = ps32_NearestIdx_pinned[s32_J];

            if(s32_K == -1)
            {
                continue;
            }

            st_SourceCentroid[0] += pst_Source_pinned->arf32_X[s32_J];
            st_SourceCentroid[1] += pst_Source_pinned->arf32_Y[s32_J];
            st_SourceCentroid[2] += pst_Source_pinned->arf32_Z[s32_J];

            st_TargetCentroid[0] += pst_Target_pinned[s32_K].x;
            st_TargetCentroid[1] += pst_Target_pinned[s32_K].y;
            st_TargetCentroid[2] += pst_Target_pinned[s32_K].z;

            f32_FitnessScore += pf32_FitnessScore_pinned[s32_J];
            s32_CorresNum++;
        }

        f32_FitnessScore /= s32_CorresNum;

        if(f32_FitnessScore < 0.05f || s32_CorresNum < 0)
        {
            break;
        }

        st_SourceCentroid[0] /= s32_CorresNum;
        st_SourceCentroid[1] /= s32_CorresNum;
        st_SourceCentroid[2] /= s32_CorresNum;

        st_TargetCentroid[0] /= s32_CorresNum;
        st_TargetCentroid[1] /= s32_CorresNum;
        st_TargetCentroid[2] /= s32_CorresNum;

        CUDA_ComputeCovarianceMatrix(pst_Source_pinned, pst_Target_pinned, ps32_NearestIdx_pinned, arf32_H,
                                     st_SourceCentroid, st_TargetCentroid);

        st_H = Eigen::Map<Eigen::Matrix3f>(arf32_H);
        st_SVD.compute(st_H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        st_R = st_SVD.matrixU() * st_SVD.matrixV().transpose();
        st_t = st_TargetCentroid - st_R * st_SourceCentroid;

        CUDA_TransformPoints(pst_Source_pinned, st_R, st_t);

        arf32_TotalTranslate[0] += st_t(0);
        arf32_TotalTranslate[1] += st_t(1);
        arf32_TotalTranslate[2] += st_t(2);

        CUDA_GetEulerAngles(st_R, f32_Roll_rad, f32_Pitch_rad, f32_Yaw_rad);
        f32_TotalRoll_rad += f32_Roll_rad;
        f32_TotalPitch_rad += f32_Pitch_rad;
        f32_TotalYaw_rad += f32_Yaw_rad;
    }
    
    pst_Source->f32_X = arf32_TotalTranslate[0];
    pst_Source->f32_Y = arf32_TotalTranslate[1];
    pst_Source->f32_Z = arf32_TotalTranslate[2];
    pst_Source->f32_Yaw_deg -= f32_TotalYaw_rad;

    cudaFreeHost(pst_Source_pinned);
    cudaFreeHost(pst_Target_pinned);
    cudaFreeHost(ps32_NearestIdx_pinned);
    cudaFreeHost(pf32_FitnessScore_pinned);

    if(s32_CorresNum > 0)
    {
        return f32_FitnessScore;
    }
    else
    {
        return 99999.f;
    }
}