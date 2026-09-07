#include "global.h"

extern float64_t f64_Vx, f64_Vy, f64_Ax, f64_Ay, f64_c_V, f64_c_A;
extern int32_t s32_FileNum;


float64_t DegtoRad()
{
    return M_PI/180;
}

float64_t RadtoDeg()
{
    return 180/M_PI;
}

void ms2hms(uint64_t &u64_Time, int32_t ars_Time[])
{
    int32_t s32_Time = u64_Time / 1000;
    ars_Time[0] = s32_Time / 3600;
    ars_Time[1] = (s32_Time%3600)/60;
    ars_Time[2] = s32_Time%60;
}   


uint64_t getMillisecond()
{
    auto st_Now = std::chrono::steady_clock::now();
    auto st_Now_ms = time_point_cast<std::chrono::milliseconds>(st_Now);
    milliseconds st_Millisecond = duration_cast<std::chrono::milliseconds>(st_Now_ms.time_since_epoch());
    return st_Millisecond.count();
}

uint64_t getMicrosecond()
{
    auto st_Now = std::chrono::steady_clock::now();
    auto st_Now_us = std::chrono::time_point_cast<std::chrono::microseconds>(st_Now);
    std::chrono::microseconds st_Microsecond = std::chrono::duration_cast<std::chrono::microseconds>(st_Now_us.time_since_epoch());
    return st_Microsecond.count();
}

float32_t deg2rad(float32_t f32_Degree)
{
    return f32_Degree * M_PI / 180.f;
}


float32_t rad2deg(float32_t f32_Radian)
{
    return f32_Radian * 180.f / M_PI;
}


float64_t deg2rad(float64_t f64_Degree)
{
    return f64_Degree * M_PI / 180.f;
}


float64_t rad2deg(float64_t f64_Radian)
{
    return f64_Radian * 180.f / M_PI;
}

float32_t ms2kph(float32_t f32_Speed)
{
    return f32_Speed * 3.6f;
}

float32_t kph2ms(float32_t f32_Speed)
{
    return f32_Speed / 3.6f;
}

float64_t ms2kph(float64_t f64_Speed)
{
    return f64_Speed * 3.6;
}

float64_t kph2ms(float64_t f64_Speed)
{
    return f64_Speed / 3.6;
}

float32_t getDistance3d(float32_t f32_X1, float32_t f32_Y1, float32_t f32_Z1, float32_t f32_X2, float32_t f32_Y2, float32_t f32_Z2)
{
    return sqrtf(powf(f32_X1 - f32_X2, 2.f) + powf(f32_Y1 - f32_Y2, 2.f) + powf(f32_Z1 - f32_Z2, 2.f));
}

float64_t getDistance3d(float64_t f64_X1, float64_t f64_Y1, float64_t f64_Z1, float64_t f64_X2, float64_t f64_Y2, float64_t f64_Z2)
{
    return sqrtf(powf(f64_X1 - f64_X2, 2.f) + powf(f64_Y1 - f64_Y2, 2.f) + powf(f64_Z1 - f64_Z2, 2.f));
}


float32_t getDistance2d(float32_t f32_X1, float32_t f32_Y1, float32_t f32_X2, float32_t f32_Y2)
{
    return sqrtf(powf(f32_X1 - f32_X2, 2.f) + powf(f32_Y1 - f32_Y2, 2.f));
}


float64_t getDistance2d(float64_t f64_X1, float64_t f64_Y1, float64_t f64_X2, float64_t f64_Y2)
{
    return sqrtf(powf(f64_X1 - f64_X2, 2.f) + powf(f64_Y1 - f64_Y2, 2.f));
}

float32_t getNorm2d(float32_t f32_X, float32_t f32_Y)
{
    return sqrtf(powf(f32_X, 2.f) + powf(f32_Y, 2.f));
}

float64_t getNorm2d(float64_t f32_X, float64_t f32_Y)
{
    return sqrtf(powf(f32_X, 2.f) + powf(f32_Y, 2.f));
}

float32_t pi2pi(float32_t f32_Angle)
{
    if (f32_Angle > M_PI)
        f32_Angle -= 2 * M_PI;
    else if(f32_Angle < -M_PI)
        f32_Angle += 2 * M_PI;

    return f32_Angle;
}

float64_t pi2pi(float64_t f64_Angle)
{
    if (f64_Angle > M_PI)
        f64_Angle -= 2 * M_PI;
    else if(f64_Angle < -M_PI)
        f64_Angle += 2 * M_PI;

    return f64_Angle;
}




float32_t axisRotate(float32_t f32_Heading_rad)
{
    float32_t f32_Result;
    if(-1*M_PI/2 < f32_Heading_rad && f32_Heading_rad < M_PI)
    {
        f32_Result = M_PI / 2.0f - f32_Heading_rad;        
    }
    else
    {
        f32_Result = -1.5f * M_PI - f32_Heading_rad;
    }
    return f32_Result;
}

float64_t axisRotate(float64_t f64_Heading_rad)
{
    float64_t f64_Result;
    if(-1*M_PI/2 < f64_Heading_rad && f64_Heading_rad < M_PI)
    {
        f64_Result = M_PI / 2.0f - f64_Heading_rad;        
    }
    else
    {
        f64_Result = -1.5f * M_PI - f64_Heading_rad;
    }
    return f64_Result;
}

void GetModData(float32_t min, float32_t max, float32_t &data)
{
    if (data > max)
    {
        data = max;
    }
    else if (data < min)
    {
        data = min;
    }
}

void GetModData(float64_t min, float64_t max, float64_t &data)
{
    if (data > max)
    {
        data = max;
    }
    else if (data < min)
    {
        data = min;
    }
}

void GetModData(int32_t min, int32_t max, int32_t &data)
{
    if (data > max)
    {
        data = max;
    }
    else if (data < min)
    {
        data = min;
    }
}

bool InRange(float32_t min, float32_t max, float32_t data)
{
    if (min <= data && data <= max)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool InRange(float64_t min, float64_t max, float64_t data)
{
    if (min <= data && data <= max)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool InRange(int32_t min, int32_t max, int32_t data)
{
    if (min <= data && data <= max)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void OverShootFilter(float32_t f32_Maxtick, float32_t f32_Pre_data, float32_t &f32_Target_data)
{
    float32_t f32_dsteer = f32_Target_data - f32_Pre_data;

    if ((f32_dsteer) >= f32_Maxtick)
    {
        f32_Target_data = f32_Pre_data + f32_Maxtick;
    }
    else if ((f32_dsteer) <= - f32_Maxtick)
    {
        f32_Target_data = f32_Pre_data - f32_Maxtick;
    }
}

void OverShootFilter(float64_t f64_Maxtick, float64_t f64_Pre_data, float64_t &f64_Target_data)
{
    float32_t f64_dsteer = f64_Target_data - f64_Pre_data;

    if (f64_dsteer >= f64_Maxtick)
    {
        f64_Target_data = f64_Pre_data + f64_Maxtick;
    }
    else if (f64_dsteer <= - f64_Maxtick)
    {
        f64_Target_data = f64_Pre_data - f64_Maxtick;
    }
}

void GetRadius(float32_t f32_X1, float32_t f32_Y1, float32_t f32_X2, float32_t f32_Y2, float32_t f32_X3, float32_t f32_Y3, float32_t &Radius)
{
    float32_t d1 = (f32_X2 - f32_X1) / (f32_Y2 - f32_Y1);
    float32_t d2 = (f32_X3 - f32_X2) / (f32_Y3 - f32_Y2);
    float32_t cx = ((f32_Y3 - f32_Y1) + (f32_X2 + f32_X3) * d2 - (f32_X1 + f32_X2) *d1) / (2 * (d2 - d1));
    float32_t cy = - d1 * (cx - (f32_X1 + f32_X2) / 2) + (f32_Y1 + f32_Y2) / 2;
    Radius = sqrtf(powf(f32_X1 - cx, 2) + powf(f32_Y1 - cy, 2));
}

void GetRadius(float64_t f32_X1, float64_t f32_Y1, float64_t f32_X2, float64_t f32_Y2, float64_t f32_X3, float64_t f32_Y3, float64_t &Radius)
{
    float64_t d1 = (f32_X2 - f32_X1) / (f32_Y2 - f32_Y1);
    float64_t d2 = (f32_X3 - f32_X2) / (f32_Y3 - f32_Y2);
    float64_t cx = ((f32_Y3 - f32_Y1) + (f32_X2 + f32_X3) * d2 - (f32_X1 + f32_X2) *d1) / (2 * (d2 - d1));
    float64_t cy = - d1 * (cx - (f32_X1 + f32_X2) / 2) + (f32_Y1 + f32_Y2) / 2;
    Radius = sqrtf(powf(f32_X1 - cx, 2) + powf(f32_Y1 - cy, 2));
}

void CalcNearestDistIdx(float64_t f64_X, float64_t f64_Y, float64_t* f64_MapX, float64_t* f64_MapY, int32_t s32_MapLength, float64_t& f64_NearDist, int32_t& s32_NearIdx)
{
    float64_t f64_MinDist = 999999999.f;
    int32_t s32_I;
    for(s32_I = 0; s32_I < s32_MapLength; s32_I++)
    {
        float64_t f64_Dist = getDistance2d(f64_X, f64_Y, f64_MapX[s32_I], f64_MapY[s32_I]);
        if(f64_MinDist > f64_Dist)
        {
            f64_MinDist = f64_Dist; 
            f64_NearDist = f64_Dist; 
            s32_NearIdx = s32_I; 
        }
    }
}


void CalcNearestDistIdx(float32_t f32_X, float32_t f32_Y, float32_t* f32_MapX, float32_t* f32_MapY, int32_t s32_MapLength, float32_t& f32_NearDist, int32_t& s32_NearIdx)
{
    float32_t f32_MinDist = 999999999.f;
    int32_t s32_I;
    for(s32_I = 0; s32_I < s32_MapLength; s32_I++)
    {
        float32_t f32_Dist = getDistance2d(f32_X, f32_Y, f32_MapX[s32_I], f32_MapY[s32_I]);
        if(f32_MinDist > f32_Dist)
        {
            f32_MinDist = f32_Dist; 
            f32_NearDist = f32_Dist; 
            s32_NearIdx = s32_I; 
        }
    }
}

void CalcVertex(float32_t f32_X_global, float32_t f32_Y_global, float32_t f32_MaxX, float32_t f32_MaxY, float32_t f32_MinX, float32_t f32_MinY, float32_t f32_heading, float32_t (&f32_Vertex)[4][2], int32_t s32_Mode)
{
    float32_t headingRad = f32_heading;
    float32_t cosHeading = cos(headingRad);
    float32_t sinHeading = sin(headingRad);

    if(s32_Mode == -1) // Object
    {
        f32_MaxX = fmaxf(f32_MaxX , 3.f);
        f32_MaxY = fmaxf(f32_MaxY , 0.9f);
        f32_MinX = fminf(f32_MinX , 0.f);
        f32_MinY = fminf(f32_MinY , -0.9f);

        f32_Vertex[0][0] = f32_X_global + (cosHeading * f32_MaxX - sinHeading * f32_MaxY);
        f32_Vertex[0][1] = f32_Y_global + (sinHeading * f32_MaxX + cosHeading * f32_MaxY);

        f32_Vertex[1][0] = f32_X_global + (cosHeading * f32_MaxX + sinHeading * f32_MaxY);
        f32_Vertex[1][1] = f32_Y_global + (sinHeading * f32_MaxX - cosHeading * f32_MaxY);

        f32_Vertex[2][0] = f32_X_global + (cosHeading * f32_MinX + sinHeading * f32_MaxY);
        f32_Vertex[2][1] = f32_Y_global + (sinHeading * f32_MinX - cosHeading * f32_MaxY);

        f32_Vertex[3][0] = f32_X_global + (cosHeading * f32_MinX - sinHeading * f32_MaxY);
        f32_Vertex[3][1] = f32_Y_global + (sinHeading * f32_MinX + cosHeading * f32_MaxY);
    }
    else if(s32_Mode == 0) //Ego
    {
        f32_Vertex[0][0] = f32_X_global + (cosHeading * f32_MaxX - sinHeading * f32_MaxY);
        f32_Vertex[0][1] = f32_Y_global + (sinHeading * f32_MaxX + cosHeading * f32_MaxY);

        f32_Vertex[1][0] = f32_X_global + (cosHeading * f32_MaxX + sinHeading * f32_MaxY);
        f32_Vertex[1][1] = f32_Y_global + (sinHeading * f32_MaxX - cosHeading * f32_MaxY);

        f32_Vertex[2][0] = f32_X_global + (cosHeading * f32_MinX + sinHeading * f32_MaxY);
        f32_Vertex[2][1] = f32_Y_global + (sinHeading * f32_MinX - cosHeading * f32_MaxY);

        f32_Vertex[3][0] = f32_X_global + (cosHeading * f32_MinX - sinHeading * f32_MaxY);
        f32_Vertex[3][1] = f32_Y_global + (sinHeading * f32_MinX + cosHeading * f32_MaxY);
    }

}

POINT_t CalcTangentVector(POINT_t p1, POINT_t p2)
{
    POINT_t edge = {p2.f32_X - p1.f32_X, p2.f32_Y - p1.f32_Y};
    return edge;
}

POINT_t CalcNormalVector(POINT_t edge)
{
    POINT_t normal = {-edge.f32_Y, edge.f32_X};
    return normal;
}

float32_t CalcDotProduct(POINT_t v1, POINT_t v2)
{
    return v1.f32_X * v2.f32_X + v1.f32_Y * v2.f32_Y;
}


void QUINTIC_t::Init(float32_t f32_tXS, float32_t f32_tVXS, float32_t f32_tAXS, float32_t f32_tXE, float32_t f32_tVXE, float32_t f32_tAXE, float32_t f32_T)
{
    f32_XS = f32_tXS;
    f32_VXS = f32_tVXS;
    f32_AXS = f32_tAXS;
    f32_XE = f32_tXE;
    f32_VXE = f32_tVXE;
    f32_AXE = f32_tAXE;

    f32_A0 = f32_XS;
    f32_A1 = f32_VXS;
    f32_A2 = f32_AXS / 2.0f;

    Eigen::MatrixXf st_A(3, 3);
    Eigen::MatrixXf st_B(3, 1);
    Eigen::MatrixXf st_X(3, 1);

    st_A << powf(f32_T, 3.f),           powf(f32_T, 4.f),               powf(f32_T, 5.f),
            3.f * powf(f32_T, 2.f),     4.f * powf(f32_T, 3.f),         5.f * powf(f32_T, 4.f),
            6.f * f32_T,                12.f * powf(f32_T, 2.f),        20 * powf(f32_T, 3.f);

    st_B << f32_XE - f32_A0 - f32_A1 * f32_T - f32_A2 * f32_T * f32_T,
            f32_VXE - f32_A1 - 2.f * f32_A2 * f32_T,
            f32_AXE - 2.f * f32_A2;

    st_X = st_A.inverse() * st_B;

    f32_A3 = st_X(0, 0);
    f32_A4 = st_X(1, 0);
    f32_A5 = st_X(2, 0);
}

float32_t QUINTIC_t::CalcPoint(float32_t f32_T)
{
    float32_t f32_XT = f32_A0 + 
                       f32_A1 * f32_T + 
                       f32_A2 * powf(f32_T, 2.f) + 
                       f32_A3 * powf(f32_T, 3.f) + 
                       f32_A4 * powf(f32_T, 4.f) + 
                       f32_A5 * powf(f32_T, 5.f) ;
    return f32_XT;
}

float32_t QUINTIC_t::CalcFirstDerivative(float32_t f32_T)
{
    float32_t f32_XT = f32_A1 + 
                       2.f * f32_A2 * f32_T + 
                       3.f * f32_A3 * powf(f32_T, 2.f) + 
                       4.f * f32_A4 * powf(f32_T, 3.f) + 
                       5.f * f32_A5 * powf(f32_T, 4.f);
    return f32_XT;
}

float32_t QUINTIC_t::CalcSecondDerivative(float32_t f32_T)
{
    float32_t f32_XT = 2.f * f32_A2 + 
                       6.f * f32_A3 * f32_T + 
                       12.f * f32_A4 * powf(f32_T, 2.f) + 
                       20.f * f32_A5 * powf(f32_T, 3.f);
    return f32_XT;
}

float32_t QUINTIC_t::CalcThirdDerivative(float32_t f32_T)
{
    float32_t f32_XT = 6.f * f32_A3 + 
                       24.f * f32_A4 * f32_T + 
                       60.f * f32_A5 * powf(f32_T, 2.f);
    return f32_XT;
}




void QUARTIC_t::Init(float32_t f32_tXS, float32_t f32_tVXS, float32_t f32_tAXS, float32_t f32_tVXE, float32_t f32_tAXE, float32_t f32_T)
{
    f32_XS = f32_tXS;
    f32_VXS = f32_tVXS;
    f32_AXS = f32_tAXS;
    f32_VXE = f32_tVXE;
    f32_AXE = f32_tAXE;

    f32_A0 = f32_XS;
    f32_A1 = f32_VXS;
    f32_A2 = f32_AXS / 2.0f;

    Eigen::MatrixXf st_A(2, 2);
    Eigen::MatrixXf st_B(2, 1);
    Eigen::MatrixXf st_X(2, 1);

    st_A << 3.f * powf(f32_T, 2.f),         4.f * powf(f32_T, 3.f),
            6.f * f32_T,                    12.f * powf(f32_T, 2.f);

    st_B << f32_VXE - f32_A1 - 2.f * f32_A2 * f32_T,
         f32_AXE - 2.f * f32_A2;

    st_X = st_A.inverse() * st_B;

    f32_A3 = st_X(0, 0);
    f32_A4 = st_X(1, 0);
}

float32_t QUARTIC_t::CalcPoint(float32_t f32_T)
{
    float32_t f32_XT = f32_A0 + 
                       f32_A1 * f32_T + 
                       f32_A2 * powf(f32_T, 2.f) + 
                       f32_A3 * powf(f32_T, 3.f) + 
                       f32_A4 * powf(f32_T, 4.f);
    return f32_XT;
}

float32_t QUARTIC_t::CalcFirstDerivative(float32_t f32_T)
{
    float32_t f32_XT = f32_A1 + 
                       2.f * f32_A2 * f32_T + 
                       3.f * f32_A3 * powf(f32_T, 2.f) + 
                       4.f * f32_A4 * powf(f32_T, 3.f);
    return f32_XT;
}

float32_t QUARTIC_t::CalcSecondDerivative(float32_t f32_T)
{
    float32_t f32_XT = 2.f * f32_A2 + 
                       6.f * f32_A3 * f32_T + 
                       12.f * f32_A4 * powf(f32_T, 2.f);
    return f32_XT;
}

float32_t QUARTIC_t::CalcThirdDerivative(float32_t f32_T)
{
    float32_t f32_XT = 6.f * f32_A3 + 
                       24.f * f32_A4 * f32_T;
    return f32_XT;
}

SPLINE_PARAM_t::_SPLINE_PARAM() : s32_Num(0) {}

SPLINE_PARAM_t::_SPLINE_PARAM(float32_t x_[], float32_t y_[], int32_t s32_Num) : s32_Num(s32_Num)
{
    std::copy(x_, x_ + s32_Num, arf32_X);
    std::copy(y_, y_ + s32_Num, arf32_Y);
    for (int32_t s32_I = 0; s32_I < s32_Num - 1; s32_I++)
    {
        arf32_H[s32_I] = arf32_X[s32_I + 1] - arf32_X[s32_I];
    }
    std::copy(y_, y_ + s32_Num, arf32_A);

    Eigen::MatrixXf A = calc_A();
    Eigen::VectorXf B = calc_B();
    Eigen::VectorXf c_eigen = A.colPivHouseholderQr().solve(B);
    float32_t *c_pointer = c_eigen.data();
    std::copy(c_pointer, c_pointer + c_eigen.rows(), arf32_C);

    for (int32_t s32_I = 0; s32_I < s32_Num - 1; s32_I++)
    {
        arf32_D[s32_I] = (arf32_C[s32_I + 1] - arf32_C[s32_I]) / (3.0 * arf32_H[s32_I]);
        arf32_B[s32_I] = (arf32_A[s32_I + 1] - arf32_A[s32_I]) / arf32_H[s32_I] - arf32_H[s32_I] * (arf32_C[s32_I + 1] + 2 * arf32_C[s32_I]) / 3.0;
    }
}

float32_t SPLINE_PARAM_t::calc(float32_t f32_T)
{
    if (f32_T < arf32_X[0] || f32_T > arf32_X[s32_Num - 1])
    {
        // printf("Error\n");
        return 0.0;
    }

    int32_t s32_NearestIndex = CalcNearestIdx(f32_T, 0, s32_Num);
    float32_t f32_DX = f32_T - arf32_X[s32_NearestIndex];
    return arf32_A[s32_NearestIndex] + arf32_B[s32_NearestIndex] * f32_DX + arf32_C[s32_NearestIndex] * f32_DX * f32_DX + arf32_D[s32_NearestIndex] * f32_DX * f32_DX * f32_DX;
}

float32_t SPLINE_PARAM_t::calc_d(float32_t f32_T)
{
    if (f32_T < arf32_X[0] || f32_T > arf32_X[s32_Num - 1])
    {
        // printf("Error\n");
        return 0.0;
    }

    int32_t s32_NearestIndex = CalcNearestIdx(f32_T, 0, s32_Num - 1);
    float32_t f32_DX = f32_T - arf32_X[s32_NearestIndex];
    return arf32_B[s32_NearestIndex] + 2 * arf32_C[s32_NearestIndex] * f32_DX + 3 * arf32_D[s32_NearestIndex] * f32_DX * f32_DX;
}

float32_t SPLINE_PARAM_t::calc_dd(float32_t f32_T)
{
    if (f32_T < arf32_X[0] || f32_T > arf32_X[s32_Num - 1])
    {
        // printf("Error\n");
        return 0.0;
    }

    int32_t s32_NearestIndex = CalcNearestIdx(f32_T, 0, s32_Num);
    float32_t f32_DX = f32_T - arf32_X[s32_NearestIndex];
    return 2 * arf32_C[s32_NearestIndex] + 6 * arf32_D[s32_NearestIndex] * f32_DX;
}

Eigen::MatrixXf SPLINE_PARAM_t::calc_A()
{
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(s32_Num, s32_Num);
    A(0, 0) = 1;
    for (int32_t s32_I = 0; s32_I < s32_Num - 1; s32_I++)
    {
        if (s32_I != s32_Num - 2)
        {
            A(s32_I + 1, s32_I + 1) = 2 * (arf32_H[s32_I] + arf32_H[s32_I + 1]);
        }
        A(s32_I + 1, s32_I) = arf32_H[s32_I];
        A(s32_I, s32_I + 1) = arf32_H[s32_I];
    }
    A(0, 1) = 0.0;
    A(s32_Num - 1, s32_Num - 2) = 0.0;
    A(s32_Num - 1, s32_Num - 1) = 1.0;
    return A;
}

Eigen::VectorXf SPLINE_PARAM_t::calc_B()
{
    Eigen::VectorXf B = Eigen::VectorXf::Zero(s32_Num);
    for (int32_t s32_I = 0; s32_I < s32_Num - 2; s32_I++)
    {
        B(s32_I + 1) = 3.0 * (arf32_A[s32_I + 2] - arf32_A[s32_I + 1]) / arf32_H[s32_I + 1] - 3.0 * (arf32_A[s32_I + 1] - arf32_A[s32_I]) / arf32_H[s32_I];
    }
    return B;
}

int32_t SPLINE_PARAM_t::CalcNearestIdx(float32_t f32_T, int32_t s32_Start, int32_t s32_End)
{
    int32_t s32_Mid = (s32_Start + s32_End) / 2;
    if (f32_T == arf32_X[s32_Mid] || s32_End - s32_Start <= 1)
    {
        return s32_Mid;
    }
    else if (f32_T > arf32_X[s32_Mid])
    {
        return CalcNearestIdx(f32_T, s32_Mid, s32_End);
    }
    else
    {
        return CalcNearestIdx(f32_T, s32_Start, s32_Mid);
    }
}

SPLINE2D_PARAM_t::_SPLINE2D_PARAM() : s32_Size(0) {}

void SPLINE2D_PARAM_t::initialize(float32_t arf32_X[], float32_t arf32_Y[], int32_t s32_Num)
{
    s32_Size = calc_s(arf32_X, arf32_Y, s32_Num);
    st_SplineX = SPLINE_PARAM_t(arf32_S, arf32_X, s32_Size);
    st_SplineY = SPLINE_PARAM_t(arf32_S, arf32_Y, s32_Size);
}

PointArray2D SPLINE2D_PARAM_t::calc_position(float32_t f32_T)
{
    float32_t f32_X = st_SplineX.calc(f32_T);
    float32_t f32_Y = st_SplineY.calc(f32_T);
    return {{f32_X, f32_Y}};
}

float32_t SPLINE2D_PARAM_t::calc_curvature(float32_t f32_T)
{
    float32_t f32_DX = st_SplineX.calc_d(f32_T);
    float32_t f32_DDX = st_SplineX.calc_dd(f32_T);
    float32_t f32_DY = st_SplineY.calc_d(f32_T);
    float32_t f32_DDY = st_SplineY.calc_dd(f32_T);
    return fabs(f32_DDY * f32_DX - f32_DDX * f32_DY) / powf(f32_DX * f32_DX + f32_DY * f32_DY, 1.5);
}

float32_t SPLINE2D_PARAM_t::calc_yaw(float32_t f32_T)
{
    float32_t f32_DX = st_SplineX.calc_d(f32_T);
    float32_t f32_DY = st_SplineY.calc_d(f32_T);
    return std::atan2(f32_DY, f32_DX);
}

int32_t SPLINE2D_PARAM_t::calc_s(float32_t arf32_X[], float32_t arf32_Y[], int32_t s32_Num)
{
    float32_t arf32_DS[c_PLANNING_MAX_SPLINE_NUM];
    float32_t arf32_TempS[c_PLANNING_MAX_SPLINE_NUM] = {0};
    int32_t s32_Size = 1;
    for (int32_t i = 1; i < s32_Num; i++)
    {
        arf32_DS[i - 1] = std::sqrt(std::pow(arf32_X[i] - arf32_X[i - 1], 2) + std::pow(arf32_Y[i] - arf32_Y[i - 1], 2));
        arf32_TempS[i] = arf32_TempS[i - 1] + arf32_DS[i - 1];
        s32_Size++;
    }
    std::memcpy(arf32_S, arf32_TempS, s32_Size * sizeof(float32_t));
    return s32_Size;
}


void PolygonProjection(POINT_t axis, POINT_t vertices[], int numVertices, float32_t *min, float32_t *max) {
    *min = CalcDotProduct(axis, vertices[0]);
    *max = *min;
    for (int i = 1; i < numVertices; i++) {
        float32_t projection = CalcDotProduct(axis, vertices[i]);
        if (projection < *min) {
            *min = projection;
        }
        if (projection > *max) {
            *max = projection;
        }
    }
}

bool OverlappingTest(float32_t minA, float32_t maxA, float32_t minB, float32_t maxB) {
    return !(maxA < minB || maxB < minA);
}

bool CalcSAT(float32_t vertices1[4][2], float32_t vertices2[4][2]) {
    POINT_t poly1[4], poly2[4];
    for (int i = 0; i < 4; i++) {
        poly1[i].f32_X = vertices1[i][0];
        poly1[i].f32_Y = vertices1[i][1];
        poly2[i].f32_X = vertices2[i][0];
        poly2[i].f32_Y = vertices2[i][1];
    }

    for (int i = 0; i < 4; i++) {
        POINT_t edge1 = CalcTangentVector(poly1[i], poly1[(i + 1) % 4]);
        POINT_t axis1 = CalcNormalVector(edge1);

        float32_t minA, maxA, minB, maxB;
        PolygonProjection(axis1, poly1, 4, &minA, &maxA);
        PolygonProjection(axis1, poly2, 4, &minB, &maxB);

        if (!OverlappingTest(minA, maxA, minB, maxB)) {
            return false;
        }
    }

    for (int i = 0; i < 4; i++) {
        POINT_t edge2 = CalcTangentVector(poly2[i], poly2[(i + 1) % 4]);
        POINT_t axis2 = CalcNormalVector(edge2);

        float32_t minA, maxA, minB, maxB;
        PolygonProjection(axis2, poly1, 4, &minA, &maxA);
        PolygonProjection(axis2, poly2, 4, &minB, &maxB);

        if (!OverlappingTest(minA, maxA, minB, maxB)) {
            return false;
        }
    }

    return true;
}

void lla2enu(float64_t f64_Lat_deg, float64_t f64_Lon_deg, float64_t f64_Alt, float32_t &f32_E, float32_t &f32_N, float32_t &f32_U)
{
    float64_t f64_Lat_rad = deg2rad(f64_Lat_deg);
    float64_t f64_Lon_rad = deg2rad(f64_Lon_deg);

    float64_t f64_Chi = sqrt(1 - c_LLA2ENU_N_2 * pow(sin(f64_Lat_rad), 2));
    float64_t f64_Q = (c_LLA2ENU_A / f64_Chi + f64_Alt) * cos(f64_Lat_rad);

    float64_t f64_X = f64_Q * cos(f64_Lon_rad);
    float64_t f64_Y = f64_Q * sin(f64_Lon_rad);
    float64_t f64_Z = ((c_LLA2ENU_A * (1 - c_LLA2ENU_N_2) / f64_Chi) + f64_Alt) * sin(f64_Lat_rad);

    float64_t f64_dX = f64_X - c_ORIGIN_REFERENCE_X;
    float64_t f64_dY = f64_Y - c_ORIGIN_REFERENCE_Y;
    float64_t f64_dZ = f64_Z - c_ORIGIN_REFERENCE_Z;

    f32_E = (float32_t)(-sin(c_ORIGIN_LONGITUDE_RAD) * f64_dX + cos(c_ORIGIN_LONGITUDE_RAD) * f64_dY);
    f32_N = (float32_t)(-sin(c_ORIGIN_LATITUDE_RAD) * cos(c_ORIGIN_LONGITUDE_RAD) * f64_dX - sin(c_ORIGIN_LATITUDE_RAD) * sin(c_ORIGIN_LONGITUDE_RAD) * f64_dY + cos(c_ORIGIN_LATITUDE_RAD) * f64_dZ);
    f32_U = (float32_t)(cos(c_ORIGIN_LATITUDE_RAD) * cos(c_ORIGIN_LONGITUDE_RAD) * f64_dX + cos(c_ORIGIN_LATITUDE_RAD) * sin(c_ORIGIN_LONGITUDE_RAD) * f64_dY + sin(c_ORIGIN_LATITUDE_RAD) * f64_dZ);
}

void CreateBagFileName(char* datetime)
{
    time_t timer;
    struct tm* t;
    timer = time(NULL);
    t = localtime(&timer);
    sprintf(datetime, "%d_%02d%02d_%d.bin", 
            t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,s32_FileNum);

}
int extractNumberFromFilename(const std::string& filename) {
    size_t pos = filename.find_last_of('_'); // '_' 문자를 기준으로 분리
    if (pos != std::string::npos) {
        try {
            return std::stoi(filename.substr(pos + 1)); // 분리된 숫자 부분을 정수로 변환하여 반환
        } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid number format in filename: " << filename << std::endl;
        }
    }
    return 0; // 숫자를 추출할 수 없을 경우 0을 반환
}
bool CompareFilenames(const std::string& filename1, const std::string& filename2) {
    int number1 = extractNumberFromFilename(filename1);
    int number2 = extractNumberFromFilename(filename2);
    return number1 < number2;
}
void rotationMatrix(float32_t f32_Roll, float32_t f32_Pitch, float32_t f32_Yaw, float32_t f32_matrix[3][3]) {
    float32_t f32_cos_roll = cos(deg2rad(f32_Roll));
    float32_t f32_sin_roll = sin(deg2rad(f32_Roll));
    float32_t f32_cos_pitch = cos(deg2rad(f32_Pitch));
    float32_t f32_sin_pitch = sin(deg2rad(f32_Pitch));
    float32_t f32_cos_yaw = cos(deg2rad(f32_Yaw));
    float32_t f32_sin_yaw = sin(deg2rad(f32_Yaw));

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

// 3차원 포인트 회전 함수
void rotatePoint(float32_t f32_X, float32_t f32_Y, float32_t f32_Z, float32_t f32_Roll, float32_t f32_Pitch, float32_t f32_Yaw, float32_t &f32_RX, float32_t &f32_RY, float32_t &f32_RZ) {
}

void getLocalCoord(float32_t f32_X, float32_t f32_Y, float32_t f32_Yaw_rad,
               float32_t f32_GlobalX, float32_t f32_GlobalY,
               float32_t &f32_ResultX, float32_t &f32_ResultY)
{
   float32_t f32_TempX = f32_GlobalX - f32_X;
   float32_t f32_TempY = f32_GlobalY - f32_Y;
   f32_ResultX = (f32_TempX * cosf(f32_Yaw_rad) + f32_TempY * sinf(f32_Yaw_rad));
   f32_ResultY = (-f32_TempX * sinf(f32_Yaw_rad) + f32_TempY * cosf(f32_Yaw_rad));
}


void getGlobalCoord(float32_t f32_X, float32_t f32_Y, float32_t f32_Yaw_rad,
               float32_t f32_LocalX, float32_t f32_LocalY,
               float32_t &f32_ResultX, float32_t &f32_ResultY)
{
   f32_ResultX = (f32_LocalX * cosf(f32_Yaw_rad) - f32_LocalY * sinf(f32_Yaw_rad)) + f32_X;
   f32_ResultY = (f32_LocalX * sinf(f32_Yaw_rad) + f32_LocalY * cosf(f32_Yaw_rad)) + f32_Y;
}

void Gray2RGB(float32_t f32_Gray, float32_t &f32_R, float32_t &f32_G, float32_t &f32_B)
{
    float32_t f32_H;
    float32_t f32_M;
    float32_t f32_z = 0.f;
    float32_t f32_m = 0.f;

    f32_H = 235.f - f32_Gray * 0.9f;
    f32_M = 255.f;
    //(M - m)[1 - | (H / 60)mod_2 - 1 |],

    float32_t f32_Temp = f32_H / 60.0f;
    while (f32_Temp >= 2)
    {
        f32_Temp -= 2.f;
    }

    f32_z = f32_M * (1.0f - abs(f32_Temp - 1.0f));

    if (f32_H < 60)
    {
        f32_R = f32_M; f32_G = f32_m + f32_z; f32_B = f32_m;
    }

    else if (f32_H < 120)
    {
        f32_R = f32_z + f32_m; f32_G = f32_M; f32_B = f32_m;
    }
    else if (f32_H < 180)
    {
        f32_R = f32_m; f32_G = f32_M; f32_B = f32_z + f32_m;
    }
    else if (f32_H < 240)
    {
        f32_R = f32_m; f32_G = f32_z + f32_m; f32_B = f32_M;
    }
    else if (f32_H < 300)
    {
        f32_R = f32_z + f32_m; f32_G = f32_m; f32_B = f32_M;
    }
    else
    {
        f32_R = f32_M; f32_G = f32_m; f32_B = f32_z + f32_m;
    }

    f32_R /= 255.f;
    f32_G /= 255.f;
    f32_B /= 255.f;
}
