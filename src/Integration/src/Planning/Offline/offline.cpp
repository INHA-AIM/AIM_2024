#include "offline.h"

extern float32_t f32_LabCheckX1, f32_LabCheckX2, f32_LabCheckY1, f32_LabCheckY2;
/////////////////////////////////////// OFFLINE /////////////////////////////////////////
/////////////////////////////////////// OFFLINE /////////////////////////////////////////
/////////////////////////////////////// OFFLINE /////////////////////////////////////////

// Cartesian Global Path {x, y} -> Cubic Spline Interpolation-> {x, y, s, yaw, curvature}
void CalcSplineCourse(PLANNING_DATA_t *pst_PlanningData, float32_t f32_DS, int32_t s32_Line)
{
    SPLINE_PATH_t *pst_SplinePath;
    SPLINE2D_PARAM_t *pst_Spline2DParam;

    if (s32_Line == 2)
    {
        pst_SplinePath = &pst_PlanningData->st_SplinePath_global;
        pst_Spline2DParam = &pst_PlanningData->st_Spline2DParam;
        pst_Spline2DParam->initialize(&pst_PlanningData->st_ReferenceLine2_global.arf32_X[0], &pst_PlanningData->st_ReferenceLine2_global.arf32_Y[0], pst_PlanningData->st_ReferenceLine2_global.s32_Num);
    }
    else if (s32_Line == 4)
    {
        pst_SplinePath = &pst_PlanningData->st_PitStopSplinePath_global;
        pst_Spline2DParam = &pst_PlanningData->st_PisStopSpline2DParam;
        pst_Spline2DParam->initialize(&pst_PlanningData->st_ReferenceSELine2_global.arf32_X[0], &pst_PlanningData->st_ReferenceSELine2_global.arf32_Y[0], pst_PlanningData->st_ReferenceSELine2_global.s32_Num);
    }

    int32_t s32_I = 0;
    float32_t f32_J;
    for (f32_J = 0.0f; f32_J < pst_Spline2DParam->arf32_S[pst_Spline2DParam->s32_Size - 1]; f32_J += f32_DS)
    {
        PointArray2D point_ = pst_Spline2DParam->calc_position(f32_J);
        pst_SplinePath->arf32_X[s32_I] = point_[0];
        pst_SplinePath->arf32_Y[s32_I] = point_[1];
        pst_SplinePath->arf32_Yaw_rad_ENU[s32_I] = pst_Spline2DParam->calc_yaw(f32_J);
        pst_SplinePath->arf32_Yaw_rad_NED[s32_I] = axisRotate(pst_SplinePath->arf32_Yaw_rad_ENU[s32_I]);
        pst_SplinePath->arf32_Curvature[s32_I] = pst_Spline2DParam->calc_curvature(f32_J);
        pst_SplinePath->arf32_S[s32_I] = f32_J;
        s32_I++;
    }

    pst_SplinePath->f32_LastS = f32_J;
    pst_SplinePath->f32_LastBeforeS = f32_J - f32_DS;
    pst_SplinePath->s32_Num = s32_I;
}


void DataLoad(std::ifstream &st_FileName, PATH_t &Path)
{
    std::string s_Line = "";
    float32_t f32_X = 0, f32_Y = 0;
    int s32_Num = 0;
    
    while (std::getline(st_FileName, s_Line))
    {
        std::istringstream st_Stream(s_Line);
        st_Stream >> f32_X >> f32_Y;

        Path.arf32_X[s32_Num] = f32_X;
        Path.arf32_Y[s32_Num] = f32_Y;
        s32_Num += 1;
    }
    Path.s32_Num = s32_Num;
    st_FileName.close();
}

void LoadPath(PLANNING_DATA_t *pst_PlanningData)
{
    std::string s_Line;
    int32_t s32_Num = 0;
    float32_t f32_X, f32_Y, f32_Z;
    FILE *f_File;
    f_File = fopen("RoadData.bin", "rb");
                            
    // RefLine4 = JokerLap 
    YAML::Node st_Config = YAML::LoadFile("configuration.yaml");
    std::string s_MapName = st_Config["MapName"].as<std::string>();
    std::string s_MapAdress = "./src/Integration/map/" + s_MapName + "/Path/";
    std::string s_BoundaryAdress = "./src/Integration/map/" + s_MapName + "/Boundary/";

    // Path Data
    std::ifstream st_RefLine1File(s_MapAdress + "Line1.txt");
    std::ifstream st_RefLine2File(s_MapAdress + "Line2.txt");
    std::ifstream st_RefLine3File(s_MapAdress + "Line3.txt");
    std::ifstream st_RefSELine1File(s_MapAdress + "SELine1.txt");
    std::ifstream st_RefSELine2File(s_MapAdress + "SELine2.txt");
    std::ifstream st_RefSELine3File(s_MapAdress + "SELine3.txt");
    
    // idx 
    std::ifstream st_IDXFile(s_MapAdress + "idx.txt");

    // Boundary Data
    std::ifstream st_RefBoundary1File(s_BoundaryAdress + "Boundary1.txt");
    std::ifstream st_RefBoundary2File(s_BoundaryAdress + "Boundary2.txt");
    std::ifstream st_RefBoundary3File(s_BoundaryAdress + "Boundary3.txt");
    std::ifstream st_RefBoundary4File(s_BoundaryAdress + "Boundary4.txt");
    std::ifstream st_RefSEBoundary1File(s_BoundaryAdress + "SEBoundary1.txt");
    std::ifstream st_RefSEBoundary2File(s_BoundaryAdress + "SEBoundary2.txt");
    std::ifstream st_RefSEBoundary3File(s_BoundaryAdress + "SEBoundary3.txt");
    std::ifstream st_RefSEBoundary4File(s_BoundaryAdress + "SEBoundary4.txt");

    // Viewer Boundary Data ~= LiDARBoundary
    std::ifstream st_RefBoundaryInOutFile(s_BoundaryAdress + "BoundaryInOut.txt");
    std::ifstream st_RefBoundaryIn2File(s_BoundaryAdress + "BoundaryIn2.txt");

    // Path or Boundary Gap 
    std::ifstream st_PathGapFile(s_MapAdress + "PathGap.txt");
    std::ifstream st_BoundaryGapFile(s_BoundaryAdress + "BoundaryGap.txt");
    std::ifstream st_SEPathGapFile(s_MapAdress + "SEPathGap.txt");
    std::ifstream st_SEBoundaryGapFile(s_BoundaryAdress + "SEBoundaryGap.txt");

    // Check set remove??
    std::ifstream st_RefLine4File(s_MapAdress + "SELine2.txt");
    std::ifstream st_RefBoundary5File(s_BoundaryAdress + "Boundary5.txt");
    std::ifstream st_RefBoundary6File(s_BoundaryAdress + "Boundary6.txt");

    // Path Data Load
    DataLoad(st_RefLine1File, pst_PlanningData->st_ReferenceLine1_global);
    DataLoad(st_RefLine2File, pst_PlanningData->st_ReferenceLine2_global);
    DataLoad(st_RefLine3File, pst_PlanningData->st_ReferenceLine3_global);

    DataLoad(st_RefSELine1File, pst_PlanningData->st_ReferenceSELine1_global);
    DataLoad(st_RefSELine2File, pst_PlanningData->st_ReferenceSELine2_global);
    DataLoad(st_RefSELine3File, pst_PlanningData->st_ReferenceSELine3_global);
    // idx
    DataLoad(st_IDXFile, pst_PlanningData->st_IDX_global);

    // Boundary Data Load
    DataLoad(st_RefBoundary1File, pst_PlanningData->st_Boundary1_global);
    DataLoad(st_RefBoundary2File, pst_PlanningData->st_Boundary2_global);
    DataLoad(st_RefBoundary3File, pst_PlanningData->st_Boundary3_global);
    DataLoad(st_RefBoundary4File, pst_PlanningData->st_Boundary4_global);

    DataLoad(st_RefSEBoundary1File, pst_PlanningData->st_SEBoundary1_global);
    DataLoad(st_RefSEBoundary2File, pst_PlanningData->st_SEBoundary2_global);
    DataLoad(st_RefSEBoundary3File, pst_PlanningData->st_SEBoundary3_global);
    DataLoad(st_RefSEBoundary4File, pst_PlanningData->st_SEBoundary4_global);

    // Viewer Boundary Data Load ~= LiDARBondary
    DataLoad(st_RefBoundaryInOutFile, pst_PlanningData->st_BoundaryInOut);
    DataLoad(st_RefBoundaryIn2File, pst_PlanningData->st_BoundaryIn2);

    // Check set remove??
    DataLoad(st_RefBoundary5File, pst_PlanningData->st_Boundary5_global);
    DataLoad(st_RefBoundary6File, pst_PlanningData->st_Boundary6_global);

    // Path Gap Load
    s32_Num = 0;
    while (std::getline(st_PathGapFile, s_Line))
    {
        std::istringstream st_Stream(s_Line);
        st_Stream >> f32_X >> f32_Y;

        pst_PlanningData->st_LineGap1.arf32_D[s32_Num] = f32_X;
        pst_PlanningData->st_LineGap3.arf32_D[s32_Num] = -1 * f32_Y;
        s32_Num += 1;
    }
    pst_PlanningData->st_LineGap1.s32_Num = s32_Num;
    pst_PlanningData->st_LineGap3.s32_Num = s32_Num;
    st_PathGapFile.close();

    s32_Num = 0;
    while (std::getline(st_BoundaryGapFile, s_Line))
    {
        std::istringstream st_Stream(s_Line);
        st_Stream >> f32_X >> f32_Y >> f32_Z;

        pst_PlanningData->st_BoundaryGap1.arf32_D[s32_Num] = f32_X;
        pst_PlanningData->st_BoundaryGap2.arf32_D[s32_Num] = f32_Y;
        pst_PlanningData->st_BoundaryGap3.arf32_D[s32_Num] = f32_Z;
        s32_Num += 1;
    }
    pst_PlanningData->st_BoundaryGap1.s32_Num = s32_Num;
    pst_PlanningData->st_BoundaryGap2.s32_Num = s32_Num;
    pst_PlanningData->st_BoundaryGap3.s32_Num = s32_Num;
    st_BoundaryGapFile.close();

    // SEPath Gap Load
    s32_Num = 0;
    while (std::getline(st_SEPathGapFile, s_Line))
    {
        std::istringstream st_Stream(s_Line);
        st_Stream >> f32_X >> f32_Y;

        pst_PlanningData->st_SELineGap1.arf32_D[s32_Num] = f32_X;
        pst_PlanningData->st_SELineGap3.arf32_D[s32_Num] = -1 * f32_Y;
        s32_Num += 1;
    }
    pst_PlanningData->st_SELineGap1.s32_Num = s32_Num;
    pst_PlanningData->st_SELineGap3.s32_Num = s32_Num;
    st_SEPathGapFile.close();

    s32_Num = 0;
    while (std::getline(st_SEBoundaryGapFile, s_Line))
    {
        std::istringstream st_Stream(s_Line);
        st_Stream >> f32_X >> f32_Y >> f32_Z;

        pst_PlanningData->st_SEBoundaryGap1.arf32_D[s32_Num] = f32_X;
        pst_PlanningData->st_SEBoundaryGap2.arf32_D[s32_Num] = f32_Y;
        pst_PlanningData->st_SEBoundaryGap3.arf32_D[s32_Num] = f32_Z;
        s32_Num += 1;
    }
    pst_PlanningData->st_SEBoundaryGap1.s32_Num = s32_Num;
    pst_PlanningData->st_SEBoundaryGap2.s32_Num = s32_Num;
    pst_PlanningData->st_SEBoundaryGap3.s32_Num = s32_Num;
    st_SEBoundaryGapFile.close();

    f32_LabCheckX1 = pst_PlanningData->st_ReferenceLine2_global.arf32_X[pst_PlanningData->st_ReferenceLine2_global.s32_Num-5];
    f32_LabCheckY1 = pst_PlanningData->st_ReferenceLine2_global.arf32_Y[pst_PlanningData->st_ReferenceLine2_global.s32_Num-5];
    f32_LabCheckX2 = pst_PlanningData->st_ReferenceLine2_global.arf32_X[5];
    f32_LabCheckY2 = pst_PlanningData->st_ReferenceLine2_global.arf32_Y[5];


    if (f_File == NULL)
    {
        f_File = fopen("RoadData.bin", "wb"); 
        CalcSplineCourse(pst_PlanningData, 0.4, 2);
        CalcSplineCourse(pst_PlanningData, 0.4, 4);
        fwrite(pst_PlanningData, sizeof(PLANNING_DATA_t), 1, f_File);
    }
    else
    {
        fread(pst_PlanningData, sizeof(PLANNING_DATA_t), 1, f_File);
    }
    fclose(f_File);
}


void CalcPathYaw(PLANNING_DATA_t *pst_PlanningData)
{
    int32_t s32_I;
    for(s32_I = 0;s32_I < pst_PlanningData->st_ReferenceLine1_global.s32_Num;s32_I++)
    {
        float32_t f32_DX = pst_PlanningData->st_ReferenceLine1_global.arf32_X[(s32_I + 1) % pst_PlanningData->st_ReferenceLine1_global.s32_Num] - pst_PlanningData->st_ReferenceLine1_global.arf32_X[s32_I]; 
        float32_t f32_DY = pst_PlanningData->st_ReferenceLine1_global.arf32_Y[(s32_I + 1) % pst_PlanningData->st_ReferenceLine1_global.s32_Num] - pst_PlanningData->st_ReferenceLine1_global.arf32_Y[s32_I]; 
        pst_PlanningData->st_ReferenceLine1_global.arf32_Yaw_rad_ENU[s32_I] = atan2f(f32_DY, f32_DX);  
        pst_PlanningData->st_ReferenceLine1_global.arf32_Yaw_rad_NED[s32_I] = axisRotate(pst_PlanningData->st_ReferenceLine1_global.arf32_Yaw_rad_ENU[s32_I]);  
    }

    for(s32_I = 0;s32_I < pst_PlanningData->st_ReferenceLine2_global.s32_Num;s32_I++)
    {
        float32_t f32_DX = pst_PlanningData->st_ReferenceLine2_global.arf32_X[(s32_I + 1) % pst_PlanningData->st_ReferenceLine2_global.s32_Num] - pst_PlanningData->st_ReferenceLine2_global.arf32_X[s32_I]; 
        float32_t f32_DY = pst_PlanningData->st_ReferenceLine2_global.arf32_Y[(s32_I + 1) % pst_PlanningData->st_ReferenceLine2_global.s32_Num] - pst_PlanningData->st_ReferenceLine2_global.arf32_Y[s32_I]; 
        pst_PlanningData->st_ReferenceLine2_global.arf32_Yaw_rad_ENU[s32_I] = atan2f(f32_DY, f32_DX);  
        pst_PlanningData->st_ReferenceLine2_global.arf32_Yaw_rad_NED[s32_I] = axisRotate(pst_PlanningData->st_ReferenceLine2_global.arf32_Yaw_rad_ENU[s32_I]);  
    }

    for(s32_I = 0;s32_I < pst_PlanningData->st_ReferenceLine3_global.s32_Num;s32_I++)
    {
        float32_t f32_DX = pst_PlanningData->st_ReferenceLine3_global.arf32_X[(s32_I + 1) % pst_PlanningData->st_ReferenceLine3_global.s32_Num] - pst_PlanningData->st_ReferenceLine3_global.arf32_X[s32_I]; 
        float32_t f32_DY = pst_PlanningData->st_ReferenceLine3_global.arf32_Y[(s32_I + 1) % pst_PlanningData->st_ReferenceLine3_global.s32_Num] - pst_PlanningData->st_ReferenceLine3_global.arf32_Y[s32_I]; 
        pst_PlanningData->st_ReferenceLine3_global.arf32_Yaw_rad_ENU[s32_I] = atan2f(f32_DY, f32_DX);  
        pst_PlanningData->st_ReferenceLine3_global.arf32_Yaw_rad_NED[s32_I] = axisRotate(pst_PlanningData->st_ReferenceLine3_global.arf32_Yaw_rad_ENU[s32_I]);  
    }
}
