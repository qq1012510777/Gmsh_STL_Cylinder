#include <Eigen/Dense>
#include <gmsh.h>
#include <iostream>
#include <vector>

using namespace Eigen;
using namespace std;

class QuaternionT
{
public:
    // 1,i,j,k
    Vector4d QuaternionNum;

public:
    // describe quaternion
    QuaternionT(){};

    void DescribeRotation(const Vector3d v,
                          const double angle)
    {
        double sina_2 = sin(angle * 0.5000);
        double cosa_2 = cos(angle * 0.5000);

        this->QuaternionNum[0] = cosa_2,
        this->QuaternionNum[1] = sina_2 * v[0],
        this->QuaternionNum[2] = sina_2 * v[1],
        this->QuaternionNum[3] = sina_2 * v[2];

        double norm_r = QuaternionNum.norm();

        this->QuaternionNum[0] /= norm_r;
        this->QuaternionNum[1] /= norm_r;
        this->QuaternionNum[2] /= norm_r;
        this->QuaternionNum[3] /= norm_r;
    }; // Quaternion::DescribeRotation
    // rotate

    Vector3d Rotate(const Vector3d v)
    {
        double t2 = QuaternionNum[0] * QuaternionNum[1],
               t3 = QuaternionNum[0] * QuaternionNum[2],
               t4 = QuaternionNum[0] * QuaternionNum[3],
               t5 = -QuaternionNum[1] * QuaternionNum[1],
               t6 = QuaternionNum[1] * QuaternionNum[2],
               t7 = QuaternionNum[1] * QuaternionNum[3],
               t8 = -QuaternionNum[2] * QuaternionNum[2],
               t9 = QuaternionNum[2] * QuaternionNum[3],
               t10 = -QuaternionNum[3] * QuaternionNum[3];

        Vector3d DF;

        DF[0] = 2.0 * ((t8 + t10) * v[0] + (t6 - t4) * v[1] + (t3 + t7) * v[2]) + v[0],
        DF[1] = 2.0 * ((t4 + t6) * v[0] + (t5 + t10) * v[1] + (t9 - t2) * v[2]) + v[1],
        DF[2] = 2.0 * ((t7 - t3) * v[0] + (t2 + t9) * v[1] + (t5 + t8) * v[2]) + v[2];

        return DF;
    };
};

int main()
{
    gmsh::initialize();

    int NUMpntsA = 16;
    double r = 5;
    double height = 100.0;
    double lc = 2 * r * M_PI / NUMpntsA;
    int NUM_pnts_lateral_without_ends = height / lc - 1;
    double SpacingOfLateral = height / (NUM_pnts_lateral_without_ends + 1);

    gmsh::model::occ::addPoint(0.0, 0.0, 0.0, 1);

    QuaternionT AS;
    Vector3d A__i, axis;
    A__i << r, 0.0, 0.0;
    axis << 0, 0, 1;
    double spacing_arc = 2.0 * M_PI / NUMpntsA;

    //------------ 1st circle ----------------------
    //------------ 1st circle ----------------------
    //------------ 1st circle ----------------------
    vector<int> pntTag_1st_circle(NUMpntsA);
    vector<Vector3d> pnts(NUMpntsA);

    for (int i = 0; i < NUMpntsA; ++i)
    {
        AS.DescribeRotation(axis, spacing_arc * i);
        Vector3d pnt_tmp = AS.Rotate(A__i);
        pntTag_1st_circle[i] = gmsh::model::occ::addPoint(pnt_tmp[0], pnt_tmp[1], pnt_tmp[2]);
        pnts[i] = pnt_tmp;
    }
    gmsh::model::occ::synchronize();
    vector<int> TagArcSeg_1st_circle(NUMpntsA);
    for (int i = 0; i < NUMpntsA; ++i)
        TagArcSeg_1st_circle[i] = gmsh::model::occ::addLine(pntTag_1st_circle[i], pntTag_1st_circle[(i + 1) % NUMpntsA]);
    gmsh::model::occ::synchronize();

    int Curveloop_arc_No_1st_circle = gmsh::model::occ::addCurveLoop(TagArcSeg_1st_circle);

    gmsh::model::occ::synchronize();
    int circularPlane_No_1st_circle = gmsh::model::occ::addPlaneSurface({Curveloop_arc_No_1st_circle});
    gmsh::model::occ::synchronize();

    //------------ 2nd circle ----------------------
    //------------ 2nd circle ----------------------
    //------------ 2nd circle ----------------------
    vector<int> pntTag_2nd_circle(NUMpntsA);
    for (int i = 0; i < NUMpntsA; ++i)
        pntTag_2nd_circle[i] = gmsh::model::occ::addPoint(pnts[i][0], pnts[i][1], pnts[i][2] + height);
    gmsh::model::occ::synchronize();
    vector<int> TagArcSeg_2nd_circle(NUMpntsA);
    for (int i = 0; i < NUMpntsA; ++i)
        TagArcSeg_2nd_circle[i] = gmsh::model::occ::addLine(pntTag_2nd_circle[i], pntTag_2nd_circle[(i + 1) % NUMpntsA]);
    gmsh::model::occ::synchronize();
    int Curveloop_arc_No_2nd_circle = gmsh::model::occ::addCurveLoop(TagArcSeg_2nd_circle);
    gmsh::model::occ::synchronize();
    int circularPlane_No_2nd_circle = gmsh::model::occ::addPlaneSurface({Curveloop_arc_No_2nd_circle});
    gmsh::model::occ::synchronize();

    //----------------------
    //----------------------
    //----------------------

    /// lateral surfaces -------------------
    /// lateral surfaces -------------------
    /// lateral surfaces -------------------
    vector<int> lateral_face_loopID(NUMpntsA);

    /*int NUMpntsA = 78;
    double r = 2.5;
    double height = 5.0;
    double lc = 2 * r * M_PI;
    int NUM_pnts_lateral_without_ends = height / lc - 1;
    double SpacingOfLateral = height / (NUM_pnts_lateral_without_ends + 1);*/
    for (int i = 0; i < NUMpntsA; ++i)
    {
        cout << i + 1 << " / " << NUMpntsA << endl;
        // int Line1 = gmsh::model::occ::addLine(pntTag_1st_circle[(i + 1) % NUMpntsA], pntTag_2nd_circle[(i + 1) % NUMpntsA]);
        // int Line2 = gmsh::model::occ::addLine(pntTag_2nd_circle[i], pntTag_1st_circle[i]);
        // gmsh::model::occ::synchronize();

        //------1st lateral line loop
        vector<int> Ponts_record_1(NUM_pnts_lateral_without_ends);

        for (int j = 0; j < NUM_pnts_lateral_without_ends; ++j)
        {
            Ponts_record_1[j] = gmsh::model::occ::addPoint(pnts[(i + 1) % NUMpntsA][0], pnts[(i + 1) % NUMpntsA][1],
                                                           pnts[(i + 1) % NUMpntsA][2] + (j + 1) * SpacingOfLateral);
            //cout << Ponts_record_1[j] << "\n";
        }
        gmsh::model::occ::synchronize();

        vector<int> LineLOOP_1(NUM_pnts_lateral_without_ends + 1);
        
        for (int j = 0; j < NUM_pnts_lateral_without_ends + 1; ++j)
        {
            int lineNO = 0;

            if (j == 0)
            {
                //cout << pntTag_1st_circle[(i + 1) % NUMpntsA] << ", " <<  Ponts_record_1[0] << endl;
                lineNO = gmsh::model::occ::addLine(pntTag_1st_circle[(i + 1) % NUMpntsA], Ponts_record_1[0]);
            }
            else if (j == NUM_pnts_lateral_without_ends)
            {
                //cout << Ponts_record_1[NUM_pnts_lateral_without_ends-1] << ", " <<  pntTag_2nd_circle[(i + 1) % NUMpntsA] << endl;
                lineNO = gmsh::model::occ::addLine(Ponts_record_1[NUM_pnts_lateral_without_ends - 1],
                                                   pntTag_2nd_circle[(i + 1) % NUMpntsA]);
            }
            else
            {
                //cout << Ponts_record_1[j - 1] << ", " <<  Ponts_record_1[j] << endl;

                lineNO = gmsh::model::occ::addLine(Ponts_record_1[j - 1],
                                                   Ponts_record_1[j]);
            }
            LineLOOP_1[j] = lineNO;
        }
        gmsh::model::occ::synchronize();
        
        //------2nd lateral line loop
        vector<int> Ponts_record_2(NUM_pnts_lateral_without_ends);
        for (int j = 0; j < NUM_pnts_lateral_without_ends; ++j)
            Ponts_record_2[j] = gmsh::model::occ::addPoint(pnts[i][0], pnts[i][1],
                                                           pnts[i][2] + (NUM_pnts_lateral_without_ends - j) * SpacingOfLateral);
        gmsh::model::occ::synchronize();
        vector<int> LineLOOP_2(NUM_pnts_lateral_without_ends + 1);
        for (int j = 0; j < NUM_pnts_lateral_without_ends + 1; ++j)
        {
            int lineNO = 0;

            if (j == 0)
                lineNO = gmsh::model::occ::addLine(pntTag_2nd_circle[i], Ponts_record_2[0]);
            else if (j == NUM_pnts_lateral_without_ends)
                lineNO = gmsh::model::occ::addLine(Ponts_record_2[NUM_pnts_lateral_without_ends - 1],
                                                   pntTag_1st_circle[i]);
            else
                lineNO = gmsh::model::occ::addLine(Ponts_record_2[j - 1],
                                                   Ponts_record_2[j]);
            LineLOOP_2[j] = lineNO;
        }
        gmsh::model::occ::synchronize();
        
        // gmsh::fltk::run();
        //----------------
        vector<int> Lateral_Line_loop(1);
        Lateral_Line_loop[0] = TagArcSeg_1st_circle[i];
        Lateral_Line_loop.insert(Lateral_Line_loop.end(), LineLOOP_1.begin(), LineLOOP_1.end());
        Lateral_Line_loop.push_back(-TagArcSeg_2nd_circle[i]);
        Lateral_Line_loop.insert(Lateral_Line_loop.end(), LineLOOP_2.begin(), LineLOOP_2.end());
        
        int Lateral_Line_loop_NO = gmsh::model::occ::addCurveLoop(Lateral_Line_loop);
        gmsh::model::occ::synchronize();
     
        int Lateral_face_NO = gmsh::model::occ::addPlaneSurface({Lateral_Line_loop_NO});
        gmsh::model::occ::synchronize();
        lateral_face_loopID[i] = Lateral_face_NO;
    }
    // cout << 2 << endl;
    // vector<int> LoopVolumeFaces(NUMpntsA + 2);
    // LoopVolumeFaces[0] = circularPlane_No_1st_circle;
    // LoopVolumeFaces[NUMpntsA + 1] = circularPlane_No_2nd_circle;
    // for (int i = 1; i <= NUMpntsA; ++i)
    //     LoopVolumeFaces[i] = lateral_face_loopID[i - 1];
    // cout << 3 << endl;
    // int surfaceLOOPNO = gmsh::model::occ::addSurfaceLoop(LoopVolumeFaces);
    // cout << 4 << endl;
    // gmsh::model::occ::synchronize();
    // gmsh::model::occ::addVolume({surfaceLOOPNO}, 1);
    // cout << 5 << endl;
    // gmsh::model::occ::synchronize();
   
    gmsh::model::mesh::generate(2);

    gmsh::fltk::run();
    gmsh::clear();
    gmsh::finalize();

    return 0;
}