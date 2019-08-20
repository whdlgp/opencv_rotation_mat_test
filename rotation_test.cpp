#include <iostream>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>

#include <fstream>

using namespace std;
using namespace cv;

#define RAD(x) ((x) * M_PI / 180.0)
#define DEGREE(x) ((x) * 180.0 / M_PI)

// from: https://www.learnopencv.com/rotation-matrix-to-euler-angles/
Vec3f rot2euler(Mat &R)
{
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6; // If
    float x, y, z;

    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }

    return Vec3f(x, y, z);
}

// From OpenCV example utils.hpp code
// Calculates rotation matrix given euler angles.
Mat eular2rot(Vec3f theta)
{
    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(theta[0]),   -sin(theta[0]),
               0,       sin(theta[0]),   cos(theta[0])
               );
     
    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3,3) <<
               cos(theta[1]),    0,      sin(theta[1]),
               0,               1,      0,
               -sin(theta[1]),   0,      cos(theta[1])
               );
     
    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3,3) <<
               cos(theta[2]),    -sin(theta[2]),      0,
               sin(theta[2]),    cos(theta[2]),       0,
               0,               0,                  1);
     
     
    // Combined rotation matrix
    Mat R = R_z * R_y * R_x;
     
    return R;
}

// from: https://www.learnopencv.com/rotation-matrix-to-euler-angles/
Vec3d rot2euler_double(Mat &R)
{
    double sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6; // If
    double x, y, z;

    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }

    return Vec3d(x, y, z);
}

// From OpenCV example utils.hpp code
// Calculates rotation matrix given euler angles.
Mat eular2rot_double(Vec3d theta)
{
    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(theta[0]),   -sin(theta[0]),
               0,       sin(theta[0]),   cos(theta[0])
               );
     
    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3,3) <<
               cos(theta[1]),    0,      sin(theta[1]),
               0,               1,      0,
               -sin(theta[1]),   0,      cos(theta[1])
               );
     
    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3,3) <<
               cos(theta[2]),    -sin(theta[2]),      0,
               sin(theta[2]),    cos(theta[2]),       0,
               0,               0,                  1);
     
     
    // Combined rotation matrix
    Mat R = R_z * R_y * R_x;
     
    return R;
}

int main(int argc, char** argv)
{
    int test_angle = 30;

    ofstream out("result.txt");

    for(int i = 0; i <= test_angle; i++)
    {
        for(int j = 0; j <= test_angle; j++)
        {
            for(int k = 0; k <= test_angle; k++)
            {
                Vec3d rot_vec_deg(i, j, k);

                Vec3d rot_vec(RAD(rot_vec_deg[0]), RAD(rot_vec_deg[1]), RAD(rot_vec_deg[2]));

                Mat rot_mat = eular2rot(rot_vec);
                Vec3d rot_vec_2_rad = rot2euler(rot_mat);
                Vec3d rot_vec_2(DEGREE(rot_vec_2_rad[0]), DEGREE(rot_vec_2_rad[1]), DEGREE(rot_vec_2_rad[2]));

                Mat rot_mat_double = eular2rot_double(rot_vec);
                Vec3d rot_vec_2_rad_double = rot2euler_double(rot_mat_double);
                Vec3d rot_vec_2_double(DEGREE(rot_vec_2_rad_double[0]), DEGREE(rot_vec_2_rad_double[1]), DEGREE(rot_vec_2_rad_double[2]));

                Mat rot_mat_inv = rot_mat.inv();
                Mat rot_mat_tran = rot_mat.t();
                Vec3d rot_vec_inv_rad = rot2euler(rot_mat_inv);
                Vec3d rot_vec_inv(DEGREE(rot_vec_inv_rad[0]), DEGREE(rot_vec_inv_rad[1]), DEGREE(rot_vec_inv_rad[2]));
                Vec3d rot_vec_tran_rad = rot2euler(rot_mat_tran);
                Vec3d rot_vec_tran(DEGREE(rot_vec_tran_rad[0]), DEGREE(rot_vec_tran_rad[1]), DEGREE(rot_vec_tran_rad[2]));

                Mat rot_mat_double_inv = rot_mat_double.inv();
                Mat rot_mat_double_tran = rot_mat_double.t();
                Vec3d rot_vec_double_inv_rad = rot2euler_double(rot_mat_double_inv);
                Vec3d rot_vec_double_inv(DEGREE(rot_vec_double_inv_rad[0]), DEGREE(rot_vec_double_inv_rad[1]), DEGREE(rot_vec_double_inv_rad[2]));
                Vec3d rot_vec_double_tran_rad = rot2euler_double(rot_mat_double_tran);
                Vec3d rot_vec_double_tran(DEGREE(rot_vec_double_tran_rad[0]), DEGREE(rot_vec_double_tran_rad[1]), DEGREE(rot_vec_double_tran_rad[2]));

                Mat rot_mat_double_inv_inv = rot_mat_double_inv.inv();
                Mat rot_mat_double_tran_tran = rot_mat_double_tran.t();
                Vec3d rot_vec_double_inv_inv_rad = rot2euler_double(rot_mat_double_inv_inv);
                Vec3d rot_vec_double_inv_inv(DEGREE(rot_vec_double_inv_inv_rad[0]), DEGREE(rot_vec_double_inv_inv_rad[1]), DEGREE(rot_vec_double_inv_inv_rad[2]));
                Vec3d rot_vec_double_tran_tran_rad = rot2euler_double(rot_mat_double_tran_tran);
                Vec3d rot_vec_double_tran_tran(DEGREE(rot_vec_double_tran_tran_rad[0]), DEGREE(rot_vec_double_tran_tran_rad[1]), DEGREE(rot_vec_double_tran_tran_rad[2]));

                Mat rot_mat_rodrig, rot_mat_rodrig_inv;
                Mat rot_vec_mat_inv;
                Rodrigues(rot_vec, rot_mat_rodrig);
                rot_mat_rodrig_inv = rot_mat_rodrig.t();
                Rodrigues(rot_mat_rodrig_inv, rot_vec_mat_inv);
                double* rot_vec_mat_inv_data = (double*)rot_vec_mat_inv.data;
                Vec3d rot_vec_inv_rodrig(rot_vec_mat_inv_data[0], rot_vec_mat_inv_data[1], rot_vec_mat_inv_data[2]);
                Vec3d rot_vec_inv_rodrig_deg(DEGREE(rot_vec_inv_rodrig[0]), DEGREE(rot_vec_inv_rodrig[1]), DEGREE(rot_vec_inv_rodrig[2]));
                
                Vec3d diff1 = rot_vec_deg - rot_vec_2;
                Vec3d diff2 = rot_vec_deg - rot_vec_2_double;
                Vec3d diff3 = rot_vec_deg + rot_vec_inv;
                Vec3d diff4 = rot_vec_deg + rot_vec_tran;
                Vec3d diff5 = rot_vec_deg + rot_vec_double_inv;
                Vec3d diff6 = rot_vec_deg + rot_vec_double_tran;
                Vec3d diff7 = rot_vec_deg - rot_vec_double_inv_inv;
                Vec3d diff8 = rot_vec_deg - rot_vec_double_tran_tran;
                Vec3d diff9 = rot_vec_deg + rot_vec_inv_rodrig_deg;

                double diff1_norm = sqrt(diff1[0]*diff1[0] + diff1[1]*diff1[1] + diff1[2]*diff1[2]);
                double diff2_norm = sqrt(diff2[0]*diff2[0] + diff2[1]*diff2[1] + diff2[2]*diff2[2]);
                double diff3_norm = sqrt(diff3[0]*diff3[0] + diff3[1]*diff3[1] + diff3[2]*diff3[2]);
                double diff4_norm = sqrt(diff4[0]*diff4[0] + diff4[1]*diff4[1] + diff4[2]*diff4[2]);
                double diff5_norm = sqrt(diff5[0]*diff5[0] + diff5[1]*diff5[1] + diff5[2]*diff5[2]);
                double diff6_norm = sqrt(diff6[0]*diff6[0] + diff6[1]*diff6[1] + diff6[2]*diff6[2]);
                double diff7_norm = sqrt(diff7[0]*diff7[0] + diff7[1]*diff7[1] + diff7[2]*diff7[2]);
                double diff8_norm = sqrt(diff8[0]*diff8[0] + diff8[1]*diff8[1] + diff8[2]*diff8[2]);
                double diff9_norm = sqrt(diff9[0]*diff9[0] + diff9[1]*diff9[1] + diff9[2]*diff9[2]);

                out << rot_vec_deg[0] << ',' << rot_vec_deg[1] << ',' << rot_vec_deg[2] << ','
                    << rot_vec[0] << ',' << rot_vec[1] << ',' << rot_vec[2] << ','
                    << rot_vec_2_rad[0] << ',' << rot_vec_2_rad[1] << ',' << rot_vec_2_rad[2] << ','
                    << rot_vec_2[0] << ',' << rot_vec_2[1] << ',' << rot_vec_2[2] << ','
                    << rot_vec_2_rad_double[0] << ',' << rot_vec_2_rad_double[1] << ',' << rot_vec_2_rad_double[2] << ','
                    << rot_vec_2_double[0] << ',' << rot_vec_2_double[1] << ',' << rot_vec_2_double[2] << ','
                    << rot_vec_inv[0] << ',' << rot_vec_inv[1] << ',' << rot_vec_inv[2] << ','
                    << rot_vec_tran[0] << ',' << rot_vec_tran[1] << ',' << rot_vec_tran[2] << ','
                    << rot_vec_double_inv[0] << ',' << rot_vec_double_inv[1] << ',' << rot_vec_double_inv[2] << ','
                    << rot_vec_double_tran[0] << ',' << rot_vec_double_tran[1] << ',' << rot_vec_double_tran[2] << ','
                    << rot_vec_double_inv_inv[0] << ',' << rot_vec_double_inv_inv[1] << ',' << rot_vec_double_inv_inv[2] << ','
                    << rot_vec_double_tran_tran[0] << ',' << rot_vec_double_tran_tran[1] << ',' << rot_vec_double_tran_tran[2] << ','
                    << rot_vec_inv_rodrig_deg[0] << ',' << rot_vec_inv_rodrig_deg[1] << ',' << rot_vec_inv_rodrig_deg[2] << ','
                    << diff1_norm << ',' << diff2_norm << ',' << diff3_norm << ',' << diff4_norm << ',' << diff5_norm << ',' << diff6_norm << ',' << diff7_norm << ',' << diff8_norm << ',' << diff9_norm
                    << endl;
            }
        }
    }

    out.close();
}
