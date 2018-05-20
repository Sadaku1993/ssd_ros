#include<ros/ros.h>
#include<iostream>
#include<Eigen/Dense>
#include<Eigen/Geometry>

using namespace std;
using namespace Eigen;

#define PRINT_MAT(x) cout<< #x << ":\n" << x << endl << endl

void sample1()
{
    cout<<"====1. 2次元空間における回転行列を使用した点の回転====="<<endl;
    //変換したい点を作成
    VectorXf point_in(2), point_out(2);
    point_in<<1,1;
    PRINT_MAT(point_in);

    // 回転行列の作成
    Matrix2f rot;
    rot = Rotation2Df(M_PI/2);
    PRINT_MAT(rot);

    // 回転行列をかけて回転
    point_out = rot*point_in;
    PRINT_MAT(point_out);
}

void sample2()
{
    cout<<"====2. 3次元空間における軸回転関数を使用した点の回転====="<<endl;
    //変換したい点を作成
    Vector3f point_in_3d(3), point_out_3d(3);
    point_in_3d<<2,0,0; //[x y z]
    PRINT_MAT(point_in_3d);
    cout<<"range:"<<point_in_3d.norm()<<endl;

    // ある軸に関する回転行列を作成
    Matrix3f AxisAngle;
    Vector3f axis;
    axis << 0,1,1;
    AxisAngle = AngleAxisf(M_PI/2, axis);
    PRINT_MAT(AxisAngle);
    
    // 回転行列をかけて回転
    point_out_3d = AxisAngle*point_in_3d;
    PRINT_MAT(point_out_3d);
    cout<<"range : "<<point_out_3d.norm()<<endl;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv,"eigen");

    cout<<"Eigen Geometry Sample Code"<<endl;

    sample1();
    sample2();

    return 0;

}
