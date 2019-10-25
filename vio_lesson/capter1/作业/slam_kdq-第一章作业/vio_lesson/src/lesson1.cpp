#include <iostream>
#include <eigen3/Eigen/Dense>
#define pi 3.1415926f
using namespace  std;

int main()
{
  Eigen::Vector3d axis(1,1,1);
  axis.normalize();
  Eigen::AngleAxisd angle_axis = Eigen::AngleAxisd(pi/4,axis);
  Eigen::Matrix3d R(angle_axis);
  Eigen::Quaterniond q(angle_axis);
  Eigen::Matrix3d delta_R;
  delta_R << 1,-0.03,0.02,0.03,1,-0.01,-0.02,0.01,1;
  Eigen::Quaterniond delta_q = Eigen::Quaterniond(1,0.01/2,0.02/2,0.03/2);

  cout << "Matrix R: \n" << R << endl;
  cout << "Quaternion q: \n" << q.coeffs().transpose() << endl;
  //cout << "Quaternion q norm:\n" << q.norm() << endl;
  cout << "Delta_R:\n" << delta_R << endl;
  cout << "Delta_q:\n" << delta_q.coeffs().transpose() << endl;
  q = q*delta_q;
  R = R*delta_R;
  q.normalize();
  Eigen::Quaterniond quat(R);
  quat.normalize();

  cout << "Quaternion q:\n" << q.coeffs().transpose() << endl;
  cout << "Quaternion R2q:\n" << quat.coeffs().transpose() << endl;

  Eigen::Vector3d ypr_du1 = q.toRotationMatrix().eulerAngles(2,1,0)*180/pi;
  Eigen::Vector3d ypr_du2 = quat.toRotationMatrix().eulerAngles(2,1,0)*180/pi;
  Eigen::Vector3d delta_ypr = ypr_du1 - ypr_du2;
  cout << "Quaterion to euler angle(yaw,pitch,roll): " << ypr_du1.transpose() <<" deg" << endl;
  cout << "Matrix to euler angle(yaw,pitch,roll): " << ypr_du2.transpose()<<" deg" << endl;
  cout << "Difference between two euler angle: " << delta_ypr.transpose()<<" deg" << endl;
}
