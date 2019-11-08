#include <iostream>
#include <random>
#include "backend/problem.h"
#include <fstream>

using namespace myslam::backend;
using namespace std;

// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class CurveFittingVertex: public Vertex
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CurveFittingVertex(): Vertex(3) {}  // abc: 三个参数， Vertex 是 3 维的
    virtual std::string TypeInfo() const { return "abc"; }
};

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge: public Edge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge( double x, double y ): Edge(1,1, std::vector<std::string>{"abc"}) {
        x_ = x;
        y_ = y;
    }
    // 计算曲线模型误差
    virtual void ComputeResidual() override
    {
        Vec3 abc = verticies_[0]->Parameters();  // 估计的参数
        //KDQ:
        //residual_(0) = std::exp( abc(0)*x_*x_ + abc(1)*x_ + abc(2) ) - y_;  // 构建残差
        residual_(0) = ( abc(0)*x_*x_ + abc(1)*x_ + abc(2) ) - y_; 
    }

    // 计算残差对变量的雅克比
    virtual void ComputeJacobians() override
    {
        Vec3 abc = verticies_[0]->Parameters();
        
        //KDQ:
        //double exp_y = std::exp( abc(0)*x_*x_ + abc(1)*x_ + abc(2) );
        Eigen::Matrix<double, 1, 3> jaco_abc;  // 误差为1维，状态量 3 个，所以是 1x3 的雅克比矩阵
        //jaco_abc << x_ * x_ * exp_y, x_ * exp_y , 1 * exp_y;
        jaco_abc << x_ * x_ , x_, 1;
        jacobians_[0] = jaco_abc;
    }
    /// 返回边的类型信息
    virtual std::string TypeInfo() const override { return "CurveFittingEdge"; }
public:
    double x_,y_;  // x 值， y 值为 _measurement
};

int main(int argc,char** argv)
{
    double a=1.0, b=2.0, c=1.0;         // 真实参数值
    int N = 2000;                          // 数据点
    //double w_sigma= 1.;                 // 噪声Sigma值
    //KDQ:
    double w_sigma = 2;
    std::ofstream info_model;
    
    std::default_random_engine generator;
    std::normal_distribution<double> noise(0.,w_sigma);

    // 构建 problem
    if (argc !=2)
    {
        printf("Please input file path for recording information of optimazition!\n");
        return -1;
    }
    std::string  file_path = argv[1];
    std::string model_name = file_path + "info_model.cvs";
    printf("model_name : %s\n",model_name.c_str());
    info_model.open(model_name);
    Problem problem(Problem::ProblemType::GENERIC_PROBLEM,file_path);
    shared_ptr< CurveFittingVertex > vertex(new CurveFittingVertex());

    // 设定待估计参数 a, b, c初始值
    vertex->SetParameters(Eigen::Vector3d (0.,0.,0.));
    // 将待估计的参数加入最小二乘问题
    problem.AddVertex(vertex);

    // 构造 N 次观测
    for (int i = 0; i < N; ++i) {

        double x = i/200.;
        double n = noise(generator);
        // 观测 y
        //KDQ:
        //double true_value = std::exp( a*x*x + b*x + c );
        double true_value = a*x*x + b*x + c; 
        double y  = true_value + n;
        info_model << x << "," << true_value << "," << y << std::endl;


        // 每个观测对应的残差函数
        shared_ptr< CurveFittingEdge > edge(new CurveFittingEdge(x,y));
        std::vector<std::shared_ptr<Vertex>> edge_vertex;
        edge_vertex.push_back(vertex);
        edge->SetVertex(edge_vertex);

        // 把这个残差添加到最小二乘问题
        problem.AddEdge(edge);
    }

    std::cout<<"\nTest CurveFitting start..."<<std::endl;
    /// 使用 LM 求解
    problem.Solve(30);

    std::cout << "-------After optimization, we got these parameters :" << std::endl;
    std::cout << vertex->Parameters().transpose() << std::endl;
    std::cout << "-------ground truth: " << std::endl;
    std::cout << "1.0,  2.0,  1.0" << std::endl;
  
    info_model.close();
    // std
    return 0;
}


