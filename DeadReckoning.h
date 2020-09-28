//
// Created by ch on 2020/9/25.
//

#ifndef DEAD_RECKONING_DEADRECKONING_H
#define DEAD_RECKONING_DEADRECKONING_H
#include <Eigen/Core>
#include <utility>
#include <ClimberKinematics5D.h>
#include <Eigen/Dense>
/*use case
    //1.初始化，从marker中获得初始位姿，赋给G0
    DeadReckoning::Transform T_w_0_init;
    T_w_0_init << 1,0,0,0,
                  0,1,0,0,
                  0,0,1,0,
                  0,0,0,1;
    DeadReckoning* dr = new DeadReckoning(T_w_0_init, DeadReckoning::G0);

    //2.给DR传入当前编码器的值以及是哪端夹紧
    DeadReckoning::EncoderDataType encoder_data = {10,10,10,10,10};
    dr->SetEncoderData(encoder_data);
    dr->SetGrasperStatus(DeadReckoning::JustG0);

    //3.DR给出两端的在世界坐标系下的位姿矩阵、Z轴坐标
    DeadReckoning::Transform T_w_0, T_w_6;
    T_w_0 = dr->GetTransformOfWorldFrame(DeadReckoning::G0);
    T_w_6 = dr->GetTransformOfWorldFrame(DeadReckoning::G6);
    auto z_G0 = dr->GetZCoordinate(DeadReckoning::G0);
    auto z_G6 = dr->GetZCoordinate(DeadReckoning::G6);

    //4.输出
    std::cout << "encoder_data = {10,10,10,10,10}:\n";
    std::cout << T_w_0 << "\n" << "\n";
    std::cout << T_w_6 << "\n" << "\n";
    std::cout << z_G0 << "\n" << "\n";
    std::cout << z_G6 << "\n" << "\n";

    //5.交换固定端
    encoder_data[0] = 20;
    encoder_data[1] = 30;
    encoder_data[2] = 40;
    encoder_data[3] = 50;
    encoder_data[4] = 60;
    dr->SetEncoderData(encoder_data);
    dr->SetGrasperStatus(DeadReckoning::JustG6);

    //6.重新获取活动端姿态与Z轴数据
    T_w_0 = dr->GetTransformOfWorldFrame(DeadReckoning::G0);
    T_w_6 = dr->GetTransformOfWorldFrame(DeadReckoning::G6);
    z_G0 = dr->GetZCoordinate(DeadReckoning::G0);
    z_G6 = dr->GetZCoordinate(DeadReckoning::G6);

    //7.输出
    std::cout << "encoder_data = {20,30,40,50,60}:\n";
    std::cout << T_w_0 << "\n" << "\n";
    std::cout << T_w_6 << "\n" << "\n";
    std::cout << z_G0 << "\n" << "\n";
    std::cout << z_G6 << "\n" << "\n";
 * */
class DeadReckoning {
public://typedef
    typedef Eigen::Matrix4d Transform;//位姿矩阵
    typedef std::vector<double> EncoderDataType;//编码器数据，单位°
    typedef Eigen::Vector3d ZCoordinateDataType;//Z轴数据类型
    typedef std::vector<std::pair<Transform, Transform>> KeyFrameTransforms;//关键帧的位姿，pair的左边是G0的，pair的右边是G6的
    enum GrasperName{G0 = 0, G6 = 6};//夹持器的代号
    enum GrasperStatus{G0AndG6 = 0, JustG0 = 1, JustG6 = 2, None = 3};//夹持器抓紧状态的代号

public://APIs

    DeadReckoning(const Transform& T, GrasperName which_end);

    void SetEncoderData(const EncoderDataType& encoder_data);

    void SetGrasperStatus(GrasperStatus grasper_status);

    GrasperStatus GetGrasperStatus();

    const Transform& GetTransformOfWorldFrame(GrasperName which_end);

    ZCoordinateDataType GetZCoordinate(GrasperName which_end);

private:
    DeadReckoning();
    void _UpdateTransform();

private://private data
    Transform _T_w_0_current, _T_w_6_current;
    Transform _T_w_0_last, _T_w_6_last;
    EncoderDataType _encoder_data;
    GrasperStatus _grasper_status;//夹持器夹紧状态
    ClimbotKinematics* _kinematics;//运动学

    /*
     * 编码器数据是否更新
     * true:调用GetTransformOfWorldFrame或GetZCoordinate时会先重新计算T_w_0_current，T_w_6_current，T_w_0_last，T_w_6_last；
     * false；直接返回T_w_0_current，T_w_6_current
     * */
    bool _is_encoder_data_updated;

};


#endif //DEAD_RECKONING_DEADRECKONING_H
