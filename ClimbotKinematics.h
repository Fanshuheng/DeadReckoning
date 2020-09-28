//
// Created by ch on 2020/9/27.
//

#ifndef DEAD_RECKONING_CLIMBOTKINEMATICS_H
#define DEAD_RECKONING_CLIMBOTKINEMATICS_H
#include <Eigen/Core>
#include <vector>

class ClimbotKinematics {
public:
    typedef Eigen::Matrix4d Transform;
    typedef std::vector<double> EncoderDataType;
    typedef std::vector<double> LinkLengthType;
    typedef enum JointIndex{I1, T2, T3, T4, I5, G6};
public:
    ClimbotKinematics() = default;

    /**
     * @brief 获取T_grasper0_joint
     * @param joint    机器人关节号引索
     * @param encoder_data 编码器数据，单位为°
     * @return 获取T_grasper0_joint
     */
    virtual Transform GetTransformMatrix(JointIndex joint_index, const EncoderDataType& encoder_data) = 0;

    /**
     * @brief 获取从夹持器0到另一个夹持器的转换矩阵T_Grasper1_Grasper2
     * @param encoder_data 编码器数据，单位为°
     * @return T_Grasper1_Grasper2
     */
    virtual Transform ForwardKinematics(const EncoderDataType& encoder_data) = 0;

    /**
     * @brief 从T_Grasper1_Grasper2解算出各关节角角度大小
     * @param T_Grasper1_Grasper2
     * @return 各关节角角度大小
     */
    virtual EncoderDataType ReverseKinematics(const Transform& T_Grasper1_Grasper2) = 0;

protected:
    EncoderDataType _encoder_data;
    LinkLengthType _links_length;
    Eigen::Matrix4d _T_b_1, _T_1_2, _T_2_3, _T_3_4, _T_4_5;
};


#endif //DEAD_RECKONING_CLIMBOTKINEMATICS_H
