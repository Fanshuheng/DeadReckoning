//
// Created by ch on 2020/9/27.
//

#include "ClimberKinematics5D.h"

ClimbotKinematics::Transform
ClimberKinematics5D::ForwardKinematics(const ClimbotKinematics::EncoderDataType &encoder_data) {
    return ClimbotKinematics::Transform();
}

ClimbotKinematics::EncoderDataType
ClimberKinematics5D::ReverseKinematics(const ClimbotKinematics::Transform &T_Grasper1_Grasper2) {
    return ClimbotKinematics::EncoderDataType();
}

ClimbotKinematics::Transform ClimberKinematics5D::GetTransformMatrix(JointIndex joint_index, const EncoderDataType& encoder_data) {
    ClimbotKinematics::EncoderDataType joint;
    joint.resize(5);
    for (int i = 0; i < 5; i++) {
        joint[i] = encoder_data[i] * M_PI / 180;
    }
    _T_b_1 << cos(joint[0]), -sin(joint[0]), 0, 0, sin(joint[0]), cos(joint[0]), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    _T_1_2 << cos(joint[1]), -sin(joint[1]), 0, 0, 0, 0, -1, 0, sin(joint[1]), cos(joint[1]), 0, _links_length[0], 0, 0, 0, 1;
    _T_2_3 << cos(joint[2]), -sin(joint[2]), 0, _links_length[1], sin(joint[2]), cos(joint[2]), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    _T_3_4 << cos(joint[3]), -sin(joint[3]), 0, _links_length[2], sin(joint[3]), cos(joint[3]), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    _T_4_5 << cos(joint[4]), -sin(joint[4]), 0, 0, 0, 0, -1, -_links_length[3], sin(joint[4]), cos(joint[4]), 0, 0, 0, 0, 0, 1;

    switch (joint_index) {
        case I1: return _T_b_1;
        case T2: return _T_b_1 * _T_1_2;
        case T3: return _T_b_1 * _T_1_2 * _T_2_3;
        case T4: return _T_b_1 * _T_1_2 * _T_2_3 * _T_3_4;
        case I5: return _T_b_1 * _T_1_2 * _T_2_3 * _T_3_4 * _T_4_5;
        case G6: return _T_b_1 * _T_1_2 * _T_2_3 * _T_3_4 * _T_4_5;
    }
}

ClimberKinematics5D::ClimberKinematics5D() {
    //Len=[329,293.2,293.2,329];
    //机器人连杆长度初始化
    _links_length.push_back(329);
    _links_length.push_back(293.2);
    _links_length.push_back(293.2);
    _links_length.push_back(329);
}
