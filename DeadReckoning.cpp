//
// Created by ch on 2020/9/25.
//

#include "DeadReckoning.h"
#include <iostream>

DeadReckoning::DeadReckoning(const DeadReckoning::Transform &T, GrasperName which_end) :
    _T_w_0_current(Eigen::Matrix<double, 4, 4>::Identity()),
    _T_w_6_current(_T_w_0_current),
    _T_w_0_last(_T_w_0_current),
    _T_w_6_last(_T_w_0_current),
    _encoder_data(EncoderDataType()),
    //        _grasper_status(JustG0),
    _kinematics(new ClimberKinematics5D),
    _is_encoder_data_updated(true) {
    //设置夹紧端,并根据夹紧端设置初始姿态
    if (which_end == G0) {
        _grasper_status = JustG0;
        _T_w_6_current = T;
    } else if (which_end == G6) {
        _grasper_status = JustG6;
        _T_w_0_current = T;
    }
}

void DeadReckoning::SetEncoderData(const DeadReckoning::EncoderDataType &encoder_data) {
    _encoder_data = encoder_data;
    _is_encoder_data_updated = true;
}

DeadReckoning::GrasperStatus DeadReckoning::GetGrasperStatus() {
    return _grasper_status;
}

const DeadReckoning::Transform &DeadReckoning::GetTransformOfWorldFrame(DeadReckoning::GrasperName which_end) {
    _UpdateTransform();//判断编码器数据是否有更新，如有则重新计算变换矩阵，如无则直接返回之前的结果
    if (which_end == G0) {
        return _T_w_0_current;
    } else if (which_end == G6) {
        return _T_w_6_current;
    }
}

DeadReckoning::DeadReckoning()  :
        _T_w_0_current(Transform()),
        _T_w_6_current(Transform()),
        _T_w_0_last(Transform()),
        _T_w_6_last(Transform()),
        _encoder_data(EncoderDataType()),
//        _grasper_status(JustG0),
        _kinematics(new ClimberKinematics5D),
    _is_encoder_data_updated(true) {
}

void DeadReckoning::_UpdateTransform() {
    //判断编码器数据是否有更新，如有则重新计算变换矩阵
    if (_is_encoder_data_updated) {
        _is_encoder_data_updated = false;
        auto T_0_6 = _kinematics->GetTransformMatrix(ClimbotKinematics::G6, _encoder_data);
        if (_grasper_status == JustG0) { //@todo:两端抓紧与全部松开应怎么处理？
            _T_w_6_current = _T_w_0_last * T_0_6;
            _T_w_0_current = _T_w_6_current * T_0_6.inverse();
        } else if (_grasper_status == JustG6) {
            _T_w_0_current = _T_w_6_last * T_0_6.inverse();
            _T_w_6_current = _T_w_0_current * T_0_6;
        }
        _T_w_0_last = _T_w_0_current;
        _T_w_6_last = _T_w_6_current;
    }
}

DeadReckoning::ZCoordinateDataType DeadReckoning::GetZCoordinate(DeadReckoning::GrasperName which_end) {
    if (which_end == G0) {
        return _T_w_0_current.block<3,1>(0,3);
    } else if (which_end == G6) {
        return _T_w_6_current.block<3,1>(0,3);
    }
}

void DeadReckoning::SetGrasperStatus(DeadReckoning::GrasperStatus grasper_status) {
    _grasper_status = grasper_status;
}

