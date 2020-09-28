//
// Created by ch on 2020/9/25.
//
#include <iostream>
#include <Eigen/Core>
#include <DeadReckoning.h>

using namespace std;

int main(int argv, char** argc) {
//    c* tryc = new c();
//    tryc->printf();

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
    std::cout << "encoder_data = {10,10,10,10,10},G0:\n";
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
    std::cout << "encoder_data = {20,30,40,50,60},G6:\n";
    std::cout << T_w_0 << "\n" << "\n";
    std::cout << T_w_6 << "\n" << "\n";
    std::cout << z_G0 << "\n" << "\n";
    std::cout << z_G6 << "\n" << "\n";

    //8.再次交换固定端
    encoder_data[0] = 10;
    encoder_data[1] = 10;
    encoder_data[2] = 10;
    encoder_data[3] = 10;
    encoder_data[4] = 10;
    dr->SetEncoderData(encoder_data);
    dr->SetGrasperStatus(DeadReckoning::JustG0);

    //9.重新获取活动端姿态与Z轴数据
    T_w_0 = dr->GetTransformOfWorldFrame(DeadReckoning::G0);
    T_w_6 = dr->GetTransformOfWorldFrame(DeadReckoning::G6);
    z_G0 = dr->GetZCoordinate(DeadReckoning::G0);
    z_G6 = dr->GetZCoordinate(DeadReckoning::G6);

    //10.输出
    std::cout << "encoder_data = {10,10,10,10,10},G0:\n";
    std::cout << T_w_0 << "\n" << "\n";
    std::cout << T_w_6 << "\n" << "\n";
    std::cout << z_G0 << "\n" << "\n";
    std::cout << z_G6 << "\n" << "\n";
}