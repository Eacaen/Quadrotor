#ifndef __FILTER_H
#define __FILTER_H

#define Q_angle 0.01      // �Ƕ��������Ŷ�
#define Q_omega 0.0003    // ���ٶ��������Ŷ�
#define R_angle 0.01      // ��������
#define  Q_gyro 0.003;

 

float First_order_filter(float Com_angle,float angle_pt,float angle,float dt);// һ�׻����㷨
float Second_order_filter(float Com2_angle,float angle_pt,float angle,float dt);// ���׻����㷨
float Kalman_filter(float Klm_angle,float angle_pt,float angle,float dt);// �������˲�
void Kalman_Filter_2_(float Gyro,float Accel,float dt);
#endif
