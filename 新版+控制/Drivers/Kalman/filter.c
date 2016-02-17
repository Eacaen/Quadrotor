#include "filter.h"

unsigned char Filter_mode=1;
// һ�׻����㷨
float First_order_filter(float Com_angle,float angle_pt,float angle,float dt)
{
    float A,K = 0.075;                             // �Լ��ٶȼ�ȡֵ��Ȩ��
    A = K / (K + dt);
    Com_angle = A * (Com_angle + angle_pt * dt) + (1-A) * angle;
	return Com_angle;
}
// ���׻����㷨
float Second_order_filter(float Com2_angle,float angle_pt,float angle,float dt)
{   
    float x1,x2,K = 0.5;
	static float y1;
    x1 = (angle - Com2_angle) * K * K;
    y1 = y1 + x1 * dt;
    x2 = y1 + 2 * K *(angle - Com2_angle) + angle_pt;
    Com2_angle = Com2_angle + x2 * dt;
	return Com2_angle;
}
// �������˲�
float Kalman_filter_1_(float Klm_angle,float angle_pt,float angle,float dt)
{
	static float bias;
	static float P_00,P_01,P_10,P_11;
	float K_0,K_1;
    Klm_angle += (angle_pt - bias) * dt;       // �������
    P_00 += -(P_10 + P_01) * dt + Q_angle *dt;
    P_01 += -P_11 * dt;
    P_10 += -P_11 * dt;
    P_11 += +Q_omega * dt;                     // ����������Э����
    
    K_0 = P_00 / (P_00 + R_angle);
    K_1 = P_10 / (P_00 + R_angle);
    
    bias += K_1 * (angle - Klm_angle);
    Klm_angle += K_0 * (angle - Klm_angle);    // �������
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;                        // ����������Э����
	return Klm_angle;
}



float Kalman_filter(float Klm_angle,float angle_pt,float angle,float dt)// �������˲�
{
	static float bias;
	static float P_00,P_01,P_10,P_11;
	float K_0,K_1;
    Klm_angle += (angle_pt - bias) * dt;       // �������
    P_00 += -(P_10 + P_01) * dt + Q_angle *dt;
    P_01 += -P_11 * dt;
    P_10 += -P_11 * dt;
    P_11 += +Q_omega * dt;                     // ����������Э����
    
    K_0 = P_00 / (P_00 + R_angle);
    K_1 = P_10 / (P_00 + R_angle);
    
    bias += K_1 * (angle - Klm_angle);
    Klm_angle += K_0 * (angle - Klm_angle);    // �������
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;                        // ����������Э����
	return Klm_angle;
}
