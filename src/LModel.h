#pragma once
/*Ҫ�ṩ��³���Ľӿ�*/

//��ȡtxt�ĵ�
bool readConfig(char * filename); 

//�˶�ģ��
//q0 �ǳ�ʼ��̬
//q1�ǽ�����̬
//type ������������
double *getPath(double *q0, double *q1, int type);

//���ģ�ͣ�ԭbool������������Ҹĳ�int�͡���
int ScoutModel(double agent_x,double agent_y,double agent_yaw,double target_x,double target_y,int type);