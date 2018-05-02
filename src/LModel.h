#pragma once
/*要提供给鲁塞的接口*/

//读取txt文档
bool readConfig(char * filename); 

//运动模型
//q0 是初始姿态
//q1是结束姿态
//type 是智能体类型
double *getPath(double *q0, double *q1, int type);

//侦察模型（原bool类型输出，被我改成int型。）
int ScoutModel(double agent_x,double agent_y,double agent_yaw,double target_x,double target_y,int type);