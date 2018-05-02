#ifndef MYSTRUCT_H
#define MYSTRUCT_H

#include "reeds_shepp.h"
#include  <vector>
struct MyStruct2
{
    double x[10000];
    double y[10000];
    double w[10000];
    size_t size = 0;
};

struct MyDubinsSturct
{
	double x[10000];
	double y[10000];
	double w[10000];
	size_t size = 0;
};

inline int myDubinsTest(double *q, double nothing, void *user_data)
{
	auto wtf = reinterpret_cast<MyDubinsSturct*>(user_data);
	wtf->size += 1;
	wtf->x[wtf->size - 1] = q[0];
	wtf->y[wtf->size - 1] = q[1];
	wtf->w[wtf->size - 1] = q[2];
	return 0;
}

inline int myTest2(double *q, void *user_data)
{
    auto wtf = reinterpret_cast<MyStruct2*>(user_data);
    wtf->size += 1;
    wtf->x[wtf->size - 1] = q[0];
    wtf->y[wtf->size - 1] = q[1];
    wtf->w[wtf->size - 1] = q[2];
    return -1;
}
//����ȫ�ֱ���
extern std::vector<double> vTurnRad;    //�˶��뾶
extern std::vector<double> vTurnVel;    //�˶��ٶ�
extern std::vector<double> vScoutRad;   //���뾶
extern std::vector<double> vScoutAngle; //��컡��
#endif // MYSTRUCT_H
