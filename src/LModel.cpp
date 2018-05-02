//LModel:�ѱ��˱��Dubins��reedsshepp���������ı��������Ҫ����ʽ
#include <iostream>
#include <memory>
#include <vector>

#include "LModel.h"
#include "ssconfig.hpp"   //C++��ȡtxt��by�̳�˼��
#include "mystruct.h"     //������Ҫ�������ʽ��by Gao)
#include "reeds_shepp.h"  //����д��rs
#include "dubins.h"       //����д��dubins




#define PI 3.1415
using namespace std;
std::vector<double> vTurnRad; //�ȶ���Ҫʹ�õ�ȫ�ֱ���
std::vector<double> vTurnVel;
std::vector<double> vScoutRad;  
std::vector<double> vScoutAngle;

//��ȡtxt�ĵ���by�̳�˼��
bool readConfig(char * filename)  //��ͨ��
{
	sscfg::ConfigFile co_list = sscfg::ConfigFile::load(filename);
	co_list.get("rad", vTurnRad); //��txt�ĵ���"rad"��ֵ����ȫ�ֱ���vTurnRad
	co_list.get("vel", vTurnVel);       //ͬ��
	co_list.get("Srad", vScoutRad);     //ͬ��
	co_list.get("Sangle", vScoutAngle); //ͬ��
	cout << "vTurnRad size is " << vTurnRad.size() << endl;
	return false;
}

double * getPath(double * q0, double * q1, int type)
{

	switch (type) //ֻ��type=7(�ɻ���ʱʹ��Dubinsģ�ͣ�����ʹ��rsģ��
	{
	case 7:
		//Dubinsģ��
	{
		DubinsPath d_path;

		int err = dubins_init(q0, q1, vTurnRad[type], &d_path);
		MyDubinsSturct data1;
		dubins_path_sample_many(&d_path, myDubinsTest, vTurnVel[type], &data1);
		std::cout << "data1 size = " << data1.size << endl;

		double *res = new double[data1.size * 3 + 1]; //new double��ȫ�ֱ�����
		res[0] = data1.size;

		for (size_t j = 0; j < data1.size; j++)
		{
			std::cout << j << "   x = " << data1.x[j] << std::endl;
			std::cout << j << "    y = " << data1.y[j] << std::endl;
			std::cout << j << "    yaw = " << data1.w[j] << std::endl;
			//�����TXT����
			//data_matlab << "index = " << j << "    x = " << data1.x[j] << std::endl;
			//data_matlab << "index = " << j << "   y = " << data1.y[j] << std::endl;
			//data_matlab << "index = " << j << "   yaw = " << data1.w[j] << std::endl;

			res[3 * j + 1] = data1.x[j];
			res[3 * j + 2] = data1.y[j];
			res[3 * j + 3] = data1.w[j];

		}
		return res; //res�������ʺϽӿ�java�������ʽ
		break;
	}
	default:
		//rsģ��
	{
		MyStruct2 data2;
		//double sampleInter = vel;
		double sampleInter = vTurnVel[type];

		//ReedsSheppStateSpace reeds_shepp_space(rad);
		ReedsSheppStateSpace reeds_shepp_space(vTurnRad[type]);

		//�������.sample�������Ǻ���
		reeds_shepp_space.sample(q0, q1, sampleInter, myTest2, &data2);
		//std::shared_ptr<double> res = std::make_shared<double>(new double[data2.size * 3 + 1]);

		cout << "data2 size is " << data2.size << endl;
		double *res = new double[data2.size * 3 + 1]; //new double�Ƕѱ������涨��ôд
		res[0] = data2.size;

		for (size_t j = 0; j < data2.size; j++)
		{
			std::cout << j << "   x = " << data2.x[j] << std::endl;
			std::cout << j << "    y = " << data2.y[j] << std::endl;
			std::cout << j << "    yaw = " << data2.w[j] << std::endl;
			//�����TXT����
			/*data_matlab << "index = " << j << "    x = " << data2.x[j] << std::endl;
			data_matlab << "index = " << j << "   y = " << data2.y[j] << std::endl;
			data_matlab << "index = " << j << "   yaw = " << data2.w[j] << std::endl;*/

			res[3 * j + 1] = data2.x[j];
			res[3 * j + 2] = data2.y[j];
			res[3 * j + 3] = data2.w[j];

		}
		return res;
		break;
	}
	}

}

//���ģ�ͣ�̽��� 0~3.1415 or Բ��6.283��
int ScoutModel(double agent_x, double agent_y, double agent_yaw, double target_x, double target_y, int type)
{
	//input��set parameter
	double initial[3] = { agent_x ,agent_y ,agent_yaw }; // { 0, 0, PI / 2 };  //Location of the detection point (x,y,angel)
	double maxangel = vScoutAngle[type];   //Maximum detection angle
	double length = vScoutRad[type]; //Detection radius
	double o[2] = { target_x ,target_y };//{ 10,0 }; //The position of the target
	int result = 0;
	//cout << "information:" << endl;
	//cout << "detection point:" << "   x:" << initial[0] << "   y:" << initial[1] << "   Initial orientation:" << initial[2] << endl;
	//cout << "Maximum detection angle:" << maxangel << endl;
	//cout << "Detection radius:" << length << endl;
	//cout << "target point:" << "   x:" << o[0] << "   y:" << o[1] << endl;



		//̽������
		double a[2], b[2], c[2];//aΪԲ�ģ�bΪ�Ҳ������ϵĵ㣻cΪ��������ϵĵ�
		a[0] = initial[0]; a[1] = initial[1];
		b[0] = a[0] + length * cos(initial[2] - maxangel / 2); b[1] = a[1] + length * sin(initial[2] - maxangel / 2);
		c[0] = a[0] + length * cos(initial[2] + maxangel / 2); c[1] = a[1] + length * sin(initial[2] + maxangel / 2);

		if (maxangel == 2 * PI)
		{
			double aa = sqrt(pow(o[0] - a[0], 2) + pow(o[1] - a[1], 2));
			if (aa <= length) //Ŀ�����������ߵ��ұ�
			{
				result = 1;
			}
		}
		else {

			//�жϵ��Ƿ���̽��������,Ŀ������ұ����ߵ���ߣ�����������ߵ��ұ�
			int r = 0, l = 0, m = 0;
			double S1 = (a[0] - o[0])*(b[1] - o[1]) - (a[1] - o[1])*(b[0] - o[0]);
			if (S1 >= 0) //Ŀ������ұ����ߵ����
			{
				r = 1;
			}

			double S2 = (a[0] - o[0])*(c[1] - o[1]) - (a[1] - o[1])*(c[0] - o[0]);
			if (S2 <= 0) //Ŀ�����������ߵ��ұ�
			{
				l = 1;
			}

			double oa = sqrt(pow(o[0] - a[0], 2) + pow(o[1] - a[1], 2));
			if (oa <= length) //Ŀ�����������ߵ��ұ�
			{
				m = 1;
			}
			//cout << "Result:" << endl;
			//cout << "r=" << r << endl;
			//cout << "l=" << l << endl;
			//cout << "m=" << m << endl;
			if (r == 1 && l == 1 && m == 1)
			{
				result = 1;
			}
		}
	//std::cout << "weather successfully scout:" << result << std::endl;
	return result;

}


