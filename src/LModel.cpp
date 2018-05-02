//LModel:把别人编的Dubins和reedsshepp程序的输出改编成我们需要的形式
#include <iostream>
#include <memory>
#include <vector>

#include "LModel.h"
#include "ssconfig.hpp"   //C++读取txt（by商成思）
#include "mystruct.h"     //我们需要的输出形式（by Gao)
#include "reeds_shepp.h"  //别人写的rs
#include "dubins.h"       //别人写的dubins




#define PI 3.1415
using namespace std;
std::vector<double> vTurnRad; //先定义要使用的全局变量
std::vector<double> vTurnVel;
std::vector<double> vScoutRad;  
std::vector<double> vScoutAngle;

//读取txt文档（by商成思）
bool readConfig(char * filename)  //可通用
{
	sscfg::ConfigFile co_list = sscfg::ConfigFile::load(filename);
	co_list.get("rad", vTurnRad); //把txt文档中"rad"的值赋给全局变量vTurnRad
	co_list.get("vel", vTurnVel);       //同理
	co_list.get("Srad", vScoutRad);     //同理
	co_list.get("Sangle", vScoutAngle); //同理
	cout << "vTurnRad size is " << vTurnRad.size() << endl;
	return false;
}

double * getPath(double * q0, double * q1, int type)
{

	switch (type) //只有type=7(飞机）时使用Dubins模型；其余使用rs模型
	{
	case 7:
		//Dubins模型
	{
		DubinsPath d_path;

		int err = dubins_init(q0, q1, vTurnRad[type], &d_path);
		MyDubinsSturct data1;
		dubins_path_sample_many(&d_path, myDubinsTest, vTurnVel[type], &data1);
		std::cout << "data1 size = " << data1.size << endl;

		double *res = new double[data1.size * 3 + 1]; //new double≈全局变量？
		res[0] = data1.size;

		for (size_t j = 0; j < data1.size; j++)
		{
			std::cout << j << "   x = " << data1.x[j] << std::endl;
			std::cout << j << "    y = " << data1.y[j] << std::endl;
			std::cout << j << "    yaw = " << data1.w[j] << std::endl;
			//输出到TXT里面
			//data_matlab << "index = " << j << "    x = " << data1.x[j] << std::endl;
			//data_matlab << "index = " << j << "   y = " << data1.y[j] << std::endl;
			//data_matlab << "index = " << j << "   yaw = " << data1.w[j] << std::endl;

			res[3 * j + 1] = data1.x[j];
			res[3 * j + 2] = data1.y[j];
			res[3 * j + 3] = data1.w[j];

		}
		return res; //res变量是适合接口java的输出形式
		break;
	}
	default:
		//rs模型
	{
		MyStruct2 data2;
		//double sampleInter = vel;
		double sampleInter = vTurnVel[type];

		//ReedsSheppStateSpace reeds_shepp_space(rad);
		ReedsSheppStateSpace reeds_shepp_space(vTurnRad[type]);

		//下面这个.sample函数才是核心
		reeds_shepp_space.sample(q0, q1, sampleInter, myTest2, &data2);
		//std::shared_ptr<double> res = std::make_shared<double>(new double[data2.size * 3 + 1]);

		cout << "data2 size is " << data2.size << endl;
		double *res = new double[data2.size * 3 + 1]; //new double是堆变量；规定这么写
		res[0] = data2.size;

		for (size_t j = 0; j < data2.size; j++)
		{
			std::cout << j << "   x = " << data2.x[j] << std::endl;
			std::cout << j << "    y = " << data2.y[j] << std::endl;
			std::cout << j << "    yaw = " << data2.w[j] << std::endl;
			//输出到TXT里面
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

//侦察模型（探测角 0~3.1415 or 圆：6.283）
int ScoutModel(double agent_x, double agent_y, double agent_yaw, double target_x, double target_y, int type)
{
	//input：set parameter
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



		//探测扇形
		double a[2], b[2], c[2];//a为圆心；b为右侧射线上的点；c为左侧射线上的点
		a[0] = initial[0]; a[1] = initial[1];
		b[0] = a[0] + length * cos(initial[2] - maxangel / 2); b[1] = a[1] + length * sin(initial[2] - maxangel / 2);
		c[0] = a[0] + length * cos(initial[2] + maxangel / 2); c[1] = a[1] + length * sin(initial[2] + maxangel / 2);

		if (maxangel == 2 * PI)
		{
			double aa = sqrt(pow(o[0] - a[0], 2) + pow(o[1] - a[1], 2));
			if (aa <= length) //目标点在左边射线的右边
			{
				result = 1;
			}
		}
		else {

			//判断点是否在探测扇形内,目标点在右边射线的左边，且在左侧射线的右边
			int r = 0, l = 0, m = 0;
			double S1 = (a[0] - o[0])*(b[1] - o[1]) - (a[1] - o[1])*(b[0] - o[0]);
			if (S1 >= 0) //目标点在右边射线的左边
			{
				r = 1;
			}

			double S2 = (a[0] - o[0])*(c[1] - o[1]) - (a[1] - o[1])*(c[0] - o[0]);
			if (S2 <= 0) //目标点在左边射线的右边
			{
				l = 1;
			}

			double oa = sqrt(pow(o[0] - a[0], 2) + pow(o[1] - a[1], 2));
			if (oa <= length) //目标点在左边射线的右边
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


