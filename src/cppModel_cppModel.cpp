#include "cppModel_cppModel.h"
#include "LModel.h"
#include <iostream>

using namespace std;
JNIEXPORT void JNICALL Java_cppModel_cppModel_ModelReadConfig(JNIEnv * env, jclass, jcharArray fname_char_array)
{
	jchar * jchar_ptr = env->GetCharArrayElements(fname_char_array, NULL);
	int len = env->GetArrayLength(fname_char_array);
	char *buf = new char[len + 1];
	for (size_t i = 0; i < len; i++)
	{
		buf[i] = jchar_ptr[i];
	}
	buf[len] = '\0';
	cout << "success" << endl;
	readConfig(buf);
	return ;
}

JNIEXPORT jdoubleArray JNICALL Java_cppModel_cppModel_ModelMotion(JNIEnv *env , jclass, jdoubleArray jq0, jdoubleArray jq1, jint jtype)
{

	jdouble * jdouble_Ptr_cfg1 = env->GetDoubleArrayElements(jq0, NULL);
	int len_x = env->GetArrayLength(jq0);

	jdouble * jdouble_Ptr_cfg2 = env->GetDoubleArrayElements(jq1, NULL);

	double q0[3], q1[3];

	for (size_t i = 0; i < 3; i++)
	{
		q0[i] = jdouble_Ptr_cfg1[i];
		q1[i] = jdouble_Ptr_cfg2[i];
	}
	int type = jtype;
	//cout << "wtf2!!!" << endl;

	double *ptr;
	ptr = getPath(q0, q1, type);    //看，这是之前LModel里编写的函数

	for (size_t i = 0; i < ptr[0]; i++)
	{
		std::cout << "index  = " << i << "x= " << ptr[3 * i + 1] << std::endl;
		std::cout << "index  = " << i << "y= " << ptr[3 * i + 2] << std::endl;
		std::cout << "index  = " << i << "w= " << ptr[3 * i + 3] << std::endl;
	}
	jdoubleArray output = env->NewDoubleArray(ptr[0] * 3 + 1);
	jboolean iscopy = JNI_FALSE;
	jdouble *destArrayElems = env->GetDoubleArrayElements(output, &iscopy);
	for (size_t i = 0; i < (ptr[0] * 3 + 1); i++)
	{
		destArrayElems[i] = ptr[i];
	}
	env->SetDoubleArrayRegion(output, 0, ptr[0] * 3 + 1, destArrayElems);
	return output;
}
//新加：
JNIEXPORT jint JNICALL Java_cppModel_cppModel_ModelScout(JNIEnv *env, jclass, jdouble jagent_x, jdouble jagent_y, jdouble jagent_yaw, jdouble jtarget_x, jdouble jtarget_y, jint jtype)
{
	double agent_x = jagent_x;
	double agent_y = jagent_y;
	double agent_yaw = jagent_yaw;
	double target_x = jtarget_x;
	double target_y = jtarget_y;
	int type = jtype;

	int ScoutAns;
	ScoutAns = ScoutModel(agent_x, agent_y, agent_yaw, target_x, target_y, type);
	jint output = ScoutAns;
	return output;
}
