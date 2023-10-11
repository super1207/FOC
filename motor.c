/*
 * 作者：super1207
 * 描述：PMSM电机的无感FOC SVPWM控制，使用滑膜+PLL，驱动24V150W小电机，使用定点计算，使用时，pwm配置成12k，计数器最大值配置为3500
 * 时间：2023年
 */


#include "motor.h"

#include <string.h>
#include <math.h>
#include <stdint.h>
#include <assert.h>


#define   _IQmpy(A,B)    (int32_t)(((A)*(B))>>15)
#define   _IQmpy2(A)     (int32_t)((A)<<1)
#define   _IQdiv2(A)     (int32_t)((A)>>1)


const static int16_t IQSin_Cos_Table[256]={\
	0x0000,0x00C9,0x0192,0x025B,0x0324,0x03ED,0x04B6,0x057F,\
	0x0648,0x0711,0x07D9,0x08A2,0x096A,0x0A33,0x0AFB,0x0BC4,\
	0x0C8C,0x0D54,0x0E1C,0x0EE3,0x0FAB,0x1072,0x113A,0x1201,\
	0x12C8,0x138F,0x1455,0x151C,0x15E2,0x16A8,0x176E,0x1833,\
	0x18F9,0x19BE,0x1A82,0x1B47,0x1C0B,0x1CCF,0x1D93,0x1E57,\
	0x1F1A,0x1FDD,0x209F,0x2161,0x2223,0x22E5,0x23A6,0x2467,\
	0x2528,0x25E8,0x26A8,0x2767,0x2826,0x28E5,0x29A3,0x2A61,\
	0x2B1F,0x2BDC,0x2C99,0x2D55,0x2E11,0x2ECC,0x2F87,0x3041,\
	0x30FB,0x31B5,0x326E,0x3326,0x33DF,0x3496,0x354D,0x3604,\
	0x36BA,0x376F,0x3824,0x38D9,0x398C,0x3A40,0x3AF2,0x3BA5,\
	0x3C56,0x3D07,0x3DB8,0x3E68,0x3F17,0x3FC5,0x4073,0x4121,\
	0x41CE,0x427A,0x4325,0x43D0,0x447A,0x4524,0x45CD,0x4675,\
	0x471C,0x47C3,0x4869,0x490F,0x49B4,0x4A58,0x4AFB,0x4B9D,\
	0x4C3F,0x4CE0,0x4D81,0x4E20,0x4EBF,0x4F5D,0x4FFB,0x5097,\
	0x5133,0x51CE,0x5268,0x5302,0x539B,0x5432,0x54C9,0x5560,\
	0x55F5,0x568A,0x571D,0x57B0,0x5842,0x58D3,0x5964,0x59F3,\
	0x5A82,0x5B0F,0x5B9C,0x5C28,0x5CB3,0x5D3E,0x5DC7,0x5E4F,\
	0x5ED7,0x5F5D,0x5FE3,0x6068,0x60EB,0x616E,0x61F0,0x6271,\
	0x62F1,0x6370,0x63EE,0x646C,0x64E8,0x6563,0x65DD,0x6656,\
	0x66CF,0x6746,0x67BC,0x6832,0x68A6,0x6919,0x698B,0x69FD,\
	0x6A6D,0x6ADC,0x6B4A,0x6BB7,0x6C23,0x6C8E,0x6CF8,0x6D61,\
	0x6DC9,0x6E30,0x6E96,0x6EFB,0x6F5E,0x6FC1,0x7022,0x7083,\
	0x70E2,0x7140,0x719D,0x71F9,0x7254,0x72AE,0x7307,0x735E,\
	0x73B5,0x740A,0x745F,0x74B2,0x7504,0x7555,0x75A5,0x75F3,\
	0x7641,0x768D,0x76D8,0x7722,0x776B,0x77B3,0x77FA,0x783F,\
	0x7884,0x78C7,0x7909,0x794A,0x7989,0x79C8,0x7A05,0x7A41,\
	0x7A7C,0x7AB6,0x7AEE,0x7B26,0x7B5C,0x7B91,0x7BC5,0x7BF8,\
	0x7C29,0x7C59,0x7C88,0x7CB6,0x7CE3,0x7D0E,0x7D39,0x7D62,\
	0x7D89,0x7DB0,0x7DD5,0x7DFA,0x7E1D,0x7E3E,0x7E5F,0x7E7E,\
	0x7E9C,0x7EB9,0x7ED5,0x7EEF,0x7F09,0x7F21,0x7F37,0x7F4D,\
	0x7F61,0x7F74,0x7F86,0x7F97,0x7FA6,0x7FB4,0x7FC1,0x7FCD,\
	0x7FD8,0x7FE1,0x7FE9,0x7FF0,0x7FF5,0x7FF9,0x7FFD,0x7FFE};


static int32_t IQsat( int32_t Uint,int32_t  U_max, int32_t U_min)
{
	int32_t Uout;
	if(Uint<= U_min)
		Uout= U_min;
	else if( Uint>=U_max)
		Uout=U_max;
	else
		Uout= Uint;
	return  Uout;
}


static void msincos(int32_t theta,int32_t * sin_val,int32_t * cos_val)
{
	uint16_t  hindex;
	assert(sin_val);
	assert(cos_val);
	hindex = (uint16_t) theta;
	hindex >>=6;
	switch (hindex & 0x0300)
	{
		case 0x0000:
			(*sin_val) = IQSin_Cos_Table[(uint8_t)(hindex)];
			(*cos_val) = IQSin_Cos_Table[(uint8_t)(0xFF-(uint8_t)(hindex))];
			break;
		case 0x0100:
			(*sin_val) = IQSin_Cos_Table[(uint8_t)(0xFF-(uint8_t)(hindex))];
			(*cos_val) = -IQSin_Cos_Table[(uint8_t)(hindex)];
			break;
		case 0x0200:
			(*sin_val) = -IQSin_Cos_Table[(uint8_t)(hindex)];
			(*cos_val) = -IQSin_Cos_Table[(uint8_t)(0xFF-(uint8_t)(hindex))];
			break;
		case 0x0300:
			(*sin_val) =  -IQSin_Cos_Table[(uint8_t)(0xFF-(uint8_t)(hindex))];
			(*cos_val) =  IQSin_Cos_Table[(uint8_t)(hindex)];
		break;
			default:
		break;
	}
}


void InitMotorParams1(MotorParams * motorParams)
{
	assert(motorParams);
	memset(motorParams,0,sizeof(MotorParams));
	motorParams->rs = 13.7f / 2;
	motorParams->ls = 14.6f / 2 / 1000;
	float ts = 1 / 12000.f;
	motorParams->vb = 24;
	motorParams->vi = 10;
	float Fsmopos_t = exp((-motorParams->rs/motorParams->ls)*ts); // 0 --- 1
	float Gsmopos_t = (motorParams->vb/motorParams->vi)*(1/motorParams->rs)*(1-Fsmopos_t);
	motorParams->Fsmopos = (int32_t)(Fsmopos_t*32768);
	motorParams->Gsmopos = (int32_t)(Gsmopos_t*32768);
	motorParams->PreTime1 = 1500;
	motorParams->PreTime2 = 3000;
	motorParams->VfTime = 24000;
	motorParams->MaxVfAngleSpeed = 150;
	motorParams->StartVfVq = 2048;
	motorParams->MaxVfVq = 6800;
	motorParams->VfAngleTime = 250;
	motorParams->VfVqTime = 6;
	motorParams->VfVq = motorParams->StartVfVq;
	motorParams->VdKi = 15800;
	motorParams->VdKp = 220;
	motorParams->VqOut = 8800;
	motorParams->Kslide = 4500;
	motorParams->PWMHalfPerMax = 1750;
	motorParams->Pole = 2;
	motorParams->PLLKp = 4000;
	motorParams->PLLKi = 100;
}

void SetMotorDir(MotorParams * motorParams,int32_t dir)
{
	motorParams->Dir = dir;
}

void SetMotorVq(MotorParams * motorParams,int32_t vq)
{
	motorParams->VqOut = vq;
}

void MotorStep(MotorParams * motorParams)
{
	assert(motorParams);
	++motorParams->count;
	if(motorParams->count == 0xFFFFFFFF)
	{
		motorParams->count = 0xFFFFFFFE;
	}

	int32_t angle;
	int32_t vd_out;
	int32_t vq_out;

	if(motorParams->count < motorParams->PreTime1) // 预定位
	{
		angle = 16384;
		vd_out = motorParams->PreVd;
		vq_out = 0;
	}
	else if(motorParams->count < motorParams->PreTime2) // 预定位
	{
		angle = 0;
		vd_out = motorParams->PreVd;
		vq_out = 0;
	}
	else if(motorParams->count < motorParams->VfTime) // 斜坡强拉
	{
		++motorParams->VfAngleTimeCount;
		++motorParams->VfVqTimeCount;
		if(motorParams->VfAngleTimeCount == motorParams->VfAngleTime)
		{
			motorParams->VfAngleTimeCount = 0;
			motorParams->VfAngleAdd += 1;
		}
		if(motorParams->VfAngleAdd > motorParams->MaxVfAngleSpeed){
			motorParams->VfAngleAdd = motorParams->MaxVfAngleSpeed;
		}
		motorParams->VfAngle += motorParams->VfAngleAdd;
		if(motorParams->VfAngle > 65536)
		{
			motorParams->VfAngle -= 65536;
		}
		else if(motorParams->VfAngle < 0)
		{
			motorParams->VfAngle += 65536;
		}
		if(motorParams->VfVqTimeCount == motorParams->VfVqTime)
		{
			motorParams->VfVqTimeCount = 0;
			motorParams->StartVfVq += 1;
		}

		if(motorParams->StartVfVq > motorParams->MaxVfVq){
			motorParams->StartVfVq = motorParams->MaxVfVq;
		}
		angle = motorParams->VfAngle;
		vd_out = 0;
		vq_out = motorParams->StartVfVq;
	}
	else // 滑膜闭环控制
	{
		angle = motorParams->SMOAngle;
		motorParams->VdISum += (-motorParams->Id);
		vd_out = _IQmpy(-motorParams->Id,motorParams->VdKp) + _IQmpy(motorParams->VdKi,motorParams->VdISum);
		vq_out = motorParams->VqOut;
	}
	int32_t sin_val,cos_val;
	msincos(angle,&sin_val,&cos_val);
	// clarke
	if(motorParams->Dir)
	{
		motorParams->Ialpha = motorParams->Iv;
		motorParams->Ibeta = _IQmpy((motorParams->Iv +_IQmpy2(motorParams->Iu)),18918);
	}
	else
	{
		motorParams->Ialpha = motorParams->Iu;
		motorParams->Ibeta = _IQmpy((motorParams->Iu +_IQmpy2(motorParams->Iv)),18918);
	}

	// park
	motorParams->Id = _IQmpy(motorParams->Ialpha,cos_val) + _IQmpy(motorParams->Ibeta,sin_val);
	motorParams->Iq = _IQmpy(motorParams->Ibeta,cos_val) - _IQmpy(motorParams->Ialpha,sin_val);
	// SMO
	const int32_t E0 = 16384; // 0.5
	motorParams->EstIalpha = _IQmpy(motorParams->Fsmopos,motorParams->EstIalpha) + _IQmpy(motorParams->Gsmopos,(motorParams->Valpha-motorParams->Ealpha-motorParams->Zalpha));
	motorParams->EstIbeta  = _IQmpy(motorParams->Fsmopos,motorParams->EstIbeta)  + _IQmpy(motorParams->Gsmopos,(motorParams->Vbeta-motorParams->Ebeta-motorParams->Zbeta));
	int32_t IalphaError = motorParams->EstIalpha - motorParams->Ialpha;
	int32_t IbetaError  = motorParams->EstIbeta  - motorParams->Ibeta;
	motorParams->Zalpha = _IQmpy(IQsat(IalphaError,E0,-E0),_IQmpy2(motorParams->Kslide));
	motorParams->Zbeta  = _IQmpy(IQsat(IbetaError ,E0,-E0),_IQmpy2(motorParams->Kslide));
	motorParams->Ealpha = motorParams->Zalpha;
	motorParams->Ebeta  = motorParams->Zbeta;
	// PLL
	int32_t PLLErr = _IQmpy(-motorParams->Ealpha,motorParams->PLLLastEa) - _IQmpy(motorParams->Ebeta,motorParams->PLLLastEb);
	motorParams->PLLISum += PLLErr;
	motorParams->SMOSpeed = _IQmpy(PLLErr,motorParams->PLLKp) + _IQmpy(motorParams->PLLKi,motorParams->PLLISum);
	motorParams->MotorSpeed = motorParams->SMOSpeed * 720000 / motorParams->Pole / 65536; // 720000 = 60 * 12000
	motorParams->SMOAngle += motorParams->SMOSpeed;
	while(motorParams->SMOAngle > 65536)motorParams->SMOAngle-=65536;
	while(motorParams->SMOAngle < 0)motorParams->SMOAngle+=65536;
	msincos(motorParams->SMOAngle,&motorParams->PLLLastEb,&motorParams->PLLLastEa);
	// ipark
	motorParams->Valpha = _IQmpy(vd_out,cos_val) - _IQmpy(vq_out,sin_val);
	motorParams->Vbeta  = _IQmpy(vq_out,cos_val) + _IQmpy(vd_out,sin_val);
	// svpwm
	int32_t Ta,Tb,Tc;
	int32_t tmp1= motorParams->Vbeta;
	int32_t tmp2= _IQdiv2(motorParams->Vbeta) + _IQmpy(28377,motorParams->Valpha);
	int32_t tmp3= tmp2 - tmp1;
	uint16_t VecSector = 3;
	VecSector=(tmp2> 0)?(VecSector-1):VecSector;
	VecSector=(tmp3> 0)?(VecSector-1):VecSector;
	VecSector=(tmp1< 0)?(7-VecSector):VecSector;
	if(VecSector==1 || VecSector==4)
	{
		Ta = tmp2;
		Tb = tmp1-tmp3;
		Tc = -tmp2;
	}
	else if(VecSector==2 || VecSector==5)
	{
		Ta = tmp3 + tmp2;
		Tb = tmp1;
		Tc = -tmp1;
	}
	else
	{
		Ta= tmp3;
		Tb= -tmp3;
		Tc= -(tmp1+tmp2);
	}
	// limit
	Ta = IQsat(Ta,32767,-32767);
	Tb = IQsat(Tb,32767,-32767);
	Tc = IQsat(Tc,32767,-32767);
	// pwm out
	if(motorParams->Dir)
	{
		motorParams->PWMOut[1]= _IQmpy(motorParams->PWMHalfPerMax,Ta)+ motorParams->PWMHalfPerMax;
		motorParams->PWMOut[0]= _IQmpy(motorParams->PWMHalfPerMax,Tb)+ motorParams->PWMHalfPerMax;
	}else
	{
		motorParams->PWMOut[0]= _IQmpy(motorParams->PWMHalfPerMax,Ta)+ motorParams->PWMHalfPerMax;
		motorParams->PWMOut[1]= _IQmpy(motorParams->PWMHalfPerMax,Tb)+ motorParams->PWMHalfPerMax;
	}
	motorParams->PWMOut[2]= _IQmpy(motorParams->PWMHalfPerMax,Tc)+ motorParams->PWMHalfPerMax;
}
