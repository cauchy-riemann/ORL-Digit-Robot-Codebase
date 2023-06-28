#include "LLAPI/lowlevelapi.h"
#include<unistd.h> //for usleep
#include <math.h>
enum MJCF_DigitMotors
{
	M_LeftHipRoll,
	M_LeftHipYaw,
	M_LeftHipPitch,
	M_LeftKnee,
	M_LeftToeA,
	M_LeftToeB,

	M_LeftShoulderRoll,
	M_LeftShoulderPitch,
	M_LeftShoulderYaw,
	M_LeftElbow,
	
	M_RightHipRoll,
	M_RightHipYaw,
	M_RightHipPitch,
	M_RightKnee,
	M_RightToeA,
	M_RightToeB,
	
	M_RightShoulderRoll,
	M_RightShoulderPitch,
	M_RightShoulderYaw,
	M_RightElbow,
};


enum MJCF_DigitJoints
{
	M_Pelvis,
	M_LeftHipRoll,
	M_LeftHipYaw,
	M_LeftHipPitch,
	M_LeftAchillesRod,
	M_LeftKnee,
	M_LeftShin,
	M_LeftTarsus,
	M_LeftHeelSpring,
	M_LeftToeA,
	M_LeftToeARod,
	M_LeftToeB,
	M_LeftToeBRod,
	M_LeftToePitch,
	M_LeftToeRoll,
	M_LeftShoulderRoll,
	M_LeftShoulderPitch,
	M_LeftShoulderYaw,
	M_LeftElbow,

	M_RightHipRoll,
	M_RightHipYaw,
	M_RightHipPitch,
	M_RightAchillesRod,
	M_RightKnee,
	M_RightShin,
	M_RightTarsus,
	M_RightHeelSpring,
	M_RightToeA,
	M_RightToeARod,
	M_RightToeB,
	M_RightToeBRod,
	M_RightToePitch,
	M_RightToeRoll,
	M_RightShoulderRoll,
	M_RightShoulderPitch,
	M_RightShoulderYaw,
	M_RightElbow,
};


DigitMotors L2M_Motor(MJCF_DigitMotors motor)
{
	if ((motor >= M_LeftHipRoll && motor <= M_LeftToeB) || (motor>=M_RightShoulderRoll && motor<=M_RightElbow))
		return motor;

	else if (motor >= M_LeftShoulderRoll && motor <= M_LeftElbow)
		return motor+6;

	else if (motor >= M_RightHipRoll && motor <= M_RightToeB)
		return motor-4;
}

MJCF_DigitMotors M2L_Motor(DigitMotors motor)
{
		if ((motor >= LeftHipRoll && motor <= LeftToeB) || (motor>=RightShoulderRoll && motor<=RightElbow))
		return motor;

	else if (motor >= LeftShoulderRoll && motor <= LeftElbow)
		return motor-6;

	else if (motor >= RightHipRoll && motor <= RightToeB)
		return motor+4;
}

DigitJoints L2M_Joint(MJCF_DigitJoints motor)
{

}

MJCF_DigitMotors M2L_Joint(DigitMotors motor)
{

}
