#include <ev3.h>
#include <stdbool.h>
int setPoint = 25;
#define DEG_TO_MOVE_ONE_FOOT 1400
#define QUARTER_TURN 80
#define SPEED 10
/*wander - robot will move through free space until it is within 1 foot of an "object"*/
/*Hopefully we can use all sensors*/

//code above needs to be in its own function
//stopMoving();

// detecting distance parameter in mm
// 508mm = 20in
// 406mm = 16in
// 305mm = 12in
// 254mm = 10in
// 203mm = 8 in
#define OBJECT_SONAR_DIST 406

void rotateCCW(void);
void playFinishingTone(void);

void forwardOneSquare(void)
{
	Wait(1000);
	ResetRotationCount(OUT_A);
	Wait(20);
	int y = readSensor(IN_1);
	while (MotorRotationCount(OUT_A) < DEG_TO_MOVE_ONE_FOOT)
	{
		OnFwdSync(OUT_AD, SPEED+40);

		y = readSensor(IN_1);
		if (2 == readSensor(IN_1))
			return;
	}

	Off(OUT_AD);
	return;
}

bool readSonarSensor(int prev)
{
	setAllSensorMode(COL_REFLECT, GYRO_ANG, US_DIST_MM, COL_COLOR);
	// returns distance in mm from 0 to 2550mm
	int dist = readSensor(IN_3);
	if ( (dist < OBJECT_SONAR_DIST) || (prev >= (dist - 100) ) )
	{
		playFinishingTone();
		rotateCCW();
		forwardOneSquare();

		if (5 == readSensor(IN_1))
			// true means it has found an object and we turn towards it
			return true;
	}
	// false, means it did not find an object yet
	return false;
}

void playFinishingTone(void)
{
	PlayTone(TONE_E2, NOTE_EIGHT);
	PlayTone(TONE_GS2, NOTE_EIGHT);
	PlayTone(TONE_E2, NOTE_EIGHT);
	PlayTone(TONE_GS2, NOTE_EIGHT);
	PlayTone(TONE_E2, NOTE_EIGHT);
	PlayTone(TONE_GS2, NOTE_EIGHT);
	PlayTone(TONE_B2, NOTE_QUARTER);
	PlayTone(NOTE_WHOLE, NOTE_QUARTER);

	StopSound();
	return;
}

void initOurSensors(void)
{
	// Inputs7
	// IN_1 IN_2 IN_3 IN_4
	setAllSensorMode(COL_REFLECT, GYRO_ANG, US_DIST_MM, COL_COLOR);
}
int readGyroAngleSensor(void)
{
    // Return of angle in ° -180 to 180
    return readSensor(IN_2);
}


void rotateCCW(void)
{
	setAllSensorMode(COL_REFLECT, GYRO_ANG, US_DIST_MM, COL_COLOR);
	Wait(1000);

	ResetRotationCount(OUT_A);
	ResetRotationCount(OUT_D);


	OnRevReg(OUT_A, SPEED/2);
	OnFwdReg(OUT_D, SPEED/2);
	Wait(3300);
//	while (MotorRotationCount(OUT_A) <= -(QUARTER_TURN-3));
//	//while (MotorRotationCount(OUT_D) >= -(QUARTER_TURN+6));
	Off(OUT_AD);
	return;
}

void rotateCW(void)
{
	Wait(1000);
	ResetRotationCount(OUT_A);
	ResetRotationCount(OUT_D);

	OnFwdReg(OUT_A, SPEED/2);
	OnRevReg(OUT_D, SPEED/2);
	Wait(3000);
//	while (MotorRotationCount(OUT_D) <= (QUARTER_TURN-3));
//	while (MotorRotationCount(OUT_D) <= QUARTER_TURN-10);

	Off(OUT_AD);

	return;
}

void forwardWander(void)
{
	setAllSensorMode(COL_REFLECT, GYRO_ANG, US_DIST_MM, COL_COLOR);
	//Wait(1000);
	ResetRotationCount(OUT_A);
	Wait(20);
	int y = readSensor(IN_4);
	while (y != 2)
	{
		y = readSensor(IN_4);
		OnFwdSync(OUT_AD, SPEED);

	}
	Wait(500);
	Off(OUT_AD);
	rotateCCW();
	return;
}

//void wander()
//{
//    setAllSensorMode(COL_COLOR, GYRO_ANG, NO_SEN, NO_SEN);
//    int a = readSensor(IN_1);
//
//    while(a != 2 )
//    {
//    	a = readSensor(IN_1);
//        forwardOneSquare();
//    }
//    rotateCCW();
//}

void wallFollowing()
{
    //initOurSensors();
	setAllSensorMode(COL_REFLECT, GYRO_ANG, US_DIST_MM, COL_COLOR);
	int kp = 100;
    int ki = 0;
    int kd = 1;
    int totalPower = 20;
    int error = 0;
    int lastError = 0;
    int integral = 0;
    int derivative = 0;
    int y = readSensor(IN_1);
    int x = readSensor(IN_4);
    int sonarDistance = 1000;
    int correction = 0;
    while(true)
    {
    	//sonar distance to wander
    	sonarDistance = readSensor(IN_3);

    	if (sonarDistance < OBJECT_SONAR_DIST)
    	{
    		Wait(200);
    		Off(OUT_AD);
    		PlaySound(SOUND_DOUBLE_BEEP);
    		rotateCCW();
    		Wait(1000);
    		forwardOneSquare();
    		Wait(1000);
    		//playFinishingTone();
    		PlaySound(SOUND_DOWN);
    		Wait(1000);
    		return;
    	}
    	y = readSensor(IN_1);
    	x = readSensor(IN_4);
    	error = setPoint - y;
    	integral += error;
    	derivative = error - lastError;
    	correction = (error*kp) + (integral*ki);
    	correction /= 100;

    	if(x == 2)
    	{
    		OnRevReg(OUT_A, SPEED/2);
    		OnRevReg(OUT_D, SPEED/2);
    		Wait(1200);
    		Off(OUT_AD);

    		Wait(500);

    		rotateCCW();

    	}
    	if(correction == 0)
    	{
    		OnFwdReg(OUT_D, totalPower);
    		OnFwdReg(OUT_A, totalPower);
    	}
    	else if(correction > 0)
    	{
    		OnFwdReg(OUT_D, totalPower);
    		OnFwdReg(OUT_A, 0);
    	}
    	else if (correction < 0)
    	{
    		OnFwdReg(OUT_A, totalPower);
    		OnFwdReg(OUT_D, 0);
    	}
    	lastError = error;
    	Wait(10);
    }
}

int main(void)
{
    InitEV3();
    //setAllSensorMode(COL_REFLECT, NO_SEN, NO_SEN, NO_SEN);
    initOurSensors();
    forwardWander();
    wallFollowing();
    //forwardOneSquare();
    FreeEV3();
}
