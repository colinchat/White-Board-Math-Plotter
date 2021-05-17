/*

LEGMOS

Student Names:
Mansheel Chahal
Colin Chatfield
Kritik Kaushal
Amjad Halis

*/

#include "PC_FileIO.c"

const int MARKER_ENCODER = 135;
const int TURN_SPD = 10;
const int DRIVE_SPD = 15;
const float RAD_TO_DEG = 180 / PI;
const float DEG_TO_CM = PI * 2.75 / 180;

typedef struct{
	float low_x;
	float high_x;
	float low_y;
	float high_y;
	float widthCM;
	float lengthCM;
	float xUnitDist;
	float yUnitDist;
} BoardInfo;

typedef struct{
	//0-linear, 1-quadratic, 2-cubic
	int type;

	// for quadratics and linear
	float x_3;
	float x_2;
	float x_1;
	float x_0;

	// for trig
	float a;    // a - vert stretch
	float b;    // b - hor stretch
	float c;    // c - hor shift
	float d;    // d - vert shift

	float yTheo; // in coordinate units
	float angleTheo; // in degrees
} FuncInfo;

typedef struct{
	float x; // in coordinate units
	float y; // in coordinate units
} Point;


void boardIO(BoardInfo &, TFileHandle);
void fileIO(FuncInfo &, TFileHandle &);
void markerDown(bool);
void rotateRobotTo(int);
void moveDist(float, int, int);
void drawAxis(BoardInfo, Point &);
void linear(float, FuncInfo &);
void quadratic(float, FuncInfo &);
void cubic(float, FuncInfo &);
void sinusoidal(float, FuncInfo &);
void funcReturn(float, FuncInfo &);
void draw(BoardInfo, FuncInfo, Point &);
void moveToLowX(Point &, BoardInfo);
void moveToStartPosition(BoardInfo, FuncInfo, Point &); 


void boardIO(BoardInfo & board, TFileHandle fin1)
{
	bool b2 = true;

	b2 = readFloatPC (fin1, board.low_x);
	b2 = readFloatPC (fin1, board.high_x);
	b2 = readFloatPC (fin1, board.low_y);
	b2 = readFloatPC (fin1, board.high_y);
	b2 = readFloatPC (fin1, board.widthCM);
	b2 = readFloatPC (fin1, board.lengthCM);

	board.xUnitDist = board.lengthCM / (fabs(board.low_x) + fabs(board.high_x));
	board.yUnitDist = board.widthCM / (fabs(board.low_y) + fabs(board.high_y));
}

void fileIO(FuncInfo & funcPara, TFileHandle & fin)
{
	int fType = 0;

	readIntPC(fin, fType);

	if(fType == 1)
	{
		funcPara.type = 0;
		readFloatPC (fin, funcPara.x_1);
		readFloatPC (fin, funcPara.x_0);
	}
	else if(fType == 2)
	{
		funcPara.type = 1;
		readFloatPC (fin, funcPara.x_2);
		readFloatPC (fin, funcPara.x_1);
		readFloatPC (fin, funcPara.x_0);
	}
	else if(fType == 3)
	{
		funcPara.type = 2;
		readFloatPC (fin, funcPara.x_3);
		readFloatPC (fin, funcPara.x_2);
		readFloatPC (fin, funcPara.x_1);
		readFloatPC (fin, funcPara.x_0);
	}
	else if(fType == 4)
	{
		funcPara.type = 3;
		readFloatPC (fin, funcPara.a);
		readFloatPC (fin, funcPara.b);
		readFloatPC (fin, funcPara.c);
		readFloatPC (fin, funcPara.d);
	}
}

void markerDown(bool condition)
{
	nMotorEncoder[motorC] = 0;
	wait1Msec(500);

	if(condition)
	{
		motor[motorC] = -10;
		while(nMotorEncoder[motorC] > -MARKER_ENCODER)
		{}
	}
	else
	{
		motor[motorC] = 10;
		while(nMotorEncoder[motorC] < MARKER_ENCODER)
		{}
	}
	motor[motorC] = 0;
	wait1Msec(500);
}

void rotateRobotTo(int degree) //rotates robot to degree wrt x-axis
{
	wait1Msec(500);
	motor[motorA] = motor[motorD] = 0;
	int testGyro = getGyroDegrees(S1);
	
	if (testGyro > degree)
	{
		motor[motorA] = TURN_SPD;
		motor[motorD] = -TURN_SPD;
		while (getGyroDegrees(S1) > degree)
		{}
	}
	else
	{
		motor[motorA] = -TURN_SPD;
		motor[motorD] = TURN_SPD;
		while (getGyroDegrees(S1) < degree)
		{}
	}
	
	motor[motorA] = motor[motorD] = 0;
	wait1Msec(500);
}

void moveDist(float dist, int motorPow, int theoAngle)
{
	float testAngle = 0, angleDiff = 0;
	nMotorEncoder[motorA] = 0;
	motor[motorA] = motor[motorD] = motorPow;
	
	while(fabs(nMotorEncoder[motorA]*DEG_TO_CM) < dist) // this loop corrects direction
	{
		testAngle = getGyroDegrees(S1);
		angleDiff = testAngle - theoAngle;
		if(fabs(motorPow) + fabs(angleDiff) > 75)
		{
			angleDiff = 75 - motorPow;
			if(motorPow < 0) //this is to account for when it moves backwards
				angleDiff = -75 + motorPow;
		}
		motor[motorA] = motorPow + angleDiff;
		motor[motorD] = motorPow - angleDiff;
	}
	motor[motorA] = motor[motorD] = 0;
}

void drawAxis(BoardInfo board, Point & current)
{
	// Assuming robot starts in the top left corner of drawing rectangle
	
	// moves to top of y-axis
	moveDist(fabs(board.low_x*board.xUnitDist), DRIVE_SPD, 0);

	// draw y-axis
	rotateRobotTo(-90);
	markerDown(true);
	moveDist(board.widthCM, DRIVE_SPD, -90);
	markerDown(false);

	// drive to right boundary
	rotateRobotTo(0);
	moveDist(board.high_x*board.xUnitDist, DRIVE_SPD, 0);

	// move up to x-axis
	rotateRobotTo(90);
	moveDist(fabs(board.low_y*board.yUnitDist), DRIVE_SPD, 90);

	// draw entire x-axis (moving backwards to avoid tangling string)
	rotateRobotTo(0);
	markerDown(true);
	moveDist(board.lengthCM, -DRIVE_SPD, 0);
	markerDown(false);
	
	// updates position (assuming perfect accuracy)
	current.x = board.low_x;
	current.y = 0;
}

/*
The following 4 functions (linear, quadratic, cubic, sinusoidal)
calculate the theoretical y value at a given x-position using the 
coefficients and type of function. They also calculate the theoretical 
angle the robot should be driving at to accurately draw the function 
by finding the inverse tan of the derivative of the function
at the given x-position.
*/

void linear(float xPos, FuncInfo & linFunc)
{
	linFunc.yTheo = linFunc.x_1 * xPos + linFunc.x_0;
	
	linFunc.angleTheo = atan(linFunc.x_1);
	linFunc.angleTheo = linFunc.angleTheo * RAD_TO_DEG;
}

void quadratic(float xPos, FuncInfo & quadFunc)
{
	quadFunc.yTheo = quadFunc.x_2 * pow(xPos, 2) + quadFunc.x_1 * xPos + quadFunc.x_0;
	
	quadFunc.angleTheo = atan(2 * quadFunc.x_2 * xPos + quadFunc.x_1);
	quadFunc.angleTheo = quadFunc.angleTheo * RAD_TO_DEG;
}

void cubic(float xPos, FuncInfo & cubFunc)
{
	cubFunc.yTheo = cubFunc.x_3 * pow(xPos, 3) + cubFunc.x_2 * pow (xPos, 2) + cubFunc.x_1 * xPos + cubFunc.x_0;
	
	cubFunc.angleTheo = atan(3 * cubFunc.x_3 * pow(xPos, 2) + 2 * cubFunc.x_2 * xPos + cubFunc.x_1);
	cubFunc.angleTheo = cubFunc.angleTheo * RAD_TO_DEG;
}

void sinusoidal(float xPos, FuncInfo & sinFunc)
{
	sinFunc.yTheo = (sinFunc.a)*sin(sinFunc.b * (xPos - sinFunc.c)) + sinFunc.d;
	
	sinFunc.angleTheo = atan(sinFunc.a*sinFunc.b*cos(sinFunc.b * (xPos - sinFunc.c)));
	sinFunc.angleTheo = sinFunc.angleTheo * RAD_TO_DEG;
}

void funcReturn(float xPos, FuncInfo & func)
{
	if(func.type == 0)
		linear(xPos, func);

	else if(func.type == 1)
		quadratic(xPos, func);

	else if(func.type == 2)
		cubic(xPos, func);
	else
		sinusoidal(xPos, func);
}

void draw(BoardInfo board, FuncInfo func, Point & pos)
{
	// Assuming robot starts at left side of board pointing the right direction
	markerDown(true);
	wait1Msec(100);
	motor[motorA] = motor[motorD] = DRIVE_SPD;
	funcReturn(pos.x, func);

	while (pos.x < board.high_x)
	{
		// if function is within boundaries, continue drawing
		if (func.yTheo > board.low_y  && func.yTheo < board.high_y )
		{
			nMotorEncoder[motorA] = 0;
			nMotorEncoder[motorD] = 0;

			int testAngle = getGyroDegrees(S1);
			int theoAngle = (int)func.angleTheo;
			// proportional angle difference 
			int pAngleDiff = (testAngle - theoAngle) * 2; 

			if(DRIVE_SPD + fabs(pAngleDiff) > 100) //maxing out speed to not exceed 100, retains direction (+/-)
				pAngleDiff = (100 - DRIVE_SPD) * pAngleDiff / fabs(pAngleDiff); 
			
			// adjusts motor speeds to correct the direction of movement
			motor[motorA] = DRIVE_SPD + pAngleDiff;
			motor[motorD] = DRIVE_SPD - pAngleDiff;
			
			//repeat loop every 50ms to avoid very small changes being rounded to 0
			wait1Msec(50);
			pos.x += (nMotorEncoder[motorA] + nMotorEncoder[motorD]) / 2.0 * cosDegrees((getGyroDegrees(S1) + testAngle) / 2.0)
					 * DEG_TO_CM / board.xUnitDist;
			pos.y += (nMotorEncoder[motorA] + nMotorEncoder[motorD]) / 2.0 * sinDegrees((getGyroDegrees(S1) + testAngle) / 2.0)
					 * DEG_TO_CM / board.yUnitDist;
			
			//uptdate position
			funcReturn(pos.x, func);
		}
		/* when function is out of bounds, turn horizontally and move forward until function re-enters boundaries 
		or robot travells outstide of boundaries */
		else
		{
			markerDown(false);
			rotateRobotTo(0);
			motor[motorA] = motor[motorD] = DRIVE_SPD;
			
			while((func.yTheo > board.high_y || func.yTheo < board.low_y) && pos.x < board.high_x)
			{
				// similar to main draw loop, except only adjusts robot to point 0 degrees 
				nMotorEncoder[motorA] = 0;
				nMotorEncoder[motorD] = 0;

				int testAngle = getGyroDegrees(S1);
				int pAngleDiff = testAngle * 2;

				if(DRIVE_SPD + fabs(pAngleDiff) > 100)
					pAngleDiff = (100 - DRIVE_SPD) * pAngleDiff / fabs(pAngleDiff);
					
				motor[motorA] = DRIVE_SPD + pAngleDiff;
				motor[motorD] = DRIVE_SPD - pAngleDiff;

				wait1Msec(50);
				pos.x += (nMotorEncoder[motorA] + nMotorEncoder[motorD]) / 2.0 * cosDegrees((getGyroDegrees(S1) + testAngle) / 2.0)
						 * DEG_TO_CM / board.xUnitDist;
				pos.y += (nMotorEncoder[motorA] + nMotorEncoder[motorD]) / 2.0 * sinDegrees((getGyroDegrees(S1) + testAngle) / 2.0)
						 * DEG_TO_CM / board.yUnitDist;
				
				funcReturn(pos.x, func);
			}
			
			motor[motorA] = motor[motorD] = 0;

			if(pos.x < board.high_x) // if not at end, continue drawing
			{
				markerDown(true);
				rotateRobotTo(func.angleTheo);
				motor[motorA] = motor[motorD] = DRIVE_SPD;
			}
		}
	}
	//after drawing, stop robot and raise marker
	motor[motorA] = motor[motorD] = 0;
	markerDown(false);
}

void moveToLowX(Point & current, BoardInfo board)
{
	int moveBackX = 0;
	int moveBackY = 0;

	// move to left side of board after function ends
	moveBackX = current.x - board.low_x;
	moveBackY = current.y;
	moveDist(moveBackX * board.xUnitDist, -20, 0);
	
	if (moveBackY < 0)
	{ // if robot is below x-axis, move up
		rotateRobotTo(90);
		moveDist(moveBackY * board.yUnitDist, 20, 90);
	}
	else if (moveBackY > 0)
	{ // if robot is above x-axis, move down
		rotateRobotTo(-90);
		moveDist(moveBackY * board.yUnitDist, 20, -90);
	}
	
	// updating position
	current.x = board.low_x;
	current.y = 0;
}

// This function moves the robot to the start position of the math function to be drawn
void moveToStartPosition(BoardInfo board, FuncInfo func, Point & current) 
{
	funcReturn(board.low_x, func);
	float startY = func.yTheo;

	// if the function starts out of bounds, the robot will move to the top/bottom of the board and rotate to face forwards
	if (startY > board.high_y) 
	{
		rotateRobotTo(90);
		moveDist(board.high_y * board.yUnitDist, DRIVE_SPD, 90);
		rotateRobotTo(0);
		current.y = board.high_y;
	}
	else if (startY < board.high_y)
	{
		rotateRobotTo(-90);
		moveDist(fabs(board.low_y * board.yUnitDist), DRIVE_SPD, -90);
		rotateRobotTo(0);
		current.y = board.low_y;
	}
	else // if the funciton starts within the bounds, move to the y-position and rotate to face forwards
	{
		if(startY > 0)
		{
			rotateRobotTo(90);
			moveDist(startY * board.yUnitDist, DRIVE_SPD, 90);
		}
		else
		{
			rotateRobotTo(-90);
			moveDist(fabs(startY * board.yUnitDist), DRIVE_SPD, -90);
		}
		rotateRobotTo(func.angleTheo);
		current.y = startY;
	}
	current.x = board.low_x;
}

task main()
{
	SensorType[S1] = sensorEV3_Gyro;
	wait1Msec(50);
	SensorMode[S1] = modeEV3Gyro_RateAndAngle;
	wait1Msec(50);
	while (getGyroRate(S1) != 0) //until still and steady
	{}
	resetGyro(S1);

	Point current;
	current.x = 0;
	current.y = 0;

	FuncInfo funcToDraw;
	funcToDraw.type = 0;
	funcToDraw.x_3 = 0;
	funcToDraw.x_2 = 0;
	funcToDraw.x_1 = 0;
	funcToDraw.x_0 = 0;
	funcToDraw.a = 0;
	funcToDraw.b = 0;
	funcToDraw.c = 0;
	funcToDraw.d = 0;
	funcToDraw.yTheo = 0;
	funcToDraw.angleTheo = 0;

	TFileHandle finFunc; TFileHandle finBoard;
	bool fileOK1 = openReadPC(finFunc, "demo.txt");
	bool fileOK2 = openReadPC(finBoard, "board.txt");

	if (!fileOK1 || !fileOK2)
		displayString(1, "File not found.");
	else
		displayString(1, "File found.");

	BoardInfo board;
	board.low_x = 0;
	board.high_x = 0;
	board.low_y = 0;
	board.high_y = 0;
	board.widthCM = 0;
	board.lengthCM = 0;
	board.xUnitDist = 0;
	board.yUnitDist = 0;

	boardIO(board, finBoard);
	closeFilePC(finBoard);

	drawAxis(board, current);

	int numFunctions = 0;
	readIntPC (finFunc, numFunctions);

	// math function drawing
	for (int count = 1; count <= numFunctions; count ++)
	{
		fileIO(funcToDraw, finFunc);

		if (count != 1)
			moveToLowX(current, board);
		
		moveToStartPosition(board, funcToDraw, current,);
		markerDown(true);

		displayTextLine(10, "Press enter");
		while(!getButtonPress(buttonEnter))
		{}
		while(getButtonPress(buttonAny))
		{}
		eraseDisplay();

		draw(board, funcToDraw, current);
		markerDown(false);
	}

	displayTextLine(10, "Press down to end");
	while(!getButtonPress(buttonDown)){}
	while(getButtonPress(buttonAny)){}
}
