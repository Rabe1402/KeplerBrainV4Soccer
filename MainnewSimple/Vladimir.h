
extern bool isLine[8];
extern int lineValues[8];
void CheckLines(int pin);
void Vladimir_Init();


extern int frontDistance;
extern void CheckDistance();


extern bool seesBall;
extern int ballX;
extern int ballY;
extern bool isYellowGoal;
extern int goalX;
extern int goalY;
extern bool seesGoal;
void CheckCamera();



//Line script declarations
#define noiseSample 5
#define averagingSample 3
#define lineThreshold 18
#define lineThresholdForward 22
extern bool useForwardSensor;
#define smartForwardSensor true



#define maxFrontDistance 13
#define maxGoalMismatches 3

