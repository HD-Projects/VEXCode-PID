using namespace vex;

extern brain Brain;
extern motor rightFWD;
extern motor leftFWD;
extern motor rightBack;
extern motor leftBack;
extern inertial turning;
extern PID::DrivePID test_PID;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
