/**
 *  Const.h
 *  Date:
 *  Last Edited By:
 */


#ifndef SRC_CONST_H_
#define SRC_CONST_H_


//OperatorInputs
//	Controllers
#define INP_DUAL 1
#define INP_JOYSTICK -1
#define INP_XBOX_1 0
#define INP_XBOX_2 1
//	Set to 1.0 or -1.0
#define INVERT_Y_AXIS 1.0
#define INVERT_X_AXIS -1.0
//	XBox Controller
//		Buttons
#define A_BUTTON  1
#define B_BUTTON  2
#define X_BUTTON  3
#define Y_BUTTON  4
#define RIGHT_BUMPER  6
#define LEFT_BUMPER  5
#define BACK_BUTTON  7
#define START_BUTTON  8
//		XBox Triggers -- Changed for 2016, previously XBOX triggers were both on a single axis
#define XBOX_RIGHT_TRIGGER_AXIS  3
#define XBOX_LEFT_TRIGGER_AXIS  2
#define RIGHT_TRIGGER_MIN  0.5
#define RIGHT_TRIGGER_MAX  1.0
#define LEFT_TRIGGER_MIN  0.5
#define LEFT_TRIGGER_MAX  1.0
#define JOYSTICK_X_AXIS  0
#define JOYSTICK_Y_AXIS  1
#define AXIS0_LEFT_MIN -1.0
#define AXIS0_LEFT_MAX -0.75
#define AXIS0_RIGHT_MIN 0.75
#define AXIS0_RIGHT_MAX 1.0
#define AXIS1_BACK_MIN -1.0
#define AXIS1_BACK_MAX -0.75
#define AXIS1_FORWARD_MIN 0.75
#define AXIS1_FORWARD_MAX 1.0
//	Controller Dead Zones
#define DEADZONE_Y  0.18
#define DEADZONE_X  0.18
#define DEADZONE_Z  0.18


// Drivetrain
#define DT_DEFAULT_DIRECTION 1.0
// Inverts
#define INVERT_LEFT -1.0
#define INVERT_RIGHT 1.0			// 2017 code is 1, WPILlib DifferentialDrive is -1 (adjusted in DriveTrain::Drive())
// Talons ports
#define CAN_LEFT_PORT_1 2			// 2
#define CAN_LEFT_PORT_2 0			// 0
#define CAN_LEFT_PORT_3 -1
#define CAN_RIGHT_PORT_1 1			// 1
#define CAN_RIGHT_PORT_2 3			// 3
#define CAN_RIGHT_PORT_3 -1
// Shifter
#define PCM_SHIFT_PORT_LOW -1		// 0
#define PCM_SHIFT_MODULE 0
#define CHILD_PROOF_SPEED 0.75
#define FLIP_HIGH_GEAR true
// Ramping
#define RAMPING_RATE_PERIOD 0.10
#define RAMPING_RATE_MIN 0.6
#define RAMPING_RATE_MAX 4.0
#define X_SCALING 1.0
#define Y_SCALING 1.0
#define LEFT_MOTOR_SCALING 0.9915
#define RIGHT_MOTOR_SCALING 1
#define LOWSPEED_MODIFIER_X 0.75
#define LOWSPEED_MODIFIER_Y 0.25
// Encoders
#define ENC_LEFT_1 FeedbackDevice::QuadEncoder
#define ENC_LEFT_2 FeedbackDevice::None
#define ENC_RIGHT_1	FeedbackDevice::QuadEncoder
#define ENC_RIGHT_2	FeedbackDevice::None
#define CODES_PER_REV 4416.0
#define CODES_PER_INCH 234.979
#define WHEEL_DIAMETER 6.0
#define WHEEL_TRACK 22.50


// Compressor
#define PCM_COMPRESSOR_SOLENOID 0


// Lifter
#define CAN_LIFTER_MOTOR 6			// 6
#define PCM_LIFTER_MODULE 0			// 0
#define PCM_LIFTER_SOLENOID 2		// 2
#define LIF_RAISESPEED -1.0
#define LIF_LOWERSPEED 1.0			// 0.75
#define LIF_LIFTERMAX 24800			// 24050
#define LIF_LIFTERMIN 300								/// minimum cutoff
#define LIF_LIFSMIDGE (LIF_LIFTERMAX * 0.07)			/// 7% of max height
#define LIF_LIFTERMINSPD (LIF_LIFTERMAX * 0.1)			/// 5% of max height
#define LIF_LIFTERSWITCH (LIF_LIFTERMAX * 0.4)			/// ~40% of max height
#define LIF_LIFTERSTART (LIF_LIFTERMAX * 0.47)			/// ~50% of max height
#define LIF_LIFTERMAXSPD (LIF_LIFTERMAX	* 0.95)			/// 5% of max height


// Intake
#define CAN_INTAKE_LEFTMOTOR 4		// 4
#define CAN_INTAKE_RIGHTMOTOR 5		// 5
#define PCM_INTAKE_MODULE 0			// 0
#define PCM_INTAKE_SOLENOID 1		// 1
#define DIO_INTAKE_CUBESENSOR 0		// 0
#define INT_INGESTSPEED 0.5
#define INT_EJECTHIGH -0.5
#define INT_EJECTMED -0.375
#define INT_EJECTLOW -0.25
#define INT_EJECTSWITCH -0.325
#define INT_EJECTSCALE -0.5


// Climber
#define CAN_CLIMBER_MOTOR 7			// 7


// Autonomous
enum AutoMode {
	kAutoDefault,
	kAutoCenterSwitchLeft1, kAutoCenterSwitchRight1,
	kAutoCenterSwitchLeft3, kAutoCenterSwitchRight3,
	kAutoLeftScaleLeft2, kAutoLeftScaleLeft1, kAutoLeftScaleRight1,
	kAutoRightScaleLeft1, kAutoRightScaleRight2, kAutoRightScaleRight1,
	kAutoStraight,
	kAutoTest
};

extern AutoMode automode;


#endif /* SRC_CONST_H_ */
