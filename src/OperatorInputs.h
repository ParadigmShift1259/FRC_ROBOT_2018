/**
 *  OperatorInputs.h
 *  Date:
 *  Last Edited By:
 */

#ifndef SRC_OPERATORINPUTS_H_
#define SRC_OPERATORINPUTS_H_


#include <WPILib.h>
#include <map>
#include <vector>


using namespace std;


class OperatorInputs
{
public:
	enum ToggleChoice { kToggle = 0, kHold = 1 };

	OperatorInputs();
	~OperatorInputs();
	// xbox
	double xBoxLeftX(unsigned int i = 0);
	double xBoxRightX(unsigned int i = 0);
	double xBoxLeftY(unsigned int i = 0);
	double xBoxRightY(unsigned int i = 0);
	bool xBoxAButton(ToggleChoice choice = kToggle, unsigned int i = 0);
	bool xBoxBButton(ToggleChoice choice = kToggle, unsigned int i = 0);
	bool xBoxXButton(ToggleChoice choice = kToggle, unsigned int i = 0);
	bool xBoxYButton(ToggleChoice choice = kToggle, unsigned int i = 0);
	bool xBoxLeftBumper(ToggleChoice choice = kToggle, unsigned int i = 0);
	bool xBoxRightBumper(ToggleChoice choice = kToggle, unsigned int i = 0);
	bool xBoxLeftTrigger(ToggleChoice choice = kToggle, unsigned int i = 0);
	bool xBoxRightTrigger(ToggleChoice choice = kToggle, unsigned int i = 0);
	bool xBoxStartButton(ToggleChoice choice = kToggle, unsigned int i = 0);
	bool xBoxBackButton(ToggleChoice choice = kToggle, unsigned int i = 0);
	bool xBoxDPadUp(ToggleChoice choice = kToggle, unsigned int i = 0);
	bool xBoxDPadRight(ToggleChoice choice = kToggle, unsigned int i = 0);
	bool xBoxDPadDown(ToggleChoice choice = kToggle, unsigned int i = 0);
	bool xBoxDPadLeft(ToggleChoice choice = kToggle, unsigned int i = 0);
	bool xBoxR3(ToggleChoice choice = kToggle, unsigned int i = 0);
	// joystick
	double joystickX();
	double joystickY();
	double joystickZ();
	bool joystickTrigger(ToggleChoice choice = kToggle);
	bool joystickAxis0Left(ToggleChoice choice = kToggle);
	bool joystickAxis0Right(ToggleChoice choice = kToggle);
	bool joystickAxis1Back(ToggleChoice choice = kToggle);
	bool joystickAxis1Forward(ToggleChoice choice = kToggle);
	bool joystickButton2(ToggleChoice choice = kToggle);
	bool joystickButton3(ToggleChoice choice = kToggle);
	bool joystickButton5(ToggleChoice choice = kToggle);
	bool joystickButton6(ToggleChoice choice = kToggle);
	bool joystickButton7(ToggleChoice choice = kToggle);
	bool joystickButton8(ToggleChoice choice = kToggle);
	bool joystickButton9(ToggleChoice choice = kToggle);
	bool joystickButton10(ToggleChoice choice = kToggle);

protected:
	Joystick *m_joystick;
	vector<Joystick*> m_xbox;

private:
	bool toggle(string buttonName, bool buttonValue);
	double deadzoneFilterY(double joyStickValue);
	double deadzoneFilterX(double joyStickValue);
	double deadzoneFilterZ(double joyStickValue);

	map<string, bool> m_togglebuttons;
};


#endif /* SRC_OPERATORINPUTS_H_ */
