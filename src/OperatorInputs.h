/**
 *  OperatorInputs.h
 *  Date:
 *  Last Edited By:
 */

#include <Joystick.h>
#include <map>
#include <vector>


using namespace std;


#ifndef SRC_OPERATORINPUTS_H_
#define SRC_OPERATORINPUTS_H_


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
	bool button2(ToggleChoice choice = kToggle);
	bool button3(ToggleChoice choice = kToggle);
	bool button5(ToggleChoice choice = kToggle);
	bool button6(ToggleChoice choice = kToggle);
	bool button7(ToggleChoice choice = kToggle);
	bool button8(ToggleChoice choice = kToggle);
	bool button9(ToggleChoice choice = kToggle);
	bool button10(ToggleChoice choice = kToggle);

	Joystick *m_joystick;
	vector<Joystick*> m_xbox;

private:
	bool toggle(string buttonName, bool buttonValue);
	double deadzoneFilterY(double joyStickValue);
	double deadzoneFilterX(double joyStickValue);
	double deadzoneFilterZ(double joyStickValue);

protected:
	 map<string, bool> m_togglebuttons;
};


#endif /* SRC_OPERATORINPUTS_H_ */
