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

	OperatorInputs(string curconfig = "SingleXBox");
	~OperatorInputs();
	// xbox
	double xBoxLeftX(unsigned int i = m_controllermap[m_curconfig]["xBoxLeftX"]);
	double xBoxRightX(unsigned int i = m_controllermap[m_curconfig]["xBoxRightX"]);
	double xBoxLeftY(unsigned int i = m_controllermap[m_curconfig]["xBoxLeftY"]);
	double xBoxRightY(unsigned int i = m_controllermap[m_curconfig]["xBoxRightY"]);
	bool xBoxAButton(ToggleChoice choice = kToggle, unsigned int i = m_controllermap[m_curconfig]["xBoxAButton"]);
	bool xBoxBButton(ToggleChoice choice = kToggle, unsigned int i = m_controllermap[m_curconfig]["xBoxBButton"]);
	bool xBoxXButton(ToggleChoice choice = kToggle, unsigned int i = m_controllermap[m_curconfig]["xBoxXButton"]);
	bool xBoxYButton(ToggleChoice choice = kToggle, unsigned int i = m_controllermap[m_curconfig]["xBoxYButton"]);
	bool xBoxLeftBumper(ToggleChoice choice = kToggle, unsigned int i = m_controllermap[m_curconfig]["xBoxLeftBumper"]);
	bool xBoxRightBumper(ToggleChoice choice = kToggle, unsigned int i = m_controllermap[m_curconfig]["xBoxRightBumper"]);
	bool xBoxLeftTrigger(ToggleChoice choice = kToggle, unsigned int i = m_controllermap[m_curconfig]["xBoxLeftTrigger"]);
	bool xBoxRightTrigger(ToggleChoice choice = kToggle, unsigned int i = m_controllermap[m_curconfig]["xBoxRightTrigger"]);
	bool xBoxStartButton(ToggleChoice choice = kToggle, unsigned int i = m_controllermap[m_curconfig]["xBoxStartButton"]);
	bool xBoxBackButton(ToggleChoice choice = kToggle, unsigned int i = m_controllermap[m_curconfig]["xBoxBackButton"]);
	bool xBoxDPadUp(ToggleChoice choice = kToggle, unsigned int i = m_controllermap[m_curconfig]["xBoxDPadUp"]);
	bool xBoxDPadRight(ToggleChoice choice = kToggle, unsigned int i = m_controllermap[m_curconfig]["xBoxDPadRight"]);
	bool xBoxDPadDown(ToggleChoice choice = kToggle, unsigned int i = m_controllermap[m_curconfig]["xBoxDPadDown"]);
	bool xBoxDPadLeft(ToggleChoice choice = kToggle, unsigned int i = m_controllermap[m_curconfig]["xBoxDPadLeft"]);
	bool xBoxR3(ToggleChoice choice = kToggle, unsigned int i = m_controllermap[m_curconfig]["xBoxR3"]);
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
	static string m_curconfig;

	map<string, bool> m_togglebuttons;
	static map<string, map<string, int> > m_controllermap;
};


#endif /* SRC_OPERATORINPUTS_H_ */
