/**
 *  OperatorInputs.cpp
 *  Date:
 *  Last Edited By:
 */


#include <OperatorInputs.h>
#include "Const.h"
#include <cmath>
#include "smartdashboard/smartdashboard.h"


using namespace std;


OperatorInputs::OperatorInputs()
{
	m_joystick = new Joystick(0);
	m_xbox.push_back(new Joystick(1));
	m_xbox.push_back(new Joystick(2));
}


OperatorInputs::~OperatorInputs()
{
	delete m_joystick;
	for (std::vector< Joystick* >::iterator it = m_xbox.begin() ; it != m_xbox.end(); it++)
	{
	     delete (*it);
	}
	m_xbox.clear();
}


double OperatorInputs::xBoxLeftX(unsigned int i)
{
	if (i < m_xbox.size())
		return deadzoneFilterX(INVERT_X_AXIS * m_xbox[i]->GetX(GenericHID::JoystickHand::kLeftHand));
	return false;
}


double OperatorInputs::xBoxRightX(unsigned int i)
{
	if (i < m_xbox.size())
		return deadzoneFilterX(m_xbox[i]->GetX(GenericHID::JoystickHand::kRightHand));
	return false;
}


double OperatorInputs::xBoxLeftY(unsigned int i)
{
	if (i < m_xbox.size())
		return deadzoneFilterY(INVERT_Y_AXIS * m_xbox[i]->GetY(GenericHID::JoystickHand::kLeftHand));
	return false;
}


double OperatorInputs::xBoxRightY(unsigned int i)
{
	if (i < m_xbox.size())
		return deadzoneFilterY(m_xbox[i]->GetY(GenericHID::JoystickHand::kRightHand));
	return false;
}


bool OperatorInputs::xBoxAButton(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(A_BUTTON);

		if (choice == kToggle)
			return toggle("xBoxAButton", button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxBButton(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(B_BUTTON);

		if (choice == kToggle)
			return toggle("xBoxBButton", button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxXButton(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(X_BUTTON);

		if (choice == kToggle)
			return toggle("xBoxXButton", button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxYButton(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(Y_BUTTON);

		if (choice == kToggle)
			return toggle("xBoxYButton", button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxLeftBumper(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(LEFT_BUMPER);

		if (choice == kToggle)
			return toggle("xBoxLeftBumper", button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxRightBumper(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(RIGHT_BUMPER);

		if (choice == kToggle)
			return toggle("xBoxRightBumper", button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxLeftTrigger(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		double axis = m_xbox[i]->GetRawAxis(XBOX_LEFT_TRIGGER_AXIS);

		if (choice == kToggle)
			return toggle("xBoxLeftTrigger", (LEFT_TRIGGER_MIN <= axis) && (axis <= LEFT_TRIGGER_MAX));
		if (choice == kHold)
			return ((LEFT_TRIGGER_MIN <= axis) && (axis <= LEFT_TRIGGER_MAX));
	}
	return false;
}


bool OperatorInputs::xBoxRightTrigger(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		double axis = m_xbox[i]->GetRawAxis(XBOX_RIGHT_TRIGGER_AXIS);

		if (choice == kToggle)
			return toggle("xBoxRightTrigger", (RIGHT_TRIGGER_MIN <= axis && axis <= RIGHT_TRIGGER_MAX));
		if (choice == kHold)
			return (RIGHT_TRIGGER_MIN <= axis && axis <= RIGHT_TRIGGER_MAX);
	}
	return false;
}


bool OperatorInputs::xBoxStartButton(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(START_BUTTON);

		if (choice == kToggle)
			return toggle("xBoxStartButton", button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxBackButton(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(BACK_BUTTON);

		if (choice == kToggle)
			return toggle("xBoxBackButton", button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxDPadUp(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = (m_xbox[i]->GetPOV() == 0);

		if (choice == kToggle)
			return toggle("xBoxDPadUp", button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxDPadRight(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = (m_xbox[i]->GetPOV() == 90);

		if (choice == kToggle)
			return toggle("xBoxDPadLeft", button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxDPadDown(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
	bool button = (m_xbox[i]->GetPOV() == 180);

	if (choice == kToggle)
		return toggle("xBoxDPadDown", button);
	if (choice == kHold)
		return button;
	}
	return false;
}


bool OperatorInputs::xBoxDPadLeft(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = (m_xbox[i]->GetPOV() == 270);

		if (choice == kToggle)
			return toggle("xBoxDPadRight", button);
		if (choice == kHold)
			return button;
	}
	return false;
}


bool OperatorInputs::xBoxR3(ToggleChoice choice, unsigned int i)
{
	if (i < m_xbox.size())
	{
		bool button = m_xbox[i]->GetRawButton(10);

		if (choice == kToggle)
			return toggle("xBoxR3", button);
		if (choice == kHold)
			return button;
	}
	return false;
}


double OperatorInputs::joystickX()
{
	return deadzoneFilterX(INVERT_X_AXIS * m_joystick->GetX());
}


double OperatorInputs::joystickY()
{
	return deadzoneFilterY(INVERT_Y_AXIS * m_joystick->GetY());
}


double OperatorInputs::joystickZ()
{
	return deadzoneFilterZ(m_joystick->GetZ());
}


bool OperatorInputs::joystickAxis0Left(ToggleChoice choice)
{
	double axis = m_joystick->GetRawAxis(JOYSTICK_X_AXIS);

	if (choice == kToggle)
		return toggle("joystickAxis0Left", (AXIS0_LEFT_MIN <= axis && axis <= AXIS0_LEFT_MAX));
	if (choice == kHold)
		return (AXIS0_LEFT_MIN <= axis && axis <= AXIS0_LEFT_MAX);
	return false;
}


bool OperatorInputs::joystickAxis0Right(ToggleChoice choice)
{
	double axis = m_joystick->GetRawAxis(JOYSTICK_X_AXIS);

	if (choice == kToggle)
		return toggle("joystickAxis0Right", (AXIS0_RIGHT_MIN <= axis && axis <= AXIS0_RIGHT_MAX));
	if (choice == kHold)
		return (AXIS0_RIGHT_MIN <= axis && axis <= AXIS0_RIGHT_MAX);
	return false;
}


bool OperatorInputs::joystickAxis1Back(ToggleChoice choice)
{
	double axis = m_joystick->GetRawAxis(JOYSTICK_Y_AXIS);

	if (choice == kToggle)
		return toggle("joystickAxis1Back", (AXIS1_BACK_MIN <= axis && axis <= AXIS1_BACK_MAX));
	if (choice == kHold)
		return (AXIS1_BACK_MIN <= axis && axis <= AXIS1_BACK_MAX);
	return false;
}


bool OperatorInputs::joystickAxis1Forward(ToggleChoice choice)
{
	double axis = m_joystick->GetRawAxis(JOYSTICK_Y_AXIS);

	if (choice == kToggle)
		return toggle("joystickAxis1Forward", (AXIS1_FORWARD_MIN <= axis && axis <= AXIS1_FORWARD_MAX));
	if (choice == kHold)
		return (AXIS1_FORWARD_MIN <= axis && axis <= AXIS1_FORWARD_MAX);
	return false;
}


bool OperatorInputs::joystickTrigger(ToggleChoice choice)
{
	bool button = m_joystick->GetTrigger();

	if (choice == kToggle)
		return toggle("joystickTrigger", button);
	if (choice == kHold)
		return button;
	return false;
}


bool OperatorInputs::button2(ToggleChoice choice)
{
	bool button = m_joystick->GetRawButton(2);

	if (choice == kToggle)
		return toggle("joystickbutton2", button);
	if (choice == kHold)
		return button;
	return false;
}


bool OperatorInputs::button3(ToggleChoice choice)
{
	bool button = m_joystick->GetRawButton(3);

	if (choice == kToggle)
		return toggle("joystickbutton3", button);
	if (choice == kHold)
		return button;
	return false;
}


bool OperatorInputs::button5(ToggleChoice choice)
{
	bool button = m_joystick->GetRawButton(5);

	if (choice == kToggle)
		return toggle("joystickbutton5", button);
	if (choice == kHold)
		return button;
	return false;
}


bool OperatorInputs::button6(ToggleChoice choice)
{
	bool button = m_joystick->GetRawButton(6);

	if (choice == kToggle)
		return toggle("joystickbutton6", button);
	if (choice == kHold)
		return button;
	return false;
}


bool OperatorInputs::button7(ToggleChoice choice)
{
	bool button = m_joystick->GetRawButton(7);

	if (choice == kToggle)
		return toggle("joystickbutton7", button);
	if (choice == kHold)
		return button;
	return false;
}


bool OperatorInputs::button8(ToggleChoice choice)
{
	bool button = m_joystick->GetRawButton(8);

	if (choice == kToggle)
		return toggle("joystickbutton8", button);
	if (choice == kHold)
		return button;
	return false;
}


bool OperatorInputs::button9(ToggleChoice choice)
{
	bool button = m_joystick->GetRawButton(9);

	if (choice == kToggle)
		return toggle("joystickbutton9", button);
	if (choice == kHold)
		return button;
	return false;
}


bool OperatorInputs::button10(ToggleChoice choice)
{
	bool button = m_joystick->GetRawButton(10);

	if (choice == kToggle)
		return toggle("joystickbutton10", button);
	if (choice == kHold)
		return button;
	return false;
}


bool OperatorInputs::toggle(string buttonname, bool buttonval)
{
	bool toggleval = !m_togglebuttons[buttonname] && buttonval;
	m_togglebuttons[buttonname] = buttonval;
	return toggleval;
}


double OperatorInputs::deadzoneFilterX(double joyStickValue)
{
	if (abs(joyStickValue) <= DEADZONE_X)
	{
		return 0;
	}
	double sub = joyStickValue / abs(joyStickValue);
	return (joyStickValue - (sub * DEADZONE_X)) / (1.0 - DEADZONE_X);
}


double OperatorInputs::deadzoneFilterY(double joyStickValue)
{
	if (abs(joyStickValue) <= DEADZONE_Y)
	{
		return 0;
	}
	double sub = joyStickValue / abs(joyStickValue);
	return (joyStickValue - (sub * DEADZONE_Y)) / (1.0 - DEADZONE_Y);
}


double OperatorInputs::deadzoneFilterZ(double joyStickValue)
{
	if (abs(joyStickValue) <= DEADZONE_Z)
	{
		return 0;
	}
	double sub = joyStickValue / abs(joyStickValue);
	return (joyStickValue - (sub * DEADZONE_Z)) / (1.0-DEADZONE_Z);
}
