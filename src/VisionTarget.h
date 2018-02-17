/*
 * VisionTarget.h
 *
 *  Created on: Feb 13, 2018
 *      Author: Developer
 */

#ifndef SRC_VISIONTARGET_H_
#define SRC_VISIONTARGET_H_
#include <WPILib.h>
#include <networktables\NetworkTable.h>
#include <networktables\NetworkTableInstance.h>

using namespace nt;

class VisionTarget {
public:
	VisionTarget();
	void Init();
	void Loop();
	double GetVisionX();
	void Stop();
	virtual ~VisionTarget();
private:
	NetworkTableInstance m_nettableinst;
	std::shared_ptr<NetworkTable> m_nettable;
	double m_robocounter;
};

#endif /* SRC_VISIONTARGET_H_ */
