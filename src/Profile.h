/*
 * Profile.h
 *
 *  Created on: Jan 13, 2018
 *      Author: Matt Wildman
 */

#ifndef SRC_PROFILE_H_
#define SRC_PROFILE_H_

/*
 * Example of what will be in there
 */
const double ProfileSize = 4;


//Left Profile
const double mMotionProfile1[][3] = {
{0,	0	,10},
{0.00004761904762,	0.5714285714	,10},
{0.0002142857143,	1.428571429	,10},
{0.0005476190476,	2.571428571	,10}};

//Right Profile
const double mMotionProfile2[][3] = {
{0,	0	,10},
{0.00004761904762,	0.5714285714	,10},
{0.0002142857143,	1.428571429	,10},
{0.0005476190476,	2.571428571	,10}};



#endif /* SRC_PROFILE_H_ */
