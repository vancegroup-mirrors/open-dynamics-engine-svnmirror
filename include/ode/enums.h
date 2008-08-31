/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2008 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#ifndef _ODE_ENUMS_H_
#define _ODE_ENUMS_H_
#include <ode/odeconfig.h>




/* joint type numbers */

typedef enum {
  dJointTypeNone = 0,		/* or "unknown" */
  dJointTypeBall,
  dJointTypeHinge,
  dJointTypeSlider,
  dJointTypeContact,
  dJointTypeUniversal,
  dJointTypeHinge2,
  dJointTypeFixed,
  dJointTypeNull,
  dJointTypeAMotor,
  dJointTypeLMotor,
  dJointTypePlane2D,
  dJointTypePR,
  dJointTypePU,
  dJointTypePiston
} dJointType;



/* standard joint parameter names. why are these here? - because we don't want
 * to include all the joint function definitions in joint.cpp. hmmmm.
 * MSVC complains if we call D_ALL_PARAM_NAMES_X with a blank second argument,
 * which is why we have the D_ALL_PARAM_NAMES macro as well. please copy and
 * paste between these two.
 */

typedef enum dJointParam {
    dParamLoStop = 0x000,
    dParamHiStop,
    dParamVel,
    dParamFMax,
    dParamFudgeFactor,
    dParamBounce,
    dParamCFM,
    dParamStopERP,
    dParamStopCFM,
    dParamSuspensionERP,
    dParamSuspensionCFM,
    dParamERP,

    dParamsInGroup, ///< Number of parameter in a group

  /* add a multiple of this constant to the basic parameter numbers to get
   * the parameters for the second, third etc axes.
   */
    dParamGroup=0x100,

    dParamGroup1 = 0x000,
    dParamLoStop1 = 0x000,
    dParamHiStop1,
    dParamVel1,
    dParamFMax1,
    dParamFudgeFactor1,
    dParamBounce1,
    dParamCFM1,
    dParamStopERP1,
    dParamStopCFM1,
    dParamSuspensionERP1,
    dParamSuspensionCFM1,
    dParamERP1,

    dParamGroup2 = dParamGroup,
    dParamLoStop2 = dParamGroup,
    dParamHiStop2,
    dParamVel2,
    dParamFMax2,
    dParamFudgeFactor2,
    dParamBounce2,
    dParamCFM2,
    dParamStopERP2,
    dParamStopCFM2,
    dParamSuspensionERP2,
    dParamSuspensionCFM2,
    dParamERP2,

    dParamGroup3 = dParamGroup*2,
    dParamLoStop3 = dParamGroup*2,
    dParamHiStop3,
    dParamVel3,
    dParamFMax3,
    dParamFudgeFactor3,
    dParamBounce3,
    dParamCFM3,
    dParamStopERP3,
    dParamStopCFM3,
    dParamSuspensionERP3,
    dParamSuspensionCFM3,
    dParamERP3,

} dJointParam;


/* angular motor mode numbers */

typedef enum dAMotorMode {
  dAMotorUser = 0,
  dAMotorEuler = 1
} dAMotorMode;


#endif

