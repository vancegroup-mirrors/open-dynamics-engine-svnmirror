/*************************************************************************
  *                                                                       *
  * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
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
//234567890123456789012345678901234567890123456789012345678901234567890123456789
//        1         2         3         4         5         6         7

////////////////////////////////////////////////////////////////////////////////
// This file create unit test for some of the functions found in:
// ode/src/joinst/hinge.cpp
//
//
////////////////////////////////////////////////////////////////////////////////

#include <UnitTest++.h>
#include <ode/ode.h>

#include "../../ode/src/joints/hinge.h"

SUITE (TestdxJointHinge)
{
    // The 2 bodies are positionned at (0, 0, 0), with no rotation
    // The joint is an Hinge Joint
    // Axis is along the X axis
    // Anchor at (0, 0, 0)
    //         ^Y
    //         |
    //         |
    //         |
    //         |
    //         |
    // Z <---- . (X going out of the page)
    struct dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X
    {
        dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);

            jId   = dJointCreateHinge (wId, 0);
            joint = (dxJointHinge*) jId;


            dJointAttach (jId, bId1, bId2);
            dJointSetHingeAnchor (jId, 0, 0, 0);

            axis[0] = 1;
            axis[1] = 0;
            axis[2] = 0;
        }

        ~dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;
        dBodyID bId2;


        dJointID jId;
        dxJointHinge* joint;

        dVector3 axis;
    };

    // Rotate 2nd body 90deg around X then back to original position
    //
    //   ^  ^       ^
    //   |  |  =>   |  <---
    //   |  |       |
    //  B1  B2     B1   B2
    //
    // Start with a Delta of 90deg
    //   ^           ^   ^
    //   | <---  =>  |   |
    //   |           |   |
    //  B1  B2      B1   B2
    TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetHingeAxisOffset_B2_90deg)
    {
        dMatrix3 R;

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

        dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  -M_PI/2.0);
        CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
    }


    // Rotate 2nd body -90deg around X then back to original position
    //
    //   ^  ^       ^
    //   |  |  =>   |  --->
    //   |  |       |
    //  B1  B2     B1   B2
    //
    // Start with a Delta of 90deg
    //   ^           ^   ^
    //   | --->  =>  |   |
    //   |           |   |
    //  B1  B2      B1   B2
    TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetHingeAxisOffset_B2_Minus90deg)
    {
        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

        dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  M_PI/2.0);
        CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
    }


    // Rotate 1st body 0.23rad around X then back to original position
    //
    //   ^  ^     ^      ^
    //   |  |  =>  \     |
    //   |  |       \    |
    //  B1  B2     B1   B2
    //
    // Start with a Delta of 0.23rad
    // ^    ^        ^   ^
    //  \   | =>     |   |
    //   \  |        |   |
    //  B1  B2      B1   B2
    TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetHingeAxisOffset_B1_0_23rad)
    {
        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, REAL(0.23) );
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

        dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  REAL(0.23));
        CHECK_CLOSE (REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
    }


    // Rotate 1st body -0.23rad around Z then back to original position
    //
    //   ^  ^         ^  ^
    //   |  |  =>    /   |
    //   |  |       /    |
    //  B1  B2     B1   B2
    //
    // Start with a Delta of 0.23rad
    //     ^ ^        ^   ^
    //    /  | =>     |   |
    //   /   |        |   |
    //  B1  B2      B1   B2
    TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
                  test_dJointSetHingeAxisOffset_B1_Minus0_23rad)
    {
        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, -REAL(0.23));
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (-REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

        dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  -REAL(0.23));
        CHECK_CLOSE (-REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
    }


    // The 2 bodies are positionned at (0, 0, 0), with no rotation
    // The joint is an Hinge Joint.
    // Axis in the inverse direction of the X axis
    // Anchor at (0, 0, 0)
    //         ^Y
    //         |
    //         |
    //         |
    //         |
    //         |
    // Z <---- x (X going out of the page)
    struct dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X
    {
        dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, -1, 0);

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 1, 0);

            jId   = dJointCreateHinge (wId, 0);
            joint = (dxJointHinge*) jId;


            dJointAttach (jId, bId1, bId2);
            dJointSetHingeAnchor (jId, 0, 0, 0);

            axis[0] = -1;
            axis[1] = 0;
            axis[2] = 0;
        }

        ~dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;
        dBodyID bId2;


        dJointID jId;
        dxJointHinge* joint;

        dVector3 axis;
    };

    // Rotate 2nd body 90deg around X then back to original position
    //
    //   ^  ^       ^
    //   |  |  =>   |  <---
    //   |  |       |
    //  B1  B2     B1   B2
    //
    // Start with a Delta of 90deg
    //   ^           ^   ^
    //   | <---  =>  |   |
    //   |           |   |
    //  B1  B2      B1   B2
    TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetHingeAxisOffset_B2_90Deg)
    {
        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

        dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  M_PI/2.0);
        CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
    }


    // Rotate 2nd body -90deg around X then back to original position
    //
    //   ^  ^       ^
    //   |  |  =>   |  --->
    //   |  |       |
    //  B1  B2     B1   B2
    //
    // Start with a Delta of 90deg
    //   ^           ^   ^
    //   | --->  =>  |   |
    //   |           |   |
    //  B1  B2      B1   B2
    TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetHingeAxisOffset_B2_Minus90Deg)
    {
        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, -M_PI/2.0);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

        dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  -M_PI/2.0);
        CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
    }


    // Rotate 1st body 0.23rad around X then back to original position
    //
    //   ^  ^     ^      ^
    //   |  |  =>  \     |
    //   |  |       \    |
    //  B1  B2     B1   B2
    //
    // Start with a Delta of 0.23rad
    // ^    ^        ^   ^
    //  \   | =>     |   |
    //   \  |        |   |
    //  B1  B2      B1   B2
    TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetHingeAxisOffset_B1_0_23rad)
    {
        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, REAL(0.23));
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (-REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

        dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  -REAL(0.23));
        CHECK_CLOSE (-REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
    }


    // Rotate 2nd body -0.23rad around Z then back to original position
    //
    //   ^  ^         ^  ^
    //   |  |  =>    /   |
    //   |  |       /    |
    //  B1  B2     B1   B2
    //
    // Start with a Delta of 0.23rad
    //     ^ ^        ^   ^
    //    /  | =>     |   |
    //   /   |        |   |
    //  B1  B2      B1   B2
    TEST_FIXTURE (dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetHingeAxisOffset_B1_Minus0_23rad)
    {
        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, -REAL(0.23));
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

        dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  REAL(0.23));
        CHECK_CLOSE (REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0.0, dJointGetHingeAngle (jId), 1e-4);
    }


    // Only one body body1 at (0,0,0)
    // The joint is an Hinge Joint.
    // Axis is along the X axis
    // Anchor at (0, 0, 0)
    //
    //       ^Y
    //       |
    //       |
    //       |
    //       |
    //       |
    // Z <-- X
    struct dxJointHinge_Fixture_B1_At_Zero_Axis_Along_X
    {
        dxJointHinge_Fixture_B1_At_Zero_Axis_Along_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            jId   = dJointCreateHinge (wId, 0);
            joint = (dxJointHinge*) jId;


            dJointAttach (jId, bId1, NULL);
            dJointSetHingeAnchor (jId, 0, 0, 0);

            axis[0] = 1;
            axis[1] = 0;
            axis[2] = 0;
        }

        ~dxJointHinge_Fixture_B1_At_Zero_Axis_Along_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;


        dJointID jId;
        dxJointHinge* joint;

        dVector3 axis;
    };

    // Rotate the body by 90deg around X then back to original position.
    // The body is attached at the second position of the joint:
    // dJointAttache(jId, 0, bId);
    //
    //   ^
    //   |  => <---
    //   |
    //  B1      B1
    //
    // Start with a Delta of 90deg
    //            ^
    //  <---  =>  |
    //            |
    //   B1      B1
    TEST_FIXTURE (dxJointHinge_Fixture_B1_At_Zero_Axis_Along_X,
                  test_dJointSetHingeAxisOffset_1Body_B1_90Deg)
    {
        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

        dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  M_PI/2.0);
        CHECK_CLOSE (M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
    }

    // Rotate the body by -0.23rad around X then back to original position.
    // The body is attached at the second position of the joint:
    // dJointAttache(jId, 0, bId);
    //
    //   ^         ^
    //   |  =>    /
    //   |       /
    //  B1      B1
    //
    // Start with a Delta of -0.23rad
    //     ^     ^
    //    /  =>  |
    //   /       |
    //   B1     B1
    TEST_FIXTURE (dxJointHinge_Fixture_B1_At_Zero_Axis_Along_X,
                  test_dJointSetHingeAxisOffset_1Body_B1_Minus0_23rad)
    {
        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, -REAL(0.23));
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (-REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

        dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  -REAL(0.23));
        CHECK_CLOSE (-REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
    }



    // Only one body body1 at (0,0,0)
    // The joint is an Hinge Joint.
    // Axis the inverse of the X axis
    // Anchor at (0, 0, 0)
    //
    //       ^Y
    //       |
    //       |
    //       |
    //       |
    //       |
    // Z <-- X
    struct dxJointHinge_Fixture_B1_At_Zero_Axis_Inverse_of_X
    {
        dxJointHinge_Fixture_B1_At_Zero_Axis_Inverse_of_X()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);

            jId   = dJointCreateHinge (wId, 0);
            joint = (dxJointHinge*) jId;


            dJointAttach (jId, bId1, NULL);
            dJointSetHingeAnchor (jId, 0, 0, 0);

            axis[0] = -1;
            axis[1] = 0;
            axis[2] = 0;
        }

        ~dxJointHinge_Fixture_B1_At_Zero_Axis_Inverse_of_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;


        dJointID jId;
        dxJointHinge* joint;

        dVector3 axis;
    };

    // Rotate B1 by 90deg around X then back to original position
    //
    //   ^
    //   |  => <---
    //   |
    //  B1      B1
    //
    // Start with a Delta of 90deg
    //            ^
    //  <---  =>  |
    //            |
    //   B1      B1
    TEST_FIXTURE (dxJointHinge_Fixture_B1_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetHingeAxisOffset_1Body_B1_90Deg)
    {
        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

        dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  -M_PI/2.0);
        CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
    }

    // Rotate B1 by -0.23rad around X then back to original position
    //
    //   ^         ^
    //   |  =>    /
    //   |       /
    //  B1      B1
    //
    // Start with a Delta of -0.23rad
    //     ^     ^
    //    /  =>  |
    //   /       |
    //   B1     B1
    TEST_FIXTURE (dxJointHinge_Fixture_B1_At_Zero_Axis_Inverse_of_X,
                  test_dJointSetHingeAxisOffset_1Body_B1_Minus0_23rad)
    {
        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, -REAL(0.23));
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

        dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  REAL(0.23));
        CHECK_CLOSE (REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId1, R);

        CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
    }





    // Only one body body2 at (0,0,0)
    // The joint is an Hinge Joint.
    // Axis is along the X axis
    // Anchor at (0, 0, 0)
    //
    //       ^Y
    //       |
    //       |
    //       |
    //       |
    //       |
    // Z <-- X
    struct dxJointHinge_Fixture_B2_At_Zero_Axis_Along_X
    {
        dxJointHinge_Fixture_B2_At_Zero_Axis_Along_X()
        {
            wId = dWorldCreate();

            bId2 = dBodyCreate (wId);
            dBodySetPosition (bId2, 0, 0, 0);

            jId   = dJointCreateHinge (wId, 0);
            joint = (dxJointHinge*) jId;


            dJointAttach (jId, NULL, bId2);
            dJointSetHingeAnchor (jId, 0, 0, 0);

            axis[0] = 1;
            axis[1] = 0;
            axis[2] = 0;
        }

        ~dxJointHinge_Fixture_B2_At_Zero_Axis_Along_X()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId2;


        dJointID jId;
        dxJointHinge* joint;

        dVector3 axis;
    };

    // Rotate B2 by 90deg around X then back to original position
    //
    //   ^
    //   |  => <---
    //   |
    //  B2      B2
    //
    // Start with a Delta of 90deg
    //            ^
    //  <---  =>  |
    //            |
    //   B2      B2
    TEST_FIXTURE (dxJointHinge_Fixture_B2_At_Zero_Axis_Along_X,
                  test_dJointSetHingeAxisOffset_1Body_B2_90Deg)
    {
        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, M_PI/2.0);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

        dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  -M_PI/2.0);
        CHECK_CLOSE (-M_PI/2.0, dJointGetHingeAngle (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
    }

    // Rotate B2 by -0.23rad around X then back to original position
    //
    //   ^         ^
    //   |  =>    /
    //   |       /
    //  B2      B2
    //
    // Start with a Delta of -0.23rad
    //     ^     ^
    //    /  =>  |
    //   /       |
    //   B2     B2
    TEST_FIXTURE (dxJointHinge_Fixture_B2_At_Zero_Axis_Along_X,
                  test_dJointSetHingeAxisOffset_1Body_B2_Minus0_23rad)
    {
        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, -REAL(0.23));
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

        dJointSetHingeAxisOffset (jId, axis[0], axis[1], axis[2],  REAL(0.23));
        CHECK_CLOSE (REAL(0.23), dJointGetHingeAngle (jId), 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, 0);
        dBodySetRotation (bId2, R);

        CHECK_CLOSE (0, dJointGetHingeAngle (jId), 1e-4);
    }



    // Create 2 bodies attached by a Hinge joint
    // Axis is along the X axis (Default value
    // Anchor at (0, 0, 0)      (Default value)
    //
    //       ^Y
    //       |
    //       * Body2
    //       |
    //       |
    // Body1 |
    // *     Z-------->
    struct dxJointHinge_Test_Initialization
    {
        dxJointHinge_Test_Initialization()
        {
            wId = dWorldCreate();

            // Remove gravity to have the only force be the force of the joint
            dWorldSetGravity(wId, 0,0,0);

            for (int j=0; j<2; ++j)
            {
                bId[j][0] = dBodyCreate (wId);
                dBodySetPosition (bId[j][0], -1, -2, -3);

                bId[j][1] = dBodyCreate (wId);
                dBodySetPosition (bId[j][1], 11, 22, 33);


                dMatrix3 R;
                dVector3 axis; // Random axis

                axis[0] =  REAL(0.53);
                axis[1] = -REAL(0.71);
                axis[2] =  REAL(0.43);
                dNormalize3(axis);
                dRFromAxisAndAngle (R, axis[0], axis[1], axis[2],
                                    REAL(0.47123)); // 27deg
                dBodySetRotation (bId[j][0], R);


                axis[0] =  REAL(1.2);
                axis[1] =  REAL(0.87);
                axis[2] = -REAL(0.33);
                dNormalize3(axis);
                dRFromAxisAndAngle (R, axis[0], axis[1], axis[2],
                                    REAL(0.47123)); // 27deg
                dBodySetRotation (bId[j][1], R);

                jId[j]   = dJointCreateHinge (wId, 0);
                dJointAttach (jId[j], bId[j][0], bId[j][1]);
                //        dJointSetHingeParam(jId[j], dParamLoStop, 1);
                //        dJointSetHingeParam(jId[j], dParamHiStop, 2);
                //        dJointSetHingeParam(jId[j], dParamFMax, 200);
            }
        }

        ~dxJointHinge_Test_Initialization()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId[2][2];


        dJointID jId[2];

    };


    // Test if setting a Hinge with its default values
    // will behave the same as a default Hinge joint
    TEST_FIXTURE (dxJointHinge_Test_Initialization,
                  test_Hinge_Initialization)
    {
        using namespace std;
        dVector3 axis;
        dJointGetHingeAxis(jId[1], axis);
        dJointSetHingeAxis(jId[1], axis[0], axis[1], axis[2]);


        dVector3 anchor;
        dJointGetHingeAnchor(jId[1], anchor);
        dJointSetHingeAnchor(jId[1], anchor[0], anchor[1], anchor[2]);


        for (int b=0; b<2; ++b)
        {
            // Compare body b of the first joint with its equivalent on the
            // second joint
            const dReal *qA = dBodyGetQuaternion(bId[0][b]);
            const dReal *qB = dBodyGetQuaternion(bId[1][b]);
            CHECK_CLOSE (qA[0], qB[0], 1e-6);
            CHECK_CLOSE (qA[1], qB[1], 1e-6);
            CHECK_CLOSE (qA[2], qB[2], 1e-6);
            CHECK_CLOSE (qA[3], qB[3], 1e-6);
        }

        dWorldStep (wId,0.5);
        dWorldStep (wId,0.5);
        dWorldStep (wId,0.5);
        dWorldStep (wId,0.5);

        for (int b=0; b<2; ++b)
        {
            // Compare body b of the first joint with its equivalent on the
            // second joint
            const dReal *qA = dBodyGetQuaternion(bId[0][b]);
            const dReal *qB = dBodyGetQuaternion(bId[1][b]);
            CHECK_CLOSE (qA[0], qB[0], 1e-6);
            CHECK_CLOSE (qA[1], qB[1], 1e-6);
            CHECK_CLOSE (qA[2], qB[2], 1e-6);
            CHECK_CLOSE (qA[3], qB[3], 1e-6);


            const dReal *posA = dBodyGetPosition(bId[0][b]);
            const dReal *posB = dBodyGetPosition(bId[1][b]);
            CHECK_CLOSE (posA[0], posB[0], 1e-6);
            CHECK_CLOSE (posA[1], posB[1], 1e-6);
            CHECK_CLOSE (posA[2], posB[2], 1e-6);
            CHECK_CLOSE (posA[3], posB[3], 1e-6);
        }
    }



    struct dxJointHinge_One_Body
    {
        dxJointHinge_One_Body()
        {
            wId = dWorldCreate();

            bId1 = dBodyCreate (wId);
            dBodySetPosition (bId1, 0, 0, 0);
            dBodyAddTorque (bId1, 4000, 0, 0);
            const dReal *pos = dBodyGetPosition(bId1);
            for (int i=0; i<3; ++i)
                start[i] = pos[i];

            jId   = dJointCreateHinge (wId, 0);
            dJointSetHingeParam(jId, dParamLoStop, 0);
            dJointSetHingeParam(jId, dParamHiStop, 0);

        }

        ~dxJointHinge_One_Body()
        {
            dWorldDestroy (wId);
        }

        dWorldID wId;

        dBodyID bId1;
        dBodyID bId2;


        dJointID jId;
        dxJointHinge* joint;

        dReal start[3];
        dReal end[3];

        dMatrix3 R;
    };



    // Test the limits [0, 0] when only one body at is attached to the joint
    // using dJointAttache(jId, bId, 0);
    //
    // The body should not move since their is no room between the twolimits
    TEST_FIXTURE(dxJointHinge_One_Body,
                 test_Limit_0_0_One_Body_on_left)
    {
        dJointAttach(jId, bId1, 0);

        for (int i=0; i<500; ++i)
            dWorldStep(wId, 1.0);

        const dReal *pos = dBodyGetPosition(bId1);
        for (int i=0; i<3; ++i)
            end[i] = pos[i];

        CHECK_CLOSE (start[0], end[0], 1e-4);
        CHECK_CLOSE (start[1], end[1], 1e-4);
        CHECK_CLOSE (start[2], end[2], 1e-4);
    }


    // Test the limits [0, 0] when only one body at is attached to the joint
    // using dJointAttache(jId, 0, bId);
    //
    // The body should not move since their is no room between the twolimits
    TEST_FIXTURE(dxJointHinge_One_Body,
                 test_Limit_0_0_One_Body_on_right)
    {
        dJointAttach(jId, 0, bId1);


        for (int i=0; i<500; ++i)
            dWorldStep(wId, 1.0);

        const dReal *pos = dBodyGetPosition(bId1);
        for (int i=0; i<3; ++i)
            end[i] = pos[i];

        CHECK_CLOSE (start[0], end[0], 1e-4);
        CHECK_CLOSE (start[1], end[1], 1e-4);
        CHECK_CLOSE (start[2], end[2], 1e-4);
    }



    // ============================================================================
    // Test limit crossing
    // ============================================================================
    dReal d2r(dReal degree)
    {
        return degree * (dReal)(M_PI / 180);
    }
    dReal r2d(dReal degree)
    {
        return degree * (dReal)(180.0/M_PI);
    }

    TEST_FIXTURE(dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
                 test_limit_crossing_hiStop)
    {
        // Only body 1 will be able to move.
        dJointID fixed = dJointCreateFixed(wId, 0);
        dJointAttach(fixed, 0, bId2);
        dJointSetFixed(fixed);


        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, d2r(169.5) );
        dBodySetRotation (bId1, R);
        dJointSetHingeAxisOffset(jId, 1, 0, 0, d2r(169.5));

        CHECK_CLOSE (r2d(dJointGetHingeAngle (jId)), 169.5, 1e-4);


        // The limit are set such that after the stepping
        // the angle will be between the 2 limits
        dJointSetHingeParam(jId, dParamLoStop, d2r(-90));
        dJointSetHingeParam(jId, dParamHiStop, d2r(170));

        dBodySetAngularVel(bId1, d2r(1), 0, 0);
        dWorldStep(wId, 1.0);
        dReal angle = r2d(dJointGetHingeAngle (jId));
        CHECK_CLOSE (170.5, angle, 5e-1);
        dWorldStep(wId, 1.0);


        int limit = ((dxJointHinge *)jId)->limot.limit;
        CHECK_EQUAL(2, limit);
    }



    // Test a simple loStop crossing with NO wrapping on the angle
    TEST_FIXTURE(dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
                 test_limit_crossing_loStop)
    {
        // Only body 1 will be able to move.
        dJointID fixed = dJointCreateFixed(wId, 0);
        dJointAttach(fixed, 0, bId2);
        dJointSetFixed(fixed);


        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, d2r(-89.5) );
        dBodySetRotation (bId1, R);
        dJointSetHingeAxisOffset(jId, 1, 0, 0, d2r(-89.5));

        CHECK_CLOSE (r2d(dJointGetHingeAngle (jId)), -89.5, 1e-4);


        // The limit are set such that after the stepping
        // the angle will be between the 2 limits
        dJointSetHingeParam(jId, dParamLoStop, d2r(-90));
        dJointSetHingeParam(jId, dParamHiStop, d2r(170));

        dBodySetAngularVel(bId1, d2r(-1), 0, 0);
        dWorldStep(wId, 1.0);
        dReal angle = r2d(dJointGetHingeAngle (jId));
        CHECK_CLOSE (-90.5, angle, 1e-1);
        dWorldStep(wId, 1.0);


        int limit = ((dxJointHinge *)jId)->limot.limit;

        CHECK_EQUAL(1, limit);
    }




    TEST_FIXTURE(dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
                 test_limit_crossing_hiStop_With_Wrapping)
    {
        // Only body 1 will be able to move.
        dJointID fixed = dJointCreateFixed(wId, 0);
        dJointAttach(fixed, 0, bId2);
        dJointSetFixed(fixed);


        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, d2r(178.5) );
        dBodySetRotation (bId1, R);
        dJointSetHingeAxisOffset(jId, 1, 0, 0, d2r(178.5));

        CHECK_CLOSE (r2d(dJointGetHingeAngle (jId)), 178.5, 1e-4);



        // The limit are set such that after the stepping
        // the angle will be between the 2 limits
        dJointSetHingeParam(jId, dParamLoStop, d2r(-90));
        dJointSetHingeParam(jId, dParamHiStop, d2r(179));

        dBodySetAngularVel(bId1, d2r(2), 0, 0);
        dWorldStep(wId, 1.0);
        dReal angle = r2d(dJointGetHingeAngle (jId));
        CHECK_CLOSE (-179.5, angle, 1e-1);
        dWorldStep(wId, 1.0);


        int limit = ((dxJointHinge *)jId)->limot.limit;
        CHECK_EQUAL(2, limit);
    }



    TEST_FIXTURE(dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
                 test_limit_crossing_loStop_With_Wrapping)
    {
        // Only body 1 will be able to move.
        dJointID fixed = dJointCreateFixed(wId, 0);
        dJointAttach(fixed, 0, bId2);
        dJointSetFixed(fixed);


        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, d2r(-178.5) );
        dBodySetRotation (bId1, R);
        dJointSetHingeAxisOffset(jId, 1, 0, 0, d2r(-178.5));


        CHECK_CLOSE (r2d(dJointGetHingeAngle (jId)), -178.5, 1e-4);


        // The limit are set such that after the stepping
        // the angle will be between the 2 limits
        dJointSetHingeParam(jId, dParamLoStop, d2r(-179));
        dJointSetHingeParam(jId, dParamHiStop, d2r(90));

        dBodySetAngularVel(bId1, d2r(-2), 0, 0);
        dWorldStep(wId, 1.0);
        dReal angle = r2d(dJointGetHingeAngle (jId));
        CHECK_CLOSE (179.5, angle, 1e-1);
        dWorldStep(wId, 1.0);


        int limit = ((dxJointHinge *)jId)->limot.limit;
        CHECK_EQUAL(1, limit);
    }



    TEST_FIXTURE(dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
                 test_limit_crossing_loStop_No_Wrapping_Really_Soft_Limit)
    {
        // Only body 1 will be able to move.
        dJointID fixed = dJointCreateFixed(wId, 0);
        dJointAttach(fixed, 0, bId2);
        dJointSetFixed(fixed);

        CHECK_CLOSE (r2d(dJointGetHingeAngle (jId)), 0, 1e-4);


        // The limit are set such that after the stepping
        // the angle will be between the 2 limits
        dJointSetHingeParam(jId, dParamLoStop, d2r(-30));
        dJointSetHingeParam(jId, dParamHiStop, d2r(30));

        // Set really soft limit
        dJointSetHingeParam(jId, dParamStopERP, REAL(0.2) );
        dJointSetHingeParam(jId, dParamStopCFM, REAL(1e-2) );


        dBodySetAngularVel(bId1, d2r(-60), 0, 0);
        dWorldStep(wId, 1.0);
        dReal angle = r2d(dJointGetHingeAngle (jId));
        dWorldStep(wId, 1.0);


        int limit = ((dxJointHinge *)jId)->limot.limit;
        CHECK_EQUAL(1, limit);

        dWorldStep(wId, 1.0);


        limit = ((dxJointHinge *)jId)->limot.limit;
        CHECK_EQUAL(1, limit);
    }





    // Test loStop crossing with really soft limit.
    // The body goes way to far pass the loStop and take 2 step to go back between the
    // limits because of the soft limits.
    TEST_FIXTURE(dxJointHinge_Fixture_B1_and_B2_At_Zero_Axis_Along_X,
                 test_limit_crossing_loStop_With_Wrapping_Really_Soft_Limit)
    {
        // Only body 1 will be able to move.
        dJointID fixed = dJointCreateFixed(wId, 0);
        dJointAttach(fixed, 0, bId2);
        dJointSetFixed(fixed);


        dMatrix3 R;

        dJointSetHingeAxis (jId, axis[0], axis[1], axis[2]);

        CHECK_CLOSE (dJointGetHingeAngle (jId), 0.0, 1e-4);

        dRFromAxisAndAngle (R, 1, 0, 0, d2r(-178) );
        dBodySetRotation (bId1, R);
        dJointSetHingeAxisOffset(jId, 1, 0, 0, d2r(-178));



        CHECK_CLOSE (r2d(dJointGetHingeAngle (jId)), -178, 1e-4);


        // The limit are set such that after the stepping
        // the angle will be between the 2 limits
        dJointSetHingeParam(jId, dParamLoStop, d2r(-179));
        dJointSetHingeParam(jId, dParamHiStop, d2r(179));

        // Set really soft limit
        dJointSetHingeParam(jId, dParamStopERP, REAL(0.2) );
        dJointSetHingeParam(jId, dParamStopCFM, REAL(1e-2) );


        dBodySetAngularVel(bId1, d2r(-60), 0, 0);
        dWorldStep(wId, 1.0);
        dWorldStep(wId, 1.0);


        int limit = ((dxJointHinge *)jId)->limot.limit;
        CHECK_EQUAL(1, limit);

        dWorldStep(wId, 1.0);


        limit = ((dxJointHinge *)jId)->limot.limit;
        CHECK_EQUAL(1, limit);
    }



    struct dxJointHinge_Fixture_as_demo_hinge
    {
        dxJointHinge_Fixture_as_demo_hinge()
        {
            wId = dWorldCreate();

            const dReal SIDE= REAL(0.5);
            const dReal MASS= REAL(1.0);


            dQuaternion q;
            dQFromAxisAndAngle (q,1,1,0,0.25*M_PI);

            dMass m;
            dMassSetBox (&m,1,SIDE,SIDE,SIDE);
            dMassAdjust (&m,MASS);

            bId1 = dBodyCreate (wId);
            dBodySetMass (bId1,&m);
            dBodySetPosition (bId1, 0.5*SIDE, 0.5*SIDE, 1);
            dBodySetQuaternion (bId1,q);

            bId2 = dBodyCreate (wId);
            dBodySetMass (bId2,&m);
            dBodySetPosition (bId2,-0.5*SIDE,-0.5*SIDE,1);
            dBodySetQuaternion (bId2,q);

            jId = dJointCreateHinge (wId, 0);
            dJointAttach (jId, bId1, bId2);
            dJointSetHingeAnchor (jId, 0, 0, 1);
            dJointSetHingeAxis (jId, 1, -1, 1.41421356);
        }

        ~dxJointHinge_Fixture_as_demo_hinge()
        {
            dWorldDestroy (wId);
        }

        simLoop()
        {
            const dReal kd = -0.3;        // angular damping constant

            static dReal a=0;
            const dReal *w = dBodyGetAngularVel (bId1);
            dBodyAddTorque (bId1,kd*w[0],kd*w[1]+0.1*cos(a),kd*w[2]+0.1*sin(a));
            dWorldStep (wId, 0.05);
            a += 0.01;
        }


        dWorldID wId;

        dBodyID bId1;
        dBodyID bId2;


        dJointID jId;
    };


    // Test loStop crossing with really soft limit.
    // The body goes way to far pass the loStop and take 2 step to go back between the
    // limits because of the soft limits.
    TEST_FIXTURE(dxJointHinge_Fixture_as_demo_hinge,
                 test_as_demo_hinge_limit_at_zero)
    {
        // The limit are set such that after the stepping
        // the angle will be between the 2 limits
        dJointSetHingeParam(jId, dParamLoStop, d2r(0));
        dJointSetHingeParam(jId, dParamHiStop, d2r(0));

        CHECK_CLOSE (0, r2d(dJointGetHingeAngle(jId)), 1e-4);

        dxJointHinge_Fixture_as_demo_hinge::simLoop();

        CHECK_CLOSE (0, r2d(dJointGetHingeAngle (jId)), 1e-4);

        dxJointHinge_Fixture_as_demo_hinge::simLoop();

        CHECK_CLOSE (0, r2d(dJointGetHingeAngle (jId)), 1e-4);
    }




} // End of SUITE TestdxJointHinge


