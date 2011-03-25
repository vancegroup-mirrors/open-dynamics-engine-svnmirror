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

#include <ode/ode.h>
#include "config.h"
#include "objects.h"
#include "joints/joint.h"
#include "util.h"

static void InternalFreeWorldProcessContext (dxWorldProcessContext *context);

//****************************************************************************
// Malloc based world stepping memory manager

/*extern */dxWorldProcessMemoryManager g_WorldProcessMallocMemoryManager(dAlloc, dRealloc, dFree);
/*extern */dxWorldProcessMemoryReserveInfo g_WorldProcessDefaultReserveInfo(dWORLDSTEP_RESERVEFACTOR_DEFAULT, dWORLDSTEP_RESERVESIZE_DEFAULT);


//****************************************************************************
// dxWorldProcessContext implementation

void dxWorldProcessContext::CleanupContext()
{
  ResetState();
  ClearPreallocations();
  FreePreallocationsContext();
}

void dxWorldProcessContext::SavePreallocations(int islandcount, int const *islandsizes, dxBody *const *bodies, dxJoint *const *joints, size_t const *islandreqs)
{
  m_IslandCount = islandcount;
  m_pIslandReqs = islandreqs;
  m_pIslandSizes = islandsizes;
  m_pBodies = bodies;
  m_pJoints = joints;
}

void dxWorldProcessContext::RetrievePreallocations(int &islandcount, int const *&islandsizes, dxBody *const *&bodies, dxJoint *const *&joints, size_t const *&islandreqs)
{
  islandcount = m_IslandCount;
  islandreqs = m_pIslandReqs;
  islandsizes = m_pIslandSizes;
  bodies = m_pBodies;
  joints = m_pJoints;
}

void dxWorldProcessContext::OffsetPreallocations(size_t stOffset)
{
  // m_IslandCount = -- no offset for count
  m_pIslandSizes = m_pIslandSizes ? (int const *)((size_t)m_pIslandSizes + stOffset) : NULL;
  m_pIslandReqs = m_pIslandReqs ? (size_t const *)((size_t)m_pIslandReqs + stOffset) : NULL;
  m_pBodies = m_pBodies ? (dxBody *const *)((size_t)m_pBodies + stOffset) : NULL;
  m_pJoints = m_pJoints ? (dxJoint *const *)((size_t)m_pJoints + stOffset) : NULL;
}

void dxWorldProcessContext::CopyPreallocations(const dxWorldProcessContext *othercontext)
{
  m_IslandCount = othercontext->m_IslandCount;
  m_pIslandSizes = othercontext->m_pIslandSizes;
  m_pIslandReqs = othercontext->m_pIslandReqs;
  m_pBodies = othercontext->m_pBodies;
  m_pJoints = othercontext->m_pJoints;
}

void dxWorldProcessContext::ClearPreallocations()
{
  m_IslandCount = 0;
  m_pIslandSizes = NULL;
  m_pIslandReqs = NULL;
  m_pBodies = NULL;
  m_pJoints = NULL;
}

void dxWorldProcessContext::FreePreallocationsContext()
{
  if (m_pPreallocationcContext) {
    InternalFreeWorldProcessContext(m_pPreallocationcContext);
    m_pPreallocationcContext = NULL;
  }
}


//****************************************************************************
// Auto disabling

void dInternalHandleAutoDisabling (dxWorld *world, dReal stepsize)
{
	dxBody *bb;
	for ( bb=world->firstbody; bb; bb=(dxBody*)bb->next )
	{
		// don't freeze objects mid-air (patch 1586738)
		if ( bb->firstjoint == NULL ) continue;

		// nothing to do unless this body is currently enabled and has
		// the auto-disable flag set
		if ( (bb->flags & (dxBodyAutoDisable|dxBodyDisabled)) != dxBodyAutoDisable ) continue;

		// if sampling / threshold testing is disabled, we can never sleep.
		if ( bb->adis.average_samples == 0 ) continue;

		//
		// see if the body is idle
		//
		
#ifndef dNODEBUG
		// sanity check
		if ( bb->average_counter >= bb->adis.average_samples )
		{
			dUASSERT( bb->average_counter < bb->adis.average_samples, "buffer overflow" );

			// something is going wrong, reset the average-calculations
			bb->average_ready = 0; // not ready for average calculation
			bb->average_counter = 0; // reset the buffer index
		}
#endif // dNODEBUG

		// sample the linear and angular velocity
		bb->average_lvel_buffer[bb->average_counter][0] = bb->lvel[0];
		bb->average_lvel_buffer[bb->average_counter][1] = bb->lvel[1];
		bb->average_lvel_buffer[bb->average_counter][2] = bb->lvel[2];
		bb->average_avel_buffer[bb->average_counter][0] = bb->avel[0];
		bb->average_avel_buffer[bb->average_counter][1] = bb->avel[1];
		bb->average_avel_buffer[bb->average_counter][2] = bb->avel[2];
		bb->average_counter++;

		// buffer ready test
		if ( bb->average_counter >= bb->adis.average_samples )
		{
			bb->average_counter = 0; // fill the buffer from the beginning
			bb->average_ready = 1; // this body is ready now for average calculation
		}

		int idle = 0; // Assume it's in motion unless we have samples to disprove it.

		// enough samples?
		if ( bb->average_ready )
		{
			idle = 1; // Initial assumption: IDLE

			// the sample buffers are filled and ready for calculation
			dVector3 average_lvel, average_avel;

			// Store first velocity samples
			average_lvel[0] = bb->average_lvel_buffer[0][0];
			average_avel[0] = bb->average_avel_buffer[0][0];
			average_lvel[1] = bb->average_lvel_buffer[0][1];
			average_avel[1] = bb->average_avel_buffer[0][1];
			average_lvel[2] = bb->average_lvel_buffer[0][2];
			average_avel[2] = bb->average_avel_buffer[0][2];
			
			// If we're not in "instantaneous mode"
			if ( bb->adis.average_samples > 1 )
			{
				// add remaining velocities together
				for ( unsigned int i = 1; i < bb->adis.average_samples; ++i )
				{
					average_lvel[0] += bb->average_lvel_buffer[i][0];
					average_avel[0] += bb->average_avel_buffer[i][0];
					average_lvel[1] += bb->average_lvel_buffer[i][1];
					average_avel[1] += bb->average_avel_buffer[i][1];
					average_lvel[2] += bb->average_lvel_buffer[i][2];
					average_avel[2] += bb->average_avel_buffer[i][2];
				}

				// make average
				dReal r1 = dReal( 1.0 ) / dReal( bb->adis.average_samples );

				average_lvel[0] *= r1;
				average_avel[0] *= r1;
				average_lvel[1] *= r1;
				average_avel[1] *= r1;
				average_lvel[2] *= r1;
				average_avel[2] *= r1;
			}

			// threshold test
			dReal av_lspeed, av_aspeed;
			av_lspeed = dCalcVectorDot3( average_lvel, average_lvel );
			if ( av_lspeed > bb->adis.linear_average_threshold )
			{
				idle = 0; // average linear velocity is too high for idle
			}
			else
			{
				av_aspeed = dCalcVectorDot3( average_avel, average_avel );
				if ( av_aspeed > bb->adis.angular_average_threshold )
				{
					idle = 0; // average angular velocity is too high for idle
				}
			}
		}

		// if it's idle, accumulate steps and time.
		// these counters won't overflow because this code doesn't run for disabled bodies.
		if (idle) {
			bb->adis_stepsleft--;
			bb->adis_timeleft -= stepsize;
		}
		else {
			// Reset countdowns
			bb->adis_stepsleft = bb->adis.idle_steps;
			bb->adis_timeleft = bb->adis.idle_time;
		}

		// disable the body if it's idle for a long enough time
		if ( bb->adis_stepsleft <= 0 && bb->adis_timeleft <= 0 )
		{
			bb->flags |= dxBodyDisabled; // set the disable flag

			// disabling bodies should also include resetting the velocity
			// should prevent jittering in big "islands"
			bb->lvel[0] = 0;
			bb->lvel[1] = 0;
			bb->lvel[2] = 0;
			bb->avel[0] = 0;
			bb->avel[1] = 0;
			bb->avel[2] = 0;
		}
	}
}


//****************************************************************************
// body rotation

// return sin(x)/x. this has a singularity at 0 so special handling is needed
// for small arguments.

static inline dReal sinc (dReal x)
{
  // if |x| < 1e-4 then use a taylor series expansion. this two term expansion
  // is actually accurate to one LS bit within this range if double precision
  // is being used - so don't worry!
  if (dFabs(x) < 1.0e-4) return REAL(1.0) - x*x*REAL(0.166666666666666666667);
  else return dSin(x)/x;
}


// given a body b, apply its linear and angular rotation over the time
// interval h, thereby adjusting its position and orientation.

void dxStepBody (dxBody *b, dReal h)
{
  // cap the angular velocity
  if (b->flags & dxBodyMaxAngularSpeed) {
    const dReal max_ang_speed = b->max_angular_speed;
    const dReal aspeed = dCalcVectorDot3( b->avel, b->avel );
    if (aspeed > max_ang_speed*max_ang_speed) {
      const dReal coef = max_ang_speed/dSqrt(aspeed);
      dScaleVector3(b->avel, coef);
    }
  }
  // end of angular velocity cap


  // handle linear velocity
  for (int j=0; j<3; j++) b->posr.pos[j] += h * b->lvel[j];

  if (b->flags & dxBodyFlagFiniteRotation) {
    dVector3 irv;	// infitesimal rotation vector
    dQuaternion q;	// quaternion for finite rotation

    if (b->flags & dxBodyFlagFiniteRotationAxis) {
      // split the angular velocity vector into a component along the finite
      // rotation axis, and a component orthogonal to it.
      dVector3 frv;		// finite rotation vector
      dReal k = dCalcVectorDot3 (b->finite_rot_axis,b->avel);
      frv[0] = b->finite_rot_axis[0] * k;
      frv[1] = b->finite_rot_axis[1] * k;
      frv[2] = b->finite_rot_axis[2] * k;
      irv[0] = b->avel[0] - frv[0];
      irv[1] = b->avel[1] - frv[1];
      irv[2] = b->avel[2] - frv[2];

      // make a rotation quaternion q that corresponds to frv * h.
      // compare this with the full-finite-rotation case below.
      h *= REAL(0.5);
      dReal theta = k * h;
      q[0] = dCos(theta);
      dReal s = sinc(theta) * h;
      q[1] = frv[0] * s;
      q[2] = frv[1] * s;
      q[3] = frv[2] * s;
    }
    else {
      // make a rotation quaternion q that corresponds to w * h
      dReal wlen = dSqrt (b->avel[0]*b->avel[0] + b->avel[1]*b->avel[1] +
        b->avel[2]*b->avel[2]);
      h *= REAL(0.5);
      dReal theta = wlen * h;
      q[0] = dCos(theta);
      dReal s = sinc(theta) * h;
      q[1] = b->avel[0] * s;
      q[2] = b->avel[1] * s;
      q[3] = b->avel[2] * s;
    }

    // do the finite rotation
    dQuaternion q2;
    dQMultiply0 (q2,q,b->q);
    for (int j=0; j<4; j++) b->q[j] = q2[j];

    // do the infitesimal rotation if required
    if (b->flags & dxBodyFlagFiniteRotationAxis) {
      dReal dq[4];
      dWtoDQ (irv,b->q,dq);
      for (int j=0; j<4; j++) b->q[j] += h * dq[j];
    }
  }
  else {
    // the normal way - do an infitesimal rotation
    dReal dq[4];
    dWtoDQ (b->avel,b->q,dq);
    for (int j=0; j<4; j++) b->q[j] += h * dq[j];
  }

  // normalize the quaternion and convert it to a rotation matrix
  dNormalize4 (b->q);
  dQtoR (b->q,b->posr.R);

  // notify all attached geoms that this body has moved
  for (dxGeom *geom = b->geom; geom; geom = dGeomGetBodyNext (geom))
    dGeomMoved (geom);

  // notify the user
  if (b->moved_callback)
    b->moved_callback(b);


  // damping
  if (b->flags & dxBodyLinearDamping) {
    const dReal lin_threshold = b->dampingp.linear_threshold;
    const dReal lin_speed = dCalcVectorDot3( b->lvel, b->lvel );
    if ( lin_speed > lin_threshold) {
      const dReal k = 1 - b->dampingp.linear_scale;
      dScaleVector3(b->lvel, k);
    }
  }
  if (b->flags & dxBodyAngularDamping) {
    const dReal ang_threshold = b->dampingp.angular_threshold;
    const dReal ang_speed = dCalcVectorDot3( b->avel, b->avel );
    if ( ang_speed > ang_threshold) {
      const dReal k = 1 - b->dampingp.angular_scale;
      dScaleVector3(b->avel, k);
    }
  }
}


//****************************************************************************
// island processing

// This estimates dynamic memory requirements for dxProcessIslands
static size_t EstimateIslandsProcessingMemoryRequirements(dxWorld *world, size_t &sesize)
{
  size_t res = 0;

  size_t islandcounts = dEFFICIENT_SIZE(world->nb * 2 * sizeof(int));
  res += islandcounts;
  size_t islandreqs = dEFFICIENT_SIZE(world->nb * sizeof(size_t));
  res += islandreqs;

  size_t bodiessize = dEFFICIENT_SIZE(world->nb * sizeof(dxBody*));
  size_t jointssize = dEFFICIENT_SIZE(world->nj * sizeof(dxJoint*));
  res += bodiessize + jointssize;

  sesize = (bodiessize < jointssize) ? bodiessize : jointssize;
  return res;
}

static size_t BuildIslandsAndEstimateStepperMemoryRequirements(dxWorldProcessContext *context, 
  dxWorld *world, dReal stepsize, dmemestimate_fn_t stepperestimate)
{
  const int sizeelements = 2;
  size_t maxreq = 0;

  // handle auto-disabling of bodies
  dInternalHandleAutoDisabling (world,stepsize);

  int nb = world->nb, nj = world->nj;
  // Make array for island body/joint counts
  int *islandsizes = context->AllocateArray<int>(2 * nb);
  int *sizescurr;
  size_t *islandreqs = context->AllocateArray<size_t>(nb);
  size_t *islandreqscurr;

  // make arrays for body and joint lists (for a single island) to go into
  dxBody **body = context->AllocateArray<dxBody *>(nb);
  dxJoint **joint = context->AllocateArray<dxJoint *>(nj);

  BEGIN_STATE_SAVE(context, stackstate) {
    // allocate a stack of unvisited bodies in the island. the maximum size of
    // the stack can be the lesser of the number of bodies or joints, because
    // new bodies are only ever added to the stack by going through untagged
    // joints. all the bodies in the stack must be tagged!
    int stackalloc = (nj < nb) ? nj : nb;
    dxBody **stack = context->AllocateArray<dxBody *>(stackalloc);

    {
      // set all body/joint tags to 0
      for (dxBody *b=world->firstbody; b; b=(dxBody*)b->next) b->island_tag = 0;
      for (dxJoint *j=world->firstjoint; j; j=(dxJoint*)j->next) j->island_tag = 0;
    }

    sizescurr = islandsizes;
    islandreqscurr = islandreqs;
    dxBody **bodystart = body;
    dxJoint **jointstart = joint;
    for (dxBody *bb=world->firstbody; bb; bb=(dxBody*)bb->next) {
      // get bb = the next enabled, untagged body, and tag it
      if (!bb->island_tag) {
        if (!(bb->flags & dxBodyDisabled)) {
          bb->island_tag = 1;

          dxBody **bodycurr = bodystart;
          dxJoint **jointcurr = jointstart;

          // tag all bodies and joints starting from bb.
          *bodycurr++ = bb;

          int stacksize = 0;
          dxBody *b = bb;

          while (true) {
            // traverse and tag all body's joints, add untagged connected bodies
            // to stack
            for (dxJointNode *n=b->firstjoint; n; n=n->next) {
              dxJoint *njoint = n->joint;
              if (!njoint->island_tag) {
                if (njoint->isEnabled()) {
                  njoint->island_tag = 1;
                  *jointcurr++ = njoint;

                  dxBody *nbody = n->body;
                  // Body disabled flag is not checked here. This is how auto-enable works.
                  if (nbody && nbody->island_tag <= 0) {
                    nbody->island_tag = 1;
                    // Make sure all bodies are in the enabled state.
                    nbody->flags &= ~dxBodyDisabled;
                    stack[stacksize++] = nbody;
                  }
                } else {
                  njoint->tag = -1; // Used in Step to prevent search over disabled joints (not needed for QuickStep so far)
                }
              }
            }
            dIASSERT(stacksize <= world->nb);
            dIASSERT(stacksize <= world->nj);

            if (stacksize == 0) {
              break;
            }

            b = stack[--stacksize];	// pop body off stack
            *bodycurr++ = b;	// put body on body list
          }

          int bcount = bodycurr - bodystart;
          int jcount = jointcurr - jointstart;
          sizescurr[0] = bcount;
          sizescurr[1] = jcount;
          sizescurr += sizeelements;

          *islandreqscurr = stepperestimate(bodystart, bcount, jointstart, jcount);
          maxreq = (maxreq > *islandreqscurr) ? maxreq : *islandreqscurr;
          islandreqscurr += 1;

          bodystart = bodycurr;
          jointstart = jointcurr;
        } else {
          bb->island_tag = -1; // Not used so far (assigned to retain consistency with joints)
        }
      }
    }
  } END_STATE_SAVE(context, stackstate);

# ifndef dNODEBUG
  // if debugging, check that all objects (except for disabled bodies,
  // unconnected joints, and joints that are connected to disabled bodies)
  // were tagged.
  {
    for (dxBody *b=world->firstbody; b; b=(dxBody*)b->next) {
      if (b->flags & dxBodyDisabled) {
        if (b->island_tag > 0) dDebug (0,"disabled body tagged");
      }
      else {
        if (b->island_tag <= 0) dDebug (0,"enabled body not tagged");
      }
    }
    for (dxJoint *j=world->firstjoint; j; j=(dxJoint*)j->next) {
      if ( (( j->node[0].body && (j->node[0].body->flags & dxBodyDisabled)==0 ) ||
        (j->node[1].body && (j->node[1].body->flags & dxBodyDisabled)==0) )
        && 
        j->isEnabled() ) {
          if (j->island_tag <= 0) dDebug (0,"attached enabled joint not tagged");
      }
      else {
        if (j->island_tag > 0) dDebug (0,"unattached or disabled joint tagged");
      }
    }
  }
# endif

  int islandcount = (sizescurr - islandsizes) / sizeelements;
  context->SavePreallocations(islandcount, islandsizes, body, joint, islandreqs);

  return maxreq;
}

// this groups all joints and bodies in a world into islands. all objects
// in an island are reachable by going through connected bodies and joints.
// each island can be simulated separately.
// note that joints that are not attached to anything will not be included
// in any island, an so they do not affect the simulation.
//
// this function starts new island from unvisited bodies. however, it will
// never start a new islands from a disabled body. thus islands of disabled
// bodies will not be included in the simulation. disabled bodies are
// re-enabled if they are found to be part of an active island.

struct IslandInfoStruct
{
    static dstepper_fn_t   stepper;
    dxWorldProcessContext *island_context;
    static dxWorld        *world;

    dxBody *const         *bodystart;
    int                    bcount;
    dxJoint *const        *jointstart;
    int                    jcount;
    dReal                  stepsize;
};

dstepper_fn_t          IslandInfoStruct::stepper        = NULL;
dxWorld               *IslandInfoStruct::world          = NULL;

static void dxProcessSingleIsland(IslandInfoStruct *island_info)
{
    dAllocateODEDataForThread(dAllocateMaskAll);

    BEGIN_STATE_SAVE(island_info->island_context, island_stepperstate) 
    {
        // now do something with body and joint lists
        island_info->stepper (island_info->island_context,
                              island_info->world,
                              island_info->bodystart,
                              island_info->bcount,
                              island_info->jointstart,
                              island_info->jcount,
                              island_info->stepsize);
    } 
    END_STATE_SAVE(island_info->island_context, island_stepperstate);

    dCleanupODEAllDataForThread();
}

//#define USE_OPENMP_TO_DO_PARALLEL_ISLANDS_PROCESSING
#ifdef USE_OPENMP_TO_DO_PARALLEL_ISLANDS_PROCESSING
    // Visual Studio notes:
    // All VC Libs DLLs require manifests to load them. Manifest should be generated and embedded in the binary when 
    // the project is built in the IDE.
    // For OpenMP the manifest is generated when omp.h is included. There are cases where a dependency on vcomp.dll can be 
    // pulled in without using the header. In these cases the manifest may not be generated.
    // To generate the manifest, include omp.h and built your project. IDE built projects will automatically embed the 
    // manifest in the binary, that is of course unless the project settings were changed to disable it.
    // See Property Pages / Configuration Properties / Linker / Manifest File / Generate Manifest. 
    #include <omp.h>
#endif

//#define USE_BOOST_THREADPOOL_TO_DO_PARALLEL_ISLANDS_PROCESSING
#ifdef USE_BOOST_THREADPOOL_TO_DO_PARALLEL_ISLANDS_PROCESSING
    #include <boost/threadpool.hpp>
    static boost::threadpool::pool *boost_threadpool = NULL;
#endif

#define USE_TBB_TO_DO_PARALLEL_ISLANDS_PROCESSING
#ifdef USE_TBB_TO_DO_PARALLEL_ISLANDS_PROCESSING
    #include <tbb/task_scheduler_init.h>
    #include <tbb/parallel_for.h>

    static tbb::task_scheduler_init *tbb_init = NULL;  

    class process_island
    {
        IslandInfoStruct *m_island_info;
    public:
        process_island(IslandInfoStruct *island_info) : m_island_info(island_info) {}
        void operator() (tbb::blocked_range<int> &r) const
        {
            for (int i = r.begin(); i != r.end(); ++i)
                dxProcessSingleIsland(&m_island_info[i]);
        }
    };
#endif

static int s_requested_num_island_threads = 0;
static int s_actual_num_island_threads    = 0;

ODE_API void dSetNumIslandThreads(int num_island_threads)
{
    // clean up immediately
#ifdef USE_OPENMP_TO_DO_PARALLEL_ISLANDS_PROCESSING
    // scale down to a single thread
    omp_set_num_threads(1);
#endif
#ifdef USE_BOOST_THREADPOOL_TO_DO_PARALLEL_ISLANDS_PROCESSING
    // terminate and delete the boost thread pool
    if (boost_threadpool) 
    {
        boost_threadpool->wait();
        delete boost_threadpool;
        boost_threadpool = NULL;
    }
#endif
#ifdef USE_TBB_TO_DO_PARALLEL_ISLANDS_PROCESSING
    // terminate and delete the TBB scheduler
    if (tbb_init) 
    {
        tbb_init->terminate();
        delete tbb_init;
        tbb_init = NULL;
    }
#endif

    // remember the requested number of island threads
    s_requested_num_island_threads = num_island_threads;

    // no need to update the actual island threads later on if none are requested
    if (s_requested_num_island_threads == 0)
        s_actual_num_island_threads = 0;
}

// Local method that takes care of the actual update of the number of island threads.
// This is called from dxProcessIslands() to make sure the update is done in the same
// thread as dxProcessIslands(), which is a requirement for TBB: the scheduler needs
// to be created in the same thread as it is used in.
static void UpdateNumIslandThreads()
{
    if (s_requested_num_island_threads == s_actual_num_island_threads)
        return;

    if (s_requested_num_island_threads > 1) 
    {
#ifdef USE_OPENMP_TO_DO_PARALLEL_ISLANDS_PROCESSING
        omp_set_num_threads(s_requested_num_island_threads);
#endif
#ifdef USE_BOOST_THREADPOOL_TO_DO_PARALLEL_ISLANDS_PROCESSING
        boost_threadpool = new boost::threadpool::pool(s_requested_num_island_threads);
#endif
#ifdef USE_TBB_TO_DO_PARALLEL_ISLANDS_PROCESSING
    if (s_requested_num_island_threads != s_actual_num_island_threads)
        tbb_init = new tbb::task_scheduler_init(s_requested_num_island_threads);
#endif
    }

    s_actual_num_island_threads = s_requested_num_island_threads;
}

// Process islands
void dxProcessIslands (dxWorld *world, dReal stepsize, dstepper_fn_t stepper)
{
    const int sizeelements = 2;

    // get the world memory
    dxStepWorkingMemory *wmem = world->wmem;
    dIASSERT(wmem != NULL);

    // get the world processing context from the world memory
    dxWorldProcessContext *context = wmem->GetWorldProcessingContext(); 
    dIASSERT(context != NULL);

    // retrieve the preallocations from the world processing context
    int             islandcount;
    int const      *islandsizes;
    dxBody *const  *body;
    dxJoint *const *joint;
    size_t const   *islandreqs;
    context->RetrievePreallocations(islandcount, islandsizes, body, joint, islandreqs);

    // limit the maximum number of islands being processed to make sure 
    // we don't access any memory outside allocated areas.
    if (islandcount > MAX_PARALLEL_ISLANDS)
        islandcount = MAX_PARALLEL_ISLANDS;

    // collect all island info up front
    static IslandInfoStruct island_info[MAX_PARALLEL_ISLANDS];
    {
        // set the stepper and world
        island_info[0].stepper = stepper;
        island_info[0].world   = world;

        // collect the other island info
        dxBody *const *bodystart = body;
        dxJoint *const *jointstart = joint;

        int island_index=0;
        int const *const sizesend = islandsizes + islandcount * sizeelements;
        for (int const *sizescurr = islandsizes; sizescurr != sizesend; sizescurr += sizeelements) 
        {
            int bcount = sizescurr[0];
            int jcount = sizescurr[1];

            island_info[island_index].island_context = world->island_wmems[island_index]->GetWorldProcessingContext();
            island_info[island_index].bodystart      = bodystart;
            island_info[island_index].bcount         = bcount;
            island_info[island_index].jointstart     = jointstart;
            island_info[island_index].jcount         = jcount;
            island_info[island_index].stepsize       = stepsize;

            island_index++;

            bodystart  += bcount;
            jointstart += jcount;
        }
    }

    // update the number of island threads to make this is done
    // in the same thread as the actual parallel island processing
    UpdateNumIslandThreads();

    // process all islands
#if defined (USE_OPENMP_TO_DO_PARALLEL_ISLANDS_PROCESSING)
    if (islandcount > 1)
    {
        #pragma omp parallel for 
        for (int island_index=0; island_index<islandcount; island_index++)
        {
            dxProcessSingleIsland(&island_info[island_index]);
        }
    }
    else

#elif (defined USE_BOOST_THREADPOOL_TO_DO_PARALLEL_ISLANDS_PROCESSING)
    if (islandcount > 1 && boost_threadpool)
    {
        for (int island_index=0; island_index<islandcount; island_index++)
        {
            boost_threadpool->schedule(boost::bind(dxProcessSingleIsland, &island_info[island_index]));
        }
        boost_threadpool->wait();
    }
    else

#elif (defined USE_TBB_TO_DO_PARALLEL_ISLANDS_PROCESSING)
    if (islandcount > 1 && tbb_init)
    {
        tbb::parallel_for(tbb::blocked_range<int>(0, islandcount, 1), 
                          process_island(island_info));
    }
    else

#endif
    {
        // here we do process the islands the old fashioned way
        for (int island_index=0; island_index<islandcount; island_index++)
        {
            dxProcessSingleIsland(&island_info[island_index]);
        }
    }

    // clean up all island contexts
    for (int island_index=0; island_index<islandcount; island_index++)
        world->island_wmems[island_index]->GetWorldProcessingContext()->CleanupContext();

    // clean up 
    context->CleanupContext();
    dIASSERT(context->IsStructureValid());
}

//****************************************************************************
// World processing context management

static size_t AdjustArenaSizeForReserveRequirements(size_t arenareq, float rsrvfactor, unsigned rsrvminimum)
{
  float scaledarena = arenareq * rsrvfactor;
  size_t adjustedarena = (scaledarena < SIZE_MAX) ? (size_t)scaledarena : SIZE_MAX;
  size_t boundedarena = (adjustedarena > rsrvminimum) ? adjustedarena : (size_t)rsrvminimum;
  return dEFFICIENT_SIZE(boundedarena);
}

static dxWorldProcessContext *InternalReallocateWorldProcessContext (
  dxWorldProcessContext *oldcontext, size_t memreq, 
  const dxWorldProcessMemoryManager *memmgr, float rsrvfactor, unsigned rsrvminimum)
{
  dxWorldProcessContext *context = oldcontext;
  bool allocsuccess = false;

  size_t oldarenasize; 
  void *pOldArena;

  do {
    size_t oldmemsize = oldcontext ? oldcontext->GetMemorySize() : 0;
    if (!oldcontext || oldmemsize < memreq) {
      oldarenasize = oldcontext ? dxWorldProcessContext::MakeArenaSize(oldmemsize) : 0;
      pOldArena = oldcontext ? oldcontext->m_pArenaBegin : NULL;

      if (!dxWorldProcessContext::IsArenaPossible(memreq)) {
        break;
      }

      size_t arenareq = dxWorldProcessContext::MakeArenaSize(memreq);
      size_t arenareq_with_reserve = AdjustArenaSizeForReserveRequirements(arenareq, rsrvfactor, rsrvminimum);
      size_t memreq_with_reserve = memreq + (arenareq_with_reserve - arenareq);

      if (oldcontext) {

        if (oldcontext->m_pAllocCurrent != oldcontext->m_pAllocBegin) {

          // Save old efficient offset and meaningful data size for the case if 
          // reallocation throws the block at different efficient offset
          size_t oldcontextofs = (size_t)oldcontext - (size_t)pOldArena;
          size_t datasize = (size_t)oldcontext->m_pAllocCurrent - (size_t)oldcontext;
          
          // Extra EFFICIENT_ALIGNMENT bytes might be needed after re-allocation with different alignment
          size_t shrunkarenasize = dEFFICIENT_SIZE(datasize + oldcontextofs) + EFFICIENT_ALIGNMENT;
          if (shrunkarenasize < oldarenasize) {

            void *pShrunkOldArena = oldcontext->m_pArenaMemMgr->m_fnShrink(pOldArena, oldarenasize, shrunkarenasize);
            if (!pShrunkOldArena) {
              break;
            }

            // In case if shrinking is not supported and memory manager had to allocate-copy-free
            if (pShrunkOldArena != pOldArena) {
              dxWorldProcessContext *shrunkcontext = (dxWorldProcessContext *)dEFFICIENT_PTR(pShrunkOldArena);

              // Preform data shift in case if efficient alignment of new block
              // does not match that of old block
              size_t shrunkcontextofs = (size_t)shrunkcontext - (size_t)pShrunkOldArena;
              size_t offsetdiff = oldcontextofs - shrunkcontextofs;
              if (offsetdiff != 0) {
                memmove(shrunkcontext, (void *)((size_t)shrunkcontext + offsetdiff), datasize);
              }

              // Make sure allocation pointers are valid - that is necessary to
              // be able to calculate size and free old arena later
              size_t shrunkdatasize = dxWorldProcessContext::MakeBufferSize(shrunkarenasize);
              void *blockbegin = dEFFICIENT_PTR(shrunkcontext + 1);
              void *blockend = dOFFSET_EFFICIENTLY(blockbegin, shrunkdatasize);
              shrunkcontext->m_pAllocBegin = blockbegin;
              shrunkcontext->m_pAllocEnd = blockend;
              shrunkcontext->m_pAllocCurrent = blockend; // -- set to end to prevent possibility of further allocation
              shrunkcontext->m_pArenaBegin = pShrunkOldArena;

              size_t stOffset = ((size_t)pShrunkOldArena - (size_t)pOldArena) - offsetdiff;
              shrunkcontext->OffsetPreallocations(stOffset);

              oldcontext = shrunkcontext;

              // Reassign to old arena variables for potential freeing at exit
              pOldArena = pShrunkOldArena;
            }

            // Reassign to old arena variables for potential freeing at exit
            oldarenasize = shrunkarenasize;
          }

        } else {
          oldcontext->m_pArenaMemMgr->m_fnFree(pOldArena, oldarenasize);
          oldcontext = NULL;
          
          // Zero variables to avoid another freeing on exit
          pOldArena = NULL;
          oldarenasize = 0;
        }
      }

      // Allocate new arena
      void *pNewArena = memmgr->m_fnAlloc(arenareq_with_reserve);
      if (!pNewArena) {
        break;
      }

      context = (dxWorldProcessContext *)dEFFICIENT_PTR(pNewArena);

      void *blockbegin = dEFFICIENT_PTR(context + 1);
      void *blockend = dOFFSET_EFFICIENTLY(blockbegin, memreq_with_reserve);

      context->m_pAllocBegin = blockbegin;
      context->m_pAllocEnd = blockend;
      context->m_pArenaBegin = pNewArena;
      context->m_pAllocCurrent = blockbegin;

      if (oldcontext) {
        context->CopyPreallocations(oldcontext);
      } else {
        context->ClearPreallocations();
      }

      context->m_pArenaMemMgr = memmgr;
      context->m_pPreallocationcContext = oldcontext;
    }

    allocsuccess = true;
  } while (false);

  if (!allocsuccess) {
    if (pOldArena) {
      dIASSERT(oldcontext);
      oldcontext->m_pArenaMemMgr->m_fnFree(pOldArena, oldarenasize);
    }
    context = NULL;
  }

  return context;
}

static void InternalFreeWorldProcessContext (dxWorldProcessContext *context)
{
  size_t memsize = context->GetMemorySize();
  size_t arenasize = dxWorldProcessContext::MakeArenaSize(memsize);

  void *pArenaBegin = context->m_pArenaBegin;
  context->m_pArenaMemMgr->m_fnFree(pArenaBegin, arenasize);
}


bool dxReallocateWorldProcessContext (dxWorld *world, 
  dReal stepsize, dmemestimate_fn_t stepperestimate)
{
  dxStepWorkingMemory *wmem = AllocateOnDemand(world->wmem);
  if (!wmem) return false;

  dxWorldProcessContext *oldcontext = wmem->GetWorldProcessingContext();
  dIASSERT (!oldcontext || oldcontext->IsStructureValid());

  const dxWorldProcessMemoryReserveInfo *reserveinfo = wmem->SureGetMemoryReserveInfo();
  const dxWorldProcessMemoryManager *memmgr = wmem->SureGetMemoryManager();

  dxWorldProcessContext *context = oldcontext;

  size_t sesize;
  size_t islandsreq = EstimateIslandsProcessingMemoryRequirements(world, sesize);
  dIASSERT(islandsreq == dEFFICIENT_SIZE(islandsreq));
  dIASSERT(sesize == dEFFICIENT_SIZE(sesize));

  size_t stepperestimatereq = islandsreq + sesize;
  context = InternalReallocateWorldProcessContext(context, stepperestimatereq, memmgr, 1.0f, reserveinfo->m_uiReserveMinimum);
  
  if (context)
  {
    size_t stepperreq = BuildIslandsAndEstimateStepperMemoryRequirements(context, world, stepsize, stepperestimate);
    dIASSERT(stepperreq == dEFFICIENT_SIZE(stepperreq));

    int islandcount;
    int const *islandsizes;
    dxBody *const *body;
    dxJoint *const *joint;
    size_t const *islandreqs;
    context->RetrievePreallocations(islandcount, islandsizes, body, joint, islandreqs);

    for (int jj = 0; jj < islandcount; jj++)
    {
        // Start a new instance of dxStepWorkingMemory for each individual island
        dxStepWorkingMemory *island_wmem = AllocateOnDemand(world->island_wmems[jj]);  
        if (!island_wmem) return false;

        dxWorldProcessContext *island_oldcontext = island_wmem->GetWorldProcessingContext();
        dIASSERT (!island_oldcontext || island_oldcontext->IsStructureValid());

        const dxWorldProcessMemoryReserveInfo *island_reserveinfo = island_wmem->SureGetMemoryReserveInfo();
        const dxWorldProcessMemoryManager *island_memmgr = island_wmem->SureGetMemoryManager();

        dxWorldProcessContext *island_context = island_oldcontext;

        size_t island_memreq = islandreqs[jj];
        island_context = InternalReallocateWorldProcessContext(island_context, island_memreq, island_memmgr, island_reserveinfo->m_fReserveFactor, island_reserveinfo->m_uiReserveMinimum);
        island_wmem->SetWorldProcessingContext(island_context); 
    }
  }

  wmem->SetWorldProcessingContext(context);
  return context != NULL;
}

dxWorldProcessContext *dxReallocateTemporayWorldProcessContext(dxWorldProcessContext *oldcontext, 
  size_t memreq, const dxWorldProcessMemoryManager *memmgr/*=NULL*/, const dxWorldProcessMemoryReserveInfo *reserveinfo/*=NULL*/)
{
  dxWorldProcessContext *context = oldcontext;

  if (context && context->GetMemorySize() < memreq) {
    dIASSERT(!context->IsPreallocationsContextAssigned());

    InternalFreeWorldProcessContext(context);
    context = NULL;
  }

  if (context == NULL) {
    const dxWorldProcessMemoryManager *surememmgr = memmgr ? memmgr : &g_WorldProcessMallocMemoryManager;
    const dxWorldProcessMemoryReserveInfo *surereserveinfo = reserveinfo ? reserveinfo : &g_WorldProcessDefaultReserveInfo;
    context = InternalReallocateWorldProcessContext(context, memreq, surememmgr, surereserveinfo->m_fReserveFactor, surereserveinfo->m_uiReserveMinimum);
  }

  return context;
}

void dxFreeWorldProcessContext (dxWorldProcessContext *context)
{
  // Free old arena for the case if context is freed after reallocation without
  // a call to world stepping function
  context->FreePreallocationsContext();

  // Assert validity after old arena is freed as validation includes checking for
  // old arena to be absent
  dUASSERT (context->IsStructureValid(), "invalid context structure");

  InternalFreeWorldProcessContext(context);
}



