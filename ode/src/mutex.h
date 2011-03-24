/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
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

/*
 *	Simple mutex class by Rene Balkenende
 */

#ifndef _ODE_MUTEX_H_
#define _ODE_MUTEX_H_

//****************************************************************************
// a simple mutex class used by e.g. space when altering a geom's linked 
// list by calling dxGeom::spaceAdd() or dxGeom::spaceRemove().
#if defined(_WIN32)
    #include <windows.h>

    class dxMutex
    {
    public:
        dxMutex() { m_handle = CreateMutex(0, FALSE, 0); }
        ~dxMutex() { CloseHandle(m_handle); }
        void lock() { WaitForSingleObject(m_handle, INFINITE); }
        void unlock() { ReleaseMutex(m_handle); }
    private:
        HANDLE m_handle;
    };
#else
    #include <pthread.h>

    class dxMutex
    {
    public:
        dxMutex() : m_inside(0), m_owner(0) { pthread_mutex_init(&m_handle, NULL); }
        ~dxMutex() { pthread_mutex_destroy(&m_handle); }
        void lock() { if (!m_owner || m_owner != pthread_self()) pthread_mutex_lock(&m_handle); if (!m_inside++) m_owner = pthread_self(); }
        void unlock() { if (!--m_inside) { m_owner = 0; pthread_mutex_unlock(&m_handle); } }
    private:
        pthread_mutex_t m_handle;
        int             m_inside;
        pthread_t       m_owner;
    };
#endif

#endif
