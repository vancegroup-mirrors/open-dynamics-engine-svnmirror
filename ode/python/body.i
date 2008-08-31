// wrapper for dBodyID


// TODO: deal with out-of-order destruction (World, Joint))


%immutable Body::id;

%inline %{
class Body {
    // intentionally undefined, don't use these
    Body (const Body &);
    void operator= (const Body &);

public:
    dBodyID id;

    Body() : id(0)
    {}
    
    Body (dWorldID world)
    {
        id = dBodyCreate (world);
    }
    Body (World& world)
    {
        id = dBodyCreate (world.id);
    }
    ~Body()
    {
        //if (id) dBodyDestroy (id);
    }

    void create (dWorldID world)
    {
        destroy();
        id = dBodyCreate (world);
    }
    void create (World& world)
    {
        create(world.id);
    }
    
    // some languages don't have reliable destructor semantics
    void destroy()
    {
        if (id) dBodyDestroy (id);
        id = 0;
    }

    // TODO: handle those
/*
    void setData (void *data)
    { dBodySetData (id,data); }
    void *getData() const
    { return dBodyGetData (id); }
*/



    // position and velocity

    void setPosition (dReal x, dReal y, dReal z)
    {
        dBodySetPosition (id, x, y, z);
    }
    void setPosition (const Vector& p)
    {
        setPosition(p[0], p[1], p[2]);
    }
    Vector getPosition() const
    {
        return Vector(dBodyGetPosition (id));
    }
    void setRotation (const Matrix& R)
    {
        dBodySetRotation (id, R.data);
    }
    Matrix getRotation() const
    {
        return Matrix(dBodyGetRotation (id));
    }
    void setQuaternion (const Quaternion& q)
    {
        dBodySetQuaternion (id, q.data);
    }
    Quaternion getQuaternion() const
    {
        return Quaternion(dBodyGetQuaternion (id));
    }
    
    void setLinearVel (dReal x, dReal y, dReal z)
    {
        dBodySetLinearVel (id, x, y, z);
    }
    void setLinearVel (const Vector& v)
    {
        setLinearVel(v[0], v[1], v[2]);
    }
    Vector getLinearVel() const
    {
        return Vector(dBodyGetLinearVel (id));
    }
    void setAngularVel (dReal x, dReal y, dReal z)
    {
        dBodySetAngularVel (id, x, y, z);
    }
    void setAngularVel (const Vector& v)
    {
        setAngularVel (v[0], v[1], v[2]);
    }
    Vector getAngularVel() const
    {
        return Vector(dBodyGetAngularVel (id));
    }

    void setMass (const Mass& mass)
    {
        dBodySetMass (id, &mass.data);
    }
    Mass getMass () const
    {
        Mass mass;
        dBodyGetMass (id, &mass.data);
        return mass;
    }


    // forces and torques

    void addForce (dReal fx, dReal fy, dReal fz)
    {
        dBodyAddForce (id, fx, fy, fz);
    }
    void addForce (const Vector& f)
    {
        addForce (f[0], f[1], f[2]);
    }
    void addTorque (dReal fx, dReal fy, dReal fz)
    {
        dBodyAddTorque (id, fx, fy, fz);
    }
    void addTorque (const Vector& t)
    {
        addTorque(t[0], t[1], t[2]);
    }
    void addRelForce (dReal fx, dReal fy, dReal fz)
    {
        dBodyAddRelForce (id, fx, fy, fz);
    }
    void addRelForce (const Vector& f)
    {
        addRelForce (f[0], f[1], f[2]);
    }
    void addRelTorque (dReal fx, dReal fy, dReal fz)
    {
        dBodyAddRelTorque (id, fx, fy, fz);
    }
    void addRelTorque (const Vector& t)
    {
        addRelTorque (t[0], t[1], t[2]);
    }

    void addForceAtPos (dReal fx, dReal fy, dReal fz,
                        dReal px, dReal py, dReal pz)
    {
        dBodyAddForceAtPos (id, fx, fy, fz, px, py, pz);
    }
    void addForceAtPos (const Vector& f, const Vector& p)
    {
        addForceAtPos (f[0], f[1], f[2], p[0], p[1], p[2]);
    }
    void addForceAtRelPos (dReal fx, dReal fy, dReal fz,
                            dReal px, dReal py, dReal pz)
    {
        dBodyAddForceAtRelPos (id, fx, fy, fz, px, py, pz);
    }
    void addForceAtRelPos (const Vector& f, const Vector& p)
    {
        addForceAtRelPos (f[0], f[1], f[2], p[0], p[1], p[2]);
    }
    void addRelForceAtPos (dReal fx, dReal fy, dReal fz,
                            dReal px, dReal py, dReal pz)
    {
        dBodyAddRelForceAtPos (id, fx, fy, fz, px, py, pz);
    }
    void addRelForceAtPos (const Vector& f, const Vector& p)
    {
        addRelForceAtPos (f[0], f[1], f[2], p[0], p[1], p[2]);
    }
    void addRelForceAtRelPos (dReal fx, dReal fy, dReal fz,
                                dReal px, dReal py, dReal pz)
    {
        dBodyAddRelForceAtRelPos (id, fx, fy, fz, px, py, pz);
    }
    void addRelForceAtRelPos (const Vector& f, const Vector& p)
    {
        addRelForceAtRelPos (f[0], f[1], f[2], p[0], p[1], p[2]);
    }

    Vector getForce() const
    {
        return Vector(dBodyGetForce(id));
    }
    Vector getTorque() const
    {
        return Vector(dBodyGetTorque(id));
    }
    void setForce (dReal x, dReal y, dReal z)
    {
        dBodySetForce (id, x, y, z);
    }
    void setForce (const Vector& f)
    {
        setForce (f[0], f[1], f[2]);
    }
    void setTorque (dReal x, dReal y, dReal z)
    {
        dBodySetTorque (id,x,y,z);
    }
    void setTorque (const Vector& t)
    {
        setTorque (t[0], t[1], t[2]);
    }



    void enable()
    {
        dBodyEnable (id);
    }
    void disable()
    {
        dBodyDisable (id);
    }
    bool isEnabled() const
    {
        return dBodyIsEnabled (id) != 0;
    }

    Vector getRelPointPos (dReal px, dReal py, dReal pz) const
    {
        Vector result;
        dBodyGetRelPointPos (id, px, py, pz, result.data);
        return result;
    }
    Vector getRelPointPos (const Vector& p) const
    {
        return getRelPointPos (p[0], p[1], p[2]);
    }
    Vector getRelPointVel (dReal px, dReal py, dReal pz) const
    {
        Vector result;
        dBodyGetRelPointVel (id, px, py, pz, result.data);
        return result;
    }
    Vector getRelPointVel (const Vector& p) const
    {
        return getRelPointVel (p[0], p[1], p[2]);
    }
    Vector getPointVel (dReal px, dReal py, dReal pz) const
    {
        Vector result;
        dBodyGetPointVel (id, px, py, pz, result.data);
        return result;
    }
    Vector getPointVel (const Vector& p) const
    {
        return getPointVel (p[0], p[1], p[2]);
    }
    Vector getPosRelPoint (dReal px, dReal py, dReal pz) const
    {
        Vector result;
        dBodyGetPosRelPoint (id, px, py, pz, result.data);
        return result;
    }
    Vector getPosRelPoint (const Vector& p) const
    {
        return getPosRelPoint (p[0], p[1], p[2]);
    }

    Vector vectorToWorld (dReal px, dReal py, dReal pz) const
    {
        Vector result;
        dBodyVectorToWorld (id, px, py, pz, result.data);
        return result;
    }
    Vector vectorToWorld (const Vector& p) const
    {
        return vectorToWorld (p[0], p[1], p[2]);
    }

    Vector vectorFromWorld (dReal px, dReal py, dReal pz) const
    {
        Vector result;
        dBodyVectorFromWorld (id, px, py, pz, result.data);
        return result;
    }
    Vector vectorFromWorld (const Vector& p) const
    {
        return vectorFromWorld (p[0], p[1], p[2]);
    }




    void setFiniteRotationMode (bool mode)
    {
        dBodySetFiniteRotationMode (id, mode);
    }
    void setFiniteRotationAxis (dReal x, dReal y, dReal z)
    {
        dBodySetFiniteRotationAxis (id, x, y, z);
    }
    void setFiniteRotationAxis (const Vector& a)
    {
        setFiniteRotationAxis (a[0], a[1], a[2]);
    }

    bool getFiniteRotationMode() const
    {
        return dBodyGetFiniteRotationMode (id) != 0;
    }
    Vector getFiniteRotationAxis () const
    {
        Vector result;
        dBodyGetFiniteRotationAxis (id, result.data);
        return result;
    }

    int getNumJoints() const
    {
        return dBodyGetNumJoints (id);
    }
    // TODO: properly wrap this
    /*dJointID getJoint (int index) const
    { return dBodyGetJoint (id, index); }*/

    void setGravityMode (bool mode)
    {
        dBodySetGravityMode (id,mode);
    }
    bool getGravityMode() const
    {
        return dBodyGetGravityMode (id) != 0;
    }

    bool isConnectedTo (dBodyID body) const
    {
        return dAreConnected (id, body) != 0;
    }
    bool isConnectedTo (const Body& body) const
    {
        return dAreConnected (id, body.id) != 0;
    }


    // autodisable

    void setAutoDisableLinearThreshold (dReal threshold) 
    {
        dBodySetAutoDisableLinearThreshold (id,threshold);
    }
    dReal getAutoDisableLinearThreshold() const
    {
        return dBodyGetAutoDisableLinearThreshold (id);
    }
    void setAutoDisableAngularThreshold (dReal threshold)
    {
        dBodySetAutoDisableAngularThreshold (id,threshold);
    }
    dReal getAutoDisableAngularThreshold() const
    {
        return dBodyGetAutoDisableAngularThreshold (id);
    }
    void setAutoDisableSteps (int steps)
    {
        dBodySetAutoDisableSteps (id,steps);
    }
    int getAutoDisableSteps() const
    {
        return dBodyGetAutoDisableSteps (id);
    }
    void setAutoDisableTime (dReal time)
    {
        dBodySetAutoDisableTime (id,time);
    }
    dReal getAutoDisableTime() const
    {
        return dBodyGetAutoDisableTime (id);
    }
    void setAutoDisableFlag (bool do_auto_disable)
    {
        dBodySetAutoDisableFlag (id,do_auto_disable);
    }
    bool getAutoDisableFlag() const
    {
        return dBodyGetAutoDisableFlag (id) != 0;
    }


    // damping

    dReal getLinearDamping() const
    {
        return dBodyGetLinearDamping(id);
    }
    void setLinearDamping(dReal scale)
    {
        dBodySetLinearDamping(id, scale);
    }
    dReal getAngularDamping() const
    {
        return dBodyGetAngularDamping(id);
    }
    void setAngularDamping(dReal scale)
    {
        dBodySetAngularDamping(id, scale);
    }
    void setDamping(dReal linear_scale, dReal angular_scale)
    {
        dBodySetDamping(id, linear_scale, angular_scale);
    }
    dReal getLinearDampingThreshold() const
    {
        return dBodyGetLinearDampingThreshold(id);
    }
    void setLinearDampingThreshold(dReal threshold) const
    {
        dBodySetLinearDampingThreshold(id, threshold);
    }
    dReal getAngularDampingThreshold() const
    {
        return dBodyGetAngularDampingThreshold(id);
    }
    void setAngularDampingThreshold(dReal threshold)
    {
        dBodySetAngularDampingThreshold(id, threshold);
    }
    void setDampingDefaults()
    {
        dBodySetDampingDefaults(id);
    }



    dReal getMaxAngularSpeed() const
    {
        return dBodyGetMaxAngularSpeed(id);
    }
    void setMaxAngularSpeed(dReal max_speed)
    {
        dBodySetMaxAngularSpeed(id, max_speed);
    }

    bool getGyroscopicMode() const
    {
        return dBodyGetGyroscopicMode(id) != 0;
    }
    void setGyroscopicMode(bool mode)
    {
        dBodySetGyroscopicMode(id, mode);
    }


    // TODO: __repr__()

};


%}
