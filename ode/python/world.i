// wrapper for dWorldID

%immutable World::id;
%immutable World::body;

%inline %{
typedef struct dxWorld* dWorldID;

class World {
    // intentionally undefined, don't use these
    World (const World &);
    void operator= (const dWorld &);
public:
    dWorldID id;

    World()
    {
        id = dWorldCreate();
    }
    ~World()
    {
        dWorldDestroy (id);
    }


    // setup functions
    
    void setGravity (dReal x, dReal y, dReal z)
    {
        dWorldSetGravity (id,x,y,z);
    }
    
    void setGravity (const Vector& g)
    {
        setGravity (g[0], g[1], g[2]);
    }
    
    Vector getGravity () const
    {
        Vector g;
        dWorldGetGravity (id, g.data);
        return g;
    }

    void setERP (dReal erp)
    {
        dWorldSetERP(id, erp);
    }
    dReal getERP() const
    {
        return dWorldGetERP(id);
    }

    void setCFM (dReal cfm)
    {
        dWorldSetCFM(id, cfm);
    }
    dReal getCFM() const
    {
        return dWorldGetCFM(id);
    }


    // step functions

    void step (dReal stepsize)
    {
        dWorldStep (id, stepsize);
    }

    void stepFast1 (dReal stepsize, int maxiterations)
    {
        dWorldStepFast1 (id, stepsize, maxiterations);
    }
    void setAutoEnableDepthSF1(dWorldID, int depth)
    {
        dWorldSetAutoEnableDepthSF1 (id, depth);
    }
    int getAutoEnableDepthSF1() const
    {
        return dWorldGetAutoEnableDepthSF1 (id);
    }

    void quickStep(dReal stepsize)
    {
        dWorldQuickStep (id, stepsize);
    }
    void setQuickStepNumIterations(int num)
    {
        dWorldSetQuickStepNumIterations (id, num);
    }
    int getQuickStepNumIterations() const
    {
        return dWorldGetQuickStepNumIterations (id);
    }
    void setQuickStepW(dReal over_relaxation)
    {
        dWorldSetQuickStepW (id, over_relaxation);
    }
    dReal getQuickStepW() const
    {
        return dWorldGetQuickStepW (id);
    }


    // autodisable

    void  setAutoDisableLinearThreshold (dReal threshold) 
    {
        dWorldSetAutoDisableLinearThreshold (id,threshold);
    }
    dReal getAutoDisableLinearThreshold() const
    {
        return dWorldGetAutoDisableLinearThreshold (id);
    }
    void setAutoDisableAngularThreshold (dReal threshold)
    {
        dWorldSetAutoDisableAngularThreshold (id,threshold);
    }
    dReal getAutoDisableAngularThreshold() const
    {
        return dWorldGetAutoDisableAngularThreshold (id);
    }
    void setAutoDisableSteps (int steps)
    {
        dWorldSetAutoDisableSteps (id, steps);
    }
    int getAutoDisableSteps() const
    {
        return dWorldGetAutoDisableSteps (id);
    }
    void setAutoDisableTime (dReal time)
    {
        dWorldSetAutoDisableTime (id, time);
    }
    dReal getAutoDisableTime() const
    {
        return dWorldGetAutoDisableTime (id);
    }
    void setAutoDisableFlag (bool do_auto_disable)
    {
        dWorldSetAutoDisableFlag (id, do_auto_disable);
    }
    bool getAutoDisableFlag() const
    {
        return dWorldGetAutoDisableFlag (id) != 0;
    }


    // damping

    dReal getLinearDampingThreshold() const
    {
        return dWorldGetLinearDampingThreshold(id);
    }
    void setLinearDampingThreshold(dReal threshold)
    {
        dWorldSetLinearDampingThreshold(id, threshold);
    }
    dReal getAngularDampingThreshold() const
    {
        return dWorldGetAngularDampingThreshold(id);
    }
    void setAngularDampingThreshold(dReal threshold)
    {
        dWorldSetAngularDampingThreshold(id, threshold);
    }
    dReal getLinearDamping() const
    {
        return dWorldGetLinearDamping(id);
    }
    void setLinearDamping(dReal scale)
    {
        dWorldSetLinearDamping(id, scale);
    }
    dReal getAngularDamping() const
    {
        return dWorldGetAngularDamping(id);
    }
    void setAngularDamping(dReal scale)
    {
        dWorldSetAngularDamping(id, scale);
    }
    void setDamping(dReal linear_scale, dReal angular_scale)
    {
        dWorldSetDamping(id, linear_scale, angular_scale);
    }


    // other functions

    dReal getMaxAngularSpeed() const
    {
        return dWorldGetMaxAngularSpeed(id);
    }
    void setMaxAngularSpeed(dReal max_speed)
    {
        dWorldSetMaxAngularSpeed(id, max_speed);
    }

    void setContactSurfaceLayer(dReal depth)
    {
        dWorldSetContactSurfaceLayer (id, depth);
    }
    dReal getContactSurfaceLayer() const
    {
        return dWorldGetContactSurfaceLayer (id);
    }

    Vector impulseToForce (dReal stepsize, const Vector& i)
    {
        Vector force;
        dWorldImpulseToForce (id, stepsize, i[0], i[1], i[2], force.data);
        return force;
    }
    
    Vector impulseToForce (dReal stepsize, dReal ix, dReal iy, dReal iz)
    {
        Vector force;
        dWorldImpulseToForce (id, stepsize, ix, iy, iz, force.data);
        return force;
    }


    // TODO: __repr__()

};


%}
