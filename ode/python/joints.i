// wrappers for all joints


%include "ode/enums.h"


%apply dReal * OUTPUT { dReal *outA, dReal *outB };



%immutable dJoint::id;

%inline %{
class Joint {
    // intentionally undefined, don't use these
    Joint (const Joint &) ;
    void operator= (const Joint &);

public:
    dJointID id;

    Joint() : id(0)
    {}

    virtual ~Joint()
    {
        //if (id) dJointDestroy (id);
    }
    
    // some languages don't have reliable destructor semantics
    void destroy()
    {
        if (id) dJointDestroy (id);
        id = 0;
    }

    int getNumBodies() const
    {
        return dJointGetNumBodies(id);
    }

    void attach (Body* body1, Body* body2)
    {
        dJointAttach (id, body1?body1->id:NULL, body2?body2->id:NULL);
    }
/*    void attach1(Body& body1)
    {
        dJointAttach (id, body1.id, NULL);
    }
    void attach2 (Body& body2)
    {
        dJointAttach (id, NULL, body2.id);
    }*/

// TODO wrap those
/*
  void setData (void *data)
    { dJointSetData (id, data); }
  void *getData() const
    { return dJointGetData (id); }
*/

    dJointType getType() const
    {
        return dJointGetType (id);
    }

    // TODO: probably not a good idea to wrap this one
    /*
    dBodyID getBody (int index) const
    {
        return dJointGetBody (id, index);
    }*/

    // TODO: don't use pointers for this
    /*
    void setFeedback(dJointFeedback *fb)
    { dJointSetFeedback(id, fb); }
  dJointFeedback *getFeedback() const
    { return dJointGetFeedback(id); }
    */

    virtual void setParam (int, dReal) {};
    virtual dReal getParam (int) const { return 0; }
};


class BallJoint : public Joint {
    // intentionally undefined, don't use these
    BallJoint (const BallJoint &);
    void operator= (const BallJoint &);

public:
    BallJoint() { }
    
    BallJoint (World& world, dJointGroupID group=0)
    {
        id = dJointCreateBall (world.id, group);
    }

    void create (World& world, dJointGroupID group=0)
    {
        destroy();
        id = dJointCreateBall (world.id, group);
    }

    void setAnchor (dReal x, dReal y, dReal z)
    {
        dJointSetBallAnchor (id, x, y, z);
    }
    void setAnchor (const Vector& a)
    {
        setAnchor (a[0], a[1], a[2]);
    }
    Vector getAnchor () const
    {
        Vector result;
        dJointGetBallAnchor (id, result.data);
        return result;
    }
    Vector getAnchor1() const
    {
        return getAnchor();
    }

    Vector getAnchor2 () const
    {
        Vector result;
        dJointGetBallAnchor2 (id, result.data);
        return result;
    }

    virtual void setParam (int parameter, dReal value)
    {
        dJointSetBallParam (id, parameter, value);
    }
    virtual dReal getParam (int parameter) const
    {
        return dJointGetBallParam (id, parameter);
    }
    // TODO: expose params through methods
} ;



class HingeJoint : public Joint {
    // intentionally undefined, don't use these
    HingeJoint (const HingeJoint &);
    void operator = (const HingeJoint &);

public:
    HingeJoint() { }
    
    HingeJoint (World& world, dJointGroupID group=0)
    {
        id = dJointCreateHinge (world.id, group);
    }

    void create (World& world, dJointGroupID group=0)
    {
        destroy();
        id = dJointCreateHinge (world.id, group);
    }
  
    void setAnchor (dReal x, dReal y, dReal z)
    {
        dJointSetHingeAnchor (id, x, y, z);
    }
    void setAnchor (const Vector& a)
    {
        setAnchor (a[0], a[1], a[2]);
    }
    Vector getAnchor () const
    {
        Vector result;
        dJointGetHingeAnchor (id, result.data);
        return result;
    }
    Vector getAnchor1() const
    {
        return getAnchor();
    }
    Vector getAnchor2 () const
    {
        Vector result;
        dJointGetHingeAnchor2 (id, result.data);
        return result;
    }

    void setAxis (dReal x, dReal y, dReal z)
    {
        dJointSetHingeAxis (id, x, y, z);
    }
    void setAxis (const Vector& a)
    {
        setAxis(a[0], a[1], a[2]);
    }
    Vector getAxis () const
    {
        Vector result;
        dJointGetHingeAxis (id, result.data);
    }

    dReal getAngle() const
    {
        return dJointGetHingeAngle (id);
    }
    dReal getAngleRate() const
    {
        return dJointGetHingeAngleRate (id);
    }

    virtual void setParam (int parameter, dReal value)
    {
        dJointSetHingeParam (id, parameter, value);
    }
    virtual dReal getParam (int parameter) const
    {
        return dJointGetHingeParam (id, parameter);
    }

    // TODO: expose params through methods

    void addTorque (dReal torque)
	{
	    dJointAddHingeTorque(id, torque);
    }
};


class SliderJoint : public Joint {
    // intentionally undefined, don't use these
    SliderJoint (const SliderJoint &);
    void operator = (const SliderJoint &);
public:

    SliderJoint() { }
    
    SliderJoint (World& world, dJointGroupID group=0)
    {
        id = dJointCreateSlider (world.id, group);
    }

    void create (World& world, dJointGroupID group=0)
    {
        destroy();
        id = dJointCreateSlider (world.id, group);
    }

    void setAxis (dReal x, dReal y, dReal z)
    {
        dJointSetSliderAxis (id, x, y, z);
    }
    void setAxis (const Vector& a)
    {
        setAxis (a[0], a[1], a[2]);
    }
    Vector getAxis () const
    {
        Vector result;
        dJointGetSliderAxis (id, result.data);
        return result;
    }

    dReal getPosition() const
    {
        return dJointGetSliderPosition (id);
    }
    dReal getPositionRate() const
    {
        return dJointGetSliderPositionRate (id);
    }

    virtual void setParam (int parameter, dReal value)
    {
        dJointSetSliderParam (id, parameter, value);
    }
    virtual dReal getParam (int parameter) const
    {
        return dJointGetSliderParam (id, parameter);
    }
    // TODO: expose params through methods

    void addForce (dReal force)
	{
	    dJointAddSliderForce(id, force);
    }
};
%}


%inline %{
class UniversalJoint : public Joint {
    // intentionally undefined, don't use these
    UniversalJoint (const UniversalJoint &);
    void operator = (const UniversalJoint &);

public:
    UniversalJoint() { }

    UniversalJoint (World& world, dJointGroupID group=0)
    {
        id = dJointCreateUniversal (world.id, group);
    }
    
    void create (World& world, dJointGroupID group=0)
    {
        destroy();
        id = dJointCreateUniversal (world.id, group);
    }

    void setAnchor (dReal x, dReal y, dReal z)
    {
        dJointSetUniversalAnchor (id, x, y, z);
    }
    void setAnchor (const Vector& a)
    {
        setAnchor(a[0], a[1], a[2]);
    }
    void setAxis1 (dReal x, dReal y, dReal z)
    {
        dJointSetUniversalAxis1 (id, x, y, z);
    }
    void setAxis1 (const Vector& a)
    {
        setAxis1 (a[0], a[1], a[2]);
    }
    void setAxis2 (dReal x, dReal y, dReal z)
    {
        dJointSetUniversalAxis2 (id, x, y, z);
    }
    void setAxis2 (const Vector& a)
    {
        setAxis2 (a[0], a[1], a[2]);
    }

    Vector getAnchor () const
    {
        Vector result;
        dJointGetUniversalAnchor (id, result.data);
        return result;
    }
    Vector getAnchor1() const
    {
        return getAnchor();
    }
    Vector getAnchor2 () const
    {
        Vector result;
        dJointGetUniversalAnchor2 (id, result.data);
        return result;
    }
    Vector getAxis1 () const
    {
        Vector result;
        dJointGetUniversalAxis1 (id, result.data);
        return result;
    }
    Vector getAxis2 () const
    {
        Vector result;
        dJointGetUniversalAxis2 (id, result.data);
        return result;
    }

    virtual void setParam (int parameter, dReal value)
    {
        dJointSetUniversalParam (id, parameter, value);
    }
    virtual dReal getParam (int parameter) const
    {
        return dJointGetUniversalParam (id, parameter);
    }
    // TODO: expose params through methods


    void getAngles(dReal *outA, dReal *outB) const
    {
        dJointGetUniversalAngles (id, outA, outB);
    }

    dReal getAngle1() const
    {
        return dJointGetUniversalAngle1 (id);
    }
  
    dReal getAngle1Rate() const
    {
        return dJointGetUniversalAngle1Rate (id);
    }
    dReal getAngle2() const
    {
        return dJointGetUniversalAngle2 (id);
    }
    dReal getAngle2Rate() const
    {
        return dJointGetUniversalAngle2Rate (id);
    }

    void addTorques (dReal torque1, dReal torque2)
	{
	    dJointAddUniversalTorques(id, torque1, torque2);
    }
};
%}


%inline %{
class Hinge2Joint : public Joint {
    // intentionally undefined, don't use these
    Hinge2Joint (const Hinge2Joint &);
    void operator = (const Hinge2Joint &);

public:
    Hinge2Joint() { }
    Hinge2Joint (World& world, dJointGroupID group=0)
    {
        id = dJointCreateHinge2 (world.id, group);
    }

    void create (World& world, dJointGroupID group=0)
    {
        destroy();
        id = dJointCreateHinge2 (world.id, group);
    }

    void setAnchor (dReal x, dReal y, dReal z)
    {
        dJointSetHinge2Anchor (id, x, y, z);
    }
    void setAnchor (const Vector& a)
    {
        setAnchor(a[0], a[1], a[2]);
    }
    void setAxis1 (dReal x, dReal y, dReal z)
    {
        dJointSetHinge2Axis1 (id, x, y, z);
    }
    void setAxis1 (const Vector& a)
    {
        setAxis1 (a[0], a[1], a[2]);
    }
    void setAxis2 (dReal x, dReal y, dReal z)
    {
        dJointSetHinge2Axis2 (id, x, y, z);
    }
    void setAxis2 (const Vector& a)
    {
        setAxis2 (a[0], a[1], a[2]);
    }
    
    Vector getAnchor () const
    {
        Vector result;
        dJointGetHinge2Anchor (id, result.data);
        return result;
    }
    Vector getAnchor1 () const
    {
        return getAnchor();
    }
    
    Vector getAnchor2 () const
    {
        Vector result;
        dJointGetHinge2Anchor2 (id, result.data);
        return result;
    }
    
    Vector getAxis1 () const
    {
        Vector result;
        dJointGetHinge2Axis1 (id, result.data);
        return result;
    }
    Vector getAxis2 () const
    {
        Vector result;
        dJointGetHinge2Axis2 (id, result.data);
    }

    dReal getAngle1() const
    {
        return dJointGetHinge2Angle1 (id);
    }
    dReal getAngle1Rate() const
    {
        return dJointGetHinge2Angle1Rate (id);
    }
    dReal getAngle2Rate() const
    {
        return dJointGetHinge2Angle2Rate (id);
    }

    virtual void setParam (int parameter, dReal value)
    {
        dJointSetHinge2Param (id, parameter, value);
    }
    virtual dReal getParam (int parameter) const
    {
        return dJointGetHinge2Param (id, parameter);
    }
    // TODO: expose params through methods

    void addTorques(dReal torque1, dReal torque2)
	{
	    dJointAddHinge2Torques(id, torque1, torque2);
    }
};
%}


%inline %{
class PRJoint : public Joint {
    PRJoint (const PRJoint &);
    void operator = (const PRJoint &);

public:
    PRJoint() { }
    PRJoint (World& world, dJointGroupID group=0)
    {
        id = dJointCreatePR (world.id, group);
    }

    void create (World& world, dJointGroupID group=0)
    {
        destroy();
        id = dJointCreatePR (world.id, group);
    }

    void setAnchor (dReal x, dReal y, dReal z)
    {
        dJointSetPRAnchor (id, x, y, z);
    }
    void setAnchor (const Vector& a)
    {
        setAnchor (a[0], a[1], a[2]);
    }
    void setAxis1 (dReal x, dReal y, dReal z)
    {
        dJointSetPRAxis1 (id, x, y, z);
    }
    void setAxis1 (const Vector& a)
    {
        setAxis1(a[0], a[1], a[2]);
    }
    void setAxis2 (dReal x, dReal y, dReal z)
    {
        dJointSetPRAxis2 (id, x, y, z);
    }
    void setAxis2 (const Vector& a)
    {
        setAxis2(a[0], a[1], a[2]);
    }

    Vector getAnchor () const
    {
        Vector result;
        dJointGetPRAnchor (id, result.data);
        return result;
    }
    Vector getAxis1 () const
    {
        Vector result;
        dJointGetPRAxis1 (id, result.data);
        return result;
    }
    Vector getAxis2 () const
    {
        Vector result;
        dJointGetPRAxis2 (id, result.data);
        return result;
    }

    dReal getPosition() const
    {
        return dJointGetPRPosition (id);
    }
    dReal getPositionRate() const
    {
        return dJointGetPRPositionRate (id);
    }

    virtual void setParam (int parameter, dReal value)
    {
        dJointSetPRParam (id, parameter, value);
    }
    virtual dReal getParam (int parameter) const
    {
        return dJointGetPRParam (id, parameter);
    }
};
%}

%inline %{
class PUJoint : public Joint
{
    PUJoint (const PUJoint &);
    void operator = (const PUJoint &);

public:
    PUJoint() { }
    PUJoint (World& world, dJointGroupID group=0)
    {
        id = dJointCreatePU (world.id, group);
    }

    void create (World& world, dJointGroupID group=0)
    {
        destroy();
        id = dJointCreatePU (world.id, group);
    }
    
    void setAnchor (dReal x, dReal y, dReal z)
    {
        dJointSetPUAnchor (id, x, y, z);
    }
    void setAnchor (const Vector& a)
    {
        setAnchor (a[0], a[1], a[2]);
    }
    void setAxis1 (dReal x, dReal y, dReal z)
    {
        dJointSetPUAxis1 (id, x, y, z);
    }
    void setAxis1 (const Vector& a)
    {
        setAxis1(a[0], a[1], a[2]);
    }
    void setAxis2 (dReal x, dReal y, dReal z)
    {
        dJointSetPUAxis2 (id, x, y, z);
    }
    void setAxis2 (const Vector& a)
    {
        setAxis2(a[0], a[1], a[2]);
    }
    void setAxis3 (dReal x, dReal y, dReal z)
    {
        dJointSetPUAxis3 (id, x, y, z);
    }
    void setAxis3 (const Vector& a)
    {
        setAxis3(a[0], a[1], a[2]);
    }
    void setAxisP (dReal x, dReal y, dReal z)
    {
        dJointSetPUAxis3 (id, x, y, z);
    }
    void setAxisP (const Vector& a)
    {
        setAxisP(a[0], a[1], a[2]);
    }

    // TODO: why is this virtual?
    virtual Vector getAnchor () const
    {
        Vector result;
        dJointGetPUAnchor (id, result.data);
        return result;
    }
    Vector getAxis1 () const
    {
        Vector result;
        dJointGetPUAxis1 (id, result.data);
        return result;
    }
    Vector getAxis2 () const
    {
        Vector result;
        dJointGetPUAxis2 (id, result.data);
        return result;
    }
    Vector getAxis3 () const
    {
        Vector result;
        dJointGetPUAxis3 (id, result.data);
        return result;
    }
    Vector getAxisP () const
    {
        Vector result;
        dJointGetPUAxis3 (id, result.data);
        return result;
    }

    dReal getAngle1() const
    {
        return dJointGetPUAngle1 (id);
    }
    dReal getAngle1Rate() const
    {
        return dJointGetPUAngle1Rate (id);
    }
    dReal getAngle2() const
    {
        return dJointGetPUAngle2 (id);
    }
    dReal getAngle2Rate() const
    {
        return dJointGetPUAngle2Rate (id);
    }

    dReal getPosition() const
    {
        return dJointGetPUPosition (id);
    }
    dReal getPositionRate() const
    {
        return dJointGetPUPositionRate (id);
    }

    virtual void setParam (int parameter, dReal value)
    {
        dJointSetPUParam (id, parameter, value);
    }
    virtual dReal getParam (int parameter) const
    {
        return dJointGetPUParam (id, parameter);
    }
  // TODO: expose params through methods
};

%}


%inline %{
class PistonJoint : public Joint {
    // intentionally undefined, don't use these
    PistonJoint (const PistonJoint &);
    void operator = (const PistonJoint &);

public:
    PistonJoint() { }
    PistonJoint (World& world, dJointGroupID group=0)
    {
        id = dJointCreatePiston (world.id, group);
    }

    void create (World& world, dJointGroupID group=0)
    {
        destroy();
        id = dJointCreatePiston (world.id, group);
    }

    void setAnchor (dReal x, dReal y, dReal z)
    {
        dJointSetPistonAnchor (id, x, y, z);
    }
    void setAnchor (const Vector& a)
    {
        setAnchor (a[0], a[1], a[2]);
    }
    Vector getAnchor () const
    {
        Vector result;
        dJointGetPistonAnchor (id, result.data);
        return result;
    }
    Vector getAnchor1() const
    {
        return getAnchor();
    }
    Vector getAnchor2 () const
    {
        Vector result;
        dJointGetPistonAnchor2 (id, result.data);
        return result;
    }

    void setAxis (dReal x, dReal y, dReal z)
    {
        dJointSetPistonAxis (id, x, y, z);
    }
    void setAxis (const Vector& a)
    {
        setAxis(a[0], a[1], a[2]);
    }
    Vector getAxis () const
    {
        Vector result;
        dJointGetPistonAxis (id, result.data);
        return result;
    }

    dReal getPosition() const
    {
        return dJointGetPistonPosition (id);
    }
    dReal getPositionRate() const
    {
        return dJointGetPistonPositionRate (id);
    }

    virtual void setParam (int parameter, dReal value)
    {
        dJointSetPistonParam (id, parameter, value);
    }
    virtual dReal getParam (int parameter) const
    {
        return dJointGetPistonParam (id, parameter);
    }
    // TODO: expose params through methods


    void addForce (dReal force)
    {
        dJointAddPistonForce (id, force);
    }
};
%}


%inline %{
class FixedJoint : public Joint {
    // intentionally undefined, don't use these
    FixedJoint (const FixedJoint &);
    void operator = (const FixedJoint &);

public:
    FixedJoint() { }
    FixedJoint (World& world, dJointGroupID group=0)
    {
        id = dJointCreateFixed (world.id, group);
    }

    void create (World& world, dJointGroupID group=0)
    {
        destroy();
        id = dJointCreateFixed (world.id, group);
    }

    void set()
    {
        dJointSetFixed (id);
    }

    virtual void setParam (int parameter, dReal value)
    {
        dJointSetFixedParam (id, parameter, value);
    }

    virtual dReal getParam (int parameter) const
    {
        return dJointGetFixedParam (id, parameter);
    }
    // TODO: expose params through methods
};
%}


//%apply dContact *INPUT { dContact *contact };

%inline %{
typedef struct dContact;

class ContactJoint : public Joint {
    // intentionally undefined, don't use these
    ContactJoint (const ContactJoint &);
    void operator = (const ContactJoint &);

public:
    ContactJoint() { }
    
    // TODO: we probably will need to wrap dContact, thus this constructor
    
    ContactJoint (World& world, dJointGroupID group, dContact *contact)
    {
        id = dJointCreateContact (world.id, group, contact);
    }

    void create (World& world, dJointGroupID group, dContact *contact)
    {
        destroy();
        id = dJointCreateContact (world.id, group, contact);
    }
};
%}

%inline %{
class NullJoint : public Joint {
    // intentionally undefined, don't use these
    NullJoint (const NullJoint &);
    void operator = (const NullJoint &);

public:
    NullJoint() { }
    NullJoint (World& world, dJointGroupID group=0)
    {
        id = dJointCreateNull (world.id, group);
    }

    void create (World& world, dJointGroupID group=0)
    {
        destroy();
        id = dJointCreateNull (world.id, group);
    }
};
%}


%inline %{
class AMotorJoint : public Joint {
    // intentionally undefined, don't use these
    AMotorJoint (const AMotorJoint &);
    void operator = (const AMotorJoint &);

public:
    AMotorJoint() { }
    AMotorJoint (World& world, dJointGroupID group=0)
    {
        id = dJointCreateAMotor (world.id, group);
    }

    void create (World& world, dJointGroupID group=0)
    {
        destroy();
        id = dJointCreateAMotor (world.id, group);
    }

    void setMode (dAMotorMode mode)
    {
        dJointSetAMotorMode (id, mode);
    }
    dAMotorMode getMode() const
    {
        return dAMotorMode(dJointGetAMotorMode (id));
    }

    void setNumAxes (int num)
    {
        dJointSetAMotorNumAxes (id, num);
    }
    int getNumAxes() const
    {
        return dJointGetAMotorNumAxes (id);
    }

    void setAxis (int anum, int rel, dReal x, dReal y, dReal z)
    {
        dJointSetAMotorAxis (id, anum, rel, x, y, z);
    }
    void setAxis (int anum, int rel, const Vector& a)
    {
        setAxis(anum, rel, a[0], a[1], a[2]);
    }
    Vector getAxis (int anum) const
    {
        Vector result;
        dJointGetAMotorAxis (id, anum, result.data);
        return result;
    }
    int getAxisRel (int anum) const
    {
        return dJointGetAMotorAxisRel (id, anum);
    }

    void setAngle (int anum, dReal angle)
    {
        dJointSetAMotorAngle (id, anum, angle);
    }
    dReal getAngle (int anum) const
    {
        return dJointGetAMotorAngle (id, anum);
    }
    dReal getAngleRate (int anum)
    {
        return dJointGetAMotorAngleRate (id,anum);
    }

    void setParam (int parameter, dReal value)
    {
        dJointSetAMotorParam (id, parameter, value);
    }
    dReal getParam (int parameter) const
    {
        return dJointGetAMotorParam (id, parameter);
    }
    // TODO: expose params through methods

    void addTorques(dReal torque1, dReal torque2, dReal torque3)
	{
        dJointAddAMotorTorques(id, torque1, torque2, torque3);
    }
};
%}


%inline %{
class LMotorJoint : public Joint {
    // intentionally undefined, don't use these
    LMotorJoint (const LMotorJoint &);
    void operator = (const LMotorJoint &);

public:
    LMotorJoint() { }
    LMotorJoint (World& world, dJointGroupID group=0)
    {
        id = dJointCreateLMotor (world.id, group);
    }

    void create (World& world, dJointGroupID group=0)
    {
        destroy();
        id = dJointCreateLMotor (world.id, group);
    }

    void setNumAxes (int num)
    {
        dJointSetLMotorNumAxes (id, num);
    }
    int getNumAxes() const
    {
        return dJointGetLMotorNumAxes (id);
    }

    void setAxis (int anum, int rel, dReal x, dReal y, dReal z)
    {
        dJointSetLMotorAxis (id, anum, rel, x, y, z);
    }
    void setAxis (int anum, int rel, const Vector a)
    {
        setAxis(anum, rel, a[0], a[1], a[2]);
    }
    Vector getAxis (int anum) const
    {
        Vector result;
        dJointGetLMotorAxis (id, anum, result.data);
        return result;
    }

    void setParam (int parameter, dReal value)
    {
        dJointSetLMotorParam (id, parameter, value);
    }
    dReal getParam (int parameter) const
    {
        return dJointGetLMotorParam (id, parameter);
    }
    // TODO: expose params through methods
};

%}


