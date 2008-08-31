// wrapper for concrete geoms

%inline %{
class Sphere : public Geom {
    // intentionally undefined, don't use these
    Sphere (Sphere &);
    void operator= (Sphere &);

public:
    //Sphere () { }
    Sphere (dReal radius)
    {
        id = dCreateSphere (0, radius);
    }
    Sphere (Space& space, dReal radius)
    {
        id = dCreateSphere (space.sid(), radius);
    }
/*
    void create (Space& space, dReal radius)
    {
        destroy();
        id = dCreateSphere (space.sid(), radius);
    }

    void create (dReal radius)
    {
        destroy();
        id = dCreateSphere (0, radius);
    }
*/
    void setRadius (dReal radius)
    {
        dGeomSphereSetRadius (id, radius);
    }
    dReal getRadius() const
    {
        return dGeomSphereGetRadius (id);
    }

    // TODO: __rep__()
};
%}
%extend Sphere {
%pythoncode {
    radius = property(getRadius, setRadius)
}
};


%inline %{
class Box : public Geom {
    // intentionally undefined, don't use these
    Box (Box &);
    void operator= (Box &);

public:
    //Box () { }
    Box (dReal lx, dReal ly, dReal lz)
    {
        id = dCreateBox (0, lx, ly, lz);
    }

    Box (Space& space, dReal lx, dReal ly, dReal lz)
    {
        id = dCreateBox (space.sid(), lx, ly, lz);
    }
/*
    void create (dReal lx, dReal ly, dReal lz)
    {
        destroy();
        id = dCreateBox (0,lx,ly,lz);
    }
    void create (Space& space, dReal lx, dReal ly, dReal lz)
    {
        destroy();
        id = dCreateBox (space.sid(),lx,ly,lz);
    }
*/
    void setLengths (dReal lx, dReal ly, dReal lz)
    {
        dGeomBoxSetLengths (id, lx, ly, lz);
    }
    void setLengths (const Vector& len)
    {
        dGeomBoxSetLengths (id, len[0], len[1], len[2]);
    }
    Vector getLengths () const
    {
        Vector result;
        dGeomBoxGetLengths (id, result.data);
        return result;
    }
};
%}
%extend Box {
%pythoncode {
    lengths = property(getLengths, setLengths)
}
};



%inline %{
class Plane : public Geom {
    // intentionally undefined, don't use these
    Plane (Plane &);
    void operator= (Plane &);

public:
    Plane (dReal a, dReal b, dReal c, dReal d)
    {
        id = dCreatePlane (0,a,b,c,d);
    }
    Plane (Space& space, dReal a, dReal b, dReal c, dReal d)
    {
        id = dCreatePlane (space.sid(),a,b,c,d);
    }

    void setParams (dReal a, dReal b, dReal c, dReal d)
    {
        dGeomPlaneSetParams (id, a, b, c, d);
    }
    void getParams (dReal *outA, dReal *outB, dReal *outC, dReal *outD) const
    {
        dVector4 result;
        dGeomPlaneGetParams (id,result);
        *outA = result[0];
        *outB = result[1];
        *outC = result[2];
        *outD = result[3];
    }
};
%}

%inline %{
class Capsule : public Geom {
    // intentionally undefined, don't use these
    Capsule (Capsule &);
    void operator= (Capsule &);

public:
    Capsule (dReal radius, dReal length)
    {
        id = dCreateCapsule (0, radius, length);
    }
    Capsule (Space& space, dReal radius, dReal length)
    {
        id = dCreateCapsule (space.sid(), radius, length);
    }

    void setParams (dReal radius, dReal length)
    {
        dGeomCapsuleSetParams (id, radius, length);
    }
    void getParams (dReal *outA, dReal *outB) const
    {
        dGeomCapsuleGetParams (id, outA, outB);
    }
};
%}

%inline %{
class Cylinder : public Geom {
    // intentionally undefined, don't use these
    Cylinder (Cylinder &);
    void operator= (Cylinder &);

public:
    Cylinder (dReal radius, dReal length)
    {
        id = dCreateCylinder (0, radius, length);
    }
    Cylinder (Space& space, dReal radius, dReal length)
    {
        id = dCreateCylinder (space.sid(), radius, length);
    }

    void setParams (dReal radius, dReal length)
    {
        dGeomCylinderSetParams (id, radius, length);
    }
    void getParams (dReal *outA, dReal *outB) const
    {
        dGeomCylinderGetParams (id, outA, outB);
    }
};
%}

%apply Vector* OUTPUT { Vector* outA, Vector* outB }

%inline %{
class Ray : public Geom {
    // intentionally undefined, don't use these
    Ray (Ray &);
    void operator= (Ray &);

public:
    Ray (dReal length)
    {
        id = dCreateRay (0, length);
    }
    Ray (Space& space, dReal length)
    {
        id = dCreateRay (space.sid(),length);
    }

    void setLength (dReal length)
    {
        dGeomRaySetLength (id, length);
    }
    dReal getLength() const
    {
        return dGeomRayGetLength (id);
    }

    void set (dReal px, dReal py, dReal pz, dReal dx, dReal dy, dReal dz)
    {
        dGeomRaySet (id, px, py, pz, dx, dy, dz);
    }
    void set (const Vector& ori, const Vector& dir)
    {
        dGeomRaySet (id, ori[0], ori[1], ori[2], dir[0], dir[1], dir[2]);
    }
    // TODO: OUTPUT map doesn't seem to be working!
    void get (Vector *outA, Vector *outB) const
    {
        dGeomRayGet (id, outA->data, outB->data);
    }

    void setParams (int firstContact, int backfaceCull)
    {
        dGeomRaySetParams (id, firstContact, backfaceCull);
    }
    void getParams (int *outA, int *outB)
    {
        dGeomRayGetParams (id, outA, outB);
    }
    // TODO: are those documented?
    void setClosestHit (int closestHit)
    {
        dGeomRaySetClosestHit (id, closestHit);
    }
    int getClosestHit()
    {
        return dGeomRayGetClosestHit (id);
    }
    
    void setOrigin(const Vector& newori)
    {
        Vector ori, dir;
        get(&ori, &dir);
        set(newori, dir);
    }
    Vector getOrigin() const
    {
        Vector ori, dir;
        get(&ori, &dir);
        return ori;
    }
    void setDirection(const Vector& newdir)
    {
        Vector ori, dir;
        get(&ori, &dir);
        set(ori, newdir);
    }
    Vector getDirection() const
    {
        Vector ori, dir;
        get(&ori, &dir);
        return dir;
    }
};
%}
%extend Ray {

%pythoncode {
    length = property(getLength, setLength)
    
    origin = property(getOrigin, setOrigin)
    direction = property(getDirection, setDirection)
}
};



