// wrapper for space classes

%ignore Space::sid;

%inline %{
class Space : public Geom {
    // intentionally undefined, don't use these
    Space (Space &);
    void operator= (Space &);

protected:
    // the default constructor is protected so that you
    // can't instance this class. you must instance one
    // of its subclasses instead.
    Space (dSpaceID s) { id = reinterpret_cast<dGeomID>(s); }
public:
    dSpaceID sid() const
    {
        return reinterpret_cast<dSpaceID>(id);
    }

    void setCleanup (int mode)
    {
        dSpaceSetCleanup (sid(), mode);
    }
    int getCleanup() const
    {
        return dSpaceGetCleanup (sid());
    }

    // TODO: dGeomID overloads might be useful here
    void add (Geom& x)
    {
        dSpaceAdd (sid(), x.id);
    }
    void remove (Geom& x)
    {
        dSpaceRemove (sid(), x.id);
    }
    bool query (Geom& x) const
    {
        return dSpaceQuery (sid(),x.id) != 0;
    }

    int getNumGeoms() const
    {
        return dSpaceGetNumGeoms (sid());
    }
    // TODO: might be a good idea to wrap this
    dGeomID getGeom (int i)
    {
        return dSpaceGetGeom (sid(),i);
    }

    // TODO: wrap this
    /*
    void collide (void *data, dNearCallback *callback)
    {
        dSpaceCollide (id(),data,callback);
    }*/
};
%}


%inline %{
class SimpleSpace : public Space {
    // intentionally undefined, don't use these
    SimpleSpace (SimpleSpace &);
    void operator= (SimpleSpace &);

public:
    SimpleSpace () :
        Space(dSimpleSpaceCreate (0))
    { }
    SimpleSpace (Space& space) :
        Space(dSimpleSpaceCreate(space.sid()))
    { }
};
%}


%apply int * OUTPUT { int *outA, int *outB };

%inline %{
class HashSpace : public Space {
    // intentionally undefined, don't use these
    HashSpace (HashSpace &);
    void operator= (HashSpace &);

public:
    HashSpace() :
        Space(dHashSpaceCreate (0))
    { }
    HashSpace (Space& space) :
        Space(dHashSpaceCreate (space.sid()))
    { }

    void setLevels (int minlevel, int maxlevel)
    {
        dHashSpaceSetLevels (sid(), minlevel, maxlevel);
    }
    int getLevels (int *outA, int *outB) const
    {
        dHashSpaceGetLevels (sid(), outA, outB);
    }
};


class QuadTreeSpace : public Space {
    // intentionally undefined, don't use these
    QuadTreeSpace (QuadTreeSpace &);
    void operator= (QuadTreeSpace &);

public:
    QuadTreeSpace (const Vector& center, const Vector& extents, int depth) :
        Space(
            dQuadTreeSpaceCreate (0,center.data,extents.data,depth)
        )
    { }

    QuadTreeSpace (Space& space, const Vector& center, const Vector& extents, int depth) :
        Space(
            dQuadTreeSpaceCreate (space.sid(),center.data,extents.data,depth)
        )
    { }
    
    // TODO: add some overloads
};

%}


// TODO: wrap SAP space too

