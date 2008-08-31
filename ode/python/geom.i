// wrapper for dGeomID

%immutable Geom::id;

%inline %{
class Geom {
    // intentionally undefined, don't use these
    Geom (Geom &);
    void operator= (Geom &);
protected:
    Geom()
    { id = 0; }
    ~Geom()
    { }

public:
    dGeomID id;

    void destroy() {
        if (id) dGeomDestroy (id);
        id = 0;
    }

    int getClass() const
    {
        return dGeomGetClass (id);
    }

    dSpaceID getSpace() const
    {
        return dGeomGetSpace (id);
    }

    // TODO: deal with this
    /*
    void setData (void *data)
    {
        dGeomSetData (id,data);
    }
    void *getData() const
    {
        return dGeomGetData (id);
    }
    */

    void setBody (dBodyID b)
    {
        dGeomSetBody (id,b);
    }
    dBodyID getBody() const
    {
        return dGeomGetBody (id);
    }

    void setPosition (dReal x, dReal y, dReal z)
    {
        dGeomSetPosition (id,x,y,z);
    }
    void setPosition (const Vector& pos)
    {
        setPosition (pos[0], pos[1], pos[2]);
    }
    
    Vector getPosition() const
    {
        return Vector(dGeomGetPosition (id));
    }

    void setRotation (const Matrix& R)
    {
        dGeomSetRotation (id,R.data);
    }
    Matrix getRotation() const
    {
        return Matrix(dGeomGetRotation (id));
    }
    
    void setQuaternion (const Quaternion& quat)
    {
        dGeomSetQuaternion (id,quat.data);
    }
    Quaternion getQuaternion () const
    {
        Quaternion quat;
        dGeomGetQuaternion (id,quat.data);
        return quat;
    }

    AABB getAABB () const
    {
        dReal aabb[6];
        dGeomGetAABB (id, aabb);
        return AABB(aabb);
    }

    bool isSpace()
    {
        return dGeomIsSpace (id) != 0;
    }

    void setCategoryBits (unsigned long bits)
    {
        dGeomSetCategoryBits (id, bits);
    }
    void setCollideBits (unsigned long bits)
    {
        dGeomSetCollideBits (id, bits);
    }
    unsigned long getCategoryBits()
    {
        return dGeomGetCategoryBits (id);
    }
    unsigned long getCollideBits()
    {
        return dGeomGetCollideBits (id);
    }

    void enable()
    {
        dGeomEnable (id);
    }
    void disable()
    {
        dGeomDisable (id);
    }
    bool isEnabled() const
    {
        return dGeomIsEnabled (id) != 0;
    }

    // TODO wrap this
    /*
    void collide2 (Geom& g, void *data, dNearCallback *callback)
    {
        dSpaceCollide2 (id, g.id, data, callback);
    }
    */

    void setOffsetPosition(dReal x, dReal y, dReal z)
    {
        dGeomSetOffsetPosition(id, x, y, z);
    }
    void setOffsetPosition(const Vector& p)
    {
        setOffsetPosition(p[0], p[1], p[2]);
    }
    void setOffsetRotation(const Matrix& R)
    {
        dGeomSetOffsetRotation (id, R.data);
    }
    void setOffsetQuaternion(const Quaternion& q)
    {
        dGeomSetOffsetQuaternion(id, q.data);
    }

    Vector getOffsetPosition() const
    {
        return Vector(dGeomGetOffsetPosition(id));
    }
    Matrix getOffsetRotation() const
    {
        return Matrix(dGeomGetOffsetRotation(id));
    }
    Quaternion getOffsetQuaternion() const
    {
        Quaternion result;
        dGeomGetOffsetQuaternion (id, result.data);
        return result;
    }

    void setOffsetWorldPosition(dReal x, dReal y, dReal z)
    {
        dGeomSetOffsetWorldPosition (id, x, y, z);
    }
    void setOffsetWorldPosition(const Vector &p)
    {
        setOffsetWorldPosition(p[0], p[1], p[2]);
    }
    void setOffsetWorldRotation(const Matrix& R)
    {
        dGeomSetOffsetWorldRotation(id, R.data);
    }
    void setOffsetWorldQuaternion(const Quaternion& q)
    {
        dGeomSetOffsetWorldQuaternion(id, q.data);
    }

    void clearOffset()
    {
        dGeomClearOffset (id);
    }
};
%}

%extend Geom {
    void setEnabled(bool e)
    {
        if (e)
            $self->enable();
        else
            $self->disable();
    }
%pythoncode {
    enabled = property(isEnabled, setEnabled)

    categoryBits = property(getCategoryBits, setCategoryBits)
    collideBits = property(getCollideBits, setCollideBits)
}
};

