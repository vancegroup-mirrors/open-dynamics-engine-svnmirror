// wrapper for dMass

%ignore Mass::data;

%inline %{
struct Mass {
    dMass data;

    char* __repr__() const
    {
        static char buffer[512];
        sprintf(buffer, "Mass ( mass = %f, center = (%f, %f, %f),\n"
        "I = ( %f, %f, %f,\n"
        "      %f, %f, %f,\n"
        "      %f, %f, %f)",
            data.mass,
            data.c[0], data.c[1], data.c[2],
            data.I[0], data.I[1], data.I[2],
            data.I[4], data.I[5], data.I[6], 
            data.I[8], data.I[9], data.I[10]
        );
        return buffer;
    }

    Mass()
    {}
    
    Mass(const Mass& m)
    {
        if (this != &m)
            memcpy(&data, &m.data, sizeof data);
    }
    Mass(const dMass *m)
    {
        memcpy(&data, m, sizeof data);
    }

    void set (dReal mass, const Vector& center,
            dReal I11, dReal I22, dReal I33,
            dReal I12, dReal I13, dReal I23)
    {
        data.setParameters(mass,
                center[0], center[1], center[2],
                I11, I22, I33, I12, I13, I23);
    }

    void setSphere (dReal density, dReal radius)
    {
        data.setSphere(density,radius);
    }
    void setSphereTotal (dReal total, dReal radius)
    {
        data.setSphereTotal (total,radius);
    }

    void setCapsule (dReal density, int direction, dReal radius, dReal length)
    {
        data.setCapsule (density,direction,radius,length);
    }
  
    void setCapsuleTotal (dReal total, int direction, dReal radius, dReal length)
    { 
        data.setCapsule (total,direction,radius,length);
    }

    void setCylinder(dReal density, int direction, dReal radius, dReal length)
    {
        data.setCylinder (density,direction,radius,length);
    }
    void setCylinderTotal(dReal total, int direction, dReal radius, dReal length)
    {
        data.setCylinderTotal (total,direction,radius,length);
    }

    void setBox (dReal density, dReal lx, dReal ly, dReal lz)
    {
        data.setBox (density,lx,ly,lz);
    }
    void setBoxTotal (dReal total, dReal lx, dReal ly, dReal lz)
    {
        data.setBoxTotal (total,lx,ly,lz);
    }

    // TODO: wrap those
    void setTrimesh(dReal density, dGeomID g)
    {
        data.setTrimesh (density, g);
    }
    void setTrimeshTotal(dReal total, dGeomID g)
    {
        data.setTrimeshTotal (total, g);
    }

    void adjust (dReal newmass)
    {
        data.adjust (newmass);
    }
  
    void translate (dReal x, dReal y, dReal z)
    {
        data.translate (x,y,z);
    }
    void translate (const Vector& v)
    {
        translate (v[0], v[1], v[2]);
    }
    
    // TODO: overload for quaternions too
    void rotate (const Matrix& R)
    {
        data.rotate (R.data);
    }
    
    void add (const Mass& b)
    {
        data.add (&b.data);
    }
    Mass& operator+=(const Mass& m)
    {
        add(m);
    }
    // TODO: make those properties
    dReal mass() const
    {
        return data.mass;
    }
    
    Vector center() const
    {
        return Vector(data.c);
    }
    
    Matrix I() const
    {
        return Matrix(data.I);
    }
};


%}
