// Here are the basic types.

%{
#include <string.h>
%}

%include "typemaps.i"

// Quaternion
%ignore Quaternion::data;

%inline %{
class Matrix;

struct Quaternion {
    dReal data[4];

    dReal __getitem__(int i) const
    {
        return data[i];
    }
    void __setitem__(int i, dReal v)
    {
        data[i] = v;
    }
    
    char* __repr__() const
    {
        static char buffer[256];
        sprintf(buffer, "Quaternion ( w = %f, x = %f, y = %f, z = %f)",
            data[0], data[1], data[2], data[3]);
        return buffer;
    }
    char* __str__() const
    {
        static char buffer[256];
        sprintf(buffer, "(%f, %f, %f, %f)",
            data[0], data[1], data[2], data[3]);
        return buffer;
    }
    
    
    
    Quaternion()
    {
        dQSetIdentity(data);
    }
    
    Quaternion(const Quaternion& q)
    {
        if (&q != this)
            memcpy(data, q.data, sizeof data);
    }
    
    Quaternion(dReal w, dReal x, dReal y, dReal z)
    {
        data[0] = w;
        data[1] = x;
        data[2] = y;
        data[3] = z;
    }

    Quaternion(const dReal *p)
    {
        memcpy(data, p, sizeof data);
    }
    
    void normalize()
    {
        dNormalize4(data);
    }
    
    // TODO: multiplication, constructor from axis and angle
    
    Matrix toR() const;
};
%}
%extend Quaternion {
%pythoncode %{
    def getw(self): return self[0]
    def setw(self,v): self[0]=v
    def getx(self): return self[1]
    def setx(self,v): self[1]=v
    def gety(self): return self[2]
    def sety(self,v): self[2]=v
    def getz(self): return self[3]
    def setz(self,v): self[3]=v

    w = property(getw, setw)
    x = property(getx, setx)
    y = property(gety, sety)
    z = property(getz, setz)
%}
};





// Vector
%ignore Vector::data;
%ignore Vector::Vector(const dReal*);
%ignore Vector::operator[];

%inline %{
struct Vector {
    dReal data[4];

    dReal __getitem__(int i) const
    {
        return data[i];
    }
    void __setitem__(int i, dReal v)
    {
        data[i] = v;
    }
    dReal operator[](int i) const
    {
        return data[i];
    }
    
    char* __repr__() const
    {
        static char buffer[256];
        sprintf(buffer, "Vector ( x = %f, y = %f, z = %f )",
            data[0], data[1], data[2]);
        return buffer;
    }
    char* __str__() const
    {
        static char buffer[256];
        sprintf(buffer, "(%f, %f, %f)",
            data[0], data[1], data[2]);
        return buffer;
    }
    
    Vector()
    {
        data[0] = data[1] = data[2] = data[3] = 0;
    }
    Vector(dReal x, dReal y, dReal z)
    {
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }
    
    Vector(const Vector& v)
    {
        if (&v != this)
            memcpy(data, v.data, sizeof data);
    }
    
    Vector(const dReal *v)
    {
        memcpy(data, v, sizeof data);
    }
    
    dReal lengthSquared() const
    {
        return dLENGTHSQUARED(data);
    }
    
    dReal length() const
    {
        return dLENGTH(data);
    }
    
    
    Vector& operator*=(dReal a)
    {
        dOPEC(data, *=, a);
    }
    Vector& operator/=(dReal a)
    {
        dOPEC(data, /=, a);
    }
    
    Vector& operator+=(const Vector& v)
    {
        dOPE(data, +=, v.data);
    }
    Vector& operator-=(const Vector& v)
    {
        dOPE(data, -=, v.data);
    }
    
    Vector operator-() const
    {
        return Vector(-data[0], -data[1], -data[2]);
    }

    Vector operator+(const Vector& b) const
    {
        return Vector(
            data[0]+b.data[0],
            data[1]+b.data[1],
            data[2]+b.data[2]
        );
    }

    Vector operator-(const Vector& b) const
    {
        return Vector(
            data[0]-b.data[0],
            data[1]-b.data[1],
            data[2]-b.data[2]
        );
    }
    
    Vector operator*(dReal b) const
    {
        return Vector(
            data[0]*b,
            data[1]*b,
            data[2]*b
        );
    }
    
    Vector operator/(dReal b) const
    {
        return Vector(
            data[0]/b,
            data[1]/b,
            data[2]/b
        );
    }
    
    dReal operator*(const Vector& b) const
    {
        return dDOT(data, b.data);
    }
    
    void normalize()
    {
        dNormalize3(data);
    }
    
    Vector copy() const
    {
        return Vector(*this);
    }
};

dReal dot(const Vector& a, const Vector& b)
{
    return a * b;
}

%}
%extend Vector {
%pythoncode %{
    def getx(self): return self[0]
    def setx(self,v): self[0]=v
    def gety(self): return self[1]
    def sety(self,v): self[1]=v
    def getz(self): return self[2]
    def setz(self,v): self[2]=v

    x = property(getx, setx)
    y = property(gety, sety)
    z = property(getz, setz)
%}
};
 



// Matrix3
%ignore Matrix::data;
%ignore Matrix::Matrix(const dReal *);

%inline %{
struct Matrix {
    dReal data[3*4];
    
    dReal get(int i, int j) const
    {
        return data[i*4 + j];
    }
    void set(int i, int j, dReal v)
    {
        data[i*4 + j] = v;
    }

    
    char* __repr__() const
    {
        static char buffer[512];
        sprintf(buffer,
            "Matrix (%f,\t%f,\t%f,\n"
            "         %f,\t%f,\t%f,\n"
            "         %f,\t%f,\t%f)\n",
            data[0], data[1], data[2],
            data[4], data[5], data[6],
            data[8], data[9], data[10]
            );
        return buffer;
    }
    char* __str__() const
    {
        static char buffer[512];
        sprintf(buffer,
            "(%f,\t%f,\t%f,\n"
            " %f,\t%f,\t%f,\n"
            " %f,\t%f,\t%f)\n",
            data[0], data[1], data[2],
            data[4], data[5], data[6],
            data[8], data[9], data[10]
            );
        return buffer;
    }



    Matrix()
    {
        dRSetIdentity(data);
    }
    
    Matrix(const Matrix& m)
    {
        if (this != &m)
            memcpy(data, m.data, sizeof data);
    }
    
    Matrix(const dReal *m)
    {
        memcpy(data, m, sizeof data);
    }
    
    Matrix copy() const
    {
        return Matrix(*this);
    }
    
    Matrix(const Vector& axis, dReal angle)
    {
        dRFromAxisAndAngle(data, axis[0], axis[1], axis[2], angle);
    }
    
    Matrix(const Vector& axisA, const Vector& axisB)
    {
        dRFrom2Axes(data, 
                    axisA[0], axisA[1], axisA[2],
                    axisB[0], axisB[1], axisB[2]);
    }
    
    // TODO: matrix*matrix
    // TODO: matrix*vector


    Quaternion toQ() const;
};
%}
%extend Matrix {
%pythoncode %{
    def __getitem__(self, i):
        return self.get(i[0], i[1])

    def __setitem__(self, i, v):
        self.set(i[0], i[1], v);

%}
};
 


// conversion between matrix and quaternion
%{
Quaternion Matrix::toQ() const
{
    Quaternion q;
    dRtoQ(data, q.data);
    return q;
}

Matrix Quaternion::toR() const
{
    Matrix m;
    dQtoR(data, m.data);
    return m;
}

%}




// TODO: maybe add some useful methods here?
%inline %{
struct AABB {
    Vector min, max;
    
    AABB() {}
    AABB(const dReal *p) :
        min(p[0], p[2], p[3]),
        max(p[1], p[3], p[5])
    {}
};
%}



