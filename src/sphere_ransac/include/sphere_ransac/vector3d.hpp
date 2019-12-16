/*
I am creating a class vector3d which will be used for vector operation . I have overloaded all the basic functions for the required for vector operation .
There is no need for extra class , so no inheritance was necessary .
*/

#ifndef VECTOR3D_HPP_
#define VECTOR3D_HPP_

#include<iostream>
#include<math.h>
#include<assert.h>
// using namespace std;
typedef float f;


class vector3d
{
public:
    f x,y,z;
    vector3d()  //constructor
    {
        x=0;
        y=0;
        z=0;
    }

    vector3d(f x1,f y1,f z1=0)     //initializing object with values.
    {
        x=x1;
        y=y1;
        z=z1;
    }
    vector3d(const vector3d &vec);    //copy constructor
    vector3d operator+(const vector3d &vec);    //addition
    vector3d &operator+=(const vector3d &vec);  ////assigning new result to the vector
    vector3d operator-(const vector3d &vec);    //substraction
    vector3d &operator-=(const vector3d &vec);  //assigning new result to the vector
    vector3d operator*(f value);    //multiplication
    vector3d &operator*=(f value);  //assigning new result to the vector.
    vector3d operator/(f value);    //division
    vector3d &operator/=(f value);  //assigning new result to the vector
    vector3d &operator=(const vector3d &vec);
    f dot_product(const vector3d &vec); //scalar dot_product
    vector3d cross_product(const vector3d &vec);    //cross_product
    f magnitude();  //magnitude of the vector
    vector3d normalization();   //nor,malized vector
    f square(); //gives square of the vector

    f distance(const vector3d &vec);    //gives distance between two vectors
    f show_X(); //return x
    f show_Y(); //return y
    f show_Z(); //return z
    void disp();    //display value of vectors
};

vector3d::vector3d(const vector3d &vec)
{
    x=vec.x;
    y=vec.y;
    z=vec.z;
}

//addition

vector3d vector3d ::operator+(const vector3d &vec)
{
    return vector3d(x+vec.x,y+vec.y,z+vec.z);
}

vector3d &vector3d ::operator+=(const vector3d &vec)
{
    x+=vec.x;
    y+=vec.y;
    z+=vec.z;
    return *this;
}
//substraction//
vector3d vector3d ::operator-(const vector3d &vec)
{
    return vector3d(x-vec.x,y-vec.y,z-vec.z);
}

vector3d &vector3d::operator-=(const vector3d &vec)
{
    x-=vec.x;
    y-=vec.y;
    z-=vec.z;
    return *this;
}

//scalar multiplication

vector3d vector3d ::operator*(f value)
{
    return vector3d(x*value,y*value,z*value);
}



vector3d &vector3d::operator*=(f value)
{
    x*=value;
    y*=value;
    z*=value;
    return *this;
}

//scalar division
vector3d vector3d ::operator/(f value)
{
    assert(value!=0);
    return vector3d(x/value,y/value,z/value);
}

vector3d &vector3d ::operator/=(f value)
{
    assert(value!=0);
    x/=value;
    y/=value;
    z/=value;
    return *this;
}


vector3d &vector3d::operator=(const vector3d &vec)
{
    x=vec.x;
    y=vec.y;
    z=vec.z;
    return *this;
}

//Dot product
f vector3d::dot_product(const vector3d &vec)
{
    return x*vec.x+vec.y*y+vec.z*z;
}

//cross product
vector3d vector3d ::cross_product(const vector3d &vec)
{
    f ni=y*vec.z-z*vec.y;
    f nj=z*vec.x-x*vec.z;
    f nk=x*vec.y-y*vec.x;
    return vector3d(ni,nj,nk);
}

f vector3d::magnitude()
{
    return sqrt(square());
}

f vector3d::square()
{
    return x*x+y*y+z*z;
}

vector3d vector3d:: normalization()
{
    assert(magnitude()!=0);
    *this/=magnitude();
    return *this;
}

f vector3d::distance(const vector3d &vec)
{
    vector3d dist=*this-vec;
    return dist.magnitude();
}

f vector3d::show_X()
{
    return x;
}

f vector3d::show_Y()
{
    return y;
}

f vector3d::show_Z()
{
    return z;
}

void vector3d::disp()
{
    cout<<x<<" "<<y<<" "<<z<<endl;
}

#endif /* VECTOR3D_HPP_ */