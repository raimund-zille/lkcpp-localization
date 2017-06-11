#include "../inc/vector2d.h"
#include <cmath>
#include <cassert>

Vector2D::Vector2D():xdir(0.),ydir(0.){}

Vector2D::Vector2D(double x, double y):xdir(x),ydir(y){}

Vector2D::Vector2D(const Vector2D &vec){
    this->xdir = vec.xdir;
    this->ydir = vec.ydir;
}

double Vector2D::dot(const Vector2D &v) const {
    return xdir*v.xdir+ydir*v.ydir;
}

Vector2D::Vector2D(double angle, Angle a){

    //assert(("Angle type unkown, make sure to use RAD or DEG",a < 0 || a > RAD));

    switch(a){
    case RAD:
        xdir = cos(angle);
        ydir = sin(angle);
        break;

    case DEG:
        xdir = cos(angle*M_PI / 180.);
        ydir = sin(angle*M_PI / 180.);
        break;

    default:
        assert("Angle type unkown, make sure to use RAD or DEG");
        break;
    }

}

double Vector2D::cross(const Vector2D &v){
    return xdir*v.ydir-ydir*v.xdir;
}


double Vector2D::magnitute(void) const{
    return sqrt(xdir*xdir+ydir*ydir);
}

double Vector2D::degree(const Vector2D &v) const{

    double scal  = this->dot(v);
    double mag = this->magnitute();
    return acos(scal/(mag*v.magnitute()))*180./PI;

}

double Vector2D::rad(const Vector2D &v) const{

    double scal  = this->dot(v);
    double mag = this->magnitute();
    return acos(scal/(mag*v.magnitute()));
}

Vector2D& Vector2D::normalize(void){

    float mag = this->magnitute();
    xdir /= mag;
    ydir /= mag;
    return *this;
}

double Vector2D::getX() const{
    return xdir;
}

double Vector2D::getY() const{
    return ydir;
}

Vector2D &Vector2D::rotate(double angle, Angle a){

    double ang = a == RAD ? angle : angle*M_PI / 180.;

    xdir = xdir*cos(ang) - sin(ang)*ydir;
    ydir = xdir*sin(ang) + ydir*cos(ang);

    return *this;
}

Vector2D Vector2D::orthogonal(){
    return Vector2D(-ydir,xdir);
}

bool Vector2D::isnan(){
    return xdir != xdir || ydir != ydir;
}

Vector2D& Vector2D::operator+=(const Vector2D &v){

    xdir += v.xdir;
    ydir += v.ydir;
    return *this;
}

Vector2D operator+(const Vector2D &v1, const Vector2D &v2){

    Vector2D result(v1);
    result += v2;
    return result;
}

Vector2D& Vector2D::operator-=(const Vector2D &v){

    xdir -= v.xdir;
    ydir -= v.ydir;
    return *this;
}

Vector2D operator-(const Vector2D &v1, const Vector2D &v2){

    Vector2D result(v1);
    result -= v2;
    return result;
}

Vector2D& Vector2D::operator*=(const double scalar){

    xdir *= scalar;
    ydir *= scalar;
    return *this;
}

Vector2D operator*(const double scalar, const Vector2D &v){

    Vector2D result(v);
    result *= scalar;
    return result;
}

Vector2D& Vector2D::operator/=(const double scalar){

    xdir /= scalar;
    ydir /= scalar;
    return *this;
}

Vector2D operator/(const double scalar, const Vector2D &v){

    Vector2D result(v);
    result /= scalar;
    return result;
}

Vector2D &Vector2D::operator=(const Vector2D &v){
    xdir = v.xdir;
    ydir = v.ydir;
    return *this;
}

bool Vector2D::operator==(Vector2D &v2){
    return (xdir == v2.getX()) && (ydir == v2.getY());
}

bool Vector2D::operator!=(Vector2D &v2){
    return !(*this == v2);
}

std::ostream& operator<<(std::ostream& os,const Vector2D &v){
    os <<"["<<v.xdir << " " << v.ydir << "]" << std::endl;
    return os;
}
