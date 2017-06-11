#include "Vector2d.h"
#include <math.h>
#include <iostream>

Vector2d::Vector2d(double newX, double newY)
    :
      x(newX),
      y(newY)
{
}

Vector2d::Vector2d(geometry_msgs::Pose const& pos)
    :
      x(pos.position.x),
      y(pos.position.y)
{
}

Vector2d::Vector2d(const Vector2d& vec)
    :
      x(vec.x),
      y(vec.y)
{
}

Vector2d::Vector2d()
    :
      x(0.),
      y(0.)
{
}

Vector2d::Vector2d(double angle)
    :
      x(cos(angle)),
      y(sin(angle))
{
}


double Vector2d::getLength() const
{
    return sqrt(x * x + y * y);
}

double Vector2d::getLengthSquared() const
{
    return x * x + y * y;
}

double Vector2d::getDistance(const Vector2d& vec) const
{
    return (*this - vec).getLength();
}

double Vector2d::getDistance(const geometry_msgs::Pose& pos) const
{
    return (*this - Vector2d(pos)).getLength();
}

double Vector2d::getAngle(const Vector2d& vec) const
{
    double angleDiff = getAngle() - vec.getAngle();
    if (angleDiff > M_PI)
    {
        angleDiff = angleDiff - 2 * M_PI;
    }
    else if (angleDiff < - M_PI)
    {
        angleDiff = angleDiff + 2 * M_PI;
    }
    return angleDiff;
}

double Vector2d::getAngle() const
{
    return atan2(y,x);
}

void Vector2d::normalize()
{
    double length = getLength();
    if (length == 0)
    {
        x = sqrt(2);
        y = sqrt(2);
        return;
    }
    x /= length;
    y /= length;
}

void Vector2d::turn(double rad)
{
    double oldX = x;
    double oldY = y;
    double c = cos(rad);
    double s = sin(rad);

    x = c * oldX - s * oldY;
    y = s * oldX + c * oldY;
}

Vector2d Vector2d::getNormalized() const
{
    Vector2d temp(*this);
    temp.normalize();
    return temp;
}

Vector2d Vector2d::getTurned(double rad) const
{
    Vector2d temp(*this);
    temp.turn(rad);
    return temp;
}

geometry_msgs::Pose Vector2d::toGeometryPose() const
{
    geometry_msgs::Pose pose;
    pose.position.x = this->x;
    pose.position.y = this->y;
    return pose;
}


Vector2d Vector2d::operator+(Vector2d const& rhs) const
{
    return Vector2d(x + rhs.x, y + rhs.y);
}

Vector2d Vector2d::operator-(Vector2d const& rhs) const
{
    return Vector2d(x - rhs.x, y - rhs.y);
}

Vector2d Vector2d::operator*(double times) const
{
    return Vector2d(x * times, y * times);
}

Vector2d Vector2d::operator/(double divide) const
{
    return Vector2d(x / divide, y / divide);
}

double Vector2d::operator*(Vector2d const& rhs) const
{
    return x * rhs.x + y * rhs.y;
}

std::ostream& operator<<(std::ostream& os, const Vector2d& vec)
{
      os << "(" << vec.x << ", " << vec.y << ")";
      return os;
}

std::ostream& operator<<(std::ostream& os, const std::vector<Vector2d>& vec)
{
    os << "[" << std::endl;
    for (auto k : vec)
    {
        os << "   (" << k.x << ", " << k.y << ")" << std::endl;
    }
    os << "]" << std::endl;
    return os;
}

//Foreign overloading

Vector2d operator-(const geometry_msgs::Pose& pos1, const geometry_msgs::Pose& pos2)
{
    return Vector2d(pos1) - Vector2d(pos2);
}


