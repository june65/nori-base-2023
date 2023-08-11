/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/warp.h>
#include <nori/vector.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

Point2f Warp::squareToUniformSquare(const Point2f &sample) {
    return sample;
}

float Warp::squareToUniformSquarePdf(const Point2f &sample) {
    return ((sample.array() >= 0).all() && (sample.array() <= 1).all()) ? 1.0f : 0.0f;
}

Point2f Warp::squareToTent(const Point2f &sample) {
    Point2f new_sample;
    for (int i = 0; i < 2; i++)
    {
        if(sample[i] < 0.5f)
            new_sample[i] = sqrt(2*sample[i]) - 1;
        else
            new_sample[i] = 1 - sqrt(2-2*sample[i]);
    }
    return new_sample;
    //throw NoriException("Warp::squareToTent() is not yet implemented!");
}

float Warp::squareToTentPdf(const Point2f &p) {
    float absolute_value[2];
    for(int i=0;i<2;i++) {
        if(p[i] <= 1 && p[i] >= -1)
            absolute_value[i] = p[i];
        else
            absolute_value[i] = 0;
    }
    return (1-abs(absolute_value[0])) * (1-abs(absolute_value[1]));
    //throw NoriException("Warp::squareToTentPdf() is not yet implemented!");
}

Point2f Warp::squareToUniformDisk(const Point2f &sample) {
    float rad = sqrt(sample[0]);
    float angle = 2 * sample[1] * M_PI;
    return  Point2f(rad * cos(angle),rad * sin(angle));
    //throw NoriException("Warp::squareToUniformDisk() is not yet implemented!");
}
float Warp::squareToUniformDiskPdf(const Point2f &p) {
    
    if(sqrt(p[0] * p[0] + p[1] * p[1]) > 1) 
        return 0.0f;
    else
        return INV_PI;
    //throw NoriException("Warp::squareToUniformDiskPdf() is not yet implemented!");
}

Vector3f Warp::squareToUniformSphere(const Point2f &sample) {
    
    float angle_1 = 2 * sample[0] * M_PI;
    float angle_2 = acos(1-2*sample[1]);
    return  Vector3f(sin(angle_2)* cos(angle_1),sin(angle_2 )* sin(angle_1), cos(angle_2));
    //throw NoriException("Warp::squareToUniformSphere() is not yet implemented!");
}

float Warp::squareToUniformSpherePdf(const Vector3f &v) {
    if(sqrt(v[0] * v[0] + v[1] * v[1]+ v[2] * v[2]) > 1) 
        return 0.0f;
    else
        return INV_PI/4;
}

Vector3f Warp::squareToUniformHemisphere(const Point2f &sample) {
    
    float angle_1 = 2*sample[0] * M_PI;
    float angle_2 = acos(1-sample[1]);
    return  Vector3f(sin(angle_2)* cos(angle_1),sin(angle_2 )* sin(angle_1), cos(angle_2));
    //throw NoriException("Warp::squareToUniformHemisphere() is not yet implemented!");
}

float Warp::squareToUniformHemispherePdf(const Vector3f &v) {
    if(v[2]<0) 
        return 0.0f;
    else
        return INV_PI/2;
    //throw NoriException("Warp::squareToUniformHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToCosineHemisphere(const Point2f &sample) {
    
    Point2f d = squareToUniformDisk(sample);
    float z = sqrt(std::max(0.f, 1 - d.x() * d.x() - d.y() * d.y()));
    return Vector3f(d.x(),d.y(),z);
    //throw NoriException("Warp::squareToCosineHemisphere() is not yet implemented!");
}

float Warp::squareToCosineHemispherePdf(const Vector3f &v) {
    
    if(v[2]<0) 
        return 0.0f;
    else
        return INV_PI * v[2];
    //throw NoriException("Warp::squareToCosineHemispherePdf() is not yet implemented!");
}

Vector3f Warp::squareToBeckmann(const Point2f &sample, float alpha) {
    float angle_1 = 2*sample[0] * M_PI;
    float angle_2 = atan(sqrt(-pow(alpha, 2) * log(1 - sample[1])));

    return  Vector3f(sin(angle_2)* cos(angle_1),sin(angle_2 )* sin(angle_1), cos(angle_2));
    //throw NoriException("Warp::squareToBeckmann() is not yet implemented!");
}

float Warp::squareToBeckmannPdf(const Vector3f &m, float alpha) {
    float cosTheta = Frame::cosTheta(m);
    float tanTheta = Frame::tanTheta(m);
    
    if(cosTheta <= 0)
        return 0.f;
     
    return (INV_PI/2) * 2 * exp(-pow(tanTheta,2)/pow(alpha,2)) / (pow(alpha,2)*pow(cosTheta,3));
    //throw NoriException("Warp::squareToBeckmannPdf() is not yet implemented!");
}

NORI_NAMESPACE_END
