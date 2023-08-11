#include <nori/integrator.h>
#include <nori/scene.h>
#include <iostream>
#include <cmath>
using namespace std;

NORI_NAMESPACE_BEGIN

class Simple : public Integrator {
public:
    Simple(const PropertyList &props) {
        m_position = props.getPoint("position");
        m_energy = props.getColor("energy");
    }
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);
        else{
            Point3f distance = m_position - its.p;
            float distance_size = distance.dot(distance);
            distance = distance.normalized();
            float cosine = std::max(0.0f , its.shFrame.n.dot(distance));
            Color3f L  = m_energy * cosine / M_PI / M_PI / 4 / distance_size;
            //std::cout << L << std::endl;

            Ray3f shadowRay = Ray3f(its.p, distance);
            
            if(scene->rayIntersect(shadowRay)) {
                return Color3f(0.0f);
            }

            return L;
            
        }
        
    }   

    std::string toString() const {
        return "NormalIntegrator[]";
    }

private:
    Point3f m_position;
    Color3f m_energy;
};


NORI_REGISTER_CLASS(Simple, "simple");
NORI_NAMESPACE_END