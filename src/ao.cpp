#include <nori/integrator.h>
#include <nori/scene.h>
#include <iostream>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Ao : public Integrator {
public:
    Ao(const PropertyList &props) {
    }
    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);
        else{
            Point3f  x = its.p;
            Color3f L(0.0f);
            int SAMPLE_NUM = 1;
            for (int i = 0; i < SAMPLE_NUM; i++)
            {
                Vector3f dir = Warp::squareToCosineHemisphere(sampler->next2D());
                float pdf = Warp::squareToCosineHemispherePdf(dir);
                Vector3f w = dir.normalized();
                float alpha = 1000.f;
                Ray3f shadowRay = Ray3f(x , x + w * alpha);
                if(!scene->rayIntersect(shadowRay)) {
                    float cosine = its.shFrame.n.dot(w);
                    L += cosine / M_PI / pdf;
                }
                
            }
            L /= float(SAMPLE_NUM);
            return L;
            
        }
        
    }   

    std::string toString() const {
        return "NormalIntegrator[]";
    }

private:
};


NORI_REGISTER_CLASS(Ao, "ao");
NORI_NAMESPACE_END