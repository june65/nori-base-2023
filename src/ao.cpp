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
        
            Point3f  x = its.p;
            Color3f L(0.0f);
            int SAMPLE_NUM = 16;
            for (int i = 0; i < SAMPLE_NUM; i++)
            {
                Vector3f dir = Warp::squareToCosineHemisphere(sampler->next2D());
                float pdf = Warp::squareToCosineHemispherePdf(dir);
                Vector3f w = its.shFrame.toWorld(dir).normalized();
                float alpha = 10000.f;
                Ray3f shadowRay = Ray3f(x + w/alpha, w*alpha);
                float cosine = its.shFrame.n.dot(w);
                if(!scene->rayIntersect(shadowRay)) {
                    L += cosine / M_PI / pdf;
                }
                //Vector3f reflection = dir - 2 * dir.dot(its.shFrame.n) * its.shFrame.n;
                Vector3f reflection = - dir + 2 * dir.dot(its.shFrame.n) * its.shFrame.n;
                Ray3f shadowRay2 = Ray3f(x + reflection/alpha, reflection*alpha);
                if(!scene->rayIntersect(shadowRay2)) {
                    L += cosine / M_PI / pdf;
                }
                
            }
            L /= float(SAMPLE_NUM)*2;
            return L;
    }   

    std::string toString() const {
        return "NormalIntegrator[]";
    }

private:
};


NORI_REGISTER_CLASS(Ao, "ao");
NORI_NAMESPACE_END