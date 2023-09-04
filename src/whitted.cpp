#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class WhittedIntegrator : public Integrator
{
public:
    WhittedIntegrator(const PropertyList& props)
    {
    }
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const
    {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
        {
            return Color3f(0.0f);
        }
        else{
            Color3f L_i(0.0f);
            Color3f L_e(0.0f);
            Color3f L_r(0.0f);
            Color3f F_r(0.0f);

            if(!its.mesh->getBSDF()->isDiffuse()) {
                if(sampler->next1D() > 0.95)
                    return Color3f(0.f);
                else {
                    BSDFQueryRecord bsdfQ = BSDFQueryRecord(its.toLocal(-ray.d));
                    its.mesh->getBSDF()->sample(bsdfQ, sampler->next2D());
                    return bsdfQ.eta * Li(scene, sampler, Ray3f(its.p, its.toWorld(bsdfQ.wo)));
                }
            }


            if (its.mesh->isEmitter()){
                EmitterQueryRecord eRec(ray.o, its.p, its.shFrame.n);
                L_e = its.mesh->getEmitter()->eval(eRec);
            }
            BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d));
            F_r = its.mesh->getBSDF()->sample(bRec, sampler->next2D());

            Ray3f rayR = Ray3f(its.p, its.shFrame.toWorld(bRec.wo));
            Intersection itsR;
            if (scene->rayIntersect(rayR, itsR))
            {
                if (itsR.mesh->isEmitter())
                {
                    EmitterQueryRecord lRec(its.p, itsR.p, itsR.shFrame.n);
                    L_i = itsR.mesh->getEmitter()->eval(lRec);
                }
            }
            return L_e + L_i * F_r;
        }
        
    }
    /// Return a human-readable description for debugging purposes
    std::string toString() const
    {
        return tfm::format(
            "WhittedIntegrator[]"
        );
    }
protected:
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END