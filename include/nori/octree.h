#pragma once

#include <nori/bbox.h>
#include <vector>

NORI_NAMESPACE_BEGIN

class OctNode {
public:
    OctNode(Point3f a, Point3f b, int depth) {
        Point3f min = Point3f(std::min(a.x(),b.x()),
                              std::min(a.y(),b.y()),
                              std::min(a.z(),b.z()));
        Point3f max = Point3f(std::max(a.x(),b.x()),
                              std::max(a.y(),b.y()),
                              std::max(a.z(),b.z()));
                              this->bbox = new BoundingBox3f(min,max);
        this->indices = new std::vector<int>();
        this->depth = depth;
        this->children = nullptr;
    }
    ~OctNode() {
        delete children;
        delete indices;
        delete bbox;
    }
    void buildChildren(const Mesh *mesh);
    void search(const Ray3f &ray, const Mesh* mesh, int &index, float &distance);
    BoundingBox3f* getBoundBoxPtr() {   return bbox;    }
    std::vector<OctNode*>* getChildern() {    return children;    }
    std::vector<int>* getIndicesPtr() {     return indices;     }
    void sortChildrenByRay(const Ray3f &ray);
    
private:
    std::vector<OctNode*>* children;
    BoundingBox3f* bbox;
    std::vector<int>* indices;
    int depth;
};

class Octree {
public:
    void buildOctree(const Mesh *m);
    void search(const Ray3f &ray, int &index) const;
private:
    OctNode* root;
    const Mesh *mesh;
};

NORI_NAMESPACE_END