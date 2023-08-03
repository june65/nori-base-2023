
#include <nori/octree.h>
#include <nori/mesh.h>
#include <algorithm>

#define MAXDEPTH 15
#define CHILDNUM 10

NORI_NAMESPACE_BEGIN

void OctNode::buildChildren(const Mesh *mesh){
    if(depth > 15)
        return;
    Point3f center = bbox->getCenter();
    children = new std::vector<OctNode*>();
    for (int i = 0; i < 8; i++){
        children -> push_back(new OctNode(center, bbox->getCorner(i), depth + 1));
    }
    for(auto child:*children){
        for (std::vector<int>::size_type j = 0; j < indices->size();j++){
            if(child->bbox->overlaps(mesh->getBoundingBox(indices->at(j)))){
                child->indices->push_back(indices->at(j));
            }
        }
    }
    for (std::vector<OctNode *>::size_type i = 0; i < 8; i++){
        if(children -> back()->indices->size()==0){
            delete children->back();
            children->pop_back();
        }
    }
    delete indices;
    indices = nullptr;
    for(auto child:*children) {
        if(child->indices->size() > CHILDNUM) {
            child->buildChildren(mesh);
        }
    }
}



void Octree::buildOctree(const Mesh *mesh) {
    this->mesh = mesh;
    root = new OctNode(mesh->getBoundingBox().min,mesh->getBoundingBox().max, 0);
    for(int i=0; i<mesh->getTriangleCount();i++) {
        root->getIndicesPtr()->push_back(i);
    }
    root->buildChildren(mesh);
}

void OctNode::search(const Ray3f &ray, const Mesh* mesh, int &index, float &distance) {
    
    if(children == nullptr) {
        for(std::vector<int>::size_type j=0;j<indices->size();j++) {
            float u,v;
            float t = distance;
            mesh->rayIntersect(indices->at(j), ray, u, v, t);
            if(t < distance) {
                index = indices->at(j);
                distance = t;
            }
        }
        return;
    }

    for(auto child:*children) {
        if(child->bbox->rayIntersect(ray)) {
            child->search(ray, mesh, index, distance);
        }
    }
}

void Octree::search(const Ray3f &ray, int &index) const {
    float distance = FLT_MAX ;
    index = -1;

    root->search(ray, mesh, index, distance);
}



NORI_NAMESPACE_END
