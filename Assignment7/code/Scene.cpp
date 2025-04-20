//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    if (depth > this->maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }
    Intersection intersection = Scene::intersect(ray);
    Vector3f hitColor = Vector3f(0.0,0.0,0.0);
    
    if(intersection.happened) {
        return shade(ray, intersection, depth);
    }

    return hitColor;
}

// keep commented out debug code and log here for forecoming ones to learn
int numSpL = 0;
int numAllSp = 0;

Vector3f Scene::shade(const Ray &ray, const Intersection& intersection, int depth) const {
    Vector3f hitPoint = intersection.coords;
    Vector3f N = intersection.normal; // normal
    Material *m = intersection.m;
    Object *hitObject = intersection.obj;
    numAllSp++;
    Vector3f outRadiance(0.0,0.0,0.0);/*!*/

    if (hitObject->hasEmit()) { // self is light 
        outRadiance += m->getEmission();
    } // self is light end

    auto wo = ray.direction;


    Intersection intersectionToLight; // sample to light logic
    float lightPdf = 0.0f;
    sampleLight(intersectionToLight, /*!*/lightPdf);
    Vector3f directLightnessRadiance(0.0,0.0,0.0);
    if (lightPdf != 0.0f) {
        Vector3f lightHitPos = intersectionToLight.coords;
        Vector3f lightHitMinusHit = lightHitPos - hitPoint;
        Vector3f hitToLightHitDir = lightHitMinusHit.normalized();
        Intersection checkDoesIntersectsLight = Scene::intersect(Ray(hitPoint, hitToLightHitDir));
        Object* mayLightObj = checkDoesIntersectsLight.obj;
        bool emits = mayLightObj != nullptr && mayLightObj->hasEmit();
        
        // if (numTryL % 50000 == 0) {
        //     std::cout << "Check hit light, lightHitPos=" << lightHitPos << ", lightHitMinusHit=" << lightHitMinusHit 
        //     << ", !!mayLightObj=" << (mayLightObj != nullptr) << ", emits=" << emits << std::endl;
        // }
        if (emits) {
            auto wi/*!*/ = hitToLightHitDir;
            auto xMinusP_2 = dotProduct(lightHitMinusHit, lightHitMinusHit);
            auto em = intersectionToLight.m->getEmission();
            auto evaled = m->eval(wi, -wo, N);

            directLightnessRadiance += em * evaled * dotProduct(wi, N) * dotProduct(-wi, intersectionToLight.normal) / xMinusP_2 / lightPdf;
            // if (numSpL % 50000 == 0) {
            //     std::cout << "Hit light, d^2=" << xMinusP_2 << ", pdf=" << lightPdf 
            //     << ", emission=" << em << ", evaled=" << evaled << ", rad=" << directLightnessRadiance << std::endl;
            // }
            numSpL++;
        }
    } // sample to light logic end



    Vector3f indirectLightnessRadiance(0.0,0.0,0.0);
    if (get_random_float() < RussianRoulette) {
        auto wi = m->sample(wo, N);
        indirectLightnessRadiance += castRay(Ray(hitPoint, wi), depth + 1) * m->eval(wi, -wo, N) * dotProduct(wi, N) * invRussianRoulette / m->pdf(wo, wi, N);
    }
    // if (numAllSp % 100000 == 0) {
    //     std::cout << "All sample stats, numSpL=" << numSpL << ", numAllSp=" << numAllSp << ", l=" << outRadiance << ", d=" << directLightnessRadiance << ", i=" << indirectLightnessRadiance << std::endl;
    // }


    return outRadiance + directLightnessRadiance + indirectLightnessRadiance;
}