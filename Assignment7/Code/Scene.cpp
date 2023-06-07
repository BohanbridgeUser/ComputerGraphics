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
    // 
    Intersection p_inter = intersect(ray);
    if (!p_inter.happened) {
        return Vector3f();
    }
    if (p_inter.m->hasEmission()) {
        return p_inter.m->getEmission();
    }

    float EPLISON = 0.0001;
    Vector3f l_dir;
    Vector3f l_indir;

    // sampleLight(inter, pdf_light)
    Intersection x_inter;
    float pdf_light = 0.0f;
    sampleLight(x_inter, pdf_light);

    // Get x, ws, NN, emit from inter
    Vector3f p = p_inter.coords;
    Vector3f x = x_inter.coords;
    Vector3f ws_dir = (x - p).normalized();
    float ws_distance = (x - p).norm();
    Vector3f N = p_inter.normal.normalized();
    Vector3f NN = x_inter.normal.normalized();
    Vector3f emit = x_inter.emit;

    // Shoot a ray from p to x
    Ray ws_ray(p, ws_dir);
    Intersection ws_ray_inter = intersect(ws_ray);
    // If the ray is not blocked in the middle
    if (ws_ray_inter.distance - ws_distance > -EPLISON) {
        l_dir = emit * p_inter.m->eval(ray.direction, ws_ray.direction, N)
            * dotProduct(ws_ray.direction, N)
            * dotProduct(-ws_ray.direction, NN)
            / std::pow(ws_distance, 2)
            / pdf_light;
    }

    // Test Russian Roulette with probability RussianRoulette
    if (get_random_float() > RussianRoulette) {
        return l_dir;
    }

    l_indir = 0.0;

    Vector3f wi_dir = p_inter.m->sample(ray.direction, N).normalized();
    Ray wi_ray(p_inter.coords, wi_dir);
    // If ray r hit a non-emitting object at q
    Intersection wi_inter = intersect(wi_ray);
    if (wi_inter.happened && (!wi_inter.m->hasEmission())) {
        l_indir = castRay(wi_ray, depth + 1) * p_inter.m->eval(ray.direction, wi_ray.direction, N)
            * dotProduct(wi_ray.direction, N)
            / p_inter.m->pdf(ray.direction, wi_ray.direction, N)
            / RussianRoulette;
    }

    return l_dir + l_indir;

    //Vector3f hitcolor(0.0f);
    //Intersection hitIntersection = intersect(ray);
    //if (hitIntersection.happened) {
    //    Vector3f hitcolor_dir(0.0f), hitcolor_indir(0.0f);

    //    if (hitIntersection.m->hasEmission()) 
    //    {
    //          return hitIntersection.m->getEmission();
    //    }else{
    //          return Vector3f(0.0f);
    //    }
    //    
    //    // Direct Light
    //    Intersection sample_light_point;
    //    float pdf = 0.0f;
    //    sampleLight(sample_light_point, pdf);
    // 
    //    Vector3f P_origin = hitIntersection.coords;
    //    Vector3f P_L_dir1 = (sample_light_point.coords - P_origin);
    //    Vector3f P_L_dir = (sample_light_point.coords - P_origin).normalized();
    //    Intersection Is_light_hit = intersect(Ray(P_origin, P_L_dir)); 
    //    Vector3f wo = ray.direction, N = hitIntersection.normal;
    //    float dis = P_L_dir1.x * P_L_dir1.x + P_L_dir1.y * P_L_dir1.y + P_L_dir1.z * P_L_dir1.z;
    //    if (!Is_light_hit.happened && ((Is_light_hit.coords - sample_light_point.coords).norm() < 0.01)) {
    //        Vector3f e = sample_light_point.emit, ws = P_L_dir, NN = Is_light_hit.normal,
    //        hitcolor_dir = e * hitIntersection.m->eval(wo, ws, N)
    //            * dotProduct(P_L_dir, N) * dotProduct(-ws, NN) / dis / pdf;
    //    }

    //    // Indirect Light
    //    if (get_random_float() < RussianRoulette) {
    //        Vector3f sphere_wi_sample = hitIntersection.m->sample(wo,N);
    //        Ray wi(P_origin, sphere_wi_sample);
    //        Intersection Is_Object_hit = intersect(wi);
    //        if (Is_Object_hit.happened && !Is_Object_hit.m->hasEmission()) {
    //            hitcolor_indir = castRay(wi, depth+1) * hitIntersection.m->eval(wo, sphere_wi_sample, N)
    //                * dotProduct(sphere_wi_sample, N) / hitIntersection.m->pdf(wo, sphere_wi_sample, N) / RussianRoulette;
    //        }
    //    }
    //    hitcolor = hitcolor_dir + hitcolor_dir;
    //}
    //return hitcolor;
}