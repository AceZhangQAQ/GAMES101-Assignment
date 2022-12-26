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
    //在光源中随机选择一个面（点）进行采样
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            //找到需要采样的发光的面
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
    //判断光线与场景是否相交
    Intersection insect = Scene::intersect(ray);
    if(!insect.happened)
        return Vector3f(0.0,0.0,0.0);

    //如果和物体相交
    Object *hitObject = insect.obj;

        //如果和光源直接相交
        if(insect.m->hasEmission()){
            if(depth == 0)
                //直接返回直接光照
                return insect.m->getEmission();
            return Vector3f(0.0,0.0,0.0);
        }
            

        //和普通物体相交
        Vector3f L_dir = Vector3f(0.0,0.0,0.0);     //光源
        Vector3f L_indir = Vector3f(0.0,0.0,0.0);   //间接光

            //先对光源直接采样
                Intersection insect_Light;  //和光源的交点
                float pdf_light = 0.0f;    //对光源采样的pdf
                sampleLight(insect_Light,pdf_light);
                if(pdf_light < EPSILON2)
                    pdf_light += EPSILON2;
                Vector3f dir = normalize(insect_Light.coords - insect.coords);
                //判断光线是否被物体遮挡
                Ray test_ray(insect.coords,dir);
                Intersection insect_object = Scene::intersect(test_ray);
                float object_to_light = 0;
                float object_to_object = 0;
                object_to_light = (insect_Light.coords - insect.coords).norm();
                object_to_object = (insect_object.coords - insect.coords).norm();
                if(insect_object.happened && object_to_light - object_to_object <= EPSILON)
                    //计算光源采样结果
                    L_dir = insect_Light.emit 
                            * insect.m->eval(ray.direction,dir,insect.normal) 
                            * dotProduct(dir,insect.normal)
                            * dotProduct(-dir,insect_Light.normal)
                            / std::pow(object_to_light,2)
                            / pdf_light;

            //再计算间接光照
                //俄罗斯轮盘赌
                float ksi = get_random_float();
                if(ksi < RussianRoulette){
                    //随机选择一个方向继续进行光线弹射
                    Vector3f to_obj_dir = insect.m->sample(ray.direction,insect.normal);
                    Ray to_obj_ray(insect.coords,normalize(to_obj_dir));
                    Intersection object_insect = Scene::intersect(to_obj_ray);
                    if(object_insect.happened){
                        //打到不发光的物体
                        float pdf_hemi = insect.m->pdf(ray.direction,to_obj_ray.direction,insect.normal);
                        if(pdf_hemi < EPSILON2)
                            pdf_hemi += EPSILON2;
                        if(!object_insect.obj->hasEmit())
                            L_indir = castRay(Ray(insect.coords,to_obj_dir),depth+1) 
                                      * insect.m->eval(ray.direction,to_obj_ray.direction,insect.normal)
                                      * dotProduct(to_obj_ray.direction,insect.normal)
                                      / pdf_hemi
                                      / RussianRoulette;
                    }
                }
        //返回直接和间接光照的和
        return L_dir + L_indir;

}