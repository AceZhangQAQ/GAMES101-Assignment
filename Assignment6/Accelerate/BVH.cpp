#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        //找到最大的包围盒
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        //找到该包围盒的最长轴
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        //当最长轴为x轴
        case 0:
            //沿着x轴，根据每个元素的包围盒的中心点x值，由小到大进行排序
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        //当最长轴为y轴
        case 1:
            //沿着y轴，根据每个元素的包围盒的中心点y值，由小到大进行排序
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        //当最长轴为z轴
        case 2:
            //沿着z轴，根据每个元素的包围盒的中心点z值，由小到大进行排序
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        //将左右元素根据排序结果划分为两半
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);//左边的元素
        auto rightshapes = std::vector<Object*>(middling, ending);//右边的元素

        //断言左边+右边元素大小是否等于所有元素和
        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        //对左边和右边元素递归建立BVH树
        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        //根据递归建立的结果，更新当前节点（左右元素的父节点）的包围盒状态
        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    //初始化Intersection
    Intersection inter;
    //判断光线是否与当前包围盒相交
    std::array<int,3> dirIsNeg =  {ray.direction.x>0, ray.direction.y>0, ray.direction.z>0};
    Vector3f invDir = Vector3f(1.0/ray.direction.x, 1.0/ray.direction.y,1.0/ray.direction.z);
    inter.happened = node->bounds.IntersectP(ray,invDir,dirIsNeg);
    //与当前节点的包围盒相交
    if(inter.happened){
        //如果没有左右孩子节点（说明此节点是叶子节点）则直接包围盒内物体与光线的相交情况
        if(!(node->left && node->right)){
            inter = node->object->getIntersection(ray);
            return inter;
        }
        //如果有左右孩子
        Intersection left_inter = getIntersection(node->left,ray);
        Intersection right_inter = getIntersection(node->right,ray);
        //左右孩子都相交了，则返回距离最近的那个
        if(left_inter.happened && left_inter.happened)
            inter = left_inter.distance < right_inter.distance ? left_inter : right_inter;
        //左孩子相交了
        else if(left_inter.happened)
            inter = left_inter;
        //右孩子相交了
        else if(right_inter.happened)
            inter = right_inter;
        //都没有相交则修改当前节点相交情况为false
        else
            inter.happened = false;
    }
    //如果不相交则直接返回未相交的Intersection

    return inter;

}
