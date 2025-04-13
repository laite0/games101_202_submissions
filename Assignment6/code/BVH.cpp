#include <algorithm>
#include <cassert>
#include "BVH.hpp"

const bool USE_SAH = false;

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

Bounds3 unionBoundsL(std::vector<Object*>& objs) {
    Bounds3 bounds;
    for (int i = 0; i < objs.size(); ++i)
        bounds = unionBounds(bounds, objs[i]->getBounds());
    return bounds;
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = unionBounds(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    } else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = unionBounds(node->left->bounds, node->right->bounds);
        return node;
    } else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                expand(centroidBounds, objects[i]->getBounds().Centroid());
        Bounds3::MaxExtent dim = centroidBounds.maxExtent();
        switch (dim) {
        case Bounds3::MaxExtent::X:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case Bounds3::MaxExtent::Y:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case Bounds3::MaxExtent::Z:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto ending = objects.end();
        std::vector<Object*>::iterator middling;
        if (USE_SAH) {
            double parentSurface = bounds.surfaceArea();
            int min_cut_id = 0;
            double min_cut_cost = std::numeric_limits<double>::max();
            size_t n = objects.size();
            for (int i = 0; i < n; i++) {
                auto l = std::vector<Object*>(beginning, beginning + i);
                auto r = std::vector<Object*>(beginning + i, ending);
                double cur_cost = unionBoundsL(l).surfaceArea() / parentSurface * l.size() + unionBoundsL(r).surfaceArea() / parentSurface * r.size() + 0.1;
                if (cur_cost < min_cut_cost) {
                    min_cut_id = i;
                    min_cut_cost = cur_cost;
                }
            }
            middling = beginning + min_cut_id;
        } else {
            middling = beginning + (objects.size() / 2);
        }
        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = unionBounds(node->left->bounds, node->right->bounds);
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
    Intersection isect;

    if (!node) {
        return isect;
    }

    if (!node->bounds.IntersectP(ray)) {
        return isect;
    }

    auto objs = node->object;

    if (objs) {
        return objs->getIntersection(ray);
    }

    Intersection lisect = getIntersection(node->left, ray);
    Intersection risect = getIntersection(node->right, ray);

    if (lisect.distance < risect.distance) {
        return lisect;
    }

    return risect;
}