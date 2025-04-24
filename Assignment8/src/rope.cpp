#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

const double DAMPING_FACTOR = 0.000005;

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Mass* prev = new Mass(start, node_mass, false);
        this->masses.push_back(prev);
        for (int i = 1; i < num_nodes; i++) {
            Vector2D pos = (start * (num_nodes - i) + end * i) / num_nodes;
            std::cout << i << " " << prev << " - " << pos << std::endl;
            Mass* m = new Mass(pos, node_mass, false);
            this->masses.push_back(m);
            Spring* s = new Spring(prev, m, k);
            this->springs.push_back(s);
            prev = m;
        }

        std::cout << "Built!" << std::endl;

        //    Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
           masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            float k = s->k;
            Mass* a = s->m1;
            Mass* b = s->m2;
            Vector2D vecBmA = b->position - a->position;
            double a2bDist = vecBmA.norm();
            double l = s->rest_length;
            double lDiff = a2bDist - l;
            Vector2D f_b2a = - k * vecBmA / a2bDist * lDiff;
            a->forces -= f_b2a;
            b->forces += f_b2a;
            // std::cout << "a@" << a->position << "  b@" << b->position << "  bf=" << b->forces << std::endl;
        }
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                
                Vector2D a = (m->forces + gravity) / m->mass;
                std::cout << "a=" << a << std::endl;
                auto curV = m->velocity;
                m->velocity += a * delta_t;
                

                // TODO (Part 2): Add global damping
                Vector2D a_damp = -(m->velocity) * DAMPING_FACTOR;
                m->velocity += a_damp;

                m->position += curV * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            float k = s->k;
            Mass* a = s->m1;
            Mass* b = s->m2;
            Vector2D vecBmA = b->position - a->position;
            double a2bDist = vecBmA.norm();
            double l = s->rest_length;
            double lDiff = a2bDist - l;
            Vector2D f_b2a = - k * vecBmA / a2bDist * lDiff;
            a->forces -= f_b2a;
            b->forces += f_b2a;
            // std::cout << "a@" << a->position << "  b@" << b->position << "  bf=" << b->forces << std::endl;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                Vector2D a = (m->forces + gravity) / m->mass;
                Vector2D pos = temp_position + (1 - DAMPING_FACTOR) * (temp_position - m->last_position) + a * delta_t * delta_t;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->position = pos;
                m->last_position = temp_position;
                // TODO (Part 4): Add global Verlet damping
            }
            m->forces = Vector2D(0, 0);
        }
    }
}
