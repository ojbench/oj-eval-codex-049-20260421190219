#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP
#include "math.h"
#include <algorithm>
#include <cmath>

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    // Helper: rotate a unit direction by angle
    static Vec rotate_dir(const Vec &u, double ang) {
        double c = std::cos(ang), s = std::sin(ang);
        return Vec(u.x * c - u.y * s, u.x * s + u.y * c);
    }

    // Check if choosing my_v would trigger a collision with robot j according to simulator logic
    bool will_collide_with(int j, const Vec &my_v) const {
        Vec pj = monitor->get_pos_cur(j);
        Vec vj = monitor->get_v_cur(j);
        double rj = monitor->get_r(j);
        Vec delta_pos = pos_cur - pj;
        Vec delta_v = my_v - vj;

        double proj = delta_pos.dot(delta_v);
        if (proj >= 0) {
            return false; // moving apart in relative frame
        }
        double dv_norm = std::sqrt(std::max(delta_v.x * delta_v.x + delta_v.y * delta_v.y, 0.0));
        // Mirror simulator's computation
        double project = proj;
        if (dv_norm > 1e-12) {
            project /= -dv_norm;
        } else {
            // Relative velocity ~ 0, use a very large project to jump to the else-branch below
            project = 1e100;
        }
        double min_dis_sqr;
        double delta_r = r + rj;
        if (project < dv_norm * TIME_INTERVAL) {
            min_dis_sqr = std::max(delta_pos.x * delta_pos.x + delta_pos.y * delta_pos.y, 1e-9) - project * project;
        } else {
            Vec end = delta_pos + delta_v * TIME_INTERVAL;
            min_dis_sqr = std::max(end.x * end.x + end.y * end.y, 1e-9);
        }
        return min_dis_sqr <= delta_r * delta_r - EPSILON;
    }

    bool velocity_safe(const Vec &my_v) const {
        int n = monitor->get_robot_number();
        for (int j = 0; j < n; ++j) if (j != id) {
            if (will_collide_with(j, my_v)) return false;
        }
        // Also ensure not speeding per simulator rule
        double sp2 = my_v.x * my_v.x + my_v.y * my_v.y;
        return sp2 < v_max * v_max + EPSILON; // strictly below threshold used by simulator
    }

public:

    Vec get_v_next() {
        // Basic goal direction
        Vec to_tar = pos_tar - pos_cur;
        double dist = std::sqrt(std::max(to_tar.x * to_tar.x + to_tar.y * to_tar.y, 0.0));
        if (!(dist > EPSILON) || !(v_max > 0)) {
            return Vec();
        }
        Vec dir = Vec(to_tar.x / std::max(dist, 1e-9), to_tar.y / std::max(dist, 1e-9));

        // Desired speed with gentle braking near target
        double desired = std::min(v_max * 0.999, dist / TIME_INTERVAL);

        // Candidate search over angles and speed scales
        const double base_step = 15.0 * PI / 180.0; // 15 degrees
        double angle_seq[9] = {0, 1, -1, 2, -2, 3, -3, 4, -4};
        double speed_mults[6] = {1.0, 0.8, 0.6, 0.4, 0.2, 0.0};

        Vec best_safe(0, 0);
        bool found = false;
        for (double ak : angle_seq) {
            Vec cand_dir = rotate_dir(dir, ak * base_step);
            for (double sm : speed_mults) {
                double sp = std::max(0.0, std::min(desired, v_max * 0.999)) * sm;
                Vec v_try = cand_dir * sp;
                if (velocity_safe(v_try)) {
                    best_safe = v_try;
                    found = true;
                    break;
                }
            }
            if (found) break;
        }

        if (found) return best_safe;

        // Fallback: try small sideways slip to avoid deadlocks
        Vec side(-dir.y, dir.x);
        Vec v_try = side * std::min(v_max * 0.2, dist / TIME_INTERVAL);
        if (velocity_safe(v_try)) return v_try;

        // Finally, stop if nothing safe found
        return Vec();
    }
};

#endif //PPCA_SRC_HPP
