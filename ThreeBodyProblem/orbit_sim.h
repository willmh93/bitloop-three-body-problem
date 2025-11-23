#pragma once
#include <bitloop.h>

SIM_BEG;

using namespace bl;

enum StopResult
{
    UNSTABLE,
    UNDETERMINED,
    STABLE
};

template<class T>
struct Particle : public Vec2<T> 
{
    T vx, vy, ax, ay;
};

template<class T>
class SimPlot
{
    using Vec2 = Vec2<T>;

    std::vector<Vec2> a_path, b_path, c_path;
    f64 path_alpha = 0.08;

    void drawPath(Viewport* ctx, const std::vector<Vec2>& path, Color col, int cur_iter, double path_w, double trail_w) const;

public:

    static constexpr int stride = 1;

    void clear();
    void recordPositions(const Vec2& a, const Vec2& b, const Vec2& c);
    void draw(Viewport* ctx, int cur_iter = -1, double path_w=2.0, double trail_w=6.0) const;

    void setPathAlpha(f64 alpha) { path_alpha = alpha; }
};

template<class T>
struct SimEnv
{
    static constexpr int max_dist = 5; // max dist from origin to be considered unstable

    int escape_freq = 10; // how often we check for escape
    int max_iter;
    T max_vel, dt;
    T G{ 1 };
    T soft2{ T(0.0002) };

    // stability params
    T pos_tolerance;
    T vel_tolerance;

    SimEnv(T _G, T _vel, int _iters, T _dt)
    {
        G = _G;
        max_iter = _iters;
        max_vel = _vel;
        dt = _dt;

        pos_tolerance = T(0.02);
        vel_tolerance = max_vel / T(10);
    }
};

template<class T> 
struct StopPolicy_None
{
    void init(const SimEnv<T>*, const Particle<T>&, const Particle<T>&, const Particle<T>&) 
    {}

    StopResult stability(int iter, const Particle<T>&, const Particle<T>&, const Particle<T>&) const
    {
        return StopResult::UNDETERMINED;
    }
};

template<class T>
struct StopPolicy_MaxDist
{
    int max_iter;
    void init(const SimEnv<T>* env, const Particle<T>&, const Particle<T>&, const Particle<T>&) 
    {
        max_iter = env->max_iter;
    }

    StopResult stability(int iter, const Particle<T>& a, const Particle<T>& b, const Particle<T>& c) const
    {
        constexpr T max_mag2 = SimEnv<T>::max_dist * SimEnv<T>::max_dist;
        if (a.mag2() > max_mag2) return StopResult::UNSTABLE;
        if (b.mag2() > max_mag2) return StopResult::UNSTABLE;
        if (c.mag2() > max_mag2) return StopResult::UNSTABLE;
        if (iter >= max_iter) return StopResult::STABLE;
        return StopResult::UNDETERMINED;
    }
};

template<class T>
struct StopPolicy_Loopable
{
    SimEnv<T>* env;
    Particle<T> beg_a, beg_b, beg_c;

    int max_iter;
    void init(const SimEnv<T>* env, const Particle<T>& a, const Particle<T>& b, const Particle<T>& c)
    {
        max_iter = env->max_iter;
        beg_a = a;
        beg_b = b;
        beg_c = c;
    }

    bool similar(const Particle<T>& beg, const Particle<T>& now) const
    {
        constexpr T max_dist2 = bl::sq(0.02);
        //constexpr T max_vel2 = 0.03 * 0.03;

        const Vec2<T> delta_pos(now.x - beg.x, now.y - beg.y);
        if (delta_pos.mag2() > max_dist2) return false;


        //const Vec2<T> delta_vel(now.vx - beg.vx, now.vy - beg.vy);
        if (std::abs((now.vx / beg.vx) - 1) > 0.1) return false;
        if (std::abs((now.vy / beg.vy) - 1) > 0.1) return false;
        //if (delta_vel.mag2() > max_vel2) return false;

        return true;
    }

    StopResult stability(int iter, const Particle<T>& a, const Particle<T>& b, const Particle<T>& c) const
    {
        if (iter < 120)
            return StopResult::UNDETERMINED;

        constexpr T max_mag2 = SimEnv<T>::max_dist * SimEnv<T>::max_dist;
        if (a.mag2() > max_mag2) return StopResult::UNSTABLE;
        if (b.mag2() > max_mag2) return StopResult::UNSTABLE;
        if (c.mag2() > max_mag2) return StopResult::UNSTABLE;

        if (similar(beg_a, a) &&
            similar(beg_b, b) &&
            similar(beg_c, c))
        {
            return StopResult::STABLE;
        }

        if (iter >= max_iter) return StopResult::UNSTABLE;
        return StopResult::UNDETERMINED;
    }
};

template<class T, template<class> class StopPolicy = StopPolicy_MaxDist>
class Sim
{
    #define SimTmpl  template<class T, template<class> class StopPolicy>
    #define SimID    Sim<T, StopPolicy>

    using SimEnv = SimEnv<T>;
    using SimPlot = SimPlot<T>;
    using Vec2 = Vec2<T>;

    Particle<T> a, b, c;
    int iter = 0;

    [[no_unique_address]] StopPolicy<T> unstable_rule;

    void pairwise_gravity(Particle<T>& p, Particle<T>& q, const T G, const T soft2);
    void compute_accels(Particle<T>& a, Particle<T>& b, Particle<T>& c, const T G, const T soft2);

public:

    Sim() = default;
    Sim(const Sim<T>& r) : a(r.a), b(r.b), c(r.c) {}
    bool operator ==(const Sim<T>& r) const {
        return (a == r.a) && (b == r.b) && (c == r.c);
    }

    void setup(const SimEnv& env, Vec2 pos, Vec2 vel_a, Vec2 vel_b, Vec2 vel_c);
    void progress(const SimEnv& env);
    StopResult stability() const      { return unstable_rule.stability(iter, a, b, c); }
    bool       escaped(int iter_lim)  { return iter >= (iter_lim - SimEnv::escape_freq); }
    int        curIter() const        { return iter; }
               

    //int run(const SimEnv& env);

    // plots a copy of itself (treats "this" as starting configuration)
    int plot(const SimEnv& env, SimPlot& plot) const;

    Vec2 particleA() const { return a; }
    Vec2 particleB() const { return b; }
    Vec2 particleC() const { return c; }
};

template<
    class T, 
    int VEL_GRID_DIM, 
    template<class> class StopPolicy = StopPolicy_MaxDist,
    bool MULTI_THREAD=true
>
struct SimGrid
{
    #define SimGridTmpl  template<class T, int VEL_GRID_DIM, template<class> class StopPolicy, bool MULTI_THREAD>
    #define SimGridID    SimGrid<T, VEL_GRID_DIM, StopPolicy, MULTI_THREAD>

    using Sim = Sim<T, StopPolicy>;
    using SimPlot = SimPlot<T>;
    using SimEnv = SimEnv<T>;
    using Vec2 = Vec2<T>;

    const SimEnv& env;

    static constexpr int VEL_GRID_LEN = (VEL_GRID_DIM * VEL_GRID_DIM);
    static constexpr int SIM_COUNT = VEL_GRID_LEN * VEL_GRID_LEN;

    Sim sims[SIM_COUNT];
    int best_stability = 0;
    int best_sim = 0;
    Vec2 start_pos{};

    SimGrid(const SimEnv& e) : env(e) {}

    void setup(Vec2 c_pos);
    void setupSim(int sim_i, Sim& sim);
    void startingVelocities(int sim_i, Vec2& vel_a, Vec2& vel_b, Vec2& vel_c);
    void run(); // progress to env.max_iter (or until all unstable)

    // treats current state as starting state, doesn't affect 'this' object's sims
    void plotBest(SimPlot& plot);

    // call after run()
    int  bestStability() const { return best_stability; }
    Sim  bestSimConfig() { Sim s; setupSim(best_sim, s); return s; }
};

SIM_END;

#include "orbit_sim.hpp"