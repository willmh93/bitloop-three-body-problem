#include <bitloop.h>

SIM_BEG;
using namespace bl;

template<class T>
void SimPlot<T>::clear()
{
    a_path.clear();
    b_path.clear();
    c_path.clear();
}

template<class T>
void SimPlot<T>::recordPositions(const Vec2& a, const Vec2& b, const Vec2& c)
{
    a_path.push_back(a);
    b_path.push_back(b);
    c_path.push_back(c);
}

template<class T>
void SimPlot<T>::drawPath(Viewport* ctx, const std::vector<Vec2>& path, f32 main_alpha, Color col, int cur_iter) const
{
    if (path.size() < 2) return;

    const Color main_color(col.r, col.g, col.b, (int)(main_alpha * 255.0f));
    ctx->setStrokeStyle(main_color);
    ctx->beginPath();
    ctx->moveTo(path[0]);
    for (size_t i = 0; i < path.size(); i++)
    {
        Vec2 p = path[i];
        ctx->lineTo(p);
    }
    ctx->stroke();

    if (cur_iter > 2)
    {
        int trail = 150;
        int i0 = std::max(0, cur_iter - trail);
        int i1 = cur_iter;
        f32 fi0 = (f32)i0;
        f32 fi1 = (f32)i1;
        for (int i = i0; i < i1; i++)
        {
            f32 f = Math::lerpFactor((f32)i, fi0, fi1);
            const Color color(col.r, col.g, col.b, (int)(f * 255.0f));
            ctx->setStrokeStyle(color);
            ctx->strokeLine(path[i-1], path[i]);
        }
    }
}

template<class T>
void SimPlot<T>::draw(Viewport* ctx, float main_alpha, int cur_iter) const
{
    cur_iter /= stride;

    drawPath(ctx, a_path, main_alpha, Color::red, cur_iter);
    drawPath(ctx, b_path, main_alpha, Color::green, cur_iter);
    drawPath(ctx, c_path, main_alpha, Color::yellow, cur_iter);
}


SimTmpl double SimID::repeatibility(const T pos_tolerance, const T vel_tolerance) const
{
    //T a_dist = 
    return 0.0;
}

//SimTmpl bool SimID::unstable() const
//{
//    constexpr T max_mag2 = SimEnv::max_dist * SimEnv::max_dist;
//    if (a.mag2() > max_mag2) return true;
//    if (b.mag2() > max_mag2) return true;
//    if (c.mag2() > max_mag2) return true;
//    return false;
//}


SimTmpl void SimID::pairwise_gravity(Particle<T>& p, Particle<T>& q, const T G, const T soft2)
{
    const T rx = q.x - p.x;
    const T ry = q.y - p.y;
    const T r2 = rx * rx + ry * ry + soft2;
    const T inv_r = T(1) / sqrt(r2);
    const T inv_r3 = inv_r * inv_r * inv_r;
    const T scale = G * inv_r3;
    const T fx = scale * rx;
    const T fy = scale * ry;
    p.ax += fx; p.ay += fy;
    q.ax -= fx; q.ay -= fy;
}

SimTmpl void SimID::compute_accels(Particle<T>& a, Particle<T>& b, Particle<T>& c, const T G, const T soft2)
{
    a.ax = a.ay = b.ax = b.ay = c.ax = c.ay = T(0);
    pairwise_gravity(a, b, G, soft2);
    pairwise_gravity(b, c, G, soft2);
    pairwise_gravity(c, a, G, soft2);
}

SimTmpl void SimID::setup(Vec2 pos, Vec2 vel_a, Vec2 vel_b, Vec2 vel_c)
{
    // a/b always start at (-1,0) and (1,0)
    // c is the only pos needed to create any triangular shape
    a.set(-1.0, 0);
    b.set(1.0, 0);
    c.set(pos);

    a.vx = vel_a.x; a.vy = vel_a.y;
    b.vx = vel_b.x; b.vy = vel_b.y;
    c.vx = vel_c.x; c.vy = vel_c.y;

    iter = 0;

    unstable_rule.init(a, b, c);
}

SimTmpl void SimID::progress(const SimEnv& env)
{
    const T dt = env.dt, half = T(0.5);

    compute_accels(a, b, c, env.G, env.soft2);

    // 2) update velocity (half-kick)
    a.vx += a.ax * (half * dt); a.vy += a.ay * (half * dt);
    b.vx += b.ax * (half * dt); b.vy += b.ay * (half * dt);
    c.vx += c.ax * (half * dt); c.vy += c.ay * (half * dt);

    // 3) update pos
    a.x += a.vx * dt; a.y += a.vy * dt;
    b.x += b.vx * dt; b.y += b.vy * dt;
    c.x += c.vx * dt; c.y += c.vy * dt;

    compute_accels(a, b, c, env.G, env.soft2);

    // 5) update velocity (half-kick)
    a.vx += a.ax * (half * dt); a.vy += a.ay * (half * dt);
    b.vx += b.ax * (half * dt); b.vy += b.ay * (half * dt);
    c.vx += c.ax * (half * dt); c.vy += c.ay * (half * dt);

    iter++;
}

//template<class T>
//bool Sim<T>::progress(const SimEnv& env)
//{
//    const T dt = env.dt;
//    const T G = env.G;
//    const T soft2 = env.soft2;
//
//    // update accelerations
//    a.ax = 0; a.ay = 0;
//    b.ax = 0; b.ay = 0;
//    c.ax = 0; c.ay = 0;
//
//    pairwise_gravity(a, b, G, soft2);
//    pairwise_gravity(b, c, G, soft2);
//    pairwise_gravity(c, a, G, soft2);
//
//    // update velocities
//    a.vx += a.ax * dt; a.vy += a.ay * dt;
//    b.vx += b.ax * dt; b.vy += b.ay * dt;
//    c.vx += c.ax * dt; c.vy += c.ay * dt;
//
//    // update positions
//    a.x += a.vx * dt; a.y += a.vy * dt;
//    b.x += b.vx * dt; b.y += b.vy * dt;
//    c.x += c.vx * dt; c.y += c.vy * dt;
//
//    // check stable
//    if (iter++ % 10 == 0) is_unstable |= unstable();
//    return !is_unstable;
//}


SimTmpl int SimID::plot(const SimEnv& env, SimPlot& plot) const
{
    Sim<T, UnstablePolicy> s = *this;

    plot.clear();
    for (int i = 0; i < env.max_iter; i++)
    {
        if (i % SimPlot::stride == 0)
            plot.recordPositions(s.particleA(), s.particleB(), s.particleC());

        s.progress(env);
        if (i % SimEnv::escape_freq == 0 && s.unstable())
            break;
    }
    return iter;
}

SimGridTmpl void SimGridID::setup(Vec2 c_pos) {
    start_pos = c_pos;
    for (int s = 0; s < SIM_COUNT; s++)
        setupSim(s, sims[s]);
}

SimGridTmpl void SimGridID::setupSim(int sim_i, Sim& sim)
{
    Vec2 vel_a, vel_b, vel_c;
    startingVelocities(sim_i, vel_a, vel_b, vel_c);
    sim.setup(start_pos, vel_a, vel_b, vel_c);
}

SimGridTmpl void SimGridID::startingVelocities(
    int sim_i, 
    Vec2& vel_a, 
    Vec2& vel_b,
    Vec2& vel_c)
{
    int iU = sim_i / VEL_GRID_LEN;
    int iW = sim_i % VEL_GRID_LEN;

    T dim_cen = T(VEL_GRID_DIM - 1) / T(2);

    T ux = (T(iU % VEL_GRID_DIM) - dim_cen) * env.max_vel;
    T uy = (T(iU / VEL_GRID_DIM) - dim_cen) * env.max_vel;
    T wx = (T(iW % VEL_GRID_DIM) - dim_cen) * env.max_vel;
    T wy = (T(iW / VEL_GRID_DIM) - dim_cen) * env.max_vel;

    T vax = -(ux + wx) / T(3);
    T vay = -(uy + wy) / T(3);

    vel_a.set(vax, vay);
    vel_b.set(vax + ux, vay + uy);
    vel_c.set(vax + wx, vay + wy);
}

SimGridTmpl void SimGridID::run()
{
    best_iter = 0;
    best_sim = 0;
    any_escaped = false;

    if constexpr (MULTI_THREAD)
    {
        std::future<void> results[SIM_COUNT];
        for (int s = 0; s < SIM_COUNT; s++)
        {
            results[s] = Thread::pool().submit_task([this, s]()
            {
                Sim& sim = sims[s];
                
                for (int i = 0; i < env.max_iter; i++) {
                    sim.progress(env);
                    if (i % SimEnv::escape_freq == 0 && sim.unstable())
                        break;
                }
            });
        }

        for (int s = 0; s < SIM_COUNT; s++)
        {
            results[s].get(); // wait for sim to finish

            Sim& sim = sims[s];
            bool escaped = sim.escaped(env.max_iter);
            if (sim.curIter() > best_iter)
            {
                best_sim = s;
                best_iter = sim.curIter();
            }
            if (escaped) any_escaped = true;
        }
    }
    else // single-threaded
    {
        for (int s = 0; s < SIM_COUNT; s++)
        {
            Sim& sim = sims[s];
            for (int i = 0; i < env.max_iter; i++) {
                sim.progress(env);
                if (i % SimEnv::escape_freq == 0 && sim.unstable())
                    break;
            }

            bool escaped = sim.escaped(env.max_iter);
            if (sim.curIter() > best_iter)
            {
                best_sim = s;
                best_iter = sim.curIter();
            }
            if (escaped) any_escaped = true;
        }
    }
}

SimGridTmpl void SimGridID::plotBest(SimPlot& plot)
{
    // plot best
    Sim best = bestSimConfig();
    best.plot(env, plot);
    //plot.clear();
    //for (int i = 0; i < env.max_iter; i++)
    //{
    //    plot.recordPositions(best.particleA(), best.particleB(), best.particleC());
    //    if (!best.progress(env)) break;
    //}
}

SIM_END;