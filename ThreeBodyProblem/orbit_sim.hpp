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
void SimPlot<T>::drawPath(Viewport* ctx, const std::vector<Vec2>& path, Color col, int cur_iter, double path_w, double trail_w) const
{
    if (path.size() < 2) return;

    bool draw_full_path = (cur_iter >= 0);

    // account for fewer path data points than actual iterations
    cur_iter /= stride;

    const Color main_color(col.r, col.g, col.b, (int)(path_alpha * 255.0f));
    ctx->setLineWidth(path_w);
    ctx->setStrokeStyle(main_color);
    if (draw_full_path)
        ctx->strokePath(path, 0, std::min((size_t)cur_iter, path.size()));
    else
        ctx->strokePath(path);

    constexpr int trail = 150;
    if (cur_iter > 1)
    {
        auto comp = ctx->scopedComposite(CompositeOperation::LIGHTER);
        auto drawTrail = [&](f64 max_w, f32 layer_alpha)
        {
            constexpr int fade_step = 10;

            int i0 = std::max(1, cur_iter - trail);
            int i1 = std::min((int)path.size(), cur_iter);
            f64 fi0 = (f64)i0;
            f64 fi1 = (f64)i1;
            f64 add_w = max_w - path_w;
            f64 path_alpha = layer_alpha / (f64)(trail / fade_step);

            for (int i = i0; i < i1; i += fade_step)
            {
                f64 f = Math::lerpFactor((f64)i, fi0, fi1);
                Color color(col.r, col.g, col.b, std::max(1, (int)(f * path_alpha)));

                ctx->setLineWidth(path_w + (add_w) * f);
                ctx->setStrokeStyle(color);
                ctx->strokePath(path, i, i1);
            }
        };

        drawTrail(trail_w, 75);
        drawTrail(trail_w * 1.5, 10);
        drawTrail(trail_w * 3, 5);
    }
}

template<class T>
void SimPlot<T>::draw(Viewport* ctx, int cur_iter, double path_w, double trail_w) const
{
    drawPath(ctx, a_path, Color::red, cur_iter, path_w, trail_w);
    drawPath(ctx, b_path, Color::green, cur_iter, path_w, trail_w);
    drawPath(ctx, c_path, Color::yellow, cur_iter, path_w, trail_w);
}

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

SimTmpl void SimID::setup(const SimEnv& env, Vec2 pos, Vec2 vel_a, Vec2 vel_b, Vec2 vel_c)
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

    unstable_rule.init(&env, a, b, c);
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
    Sim<T, StopPolicy> s = *this;

    plot.clear();
    for (int i = 0; i < env.max_iter; i++)
    {
        if (i % SimPlot::stride == 0)
            plot.recordPositions(s.particleA(), s.particleB(), s.particleC());

        s.progress(env);
        if (i % env.escape_freq == 0 && (int)s.stability().type & (int)StopResult::ABORT_MASK)
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
    sim.setup(env, start_pos, vel_a, vel_b, vel_c);
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
    best_stability = StopResult(StopResult::INVALID, -1.0);
    best_sim = 0;

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
                    if (i % env.escape_freq == 0 && (int)sim.stability().type & (int)StopResult::ABORT_MASK)
                        break;
                }
            });
        }

        for (int s = 0; s < SIM_COUNT; s++)
        {
            results[s].get(); // wait for sim to finish

            Sim& sim = sims[s];
            StopResult sim_stability = sim.stability();

            if (sim_stability.type == StopResult::INVALID)
                continue;

            if (StopPolicy<T>::isBetterResult(sim_stability, best_stability))
            {
                best_sim = s;
                best_stability = sim.stability();
            }
        }
    }
    else // single-threaded
    {
        for (int s = 0; s < SIM_COUNT; s++)
        {
            Sim& sim = sims[s];
            for (int i = 0; i < env.max_iter; i++) {
                sim.progress(env);
                if (i % env.escape_freq == 0 && (int)sim.stability().type & (int)StopResult::ABORT_MASK)
                    break;
            }

            StopResult sim_stability = sim.stability();
            if (sim_stability.type == StopResult::INVALID)
                continue;

            if (StopPolicy<T>::isBetterResult(sim.stability(), best_stability))
            {
                best_sim = s;
                best_stability = sim.stability();
            }
        }
    }
}

SIM_END;