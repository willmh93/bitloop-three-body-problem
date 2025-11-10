#pragma once
#include "orbit_sim.h"

SIM_BEG;

using namespace bl;

struct ThreeBodyProblem_Scene : public Scene<ThreeBodyProblem_Scene>
{
    CameraInfo       camera;
    CameraNavigator  navigator;
    
    /// ─────── variables ───────
    using flt = f64;
    using vec2 = Vec2<flt>;

    // tweakable
    template<class T> 
    using UnstablePolicy                = UnstablePolicy_MaxDist<T>;
    static constexpr int vel_grid_size  = 3;
    int iter_lim                        = 100000;
    flt G                               = flt(1);
    flt max_vel                         = flt(1);
    flt dt                              = flt(0.005);
    int animation_speed                 = 5;

    static constexpr f64 particle_r = 0.0075;// 2.0;
    static constexpr f64 glow_r = 0.08;// 24.0;

    // determine types
    using SimEnv  = SimEnv<flt>;
    using SimPlot = SimPlot<flt>;
    using Sim     = Sim<flt, UnstablePolicy>;
    using SimGrid = SimGrid<flt, vel_grid_size, UnstablePolicy>;

    const vec2 undefined_pos = vec2::highest();

    // on click, lock body C to mouse world-pos
    vec2 chosen_point = undefined_pos;

    SimEnv   env = SimEnv(G, max_vel, iter_lim, dt);
    Sim      best_sim;
    SimPlot  best_plot;

    // stats for UI
    int cur_iter = 0;

    /// ─────── methods ───────
    bool hasChosenPoint() const { return chosen_point != undefined_pos; }
    void launchSim(vec2 c, vec2 vel_a, vec2 vel_b, vec2 vel_c, double path_alpha = 0.08);
    

    /// ─────── launch config (overridable by Project) ───────
    struct Config { /* double gravity = 9.8; */ };

    ThreeBodyProblem_Scene(Config& info [[maybe_unused]])
        // : gravity(info.gravity)
    {}

    /// ─────── Thread-safe UI for editing Scene inputs with ImGui ───────
    struct UI : Interface
    {
        using Interface::Interface;
        void sidebar();
        //void overlay();

        /// ─────── Your UI-only variables ───────
        // bool test_popup_open = false;
    };


    /// ─────── Scene methods ───────
    void sceneStart() override;
    void sceneMounted(Viewport* viewport) override;
    void sceneDestroy() override;
    void sceneProcess() override;

    /// ─────── Viewport methods ───────
    void viewportProcess(Viewport* ctx, double dt) override;
    void viewportDraw(Viewport* ctx) const override;

    /// ─────── Input handling ───────
    void onEvent(Event e) override; // All event types

    void onPointerDown(PointerEvent e) override;
    void onPointerMove(PointerEvent e) override;
};

struct ThreeBodyProblem_Project : public Project<ThreeBodyProblem_Project>
{
    static ProjectInfo info() {
        // Categorize your project
        return ProjectInfo({ "Physics", "Three-Body Problem" });
    }

    struct UI : Interface {
        using Interface::Interface;
        void sidebar();
        //void overlay();
    };

    int viewport_count = 1;

    void projectPrepare(Layout& layout) override;
};

SIM_END;
