#pragma once
#include "orbit_sim.h"

SIM_BEG;

using namespace bl;

struct ThreeBodyProblem_Scene : public Scene<ThreeBodyProblem_Scene>
{

    /// ─────── choose float type ───────

    using flt = f64;
    using vec2 = Vec2<flt>;

    
    /// ─────── sim configuration ───────

    // adjustable stop policies:
    template<class T> using StopPolicy  = StopPolicy_MaxDist<T>;
    //template<class T> using StopPolicy  = StopPolicy_Periodic<T>;

    static constexpr int vel_grid_size  = 5;
    int iter_lim                        = 300000;
    flt G                               = 1.0f;
    flt max_vel                         = 1.0f;//1.0f;
    flt dt = 0.02f;
    int animation_speed                 = 5;

    //static constexpr f64 particle_r = 0.0075;// 2.0;
    //static constexpr f64 glow_r = 0.08;// 24.0;
    static constexpr f64 particle_r = 2.0;
    static constexpr f64 glow_r = 24.0;

    // aliases
    using SimEnv  = SimEnv<flt>;
    using SimPlot = SimPlot<flt>;
    using Sim     = Sim<flt, StopPolicy>;
    using SimGrid = SimGrid<flt, vel_grid_size, StopPolicy>;

    const vec2 undefined_pos = vec2::highest();

    /// ─────── states ───────
   
    CameraInfo             camera;
    CameraNavigator        navigator;
    WorldImageT<flt>       bmp;

    bool scanning = false;
    int current_row = 0;
    bool interactive_enabled = true;

    std::vector<Sim>         results;
    std::vector<std::string> results_str;
    std::vector<const char*> results_cstr;
    int selected_result = 0;

    // on click, lock body C to mouse world-pos
    //vec2 chosen_point = undefined_pos;
    //Sim      chosen_sim;

    vec2     input_pos{};

    SimEnv   env = SimEnv(G, max_vel, iter_lim, dt);
    Sim      current_sim;
    SimPlot  current_plot;

    bool     sim_animating = false;
    Sim      sim_animation;

    // stats for UI
    int cur_iter = 0;

    /// ─────── methods ───────
    bool playingAnimation() const { return sim_animating; }

    //void setChosenSim()       { has_chosen_sim = true; }
    //bool hasChosenSim() const { return has_chosen_sim; }

    //bool hasChosenSim() const { return chosen_point != undefined_pos; }
    
    void setCurrentSim(Sim sim) { 
        current_sim = sim; 
        current_sim.plot(env, current_plot);
    }
    void startAnimation(double full_path_alpha=0.15, int fade_step=10) {
        sim_animation = current_sim; 
        sim_animating = true; 
        current_plot.setFullPathAlpha(full_path_alpha);
        current_plot.setFadeStepIters(fade_step);
    }
    void endAnimation() { 
        sim_animating = false; 
        current_plot.setFullPathAlpha(0.2);
        current_plot.setFadeStepIters(10);
    }

    void setCurrentSimFromResult(int index);
    void launchPreset(vec2 c, vec2 vel_a, vec2 vel_b, vec2 vel_c, double path_alpha = 0.08, int fade_step=10);
    void beginScan();

    /// ─────── launch config (overridable by Project) ───────
    struct Config {};

    ThreeBodyProblem_Scene(Config& info [[maybe_unused]]) {}
    ~ThreeBodyProblem_Scene() {}

    /// ─────── Thread-safe UI for editing Scene inputs with ImGui ───────
    struct UI : BufferedInterfaceModel
    {
        using BufferedInterfaceModel::BufferedInterfaceModel;
        void sidebar() override;
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

    struct UI : BufferedInterfaceModel {
        using BufferedInterfaceModel::BufferedInterfaceModel;
        void sidebar();
        //void overlay();
    };

    int viewport_count = 4;

    void projectPrepare(Layout& layout) override;
};

SIM_END;
