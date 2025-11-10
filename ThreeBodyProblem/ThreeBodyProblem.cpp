#include "ThreeBodyProblem.h"

SIM_BEG;

using namespace bl;

/// ─────────────────────── project setup ───────────────────────

void ThreeBodyProblem_Project::UI::sidebar()
{
    bl_scoped(viewport_count);
    ImGui::SliderInt("Viewport Count", &viewport_count, 1, 8);
}

void ThreeBodyProblem_Project::projectPrepare(Layout& layout)
{
    layout << create<ThreeBodyProblem_Scene>(viewport_count);
}

/// ─────────────────────── scene ───────────────────────

void ThreeBodyProblem_Scene::UI::sidebar()
{
    if (ImGui::Section("View", true))
    {
        // imgui camera controls
        bl_scoped(camera);
        camera.populateUI();
    }

    if (ImGui::Section("Animation", true))
    {
        bl_scoped(animation_speed);
        ImGui::SliderInt("Animation Speed", &animation_speed, 1, 200);
    }

    if (ImGui::Section("Presets", true))
    {
        if (ImGui::Button("Figure-Eight"))
        {
            bl_schedule([](ThreeBodyProblem_Scene& scene) {
                scene.launchSim(
                    vec2(0, 0),
                    vec2((flt)0.347116889244505, (flt)0.532724944724257),
                    vec2((flt)0.347116889244505, (flt)0.532724944724257),
                    vec2((flt)-0.694233778489010, (flt)-1.065449889448514),
                    0.0);
            });
        }
    }

    if (ImGui::Section("Stats", true))
    {
        if (ImGui::BeginTable("sim_stats", 2, ImGuiTableFlags_SizingStretchProp))
        {
            bl_pull(cur_iter);

            ImGui::TableNextColumn();
            ImGui::Text("Iteration");
            ImGui::Text("%d", cur_iter);

            ImGui::EndTable();
        }
    }
}

void ThreeBodyProblem_Scene::sceneStart()
{
    /// initialize scene
}

void ThreeBodyProblem_Scene::sceneMounted(Viewport* ctx)
{
    // initialize viewport
    camera.setSurface(ctx);
    camera.setOriginViewportAnchor(Anchor::CENTER);
    camera.focusWorldRect(-2.5, -2.5, 2.5, 2.5);
    camera.uiSetCurrentAsDefault();

    navigator.setTarget(camera);
    navigator.setDirectCameraPanning(true);
}

void ThreeBodyProblem_Scene::sceneDestroy()
{
    /// destroy scene (dynamically allocated resources, etc.)
}

void ThreeBodyProblem_Scene::sceneProcess()
{
    /// process scene once each frame (not per viewport)
    
    if (hasChosenPoint())
    {
        for (int i = 0; i < animation_speed; i++)
            best_sim.progress(env);

        cur_iter = best_sim.curIter();
    }
}

void ThreeBodyProblem_Scene::launchSim(vec2 c, vec2 vel_a, vec2 vel_b, vec2 vel_c, double path_alpha)
{
    chosen_point = c;
    best_plot.setPathAlpha(path_alpha);
    best_sim.setup(c, vel_a, vel_b, vel_c);
    best_sim.plot(env, best_plot);
}

void ThreeBodyProblem_Scene::viewportProcess(
    [[maybe_unused]] Viewport* ctx,
    [[maybe_unused]] double dt)
{
    /// process scene update (called once per mounted viewport each frame)
}

void ThreeBodyProblem_Scene::viewportDraw(Viewport* ctx) const
{
    /// draw scene on this viewport (no modifying sim state)
    ctx->transform(camera.getTransform());
    ctx->drawWorldAxis(0.2, 0.00, 0.3);

    //ctx->worldHudMode();
    ctx->worldMode();
    ctx->setLineCap(LineCap::CAP_ROUND);


    best_plot.draw(ctx, hasChosenPoint() ? best_sim.curIter() : -1, particle_r, particle_r*3);

    if (hasChosenPoint())
    {
        auto _c = ctx->scopedComposite(CompositeOperation::LIGHTER);



        auto drawParticle = [&](vec2 p, Color c, f64 glow_a, f64 particle_r, f64 glow_r, int steps=7)
        {
            Color c1 = Color(c, 0);
            Color c0 = Color(c, (int)(glow_a * (f64)(255/steps)));
            f64 step = 1.0f / (double)steps;
            for (f64 r = 1.0; r > 0.0; r -= step) {
                ctx->setFillRadialGradient(p, 0.0, r * glow_r, c0, c1);
                ctx->fillEllipse(p, r * glow_r);
            }

            ctx->fillEllipse(p, particle_r, c);
        };

        drawParticle(best_sim.particleA(), Color::red, 0.5, particle_r, glow_r);
        drawParticle(best_sim.particleB(), Color::green, 0.5, particle_r, glow_r);
        drawParticle(best_sim.particleC(), Color::yellow, 0.5, particle_r, glow_r);
    }
}

void ThreeBodyProblem_Scene::onEvent(Event e)
{
    navigator.handleWorldNavigation(e, true);
}

void ThreeBodyProblem_Scene::onPointerDown(PointerEvent e)
{
    if (!hasChosenPoint())
    {
        // on click, lock body C to mouse world-pos
        vec2 wp = camera.getTransform().toWorld<flt>(e.x(), e.y());
        chosen_point = wp;

        SimGrid sims(env);
        sims.setup(chosen_point);
        sims.run();

        sims.plotBest(best_plot);


        best_sim = sims.bestSimConfig();
    }
    else // unlock chosen point (go back to explore mode)
        chosen_point = undefined_pos;

    best_plot.setPathAlpha(0.08);
}

void ThreeBodyProblem_Scene::onPointerMove(PointerEvent e)
{
    if (!hasChosenPoint())
    {
        vec2 wp = camera.getTransform().toWorld<flt>(e.x(), e.y());

        SimGrid sims(env);
        sims.setup(wp);
        sims.run();
        sims.plotBest(best_plot);

        best_sim = sims.bestSimConfig();
    }
}

SIM_END;
