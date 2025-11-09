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

    /*if (ImGui::Section("Presets", true))
    {
        if (ImGui::Button("Figure-Eight"))
        {
            bl_schedule([](ThreeBodyProblem_Scene& scene) {
                scene.launchSim(
                    vec2(0, 0),
                    vec2((flt)0.347116889244505, (flt)0.532724944724257),
                    vec2((flt)0.347116889244505, (flt)0.532724944724257),
                    vec2((flt)-0.694233778489010, (flt)-1.065449889448514));
            });
        }
    }*/

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

void ThreeBodyProblem_Scene::launchSim(vec2 c, vec2 vel_a, vec2 vel_b, vec2 vel_c)
{
    chosen_point = c;
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
    ctx->drawWorldAxis(1.0, 0.0, 1.0);

    ctx->worldHudMode();
    ctx->setLineWidth(2);

    best_plot.draw(ctx, hasChosenPoint() ? 0.08f : 1.0f, best_sim.curIter());
    //best_plot.draw(ctx, hasChosenPoint() ? 0.0f : 1.0f, best_sim.curIter());

    if (hasChosenPoint())
    {
        ctx->fillEllipse(best_sim.particleA(), flt(4), Color::red);
        ctx->fillEllipse(best_sim.particleB(), flt(4), Color::green);
        ctx->fillEllipse(best_sim.particleC(), flt(4), Color::yellow);
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
