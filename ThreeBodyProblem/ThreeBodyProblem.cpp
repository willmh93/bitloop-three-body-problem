#include "ThreeBodyProblem.h"
#include <fstream>

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
    // Every scene gets its own viewport
    for (int i = 0; i < viewport_count; i++)
        layout << create<ThreeBodyProblem_Scene>();
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
                scene.launchPreset(
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

    // Screener
    {
        bl_pull(results_cstr);
        bl_scoped(selected_result);

        if (ImGui::Button("Run Screener"))
            bl_schedule([](ThreeBodyProblem_Scene& scene) { scene.beginScan(); });

        if (ImGui::ListBox("Stable Results", &selected_result, results_cstr.data(), (int)results_cstr.size(), 15))
            bl_schedule([&](ThreeBodyProblem_Scene& scene) { scene.setCurrentSimFromResult(selected_result); });

        if (ImGui::Button("Start"))
            bl_schedule([&](ThreeBodyProblem_Scene& scene) { scene.startAnimation(); });

        if (ImGui::Button("Stop"))
            bl_schedule([&](ThreeBodyProblem_Scene& scene) { scene.endAnimation(); });

        if (ImGui::Button("Save"))
        {
            bl_pull(results);

            std::ofstream out("results.bin", std::ios::binary);
            out.write((const char*)results.data(), sizeof(Sim)*results.size());
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

    bmp.setCamera(camera);

    current_plot.setPathAlpha(0.2);
}

void ThreeBodyProblem_Scene::sceneDestroy()
{
    /// destroy scene (dynamically allocated resources, etc.)
}

void ThreeBodyProblem_Scene::sceneProcess()
{
    /// process scene once each frame (not per viewport)
    
    if (playingAnimation())
    {
        for (int i = 0; i < animation_speed; i++)
            sim_animation.progress(env);

        cur_iter = sim_animation.curIter();
    }
}

void ThreeBodyProblem_Scene::setCurrentSimFromResult(int index)
{
    if (index < 0 || index >= (int)results.size())
        return;

    setCurrentSim(results[index]);
}

void ThreeBodyProblem_Scene::launchPreset(vec2 c, vec2 vel_a, vec2 vel_b, vec2 vel_c, double path_alpha)
{
    current_plot.setPathAlpha(path_alpha);
    current_sim.setup(env, c, vel_a, vel_b, vel_c);

    setCurrentSim(current_sim);
    startAnimation();
}

void ThreeBodyProblem_Scene::beginScan()
{
    results.clear();
    results_str.clear();
    results_cstr.clear();
    scanning = true;
}

void ThreeBodyProblem_Scene::viewportProcess(
    [[maybe_unused]] Viewport* ctx,
    [[maybe_unused]] double dt)
{
    int iw = (int)ctx->width();
    int ih = (int)ctx->height();
    bmp.setBitmapSize(iw, ih);
    bmp.setStageRect(0, 0, iw, ih);

    if (scanning)
    {
        bool complete = bmp.forEachWorldPixel(current_row, [&](int px, int py, flt wx, flt wy)
        {
            //if (wy > 0.00000001) return;

            SimGrid sims(env);
            sims.setup({ wx, wy });
            sims.run();

            //int stability = sims.bestStability();
            Color col;

            //if (stability >= UNDETERMINED)
            //{
                Sim& best = sims.sims[sims.best_sim];
                col = Color::red;

                float ratio = ((float)best.curIter() / (float)iter_lim);
                ratio = std::sqrt(ratio);
                col.adjustHue(ratio * 360.0f);

                //if (stability == STABLE)
                //{
                //    col = Color(0, 255, 0, 255);
                //    results.push_back(sims.bestSimConfig());
                //
                //    std::string name = "(" + std::to_string(wx) + std::to_string(wy) + ")";
                //    results_str.push_back(name);
                //    results_cstr.push_back(results_str.back().c_str());
                //
                //}
                //else
                //    col = Color(255, 0, 0, 80);

                
            //}
            //else
            //    col = Color(0, 0, 255, 40);

            bmp.setPixel(px, py, col);

        }, 4, 512);

        if (complete)
        {
            scanning = false;
        }
    }
}

void ThreeBodyProblem_Scene::viewportDraw(Viewport* ctx) const
{
    /// draw scene on this viewport (no modifying sim state)
    ctx->transform(camera.getTransform());

    ctx->drawImage(bmp);
    ctx->drawWorldAxis(0.1, 0.00, 0.3);

    ctx->worldHudMode();
    ctx->setLineCap(LineCap::CAP_ROUND);

    current_plot.draw(ctx, playingAnimation() ? sim_animation.curIter() : -1, particle_r, particle_r*3);

    if (playingAnimation())
    {
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

        auto _c = ctx->scopedComposite(CompositeOperation::LIGHTER);
        drawParticle(sim_animation.particleA(), Color::red, 0.5, particle_r, glow_r);
        drawParticle(sim_animation.particleB(), Color::green, 0.5, particle_r, glow_r);
        drawParticle(sim_animation.particleC(), Color::yellow, 0.5, particle_r, glow_r);
    }

    if (isRecording())
    {
        ctx->stageMode();
        ctx->drawCursor(mouse->stage_x, mouse->stage_y);
    }

    ctx->worldHudMode();
    ctx->fillEllipse((f64)mouse->world_x, (f64)mouse->world_y, 5.0);
}

void ThreeBodyProblem_Scene::onEvent(Event e)
{
    if (!ownsEvent(e)) return;

    navigator.handleWorldNavigation(e, true);
}

void ThreeBodyProblem_Scene::onPointerDown(PointerEvent e)
{
    if (!ownsEvent(e)) return;

    if (!playingAnimation())
    {
        // on click, lock body C to mouse world-pos
        vec2 pos = camera.getTransform().toWorld<flt>(e.x(), e.y());

        SimGrid sims(env);
        sims.setup(pos);
        sims.run();

        //if (sims.bestStability() >= UNDETERMINED)
        if (sims.bestStability().type != StopResult::INVALID)
        {
            setCurrentSim(sims.bestSimConfig());
            startAnimation();
        }
    }
    else
    {
        endAnimation();
    }

}

void ThreeBodyProblem_Scene::onPointerMove(PointerEvent e)
{
    if (!ownsEvent(e)) return;

    if (!playingAnimation())
    {
        vec2 wp = camera.getTransform().toWorld<flt>(e.x(), e.y());

        SimGrid sims(env);
        sims.setup(wp);
        sims.run();

        if (sims.bestStability().type != StopResult::INVALID)
        {
            setCurrentSim(sims.bestSimConfig());
        }
    }
}

SIM_END;
