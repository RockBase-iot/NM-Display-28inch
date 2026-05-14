#include "factory_test.h"
#include "../../app/task_config.h"
#include "../../utils/logger.h"
#include "../../drivers/display/lvgl_port.h"
#include "../../bsp/nm_display_28/config.h"
#include "../../alg/quaternion/mahony.h"
#include "../../alg/quaternion/imu_bias.h"

#include <lvgl.h>
#include <esp_heap_caps.h>
#include <Wire.h>
#include <WiFi.h>
#include <SD_MMC.h>
#include <SPIFFS.h>
#include <esp_camera.h>
#include "../fonts/fonts.h"

// ─── Colour palette used across test screens ─────────────────────────────────
#define COLOR_BG        0x1A1A2E
#define COLOR_TITLE_BG  0x16213E
#define COLOR_PASS      0x2ECC71
#define COLOR_FAIL      0xE74C3C
#define COLOR_SKIP      0x7F8C8D
#define COLOR_TEXT      0xECF0F1
#define COLOR_SUBTEXT   0xBDC3C7

// ─── Verdict button callbacks ─────────────────────────────────────────────────

void FactoryTest::_on_ok_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    static_cast<FactoryTest *>(lv_event_get_user_data(e))->_verdict = 1;
}

void FactoryTest::_on_fail_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    static_cast<FactoryTest *>(lv_event_get_user_data(e))->_verdict = 0;
}

// ─── IMU live-view: toggle context passed to _on_imu_toggle_btn ──────────────
// Lifetime: allocated on the stack of _test_imu(); the pointer is valid for
// the entire duration of the IMU test because LVGL only dispatches input events
// to widgets on the active screen, and the screen is replaced before
// _test_imu() returns.
struct ImuToggleCtx {
    volatile bool *show_3d;
    lv_obj_t      *toggle_lbl;   // label on the toggle button
    lv_obj_t      *canvas;       // 3D canvas widget
    lv_obj_t      *tbl_panel;    // raw-data text panel
};

void FactoryTest::_on_imu_toggle_btn(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    auto *ctx = static_cast<ImuToggleCtx *>(lv_event_get_user_data(e));
    bool new3d = !(*ctx->show_3d);
    *ctx->show_3d = new3d;
    lv_label_set_text(ctx->toggle_lbl, new3d ? "RAW" : "AHRS");
    if (new3d) {
        lv_obj_add_flag  (ctx->tbl_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ctx->canvas,    LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_clear_flag(ctx->tbl_panel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag  (ctx->canvas,    LV_OBJ_FLAG_HIDDEN);
    }
}

// ─── AHRS artificial-horizon helper ──────────────────────────────────────────
// Draws artificial horizon (roll+pitch), roll-arc indicator, yaw compass tape,
// and numeric readouts onto canvas.
// roll / pitch / yaw are in radians.  Must be called with the LVGL mutex held.
//
// Geometry conventions (screen coords: x→right, y↓down):
//   Horizon direction : (cos R,  −sin R)      R = roll
//   Sky normal        : (−sin R, −cos R)
//   Ground-side test  : (Px−hcx)·sinR + (Py−hcy)·cosR > 0
//   Pitch offset      : nose-up → hcy > cy_m (horizon moves DOWN)
//   Roll arc ptr angle: LVGL_angle = 90 − roll_deg
//     (LVGL 0°=right, CW;  90°=bottom of arc = level flight)
static void _draw_ahrs_frame(lv_obj_t *canvas, float roll, float pitch, float yaw)
{
    constexpr int  CW     = SCREEN_WIDTH;            // 320
    constexpr int  CH     = SCREEN_HEIGHT - 40 - 54; // 146
    constexpr int  YAW_H  = 26;                      // yaw tape height (bottom)
    constexpr int  MAIN_H = CH - YAW_H;              // 120
    const float    cx     = CW   * 0.5f;             // 160
    const float    cy_m   = MAIN_H * 0.5f;           // 60

    const float pitch_deg = pitch * 57.295779f;
    const float roll_deg  = roll  * 57.295779f;
    float       yaw_deg   = yaw   * 57.295779f;
    while (yaw_deg <    0.f) yaw_deg += 360.f;
    while (yaw_deg >= 360.f) yaw_deg -= 360.f;

    // ── Horizon geometry ──────────────────────────────────────────────────
    const float sr = sinf(roll), cr = cosf(roll);
    constexpr float PPD = 2.8f;   // px / degree of pitch
    float pitch_px = pitch_deg * PPD;
    const float max_pp = cy_m - 4.f;
    if (pitch_px >  max_pp) pitch_px =  max_pp;
    if (pitch_px < -max_pp) pitch_px = -max_pp;
    const float hcx = cx, hcy = cy_m + pitch_px;

    // ── Horizon-canvas intersections ──────────────────────────────────────
    // Parametric: P(t) = (hcx + t·cr,  hcy − t·sr)
    lv_point_t h_pts[2]; int hpc = 0;
    auto try_edge = [&](float t) {
        if (hpc >= 2) return;
        float px_ = hcx + t * cr, py_ = hcy - t * sr;
        if (px_ < -1.f || px_ > CW+1.f || py_ < -1.f || py_ > MAIN_H+1.f) return;
        if (hpc==1 && fabsf(px_-h_pts[0].x)<2.f && fabsf(py_-h_pts[0].y)<2.f) return;
        h_pts[hpc++] = {
            (lv_coord_t)roundf(fmaxf(0.f, fminf((float)CW,     px_))),
            (lv_coord_t)roundf(fmaxf(0.f, fminf((float)MAIN_H, py_)))
        };
    };
    if (fabsf(cr) > 0.01f) { try_edge(-hcx/cr); try_edge((CW-hcx)/cr); }
    if (fabsf(sr) > 0.01f) { try_edge(hcy/sr);  try_edge((hcy-MAIN_H)/sr); }
    if (hpc < 2 && fabsf(cr) <= 0.01f) {          // near-vertical fallback
        lv_coord_t hx = (lv_coord_t)roundf(fmaxf(0.f, fminf((float)CW, hcx)));
        h_pts[0] = {hx, 0}; h_pts[1] = {hx, (lv_coord_t)MAIN_H}; hpc = 2;
    }

    // ── 1. Sky fill ───────────────────────────────────────────────────────
    lv_canvas_fill_bg(canvas, lv_color_hex(0x1A6FA8), LV_OPA_COVER);

    // ── 2. Ground polygon ─────────────────────────────────────────────────
    {
        auto is_ground = [&](float px_, float py_) {
            return (px_ - hcx)*sr + (py_ - hcy)*cr > 0.f;
        };
        lv_point_t gnd[8]; int gnc = 0;
        if (hpc == 2) { gnd[gnc++] = h_pts[0]; gnd[gnc++] = h_pts[1]; }
        lv_point_t corners[4] = {
            {0,0}, {(lv_coord_t)CW,0},
            {(lv_coord_t)CW,(lv_coord_t)MAIN_H}, {0,(lv_coord_t)MAIN_H}
        };
        for (auto &c : corners) { if (is_ground(c.x, c.y)) gnd[gnc++] = c; }
        if (gnc >= 3) {
            float gcx = 0, gcyc = 0;
            for (int i = 0; i < gnc; i++) { gcx += gnd[i].x; gcyc += gnd[i].y; }
            gcx /= gnc; gcyc /= gnc;
            for (int i = 0; i < gnc-1; i++)
                for (int j = i+1; j < gnc; j++)
                    if (atan2f(gnd[i].y-gcyc, gnd[i].x-gcx) >
                        atan2f(gnd[j].y-gcyc, gnd[j].x-gcx))
                    { lv_point_t tmp=gnd[i]; gnd[i]=gnd[j]; gnd[j]=tmp; }
            lv_draw_rect_dsc_t rdsc; lv_draw_rect_dsc_init(&rdsc);
            rdsc.bg_color = lv_color_hex(0x7B4F1E); rdsc.bg_opa = LV_OPA_COVER;
            rdsc.border_width = 0; rdsc.radius = 0;
            lv_canvas_draw_polygon(canvas, gnd, gnc, &rdsc);
        }
    }

    // ── 3. Horizon line ───────────────────────────────────────────────────
    if (hpc == 2) {
        lv_draw_line_dsc_t ldsc; lv_draw_line_dsc_init(&ldsc);
        ldsc.color = lv_color_hex(0xFFFFFF); ldsc.width = 2; ldsc.opa = LV_OPA_COVER;
        lv_canvas_draw_line(canvas, h_pts, 2, &ldsc);
    }

    // ── 4. Pitch ladder ───────────────────────────────────────────────────
    // Rung at Δp°: centre = (hcx − Δp·PPD·sr,  hcy − Δp·PPD·cr)
    // Rung direction: (cr, −sr)  (same as horizon)
    {
        lv_draw_line_dsc_t ldsc; lv_draw_line_dsc_init(&ldsc);
        ldsc.width = 1; ldsc.opa = LV_OPA_COVER;
        ldsc.round_start = 0; ldsc.round_end = 0;
        for (int dp = -30; dp <= 30; dp += 10) {
            if (dp == 0) continue;
            float rcx_ = hcx - dp * PPD * sr;
            float rcy_  = hcy - dp * PPD * cr;
            if (rcy_ < -25 || rcy_ > MAIN_H+25 || rcx_ < -70 || rcx_ > CW+70) continue;
            float rlen = (abs(dp) >= 20) ? 42.f : 30.f;
            ldsc.color = (dp > 0) ? lv_color_hex(0xFFFFFF) : lv_color_hex(0xD4B483);
            lv_point_t rung[2] = {
                {(lv_coord_t)roundf(rcx_-rlen*cr), (lv_coord_t)roundf(rcy_+rlen*sr)},
                {(lv_coord_t)roundf(rcx_+rlen*cr), (lv_coord_t)roundf(rcy_-rlen*sr)}
            };
            lv_canvas_draw_line(canvas, rung, 2, &ldsc);
            // End ticks (perpendicular, pointing toward horizon)
            float tx = (dp>0)?sr:-sr, ty_t = (dp>0)?cr:-cr;
            constexpr float TK = 6.f;
            lv_point_t tkL[2] = {
                {(lv_coord_t)roundf(rcx_-rlen*cr),       (lv_coord_t)roundf(rcy_+rlen*sr)},
                {(lv_coord_t)roundf(rcx_-rlen*cr+TK*tx), (lv_coord_t)roundf(rcy_+rlen*sr+TK*ty_t)}
            };
            lv_point_t tkR[2] = {
                {(lv_coord_t)roundf(rcx_+rlen*cr),       (lv_coord_t)roundf(rcy_-rlen*sr)},
                {(lv_coord_t)roundf(rcx_+rlen*cr+TK*tx), (lv_coord_t)roundf(rcy_-rlen*sr+TK*ty_t)}
            };
            lv_canvas_draw_line(canvas, tkL, 2, &ldsc);
            lv_canvas_draw_line(canvas, tkR, 2, &ldsc);
        }
    }

    // ── 5. Roll arc indicator ─────────────────────────────────────────────
    // Arc centre at canvas top-centre (160, 0), radius 55.
    // LVGL arc 30°–150° = the ∪-shape visible in the canvas.
    // Pointer LVGL angle = 90 − roll_deg  (right bank → pointer right of bottom)
    {
        constexpr int ARC_R = 55, ARC_CX = CW/2, ARC_CY = 0;
        // Rail
        lv_draw_arc_dsc_t adsc; lv_draw_arc_dsc_init(&adsc);
        adsc.color = lv_color_hex(0xAAAAAA); adsc.width = 1; adsc.opa = LV_OPA_80;
        lv_canvas_draw_arc(canvas, ARC_CX, ARC_CY, ARC_R, 30, 150, &adsc);
        // Tick marks
        lv_draw_line_dsc_t ldsc; lv_draw_line_dsc_init(&ldsc);
        ldsc.color = lv_color_hex(0xAAAAAA); ldsc.width = 1; ldsc.opa = LV_OPA_80;
        const int tick_rolls[] = {-60,-45,-30,-20,-10,0,10,20,30,45,60};
        for (int tr : tick_rolls) {
            float ang = (90 - tr) * 0.017453293f;
            float tc = cosf(ang), ts = sinf(ang);
            float inner = (float)ARC_R - ((abs(tr)%30==0||tr==0)?8.f:(abs(tr)%45==0?7.f:4.f));
            lv_point_t tp[2] = {
                {(lv_coord_t)roundf(ARC_CX + ARC_R*tc), (lv_coord_t)roundf(ARC_CY + ARC_R*ts)},
                {(lv_coord_t)roundf(ARC_CX + inner*tc), (lv_coord_t)roundf(ARC_CY + inner*ts)}
            };
            lv_canvas_draw_line(canvas, tp, 2, &ldsc);
        }
        // Gold pointer triangle
        float ptr_ang = 90.f - roll_deg;
        if (ptr_ang <  30.f) ptr_ang =  30.f;
        if (ptr_ang > 150.f) ptr_ang = 150.f;
        float prad = ptr_ang * 0.017453293f;
        float pc = cosf(prad), ps = sinf(prad);
        float baser = ARC_R - 10.f;
        float perp_x = -ps, perp_y = pc;
        constexpr float TRI_W = 4.5f;
        lv_point_t tri[3] = {
            {(lv_coord_t)roundf(ARC_CX + ARC_R*pc), (lv_coord_t)roundf(ARC_CY + ARC_R*ps)},
            {(lv_coord_t)roundf(ARC_CX + baser*pc + TRI_W*perp_x),
             (lv_coord_t)roundf(ARC_CY + baser*ps + TRI_W*perp_y)},
            {(lv_coord_t)roundf(ARC_CX + baser*pc - TRI_W*perp_x),
             (lv_coord_t)roundf(ARC_CY + baser*ps - TRI_W*perp_y)}
        };
        lv_draw_rect_dsc_t rdsc; lv_draw_rect_dsc_init(&rdsc);
        rdsc.bg_color = lv_color_hex(0xFFD700); rdsc.bg_opa = LV_OPA_COVER;
        rdsc.border_width = 0; rdsc.radius = 0;
        lv_canvas_draw_polygon(canvas, tri, 3, &rdsc);
    }

    // ── 6. Fixed aircraft symbol (gold) ──────────────────────────────────
    {
        const lv_coord_t acx = CW/2, acy = MAIN_H/2;
        lv_draw_line_dsc_t ldsc; lv_draw_line_dsc_init(&ldsc);
        ldsc.color = lv_color_hex(0xFFD700); ldsc.width = 3;
        ldsc.opa = LV_OPA_COVER; ldsc.round_start = 1; ldsc.round_end = 1;
        lv_point_t lw[2] = {{(lv_coord_t)(acx-38),(lv_coord_t)acy},{(lv_coord_t)(acx-8),(lv_coord_t)acy}};
        lv_point_t rw[2] = {{(lv_coord_t)(acx+8), (lv_coord_t)acy},{(lv_coord_t)(acx+38),(lv_coord_t)acy}};
        lv_canvas_draw_line(canvas, lw, 2, &ldsc);
        lv_canvas_draw_line(canvas, rw, 2, &ldsc);
        lv_draw_rect_dsc_t rdsc; lv_draw_rect_dsc_init(&rdsc);
        rdsc.bg_color = lv_color_hex(0xFFD700); rdsc.bg_opa = LV_OPA_COVER;
        rdsc.border_width = 0; rdsc.radius = LV_RADIUS_CIRCLE;
        lv_canvas_draw_rect(canvas, acx-5, acy-5, 10, 10, &rdsc);
        lv_point_t tail[2] = {{acx,(lv_coord_t)(acy+5)},{acx,(lv_coord_t)(acy+13)}};
        lv_canvas_draw_line(canvas, tail, 2, &ldsc);
    }

    // ── 7. Roll / Pitch numeric readout (top-left, below arc) ────────────
    {
        lv_draw_label_dsc_t tdsc; lv_draw_label_dsc_init(&tdsc);
        tdsc.color = lv_color_hex(0xFFFFFF); tdsc.font = &lv_font_montserrat_14;
        char buf[16];
        snprintf(buf, sizeof(buf), "R%+5.1f", roll_deg);
        lv_canvas_draw_text(canvas, 4, 20, 74, &tdsc, buf);
        snprintf(buf, sizeof(buf), "P%+5.1f", pitch_deg);
        lv_canvas_draw_text(canvas, 4, 36, 74, &tdsc, buf);
    }

    // ── 8. Yaw compass tape (bottom YAW_H px) ────────────────────────────
    {
        const lv_coord_t ty = (lv_coord_t)MAIN_H;   // tape top y = 120
        // Background
        {
            lv_draw_rect_dsc_t rdsc; lv_draw_rect_dsc_init(&rdsc);
            rdsc.bg_color = lv_color_hex(0x0F1A2E); rdsc.bg_opa = LV_OPA_COVER;
            rdsc.border_width = 0; rdsc.radius = 0;
            lv_canvas_draw_rect(canvas, 0, ty, CW, YAW_H, &rdsc);
        }
        // Gold centre pointer triangle (tip points down)
        {
            lv_draw_rect_dsc_t rdsc; lv_draw_rect_dsc_init(&rdsc);
            rdsc.bg_color = lv_color_hex(0xFFD700); rdsc.bg_opa = LV_OPA_COVER;
            rdsc.border_width = 0; rdsc.radius = 0;
            lv_point_t cptr[3] = {
                {(lv_coord_t)(CW/2-5), ty},
                {(lv_coord_t)(CW/2+5), ty},
                {(lv_coord_t)(CW/2),   (lv_coord_t)(ty+7)}
            };
            lv_canvas_draw_polygon(canvas, cptr, 3, &rdsc);
        }
        // Ticks + cardinal labels (every 10°, labelled at 45° multiples)
        constexpr float PPD_YAW = 3.0f;
        lv_draw_line_dsc_t ldsc; lv_draw_line_dsc_init(&ldsc);
        ldsc.opa = LV_OPA_COVER; ldsc.width = 1;
        lv_draw_label_dsc_t tdsc; lv_draw_label_dsc_init(&tdsc);
        tdsc.font = &lv_font_montserrat_14;
        int first10 = (int)floorf((yaw_deg - 54.f) / 10.f) * 10;
        for (int hi = first10; hi <= (int)(yaw_deg + 55.f); hi += 10) {
            float sx = cx + (hi - yaw_deg) * PPD_YAW;
            if (sx < 2.f || sx > CW - 2.f) continue;
            int hnorm = ((hi % 360) + 360) % 360;
            bool is_card  = (hnorm % 90 == 0);
            bool is_icard = (hnorm % 45 == 0 && !is_card);
            int tick_h = is_card ? 10 : (is_icard ? 7 : 4);
            ldsc.color = is_card  ? lv_color_hex(0xFFFFFF) :
                         is_icard ? lv_color_hex(0xBBBBBB) : lv_color_hex(0x555555);
            lv_point_t tick[2] = {
                {(lv_coord_t)roundf(sx), (lv_coord_t)(ty + YAW_H - tick_h)},
                {(lv_coord_t)roundf(sx), (lv_coord_t)(ty + YAW_H)}
            };
            lv_canvas_draw_line(canvas, tick, 2, &ldsc);
            if (is_card || is_icard) {
                const char *lbl = nullptr;
                switch (hnorm) {
                    case   0: lbl = "N";  break; case  45: lbl = "NE"; break;
                    case  90: lbl = "E";  break; case 135: lbl = "SE"; break;
                    case 180: lbl = "S";  break; case 225: lbl = "SW"; break;
                    case 270: lbl = "W";  break; case 315: lbl = "NW"; break;
                }
                if (lbl) {
                    int lw = (strlen(lbl)==1) ? 11 : 20;
                    tdsc.color = is_card ? lv_color_hex(0xFFFFFF) : lv_color_hex(0xBBBBBB);
                    lv_canvas_draw_text(canvas,
                        (lv_coord_t)roundf(sx) - lw/2, ty+2,
                        (lv_coord_t)(lw+2), &tdsc, lbl);
                }
            }
        }
        // Current heading (3 digits, gold, right side of tape)
        {
            char hdg[5]; snprintf(hdg, sizeof(hdg), "%03d", ((int)roundf(yaw_deg)) % 360);
            lv_draw_label_dsc_t tdsc2; lv_draw_label_dsc_init(&tdsc2);
            tdsc2.color = lv_color_hex(0xFFD700); tdsc2.font = &lv_font_montserrat_14;
            lv_canvas_draw_text(canvas, CW-36, ty+2, 34, &tdsc2, hdg);
        }
    }
}

// ─── Constructor ─────────────────────────────────────────────────────────────

FactoryTest::FactoryTest(Board &board) : _board(board) {}

// ─── Public API ──────────────────────────────────────────────────────────────

void FactoryTest::start()
{
    if (_task_handle) return;
    xTaskCreatePinnedToCore(
        _task_entry,
        "factory_test",
        TASK_STACK_FACTORY_TEST,
        this,
        TASK_PRIORITY_SENSOR,
        &_task_handle,
        FactoryTestTaskCore);
}

// ─── FreeRTOS task ───────────────────────────────────────────────────────────

void FactoryTest::_task_entry(void *arg)
{
    FactoryTest *self = static_cast<FactoryTest *>(arg);
    self->_run_all();
    self->_task_handle = nullptr;
    vTaskDelete(nullptr);
}

// ─── Test orchestration ──────────────────────────────────────────────────────

void FactoryTest::_run_all()
{
    struct TestCase {
        const char *name;
        Result (FactoryTest::*fn)();
    };

    constexpr TestCase cases[] = {
        { "Display",  &FactoryTest::_test_display  },
        { "Touch",    &FactoryTest::_test_touch    },
        { "SD Card",  &FactoryTest::_test_sdcard   },
        { "WiFi",     &FactoryTest::_test_wifi     },
        { "IMU",      &FactoryTest::_test_imu      },
        { "PMU",      &FactoryTest::_test_pmu      },
        { "RTC",      &FactoryTest::_test_rtc      },
        { "Camera",   &FactoryTest::_test_camera   },
        { "Audio",    &FactoryTest::_test_audio    },
    };
    constexpr int TOTAL = sizeof(cases) / sizeof(cases[0]);

    const char *names[TOTAL];
    Result      results[TOTAL];

    for (int i = 0; i < TOTAL; i++) {
        LOG_I("FactoryTest [%d/%d]: %s", i + 1, TOTAL, cases[i].name);
        names[i]   = cases[i].name;
        results[i] = (this->*(cases[i].fn))();
        vTaskDelay(pdMS_TO_TICKS(800));   // brief pause between tests
    }

    _show_summary(names, results, TOTAL);
}

// ─── LVGL screen helpers ─────────────────────────────────────────────────────

void FactoryTest::_show_screen(uint8_t index, const char *name,
                                const char *body, Result /*result*/)
{
    if (!LVGL_LOCK(500)) return;

    lv_obj_t *scr = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr, lv_color_hex(COLOR_BG), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scr, 0, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // ── Title bar ─────────────────────────────────────────────────────────────
    lv_obj_t *title_bar = lv_obj_create(scr);
    lv_obj_set_size(title_bar, SCREEN_WIDTH, 40);
    lv_obj_align(title_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(title_bar, lv_color_hex(COLOR_TITLE_BG), 0);
    lv_obj_set_style_border_width(title_bar, 0, 0);
    lv_obj_set_style_radius(title_bar, 0, 0);
    lv_obj_set_style_pad_all(title_bar, 4, 0);
    lv_obj_clear_flag(title_bar, LV_OBJ_FLAG_SCROLLABLE);

    char title_buf[48];
    snprintf(title_buf, sizeof(title_buf), "Test %d/9: %s", index, name);
    lv_obj_t *title_lbl = lv_label_create(title_bar);
    lv_label_set_text(title_lbl, title_buf);
    lv_obj_set_style_text_color(title_lbl, lv_color_hex(COLOR_TEXT), 0);
    // Bold-weight title: use Montserrat 16 (larger = visually bold on small screen)
    lv_obj_set_style_text_font(title_lbl, &lv_font_montserrat_16, 0);
    lv_obj_center(title_lbl);

    // ── Scrollable body panel (title=40px, buttons=54px → 146px tall) ──────────
    lv_obj_t *panel = lv_obj_create(scr);
    lv_obj_set_size(panel, SCREEN_WIDTH, SCREEN_HEIGHT - 40 - 54);
    lv_obj_set_pos(panel, 0, 40);
    lv_obj_set_style_bg_color(panel, lv_color_hex(COLOR_BG), 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(panel, 0, 0);
    lv_obj_set_style_radius(panel, 0, 0);
    lv_obj_set_style_pad_left(panel, 8, 0);
    lv_obj_set_style_pad_right(panel, 8, 0);
    lv_obj_set_style_pad_top(panel, 4, 0);
    lv_obj_set_style_pad_bottom(panel, 4, 0);
    lv_obj_add_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(panel, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(panel, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_bg_color(panel, lv_color_hex(COLOR_SUBTEXT), LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, LV_PART_SCROLLBAR);
    lv_obj_set_style_width(panel, 3, LV_PART_SCROLLBAR);
    lv_obj_set_style_radius(panel, 2, LV_PART_SCROLLBAR);

    lv_obj_t *body_lbl = lv_label_create(panel);
    lv_label_set_text(body_lbl, body);
    lv_obj_set_style_text_color(body_lbl, lv_color_hex(COLOR_SUBTEXT), 0);
    lv_obj_set_style_text_font(body_lbl, &Inconsolata_16, 0);
    lv_obj_set_style_text_line_space(body_lbl, 2, 0);
    lv_label_set_recolor(body_lbl, true);
    lv_obj_set_width(body_lbl, SCREEN_WIDTH - 24);
    lv_label_set_long_mode(body_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_align(body_lbl, LV_ALIGN_TOP_LEFT, 0, 0);

    // Cache body widget pointer for live updates
    _lv_body_label  = body_lbl;
    _lv_body_panel  = panel;
    _lv_badge_label = nullptr;

    lv_scr_load(scr);
    LVGL_UNLOCK();
}

void FactoryTest::_update_screen(const char *body, Result /*result*/)
{
    if (!_lv_body_label) return;
    if (!LVGL_LOCK(100)) return;
    lv_label_set_text(static_cast<lv_obj_t *>(_lv_body_label), body);
    LVGL_UNLOCK();
}

void FactoryTest::_add_verdict_buttons(bool auto_passed)
{
    _verdict = -1;
    if (!LVGL_LOCK(500)) return;

    lv_obj_t *scr = lv_scr_act();

    if (auto_passed) {
        // ── Single full-width green "PASS >> Continue" button ──────────────
        lv_obj_t *btn = lv_btn_create(scr);
        lv_obj_set_size(btn, 304, 44);
        lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, -6);
        lv_obj_set_style_bg_color(btn, lv_color_hex(COLOR_PASS), 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x27AE60), LV_STATE_PRESSED);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_set_style_radius(btn, 6, 0);
        lv_obj_add_event_cb(btn, _on_ok_btn, LV_EVENT_CLICKED, this);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, LV_SYMBOL_OK "  PASS >> Continue");
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(lbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(lbl);
    } else {
        // ── Ok button (green, left) ────────────────────────────────────────
        lv_obj_t *ok_btn = lv_btn_create(scr);
        lv_obj_set_size(ok_btn, 142, 44);
        lv_obj_align(ok_btn, LV_ALIGN_BOTTOM_LEFT, 8, -6);
        lv_obj_set_style_bg_color(ok_btn, lv_color_hex(COLOR_PASS), 0);
        lv_obj_set_style_bg_color(ok_btn, lv_color_hex(0x27AE60), LV_STATE_PRESSED);
        lv_obj_set_style_border_width(ok_btn, 0, 0);
        lv_obj_set_style_radius(ok_btn, 6, 0);
        lv_obj_add_event_cb(ok_btn, _on_ok_btn, LV_EVENT_CLICKED, this);
        lv_obj_t *ok_lbl = lv_label_create(ok_btn);
        lv_label_set_text(ok_lbl, LV_SYMBOL_OK "  Ok");
        lv_obj_set_style_text_font(ok_lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(ok_lbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(ok_lbl);

        // ── Failed button (red, right) ─────────────────────────────────────
        lv_obj_t *fail_btn = lv_btn_create(scr);
        lv_obj_set_size(fail_btn, 142, 44);
        lv_obj_align(fail_btn, LV_ALIGN_BOTTOM_RIGHT, -8, -6);
        lv_obj_set_style_bg_color(fail_btn, lv_color_hex(COLOR_FAIL), 0);
        lv_obj_set_style_bg_color(fail_btn, lv_color_hex(0xC0392B), LV_STATE_PRESSED);
        lv_obj_set_style_border_width(fail_btn, 0, 0);
        lv_obj_set_style_radius(fail_btn, 6, 0);
        lv_obj_add_event_cb(fail_btn, _on_fail_btn, LV_EVENT_CLICKED, this);
        lv_obj_t *fail_lbl = lv_label_create(fail_btn);
        lv_label_set_text(fail_lbl, LV_SYMBOL_CLOSE "  Failed");
        lv_obj_set_style_text_font(fail_lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(fail_lbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(fail_lbl);
    }

    LVGL_UNLOCK();
}

bool FactoryTest::_wait_verdict()
{
    _add_verdict_buttons(false);
    while (_verdict < 0) vTaskDelay(pdMS_TO_TICKS(50));
    return (_verdict == 1);
}

bool FactoryTest::_auto_or_verdict(bool auto_passed)
{
    _add_verdict_buttons(auto_passed);
    while (_verdict < 0) vTaskDelay(pdMS_TO_TICKS(50));
    return auto_passed || (_verdict == 1);
}

bool FactoryTest::_wait_touch(uint32_t wait_ms)
{
    Touch *touch = _board.get_touch();
    if (!touch) return false;

    const uint32_t poll_ms  = 50;
    uint32_t       elapsed  = 0;
    while (elapsed < wait_ms) {
        touch_point_t tp;
        touch->read(&tp);
        if (tp.pressed) return true;
        vTaskDelay(pdMS_TO_TICKS(poll_ms));
        elapsed += poll_ms;
    }
    return false;
}

void FactoryTest::_show_summary(const char * const names[], const Result results[], int count)
{
    int pass = 0, fail = 0, skip = 0;
    for (int i = 0; i < count; i++) {
        switch (results[i]) {
            case Result::PASS: pass++; break;
            case Result::FAIL: fail++; break;
            default:           skip++; break;
        }
    }

    // Build recolored table body: "%-13.13s  #COLOR RESULT#\n" per row
    char body[640];
    int  n = 0;
    for (int i = 0; i < count && n < (int)sizeof(body) - 48; i++) {
        const char *col, *tag;
        switch (results[i]) {
            case Result::PASS: col = "2ECC71"; tag = "OK    "; break;
            case Result::FAIL: col = "E74C3C"; tag = "FAILED"; break;
            default:           col = "7F8C8D"; tag = "SKIP  "; break;
        }
        n += snprintf(body + n, sizeof(body) - n,
                      "%-13.13s#%s %s#\n", names[i], col, tag);
    }

    if (!LVGL_LOCK(500)) return;

    lv_obj_t *scr = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr, lv_color_hex(COLOR_BG), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scr, 0, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // ── Title bar ─────────────────────────────────────────────────────────────
    lv_obj_t *title_bar = lv_obj_create(scr);
    lv_obj_set_size(title_bar, SCREEN_WIDTH, 40);
    lv_obj_align(title_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(title_bar, lv_color_hex(COLOR_TITLE_BG), 0);
    lv_obj_set_style_border_width(title_bar, 0, 0);
    lv_obj_set_style_radius(title_bar, 0, 0);
    lv_obj_clear_flag(title_bar, LV_OBJ_FLAG_SCROLLABLE);

    char title_buf[48];
    snprintf(title_buf, sizeof(title_buf),
             "Results  P:%d F:%d S:%d", pass, fail, skip);
    lv_obj_t *title_lbl = lv_label_create(title_bar);
    lv_label_set_text(title_lbl, title_buf);
    lv_obj_set_style_text_color(title_lbl,
        lv_color_hex(fail > 0 ? COLOR_FAIL : COLOR_PASS), 0);
    lv_obj_set_style_text_font(title_lbl, &lv_font_montserrat_16, 0);
    lv_obj_center(title_lbl);

    // ── Table body ────────────────────────────────────────────────────────────
    lv_obj_t *body_lbl = lv_label_create(scr);
    lv_label_set_text(body_lbl, body);
    lv_label_set_recolor(body_lbl, true);
    lv_obj_set_style_text_color(body_lbl, lv_color_hex(COLOR_TEXT), 0);
    lv_obj_set_style_text_font(body_lbl, &Inconsolata_16, 0);
    lv_obj_set_style_text_line_space(body_lbl, 2, 0);
    lv_obj_set_width(body_lbl, SCREEN_WIDTH - 16);
    lv_label_set_long_mode(body_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_align(body_lbl, LV_ALIGN_TOP_LEFT, 8, 44);

    // ── Bottom hint ───────────────────────────────────────────────────────────
    lv_obj_t *hint = lv_label_create(scr);
    lv_label_set_text(hint, "Tap screen to restart");
    lv_obj_set_style_text_color(hint, lv_color_hex(COLOR_SKIP), 0);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_14, 0);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -8);

    lv_scr_load(scr);
    LVGL_UNLOCK();

    LOG_I("FactoryTest summary: PASS=%d FAIL=%d SKIP=%d", pass, fail, skip);
    _wait_touch(60000);
    ESP.restart();
}

// ─── Test cases ──────────────────────────────────────────────────────────────

FactoryTest::Result FactoryTest::_test_display()
{
    _show_screen(1, "Display", "Filling colors...", Result::SKIP);

    struct { uint32_t color; const char *name; } steps[] = {
        { 0xFF0000, "Red"   },
        { 0x00FF00, "Green" },
        { 0x0000FF, "Blue"  },
        { 0xFFFFFF, "White" },
        { 0x000000, "Black" },
    };

    Display *disp = _board.get_display();
    if (!disp) {
        _update_screen("No display driver!", Result::FAIL);
        return _wait_verdict() ? Result::PASS : Result::FAIL;
    }

    for (auto &s : steps) {
        // Full-screen solid-colour overlay for visual check
        lv_obj_t *overlay = nullptr;
        if (LVGL_LOCK(200)) {
            overlay = lv_obj_create(lv_scr_act());
            lv_obj_set_size(overlay, SCREEN_WIDTH, SCREEN_HEIGHT);
            lv_obj_set_pos(overlay, 0, 0);
            lv_obj_set_style_bg_color(overlay, lv_color_hex(s.color), 0);
            lv_obj_set_style_bg_opa(overlay, LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(overlay, 0, 0);
            lv_obj_set_style_radius(overlay, 0, 0);
            lv_obj_set_style_pad_all(overlay, 0, 0);
            // Large centred colour-name label — monospace, white/black for contrast
            uint32_t txt_col = (s.color <= 0x0000FF || s.color == 0x000000) ? 0xFFFFFF : 0x000000;
            lv_obj_t *name_lbl = lv_label_create(overlay);
            lv_label_set_text(name_lbl, s.name);
            lv_obj_set_style_text_color(name_lbl, lv_color_hex(txt_col), 0);
            lv_obj_set_style_text_font(name_lbl, &Inconsolata_20, 0);
            lv_obj_align(name_lbl, LV_ALIGN_CENTER, 0, 0);
            LVGL_UNLOCK();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (overlay && LVGL_LOCK(200)) {
            lv_obj_del(overlay);
            LVGL_UNLOCK();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    _update_screen("Red / Green / Blue\nWhite / Black shown.\n\n"
                   "Did all 5 colors look\ncorrect?", Result::SKIP);
    return _wait_verdict() ? Result::PASS : Result::FAIL;
}

FactoryTest::Result FactoryTest::_test_touch()
{
    Touch *touch = _board.get_touch();
    _verdict = -1;

    // Canvas occupies the space between title bar (40px) and buttons (54px).
    constexpr uint16_t TITLE_H  = 40;
    constexpr uint16_t BTN_H    = 54;
    constexpr uint16_t CANVAS_W = SCREEN_WIDTH;
    constexpr uint16_t CANVAS_H = SCREEN_HEIGHT - TITLE_H - BTN_H;  // 146px

    // Allocate canvas pixel buffer in PSRAM (~91 KB for 320×146 RGB565)
    const size_t buf_sz = LV_CANVAS_BUF_SIZE_TRUE_COLOR(CANVAS_W, CANVAS_H);
    auto *cbuf = static_cast<lv_color_t *>(
        heap_caps_malloc(buf_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

    lv_obj_t *canvas   = nullptr;
    lv_obj_t *hint_lbl = nullptr;

    if (!LVGL_LOCK(500)) {
        if (cbuf) heap_caps_free(cbuf);
        return Result::SKIP;
    }

    // ── Screen ───────────────────────────────────────────────────────────────
    lv_obj_t *scr = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr, lv_color_hex(COLOR_BG), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scr, 0, 0);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // ── Title bar (bold Montserrat 16) ────────────────────────────────────────
    lv_obj_t *title_bar = lv_obj_create(scr);
    lv_obj_set_size(title_bar, SCREEN_WIDTH, TITLE_H);
    lv_obj_align(title_bar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(title_bar, lv_color_hex(COLOR_TITLE_BG), 0);
    lv_obj_set_style_border_width(title_bar, 0, 0);
    lv_obj_set_style_radius(title_bar, 0, 0);
    lv_obj_set_style_pad_all(title_bar, 4, 0);
    lv_obj_clear_flag(title_bar, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title_lbl = lv_label_create(title_bar);
    lv_label_set_text(title_lbl, "Test 2/9: Touch");
    lv_obj_set_style_text_color(title_lbl, lv_color_hex(COLOR_TEXT), 0);
    lv_obj_set_style_text_font(title_lbl, &lv_font_montserrat_16, 0);
    lv_obj_center(title_lbl);

    // ── Drawing canvas ────────────────────────────────────────────────────────
    if (cbuf) {
        canvas = lv_canvas_create(scr);
        lv_canvas_set_buffer(canvas, cbuf, CANVAS_W, CANVAS_H, LV_IMG_CF_TRUE_COLOR);
        lv_obj_set_pos(canvas, 0, TITLE_H);
        lv_canvas_fill_bg(canvas, lv_color_hex(0x0D0D20), LV_OPA_COVER);
    }

    // Hint label overlaid on the canvas area, hidden on first touch
    hint_lbl = lv_label_create(scr);
    lv_label_set_text(hint_lbl, touch ? "Draw with your finger..." : "No touch driver");
    lv_obj_set_style_text_color(hint_lbl, lv_color_hex(COLOR_SKIP), 0);
    lv_obj_set_style_text_font(hint_lbl, &Inconsolata_16, 0);
    lv_obj_set_width(hint_lbl, CANVAS_W);
    lv_obj_set_style_text_align(hint_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(hint_lbl, 0, TITLE_H + CANVAS_H / 2 - 5);

    // ── Ok / Failed buttons ───────────────────────────────────────────────────
    lv_obj_t *ok_btn = lv_btn_create(scr);
    lv_obj_set_size(ok_btn, 142, 44);
    lv_obj_align(ok_btn, LV_ALIGN_BOTTOM_LEFT, 8, -6);
    lv_obj_set_style_bg_color(ok_btn, lv_color_hex(COLOR_PASS), 0);
    lv_obj_set_style_bg_color(ok_btn, lv_color_hex(0x27AE60), LV_STATE_PRESSED);
    lv_obj_set_style_border_width(ok_btn, 0, 0);
    lv_obj_set_style_radius(ok_btn, 6, 0);
    lv_obj_add_event_cb(ok_btn, _on_ok_btn, LV_EVENT_CLICKED, this);
    lv_obj_t *ok_lbl = lv_label_create(ok_btn);
    lv_label_set_text(ok_lbl, LV_SYMBOL_OK "  Ok");
    lv_obj_set_style_text_font(ok_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(ok_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(ok_lbl);

    lv_obj_t *fail_btn = lv_btn_create(scr);
    lv_obj_set_size(fail_btn, 142, 44);
    lv_obj_align(fail_btn, LV_ALIGN_BOTTOM_RIGHT, -8, -6);
    lv_obj_set_style_bg_color(fail_btn, lv_color_hex(COLOR_FAIL), 0);
    lv_obj_set_style_bg_color(fail_btn, lv_color_hex(0xC0392B), LV_STATE_PRESSED);
    lv_obj_set_style_border_width(fail_btn, 0, 0);
    lv_obj_set_style_radius(fail_btn, 6, 0);
    lv_obj_add_event_cb(fail_btn, _on_fail_btn, LV_EVENT_CLICKED, this);
    lv_obj_t *fail_lbl = lv_label_create(fail_btn);
    lv_label_set_text(fail_lbl, LV_SYMBOL_CLOSE "  Failed");
    lv_obj_set_style_text_font(fail_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(fail_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(fail_lbl);

    lv_scr_load(scr);
    LVGL_UNLOCK();

    // ── Touch-drawing poll loop ───────────────────────────────────────────────
    // Smooth stroke: draw a thick line segment from the previous sample to the
    // current one, then cap each joint with a filled circle so there are no
    // gaps at direction changes.
    const uint32_t poll_ms = 16;   // ~60 Hz sampling
    bool  hint_hidden = false;
    bool  has_prev    = false;
    int   prev_cx = 0, prev_cy = 0;

    // Pre-init draw descriptors (reused every frame to avoid stack churn)
    lv_draw_line_dsc_t  ldsc;
    lv_draw_line_dsc_init(&ldsc);
    ldsc.color      = lv_color_hex(0x00E5AA);
    ldsc.width      = 3;
    ldsc.round_start = 1;
    ldsc.round_end   = 1;
    ldsc.opa         = LV_OPA_COVER;

    lv_draw_rect_dsc_t  rdsc;
    lv_draw_rect_dsc_init(&rdsc);
    rdsc.bg_color = lv_color_hex(0x00E5AA);
    rdsc.bg_opa   = LV_OPA_COVER;
    rdsc.radius   = LV_RADIUS_CIRCLE;

    while (_verdict < 0) {
        if (touch && canvas && cbuf) {
            touch_point_t tp;
            touch->read(&tp);

            if (tp.pressed) {
                int cx = (int)tp.x;
                int cy = (int)tp.y - TITLE_H;

                // Clamp to canvas bounds
                if (cx < 0) cx = 0;
                if (cx >= CANVAS_W) cx = CANVAS_W - 1;
                if (cy < 0) cy = 0;
                if (cy >= CANVAS_H) cy = CANVAS_H - 1;

                if (LVGL_LOCK(10)) {
                    if (!hint_hidden) {
                        lv_obj_add_flag(hint_lbl, LV_OBJ_FLAG_HIDDEN);
                        hint_hidden = true;
                    }

                    if (has_prev) {
                        // Draw segment from previous → current (round caps)
                        lv_point_t pts[2] = {
                            { (lv_coord_t)prev_cx, (lv_coord_t)prev_cy },
                            { (lv_coord_t)cx,      (lv_coord_t)cy      }
                        };
                        lv_canvas_draw_line(canvas, pts, 2, &ldsc);
                    } else {
                        // First point: just draw a filled circle dot
                        int r = ldsc.width / 2;
                        lv_canvas_draw_rect(canvas,
                            cx - r, cy - r,
                            ldsc.width, ldsc.width, &rdsc);
                    }

                    has_prev = true;
                    prev_cx  = cx;
                    prev_cy  = cy;
                    LVGL_UNLOCK();
                }
            } else {
                // Finger lifted — break the stroke so the next touch starts fresh
                has_prev = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(poll_ms));
    }

    bool passed = (_verdict == 1);
    vTaskDelay(pdMS_TO_TICKS(80));   // let LVGL flush the last frame before cbuf is freed
    if (cbuf) heap_caps_free(cbuf);
    return passed ? Result::PASS : Result::FAIL;
}

FactoryTest::Result FactoryTest::_test_sdcard()
{
    _show_screen(3, "SD Card", "Mounting...", Result::SKIP);

    SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN);
    if (!SD_MMC.begin("/sdcard", true)) {   // true = 1-bit mode
        _update_screen("Mount        #E74C3C FAIL #\n(SD inserted?)", Result::FAIL);
        return _auto_or_verdict(false) ? Result::PASS : Result::FAIL;
    }

    // Card info
    uint8_t  cardType = SD_MMC.cardType();
    uint64_t cardBytes = SD_MMC.cardSize();
    float    capGB = (float)cardBytes / (1024.0f * 1024.0f * 1024.0f);
    const char *typeStr;
    switch (cardType) {
        case CARD_MMC:  typeStr = "MMC";  break;
        case CARD_SD:   typeStr = "SDSC"; break;
        case CARD_SDHC: typeStr = "SDHC"; break;
        default:        typeStr = "UNKN"; break;
    }

    char l_type[40], l_cap[40], body_buf[384];
    snprintf(l_type, sizeof(l_type), "%-13s%s",   "Type",     typeStr);
    snprintf(l_cap,  sizeof(l_cap),  "%-13s%.2f GB", "Capacity", capGB);
    snprintf(body_buf, sizeof(body_buf),
             "%-13s#2ECC71 OK   #\n%s\n%s\n%-13s...",
             "Mount", l_type, l_cap, "Write");
    _update_screen(body_buf, Result::SKIP);

    // ── 512 KiB write benchmark ───────────────────────────────────────────────
    const char *test_path = "/factory_test.tmp";   // path relative to SD_MMC mount point
    constexpr uint32_t BLOCK_SZ    = 4096u;
    constexpr uint32_t BENCH_BYTES = 512u * 1024u;

    static uint8_t wbuf[BLOCK_SZ];
    for (uint32_t i = 0; i < BLOCK_SZ; i++) wbuf[i] = (uint8_t)(i ^ (i >> 8));

    if (SD_MMC.exists(test_path)) SD_MMC.remove(test_path);
    File f = SD_MMC.open(test_path, FILE_WRITE);
    if (!f) {
        snprintf(body_buf, sizeof(body_buf),
                 "%-13s#2ECC71 OK   #\n%s\n%s\n%-13s#E74C3C FAIL #",
                 "Mount", l_type, l_cap, "Write open");
        _update_screen(body_buf, Result::FAIL);
        SD_MMC.end();
        return _auto_or_verdict(false) ? Result::PASS : Result::FAIL;
    }

    uint32_t written = 0;
    uint32_t tw0 = millis();
    while (written < BENCH_BYTES) {
        size_t n = f.write(wbuf, BLOCK_SZ);
        if (n != BLOCK_SZ) break;
        written += n;
    }
    f.flush(); f.close();
    uint32_t twDt = millis() - tw0;
    float writeMBs = (twDt > 0)
        ? ((float)written / (1048576.0f)) / ((float)twDt / 1000.0f) : 0.0f;
    bool writeOk = (written == BENCH_BYTES);

    // ── 512 KiB read + verify ─────────────────────────────────────────────────
    static uint8_t rbuf[BLOCK_SZ];
    bool readOk = false, verifyOk = false;
    float readMBs = 0.0f;
    uint32_t readBytes = 0;

    f = SD_MMC.open(test_path, FILE_READ);
    if (f) {
        verifyOk = true;
        uint32_t tr0 = millis();
        while (readBytes < BENCH_BYTES) {
            size_t n = f.read(rbuf, BLOCK_SZ);
            if (n != BLOCK_SZ) break;
            if (memcmp(rbuf, wbuf, BLOCK_SZ) != 0) { verifyOk = false; break; }
            readBytes += n;
        }
        uint32_t trDt = millis() - tr0;
        f.close();
        readOk = (readBytes == BENCH_BYTES);
        readMBs = (trDt > 0)
            ? ((float)readBytes / (1048576.0f)) / ((float)trDt / 1000.0f) : 0.0f;
    }

    SD_MMC.remove(test_path);
    SD_MMC.end();

    // ── Format result table ───────────────────────────────────────────────────
    char vWr[20], vRd[20];
    snprintf(vWr, sizeof(vWr), "%.2f MB/s", writeMBs);
    snprintf(vRd, sizeof(vRd), "%.2f MB/s", readMBs);
    const char *wCol = writeOk  ? "2ECC71" : "E74C3C";
    const char *rCol = readOk   ? "2ECC71" : "E74C3C";
    const char *vCol = verifyOk ? "2ECC71" : "E74C3C";

    snprintf(body_buf, sizeof(body_buf),
             "%-13s#2ECC71 OK   #\n"
             "%s\n%s\n"
             "%-13s#%s %s#\n"
             "%-13s#%s %s#\n"
             "%-13s#%s %s#",
             "Mount", l_type, l_cap,
             "Write",  wCol, writeOk  ? vWr : "FAIL ",
             "Read",   rCol, readOk   ? vRd : "FAIL ",
             "Verify", vCol, verifyOk ? "OK   " : "FAIL ");

    bool allOk = writeOk && readOk && verifyOk;
    _update_screen(body_buf, allOk ? Result::PASS : Result::FAIL);
    return _auto_or_verdict(allOk) ? Result::PASS : Result::FAIL;
}

FactoryTest::Result FactoryTest::_test_wifi()
{
    _show_screen(4, "WiFi", "Scanning...", Result::SKIP);

    // Body label already uses Inconsolata_16 + recolor from _show_screen().
    // UNSCII_16 font switch removed — Inconsolata_16 is already set.
    constexpr int MAX_APS = 6;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));

    int n = WiFi.scanNetworks(false, true);   // sync scan, include hidden
    WiFi.mode(WIFI_OFF);

    if (n <= 0) {
        _update_screen("Scan         #E74C3C FAIL #\nNo APs found", Result::FAIL);
        return _auto_or_verdict(false) ? Result::PASS : Result::FAIL;
    }

    char msg[384];
    int written = snprintf(msg, sizeof(msg),
                           "Scan         #2ECC71 OK   #\nFound %-3d APs\n", n);

    int shown = 0;
    for (int i = 0; i < n && shown < MAX_APS && written < (int)sizeof(msg) - 60; i++) {
        // Skip APs with empty SSID (hidden networks without a broadcast name)
        String ssid = WiFi.SSID(i);
        if (ssid.length() == 0) continue;
        int rssi = WiFi.RSSI(i);
        // RSSI colour: >= -60 excellent(green), >= -70 good(yellow),
        //              >= -80 fair(orange), < -80 poor(red)
        const char *rssiCol;
        if      (rssi >= -60) rssiCol = "2ECC71";
        else if (rssi >= -70) rssiCol = "F1C40F";
        else if (rssi >= -80) rssiCol = "E67E22";
        else                  rssiCol = "E74C3C";
        written += snprintf(msg + written, sizeof(msg) - written,
                            "%-16.16s #%s %4ddBm#\n",
                            ssid.c_str(), rssiCol, rssi);
        shown++;
    }
    WiFi.scanDelete();

    _update_screen(msg, Result::PASS);
    return _auto_or_verdict(shown > 0) ? Result::PASS : Result::FAIL;
}

// Helper: probe I2C address, return true if ACK received.
static bool i2c_probe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

FactoryTest::Result FactoryTest::_test_imu()
{
    // QMI8658 default address: 0x6B (SA0 tied high) or 0x6A.
    constexpr uint8_t QMI8658_ADDR = 0x6B;
    _show_screen(5, "IMU (QMI8658)", "Probing I2C...", Result::SKIP);

    // Helper: show error text in red, centered in the test body area.
    auto show_error = [&](const char *text) {
        if (!LVGL_LOCK(500)) return;
        lv_obj_t *lbl   = static_cast<lv_obj_t *>(_lv_body_label);
        lv_obj_t *panel = static_cast<lv_obj_t *>(_lv_body_panel);
        lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
        lv_label_set_text(lbl, text);
        lv_obj_set_style_text_color(lbl, lv_color_hex(COLOR_FAIL), 0);
        lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_width(lbl, SCREEN_WIDTH - 24);
        lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);
        LVGL_UNLOCK();
    };

    // ── Step 1: I2C probe ─────────────────────────────────────────────────
    if (!i2c_probe(QMI8658_ADDR)) {
        show_error("IMU Not Found\n\nAddr:  0x6B\nNo I2C ACK");
        return _auto_or_verdict(false) ? Result::PASS : Result::FAIL;
    }

    // ── Step 2: WHO_AM_I check ────────────────────────────────────────────
    Wire.beginTransmission(QMI8658_ADDR);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(QMI8658_ADDR, (uint8_t)1);
    uint8_t who = Wire.available() ? Wire.read() : 0xFF;

    if (who != 0x05) {
        char err[80];
        snprintf(err, sizeof(err),
                 "IMU ID Mismatch\n\nAddr:   0x6B\nID:     0x%02X\nExpect: 0x05", who);
        show_error(err);
        return _auto_or_verdict(false) ? Result::PASS : Result::FAIL;
    }

    // ── Step 3: Initialize QMI8658 ────────────────────────────────────────
    auto write_reg = [&](uint8_t reg, uint8_t val) {
        Wire.beginTransmission(QMI8658_ADDR);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    };

    write_reg(0x02, 0x60);   // CTRL1: auto-increment, little-endian output
    write_reg(0x03, 0x25);   // CTRL2: Accel ±8g (aFS=010), ODR=500Hz
    write_reg(0x04, 0x55);   // CTRL3: Gyro ±512dps (gFS=101), ODR=500Hz
    write_reg(0x08, 0x03);   // CTRL7: AccEn | GyrEn
    vTaskDelay(pdMS_TO_TICKS(50));

    // ±8g  → 32768/8  = 4096 LSB/g  → scale = 1/4096
    // ±512dps → 32768/512 = 64 LSB/dps → scale = 1/64
    constexpr float ACC_SCALE  = 1.0f / 4096.0f;      // LSB → g
    constexpr float GYR_SCALE  = 1.0f / 64.0f;        // LSB → dps
    // DEG_TO_RAD / RAD_TO_DEG are already defined as macros in Arduino.h
    constexpr float kDeg2Rad = 0.017453293f;
    constexpr float kRad2Deg = 57.295779f;

    // Burst-read helper
    auto read_raw = [&](int16_t &ax, int16_t &ay, int16_t &az,
                        int16_t &gx, int16_t &gy, int16_t &gz) {
        Wire.beginTransmission(QMI8658_ADDR);
        Wire.write(0x35);
        Wire.endTransmission(false);
        Wire.requestFrom(QMI8658_ADDR, (uint8_t)12);
        ax = ay = az = gx = gy = gz = 0;
        if (Wire.available() >= 12) {
            uint8_t b[12];
            for (int i = 0; i < 12; i++) b[i] = (uint8_t)Wire.read();
            ax = (int16_t)((uint16_t)(b[1]<<8)|b[0]);
            ay = (int16_t)((uint16_t)(b[3]<<8)|b[2]);
            az = (int16_t)((uint16_t)(b[5]<<8)|b[4]);
            gx = (int16_t)((uint16_t)(b[7]<<8)|b[6]);
            gy = (int16_t)((uint16_t)(b[9]<<8)|b[8]);
            gz = (int16_t)((uint16_t)(b[11]<<8)|b[10]);
        }
    };

    // ── Step 4: Gyro zero-bias calibration (keep still ~2 s) ─────────────
    // 100 samples × 20 ms = 2 s
    constexpr uint16_t CALIB_N = 100;
    ImuBias  bias(CALIB_N);
    MahonyAHRS ahrs(1.0f, 0.005f, 0.02f);  // Kp, Ki, dt_s

    {
        char prog[64];
        for (uint16_t i = 0; i < CALIB_N; i++) {
            int16_t rax, ray, raz, rgx, rgy, rgz;
            read_raw(rax, ray, raz, rgx, rgy, rgz);
            bias.feed(rgx * GYR_SCALE, rgy * GYR_SCALE, rgz * GYR_SCALE);
            if ((i & 0x0F) == 0) {
                int pct = (int)(i * 100 / CALIB_N);
                snprintf(prog, sizeof(prog),
                         "Keep still...\nCalibrating gyro ZRO\n[%d%%]", pct);
                _update_screen(prog, Result::SKIP);
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

    // Float formatters (avoid %f)
    // fmt_a: ±N.NNN  (6 chars) — accel in g (0–8 g)
    auto fmt_a = [](char *b, int sz, float v) {
        bool neg = (v < 0.0f);  float av = neg ? -v : v;
        int iv = (int)av;
        int fv = (int)((av - (float)iv) * 1000.0f + 0.5f);
        if (fv >= 1000) { iv++; fv = 0; }
        snprintf(b, sz, "%c%d.%03d", neg?'-':'+', iv, fv);
    };
    // fmt_b: ±NNN.N  (6 chars) — gyro dps or attitude deg
    auto fmt_b = [](char *b, int sz, float v) {
        bool neg = (v < 0.0f);  float av = neg ? -v : v;
        int iv = (int)av;
        int fv = (int)((av - (float)iv) * 10.0f + 0.5f);
        if (fv >= 10) { iv++; fv = 0; }
        snprintf(b, sz, "%c%3d.%1d", neg?'-':'+', iv, fv);
    };

    // ── Warm-start: snap Mahony to accel-derived tilt in ~1 s instead of ~10 s ─
    // Feed 100 zero-gyro iterations using the corrected accel sample so the
    // quaternion approaches the true tilt before the live-view starts.
    // Each step moves the quat by Kp*dt ≈ 0.02 rad; 100 steps covers ~90°.
    {
        int16_t ax0, ay0, az0, gx0, gy0, gz0;
        read_raw(ax0, ay0, az0, gx0, gy0, gz0);
        float nx = ax0 * ACC_SCALE;
        float ny = ay0 * ACC_SCALE;
        float nz = -az0 * ACC_SCALE;   // same Z-inversion as main loop
        for (int wi = 0; wi < 100; wi++) ahrs.update(0.f, 0.f, 0.f, nx, ny, nz);
    }

    // ── Step 5: Live view — RAW table (default) or AHRS display (toggle) ─
    constexpr uint16_t TITLE_H = 40;
    constexpr uint16_t BTN_H   = 54;
    constexpr uint16_t BODY_H  = SCREEN_HEIGHT - TITLE_H - BTN_H; // 146

    // Allocate 3D canvas pixel buffer from PSRAM (~91 KB for 320×146 RGB565)
    const size_t cbuf_sz = LV_CANVAS_BUF_SIZE_TRUE_COLOR(SCREEN_WIDTH, BODY_H);
    auto *cbuf3d = static_cast<lv_color_t *>(
        heap_caps_malloc(cbuf_sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));

    // Live-view widget handles (set during screen build, used in the loop)
    // Default to 3D mode when PSRAM buffer is available, otherwise fall back to RAW
    volatile bool show_3d  = (cbuf3d != nullptr);
    lv_obj_t *canvas3d     = nullptr;
    lv_obj_t *tbl_panel    = nullptr;
    lv_obj_t *tbl_lbl      = nullptr;
    ImuToggleCtx toggle_ctx = {};   // on-stack; valid for the entire function

    // ── Build custom live-view screen ──────────────────────────────────────
    if (!LVGL_LOCK(500)) {
        if (cbuf3d) heap_caps_free(cbuf3d);
        return Result::SKIP;
    }

    {
        lv_obj_t *scr = lv_obj_create(nullptr);
        lv_obj_set_style_bg_color(scr, lv_color_hex(COLOR_BG), 0);
        lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(scr, 0, 0);
        lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

        // Title bar
        lv_obj_t *tb = lv_obj_create(scr);
        lv_obj_set_size(tb, SCREEN_WIDTH, TITLE_H);
        lv_obj_align(tb, LV_ALIGN_TOP_MID, 0, 0);
        lv_obj_set_style_bg_color(tb, lv_color_hex(COLOR_TITLE_BG), 0);
        lv_obj_set_style_border_width(tb, 0, 0);
        lv_obj_set_style_radius(tb, 0, 0);
        lv_obj_set_style_pad_all(tb, 4, 0);
        lv_obj_clear_flag(tb, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t *tb_lbl = lv_label_create(tb);
        lv_label_set_text(tb_lbl, "Test 5/9: IMU (QMI8658)");
        lv_obj_set_style_text_color(tb_lbl, lv_color_hex(COLOR_TEXT), 0);
        lv_obj_set_style_text_font(tb_lbl, &lv_font_montserrat_16, 0);
        lv_obj_center(tb_lbl);

        // ── RAW table panel (default: hidden when 3D mode is active) ───────────
        tbl_panel = lv_obj_create(scr);
        lv_obj_set_size(tbl_panel, SCREEN_WIDTH, BODY_H);
        lv_obj_set_pos(tbl_panel, 0, TITLE_H);
        lv_obj_set_style_bg_color(tbl_panel, lv_color_hex(COLOR_BG), 0);
        lv_obj_set_style_bg_opa(tbl_panel, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(tbl_panel, 0, 0);
        lv_obj_set_style_radius(tbl_panel, 0, 0);
        lv_obj_set_style_pad_left(tbl_panel, 8, 0);
        lv_obj_set_style_pad_right(tbl_panel, 8, 0);
        lv_obj_set_style_pad_top(tbl_panel, 4, 0);
        lv_obj_set_style_pad_bottom(tbl_panel, 4, 0);
        lv_obj_clear_flag(tbl_panel, LV_OBJ_FLAG_SCROLLABLE);
        if (show_3d) lv_obj_add_flag(tbl_panel, LV_OBJ_FLAG_HIDDEN);  // hide when starting in 3D

        tbl_lbl = lv_label_create(tbl_panel);
        lv_label_set_text(tbl_lbl, "");
        lv_obj_set_style_text_color(tbl_lbl, lv_color_hex(COLOR_SUBTEXT), 0);
        lv_obj_set_style_text_font(tbl_lbl, &Inconsolata_16, 0);
        lv_obj_set_style_text_line_space(tbl_lbl, 2, 0);
        lv_obj_set_width(tbl_lbl, SCREEN_WIDTH - 24);
        lv_label_set_long_mode(tbl_lbl, LV_LABEL_LONG_WRAP);
        lv_obj_align(tbl_lbl, LV_ALIGN_TOP_LEFT, 0, 0);

        // ── 3D canvas (initially hidden) ─────────────────────────────────────
        if (cbuf3d) {
            canvas3d = lv_canvas_create(scr);
            lv_canvas_set_buffer(canvas3d, cbuf3d, SCREEN_WIDTH, BODY_H,
                                 LV_IMG_CF_TRUE_COLOR);
            lv_obj_set_pos(canvas3d, 0, TITLE_H);
            lv_canvas_fill_bg(canvas3d, lv_color_hex(0x0D1A30), LV_OPA_COVER);
            // Visible by default (we start in 3D mode); hidden only if fallback to RAW
            if (!show_3d) lv_obj_add_flag(canvas3d, LV_OBJ_FLAG_HIDDEN);
        }

        // ── Bottom button row: [3D 80px] [gap 8px] [PASS >> Continue 216px] ─
        // Toggle button (left, blue)
        lv_obj_t *tog_btn = lv_btn_create(scr);
        lv_obj_set_size(tog_btn, 80, 44);
        lv_obj_align(tog_btn, LV_ALIGN_BOTTOM_LEFT, 8, -6);
        lv_obj_set_style_bg_color(tog_btn, lv_color_hex(0x2980B9), 0);
        lv_obj_set_style_bg_color(tog_btn, lv_color_hex(0x1F618D), LV_STATE_PRESSED);
        lv_obj_set_style_border_width(tog_btn, 0, 0);
        lv_obj_set_style_radius(tog_btn, 6, 0);
        lv_obj_t *tog_lbl = lv_label_create(tog_btn);
        lv_label_set_text(tog_lbl, show_3d ? "RAW" : "AHRS");  // label reflects current mode
        lv_obj_set_style_text_font(tog_lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(tog_lbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(tog_lbl);

        toggle_ctx = {&show_3d, tog_lbl, canvas3d, tbl_panel};
        lv_obj_add_event_cb(tog_btn, _on_imu_toggle_btn, LV_EVENT_CLICKED, &toggle_ctx);

        // PASS button (right, green)
        _verdict = -1;
        lv_obj_t *pass_btn = lv_btn_create(scr);
        lv_obj_set_size(pass_btn, 216, 44);
        lv_obj_align(pass_btn, LV_ALIGN_BOTTOM_RIGHT, -8, -6);
        lv_obj_set_style_bg_color(pass_btn, lv_color_hex(COLOR_PASS), 0);
        lv_obj_set_style_bg_color(pass_btn, lv_color_hex(0x27AE60), LV_STATE_PRESSED);
        lv_obj_set_style_border_width(pass_btn, 0, 0);
        lv_obj_set_style_radius(pass_btn, 6, 0);
        lv_obj_add_event_cb(pass_btn, _on_ok_btn, LV_EVENT_CLICKED, this);
        lv_obj_t *pass_lbl = lv_label_create(pass_btn);
        lv_label_set_text(pass_lbl, LV_SYMBOL_OK "  PASS >> Continue");
        lv_obj_set_style_text_font(pass_lbl, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(pass_lbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_center(pass_lbl);

        lv_scr_load(scr);
    }
    LVGL_UNLOCK();

    // ── Main refresh loop (20 ms) ──────────────────────────────────────────
    while (_verdict < 0) {
        int16_t rax, ray, raz, rgx, rgy, rgz;
        read_raw(rax, ray, raz, rgx, rgy, rgz);

        float fax = rax * ACC_SCALE;
        float fay = ray * ACC_SCALE;
        float faz = -raz * ACC_SCALE;   // QMI8658 Z-axis is physically inverted on this PCB
        float fgx = rgx * GYR_SCALE;
        float fgy = rgy * GYR_SCALE;
        float fgz = rgz * GYR_SCALE;

        // Bias-corrected Mahony update
        float bgx = fgx, bgy = fgy, bgz = fgz;
        bias.apply(bgx, bgy, bgz);
        ahrs.update(bgx * kDeg2Rad, bgy * kDeg2Rad, bgz * kDeg2Rad,
                    fax, fay, faz);

        if (LVGL_LOCK(10)) {
            if (show_3d && canvas3d) {
                // ── AHRS mode ──────────────────────────────────────────────
                // Physical calibration: IMU pitch/roll axes are swapped
                Euler e3 = ahrs.euler();
                _draw_ahrs_frame(canvas3d, e3.roll, e3.pitch, e3.yaw);
            } else if (!show_3d && tbl_lbl) {
                // ── RAW table mode ─────────────────────────────────────────
                Euler e = ahrs.euler();
                float roll_d  = e.roll  * kRad2Deg;
                float pitch_d = e.pitch * kRad2Deg;
                float yaw_d   = e.yaw   * kRad2Deg;

                char sa[3][10], sg[3][10], se[3][10];
                fmt_a(sa[0], sizeof(sa[0]), fax);
                fmt_a(sa[1], sizeof(sa[1]), fay);
                fmt_a(sa[2], sizeof(sa[2]), faz);
                fmt_b(sg[0], sizeof(sg[0]), fgx);
                fmt_b(sg[1], sizeof(sg[1]), fgy);
                fmt_b(sg[2], sizeof(sg[2]), fgz);
                fmt_b(se[0], sizeof(se[0]), roll_d);
                fmt_b(se[1], sizeof(se[1]), pitch_d);
                fmt_b(se[2], sizeof(se[2]), yaw_d);

                char msg[240];
                snprintf(msg, sizeof(msg),
                         "Acc/g         Gyr/dps     RPY/deg\n"
                         "X:%s      %s      %s  (Roll)\n"
                         "Y:%s      %s      %s  (Pitch)\n"
                         "Z:%s      %s      %s  (Yaw)",
                         sa[0], sg[0], se[0],
                         sa[1], sg[1], se[1],
                         sa[2], sg[2], se[2]);
                lv_label_set_text(tbl_lbl, msg);
            }
            LVGL_UNLOCK();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    vTaskDelay(pdMS_TO_TICKS(80));   // let LVGL flush last frame before freeing
    if (cbuf3d) heap_caps_free(cbuf3d);
    return Result::PASS;
}

FactoryTest::Result FactoryTest::_test_pmu()
{
    _show_screen(6, "PMU (AXP2101)", "Probing I2C...", Result::SKIP);

    // AXP2101 I2C address: 0x34 (ADDR=GND default) or 0x35 (ADDR=VCC alt)
    constexpr uint8_t AXP2101_ADDR_A = 0x34;
    constexpr uint8_t AXP2101_ADDR_B = 0x35;
    uint8_t found_addr = 0;
    if      (i2c_probe(AXP2101_ADDR_A)) found_addr = AXP2101_ADDR_A;
    else if (i2c_probe(AXP2101_ADDR_B)) found_addr = AXP2101_ADDR_B;

    if (!found_addr) {
        _update_screen("I2C 0x34/35  #E74C3C FAIL #\nNo ACK on either addr", Result::FAIL);
        return _auto_or_verdict(false) ? Result::PASS : Result::FAIL;
    }

    // I2C register helpers
    auto read_reg = [&](uint8_t reg) -> uint8_t {
        Wire.beginTransmission(found_addr);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(found_addr, (uint8_t)1);
        return Wire.available() ? Wire.read() : 0xFF;
    };
    auto write_reg = [&](uint8_t reg, uint8_t val) {
        Wire.beginTransmission(found_addr);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    };

    // ── One-time static reads ──────────────────────────────────────────────
    // Chip ID — reg 0x03 (IC_TYPE)
    // AXP2101 standard: 0x4A;  AXP2101B/alt variant: 0x47
    uint8_t chip_id = read_reg(0x03);
    bool id_ok = (chip_id == 0x4A || chip_id == 0x47);

    // Enable all ADC channels (reg 0x30 = ADC_CHANNEL_CTRL, write 0xFF)
    write_reg(0x30, 0xFF);
    // Also ensure fuel-gauge (coulomb counter) is active: reg 0x18 bit 2
    uint8_t fg_ctrl = read_reg(0x18);
    write_reg(0x18, fg_ctrl | 0x04);
    vTaskDelay(pdMS_TO_TICKS(100));   // let ADC complete first conversion

    // ── Static: LDO/DCDC config voltage registers (read once, don't change) ─
    // DCDC1: reg 0x82, bits[4:0], step 100mV from 1500mV
    uint16_t dc1_mv   = (uint16_t)((read_reg(0x82) & 0x1F) * 100u + 1500u);
    // ALDO1-4: regs 0x92-0x95, bits[4:0], step 100mV from 500mV
    auto ldo_mv = [&](uint8_t reg) -> uint16_t {
        return (uint16_t)((read_reg(reg) & 0x1F) * 100u + 500u);
    };
    uint16_t aldo1_mv = ldo_mv(0x92);
    uint16_t aldo2_mv = ldo_mv(0x93);
    uint16_t aldo3_mv = ldo_mv(0x94);
    uint16_t aldo4_mv = ldo_mv(0x95);
    // BLDO1-2: regs 0x96-0x97, same formula
    uint16_t bldo1_mv = ldo_mv(0x96);
    uint16_t bldo2_mv = ldo_mv(0x97);

    // ── Add verdict buttons (non-blocking) ────────────────────────────────
    _add_verdict_buttons(id_ok);

    // ── 100 ms real-time refresh loop ─────────────────────────────────────
    while (_verdict < 0) {
        // STATUS1 (0x00): bit5=VBUS_GOOD
        uint8_t s1       = read_reg(0x00);
        // STATUS2 (0x01): bit[2:0]=charge state
        uint8_t s2       = read_reg(0x01);
        bool vbus_good   = (s1 & (1u << 5)) != 0;
        uint8_t chg_stat = s2 & 0x07;
        // Charging when state is 1(Trickle)..5(Done)
        bool is_charging = (chg_stat >= 1 && chg_stat <= 5);

        // VBAT : H5L8 from 0x34/0x35 (13-bit, 1 mV/LSB)
        uint16_t vbat    = (uint16_t)(((uint16_t)(read_reg(0x34) & 0x1F) << 8) | read_reg(0x35));
        // VSYS : H6L8 from 0x36/0x37 (14-bit, 1 mV/LSB)
        uint16_t vsys    = (uint16_t)(((uint16_t)(read_reg(0x36) & 0x3F) << 8) | read_reg(0x37));
        // VBUS : H6L8 from 0x38/0x39 (14-bit, 1 mV/LSB)
        uint16_t vbus_mv = (uint16_t)(((uint16_t)(read_reg(0x38) & 0x3F) << 8) | read_reg(0x39));
        // Die Temp: formula from XPowersLib: 22.0 + (7274 - raw) / 20.0
        uint16_t ts_raw  = (uint16_t)(((uint16_t)(read_reg(0x3C) & 0x3F) << 8) | read_reg(0x3D));
        float temp_c     = 22.0f + (7274.0f - (float)ts_raw) / 20.0f;
        bool temp_valid  = (temp_c >= -10.0f && temp_c <= 125.0f);
        // Battery percent from fuel gauge (reg 0xA4, 0–100)
        uint8_t bat_pct  = read_reg(0xA4);

        char msg[1200];
        int  n = 0;

        // Error header: only when chip ID is wrong
        if (!id_ok) {
            n += snprintf(msg+n, sizeof(msg)-n,
                "Chip ID      #E74C3C 0x%02X FAIL# (exp 4A/47)\n\n", chip_id);
        }

        // ── Dynamic section ──────────────────────────────────────────────
        n += snprintf(msg+n, sizeof(msg)-n,
            "isCharging   #%s %-3s#\n",
            is_charging ? "2ECC71" : "BDC3C7", is_charging ? "YES" : "NO ");
        n += snprintf(msg+n, sizeof(msg)-n,
            "isVbusIn     #%s %-3s#\n",
            vbus_good ? "2ECC71" : "BDC3C7", vbus_good ? "YES" : "NO ");
        n += snprintf(msg+n, sizeof(msg)-n,
            "BatteryPct   %d %%\n",  bat_pct);
        n += snprintf(msg+n, sizeof(msg)-n,
            "BatteryVolt  %u mV\n",  vbat);
        if (vbus_good && vbus_mv > 0)
            n += snprintf(msg+n, sizeof(msg)-n,
                "VbusVoltage  %u mV\n", vbus_mv);
        n += snprintf(msg+n, sizeof(msg)-n,
            "SystemVolt   %u mV\n",  vsys);
        if (temp_valid) {
            int t_int  = (int)temp_c;
            int t_frac = (int)((temp_c - (float)t_int) * 10.0f + 0.5f);
            if (t_frac >= 10) { t_int++; t_frac = 0; }
            n += snprintf(msg+n, sizeof(msg)-n,
                "DieTemp      %d.%d C\n", t_int, t_frac);
        }

        // ── Static: power rail config voltages ───────────────────────────
        n += snprintf(msg+n, sizeof(msg)-n, "\n");
        n += snprintf(msg+n, sizeof(msg)-n, "DC1Voltage   %u mV\n",  dc1_mv);
        n += snprintf(msg+n, sizeof(msg)-n, "ALDO1Voltage %u mV\n",  aldo1_mv);
        n += snprintf(msg+n, sizeof(msg)-n, "ALDO2Voltage %u mV\n",  aldo2_mv);
        n += snprintf(msg+n, sizeof(msg)-n, "ALDO3Voltage %u mV\n",  aldo3_mv);
        n += snprintf(msg+n, sizeof(msg)-n, "ALDO4Voltage %u mV\n",  aldo4_mv);
        n += snprintf(msg+n, sizeof(msg)-n, "BLDO1Voltage %u mV\n",  bldo1_mv);
        n += snprintf(msg+n, sizeof(msg)-n, "BLDO2Voltage %u mV\n",  bldo2_mv);

        _update_screen(msg, id_ok ? Result::PASS : Result::FAIL);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return (id_ok || _verdict == 1) ? Result::PASS : Result::FAIL;
}

FactoryTest::Result FactoryTest::_test_rtc()
{
    // PCF85063 I2C address: 0x51.
    constexpr uint8_t PCF85063_ADDR = 0x51;
    _show_screen(7, "RTC (PCF85063)", "Probing I2C...", Result::SKIP);

    if (!i2c_probe(PCF85063_ADDR)) {
        _update_screen("I2C 0x51     #E74C3C FAIL #\nNo ACK", Result::FAIL);
        return _auto_or_verdict(false) ? Result::PASS : Result::FAIL;
    }

    // Read seconds register (0x04) twice 1s apart; it should increment.
    auto read_seconds = [&]() -> uint8_t {
        Wire.beginTransmission(PCF85063_ADDR);
        Wire.write(0x04);
        Wire.endTransmission(false);
        Wire.requestFrom(PCF85063_ADDR, (uint8_t)1);
        uint8_t v = Wire.available() ? Wire.read() : 0xFF;
        return v & 0x7F;   // strip OS bit
    };

    uint8_t s1 = read_seconds();
    _update_screen("I2C 0x51     #2ECC71 OK   #\nWaiting...", Result::SKIP);
    vTaskDelay(pdMS_TO_TICKS(1100));
    uint8_t s2 = read_seconds();

    bool ticking = (s2 != s1);
    char l1[40], l2[40], l3[40], l4[40], msg[200];
    snprintf(l1, sizeof(l1), "%-13s#2ECC71 OK   #", "I2C 0x51");
    snprintf(l2, sizeof(l2), "%-13s%u",             "Seconds[0]", s1);
    snprintf(l3, sizeof(l3), "%-13s%u",             "Seconds[1]", s2);
    snprintf(l4, sizeof(l4), "%-13s#%s %-5s#",     "Ticking",
             ticking ? "2ECC71" : "E74C3C",
             ticking ? "YES" : "NO");
    snprintf(msg, sizeof(msg), "%s\n%s\n%s\n%s", l1, l2, l3, l4);
    _update_screen(msg, ticking ? Result::PASS : Result::FAIL);
    return _auto_or_verdict(ticking) ? Result::PASS : Result::FAIL;
}

FactoryTest::Result FactoryTest::_test_camera()
{
    // TODO: Camera hardware test not yet implemented.
    // Requires a physical OV2640 (or compatible) module connected to CAM_* pins.
    // TODO: Verify CAM_LEDC_TIMER / CAM_LEDC_CH don't conflict with backlight
    //       LEDC before enabling esp_camera_init() here.
    // TODO: Add frame-quality check: verify captured JPEG size > threshold
    //       and that the frame is not an all-black "dark frame".
    _show_screen(8, "Camera",
        "TODO: Camera test\n"
        "not implemented yet.\n"
        "\n"
        "Requires OV2640 module\n"
        "on CAM_* pins.\n"
        "\n"
        "Press Ok to SKIP.",
        Result::SKIP);
    _wait_verdict();
    return Result::SKIP;   // always SKIP until TODO resolved
}

FactoryTest::Result FactoryTest::_test_audio()
{
    _show_screen(9, "Audio (ES8311)", "Probing I2C 0x18...", Result::SKIP);

    if (!i2c_probe(ES8311_I2C_ADDR)) {
        _update_screen("I2C 0x18     #E74C3C FAIL #\nES8311 not found", Result::FAIL);
        return _auto_or_verdict(false) ? Result::PASS : Result::FAIL;
    }

    // Read chip ID registers (0xFD / 0xFE) — ES8311 returns 0x83 / 0x11.
    Wire.beginTransmission(ES8311_I2C_ADDR);
    Wire.write(0xFD);
    Wire.endTransmission(false);
    Wire.requestFrom(ES8311_I2C_ADDR, (uint8_t)2);
    uint8_t id0 = Wire.available() ? Wire.read() : 0xFF;
    uint8_t id1 = Wire.available() ? Wire.read() : 0xFF;
    bool i2c_ok = (id0 == 0x83 && id1 == 0x11);

    char l1[40], l2[40], l3[40], l4[40], msg[256];
    snprintf(l1, sizeof(l1), "%-13s#2ECC71 OK   #",  "I2C 0x18");
    snprintf(l2, sizeof(l2), "%-13s0x%02X  #%s %-5s#", "ID[FD]",  id0,
             (id0==0x83)?"2ECC71":"E74C3C", (id0==0x83)?"OK":"FAIL");
    snprintf(l3, sizeof(l3), "%-13s0x%02X  #%s %-5s#", "ID[FE]",  id1,
             (id1==0x11)?"2ECC71":"E74C3C", (id1==0x11)?"OK":"FAIL");
    snprintf(l4, sizeof(l4), "%-13s#7F8C8D TODO #",   "I2S test");
    snprintf(msg, sizeof(msg), "%s\n%s\n%s\n%s", l1, l2, l3, l4);
    _update_screen(msg, i2c_ok ? Result::PASS : Result::FAIL);
    return _auto_or_verdict(i2c_ok) ? Result::PASS : Result::FAIL;
}
