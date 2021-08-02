#include "stereokit.h"
#include "stereokit_ui.h"
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/rs.hpp>
#include <vector>
using namespace sk;

#include "demo_basics.h"
#include "demo_lines.h"
#include "demo_picker.h"
#include "demo_sprites.h"
#include "demo_ui.h"
#include "scene.h"

#include <stdio.h>

#include <list>
#include <string>

solid_t floor_solid;
matrix floor_tr;
material_t floor_mat;
model_t floor_model;

sk::bool32_t hand_mesh = true;

sk::bool32_t hand_axes = false;

sk::bool32_t hand_lines = false;

sk::bool32_t leftright = false;

material_t line_hand_mat1;

scene_t demos[] = {{
                       "UI",
                       demo_ui_init,
                       demo_ui_update,
                       demo_ui_shutdown,
                   },
                   {
                       "Basics",
                       demo_basics_init,
                       demo_basics_update,
                       demo_basics_shutdown,
                   },
                   {
                       "Sprites",
                       demo_sprites_init,
                       demo_sprites_update,
                       demo_sprites_shutdown,
                   },
                   {
                       "Lines",
                       demo_lines_init,
                       demo_lines_update,
                       demo_lines_shutdown,
                   },
                   {
                       "Picker",
                       demo_picker_init,
                       demo_picker_update,
                       demo_picker_shutdown,
                   },
                   {
                       "Exit",
                       sk_quit,
                   }};

pose_t demo_select_pose;

void on_log(log_, const char *);
void log_window();
void common_init();
void common_update();
void common_shutdown();
void ruler_window();

pose_t log_pose =
    pose_t{vec3{0, -0.1f, 0.5f}, quat_lookat(vec3_zero, vec3_forward)};
std::list<std::string> log_list;

void on_log(log_ log_level, const char *log_c_str) {
  if (log_list.size() > 10) {
    log_list.pop_front();
  }
  std::string log_str(log_c_str);
  if (log_str.size() >= 100) {
    log_str.resize(100);
    log_str += "...";
  }
  log_list.push_back(log_str);
}

void log_window() {
  ui_window_begin("Log", log_pose, vec2{40 * cm2m, 0 * cm2m});
  for (auto &log_str : log_list) {
    ui_label(log_str.c_str(), false);
  }
  ui_window_end();
}

// int setup_realsense_thing() {

//   // Start streaming with default recommended configuration
//   pipe = new rs2::pipeline();
//   pipe->start();
//   return 0;
// }

// Declare pointcloud object, for calculating pointclouds and texture mappings
rs2::pointcloud pc;
// We want the points object to be persistent so we can display the last cloud
// when a frame drops
rs2::points points;

static color32 line_color = {140, 140, 140, 255};
// static color32 line_color = {0, 0, 0, 255};
static float line_size = 0.001f;

int realsense_thing_one_frame(rs2::pipeline pipe) { return 0; }

int main() {
  log_subscribe(on_log);
  log_set_filter(log_diagnostic);

  sk_settings_t settings = {};
  settings.app_name = "StereoKit C";
  settings.assets_folder = "/2/XR/skNotes/Assets";
  settings.display_preference = display_mode_mixedreality;
  if (!sk_init(settings))
    return 1;

  common_init();

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  rs2::pipeline pipe;
  rs2::config cfg;
  rs2::pipeline_profile myProf;
  rs2::context ctx;
  int width = 640;
  int height = 360;

  cfg.disable_all_streams();
  cfg.enable_stream(rs2_stream::RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16,
                    90);

  pipe.start(cfg);
  bool has_thing = 0;

  mesh_t depth_mesh = mesh_create();

  vert_t *depthmap = (vert_t *)calloc(height * width, sizeof(vert_t));
  vind_t *depthmapind = (vind_t *)calloc((height * width) * 6, sizeof(vind_t));

  material_t depth_material = material_copy(material_find(default_id_material));
  material_set_cull(depth_material, cull_none);
  material_set_color(depth_material, "color", color128{255.0f,0,1.0f, 255.0f});
  material_set_color(depth_material, "color", color128{3.0f,3.0f,3.0f, 0.0f});
  // material_set_color(depth_material, "color", color128{0.0f,0.0f,0.0f, 2550.0f});


  scene_set_active(demos[0]);

  while (sk_step([]() {
    scene_update();
    common_update();
  })) {
    // sk::pose_t hpose = sk::input_head();
    // Wait for the next set of frames from the camera
    rs2::frameset frames;
    //  = pipe.wait_for_frames();

    if (pipe.poll_for_frames(&frames)) {

      auto depth = frames.get_depth_frame();

      // Generate the pointcloud and texture mappings
      points = pc.calculate(depth);
      has_thing = true;
      // printf("Size is %zu\n", points.size());
      auto v = points.get_vertices();
      // for (int i = 0; i < points.size() - 1; i++) {
      //   if (v[i].z && v[i + 1].z) {
      //     vec3 o = {-v[i].x, v[i].y, -v[i].z};
      //     vec3 o1 = {-v[i+1].x, v[i+1].y, -v[i+1].z};
      //     line_add(o, o1, line_color, line_color, line_size);
      //   }
      //   // if (v[i].z) {
      //     vec3 o = {-v[i].x, v[i].y, -v[i].z};
      //     depthmap[i].pos = o;
      //     depthmap[i].norm = {0,0,1};
      //     depthmap[i].col = {1,1,1,1};
      //     depthmap[i].uv = {1,1};
      //   // } else {
      //   //   depthmap[i].pos.z = (float)false;
      //   // }
      // }
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          int i = (x) + (y)*width;
          if (v[i].z) {
            vec3 o = {-v[i].x, v[i].y, -v[i].z};
            depthmap[i].pos = o;
            depthmap[i].norm = {0, 0, 1};
            depthmap[i].col = {1, 1, 1, 1};
            depthmap[i].uv = {1, 1};
          } else {
            depthmap[i].pos.z = (float)false;
          }
          /*
          if (x < (width - 1) && y < (height - 1)) {
            // if (v[x + y * width].z && v[x + (y + 1) * width].z) {
            //   vec3 tl = {-v[x].x, v[i].y, -v[i].z};
            //   line_add(o, o1, line_color, line_color, line_size);
            // }

            int idx_tl = (x) + (y * width);
            int idx_tr = (x + 1) + (y * width);
            int idx_bl = (x) + (y + 1) * width;
            int idx_br = (x + 1) + (y + 1) * width;

            vec3 tl = {-v[idx_tl].x, v[idx_tl].y, -v[idx_tl].z};
            vec3 tr = {-v[idx_tr].x, v[idx_tr].y, -v[idx_tr].z};
            vec3 bl = {-v[idx_bl].x, v[idx_bl].y, -v[idx_bl].z};
            vec3 br = {-v[idx_br].x, v[idx_br].y, -v[idx_br].z};
            if (v[idx_tl].z && v[idx_tr].z)
              line_add(tl, tr, line_color, line_color, line_size);

            if (v[idx_tl].z && v[idx_bl].z)
              line_add(tl, bl, line_color, line_color, line_size);
            // line_add(tr, br, line_color, line_color, line_size);
            // line_add(bl, br, line_color, line_color, line_size);
          }*/
        }
      }
      int ind = 0;
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {

          // Create triangle face indices from the current vertex, and the
          // vertices on the next row and column! Since there is no 'next' row
          // and column on the last row and column, we guard this with a check
          // for size-1.
          if (x < width - 1 && y < height - 1) {
            // 1
            // |\
          // | \
          // |__\
          // 2   3
            if (depthmap[x + y * width].pos.z &&
                depthmap[x + (y + 1) * width].pos.z &&
                depthmap[(x + 1) + (y + 1) * width].pos.z) {
              depthmapind[ind++] = (uint)((x) + (y)*width);
              depthmapind[ind++] = (uint)((x) + (y + 1) * width);
              depthmapind[ind++] = (uint)((x + 1) + (y + 1) * width);
            }
            // 1 __ 2
            //  \  |
            //   \ |
            //    \|
            //     3
            if (depthmap[x + y * width].pos.z &&
                depthmap[(x + 1) + (y)*width].pos.z &&
                depthmap[(x + 1) + (y + 1) * width].pos.z) {

              depthmapind[ind++] = ((x) + (y)*width);
              depthmapind[ind++] = ((x + 1) + (y)*width);
              depthmapind[ind++] = ((x + 1) + (y + 1) * width);
            }
          }
        }
      }
      // printf("Doing bad thign %i\n", width*height);
      mesh_set_verts(depth_mesh, depthmap, width * height);
      mesh_set_inds(depth_mesh, depthmapind, ind);
    }

    if (has_thing) {
      hierarchy_push(pose_matrix(*input_head()));
      hierarchy_push(matrix_trs({0,0,0},quat_from_angles(0, 0, 0)));
      hierarchy_push(matrix_trs(vec3{mm2m*-80.5f, mm2m*-44.2f, mm2m*-67.9f}));
      render_add_mesh(depth_mesh, depth_material,
                      matrix_identity);
      hierarchy_pop();
      hierarchy_pop();
      hierarchy_pop();
    }
  };

  scene_shutdown();
  common_shutdown();
  sk_shutdown();
  return 0;
}

void common_init() {
  // Create a PBR floor material
  tex_t tex_color = tex_create_file("test.png");
  tex_t tex_norm = tex_create_file("test_normal.png");
  floor_mat = material_copy_id("default/material");
  material_set_texture(floor_mat, "diffuse", tex_color);
  material_set_texture(floor_mat, "normal", tex_norm);
  material_set_float(floor_mat, "tex_scale", 6);
  material_set_float(floor_mat, "roughness", 1.0f);
  material_set_float(floor_mat, "metallic", 0.5f);
  material_set_queue_offset(floor_mat, 1);
  if (tex_color != nullptr)
    tex_release(tex_color);
  if (tex_norm != nullptr)
    tex_release(tex_norm);

  // Procedurally create a cube model
  mesh_t mesh_cube = mesh_gen_cube(vec3_one, 0);
  floor_model = model_create_mesh(mesh_cube, floor_mat);
  mesh_release(mesh_cube);

  // Build a physical floor!
  vec3 pos = vec3{0, -1.5f, 0};
  vec3 scale = vec3{5, 1, 5};
  floor_tr = matrix_trs(pos, quat_identity, scale);
  floor_solid = solid_create(pos, quat_identity, solid_type_immovable);
  solid_add_box(floor_solid, scale);

  demo_select_pose.position = vec3{0, 0.2f, -0.4f};
  demo_select_pose.orientation = quat_lookat(vec3_forward, vec3_zero);

  line_hand_mat1 = material_find(default_id_material_hand);
  material_set_color(line_hand_mat1, "color", {1.0, 1.0, 0.0, 1});
}

void draw_axis(pose_t &pose, float size, float thickness) {
  matrix pose_matrix_ = pose_matrix(pose);

  line_add(pose.position,
           matrix_mul_direction(pose_matrix_, {1.0f, 0, 0}) * size +
               pose.position,
           {255, 0, 0, 255}, {255, 0, 0, 255}, thickness);
  line_add(pose.position,
           matrix_mul_direction(pose_matrix_, {0, 1.0f, 0}) * size +
               pose.position,
           {0, 255, 0, 255}, {0, 255, 0, 255}, thickness);
  line_add(pose.position,
           matrix_mul_direction(pose_matrix_, {0, 0, 1.0f}) * size +
               pose.position,
           {0, 0, 255, 255}, {0, 0, 255, 255}, thickness);
}

bool draw_hand_axes() {
  for (int i = 0; i < handed_max; i++) {
    const hand_t *hand = input_hand((handed_)i);
    if (hand->tracked_state == button_state_inactive)
      continue;

    for (int finger = 0; finger < 5; finger++) {
      for (int joint = 0; joint < 5; joint++) {
        hand_joint_t joint_ = hand->fingers[finger][joint];
        pose_t joint_pose = {joint_.position, joint_.orientation};

        draw_axis(joint_pose, 0.015f, 0.002);
      }
    }

    pose_t palm_pose = hand->palm;
    draw_axis(palm_pose, 0.015f, 0.002);

    pose_t wrist_pose = hand->wrist;
    // log_diagf("I think wrist is at %f %f %f", wrist_pose.position.x,
    // wrist_pose.position.y, wrist_pose.position.z);
    draw_axis(wrist_pose, 15.0f, 0.002);
    vec3 wrist = wrist_pose.position;
    vec3 middle_distal = hand->fingers[2][1].position;
    vec3 sub = wrist - middle_distal;
  }
  return true;
}

void draw_hand_lines() {
  for (int side = 0; side < handed_max; side++) {
    const hand_t *hand = input_hand((handed_)side);
    if (hand->tracked_state == button_state_inactive)
      continue;

    for (int finger = 0; finger < 5; finger++) {
      for (int joint = 0; joint < 4; joint++) {
        hand_joint_t joint0 = hand->fingers[finger][joint];
        hand_joint_t joint1 = hand->fingers[finger][joint + 1];
        float hue0 = ((finger * 5) + joint) / 24.0f;
        float hue1 = ((finger * 5) + joint + 1) / 24.0f;
        if (finger == 0 && joint == 0) {
          line_add(hand->wrist.position, joint0.position,
                   color_to_32(color_hsv(hue0, 1.0f, 1.0f, 1.0f)),
                   color_to_32(color_hsv(hue1, 1.0f, 1.0f, 1.0f)), 0.002);
        } else {
          line_add(joint0.position, joint1.position,
                   color_to_32(color_hsv(hue0, 1.0f, 1.0f, 1.0f)),
                   color_to_32(color_hsv(hue1, 1.0f, 1.0f, 1.0f)), 0.002);
        }
      }
    }
  }
}

void hand_window(sk::handed_ hand, const char *hi) {
  vec3 left_position = input_hand(hand)->palm.position;
  quat head_potition = input_head()->orientation;
  pose_t pose;
  pose.position = left_position;
  pose.orientation =
      quat_mul(head_potition, quat_from_angles(0, 180, 0)); // head_potition;
  if (input_hand(hand)->tracked_state == button_state_active) {
    ui_window_begin(hi, pose, vec2{0, 0});
    ui_window_end();
  }
}

void common_update() {
  // Render floor
  // render_add_model(floor_model, floor_tr);

  // input_hand_solid(handed_max, false);

  ui_window_begin("Demos", demo_select_pose, vec2{50 * cm2m, 0 * cm2m});
  for (int i = 0; i < sizeof(demos) / sizeof(scene_t); i++) {
    std::string &name = demos[i].name;

    if (ui_button(name.c_str())) {
      log_write(log_inform, name.c_str());
      scene_set_active(demos[i]);
    }
    ui_sameline();
  }
  ui_nextline();
  ui_toggle("Hand axes", hand_axes);
  ui_sameline();
  ui_toggle("Hand mesh", hand_mesh);
  ui_sameline();
  ui_toggle("Hand lines", hand_lines);
  ui_sameline();
  ui_toggle("Leftright", leftright);
  ui_sameline();
  ui_window_end();

  if (hand_axes) {
    draw_hand_axes();
  }

  if (!hand_mesh) {
    input_hand_visible(sk::handed_left, false);
    input_hand_visible(sk::handed_right, false);
  } else {
    input_hand_visible(sk::handed_left, true);
    input_hand_visible(sk::handed_right, true);
  }

  if (hand_lines) {
    draw_hand_lines();
  }

  if (leftright) {
    hand_window(sk::handed_left, "left");
    hand_window(sk::handed_right, "right");
  }

  ruler_window();
  log_window();
}

void common_shutdown() {
  solid_release(floor_solid);
  material_release(floor_mat);
  model_release(floor_model);
}

void ruler_window() {
  static pose_t window_pose = pose_t{{0, 0, 0.5f}, quat_identity};
  ui_handle_begin("Ruler", window_pose,
                  bounds_t{vec3_zero, vec3{30 * cm2m, 4 * cm2m, 1 * cm2m}},
                  true, ui_move_exact);
  color32 color = color_to_32(color_hsv(0.6f, 0.5f, 1, 1));
  text_add_at("Centimeters",
              matrix_trs(vec3{14.5f * cm2m, -1.5f * cm2m, -0.6f * cm2m},
                         quat_identity, vec3{0.3f, 0.3f, 0.3f}),
              -1, text_align_bottom_left);
  for (int d = 0; d <= 60; d++) {
    float x = d / 2.0f;
    float size = (d % 2 == 0) ? 1.0f : 0.15f;
    line_add(vec3{(15 - x) * cm2m, 2 * cm2m, -0.6f * cm2m},
             vec3{(15 - x) * cm2m, (2 - size) * cm2m, -0.6f * cm2m}, color,
             color, 0.5f * mm2m);

    if (d % 2 == 0 && d / 2 != 30) {
      text_add_at(std::to_string(d / 2).c_str(),
                  matrix_trs(vec3{(15 - x - 0.1f) * cm2m, (2 - size) * cm2m,
                                  -0.6f * cm2m},
                             quat_identity, vec3{0.2f, 0.2f, 0.2f}),
                  -1, text_align_bottom_left);
    }
  }
  ui_handle_end();
}
