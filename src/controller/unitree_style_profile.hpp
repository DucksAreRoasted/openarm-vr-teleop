#pragma once

#include "controller/human_mimic_types.hpp"

// OpenArm adaptation of Unitree's Cartesian pose + joint regularization objective.
// Uses our bounded LM solver, robot model, clutch and reference generator; does
// not use Unitree's motor SDK, human body offsets or moving head reference.
inline void apply_unitree_style_profile(HumanMimicConfig* cfg) {
    cfg->consistent_world_mapping = true;
    cfg->position_map_world_frame = true;
    cfg->mirror_translation_y = false;
    cfg->use_elbow_skeleton = false;
    cfg->reach_soft_enabled = false;
    cfg->tip_priority_enabled = false;
    cfg->extension_yield_enabled = false;
    cfg->fine_motion_hold_enabled = false;
    cfg->reanchor_to_commanded = false;
    cfg->elbow_nullspace_enabled = false;
    cfg->w_elbow = 0.0;
    cfg->w_wrist_pos = 50.0;
    cfg->w_hand_ori = 1.0;
    cfg->w_continuity = 0.1;
    cfg->w_nominal = 0.02;
    cfg->proximal_continuity_scale = 1.0;
    cfg->j5_continuity_scale = 1.0;
    cfg->j6_continuity_scale = 1.0;
    cfg->j7_continuity_scale = 1.0;
    cfg->ik_backend = HumanMimicIkBackend::LM;
    cfg->ik_max_iters = 30;
}
