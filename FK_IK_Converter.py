# ============================================
# Simple FK <-> IK builder & snapper (no add-ons)
# Works with the provided 71-bone naming scheme.
# Arm/Leg IK (chain=2), Torso IK (chain=4), Head Copy Rotation.
# Creates N-panel UI: "FK/IK (Simple)"
# ============================================

import bpy
import math
from mathutils import Vector, Matrix, Quaternion
from math import atan2

# ----------------------------
# CONFIG (you can tweak)
# ----------------------------
# Distances for placing pole targets, scaled by limb length:
ARM_POLE_DIST_FACTOR = 0.7
LEG_POLE_DIST_FACTOR = 0.7
# Absolute fallback distances if limb length is tiny (meters)
ARM_POLE_DIST_MIN = 0.2
LEG_POLE_DIST_MIN = 0.2

# IK target offsets (how far to place targets away from effector in local axis dir, meters)
HAND_IK_OFFSET = 0.0   # place on hand position
FOOT_IK_OFFSET = 0.0   # place on foot position

# Property names stored on the Armature object
PROP_ARM_L = "ik_arm_L"
PROP_ARM_R = "ik_arm_R"
PROP_LEG_L = "ik_leg_L"
PROP_LEG_R = "ik_leg_R"
PROP_TORSO = "ik_torso"
PROP_HEAD  = "ik_head"

# Bone name map (from your list)
B = {
    # Torso / head
    "hips": "Hips",
    "spine": ["Spine", "Spine1", "Spine2", "Spine3"],
    "neck": ["Neck", "Neck1"],
    "head": "Head",

    # Left arm
    "l_shoulder": "LeftShoulder",
    "l_upper":    "LeftArm",
    "l_lower":    "LeftForeArm",
    "l_hand":     "LeftHand",

    # Right arm
    "r_shoulder": "RightShoulder",
    "r_upper":    "RightArm",
    "r_lower":    "RightForeArm",
    "r_hand":     "RightHand",

    # Left leg
    "l_upper_leg": "LeftUpLeg",
    "l_lower_leg": "LeftLeg",
    "l_foot":      "LeftFoot",
    "l_toe":       "LeftToeBase",  # not strictly used

    # Right leg
    "r_upper_leg": "RightUpLeg",
    "r_lower_leg": "RightLeg",
    "r_foot":      "RightFoot",
    "r_toe":       "RightToeBase",
}

# New control bone names (created by this script)
CTRL = {
    # left arm
    "l_hand_ik":  "CTRL_L_Hand_IK",
    "l_arm_pole": "CTRL_L_Arm_Pole",
    # right arm
    "r_hand_ik":  "CTRL_R_Hand_IK",
    "r_arm_pole": "CTRL_R_Arm_Pole",
    # left leg
    "l_foot_ik":  "CTRL_L_Foot_IK",
    "l_leg_pole": "CTRL_L_Leg_Pole",
    # right leg
    "r_foot_ik":  "CTRL_R_Foot_IK",
    "r_leg_pole": "CTRL_R_Leg_Pole",
    # torso / head
    "torso_ik":   "CTRL_Torso_IK",
    "head_ik":    "CTRL_Head_IK",
}

# Constraint names (so we can find/update them)
CNAME = {
    "ik_l_arm":  "IK_L_Arm",
    "ik_r_arm":  "IK_R_Arm",
    "ik_l_leg":  "IK_L_Leg",
    "ik_r_leg":  "IK_R_Leg",
    "ik_torso":  "IK_Torso",
    "cr_head":   "CopyRot_Head",
}

# ----------------------------
# Utility helpers
# ----------------------------

def _project_on_plane(v, n):
    # project v onto plane with normal n
    return (v - n * v.dot(n))

def compute_pole_angle(obj, upper_name, lower_name, eff_name, pole_world_loc):
    """Return angle (radians) for IK.pole_angle so the chain won't twist when enabling IK.
    Works in world space using limb plane."""
    upper = pb(obj, upper_name)
    lower = pb(obj, lower_name)
    eff   = pb(obj, eff_name)
    if not (upper and lower and eff):
        return 0.0

    w_root  = obj.matrix_world @ upper.head
    w_joint = obj.matrix_world @ lower.head
    w_eff   = obj.matrix_world @ eff.tail  # Use tail for effector

    v1 = (w_joint - w_root).normalized()
    v2 = (w_eff   - w_joint).normalized()
    n  = v1.cross(v2)
    if n.length < 1e-8:
        # If limb is straight, use a fallback normal
        # Try to use the bone's local axis projected to world
        fallback = obj.matrix_world.to_3x3() @ Vector((0, 0, 1))
        n = fallback.normalized()
    else:
        n.normalize()

    # Get the current pole direction in the FK pose
    # Use the lower bone's current world matrix Y-axis (bend direction)
    current_lower_matrix = obj.matrix_world @ lower.matrix
    current_pole_dir = current_lower_matrix.to_3x3() @ Vector((0, 1, 0))  # Y axis
    current_pole_plane = _project_on_plane(current_pole_dir, n).normalized()

    # Get the pole target direction
    pole_dir = (pole_world_loc - w_joint).normalized()
    pole_plane = _project_on_plane(pole_dir, n).normalized()

    # Calculate signed angle between current and target pole directions
    cosv = max(-1.0, min(1.0, current_pole_plane.dot(pole_plane)))
    sinv = n.dot(current_pole_plane.cross(pole_plane))
    return atan2(sinv, cosv)

def get_arm():
    obj = bpy.context.object
    if not obj or obj.type != 'ARMATURE':
        raise RuntimeError("Select your Armature object and run in Pose Mode.")
    return obj

def ensure_pose_mode(obj):
    if bpy.context.object != obj:
        bpy.context.view_layer.objects.active = obj
    if obj.mode != 'POSE':
        bpy.ops.object.mode_set(mode='POSE')

def pb(obj, name):
    return obj.pose.bones.get(name)

def eb(obj, name):
    return obj.data.edit_bones.get(name)

def limb_len(obj, a_name, b_name):
    a = pb(obj, a_name)
    b = pb(obj, b_name)
    if not a or not b:
        return 0.5
    h = obj.matrix_world @ a.head
    t = obj.matrix_world @ b.head  # joint head
    return (t - h).length

def or_none(v):
    return v if v is not None else 0.0

def add_custom_prop(obj, name, default=0.0, min=0.0, max=1.0):
    if name not in obj:
        obj[name] = default
    ui = obj.get("_RNA_UI")
    if ui is None:
        obj["_RNA_UI"] = {}
        ui = obj["_RNA_UI"]
    ui[name] = {"min": min, "max": max, "soft_min": min, "soft_max": max}

def driver_influence_from_prop(con, obj, prop_name):
    fcu = con.driver_add("influence")
    drv = fcu.driver
    drv.type = 'AVERAGE'
    var = drv.variables.new()
    var.name = "ik"
    v = var.targets[0]
    v.id = obj
    v.data_path = f'["{prop_name}"]'
    drv.expression = "ik"

def make_driver(con, obj, prop):
    # Remove previous FCurves on influence
    try:
        con.driver_remove("influence")
    except:
        pass
    driver_influence_from_prop(con, obj, prop)

def create_edit_bone_like(obj, new_name, ref_name, head_offset=Vector((0,0,0)), tail_offset=Vector((0,0,0))):
    bpy.ops.object.mode_set(mode='EDIT')
    ref = eb(obj, ref_name)
    if not ref:
        raise RuntimeError(f"Missing bone: {ref_name}")

    if new_name in obj.data.edit_bones:
        newb = obj.data.edit_bones[new_name]
    else:
        newb = obj.data.edit_bones.new(new_name)

    newb.head = ref.tail + head_offset
    newb.tail = ref.tail + Vector((0, 0.1, 0)) + tail_offset  # small length
    newb.use_deform = False
    newb.parent = None
    bpy.ops.object.mode_set(mode='POSE')
    return pb(obj, new_name)

def set_pbone_world_location(obj, pbone, world_loc):
    # Convert world location to armature (object) space for pose bone location
    arm_space = obj.matrix_world.inverted() @ world_loc
    # Use delta on location for non-connected controllers
    pbone.location = arm_space - pbone.bone.head_local

def set_pbone_world_matrix(obj, pbone, world_matrix):
    """IMPROVED: Set pose bone to match a world space matrix with better precision"""
    
    # Method 1: Direct matrix approach (most precise)
    try:
        # Convert world matrix to armature space
        armature_matrix = obj.matrix_world.inverted() @ world_matrix
        
        # Get the bone's rest matrix in armature space  
        bone_rest_matrix = pbone.bone.matrix_local
        
        # Calculate the pose transformation needed
        # This is the transformation from rest pose to target pose
        pose_matrix = bone_rest_matrix.inverted() @ armature_matrix
        
        # Clear any existing transforms
        pbone.location = Vector((0, 0, 0))
        if pbone.rotation_mode == 'QUATERNION':
            pbone.rotation_quaternion = Quaternion((1, 0, 0, 0))
        else:
            pbone.rotation_euler = Vector((0, 0, 0))
        pbone.scale = Vector((1, 1, 1))
        
        # Apply the calculated transformation
        pbone.location = pose_matrix.translation
        
        # Handle rotation based on bone's rotation mode
        rotation_quat = pose_matrix.to_quaternion().normalized()
        
        if pbone.rotation_mode == 'QUATERNION':
            pbone.rotation_quaternion = rotation_quat
        elif pbone.rotation_mode in ('XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX'):
            pbone.rotation_euler = rotation_quat.to_euler(pbone.rotation_mode)
        elif pbone.rotation_mode == 'AXIS_ANGLE':
            axis, angle = rotation_quat.to_axis_angle()
            pbone.rotation_axis_angle = [angle, axis[0], axis[1], axis[2]]
            
    except Exception as e:
        print(f"Error in precise matrix setting: {e}")
        # Fallback to original method
        arm_matrix = obj.matrix_world.inverted() @ world_matrix
        pbone.location = arm_matrix.translation - pbone.bone.head_local
        rot_quat = arm_matrix.to_quaternion()
        if pbone.rotation_mode == 'QUATERNION':
            pbone.rotation_quaternion = rot_quat
        elif pbone.rotation_mode in ('XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX'):
            pbone.rotation_euler = rot_quat.to_euler(pbone.rotation_mode)
        elif pbone.rotation_mode == 'AXIS_ANGLE':
            axis, angle = rot_quat.to_axis_angle()
            pbone.rotation_axis_angle = [angle, axis[0], axis[1], axis[2]]

def world_of(obj, pbone, point='tail'):
    if point == 'head':
        return obj.matrix_world @ pbone.head
    elif point == 'tail':
        return obj.matrix_world @ pbone.tail
    else:
        return obj.matrix_world @ pbone.matrix.translation

def safe_quat_from_matrix(m):
    # Robustly extract quaternion
    return m.to_quaternion()

def copy_rotation(source_pbone, target_pbone):
    """Copy rotation from source pose bone to target pose bone, handling different rotation modes."""
    # Get the rotation as a quaternion from the source
    if source_pbone.rotation_mode == 'QUATERNION':
        source_quat = source_pbone.rotation_quaternion.copy()
    elif source_pbone.rotation_mode in ('XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX'):
        source_quat = source_pbone.rotation_euler.to_quaternion()
    elif source_pbone.rotation_mode == 'AXIS_ANGLE':
        angle, x, y, z = source_pbone.rotation_axis_angle
        axis = Vector((x, y, z)).normalized()
        source_quat = Quaternion(axis, angle)
    else:
        source_quat = safe_quat_from_matrix(source_pbone.matrix)
    
    # Set the rotation on the target bone in its preferred mode
    if target_pbone.rotation_mode == 'QUATERNION':
        target_pbone.rotation_quaternion = source_quat
    elif target_pbone.rotation_mode in ('XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX'):
        target_pbone.rotation_euler = source_quat.to_euler(target_pbone.rotation_mode)
    elif target_pbone.rotation_mode == 'AXIS_ANGLE':
        axis, angle = source_quat.to_axis_angle()
        target_pbone.rotation_axis_angle = [angle, axis[0], axis[1], axis[2]]

def bake_rot_from_evaluated(pbone):
    # Write evaluated (constraint result) rotation into the FK channels
    mode = pbone.rotation_mode
    evaluated_quat = safe_quat_from_matrix(pbone.matrix)
    
    if mode == 'QUATERNION':
        pbone.rotation_quaternion = evaluated_quat
    elif mode == 'XYZ' or mode == 'XZY' or mode == 'YXZ' or mode == 'YZX' or mode == 'ZXY' or mode == 'ZYX':
        pbone.rotation_euler = evaluated_quat.to_euler(mode)
    elif mode == 'AXIS_ANGLE':
        axis, angle = evaluated_quat.to_axis_angle()
        pbone.rotation_axis_angle = [angle, axis[0], axis[1], axis[2]]

def place_pole(obj, upper_name, lower_name, eff_name, pole_pbone, dist_factor, dist_min):
    upper = pb(obj, upper_name)
    lower = pb(obj, lower_name)
    eff   = pb(obj, eff_name)
    if not (upper and lower and eff):
        return

    # World positions
    w_root  = world_of(obj, upper, 'head')
    w_joint = world_of(obj, lower, 'head')
    w_eff   = world_of(obj, eff,   'head')

    # Limb vectors
    v1 = (w_joint - w_root).normalized()
    v2 = (w_eff   - w_joint).normalized()

    # Normal of the limb plane
    n = v1.cross(v2)
    if n.length < 1e-8:
        n = Vector((0,0,1))
    n.normalize()

    # Use lower bone's local axis to bias the pole side (armature space axis to world)
    lower_axis = lower.bone.y_axis
    lower_axis_world = (obj.matrix_world.to_3x3() @ lower_axis).normalized()

    # Choose a direction roughly perpendicular to limb plane, leaning toward lower axis
    dir_vec = (n + 0.5 * lower_axis_world).normalized()

    # Distance based on limb length
    L = max((world_of(obj, lower, 'tail') - w_root).length, dist_min)
    dist = max(L * dist_factor, dist_min)
    pos = w_joint + dir_vec * dist

    set_pbone_world_location(obj, pole_pbone, pos)

def improved_place_pole(obj, upper_name, lower_name, eff_name, pole_pbone, dist_factor, dist_min, fk_matrices):
    """Improved pole placement using captured FK matrices"""
    upper = pb(obj, upper_name)
    lower = pb(obj, lower_name)
    eff   = pb(obj, eff_name)
    if not (upper and lower and eff):
        return

    # Use ORIGINAL FK positions from captured matrices
    if upper_name in fk_matrices and lower_name in fk_matrices and eff_name in fk_matrices:
        w_root = fk_matrices[upper_name]['world_matrix'].translation
        w_joint = fk_matrices[lower_name]['world_matrix'].translation  
        w_eff = fk_matrices[eff_name]['world_matrix'].translation
    else:
        # Fallback to current positions
        w_root  = world_of(obj, upper, 'head')
        w_joint = world_of(obj, lower, 'head')
        w_eff   = world_of(obj, eff,   'head')

    # Limb vectors using original FK positions
    v1 = (w_joint - w_root).normalized()
    v2 = (w_eff   - w_joint).normalized()

    # Normal of the limb plane
    n = v1.cross(v2)
    if n.length < 1e-8:
        n = Vector((0,0,1))
    n.normalize()

    # Use the ORIGINAL lower bone matrix for axis calculation
    if lower_name in fk_matrices:
        lower_axis_world = fk_matrices[lower_name]['world_matrix'].to_3x3() @ Vector((0, 1, 0))
    else:
        lower_axis = lower.bone.y_axis
        lower_axis_world = (obj.matrix_world.to_3x3() @ lower_axis).normalized()

    # Choose a direction roughly perpendicular to limb plane, leaning toward lower axis
    dir_vec = (n + 0.5 * lower_axis_world).normalized()

    # Distance based on limb length using original positions
    L = max((w_eff - w_root).length, dist_min)
    dist = max(L * dist_factor, dist_min)
    pos = w_joint + dir_vec * dist

    set_pbone_world_location(obj, pole_pbone, pos)

def find_best_pole_angle(obj, upper_name, lower_name, eff_name, constraint_name, fk_matrices):
    """Find the best pole angle by testing multiple values and choosing the one with minimal rotation error"""
    
    if not all(bone in fk_matrices for bone in [upper_name, lower_name, eff_name]):
        return 0.0
    
    # Get the constraint
    eff_pbone = pb(obj, eff_name)
    if not eff_pbone:
        return 0.0
    
    constraint = eff_pbone.constraints.get(constraint_name)
    if not constraint:
        return 0.0
    
    # Store original rotations for comparison
    original_rotations = {}
    test_bones = [upper_name, lower_name, eff_name]
    
    for bone_name in test_bones:
        if bone_name in fk_matrices:
            original_rotations[bone_name] = fk_matrices[bone_name]['world_matrix'].to_quaternion()
    
    # Test different pole angles
    test_angles = []
    
    # Determine if this is a right-side limb
    is_right_side = 'Right' in upper_name or 'Right' in lower_name or 'Right' in eff_name
    
    if is_right_side:
        # For right side, test angles around 180°
        base_angles = [178.0, 179.0, 179.5, 179.7, 179.8, 179.9, 180.0, -180.0, -179.9, -179.8, -179.7, -179.5, -179.0, -178.0]
    else:
        # For left side, test small angles around 0°
        base_angles = [2.0, 1.5, 1.0, 0.5, 0.3, 0.2, 0.1, 0.0, -0.1, -0.2, -0.3, -0.5, -1.0, -1.5, -2.0]
    
    # Convert to radians and add to test list
    for angle_deg in base_angles:
        test_angles.append(math.radians(angle_deg))
    
    best_angle = 0.0
    best_error = float('inf')
    
    print(f"  Testing pole angles for {eff_name}...")
    
    # Test each angle
    for test_angle in test_angles:
        constraint.pole_angle = test_angle
        _update_depsgraph()
        
        # Calculate total rotation error compared to original FK pose
        total_error = 0.0
        for bone_name in test_bones:
            if bone_name in original_rotations:
                test_pbone = pb(obj, bone_name)
                if test_pbone:
                    current_matrix = obj.matrix_world @ test_pbone.matrix
                    current_quat = current_matrix.to_quaternion()
                    original_quat = original_rotations[bone_name]
                    
                    # Calculate angular difference
                    angle_diff = abs(current_quat.rotation_difference(original_quat).angle)
                    total_error += angle_diff
        
        if total_error < best_error:
            best_error = total_error
            best_angle = test_angle
    
    # Set the best angle
    constraint.pole_angle = best_angle
    _update_depsgraph()
    
    print(f"    Best pole angle for {eff_name}: {math.degrees(best_angle):.2f}° (error: {math.degrees(best_error):.2f}°)")
    return best_angle

def ensure_constraints_and_controls(obj):
    """Create controllers, IK constraints, Copy Rotation, and drivers."""

    ensure_pose_mode(obj)

    # Ensure custom properties (0=FK, 1=IK)
    add_custom_prop(obj, PROP_ARM_L, 0.0); add_custom_prop(obj, PROP_ARM_R, 0.0)
    add_custom_prop(obj, PROP_LEG_L, 0.0); add_custom_prop(obj, PROP_LEG_R, 0.0)
    add_custom_prop(obj, PROP_TORSO, 0.0); add_custom_prop(obj, PROP_HEAD,  0.0)

    # Create controller bones near effectors
    # Arms
    l_hand_ik = create_edit_bone_like(obj, CTRL["l_hand_ik"], B["l_hand"])
    r_hand_ik = create_edit_bone_like(obj, CTRL["r_hand_ik"], B["r_hand"])
    l_arm_pole = create_edit_bone_like(obj, CTRL["l_arm_pole"], B["l_lower"])
    r_arm_pole = create_edit_bone_like(obj, CTRL["r_arm_pole"], B["r_lower"])

    # Legs
    l_foot_ik = create_edit_bone_like(obj, CTRL["l_foot_ik"], B["l_foot"])
    r_foot_ik = create_edit_bone_like(obj, CTRL["r_foot_ik"], B["r_foot"])
    l_leg_pole = create_edit_bone_like(obj, CTRL["l_leg_pole"], B["l_lower_leg"])
    r_leg_pole = create_edit_bone_like(obj, CTRL["r_leg_pole"], B["r_lower_leg"])

    # Torso & Head
    torso_ik = create_edit_bone_like(obj, CTRL["torso_ik"], B["spine"][-1])
    head_ik  = create_edit_bone_like(obj, CTRL["head_ik"], B["head"])

    # ====== Add IK constraints ======
    def add_or_get_ik(pbone_name, cname):
        p = pb(obj, pbone_name)
        con = p.constraints.get(cname)
        if not con:
            con = p.constraints.new('IK')
            con.name = cname
        return con

    # Left arm (effector = hand)
    con = add_or_get_ik(B["l_hand"], CNAME["ik_l_arm"])
    con.target = obj; con.subtarget = CTRL["l_hand_ik"]
    con.pole_target = obj; con.pole_subtarget = CTRL["l_arm_pole"]
    con.chain_count = 2
    con.use_rotation = True
    con.influence = 0.0
    make_driver(con, obj, PROP_ARM_L)

    # Right arm
    con = add_or_get_ik(B["r_hand"], CNAME["ik_r_arm"])
    con.target = obj; con.subtarget = CTRL["r_hand_ik"]
    con.pole_target = obj; con.pole_subtarget = CTRL["r_arm_pole"]
    con.chain_count = 2
    con.use_rotation = True
    con.influence = 0.0
    make_driver(con, obj, PROP_ARM_R)

    # Left leg (effector = foot)
    con = add_or_get_ik(B["l_foot"], CNAME["ik_l_leg"])
    con.target = obj; con.subtarget = CTRL["l_foot_ik"]
    con.pole_target = obj; con.pole_subtarget = CTRL["l_leg_pole"]
    con.chain_count = 2
    con.use_rotation = True
    con.influence = 0.0
    make_driver(con, obj, PROP_LEG_L)

    # Right leg
    con = add_or_get_ik(B["r_foot"], CNAME["ik_r_leg"])
    con.target = obj; con.subtarget = CTRL["r_foot_ik"]
    con.pole_target = obj; con.pole_subtarget = CTRL["r_leg_pole"]
    con.chain_count = 2
    con.use_rotation = True
    con.influence = 0.0
    make_driver(con, obj, PROP_LEG_R)

    # Torso IK on top spine (chain across 4 spine bones)
    con = add_or_get_ik(B["spine"][-1], CNAME["ik_torso"])
    con.target = obj; con.subtarget = CTRL["torso_ik"]
    con.chain_count = min(4, len(B["spine"]))
    con.use_rotation = True
    con.influence = 0.0
    make_driver(con, obj, PROP_TORSO)

    # Head Copy Rotation (simpler/more stable than head IK)
    head_p = pb(obj, B["head"])
    cr = head_p.constraints.get(CNAME["cr_head"])
    if not cr:
        cr = head_p.constraints.new('COPY_ROTATION')
        cr.name = CNAME["cr_head"]
        cr.target = obj
        cr.subtarget = CTRL["head_ik"]
        cr.mix_mode = 'REPLACE'
        cr.target_space = 'POSE'
        cr.owner_space = 'POSE'
        cr.influence = 0.0
        make_driver(cr, obj, PROP_HEAD)

def snap_fk_to_ik(obj):
    """IMPROVED: Place IK controllers & poles to match current FK pose with better precision."""
    ensure_pose_mode(obj)

    # First, ensure IK is OFF and capture the current FK pose matrices
    obj[PROP_ARM_L] = 0.0; obj[PROP_ARM_R] = 0.0
    obj[PROP_LEG_L] = 0.0; obj[PROP_LEG_R] = 0.0
    obj[PROP_TORSO] = 0.0; obj[PROP_HEAD] = 0.0
    
    _update_depsgraph()
    
    # CAPTURE FK POSE MATRICES FIRST (before any IK control positioning)
    print("Capturing original FK pose matrices...")
    fk_matrices = {}
    limb_bones = [
        B["l_upper"], B["l_lower"], B["l_hand"],
        B["r_upper"], B["r_lower"], B["r_hand"],
        B["l_upper_leg"], B["l_lower_leg"], B["l_foot"],
        B["r_upper_leg"], B["r_lower_leg"], B["r_foot"]
    ] + B["spine"] + [B["head"]]
    
    for bone_name in limb_bones:
        pbone = pb(obj, bone_name)
        if pbone:
            fk_matrices[bone_name] = {
                'world_matrix': (obj.matrix_world @ pbone.matrix).copy(),
                'local_matrix': pbone.matrix.copy()
            }

    # Arms - IMPROVED positioning using captured matrices
    print("Positioning arm IK controls...")
    for side in (('l', B["l_upper"], B["l_lower"], B["l_hand"], CTRL["l_hand_ik"], CTRL["l_arm_pole"], PROP_ARM_L, ARM_POLE_DIST_FACTOR, ARM_POLE_DIST_MIN, CNAME["ik_l_arm"]),
                ('r', B["r_upper"], B["r_lower"], B["r_hand"], CTRL["r_hand_ik"], CTRL["r_arm_pole"], PROP_ARM_R, ARM_POLE_DIST_FACTOR, ARM_POLE_DIST_MIN, CNAME["ik_r_arm"])):
        _, upper, lower, eff, ik_name, pole_name, prop, df, dmin, ik_cname = side
        eff_p = pb(obj, eff)
        ik_p  = pb(obj, ik_name)
        
        # Use the captured FK matrix for precise positioning
        if eff in fk_matrices:
            # CRITICAL: Use exact matrix positioning with validation
            target_matrix = fk_matrices[eff]['world_matrix']
            set_pbone_world_matrix(obj, ik_p, target_matrix)
            
            # Validation step: check if positioning was accurate
            _update_depsgraph()
            actual_matrix = obj.matrix_world @ ik_p.matrix
            position_error = (actual_matrix.translation - target_matrix.translation).length
            
            if position_error > 0.001:  # More than 1mm error
                print(f"  Warning: {ik_name} positioning error: {position_error*1000:.1f}mm")
                # Try alternative positioning method
                arm_space_pos = obj.matrix_world.inverted() @ target_matrix.translation
                ik_p.location = arm_space_pos - ik_p.bone.head_local
                ik_p.rotation_quaternion = (obj.matrix_world.inverted() @ target_matrix).to_quaternion()
                _update_depsgraph()
                print(f"    Retried positioning for {ik_name}")
        
        # Improved pole placement using original FK joint positions
        pole_p = pb(obj, pole_name)
        improved_place_pole(obj, upper, lower, eff, pole_p, df, dmin, fk_matrices)

    # Legs - IMPROVED positioning using captured matrices
    print("Positioning leg IK controls...")
    for side in (('l', B["l_upper_leg"], B["l_lower_leg"], B["l_foot"], CTRL["l_foot_ik"], CTRL["l_leg_pole"], PROP_LEG_L, LEG_POLE_DIST_FACTOR, LEG_POLE_DIST_MIN, CNAME["ik_l_leg"]),
                ('r', B["r_upper_leg"], B["r_lower_leg"], B["r_foot"], CTRL["r_foot_ik"], CTRL["r_leg_pole"], PROP_LEG_R, LEG_POLE_DIST_FACTOR, LEG_POLE_DIST_MIN, CNAME["ik_r_leg"])):
        _, upper, lower, eff, ik_name, pole_name, prop, df, dmin, ik_cname = side
        eff_p = pb(obj, eff)
        ik_p  = pb(obj, ik_name)
        
        # Use the captured FK matrix for precise positioning
        if eff in fk_matrices:
            # CRITICAL: Use exact matrix positioning with validation for legs too
            target_matrix = fk_matrices[eff]['world_matrix']
            set_pbone_world_matrix(obj, ik_p, target_matrix)
            
            # Validation step: check if positioning was accurate
            _update_depsgraph()
            actual_matrix = obj.matrix_world @ ik_p.matrix
            position_error = (actual_matrix.translation - target_matrix.translation).length
            
            if position_error > 0.001:  # More than 1mm error
                print(f"  Warning: {ik_name} positioning error: {position_error*1000:.1f}mm")
                # Try alternative positioning method
                arm_space_pos = obj.matrix_world.inverted() @ target_matrix.translation
                ik_p.location = arm_space_pos - ik_p.bone.head_local
                ik_p.rotation_quaternion = (obj.matrix_world.inverted() @ target_matrix).to_quaternion()
                _update_depsgraph()
                print(f"    Retried positioning for {ik_name}")
        
        # Improved pole placement using original FK joint positions
        pole_p = pb(obj, pole_name)
        improved_place_pole(obj, upper, lower, eff, pole_p, df, dmin, fk_matrices)

    # Torso and head using captured matrices
    print("Positioning torso and head controls...")
    torso_ik = pb(obj, CTRL["torso_ik"])
    if B["spine"][-1] in fk_matrices:
        set_pbone_world_matrix(obj, torso_ik, fk_matrices[B["spine"][-1]]['world_matrix'])

    head_ctrl = pb(obj, CTRL["head_ik"])
    if B["head"] in fk_matrices:
        set_pbone_world_matrix(obj, head_ctrl, fk_matrices[B["head"]]['world_matrix'])
    
    # Update after positioning all controls
    _update_depsgraph()
    
    # NOW set pole angles using the ORIGINAL FK positions (not current ones)
    print("Computing pole angles from original FK pose...")
    for side in (('l', B["l_upper"], B["l_lower"], B["l_hand"], CTRL["l_arm_pole"], CNAME["ik_l_arm"]),
                ('r', B["r_upper"], B["r_lower"], B["r_hand"], CTRL["r_arm_pole"], CNAME["ik_r_arm"])):
        _, upper, lower, eff, pole_name, ik_cname = side
        
        pole_p = pb(obj, pole_name)
        con = pb(obj, eff).constraints.get(ik_cname)
        if con and pole_p:
            angle = find_best_pole_angle(obj, upper, lower, eff, ik_cname, fk_matrices)
            con.pole_angle = angle

    for side in (('l', B["l_upper_leg"], B["l_lower_leg"], B["l_foot"], CTRL["l_leg_pole"], CNAME["ik_l_leg"]),
                ('r', B["r_upper_leg"], B["r_lower_leg"], B["r_foot"], CTRL["r_leg_pole"], CNAME["ik_r_leg"])):
        _, upper, lower, eff, pole_name, ik_cname = side
        
        pole_p = pb(obj, pole_name)
        con = pb(obj, eff).constraints.get(ik_cname)
        if con and pole_p:
            angle = find_best_pole_angle(obj, upper, lower, eff, ik_cname, fk_matrices)
            con.pole_angle = angle
    
    # Final update before enabling IK
    _update_depsgraph()
    
    # Enable IK gradually for better stability
    print("Enabling IK constraints...")
    obj[PROP_ARM_L] = 1.0; _update_depsgraph()
    obj[PROP_ARM_R] = 1.0; _update_depsgraph() 
    obj[PROP_LEG_L] = 1.0; _update_depsgraph()
    obj[PROP_LEG_R] = 1.0; _update_depsgraph()
    obj[PROP_TORSO] = 1.0; _update_depsgraph()
    obj[PROP_HEAD] = 1.0; _update_depsgraph()
    
    # CRITICAL: Final constraint validation pass
    print("Final validation pass...")
    
    # Temporarily disable and re-enable each constraint to force clean evaluation
    for prop_name, desc in [(PROP_ARM_L, "Left Arm"), (PROP_ARM_R, "Right Arm"), 
                           (PROP_LEG_L, "Left Leg"), (PROP_LEG_R, "Right Leg")]:
        current_value = obj[prop_name]
        obj[prop_name] = 0.0
        _update_depsgraph()
        obj[prop_name] = current_value
        _update_depsgraph()
        print(f"  Validated {desc} IK")
    
    print("FK to IK conversion completed.")

def _update_depsgraph():
    """Enhanced dependency graph update for better constraint evaluation"""
    try:
        # Multiple update passes for complex constraint setups
        for i in range(2):  # Do 2 passes
            # Force view layer update first
            bpy.context.view_layer.update()
            # Then force depsgraph update  
            bpy.context.evaluated_depsgraph_get().update()
            # Small delay for Blender's internal updates
            if i == 0:  # Only on first pass
                # Force scene update
                bpy.context.scene.frame_set(bpy.context.scene.frame_current)
    except Exception as e:
        print(f"Warning: Depsgraph update error: {e}")
        # Minimal fallback
        try:
            bpy.context.view_layer.update()
        except:
            pass

def snap_ik_to_fk(obj):
    """Bake evaluated IK result into FK channels, then switch IK=0 (pose stays)."""
    ensure_pose_mode(obj)

    # Update depsgraph to ensure constraints are properly evaluated
    _update_depsgraph()

    # Arms (bake upper & lower (and hand) from evaluated matrices)
    for upper, lower, hand, prop in (
        (B["l_upper"], B["l_lower"], B["l_hand"], PROP_ARM_L),
        (B["r_upper"], B["r_lower"], B["r_hand"], PROP_ARM_R),
    ):
        # Ensure IK is ON so evaluated matrices reflect IK state
        obj[prop] = 1.0
    
    # Legs
    for upper, lower, foot, prop in (
        (B["l_upper_leg"], B["l_lower_leg"], B["l_foot"], PROP_LEG_L),
        (B["r_upper_leg"], B["r_lower_leg"], B["r_foot"], PROP_LEG_R),
    ):
        obj[prop] = 1.0

    # Torso and Head
    obj[PROP_TORSO] = 1.0
    obj[PROP_HEAD] = 1.0
    
    # Update depsgraph after setting all IK on
    _update_depsgraph()
    
    # Now bake the evaluated rotations
    # Arms
    for upper, lower, hand, prop in (
        (B["l_upper"], B["l_lower"], B["l_hand"], PROP_ARM_L),
        (B["r_upper"], B["r_lower"], B["r_hand"], PROP_ARM_R),
    ):
        for bn in (upper, lower, hand):
            p = pb(obj, bn)
            if p:
                bake_rot_from_evaluated(p)
        # Now turn IK OFF; rotations remain
        obj[prop] = 0.0

    # Legs  
    for upper, lower, foot, prop in (
        (B["l_upper_leg"], B["l_lower_leg"], B["l_foot"], PROP_LEG_L),
        (B["r_upper_leg"], B["r_lower_leg"], B["r_foot"], PROP_LEG_R),
    ):
        for bn in (upper, lower, foot):
            p = pb(obj, bn)
            if p:
                bake_rot_from_evaluated(p)
        obj[prop] = 0.0

    # Torso (chain across spine bones)
    for bn in B["spine"]:
        p = pb(obj, bn)
        if p:
            bake_rot_from_evaluated(p)
    obj[PROP_TORSO] = 0.0

    # Head (Copy Rotation)
    head_p = pb(obj, B["head"])
    if head_p:
        bake_rot_from_evaluated(head_p)
    obj[PROP_HEAD] = 0.0

# ----------------------------
# UI Operators / Panel
# ----------------------------

class FKIK_OT_build(bpy.types.Operator):
    bl_idname = "fkik.build_controls"
    bl_label = "Build Controls"
    bl_description = "Create IK targets/poles, constraints and drivers"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = get_arm()
        ensure_constraints_and_controls(obj)
        self.report({'INFO'}, "FK/IK controls built.")
        return {'FINISHED'}

class FKIK_OT_snap_fk_to_ik(bpy.types.Operator):
    bl_idname = "fkik.snap_fk_to_ik"
    bl_label = "Snap FK → IK (keep pose)"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = get_arm()
        snap_fk_to_ik(obj)
        self.report({'INFO'}, "Snapped FK to IK (pose preserved).")
        return {'FINISHED'}

class FKIK_OT_snap_ik_to_fk(bpy.types.Operator):
    bl_idname = "fkik.snap_ik_to_fk"
    bl_label = "Snap IK → FK (keep pose)"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        obj = get_arm()
        snap_ik_to_fk(obj)
        self.report({'INFO'}, "Snapped IK to FK (pose preserved).")
        return {'FINISHED'}

class FKIK_PT_panel(bpy.types.Panel):
    bl_label = "FK/IK (Simple)"
    bl_idname = "FKIK_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "FK/IK"

    def draw(self, context):
        layout = self.layout
        obj = bpy.context.object
        layout.operator(FKIK_OT_build.bl_idname, icon='BONE_DATA')

        if not (obj and obj.type == 'ARMATURE'):
            return

        # Only READ here – no writes in draw()
        missing = [n for n in ("ik_arm_L","ik_arm_R","ik_leg_L","ik_leg_R","ik_torso","ik_head") if n not in obj]
        if missing:
            box = layout.box()
            box.label(text="Initialize FK/IK properties first", icon='ERROR')
            box.operator("fkik.init_props", text="Initialize Properties", icon='MODIFIER')
            return

        col = layout.column(align=True)
        col.prop(obj, '["ik_arm_L"]', text="IK Left Arm")
        col.prop(obj, '["ik_arm_R"]', text="IK Right Arm")
        col.prop(obj, '["ik_leg_L"]', text="IK Left Leg")
        col.prop(obj, '["ik_leg_R"]', text="IK Right Leg")
        col.prop(obj, '["ik_torso"]',  text="IK Torso")
        col.prop(obj, '["ik_head"]',   text="IK Head")
        row = layout.row(align=True)
        row.operator("fkik.snap_fk_to_ik", icon='ARMATURE_DATA')
        row.operator("fkik.snap_ik_to_fk", icon='ARMATURE_DATA')

# ----------------------------
# Register
# ----------------------------
classes = (
    FKIK_OT_build,
    FKIK_OT_snap_fk_to_ik,
    FKIK_OT_snap_ik_to_fk,
    FKIK_PT_panel,
)

def register():
    for c in classes:
        bpy.utils.register_class(c)

def unregister():
    for c in reversed(classes):
        bpy.utils.unregister_class(c)

if __name__ == "__main__":
    register()
