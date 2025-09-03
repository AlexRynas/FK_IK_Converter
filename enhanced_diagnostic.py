# Enhanced diagnostic to capture both position AND rotation changes
import bpy
from mathutils import Vector, Quaternion
import math

def enhanced_pose_capture():
    """Capture both position and rotation data for detailed analysis"""
    obj = bpy.context.object
    if not obj or obj.type != 'ARMATURE':
        print("ERROR: Select armature in pose mode")
        return None
    
    if obj.mode != 'POSE':
        bpy.ops.object.mode_set(mode='POSE')
    
    # Force update
    bpy.context.evaluated_depsgraph_get().update()
    
    print(f"\n=== ENHANCED POSE CAPTURE ===")
    print(f"Armature: {obj.name}")
    
    # Key bones to analyze
    key_bones = ["LeftArm", "LeftForeArm", "LeftHand", 
                 "RightArm", "RightForeArm", "RightHand",
                 "LeftUpLeg", "LeftLeg", "LeftFoot",
                 "RightUpLeg", "RightLeg", "RightFoot"]
    
    # Check IK properties
    ik_props = ["ik_arm_L", "ik_arm_R", "ik_leg_L", "ik_leg_R", "ik_torso", "ik_head"]
    print(f"\nIK Properties:")
    for prop in ik_props:
        value = obj.get(prop, "Not Found")
        print(f"  {prop}: {value}")
    
    bone_data = {}
    print(f"\nDetailed Bone Data:")
    
    for bone_name in key_bones:
        pbone = obj.pose.bones.get(bone_name)
        if pbone:
            # World matrix
            world_matrix = obj.matrix_world @ pbone.matrix
            
            # Extract components
            world_pos = world_matrix.translation
            world_quat = world_matrix.to_quaternion()
            world_euler = world_quat.to_euler()
            
            bone_data[bone_name] = {
                'position': list(world_pos),
                'quaternion': list(world_quat),
                'euler': list(world_euler),
                'local_position': list(pbone.location),
                'local_rotation_quaternion': list(pbone.rotation_quaternion) if pbone.rotation_mode == 'QUATERNION' else None,
                'local_rotation_euler': list(pbone.rotation_euler) if pbone.rotation_mode != 'QUATERNION' else None,
                'rotation_mode': pbone.rotation_mode
            }
            
            print(f"  {bone_name}:")
            print(f"    World Pos: ({world_pos.x:.4f}, {world_pos.y:.4f}, {world_pos.z:.4f})")
            print(f"    World Rot: ({math.degrees(world_euler.x):.1f}°, {math.degrees(world_euler.y):.1f}°, {math.degrees(world_euler.z):.1f}°)")
            print(f"    Local Pos: ({pbone.location.x:.4f}, {pbone.location.y:.4f}, {pbone.location.z:.4f})")
            if pbone.rotation_mode == 'QUATERNION':
                q = pbone.rotation_quaternion
                print(f"    Local Quat: ({q.w:.3f}, {q.x:.3f}, {q.y:.3f}, {q.z:.3f})")
            else:
                e = pbone.rotation_euler
                print(f"    Local Euler: ({math.degrees(e.x):.1f}°, {math.degrees(e.y):.1f}°, {math.degrees(e.z):.1f}°)")
    
    # Check constraint influences
    print(f"\nActive IK Constraints:")
    for bone_name in key_bones:
        pbone = obj.pose.bones.get(bone_name)
        if pbone:
            for con in pbone.constraints:
                if con.type == 'IK' and con.influence > 0.001:
                    print(f"  {bone_name}.{con.name}: influence={con.influence:.3f}, pole_angle={con.pole_angle:.3f}")
    
    return bone_data

def compare_enhanced_captures(before_data, after_data):
    """Compare two enhanced captures showing position AND rotation changes"""
    if not before_data or not after_data:
        print("ERROR: Invalid data for comparison")
        return
    
    print(f"\n=== ENHANCED COMPARISON ===")
    print("Changes in FK→IK conversion:\n")
    
    pos_threshold = 0.001  # 1mm
    rot_threshold = math.radians(0.1)  # 0.1 degrees
    
    total_pos_change = 0
    total_rot_change = 0
    bones_with_pos_change = 0
    bones_with_rot_change = 0
    
    for bone_name in before_data:
        if bone_name not in after_data:
            continue
            
        before = before_data[bone_name]
        after = after_data[bone_name]
        
        # Position change
        pos1 = Vector(before['position'])
        pos2 = Vector(after['position'])
        pos_diff = (pos2 - pos1).length
        
        # Rotation change (using quaternions for accuracy)
        quat1 = Quaternion(before['quaternion'])
        quat2 = Quaternion(after['quaternion'])
        
        # Calculate angular difference
        quat_diff = quat1.rotation_difference(quat2)
        angle_diff = abs(quat_diff.angle)
        
        # Check if changes are significant
        has_pos_change = pos_diff > pos_threshold
        has_rot_change = angle_diff > rot_threshold
        
        if has_pos_change or has_rot_change:
            print(f"{bone_name}:")
            
            if has_pos_change:
                bones_with_pos_change += 1
                total_pos_change += pos_diff
                delta = pos2 - pos1
                print(f"  Position: moved {pos_diff:.4f} units ({pos_diff*1000:.1f}mm)")
                print(f"    Delta: ({delta.x:.4f}, {delta.y:.4f}, {delta.z:.4f})")
            
            if has_rot_change:
                bones_with_rot_change += 1
                total_rot_change += angle_diff
                print(f"  Rotation: changed {math.degrees(angle_diff):.2f}°")
                
                # Show euler angle differences
                euler1 = Vector(before['euler'])
                euler2 = Vector(after['euler'])
                euler_diff = euler2 - euler1
                print(f"    Euler Delta: ({math.degrees(euler_diff.x):.2f}°, {math.degrees(euler_diff.y):.2f}°, {math.degrees(euler_diff.z):.2f}°)")
            
            print()
    
    print(f"SUMMARY:")
    print(f"  Bones with position changes: {bones_with_pos_change}")
    print(f"  Bones with rotation changes: {bones_with_rot_change}")
    print(f"  Total position change: {total_pos_change:.4f} units")
    print(f"  Total rotation change: {math.degrees(total_rot_change):.2f}°")
    
    if bones_with_pos_change > 0:
        print(f"  Average position change: {total_pos_change/bones_with_pos_change:.4f} units")
    if bones_with_rot_change > 0:
        print(f"  Average rotation change: {math.degrees(total_rot_change/bones_with_rot_change):.2f}°")

# Global storage for comparison
_before_capture = None
_after_capture = None

def capture_before():
    """Capture BEFORE state"""
    global _before_capture
    print("=== CAPTURING BEFORE STATE ===")
    _before_capture = enhanced_pose_capture()
    print("BEFORE state captured. Now run FK→IK conversion and then capture_after()")
    return _before_capture

def capture_after():
    """Capture AFTER state and compare"""
    global _after_capture, _before_capture
    print("\n=== CAPTURING AFTER STATE ===")
    _after_capture = enhanced_pose_capture()
    
    if _before_capture and _after_capture:
        print("\n" + "="*50)
        compare_enhanced_captures(_before_capture, _after_capture)
    else:
        print("ERROR: No BEFORE data to compare with. Run capture_before() first.")
    
    return _after_capture

# Make functions globally available
import builtins
builtins.enhanced_pose_capture = enhanced_pose_capture
builtins.compare_enhanced_captures = compare_enhanced_captures
builtins.capture_before = capture_before
builtins.capture_after = capture_after

print("Enhanced diagnostic loaded!")
print("Usage:")
print("1. capture_before()  # Before FK→IK conversion")
print("2. Run FK→IK conversion")
print("3. capture_after()   # Will automatically compare")
