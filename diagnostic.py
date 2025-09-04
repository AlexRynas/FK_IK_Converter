"""
Enhanced Diagnostic Tools for FK/IK Converter
=============================================

Comprehensive diagnostic utilities for analyzing pose changes during FK/IK conversion.
Provides detailed position and rotation analysis to help debug and optimize conversions.

Features:
- Detailed bone position and rotation capture
- Before/after comparison with precise error metrics
- Support for multiple rotation modes (Quaternion, Euler, Axis-Angle)
- Global functions for easy interactive use

Usage:
1. capture_before()  # Before FK→IK conversion
2. Run FK→IK conversion
3. capture_after()   # Will automatically compare and show results
"""

import bpy
import math
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from mathutils import Vector, Quaternion


class DiagnosticConfig:
    """Configuration for diagnostic thresholds and settings."""
    POSITION_THRESHOLD = 0.001  # 1mm in Blender units
    ROTATION_THRESHOLD = math.radians(0.1)  # 0.1 degrees
    
    # Key bones to analyze for diagnostics
    KEY_BONES = [
        "LeftArm", "LeftForeArm", "LeftHand",
        "RightArm", "RightForeArm", "RightHand",
        "LeftUpLeg", "LeftLeg", "LeftFoot",
        "RightUpLeg", "RightLeg", "RightFoot"
    ]
    
    IK_PROPERTIES = [
        "ik_arm_L", "ik_arm_R", "ik_leg_L", 
        "ik_leg_R", "ik_torso", "ik_head"
    ]


@dataclass
class BoneTransformData:
    """Container for comprehensive bone transformation data."""
    name: str
    world_position: List[float]
    world_quaternion: List[float]
    world_euler: List[float]
    local_position: List[float]
    local_quaternion: Optional[List[float]]
    local_euler: Optional[List[float]]
    rotation_mode: str


class PoseAnalyzer:
    """Analyzes and captures detailed pose information."""
    
    @staticmethod
    def capture_pose_data(armature: bpy.types.Object) -> Dict[str, BoneTransformData]:
        """Capture comprehensive pose data for analysis."""
        if not armature or armature.type != 'ARMATURE':
            raise ValueError("Invalid armature object provided")
        
        if armature.mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
        
        # Force dependency graph update
        bpy.context.evaluated_depsgraph_get().update()
        
        pose_data = {}
        
        for bone_name in DiagnosticConfig.KEY_BONES:
            pose_bone = armature.pose.bones.get(bone_name)
            if not pose_bone:
                continue
            
            # Calculate world matrix and extract components
            world_matrix = armature.matrix_world @ pose_bone.matrix
            world_position = world_matrix.translation
            world_quaternion = world_matrix.to_quaternion()
            world_euler = world_quaternion.to_euler()
            
            # Extract local rotation based on bone's rotation mode
            local_quaternion = None
            local_euler = None
            
            if pose_bone.rotation_mode == 'QUATERNION':
                local_quaternion = list(pose_bone.rotation_quaternion)
            else:
                local_euler = list(pose_bone.rotation_euler)
            
            # Create bone data structure
            bone_data = BoneTransformData(
                name=bone_name,
                world_position=list(world_position),
                world_quaternion=list(world_quaternion),
                world_euler=list(world_euler),
                local_position=list(pose_bone.location),
                local_quaternion=local_quaternion,
                local_euler=local_euler,
                rotation_mode=pose_bone.rotation_mode
            )
            
            pose_data[bone_name] = bone_data
        
        return pose_data
    
    @staticmethod
    def print_pose_summary(armature: bpy.types.Object, pose_data: Dict[str, BoneTransformData]) -> None:
        """Print detailed summary of captured pose data."""
        print(f"\n=== POSE CAPTURE SUMMARY ===")
        print(f"Armature: {armature.name}")
        
        # Display IK properties status
        print(f"\nIK Properties Status:")
        for prop_name in DiagnosticConfig.IK_PROPERTIES:
            value = armature.get(prop_name, "Not Found")
            print(f"  {prop_name}: {value}")
        
        # Display bone data
        print(f"\nDetailed Bone Analysis:")
        for bone_name, bone_data in pose_data.items():
            print(f"  {bone_name}:")
            
            # World space information
            pos = bone_data.world_position
            euler = bone_data.world_euler
            print(f"    World Pos: ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
            print(f"    World Rot: ({math.degrees(euler[0]):.1f}°, "
                  f"{math.degrees(euler[1]):.1f}°, {math.degrees(euler[2]):.1f}°)")
            
            # Local space information
            local_pos = bone_data.local_position
            print(f"    Local Pos: ({local_pos[0]:.4f}, {local_pos[1]:.4f}, {local_pos[2]:.4f})")
            
            if bone_data.rotation_mode == 'QUATERNION' and bone_data.local_quaternion:
                q = bone_data.local_quaternion
                print(f"    Local Quat: ({q[0]:.3f}, {q[1]:.3f}, {q[2]:.3f}, {q[3]:.3f})")
            elif bone_data.local_euler:
                e = bone_data.local_euler
                print(f"    Local Euler: ({math.degrees(e[0]):.1f}°, "
                      f"{math.degrees(e[1]):.1f}°, {math.degrees(e[2]):.1f}°)")
        
        # Display active constraints
        PoseAnalyzer._print_active_constraints(armature)
    
    @staticmethod
    def _print_active_constraints(armature: bpy.types.Object) -> None:
        """Print information about active IK constraints."""
        print(f"\nActive IK Constraints:")
        constraint_found = False
        
        for bone_name in DiagnosticConfig.KEY_BONES:
            pose_bone = armature.pose.bones.get(bone_name)
            if not pose_bone:
                continue
                
            for constraint in pose_bone.constraints:
                if constraint.type == 'IK' and constraint.influence > 0.001:
                    print(f"  {bone_name}.{constraint.name}: "
                          f"influence={constraint.influence:.3f}, "
                          f"pole_angle={constraint.pole_angle:.3f}")
                    constraint_found = True
        
        if not constraint_found:
            print("  No active IK constraints found")


class PoseComparator:
    """Compares pose data and analyzes changes."""
    
    @staticmethod
    def compare_poses(before_data: Dict[str, BoneTransformData], 
                     after_data: Dict[str, BoneTransformData]) -> None:
        """Compare two pose captures and display detailed analysis."""
        if not before_data or not after_data:
            print("ERROR: Invalid data for comparison")
            return
        
        print(f"\n=== DETAILED POSE COMPARISON ===")
        print("Changes detected during FK→IK conversion:\n")
        
        # Initialize tracking variables
        total_position_change = 0.0
        total_rotation_change = 0.0
        bones_with_position_changes = 0
        bones_with_rotation_changes = 0
        
        # Compare each bone
        for bone_name in before_data:
            if bone_name not in after_data:
                continue
            
            before_bone = before_data[bone_name]
            after_bone = after_data[bone_name]
            
            # Calculate position changes
            pos_change, has_pos_change = PoseComparator._calculate_position_change(
                before_bone, after_bone
            )
            
            # Calculate rotation changes
            rot_change, has_rot_change = PoseComparator._calculate_rotation_change(
                before_bone, after_bone
            )
            
            # Report significant changes
            if has_pos_change or has_rot_change:
                print(f"{bone_name}:")
                
                if has_pos_change:
                    bones_with_position_changes += 1
                    total_position_change += pos_change
                    PoseComparator._report_position_change(before_bone, after_bone, pos_change)
                
                if has_rot_change:
                    bones_with_rotation_changes += 1
                    total_rotation_change += rot_change
                    PoseComparator._report_rotation_change(before_bone, after_bone, rot_change)
                
                print()  # Empty line for readability
        
        # Print summary statistics
        PoseComparator._print_comparison_summary(
            total_position_change, total_rotation_change,
            bones_with_position_changes, bones_with_rotation_changes
        )
    
    @staticmethod
    def _calculate_position_change(before: BoneTransformData, 
                                 after: BoneTransformData) -> Tuple[float, bool]:
        """Calculate position change between two bone states."""
        pos_before = Vector(before.world_position)
        pos_after = Vector(after.world_position)
        position_change = (pos_after - pos_before).length
        
        has_significant_change = position_change > DiagnosticConfig.POSITION_THRESHOLD
        return position_change, has_significant_change
    
    @staticmethod
    def _calculate_rotation_change(before: BoneTransformData, 
                                 after: BoneTransformData) -> Tuple[float, bool]:
        """Calculate rotation change between two bone states."""
        quat_before = Quaternion(before.world_quaternion)
        quat_after = Quaternion(after.world_quaternion)
        
        # Calculate angular difference
        rotation_difference = quat_before.rotation_difference(quat_after)
        rotation_change = abs(rotation_difference.angle)
        
        has_significant_change = rotation_change > DiagnosticConfig.ROTATION_THRESHOLD
        return rotation_change, has_significant_change
    
    @staticmethod
    def _report_position_change(before: BoneTransformData, after: BoneTransformData, 
                              change_magnitude: float) -> None:
        """Report detailed position change information."""
        pos_before = Vector(before.world_position)
        pos_after = Vector(after.world_position)
        delta = pos_after - pos_before
        
        print(f"  Position: moved {change_magnitude:.4f} units ({change_magnitude*1000:.1f}mm)")
        print(f"    Delta: ({delta.x:.4f}, {delta.y:.4f}, {delta.z:.4f})")
    
    @staticmethod
    def _report_rotation_change(before: BoneTransformData, after: BoneTransformData, 
                              change_magnitude: float) -> None:
        """Report detailed rotation change information."""
        print(f"  Rotation: changed {math.degrees(change_magnitude):.2f}°")
        
        # Show Euler angle differences for additional context
        euler_before = Vector(before.world_euler)
        euler_after = Vector(after.world_euler)
        euler_delta = euler_after - euler_before
        
        print(f"    Euler Delta: ({math.degrees(euler_delta.x):.2f}°, "
              f"{math.degrees(euler_delta.y):.2f}°, {math.degrees(euler_delta.z):.2f}°)")
    
    @staticmethod
    def _print_comparison_summary(total_pos_change: float, total_rot_change: float,
                                bones_with_pos_change: int, bones_with_rot_change: int) -> None:
        """Print summary statistics of the comparison."""
        print(f"COMPARISON SUMMARY:")
        print(f"  Bones with position changes: {bones_with_pos_change}")
        print(f"  Bones with rotation changes: {bones_with_rot_change}")
        print(f"  Total position change: {total_pos_change:.4f} units")
        print(f"  Total rotation change: {math.degrees(total_rot_change):.2f}°")
        
        if bones_with_pos_change > 0:
            avg_pos_change = total_pos_change / bones_with_pos_change
            print(f"  Average position change: {avg_pos_change:.4f} units")
        
        if bones_with_rot_change > 0:
            avg_rot_change = total_rot_change / bones_with_rot_change
            print(f"  Average rotation change: {math.degrees(avg_rot_change):.2f}°")


class DiagnosticSession:
    """Manages diagnostic capture sessions."""
    
    def __init__(self):
        """Initialize diagnostic session."""
        self._before_data: Optional[Dict[str, BoneTransformData]] = None
        self._after_data: Optional[Dict[str, BoneTransformData]] = None
    
    def capture_before_state(self) -> Dict[str, BoneTransformData]:
        """Capture the 'before' state for comparison."""
        armature = bpy.context.object
        if not armature or armature.type != 'ARMATURE':
            raise ValueError("Please select an armature object and switch to pose mode")
        
        print("=== CAPTURING BEFORE STATE ===")
        self._before_data = PoseAnalyzer.capture_pose_data(armature)
        PoseAnalyzer.print_pose_summary(armature, self._before_data)
        
        print("\nBEFORE state captured. Now run FK→IK conversion and then call capture_after_state()")
        return self._before_data
    
    def capture_after_state(self) -> Dict[str, BoneTransformData]:
        """Capture the 'after' state and perform comparison."""
        armature = bpy.context.object
        if not armature or armature.type != 'ARMATURE':
            raise ValueError("Please select an armature object and switch to pose mode")
        
        print("\n=== CAPTURING AFTER STATE ===")
        self._after_data = PoseAnalyzer.capture_pose_data(armature)
        PoseAnalyzer.print_pose_summary(armature, self._after_data)
        
        # Perform comparison if before data exists
        if self._before_data and self._after_data:
            print("\n" + "=" * 60)
            PoseComparator.compare_poses(self._before_data, self._after_data)
        else:
            print("ERROR: No BEFORE data available for comparison. Run capture_before_state() first.")
        
        return self._after_data
    
    def get_before_data(self) -> Optional[Dict[str, BoneTransformData]]:
        """Get the captured before data."""
        return self._before_data
    
    def get_after_data(self) -> Optional[Dict[str, BoneTransformData]]:
        """Get the captured after data."""
        return self._after_data


# Global session instance for interactive use
_diagnostic_session = DiagnosticSession()


# Global convenience functions for easy interactive use
def enhanced_pose_capture() -> Optional[Dict[str, BoneTransformData]]:
    """Capture current pose data (standalone function for compatibility)."""
    try:
        armature = bpy.context.object
        if not armature or armature.type != 'ARMATURE':
            print("ERROR: Select armature in pose mode")
            return None
        
        pose_data = PoseAnalyzer.capture_pose_data(armature)
        PoseAnalyzer.print_pose_summary(armature, pose_data)
        return pose_data
    except Exception as e:
        print(f"Error during pose capture: {e}")
        return None


def compare_enhanced_captures(before_data: Dict[str, BoneTransformData], 
                            after_data: Dict[str, BoneTransformData]) -> None:
    """Compare two enhanced captures (standalone function for compatibility)."""
    PoseComparator.compare_poses(before_data, after_data)


def capture_before() -> Optional[Dict[str, BoneTransformData]]:
    """Capture BEFORE state using global session."""
    try:
        return _diagnostic_session.capture_before_state()
    except Exception as e:
        print(f"Error capturing before state: {e}")
        return None


def capture_after() -> Optional[Dict[str, BoneTransformData]]:
    """Capture AFTER state and compare using global session."""
    try:
        return _diagnostic_session.capture_after_state()
    except Exception as e:
        print(f"Error capturing after state: {e}")
        return None


# Make functions globally available for interactive use
import builtins
builtins.enhanced_pose_capture = enhanced_pose_capture
builtins.compare_enhanced_captures = compare_enhanced_captures
builtins.capture_before = capture_before
builtins.capture_after = capture_after

# Display usage instructions
print("Enhanced FK/IK Diagnostic Tools Loaded!")
print("=" * 50)
print("Usage Instructions:")
print("1. capture_before()  # Capture pose before FK→IK conversion")
print("2. Run your FK→IK conversion operation")
print("3. capture_after()   # Capture pose after conversion and auto-compare")
print()
print("Alternative usage:")
print("- enhanced_pose_capture()  # Capture current pose anytime")
print("- compare_enhanced_captures(before_data, after_data)  # Manual comparison")
print("=" * 50)
