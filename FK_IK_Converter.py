"""
FK/IK Converter for Blender
============================

A comprehensive tool for creating and managing Forward Kinematics (FK) and
Inverse Kinematics (IK) controls for character armatures in Blender.

Features:
- Automatic IK control creation for arms, legs, torso, and head
- Seamless FK to IK and IK to FK conversion while preserving poses
- Optimized pole angle calculation for natural limb positioning
- Clean UI panel for easy switching between FK and IK modes

Compatible with standard humanoid character rigs using common bone naming conventions.

Author: Refactored for improved maintainability and PEP 8 compliance
"""

import bpy
import math
from typing import Dict, List, Optional, Tuple, Any
from mathutils import Vector, Matrix, Quaternion
from dataclasses import dataclass


class Config:
    """Configuration constants for the FK/IK system."""
    
    # Pole target placement factors
    ARM_POLE_DISTANCE_FACTOR = 0.9
    LEG_POLE_DISTANCE_FACTOR = 0.9
    ARM_POLE_DISTANCE_MIN = 0.5  # meters
    LEG_POLE_DISTANCE_MIN = 0.5  # meters
    
    # IK target offsets from effector position
    HAND_IK_OFFSET = 0.2  # meters
    FOOT_IK_OFFSET = 0.2  # meters


class PropertyNames:
    """Names of custom properties stored on the Armature object."""
    ARM_LEFT = "ik_arm_L"
    ARM_RIGHT = "ik_arm_R"
    LEG_LEFT = "ik_leg_L"
    LEG_RIGHT = "ik_leg_R"
    TORSO = "ik_torso"
    HEAD = "ik_head"


class BoneNames:
    """Standard bone names mapping for humanoid characters."""
    
    # Core torso and head bones
    HIPS = "Hips"
    SPINE_BONES = ["Spine", "Spine1", "Spine2", "Spine3"]
    NECK_BONES = ["Neck", "Neck1"]
    HEAD = "Head"
    
    # Left arm chain
    LEFT_SHOULDER = "LeftShoulder"
    LEFT_UPPER_ARM = "LeftArm"
    LEFT_FOREARM = "LeftForeArm"
    LEFT_HAND = "LeftHand"
    
    # Right arm chain
    RIGHT_SHOULDER = "RightShoulder"
    RIGHT_UPPER_ARM = "RightArm"
    RIGHT_FOREARM = "RightForeArm"
    RIGHT_HAND = "RightHand"
    
    # Left leg chain
    LEFT_UPPER_LEG = "LeftUpLeg"
    LEFT_LOWER_LEG = "LeftLeg"
    LEFT_FOOT = "LeftFoot"
    LEFT_TOE = "LeftToeBase"
    
    # Right leg chain
    RIGHT_UPPER_LEG = "RightUpLeg"
    RIGHT_LOWER_LEG = "RightLeg"
    RIGHT_FOOT = "RightFoot"
    RIGHT_TOE = "RightToeBase"


class ControlNames:
    """Names for generated IK control bones."""
    
    # Arm controls
    LEFT_HAND_IK = "CTRL_L_Hand_IK"
    LEFT_ARM_POLE = "CTRL_L_Arm_Pole"
    RIGHT_HAND_IK = "CTRL_R_Hand_IK"
    RIGHT_ARM_POLE = "CTRL_R_Arm_Pole"
    
    # Leg controls
    LEFT_FOOT_IK = "CTRL_L_Foot_IK"
    LEFT_LEG_POLE = "CTRL_L_Leg_Pole"
    RIGHT_FOOT_IK = "CTRL_R_Foot_IK"
    RIGHT_LEG_POLE = "CTRL_R_Leg_Pole"
    
    # Torso and head controls
    TORSO_IK = "CTRL_Torso_IK"
    HEAD_IK = "CTRL_Head_IK"


class ConstraintNames:
    """Names for generated IK constraints."""
    IK_LEFT_ARM = "IK_L_Arm"
    IK_RIGHT_ARM = "IK_R_Arm"
    IK_LEFT_LEG = "IK_L_Leg"
    IK_RIGHT_LEG = "IK_R_Leg"
    IK_TORSO = "IK_Torso"
    COPY_ROTATION_HEAD = "CopyRot_Head"


@dataclass
class LimbConfiguration:
    """Configuration data for a limb chain (arm or leg)."""
    upper_bone: str
    lower_bone: str
    effector_bone: str
    ik_control: str
    pole_control: str
    property_name: str
    constraint_name: str
    pole_distance_factor: float
    pole_distance_min: float


class MathUtils:
    """Mathematical utilities for IK calculations."""
    
    @staticmethod
    def project_vector_on_plane(vector: Vector, plane_normal: Vector) -> Vector:
        """Project a vector onto a plane defined by its normal."""
        return vector - plane_normal * vector.dot(plane_normal)
    
    @staticmethod
    def safe_quaternion_from_matrix(matrix: Matrix) -> Quaternion:
        """Safely extract quaternion from matrix, handling edge cases."""
        try:
            return matrix.to_quaternion().normalized()
        except ValueError:
            # Return identity quaternion if matrix is invalid
            return Quaternion((1, 0, 0, 0))
    
    @staticmethod
    def calculate_signed_angle(vec1: Vector, vec2: Vector, normal: Vector) -> float:
        """Calculate signed angle between two vectors using a reference normal."""
        cos_angle = max(-1.0, min(1.0, vec1.dot(vec2)))
        sin_angle = normal.dot(vec1.cross(vec2))
        return math.atan2(sin_angle, cos_angle)


class BlenderUtils:
    """Utilities for working with Blender objects and bones."""
    
    @staticmethod
    def get_active_armature() -> bpy.types.Object:
        """Get the currently active armature object."""
        obj = bpy.context.object
        if not obj or obj.type != 'ARMATURE':
            raise RuntimeError("Please select an Armature object and run in Pose Mode.")
        return obj
    
    @staticmethod
    def ensure_pose_mode(armature: bpy.types.Object) -> None:
        """Ensure the armature is in pose mode."""
        if bpy.context.object != armature:
            bpy.context.view_layer.objects.active = armature
        if armature.mode != 'POSE':
            bpy.ops.object.mode_set(mode='POSE')
    
    @staticmethod
    def get_pose_bone(armature: bpy.types.Object, bone_name: str) -> Optional[bpy.types.PoseBone]:
        """Get pose bone by name, returning None if not found."""
        return armature.pose.bones.get(bone_name)
    
    @staticmethod
    def get_edit_bone(armature: bpy.types.Object, bone_name: str) -> Optional[bpy.types.EditBone]:
        """Get edit bone by name, returning None if not found."""
        return armature.data.edit_bones.get(bone_name)
    
    @staticmethod
    def calculate_limb_length(armature: bpy.types.Object, bone_a: str, bone_b: str) -> float:
        """Calculate world space distance between two bones."""
        pose_bone_a = BlenderUtils.get_pose_bone(armature, bone_a)
        pose_bone_b = BlenderUtils.get_pose_bone(armature, bone_b)
        
        if not pose_bone_a or not pose_bone_b:
            return 0.5  # Fallback length
            
        world_head_a = armature.matrix_world @ pose_bone_a.head
        world_head_b = armature.matrix_world @ pose_bone_b.head
        return (world_head_b - world_head_a).length
    
    @staticmethod
    def add_custom_property(obj: bpy.types.Object, name: str, default: float = 0.0, 
                          min_val: float = 0.0, max_val: float = 1.0) -> None:
        """Add a custom property to an object with UI metadata."""
        if name not in obj:
            obj[name] = default
            
        # Add RNA UI information for proper slider display
        rna_ui = obj.get("_RNA_UI")
        if rna_ui is None:
            obj["_RNA_UI"] = {}
            rna_ui = obj["_RNA_UI"]
            
        rna_ui[name] = {
            "min": min_val, 
            "max": max_val, 
            "soft_min": min_val, 
            "soft_max": max_val
        }
    
    @staticmethod
    def create_driver_for_constraint(constraint: bpy.types.Constraint, 
                                   target_object: bpy.types.Object, 
                                   property_name: str) -> None:
        """Create a driver that connects a constraint's influence to a custom property."""
        # Remove any existing drivers on influence
        try:
            constraint.driver_remove("influence")
        except:
            pass
            
        # Create new driver
        fcurve = constraint.driver_add("influence")
        driver = fcurve.driver
        driver.type = 'AVERAGE'
        
        # Add variable pointing to the custom property
        variable = driver.variables.new()
        variable.name = "ik_switch"
        target = variable.targets[0]
        target.id = target_object
        target.data_path = f'["{property_name}"]'
        
        driver.expression = "ik_switch"
    
    @staticmethod
    def update_dependency_graph() -> None:
        """Force update of Blender's dependency graph for constraint evaluation."""
        try:
            for _ in range(2):  # Multiple passes for complex constraint setups
                bpy.context.view_layer.update()
                bpy.context.evaluated_depsgraph_get().update()
        except Exception as e:
            print(f"Warning: Dependency graph update error: {e}")
            # Minimal fallback
            try:
                bpy.context.view_layer.update()
            except:
                pass


class BoneManipulator:
    """Handles bone creation and transformation operations."""
    
    @staticmethod
    def create_control_bone(armature: bpy.types.Object, new_bone_name: str, 
                          reference_bone_name: str, head_offset: Vector = None, 
                          tail_offset: Vector = None) -> Optional[bpy.types.PoseBone]:
        """Create a new control bone based on a reference bone."""
        if head_offset is None:
            head_offset = Vector((0, 0, 0))
        if tail_offset is None:
            tail_offset = Vector((0, 0, 0))
            
        bpy.ops.object.mode_set(mode='EDIT')
        
        reference_bone = BlenderUtils.get_edit_bone(armature, reference_bone_name)
        if not reference_bone:
            raise RuntimeError(f"Reference bone '{reference_bone_name}' not found")
        
        # Create or get existing bone
        if new_bone_name in armature.data.edit_bones:
            new_bone = armature.data.edit_bones[new_bone_name]
        else:
            new_bone = armature.data.edit_bones.new(new_bone_name)
        
        # Position the new bone
        new_bone.head = reference_bone.tail + head_offset
        new_bone.tail = reference_bone.tail + Vector((0, 0.1, 0)) + tail_offset
        new_bone.use_deform = False
        new_bone.parent = None
        
        bpy.ops.object.mode_set(mode='POSE')
        return BlenderUtils.get_pose_bone(armature, new_bone_name)
    
    @staticmethod
    def set_pose_bone_world_location(armature: bpy.types.Object, pose_bone: bpy.types.PoseBone, 
                                   world_location: Vector) -> None:
        """Set pose bone to a specific world location."""
        # Convert world location to armature space
        armature_space_location = armature.matrix_world.inverted() @ world_location
        # Set location relative to bone's rest position
        pose_bone.location = armature_space_location - pose_bone.bone.head_local
    
    @staticmethod
    def set_pose_bone_world_matrix(armature: bpy.types.Object, pose_bone: bpy.types.PoseBone, 
                                 world_matrix: Matrix) -> None:
        """Set pose bone to match a world space transformation matrix."""
        try:
            # Convert world matrix to armature space
            armature_matrix = armature.matrix_world.inverted() @ world_matrix
            
            # Calculate pose transformation from rest pose to target
            bone_rest_matrix = pose_bone.bone.matrix_local
            pose_transformation = bone_rest_matrix.inverted() @ armature_matrix
            
            # Clear existing transforms
            pose_bone.location = Vector((0, 0, 0))
            if pose_bone.rotation_mode == 'QUATERNION':
                pose_bone.rotation_quaternion = Quaternion((1, 0, 0, 0))
            else:
                pose_bone.rotation_euler = Vector((0, 0, 0))
            pose_bone.scale = Vector((1, 1, 1))
            
            # Apply calculated transformation
            pose_bone.location = pose_transformation.translation
            
            # Set rotation based on bone's rotation mode
            rotation_quaternion = pose_transformation.to_quaternion().normalized()
            BoneManipulator._set_pose_bone_rotation(pose_bone, rotation_quaternion)
            
        except Exception as e:
            print(f"Error in precise matrix setting: {e}")
            # Fallback method
            BoneManipulator._fallback_matrix_setting(armature, pose_bone, world_matrix)
    
    @staticmethod
    def _set_pose_bone_rotation(pose_bone: bpy.types.PoseBone, quaternion: Quaternion) -> None:
        """Set pose bone rotation in its preferred rotation mode."""
        if pose_bone.rotation_mode == 'QUATERNION':
            pose_bone.rotation_quaternion = quaternion
        elif pose_bone.rotation_mode in ('XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX'):
            pose_bone.rotation_euler = quaternion.to_euler(pose_bone.rotation_mode)
        elif pose_bone.rotation_mode == 'AXIS_ANGLE':
            axis, angle = quaternion.to_axis_angle()
            pose_bone.rotation_axis_angle = [angle, axis[0], axis[1], axis[2]]
    
    @staticmethod
    def _fallback_matrix_setting(armature: bpy.types.Object, pose_bone: bpy.types.PoseBone, 
                                world_matrix: Matrix) -> None:
        """Fallback method for setting pose bone transformation."""
        armature_matrix = armature.matrix_world.inverted() @ world_matrix
        pose_bone.location = armature_matrix.translation - pose_bone.bone.head_local
        
        rotation_quaternion = armature_matrix.to_quaternion()
        BoneManipulator._set_pose_bone_rotation(pose_bone, rotation_quaternion)
    
    @staticmethod
    def get_world_bone_position(armature: bpy.types.Object, pose_bone: bpy.types.PoseBone, 
                               point: str = 'tail') -> Vector:
        """Get world position of bone head, tail, or center."""
        if point == 'head':
            return armature.matrix_world @ pose_bone.head
        elif point == 'tail':
            return armature.matrix_world @ pose_bone.tail
        else:
            return armature.matrix_world @ pose_bone.matrix.translation
    
    @staticmethod
    def copy_pose_bone_rotation(source_bone: bpy.types.PoseBone, 
                              target_bone: bpy.types.PoseBone) -> None:
        """Copy rotation from source pose bone to target, handling different rotation modes."""
        # Extract rotation as quaternion from source
        if source_bone.rotation_mode == 'QUATERNION':
            source_quaternion = source_bone.rotation_quaternion.copy()
        elif source_bone.rotation_mode in ('XYZ', 'XZY', 'YXZ', 'YZX', 'ZXY', 'ZYX'):
            source_quaternion = source_bone.rotation_euler.to_quaternion()
        elif source_bone.rotation_mode == 'AXIS_ANGLE':
            angle, x, y, z = source_bone.rotation_axis_angle
            axis = Vector((x, y, z)).normalized()
            source_quaternion = Quaternion(axis, angle)
        else:
            source_quaternion = MathUtils.safe_quaternion_from_matrix(source_bone.matrix)
        
        # Apply rotation to target in its preferred mode
        BoneManipulator._set_pose_bone_rotation(target_bone, source_quaternion)
    
    @staticmethod
    def bake_evaluated_rotation(pose_bone: bpy.types.PoseBone) -> None:
        """Bake constraint-evaluated rotation into FK channels."""
        evaluated_quaternion = MathUtils.safe_quaternion_from_matrix(pose_bone.matrix)
        BoneManipulator._set_pose_bone_rotation(pose_bone, evaluated_quaternion)


class PoleTargetPlacer:
    """Handles pole target positioning for IK chains."""
    
    @staticmethod
    def place_pole_target(armature: bpy.types.Object, upper_bone: str, lower_bone: str,
                         effector_bone: str, pole_bone: bpy.types.PoseBone,
                         distance_factor: float, minimum_distance: float,
                         fk_matrices: Optional[Dict[str, Dict[str, Any]]] = None) -> None:
        """Position pole target for optimal IK chain behavior."""
        pose_upper = BlenderUtils.get_pose_bone(armature, upper_bone)
        pose_lower = BlenderUtils.get_pose_bone(armature, lower_bone) 
        pose_effector = BlenderUtils.get_pose_bone(armature, effector_bone)
        
        if not all([pose_upper, pose_lower, pose_effector]):
            return
        
        # Use original FK positions if available, otherwise current positions
        if (fk_matrices and 
            all(bone in fk_matrices for bone in [upper_bone, lower_bone, effector_bone])):
            world_root = fk_matrices[upper_bone]['world_matrix'].translation
            world_joint = fk_matrices[lower_bone]['world_matrix'].translation
            world_effector = fk_matrices[effector_bone]['world_matrix'].translation
        else:
            world_root = BoneManipulator.get_world_bone_position(armature, pose_upper, 'head')
            world_joint = BoneManipulator.get_world_bone_position(armature, pose_lower, 'head')
            world_effector = BoneManipulator.get_world_bone_position(armature, pose_effector, 'head')
        
        # Calculate limb vectors
        limb_vector_1 = (world_joint - world_root).normalized()
        limb_vector_2 = (world_effector - world_joint).normalized()
        
        # Calculate limb plane normal
        plane_normal = limb_vector_1.cross(limb_vector_2)
        if plane_normal.length < 1e-8:
            plane_normal = Vector((0, 0, 1))  # Fallback for straight limb
        plane_normal.normalize()
        
        # Determine pole direction using lower bone's axis
        if fk_matrices and lower_bone in fk_matrices:
            lower_axis_world = fk_matrices[lower_bone]['world_matrix'].to_3x3() @ Vector((0, 1, 0))
        else:
            lower_axis = pose_lower.bone.y_axis
            lower_axis_world = (armature.matrix_world.to_3x3() @ lower_axis).normalized()
        
        # Calculate pole direction (perpendicular to limb plane, biased toward lower axis)
        pole_direction = (plane_normal + 0.5 * lower_axis_world).normalized()
        
        # Calculate distance based on limb length
        limb_length = max((world_effector - world_root).length, minimum_distance)
        pole_distance = max(limb_length * distance_factor, minimum_distance)
        
        # Position pole target
        pole_position = world_joint + pole_direction * pole_distance
        BoneManipulator.set_pose_bone_world_location(armature, pole_bone, pole_position)


class PoleAngleOptimizer:
    """Optimizes pole angles for IK constraints."""
    
    @staticmethod
    def find_optimal_pole_angle(armature: bpy.types.Object, upper_bone: str,
                               lower_bone: str, effector_bone: str,
                               constraint_name: str, fk_matrices: Dict[str, Dict[str, Any]]) -> float:
        """Find optimal pole angle by testing multiple values and choosing best match."""
        
        if not all(bone in fk_matrices for bone in [upper_bone, lower_bone, effector_bone]):
            return 0.0
        
        # Get the constraint
        effector_pose_bone = BlenderUtils.get_pose_bone(armature, effector_bone)
        if not effector_pose_bone:
            return 0.0
        
        constraint = effector_pose_bone.constraints.get(constraint_name)
        if not constraint:
            return 0.0
        
        # Store original rotations for comparison
        original_rotations = {}
        test_bones = [upper_bone, lower_bone, effector_bone]
        
        for bone_name in test_bones:
            if bone_name in fk_matrices:
                original_rotations[bone_name] = fk_matrices[bone_name]['world_matrix'].to_quaternion()
        
        # Determine test angles based on limb side
        is_right_side = any('Right' in bone_name for bone_name in [upper_bone, lower_bone, effector_bone])
        
        if is_right_side:
            # For right side, test angles around 180°
            base_angles = [178.0, 179.0, 179.5, 179.7, 179.8, 179.9, 180.0, 
                          -180.0, -179.9, -179.8, -179.7, -179.5, -179.0, -178.0]
        else:
            # For left side, test small angles around 0°
            base_angles = [2.0, 1.5, 1.0, 0.5, 0.3, 0.2, 0.1, 0.0, 
                          -0.1, -0.2, -0.3, -0.5, -1.0, -1.5, -2.0]
        
        # Convert to radians
        test_angles = [math.radians(angle_deg) for angle_deg in base_angles]
        
        best_angle = 0.0
        best_error = float('inf')
        
        print(f"  Testing pole angles for {effector_bone}...")
        
        # Test each angle
        for test_angle in test_angles:
            constraint.pole_angle = test_angle
            BlenderUtils.update_dependency_graph()
            
            # Calculate total rotation error compared to original FK pose
            total_error = 0.0
            for bone_name in test_bones:
                if bone_name in original_rotations:
                    test_pose_bone = BlenderUtils.get_pose_bone(armature, bone_name)
                    if test_pose_bone:
                        current_matrix = armature.matrix_world @ test_pose_bone.matrix
                        current_quaternion = current_matrix.to_quaternion()
                        original_quaternion = original_rotations[bone_name]
                        
                        # Calculate angular difference
                        angle_difference = abs(current_quaternion.rotation_difference(original_quaternion).angle)
                        total_error += angle_difference
            
            if total_error < best_error:
                best_error = total_error
                best_angle = test_angle
        
        # Set the best angle
        constraint.pole_angle = best_angle
        BlenderUtils.update_dependency_graph()
        
        print(f"    Best pole angle for {effector_bone}: {math.degrees(best_angle):.2f}° "
              f"(error: {math.degrees(best_error):.2f}°)")
        return best_angle


class FKIKConverter:
    """Main class for FK/IK conversion operations."""
    
    def __init__(self):
        """Initialize the FK/IK converter."""
        self._limb_configurations = self._create_limb_configurations()
    
    @staticmethod
    def _create_limb_configurations() -> List[LimbConfiguration]:
        """Create configuration objects for all limbs."""
        return [
            # Left arm
            LimbConfiguration(
                upper_bone=BoneNames.LEFT_UPPER_ARM,
                lower_bone=BoneNames.LEFT_FOREARM,
                effector_bone=BoneNames.LEFT_HAND,
                ik_control=ControlNames.LEFT_HAND_IK,
                pole_control=ControlNames.LEFT_ARM_POLE,
                property_name=PropertyNames.ARM_LEFT,
                constraint_name=ConstraintNames.IK_LEFT_ARM,
                pole_distance_factor=Config.ARM_POLE_DISTANCE_FACTOR,
                pole_distance_min=Config.ARM_POLE_DISTANCE_MIN
            ),
            # Right arm
            LimbConfiguration(
                upper_bone=BoneNames.RIGHT_UPPER_ARM,
                lower_bone=BoneNames.RIGHT_FOREARM,
                effector_bone=BoneNames.RIGHT_HAND,
                ik_control=ControlNames.RIGHT_HAND_IK,
                pole_control=ControlNames.RIGHT_ARM_POLE,
                property_name=PropertyNames.ARM_RIGHT,
                constraint_name=ConstraintNames.IK_RIGHT_ARM,
                pole_distance_factor=Config.ARM_POLE_DISTANCE_FACTOR,
                pole_distance_min=Config.ARM_POLE_DISTANCE_MIN
            ),
            # Left leg
            LimbConfiguration(
                upper_bone=BoneNames.LEFT_UPPER_LEG,
                lower_bone=BoneNames.LEFT_LOWER_LEG,
                effector_bone=BoneNames.LEFT_FOOT,
                ik_control=ControlNames.LEFT_FOOT_IK,
                pole_control=ControlNames.LEFT_LEG_POLE,
                property_name=PropertyNames.LEG_LEFT,
                constraint_name=ConstraintNames.IK_LEFT_LEG,
                pole_distance_factor=Config.LEG_POLE_DISTANCE_FACTOR,
                pole_distance_min=Config.LEG_POLE_DISTANCE_MIN
            ),
            # Right leg
            LimbConfiguration(
                upper_bone=BoneNames.RIGHT_UPPER_LEG,
                lower_bone=BoneNames.RIGHT_LOWER_LEG,
                effector_bone=BoneNames.RIGHT_FOOT,
                ik_control=ControlNames.RIGHT_FOOT_IK,
                pole_control=ControlNames.RIGHT_LEG_POLE,
                property_name=PropertyNames.LEG_RIGHT,
                constraint_name=ConstraintNames.IK_RIGHT_LEG,
                pole_distance_factor=Config.LEG_POLE_DISTANCE_FACTOR,
                pole_distance_min=Config.LEG_POLE_DISTANCE_MIN
            )
        ]
    
    def setup_ik_system(self, armature: bpy.types.Object) -> None:
        """Create all IK controls, constraints, and drivers for the armature."""
        BlenderUtils.ensure_pose_mode(armature)
        
        # Create custom properties for IK/FK switching
        self._create_custom_properties(armature)
        
        # Create control bones
        self._create_control_bones(armature)
        
        # Create IK constraints
        self._create_ik_constraints(armature)
        
        # Create head copy rotation constraint
        self._create_head_constraint(armature)
    
    def _create_custom_properties(self, armature: bpy.types.Object) -> None:
        """Create custom properties for IK/FK switching (0=FK, 1=IK)."""
        properties = [
            PropertyNames.ARM_LEFT, PropertyNames.ARM_RIGHT,
            PropertyNames.LEG_LEFT, PropertyNames.LEG_RIGHT,
            PropertyNames.TORSO, PropertyNames.HEAD
        ]
        
        for prop_name in properties:
            BlenderUtils.add_custom_property(armature, prop_name, 0.0)
    
    def _create_control_bones(self, armature: bpy.types.Object) -> None:
        """Create all IK control bones."""
        # Create limb controls
        for config in self._limb_configurations:
            BoneManipulator.create_control_bone(armature, config.ik_control, config.effector_bone)
            BoneManipulator.create_control_bone(armature, config.pole_control, config.lower_bone)
        
        # Create torso and head controls
        BoneManipulator.create_control_bone(armature, ControlNames.TORSO_IK, BoneNames.SPINE_BONES[-1])
        BoneManipulator.create_control_bone(armature, ControlNames.HEAD_IK, BoneNames.HEAD)
    
    def _create_ik_constraints(self, armature: bpy.types.Object) -> None:
        """Create IK constraints for all limbs."""
        for config in self._limb_configurations:
            effector_bone = BlenderUtils.get_pose_bone(armature, config.effector_bone)
            if not effector_bone:
                continue
                
            # Create or get existing IK constraint
            ik_constraint = effector_bone.constraints.get(config.constraint_name)
            if not ik_constraint:
                ik_constraint = effector_bone.constraints.new('IK')
                ik_constraint.name = config.constraint_name
            
            # Configure IK constraint
            ik_constraint.target = armature
            ik_constraint.subtarget = config.ik_control
            ik_constraint.pole_target = armature
            ik_constraint.pole_subtarget = config.pole_control
            ik_constraint.chain_count = 2
            ik_constraint.use_rotation = True
            ik_constraint.influence = 0.0
            
            # Create driver linking constraint to custom property
            BlenderUtils.create_driver_for_constraint(ik_constraint, armature, config.property_name)
        
        # Create torso IK constraint
        if BoneNames.SPINE_BONES:
            top_spine_bone = BlenderUtils.get_pose_bone(armature, BoneNames.SPINE_BONES[-1])
            if top_spine_bone:
                torso_constraint = top_spine_bone.constraints.get(ConstraintNames.IK_TORSO)
                if not torso_constraint:
                    torso_constraint = top_spine_bone.constraints.new('IK')
                    torso_constraint.name = ConstraintNames.IK_TORSO
                
                torso_constraint.target = armature
                torso_constraint.subtarget = ControlNames.TORSO_IK
                torso_constraint.chain_count = min(4, len(BoneNames.SPINE_BONES))
                torso_constraint.use_rotation = True
                torso_constraint.influence = 0.0
                
                BlenderUtils.create_driver_for_constraint(torso_constraint, armature, PropertyNames.TORSO)
    
    def _create_head_constraint(self, armature: bpy.types.Object) -> None:
        """Create head copy rotation constraint."""
        head_bone = BlenderUtils.get_pose_bone(armature, BoneNames.HEAD)
        if not head_bone:
            return
            
        head_constraint = head_bone.constraints.get(ConstraintNames.COPY_ROTATION_HEAD)
        if not head_constraint:
            head_constraint = head_bone.constraints.new('COPY_ROTATION')
            head_constraint.name = ConstraintNames.COPY_ROTATION_HEAD
        
        head_constraint.target = armature
        head_constraint.subtarget = ControlNames.HEAD_IK
        head_constraint.mix_mode = 'REPLACE'
        head_constraint.target_space = 'POSE'
        head_constraint.owner_space = 'POSE'
        head_constraint.influence = 0.0
        
        BlenderUtils.create_driver_for_constraint(head_constraint, armature, PropertyNames.HEAD)
    
    def convert_fk_to_ik(self, armature: bpy.types.Object) -> None:
        """Convert from FK to IK while preserving the current pose."""
        BlenderUtils.ensure_pose_mode(armature)
        
        # Disable all IK and capture FK matrices
        self._disable_all_ik(armature)
        BlenderUtils.update_dependency_graph()
        
        print("Capturing original FK pose matrices...")
        fk_matrices = self._capture_fk_matrices(armature)
        
        # Position IK controls and poles based on FK pose
        self._position_ik_controls(armature, fk_matrices)
        
        # Calculate optimal pole angles
        self._optimize_pole_angles(armature, fk_matrices)
        
        # Enable IK constraints
        self._enable_all_ik(armature)
        
        print("FK to IK conversion completed.")
    
    def convert_ik_to_fk(self, armature: bpy.types.Object) -> None:
        """Convert from IK to FK while preserving the current pose."""
        BlenderUtils.ensure_pose_mode(armature)
        
        # Enable all IK to ensure constraints are evaluated
        self._enable_all_ik(armature)
        BlenderUtils.update_dependency_graph()
        
        # Bake evaluated rotations into FK channels
        self._bake_ik_to_fk(armature)
        
        # Disable IK (rotations remain)
        self._disable_all_ik(armature)
        
        print("IK to FK conversion completed.")
    
    def _disable_all_ik(self, armature: bpy.types.Object) -> None:
        """Disable all IK constraints by setting custom properties to 0."""
        properties = [
            PropertyNames.ARM_LEFT, PropertyNames.ARM_RIGHT,
            PropertyNames.LEG_LEFT, PropertyNames.LEG_RIGHT,
            PropertyNames.TORSO, PropertyNames.HEAD
        ]
        
        for prop_name in properties:
            armature[prop_name] = 0.0
    
    def _enable_all_ik(self, armature: bpy.types.Object) -> None:
        """Enable all IK constraints by setting custom properties to 1."""
        properties = [
            PropertyNames.ARM_LEFT, PropertyNames.ARM_RIGHT,
            PropertyNames.LEG_LEFT, PropertyNames.LEG_RIGHT,
            PropertyNames.TORSO, PropertyNames.HEAD
        ]
        
        print("Enabling IK constraints...")
        for prop_name in properties:
            armature[prop_name] = 1.0
            BlenderUtils.update_dependency_graph()
    
    def _capture_fk_matrices(self, armature: bpy.types.Object) -> Dict[str, Dict[str, Any]]:
        """Capture FK pose matrices for all relevant bones."""
        fk_matrices = {}
        
        # Collect all bones that need matrix capture
        limb_bones = []
        for config in self._limb_configurations:
            limb_bones.extend([config.upper_bone, config.lower_bone, config.effector_bone])
        
        all_bones = limb_bones + BoneNames.SPINE_BONES + [BoneNames.HEAD]
        
        for bone_name in all_bones:
            pose_bone = BlenderUtils.get_pose_bone(armature, bone_name)
            if pose_bone:
                fk_matrices[bone_name] = {
                    'world_matrix': (armature.matrix_world @ pose_bone.matrix).copy(),
                    'local_matrix': pose_bone.matrix.copy()
                }
        
        return fk_matrices
    
    def _position_ik_controls(self, armature: bpy.types.Object, 
                            fk_matrices: Dict[str, Dict[str, Any]]) -> None:
        """Position IK controls and pole targets based on captured FK matrices."""
        print("Positioning IK controls...")
        
        # Position limb controls
        for config in self._limb_configurations:
            self._position_limb_controls(armature, config, fk_matrices)
        
        # Position torso and head controls
        self._position_torso_and_head_controls(armature, fk_matrices)
    
    def _position_limb_controls(self, armature: bpy.types.Object, config: LimbConfiguration,
                              fk_matrices: Dict[str, Dict[str, Any]]) -> None:
        """Position IK control and pole for a single limb."""
        effector_pose_bone = BlenderUtils.get_pose_bone(armature, config.effector_bone)
        ik_control_bone = BlenderUtils.get_pose_bone(armature, config.ik_control)
        pole_control_bone = BlenderUtils.get_pose_bone(armature, config.pole_control)
        
        if not all([effector_pose_bone, ik_control_bone, pole_control_bone]):
            return
        
        # Position IK control using captured FK matrix
        if config.effector_bone in fk_matrices:
            target_matrix = fk_matrices[config.effector_bone]['world_matrix']
            BoneManipulator.set_pose_bone_world_matrix(armature, ik_control_bone, target_matrix)
            
            # Validate positioning accuracy
            BlenderUtils.update_dependency_graph()
            actual_matrix = armature.matrix_world @ ik_control_bone.matrix
            position_error = (actual_matrix.translation - target_matrix.translation).length
            
            if position_error > 0.001:  # More than 1mm error
                print(f"  Warning: {config.ik_control} positioning error: {position_error*1000:.1f}mm")
                # Retry with alternative method
                armature_space_pos = armature.matrix_world.inverted() @ target_matrix.translation
                ik_control_bone.location = armature_space_pos - ik_control_bone.bone.head_local
                ik_control_bone.rotation_quaternion = (armature.matrix_world.inverted() @ target_matrix).to_quaternion()
                BlenderUtils.update_dependency_graph()
                print(f"    Retried positioning for {config.ik_control}")
        
        # Position pole target
        PoleTargetPlacer.place_pole_target(
            armature, config.upper_bone, config.lower_bone, config.effector_bone,
            pole_control_bone, config.pole_distance_factor, config.pole_distance_min, fk_matrices
        )
    
    def _position_torso_and_head_controls(self, armature: bpy.types.Object,
                                        fk_matrices: Dict[str, Dict[str, Any]]) -> None:
        """Position torso and head IK controls."""
        print("Positioning torso and head controls...")
        
        # Position torso control
        if BoneNames.SPINE_BONES:
            torso_control_bone = BlenderUtils.get_pose_bone(armature, ControlNames.TORSO_IK)
            top_spine = BoneNames.SPINE_BONES[-1]
            
            if torso_control_bone and top_spine in fk_matrices:
                BoneManipulator.set_pose_bone_world_matrix(
                    armature, torso_control_bone, fk_matrices[top_spine]['world_matrix']
                )
        
        # Position head control
        head_control_bone = BlenderUtils.get_pose_bone(armature, ControlNames.HEAD_IK)
        if head_control_bone and BoneNames.HEAD in fk_matrices:
            BoneManipulator.set_pose_bone_world_matrix(
                armature, head_control_bone, fk_matrices[BoneNames.HEAD]['world_matrix']
            )
    
    def _optimize_pole_angles(self, armature: bpy.types.Object,
                            fk_matrices: Dict[str, Dict[str, Any]]) -> None:
        """Calculate and set optimal pole angles for all IK constraints."""
        print("Computing optimal pole angles from original FK pose...")
        
        for config in self._limb_configurations:
            PoleAngleOptimizer.find_optimal_pole_angle(
                armature, config.upper_bone, config.lower_bone, config.effector_bone,
                config.constraint_name, fk_matrices
            )
        
        # Final update after setting all pole angles
        BlenderUtils.update_dependency_graph()
    
    def _bake_ik_to_fk(self, armature: bpy.types.Object) -> None:
        """Bake IK-evaluated rotations into FK channels."""
        # Bake limb rotations
        for config in self._limb_configurations:
            bones_to_bake = [config.upper_bone, config.lower_bone, config.effector_bone]
            for bone_name in bones_to_bake:
                pose_bone = BlenderUtils.get_pose_bone(armature, bone_name)
                if pose_bone:
                    BoneManipulator.bake_evaluated_rotation(pose_bone)
        
        # Bake spine bones
        for spine_bone_name in BoneNames.SPINE_BONES:
            pose_bone = BlenderUtils.get_pose_bone(armature, spine_bone_name)
            if pose_bone:
                BoneManipulator.bake_evaluated_rotation(pose_bone)
        
        # Bake head bone
        head_bone = BlenderUtils.get_pose_bone(armature, BoneNames.HEAD)
        if head_bone:
            BoneManipulator.bake_evaluated_rotation(head_bone)


# Blender operator classes
class FKIK_OT_BuildControls(bpy.types.Operator):
    """Build IK controls, constraints, and drivers."""
    bl_idname = "fkik.build_controls"
    bl_label = "Build Controls"
    bl_description = "Create IK targets, poles, constraints and drivers"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        try:
            armature = BlenderUtils.get_active_armature()
            converter = FKIKConverter()
            converter.setup_ik_system(armature)
            self.report({'INFO'}, "FK/IK controls built successfully.")
        except Exception as e:
            self.report({'ERROR'}, f"Error building controls: {str(e)}")
            return {'CANCELLED'}
        
        return {'FINISHED'}


class FKIK_OT_SnapFKToIK(bpy.types.Operator):
    """Snap FK pose to IK while preserving the current pose."""
    bl_idname = "fkik.snap_fk_to_ik"
    bl_label = "Snap FK → IK"
    bl_description = "Convert from FK to IK while keeping the current pose"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        try:
            armature = BlenderUtils.get_active_armature()
            converter = FKIKConverter()
            converter.convert_fk_to_ik(armature)
            self.report({'INFO'}, "Successfully snapped FK to IK (pose preserved).")
        except Exception as e:
            self.report({'ERROR'}, f"Error during FK to IK conversion: {str(e)}")
            return {'CANCELLED'}
        
        return {'FINISHED'}


class FKIK_OT_SnapIKToFK(bpy.types.Operator):
    """Snap IK pose to FK while preserving the current pose."""
    bl_idname = "fkik.snap_ik_to_fk"
    bl_label = "Snap IK → FK"
    bl_description = "Convert from IK to FK while keeping the current pose"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        try:
            armature = BlenderUtils.get_active_armature()
            converter = FKIKConverter()
            converter.convert_ik_to_fk(armature)
            self.report({'INFO'}, "Successfully snapped IK to FK (pose preserved).")
        except Exception as e:
            self.report({'ERROR'}, f"Error during IK to FK conversion: {str(e)}")
            return {'CANCELLED'}
        
        return {'FINISHED'}


class FKIK_PT_Panel(bpy.types.Panel):
    """Main UI panel for FK/IK controls."""
    bl_label = "FK/IK Converter"
    bl_idname = "FKIK_PT_panel"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "FK/IK"

    def draw(self, context):
        layout = self.layout
        obj = context.object
        
        # Build controls button
        layout.operator(FKIK_OT_BuildControls.bl_idname, icon='BONE_DATA')

        if not (obj and obj.type == 'ARMATURE'):
            return

        # Check if properties exist
        required_properties = [
            PropertyNames.ARM_LEFT, PropertyNames.ARM_RIGHT,
            PropertyNames.LEG_LEFT, PropertyNames.LEG_RIGHT,
            PropertyNames.TORSO, PropertyNames.HEAD
        ]
        
        missing_properties = [prop for prop in required_properties if prop not in obj]
        
        if missing_properties:
            box = layout.box()
            box.label(text="Please build FK/IK controls first", icon='ERROR')
            return

        # IK/FK sliders
        col = layout.column(align=True)
        col.prop(obj, f'["{PropertyNames.ARM_LEFT}"]', text="IK Left Arm", slider=True)
        col.prop(obj, f'["{PropertyNames.ARM_RIGHT}"]', text="IK Right Arm", slider=True)
        col.prop(obj, f'["{PropertyNames.LEG_LEFT}"]', text="IK Left Leg", slider=True)
        col.prop(obj, f'["{PropertyNames.LEG_RIGHT}"]', text="IK Right Leg", slider=True)
        col.prop(obj, f'["{PropertyNames.TORSO}"]', text="IK Torso", slider=True)
        col.prop(obj, f'["{PropertyNames.HEAD}"]', text="IK Head", slider=True)
        
        # Conversion buttons
        layout.separator()
        row = layout.row(align=True)
        row.operator(FKIK_OT_SnapFKToIK.bl_idname, icon='ARMATURE_DATA')
        row.operator(FKIK_OT_SnapIKToFK.bl_idname, icon='ARMATURE_DATA')


# Registration
classes = (
    FKIK_OT_BuildControls,
    FKIK_OT_SnapFKToIK,
    FKIK_OT_SnapIKToFK,
    FKIK_PT_Panel,
)


def register():
    """Register all Blender classes."""
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    """Unregister all Blender classes."""
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
