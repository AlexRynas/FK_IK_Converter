# FK/IK Converter for Blender

A comprehensive tool for creating and managing Forward Kinematics (FK) and Inverse Kinematics (IK) controls for character armatures in Blender. This addon provides seamless switching between FK and IK animation modes while preserving poses during conversion.

## Features

### 🦴 Automatic IK Control Creation
- Creates IK targets and pole targets for arms, legs, torso, and head
- Supports standard humanoid character rigs with common bone naming conventions
- Generates clean, organized control bones for easy animation

### 🔄 Seamless FK/IK Conversion
- **FK → IK**: Convert from FK to IK while preserving the exact current pose
- **IK → FK**: Convert from IK to FK while maintaining pose accuracy
- Optimized pole angle calculation for natural limb positioning
- Sub-millimeter precision in pose preservation

### 🎛️ Interactive UI Panel
- Clean UI panel in the 3D Viewport sidebar
- Individual IK/FK sliders for each limb system
- One-click conversion buttons
- Real-time switching between animation modes

### ⚙️ Advanced Features
- Automatic constraint setup with proper drivers
- Custom property management for IK/FK blending
- Dependency graph optimization for smooth performance
- Support for multiple rotation modes (Quaternion, Euler, Axis-Angle)

## Installation

1. Download the `FK_IK_Converter.py` file
2. In Blender, go to **Edit → Preferences → Add-ons**
3. Click **Install...** and select the downloaded file
4. Enable the "FK/IK Converter" addon in the list
5. The panel will appear in the 3D Viewport sidebar under the "FK/IK" tab

## Usage

### Initial Setup

1. **Select your armature** in the 3D Viewport
2. Switch to **Pose Mode**
3. In the FK/IK panel, click **"Build Controls"**
   - This creates all necessary IK targets, pole targets, and constraints
   - Sets up custom properties for IK/FK blending
   - Creates drivers linking constraints to the control properties

### Converting Between FK and IK

#### FK to IK Conversion
1. Pose your character using FK (normal bone rotation)
2. Click **"Snap FK → IK"**
3. The system will:
   - Capture the current FK pose matrices
   - Position IK targets to match the current pose
   - Calculate optimal pole angles
   - Enable IK constraints while preserving the pose

#### IK to FK Conversion
1. With IK constraints active, pose your character
2. Click **"Snap IK → FK"**
3. The system will:
   - Bake the IK-evaluated rotations into FK channels
   - Disable IK constraints
   - Preserve the exact pose in FK mode

### Manual IK/FK Blending

Use the sliders in the panel to blend between FK and IK modes:
- **IK Left/Right Arm**: Controls arm IK influence (0 = FK, 1 = IK)
- **IK Left/Right Leg**: Controls leg IK influence
- **IK Torso**: Controls spine IK influence
- **IK Head**: Controls head IK influence

## Supported Bone Naming Conventions

The addon works with standard humanoid rigs using these bone names:

### Arms
- `LeftArm`, `LeftForeArm`, `LeftHand`
- `RightArm`, `RightForeArm`, `RightHand`
- `LeftShoulder`, `RightShoulder` (optional)

### Legs  
- `LeftUpLeg`, `LeftLeg`, `LeftFoot`
- `RightUpLeg`, `RightLeg`, `RightFoot`
- `LeftToeBase`, `RightToeBase` (optional)

### Torso & Head
- `Hips`
- `Spine`, `Spine1`, `Spine2`, `Spine3`
- `Neck`, `Neck1`
- `Head`

## Generated Control Bones

The addon creates the following control bones:

### IK Targets
- `CTRL_L_Hand_IK`, `CTRL_R_Hand_IK`
- `CTRL_L_Foot_IK`, `CTRL_R_Foot_IK`
- `CTRL_Torso_IK`
- `CTRL_Head_IK`

### Pole Targets
- `CTRL_L_Arm_Pole`, `CTRL_R_Arm_Pole`
- `CTRL_L_Leg_Pole`, `CTRL_R_Leg_Pole`

## Technical Details

### Architecture
- **Modular Design**: Separate classes handle different aspects (constraints, bone manipulation, pole positioning)
- **Type Safety**: Full type hints for better code reliability
- **Error Handling**: Comprehensive error checking and user feedback
- **Performance Optimized**: Efficient dependency graph updates

### Key Components
- **`FKIKConverter`**: Main conversion logic
- **`BoneManipulator`**: Bone creation and transformation utilities
- **`PoleTargetPlacer`**: Intelligent pole target positioning
- **`PoleAngleOptimizer`**: Advanced pole angle calculation
- **`BlenderUtils`**: Blender API utilities and helpers

### Pole Angle Optimization
The system uses advanced algorithms to calculate optimal pole angles:
- Tests multiple angle candidates for each limb
- Compares IK results against original FK pose
- Selects the angle that minimizes rotation differences
- Handles left/right side differences automatically

## Diagnostics

The included `diagnostic.py` provides advanced diagnostic tools:
- Detailed bone position and rotation analysis
- Before/after comparison with precision metrics
- Interactive diagnostic functions for troubleshooting

## Compatibility

- **Blender Version**: 2.8+ (tested up to 4.0+)
- **Character Types**: Humanoid rigs with standard bone naming
- **Animation Systems**: Compatible with existing FK animation workflows

## Troubleshooting

### Common Issues

**"Please build FK/IK controls first" message**
- Click the "Build Controls" button to set up the system

**Controls not working correctly**
- Ensure your armature uses standard bone naming conventions
- Check that you're in Pose Mode when using the tools

**Pose changes during conversion**
- This indicates a bug - please report with your rig for investigation
- Try using the diagnostic tools to identify the issue

**Performance issues**
- Large dependency graphs may cause delays during conversion
- Consider simplifying complex constraint setups

### Getting Help

1. Check bone naming conventions match the expected format
2. Use the diagnostic tools to analyze conversion accuracy
3. Report issues with specific rig examples for investigation

## Contributing

This project welcomes contributions! Areas for improvement:
- Support for additional bone naming conventions
- Enhanced UI features
- Performance optimizations
- Additional diagnostic tools

## License

This addon is provided as-is for educational and production use. Please credit the original author when redistributing or modifying.

## Changelog

### Current Version
- Complete rewrite with improved architecture
- Enhanced pole angle calculation algorithms
- Better error handling and user feedback
- Comprehensive diagnostic tools
- Support for multiple rotation modes
- Optimized performance for large rigs

---

**Happy Animating!** 🎬
