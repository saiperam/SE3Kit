# se3kit — API Reference

Compact reference for the public classes, functions, and files in this repository.

Overview
--------
se3kit provides small, well-documented building blocks for SE(3) transforms, rotations, translations, homogeneous points, and simple serial-robot forward kinematics.

Homogeneous transform convention:
$$
T = \begin{bmatrix} R & t \\[4pt] 0 & 1 \end{bmatrix}
$$
with $R\in SO(3)$ and $t\in\mathbb{R}^3$.

Core classes and symbols
------------------------

- [`transformation.Transformation`](src/transformation.py) — [src/transformation.py](src/transformation.py)  
  - Constructors:
    - `Transformation(np.ndarray(4x4))`
    - `Transformation(Pose)` (ROS Pose via [`ros_compat.get_ros_geometry_msgs`](src/ros_compat.py))
    - `Transformation(Translation)` (translation only)
    - `Transformation(Translation, Rotation)`
  - Key properties / methods:
    - `m` — full 4×4 matrix
    - `rotation`, `rotation = ...` — uses [`rotation.Rotation`](src/rotation.py)
    - `translation`, `translation = ...` — uses [`translation.Translation`](src/translation.py)
    - `inv` — inverse transform
    - `transform_hpoint(HPoint)` — uses [`hpoint.HPoint`](src/hpoint.py)
    - `as_geometry_pose()` — ROS Pose conversion
    - `convert_m_to_mm()`, `convert_mm_to_m()`, `scaled()` helpers

- [`rotation.Rotation`](src/rotation.py) — [src/rotation.py](src/rotation.py)  
  - Stores 3×3 rotation matrix (`.m`).
  - Constructors accept `None` (identity), `np.ndarray` (3×3), `np.quaternion`, ROS `Quaternion`, or another `Rotation`.
  - Factories / conversions:
    - `Rotation.from_zyx(euler, degrees=False)` / `from_ABC`
    - `Rotation.from_rpy(rpy, degrees=False)`
    - `as_zyx(degrees=False)`, `as_rpy(degrees=False)`
    - `as_quat()` → `np.quaternion`
    - `as_geometry_orientation()` → ROS `Quaternion`
    - `as_axisangle()` → axis, angle
  - Axis accessors: `x_axis`, `y_axis`, `z_axis`.

- [`translation.Translation`](src/translation.py) — [src/translation.py](src/translation.py)  
  - 3-vector wrapper (`.m`), initializable from list/ndarray, [`hpoint.HPoint`](src/hpoint.py), ROS `Point`/`Vector3`, or another `Translation`.
  - Arithmetic: `+`, `-`, `*`, `/`.
  - Unit helpers: `convert_m_to_mm()`, `convert_mm_to_m()`, `scaled_m_to_mm()`, `scaled_mm_to_m()`.
  - `as_geometry_point()` → ROS `Point`.

- [`hpoint.HPoint`](src/hpoint.py) — [src/hpoint.py](src/hpoint.py)  
  - Homogeneous 4×1 point container (`.m`).
  - Constructors: `HPoint(x,y,z)`, `HPoint(np.array([x,y,z]))`, or `HPoint(np.array([x,y,z,w]))`.
  - Accessors: `x`, `y`, `z`, `xyz`, `as_array()`.

- [`robot.Robot`](src/robot.py) — [src/robot.py](src/robot.py)  
  - Robot models (factory methods):
    - `Robot.create_iiwa()` — KUKA iiwa R14 (mm)
    - `Robot.create_franka_fp3()` — Franka Panda (m, approx.)
  - Kinematics:
    - `FK_space(joint_angles)` — space-frame FK using screw exponentials (`scipy.linalg.expm` and [`utils.vector_to_skew`](src/utils.py)).
  - Data: `.home`, `.axes`, `.offset`, `.screw`, `.dof`.

Utilities
---------
- [`utils.deg2rad`](src/utils.py), [`utils.rad2deg`](src/utils.py) — angle conversion.  
- [`utils.is_near`](src/utils.py), [`utils.is_identity`](src/utils.py) — numeric predicates.  
- [`utils.vector_to_skew`](src/utils.py), [`utils.skew_to_vector`](src/utils.py) — skew matrix / vector conversions.  
  Files: [src/utils.py](src/utils.py)

Angles helper
-------------
- [`degrees.Degrees`](src/degrees.py) — [src/degrees.py](src/degrees.py)  
  - Simple container with `.deg` and `.rad` properties.

ROS compatibility
-----------------
- [`ros_compat.get_ros_geometry_msgs`](src/ros_compat.py), [`ros_compat.ROS_VERSION`](src/ros_compat.py) — [src/ros_compat.py](src/ros_compat.py)  
  - Detects ROS2 (rclpy) vs ROS1 (rospy) vs none; exposes `Pose, Point, Quaternion, Vector3` or `None`.

Tests and examples
------------------
- Unit tests: [src/tests.py](src/tests.py) — `unittest`-based checks for FK and ROS detection.
- Quick usage examples are in [README.md](README.md).

Repository files
----------------
- [src/__init__.py](src/__init__.py) — package exports
- [src/transformation.py](src/transformation.py)
- [src/rotation.py](src/rotation.py)
- [src/translation.py](src/translation.py)
- [src/hpoint.py](src/hpoint.py)
- [src/robot.py](src/robot.py)
- [src/utils.py](src/utils.py)
- [src/degrees.py](src/degrees.py)
- [src/ros_compat.py](src/ros_compat.py)
- [src/tests.py](src/tests.py)
- [README.md](README.md)
- [LICENSE](LICENSE)

How to use this reference
-------------------------
- Open the implementation for full docstrings and examples: follow the links above to each source file.
- For FK debugging, inspect [`robot.Robot.FK_space`](src/robot.py) and the screw generation in each factory.
- For ROS conversions, use the `as_geometry_*` helpers and check `ros_compat.ROS_VERSION` to confirm availability.

Notes
-----
- All class/method links above point to the implementing module file.
- Numeric tolerances and small helper behaviors are defined in [src/utils.py](src/utils.py).

