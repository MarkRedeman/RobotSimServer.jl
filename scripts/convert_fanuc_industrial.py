#!/usr/bin/env python3
"""
Convert ROS-Industrial Fanuc URDF/xacro robot descriptions to MuJoCo XML format.

This script parses the xacro macro files from the fanuc-industrial repository
and generates MuJoCo-compatible XML with proper mesh paths, colors, and actuators.

Usage:
    # Generate all default variants (one per family)
    python scripts/convert_fanuc_industrial.py

    # Generate specific variant(s)
    python scripts/convert_fanuc_industrial.py m10ia7l crx10ial

    # List available variants
    python scripts/convert_fanuc_industrial.py --list
"""

import argparse
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

# Fanuc colors from common_colours.xacro
# These are the RGBA values used for visual geoms
FANUC_COLORS = {
    "material_fanuc_yellow": (0.96, 0.76, 0.13, 1.0),
    "material_fanuc_white": (0.86, 0.85, 0.81, 1.0),
    "material_fanuc_grey": (0.34, 0.35, 0.36, 1.0),
    "material_fanuc_gray24": (0.239, 0.239, 0.239, 1.0),
    "material_fanuc_gray28": (0.278, 0.278, 0.278, 1.0),
    "material_fanuc_gray31": (0.310, 0.310, 0.310, 1.0),
    "material_fanuc_gray40": (0.400, 0.400, 0.400, 1.0),
    "material_fanuc_yellow1": (1.0, 1.0, 0.0, 1.0),
    "material_fanuc_BAB0B0": (0.729, 0.690, 0.690, 1.0),
    "material_fanuc_E6E6E6": (0.902, 0.902, 0.902, 1.0),
    "material_fanuc_00FF73": (0.0, 1.0, 0.451, 1.0),
    "material_fanuc_black": (0.15, 0.15, 0.15, 1.0),
    "material_fanuc_greyish": (0.75, 0.75, 0.75, 1.0),
    "material_fanuc_green": (0.0, 0.608, 0.275, 1.0),
}

# Default color for links without material specification
DEFAULT_COLOR = (0.7, 0.7, 0.7, 1.0)

# Fixed mass and inertia for all links (keeping it simple)
LINK_MASS = 1.0
LINK_INERTIA = "0.01 0.01 0.01"

# Default variants for each family (one per family for generation)
DEFAULT_VARIANTS = {
    "cr35ia": "cr35ia",
    "cr7ia": "cr7ia",
    "crx10ia": "crx10ial",
    "lrmate200i": "lrmate200i",
    "lrmate200ib": "lrmate200ib",
    "lrmate200ic": "lrmate200ic",
    "lrmate200id": "lrmate200id",
    "m10ia": "m10ia",
    "m16ib": "m16ib20",
    "m20ia": "m20ia",
    "m20ib": "m20ib25",
    "m430ia": "m430ia2f",
    "m6ib": "m6ib",
    "m710ic": "m710ic50",
    "m900ia": "m900ia260l",
    "m900ib": "m900ib700",
    "r1000ia": "r1000ia80f",
    "r2000ib": "r2000ib210f",
    "r2000ic": "r2000ic165f",
}


@dataclass
class LinkInfo:
    name: str
    visual_mesh: str
    collision_mesh: str
    material: str  # Material macro name


@dataclass
class JointInfo:
    name: str
    type: str
    parent: str
    child: str
    origin_xyz: str
    origin_rpy: str
    axis: str
    lower_limit: float
    upper_limit: float


@dataclass
class RobotConfig:
    family: str  # e.g., "m10ia"
    variant: str  # e.g., "m10ia" or "m10ia7l"
    package_dir: Path  # e.g., fanuc_m10ia_support
    xacro_file: Path


def find_all_robots(fanuc_dir: Path) -> dict[str, RobotConfig]:
    """Discover all robot configurations in the fanuc-industrial directory."""
    robots = {}

    for support_dir in sorted(fanuc_dir.glob("fanuc_*_support")):
        # Extract family name from directory
        family = support_dir.name.replace("fanuc_", "").replace("_support", "")

        urdf_dir = support_dir / "urdf"
        if not urdf_dir.exists():
            continue

        for xacro_file in sorted(urdf_dir.glob("*_macro.xacro")):
            # Extract variant name from filename
            variant = xacro_file.stem.replace("_macro", "")

            # Verify meshes exist
            mesh_dir = support_dir / "meshes" / variant
            if not mesh_dir.exists():
                continue

            robots[variant] = RobotConfig(
                family=family,
                variant=variant,
                package_dir=support_dir,
                xacro_file=xacro_file,
            )

    return robots


def parse_xacro(xacro_path: Path) -> tuple[list[LinkInfo], list[JointInfo]]:
    """Parse a xacro macro file and extract link/joint information."""
    tree = ET.parse(xacro_path)
    root = tree.getroot()

    # Find all macro elements and pick the one with the most content
    # Some files have a wrapper macro that calls a base macro
    ns = {"xacro": "http://wiki.ros.org/xacro"}
    macros = root.findall(".//{http://wiki.ros.org/xacro}macro")

    if not macros:
        # Try without namespace
        macros = [elem for elem in root.iter() if elem.tag.endswith("macro")]

    if not macros:
        raise ValueError(f"No xacro:macro found in {xacro_path}")

    # Pick the macro with the most link elements (the actual definition, not wrapper)
    macro = max(macros, key=lambda m: len(m.findall(".//link")))

    links = []
    joints = []

    # Parse links
    for link in macro.findall(".//link"):
        name = link.get("name", "")
        # Clean up ${prefix}
        name = name.replace("${prefix}", "")

        # Skip helper links (base, flange, tool0)
        if name in ("base", "flange", "tool0", ""):
            continue

        visual = link.find("visual")
        collision = link.find("collision")

        visual_mesh = ""
        collision_mesh = ""
        material = ""

        if visual is not None:
            mesh = visual.find("geometry/mesh")
            if mesh is not None:
                visual_mesh = mesh.get("filename", "")

            # Find material macro
            for child in visual:
                if child.tag.startswith("{http://wiki.ros.org/xacro}"):
                    material = child.tag.split("}")[-1]
                elif child.tag.startswith("xacro:"):
                    material = child.tag.replace("xacro:", "")

        if collision is not None:
            mesh = collision.find("geometry/mesh")
            if mesh is not None:
                collision_mesh = mesh.get("filename", "")

        links.append(
            LinkInfo(
                name=name,
                visual_mesh=visual_mesh,
                collision_mesh=collision_mesh,
                material=material,
            )
        )

    # Parse joints
    for joint in macro.findall(".//joint"):
        name = joint.get("name", "")
        jtype = joint.get("type", "fixed")

        # Clean up ${prefix}
        name = name.replace("${prefix}", "")

        # Skip fixed joints
        if jtype == "fixed":
            continue

        parent = joint.find("parent")
        child = joint.find("child")
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")

        parent_link = (
            parent.get("link", "").replace("${prefix}", "")
            if parent is not None
            else ""
        )
        child_link = (
            child.get("link", "").replace("${prefix}", "") if child is not None else ""
        )

        origin_xyz = "0 0 0"
        origin_rpy = "0 0 0"
        if origin is not None:
            origin_xyz = origin.get("xyz", "0 0 0")
            origin_rpy = origin.get("rpy", "0 0 0")

        axis_xyz = "0 0 1"
        if axis is not None:
            axis_xyz = axis.get("xyz", "0 0 1")

        lower = upper = 0.0
        if limit is not None:
            lower_str = limit.get("lower", "0")
            upper_str = limit.get("upper", "0")

            # Evaluate radians() expressions
            lower = evaluate_limit(lower_str)
            upper = evaluate_limit(upper_str)

        joints.append(
            JointInfo(
                name=name,
                type=jtype,
                parent=parent_link,
                child=child_link,
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
                axis=axis_xyz,
                lower_limit=lower,
                upper_limit=upper,
            )
        )

    return links, joints


def evaluate_limit(expr: str) -> float:
    """Evaluate joint limit expression, handling radians() function."""
    expr = expr.strip()

    # Handle ${radians(...)} or radians(...)
    radians_match = re.search(r"radians\s*\(\s*(-?[\d.]+)\s*\)", expr)
    if radians_match:
        return math.radians(float(radians_match.group(1)))

    # Handle ${pi}, ${-pi}, etc.
    if "pi" in expr.lower():
        expr = re.sub(r"\$\{", "", expr)
        expr = re.sub(r"\}", "", expr)
        expr = re.sub(r"pi", str(math.pi), expr, flags=re.IGNORECASE)
        try:
            return float(eval(expr))
        except:
            pass

    try:
        return float(expr)
    except ValueError:
        return 0.0


def rpy_to_quat(rpy_str: str) -> str:
    """Convert roll-pitch-yaw to quaternion string for MuJoCo."""
    try:
        parts = rpy_str.split()
        if len(parts) != 3:
            return "1 0 0 0"

        roll, pitch, yaw = [float(x) for x in parts]

        # Convert RPY to quaternion (ZYX convention)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        # MuJoCo uses w, x, y, z order
        return f"{w:.6f} {x:.6f} {y:.6f} {z:.6f}"
    except:
        return "1 0 0 0"


def get_color_rgba(material: str) -> str:
    """Get RGBA string for a material name."""
    color = FANUC_COLORS.get(material, DEFAULT_COLOR)
    return f"{color[0]:.3f} {color[1]:.3f} {color[2]:.3f} {color[3]:.3f}"


def generate_mujoco_xml(
    robot: RobotConfig,
    links: list[LinkInfo],
    joints: list[JointInfo],
    output_dir: Path,
    fanuc_dir: Path,
) -> None:
    """Generate MuJoCo XML files for a robot."""
    robot_dir = output_dir / robot.variant
    robot_dir.mkdir(parents=True, exist_ok=True)

    # Calculate relative path to mesh directories
    visual_mesh_dir = robot.package_dir / "meshes" / robot.variant / "visual"
    collision_mesh_dir = robot.package_dir / "meshes" / robot.variant / "collision"

    # Relative path from output dir to mesh dir
    rel_visual_path = Path("..") / ".." / visual_mesh_dir.relative_to(fanuc_dir.parent)
    rel_collision_path = (
        Path("..") / ".." / collision_mesh_dir.relative_to(fanuc_dir.parent)
    )

    # Build link and joint maps
    link_map = {l.name: l for l in links}
    joint_map = {}  # child -> joint
    children_map = {}  # parent -> [children]

    for j in joints:
        joint_map[j.child] = j
        if j.parent not in children_map:
            children_map[j.parent] = []
        children_map[j.parent].append(j.child)

    # Find root link (not a child of any joint)
    child_links = {j.child for j in joints}
    root_link = None
    for link in links:
        if link.name not in child_links:
            root_link = link.name
            break

    if root_link is None:
        root_link = "base_link"

    # Start building XML
    xml_lines = [
        '<?xml version="1.0" ?>',
        f"<!-- MuJoCo model for FANUC {robot.variant.upper()} -->",
        f"<!-- Auto-generated from fanuc-industrial/{robot.package_dir.name} -->",
        f'<mujoco model="{robot.variant}">',
        '  <compiler angle="radian" autolimits="true"/>',
        "",
        "  <default>",
        f'    <default class="{robot.variant}">',
        '      <joint damping="100" frictionloss="10" armature="1"/>',
        '      <position kp="1000" kv="100"/>',
        '      <default class="visual">',
        '        <geom type="mesh" contype="0" conaffinity="0" group="2"/>',
        "      </default>",
        '      <default class="collision">',
        '        <geom type="mesh" group="3"/>',
        "      </default>",
        "    </default>",
        "  </default>",
        "",
        "  <asset>",
    ]

    # Add mesh assets
    for link in links:
        if link.visual_mesh:
            mesh_file = link.visual_mesh.split("/")[-1]
            xml_lines.append(
                f'    <mesh name="{link.name}_visual" '
                f'file="{rel_visual_path}/{mesh_file}"/>'
            )
        if link.collision_mesh:
            mesh_file = link.collision_mesh.split("/")[-1]
            xml_lines.append(
                f'    <mesh name="{link.name}_collision" '
                f'file="{rel_collision_path}/{mesh_file}"/>'
            )

    xml_lines.extend(
        [
            "  </asset>",
            "",
            "  <worldbody>",
        ]
    )

    def write_body(link_name: str, indent: int) -> list[str]:
        """Recursively write body elements."""
        if link_name not in link_map:
            return []

        link = link_map[link_name]
        pad = "  " * indent
        lines = []

        lines.append(f'{pad}<body name="{link_name}" childclass="{robot.variant}">')

        # Add inertial
        lines.append(
            f'{pad}  <inertial pos="0 0 0" mass="{LINK_MASS}" '
            f'diaginertia="{LINK_INERTIA}"/>'
        )

        # Add visual geom with color
        if link.visual_mesh:
            rgba = get_color_rgba(link.material)
            lines.append(
                f'{pad}  <geom class="visual" mesh="{link_name}_visual" rgba="{rgba}"/>'
            )

        # Add collision geom
        if link.collision_mesh:
            lines.append(
                f'{pad}  <geom class="collision" mesh="{link_name}_collision"/>'
            )

        # Add child bodies
        if link_name in children_map:
            for child_name in children_map[link_name]:
                if child_name not in link_map:
                    continue

                joint = joint_map[child_name]
                quat = rpy_to_quat(joint.origin_rpy)

                # Check if quaternion is identity (no rotation needed)
                if quat == "1.000000 0.000000 0.000000 0.000000":
                    quat_attr = ""
                else:
                    quat_attr = f' quat="{quat}"'

                lines.append(
                    f'{pad}  <body name="{child_name}" pos="{joint.origin_xyz}"'
                    f'{quat_attr} childclass="{robot.variant}">'
                )

                # Add joint
                lines.append(
                    f'{pad}    <joint name="{joint.name}" type="hinge" '
                    f'axis="{joint.axis}" range="{joint.lower_limit:.4f} {joint.upper_limit:.4f}"/>'
                )

                # Add inertial
                child_link = link_map[child_name]
                lines.append(
                    f'{pad}    <inertial pos="0 0 0" mass="{LINK_MASS}" '
                    f'diaginertia="{LINK_INERTIA}"/>'
                )

                # Add visual geom with color
                if child_link.visual_mesh:
                    rgba = get_color_rgba(child_link.material)
                    lines.append(
                        f'{pad}    <geom class="visual" mesh="{child_name}_visual" rgba="{rgba}"/>'
                    )

                # Add collision geom
                if child_link.collision_mesh:
                    lines.append(
                        f'{pad}    <geom class="collision" mesh="{child_name}_collision"/>'
                    )

                # Recursively add grandchildren
                if child_name in children_map:
                    for grandchild in children_map[child_name]:
                        lines.extend(write_nested_body(grandchild, indent + 2))

                lines.append(f"{pad}  </body>")

        lines.append(f"{pad}</body>")
        return lines

    def write_nested_body(link_name: str, indent: int) -> list[str]:
        """Write a nested body with its joint."""
        if link_name not in link_map or link_name not in joint_map:
            return []

        link = link_map[link_name]
        joint = joint_map[link_name]
        pad = "  " * indent

        quat = rpy_to_quat(joint.origin_rpy)
        if quat == "1.000000 0.000000 0.000000 0.000000":
            quat_attr = ""
        else:
            quat_attr = f' quat="{quat}"'

        lines = [
            f'{pad}<body name="{link_name}" pos="{joint.origin_xyz}"'
            f'{quat_attr} childclass="{robot.variant}">'
        ]

        # Add joint
        lines.append(
            f'{pad}  <joint name="{joint.name}" type="hinge" '
            f'axis="{joint.axis}" range="{joint.lower_limit:.4f} {joint.upper_limit:.4f}"/>'
        )

        # Add inertial
        lines.append(
            f'{pad}  <inertial pos="0 0 0" mass="{LINK_MASS}" '
            f'diaginertia="{LINK_INERTIA}"/>'
        )

        # Add visual geom with color
        if link.visual_mesh:
            rgba = get_color_rgba(link.material)
            lines.append(
                f'{pad}  <geom class="visual" mesh="{link_name}_visual" rgba="{rgba}"/>'
            )

        # Add collision geom
        if link.collision_mesh:
            lines.append(
                f'{pad}  <geom class="collision" mesh="{link_name}_collision"/>'
            )

        # Recursively add children
        if link_name in children_map:
            for child in children_map[link_name]:
                lines.extend(write_nested_body(child, indent + 1))

        lines.append(f"{pad}</body>")
        return lines

    # Write the root body
    xml_lines.extend(write_body(root_link, 2))

    xml_lines.extend(
        [
            "  </worldbody>",
            "",
            "  <actuator>",
        ]
    )

    # Add position actuators for each joint
    for j in sorted(joints, key=lambda x: x.name):
        xml_lines.append(
            f'    <position name="{j.name}_actuator" joint="{j.name}" '
            f'class="{robot.variant}"/>'
        )

    xml_lines.extend(
        [
            "  </actuator>",
            "</mujoco>",
        ]
    )

    # Write robot XML
    robot_xml_path = robot_dir / f"{robot.variant}.xml"
    with open(robot_xml_path, "w") as f:
        f.write("\n".join(xml_lines))
    print(f"  Created {robot_xml_path}")

    # Create scene XML
    scene_xml = f'''<?xml version="1.0" ?>
<!-- Scene file for FANUC {robot.variant.upper()} -->
<mujoco model="scene">
  <include file="{robot.variant}.xml"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="120" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
             rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8"
             width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true"
              texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <worldbody>
    <light pos="0 0 3.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
'''

    scene_xml_path = robot_dir / "scene.xml"
    with open(scene_xml_path, "w") as f:
        f.write(scene_xml)
    print(f"  Created {scene_xml_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Convert ROS-Industrial Fanuc robots to MuJoCo XML"
    )
    parser.add_argument(
        "variants",
        nargs="*",
        help="Specific variant(s) to convert. If empty, converts all default variants.",
    )
    parser.add_argument(
        "--list", "-l", action="store_true", help="List all available variants"
    )
    args = parser.parse_args()

    # Find project root
    script_dir = Path(__file__).parent.resolve()
    project_root = script_dir.parent
    fanuc_dir = project_root / "robots" / "fanuc-industrial"

    if not fanuc_dir.exists():
        print(f"Error: fanuc-industrial directory not found at {fanuc_dir}")
        return 1

    # Discover all robots
    all_robots = find_all_robots(fanuc_dir)

    if not all_robots:
        print("No robots found!")
        return 1

    if args.list:
        print(f"Available variants ({len(all_robots)}):\n")
        by_family = {}
        for variant, config in sorted(all_robots.items()):
            if config.family not in by_family:
                by_family[config.family] = []
            by_family[config.family].append(variant)

        for family in sorted(by_family.keys()):
            default = DEFAULT_VARIANTS.get(family, "")
            variants = by_family[family]
            variant_str = ", ".join(
                f"{v}*" if v == default else v for v in sorted(variants)
            )
            print(f"  {family}: {variant_str}")

        print("\n  * = default variant (generated with no arguments)")
        return 0

    # Determine which variants to convert
    if args.variants:
        variants_to_convert = []
        for v in args.variants:
            if v in all_robots:
                variants_to_convert.append(v)
            else:
                print(f"Warning: Unknown variant '{v}', skipping")
    else:
        # Convert all default variants
        variants_to_convert = list(DEFAULT_VARIANTS.values())

    output_dir = project_root / "robots" / "fanuc_mujoco"
    output_dir.mkdir(exist_ok=True)

    print(f"Converting {len(variants_to_convert)} variant(s) to {output_dir}...\n")

    success_count = 0
    for variant in sorted(variants_to_convert):
        if variant not in all_robots:
            print(f"Skipping {variant}: not found")
            continue

        robot = all_robots[variant]
        print(f"Processing {variant} ({robot.family})...")

        try:
            links, joints = parse_xacro(robot.xacro_file)
            print(f"  Parsed {len(links)} links, {len(joints)} joints")

            generate_mujoco_xml(robot, links, joints, output_dir, fanuc_dir)
            success_count += 1
        except Exception as e:
            print(f"  Error: {e}")
            import traceback

            traceback.print_exc()

    print(
        f"\nDone! Successfully converted {success_count}/{len(variants_to_convert)} robots."
    )
    return 0 if success_count == len(variants_to_convert) else 1


if __name__ == "__main__":
    exit(main())
