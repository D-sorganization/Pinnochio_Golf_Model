"""URDF exporter from canonical YAML specification."""

from __future__ import annotations

import logging
from pathlib import Path

import typing

import yaml  # type: ignore[import-untyped]

logger = logging.getLogger(__name__)


# Maximum joint effort (torque) allowed in URDF export.
# Units: Newton-meters (N⋅m)
# Source: Typical upper bound for hobbyist/educational robot actuators.
MAX_EFFORT = 1000.0

# Maximum joint velocity allowed in URDF export.
# Units: Radians per second (rad/s)
# Source: Typical upper bound for safe robot simulation; adjust as needed for hardware.
MAX_VELOCITY = 10.0

# Minimum degrees of freedom for a universal joint.
# Universal joints, by definition, require at least 2 rotational axes.
# Source: See https://en.wikipedia.org/wiki/Universal_joint
MIN_UNIVERSAL_DOFS = 2

# Minimum degrees of freedom for a gimbal joint.
# Gimbal joints require 3 rotational axes (pitch, yaw, roll).
# Source: See https://en.wikipedia.org/wiki/Gimbal
MIN_GIMBAL_DOFS = 3
# Number of joint limits expected for a revolute joint [lower, upper]
JOINT_LIMIT_COUNT = 2


class URDFExporter:
    """Export URDF from canonical YAML model specification."""

    def __init__(self, yaml_path: Path | str) -> None:
        """Initialize URDF exporter.

        Args:
            yaml_path: Path to canonical YAML specification
        """
        self.yaml_path = Path(yaml_path)
        with self.yaml_path.open() as f:
            self.spec = yaml.safe_load(f)

    def export(self, output_path: Path | str) -> None:
        """Export URDF file.

        Args:
            output_path: Path to output URDF file
        """
        output = Path(output_path)
        urdf_content = self._generate_urdf()
        output.write_text(urdf_content, encoding="utf-8")
        logger.info("Exported URDF to %s", output)

    def _generate_urdf(self) -> str:
        """Generate URDF XML content.

        Returns:
            URDF XML string
        """
        lines = ['<?xml version="1.0"?>', '<robot name="golfer">']
        lines.append("  <!-- Generated from canonical YAML specification -->")

        # Add root link
        root = self.spec["root"]
        lines.append(f'  <link name="{root["name"]}">')
        lines.extend(self._generate_inertial(root))
        lines.extend(self._generate_visual(root))
        lines.append("  </link>")

        # Add segments as links and joints
        for segment in self.spec.get("segments", []):
            lines.extend(self._generate_segment_urdf(segment, root["name"]))

        lines.append("</robot>")
        return "\n".join(lines)

    def _generate_segment_urdf(
        self, segment: dict[str, typing.Any], parent_name: str
    ) -> list[str]:
        """Generate URDF for a segment.

        Handles revolute, universal (2 revolute), and gimbal (3 revolute) joints.

        Args:
            segment: Segment specification
            parent_name: Parent link name

        Returns:
            List of URDF lines
        """
        lines = []
        seg_name = segment["name"]
        joint = segment.get("joint", {})
        joint_type = joint.get("type", "revolute")

        # Handle joint types that require multiple URDF joints
        if joint_type == "gimbal":
            # Gimbal joint: 3 revolute joints (Z, Y, X axes)
            lines.extend(
                self._generate_gimbal_joint(parent_name, seg_name, joint, segment)
            )
        elif joint_type == "universal":
            # Universal joint: 2 revolute joints (perpendicular axes)
            lines.extend(
                self._generate_universal_joint(parent_name, seg_name, joint, segment)
            )
        else:
            # Single revolute joint
            lines.extend(
                self._generate_single_joint(parent_name, seg_name, joint, segment)
            )

        return lines

    def _generate_single_joint(
        self,
        parent_name: str,
        seg_name: str,
        joint: dict[str, typing.Any],
        segment: dict[str, typing.Any],
    ) -> list[str]:
        """Generate URDF for a single revolute joint.

        Args:
            parent_name: Parent link name
            seg_name: Segment link name
            joint: Joint specification
            segment: Segment specification

        Returns:
            List of URDF lines
        """
        lines = []
        joint_name = f"{parent_name}_to_{seg_name}"

        # Get joint properties
        axis = joint.get("axis", [0, 0, 1])
        limits = joint.get("limits")
        damping = joint.get("damping")

        # Generate joint
        lines.extend(
            self._generate_revolute_joint_block(
                joint_name, parent_name, seg_name, axis, limits, damping
            )
        )

        # Link
        lines.append(f'  <link name="{seg_name}">')
        lines.extend(self._generate_inertial(segment))
        lines.extend(self._generate_visual(segment))
        lines.append("  </link>")

        return lines

    def _generate_universal_joint(
        self,
        parent_name: str,
        seg_name: str,
        joint: dict[str, typing.Any],
        segment: dict[str, typing.Any],
    ) -> list[str]:
        """Generate URDF for a universal joint (2 revolute joints).

        Args:
            parent_name: Parent link name
            seg_name: Segment link name
            joint: Joint specification with 'dofs' list
            segment: Segment specification

        Returns:
            List of URDF lines
        """
        lines = []
        intermediate_link = f"{seg_name}_intermediate"

        # Get DOF specifications
        dofs = joint.get("dofs", [])
        if len(dofs) < MIN_UNIVERSAL_DOFS:
            # Default: X and Y axes
            dofs = [
                {"axis": [1, 0, 0], "limits": [-1.57, 1.57]},
                {"axis": [0, 1, 0], "limits": [-1.57, 1.57]},
            ]

        # First DOF (X-axis typically)
        dof1 = dofs[0]
        lines.extend(
            self._generate_revolute_joint_block(
                f"{parent_name}_to_{intermediate_link}",
                parent_name,
                intermediate_link,
                dof1.get("axis", [1, 0, 0]),
                dof1.get("limits", [-1.57, 1.57]),
                joint.get("damping"),
            )
        )

        # Intermediate link (massless)
        lines.append(f'  <link name="{intermediate_link}">')
        lines.extend(self._generate_massless_inertial())
        lines.append("  </link>")

        # Second DOF (Y-axis typically)
        dof2 = dofs[1]
        lines.extend(
            self._generate_revolute_joint_block(
                f"{intermediate_link}_to_{seg_name}",
                intermediate_link,
                seg_name,
                dof2.get("axis", [0, 1, 0]),
                dof2.get("limits", [-1.57, 1.57]),
                joint.get("damping"),
            )
        )

        # Final link
        lines.append(f'  <link name="{seg_name}">')
        lines.extend(self._generate_inertial(segment))
        lines.extend(self._generate_visual(segment))
        lines.append("  </link>")

        return lines

    def _generate_gimbal_joint(
        self,
        parent_name: str,
        seg_name: str,
        joint: dict[str, typing.Any],
        segment: dict[str, typing.Any],
    ) -> list[str]:
        """Generate URDF for a gimbal joint (3 revolute joints: Z, Y, X).

        Args:
            parent_name: Parent link name
            seg_name: Segment link name
            joint: Joint specification with 'dofs' list
            segment: Segment specification

        Returns:
            List of URDF lines
        """
        lines = []
        intermediate1 = f"{seg_name}_gimbal_z"
        intermediate2 = f"{seg_name}_gimbal_y"

        # Get DOF specifications (default: Z, Y, X)
        dofs = joint.get("dofs", [])
        if len(dofs) < MIN_GIMBAL_DOFS:
            dofs = [
                {"axis": [0, 0, 1], "limits": [-3.14, 3.14]},  # Z
                {"axis": [0, 1, 0], "limits": [-1.57, 1.57]},  # Y
                {"axis": [1, 0, 0], "limits": [-1.57, 1.57]},  # X
            ]

        # First DOF (Z-axis)
        dof1 = dofs[0]
        lines.extend(
            self._generate_revolute_joint_block(
                f"{parent_name}_to_{intermediate1}",
                parent_name,
                intermediate1,
                dof1.get("axis", [0, 0, 1]),
                dof1.get("limits", [-3.14, 3.14]),
                joint.get("damping"),
            )
        )

        # First intermediate link (massless)
        lines.append(f'  <link name="{intermediate1}">')
        lines.extend(self._generate_massless_inertial())
        lines.append("  </link>")

        # Second DOF (Y-axis)
        dof2 = dofs[1]
        lines.extend(
            self._generate_revolute_joint_block(
                f"{intermediate1}_to_{intermediate2}",
                intermediate1,
                intermediate2,
                dof2.get("axis", [0, 1, 0]),
                dof2.get("limits", [-1.57, 1.57]),
                joint.get("damping"),
            )
        )

        # Second intermediate link (massless)
        lines.append(f'  <link name="{intermediate2}">')
        lines.extend(self._generate_massless_inertial())
        lines.append("  </link>")

        # Third DOF (X-axis)
        dof3 = dofs[2]
        lines.extend(
            self._generate_revolute_joint_block(
                f"{intermediate2}_to_{seg_name}",
                intermediate2,
                seg_name,
                dof3.get("axis", [1, 0, 0]),
                dof3.get("limits", [-1.57, 1.57]),
                joint.get("damping"),
            )
        )

        # Final link
        lines.append(f'  <link name="{seg_name}">')
        lines.extend(self._generate_inertial(segment))
        lines.extend(self._generate_visual(segment))
        lines.append("  </link>")

        return lines

    def _generate_inertial(self, body: dict[str, typing.Any]) -> list[str]:
        """Generate inertial properties.

        Args:
            body: Body specification with mass and inertia

        Returns:
            List of URDF lines
        """
        lines = ["    <inertial>"]
        lines.append(f'      <mass value="{body["mass"]}"/>')
        lines.append("      <inertia")
        lines.append(f'        ixx="{body["inertia"]["ixx"]}"')
        lines.append(f'        ixy="{body["inertia"]["ixy"]}"')
        lines.append(f'        ixz="{body["inertia"]["ixz"]}"')
        lines.append(f'        iyy="{body["inertia"]["iyy"]}"')
        lines.append(f'        iyz="{body["inertia"]["iyz"]}"')
        lines.append(f'        izz="{body["inertia"]["izz"]}"/>')
        lines.append("    </inertial>")
        return lines

    def _generate_massless_inertial(self) -> list[str]:
        """Generate inertial properties for a massless link.

        Returns:
            List of URDF lines
        """
        return [
            "    <inertial>",
            '      <mass value="0.001"/>',
            '      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>',
            "    </inertial>",
        ]

    def _generate_revolute_joint_block(  # noqa: PLR0913
        self,
        name: str,
        parent: str,
        child: str,
        axis: list[float],
        limits: list[float] | None = None,
        damping: float | None = None,
    ) -> list[str]:
        """Generate URDF for a revolute joint.

        Args:
            name: Joint name
            parent: Parent link name
            child: Child link name
            axis: Rotation axis [x, y, z]
            limits: Joint limits [lower, upper]
            damping: Damping coefficient

        Returns:
            List of URDF lines
        """
        lines = [
            f'  <joint name="{name}" type="revolute">',
            f'    <parent link="{parent}"/>',
            f'    <child link="{child}"/>',
            '    <origin xyz="0 0 0" rpy="0 0 0"/>',
            f'    <axis xyz="{axis[0]} {axis[1]} {axis[2]}"/>',
        ]

        if limits and len(limits) == JOINT_LIMIT_COUNT:
            lines.append(
                f'    <limit lower="{limits[0]}" upper="{limits[1]}" '
                f'effort="{MAX_EFFORT}" velocity="{MAX_VELOCITY}"/>'
            )

        if damping is not None:
            lines.append(f'    <dynamics damping="{damping}"/>')

        lines.append("  </joint>")
        return lines

    def _generate_visual(self, body: dict[str, typing.Any]) -> list[str]:
        """Generate visual geometry.

        Args:
            body: Body specification with geometry

        Returns:
            List of URDF lines
        """
        lines = ["    <visual>"]
        geom = body.get("geometry", {})
        geom_type = geom.get("type", "box")

        if geom_type == "box":
            size = geom.get("size", [0.1, 0.1, 0.1])
            lines.append("      <geometry>")
            lines.append(f'        <box size="{size[0]} {size[1]} {size[2]}"/>')
            lines.append("      </geometry>")
        elif geom_type == "sphere":
            size = geom.get("size", 0.1)
            lines.append("      <geometry>")
            lines.append(f'        <sphere radius="{size}"/>')
            lines.append("      </geometry>")
        elif geom_type in ("cylinder", "capsule"):
            size = geom.get("size", [0.1, 0.1])
            lines.append("      <geometry>")
            lines.append(f'        <cylinder radius="{size[0]}" length="{size[1]*2}"/>')
            lines.append("      </geometry>")

        rgba = geom.get("visual_rgba", [0.5, 0.5, 0.5, 1.0])
        lines.append(f'      <material name="mat_{body["name"]}">')
        lines.append(f'        <color rgba="{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}"/>')
        lines.append("      </material>")
        lines.append("    </visual>")
        return lines
