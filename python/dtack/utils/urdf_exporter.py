"""URDF exporter from canonical YAML specification."""

from __future__ import annotations

import logging
from pathlib import Path

from typing import Any

import yaml  # type: ignore[import-untyped]

logger = logging.getLogger(__name__)


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

    def _generate_segment_urdf(self, segment: dict[str, Any], parent_name: str) -> list[str]:
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
                self._generate_gimbal_joint(
                    parent_name, seg_name, joint, segment
                )
            )
        elif joint_type == "universal":
            # Universal joint: 2 revolute joints (perpendicular axes)
            lines.extend(
                self._generate_universal_joint(
                    parent_name, seg_name, joint, segment
                )
            )
        else:
            # Single revolute joint
            lines.extend(
                self._generate_single_joint(
                    parent_name, seg_name, joint, segment
                )
            )

        return lines

    def _generate_single_joint(
        self, parent_name: str, seg_name: str, joint: dict[str, Any], segment: dict[str, Any]
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

        # Joint
        lines.append(f'  <joint name="{joint_name}" type="revolute">')
        lines.append(f'    <parent link="{parent_name}"/>')
        lines.append(f'    <child link="{seg_name}"/>')
        lines.append('    <origin xyz="0 0 0" rpy="0 0 0"/>')

        if "axis" in joint:
            axis = joint["axis"]
            lines.append(f'    <axis xyz="{axis[0]} {axis[1]} {axis[2]}"/>')

        if "limits" in joint:
            limits = joint["limits"]
            if isinstance(limits, list) and len(limits) == 2:
                lines.append(
                    f'    <limit lower="{limits[0]}" upper="{limits[1]}" effort="1000" velocity="10"/>'
                )

        if "damping" in joint:
            damping = joint["damping"]
            lines.append(f'    <dynamics damping="{damping}"/>')

        lines.append("  </joint>")

        # Link
        lines.append(f'  <link name="{seg_name}">')
        lines.extend(self._generate_inertial(segment))
        lines.extend(self._generate_visual(segment))
        lines.append("  </link>")

        return lines

    def _generate_universal_joint(
        self, parent_name: str, seg_name: str, joint: dict[str, Any], segment: dict[str, Any]
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
        if len(dofs) < 2:
            # Default: X and Y axes
            dofs = [
                {"axis": [1, 0, 0], "limits": [-1.57, 1.57]},
                {"axis": [0, 1, 0], "limits": [-1.57, 1.57]},
            ]

        # First DOF (X-axis typically)
        dof1 = dofs[0]
        axis1 = dof1.get("axis", [1, 0, 0])
        limits1 = dof1.get("limits", [-1.57, 1.57])

        lines.append(
            f'  <joint name="{parent_name}_to_{intermediate_link}" type="revolute">'
        )
        lines.append(f'    <parent link="{parent_name}"/>')
        lines.append(f'    <child link="{intermediate_link}"/>')
        lines.append('    <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append(f'    <axis xyz="{axis1[0]} {axis1[1]} {axis1[2]}"/>')
        lines.append(
            f'    <limit lower="{limits1[0]}" upper="{limits1[1]}" effort="1000" velocity="10"/>'
        )
        if "damping" in joint:
            lines.append(f'    <dynamics damping="{joint["damping"]}"/>')
        lines.append("  </joint>")

        # Intermediate link (massless)
        lines.append(f'  <link name="{intermediate_link}">')
        lines.append("    <inertial>")
        lines.append('      <mass value="0.001"/>')
        lines.append('      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>')
        lines.append("    </inertial>")
        lines.append("  </link>")

        # Second DOF (Y-axis typically)
        dof2 = dofs[1]
        axis2 = dof2.get("axis", [0, 1, 0])
        limits2 = dof2.get("limits", [-1.57, 1.57])

        lines.append(
            f'  <joint name="{intermediate_link}_to_{seg_name}" type="revolute">'
        )
        lines.append(f'    <parent link="{intermediate_link}"/>')
        lines.append(f'    <child link="{seg_name}"/>')
        lines.append('    <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append(f'    <axis xyz="{axis2[0]} {axis2[1]} {axis2[2]}"/>')
        lines.append(
            f'    <limit lower="{limits2[0]}" upper="{limits2[1]}" effort="1000" velocity="10"/>'
        )
        if "damping" in joint:
            lines.append(f'    <dynamics damping="{joint["damping"]}"/>')
        lines.append("  </joint>")

        # Final link
        lines.append(f'  <link name="{seg_name}">')
        lines.extend(self._generate_inertial(segment))
        lines.extend(self._generate_visual(segment))
        lines.append("  </link>")

        return lines

    def _generate_gimbal_joint(
        self, parent_name: str, seg_name: str, joint: dict[str, Any], segment: dict[str, Any]
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
        if len(dofs) < 3:
            dofs = [
                {"axis": [0, 0, 1], "limits": [-3.14, 3.14]},  # Z
                {"axis": [0, 1, 0], "limits": [-1.57, 1.57]},  # Y
                {"axis": [1, 0, 0], "limits": [-1.57, 1.57]},  # X
            ]

        # First DOF (Z-axis)
        dof1 = dofs[0]
        axis1 = dof1.get("axis", [0, 0, 1])
        limits1 = dof1.get("limits", [-3.14, 3.14])

        lines.append(
            f'  <joint name="{parent_name}_to_{intermediate1}" type="revolute">'
        )
        lines.append(f'    <parent link="{parent_name}"/>')
        lines.append(f'    <child link="{intermediate1}"/>')
        lines.append('    <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append(f'    <axis xyz="{axis1[0]} {axis1[1]} {axis1[2]}"/>')
        lines.append(
            f'    <limit lower="{limits1[0]}" upper="{limits1[1]}" effort="1000" velocity="10"/>'
        )
        if "damping" in joint:
            lines.append(f'    <dynamics damping="{joint["damping"]}"/>')
        lines.append("  </joint>")

        # First intermediate link (massless)
        lines.append(f'  <link name="{intermediate1}">')
        lines.append("    <inertial>")
        lines.append('      <mass value="0.001"/>')
        lines.append('      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>')
        lines.append("    </inertial>")
        lines.append("  </link>")

        # Second DOF (Y-axis)
        dof2 = dofs[1]
        axis2 = dof2.get("axis", [0, 1, 0])
        limits2 = dof2.get("limits", [-1.57, 1.57])

        lines.append(
            f'  <joint name="{intermediate1}_to_{intermediate2}" type="revolute">'
        )
        lines.append(f'    <parent link="{intermediate1}"/>')
        lines.append(f'    <child link="{intermediate2}"/>')
        lines.append('    <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append(f'    <axis xyz="{axis2[0]} {axis2[1]} {axis2[2]}"/>')
        lines.append(
            f'    <limit lower="{limits2[0]}" upper="{limits2[1]}" effort="1000" velocity="10"/>'
        )
        if "damping" in joint:
            lines.append(f'    <dynamics damping="{joint["damping"]}"/>')
        lines.append("  </joint>")

        # Second intermediate link (massless)
        lines.append(f'  <link name="{intermediate2}">')
        lines.append("    <inertial>")
        lines.append('      <mass value="0.001"/>')
        lines.append('      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>')
        lines.append("    </inertial>")
        lines.append("  </link>")

        # Third DOF (X-axis)
        dof3 = dofs[2]
        axis3 = dof3.get("axis", [1, 0, 0])
        limits3 = dof3.get("limits", [-1.57, 1.57])

        lines.append(
            f'  <joint name="{intermediate2}_to_{seg_name}" type="revolute">'
        )
        lines.append(f'    <parent link="{intermediate2}"/>')
        lines.append(f'    <child link="{seg_name}"/>')
        lines.append('    <origin xyz="0 0 0" rpy="0 0 0"/>')
        lines.append(f'    <axis xyz="{axis3[0]} {axis3[1]} {axis3[2]}"/>')
        lines.append(
            f'    <limit lower="{limits3[0]}" upper="{limits3[1]}" effort="1000" velocity="10"/>'
        )
        if "damping" in joint:
            lines.append(f'    <dynamics damping="{joint["damping"]}"/>')
        lines.append("  </joint>")

        # Final link
        lines.append(f'  <link name="{seg_name}">')
        lines.extend(self._generate_inertial(segment))
        lines.extend(self._generate_visual(segment))
        lines.append("  </link>")

        return lines

    def _generate_inertial(self, body: dict[str, Any]) -> list[str]:
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

    def _generate_visual(self, body: dict[str, Any]) -> list[str]:
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
