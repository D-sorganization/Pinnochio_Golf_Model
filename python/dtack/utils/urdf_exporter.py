"""URDF exporter from canonical YAML specification."""

from __future__ import annotations

import logging
from pathlib import Path

import yaml

logger = logging.getLogger(__name__)


class URDFExporter:
    """Export URDF from canonical YAML model specification."""

    def __init__(self, yaml_path: Path | str) -> None:
        """Initialize URDF exporter.

        Args:
            yaml_path: Path to canonical YAML specification
        """
        self.yaml_path = Path(yaml_path)
        with open(self.yaml_path) as f:
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
        return "
".join(lines)

    def _generate_segment_urdf(self, segment: dict, parent_name: str) -> list[str]:
        """Generate URDF for a segment.

        Args:
            segment: Segment specification
            parent_name: Parent link name

        Returns:
            List of URDF lines
        """
        lines = []
        seg_name = segment["name"]

        # Joint
        lines.append(f'  <joint name="{parent_name}_to_{seg_name}" type="{segment["joint"]["type"]}">')
        lines.append(f'    <parent link="{parent_name}"/>')
        lines.append(f'    <child link="{seg_name}"/>')
        if "axis" in segment["joint"]:
            axis = segment["joint"]["axis"]
            lines.append(f'    <axis xyz="{axis[0]} {axis[1]} {axis[2]}"/>')
        if "limits" in segment["joint"]:
            limits = segment["joint"]["limits"]
            if isinstance(limits, list) and len(limits) == 2:
                lines.append(f'    <limit lower="{limits[0]}" upper="{limits[1]}"/>')
        lines.append("  </joint>")

        # Link
        lines.append(f'  <link name="{seg_name}">')
        lines.extend(self._generate_inertial(segment))
        lines.extend(self._generate_visual(segment))
        lines.append("  </link>")

        return lines

    def _generate_inertial(self, body: dict) -> list[str]:
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

    def _generate_visual(self, body: dict) -> list[str]:
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
            lines.append(f'      <geometry>')
            lines.append(f'        <box size="{size[0]} {size[1]} {size[2]}"/>')
            lines.append(f'      </geometry>')
        elif geom_type == "sphere":
            size = geom.get("size", 0.1)
            lines.append(f'      <geometry>')
            lines.append(f'        <sphere radius="{size}"/>')
            lines.append(f'      </geometry>')
        elif geom_type == "cylinder":
            size = geom.get("size", [0.1, 0.1])
            lines.append(f'      <geometry>')
            lines.append(f'        <cylinder radius="{size[0]}" length="{size[1]*2}"/>')
            lines.append(f'      </geometry>')
        elif geom_type == "capsule":
            size = geom.get("size", [0.1, 0.1])
            lines.append(f'      <geometry>')
            lines.append(f'        <cylinder radius="{size[0]}" length="{size[1]*2}"/>')
            lines.append(f'      </geometry>')

        rgba = geom.get("visual_rgba", [0.5, 0.5, 0.5, 1.0])
        lines.append(f'      <material name="mat_{body["name"]}">')
        lines.append(f'        <color rgba="{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}"/>')
        lines.append(f'      </material>')
        lines.append("    </visual>")
        return lines
