"""Shared holder geometry loads in ROS 1/2 descriptions without hardware."""

import hashlib
import json
import unittest
from xml.etree import ElementTree as ET

from detachable_tools_common.description import build_robot_description, package_share_path


class HolderVisualTest(unittest.TestCase):
    def test_pean_has_distinct_assets_and_simulation_controller(self):
        from detachable_tools_common.description import describe_hand, build_control_description, extract_couplings
        from detachable_tools_common.tool_detection import select_tool
        for side in ("lhand", "rhand"):
            for prefix in (False, True):
                item = describe_hand(side, "pean", include_detacher=False, prefix=prefix)
                root = ET.fromstring(item.robot_description)
                self.assertFalse(item.motors)
                self.assertEqual(root.find("ros2_control/hardware/plugin").text, "mock_components/GenericSystem")
                self.assertEqual(len(extract_couplings(item.robot_description)), 2)
                self.assertIsNotNone(root.find(f"link[@name='{item.tip_link}']"))
                self.assertEqual(len([m for m in root.findall("link/visual/geometry/mesh") if '/meshes/pean/' in m.get('filename', '')]), 6)
            root = ET.fromstring(build_control_description(side))
            for tag in ("link", "joint", "ros2_control"):
                names = [n.get("name") for n in root.findall(tag)]
                self.assertEqual(len(names), len(set(names)))
            holder = describe_hand(side, "needle_holder", include_detacher=False)
            self.assertEqual(select_tool(side, [m.id for m in holder.motors], include_detacher=False), "needle_holder")

    def test_all_holder_descriptions_use_shared_assets(self):
        package = package_share_path()
        descriptions = [build_robot_description(side, "needle_holder", prefix=prefix)
                        for side in ("lhand", "rhand") for prefix in (False, True)]
        descriptions.append((package / "urdf/surgenoid_needle_holder/surgenoid_needle_holder.urdf").read_text())
        for text in descriptions:
            root = ET.fromstring(text)
            visuals = [v for v in root.findall("link/visual")
                       if "simready_holder" in (v.find("geometry/mesh").get("filename", "")
                          if v.find("geometry/mesh") is not None else "")]
            self.assertEqual(len(visuals), 6)
            for v in visuals:
                uri = v.find("geometry/mesh").get("filename")
                self.assertTrue((package / uri.split("package://detachable_tools/")[1]).is_file())
                self.assertIsNotNone(v.find("material/color"))
            collisions = [m for m in root.findall("link/collision/geometry/mesh") if "simready_holder" in m.get("filename", "")]
            self.assertEqual(len(collisions), 2)
            for m in collisions:
                self.assertTrue((package / m.get("filename").split("package://detachable_tools/")[1]).is_file())
            # Hardware still connects to the original holder mechanism.
            self.assertTrue(any("holder" in j.get("name", "").lower() or "CLAMP" in j.get("name", "") for j in root.findall("joint")))

    def test_exported_meshes_match_sync_manifest(self):
        folder = package_share_path() / "meshes/simready_holder"
        manifest = json.loads((folder / "sync.json").read_text())
        for part, record in manifest["parts"].items():
            self.assertEqual(hashlib.sha256((folder / f"{part}.obj").read_bytes()).hexdigest(), record["export_sha256"])
            self.assertIn("mtllib holder.mtl", (folder / f"{part}.obj").read_text())
        self.assertTrue((folder / "holder.mtl").is_file())


if __name__ == "__main__":
    unittest.main()
