#!/usr/bin/env python3
"""
CleanWalker URDF Validator
Parses cleanwalker.urdf and reports structure, mass, joint limits,
and any missing inertias or malformed elements.
"""

import xml.etree.ElementTree as ET
import sys
from pathlib import Path

URDF_PATH = Path(__file__).parent / "cleanwalker.urdf"


def validate(path: Path) -> bool:
    """Parse and validate the URDF file. Returns True if no issues found."""
    try:
        tree = ET.parse(path)
    except ET.ParseError as e:
        print(f"FATAL: XML parse error: {e}")
        return False

    root = tree.getroot()
    if root.tag != "robot":
        print(f"FATAL: Root element is <{root.tag}>, expected <robot>")
        return False

    robot_name = root.attrib.get("name", "(unnamed)")
    print(f"Robot: {robot_name}")
    print("=" * 60)

    links = root.findall("link")
    joints = root.findall("joint")

    # --- Links ---
    print(f"\nLinks: {len(links)}")
    total_mass = 0.0
    issues = []

    for link in links:
        name = link.attrib.get("name", "(unnamed)")
        inertial = link.find("inertial")
        visual = link.find("visual")
        collision = link.find("collision")

        mass_val = 0.0
        if inertial is not None:
            mass_el = inertial.find("mass")
            inertia_el = inertial.find("inertia")
            if mass_el is None:
                issues.append(f"  Link '{name}': <inertial> present but missing <mass>")
            else:
                try:
                    mass_val = float(mass_el.attrib.get("value", 0))
                except ValueError:
                    issues.append(f"  Link '{name}': non-numeric mass value")
            if inertia_el is None:
                issues.append(f"  Link '{name}': <inertial> present but missing <inertia>")
            else:
                required_attrs = ["ixx", "iyy", "izz"]
                for attr in required_attrs:
                    if attr not in inertia_el.attrib:
                        issues.append(f"  Link '{name}': <inertia> missing '{attr}'")
        else:
            if visual is not None:
                issues.append(f"  Link '{name}': has <visual> but no <inertial> (cosmetic-only link)")

        if visual is not None and collision is None and mass_val > 0.1:
            issues.append(f"  Link '{name}': has <visual> + mass={mass_val:.2f} kg but no <collision>")

        total_mass += mass_val
        print(f"  {name:30s}  mass={mass_val:6.2f} kg"
              f"  visual={'Y' if visual is not None else '-'}"
              f"  collision={'Y' if collision is not None else '-'}"
              f"  inertial={'Y' if inertial is not None else '-'}")

    print(f"\nTotal mass: {total_mass:.2f} kg")

    # --- Joints ---
    print(f"\nJoints: {len(joints)}")

    joint_types = {}
    link_names = {l.attrib.get("name") for l in links}

    for joint in joints:
        jname = joint.attrib.get("name", "(unnamed)")
        jtype = joint.attrib.get("type", "(unspecified)")
        joint_types.setdefault(jtype, []).append(jname)

        parent = joint.find("parent")
        child = joint.find("child")

        if parent is None:
            issues.append(f"  Joint '{jname}': missing <parent>")
        elif parent.attrib.get("link") not in link_names:
            issues.append(f"  Joint '{jname}': parent link '{parent.attrib.get('link')}' not defined")

        if child is None:
            issues.append(f"  Joint '{jname}': missing <child>")
        elif child.attrib.get("link") not in link_names:
            issues.append(f"  Joint '{jname}': child link '{child.attrib.get('link')}' not defined")

        limit = joint.find("limit")
        axis = joint.find("axis")

        if jtype == "revolute":
            if limit is None:
                issues.append(f"  Joint '{jname}': revolute joint missing <limit>")
            if axis is None:
                issues.append(f"  Joint '{jname}': revolute joint missing <axis>")

        detail = ""
        if limit is not None:
            lo = limit.attrib.get("lower", "?")
            hi = limit.attrib.get("upper", "?")
            eff = limit.attrib.get("effort", "?")
            vel = limit.attrib.get("velocity", "?")
            detail = f"  limits=[{lo}, {hi}]  effort={eff}  vel={vel}"

        axis_str = ""
        if axis is not None:
            axis_str = f"  axis={axis.attrib.get('xyz', '?')}"

        print(f"  {jname:30s}  type={jtype:10s}{axis_str}{detail}")

    print("\nJoint type summary:")
    for jtype, names in sorted(joint_types.items()):
        print(f"  {jtype}: {len(names)}")

    # --- Tree connectivity check ---
    children_of = {}
    parent_of = {}
    for joint in joints:
        p = joint.find("parent")
        c = joint.find("child")
        if p is not None and c is not None:
            pname = p.attrib.get("link")
            cname = c.attrib.get("link")
            children_of.setdefault(pname, []).append(cname)
            if cname in parent_of:
                issues.append(f"  Link '{cname}': has multiple parent joints")
            parent_of[cname] = pname

    # Find root links (no parent joint)
    root_links = [l.attrib.get("name") for l in links if l.attrib.get("name") not in parent_of]
    if len(root_links) != 1:
        issues.append(f"  Expected 1 root link, found {len(root_links)}: {root_links}")

    # Check for orphan links (not root and not a child of any joint)
    all_joint_links = set(parent_of.keys()) | set(children_of.keys())
    for link in links:
        lname = link.attrib.get("name")
        if lname not in all_joint_links and lname not in root_links:
            issues.append(f"  Link '{lname}': orphan (not connected to any joint)")

    # --- Report ---
    print(f"\n{'=' * 60}")
    if issues:
        print(f"WARNINGS ({len(issues)}):")
        for issue in issues:
            print(issue)
    else:
        print("No issues found.")

    print(f"\nSummary: {len(links)} links, {len(joints)} joints, {total_mass:.2f} kg, {len(issues)} warnings")
    return len(issues) == 0


if __name__ == "__main__":
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else URDF_PATH
    if not path.exists():
        print(f"FATAL: File not found: {path}")
        sys.exit(1)
    ok = validate(path)
    sys.exit(0 if ok else 1)
