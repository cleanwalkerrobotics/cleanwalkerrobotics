#!/usr/bin/env python3
"""
CleanWalker CW-1 URDF Validator

Validates the URDF file structure without external dependencies.
Checks: XML parsing, link/joint consistency, tree structure, inertial
properties, joint limits, and prints a summary.
"""

import xml.etree.ElementTree as ET
import math
import sys
import os

URDF_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cleanwalker_cw1.urdf")


def validate_urdf(path):
    errors = []
    warnings = []

    # --- 1. Parse XML ---
    try:
        tree = ET.parse(path)
        root = tree.getroot()
    except ET.ParseError as e:
        print(f"FATAL: XML parse error: {e}")
        return False

    if root.tag != "robot":
        errors.append(f"Root element is <{root.tag}>, expected <robot>")
        return report(errors, warnings)

    robot_name = root.attrib.get("name", "UNNAMED")
    print(f"Robot: {robot_name}")
    print("=" * 60)

    # --- 2. Collect links and joints ---
    links = {}
    for link_elem in root.findall("link"):
        name = link_elem.attrib.get("name")
        if not name:
            errors.append("Found <link> without 'name' attribute")
            continue
        if name in links:
            errors.append(f"Duplicate link name: {name}")
        links[name] = link_elem

    joints = {}
    for joint_elem in root.findall("joint"):
        name = joint_elem.attrib.get("name")
        if not name:
            errors.append("Found <joint> without 'name' attribute")
            continue
        if name in joints:
            errors.append(f"Duplicate joint name: {name}")
        joints[name] = joint_elem

    print(f"Links:  {len(links)}")
    print(f"Joints: {len(joints)}")
    print()

    # --- 3. Validate joint parent/child references ---
    child_links = set()
    parent_map = {}  # child -> parent

    for jname, jelem in joints.items():
        jtype = jelem.attrib.get("type", "MISSING")
        parent_elem = jelem.find("parent")
        child_elem = jelem.find("child")

        if parent_elem is None:
            errors.append(f"Joint '{jname}' missing <parent>")
            continue
        if child_elem is None:
            errors.append(f"Joint '{jname}' missing <child>")
            continue

        parent = parent_elem.attrib.get("link")
        child = child_elem.attrib.get("link")

        if parent not in links:
            errors.append(f"Joint '{jname}' references unknown parent link '{parent}'")
        if child not in links:
            errors.append(f"Joint '{jname}' references unknown child link '{child}'")

        if child in child_links:
            errors.append(f"Link '{child}' is child of multiple joints")
        child_links.add(child)
        parent_map[child] = parent

        # Check revolute joints have limits
        if jtype == "revolute":
            limit = jelem.find("limit")
            if limit is None:
                errors.append(f"Revolute joint '{jname}' missing <limit>")
            else:
                lower = float(limit.attrib.get("lower", 0))
                upper = float(limit.attrib.get("upper", 0))
                if lower >= upper:
                    errors.append(f"Joint '{jname}': lower ({lower}) >= upper ({upper})")

            axis = jelem.find("axis")
            if axis is None:
                warnings.append(f"Joint '{jname}' missing <axis> (defaults to X)")

    # --- 4. Find root link (not a child of any joint) ---
    root_links = set(links.keys()) - child_links
    if len(root_links) == 0:
        errors.append("No root link found (cycle detected?)")
    elif len(root_links) > 1:
        errors.append(f"Multiple root links: {root_links}")
    else:
        root_link = root_links.pop()
        print(f"Root link: {root_link}")

    # --- 5. Check all links are connected ---
    reachable = set()

    def walk(link_name):
        reachable.add(link_name)
        for child, parent in parent_map.items():
            if parent == link_name and child not in reachable:
                walk(child)

    if len(root_links) == 0 and len(links) > 0:
        walk(next(iter(links)))
    elif len(root_links) >= 1:
        walk(root_link)

    orphaned = set(links.keys()) - reachable
    if orphaned:
        errors.append(f"Orphaned links (disconnected from tree): {orphaned}")

    # --- 6. Inertial check ---
    links_with_mass = 0
    total_mass = 0.0
    for lname, lelem in links.items():
        inertial = lelem.find("inertial")
        if inertial is not None:
            mass_elem = inertial.find("mass")
            if mass_elem is not None:
                m = float(mass_elem.attrib.get("value", 0))
                if m > 0:
                    links_with_mass += 1
                    total_mass += m
                elif lname != root_link if 'root_link' in dir() else True:
                    warnings.append(f"Link '{lname}' has zero mass")

            inertia = inertial.find("inertia")
            if inertia is not None:
                ixx = float(inertia.attrib.get("ixx", 0))
                iyy = float(inertia.attrib.get("iyy", 0))
                izz = float(inertia.attrib.get("izz", 0))
                if ixx <= 0 or iyy <= 0 or izz <= 0:
                    mass_val = float(mass_elem.attrib.get("value", 0)) if mass_elem is not None else 0
                    if mass_val > 0:
                        warnings.append(f"Link '{lname}' has non-positive diagonal inertia")

    print(f"Links with mass: {links_with_mass}")
    print(f"Total mass: {total_mass:.2f} kg")
    print()

    # --- 7. Joint summary ---
    joint_types = {}
    revolute_joints = []
    for jname, jelem in joints.items():
        jtype = jelem.attrib.get("type", "unknown")
        joint_types[jtype] = joint_types.get(jtype, 0) + 1
        if jtype == "revolute":
            limit = jelem.find("limit")
            lower_deg = math.degrees(float(limit.attrib.get("lower", 0)))
            upper_deg = math.degrees(float(limit.attrib.get("upper", 0)))
            revolute_joints.append((jname, lower_deg, upper_deg))

    print("Joint types:")
    for jtype, count in sorted(joint_types.items()):
        print(f"  {jtype}: {count}")

    actuated = joint_types.get("revolute", 0) + joint_types.get("prismatic", 0) + joint_types.get("continuous", 0)
    print(f"\nActuated DOF: {actuated}")
    print()

    # --- 8. Revolute joint details ---
    print("Revolute joints:")
    print(f"  {'Joint':<35} {'Lower':>8}  {'Upper':>8}")
    print(f"  {'-'*35} {'-'*8}  {'-'*8}")
    for jname, lower, upper in revolute_joints:
        print(f"  {jname:<35} {lower:>7.1f}\u00b0  {upper:>7.1f}\u00b0")
    print()

    # --- 9. Kinematic tree ---
    print("Kinematic tree:")
    printed = set()

    def print_tree(link_name, depth=0):
        indent = "  " * depth
        connector = "\u251c\u2500 " if depth > 0 else ""
        mass_str = ""
        lelem = links.get(link_name)
        if lelem is not None:
            inertial = lelem.find("inertial")
            if inertial is not None:
                mass_elem = inertial.find("mass")
                if mass_elem is not None:
                    m = float(mass_elem.attrib.get("value", 0))
                    if m > 0:
                        mass_str = f" ({m:.1f} kg)"

        print(f"  {indent}{connector}{link_name}{mass_str}")
        printed.add(link_name)

        for child, parent in sorted(parent_map.items()):
            if parent == link_name and child not in printed:
                # Find joint connecting them
                for jn, je in joints.items():
                    p = je.find("parent").attrib.get("link")
                    c = je.find("child").attrib.get("link")
                    if p == link_name and c == child:
                        jtype = je.attrib.get("type", "?")
                        symbol = "\u21bb" if jtype == "revolute" else "\u2501" if jtype == "fixed" else "?"
                        print(f"  {'  ' * (depth + 1)}[{symbol} {jn}]")
                        break
                print_tree(child, depth + 1)

    if 'root_link' in dir():
        print_tree(root_link)
    elif len(links) > 0:
        first = next(iter(links))
        print_tree(first)
    print()

    # --- 10. Report ---
    return report(errors, warnings)


def report(errors, warnings):
    if warnings:
        print(f"WARNINGS ({len(warnings)}):")
        for w in warnings:
            print(f"  \u26a0  {w}")
        print()

    if errors:
        print(f"ERRORS ({len(errors)}):")
        for e in errors:
            print(f"  \u2717  {e}")
        print()
        print("VALIDATION FAILED")
        return False
    else:
        print("VALIDATION PASSED")
        return True


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else URDF_PATH
    print(f"Validating: {path}")
    print()
    success = validate_urdf(path)
    sys.exit(0 if success else 1)
