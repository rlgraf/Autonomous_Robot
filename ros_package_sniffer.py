#!/usr/bin/env python3

from pathlib import Path
from datetime import datetime
from collections import defaultdict
import xml.etree.ElementTree as ET
import re


WORKSPACE_ROOT = Path("/home/corelab/Autonomous_Robot")
PACKAGE_ROOT = WORKSPACE_ROOT / "src" / "mobile_robot"


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="ignore")
    except Exception:
        return ""


def parse_package_xml(path: Path):
    out = {
        "name": "unknown",
        "version": "unknown",
        "description": "",
        "dependencies": [],
    }
    if not path.exists():
        return out

    root = ET.parse(path).getroot()
    out["name"] = (root.findtext("name") or "unknown").strip()
    out["version"] = (root.findtext("version") or "unknown").strip()
    out["description"] = (root.findtext("description") or "").strip()

    deps = []
    for tag in ["depend", "exec_depend", "build_depend", "buildtool_depend"]:
        for elem in root.findall(tag):
            if elem.text:
                dep = elem.text.strip()
                if dep and dep not in deps:
                    deps.append(dep)

    out["dependencies"] = sorted(deps)
    return out


def parse_console_scripts(setup_py: Path):
    text = read_text(setup_py)

    m = re.search(
        r"console_scripts\s*:\s*\[(.*?)\]",
        text,
        flags=re.DOTALL,
    )
    if not m:
        m = re.search(
            r"['\"]console_scripts['\"]\s*:\s*\[(.*?)\]",
            text,
            flags=re.DOTALL,
        )

    blob = m.group(1) if m else ""
    entries = re.findall(r"['\"]([^'\"]+?)\s*=\s*([^'\"]+?)['\"]", blob)

    return [{"name": a.strip(), "target": b.strip()} for a, b in entries]


def build_ascii_tree(root_dir: Path, max_depth=4):
    if not root_dir.exists():
        return f"{root_dir}\n  (missing)"

    lines = [root_dir.name + "/"]

    ignore_names = {
        "__pycache__", ".git", ".pytest_cache", ".mypy_cache",
        "build", "install", "log", ".venv", "venv"
    }

    def walk(path: Path, prefix="", depth=0):
        if depth >= max_depth:
            return

        children = sorted(
            [p for p in path.iterdir() if p.name not in ignore_names],
            key=lambda p: (p.is_file(), p.name.lower())
        )

        for i, child in enumerate(children):
            last = i == len(children) - 1
            connector = "└── " if last else "├── "
            lines.append(prefix + connector + child.name + ("/" if child.is_dir() else ""))

            if child.is_dir():
                next_prefix = prefix + ("    " if last else "│   ")
                walk(child, next_prefix, depth + 1)

    walk(root_dir)
    return "\n".join(lines)


def parse_launch_files(launch_dir: Path):
    out = []
    if not launch_dir.exists():
        return out

    for path in sorted(launch_dir.glob("*.launch.py")):
        text = read_text(path)

        nodes = re.findall(
            r'Node\s*\([\s\S]*?package\s*=\s*[\'"]([^\'"]+)[\'"][\s\S]*?executable\s*=\s*[\'"]([^\'"]+)[\'"]',
            text
        )

        names = re.findall(r'name\s*=\s*[\'"]([^\'"]+)[\'"]', text)
        param_blocks = len(re.findall(r'parameters\s*=', text))

        out.append({
            "path": str(path),
            "nodes": [{"package": p, "executable": e} for p, e in nodes],
            "names": sorted(set(names)),
            "parameter_blocks": param_blocks,
        })

    return out


def find_call_args(text: str, call_name: str):
    pattern = rf"{call_name}\s*\((.*?)\)"
    return re.findall(pattern, text, flags=re.DOTALL)


def extract_topic_and_type_from_create_publisher(blob: str):
    m = re.search(r"([A-Za-z_][A-Za-z0-9_\.]*)\s*,\s*[\'\"]([^'\"]+)[\'\"]", blob)
    if m:
        return m.group(2), m.group(1).split(".")[-1]
    return "?", "?"


def extract_topic_and_type_from_create_subscription(blob: str):
    m = re.search(r"([A-Za-z_][A-Za-z0-9_\.]*)\s*,\s*[\'\"]([^'\"]+)[\'\"]", blob)
    if m:
        return m.group(2), m.group(1).split(".")[-1]
    return "?", "?"


def normalize_topic(topic: str):
    if not topic or topic == "?":
        return topic
    if not topic.startswith("/"):
        return "/" + topic
    return topic


def parse_python_node(py_path: Path):
    text = read_text(py_path)

    node_names = re.findall(r"super\(\).__init__\(\s*[\'\"]([^'\"]+)[\'\"]\s*\)", text)
    if not node_names:
        node_names = re.findall(r"Node\s*\.\s*__init__\(\s*self\s*,\s*[\'\"]([^'\"]+)[\'\"]\s*\)", text)

    publishers = []
    for blob in find_call_args(text, "create_publisher"):
        topic, msg_type = extract_topic_and_type_from_create_publisher(blob)
        publishers.append({"topic": normalize_topic(topic), "type": msg_type})

    subscriptions = []
    for blob in find_call_args(text, "create_subscription"):
        topic, msg_type = extract_topic_and_type_from_create_subscription(blob)
        subscriptions.append({"topic": normalize_topic(topic), "type": msg_type})

    declared = re.findall(r"declare_parameter\(\s*[\'\"]([^'\"]+)[\'\"]", text)
    accessed = re.findall(r"get_parameter\(\s*[\'\"]([^'\"]+)[\'\"]", text)
    timers = re.findall(r"create_timer\(\s*([0-9.]+)", text)

    return {
        "path": str(py_path),
        "node_names": sorted(set(node_names)),
        "publishers": publishers,
        "subscriptions": subscriptions,
        "parameters_declared": sorted(set(declared)),
        "parameters_accessed": sorted(set(accessed)),
        "timers_sec": sorted(set(timers), key=lambda x: float(x)),
    }


def parse_python_nodes(module_root: Path):
    nodes = []
    if not module_root.exists():
        return nodes

    for py_path in sorted(module_root.rglob("*.py")):
        if py_path.name == "__init__.py":
            continue
        nodes.append(parse_python_node(py_path))

    return nodes


def parse_yaml_files(param_dir: Path):
    out = []
    if not param_dir.exists():
        return out

    for path in sorted(param_dir.glob("*.yaml")):
        text = read_text(path)

        top_sections = []
        keys = []

        for line in text.splitlines():
            if not line.strip() or line.strip().startswith("#"):
                continue

            if re.match(r"^[A-Za-z0-9_]+:\s*$", line):
                name = line.strip().rstrip(":")
                if name not in top_sections:
                    top_sections.append(name)

            if ":" in line:
                key = line.split(":", 1)[0].strip()
                if key and key not in keys:
                    keys.append(key)

        out.append({
            "path": str(path),
            "top_sections": top_sections[:20],
            "keys_preview": keys[:40],
        })

    return out


def group_python_nodes(py_nodes):
    groups = {
        "battery_behavior": [],
        "people_behavior": [],
        "supervisor": [],
        "other": [],
    }

    for node in py_nodes:
        p = node["path"]
        if "battery_behavior" in p:
            groups["battery_behavior"].append(node)
        elif "people_behavior" in p:
            groups["people_behavior"].append(node)
        elif "supervisor" in p:
            groups["supervisor"].append(node)
        else:
            groups["other"].append(node)

    return groups


def summarize_launch_purpose(name: str):
    if name == "gazebo_model.launch.py":
        return [
            "world loading",
            "robot spawn",
            "bridge setup",
            "battery node startup",
        ]
    if name == "gazebo_model2.launch.py":
        return [
            "behavior and supervisor stack",
        ]
    if "simple_avoid" in name:
        return [
            "standalone obstacle avoidance test",
        ]
    return ["(not inferred)"]


def build_architecture_flow(py_nodes):
    topic_publishers = defaultdict(list)
    topic_subscribers = defaultdict(list)

    for node in py_nodes:
        node_name = node["node_names"][0] if node["node_names"] else Path(node["path"]).stem

        for pub in node["publishers"]:
            topic_publishers[pub["topic"]].append(node_name)

        for sub in node["subscriptions"]:
            topic_subscribers[sub["topic"]].append(node_name)

    sections = [
        ("Perception", {"/scan", "/detected_objects", "/perception_alive", "/odom_gt"}),
        ("Navigation / motion", {"/cmd_vel_nav", "/cmd_vel", "/depart_request", "/supervisor_active"}),
        ("Interaction avoidance", {"/avoid_active", "/cmd_vel_avoid", "/navigation_paused"}),
        ("Recharge / battery", {"/battery_status", "/battery_charging", "/battery_depleted", "/recharge_active", "/cmd_vel_recharge", "/cmd_vel_soft_avoid"}),
    ]

    lines = []
    for title, topics in sections:
        lines.append(title + ":")
        found = False
        for topic in sorted(topics):
            pubs = sorted(set(topic_publishers.get(topic, [])))
            subs = sorted(set(topic_subscribers.get(topic, [])))

            if pubs and subs:
                found = True
                for p in pubs:
                    for s in subs:
                        lines.append(f"  - {p} -> {topic} -> {s}")
            elif pubs:
                found = True
                for p in pubs:
                    lines.append(f"  - {p} -> {topic}")
            elif subs:
                found = True
                for s in subs:
                    lines.append(f"  - {topic} -> {s}")

        if not found:
            lines.append("  - (none)")
        lines.append("")

    return "\n".join(lines).rstrip()


def render_structure_summary(report):
    pkg = report["package_overview"]
    lines = []

    lines.append("ROS PACKAGE STRUCTURE SUMMARY")
    lines.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"Root: {report['root']}")
    lines.append("")

    lines.append("=" * 60)
    lines.append("PACKAGE")
    lines.append("=" * 60)
    lines.append(f"Name: {pkg['name']}")
    lines.append(f"Version: {pkg['version']}")
    lines.append(f"Description: {pkg['description'] or '(none)'}")
    lines.append("")
    lines.append("Dependencies:")
    if pkg["dependencies"]:
        for dep in pkg["dependencies"]:
            lines.append(f"  - {dep}")
    else:
        lines.append("  - (none)")
    lines.append("")
    lines.append("Console scripts:")
    if report["console_scripts"]:
        for s in report["console_scripts"]:
            lines.append(f"  - {s['name']} -> {s['target']}")
    else:
        lines.append("  - (none)")
    lines.append("")

    lines.append("=" * 60)
    lines.append("SOURCE TREE")
    lines.append("=" * 60)
    lines.append(build_ascii_tree(Path(report["package_root"]), max_depth=4))
    lines.append("")

    lines.append("=" * 60)
    lines.append("LAUNCH STRUCTURE")
    lines.append("=" * 60)
    if report["launch_files"]:
        for lf in report["launch_files"]:
            name = Path(lf["path"]).name
            lines.append(name)
            lines.append("  Purpose:")
            for purpose in summarize_launch_purpose(name):
                lines.append(f"    - {purpose}")
            lines.append("")
            lines.append("  Nodes:")
            if lf["nodes"]:
                for n in lf["nodes"]:
                    lines.append(f"    - {n['package']} / {n['executable']}")
            else:
                lines.append("    - (none)")
            if lf["names"]:
                lines.append("")
                lines.append("  Names:")
                for n in lf["names"]:
                    lines.append(f"    - {n}")
            lines.append(f"  parameter blocks: {lf['parameter_blocks']}")
            lines.append("")
    else:
        lines.append("(none)")
        lines.append("")

    lines.append("=" * 60)
    lines.append("NODE STRUCTURE")
    lines.append("=" * 60)
    grouped = group_python_nodes(report["python_nodes"])
    any_nodes = False

    for group_name, nodes in grouped.items():
        if not nodes:
            continue
        any_nodes = True
        lines.append(group_name)
        for node in nodes:
            name = node["node_names"][0] if node["node_names"] else Path(node["path"]).stem
            lines.append(f"  - {name}")

            if node["publishers"]:
                lines.append("      publishes: " + ", ".join(
                    f"{p['topic']} [{p['type']}]" for p in node["publishers"]
                ))
            if node["subscriptions"]:
                lines.append("      subscribes: " + ", ".join(
                    f"{s['topic']} [{s['type']}]" for s in node["subscriptions"]
                ))
            if node["parameters_declared"]:
                lines.append("      parameters declared: " + ", ".join(node["parameters_declared"]))
            if node["parameters_accessed"]:
                lines.append("      parameters accessed: " + ", ".join(node["parameters_accessed"]))
            if node["timers_sec"]:
                lines.append("      timers: " + ", ".join(node["timers_sec"]))
            lines.append("")
        lines.append("")

    if not any_nodes:
        lines.append("(none)")
        lines.append("")

    lines.append("=" * 60)
    lines.append("ROS NODE ARCHITECTURE")
    lines.append("=" * 60)
    lines.append(build_architecture_flow(report["python_nodes"]))
    lines.append("")

    lines.append("=" * 60)
    lines.append("PARAMETER STRUCTURE")
    lines.append("=" * 60)
    if report["yaml_files"]:
        for yf in report["yaml_files"]:
            lines.append(Path(yf["path"]).name)
            if yf["top_sections"]:
                lines.append("  Sections:")
                for sec in yf["top_sections"]:
                    lines.append(f"    - {sec}")
            if yf["keys_preview"]:
                lines.append("  Keys:")
                for key in yf["keys_preview"]:
                    lines.append(f"    - {key}")
            lines.append("")
    else:
        lines.append("(none)")
        lines.append("")

    return "\n".join(lines)


def build_report():
    return {
        "root": str(WORKSPACE_ROOT),
        "package_root": str(PACKAGE_ROOT),
        "package_overview": parse_package_xml(PACKAGE_ROOT / "package.xml"),
        "console_scripts": parse_console_scripts(PACKAGE_ROOT / "setup.py"),
        "launch_files": parse_launch_files(PACKAGE_ROOT / "launch"),
        "python_nodes": parse_python_nodes(PACKAGE_ROOT / "mobile_robot"),
        "yaml_files": parse_yaml_files(PACKAGE_ROOT / "parameters"),
    }


def main():
    if not PACKAGE_ROOT.exists():
        print(f"Package root not found: {PACKAGE_ROOT}")
        return

    report = build_report()
    summary = render_structure_summary(report)

    out_path = WORKSPACE_ROOT / "ros_package_structure_summary.txt"
    out_path.write_text(summary, encoding="utf-8")

    print(summary)
    print(f"\nSaved to: {out_path}")


if __name__ == "__main__":
    main()