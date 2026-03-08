#!/usr/bin/env python3
from __future__ import annotations

import os
import json
import fnmatch
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Any

from ament_index_python.packages import (
    get_packages_with_prefixes,
    get_package_share_directory,
    PackageNotFoundError,
)


INTERESTING_PATTERNS = {
    "launch_files": ["*.launch.py", "*.launch.xml", "*.launch.yaml"],
    "config_files": ["*.yaml", "*.yml", "*.json", "*.toml", "*.ini"],
    "urdf_xacro": ["*.urdf", "*.xacro", "*.sdf"],
    "rviz": ["*.rviz"],
    "meshes": ["*.dae", "*.stl", "*.obj"],
    "plugins": ["*plugin*.xml", "*plugins*.xml"],
    "worlds": ["*.world", "*.sdf"],
    "maps": ["*.pgm", "*.png", "*.jpg", "*.yaml"],
}


def safe_parse_package_xml(package_xml: Path) -> Dict[str, Any]:
    info: Dict[str, Any] = {
        "name": None,
        "version": None,
        "description": None,
        "maintainers": [],
        "licenses": [],
        "buildtool_depends": [],
        "build_depends": [],
        "exec_depends": [],
        "test_depends": [],
        "export": [],
    }

    if not package_xml.exists():
        return info

    try:
        root = ET.parse(package_xml).getroot()
        info["name"] = (root.findtext("name") or "").strip()
        info["version"] = (root.findtext("version") or "").strip()
        info["description"] = (root.findtext("description") or "").strip()

        for tag in ["maintainer", "license", "buildtool_depend", "build_depend",
                    "exec_depend", "test_depend"]:
            values = []
            for elem in root.findall(tag):
                text = (elem.text or "").strip()
                if text:
                    if tag == "maintainer":
                        email = elem.attrib.get("email", "")
                        values.append({"name": text, "email": email})
                    else:
                        values.append(text)

            if tag == "maintainer":
                info["maintainers"] = values
            elif tag == "license":
                info["licenses"] = values
            elif tag == "buildtool_depend":
                info["buildtool_depends"] = values
            elif tag == "build_depend":
                info["build_depends"] = values
            elif tag == "exec_depend":
                info["exec_depends"] = values
            elif tag == "test_depend":
                info["test_depends"] = values

        export_elem = root.find("export")
        if export_elem is not None:
            for child in export_elem:
                info["export"].append({
                    "tag": child.tag,
                    "attrib": dict(child.attrib),
                    "text": (child.text or "").strip(),
                })

    except Exception as e:
        info["parse_error"] = str(e)

    return info


def walk_and_match(root_dir: Path, patterns: List[str]) -> List[str]:
    matches: List[str] = []
    if not root_dir.exists():
        return matches

    for path in root_dir.rglob("*"):
        if not path.is_file():
            continue
        for pat in patterns:
            if fnmatch.fnmatch(path.name, pat):
                matches.append(str(path))
                break
    return sorted(matches)


def sniff_package(pkg_name: str) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "package": pkg_name,
        "prefix": None,
        "share_dir": None,
        "package_xml": None,
        "metadata": {},
        "assets": {},
        "likely_package_type": "unknown",
        "has_python_module": False,
        "has_libexec": False,
    }

    packages = get_packages_with_prefixes()
    prefix = packages.get(pkg_name)
    if prefix is None:
        raise PackageNotFoundError(pkg_name)

    result["prefix"] = prefix

    share_dir = Path(get_package_share_directory(pkg_name))
    result["share_dir"] = str(share_dir)

    package_xml = share_dir / "package.xml"
    result["package_xml"] = str(package_xml) if package_xml.exists() else None
    result["metadata"] = safe_parse_package_xml(package_xml)

    for category, pats in INTERESTING_PATTERNS.items():
        result["assets"][category] = walk_and_match(share_dir, pats)

    # Heuristics
    buildtool_depends = set(result["metadata"].get("buildtool_depends", []))
    exports = result["metadata"].get("export", [])

    if "ament_python" in buildtool_depends:
        result["likely_package_type"] = "ament_python"
    elif "ament_cmake" in buildtool_depends:
        result["likely_package_type"] = "ament_cmake"

    # Look for lib/<package> executables
    libexec_dir = Path(prefix) / "lib" / pkg_name
    if libexec_dir.exists() and libexec_dir.is_dir():
        result["has_libexec"] = True
        result["assets"]["libexec"] = sorted(
            str(p) for p in libexec_dir.iterdir() if p.is_file()
        )
    else:
        result["assets"]["libexec"] = []

    # Look for installed python module
    py_candidates = [
        Path(prefix) / "lib" / "python3.12" / "site-packages" / pkg_name,
        Path(prefix) / "lib" / "python3.10" / "site-packages" / pkg_name,
        Path(prefix) / "local" / "lib" / "python3.12" / "dist-packages" / pkg_name,
        Path(prefix) / "local" / "lib" / "python3.10" / "dist-packages" / pkg_name,
    ]
    result["has_python_module"] = any(p.exists() for p in py_candidates)

    # Surface export tags that often matter
    result["plugin_exports"] = [
        x for x in exports
        if "plugin" in x["tag"].lower() or "plugin" in json.dumps(x).lower()
    ]

    return result


def sniff_all_packages(limit: int | None = None) -> Dict[str, Any]:
    pkgs = sorted(get_packages_with_prefixes().keys())
    if limit is not None:
        pkgs = pkgs[:limit]

    out = {
        "package_count": len(pkgs),
        "packages": [],
    }

    for name in pkgs:
        try:
            out["packages"].append(sniff_package(name))
        except Exception as e:
            out["packages"].append({
                "package": name,
                "error": str(e),
            })

    return out


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="ROS 2 package sniffer")
    parser.add_argument("--package", type=str, default=None,
                        help="Inspect one package only")
    parser.add_argument("--limit", type=int, default=None,
                        help="Limit number of packages")
    parser.add_argument("--pretty", action="store_true",
                        help="Pretty-print JSON")
    args = parser.parse_args()

    if args.package:
        data = sniff_package(args.package)
    else:
        data = sniff_all_packages(limit=args.limit)

    if args.pretty:
        print(json.dumps(data, indent=2))
    else:
        print(json.dumps(data))