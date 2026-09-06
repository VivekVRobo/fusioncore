#!/usr/bin/env python3
"""Fail if a shipped config YAML sets a parameter the node never declares.

ROS 2 silently ignores a parameter a node did not declare. No error, no warning,
nothing in the log: the value simply has no effect, forever. A typo like
gnss.max_hodp is therefore invisible, and the user drives believing the gate is
tuned. Three certified configs carried gnss.max_hdop as though it were the active
gate for months (issue #79) for exactly this reason.

Usage:
    python3 tools/check_config_params.py [--quiet]

Exit 0 if every config is clean, 1 otherwise.
"""
import glob
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NODE = os.path.join(ROOT, "fusioncore_ros", "src", "fusion_node.cpp")

# Configs that configure OUR node. robot_localization and navsat_transform
# comparison configs are deliberately excluded: they configure other people's
# nodes and their keys are none of our business.
CONFIG_GLOBS = [
    "fusioncore_ros/config/*.yaml",
    "fusioncore_gazebo/config/fusioncore_gazebo.yaml",
    "fusioncore_datasets/config/nclt_fusioncore.yaml",
    "tools/quick_test_params.yaml",
    "docs/**/*.yaml",
]


def declared_parameters(path):
    """Every name passed to declare_parameter() in the node."""
    src = open(path).read()
    names = set(re.findall(r'declare_parameter[<\w,\s>]*\(\s*"([^"]+)"', src))
    if not names:
        sys.exit(f"{path}: found no declare_parameter calls, the regex is wrong")
    return names


def config_parameters(path):
    """Parameter keys under a node's ros__parameters, as dotted names.

    Hand-rolled rather than yaml.safe_load because these files are heavily
    commented and the comments carry the reasoning: keeping the parser dumb means
    it reports the line number a bad key is actually on.
    """
    # Only blocks belonging to OUR node. A YAML can configure several nodes at
    # once, and the docs ship exactly such a file: a Nav2 collision monitor sits
    # alongside fusioncore and its keys (PolygonStop, observation_sources,
    # cmd_vel_in_topic) are none of our business. Keying off the node name above
    # ros__parameters is what separates them.
    out = []
    in_params = False
    params_indent = 0
    stack = []
    node_name = None
    mine = False
    for n, raw in enumerate(open(path), 1):
        line = raw.rstrip("\n")
        if not line.strip() or line.strip().startswith("#"):
            continue
        indent = len(line) - len(line.lstrip())
        key = line.strip().split(":")[0].strip()
        if key == "ros__parameters":
            in_params, params_indent, stack = True, indent, []
            # "fusioncore", "/fusioncore", "/**/fusioncore", "a200_0000/fusioncore"
            mine = bool(node_name) and (node_name.rstrip("/").split("/")[-1] == "fusioncore"
                                        or node_name in ("/**", "**"))
            continue
        if not in_params and ":" in line and not line.split(":", 1)[1].strip():
            node_name = key
        if not in_params or not mine:
            continue
        if indent <= params_indent:
            in_params = False
            continue
        stack = [(i, k) for (i, k) in stack if i < indent]
        value = line.split(":", 1)[1].strip() if ":" in line else ""
        if not value:                       # a nesting level, not a leaf
            stack.append((indent, key))
            continue
        if line.strip().startswith("-"):    # list continuation
            continue
        out.append((".".join([k for _, k in stack] + [key]), n))
    return out


def main():
    quiet = "--quiet" in sys.argv
    declared = declared_parameters(NODE)
    files, bad = 0, 0
    for pattern in CONFIG_GLOBS:
        for path in sorted(glob.glob(os.path.join(ROOT, pattern), recursive=True)):
            rel = os.path.relpath(path, ROOT)
            unknown = [(k, n) for k, n in config_parameters(path) if k not in declared]
            files += 1
            if unknown:
                bad += 1
                print(f"\n{rel}")
                for k, n in unknown:
                    near = sorted(d for d in declared if d.split(".")[-1] == k.split(".")[-1])
                    hint = f"   did you mean {near[0]}?" if near else ""
                    print(f"  line {n:4d}  {k}  NOT DECLARED BY THE NODE{hint}")
            elif not quiet:
                print(f"ok  {rel}")
    print(f"\n{files} config files checked, {bad} with undeclared parameters")
    return 1 if bad else 0


sys.exit(main())
