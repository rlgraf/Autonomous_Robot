from pathlib import Path
from xml.sax.saxutils import escape
from math import atan2, cos, sin

try:
    import cairosvg
except ImportError:
    cairosvg = None

OUT_DIR = Path(".")
DRAWIO_OUT = OUT_DIR / "ros2_architecture_nodes_only.drawio"
SVG_OUT = OUT_DIR / "ros2_architecture_nodes_only.svg"
PNG_OUT = OUT_DIR / "ros2_architecture_nodes_only.png"

nodes = [
    ("ros_gz_bridge", "ros_gz_bridge\nPublishes: /clock, /scan, /odom_gt, /odom, /joint_states\nSubscribes: /cmd_vel", 620, 720, 220, 110, "rounded=1;whiteSpace=wrap;html=1;fillColor=#e0e0e0;strokeColor=#666666;"),
    ("robot_state_publisher", "robot_state_publisher\nSubscribes: /joint_states, /clock\nPublishes: /tf, /tf_static, /robot_description", 890, 720, 220, 110, "rounded=1;whiteSpace=wrap;html=1;fillColor=#e0e0e0;strokeColor=#666666;"),
    ("odom_gt_bridge", "odom_gt_bridge\nBridge/helper for /odom_gt", 360, 740, 180, 70, "rounded=1;whiteSpace=wrap;html=1;fillColor=#e0e0e0;strokeColor=#666666;"),
    ("identify5", "identify5\nSub: /odom_gt, /scan, /clock\nPub: /detected_objects, /perception_alive", 60, 110, 220, 100, "rounded=1;whiteSpace=wrap;html=1;fillColor=#dae8fc;strokeColor=#6c8ebf;"),
    ("move5", "move5\nSub: /detected_objects, /odom_gt,\n/navigation_paused, /supervisor_active, /clock\nPub: /cmd_vel_nav, /depart_request, /visited_columns", 60, 300, 240, 120, "rounded=1;whiteSpace=wrap;html=1;fillColor=#dae8fc;strokeColor=#6c8ebf;"),
    ("soft_avoid", "soft_obstacle_avoidance_node\nSub: /scan, /recharge_active, /clock\nPub: /cmd_vel_soft_avoid, /navigation_paused", 1010, 120, 250, 100, "rounded=1;whiteSpace=wrap;html=1;fillColor=#f8cecc;strokeColor=#b85450;"),
    ("avoid_interact", "avoid_while_interact\nSub: /scan, /battery_charging,\n/recharge_active, /clock\nPub: /cmd_vel_avoid, /avoid_active, /navigation_paused", 1010, 270, 250, 110, "rounded=1;whiteSpace=wrap;html=1;fillColor=#f8cecc;strokeColor=#b85450;"),
    ("battery_node", "battery_node\nSub: /odom_gt\nPub: /battery_status, /battery_charging,\n/battery_depleted", 1010, 460, 250, 100, "rounded=1;whiteSpace=wrap;html=1;fillColor=#d5e8d4;strokeColor=#82b366;"),
    ("auto_recharge", "auto_recharge_node\nSub: /battery_status, /odom_gt,\n/navigation_paused, /clock\nPub: /cmd_vel_recharge, /recharge_active", 1010, 610, 250, 110, "rounded=1;whiteSpace=wrap;html=1;fillColor=#d5e8d4;strokeColor=#82b366;"),
    ("supervisor", "supervisor_node\nArbitrates navigation, avoidance,\nand recharge commands\nPub: /cmd_vel, /decision_metrics,\n/supervisor_active, /supervisor_events", 520, 290, 280, 140, "rounded=1;whiteSpace=wrap;html=1;fillColor=#f4cccc;strokeColor=#cc0000;strokeWidth=2;"),
    ("data_logger", "data_logger\nSub: /battery_status, /battery_charging,\n/decision_metrics, /supervisor_events,\n/odom_gt, /clock", 1330, 360, 240, 110, "rounded=1;whiteSpace=wrap;html=1;fillColor=#e0e0e0;strokeColor=#666666;"),
    ("title", "ROS2 Architecture for Autonomous Navigation,\nObstacle Avoidance, Recharge, and Supervisory Control", 350, 20, 650, 55, "text;html=1;strokeColor=none;fillColor=none;align=center;verticalAlign=middle;fontSize=20;fontStyle=1;"),
]

# Connections kept for visual architecture, but labels omitted.
edges = [
    ("e1", "ros_gz_bridge", "identify5"),
    ("e2", "ros_gz_bridge", "move5"),
    ("e3", "ros_gz_bridge", "soft_avoid"),
    ("e4", "ros_gz_bridge", "avoid_interact"),
    ("e5", "ros_gz_bridge", "battery_node"),
    ("e6", "ros_gz_bridge", "auto_recharge"),
    ("e7", "ros_gz_bridge", "supervisor"),
    ("e8", "ros_gz_bridge", "robot_state_publisher"),
    ("e9", "ros_gz_bridge", "data_logger"),
    ("e10", "identify5", "move5"),
    ("e11", "identify5", "supervisor"),
    ("e12", "move5", "supervisor"),
    ("e13", "soft_avoid", "supervisor"),
    ("e14", "soft_avoid", "move5"),
    ("e15", "soft_avoid", "auto_recharge"),
    ("e16", "avoid_interact", "supervisor"),
    ("e17", "avoid_interact", "move5"),
    ("e18", "avoid_interact", "auto_recharge"),
    ("e19", "battery_node", "auto_recharge"),
    ("e20", "battery_node", "avoid_interact"),
    ("e21", "battery_node", "supervisor"),
    ("e22", "auto_recharge", "supervisor"),
    ("e23", "auto_recharge", "soft_avoid"),
    ("e24", "auto_recharge", "avoid_interact"),
    ("e25", "supervisor", "move5"),
    ("e26", "supervisor", "ros_gz_bridge"),
    ("e27", "supervisor", "data_logger"),
    ("e28", "battery_node", "data_logger"),
]

WIDTH = 1600
HEIGHT = 1000
NODE_MAP = {n[0]: n for n in nodes}

def parse_style(style: str):
    parts = {}
    for item in style.split(";"):
        if "=" in item:
            k, v = item.split("=", 1)
            parts[k] = v
    return parts

def mxcell_vertex(id_, label, x, y, w, h, style):
    return f'''<mxCell id="{id_}" value="{escape(label)}" style="{style}" vertex="1" parent="1">
      <mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry" />
    </mxCell>'''

def mxcell_edge(id_, source, target):
    style = "edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;html=1;endArrow=block;endFill=1;strokeColor=#666666;"
    return f'''<mxCell id="{id_}" value="" style="{style}" edge="1" parent="1" source="{source}" target="{target}">
      <mxGeometry relative="1" as="geometry" />
    </mxCell>'''

def node_center(node_id):
    _, _, x, y, w, h, _ = NODE_MAP[node_id]
    return x + w / 2, y + h / 2

def connection_points(src_id, dst_id):
    sx, sy = node_center(src_id)
    dx, dy = node_center(dst_id)
    _, _, x1, y1, w1, h1, _ = NODE_MAP[src_id]
    _, _, x2, y2, w2, h2, _ = NODE_MAP[dst_id]
    vx, vy = dx - sx, dy - sy

    if abs(vx / max(w1, 1)) > abs(vy / max(h1, 1)):
        p1 = (x1 + w1, sy) if vx >= 0 else (x1, sy)
    else:
        p1 = (sx, y1 + h1) if vy >= 0 else (sx, y1)

    if abs(vx / max(w2, 1)) > abs(vy / max(h2, 1)):
        p2 = (x2, dy) if vx >= 0 else (x2 + w2, dy)
    else:
        p2 = (dx, y2) if vy >= 0 else (dx, y2 + h2)

    return p1, p2

def orthogonal_path(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    midx = (x1 + x2) / 2
    return [(x1, y1), (midx, y1), (midx, y2), (x2, y2)]

def arrow_polygon(p1, p2, head_len=10, head_w=5):
    ang = atan2(p2[1] - p1[1], p2[0] - p1[0])
    bx = p2[0] - head_len * cos(ang)
    by = p2[1] - head_len * sin(ang)
    left = (bx + head_w * sin(ang), by - head_w * cos(ang))
    right = (bx - head_w * sin(ang), by + head_w * cos(ang))
    return [p2, left, right]

def svg_text_lines(text, x, y, w, h, font_size=14, bold=False):
    lines = text.split("\n")
    line_height = font_size * 1.28
    total_h = line_height * len(lines)
    y0 = y + (h - total_h) / 2 + font_size
    weight = "700" if bold else "400"
    out = []
    for i, line in enumerate(lines):
        out.append(
            f'<text x="{x + w/2:.1f}" y="{y0 + i*line_height:.1f}" '
            f'font-family="Arial, Helvetica, sans-serif" font-size="{font_size}" '
            f'font-weight="{weight}" text-anchor="middle" fill="#111111">{escape(line)}</text>'
        )
    return "\n".join(out)

def build_drawio():
    cells = ['<mxCell id="0" />', '<mxCell id="1" parent="0" />']
    for n in nodes:
        cells.append(mxcell_vertex(*n))
    for e in edges:
        cells.append(mxcell_edge(*e))

    xml = f'''<mxfile host="app.diagrams.net" modified="2026-03-10T00:00:00.000Z" agent="Python" version="24.7.17" type="device">
  <diagram id="ros2arch" name="ROS2 Architecture Nodes Only">
    <mxGraphModel dx="1564" dy="906" grid="1" gridSize="10" guides="1" tooltips="1" connect="1" arrows="1" fold="1" page="1" pageScale="1" pageWidth="1600" pageHeight="1000" math="0" shadow="0">
      <root>
        {''.join(cells)}
      </root>
    </mxGraphModel>
  </diagram>
</mxfile>
'''
    DRAWIO_OUT.write_text(xml, encoding="utf-8")

def build_svg():
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}">',
        '<rect x="0" y="0" width="100%" height="100%" fill="#ffffff"/>',
    ]

    for _, src, dst in edges:
        p1, p2 = connection_points(src, dst)
        pts = orthogonal_path(p1, p2)
        d = " L ".join([f"{x:.1f},{y:.1f}" for x, y in pts])
        parts.append(f'<path d="M {d}" fill="none" stroke="#666666" stroke-width="2"/>')
        tri = arrow_polygon(pts[-2], pts[-1])
        tri_str = " ".join(f"{x:.1f},{y:.1f}" for x, y in tri)
        parts.append(f'<polygon points="{tri_str}" fill="#666666"/>')

    for node_id, label, x, y, w, h, style in nodes:
        s = parse_style(style)
        fill = s.get("fillColor", "none")
        stroke = s.get("strokeColor", "none")
        stroke_width = s.get("strokeWidth", "1")
        rounded = s.get("rounded", "0") == "1"

        if s.get("text") == "text":
            parts.append(svg_text_lines(label, x, y, w, h, font_size=20, bold=True))
            continue

        rx = 16 if rounded else 0
        parts.append(
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" ry="{rx}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="{stroke_width}"/>'
        )
        parts.append(svg_text_lines(label, x, y, w, h, font_size=14, bold=False))

    parts.append("</svg>")
    SVG_OUT.write_text("\n".join(parts), encoding="utf-8")
    

if __name__ == "__main__":
    build_drawio()
    build_svg()
    print(f"Created {DRAWIO_OUT.resolve()}")
    print("For a real Visio .vsdx file, import the SVG into Visio and save as .vsdx.")