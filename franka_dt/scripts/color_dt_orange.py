import re
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent

FOLDERS = [
    BASE / "meshes/dt/robots/fr3/visual",
    BASE / "meshes/dt/robot_ee/franka_hand_white/visual"
]

ORANGE = (1.0, 0.45, 0.0, 1.0)


def recolor_mesh(path):
    text = path.read_text()

    # Backup della mesh originale
    backup = path.with_suffix(".dae.pre_orange")
    if not backup.exists():
        shutil.copy2(path, backup)

    modified = 0

    pattern = re.compile(
        r'(<diffuse>\s*<color[^>]*>)(.*?)(</color>\s*</diffuse>)',
        re.DOTALL
    )

    def replace_color(match):
        nonlocal modified

        values = [float(v) for v in match.group(2).split()]

        if len(values) != 4:
            return match.group(0)

        r, g, b, a = values

        # Ricolora soltanto materiali chiari e quasi neutri.
        neutral = max(r, g, b) - min(r, g, b) <= 0.08
        bright = (r + g + b) / 3 >= 0.50

        if neutral and bright:
            modified += 1
            color = " ".join(map(str, ORANGE))
            return match.group(1) + color + match.group(3)

        return match.group(0)

    new_text = pattern.sub(replace_color, text)

    # Controllo XML prima di salvare
    ET.fromstring(new_text)

    path.write_text(new_text)

    print(f"{path.name}: {modified} materiali colorati")


for folder in FOLDERS:
    for path in sorted(folder.glob("*.dae")):
        recolor_mesh(path)

print("\nColorazione del Digital Twin completata.")
