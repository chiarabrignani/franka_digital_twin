import re
import shutil
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent

FOLDERS = [
    BASE / "meshes/pt/robots/fr3/visual",
    BASE / "meshes/pt/robot_ee/franka_hand_white/visual"
]

GREEN = (0.0, 1.0, 0.15, 1.0)


def recolor_mesh(path):
    text = path.read_text()

    # Backup della mesh originale
    backup = path.with_suffix(".dae.pre_green")

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

        not_dark = max(r, g, b) >= 0.15

        if not_dark:
            modified += 1
            color = " ".join(map(str, GREEN))
            return match.group(1) + color + match.group(3)

        return match.group(0)

    new_text = pattern.sub(replace_color, text)

    path.write_text(new_text)

    print(f"{path.name}: {modified} materiali colorati")


for folder in FOLDERS:
    for path in sorted(folder.glob("*.dae")):
        recolor_mesh(path)

print("\nColorazione del Physical Twin completata.")
