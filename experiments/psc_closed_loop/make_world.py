"""Write nezha_lake_waves.world: nezha_lake.world with an inline wave field of the given
amplitude and period, plus the per-step timer world plugin.

Usage: python3 make_world.py <nezha_gazebo dir> <amplitude m> <period s> <out name>
"""
import pathlib
import re
import sys

gz = pathlib.Path(sys.argv[1])
amp, period, out = float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]

world = (gz / "worlds" / "nezha_lake.world").read_text(encoding="utf-8")
model = (gz / "models" / "nezha_ocean_waves" / "model.sdf").read_text(encoding="utf-8")
model = model[model.index("<model"):model.rindex("</model>") + len("</model>")]
model = re.sub(r"<amplitude>[^<]*</amplitude>", f"<amplitude>{amp}</amplitude>", model)
model = re.sub(r"<period>[^<]*</period>", f"<period>{period}</period>", model)

include = re.search(r"<include>\s*<uri>model://nezha_ocean_waves</uri>.*?</include>", world, re.S)
assert include, "nezha_ocean_waves include not found"
world = world[:include.start()] + model + world[include.end():]
timer = '<plugin name="nezha_step_timer" filename="libnezha_step_timer_plugin.so"/>\n'
world = world.replace("</world>", timer + "  </world>", 1)
(gz / "worlds" / out).write_text(world, encoding="utf-8")
print("wrote", gz / "worlds" / out)
