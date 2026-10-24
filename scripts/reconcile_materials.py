"""Local-only import of supplied archives; no downloads, source edits, or publishing.

Run after extracting into materials/ as described in docs/ASSIGNMENT_INVENTORY.md.
Hashes retain traceability; only selected small source files are versioned.
"""
from pathlib import Path
import ast
import hashlib
import json
import shutil

root=Path(__file__).resolve().parents[1]
materials=root/'materials'
destination=root/'course_sources'
selected={
 'hw1':('archive/HW1/codes',['phase_portrait_p5c.py','5c.py']),
 'hw2':('archive/HW2',['quadrotor.py','grid_world.py','value_iteration.py']),
 'hw4_template':('hw4',['minsnap.py','pos_constraints.py']),
 'hw4_completed':('archive/HW4/code',['minsnap.py','pos_constraints.py']),
 'hw5':('hw5',['iLQR.py','find_swingup_trajectory.py','dynamics_constraints.py','kinematic_constraints.py','acrobot.urdf','quad_visualizer.py']),
 'hw6':('hw6',['quadrotor.py','quad_sim.py','osc.py','osc_tracking_objective.py','footstep_planner.py','fsm_utils.py','planar_walker.urdf','create_animation.py'])}
for label,(directory,names) in selected.items():
    folder=destination/label;folder.mkdir(parents=True,exist_ok=True)
    for name in names:
        original=materials/directory/name
        if not original.exists(): raise FileNotFoundError(original)
        target=folder/name
        if target.exists() and target.read_bytes()!=original.read_bytes():raise RuntimeError(f'Refusing to overwrite {target}')
        shutil.copyfile(original,target)
manifest=json.loads((materials/'manifest.json').read_text())
(root/'docs/materials_manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=True))
duplicates=[]
folder=materials/'archive/HW3/Homework3-MEAM-5170-Fall-2025-Duplicate'
for source in folder.glob('*.py'):
    current=root/source.name
    if current.exists():
        duplicates.append(dict(file=source.name,identical_bytes=current.read_bytes()==source.read_bytes(),
            identical_AST=ast.dump(ast.parse(current.read_text(encoding='utf-8-sig')))==ast.dump(ast.parse(source.read_text(encoding='utf-8-sig')))))
(root/'docs/duplicate_comparison.json').write_text(json.dumps(duplicates,indent=2))
print(json.dumps(duplicates,indent=2))
