"""
Quick viewer for the KILTRO URDF.

python view_in_mujoco.py          # static pose, nothing moves
python view_in_mujoco.py --drop   # free base with gravity: it collapses, see below
python view_in_mujoco.py --check  # no window: masses, inertias, joint limits

Default mode turns gravity and contacts off, so the robot floats in the zero pose
with nothing to disturb it. Hit Space to pause and the sliders in the right-hand panel
("Joint") let you drag each joint to its limits.
"""
import os, sys, tempfile
import mujoco, numpy as np

HERE  = os.path.dirname(os.path.abspath(__file__))
DROP  = '--drop'  in sys.argv
SWEEP = '--sweep' in sys.argv
CHECK = '--check' in sys.argv

URDF_PATH = next((p for p in (os.path.join(HERE, 'urdf', 'kiltro.urdf'),
                              os.path.join(HERE, 'kiltro.urdf'),
                              os.path.join(HERE, '..', 'urdf', 'kiltro.urdf'))
                  if os.path.isfile(p)), None)
MESH_PATH = next((p for p in (os.path.join(HERE, 'meshes'),
                              os.path.join(HERE, '..', 'meshes'))
                  if os.path.isdir(p)), None)
if URDF_PATH is None or MESH_PATH is None:
    sys.exit(f'Could not find kiltro.urdf or the meshes/ folder from {HERE}.\n'
             'Keep this script inside kiltro/, next to urdf/ and meshes/.')
print(f'urdf:   {os.path.normpath(URDF_PATH)}')
print(f'meshes: {os.path.normpath(MESH_PATH)}')

urdf = open(URDF_PATH, encoding='utf-8').read()
urdf = urdf.replace('package://kiltro/meshes/', '')
header = f'''<robot name="kiltro">
  <mujoco><compiler meshdir="{os.path.abspath(MESH_PATH)}" discardvisual="false"/></mujoco>'''

# Always give the base a free joint: MuJoCo merges a pinned base into the world body,
# which drops the trunk mass and its inertia box from the model.
if True:
    header += '''
  <link name="world_root"/>
  <joint name="base_free" type="floating">
    <parent link="world_root"/><child link="base_link"/><origin xyz="0 0 0.55"/>
  </joint>'''
if DROP:
    header += '''
  <link name="ground">
    <collision><origin xyz="0 0 -0.55"/><geometry><box size="20 20 0.1"/></geometry></collision>
    <visual><origin xyz="0 0 -0.55"/><geometry><box size="20 20 0.1"/></geometry>
      <material name="ground"><color rgba="0.25 0.3 0.35 1"/></material></visual>
  </link>
  <joint name="ground_fixed" type="fixed"><parent link="world_root"/><child link="ground"/></joint>'''
urdf = urdf.replace('<robot name="kiltro">', header)

tmp = os.path.join(tempfile.gettempdir(), 'kiltro_tmp.urdf')
open(tmp, 'w', encoding='utf-8').write(urdf)
m = mujoco.MjModel.from_xml_path(tmp)
d = mujoco.MjData(m)

# MuJoCo imports everything into group 0. Split meshes (2) from collision shapes (3)
# so the 2 and 3 keys toggle them in the viewer.
for g in range(m.ngeom):
    m.geom_group[g] = 2 if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH else 3
if not DROP:
    m.opt.gravity[:] = 0
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT

print(f'loaded: {m.nbody-1} bodies, {m.njnt} joints, {m.ngeom} geoms, '
      f'{m.body_mass.sum():.3f} kg')

if CHECK:
    mujoco.mj_forward(m, d)
    print(f'\n{"joint":18s} {"axis":>5s} {"limits (rad)":>18s} {"deg":>16s}')
    for i in range(m.njnt):
        if m.jnt_type[i] != mujoco.mjtJoint.mjJNT_HINGE: continue
        lo, hi = m.jnt_range[i]
        print(f'{mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i):18s} '
              f'{"XYZ"[int(np.argmax(np.abs(m.jnt_axis[i])))]:>5s} '
              f'{lo:8.3f} {hi:8.3f} {np.degrees(lo):8.1f} {np.degrees(hi):7.1f}')
    print(f'\n{"body":14s} {"mass (kg)":>10s}   principal inertias (kg m2)')
    for i in range(1, m.nbody):
        if m.body_mass[i] == 0: continue
        print(f'{mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i):14s} '
              f'{m.body_mass[i]:10.4f}   {np.round(m.body_inertia[i], 6)}')
    sys.exit()

import mujoco.viewer

if SWEEP:
    hinges = [i for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE]
    print('\nSweeping each joint through its range, 3 s each. Ctrl-C to stop.')
    with mujoco.viewer.launch_passive(m, d) as v:
        import time
        while v.is_running():
            for i in hinges:
                lo, hi = m.jnt_range[i]
                adr = m.jnt_qposadr[i]
                name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i)
                print(f'  {name:18s} {lo:+.3f} -> {hi:+.3f} rad')
                t0 = time.time()
                while time.time() - t0 < 3.0 and v.is_running():
                    # triangular ramp: lower limit -> upper limit -> lower limit
                    u = (time.time() - t0) / 3.0
                    s_ = 2 * u if u < 0.5 else 2 * (1 - u)
                    d.qpos[adr] = lo + (hi - lo) * s_
                    mujoco.mj_forward(m, d)
                    v.sync()
                    time.sleep(1 / 50)
                d.qpos[adr] = 0.0
    sys.exit()

print('\n' + ('drop mode: no actuators, so the robot folds up. Expected.'
              if DROP else
              'Gravity and contacts off. Hit Space to pause before using the\n'
              'joint sliders (right panel, Shift+Tab). Key 3 shows the collision shapes.'))
mujoco.viewer.launch(m, d)
