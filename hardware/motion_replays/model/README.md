# Panda model used for kinematic replay

`panda_nohand.xml` is the project model copied without modification from
`mjcf/franka_emika_panda/panda_nohand.xml`. It follows the
[MuJoCo Menagerie Franka Emika Panda description](https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda),
which derives from the Franka URDF. The existing model package's
[Apache-2.0 license](LICENSE) is included unchanged. The project's
`panda_link8_site` preserves the end-frame convention used by the experiment.

The 59 referenced mesh files are intentionally external (approximately 32 MB).
Obtain the `franka_emika_panda/assets` folder from the model's
[upstream package](https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda/assets).
For example, a sparse Git checkout downloads the Panda package:

```bash
git clone --depth 1 --filter=blob:none --sparse https://github.com/google-deepmind/mujoco_menagerie.git
git -C mujoco_menagerie sparse-checkout set franka_emika_panda
```

From the data release root:

```bash
python processing/render_real_robot_replays.py --model hardware/motion_replays/model/panda_nohand.xml --assets mujoco_menagerie/franka_emika_panda/assets
```

The renderer checks each referenced file against [mesh_manifest.json](mesh_manifest.json),
which records the exact mesh bytes used for the released movies. The manifest
also records the XML hash. If the upstream assets have changed, use matching
archived assets; do not replace the hash checks with an unverified model.
Alternatively, place the matching `assets/` directory beside the bundled XML
and omit `--assets`.

The XML is used only for forward kinematics and appearance. The renderer sets
the seven joint positions from measured data and calls `mj_forward`; it never
integrates dynamics or infers the trial's payload.
