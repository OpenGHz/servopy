# Panda model assets

`panda.zip` contains the unmodified `panda.xml`, `scene.xml`, `README.md`,
`LICENSE`, and all mesh files from Google DeepMind MuJoCo Menagerie:

- Source: https://github.com/google-deepmind/mujoco_menagerie/tree/71f066ad0be9cd271f7ed58c030243ef157af9f4/franka_emika_panda
- Revision: `71f066ad0be9cd271f7ed58c030243ef157af9f4`
- License: Apache-2.0, reproduced in [PANDA-LICENSE.txt](PANDA-LICENSE.txt).
- Provenance: [panda-source.json](panda-source.json) records SHA-256 checksums
  for the archive and each original file.

The compressed archive keeps the original mesh assets available offline. The
example reads them directly into MuJoCo's virtual file system, without network
access or extraction. `load_panda()` in `../mujoco_panda.py` changes the scene
and arm actuators in memory and adds a TCP site; it does not rewrite upstream
files. The upstream scene is preserved for reference; the demo builds its own
scene around `panda.xml`.

The Panda assets retain their upstream license, independently of the MIT
license on servo-py code. This example is not affiliated with or endorsed by
Franka Robotics or Google DeepMind.
