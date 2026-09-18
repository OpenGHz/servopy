# Panda model assets

`panda.zip` contains the unmodified `panda.xml`, `scene.xml`, `README.md`,
`LICENSE`, and all mesh files from Google DeepMind MuJoCo Menagerie:

- Source: https://github.com/google-deepmind/mujoco_menagerie/tree/71f066ad0be9cd271f7ed58c030243ef157af9f4/franka_emika_panda
- Revision: `71f066ad0be9cd271f7ed58c030243ef157af9f4`
- License: Apache-2.0, reproduced in [PANDA-LICENSE.txt](PANDA-LICENSE.txt).
- Provenance: [panda-source.json](panda-source.json) records SHA-256 checksums
  for the archive and each original file.

The Git checkout keeps the compressed archive for offline development. It is
excluded from wheels and source distributions. Installed demos download the
fixed archive URL in `panda-source.json` on first use, verify its SHA-256, and
cache it under `${XDG_CACHE_HOME:-~/.cache}/servo-py/panda/<sha256>/panda.zip`.
For offline use, set `SERVO_PY_PANDA_ARCHIVE` to a copy of this ZIP; the same
checksum is required. Installation, importing ServoPy and `--help` do not
download the model. A cached model is reused without network access.

The example reads the verified archive directly into MuJoCo's virtual file
system without extraction. `load_panda()` in `../mujoco_panda.py` changes the scene
and arm actuators in memory and adds a TCP site; it does not rewrite upstream
files. The upstream scene is preserved for reference; the demo builds its own
scene around `panda.xml`.

The Panda assets retain their upstream license, independently of the MIT
license on servo-py code. This example is not affiliated with or endorsed by
Franka Robotics or Google DeepMind.
