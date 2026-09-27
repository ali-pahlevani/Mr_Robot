# Third-party packages

Packages from other projects, built from source with the rest of the
workspace. Each keeps its own license (see the `LICENSE` file in its folder).

| Package | Origin | License | Changes made here |
|---|---|---|---|
| `openarmx_description` | OpenArmX arms and hands, from [OpenFleX](https://github.com/OpenFleX-Wheeled-Humanoid) ([openarmx_description](https://github.com/openarmx-arm/openarmx_description)), branch `v1.0_basic` | CC BY-NC-SA 4.0 | Hand and finger inertias replaced by those of boxes the size of their collision meshes, fingers 0.10 kg (`config/hand/openarmx_hand/inertials.yaml`); finger travel 0.070 m instead of 0.044 m and finger effort 60 N instead of 333 N (`urdf/ee/openarmx_hand.xacro`) |
| `openarmx_head_description` | OpenArmX two-axis head, from OpenFleX, branch `v1.0_basic` | CC BY-NC-SA 4.0 | None |
| `lift_slide_description` | OpenFleX lift column, branch `v1.0_basic` | CC BY-NC-SA 4.0 | An inertial on the chest cover, so the Webots importer keeps the link (`urdf/lift_slide_module.urdf.xacro`) |
| `pymoveit2` | [AndrejOrsula/pymoveit2](https://github.com/AndrejOrsula/pymoveit2) 4.2.0 | BSD-3-Clause | None |

Every change is marked in its file with a `mrRobot:` comment that explains why
it was needed. In short: with the original inertias and finger force the
physics engine threw the arm apart when a finger met the microwave door, and
with the original finger travel the hand could not open wide enough for the
90 mm jars.

The OpenFleX packages are licensed for non-commercial use only (CC BY-NC-SA
4.0). That also applies to anything that redistributes their meshes or URDFs,
including the Webots PROTO that `mrrobot_webots` generates from them.
