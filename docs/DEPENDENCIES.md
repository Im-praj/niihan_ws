# Dependencies and reproducible source setup

Tested platform: Ubuntu 22.04 amd64, ROS Humble and Gazebo Harmonic. Ubuntu 24.04/Jazzy is a separate build target whose simulation acceptance is pending.

```bash
sudo apt-get update
sudo apt-get install git git-lfs
git lfs install
git clone --recurse-submodules --branch feat/l2-3d-slam-release https://github.com/Im-praj/niihan_ws.git
cd niihan_ws
git lfs pull
git submodule sync --recursive
git submodule update --init --recursive
./setup.sh --source
./run.sh
```

Setup installs apt dependencies, compiles the CPU libraries, builds the supported source selection and validates it. Run as your normal user; setup uses sudo only for apt. Default binary setup (`./setup.sh`) requires the release for the exact source commit; use `--source` until that release is available. Logs/build outputs and runtime maps are generated locally.

## Pinned source submodules

Git records these exact commits. Public upstream URLs allow recursive cloning without upstream write access. KISS-ICP is optional and is not the active SLAM estimator.

| Library | Source | Commit |
|---|---|---|
| geometry2 | https://github.com/ros2/geometry2.git | `404b7224d623d614f18fa9738dbf1716403d857e` |
| glim | https://github.com/koide3/glim.git | `2262aafa2369acff6afab2268a7743aec7a7fa49` |
| glim_ros2 | https://github.com/koide3/glim_ros2.git | `4d4ec524ccf1b02aa09b0af2af767ecc54343798` |
| navigation2 | https://github.com/ros-navigation/navigation2.git | `3c3db59d6969d8ecee8e68468693d006397f4a0c` |
| kiss-icp | https://github.com/PRBonn/kiss-icp.git | `85488b7b6a6e1566cff0f8af4b83a9ea77dca7a0` |

Local edits to upstream source are preserved as patches in the parent repository rather than unpublished submodule commits. `scripts/apply_nav2_patch.sh` idempotently applies `patches/nav2-humble.patch` during the Humble build. Jazzy uses native Nav2 and geometry2 apt binaries. The optional GLIM desktop viewer is disabled in the CPU/browser profile; its compatibility patches remain available:

```bash
git -C src/glim apply ../../patches/glim-viewer.patch
git -C src/glim_ros2 apply ../../patches/glim-ros-viewer.patch
```

Do not apply patches twice; use `git apply --check` and `git apply --reverse --check` to inspect their state. Existing dirty submodules in the developer workspace correspond to these preserved patches.

## Additional compiled dependencies

`scripts/build_dependencies.sh` automatically clones and builds the following into the private `.deps` prefix. For manual source inspection:

```bash
git clone https://github.com/borglab/gtsam.git gtsam
git -C gtsam checkout --detach 3ad4b4c3cb28394c9597f48fa02dad361c8450e3
git clone https://github.com/koide3/gtsam_points.git gtsam_points
git -C gtsam_points checkout --detach 9d32e7dbecf6015560d84b4901d6b0a6f483ec46
```

These are GTSAM 4.3a0 and gtsam_points 1.2.2. CPU-only build commands are in the script. CUDA and Iridescence are not required. ROS, Gazebo, Eigen, Boost, OpenCV, OpenMP, fmt, spdlog and METIS are installed via configured apt repositories. Python requires numpy, scipy, PyYAML, aiohttp, websockets, pyserial, OpenCV, psutil and Pillow; validation uses pytest and Node.js. Setup installs these through apt without system pip changes.

## Browser code and assets

Three.js and OrbitControls are vendored with license notices; no CDN fetch is required. The seaport observer has a frozen dashboard snapshot for experiment reproduction.

Portable/seaport worlds are procedural geometry. The detailed `construction_site_m1.glb` is stored in Git LFS because it exceeds GitHub's regular 100 MB limit. Install Git LFS and run `git lfs pull` before building. Companion `construction_site_chunk5` collision meshes and their original license are included. The owner explicitly authorized publishing these supplied assets on 21 October 2026. That authorization does not replace or broaden the supplied proprietary license; downstream users must respect it. No separate public asset-source URL was supplied. Older documentation describing the meshes as local-only reflects the earlier publication state.
