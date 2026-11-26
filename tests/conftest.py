import os

# Render without a window. CI sets MUJOCO_GL=osmesa itself (no GPU on the runner).
os.environ.setdefault("MUJOCO_GL", "egl")
