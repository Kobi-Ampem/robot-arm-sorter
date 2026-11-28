import mujoco
import numpy as np

from arm_sorter import scene as sc
from arm_sorter.perception import CLEAR_POSE, Camera, detect
from arm_sorter.viz import FrameRecorder, draw_detections


def test_recorder_keeps_every_nth_frame():
    model = sc.build(sc.random_scene(0))
    data = mujoco.MjData(model)
    recorder = FrameRecorder(model, width=64, height=48, every=10)
    for _ in range(25):
        mujoco.mj_step(model, data)
        recorder(data)
    recorder.close()
    assert len(recorder.frames) == 3 and recorder.frames[0].shape == (48, 64, 3)


def test_draw_detections_marks_the_image():
    model = sc.build(sc.random_scene(0))
    data = mujoco.MjData(model)
    data.qpos[:4] = CLEAR_POSE
    mujoco.mj_forward(model, data)
    camera = Camera(model)
    rgb = camera.render(data)
    found = detect(camera, data, rgb=rgb)
    camera.close()
    marked = draw_detections(rgb, found)
    assert marked.shape == rgb.shape and np.any(marked != rgb)
