from arm_sorter.scene import BINS
from arm_sorter.trials import run_episode, where


def test_where_classifies_final_positions():
    bx, by = BINS["ripe"]
    assert where((bx, by, 0.03), "ripe") == "correct"
    assert where((bx, by, 0.03), "green") == "wrong_bin"
    assert where((0.25, 0.0, 0.03), "ripe") == "on_tray"
    assert where((0.9, 0.9, -0.5), "ripe") == "elsewhere"


def test_an_episode_is_scored():
    ep = run_episode(0)
    assert ep.correct + ep.wrong_bin + ep.on_tray + ep.elsewhere == ep.tomatoes
    assert ep.success and ep.picks >= ep.tomatoes and ep.sim_time > 0
