"""Scene resets must not push the robot or objects before control is enabled."""
import numpy as np
import pytest

from spd_envs import get_task
from spd_vr.viewer import PlantController


@pytest.mark.parametrize("task", [
    "dishes/rack_dishes", "mugs/hang_mug", "jenga/playing",
    "cups/pyramid", "bottles/toss_in_bin", "spelling_blocks/spelling",
])
def test_task_reset_is_clear_of_home_robot_and_settles(task):
    result = get_task(task).reset(0)
    plant = PlantController(command_only=True, scene_result=result)
    try:
        model, data = plant.model, plant.data
        scene_bodies = {model.body(obj.name).id for obj in result.objects}
        table = model.geom("scene_table").id
        for contact in data.contact:
            first, second = int(contact.geom1), int(contact.geom2)
            first_scene = first == table or model.geom_bodyid[first] in scene_bodies
            second_scene = second == table or model.geom_bodyid[second] in scene_bodies
            if first_scene != second_scene:
                assert contact.dist >= -1e-5, (task, first, second, contact.dist)
        initial = np.array([data.body(obj.name).xpos.copy() for obj in result.objects])
        for tick in range(480):
            plant.physics_tick(now_ns=round(tick * 1e9 / 480))
        final = np.array([data.body(obj.name).xpos.copy() for obj in result.objects])
        # Permit sub-mm contact settling, not an object being hit/tipped at reset.
        assert np.max(np.linalg.norm(final - initial, axis=1)) < .005
        assert not any(warning.number for warning in data.warning)
    finally:
        plant.close()
