from isaacsim import SimulationApp
sim = SimulationApp({'headless': True})
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")
import omni.graph.core as og
og.Controller.edit(
    {"graph_path": "/ROS2Publishers", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnTick"),
            ("PublishForce", "omni.isaac.ros2_bridge.ROS2PublishFloat64"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "PublishForce.inputs:execIn"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishForce.inputs:topicName", "/contact_force/data"),
            ("PublishForce.inputs:data", 123.4),
        ],
    },
)
print("OMNIGRAPH_OK")
sim.close()
