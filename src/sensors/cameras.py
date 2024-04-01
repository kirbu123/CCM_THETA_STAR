import omni.graph.core as og
from omni.isaac.core.utils.prims import set_targets
from omni.isaac.kit import SimulationApp
from pxr import UsdGeom
from pxr.Usd import Stage
import omni
import omni.replicator.core as rep
from src.config import Config
import omni.isaac.core.utils.prims as prims_utils

def setup_cameras(
    cfg: Config, simulation_app: SimulationApp, stage: Stage, cameras_list: list = None
):
    if cameras_list is None:
        cameras_list = cfg.cameras.cameras_list

    graph_controller = setup_cameras_graph(cfg, simulation_app)

    for camera_name in cameras_list:

        zed_left_camera_prim = UsdGeom.Camera(
            stage.GetPrimAtPath(cfg.cameras[camera_name].stage_path)
        )
        zed_left_camera_prim.GetHorizontalApertureAttr().Set(
            cfg.cameras[camera_name].horizontal_aperture
        )
        zed_left_camera_prim.GetVerticalApertureAttr().Set(
            cfg.cameras[camera_name].vertical_aperture
        )
        zed_left_camera_prim.GetProjectionAttr().Set(
            cfg.cameras[camera_name].projection_type
        )
        zed_left_camera_prim.GetFocalLengthAttr().Set(
            cfg.cameras[camera_name].focal_length
        )
        simulation_app.update()

        setup_camera_graph(cfg, simulation_app, stage, graph_controller, camera_name)


def setup_cameras_graph(cfg: Config, simulation_app: SimulationApp) -> og.Controller:
    keys = og.Controller.Keys
    controller = og.Controller(graph_id="ros_cameras_graph")

    (graph, _, _, _) = controller.edit(
        {
            "graph_path": cfg.cameras.action_graph_stage_path,
            "evaluator_name": "execution",
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ]
        },
    )

    simulation_app.update()
    return controller


def setup_camera_graph(
    cfg: Config,
    simulation_app: SimulationApp,
    stage: Stage,
    controller: og.Controller,
    camera_name: str,
):
    """Setup the action graph for publishing Images, Depths and CameraInfo to ROS1/2"""

    keys = og.Controller.Keys
    graph = og.get_graph_by_path(cfg.cameras.action_graph_stage_path)

    ros_bridge = cfg.ros_cfg[cfg.ros].ros_bridge_extension.split("-")[0]
    ros_v = cfg.ros_cfg[cfg.ros].ros_v

    controller.edit(
        graph,
        {
            keys.CREATE_NODES: [
                (
                    f"create_{camera_name}Viewport",
                    "omni.isaac.core_nodes.IsaacCreateViewport",
                ),
                (
                    f"get_{camera_name}RenderProduct",
                    "omni.isaac.core_nodes.IsaacGetViewportRenderProduct",
                ),
                (
                    f"set_{camera_name}Camera",
                    "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct",
                ),
                (
                    f"camera_{camera_name}HelperRgb",
                    f"{ros_bridge}.ROS{ros_v}CameraHelper",
                ),
                (
                    f"camera_{camera_name}HelperRgbInfo",
                    f"{ros_bridge}.ROS{ros_v}CameraHelper",
                ),
                (
                    f"camera_{camera_name}HelperDepthInfo",
                    f"{ros_bridge}.ROS{ros_v}CameraHelper",
                ),
                (
                    f"camera_{camera_name}HelperDepth",
                    f"{ros_bridge}.ROS{ros_v}CameraHelper",
                ),
                (
                    f"camera_{camera_name}HelperSemanticSegmentation",
                    f"{ros_bridge}.ROS{ros_v}CameraHelper",
                ),
                (
                    f"camera_{camera_name}HelperInstanceSegmentation",
                    f"{ros_bridge}.ROS{ros_v}CameraHelper",
                ),
                (
                    f"camera_{camera_name}HelperBBox2DTight",
                    f"{ros_bridge}.ROS{ros_v}CameraHelper",
                ),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", f"create_{camera_name}Viewport.inputs:execIn"),
                (
                    f"create_{camera_name}Viewport.outputs:execOut",
                    f"get_{camera_name}RenderProduct.inputs:execIn",
                ),
                (
                    f"create_{camera_name}Viewport.outputs:viewport",
                    f"get_{camera_name}RenderProduct.inputs:viewport",
                ),
                (
                    f"get_{camera_name}RenderProduct.outputs:execOut",
                    f"set_{camera_name}Camera.inputs:execIn",
                ),
                (
                    f"get_{camera_name}RenderProduct.outputs:renderProductPath",
                    f"set_{camera_name}Camera.inputs:renderProductPath",
                ),
                (
                    f"set_{camera_name}Camera.outputs:execOut",
                    f"camera_{camera_name}HelperRgb.inputs:execIn",
                ),
                (
                    f"set_{camera_name}Camera.outputs:execOut",
                    f"camera_{camera_name}HelperRgbInfo.inputs:execIn",
                ),
                (
                    f"set_{camera_name}Camera.outputs:execOut",
                    f"camera_{camera_name}HelperDepthInfo.inputs:execIn",
                ),
                (
                    f"set_{camera_name}Camera.outputs:execOut",
                    f"camera_{camera_name}HelperDepth.inputs:execIn",
                ),
                (
                    f"set_{camera_name}Camera.outputs:execOut",
                    f"camera_{camera_name}HelperSemanticSegmentation.inputs:execIn",
                ),
                (
                    f"set_{camera_name}Camera.outputs:execOut",
                    f"camera_{camera_name}HelperInstanceSegmentation.inputs:execIn",
                ),
                (
                    f"set_{camera_name}Camera.outputs:execOut",
                    f"camera_{camera_name}HelperBBox2DTight.inputs:execIn",
                ),
                (
                    f"get_{camera_name}RenderProduct.outputs:renderProductPath",
                    f"camera_{camera_name}HelperRgb.inputs:renderProductPath",
                ),
                (
                    f"get_{camera_name}RenderProduct.outputs:renderProductPath",
                    f"camera_{camera_name}HelperRgbInfo.inputs:renderProductPath",
                ),
                (
                    f"get_{camera_name}RenderProduct.outputs:renderProductPath",
                    f"camera_{camera_name}HelperDepthInfo.inputs:renderProductPath",
                ),
                (
                    f"get_{camera_name}RenderProduct.outputs:renderProductPath",
                    f"camera_{camera_name}HelperDepth.inputs:renderProductPath",
                ),
                (
                    f"get_{camera_name}RenderProduct.outputs:renderProductPath",
                    f"camera_{camera_name}HelperSemanticSegmentation.inputs:renderProductPath",
                ),
                (
                    f"get_{camera_name}RenderProduct.outputs:renderProductPath",
                    f"camera_{camera_name}HelperInstanceSegmentation.inputs:renderProductPath",
                ),
                (
                    f"get_{camera_name}RenderProduct.outputs:renderProductPath",
                    f"camera_{camera_name}HelperBBox2DTight.inputs:renderProductPath",
                ),
            ],
            keys.SET_VALUES: [
                (
                    f"create_{camera_name}Viewport.inputs:viewportId",
                    cfg.cameras.cameras_list.index(camera_name),
                ),
                (f"camera_{camera_name}HelperRgb.inputs:frameId", f"{cfg.cameras[camera_name].frame_id}"),
                (
                    f"camera_{camera_name}HelperRgb.inputs:topicName",
                    f"{cfg.cameras[camera_name].topic_name_color}/image_raw",
                ),
                (f"camera_{camera_name}HelperRgb.inputs:type", "rgb"),
                # (f"camera_{camera_name}HelperRgb.inputs:nodeNamespace",
                #  f"/sensor/camera/cam_{cfg.cameras.cameras_list.index(camera_name)}/color"),
                (f"camera_{camera_name}HelperRgbInfo.inputs:frameId", f"{cfg.cameras[camera_name].frame_id}"),
                (
                    f"camera_{camera_name}HelperRgbInfo.inputs:topicName",
                    f"{cfg.cameras[camera_name].topic_name_color_camera_info}",
                ),
                (f"camera_{camera_name}HelperRgbInfo.inputs:type", "camera_info"),

                (f"camera_{camera_name}HelperDepthInfo.inputs:frameId", f"{cfg.cameras[camera_name].frame_id}"),
                (
                    f"camera_{camera_name}HelperDepthInfo.inputs:topicName",
                    f"{cfg.cameras[camera_name].topic_name_depth_camera_info}",
                ),
                (f"camera_{camera_name}HelperDepthInfo.inputs:type", "camera_info"),
                # (f"camera_{camera_name}HelperInfo.inputs:nodeNamespace",
                #  f"/sensor/camera/cam_{cfg.cameras.cameras_list.index(camera_name)}/color"),
                (f"camera_{camera_name}HelperDepth.inputs:frameId", f"{cfg.cameras[camera_name].frame_id}"),
                (
                    f"camera_{camera_name}HelperDepth.inputs:topicName",
                    f"{cfg.cameras[camera_name].topic_name_depth}",
                ),
                (f"camera_{camera_name}HelperDepth.inputs:type", "depth"),
                # (f"camera_{camera_name}HelperDepth.inputs:nodeNamespace",
                #  f"/sensor/camera/cam_{cfg.cameras.cameras_list.index(camera_name)}/depth")
                (
                    f"camera_{camera_name}HelperInstanceSegmentation.inputs:frameId",
                    f"{cfg.cameras[camera_name].frame_id}",
                ),
                (
                    f"camera_{camera_name}HelperInstanceSegmentation.inputs:topicName",
                    f"{cfg.cameras[camera_name].topic_name_color}/instance_segmentation",
                ),
                (
                    f"camera_{camera_name}HelperInstanceSegmentation.inputs:type",
                    "instance_segmentation",
                ),
                (
                    f"camera_{camera_name}HelperInstanceSegmentation.inputs:enableSemanticLabels",
                    True,
                ),
                (
                    f"camera_{camera_name}HelperInstanceSegmentation.inputs:semanticLabelsTopicName",
                    f"{cfg.cameras[camera_name].topic_name_color}/instance_label",
                ),
                (
                    f"camera_{camera_name}HelperSemanticSegmentation.inputs:frameId",
                    f"{cfg.cameras[camera_name].frame_id}",
                ),
                (
                    f"camera_{camera_name}HelperSemanticSegmentation.inputs:topicName",
                    f"{cfg.cameras[camera_name].topic_name_color}/semantic_segmentation",
                ),
                (
                    f"camera_{camera_name}HelperSemanticSegmentation.inputs:type",
                    "semantic_segmentation",
                ),
                (
                    f"camera_{camera_name}HelperSemanticSegmentation.inputs:enableSemanticLabels",
                    True,
                ),
                (
                    f"camera_{camera_name}HelperSemanticSegmentation.inputs:semanticLabelsTopicName",
                    f"{cfg.cameras[camera_name].topic_name_color}/semantic_label",
                ),
                (
                    f"camera_{camera_name}HelperBBox2DTight.inputs:frameId",
                    f"{cfg.cameras[camera_name].frame_id}",
                ),
                (
                    f"camera_{camera_name}HelperBBox2DTight.inputs:topicName",
                    f"{cfg.cameras[camera_name].topic_name_color}/bbox_2d_tight",
                ),
                (f"camera_{camera_name}HelperBBox2DTight.inputs:type", "bbox_2d_tight"),
                (
                    f"camera_{camera_name}HelperBBox2DTight.inputs:enableSemanticLabels",
                    True,
                ),
                (
                    f"camera_{camera_name}HelperBBox2DTight.inputs:semanticLabelsTopicName",
                    f"{cfg.cameras[camera_name].topic_name_color}/bbox_2d_tight_label",
                ),
            ],
        },
    )

    set_targets(
        prim=stage.GetPrimAtPath(
            cfg.cameras.action_graph_stage_path + f"/set_{camera_name}Camera"
        ),
        attribute="inputs:cameraPrim",
        target_prim_paths=[cfg.cameras[camera_name].stage_path],
    )

    simulation_app.update()

    controller.evaluate_sync(graph)