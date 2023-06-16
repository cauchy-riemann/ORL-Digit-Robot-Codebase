import sys
from typing import Optional, List, Any
from .messages import *


# Enum definitions

class EndEffector:
    LEFT_FOOT='left-foot'
    LEFT_HAND='left-hand'
    RIGHT_FOOT='right-foot'
    RIGHT_HAND='right-hand'

class RobotFrame:
    """Not a comprehensive list, additional sensor names are also allowed."""
    BASE='base'
    LEFT_HAND='left-hand'
    RIGHT_HAND='right-hand'
    LEFT_FOOT='left-foot'
    RIGHT_FOOT='right-foot'
    CENTER_OF_MASS='center-of-mass'
    MEAN_HAND='mean-hand'
    MEAN_FOOT='mean-foot'
    IMU='imu'
    GPS='gps'

class ObjectAttribute:
    NAME='name'
    PARENT='parent'
    CHILDREN='children'
    POSE='pose'
    VELOCITY='velocity'
    TIMEOUT='timeout'
    MASS='mass'
    COLOR='color'
    OPACITY='opacity'
    POLYGON='polygon'
    APRIL_TAG_ID='april-tag-id'
    KEEP_OUT='keep-out'
    STEPPABLE='steppable'
    BOX_GEOMETRY='box-geometry'
    ELLIPSOID_GEOMETRY='ellipsoid-geometry'
    PICKABLE='pickable'

class Privilege:
    CHANGE_ACTION_COMMAND='change-action-command'

class ActionStatusDetail:
    NOT_STARTED='not-started'
    REMOVED='removed'
    RUNNING='running'
    SUCCESS='success'
    FAILURE='failure'
    BEHAVIOR_BUG='behavior-bug'
    INFINITE_LOOP='infinite-loop'
    LOOP_COUNT='loop-count'
    INVALID_BOX='invalid-box'
    NODE_ALLOCATION_FAILED='node-allocation-failed'
    TIMEOUT_EXCEEDED='timeout-exceeded'
    DURATION_REACHED='duration-reached'
    DURATION_NOT_REACHED='duration-not-reached'
    IDLE='idle'
    INCOMPATIBLE_ACTIONS='incompatible-actions'
    NOT_HOLDING_BOX='not-holding-box'
    ROBOT_FELL='robot-fell'
    DROPPED_BOX='dropped-box'
    DRAG_FAILED='drag-failed'
    CAMERA_ALIGNED='camera-aligned'
    CAMERA_ALIGNMENT_FAILED='camera-alignment-failed'
    NO_ALIGNMENT_POSE_FOUND='no-alignment-pose-found'
    OBJECT_FOUND='object-found'
    OBJECT_NOT_FOUND='object-not-found'
    NO_VALID_TRAJECTORY='no-valid-trajectory'
    PICK_BEHAVIOR_SUCCESS='pick-behavior-success'
    PICK_BEHAVIOR_RUN='pick-behavior-run'
    PICK_BEHAVIOR_FAIL='pick-behavior-fail'
    FIND_OBJECT_COMPLETE='find-object-complete'
    FIND_OBJECT_FAIL='find-object-fail'
    GOTO_OBJECT_COMPLETE='goto-object-complete'
    GOTO_OBJECT_FAIL='goto-object-fail'
    MOVE_ARMS_COMPLETE='move-arms-complete'
    MOVE_ARMS_FAIL='move-arms-fail'
    LOAD_BOX_COMPLETE='load-box-complete'
    LOAD_BOX_FAIL='load-box-fail'
    HOLD_STAND_COMPLETE='hold-stand-complete'
    HOLD_STAND_FAIL='hold-stand-fail'
    PLACE_BEHAVIOR_SUCCESS='place-behavior-success'
    PLACE_BEHAVIOR_RUN='place-behavior-run'
    PLACE_BEHAVIOR_FAIL='place-behavior-fail'
    GOTO_PLACE_COMPLETE='goto-place-complete'
    GOTO_PLACE_FAIL='goto-place-fail'
    MOVE_ARMS_TO_PLACE_COMPLETE='move-arms-to-place-complete'
    MOVE_ARMS_TO_PLACE_FAIL='move-arms-to-place-fail'
    UNLOAD_BOX_COMPLETE='unload-box-complete'
    UNLOAD_BOX_FAIL='unload-box-fail'
    WAIT_FOR_STAND_COMPLETE='wait-for-stand-complete'
    WAIT_FOR_STAND_FAIL='wait-for-stand-fail'
    PLAN_STANCE_COMPLETE='plan-stance-complete'
    PLAN_STANCE_ACHIEVED='plan-stance-achieved'
    PLAN_STANCE_FAIL='plan-stance-fail'
    ASSISTED_STARTING='assisted-starting'
    ROPE_STARTING='rope-starting'
    UNDOCKING='undocking'
    PUSHING_UP='pushing-up'

class ObjectPriorityOrder:
    OLDEST='oldest'
    NEWEST='newest'
    LEAST_RECENTLY_UPDATED='least-recently-updated'
    MOST_RECENTLY_UPDATED='most-recently-updated'

class OperationMode:
    DISABLED='disabled'
    DAMPING='damping'
    LOCOMOTION='locomotion'
    JOINT='joint'
    LOW_LEVEL_API='low-level-api'

class LocomotionType:
    GROUNDED_RUN='grounded-run'
    WALK='walk'
    RUN='run'

class RecoveryMode:
    AUTO='auto'
    STEP='step'
    FALL='fall'
    DAMPING='damping'

class PoseRepresentation:
    RPYXYZ='rpyxyz'
    QXYZ='qxyz'

class StartMode:
    AUTO='auto'
    ASSISTED='assisted'
    ROPE='rope'
    DEPLOYMENT='deployment'
    PUSH_UP='push-up'

class SitMode:
    AUTO='auto'
    FORWARD='forward'
    BACKWARD='backward'

class ActionStatus:
    RUNNING='running'
    SUCCESS='success'
    FAILURE='failure'
    INACTIVE='inactive'

class ReplaceActionFallback:
    ERROR='error'
    SEQUENTIAL='sequential'
    CONCURRENT='concurrent'

class DisturbanceFrame:
    WORLD='world'
    BODY='body'
    COM='com'
    COM_WORLD='com-world'

class FlowControl:
    NONE='none'
    FRAMERATE='framerate'
    REQUEST='request'

class SimulationMode:
    KINEMATIC='kinematic'
    DYNAMIC='dynamic'



# Struct definitions

class Pose(Struct):
    rpy: List[float]
    a: float
    xyz: List[float]
    xy: List[float]
    rpyxyz: List[float]
    rpyxy: List[float]
    axyz: List[float]
    axy: List[float]
    rpyd: List[float]
    rpyxyzd: List[float]
    rpyxyd: List[float]
    axyzd: List[float]
    axyd: List[float]
    ad: float
    q: List[float]
    qxyz: List[float]
    qxy: List[float]
    m34: List[float]
    def __init__(
            self,
            rpy: List[float] = OMIT,
            a: float = OMIT,
            xyz: List[float] = OMIT,
            xy: List[float] = OMIT,
            rpyxyz: List[float] = OMIT,
            rpyxy: List[float] = OMIT,
            axyz: List[float] = OMIT,
            axy: List[float] = OMIT,
            rpyd: List[float] = OMIT,
            rpyxyzd: List[float] = OMIT,
            rpyxyd: List[float] = OMIT,
            axyzd: List[float] = OMIT,
            axyd: List[float] = OMIT,
            ad: float = OMIT,
            q: List[float] = OMIT,
            qxyz: List[float] = OMIT,
            qxy: List[float] = OMIT,
            m34: List[float] = OMIT,
    ):
        super().__init__(
            rpy=rpy,
            a=a,
            xyz=xyz,
            xy=xy,
            rpyxyz=rpyxyz,
            rpyxy=rpyxy,
            axyz=axyz,
            axy=axy,
            rpyd=rpyd,
            rpyxyzd=rpyxyzd,
            rpyxyd=rpyxyd,
            axyzd=axyzd,
            axyd=axyd,
            ad=ad,
            q=q,
            qxyz=qxyz,
            qxy=qxy,
            m34=m34,
        )

class GpsPose(Struct):
    rpy: List[float]
    a: float
    xyz: List[float]
    xy: List[float]
    rpyxyz: List[float]
    rpyxy: List[float]
    axyz: List[float]
    axy: List[float]
    rpyd: List[float]
    rpyxyzd: List[float]
    rpyxyd: List[float]
    axyzd: List[float]
    axyd: List[float]
    ad: float
    q: List[float]
    qxyz: List[float]
    qxy: List[float]
    m34: List[float]
    h: float
    ll: List[float]
    hll: List[float]
    def __init__(
            self,
            rpy: List[float] = OMIT,
            a: float = OMIT,
            xyz: List[float] = OMIT,
            xy: List[float] = OMIT,
            rpyxyz: List[float] = OMIT,
            rpyxy: List[float] = OMIT,
            axyz: List[float] = OMIT,
            axy: List[float] = OMIT,
            rpyd: List[float] = OMIT,
            rpyxyzd: List[float] = OMIT,
            rpyxyd: List[float] = OMIT,
            axyzd: List[float] = OMIT,
            axyd: List[float] = OMIT,
            ad: float = OMIT,
            q: List[float] = OMIT,
            qxyz: List[float] = OMIT,
            qxy: List[float] = OMIT,
            m34: List[float] = OMIT,
            h: float = OMIT,
            ll: List[float] = OMIT,
            hll: List[float] = OMIT,
    ):
        super().__init__(
            rpy=rpy,
            a=a,
            xyz=xyz,
            xy=xy,
            rpyxyz=rpyxyz,
            rpyxy=rpyxy,
            axyz=axyz,
            axy=axy,
            rpyd=rpyd,
            rpyxyzd=rpyxyzd,
            rpyxyd=rpyxyd,
            axyzd=axyzd,
            axyd=axyd,
            ad=ad,
            q=q,
            qxyz=qxyz,
            qxy=qxy,
            m34=m34,
            h=h,
            ll=ll,
            hll=hll,
        )

class SpatialVector(Struct):
    rpy: List[float]
    a: float
    xyz: List[float]
    xy: List[float]
    rpyxyz: List[float]
    rpyxy: List[float]
    axyz: List[float]
    axy: List[float]
    rpyd: List[float]
    rpyxyzd: List[float]
    rpyxyd: List[float]
    axyzd: List[float]
    axyd: List[float]
    ad: float
    def __init__(
            self,
            rpy: List[float] = OMIT,
            a: float = OMIT,
            xyz: List[float] = OMIT,
            xy: List[float] = OMIT,
            rpyxyz: List[float] = OMIT,
            rpyxy: List[float] = OMIT,
            axyz: List[float] = OMIT,
            axy: List[float] = OMIT,
            rpyd: List[float] = OMIT,
            rpyxyzd: List[float] = OMIT,
            rpyxyd: List[float] = OMIT,
            axyzd: List[float] = OMIT,
            axyd: List[float] = OMIT,
            ad: float = OMIT,
    ):
        super().__init__(
            rpy=rpy,
            a=a,
            xyz=xyz,
            xy=xy,
            rpyxyz=rpyxyz,
            rpyxy=rpyxy,
            axyz=axyz,
            axy=axy,
            rpyd=rpyd,
            rpyxyzd=rpyxyzd,
            rpyxyd=rpyxyd,
            axyzd=axyzd,
            axyd=axyd,
            ad=ad,
        )

class ObjectSelector(Struct):
    object_id: Optional[int]
    has_attributes: Optional[List[str]]
    name: Optional[str]
    all: Optional[bool]
    owned: Optional[bool]
    user_object: Optional[bool]
    april_tag_id: Optional[int]
    charuco_id: Optional[int]
    descendant_of: Optional['ObjectSelector']
    ancestor_of: Optional['ObjectSelector']
    include: List['ObjectSelector']
    exclude: List['ObjectSelector']
    special_frame: Optional[str]
    robot_frame: Optional[str]
    command_frame: Optional[str]
    owned_object_name: Optional[str]
    user_object_name: Optional[str]
    map_name: Optional[str]
    terrain_map_name: Optional[str]
    snapshot_stream_name: Optional[str]
    priority_order: str
    def __init__(
            self,
            object_id: Optional[int] = OMIT,
            has_attributes: Optional[List[str]] = OMIT,
            name: Optional[str] = OMIT,
            all: Optional[bool] = OMIT,
            owned: Optional[bool] = OMIT,
            user_object: Optional[bool] = OMIT,
            april_tag_id: Optional[int] = OMIT,
            charuco_id: Optional[int] = OMIT,
            descendant_of: Optional['ObjectSelector'] = OMIT,
            ancestor_of: Optional['ObjectSelector'] = OMIT,
            include: List['ObjectSelector'] = OMIT,
            exclude: List['ObjectSelector'] = OMIT,
            special_frame: Optional[str] = OMIT,
            robot_frame: Optional[str] = OMIT,
            command_frame: Optional[str] = OMIT,
            owned_object_name: Optional[str] = OMIT,
            user_object_name: Optional[str] = OMIT,
            map_name: Optional[str] = OMIT,
            terrain_map_name: Optional[str] = OMIT,
            snapshot_stream_name: Optional[str] = OMIT,
            priority_order: str = OMIT,
    ):
        super().__init__(
            object_id=object_id,
            has_attributes=has_attributes,
            name=name,
            all=all,
            owned=owned,
            user_object=user_object,
            april_tag_id=april_tag_id,
            charuco_id=charuco_id,
            descendant_of=descendant_of,
            ancestor_of=ancestor_of,
            include=include,
            exclude=exclude,
            special_frame=special_frame,
            robot_frame=robot_frame,
            command_frame=command_frame,
            owned_object_name=owned_object_name,
            user_object_name=user_object_name,
            map_name=map_name,
            terrain_map_name=terrain_map_name,
            snapshot_stream_name=snapshot_stream_name,
            priority_order=priority_order,
        )

class MobilityParameters(Struct):
    velocity_max: List[float]
    leg_length: float
    step_clearance: float
    avoid_obstacles: bool
    obstacle_threshold: float
    feet_avoid_unsteppable_regions: bool
    feet_match_terrain: bool
    heeldown: bool
    bounciness: float
    def __init__(
            self,
            velocity_max: List[float] = OMIT,
            leg_length: float = OMIT,
            step_clearance: float = OMIT,
            avoid_obstacles: bool = OMIT,
            obstacle_threshold: float = OMIT,
            feet_avoid_unsteppable_regions: bool = OMIT,
            feet_match_terrain: bool = OMIT,
            heeldown: bool = OMIT,
            bounciness: float = OMIT,
            locomotion_type: str = OMIT,
            stance_width: float = OMIT,
            recovery_mode: str = OMIT,
    ):
        super().__init__(
            velocity_max=velocity_max,
            leg_length=leg_length,
            step_clearance=step_clearance,
            avoid_obstacles=avoid_obstacles,
            obstacle_threshold=obstacle_threshold,
            feet_avoid_unsteppable_regions=feet_avoid_unsteppable_regions,
            feet_match_terrain=feet_match_terrain,
            heeldown=heeldown,
            bounciness=bounciness,
        )

class FrameInfo(Struct):
    name: str
    body_to_frame_pose: Pose
    contact_patch: Optional[List[List[float]]]
    perception_keypoint: Optional[bool]
    def __init__(
            self,
            name: str = OMIT,
            body_to_frame_pose: Pose = OMIT,
            contact_patch: Optional[List[List[float]]] = OMIT,
            perception_keypoint: Optional[bool] = OMIT,
    ):
        super().__init__(
            name=name,
            body_to_frame_pose=body_to_frame_pose,
            contact_patch=contact_patch,
            perception_keypoint=perception_keypoint,
        )

class Contact(Struct):
    model: str
    frame: 'FrameInfo'
    def __init__(
            self,
            model: str = OMIT,
            frame: 'FrameInfo' = OMIT,
    ):
        super().__init__(
            model=model,
            frame=frame,
        )

class ContactPair(Struct):
    contact_a: 'Contact'
    contact_b: Optional['Contact']
    mu: float
    def __init__(
            self,
            contact_a: 'Contact' = OMIT,
            contact_b: Optional['Contact'] = OMIT,
            mu: float = OMIT,
    ):
        super().__init__(
            contact_a=contact_a,
            contact_b=contact_b,
            mu=mu,
        )

class ForceCommand(Struct):
    force: SpatialVector
    is_in_contact: bool
    def __init__(
            self,
            force: SpatialVector = OMIT,
            is_in_contact: bool = OMIT,
    ):
        super().__init__(
            force=force,
            is_in_contact=is_in_contact,
        )

class TaskIdObjective(Struct):
    kp: List[float]
    kd: List[float]
    w: List[float]
    w_force: float
    def __init__(
            self,
            kp: List[float] = OMIT,
            kd: List[float] = OMIT,
            w: List[float] = OMIT,
            w_force: float = OMIT,
    ):
        super().__init__(
            kp=kp,
            kd=kd,
            w=w,
            w_force=w_force,
        )

class TaskIkObjective(Struct):
    w0: List[float]
    w1: List[float]
    def __init__(
            self,
            w0: List[float] = OMIT,
            w1: List[float] = OMIT,
    ):
        super().__init__(
            w0=w0,
            w1=w1,
        )

class TaskCommand(Struct):
    name: str
    pose: Pose
    velocity: Optional[SpatialVector]
    acceleration: Optional[SpatialVector]
    force_command: Optional['ForceCommand']
    id_objective: Optional['TaskIdObjective']
    ik_objective: Optional['TaskIkObjective']
    def __init__(
            self,
            name: str = OMIT,
            pose: Pose = OMIT,
            velocity: Optional[SpatialVector] = OMIT,
            acceleration: Optional[SpatialVector] = OMIT,
            force_command: Optional['ForceCommand'] = OMIT,
            id_objective: Optional['TaskIdObjective'] = OMIT,
            ik_objective: Optional['TaskIkObjective'] = OMIT,
    ):
        super().__init__(
            name=name,
            pose=pose,
            velocity=velocity,
            acceleration=acceleration,
            force_command=force_command,
            id_objective=id_objective,
            ik_objective=ik_objective,
        )

class JointIdObjective(Struct):
    kp: float
    kd: float
    w: float
    use_elmo_damping: bool
    def __init__(
            self,
            kp: float = OMIT,
            kd: float = OMIT,
            w: float = OMIT,
            use_elmo_damping: bool = OMIT,
    ):
        super().__init__(
            kp=kp,
            kd=kd,
            w=w,
            use_elmo_damping=use_elmo_damping,
        )

class JointIkObjective(Struct):
    w0: List[float]
    w1: List[float]
    def __init__(
            self,
            w0: List[float] = OMIT,
            w1: List[float] = OMIT,
    ):
        super().__init__(
            w0=w0,
            w1=w1,
        )

class ActuatorCommand(Struct):
    name: str
    position: float
    velocity: Optional[float]
    acceleration: Optional[float]
    torque: Optional[float]
    id_objective: Optional['JointIdObjective']
    ik_objective: Optional['JointIkObjective']
    def __init__(
            self,
            name: str = OMIT,
            position: float = OMIT,
            velocity: Optional[float] = OMIT,
            acceleration: Optional[float] = OMIT,
            torque: Optional[float] = OMIT,
            id_objective: Optional['JointIdObjective'] = OMIT,
            ik_objective: Optional['JointIkObjective'] = OMIT,
    ):
        super().__init__(
            name=name,
            position=position,
            velocity=velocity,
            acceleration=acceleration,
            torque=torque,
            id_objective=id_objective,
            ik_objective=ik_objective,
        )

class MomentumIdObjective(Struct):
    kp: List[float]
    kd: List[float]
    w: List[float]
    def __init__(
            self,
            kp: List[float] = OMIT,
            kd: List[float] = OMIT,
            w: List[float] = OMIT,
    ):
        super().__init__(
            kp=kp,
            kd=kd,
            w=w,
        )

class MomentumIkObjective(Struct):
    w0: List[float]
    w1: List[float]
    def __init__(
            self,
            w0: List[float] = OMIT,
            w1: List[float] = OMIT,
    ):
        super().__init__(
            w0=w0,
            w1=w1,
        )

class MomentumCommand(Struct):
    position: List[float]
    momentum: Optional[SpatialVector]
    force: Optional[SpatialVector]
    id_objective: Optional['MomentumIdObjective']
    ik_objective: Optional['MomentumIkObjective']
    def __init__(
            self,
            position: List[float] = OMIT,
            momentum: Optional[SpatialVector] = OMIT,
            force: Optional[SpatialVector] = OMIT,
            id_objective: Optional['MomentumIdObjective'] = OMIT,
            ik_objective: Optional['MomentumIkObjective'] = OMIT,
    ):
        super().__init__(
            position=position,
            momentum=momentum,
            force=force,
            id_objective=id_objective,
            ik_objective=ik_objective,
        )

class ManipulationCommand(Struct):
    id: int
    task: List['TaskCommand']
    def __init__(
            self,
            id: int = OMIT,
            task: List['TaskCommand'] = OMIT,
    ):
        super().__init__(
            id=id,
            task=task,
        )

class Command(Struct):
    momentum: 'MomentumCommand'
    task: List['TaskCommand']
    actuator: List['ActuatorCommand']
    manipulation: Optional['ManipulationCommand']
    contacts: List['ContactPair']
    update_time: float
    center_of_pressure: Optional[List[float]]
    def __init__(
            self,
            momentum: 'MomentumCommand' = OMIT,
            task: List['TaskCommand'] = OMIT,
            actuator: List['ActuatorCommand'] = OMIT,
            manipulation: Optional['ManipulationCommand'] = OMIT,
            contacts: List['ContactPair'] = OMIT,
            update_time: float = OMIT,
            center_of_pressure: Optional[List[float]] = OMIT,
    ):
        super().__init__(
            momentum=momentum,
            task=task,
            actuator=actuator,
            manipulation=manipulation,
            contacts=contacts,
            update_time=update_time,
            center_of_pressure=center_of_pressure,
        )

class SessionIdentifier(Struct):
    name: Optional[str]
    token: Optional[str]
    def __init__(
            self,
            name: Optional[str] = OMIT,
            token: Optional[str] = OMIT,
    ):
        super().__init__(
            name=name,
            token=token,
        )

class TagMeasurement(Struct):
    id: int
    update_time: float
    base_to_tag_pose: Pose
    corners: Optional[List[float]]
    def __init__(
            self,
            id: int = OMIT,
            update_time: float = OMIT,
            base_to_tag_pose: Pose = OMIT,
            corners: Optional[List[float]] = OMIT,
    ):
        super().__init__(
            id=id,
            update_time=update_time,
            base_to_tag_pose=base_to_tag_pose,
            corners=corners,
        )

class Marker(Struct):
    position: List[float]
    size: float
    def __init__(
            self,
            position: List[float] = OMIT,
            size: float = OMIT,
    ):
        super().__init__(
            position=position,
            size=size,
        )

class MocapMeasurement(Struct):
    measurement_time_utc: float
    transform: Pose
    relative_to: 'ObjectSelector'
    in_coordinates_of: 'ObjectSelector'
    stored_relative_to: 'ObjectSelector'
    def __init__(
            self,
            measurement_time_utc: float = OMIT,
            transform: Pose = OMIT,
            relative_to: 'ObjectSelector' = OMIT,
            in_coordinates_of: 'ObjectSelector' = OMIT,
            stored_relative_to: 'ObjectSelector' = OMIT,
    ):
        super().__init__(
            measurement_time_utc=measurement_time_utc,
            transform=transform,
            relative_to=relative_to,
            in_coordinates_of=in_coordinates_of,
            stored_relative_to=stored_relative_to,
        )

class MeasurementTimestamp(Struct):
    measurement_time: float
    def __init__(
            self,
            measurement_time: float = OMIT,
    ):
        super().__init__(
            measurement_time=measurement_time,
        )

class ObjectAttributes(Struct):
    name: Optional[str]
    parent: Optional[int]
    children: Optional[List[int]]
    timeout: Optional[float]
    mass: Optional[float]
    opacity: Optional[float]
    box_geometry: Optional[List[float]]
    ellipsoid_geometry: Optional[List[float]]
    color: Optional[str]
    mocap_markers: Optional[List['Marker']]
    box_seen_time: Optional[float]
    polygon: Optional[List[List[float]]]
    tag_measurement: Optional['TagMeasurement']
    april_tag_id: Optional[int]
    charuco_id: Optional[int]
    mocap_measurement: Optional['MocapMeasurement']
    measurement_timestamp: Optional['MeasurementTimestamp']
    keep_out: bool
    steppable: bool
    pickable: bool
    is_manipulated: bool
    graspable: bool
    transient: bool
    keep_orphaned: bool
    def __init__(
            self,
            name: Optional[str] = OMIT,
            parent: Optional[int] = OMIT,
            children: Optional[List[int]] = OMIT,
            timeout: Optional[float] = OMIT,
            mass: Optional[float] = OMIT,
            opacity: Optional[float] = OMIT,
            box_geometry: Optional[List[float]] = OMIT,
            ellipsoid_geometry: Optional[List[float]] = OMIT,
            color: Optional[str] = OMIT,
            mocap_markers: Optional[List['Marker']] = OMIT,
            box_seen_time: Optional[float] = OMIT,
            polygon: Optional[List[List[float]]] = OMIT,
            tag_measurement: Optional['TagMeasurement'] = OMIT,
            april_tag_id: Optional[int] = OMIT,
            charuco_id: Optional[int] = OMIT,
            mocap_measurement: Optional['MocapMeasurement'] = OMIT,
            measurement_timestamp: Optional['MeasurementTimestamp'] = OMIT,
            keep_out: bool = OMIT,
            steppable: bool = OMIT,
            pickable: bool = OMIT,
            is_manipulated: bool = OMIT,
            graspable: bool = OMIT,
            transient: bool = OMIT,
            keep_orphaned: bool = OMIT,
    ):
        super().__init__(
            name=name,
            parent=parent,
            children=children,
            timeout=timeout,
            mass=mass,
            opacity=opacity,
            box_geometry=box_geometry,
            ellipsoid_geometry=ellipsoid_geometry,
            color=color,
            mocap_markers=mocap_markers,
            box_seen_time=box_seen_time,
            polygon=polygon,
            tag_measurement=tag_measurement,
            april_tag_id=april_tag_id,
            charuco_id=charuco_id,
            mocap_measurement=mocap_measurement,
            measurement_timestamp=measurement_timestamp,
            keep_out=keep_out,
            steppable=steppable,
            pickable=pickable,
            is_manipulated=is_manipulated,
            graspable=graspable,
            transient=transient,
            keep_orphaned=keep_orphaned,
        )

class Landmark(Struct):
    id: int
    pose: Pose
    std_dev: SpatialVector
    def __init__(
            self,
            id: int = OMIT,
            pose: Pose = OMIT,
            std_dev: SpatialVector = OMIT,
    ):
        super().__init__(
            id=id,
            pose=pose,
            std_dev=std_dev,
        )



# Message definitions

class ActionSetOperationMode(Message):
    mode: str
    def __init__(
            self,
            mode: str = OMIT,
    ):
        super().__init__(
            'action-set-operation-mode',
            mode=mode,
        )

class ActionIdle(Message):
    def __init__(
            self,
    ):
        super().__init__(
            'action-idle',
        )

class ActionStart(Message):
    mode: str
    def __init__(
            self,
            mode: str = OMIT,
    ):
        super().__init__(
            'action-start',
            mode=mode,
        )

class ActionSit(Message):
    mode: str
    mobility_parameters: 'MobilityParameters'
    def __init__(
            self,
            mode: str = OMIT,
            mobility_parameters: 'MobilityParameters' = OMIT,
    ):
        super().__init__(
            'action-sit',
            mode=mode,
            mobility_parameters=mobility_parameters,
        )

class ActionShutdown(Message):
    restart: bool
    def __init__(
            self,
            restart: bool = OMIT,
    ):
        super().__init__(
            'action-shutdown',
            restart=restart,
        )

class ActionCalibrateImu(Message):
    pause_duration: Optional[float]
    def __init__(
            self,
            pause_duration: Optional[float] = OMIT,
    ):
        super().__init__(
            'action-calibrate-imu',
            pause_duration=pause_duration,
        )

class ActionGoto(Message):
    target: GpsPose
    heading_constraint: Optional[float]
    reference_frame: 'ObjectSelector'
    position_tolerance: float
    orientation_tolerance: float
    mobility_parameters: 'MobilityParameters'
    def __init__(
            self,
            target: GpsPose = OMIT,
            heading_constraint: Optional[float] = OMIT,
            reference_frame: 'ObjectSelector' = OMIT,
            position_tolerance: float = OMIT,
            orientation_tolerance: float = OMIT,
            success_after_stand: bool = OMIT,
            mobility_parameters: 'MobilityParameters' = OMIT,
            command: Optional['Command'] = OMIT,
    ):
        super().__init__(
            'action-goto',
            target=target,
            heading_constraint=heading_constraint,
            reference_frame=reference_frame,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            mobility_parameters=mobility_parameters,
        )

class ActionMove(Message):
    velocity: SpatialVector
    mobility_parameters: 'MobilityParameters'
    def __init__(
            self,
            velocity: SpatialVector = OMIT,
            mobility_parameters: 'MobilityParameters' = OMIT,
    ):
        super().__init__(
            'action-move',
            velocity=velocity,
            mobility_parameters=mobility_parameters,
        )

class ActionPick(Message):
    object: 'ObjectSelector'
    mobility_parameters: 'MobilityParameters'
    def __init__(
            self,
            object: 'ObjectSelector' = OMIT,
            mobility_parameters: 'MobilityParameters' = OMIT,
    ):
        super().__init__(
            'action-pick',
            object=object,
            mobility_parameters=mobility_parameters,
        )

class ActionPlace(Message):
    pose: Pose
    reference_frame: 'ObjectSelector'
    position_tolerance: float
    mobility_parameters: 'MobilityParameters'
    def __init__(
            self,
            pose: Pose = OMIT,
            reference_frame: 'ObjectSelector' = OMIT,
            position_tolerance: float = OMIT,
            mobility_parameters: 'MobilityParameters' = OMIT,
    ):
        super().__init__(
            'action-place',
            pose=pose,
            reference_frame=reference_frame,
            position_tolerance=position_tolerance,
            mobility_parameters=mobility_parameters,
        )

class ActionDropObject(Message):
    def __init__(
            self,
    ):
        super().__init__(
            'action-drop-object',
        )

class ActionStand(Message):
    base_pose: Pose
    duration: float
    mobility_parameters: 'MobilityParameters'
    def __init__(
            self,
            base_pose: Pose = OMIT,
            duration: float = OMIT,
            mobility_parameters: 'MobilityParameters' = OMIT,
    ):
        super().__init__(
            'action-stand',
            base_pose=base_pose,
            duration=duration,
            mobility_parameters=mobility_parameters,
        )

class ActionEndEffectorMove(Message):
    end_effector: str
    waypoints: List[Pose]
    reference_frame: 'ObjectSelector'
    stall_threshold: Optional[float]
    cyclic: bool
    max_speed: float
    duration: float
    transition_duration: Optional[float]
    def __init__(
            self,
            end_effector: str = OMIT,
            waypoints: List[Pose] = OMIT,
            reference_frame: 'ObjectSelector' = OMIT,
            stall_threshold: Optional[float] = OMIT,
            cyclic: bool = OMIT,
            max_speed: float = OMIT,
            duration: float = OMIT,
            transition_duration: Optional[float] = OMIT,
            use_time_optimal: bool = OMIT,
            avoid_obstacles: bool = OMIT,
    ):
        super().__init__(
            'action-end-effector-move',
            end_effector=end_effector,
            waypoints=waypoints,
            reference_frame=reference_frame,
            stall_threshold=stall_threshold,
            cyclic=cyclic,
            max_speed=max_speed,
            duration=duration,
            transition_duration=transition_duration,
        )

class ActionSequential(Message):
    actions: List[Message]
    def __init__(
            self,
            actions: List[Message] = OMIT,
    ):
        super().__init__(
            'action-sequential',
            actions=actions,
        )

class ActionConcurrent(Message):
    actions: List[Message]
    def __init__(
            self,
            actions: List[Message] = OMIT,
    ):
        super().__init__(
            'action-concurrent',
            actions=actions,
        )

class ActionTimeout(Message):
    action: Message
    timeout: float
    def __init__(
            self,
            action: Message = OMIT,
            timeout: float = OMIT,
    ):
        super().__init__(
            'action-timeout',
            action=action,
            timeout=timeout,
        )

class ActionDuration(Message):
    action: Message
    duration: float
    after_success: bool
    def __init__(
            self,
            action: Message = OMIT,
            duration: float = OMIT,
            after_success: bool = OMIT,
    ):
        super().__init__(
            'action-duration',
            action=action,
            duration=duration,
            after_success=after_success,
        )

class ActionLoop(Message):
    action: Message
    count: Optional[int]
    def __init__(
            self,
            action: Message = OMIT,
            count: Optional[int] = OMIT,
    ):
        super().__init__(
            'action-loop',
            action=action,
            count=count,
        )

class Error(Message):
    info: str
    def __init__(
            self,
            info: str = OMIT,
    ):
        super().__init__(
            'error',
            info=info,
        )

class Warning(Message):
    info: str
    def __init__(
            self,
            info: str = OMIT,
    ):
        super().__init__(
            'warning',
            info=info,
        )

class Event(Message):
    info: str
    def __init__(
            self,
            info: str = OMIT,
    ):
        super().__init__(
            'event',
            info=info,
        )

class RequestPrivilege(Message):
    privilege: str
    priority: int
    def __init__(
            self,
            privilege: str = OMIT,
            priority: int = OMIT,
    ):
        super().__init__(
            'request-privilege',
            privilege=privilege,
            priority=priority,
        )

class DropPrivilege(Message):
    privilege: str
    def __init__(
            self,
            privilege: str = OMIT,
    ):
        super().__init__(
            'drop-privilege',
            privilege=privilege,
        )

class DefaultMobilityParameters(Message):
    mobility_parameters: 'MobilityParameters'
    def __init__(
            self,
            mobility_parameters: 'MobilityParameters' = OMIT,
    ):
        super().__init__(
            'default-mobility-parameters',
            mobility_parameters=mobility_parameters,
        )

class SessionOptions(Message):
    token: str
    name: Optional[str]
    persistent: bool
    persistent_timeout: Optional[float]
    allow_unsolicited_messages: bool
    def __init__(
            self,
            token: str = OMIT,
            name: Optional[str] = OMIT,
            persistent: bool = OMIT,
            persistent_timeout: Optional[float] = OMIT,
            allow_unsolicited_messages: bool = OMIT,
    ):
        super().__init__(
            'session-options',
            token=token,
            name=name,
            persistent=persistent,
            persistent_timeout=persistent_timeout,
            allow_unsolicited_messages=allow_unsolicited_messages,
        )

class SendPeerMessage(Message):
    to: 'SessionIdentifier'
    message: str
    def __init__(
            self,
            to: 'SessionIdentifier' = OMIT,
            message: str = OMIT,
    ):
        super().__init__(
            'send-peer-message',
            to=to,
            message=message,
        )

class ResumePersistentSession(Message):
    session: 'SessionIdentifier'
    def __init__(
            self,
            session: 'SessionIdentifier' = OMIT,
    ):
        super().__init__(
            'resume-persistent-session',
            session=session,
        )

class StopPersistentSession(Message):
    session: 'SessionIdentifier'
    def __init__(
            self,
            session: 'SessionIdentifier' = OMIT,
    ):
        super().__init__(
            'stop-persistent-session',
            session=session,
        )

class QueryGroup(Message):
    queries: List[Message]
    period: Optional[float]
    def __init__(
            self,
            queries: List[Message] = OMIT,
            period: Optional[float] = OMIT,
    ):
        super().__init__(
            'query-group',
            queries=queries,
            period=period,
        )

class GetPrivileges(Message):
    def __init__(
            self,
    ):
        super().__init__(
            'get-privileges',
        )

class GetConfigInfo(Message):
    def __init__(
            self,
    ):
        super().__init__(
            'get-config-info',
        )

class AddSequentialActions(Message):
    actions: List[Message]
    reference_number: Optional[int]
    append: bool
    def __init__(
            self,
            actions: List[Message] = OMIT,
            reference_number: Optional[int] = OMIT,
            append: bool = OMIT,
    ):
        super().__init__(
            'add-sequential-actions',
            actions=actions,
            reference_number=reference_number,
            append=append,
        )

class AddConcurrentActions(Message):
    actions: List[Message]
    reference_number: Optional[int]
    def __init__(
            self,
            actions: List[Message] = OMIT,
            reference_number: Optional[int] = OMIT,
    ):
        super().__init__(
            'add-concurrent-actions',
            actions=actions,
            reference_number=reference_number,
        )

class ReplaceAction(Message):
    action: Message
    reference_number: Optional[int]
    fallback: str
    def __init__(
            self,
            action: Message = OMIT,
            reference_number: Optional[int] = OMIT,
            fallback: str = OMIT,
    ):
        super().__init__(
            'replace-action',
            action=action,
            reference_number=reference_number,
            fallback=fallback,
        )

class RemoveAction(Message):
    reference_number: Optional[int]
    def __init__(
            self,
            reference_number: Optional[int] = OMIT,
    ):
        super().__init__(
            'remove-action',
            reference_number=reference_number,
        )

class GetActionCommand(Message):
    def __init__(
            self,
    ):
        super().__init__(
            'get-action-command',
        )

class GetExecutionState(Message):
    def __init__(
            self,
    ):
        super().__init__(
            'get-execution-state',
        )

class SimulatorEmergencyStop(Message):
    enable_motors: bool
    def __init__(
            self,
            enable_motors: bool = OMIT,
    ):
        super().__init__(
            'simulator-emergency-stop',
            enable_motors=enable_motors,
        )

class SimulatorApplyForce(Message):
    model_id: int
    body: Optional[str]
    offset: List[float]
    reference: str
    force: SpatialVector
    duration: float
    def __init__(
            self,
            model_id: int = OMIT,
            body: Optional[str] = OMIT,
            offset: List[float] = OMIT,
            reference: str = OMIT,
            force: SpatialVector = OMIT,
            duration: float = OMIT,
    ):
        super().__init__(
            'simulator-apply-force',
            model_id=model_id,
            body=body,
            offset=offset,
            reference=reference,
            force=force,
            duration=duration,
        )

class SimulatorSetModelKinematics(Message):
    model_id: int
    transform: Pose
    velocity: Optional[SpatialVector]
    def __init__(
            self,
            model_id: int = OMIT,
            transform: Pose = OMIT,
            velocity: Optional[SpatialVector] = OMIT,
    ):
        super().__init__(
            'simulator-set-model-kinematics',
            model_id=model_id,
            transform=transform,
            velocity=velocity,
        )

class SimulatorPause(Message):
    def __init__(
            self,
    ):
        super().__init__(
            'simulator-pause',
        )

class SimulatorStart(Message):
    def __init__(
            self,
    ):
        super().__init__(
            'simulator-start',
        )

class SimulatorStep(Message):
    dt: float
    def __init__(
            self,
            dt: float = OMIT,
    ):
        super().__init__(
            'simulator-step',
            dt=dt,
        )

class SimulatorGetModelKinematics(Message):
    model_id: int
    def __init__(
            self,
            model_id: int = OMIT,
    ):
        super().__init__(
            'simulator-get-model-kinematics',
            model_id=model_id,
        )

class SimulatorGetStatus(Message):
    def __init__(
            self,
    ):
        super().__init__(
            'simulator-get-status',
        )

class GetBatteryStatus(Message):
    def __init__(
            self,
    ):
        super().__init__(
            'get-battery-status',
        )

class CalibrationTable(Message):
    has_rgb: bool
    resolution_left_right: List[int]
    resolution_rgb: List[int]
    focal_length_left: List[float]
    focal_length_right: List[float]
    focal_length_rgb: List[float]
    principal_point_left: List[float]
    principal_point_right: List[float]
    principal_point_rgb: List[float]
    distortion_left: List[float]
    distortion_right: List[float]
    distortion_rgb: List[float]
    rotation_left_external: List[float]
    rotation_left_right: List[float]
    rotation_left_rgb: List[float]
    translation_left_external: List[float]
    translation_left_right: List[float]
    translation_left_rgb: List[float]
    def __init__(
            self,
            has_rgb: bool = OMIT,
            resolution_left_right: List[int] = OMIT,
            resolution_rgb: List[int] = OMIT,
            focal_length_left: List[float] = OMIT,
            focal_length_right: List[float] = OMIT,
            focal_length_rgb: List[float] = OMIT,
            principal_point_left: List[float] = OMIT,
            principal_point_right: List[float] = OMIT,
            principal_point_rgb: List[float] = OMIT,
            distortion_left: List[float] = OMIT,
            distortion_right: List[float] = OMIT,
            distortion_rgb: List[float] = OMIT,
            rotation_left_external: List[float] = OMIT,
            rotation_left_right: List[float] = OMIT,
            rotation_left_rgb: List[float] = OMIT,
            translation_left_external: List[float] = OMIT,
            translation_left_right: List[float] = OMIT,
            translation_left_rgb: List[float] = OMIT,
    ):
        super().__init__(
            'calibration-table',
            has_rgb=has_rgb,
            resolution_left_right=resolution_left_right,
            resolution_rgb=resolution_rgb,
            focal_length_left=focal_length_left,
            focal_length_right=focal_length_right,
            focal_length_rgb=focal_length_rgb,
            principal_point_left=principal_point_left,
            principal_point_right=principal_point_right,
            principal_point_rgb=principal_point_rgb,
            distortion_left=distortion_left,
            distortion_right=distortion_right,
            distortion_rgb=distortion_rgb,
            rotation_left_external=rotation_left_external,
            rotation_left_right=rotation_left_right,
            rotation_left_rgb=rotation_left_rgb,
            translation_left_external=translation_left_external,
            translation_left_right=translation_left_right,
            translation_left_rgb=translation_left_rgb,
        )

class CancelQuery(Message):
    reference_number: Optional[int]
    def __init__(
            self,
            reference_number: Optional[int] = OMIT,
    ):
        super().__init__(
            'cancel-query',
            reference_number=reference_number,
        )

class SetFloorplanMap(Message):
    name: str
    image_data: str
    resolution: float
    origin: Pose
    landmarks: List['Landmark']
    initial_pose: Pose
    def __init__(
            self,
            name: str = OMIT,
            image_data: str = OMIT,
            resolution: float = OMIT,
            origin: Pose = OMIT,
            landmarks: List['Landmark'] = OMIT,
            initial_pose: Pose = OMIT,
    ):
        super().__init__(
            'set-floorplan-map',
            name=name,
            image_data=image_data,
            resolution=resolution,
            origin=origin,
            landmarks=landmarks,
            initial_pose=initial_pose,
        )

class AddLandmarks(Message):
    map_name: str
    landmarks: List['Landmark']
    def __init__(
            self,
            map_name: str = OMIT,
            landmarks: List['Landmark'] = OMIT,
    ):
        super().__init__(
            'add-landmarks',
            map_name=map_name,
            landmarks=landmarks,
        )

class ClearFloorplanMap(Message):
    name: str
    def __init__(
            self,
            name: str = OMIT,
    ):
        super().__init__(
            'clear-floorplan-map',
            name=name,
        )

class SetFloorplanMapToBasePose(Message):
    name: str
    pose: Pose
    def __init__(
            self,
            name: str = OMIT,
            pose: Pose = OMIT,
    ):
        super().__init__(
            'set-floorplan-map-to-base-pose',
            name=name,
            pose=pose,
        )

class AddObject(Message):
    attributes: 'ObjectAttributes'
    transform: Optional[Pose]
    velocity: Optional[SpatialVector]
    relative_to: 'ObjectSelector'
    in_coordinates_of: 'ObjectSelector'
    stored_relative_to: 'ObjectSelector'
    def __init__(
            self,
            attributes: 'ObjectAttributes' = OMIT,
            transform: Optional[Pose] = OMIT,
            velocity: Optional[SpatialVector] = OMIT,
            relative_to: 'ObjectSelector' = OMIT,
            in_coordinates_of: 'ObjectSelector' = OMIT,
            stored_relative_to: 'ObjectSelector' = OMIT,
    ):
        super().__init__(
            'add-object',
            attributes=attributes,
            transform=transform,
            velocity=velocity,
            relative_to=relative_to,
            in_coordinates_of=in_coordinates_of,
            stored_relative_to=stored_relative_to,
        )

class RemoveObjects(Message):
    objects: 'ObjectSelector'
    def __init__(
            self,
            objects: 'ObjectSelector' = OMIT,
    ):
        super().__init__(
            'remove-objects',
            objects=objects,
        )

class SetObjectAttributes(Message):
    object: 'ObjectSelector'
    attributes: 'ObjectAttributes'
    def __init__(
            self,
            object: 'ObjectSelector' = OMIT,
            attributes: 'ObjectAttributes' = OMIT,
    ):
        super().__init__(
            'set-object-attributes',
            object=object,
            attributes=attributes,
        )

class RemoveObjectAttributes(Message):
    object: 'ObjectSelector'
    attributes: List[str]
    def __init__(
            self,
            object: 'ObjectSelector' = OMIT,
            attributes: List[str] = OMIT,
    ):
        super().__init__(
            'remove-object-attributes',
            object=object,
            attributes=attributes,
        )

class SetObjectKinematics(Message):
    object: 'ObjectSelector'
    transform: Pose
    velocity: Optional[SpatialVector]
    relative_to: 'ObjectSelector'
    in_coordinates_of: 'ObjectSelector'
    stored_relative_to: 'ObjectSelector'
    def __init__(
            self,
            object: 'ObjectSelector' = OMIT,
            transform: Pose = OMIT,
            velocity: Optional[SpatialVector] = OMIT,
            relative_to: 'ObjectSelector' = OMIT,
            in_coordinates_of: 'ObjectSelector' = OMIT,
            stored_relative_to: 'ObjectSelector' = OMIT,
    ):
        super().__init__(
            'set-object-kinematics',
            object=object,
            transform=transform,
            velocity=velocity,
            relative_to=relative_to,
            in_coordinates_of=in_coordinates_of,
            stored_relative_to=stored_relative_to,
        )

class PerceptionStreamStart(Message):
    stream: str
    flow_control: str
    def __init__(
            self,
            stream: str = OMIT,
            flow_control: str = OMIT,
    ):
        super().__init__(
            'perception-stream-start',
            stream=stream,
            flow_control=flow_control,
        )

class PerceptionStreamStop(Message):
    stream: str
    def __init__(
            self,
            stream: str = OMIT,
    ):
        super().__init__(
            'perception-stream-stop',
            stream=stream,
        )

class GetPerceptionStreams(Message):
    def __init__(
            self,
    ):
        super().__init__(
            'get-perception-streams',
        )

class GetLandmarks(Message):
    map_name: str
    def __init__(
            self,
            map_name: str = OMIT,
    ):
        super().__init__(
            'get-landmarks',
            map_name=map_name,
        )

class GetTimestamp(Message):
    def __init__(
            self,
    ):
        super().__init__(
            'get-timestamp',
        )

class GetObjectKinematics(Message):
    object: 'ObjectSelector'
    relative_to: 'ObjectSelector'
    in_coordinates_of: 'ObjectSelector'
    representation: str
    def __init__(
            self,
            object: 'ObjectSelector' = OMIT,
            relative_to: 'ObjectSelector' = OMIT,
            in_coordinates_of: 'ObjectSelector' = OMIT,
            representation: str = OMIT,
    ):
        super().__init__(
            'get-object-kinematics',
            object=object,
            relative_to=relative_to,
            in_coordinates_of=in_coordinates_of,
            representation=representation,
        )

class GetRobotInfo(Message):
    def __init__(
            self,
    ):
        super().__init__(
            'get-robot-info',
        )

class GetRobotStatus(Message):
    def __init__(
            self,
    ):
        super().__init__(
            'get-robot-status',
        )

class GetSnapshot(Message):
    stream: str
    quality: int
    def __init__(
            self,
            stream: str = OMIT,
            quality: int = OMIT,
    ):
        super().__init__(
            'get-snapshot',
            stream=stream,
            quality=quality,
        )

class GetObject(Message):
    object: 'ObjectSelector'
    def __init__(
            self,
            object: 'ObjectSelector' = OMIT,
    ):
        super().__init__(
            'get-object',
            object=object,
        )

class NotifyObjects(Message):
    objects: 'ObjectSelector'
    persistent: bool
    def __init__(
            self,
            objects: 'ObjectSelector' = OMIT,
            persistent: bool = OMIT,
    ):
        super().__init__(
            'notify-objects',
            objects=objects,
            persistent=persistent,
        )

class FindObjects(Message):
    objects: 'ObjectSelector'
    def __init__(
            self,
            objects: 'ObjectSelector' = OMIT,
    ):
        super().__init__(
            'find-objects',
            objects=objects,
        )

class GetGpsCoordinates(Message):
    object: 'ObjectSelector'
    def __init__(
            self,
            object: 'ObjectSelector' = OMIT,
    ):
        super().__init__(
            'get-gps-coordinates',
            object=object,
        )

