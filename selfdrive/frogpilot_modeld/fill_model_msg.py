import os
import capnp
import numpy as np
from cereal import log
from openpilot.selfdrive.frogpilot_modeld.constants import ModelConstants, Plan, Meta

SEND_RAW_PRED = os.getenv('SEND_RAW_PRED')

ConfidenceClass = log.ModelDataV2.ConfidenceClass

class PublishState:
  def __init__(self):
    self.disengage_buffer = np.zeros(ModelConstants.CONFIDENCE_BUFFER_LEN*ModelConstants.DISENGAGE_WIDTH, dtype=np.float32)
    self.prev_brake_5ms2_probs = np.zeros(ModelConstants.FCW_5MS2_PROBS_WIDTH, dtype=np.float32)
    self.prev_brake_3ms2_probs = np.zeros(ModelConstants.FCW_3MS2_PROBS_WIDTH, dtype=np.float32)

def fill_xyzt(builder, t, x, y, z, x_std=None, y_std=None, z_std=None):
  builder.t = t
  builder.x = x.tolist()
  builder.y = y.tolist()
  builder.z = z.tolist()
  if x_std is not None:
    builder.xStd = x_std.tolist()
  if y_std is not None:
    builder.yStd = y_std.tolist()
  if z_std is not None:
    builder.zStd = z_std.tolist()

def fill_xyvat(builder, t, x, y, v, a, x_std=None, y_std=None, v_std=None, a_std=None):
  builder.t = t
  builder.x = x.tolist()
  builder.y = y.tolist()
  builder.v = v.tolist()
  builder.a = a.tolist()
  if x_std is not None:
    builder.xStd = x_std.tolist()
  if y_std is not None:
    builder.yStd = y_std.tolist()
  if v_std is not None:
    builder.vStd = v_std.tolist()
  if a_std is not None:
    builder.aStd = a_std.tolist()

def fill_model_msg(msg: capnp._DynamicStructBuilder, lateral_output_data: dict[str, np.ndarray], publish_state: PublishState,
                   vipc_frame_id: int, vipc_frame_id_extra: int, frame_id: int, frame_drop: float,
                   timestamp_eof: int, timestamp_llk: int, model_execution_time: float, valid: bool, nav_enabled: bool) -> None:
  frame_age = frame_id - vipc_frame_id if frame_id > vipc_frame_id else 0
  msg.valid = valid

  frogpilotModelV2 = msg.frogpilotModelV2
  frogpilotModelV2.frameId = vipc_frame_id
  frogpilotModelV2.frameIdExtra = vipc_frame_id_extra
  frogpilotModelV2.frameAge = frame_age
  frogpilotModelV2.frameDropPerc = frame_drop * 100
  frogpilotModelV2.timestampEof = timestamp_eof
  frogpilotModelV2.modelExecutionTime = model_execution_time
  frogpilotModelV2.navEnabled = nav_enabled

  # plan
  position = frogpilotModelV2.position
  fill_xyzt(position, ModelConstants.T_IDXS, *lateral_output_data['plan'][0,:,Plan.POSITION].T, *lateral_output_data['plan_stds'][0,:,Plan.POSITION].T)
  velocity = frogpilotModelV2.velocity
  fill_xyzt(velocity, ModelConstants.T_IDXS, *lateral_output_data['plan'][0,:,Plan.VELOCITY].T)
  acceleration = frogpilotModelV2.acceleration
  fill_xyzt(acceleration, ModelConstants.T_IDXS, *lateral_output_data['plan'][0,:,Plan.ACCELERATION].T)
  orientation = frogpilotModelV2.orientation
  fill_xyzt(orientation, ModelConstants.T_IDXS, *lateral_output_data['plan'][0,:,Plan.T_FROM_CURRENT_EULER].T)
  orientation_rate = frogpilotModelV2.orientationRate
  fill_xyzt(orientation_rate, ModelConstants.T_IDXS, *lateral_output_data['plan'][0,:,Plan.ORIENTATION_RATE].T)

  # lateral planning
  action = frogpilotModelV2.action
  action.desiredCurvature = float(lateral_output_data['desired_curvature'][0,0])

  # times at X_IDXS according to model plan
  PLAN_T_IDXS = [np.nan] * ModelConstants.IDX_N
  PLAN_T_IDXS[0] = 0.0
  plan_x = lateral_output_data['plan'][0,:,Plan.POSITION][:,0].tolist()
  for xidx in range(1, ModelConstants.IDX_N):
    tidx = 0
    # increment tidx until we find an element that's further away than the current xidx
    while tidx < ModelConstants.IDX_N - 1 and plan_x[tidx+1] < ModelConstants.X_IDXS[xidx]:
      tidx += 1
    if tidx == ModelConstants.IDX_N - 1:
      # if the Plan doesn't extend far enough, set plan_t to the max value (10s), then break
      PLAN_T_IDXS[xidx] = ModelConstants.T_IDXS[ModelConstants.IDX_N - 1]
      break
    # interpolate to find `t` for the current xidx
    current_x_val = plan_x[tidx]
    next_x_val = plan_x[tidx+1]
    p = (ModelConstants.X_IDXS[xidx] - current_x_val) / (next_x_val - current_x_val) if abs(next_x_val - current_x_val) > 1e-9 else float('nan')
    PLAN_T_IDXS[xidx] = p * ModelConstants.T_IDXS[tidx+1] + (1 - p) * ModelConstants.T_IDXS[tidx]

  # lane lines
  frogpilotModelV2.init('laneLines', 6)
  for i in range(6):
    lane_line = frogpilotModelV2.laneLines[i]
    if i < 4:
      fill_xyzt(lane_line, PLAN_T_IDXS, np.array(ModelConstants.X_IDXS), lateral_output_data['lane_lines'][0,i,:,0], lateral_output_data['lane_lines'][0,i,:,1])
    else:
      far_lane, near_lane, road_edge = (0, 1, 0) if i == 4 else (3, 2, 1)

      near_lane_y = lateral_output_data['lane_lines'][0,near_lane,:,0]
      road_edge_y = lateral_output_data['road_edges'][0,road_edge,:,0]
      far_lane_y = lateral_output_data['lane_lines'][0,far_lane,:,0]

      road_edge_distance = abs(np.linalg.norm(road_edge_y - near_lane_y))
      far_lane_distance = abs(np.linalg.norm(far_lane_y - near_lane_y))

      if road_edge_distance < far_lane_distance:
        closest_lane_y = road_edge_y
      else:
        closest_lane_y = far_lane_y

      diff_y = closest_lane_y - near_lane_y
      new_lane_y = near_lane_y + diff_y / 2

      fill_xyzt(lane_line, PLAN_T_IDXS, np.array(ModelConstants.X_IDXS), new_lane_y, lateral_output_data['lane_lines'][0,near_lane,:,1])

  frogpilotModelV2.laneLineStds = lateral_output_data['lane_lines_stds'][0,:,0,0].tolist()
  frogpilotModelV2.laneLineProbs = lateral_output_data['lane_lines_prob'][0,1::2].tolist()

  # road edges
  frogpilotModelV2.init('roadEdges', 2)
  for i in range(2):
    road_edge = frogpilotModelV2.roadEdges[i]
    fill_xyzt(road_edge, PLAN_T_IDXS, np.array(ModelConstants.X_IDXS), lateral_output_data['road_edges'][0,i,:,0], lateral_output_data['road_edges'][0,i,:,1])
  frogpilotModelV2.roadEdgeStds = lateral_output_data['road_edges_stds'][0,:,0,0].tolist()

  # leads
  frogpilotModelV2.init('leadsV3', 3)
  for i in range(3):
    lead = frogpilotModelV2.leadsV3[i]
    fill_xyvat(lead, ModelConstants.LEAD_T_IDXS, *lateral_output_data['lead'][0,i].T, *lateral_output_data['lead_stds'][0,i].T)
    lead.prob = lateral_output_data['lead_prob'][0,i].tolist()
    lead.probTime = ModelConstants.LEAD_T_OFFSETS[i]

  # meta
  meta = frogpilotModelV2.meta
  meta.desireState = lateral_output_data['desire_state'][0].reshape(-1).tolist()
  meta.desirePrediction = lateral_output_data['desire_pred'][0].reshape(-1).tolist()
  meta.engagedProb = lateral_output_data['meta'][0,Meta.ENGAGED].item()
  meta.init('disengagePredictions')
  disengage_predictions = meta.disengagePredictions
  disengage_predictions.t = ModelConstants.META_T_IDXS
  disengage_predictions.brakeDisengageProbs = lateral_output_data['meta'][0,Meta.BRAKE_DISENGAGE].tolist()
  disengage_predictions.gasDisengageProbs = lateral_output_data['meta'][0,Meta.GAS_DISENGAGE].tolist()
  disengage_predictions.steerOverrideProbs = lateral_output_data['meta'][0,Meta.STEER_OVERRIDE].tolist()
  disengage_predictions.brake3MetersPerSecondSquaredProbs = lateral_output_data['meta'][0,Meta.HARD_BRAKE_3].tolist()
  disengage_predictions.brake4MetersPerSecondSquaredProbs = lateral_output_data['meta'][0,Meta.HARD_BRAKE_4].tolist()
  disengage_predictions.brake5MetersPerSecondSquaredProbs = lateral_output_data['meta'][0,Meta.HARD_BRAKE_5].tolist()

  publish_state.prev_brake_5ms2_probs[:-1] = publish_state.prev_brake_5ms2_probs[1:]
  publish_state.prev_brake_5ms2_probs[-1] = lateral_output_data['meta'][0,Meta.HARD_BRAKE_5][0]
  publish_state.prev_brake_3ms2_probs[:-1] = publish_state.prev_brake_3ms2_probs[1:]
  publish_state.prev_brake_3ms2_probs[-1] = lateral_output_data['meta'][0,Meta.HARD_BRAKE_3][0]
  hard_brake_predicted = (publish_state.prev_brake_5ms2_probs > ModelConstants.FCW_THRESHOLDS_5MS2).all() and \
    (publish_state.prev_brake_3ms2_probs > ModelConstants.FCW_THRESHOLDS_3MS2).all()
  meta.hardBrakePredicted = hard_brake_predicted.item()

  # temporal pose
  temporal_pose = frogpilotModelV2.temporalPose
  temporal_pose.trans = lateral_output_data['sim_pose'][0,:3].tolist()
  temporal_pose.transStd = lateral_output_data['sim_pose_stds'][0,:3].tolist()
  temporal_pose.rot = lateral_output_data['sim_pose'][0,3:].tolist()
  temporal_pose.rotStd = lateral_output_data['sim_pose_stds'][0,3:].tolist()

  # confidence
  if vipc_frame_id % (2*ModelConstants.MODEL_FREQ) == 0:
    # any disengage prob
    brake_disengage_probs = lateral_output_data['meta'][0,Meta.BRAKE_DISENGAGE]
    gas_disengage_probs = lateral_output_data['meta'][0,Meta.GAS_DISENGAGE]
    steer_override_probs = lateral_output_data['meta'][0,Meta.STEER_OVERRIDE]
    any_disengage_probs = 1-((1-brake_disengage_probs)*(1-gas_disengage_probs)*(1-steer_override_probs))
    # independent disengage prob for each 2s slice
    ind_disengage_probs = np.r_[any_disengage_probs[0], np.diff(any_disengage_probs) / (1 - any_disengage_probs[:-1])]
    # rolling buf for 2, 4, 6, 8, 10s
    publish_state.disengage_buffer[:-ModelConstants.DISENGAGE_WIDTH] = publish_state.disengage_buffer[ModelConstants.DISENGAGE_WIDTH:]
    publish_state.disengage_buffer[-ModelConstants.DISENGAGE_WIDTH:] = ind_disengage_probs

  score = 0.
  for i in range(ModelConstants.DISENGAGE_WIDTH):
    score += publish_state.disengage_buffer[i*ModelConstants.DISENGAGE_WIDTH+ModelConstants.DISENGAGE_WIDTH-1-i].item() / ModelConstants.DISENGAGE_WIDTH
  if score < ModelConstants.RYG_GREEN:
    frogpilotModelV2.confidence = ConfidenceClass.green
  elif score < ModelConstants.RYG_YELLOW:
    frogpilotModelV2.confidence = ConfidenceClass.yellow
  else:
    frogpilotModelV2.confidence = ConfidenceClass.red

  # raw prediction if enabled
  if SEND_RAW_PRED:
    frogpilotModelV2.rawPredictions = lateral_output_data['raw_pred'].tobytes()

def fill_pose_msg(msg: capnp._DynamicStructBuilder, net_output_data: dict[str, np.ndarray],
                  vipc_frame_id: int, vipc_dropped_frames: int, timestamp_eof: int, live_calib_seen: bool) -> None:
  msg.valid = live_calib_seen & (vipc_dropped_frames < 1)
  cameraOdometry = msg.cameraOdometry

  cameraOdometry.frameId = vipc_frame_id
  cameraOdometry.timestampEof = timestamp_eof

  cameraOdometry.trans = net_output_data['pose'][0,:3].tolist()
  cameraOdometry.rot = net_output_data['pose'][0,3:].tolist()
  cameraOdometry.wideFromDeviceEuler = net_output_data['wide_from_device_euler'][0,:].tolist()
  cameraOdometry.roadTransformTrans = net_output_data['road_transform'][0,:3].tolist()
  cameraOdometry.transStd = net_output_data['pose_stds'][0,:3].tolist()
  cameraOdometry.rotStd = net_output_data['pose_stds'][0,3:].tolist()
  cameraOdometry.wideFromDeviceEulerStd = net_output_data['wide_from_device_euler_stds'][0,:].tolist()
  cameraOdometry.roadTransformTransStd = net_output_data['road_transform_stds'][0,:3].tolist()
