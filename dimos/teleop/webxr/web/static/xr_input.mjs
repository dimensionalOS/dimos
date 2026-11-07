// Copyright 2026 Dimensional Inc. Licensed under the Apache License, Version 2.0.
// Device input only: no robot geometry, command transport or scene placement.
const point = p => [p.x, p.y, p.z];
const quaternion = q => [q.x, q.y, q.z, q.w];
const separation = (a, b) => Math.hypot(...a.map((v, i) => v - b[i]));
const buttonDistance = value => 0.05 - 0.04 * value;
const value = button => Math.max(0, Math.min(1, button?.value ?? 0));

export function xrSessionOptions() {
  return {requiredFeatures: ['local-floor'], optionalFeatures: ['hand-tracking']};
}

export function interactionMode(preference, samples) {
  return preference === 'auto'
    ? Object.values(samples).some(sample => sample.kind === 'controller' && Number.isFinite(sample.clutchDistance)) ? 'clutch' : 'tools'
    : preference;
}

export function sampleXRInputs(frame, reference) {
  const samples = {};
  for (const source of frame.session.inputSources) {
    const side = source.handedness;
    // Vision Pro gaze/pinch transient pointers aren't tracked hand poses.
    if (!['left', 'right'].includes(side) || source.targetRayMode !== 'tracked-pointer') continue;
    const rayPose = frame.getPose(source.targetRaySpace, reference);
    const ray = rayPose ? {
      position: point(rayPose.transform.position), orientation: quaternion(rayPose.transform.orientation),
    } : null;
    let sample;
    if (source.hand) {
      const poses = Object.fromEntries([...source.hand].map(([name, joint]) => [name, frame.getJointPose(joint, reference)]));
      const thumb = poses['thumb-tip'], index = poses['index-finger-tip'], wrist = poses.wrist;
      if (!thumb || !index || !wrist || samples[side]?.kind === 'controller') continue;
      const a = point(thumb.transform.position), b = point(index.transform.position);
      const middle = poses['middle-finger-tip'];
      const distance = separation(a, b);
      sample = {kind: 'hand', poses, position: a.map((v, i) => (v + b[i]) / 2),
        orientation: quaternion(wrist.transform.orientation), distance, clutchDistance: distance,
        trigger: middle ? Math.max(0, Math.min(1, (0.04 - separation(a, point(middle.transform.position))) / 0.02)) : 0};
    } else {
      const pose = source.gripSpace && frame.getPose(source.gripSpace, reference);
      if (!pose || !source.gamepad) continue;
      const buttons = source.gamepad.buttons;
      // xr-standard's trigger/squeeze slots, also used by the existing teleop client.
      sample = {kind: 'controller', position: point(pose.transform.position),
        orientation: quaternion(pose.transform.orientation), trigger: value(buttons[0]),
        distance: buttonDistance(value(buttons[0])),
        clutchDistance: buttons[1] ? buttonDistance(value(buttons[1])) : NaN};
    }
    samples[side] = {...sample, source, ray};
  }
  return samples;
}
