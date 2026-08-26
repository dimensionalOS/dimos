// Capture every body-joint pose that resolves in this animation frame.
// A missing body source is different from a present source with no usable poses.
export function captureBody(frame, referenceSpace) {
    const body = frame.body;
    if (!body) return null;

    const joints = {};
    for (const [jointName, jointSpace] of body) {
        const pose = frame.getPose(jointSpace, referenceSpace);
        if (!pose) continue;

        const position = pose.transform.position;
        const orientation = pose.transform.orientation;
        joints[jointName] = {
            position: [position.x, position.y, position.z],
            orientation: [orientation.x, orientation.y, orientation.z, orientation.w],
        };
    }
    return joints;
}
