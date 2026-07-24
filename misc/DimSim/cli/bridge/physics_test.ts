import { applyEvalReset } from "./eval-reset.ts";
import { ServerPhysics } from "./physics.ts";

function assert(
  condition: unknown,
  message = "assertion failed",
): asserts condition {
  if (!condition) throw new Error(message);
}

function assertClose(actual: number, expected: number): void {
  if (Math.abs(actual - expected) > 1e-9) {
    throw new Error(`expected ${expected}, got ${actual}`);
  }
}

Deno.test("server physics reset applies position/yaw and zeroes motion", () => {
  const physics = Object.create(ServerPhysics.prototype) as any;
  let nextPosition = { x: 0, y: 0, z: 0 };
  let rotation = { x: 0, y: 0, z: 0, w: 1 };
  let odomCount = 0;
  let poseUpdate: number[] = [];
  let linvel: unknown;
  let angvel: unknown;
  physics.body = {
    setNextKinematicTranslation(value: typeof nextPosition) {
      nextPosition = value;
    },
    setNextKinematicRotation(value: typeof rotation) {
      rotation = value;
    },
    translation() {
      return nextPosition;
    },
    setLinvel(value: unknown) {
      linvel = value;
    },
    setAngvel(value: unknown) {
      angvel = value;
    },
  };
  physics.world = { step() {} };
  physics._publishOdom = () => {
    odomCount++;
  };
  physics.onPoseUpdate = (...values: number[]) => {
    poseUpdate = values;
  };
  physics.linX = 1;
  physics.linY = 2;
  physics.linZ = 3;
  physics.angZ = 4;
  physics.cmdVelStamp = 123;
  physics.lastStepAt = 456;

  const result = physics.resetPose({ x: 1, y: 2, z: 3, yaw: 90 });

  assertClose(result.x, 1);
  assertClose(result.y, 2);
  assertClose(result.z, 3);
  assertClose(result.yaw, 90);
  assertClose(rotation.y, Math.SQRT1_2);
  assertClose(rotation.w, Math.SQRT1_2);
  assert(physics.linX === 0 && physics.linY === 0 && physics.linZ === 0);
  assert(physics.angZ === 0 && physics.cmdVelStamp === 0);
  assert(physics.lastStepAt === 0);
  assert(JSON.stringify(linvel) === JSON.stringify({ x: 0, y: 0, z: 0 }));
  assert(JSON.stringify(angvel) === JSON.stringify({ x: 0, y: 0, z: 0 }));
  assert(odomCount === 1);
  assertClose(poseUpdate[3], Math.PI / 2);
});

Deno.test("bridge reset acknowledgement returns actual pose", () => {
  const ack = applyEvalReset(
    {
      clearMotion() {},
      resetPose: () => ({ x: 1.25, y: 0.5, z: -2, yaw: 45 }),
    },
    {
      type: "evalReset",
      runId: "run-1",
      startPose: { x: 1, y: 0.5, z: -2, yaw: 45 },
    },
  );
  assert(ack.ok);
  assert(ack.pose?.x === 1.25);
  assert(ack.runId === "run-1");
});

Deno.test("bridge reset fails when server physics is unavailable", () => {
  const ack = applyEvalReset(null, {
    type: "evalReset",
    runId: "run-2",
    startPose: { x: 0, y: 0.5, z: 3, yaw: 0 },
  });
  assert(!ack.ok);
  assert(ack.reason?.includes("unavailable"));
});
