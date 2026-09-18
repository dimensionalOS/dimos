import type { RobotInfo } from "@dimos/sdk";
import styles from "./RobotPicker.module.css";

/**
 * The robots registered on the relay, one button each; picking one pins the
 * session's watch to it. Sorted by name so a `robots` push (a robot joining
 * or leaving) does not reorder the list under the operator's pointer.
 */
export function RobotPicker({ robots, current, onPick }: {
  robots: RobotInfo[];
  /** Id of the robot being watched, marked in the list; null when none. */
  current: string | null;
  onPick: (id: string) => void;
}) {
  const sorted = [...robots].sort((a, b) =>
    a.name.localeCompare(b.name) || a.id.localeCompare(b.id)
  );
  return (
    <div className={styles.picker} data-testid="robot-picker">
      <p className={styles.hint}>Pick a robot to watch</p>
      {sorted.map((robot) => (
        <button
          key={robot.id}
          type="button"
          className={styles.robot}
          aria-current={robot.id === current}
          data-testid={`robot-pick-${robot.id}`}
          onClick={() => onPick(robot.id)}
        >
          <span className={styles.name}>{robot.name}</span>
          <span className={styles.meta}>
            {robot.model !== "" && <span>{robot.model}</span>}
            <span className={styles.id}>{robot.id}</span>
          </span>
        </button>
      ))}
    </div>
  );
}
