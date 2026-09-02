import type { ChannelSpec, ChannelStore, PanelSpec } from "@dimos/sdk";
import { useStoreChannel } from "@dimos/sdk/react";
import { channelSubscribable, cockpitDecoders } from "../subscriptions.ts";
import styles from "./ChannelList.module.css";

function ChannelRow(
  { spec, panels, store }: { spec: ChannelSpec; panels: PanelSpec[]; store: ChannelStore },
) {
  const { slot, stats } = useStoreChannel(store, spec.ch);
  const supported = cockpitDecoders.get(spec.encoding) !== undefined;
  const subscribed = channelSubscribable(spec, panels);

  let value;
  if (!supported) {
    // Not an error: this build has no decoder for the encoding (binary
    // decoders arrive with their panels), so the channel is not subscribed.
    value = <span className={styles.muted}>not subscribed (no decoder for {spec.encoding})</span>;
  } else if (!subscribed) {
    // Decodable, but a panel-only encoding with no renderable panel binding
    // it: the session skipped the sub to save encode CPU and bandwidth.
    value = <span className={styles.muted}>not subscribed (no panel binds it)</span>;
  } else if (slot === null) {
    value = <span className={styles.muted}>waiting for data...</span>;
  } else if (slot.preview !== undefined) {
    value = <code>{slot.preview}</code>;
  } else {
    value = <span className={styles.muted}>(no preview)</span>;
  }

  const age = stats.ageMs === null ? "-" : `${(stats.ageMs / 1000).toFixed(1)} s`;

  return (
    <tr>
      <td className={styles.ch}>{spec.ch}</td>
      <td className={styles.muted}>{spec.encoding}</td>
      <td className={styles.num}>{stats.hz.toFixed(1)}</td>
      <td className={styles.num}>{age}</td>
      <td className={styles.num} data-testid={`ch-${spec.ch}-seq`}>
        {slot?.seq ?? "-"}
      </td>
      <td className={styles.value} data-testid={`ch-${spec.ch}-value`}>
        {stats.decodeFailing && (
          <span className={styles.error} data-testid={`ch-${spec.ch}-decode-error`}>
            decode failing ({stats.decodeErrors} bad frames)
          </span>
        )}
        {value}
      </td>
    </tr>
  );
}

export function ChannelList(
  { channels, panels, store }: {
    channels: ChannelSpec[];
    panels: PanelSpec[];
    store: ChannelStore;
  },
) {
  return (
    <table className={styles.table}>
      <thead>
        <tr>
          <th>channel</th>
          <th>encoding</th>
          <th>Hz</th>
          <th>age</th>
          <th>seq</th>
          <th>value</th>
        </tr>
      </thead>
      <tbody>
        {channels.map((spec) => (
          <ChannelRow key={spec.ch} spec={spec} panels={panels} store={store} />
        ))}
      </tbody>
    </table>
  );
}
