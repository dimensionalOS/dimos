# Pin numbered object selections before picking

The operator and agent select objects by their number in one immutable Detection Snapshot, then invoke `pick_selected` on the pinned selection. The module resolves the number immediately and retains the object's stable identity and geometry, so later detection ordering cannot redirect the transaction; state-changing execution never reinterprets a number against a newer snapshot.
