import type { Static } from "typebox";
import { Type } from "typebox";

/** What a cut removed, kept in the tool result's details or the input's entry. */
export const LimitRecordSchema = Type.Object({
  shown: Type.Integer({ minimum: 0 }),
  total: Type.Integer({ minimum: 0 }),
  /** The file that holds the whole text. */
  fullText: Type.String(),
  maxChars: Type.Integer({ minimum: 1 }),
  /** The requested limit, when the remaining context forced a smaller one. */
  clampedFrom: Type.Union([Type.Integer({ minimum: 1 }), Type.Null()]),
});

export type LimitRecord = Static<typeof LimitRecordSchema>;
