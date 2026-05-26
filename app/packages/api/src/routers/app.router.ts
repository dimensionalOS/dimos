import type { RouterClient } from "@orpc/server";
import { frames, maps, messages, user } from "@robomoo/db";
import {
  createMessageInput,
  type Frame,
  frameSchema,
  type MapSnapshot,
  mapSnapshotSchema,
  type Message,
  messageSchema,
  newId,
} from "@robomoo/shared";
import { desc, eq } from "drizzle-orm";
import { z } from "zod";
import { protectedProcedure, publicProcedure } from "../api";

const list = publicProcedure
  .output(z.array(messageSchema))
  .handler(async ({ context }): Promise<Message[]> => {
    const rows = await context.db
      .select({
        id: messages.id,
        body: messages.body,
        imageKey: messages.imageKey,
        createdAt: messages.createdAt,
        authorName: user.name,
      })
      .from(messages)
      .innerJoin(user, eq(messages.authorId, user.id))
      .orderBy(desc(messages.createdAt))
      .limit(50);

    return Promise.all(
      rows.map(async (r) => ({
        id: r.id,
        body: r.body,
        imageUrl: r.imageKey ? await context.presignGet(r.imageKey) : null,
        authorName: r.authorName,
        createdAt: r.createdAt.toISOString(),
      })),
    );
  });

const add = protectedProcedure
  .input(createMessageInput)
  .output(messageSchema)
  .handler(async ({ context, input }): Promise<Message> => {
    const [row] = await context.db
      .insert(messages)
      .values({
        id: newId("msg"),
        body: input.body,
        imageKey: input.imageKey,
        authorId: context.session.user.id,
      })
      .returning();

    if (!row) {
      throw new Error("insert returned no row");
    }

    return {
      id: row.id,
      body: row.body,
      imageUrl: row.imageKey ? await context.presignGet(row.imageKey) : null,
      authorName: context.session.user.name,
      createdAt: row.createdAt.toISOString(),
    };
  });

// Robot frame gallery — public list, newest first, with presigned image URLs.
const framesList = publicProcedure
  .output(z.array(frameSchema))
  .handler(async ({ context }): Promise<Frame[]> => {
    const rows = await context.db
      .select()
      .from(frames)
      .orderBy(desc(frames.createdAt))
      .limit(100);

    return Promise.all(
      rows.map(async (r) => ({
        id: r.id,
        imageUrl: await context.presignGet(r.imageKey),
        note: r.note,
        label: r.label,
        poseX: r.poseX,
        poseY: r.poseY,
        createdAt: r.createdAt.toISOString(),
      })),
    );
  });

// Newest map snapshot (or null if none yet) with a presigned PNG URL + the grid
// metadata the web needs to place world coordinates onto the image.
const mapLatest = publicProcedure
  .output(mapSnapshotSchema.nullable())
  .handler(async ({ context }): Promise<MapSnapshot | null> => {
    const [row] = await context.db
      .select()
      .from(maps)
      .orderBy(desc(maps.createdAt))
      .limit(1);
    if (!row) return null;
    return {
      imageUrl: await context.presignGet(row.imageKey),
      resolution: row.resolution,
      originX: row.originX,
      originY: row.originY,
      width: row.width,
      height: row.height,
      createdAt: row.createdAt.toISOString(),
    };
  });

export const appRouter = {
  messages: { list, add },
  frames: { list: framesList },
  map: { latest: mapLatest },
};

export type AppRouter = typeof appRouter;
export type AppRouterClient = RouterClient<typeof appRouter>;
