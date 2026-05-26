import type { RouterClient } from "@orpc/server";
import { messages, user } from "@robomoo/db";
import {
  createMessageInput,
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

export const appRouter = {
  messages: { list, add },
};

export type AppRouter = typeof appRouter;
export type AppRouterClient = RouterClient<typeof appRouter>;
