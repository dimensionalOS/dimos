import { drizzle } from "drizzle-orm/node-postgres";
import pg from "pg";
import * as schema from "./schema";

export type Database = ReturnType<typeof createDb>;

// One pool per connection string. Cheap to call repeatedly.
const pools = new Map<string, pg.Pool>();

export function createDb(databaseUrl: string) {
  let pool = pools.get(databaseUrl);
  if (!pool) {
    pool = new pg.Pool({ connectionString: databaseUrl });
    pools.set(databaseUrl, pool);
  }
  return drizzle(pool, { schema });
}

export { schema };
export * from "./schema";
