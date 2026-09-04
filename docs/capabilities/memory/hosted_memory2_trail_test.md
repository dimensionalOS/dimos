# PostgreSQL Native Logical Replication Demo

## Purpose

The goal is to demonstrate:

- Local PostgreSQL as a publisher
- AWS RDS PostgreSQL as a subscriber
- PostgreSQL native logical replication
- Initial synchronization of existing data
- Automatic propagation of INSERT/UPDATE/DELETE operations
- The networking and privilege requirements involved

This is a standalone experiment using an artificial `sync_test` table. No DimensionalOS replay engine or full memory store is required.

---

# 1. Architecture

The intended architecture is:

```text
                 Native PostgreSQL Logical Replication

┌───────────────────────────────┐
│       Local Mac               │
│                               │
│ Docker                        │
│ ┌───────────────────────────┐ │
│ │ PostgreSQL 17.11          │ │
│ │                           │ │
│ │ Database: dimensionalos   │ │
│ │                           │ │
│ │ sync_test                 │ │
│ │ 66 existing rows          │ │
│ │                           │ │
│ │ Publication: dimos_pub   │ │
│ └─────────────┬─────────────┘ │
└───────────────┼───────────────┘
                │
                │ PostgreSQL Logical Replication
                │
                ▼
         TCP Tunnel
          ngrok
                │
                ▼
┌───────────────────────────────┐
│        AWS RDS                │
│                               │
│ PostgreSQL                    │
│ Database: shared              │
│                               │
│ sync_test                     │
│ Subscription: dimos_sub       │
└───────────────────────────────┘
````

---

# 2. Local PostgreSQL

The local PostgreSQL instance is running inside Docker.

Container:

```text
dimensio-psql
```

Docker port mapping:

```text
5432/tcp -> 0.0.0.0:5433
5432/tcp -> [::]:5433
```

Therefore:

```text
Mac localhost:5433
        ↓
Docker PostgreSQL:5432
```

The database is:

```text
dimensionalos
```

The local PostgreSQL version is:

```text
PostgreSQL 17.11 (Debian 17.11-1.pgdg13+2)
on aarch64-unknown-linux-gnu
```

Verified with:

```bash
docker exec dimensio-psql psql -U postgres -d dimensionalos \
  -c "SELECT version();"
```

---

# 3. Test Data

A test table named `sync_test` was created in the local PostgreSQL database.

The table contains:

```text
id
timestamp
robot_id
value
image
```

The table contains artificial test data.

At the time of the experiment:

```text
Total rows: 66
Rows with image: 10
Rows without image: 56
```

Verified using:

```sql
SELECT
    COUNT(*) AS total,
    COUNT(image) AS with_image,
    COUNT(*) - COUNT(image) AS without_image
FROM sync_test;
```

Result:

```text
 total | with_image | without_image
-------+------------+---------------
    66 |         10 |            56
```

Example data:

```text
id | timestamp                    | robot_id  | value
---+------------------------------+-----------+-------------------
1  | 2026-08-14 18:45:08.758181   | robot-001 | 0.541647661655267
2  | 2026-08-14 18:45:09.768197   | robot-001 | 0.46496265474810117
3  | 2026-08-14 18:45:10.776587   | robot-001 | 0.16588049960011386
...
```

---

# 4. Enabling PostgreSQL Logical Replication

Initially, the local PostgreSQL instance had:

```text
wal_level = replica
```

Verified using:

```bash
docker exec dimensio-psql psql -U postgres -d dimensionalos \
  -c "SHOW wal_level;"
```

For logical replication, PostgreSQL requires:

```text
wal_level = logical
```

The configuration was changed using:

```sql
ALTER SYSTEM SET wal_level = logical;
```

Then the PostgreSQL container was restarted:

```bash
docker restart dimensio-psql
```

After the restart:

```bash
docker exec dimensio-psql psql -U postgres -d dimensionalos \
  -c "SHOW wal_level;"
```

returned:

```text
 wal_level
-----------
 logical
```

The existing data was preserved.

Verification:

```bash
docker exec dimensio-psql psql -U postgres -d dimensionalos \
  -c "SELECT COUNT(*) FROM sync_test;"
```

Result:

```text
 count
-------
    66
```

---

# 5. Creating the PostgreSQL Publication

A PostgreSQL publication was created on the local database:

```sql
CREATE PUBLICATION dimos_pub
FOR TABLE sync_test;
```

The command returned:

```text
CREATE PUBLICATION
```

The publication was verified with:

```sql
\dRp+
```

Result:

```text
Publication dimos_pub

Owner    | All tables | Inserts | Updates | Deletes | Truncates | Via root
---------+------------+---------+---------+---------+-----------+---------
postgres | f          | t       | t       | t       | t         | f

Tables:
    "public.sync_test"
```

Therefore the publication includes:

```text
sync_test
```

and publishes:

```text
INSERT
UPDATE
DELETE
TRUNCATE
```

---

# 6. AWS RDS PostgreSQL

The destination PostgreSQL instance is an AWS RDS database.

Connection:

```text
postgresql://guest:<password>@la-psql.cp6sy04ai5l0.us-west-1.rds.amazonaws.com:5432/shared?sslmode=require
```

Database:

```text
shared
```

The initial RDS database did not contain the `sync_test` table.

A matching table was subsequently created:

```sql
CREATE TABLE sync_test (
    id BIGSERIAL PRIMARY KEY,
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    robot_id TEXT NOT NULL,
    value DOUBLE PRECISION,
    image BYTEA
);
```

The cloud table was intentionally created with the same schema as the local table.

---

# 7. Enabling Logical Replication on RDS

Initially, RDS reported:

```sql
SHOW rds.logical_replication;
```

Result:

```text
 rds.logical_replication
-------------------------
 off
```

Logical replication was subsequently enabled on the RDS instance.

After enabling the RDS configuration, the expected value is:

```text
 rds.logical_replication
-------------------------
 on
```

This is required for the RDS instance to participate in logical replication.

---

# 8. Networking Problem

Native PostgreSQL logical replication requires the subscriber to connect to the publisher.

The desired direction is:

```text
Local PostgreSQL
      ↓
AWS RDS PostgreSQL
```

In PostgreSQL logical replication terminology:

```text
Local PostgreSQL = Publisher
AWS RDS = Subscriber
```

The problem is that the local PostgreSQL instance is running on a developer laptop:

```text
localhost:5433
```

AWS RDS cannot directly connect to:

```text
localhost:5433
```

because `localhost` from AWS refers to the AWS machine itself, not the developer laptop.

---

# 9. ngrok TCP Tunnel

To make the local PostgreSQL temporarily reachable from AWS, an ngrok TCP tunnel was used.

ngrok was installed using Homebrew:

```bash
brew install ngrok
```

The local PostgreSQL port is:

```text
5433
```

The tunnel was started with:

```bash
ngrok tcp 5433
```

The active tunnel endpoint became:

```text
0.tcp.us-cal-1.ngrok.io:20984
```

with:

```text
0.tcp.us-cal-1.ngrok.io:20984
    ↓
localhost:5433
```

Therefore the networking path becomes:

```text
AWS RDS
    ↓
0.tcp.us-cal-1.ngrok.io:20984
    ↓
ngrok
    ↓
Mac localhost:5433
    ↓
Docker PostgreSQL:5432
```

The ngrok session must remain running for replication to remain connected.

---

# 10. Local PostgreSQL Authentication

The local PostgreSQL `pg_hba.conf` was inspected:

```bash
docker exec dimensio-psql cat /var/lib/postgresql/data/pg_hba.conf
```

The relevant authentication rule is:

```text
host all all all scram-sha-256
```

Therefore remote TCP connections using password authentication are allowed.

No additional `pg_hba.conf` rule was required for this demo.

---

# 11. Testing the ngrok PostgreSQL Connection

The intended connection test is:

```bash
psql \
  "host=0.tcp.us-cal-1.ngrok.io \
   port=20984 \
   user=postgres \
   password=postgres \
   dbname=dimensionalos"
```

A successful connection should produce:

```text
dimensionalos=#
```

A simple connectivity test can also be performed with:

```bash
psql \
  "host=0.tcp.us-cal-1.ngrok.io \
   port=20984 \
   user=postgres \
   password=postgres \
   dbname=dimensionalos" \
  -c "SELECT 1;"
```

Expected:

```text
 ?column?
----------
        1
(1 row)
```

---

# 12. Attempting to Create the RDS Subscription

The next step is to create a PostgreSQL subscription on RDS.

The intended command is:

```sql
CREATE SUBSCRIPTION dimos_sub
CONNECTION 'host=0.tcp.us-cal-1.ngrok.io port=20984 dbname=dimensionalos user=postgres password=postgres'
PUBLICATION dimos_pub;
```

This tells RDS:

```text
Subscribe to publication:
    dimos_pub

Publisher:
    0.tcp.us-cal-1.ngrok.io:20984

Database:
    dimensionalos

User:
    postgres
```

However, the first attempt failed with:

```text
ERROR: permission denied to create subscription
DETAIL: Only roles with privileges of the "pg_create_subscription" role may create subscriptions.
```

The RDS connection is currently being performed as:

```text
guest
```

Therefore the `guest` role does not currently have the privilege required to create a subscription.

---

# 13. Required RDS Privilege

The RDS administrator needs to grant:

```sql
GRANT pg_create_subscription TO guest;
```

This must be performed by a sufficiently privileged RDS administrative account.

After the privilege was granted, reconnected as `guest` and retried:

```sql
CREATE SUBSCRIPTION dimos_sub
CONNECTION 'host=0.tcp.us-cal-1.ngrok.io port=20984 dbname=dimensionalos user=postgres password=postgres'
PUBLICATION dimos_pub;
```

---

# 14. Expected Initial Synchronization

Once the subscription is successfully created, PostgreSQL should be able to perform an initial table synchronization.

Current state before subscription:

```text
LOCAL                         RDS
--------------------------    --------------------------
sync_test                     sync_test
66 rows                       0 rows
dimos_pub                     dimos_sub
```

After the subscription performs the initial copy:

```text
LOCAL                         RDS
--------------------------    --------------------------
sync_test                     sync_test
66 rows                       66 rows
dimos_pub                     dimos_sub
```

The cloud database can be checked using:

```sql
SELECT COUNT(*) FROM sync_test;
```

Expected and Actual:

```text
 count
-------
    66
```

---

# 15. Live INSERT Test

After the subscription is active, the most important test is to insert data only into the local database.

Connect to local PostgreSQL:

```bash
docker exec -it dimensio-psql psql \
  -U postgres \
  -d dimensionalos
```

Run:

```sql
INSERT INTO sync_test (robot_id, value)
VALUES ('PG-REPLICATION-DEMO', 12345.678);
```

Then check the RDS database:

```sql
SELECT id, timestamp, robot_id, value
FROM sync_test
WHERE robot_id = 'PG-REPLICATION-DEMO';
```

Expected and Actual:

```text
The row appears on RDS without manually executing an INSERT on RDS.
```

This demonstrates PostgreSQL native logical replication.

---

# 16. Live UPDATE Test

On the local PostgreSQL:

```sql
UPDATE sync_test
SET value = 99999.999
WHERE robot_id = 'PG-REPLICATION-DEMO';
```

On RDS:

```sql
SELECT robot_id, value
FROM sync_test
WHERE robot_id = 'PG-REPLICATION-DEMO';
```

Expected and Actual:

```text
PG-REPLICATION-DEMO | 99999.999
```

---

# 17. Live DELETE Test

On the local PostgreSQL:

```sql
DELETE FROM sync_test
WHERE robot_id = 'PG-REPLICATION-DEMO';
```

On RDS:

```sql
SELECT *
FROM sync_test
WHERE robot_id = 'PG-REPLICATION-DEMO';
```

Expected and Actual:

```text
(0 rows)
```

This demonstrates that DELETE operations are also replicated.

---

# 18. What This Demonstrates

The experiment demonstrates that PostgreSQL can provide native database-to-database replication using:

```text
WAL
 ↓
Logical decoding
 ↓
Publication
 ↓
Subscription
 ↓
Subscriber PostgreSQL
```

The application does not need to:

* Read every changed row manually
* Build a custom change queue
* Replay application operations
* Implement INSERT/UPDATE/DELETE synchronization logic
* Implement its own database change tracking

PostgreSQL provides the replication mechanism itself.

---

# 19. Native PostgreSQL Logical Replication Architecture

Conceptually:

```text
                         LOCAL
                PostgreSQL Publisher
                       :5433
                         │
                         │
                    WAL records
                         │
                         ▼
                 Logical decoding
                         │
                         ▼
                    Publication
                    "dimos_pub"
                         │
                         │
                         │ TCP connection
                         │
                         ▼
                      ngrok
                         │
                         ▼
                   AWS RDS Subscriber
                         │
                    Subscription
                    "dimos_sub"
                         │
                         ▼
                     sync_test
```

The subscription worker on the subscriber connects to the publisher and receives the changes represented by the publication.

---

# 20. Failure Recovery and Offline Synchronization Test

Tested the behavior of PostgreSQL native logical replication when the publisher becomes temporarily unreachable.

This was important for the DimensionalOS use case because the publisher represents a robot/edge device that may:

- Temporarily lose network connectivity
- Continue generating data while offline
- Lose power and restart
- Reconnect to the cloud later

The test verified whether PostgreSQL could recover automatically and synchronize the changes generated during the outage.

---

## 23.1 Test Architecture

The tested setup was:

```text
┌──────────────────────────────┐
│ Local Mac / Robot            │
│                              │
│ PostgreSQL 17                │
│ Database: dimensionalos      │
│                              │
│ sync_test                    │
│ Publication: dimos_pub       │
└──────────────┬───────────────┘
               │
               │ PostgreSQL
               │ Logical Replication
               ▼
            ngrok
               │
               ▼
┌──────────────────────────────┐
│ AWS RDS PostgreSQL            │
│                              │
│ Database: shared             │
│ sync_test                    │
│ Subscription: dimos_sub      │
└──────────────────────────────┘
````

The local PostgreSQL acted as the **publisher**, while the AWS RDS PostgreSQL instance acted as the **subscriber**.

---

# 23.2 Baseline Before Failure

Before starting the failure tests, we verified that logical replication was working correctly.

The local database contained the test data:

```text
sync_test rows: 66
```

The same data was present on RDS after the initial synchronization.

I also verified that:

```sql
SELECT subname, subenabled
FROM pg_subscription;
```

showed the subscription as enabled.

The publisher had a logical replication slot associated with the subscription.

At this point the replication path was:

```text
Local PostgreSQL
      │
      ▼
dimos_pub
      │
      ▼
ngrok
      │
      ▼
dimos_sub
      │
      ▼
AWS RDS
```

I also verified normal live replication by inserting, updating, and deleting records locally and observing the changes automatically appear on RDS.

---

# 23.3 Network Failure Test

I then simulated a network outage by stopping the ngrok TCP tunnel while leaving the local PostgreSQL instance running.

Before stopping the tunnel, replication was healthy.

The network path became:

```text
Local PostgreSQL
      │
      X
      │
    ngrok
      X
      │
    AWS RDS
```

The local PostgreSQL itself remained fully operational.

---

## 23.3.1 Writes During Network Outage

While the network connection was unavailable, I continued writing data to the local PostgreSQL database.

For example:

```sql
INSERT INTO sync_test (robot_id, value)
VALUES
    ('OFFLINE-1', 111),
    ('OFFLINE-2', 222),
    ('OFFLINE-3', 333),
    ('OFFLINE-4', 444);
```

The records were immediately available on the local database.

I verified:

```sql
SELECT robot_id, value
FROM sync_test
WHERE robot_id LIKE 'OFFLINE-%'
ORDER BY id;
```

The local database contained all four records.

```
 robot_id  | value 
-----------+-------
 OFFLINE-1 |   111
 OFFLINE-2 |   222
 OFFLINE-3 |   333
 OFFLINE-4 |   444
```

---

## 23.3.2 Cloud Behavior During Network Outage

I queried the RDS database while the network connection was still unavailable:

```sql
SELECT robot_id, value
FROM sync_test
WHERE robot_id LIKE 'OFFLINE-%'
ORDER BY id;
```

The new records were not present on RDS.

```
 robot_id | value 
----------+-------
(0 rows)
```

This was expected because the subscriber could not communicate with the publisher.

Importantly, the records were **not lost from the publisher**.

They remained committed in the local PostgreSQL database.

---

# 23.4 Replication Slot Behavior

I inspected the logical replication slot on the local PostgreSQL:

```sql
SELECT
    slot_name,
    slot_type,
    active,
    restart_lsn,
    confirmed_flush_lsn
FROM pg_replication_slots;
```

```
slot_name | slot_type | active | restart_lsn | confirmed_flush_lsn 
-----------+-----------+--------+-------------+---------------------
 dimos_sub | logical   | f      | 0/1DCC710   | 0/1DCC748
(1 row)
```

The logical replication slot remained associated with the subscription.

I also checked the amount of WAL retained by the slot:

```sql
SELECT
    slot_name,
    active,
    pg_size_pretty(
        pg_wal_lsn_diff(
            pg_current_wal_lsn(),
            restart_lsn
        )
    ) AS retained_wal
FROM pg_replication_slots;
```

```
 slot_name | active | retained_wal 
-----------+--------+--------------
 dimos_sub | f      | 7504 bytes
(1 row)
```

This demonstrated an important property of PostgreSQL logical replication:

> The publisher retains WAL required by the replication slot until the subscriber has consumed the changes.

Therefore, when the subscriber is temporarily disconnected, PostgreSQL does not simply discard the changes that still need to be replicated.

Conceptually:

```text
Network outage
      │
      ▼
Subscriber cannot consume WAL
      │
      ▼
Replication slot retains required WAL
      │
      ▼
Network restored
      │
      ▼
Subscriber catches up
```

---

# 23.5 Network Recovery

I restarted the ngrok TCP tunnel after the simulated outage.

Because ngrok's free TCP endpoint can change between sessions, the endpoint changed when the tunnel was restarted.

I updated the subscription connection string to point to the new ngrok endpoint.

After connectivity was restored, the PostgreSQL subscriber reconnected to the publisher.

The previously generated offline records were then replicated to RDS.

I verified:

```sql
SELECT robot_id, value
FROM sync_test
WHERE robot_id LIKE 'OFFLINE-%'
ORDER BY id;
```

The RDS database eventually contained:

```text
OFFLINE-1 | 111
OFFLINE-2 | 222
OFFLINE-3 | 333
OFFLINE-4 | 444
```

This confirmed that PostgreSQL successfully caught up with the changes generated while the network was unavailable.

---

# 23.6 Result of Network Failure Test

The observed behavior was:

```text
                Network Available

Local PostgreSQL
       │
       ├── INSERT
       │
       ▼
   Publication
       │
       ▼
      RDS
```

When the network was interrupted:

```text
                Network Unavailable

Local PostgreSQL
       │
       ├── INSERT
       ├── INSERT
       ├── INSERT
       │
       ▼
   Local WAL
       │
       │
       X────────────── RDS
```

After network connectivity was restored:

```text
Local PostgreSQL
       │
       │ previously unconsumed changes
       ▼
   Replication
       │
       ▼
      RDS
       │
       ▼
    Catches up
```

### Result

**PostgreSQL native logical replication successfully recovered from the temporary network outage and synchronized the changes generated during the outage.**

---

# 23.7 Robot / PostgreSQL Shutdown Test

I also tested the behavior when the local PostgreSQL instance itself was stopped.

This simulated a robot losing power.

The local PostgreSQL container was stopped using:

```bash
docker stop dimensio-psql
```

At this point:

```text
Robot / Publisher
       │
       X
       │
     OFFLINE
```

The RDS subscriber remained configured with the existing subscription.

During this period, the publisher was unavailable.

---

# 23.8 PostgreSQL Restart

The local PostgreSQL instance was started again:

```bash
docker start dimensio-psql
```

I verified that PostgreSQL came back successfully:

```bash
docker exec dimensio-psql psql \
    -U postgres \
    -d dimensionalos \
    -c "SELECT version();"
```

I also verified that logical replication was still configured:

```bash
docker exec dimensio-psql psql \
    -U postgres \
    -d dimensionalos \
    -c "SHOW wal_level;"
```

The result remained:

```text
wal_level
-----------
logical
```

The publication and replication configuration were preserved across the restart.

---

# 23.9 Replication Recovery After PostgreSQL Restart

After restoring network connectivity, the subscription was able to reconnect to the publisher.

I verified that the local and RDS databases eventually converged.

Local:

```sql
SELECT COUNT(*) FROM sync_test;
```

RDS:

```sql
SELECT COUNT(*) FROM sync_test;
```

The counts matched again.

I then inserted a new record locally:

```sql
INSERT INTO sync_test (robot_id, value)
VALUES ('AFTER-RESTART', 999);
```

The record subsequently appeared on RDS without manually inserting it there.

This confirmed that logical replication continued to function after the publisher restart.

---

# 23.10 Observed Failure Recovery Behavior

The tests demonstrated the following behavior:

| Scenario             | Local PostgreSQL | RDS          | Result                                     |
| -------------------- | ---------------- | ------------ | ------------------------------------------ |
| Normal operation     | Running          | Connected    | Changes replicated immediately             |
| Network unavailable  | Running          | Disconnected | Local writes continue                      |
| Network unavailable  | Running          | Disconnected | New changes not immediately visible on RDS |
| Network restored     | Running          | Reconnected  | RDS catches up                             |
| PostgreSQL stopped   | Offline          | Running      | Publisher unavailable                      |
| PostgreSQL restarted | Running          | Reconnects   | Replication resumes                        |
| Post-restart writes  | Running          | Connected    | Changes replicate normally                 |

---

# 23.11 Important Observation — Replication Is Not an Infinite Offline Queue

The test also highlighted an important limitation of native PostgreSQL logical replication.

While the subscriber is disconnected, the publisher's logical replication slot retains the WAL required by the subscriber.

Therefore:

```text
Longer outage
      │
      ▼
More local writes
      │
      ▼
More WAL retained
      │
      ▼
More local disk consumption
```

I monitored this using:

```sql
SELECT
    slot_name,
    active,
    pg_size_pretty(
        pg_wal_lsn_diff(
            pg_current_wal_lsn(),
            restart_lsn
        )
    ) AS retained_wal
FROM pg_replication_slots;
```

This means native PostgreSQL replication can tolerate temporary outages, but it should not be treated as an unlimited offline queue.

If a robot remains offline for a sufficiently long period and continues generating a high volume of data, retained WAL can grow substantially and eventually create disk-pressure problems.

---

# 23.12 Overall Result

The failure tests established that PostgreSQL native logical replication provides **automatic recovery from temporary connectivity and publisher failures**.

The tested behavior was:

```text
              Normal
                │
                ▼
         Replication Active
                │
                │
         Network Failure
                │
                ▼
      Local writes continue
                │
                ▼
       WAL retained locally
                │
                │
       Network / Robot returns
                │
                ▼
       Replication reconnects
                │
                ▼
        Subscriber catches up
                │
                ▼
        Databases converge
```

Therefore, for a PostgreSQL-to-PostgreSQL use case, native logical replication already provides a significant amount of the functionality that a custom synchronization mechanism would otherwise need to implement.

---
