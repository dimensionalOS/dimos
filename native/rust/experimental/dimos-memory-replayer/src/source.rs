// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use std::io::Read;

use anyhow::{anyhow, Context, Result};
use lcm_msgs::sensor_msgs::Image;
use lz4_flex::frame::FrameDecoder;
use rusqlite::{params_from_iter, Connection, OptionalExtension};
use serde::Deserialize;
use tokio::sync::mpsc;
use turbojpeg::PixelFormat;

use crate::{ReplayConfig, StreamConfig};

const READ_AHEAD: usize = 1;

#[derive(Clone, Debug)]
pub(crate) struct Observation {
    pub(crate) stream_index: usize,
    pub(crate) ts: f64,
    pub(crate) data: Vec<u8>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Codec {
    Lcm,
    Jpeg,
    Lz4Lcm,
}

impl Codec {
    fn parse(value: &str) -> Result<Self> {
        match value {
            "lcm" => Ok(Self::Lcm),
            "jpeg" => Ok(Self::Jpeg),
            "lz4+lcm" => Ok(Self::Lz4Lcm),
            other => Err(anyhow!("unsupported replay codec {other:?}")),
        }
    }
}

#[derive(Deserialize)]
struct RegistryConfig {
    payload_module: String,
    codec_id: String,
}

#[derive(Clone, Debug)]
pub(crate) struct SqliteReplaySource {
    path: String,
    sql: String,
    parameters: Vec<f64>,
    codecs: Vec<Codec>,
    start: Option<f64>,
    end: Option<f64>,
    first_timestamp: Option<f64>,
}

impl SqliteReplaySource {
    pub(crate) fn prepare(config: &ReplayConfig) -> Result<Self> {
        let path = &config.store.path;
        let connection = open(path)?;
        let codecs = validate_streams(&connection, path, &config.streams)?;
        let recording_first = query_min(&connection, &min_sql(&config.streams, ""), &[])?;
        let start = config.from_timestamp.value().or_else(|| {
            config
                .seek
                .value()
                .and_then(|seek| recording_first.map(|first| first + seek))
        });
        let end = config.duration.value().and_then(|duration| {
            start
                .or(recording_first)
                .map(|window_start| window_start + duration)
        });
        let (min_bounds, parameters) = bounds_sql("", start, end);
        let (observation_bounds, _) = bounds_sql("stream.", start, end);
        let sql = observations_sql(&config.streams, &observation_bounds);

        // Preparing catches missing observation/blob tables before any payload is published.
        connection
            .prepare(&sql)
            .context("invalid SQLite replay schema")?;
        let first_timestamp = query_min(
            &connection,
            &min_sql(&config.streams, &min_bounds),
            &parameters,
        )?;

        Ok(Self {
            path: path.clone(),
            sql,
            parameters,
            codecs,
            start,
            end,
            first_timestamp,
        })
    }

    pub(crate) fn first_timestamp(&self) -> Option<f64> {
        self.first_timestamp
    }

    pub(crate) fn window(&self) -> (Option<f64>, Option<f64>) {
        (self.start, self.end)
    }

    pub(crate) fn stream(&self) -> mpsc::Receiver<Result<Observation>> {
        let source = self.clone();
        let (sender, receiver) = mpsc::channel(READ_AHEAD);
        tokio::task::spawn_blocking(move || {
            let result = source.read(|observation| {
                sender
                    .blocking_send(Ok(observation))
                    .map_err(|_| anyhow!("replay stopped"))
            });
            if let Err(error) = result {
                let _ = sender.blocking_send(Err(error));
            }
        });
        receiver
    }

    fn read(&self, mut emit: impl FnMut(Observation) -> Result<()>) -> Result<()> {
        let connection = open(&self.path)?;
        let mut statement = connection.prepare(&self.sql)?;
        let mut rows = statement.query(params_from_iter(self.parameters.iter()))?;
        while let Some(row) = rows.next()? {
            let stream_index: usize = row
                .get::<_, i64>(0)?
                .try_into()
                .context("negative replay stream index")?;
            let sequence = row.get::<_, i64>(1)?;
            let ts = row.get::<_, f64>(2)?;
            let stored = row.get::<_, Vec<u8>>(3)?;
            let codec = *self
                .codecs
                .get(stream_index)
                .ok_or_else(|| anyhow!("invalid replay stream index {stream_index}"))?;
            let data = decode(codec, stored).with_context(|| {
                format!("failed to decode stream {stream_index} observation {sequence}")
            })?;
            emit(Observation {
                stream_index,
                ts,
                data,
            })?;
        }
        Ok(())
    }
}

fn open(path: &str) -> Result<Connection> {
    Connection::open_with_flags(
        path,
        rusqlite::OpenFlags::SQLITE_OPEN_READ_ONLY | rusqlite::OpenFlags::SQLITE_OPEN_NO_MUTEX,
    )
    .with_context(|| format!("failed to open replay database {path}"))
}

fn validate_streams(
    connection: &Connection,
    path: &str,
    streams: &[StreamConfig],
) -> Result<Vec<Codec>> {
    streams
        .iter()
        .map(|stream| {
            validate_identifier(&stream.name)?;
            let serialized: String = connection
                .query_row(
                    "SELECT config FROM _streams WHERE name = ?1",
                    [&stream.name],
                    |row| row.get(0),
                )
                .with_context(|| format!("stream {:?} is not registered in {path}", stream.name))?;
            let metadata: RegistryConfig =
                serde_json::from_str(&serialized.replace("dimos.memory2.", "dimos.memory."))
                    .with_context(|| {
                        format!("invalid registry metadata for stream {:?}", stream.name)
                    })?;
            validate_payload(stream, &metadata.payload_module)?;
            validate_timestamp_index(connection, &stream.name)?;
            Codec::parse(&metadata.codec_id)
        })
        .collect()
}

fn validate_timestamp_index(connection: &Connection, stream: &str) -> Result<()> {
    let mut statement = connection.prepare("SELECT name FROM pragma_index_list(?1)")?;
    let indexes = statement
        .query_map([stream], |row| row.get::<_, String>(0))?
        .collect::<rusqlite::Result<Vec<_>>>()?;
    for index in indexes {
        let leading: Option<String> = connection
            .query_row(
                "SELECT name FROM pragma_index_info(?1) WHERE seqno = 0",
                [&index],
                |row| row.get(0),
            )
            .optional()?;
        if leading.as_deref() == Some("ts") {
            return Ok(());
        }
    }
    Err(anyhow!(
        "stream {stream:?} has no index beginning with ts; create an index on ({stream}.ts, {stream}.id)"
    ))
}

fn bounds_sql(prefix: &str, start: Option<f64>, end: Option<f64>) -> (String, Vec<f64>) {
    match (start, end) {
        (Some(start), Some(end)) => (
            format!(" WHERE {prefix}ts >= ?1 AND {prefix}ts <= ?2"),
            vec![start, end],
        ),
        (Some(start), None) => (format!(" WHERE {prefix}ts >= ?1"), vec![start]),
        (None, Some(end)) => (format!(" WHERE {prefix}ts <= ?1"), vec![end]),
        (None, None) => (String::new(), Vec::new()),
    }
}

fn min_sql(streams: &[StreamConfig], bounds: &str) -> String {
    let branches = streams
        .iter()
        .map(|stream| format!(r#"SELECT MIN(ts) AS ts FROM "{}"{bounds}"#, stream.name))
        .collect::<Vec<_>>()
        .join(" UNION ALL ");
    format!("SELECT MIN(ts) FROM ({branches})")
}

fn query_min(connection: &Connection, sql: &str, parameters: &[f64]) -> Result<Option<f64>> {
    connection
        .query_row(sql, params_from_iter(parameters.iter()), |row| row.get(0))
        .context("failed to resolve SQLite replay window")
}

fn observations_sql(streams: &[StreamConfig], bounds: &str) -> String {
    let branches = streams
        .iter()
        .enumerate()
        .map(|(stream_index, stream)| {
            format!(
                r#"SELECT {stream_index} AS stream_index,
                          stream.id AS sequence,
                          stream.ts AS ts,
                          blob.data AS data
                   FROM "{name}" AS stream
                   JOIN "{name}_blob" AS blob ON blob.id = stream.id
                   {bounds}"#,
                name = stream.name,
            )
        })
        .collect::<Vec<_>>()
        .join(" UNION ALL ");
    format!("{branches} ORDER BY ts, stream_index, sequence")
}

fn validate_payload(stream: &StreamConfig, actual: &str) -> Result<()> {
    if stream.payload_type == actual {
        Ok(())
    } else {
        Err(anyhow!(
            "stream {:?} contains {actual:?}, expected {:?}",
            stream.name,
            stream.payload_type
        ))
    }
}

fn decode(codec: Codec, data: Vec<u8>) -> Result<Vec<u8>> {
    match codec {
        Codec::Lcm => Ok(data),
        Codec::Lz4Lcm => {
            let mut decoder = FrameDecoder::new(data.as_slice());
            let mut decoded = Vec::new();
            decoder
                .read_to_end(&mut decoded)
                .context("invalid LZ4 replay payload")?;
            Ok(decoded)
        }
        Codec::Jpeg => decode_jpeg(&data),
    }
}

fn decode_jpeg(data: &[u8]) -> Result<Vec<u8>> {
    let mut message = Image::decode(data).context("invalid stored LCM Image")?;
    if message.encoding != "jpeg" {
        return Err(anyhow!(
            "JPEG storage payload has encoding {:?}",
            message.encoding
        ));
    }
    let image = turbojpeg::decompress(&message.data, PixelFormat::RGB)
        .context("TurboJPEG replay decompression failed")?;
    message.height = image
        .height
        .try_into()
        .context("image height exceeds i32")?;
    message.width = image.width.try_into().context("image width exceeds i32")?;
    message.encoding = "rgb8".to_string();
    message.is_bigendian = 0;
    message.step = (image.width * PixelFormat::RGB.size())
        .try_into()
        .context("image row exceeds i32")?;
    message.data = image.pixels;
    Ok(message.encode())
}

fn validate_identifier(name: &str) -> Result<()> {
    let mut characters = name.chars();
    let first = characters
        .next()
        .ok_or_else(|| anyhow!("stream name is empty"))?;
    if !(first == '_' || first.is_ascii_alphabetic())
        || !characters.all(|value| value == '_' || value.is_ascii_alphanumeric())
    {
        return Err(anyhow!("invalid stream name {name:?}"));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{NullableF64, ReplayStoreConfig};
    use rusqlite::params;
    use std::io::Write;

    fn replay_config(path: &str, streams: Vec<StreamConfig>) -> ReplayConfig {
        ReplayConfig {
            store: ReplayStoreConfig {
                path: path.to_string(),
            },
            speed: 1.0,
            seek: NullableF64::Null,
            duration: NullableF64::Null,
            from_timestamp: NullableF64::Null,
            r#loop: false,
            streams,
        }
    }

    fn stream(name: &str, payload_type: &str) -> StreamConfig {
        StreamConfig {
            port: name.to_string(),
            name: name.to_string(),
            payload_type: payload_type.to_string(),
        }
    }

    fn create_stream(connection: &Connection, name: &str, codec: &str) {
        connection
            .execute_batch(&format!(
                r#"
                CREATE TABLE "{name}" (id INTEGER PRIMARY KEY, ts REAL NOT NULL);
                CREATE INDEX "{name}_ts_id" ON "{name}"(ts, id);
                CREATE TABLE "{name}_blob" (id INTEGER PRIMARY KEY, data BLOB NOT NULL);
                "#
            ))
            .unwrap();
        connection
            .execute(
                "INSERT INTO _streams VALUES (?1, ?2)",
                params![
                    name,
                    format!(r#"{{"payload_module":"example.{name}","codec_id":"{codec}"}}"#)
                ],
            )
            .unwrap();
    }

    fn insert(connection: &Connection, name: &str, id: i64, ts: f64, data: &[u8]) {
        connection
            .execute(
                &format!(r#"INSERT INTO "{name}" VALUES (?1, ?2)"#),
                params![id, ts],
            )
            .unwrap();
        connection
            .execute(
                &format!(r#"INSERT INTO "{name}_blob" VALUES (?1, ?2)"#),
                params![id, data],
            )
            .unwrap();
    }

    fn collect(source: &SqliteReplaySource) -> Result<Vec<Observation>> {
        let mut observations = Vec::new();
        source.read(|observation| {
            observations.push(observation);
            Ok(())
        })?;
        Ok(observations)
    }

    #[test]
    fn lz4_payload_round_trips() {
        let expected = b"lcm bytes";
        let mut encoder = lz4_flex::frame::FrameEncoder::new(Vec::new());
        encoder.write_all(expected).unwrap();
        let stored = encoder.finish().unwrap();
        assert_eq!(decode(Codec::Lz4Lcm, stored).unwrap(), expected);
    }

    #[test]
    fn identifiers_are_strict() {
        assert!(validate_identifier("go2_lidar").is_ok());
        assert!(validate_identifier("bad-name").is_err());
    }

    #[test]
    fn sqlite_streams_are_bounded_and_globally_ordered() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let connection = Connection::open(file.path()).unwrap();
        connection
            .execute_batch("CREATE TABLE _streams (name TEXT PRIMARY KEY, config TEXT NOT NULL);")
            .unwrap();
        create_stream(&connection, "imu", "lcm");
        create_stream(&connection, "odom", "lcm");
        insert(&connection, "imu", 1, 10.0, &[1]);
        insert(&connection, "imu", 2, 12.0, &[3]);
        insert(&connection, "odom", 1, 11.0, &[2]);
        insert(&connection, "odom", 2, 12.0, &[4]);
        drop(connection);

        let mut config = replay_config(
            &file.path().to_string_lossy(),
            vec![stream("imu", "example.imu"), stream("odom", "example.odom")],
        );
        config.seek = NullableF64::Value(1.0);
        config.duration = NullableF64::Value(1.0);
        let source = SqliteReplaySource::prepare(&config).unwrap();
        let observations = collect(&source).unwrap();

        assert_eq!(source.window(), (Some(11.0), Some(12.0)));
        assert_eq!(source.first_timestamp(), Some(11.0));
        assert_eq!(
            observations
                .iter()
                .map(|item| (item.ts, item.stream_index, item.data.clone()))
                .collect::<Vec<_>>(),
            vec![(11.0, 1, vec![2]), (12.0, 0, vec![3]), (12.0, 1, vec![4]),]
        );
    }

    #[test]
    fn corrupt_payload_fails_only_when_the_cursor_reaches_it() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let connection = Connection::open(file.path()).unwrap();
        connection
            .execute_batch("CREATE TABLE _streams (name TEXT PRIMARY KEY, config TEXT NOT NULL);")
            .unwrap();
        create_stream(&connection, "imu", "lz4+lcm");
        for (id, ts) in [(1, 10.0), (2, 11.0)] {
            let mut encoder = lz4_flex::frame::FrameEncoder::new(Vec::new());
            encoder.write_all(&[id as u8]).unwrap();
            insert(&connection, "imu", id, ts, &encoder.finish().unwrap());
        }
        insert(&connection, "imu", 3, 12.0, b"not lz4");
        drop(connection);

        let config = replay_config(
            &file.path().to_string_lossy(),
            vec![stream("imu", "example.imu")],
        );
        let source = SqliteReplaySource::prepare(&config).unwrap();
        let mut decoded = Vec::new();
        let error = source
            .read(|observation| {
                decoded.push(observation.data);
                Ok(())
            })
            .unwrap_err();

        assert_eq!(decoded, vec![vec![1], vec![2]]);
        assert!(error.to_string().contains("observation 3"));
    }

    #[test]
    fn corrupt_payload_outside_the_window_is_not_decoded() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let connection = Connection::open(file.path()).unwrap();
        connection
            .execute_batch("CREATE TABLE _streams (name TEXT PRIMARY KEY, config TEXT NOT NULL);")
            .unwrap();
        create_stream(&connection, "imu", "lz4+lcm");
        let mut encoder = lz4_flex::frame::FrameEncoder::new(Vec::new());
        encoder.write_all(b"valid").unwrap();
        insert(&connection, "imu", 1, 10.0, &encoder.finish().unwrap());
        insert(&connection, "imu", 2, 20.0, b"not lz4");
        drop(connection);

        let mut config = replay_config(
            &file.path().to_string_lossy(),
            vec![stream("imu", "example.imu")],
        );
        config.duration = NullableF64::Value(0.0);
        let source = SqliteReplaySource::prepare(&config).unwrap();

        assert_eq!(collect(&source).unwrap()[0].data, b"valid");
    }

    #[test]
    fn sqlite_stream_requires_timestamp_index() {
        let file = tempfile::NamedTempFile::new().unwrap();
        let connection = Connection::open(file.path()).unwrap();
        connection
            .execute_batch(
                r#"
                CREATE TABLE _streams (name TEXT PRIMARY KEY, config TEXT NOT NULL);
                CREATE TABLE imu (id INTEGER PRIMARY KEY, ts REAL NOT NULL);
                CREATE TABLE imu_blob (id INTEGER PRIMARY KEY, data BLOB NOT NULL);
                INSERT INTO _streams VALUES ('imu', '{"payload_module":"example.imu","codec_id":"lcm"}');
                "#,
            )
            .unwrap();
        drop(connection);
        let config = replay_config(
            &file.path().to_string_lossy(),
            vec![stream("imu", "example.imu")],
        );

        let error = SqliteReplaySource::prepare(&config).unwrap_err();

        assert!(error.to_string().contains("index beginning with ts"));
    }
}
