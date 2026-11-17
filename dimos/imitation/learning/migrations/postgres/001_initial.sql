CREATE TABLE documents (
    rowid BIGSERIAL UNIQUE NOT NULL,
    kind TEXT NOT NULL,
    id TEXT NOT NULL,
    body TEXT NOT NULL CHECK (jsonb_typeof(body::jsonb) = 'object'),
    PRIMARY KEY(kind, id)
);
