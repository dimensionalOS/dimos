CREATE TRIGGER immutable_revision_update BEFORE UPDATE ON documents WHEN OLD.kind='revision'
BEGIN SELECT RAISE(ABORT,'immutable annotation revision'); END;
CREATE TRIGGER immutable_revision_delete BEFORE DELETE ON documents WHEN OLD.kind='revision'
BEGIN SELECT RAISE(ABORT,'immutable annotation revision'); END;
