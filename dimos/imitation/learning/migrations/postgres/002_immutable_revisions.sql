CREATE FUNCTION reject_revision_mutation() RETURNS trigger LANGUAGE plpgsql AS $$
BEGIN
    IF OLD.kind = 'revision' THEN
        RAISE EXCEPTION 'immutable annotation revision';
    END IF;
    IF TG_OP = 'DELETE' THEN RETURN OLD; END IF;
    RETURN NEW;
END;
$$;
CREATE TRIGGER immutable_revision BEFORE UPDATE OR DELETE ON documents
    FOR EACH ROW EXECUTE FUNCTION reject_revision_mutation();
