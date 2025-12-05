def dimos_msg_to_rr(msg):
    """Attempt to convert an unknown message to a Rerun object.

    If the object exposes a callable `to_rerun` attribute, it is used; otherwise returns None.
    """
    to_rr = getattr(msg, "to_rerun", None)
    if callable(to_rr):
        return to_rr()
    return None