from dimos.core.stream import Out, RemoteIn, RemoteOut, Transport

def dimos_msg_to_rr(msg):
    """Attempt to convert an unknown message to a Rerun object.

    If the object exposes a callable `to_rerun` attribute, it is used; otherwise returns None.
    """
    to_rr = getattr(msg, "to_rerun", None)
    if callable(to_rr):
        return to_rr()
    return None

class BaseModule:
    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        print("Class created:", cls.__name__)

        # self-analyze to create class-values that 
        for each_attr_name in dir(self):
            each_attr = getattr(self, each_attr_name)
            # TODO: i doubt this will actually work (I don't think they're)
            if isinstance(each_attr, Out):
                # if convertable
                if hasattr(each_attr, "to_rerun"):
                    # then add a new output 
