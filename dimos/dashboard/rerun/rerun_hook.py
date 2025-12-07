import time

import rerun as rr # pip install rerun-sdk
from reactivex.disposable import Disposable

from dimos.core import In, Out, rpc, Module
from dimos.dashboard.support.utils import rate_limit as rate_limiter

# TODO: it'd be nice to show that target_entity is its own class, not just "any" string
# FIXME: allow greater specificity by setting up the topic
def RerunHook(attrname, input_type, target_entity: str, *, topic = None, publish_callback = None, rate_limit = None):
    """
    Example:
        from dimos.dashboard.rerun.hooks import RerunHook
        
        autoconnect(
            RerunHook(Image, "/spatial2d/image").blueprint(),
        )
        
    """
    
    # set default callback
    if publish_callback is None:
        def publish_callback(val: input_type):
            if hasattr(val, "to_rerun") and callable(val.to_rerun):
                val = val.to_rerun()
            rr.log(target_entity, val)
    
    if rate_limit != None:
        publish_callback = rate_limiter(rate_limit)(publish_callback)
    
    def start(self) -> None:
        self._disposables.add(
            Disposable(getattr(self, attrname).subscribe(publish_callback))
        )

    cls_dict = {
        "__annotations__": {attrname: In[input_type]},
        attrname: None,
        "start": rpc(start),
        "__module__": __name__,
    }

    return type(f"RerunHookModule_{input_type.__name__}", (Module,), cls_dict)
