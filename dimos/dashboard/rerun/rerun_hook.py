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
            print(f'''[RerunHook] got {val}''')
            # convert to rerun value
            if hasattr(val, "to_rerun") and callable(val.to_rerun):
                val = val.to_rerun()
            rr.log(target_entity, val)
    
    if rate_limit != None:
        publish_callback = rate_limiter(rate_limit)(publish_callback)
    
    # FIXME: something really weird is stopping vars from being defined inside the In[input_type] scope  (global()[] is a workaround)
    globals()["input_type"] = input_type
    globals()["publish_callback"] = publish_callback
    print(f'''input_type = {input_type}''')
    exec(f"""
    class RerunHookModule(Module):
        {attrname}: In[globals()["input_type"]] = None
        
        @rpc
        def start(self) -> None:
            self._disposables.add(Disposable(
                self.{attrname}.subscribe(globals()["publish_callback"])
            ))
    globals()["RerunHookModule"] = RerunHookModule
    """.replace('\n    ', '\n'))
    return RerunHookModule