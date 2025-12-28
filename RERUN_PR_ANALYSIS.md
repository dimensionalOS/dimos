# Rerun PR Comparison: Jeff vs Lesh

## TL;DR - The Core Issue

**Lesh**: Simple, direct rerun integration (~50 lines)
**Jeff**: Over-engineered with Dashboard abstraction, file locks, process safety (~500+ lines)

**Paul/Lesh's concern**: Why all the complexity?

---

## Lesh's Approach (#869) - SIMPLE

### What it does:
```python
# In go2.py - ONE rr.init() call at module level
import rerun as rr
rr.init("rerun_go2", spawn=True)

# In GO2Connection.start()
def onimage(image: Image):
    rr.log("go2_image", image.to_rerun())

def onodom(odom: PoseStamped):
    rr.log("go2_odom", odom.to_rerun())
```

### Files changed:
- `go2.py`: Add `rr.init()` + `rr.log()` calls
- `voxels.py`: Add `rr.log()` for point cloud map
- `Image.py`, `PoseStamped.py`, `PointCloud2.py`: Add `to_rerun()` methods
- `unitree_go2_blueprints.py`: Increase workers from 4 to 7

**Total additions**: ~100 lines

### Pros:
✅ Simple, works immediately  
✅ No abstractions  
✅ Easy to debug  
✅ Minimal code  

### Cons (from code review):
⚠️ `rr.init()` at module level (runs on import even in tests)  
⚠️ `to_rerun()` will crash for CudaImage (not implemented)  
⚠️ Leftover test code (`example()` function in PoseStamped)  

---

## Jeff's Approach (#868) - COMPLEX

### What it does:
```python
# Dashboard module manages everything
from dimos.dashboard.module import Dashboard, RerunConnection

# In your module
class MyModule(Module):
    def start(self):
        self.rc = RerunConnection()  # One per worker
        
        def on_image(img):
            self.rc.log("my_image", img.to_rerun())

# In blueprint
blueprint = autoconnect(
    go2_connection(),
    Dashboard.blueprint(auto_open=True),  # Manages rr.init()
)
```

### New infrastructure:
- **Dashboard module** (144 lines) - Manages rerun server lifecycle
- **RerunConnection** class - Lazy gRPC connection with PID checks
- **FileBasedBoolean** - File-based locks for worker coordination
- **Zellij integration** - Terminal sessions in web UI
- **make_constant_across_workers()** - Share config via PID hierarchy

**Total additions**: ~500+ lines

### Pros:
✅ Proper multi-worker safety  
✅ Centralized rerun management  
✅ Extensible (web terminal, etc.)  

### Cons:
❌ Over-engineered for simple use case  
❌ File-based locks seem hacky  
❌ Harder to understand/debug  
❌ Still has race conditions (see Paul's comments)  

---

## The Debate (From PR Comments)

### Paul's Questions:
> "About Zellij, why do we want this?"  
> "I find the file-based parent PID search a bit odd"  
> "Why not just environment variables?"

### Jeff's Defense:
> "File locks prevent gRPC crashes on MacOS"  
> "Zellij is flexible for CLI tools (htop, etc.)"  
> "Shared memory requires locks in main module"

### Lesh's Response:
> "Not sold on file locks until shown the actual issue"  
> "My PR doesn't use locks, just getting started instructions, seems to work"

---

## Technical Breakdown

### Issue 1: Multi-Worker Safety

**Problem**: Dask spawns multiple workers. Each needs its own rerun connection.

**Lesh's solution**: Just `rr.init()` once at module level, rerun SDK handles it
**Jeff's solution**: `RerunConnection` per worker with PID checks

**Reality**: Rerun SDK already handles multi-process via recording_id. Jeff's PID check is redundant.

---

### Issue 2: Initialization Timing

**Problem**: Need to init rerun before logging, but modules start in parallel.

**Lesh's solution**: `rr.init()` at import time (module level)
**Jeff's solution**: Dashboard module with `FileBasedBoolean` lock

**Reality**: Lesh's approach has the import-time issue Paul flagged. Jeff's file lock seems like overkill.

**Better solution**: Init in module's `start()` method, not at import.

---

### Issue 3: CudaImage Support

**Problem**: `Image.to_rerun()` fails for CudaImage backend.

**Both miss this**: Neither implemented `CudaImage.to_rerun()`

**Solution**: Add method to CudaImage or convert to numpy first.

---

## What Should Actually Be Done

### Minimal Working Version (Take from Lesh):
1. Add `to_rerun()` methods to: Image, PoseStamped, PointCloud2
2. Init rerun in module `start()` (not at import)
3. Direct `rr.log()` calls where needed

### Don't Take from Jeff:
- ❌ Dashboard module (unnecessary abstraction)
- ❌ FileBasedBoolean (hacky workaround)
- ❌ RerunConnection class (redundant with SDK)
- ❌ Zellij (out of scope for basic rerun)

### What Needs Fixing (From Lesh's PR):
- Move `rr.init()` from module level to `start()` method
- Implement `CudaImage.to_rerun()`
- Remove leftover test code

---

## Recommended Approach

```python
# In go2.py
class GO2Connection(Module):
    def __init__(self):
        super().__init__()
        self._rerun_initialized = False
    
    @rpc
    def start(self):
        super().start()
        
        # Init rerun once per module
        if not self._rerun_initialized:
            rr.init("dimos_go2", spawn=True)
            self._rerun_initialized = True
        
        # Log to rerun
        def onimage(image: Image):
            self.color_image.publish(image)
            rr.log("go2/color_image", image.to_rerun())
        
        def onodom(odom: PoseStamped):
            self._publish_tf(odom)
            rr.log("go2/odom", odom.to_rerun())
        
        self._disposables.add(self.connection.video_stream().subscribe(onimage))
        self._disposables.add(self.connection.odom_stream().subscribe(onodom))
```

**Clean, simple, no file locks, no Dashboard abstraction.**

---

## Summary Table

| Aspect | Lesh | Jeff | Recommended |
|--------|------|------|-------------|
| Lines of code | ~100 | ~500+ | ~100 |
| Abstractions | None | Dashboard, RerunConnection | None |
| Init strategy | Module-level (bad) | FileBasedBoolean (hacky) | In `start()` method |
| Multi-worker | Relies on SDK | Custom PID checks | Relies on SDK |
| Complexity | Low | High | Low |
| Maintainability | Good | Poor | Good |

**Verdict**: Take Lesh's approach but fix the `rr.init()` placement.

