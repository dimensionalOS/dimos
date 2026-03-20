
```python
with db() as db:
   with db.stream as image:
      image.put(...)
```

DB specifies some general configuration for all sessions/streams.

`db.stream` initializes these sessions?
