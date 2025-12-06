#!/usr/bin/env python3
import rerun.blueprint as rrb
from collections import namedtuple

class Layout:
    entities: namedtuple
    viewer_blueprint: rrb.Blueprint