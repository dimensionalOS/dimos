"""User-built applications layered on top of the DimOS core / robot stacks.

Anything in this package belongs to the application authors. The core
`dimos/robot/custom/` modules (e.g. jamjam's BBox / target-lock / behavior
modules) are imported and composed here but never modified -- so the upstream
jamjam control stack can be fast-forwarded independently of the apps that
ride on top.
"""
