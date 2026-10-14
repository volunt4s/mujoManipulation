"""The web layer: a fixed-rate sim loop behind a WebSocket, and the page that
draws it.

This package imports the control library. The control library does not import
this package, and must stay fully usable with the server switched off — that is
the one rule here, and `examples/` is the test of it.
"""
