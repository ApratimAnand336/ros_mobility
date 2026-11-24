"""Held-key tracking for the teleop key window. Pure Python (no Tk, no ROS), unit-tested.

Ported from the team's rover_gazebo/teleop_rover_gui.py. A window gets real press and
release events per key, so several keys can be held at once (W+A drives an arc). Two
traps it handles:

  * X11 auto-repeat sends a fake release+press pair many times a second while a key is
    held. A release only counts once no press of that key follows within `debounce`.
  * Space and mode changes stop driving at once. A key still physically held (and so
    still auto-repeating) must not quietly resume driving: it is *suppressed* until a
    real release is seen, and only a fresh press drives again.
  * Losing window focus means release events may never arrive: everything is released.
"""


class KeyState:
    def __init__(self, keys, debounce=0.25):
        self.keys = list(keys)
        self.debounce = float(debounce)        # s
        self.down = {k: False for k in self.keys}
        self.pending = {}                      # key -> time its release was seen
        self.suppressed = set()

    def press(self, key, now):
        if key not in self.down:
            return
        self.pending.pop(key, None)            # auto-repeat: cancels the fake release
        if key in self.suppressed:
            return
        self.down[key] = True

    def release(self, key, now):
        if key in self.down:
            self.pending[key] = now

    def update(self, now):
        """Confirm releases that were not followed by a press within `debounce`."""
        for k, t in list(self.pending.items()):
            if now - t >= self.debounce:
                del self.pending[k]
                self.down[k] = False
                self.suppressed.discard(k)

    def clear(self, keys=None, suppress=True):
        """Stop the given keys now (all by default). With suppress, a key that is
        still held stays ignored until it is really released and pressed again.
        Keys that are not held are not suppressed, so the next press works at once."""
        for k in (keys or self.keys):
            held = self.down[k] or k in self.pending
            self.down[k] = False
            self.pending.pop(k, None)
            if suppress and held:
                self.suppressed.add(k)

    def release_all(self):
        """Focus lost: nothing held any more, nothing suppressed."""
        self.clear(suppress=False)
        self.suppressed.clear()

    def axis(self, positive, negative):
        """+1, -1 or 0 from a pair of keys (both held cancels out)."""
        return (1.0 if self.down.get(positive) else 0.0) - \
            (1.0 if self.down.get(negative) else 0.0)
