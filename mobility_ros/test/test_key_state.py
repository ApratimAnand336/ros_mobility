from mobility_ros.key_state import KeyState


def ks():
    return KeyState(["w", "a", "s", "d", "shift"], debounce=0.25)


def test_two_keys_held_together():
    k = ks()
    k.press("w", 0.0)
    k.press("a", 0.1)
    k.update(1.0)
    assert k.axis("w", "s") == 1.0 and k.axis("a", "d") == 1.0


def test_autorepeat_fake_release_is_ignored():
    k = ks()
    k.press("w", 0.0)
    for i in range(1, 30):                 # X11 auto-repeat: release+press pairs
        k.release("w", i * 0.033)
        k.press("w", i * 0.033 + 0.001)
        k.update(i * 0.033 + 0.002)
    assert k.down["w"]


def test_real_release_after_debounce():
    k = ks()
    k.press("w", 0.0)
    k.release("w", 1.0)
    k.update(1.1)
    assert k.down["w"]                     # still inside the debounce window
    k.update(1.3)
    assert not k.down["w"]


def test_space_suppresses_held_key_until_real_release():
    k = ks()
    k.press("w", 0.0)
    k.clear(["w"])                         # Space while W is still held
    k.press("w", 0.05)                     # auto-repeat keeps coming
    assert not k.down["w"]
    k.release("w", 0.1)
    k.update(0.4)                          # real release confirmed
    k.press("w", 0.5)                      # fresh press drives again
    assert k.down["w"]


def test_focus_loss_releases_everything():
    k = ks()
    k.press("w", 0.0)
    k.press("shift", 0.0)
    k.release_all()
    assert not any(k.down.values())
    k.press("w", 0.1)
    assert k.down["w"]                     # not suppressed after focus loss


def test_opposite_keys_cancel():
    k = ks()
    k.press("w", 0.0)
    k.press("s", 0.0)
    assert k.axis("w", "s") == 0.0


def test_clear_does_not_block_keys_that_were_not_held():
    k = ks()
    k.press("w", 0.0)
    k.clear(["w", "a", "s", "d"])          # mode switch while only W is held
    k.press("a", 0.1)                      # A was not held: works immediately
    assert k.down["a"] and not k.down["w"]
