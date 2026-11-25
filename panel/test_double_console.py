import unittest
if __package__:
    from .double_state import ConsoleState
else:
    from double_state import ConsoleState


def packet(state, src, dest="-"):
    return f"DP1,1000,{state},{src},{dest},0,3.1416,0,0.3,-0.2,0.14,-1.2"


class ConsoleTests(unittest.TestCase):
    def test_no_route_while_off_or_uncalibrated(self):
        for phase in ("ZERO", "OFF", "BRAKE", "DAMP"):
            m = ConsoleState()
            m.feed(packet(phase, "-"), 2)
            for dest in "rewq":
                self.assertIsNone(m.command(dest, 2.1))
            self.assertEqual(m.command("z", 2.1), "z" if phase in ("ZERO", "OFF") else None)

    def test_all_routes(self):
        for src in "rewq":
            for dst in "rewq":
                for phase in (("READY", "WAIT") if src == "r" else ("HOLD",)):
                    m = ConsoleState()
                    self.assertTrue(m.feed(packet(phase, src), 1))
                    want = (("r" if phase == "READY" else "") + dst) if dst != src else None
                    self.assertEqual(m.command(dst, 1.1), want)

    def test_wait_for_board_not_click(self):
        m = ConsoleState()
        m.feed(packet("HOLD", "e"), 0)
        self.assertEqual(m.request("q", 0.1), "q")
        m.feed(packet("HOLD", "e"), 0.2)
        self.assertIsNone(m.command("w", 0.2))
        for phase in ("ARM", "SWING"):
            m.feed(packet(phase, "e", "q"), 0.3)
            self.assertIsNone(m.command("w", 0.3))
        m.feed(packet("HOLD", "q"), 0.4)
        self.assertEqual(m.command("w", 0.4), "w")
        self.assertIsNone(m.command("w", 2))

    def test_untrusted_input_and_rejection(self):
        m = ConsoleState()
        for line in ("", "POSE,1,q,0,0,0", packet("HOLD", "z"), packet("HOLD", "e").replace("0.14", "nan")):
            self.assertFalse(m.feed(line, 0))
        self.assertIsNone(m.command("q", 0))
        m.feed(packet("HOLD", "e"), 1)
        m.request("q", 1)
        m.feed("DU->UU refused: insufficient rail; DU hold continues.", 1)
        self.assertIsNone(m.command("w", 1))
        m.feed(packet("HOLD", "e"), 1.1)
        self.assertEqual(m.command("w", 1.1), "w")
        m.feed("STOP_CODE=TILT_LIMIT", 1.2)
        self.assertIsNone(m.command("w", 1.2))
