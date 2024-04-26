import time
from typing import Dict, Tuple, Optional
from contextlib import contextmanager

class Perf():
    def __init__(self, window: int = 0):
        self.starts: Dict[str, float] = {}
        self.averages: Dict[str, Tuple[int, float]] = {}

        self.last_block = None
        self.window = window

    def update_average(self, block: str, dt: float) -> float:
        if not block in self.averages or self.averages[block][0] == self.window:
            self.averages[block] = (1, dt)
        else:
            n, avg_dt = self.averages[block]
            self.averages[block] = (n + 1, (avg_dt * n + dt) / (n + 1))

        return self.averages[block][1]

    def start(self, block: str):
        self.starts[block] = time.perf_counter()
        self.last_block = block
    
    def end(self, block: Optional[str] = None, report: bool = False):
        if block is None:
            block = self.last_block

        dt = time.perf_counter() - self.starts[block]
        average_dt = self.update_average(block, dt)

        if report:
            print(f"{block}: current={dt * 1000 * 1000:.2f}us, average[w={self.window}]={average_dt * 1000 * 1000:.2f}us")

    @contextmanager
    def profile_block(self, block: str, report: bool = False):
        self.start(block)
        try:
            yield
        finally:
            self.end(block, report=report)

    def generate_final_report(self):
        print("\n")
        print("=== FINAL REPORT ===")
        for block, (n, average) in sorted(self.averages.items(), key=lambda item: item[1][1], reverse=True):
            print(f"{block}: average[w={self.window}, n={n}]={average * 1000 * 1000:.2f}us")
