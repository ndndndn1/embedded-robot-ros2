# Verification benchmark

Reference run on 2026-08-29, CPython 3.12.3, Linux x86_64. The benchmark uses the
deterministic in-process ROS graph and measures command validation, mapping, dispatch,
and terminal status handling. It does not estimate physical controller or DDS latency.

```text
iterations=1000 median_ms=0.099 p95_ms=0.114 live_tasks=0
iterations=5000 retained_records=1000 growth_kib=4763 live_tasks=0
```

Reproduce with:

```bash
uv run python bench/benchmark.py --iterations 1000
uv run python bench/soak.py --iterations 5000 --max-growth-kib 12000
```

The production service retains a bounded 10,000-command idempotency history by default
and evicts
the oldest terminal record at capacity. It never evicts active work; a fully active
history fails closed. The soak threshold covers retained records and allocator overhead,
while `live_tasks=0` proves mock action tasks are reclaimed before shutdown. The soak
uses a 1,000-record window so five complete rotations distinguish bounded retention
from growth proportional to total processed commands.
