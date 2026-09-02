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

## Perception CPU reference gate

`tools/run_cpp_checks.sh` compiles the C++20 core with warnings as errors, runs deterministic
alignment/calibration/6DoF/grasp/backend tests, measures the 640x480 RGB-D path, and repeats tests
under AddressSanitizer and UndefinedBehaviorSanitizer. A 2026-09-02 run on the development host
reported:

```json
{"backend":"cpu","width":640,"height":480,"iterations":20,"p50_ms":7.218,"p95_ms":12.149,"throughput_hz":138.54}
```

The release gate is p95 at most 100 ms and at least 10 Hz on a defined 4-vCPU x86 runner. CI
records new measurements; the number above is evidence, not a permanent performance promise.
GPU latency is deliberately unscored until a healthy NVIDIA runtime executes the CUDA self-test.

The production soak gate is 60 minutes with RSS growth at most 5%, GPU allocation growth at most
2%, queue depth at most four, and no live callback/thread/device allocation after lifecycle cleanup.
CI uses bounded sanitizer and graph tests; D455/D457 and CUDA HIL runners own the long soak.
