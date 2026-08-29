#!/usr/bin/env python3
import argparse
import asyncio
import statistics
import time

from embedded_robot_ros2.mock_graph import MockRosGraph
from embedded_robot_ros2.models import CommandRequest
from embedded_robot_ros2.service import RobotEdgeService


async def benchmark(iterations: int) -> None:
    graph = MockRosGraph(action_delay_s=0)
    service = RobotEdgeService(graph)
    await service.start()
    timings: list[float] = []
    try:
        for index in range(iterations):
            version = service.state("MM-01").state_version
            request = CommandRequest.model_validate(
                {
                    "command_id": f"bench-{index}",
                    "robot_id": "MM-01",
                    "kind": "navigate",
                    "ttl_ms": 500,
                    "expected_state_version": version,
                    "payload": {
                        "pose": {"frame_id": "map", "x": index % 10, "y": 0, "yaw": 0}
                    },
                }
            )
            started = time.perf_counter_ns()
            await service.submit(request)
            await asyncio.sleep(0)
            await asyncio.sleep(0)
            timings.append((time.perf_counter_ns() - started) / 1_000_000)
        ordered = sorted(timings)
        p95 = ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))]
        print(
            f"iterations={iterations} median_ms={statistics.median(timings):.3f} "
            f"p95_ms={p95:.3f} live_tasks={graph.live_tasks}"
        )
    finally:
        await service.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--iterations", type=int, default=1000)
    args = parser.parse_args()
    asyncio.run(benchmark(args.iterations))

