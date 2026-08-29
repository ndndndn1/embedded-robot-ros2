#!/usr/bin/env python3
import argparse
import asyncio
import statistics
import time
from datetime import timedelta

from embedded_robot_ros2.mock_graph import MockRosGraph
from embedded_robot_ros2.models import CommandRequest, now_utc
from embedded_robot_ros2.service import RobotEdgeService


async def benchmark(iterations: int) -> None:
    graph = MockRosGraph(action_delay_s=0)
    service = RobotEdgeService(graph)
    await service.start()
    timings: list[float] = []
    try:
        for index in range(iterations):
            version = service.state("mm-01-a").state_version
            now = now_utc()
            request = CommandRequest.model_validate(
                {
                    "command_id": f"bench-{index}",
                    "robot_id": "mm-01-a",
                    "issued_at": now.isoformat(),
                    "expires_at": (now + timedelta(seconds=1)).isoformat(),
                    "expected_state_version": version,
                    "action": {
                        "type": "navigate",
                        "target": {
                            "frame": "map",
                            "x_m": index % 10,
                            "y_m": 0,
                            "yaw_rad": 0,
                        },
                        "max_speed_mps": 0.5,
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
