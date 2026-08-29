#!/usr/bin/env python3
import argparse
import asyncio
import gc
import tracemalloc

from embedded_robot_ros2.mock_graph import MockRosGraph
from embedded_robot_ros2.models import CommandRequest
from embedded_robot_ros2.service import RobotEdgeService


async def soak(iterations: int, max_growth_kib: int) -> None:
    tracemalloc.start()
    graph = MockRosGraph(action_delay_s=0)
    service = RobotEdgeService(graph)
    await service.start()
    gc.collect()
    baseline = tracemalloc.get_traced_memory()[0]
    try:
        for index in range(iterations):
            version = service.state("MM-01").state_version
            request = CommandRequest.model_validate(
                {
                    "command_id": f"soak-{index}",
                    "robot_id": "MM-01",
                    "kind": "navigate",
                    "ttl_ms": 500,
                    "expected_state_version": version,
                    "payload": {
                        "pose": {"frame_id": "map", "x": index % 10, "y": 0, "yaw": 0}
                    },
                }
            )
            await service.submit(request)
            await asyncio.sleep(0)
            await asyncio.sleep(0)
        gc.collect()
        growth = (tracemalloc.get_traced_memory()[0] - baseline) // 1024
        print(f"iterations={iterations} growth_kib={growth} live_tasks={graph.live_tasks}")
        if growth > max_growth_kib or graph.live_tasks:
            raise SystemExit(1)
    finally:
        await service.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--iterations", type=int, default=5000)
    parser.add_argument("--max-growth-kib", type=int, default=12000)
    args = parser.parse_args()
    asyncio.run(soak(args.iterations, args.max_growth_kib))

