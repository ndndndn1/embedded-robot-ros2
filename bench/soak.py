#!/usr/bin/env python3
import argparse
import asyncio
import gc
import tracemalloc
from datetime import timedelta

from embedded_robot_ros2.mock_graph import MockRosGraph
from embedded_robot_ros2.models import CommandRequest, now_utc
from embedded_robot_ros2.service import RobotEdgeService


async def soak(iterations: int, max_growth_kib: int, retained_records: int) -> None:
    tracemalloc.start()
    graph = MockRosGraph(action_delay_s=0)
    service = RobotEdgeService(graph, max_command_records=retained_records)
    await service.start()
    gc.collect()
    baseline = tracemalloc.get_traced_memory()[0]
    try:
        for index in range(iterations):
            version = service.state("mm-01-a").state_version
            now = now_utc()
            request = CommandRequest.model_validate(
                {
                    "command_id": f"soak-{index}",
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
    parser.add_argument("--retained-records", type=int, default=1000)
    args = parser.parse_args()
    asyncio.run(soak(args.iterations, args.max_growth_kib, args.retained_records))
