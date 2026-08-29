from __future__ import annotations

from collections import Counter
from threading import Lock


class Metrics:
    def __init__(self) -> None:
        self._counts: Counter[tuple[str, tuple[tuple[str, str], ...]]] = Counter()
        self._lock = Lock()

    def increment(self, name: str, **labels: str) -> None:
        key = (name, tuple(sorted(labels.items())))
        with self._lock:
            self._counts[key] += 1

    def render(self) -> str:
        lines = [
            "# HELP robot_adapter_events_total Adapter lifecycle events.",
            "# TYPE robot_adapter_events_total counter",
        ]
        with self._lock:
            items = sorted(self._counts.items())
        for (_name, labels), count in items:
            label_text = ",".join(f'{key}="{value}"' for key, value in labels)
            suffix = f"{{{label_text}}}" if label_text else ""
            lines.append(f"robot_adapter_events_total{suffix} {count}")
        return "\n".join(lines) + "\n"
