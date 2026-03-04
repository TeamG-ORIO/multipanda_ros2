"""Retry counter management for the task supervisor."""


class RetryManager:
    """Tracks retry counts for each configurable category."""

    def __init__(self, max_grasp: int, max_label: int, max_perception: int):
        self._limits = {
            'grasp':      max_grasp,
            'label':      max_label,
            'perception': max_perception,
        }
        self._counts: dict[str, int] = {k: 0 for k in self._limits}

    def increment(self, category: str) -> bool:
        """Increment counter.  Returns True if the limit has been reached."""
        if category not in self._counts:
            raise KeyError(f"Unknown retry category: {category}")
        self._counts[category] += 1
        return self._counts[category] >= self._limits[category]

    def count(self, category: str) -> int:
        return self._counts.get(category, 0)

    def reset(self, category: str | None = None) -> None:
        """Reset one category or all categories when category is None."""
        if category is None:
            for k in self._counts:
                self._counts[k] = 0
        else:
            self._counts[category] = 0

    def as_dict(self) -> dict[str, int]:
        return dict(self._counts)
