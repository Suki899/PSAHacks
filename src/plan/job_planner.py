import copy
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional

from src.constant import CONSTANT
from src.floor import Coordinate, SectorMapSnapshot
from src.job import InstructionType, Job, JobInstruction
from src.operators import HT_Coordinate_View
from src.plan.job_tracker import JobTracker


class HTSelectionStrategy:
    """Strategy base class for choosing an HT for a job."""

    name: str = "base"

    def select(
        self,
        *,
        job_type: str,
        job_info: Dict[str, object],
        ht_coord_tracker: HT_Coordinate_View,
        sector_map_snapshot: SectorMapSnapshot,
        selected_HT_names: Iterable[str],
    ) -> Optional[str]:
        raise NotImplementedError


class LeftRightHTSelectionStrategy(HTSelectionStrategy):
    """Matches the legacy behaviour: discharge jobs prefer the left-most HT."""

    name = "left_right_bias"

    def select(
        self,
        *,
        job_type: str,
        job_info: Dict[str, object],
        ht_coord_tracker: HT_Coordinate_View,
        sector_map_snapshot: SectorMapSnapshot,
        selected_HT_names: Iterable[str],
    ) -> Optional[str]:
        plannable_HTs = ht_coord_tracker.get_available_HTs()
        taken = set(selected_HT_names)
        if job_type == CONSTANT.JOB_PARAMETER.DISCHARGE_JOB_TYPE:
            for HT_name in plannable_HTs:
                if HT_name not in taken:
                    return HT_name
        else:
            for HT_name in reversed(plannable_HTs):
                if HT_name not in taken:
                    return HT_name
        return None


class ClosestTargetHTSelectionStrategy(HTSelectionStrategy):
    """Selects the HT whose current coordinate is closest to the next waypoint."""

    name = "closest_to_target"

    def select(
        self,
        *,
        job_type: str,
        job_info: Dict[str, object],
        ht_coord_tracker: HT_Coordinate_View,
        sector_map_snapshot: SectorMapSnapshot,
        selected_HT_names: Iterable[str],
    ) -> Optional[str]:
        available = [
            HT_name
            for HT_name in ht_coord_tracker.get_available_HTs()
            if HT_name not in set(selected_HT_names)
        ]
        if not available:
            return None

        if job_type == CONSTANT.JOB_PARAMETER.DISCHARGE_JOB_TYPE:
            target_coord = sector_map_snapshot.get_QC_sector(
                job_info["QC_name"]
            ).in_coord
        else:
            target_coord = sector_map_snapshot.get_yard_sector(
                job_info["yard_name"]
            ).in_coord

        def manhattan_distance(HT_name: str) -> int:
            coord = ht_coord_tracker.get_coordinate(HT_name)
            return abs(coord.x - target_coord.x) + abs(coord.y - target_coord.y)

        return min(available, key=manhattan_distance)


class RoundRobinHTSelectionStrategy(HTSelectionStrategy):
    """Rotates HT assignments to spread utilisation evenly."""

    name = "round_robin"

    def __init__(self):
        self._next_index = 0

    def select(
        self,
        *,
        job_type: str,
        job_info: Dict[str, object],
        ht_coord_tracker: HT_Coordinate_View,
        sector_map_snapshot: SectorMapSnapshot,
        selected_HT_names: Iterable[str],
    ) -> Optional[str]:
        available = ht_coord_tracker.get_available_HTs()
        if not available:
            return None

        taken = set(selected_HT_names)
        num_available = len(available)
        for i in range(num_available):
            index = (self._next_index + i) % num_available
            candidate = available[index]
            if candidate not in taken:
                self._next_index = (index + 1) % num_available
                return candidate

        return None


class YardSelectionStrategy:
    """Strategy base class for selecting a yard candidate."""

    name: str = "base"

    def select(
        self,
        *,
        job_type: str,
        primary_yard: str,
        alternate_yards: List[str],
        job_info: Dict[str, object],
    ) -> str:
        raise NotImplementedError


class PrimaryYardStrategy(YardSelectionStrategy):
    name = "primary"

    def select(
        self,
        *,
        job_type: str,
        primary_yard: str,
        alternate_yards: List[str],
        job_info: Dict[str, object],
    ) -> str:
        return primary_yard


class AlternateFirstYardStrategy(YardSelectionStrategy):
    """Prefers the first alternate yard when present for discharge jobs."""

    name = "alternate_first"

    def select(
        self,
        *,
        job_type: str,
        primary_yard: str,
        alternate_yards: List[str],
        job_info: Dict[str, object],
    ) -> str:
        if (
            job_type == CONSTANT.JOB_PARAMETER.DISCHARGE_JOB_TYPE
            and alternate_yards
        ):
            return alternate_yards[0]
        return primary_yard


class RoundRobinYardStrategy(YardSelectionStrategy):
    """Cycles through primary and alternate yards to balance usage."""

    name = "yard_round_robin"

    def __init__(self):
        self._next_index = 0

    def select(
        self,
        *,
        job_type: str,
        primary_yard: str,
        alternate_yards: List[str],
        job_info: Dict[str, object],
    ) -> str:
        candidates = [primary_yard] + [
            yard for yard in alternate_yards if yard
        ]
        if not candidates:
            return primary_yard

        selected = candidates[self._next_index % len(candidates)]
        self._next_index = (self._next_index + 1) % len(candidates)
        return selected


class RouteStrategy:
    """Strategy base class for generating HT navigation paths."""

    name: str = "base"

    @staticmethod
    def _straight_path(
        start: Coordinate, end: Coordinate, *, horizontal_first: bool = True
    ) -> List[Coordinate]:
        path: List[Coordinate] = []
        current_x, current_y = start.x, start.y

        def extend_horizontal(target_x: int):
            nonlocal current_x
            if target_x == current_x:
                return
            step = 1 if target_x > current_x else -1
            for x in range(current_x + step, target_x + step, step):
                path.append(Coordinate(x, current_y))
            current_x = target_x

        def extend_vertical(target_y: int):
            nonlocal current_y
            if target_y == current_y:
                return
            step = 1 if target_y > current_y else -1
            for y in range(current_y + step, target_y + step, step):
                path.append(Coordinate(current_x, y))
            current_y = target_y

        if horizontal_first:
            extend_horizontal(end.x)
            extend_vertical(end.y)
        else:
            extend_vertical(end.y)
            extend_horizontal(end.x)

        if not path or path[-1] != end:
            path.append(Coordinate(end.x, end.y))
        return path

    def buffer_to_qc(
        self,
        snapshot: SectorMapSnapshot,
        buffer_coord: Coordinate,
        QC_name: str,
    ) -> List[Coordinate]:
        raise NotImplementedError

    def buffer_to_yard(
        self,
        snapshot: SectorMapSnapshot,
        buffer_coord: Coordinate,
        yard_name: str,
    ) -> List[Coordinate]:
        raise NotImplementedError

    def yard_to_buffer(
        self,
        snapshot: SectorMapSnapshot,
        yard_name: str,
        buffer_coord: Coordinate,
    ) -> List[Coordinate]:
        raise NotImplementedError

    def qc_to_buffer(
        self,
        snapshot: SectorMapSnapshot,
        QC_name: str,
        buffer_coord: Coordinate,
    ) -> List[Coordinate]:
        raise NotImplementedError


class BaselineRouteStrategy(RouteStrategy):
    name = "baseline"

    def buffer_to_qc(
        self,
        snapshot: SectorMapSnapshot,
        buffer_coord: Coordinate,
        QC_name: str,
    ) -> List[Coordinate]:
        QC_in_coord = snapshot.get_QC_sector(QC_name).in_coord

        highway_lane_y = 7
        path = [Coordinate(buffer_coord.x, highway_lane_y)]
        path.extend(
            [Coordinate(x, highway_lane_y) for x in range(buffer_coord.x - 1, 0, -1)]
        )
        up_path_x = 1
        path.extend([Coordinate(up_path_x, y) for y in range(6, 3, -1)])
        qc_travel_lane_y = 4
        path.extend(
            [Coordinate(x, qc_travel_lane_y) for x in range(2, QC_in_coord.x + 1, 1)]
        )
        path.append(QC_in_coord)
        return path

    def buffer_to_yard(
        self,
        snapshot: SectorMapSnapshot,
        buffer_coord: Coordinate,
        yard_name: str,
    ) -> List[Coordinate]:
        yard_in_coord = snapshot.get_yard_sector(yard_name).in_coord

        path = [Coordinate(buffer_coord.x, buffer_coord.y - 1)]
        qc_lane_y = 5
        path.extend(
            [Coordinate(x, qc_lane_y) for x in range(buffer_coord.x + 1, 43, 1)]
        )
        down_path_x = 42
        path.extend([Coordinate(down_path_x, y) for y in range(6, 12, 1)])
        highway_lane_y = 11
        path.extend([Coordinate(x, highway_lane_y) for x in range(41, 0, -1)])
        highway_lane_y = 12
        path.append(Coordinate(1, highway_lane_y))
        path.extend(
            [Coordinate(x, highway_lane_y) for x in range(2, yard_in_coord.x + 1, 1)]
        )
        path.append(yard_in_coord)
        return path

    def yard_to_buffer(
        self,
        snapshot: SectorMapSnapshot,
        yard_name: str,
        buffer_coord: Coordinate,
    ) -> List[Coordinate]:
        yard_out_coord = snapshot.get_yard_sector(yard_name).out_coord

        path = [yard_out_coord]
        highway_lane_y = 12
        path.extend(
            [Coordinate(x, highway_lane_y) for x in range(yard_out_coord.x, 42, 1)]
        )
        up_path_x = 41
        path.extend([Coordinate(up_path_x, y) for y in range(11, 6, -1)])
        highway_lane_y = 7
        path.extend(
            [Coordinate(x, highway_lane_y) for x in range(40, buffer_coord.x - 1, -1)]
        )
        path.append(buffer_coord)
        return path

    def qc_to_buffer(
        self,
        snapshot: SectorMapSnapshot,
        QC_name: str,
        buffer_coord: Coordinate,
    ) -> List[Coordinate]:
        QC_out_coord = snapshot.get_QC_sector(QC_name).out_coord

        path = [QC_out_coord]
        qc_travel_lane_y = 4
        path.append(Coordinate(QC_out_coord.x, qc_travel_lane_y))
        path.extend(
            [Coordinate(x, qc_travel_lane_y) for x in range(QC_out_coord.x + 1, 43, 1)]
        )
        down_path_x = 42
        path.extend([Coordinate(down_path_x, y) for y in range(5, 8, 1)])
        highway_lane_y = 7
        path.extend(
            [Coordinate(x, highway_lane_y) for x in range(41, buffer_coord.x - 1, -1)]
        )
        path.append(buffer_coord)
        return path


class DirectLaneRouteStrategy(RouteStrategy):
    """Moves horizontally then vertically to reach the target quickly."""

    name = "direct_lane"

    def buffer_to_qc(
        self,
        snapshot: SectorMapSnapshot,
        buffer_coord: Coordinate,
        QC_name: str,
    ) -> List[Coordinate]:
        target = snapshot.get_QC_sector(QC_name).in_coord
        return self._straight_path(buffer_coord, target, horizontal_first=True)

    def buffer_to_yard(
        self,
        snapshot: SectorMapSnapshot,
        buffer_coord: Coordinate,
        yard_name: str,
    ) -> List[Coordinate]:
        target = snapshot.get_yard_sector(yard_name).in_coord
        return self._straight_path(buffer_coord, target, horizontal_first=False)

    def yard_to_buffer(
        self,
        snapshot: SectorMapSnapshot,
        yard_name: str,
        buffer_coord: Coordinate,
    ) -> List[Coordinate]:
        start = snapshot.get_yard_sector(yard_name).out_coord
        return self._straight_path(start, buffer_coord, horizontal_first=True)

    def qc_to_buffer(
        self,
        snapshot: SectorMapSnapshot,
        QC_name: str,
        buffer_coord: Coordinate,
    ) -> List[Coordinate]:
        start = snapshot.get_QC_sector(QC_name).out_coord
        return self._straight_path(start, buffer_coord, horizontal_first=False)


class PerimeterRouteStrategy(RouteStrategy):
    """Follows the perimeter lanes to avoid congested central traffic."""

    name = "perimeter"

    def _stitch_waypoints(
        self, waypoints: List[Coordinate]
    ) -> List[Coordinate]:
        path: List[Coordinate] = []
        current = waypoints[0]
        for target in waypoints[1:]:
            segment = self._straight_path(current, target, horizontal_first=True)
            if path and segment and path[-1] == segment[0]:
                segment = segment[1:]
            path.extend(segment)
            current = target
        return path

    def buffer_to_qc(
        self,
        snapshot: SectorMapSnapshot,
        buffer_coord: Coordinate,
        QC_name: str,
    ) -> List[Coordinate]:
        qc_in = snapshot.get_QC_sector(QC_name).in_coord
        waypoints = [
            buffer_coord,
            Coordinate(buffer_coord.x, 12),
            Coordinate(42, 12),
            Coordinate(42, 4),
            Coordinate(qc_in.x, 4),
            qc_in,
        ]
        return self._stitch_waypoints(waypoints)

    def buffer_to_yard(
        self,
        snapshot: SectorMapSnapshot,
        buffer_coord: Coordinate,
        yard_name: str,
    ) -> List[Coordinate]:
        yard_in = snapshot.get_yard_sector(yard_name).in_coord
        waypoints = [
            buffer_coord,
            Coordinate(buffer_coord.x, 4),
            Coordinate(1, 4),
            Coordinate(1, 12),
            Coordinate(yard_in.x, 12),
            yard_in,
        ]
        return self._stitch_waypoints(waypoints)

    def yard_to_buffer(
        self,
        snapshot: SectorMapSnapshot,
        yard_name: str,
        buffer_coord: Coordinate,
    ) -> List[Coordinate]:
        yard_out = snapshot.get_yard_sector(yard_name).out_coord
        waypoints = [
            yard_out,
            Coordinate(yard_out.x, 12),
            Coordinate(1, 12),
            Coordinate(1, 7),
            Coordinate(buffer_coord.x, 7),
            buffer_coord,
        ]
        return self._stitch_waypoints(waypoints)

    def qc_to_buffer(
        self,
        snapshot: SectorMapSnapshot,
        QC_name: str,
        buffer_coord: Coordinate,
    ) -> List[Coordinate]:
        qc_out = snapshot.get_QC_sector(QC_name).out_coord
        waypoints = [
            qc_out,
            Coordinate(qc_out.x, 4),
            Coordinate(42, 4),
            Coordinate(42, 7),
            Coordinate(buffer_coord.x, 7),
            buffer_coord,
        ]
        return self._stitch_waypoints(waypoints)


@dataclass
class PlannerVariant:
    name: str
    ht_selector: HTSelectionStrategy
    yard_selector: YardSelectionStrategy
    route_strategy: RouteStrategy


@dataclass
class VariantPlanResult:
    name: str
    jobs: List[Job]
    total_drive_time: int
    max_drive_time: int
    average_drive_time: float


class JobPlanner:
    """Coordinates job planning activities using HT tracker and sector map data."""

    def __init__(
        self,
        ht_coord_tracker: HT_Coordinate_View,
        sector_map_snapshot: SectorMapSnapshot,
    ):
        self.ht_coord_tracker = ht_coord_tracker
        self.sector_map_snapshot = sector_map_snapshot
        self._variants: Dict[str, PlannerVariant] = self._build_default_variants()
        self._default_variant_name: str = "baseline"

    def _build_default_variants(self) -> Dict[str, PlannerVariant]:
        return {
            "baseline": PlannerVariant(
                name="baseline",
                ht_selector=LeftRightHTSelectionStrategy(),
                yard_selector=PrimaryYardStrategy(),
                route_strategy=BaselineRouteStrategy(),
            ),
            "distance_priority": PlannerVariant(
                name="distance_priority",
                ht_selector=ClosestTargetHTSelectionStrategy(),
                yard_selector=AlternateFirstYardStrategy(),
                route_strategy=DirectLaneRouteStrategy(),
            ),
            "balanced_rotation": PlannerVariant(
                name="balanced_rotation",
                ht_selector=RoundRobinHTSelectionStrategy(),
                yard_selector=RoundRobinYardStrategy(),
                route_strategy=PerimeterRouteStrategy(),
            ),
        }

    def is_deadlock(self):
        return self.ht_coord_tracker.is_deadlock()

    def get_non_moving_HT(self):
        return self.ht_coord_tracker.get_non_moving_HT()

    def set_default_variant(self, variant_name: str):
        if variant_name not in self._variants:
            raise ValueError(
                f"Unknown planner variant '{variant_name}'. Available: {list(self._variants)}"
            )
        self._default_variant_name = variant_name

    def get_available_variants(self) -> List[str]:
        return list(self._variants.keys())

    def _resolve_variant(self, variant_name: Optional[str]) -> PlannerVariant:
        if variant_name is None:
            variant_name = self._default_variant_name
        if variant_name not in self._variants:
            raise ValueError(
                f"Unknown planner variant '{variant_name}'. Available: {list(self._variants)}"
            )
        return self._variants[variant_name]

    def plan(
        self, job_tracker: JobTracker, variant_name: Optional[str] = None
    ) -> List[Job]:
        variant = self._resolve_variant(variant_name)
        return self._plan_with_variant(job_tracker, variant, persist=True)

    def plan_variants(
        self, job_tracker: JobTracker, variant_names: Optional[List[str]] = None
    ) -> List[VariantPlanResult]:
        if variant_names is None:
            variant_names = self.get_available_variants()

        results: List[VariantPlanResult] = []
        for name in variant_names:
            variant = copy.deepcopy(self._resolve_variant(name))
            planned_jobs = self._plan_with_variant(job_tracker, variant, persist=False)
            results.append(self._summarize_variant(name, planned_jobs))
        return results

    def _plan_with_variant(
        self,
        job_tracker: JobTracker,
        variant: PlannerVariant,
        *,
        persist: bool,
    ) -> List[Job]:
        new_jobs: List[Job] = []
        selected_HT_names: List[str] = []

        for job_seq in job_tracker.get_plannable_job_sequences():
            job = job_tracker.get_job(job_seq)
            if job is None:
                continue
            if not persist:
                job = copy.deepcopy(job)

            job_info = job.get_job_info()
            job_type, QC_name, yard_name, alt_yard_names = [
                job_info[k]
                for k in ["job_type", "QC_name", "yard_name", "alt_yard_names"]
            ]

            HT_name = variant.ht_selector.select(
                job_type=job_type,
                job_info=job_info,
                ht_coord_tracker=self.ht_coord_tracker,
                sector_map_snapshot=self.sector_map_snapshot,
                selected_HT_names=selected_HT_names,
            )

            if HT_name is None:
                break
            selected_HT_names.append(HT_name)

            yard_name = variant.yard_selector.select(
                job_type=job_type,
                primary_yard=yard_name,
                alternate_yards=alt_yard_names,
                job_info=job_info,
            )

            job.assign_job(HT_name=HT_name, yard_name=yard_name)
            job_instructions: List[JobInstruction] = []
            buffer_coord = self.ht_coord_tracker.get_coordinate(HT_name)

            if job_type == CONSTANT.JOB_PARAMETER.DISCHARGE_JOB_TYPE:
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.BOOK_QC,
                    )
                )

                path = variant.route_strategy.buffer_to_qc(
                    self.sector_map_snapshot, buffer_coord, QC_name
                )
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.DRIVE,
                        HT_name=HT_name,
                        path=path,
                    )
                )

                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.WORK_QC,
                        HT_name=HT_name,
                        QC_name=QC_name,
                    )
                )

                path = variant.route_strategy.qc_to_buffer(
                    self.sector_map_snapshot, QC_name, buffer_coord
                )
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.DRIVE,
                        HT_name=HT_name,
                        path=path,
                    )
                )

                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.BOOK_YARD,
                    )
                )

                path = variant.route_strategy.buffer_to_yard(
                    self.sector_map_snapshot, buffer_coord, yard_name
                )
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.DRIVE,
                        HT_name=HT_name,
                        path=path,
                    )
                )

                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.WORK_YARD,
                        HT_name=HT_name,
                        yard_name=yard_name,
                    )
                )

                path = variant.route_strategy.yard_to_buffer(
                    self.sector_map_snapshot, yard_name, buffer_coord
                )
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.DRIVE,
                        HT_name=HT_name,
                        path=path,
                    )
                )

            else:
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.BOOK_YARD,
                    )
                )

                path = variant.route_strategy.buffer_to_yard(
                    self.sector_map_snapshot, buffer_coord, yard_name
                )
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.DRIVE,
                        HT_name=HT_name,
                        path=path,
                    )
                )

                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.WORK_YARD,
                        HT_name=HT_name,
                        yard_name=yard_name,
                    )
                )

                path = variant.route_strategy.yard_to_buffer(
                    self.sector_map_snapshot, yard_name, buffer_coord
                )
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.DRIVE,
                        HT_name=HT_name,
                        path=path,
                    )
                )

                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.BOOK_QC,
                    )
                )

                path = variant.route_strategy.buffer_to_qc(
                    self.sector_map_snapshot, buffer_coord, QC_name
                )
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.DRIVE,
                        HT_name=HT_name,
                        path=path,
                    )
                )

                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.WORK_QC,
                        HT_name=HT_name,
                        QC_name=QC_name,
                    )
                )

                path = variant.route_strategy.qc_to_buffer(
                    self.sector_map_snapshot, QC_name, buffer_coord
                )
                job_instructions.append(
                    JobInstruction(
                        instruction_type=InstructionType.DRIVE,
                        HT_name=HT_name,
                        path=path,
                    )
                )

            job.set_instructions(job_instructions)
            new_jobs.append(job)

        return new_jobs

    def _summarize_variant(
        self, variant_name: str, planned_jobs: List[Job]
    ) -> VariantPlanResult:
        drive_time_per_sector = CONSTANT.HT_FLEET.HT_DRIVE_TIME_PER_SECTOR
        drive_times = []
        for job in planned_jobs:
            total_steps = 0
            for instruction in job.get_instructions():
                if instruction.get_instruction_type() == InstructionType.DRIVE:
                    path = instruction.get_paths() or []
                    total_steps += len(path)
            drive_times.append(total_steps * drive_time_per_sector)

        total_drive = sum(drive_times)
        max_drive = max(drive_times, default=0)
        avg_drive = total_drive / len(drive_times) if drive_times else 0.0

        return VariantPlanResult(
            name=variant_name,
            jobs=planned_jobs,
            total_drive_time=total_drive,
            max_drive_time=max_drive,
            average_drive_time=avg_drive,
        )
