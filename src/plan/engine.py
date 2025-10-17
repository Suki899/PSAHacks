from collections import namedtuple
from typing import List

import pandas as pd

from src.plan.job_planner import JobPlanner, VariantPlanResult
from src.plan.job_tracker import JobTracker


class PlanningEngine:
    """
    Coordinates the planning of jobs by integrating job tracking and job planning components.

    Parameters
    ----------
    job_df : pd.DataFrame
        DataFrame containing job information used to initialize the job tracker.
    monitoring_resources : namedtuple
        A namedtuple containing monitoring resources required for planning, including
        HT coordinate tracker and sector map snapshot.

    Attributes
    ----------
    job_tracker : JobTracker
        Tracks and manages jobs parsed from the input DataFrame.
    job_planner : JobPlanner
        Manages job planning activities based on HT coordinates and sector map state.
    """

    def __init__(
        self,
        job_df: pd.DataFrame,
        monitoring_resources: namedtuple,
    ):
        self.job_tracker: JobTracker = JobTracker(job_df)
        self.job_planner: JobPlanner = JobPlanner(
            ht_coord_tracker=monitoring_resources.HT_coord_tracker,
            sector_map_snapshot=monitoring_resources.sector_map_snapshot,
        )

    def fetch_job_status(self):
        self.job_tracker.fetch_and_update_job_status()

    def is_all_job_completed(self):
        return self.job_tracker.is_all_job_completed()

    def is_deadlock(self):
        return self.job_planner.is_deadlock()

    def get_non_moving_HT(self):
        return self.job_planner.get_non_moving_HT()

    def get_number_of_completed_jobs(self):
        return self.job_tracker.get_number_of_completed_jobs()

    def export_job_report(self):
        return self.job_tracker.export_job_report()

    def set_planner_variant(self, variant_name: str):
        self.job_planner.set_default_variant(variant_name)

    def plan(self, variant_name: str = None):
        return self.job_planner.plan(self.job_tracker, variant_name=variant_name)

    def plan_variants(
        self, variant_names: List[str] = None
    ) -> List[VariantPlanResult]:
        return self.job_planner.plan_variants(
            self.job_tracker, variant_names=variant_names
        )
