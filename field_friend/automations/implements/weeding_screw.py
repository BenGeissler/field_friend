
from copy import deepcopy
from typing import TYPE_CHECKING

import rosys
from nicegui import ui

from . import ImplementException, WeedingImplement

if TYPE_CHECKING:
    from system import System


class WeedingScrew(WeedingImplement):

    def __init__(self, system: 'System') -> None:
        super().__init__('Weed Screw', system)
        self.relevant_weeds = system.small_weed_category_names + system.big_weed_category_names
        self.log.info(f'Using relevant weeds: {self.relevant_weeds}')

    async def start_workflow(self) -> None:
        await super().start_workflow()
        try:
            self._keep_crops_safe()
            weeds_in_range = {weed_id: position for weed_id, position in self.weeds_to_handle.items()
                              if position.x < self.system.field_friend.WORK_X + self.WORKING_DISTANCE and self.system.field_friend.can_reach(position)}
            if not weeds_in_range:
                return
            self.log.info(f'Weeds in range {[f"{p.x:.3f},{p.y:.3f}" for p in weeds_in_range.values()]}')
            next_weed_id, next_weed_position = list(weeds_in_range.items())[0]
            weed_world_position = self.system.odometer.prediction.transform(next_weed_position)
            self.log.info(f'Targeting weed at world: {weed_world_position}, local: {next_weed_position}')
            await self.system.puncher.drive_and_punch(plant_id=next_weed_id,
                                                      x=next_weed_position.x,
                                                      y=next_weed_position.y,
                                                      depth=self.weed_screw_depth,
                                                      backwards_allowed=False)
            punched_weeds = [weed_id for weed_id, position in weeds_in_range.items()
                             if position.distance(next_weed_position) <= self.system.field_friend.DRILL_RADIUS
                             or weed_id == next_weed_id]
            for weed_id in punched_weeds:
                self.system.plant_provider.remove_weed(weed_id)
                if weed_id in weeds_in_range:
                    del weeds_in_range[weed_id]
                self.kpi_provider.increment_weeding_kpi('weeds_removed')
            if isinstance(self.system.detector, rosys.vision.DetectorSimulation):
                screw_world_position = self.system.odometer.prediction.transform(
                    rosys.geometry.Point(x=self.system.field_friend.WORK_X, y=self.system.field_friend.y_axis.position))
                self.log.info(f'removing weeds at screw world position {screw_world_position} '
                              f'with radius {self.system.field_friend.DRILL_RADIUS}')
                self.system.detector.simulated_objects = [
                    obj for obj in self.system.detector.simulated_objects
                    if obj.position.projection().distance(screw_world_position) > self.system.field_friend.DRILL_RADIUS]
        except Exception as e:
            raise ImplementException(f'Error while Weed Screw Workflow: {e}') from e

    def _has_plants_to_handle(self) -> bool:
        super()._has_plants_to_handle()
        return any(self.weeds_to_handle)

    def settings_ui(self):
        super().settings_ui()
        ui.number('Drill depth', value=0.02, format='%.2f', step=0.01,
                  min=self.system.field_friend.z_axis.max_position, max=self.system.field_friend.z_axis.min_position*-1) \
            .props('dense outlined suffix=°') \
            .classes('w-24') \
            .bind_value(self, 'weed_screw_depth') \
            .tooltip('Set the drill depth for the weeding automation')
        ui.number('Crop safety distance', value=0.01, step=0.001, min=0.001, max=0.05, format='%.3f') \
            .props('dense outlined suffix=m') \
            .classes('w-24') \
            .bind_value(self, 'crop_safety_distance') \
            .tooltip('Set the crop safety distance for the weeding automation')

    def _keep_crops_safe(self) -> None:
        self.log.info('Keeping crops safe...')
        for crop in self.system.plant_provider.crops:
            crop_position = self.system.odometer.prediction.relative_point(crop.position)
            for weed, weed_position in self.weeds_to_handle.items():
                offset = self.system.field_friend.DRILL_RADIUS + \
                    self.crop_safety_distance - crop_position.distance(weed_position)
                if offset > 0:
                    safe_weed_position = weed_position.polar(offset, crop_position.direction(weed_position))
                    self.weeds_to_handle[weed] = safe_weed_position
                    self.log.info(f'Moved weed {weed} from {weed_position} to {safe_weed_position} ' +
                                  f'by {offset} to safe {crop.id} at {crop_position}')
