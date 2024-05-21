
from copy import deepcopy

import rosys

from . import Weeding, WorkflowException


class WeedingScrew(Weeding):

    async def _perform_workflow(self) -> None:
        self.log.info('Performing Screw Workflow..')
        try:
            starting_position = deepcopy(self.system.odometer.prediction)
            self._keep_crops_safe()
            weeds_in_range = {weed_id: position for weed_id, position in self.weeds_to_handle.items(
            ) if position.x < self.system.field_friend.WORK_X + self.WORKING_DISTANCE and self.system.field_friend.can_reach(position)}
            if weeds_in_range:
                self.log.info(f'Weeds in range {len(weeds_in_range)}')
                while weeds_in_range:
                    next_weed_id, next_weed_position = list(self.crops_to_handle.items())[0]
                    weed_world_position = starting_position.transform(next_weed_position)
                    corrected_relative_weed_position = self.system.odometer.prediction.relative_point(
                        weed_world_position)
                    self.log.info(f'Targeting weed at {next_weed_position}')
                    if not self.only_monitoring:
                        await self.system.puncher.drive_and_punch(plant_id=next_weed_id,
                                                                  x=corrected_relative_weed_position.x, y=corrected_relative_weed_position.y, depth=self.weed_screw_depth, backwards_allowed=False)
                    punched_weeds = [weed_id for weed_id, position in weeds_in_range.items(
                    ) if position.distance(next_weed_position) <= self.system.field_friend.DRILL_RADIUS]
                    for weed_id in punched_weeds:
                        self.system.plant_provider.remove_weed(weed_id)
                        if weed_id in weeds_in_range:
                            del weeds_in_range[weed_id]
                        self.kpi_provider.increment_weeding_kpi('weeds_removed')

            elif self.crops_to_handle:
                await self._follow_line_of_crops()
            else:
                await self._driving_a_bit_forward()
            await rosys.sleep(0.2)
            self.log.info('Workflow completed')
        except Exception as e:
            raise WorkflowException(f'Error while Weed Screw Workflow: {e}') from e

    def _has_plants_to_handle(self) -> bool:
        super()._has_plants_to_handle()
        return any(self.weeds_to_handle)
