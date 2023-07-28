import logging
import uuid

import rosys
from nicegui import ui

from ..navigation import Gnss, Path, PathProvider


class path_planner:

    def __init__(
            self, path_provider: PathProvider, automator: rosys.automation.Automator, driver: rosys.driving.Driver,
            gnss: Gnss) -> None:
        self.path_recorder = path_provider
        self.automator = automator
        self.driver = driver
        self.gnss = gnss
        self.log = logging.getLogger('field_friend.path_planner')
        self.new_name = ''
        with ui.card():
            with ui.row():
                ui.button('Add path', on_click=self.add_path)
                ui.button('Clear paths', on_click=self.clear_paths)
            with ui.row():
                self.show_path_settings()
        # rosys.on_repeat(self.check_for_reference, 1)

    @ ui.refreshable
    def show_path_settings(self) -> None:
        for path in self.path_recorder.paths:
            with ui.card().classes('items-stretch'):
                with ui.row():
                    ui.input('name', value=f'{path.name}').bind_value(path, 'name')
                    ui.button(on_click=lambda path=path: self.delete_path(path)).props(
                        'icon=delete color=warning fab-mini flat').classes('ml-auto')

                with ui.row().classes('items-canter'):
                    with ui.row().bind_visibility_from(self.path_recorder, 'current_path_recording', lambda p, name=path.name: p != name):
                        ui.button('record', on_click=lambda path=path: self.automator.start(
                            self.path_recorder.record_path(path))).props(
                            'icon=radio_button_checked color=grey fab-mini flat').tooltip('start recording')
                    with ui.row().bind_visibility_from(self.path_recorder, 'current_path_recording', lambda p, name=path.name: p == name):
                        ui.button(
                            'recording...', on_click=self.stop_recording).props(
                            'icon=radio_button_checked color=red fab-mini flat').bind_visibility_from(
                            self.path_recorder, 'state', lambda s: s == 'recording').tooltip('stop recording')
                    with ui.row().bind_visibility_from(
                            path, 'path', lambda path: path != []):
                        ui.button('play', on_click=lambda path=path: self.automator.start(
                            self.path_recorder.drive_path(path))).props(
                            'icon=play_arrow fab-mini flat').tooltip('drive recorded path').bind_visibility_from(self.path_recorder,
                                                                                                                 'state', lambda s: s == 'idle')
                        ui.button('stop', on_click=self.stop_driving).props(
                            'icon=stop fab-mini flat').tooltip('stop driving').bind_visibility_from(self.path_recorder,
                                                                                                    'state', lambda s: s == 'driving')

    def stop_recording(self) -> None:
        self.path_recorder.state = 'idle'
        self.path_recorder.current_path_recording = ''
        self.show_path_settings.refresh()

    def stop_driving(self) -> None:
        self.automator.stop('Stopped because stop button was pressed')
        self.path_recorder.state = 'idle'
        self.show_path_settings.refresh()

    def add_path(self) -> None:
        path = Path(name=f'{str(uuid.uuid4())}')
        self.path_recorder.add_path(path)
        self.show_path_settings.refresh()

    def delete_path(self, path: Path) -> None:
        self.path_recorder.remove_path(path)
        self.show_path_settings.refresh()

    def clear_paths(self) -> None:
        self.path_recorder.clear_paths()
        self.show_path_settings.refresh()

    def check_for_reference(self) -> None:
        if self.gnss.reference_lat is None or self.gnss.reference_lon is None:
            self.clear_paths()
