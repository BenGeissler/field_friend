import math
import threading
from pathlib import Path
import asyncio

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nicegui import Client, app, ui, ui_run
import rosys
from rosys.hardware import (CanHardware, RobotBrain, RobotHardware, RobotSimulation, SerialCommunication,
                            WheelsHardware, WheelsSimulation, communication, ModuleHardware)


class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.subscription = self.create_subscription(Pose, 'pose', self.handle_pose, 1)

        with Client.auto_index_client:
            with ui.row().classes('items-stretch'):
                with ui.card().classes('w-44 text-center items-center'):
                    ui.label('Keyboard_Control').classes('mt-6')
                    ui.keyboard(on_key=lambda e: self.send_speed(float(e.key == 'w'), float(e.key == 's')))
                    ui.label('Control').classes('text-2xl')
                    ui.joystick(color='blue', size=50,
                                on_move=lambda e: self.send_speed(float(e.y), float(e.x)),
                                on_end=lambda _: self.send_speed(0.0, 0.0))
                    ui.label('Publish steering commands by dragging your mouse around in the blue field').classes('mt-6')

                    

                with ui.card().classes('w-44 text-center items-center'):
                    ui.label('Data').classes('text-2xl')
                    ui.label('linear velocity').classes('text-xs mb-[-1.8em]')
                    slider_props = 'readonly selection-color=transparent'
                    self.linear = ui.slider(min=-1, max=1, step=0.05, value=0).props(slider_props)
                    ui.label('angular velocity').classes('text-xs mb-[-1.8em]')
                    self.angular = ui.slider(min=-1, max=1, step=0.05, value=0).props(slider_props)
                    ui.label('position').classes('text-xs mb-[-1.4em]')
                    self.position = ui.label('---')
                with ui.card().classes('w-96 h-96 items-center'):
                    ui.label('Visualization').classes('text-2xl')
                    with ui.scene(350, 300) as scene:
                        with scene.group() as self.robot_3d:
                            prism = [[-0.5, -0.5], [0.5, -0.5], [0.75, 0], [0.5, 0.5], [-0.5, 0.5]]
                            self.robot_object = scene.extrusion(prism, 0.4).material('#4488ff', 0.5)
                with ui.card().classes('w-44 text-center items-center'):
                    ui.label('Control').classes('text-2xl')
                    ui.button('Drive in Circle', on_click=lambda ello: self.send_speed(float(0.5), float(0.5)))
                    ui.button('Stop', on_click=lambda jello: self.send_speed(float(0.0), float(0.0)))
                    ui.button('Drive Rectangle', on_click=lambda mello: self.drive_rectangle())
                    

    def send_speed(self, x: float, y: float) -> None:
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = -y
        self.linear.value = x
        self.angular.value = y
        self.cmd_vel_publisher.publish(msg)

    def handle_pose(self, msg: Pose) -> None:
        self.position.text = f'x: {msg.position.x:.2f}, y: {msg.position.y:.2f}'
        self.robot_3d.move(msg.position.x, msg.position.y)
        self.robot_3d.rotate(0, 0, 2 * math.atan2(msg.orientation.z, msg.orientation.w))
    
    async def drive_rectangle(self):
        for i in range(4):
        # Drive forward for 3 seconds
            self.send_speed(0.5, 0.0)
            await asyncio.sleep(2)

            # Turn left for 2 seconds
            self.send_speed(0.0, 0.5)
            await asyncio.sleep(7.5)
            i +=1
        self.send_speed(0.0, 0.0)


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass

# shape = rosys.geometry.Prism.default_robot_shape()
# is_real = SerialCommunication.is_possible()
# try:
#     communication = SerialCommunication()
#     robot_brain = RobotBrain(communication)
#     can = CanHardware(robot_brain)
#     wheels = WheelsHardware(robot_brain, can=can, left_can_address=0x000, right_can_address=0x100,  m_per_tick=0.05567092651757189, width=0.47, is_right_reversed=True) 
#     robot = rosys.hardware.RobotHardware([can, wheels], robot_brain)
#     print("----------------------------------------\n This works TestOwnCode.py \n----------------------------------------")
# except Exception as e:
#     print("----------------------------------------\n This doesn't work \n ------------------------------------------------")
#     print(e)
#     wheels = rosys.hardware.WheelsSimulation()
#     robot = rosys.hardware.RobotSimulation([wheels])
# odometer = rosys.driving.Odometer(wheels)
# steerer = rosys.driving.Steerer(wheels)


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')
