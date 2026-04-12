from kivy.config import Config
import os

if os.name != 'nt': 
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'  # Force software rendering on Linux

Config.set('graphics', 'width', '800')
Config.set('graphics', 'height', '480')
Config.set('graphics', 'resizable', '0')
Config.set('graphics', 'borderless', '1')

from kivy.core.window import Window

if os.name != 'nt': 
    Window.show_cursor = False


from kivy.app import App
from kivy.uix.widget import Widget
from kivy.graphics import Color, Rectangle, Line, RoundedRectangle
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.image import Image
from kivy.clock import Clock
from threading import Thread
from kivy.uix.label import Label
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.button import Button
from kivy.uix.gridlayout import GridLayout
from kivy.uix.screenmanager import ScreenManager, Screen

from time import time
from tachometer import Tachometer
from topbar import TopBar

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))

from model.dataParser import Parser
from model.vehicleState import VehicleState

class VehicleStateScreen(Screen):
    def __init__(self, screenManager, analyser, vehicleState, **kwargs):
        super().__init__(**kwargs)
        self.screenManager = screenManager

        self.boxLayout = BoxLayout()
        self.boxLayout.orientation = 'vertical'
        self.analyser = analyser
        self.vehicleState = vehicleState

        with self.canvas.before:
            Color(0.7, 0.7, 0.7, 1)
            self.bg_rect = Rectangle(pos=self.pos, size=Window.size)

        self.bind(size=self.update_bg_rect, pos=self.update_bg_rect)
        
        self.topbar = TopBar(analyser=self.analyser, vehicleState=self.vehicleState)
        self.boxLayout.add_widget(self.topbar)
        # ?self.topbar.tripButton.bind(on_press=lambda _: self.stop_start_button_handler())
        self.content = Widget()

        activeDriveBtn = Button(text="Active Drive", size_hint=(None, None), size=(Window.width / 4, Window.height/10), pos=(0, 0))
        activeDriveBtn.bind(on_press=lambda instance: self.swap_screen('realTime'))
                     #self.screenManager.switch_to(self.screenManager.get_screen('realTime')))
        vehicleStateBtn = Button(text="Vehicle State", size_hint=(None, None), size=(Window.width / 4, Window.height/10), pos=(Window.width / 4, 0))
        vehicleStateBtn.set_disabled(True)


        self.disabledBtns = False
        self.replayBtn = Button(text="Replay", size_hint=(None, None), size=(Window.width / 4, Window.height/10), pos=(Window.width / 2, 0))
        self.settingsBtn = Button(text="Settings", size_hint=(None, None), size=(Window.width / 4, Window.height/10), pos=(3 * Window.width / 4, 0))


        self._last_trip_btn_press_s = 0.0

        for btn in [activeDriveBtn, vehicleStateBtn, self.replayBtn, self.settingsBtn]:
            btn.background_normal = ''
            btn.background_color = (1, 1, 1, 1)
            btn.color = (0, 0, 0, 1)
            self.content.add_widget(btn)

        with self.content.canvas:
            Color(0, 0, 0, 1)
            Line(points=[3 * Window.width / 4, Window.height - self.topbar.height, 3 * Window.width / 4, 0], width=1)


        # shadows
        shadowBox = BoxLayout(orientation='vertical', size_hint=(None, None), height=Window.height - self.topbar.height - activeDriveBtn.height, width=3 * Window.width / 4)


        ## sizing and positioning required
        operationalBox = BoxLayout(orientation='horizontal', size_hint=(None, None))
        self.opImg = Image(source=os.path.join("assets","operationalState", "unknownState.png"), size_hint=(None, None))
        shadowBox.add_widget(self.opImg)

        powertrainBox = BoxLayout(orientation='horizontal', size_hint=(None, None))
        self.ptImg = Image(source=os.path.join("assets","powertrainState", "unknownPTState.png"), size_hint=(None, None))
        shadowBox.add_widget(self.ptImg)

        self.currentThermalState = "Unknown"
        thermalBox = BoxLayout(orientation='horizontal', size_hint=(None, None))
        self.thImg = Image(source=os.path.join("assets","thermalState", "thermUnknown.png"), size_hint=(None, None))
        shadowBox.add_widget(self.thImg)

        self.content.add_widget(shadowBox)

        self.boxLayout.add_widget(self.content)
        self.add_widget(self.boxLayout)
        Clock.schedule_interval(self.refresh, 0.1)

    def update_bg_rect(self, *args):
        self.bg_rect.pos = self.pos
        self.bg_rect.size = self.size

    def swap_screen(self, screen_name):
        screen = self.screenManager.get_screen(screen_name)
        if screen:
            state = self.analyser.get_most_recent()
            if state is None:
                return
            else:
                self.topbar.update_topBar(state)
            self.screenManager.current = screen_name

    def thermal_to_img(self, thermal):
        if thermal == self.currentThermalState:
            return
        if thermal == "Unknown":
            self.thImg.source = os.path.join("assets","thermalState", "thermUnknown.png")
        elif thermal == "Cold Start":
            self.thImg.source = os.path.join("assets","thermalState", "thermColdStrt.png")
        elif thermal == "Cold Start: Warming":
            self.thImg.source = os.path.join("assets","thermalState", "thermColdWarming.png")
        elif thermal == "Warm Start":
            self.thImg.source = os.path.join("assets","thermalState", "thermWarmStrt.png")
        elif thermal == "Warm Start: Warming":
            self.thImg.source = os.path.join("assets","thermalState", "thermWarmStrtWarming.png")
        elif thermal == "Normal Running":
            self.thImg.source = os.path.join("assets","thermalState", "thermNormalRunning.png")
        elif thermal == "Overheating":
            self.thImg.source = os.path.join("assets","thermalState", "thermOverheating.png")
        elif thermal == "Overheat Risk":
            self.thImg.source = os.path.join("assets","thermalState", "thermOverheatRisk.png")
        elif thermal == "Cooling":
            self.thImg.source = os.path.join("assets","thermalState", "thermCooling.png")
        self.currentThermalState = thermal

    def refresh(self, dt):
        if self.screenManager.current != 'vehicleState':
            return
        
        state = self.analyser.get_most_recent()
        if state is None:
            return

        self.topbar.update_topBar(state)

        # if self.analyser.connected != self.topbar.connectionState:
        #     self.topbar.set_connection(self.analyser.connected)

        # if not self.analyser.running and self.topbar.tripButton.text == "Start Trip":
        #     self.topbar.tripButton.text = "Start Trip"

        if self.analyser.connected and self.analyser.running:
            # update vehicle state
            shadow =self.vehicleState.update(self.analyser.get_snapshot())
            self.thermal_to_img(shadow.thermal)
        #    print(shadow.powertrain)
            # self.driverTitle.text = f"Powertrain: {shadow.powertrain} \nOperational: {shadow.operational} \nThermal: {shadow.thermal}"
            
            ## topbar update
            
            # if self.topbar.tripButton.text != "Stop Trip":
            #     self.topbar.tripButton.text = "Stop Trip"

            state = self.analyser.get_most_recent()
            if state is None:
                return
            # self.topbar.timeLabel.text = state["time"]
            # self.topbar.distLabel.text = f"{state['distance']} km"


            speed = state["speed"].metrics if state["speed"].metrics else None
            # TODO: maybe also disble when trip running
            if speed is not None and speed.current is not None:
                if speed.current > 5 and not self.disabledBtns:
                    self.replayBtn.set_disabled(True)
                    self.settingsBtn.set_disabled(True)
                elif speed.current <= 5 and not self.disabledBtns:
                    self.replayBtn.set_disabled(False)
                    self.settingsBtn.set_disabled(False)
            else:
                self.replayBtn.set_disabled(False)
                self.settingsBtn.set_disabled(False)

            # freshness = state["freshness"]
            # if freshness is not None:
            #     if freshness > 0.95:
            #         self.topbar.set_signal("high")
            #     elif freshness > 0.33:
            #         self.topbar.set_signal("medium")
            #     else:
            #         self.topbar.set_signal("low")

            # event = state["event"]
            # if event is None:
            #     if not self.topbar.eventLabelHidden:
            #         self.topbar.eventLabelHidden = True
            #         self.topbar.eventLabel.opacity = 0
            #         self.topbar.eventLabel.disabled = True
            #         self.topbar.set_event_text("")
            # else:
            #     self.topbar.eventLabelHidden = False
            #     self.topbar.eventLabel.opacity = 1
            #     self.topbar.eventLabel.disabled = False
            #     # print(f"Event detected in UI: {event}")
            #     endStr = f"ended duration {event.length * event.pid.period_ms / 1000:.1f} s" if event.ended else "detected"
            #     startTime_ms = event.timestamp * event.pid.period_ms
            #     startTime = f"{int(startTime_ms // 60000):02d}:{int((startTime_ms % 60000) // 1000):02d}"
            #     self.topbar.set_event_text(f"[{startTime}] {event.pid.name} - {event.type} {endStr}")