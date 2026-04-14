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
from topbar import TopBar, EventWidget

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

        self.disabledBtns = False
        activeDriveBtn = Button(text="Active Drive")
        activeDriveBtn.bind(on_press=lambda instance: self.swap_screen('realTime'))

        vehicleStateBtn = Button(text="Vehicle State")
        vehicleStateBtn.set_disabled(True)

        self._last_trip_btn_press_s = 0.0

        self.content = BoxLayout(
            orientation='horizontal',
            size_hint=(1, 1)
        )

        bottomBar = BoxLayout(
            orientation='horizontal',
            size_hint=(1, None),
            height=Window.height * 0.1
        )

        for btn in [activeDriveBtn, vehicleStateBtn]:
            btn.background_normal = ''
            btn.background_color = (1, 1, 1, 1)
            btn.color = (0, 0, 0, 1)
            btn.size_hint = (1, 1)
            bottomBar.add_widget(btn)

        # shadows
        imgBox = BoxLayout(
            orientation='vertical',
            size_hint=(0.25, 1)
        )

        labelBox = FloatLayout(pos_hint={'x': 0, 'y': 0}, size_hint=(0.25, 1))
        # labelBox.pos = (0, 0)

        # operationalBox = BoxLayout(orientation='horizontal',size_hint=(1, 1))
        self.currentOperationalState = "Unknown"
        self.opImg = Image(source=os.path.join("assets","operationalState", "unknownState.png"), size_hint=(None, None), height=128.1, width = 134.4, x=0, y=0)
        imgBox.add_widget(self.opImg)

        opTitle = Label(text="Operational State:",color=(0,0,0,1), font_size=20)
        opTitle.pos = (Window.width/4 - opTitle.width/2, Window.height/2 - opTitle.height/4)

        self.opLabel = Label(text=self.currentOperationalState, font_size=28, color=(0,0,0,1))
        self.opLabel.pos = (Window.width/4 - self.opLabel.width/2, Window.height/2 - opTitle.height/4 - self.opLabel.height/2)

        labelBox.add_widget(opTitle)
        labelBox.add_widget(self.opLabel)

        self.currentPowertrainState = "Unknown"
        self.ptImg = Image(source=os.path.join("assets","powertrainState", "unknownState.png"), size_hint=(None, None), height=128.1, width = 134.4)
        imgBox.add_widget(self.ptImg)

        ptTitle = Label(text="Powertrain State:",color=(0,0,0,1), font_size=20)
        ptTitle.pos = (Window.width/4 - ptTitle.width/2, Window.height/4 - ptTitle.height/3)
        self.ptLabel = Label(text=self.currentPowertrainState,color=(0,0,0,1), font_size=28)
        self.ptLabel.pos = (Window.width/4 - self.ptLabel.width/2, Window.height/4 - ptTitle.height/3 - self.ptLabel.height/2)
        labelBox.add_widget(ptTitle)
        labelBox.add_widget(self.ptLabel)

        self.currentThermalState = "Unknown"    
        self.thImg = Image(source=os.path.join("assets","thermalState", "thermUnknown.png"), size_hint=(None, None), height=128.1, width = 134.4)
        imgBox.add_widget(self.thImg)
        
        thTitle = Label(text="Thermal State:   ",color=(0,0,0,1), font_size=20)
        thTitle.pos = (Window.width/4 - thTitle.width/2, -Window.height/8 + thTitle.height/6)
        self.thLabel = Label(text=self.currentThermalState, color=(0,0,0,1), font_size=28)
        self.thLabel.pos = (Window.width/4 - self.thLabel.width/2, -Window.height/8 + thTitle.height/6 - self.thLabel.height/2)
        labelBox.add_widget(thTitle)
        labelBox.add_widget(self.thLabel)

        self.content.add_widget(imgBox)
        self.content.add_widget(labelBox)

        eventBox = BoxLayout(orientation='vertical', size_hint=(0.5, 1), spacing=5, padding=(5,5,5,5))
        eventWidget = EventWidget(width=Window.width*0.5 - 10)
        eventWidget.hide()
        eventWidget2 = EventWidget(width=Window.width*0.5 - 10)
        eventWidget2.hide()
        eventWidget3 = EventWidget(width=Window.width*0.5 - 10)
        eventWidget3.hide()
        eventWidget4 = EventWidget(width=Window.width*0.5 - 10)
        eventWidget4.hide()
        eventWidget5 = EventWidget(width=Window.width*0.5 - 10)
        eventWidget5.hide()

        self.eventWidgets = [eventWidget, eventWidget2, eventWidget3, eventWidget4, eventWidget5]
        self.eventsStored = {}
        
        eventBox.add_widget(eventWidget)
        eventBox.add_widget(eventWidget2)
        eventBox.add_widget(eventWidget3)
        eventBox.add_widget(eventWidget4)
        eventBox.add_widget(eventWidget5)
        
        self.content.add_widget(eventBox)

        self.boxLayout.add_widget(self.content)

        self.boxLayout.add_widget(bottomBar)
        self.add_widget(self.boxLayout)

        with self.boxLayout.canvas.after:
            Color(0, 0, 0, 1)
            Line(points=[ Window.width /2, Window.height - self.topbar.height, Window.width / 2, Window.height/10], width=1)
            # button borders
            Line(points=[0, Window.height/10, Window.width, Window.height/10], width=1)
            Line(points=[Window.width, 0, 0, 0], width=1)
            Line(points=[0, Window.height/10, 0, 0], width=1)
            Line(points=[Window.width, Window.height/10, Window.width, 0], width=1)
            # Line(points=[Window.width/4, Window.height/10, Window.width/4, 0], width=1)
            Line(points=[Window.width/2, Window.height/10, Window.width/2, 0], width=1)

        Clock.schedule_interval(self.refresh, 0.1)

    def reset(self):
        self.currentOperationalState = "Unknown"
        self.opImg.source = os.path.join("assets","operationalState", "unknownState.png")
        self.opLabel.text = self.currentOperationalState

        self.currentPowertrainState = "Unknown"
        self.ptImg.source = os.path.join("assets","powertrainState", "unknownState.png")
        self.ptLabel.text = self.currentPowertrainState

        self.currentThermalState = "Unknown"
        self.thImg.source = os.path.join("assets","thermalState", "thermUnknown.png")
        self.thLabel.text = self.currentThermalState

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

    def refresh_events(self):
        # Sort events by timestamp
        sorted_events = sorted(self.eventsStored.values(), key=lambda e: e.timestamp, reverse=True)

        for i, widget in enumerate(self.eventWidgets):
            if i < len(sorted_events):
                widget.update(sorted_events[i])
                widget.show() 
            else:
                widget.hide()

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
        self.thLabel.text = thermal

    def powertrain_to_img(self, powertrain):
        if powertrain == self.currentPowertrainState:
            return
        if powertrain == "Unknown":
            self.ptImg.source = os.path.join("assets","powertrainState", "unknownState.png")
        elif powertrain == "Engine Braking":
            self.ptImg.source = os.path.join("assets","powertrainState", "engineBrakingState.png")
        elif powertrain == "Neutral":
            self.ptImg.source = os.path.join("assets","powertrainState", "neutralState.png")
        elif powertrain == "Under Load":
            self.ptImg.source = os.path.join("assets","powertrainState", "underLoadState.png")
        elif powertrain == "High Load":
            self.ptImg.source = os.path.join("assets","powertrainState", "highLoadState.png")
        self.currentPowertrainState = powertrain
        self.ptLabel.text = powertrain

    def operational_to_img(self, operational):
        if operational == self.currentOperationalState:
            return
        if operational == "Unknown":
            self.opImg.source = os.path.join("assets","operationalState", "unknownState.png")
        elif operational == "Idling":
            self.opImg.source = os.path.join("assets","operationalState", "idleState.png")
        elif operational == "Accelerating":
            self.opImg.source = os.path.join("assets","operationalState", "accelState.png")
        elif operational == "Decelerating":
            self.opImg.source = os.path.join("assets","operationalState", "decelState.png")
        elif operational == "Cruising":
            self.opImg.source = os.path.join("assets","operationalState", "cruiseState.png")
        elif operational == "Stopped":
            self.opImg.source = os.path.join("assets","operationalState", "offState.png")
        self.currentOperationalState = operational
        self.opLabel.text = operational

    def refresh(self, dt):
        # if self.screenManager.current != 'vehicleState':
        #     return
        
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
            self.powertrain_to_img(shadow.powertrain)
            self.operational_to_img(shadow.operational)

        #    print(shadow.powertrain)
            # self.driverTitle.text = f"Powertrain: {shadow.powertrain} \nOperational: {shadow.operational} \nThermal: {shadow.thermal}"
            
            ## topbar update
            
            # if self.topbar.tripButton.text != "Stop Trip":
            #     self.topbar.tripButton.text = "Stop Trip"

            # state = self.analyser.get_most_recent()
            # if state is None:
            #     return

            self.eventsStored = state["allEvents"]
            if self.eventsStored is not None:
                self.refresh_events()
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