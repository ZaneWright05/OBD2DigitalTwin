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
from kivy.uix.screenmanager import NoTransition

from time import time
from tachometer import Tachometer
from topbar import TopBar
from vehicleStateScreen import VehicleStateScreen

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))

from model.dataParser import Parser
from model.vehicleState import VehicleState

imgDir = os.path.join(os.path.dirname(__file__), "assets")
screen_width_px = 800 # 154.08 mm
screen_width_mm = 154.08
screen_height_px = 480 # 85.92 mm
screen_height_mm = 85.92

def convert_dimensions(width_mm, height_mm):
    if os.name == 'nt':  # Windows
        px_per_mm_x = 3.78
        px_per_mm_y = 3.78
    else:
        px_per_mm_x = screen_width_px / screen_width_mm
        px_per_mm_y = screen_height_px / screen_height_mm
    width_px = int(width_mm * px_per_mm_x)
    height_px = int(height_mm * px_per_mm_y)
    return width_px, height_px

def bind_widget_to_parent(widget, parent, pos_func):
    def update_pos(*args):
        widget.pos = pos_func(parent, widget)
    parent.bind(size=update_pos)
    update_pos()

class MetricWidget(BoxLayout):
    def __init__(self, imageName, label_text, **kwargs):
        super().__init__(**kwargs)
        self.orientation = 'horizontal'
        self.size_hint = (None, 1)
        self.spacing = 12
        self.bind(minimum_width=self.setter('width'))
        self.image = Image(
            source=os.path.join(imgDir, imageName),
            size_hint=(None, 1),
            size=(40, 40),
            fit_mode='contain'
        )

        self.label = Label(
            text=label_text,
            size_hint=(None, 1),
            width=100,
            color=(0, 0, 0, 1),
            font_size=20,
            halign='left',
            valign='middle'
        )
        self.label.bind(size=lambda w, _: setattr(w, 'text_size', w.size))
        self.add_widget(self.image)
        self.add_widget(self.label)

class RealTimeScreen(Screen):
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
        self.content = Widget()

        gearBox = Widget(size=(Window.width/4, Window.height - self.topbar.height - Window.height/10), pos=(3*Window.width/4, self.topbar.height))
        self.estGearLabel = Label(text="N",
                                size=(185, 40), 
                                pos=(gearBox.pos[0] + 10, gearBox.pos[1] + 325),
                                color=(0, 0, 0, 1),
                                font_size=48)
        
        gears = GridLayout(cols=2, size=(Window.width/4, Window.height/2),pos=(gearBox.pos[0], gearBox.pos[1]  + Window.height/6), spacing=8, padding=8)
        for i in range(1, 7):
            btn = Button(text=f"{i}",
            size_hint=(None, None),  
            size=convert_dimensions(17.5, 17.5),
            color=(0, 0, 0, 1),
            background_normal='', background_color=(1, 1, 1, 1))
            btn.bind(on_press=lambda instance, x=i: analyser.store_gear(int(x)))
            # btn.bind(on_press=lambda instance, x=i: self.set_event_text(f"Manually set gear to {x}"))
            gears.add_widget(btn)

        gearBox.add_widget(self.estGearLabel)
        gearBox.add_widget(gears)

        self.tach = Tachometer(200)
        self.tach.pos = (20, 220)
        self.content.add_widget(self.tach)

        metricBox = BoxLayout(orientation='vertical', size_hint=(None, None), size=(170, 140))
        metricBox.pos = (315,200)
        with metricBox.canvas.before:
            Color(1, 1, 1, 1)
            self.metricBox_bg = RoundedRectangle(
                size=(metricBox.width + 10, metricBox.height + 10),
                pos=(metricBox.x - 5, metricBox.y - 5),
                radius=[10]
            )
            Color(0, 0, 0, 1)
            Line(
                rounded_rectangle=(self.metricBox_bg.pos[0], self.metricBox_bg.pos[1], self.metricBox_bg.size[0], self.metricBox_bg.size[1], 10),
                width=1
            )

        self.accMet = MetricWidget("acceleration.png", "-- m/s²")
        metricBox.add_widget(self.accMet)
        self.voltMet = MetricWidget("voltage.png", "-- V")
        metricBox.add_widget(self.voltMet)
        self.tempMet = MetricWidget("temperature.png", "-- °C")
        metricBox.add_widget(self.tempMet)

        self.speedLabel = Label(text="-- km/h", size_hint=(None, None), color=(0, 0, 0, 1), font_size=32)
        self.speedLabel.pos = (400 - self.speedLabel.width / 2, 340)
        self.content.add_widget(self.speedLabel)

        self.content.add_widget(metricBox)
        self.content.add_widget(gearBox)

        fuelTitle = Label(text="Est. Fuel Cons", size_hint=(None, None), color=(0, 0, 0, 1), font_size=26)
        fuelTitle.pos=(Window.width/2 + fuelTitle.width/2.5, 165 - fuelTitle.height/2)

        self.instConsLabel = Label(text="Inst: 0.00", size_hint=(None, None), color=(0, 0, 0, 1), font_size=24)
        self.instConsLabel.pos=(Window.width/2 + 20, 125 - self.instConsLabel.height/2)
        constUnitLabel = Label(text="L/100km", size_hint=(None, None), color=(0, 0, 0, 1), font_size=12)
        constUnitLabel.pos=(self.instConsLabel.x + self.instConsLabel.width - 10, self.instConsLabel.y - 2.5)
        
        self.aveConsLabel = Label(text="Ave: 0.00", size_hint=(None, None), color=(0, 0, 0, 1), font_size=24)
        self.aveConsLabel.pos=(Window.width/2 + 20, 85 - self.aveConsLabel.height/2)
        constUnitLabel2 = Label(text="L/100km", size_hint=(None, None), color=(0, 0, 0, 1), font_size=12)
        constUnitLabel2.pos=(self.aveConsLabel.x + self.aveConsLabel.width - 10, self.aveConsLabel.y - 2.5)

        self.content.add_widget(fuelTitle)
        self.content.add_widget(self.instConsLabel)
        self.content.add_widget(constUnitLabel)
        self.content.add_widget(self.aveConsLabel)
        self.content.add_widget(constUnitLabel2)

        engLoadTitle = Label(text="Eng. Load", size_hint=(None, None), color=(0, 0, 0, 1), font_size=20)
        engLoadTitle.pos = (0, 180 - (engLoadTitle.height/1.5))
        self.engLoadLabel = Label(text="-- %", size_hint=(None, None), color=(0, 0, 0, 1), font_size=32)
        self.engLoadLabel.pos = (Window.width/8 - self.engLoadLabel.width/2, 180 - 1.25*(self.engLoadLabel.height))

        thrttleTitle = Label(text="Throttle", size_hint=(None, None), color=(0, 0, 0, 1), font_size=20)
        thrttleTitle.pos = (Window.width/4, 180 - (thrttleTitle.height/1.5))
        self.throttleLabel = Label(text="-- %", size_hint=(None, None), color=(0, 0, 0, 1), font_size=32)
        self.throttleLabel.pos = (3*(Window.width/8) - self.throttleLabel.width/2, 180 - 1.25*(self.throttleLabel.height))

        self.content.add_widget(engLoadTitle)
        self.content.add_widget(self.engLoadLabel)
        self.content.add_widget(thrttleTitle)
        self.content.add_widget(self.throttleLabel)

        activeDriveBtn = Button(text="Active Drive", size_hint=(None, None), size=(Window.width / 2, Window.height/10), pos=(0, 0))
        activeDriveBtn.set_disabled(True)

        vehicleStateBtn = Button(text="Vehicle State", size_hint=(None, None), size=(Window.width / 2, Window.height/10), pos=(Window.width / 2, 0))
        vehicleStateBtn.bind(on_press=lambda instance: self.swap_screen('vehicleState'))
        

        # self.disabledBtns = False
        # self.replayBtn = Button(text="Replay", size_hint=(None, None), size=(Window.width / 4, Window.height/10), pos=(Window.width / 2, 0))
        # self.settingsBtn = Button(text="Settings", size_hint=(None, None), size=(Window.width / 4, Window.height/10), pos=(3 * Window.width / 4, 0))

        for btn in [activeDriveBtn, vehicleStateBtn]:
            btn.background_normal = ''
            btn.background_color = (1, 1, 1, 1)
            btn.color = (0, 0, 0, 1)
            self.content.add_widget(btn)

        with self.content.canvas:
            Color(0, 0, 0, 1)
            Line(points=[0, 180, 3 * Window.width / 4, 180, 3 * Window.width / 4, Window.height/10], width=1)
            Line(points=[Window.width/4, 180, Window.width/4, Window.height/10], width=1)
            Line(points=[Window.width/2, 180, Window.width/2, 0], width=1)
            Line(points=[0, Window.height/10, Window.width, Window.height/10], width=1)
            Line(points=[Window.width, 0, 0, 0], width=1)
            Line(points=[0, Window.height/10, 0, 0], width=1)
            Line(points=[Window.width, Window.height/10, Window.width, 0], width=1)
            # Line(points=[Window.width/4, Window.height/10, Window.width/4, 0], width=1)


        self.boxLayout.add_widget(self.content)
        self.add_widget(self.boxLayout)
        Clock.schedule_interval(self.refresh, 0.1)

    def set_event_text(self, msg):
        self.topbar.eventLabel.text = msg

    def swap_screen(self, screen_name):
        # print(f"Swapping to screen: {screen_name}")
        screen = self.screenManager.get_screen(screen_name)
        if screen:
            state = self.analyser.get_most_recent()
            if state is not None:
                self.topbar.update_topBar(state)
            self.screenManager.current = screen_name


    def refresh(self, dt):
        if self.screenManager.current != 'realTime':
            return
    
        state = self.analyser.get_most_recent()
        if state is None:
            self.speedLabel.text = "0 km/h"
            self.accMet.label.text = "0.00 m/s²"
            self.tach.set_target_rpm(0)
            self.tach.rpmLabel.text = "0 RPM"
            self.voltMet.label.text = "-- V"
            self.tempMet.label.text = "-- °C"
            self.instConsLabel.text = "Inst: 0.00"
            self.aveConsLabel.text = "Ave: 0.00"
            self.estGearLabel.text = "N"
            self.engLoadLabel.text = "-- %"
            self.throttleLabel.text = "-- %"
            return

        self.topbar.update_topBar(state)

        if self.analyser.connected and self.analyser.running:

            state = self.analyser.get_most_recent()
            if state is None:
                return
                
            speed = state["speed"].metrics if state["speed"].metrics else None
            if speed is not None and speed.current is not None:
                self.speedLabel.text = f"{int(speed.current * 3.6)} km/h"
            else:
                self.speedLabel.text = "0 km/h"

            accStr = " 0.00 m/s²"
            if speed and speed.wAvgROC is not None:
                if(speed.wAvgROC) >= 0:
                    accStr = f" {speed.wAvgROC:.2f} m/s²"
                else:
                    accStr = f"{speed.wAvgROC:.2f} m/s²"
            self.accMet.label.text = accStr

            rpm = state["rpm"].metrics.current if state["rpm"].metrics else None
            self.tach.set_target_rpm(rpm)
            self.tach.rpmLabel.text = f"{int(rpm) if rpm is not None else '0'} RPM"

            volt = state["volt"].metrics.current if state["volt"].metrics else None
            self.voltMet.label.text = f"{volt:.2f} V" if volt is not None else "-- V"

            temp = state["temp"].metrics.current if state["temp"].metrics else None
            self.tempMet.label.text = f"{int(temp)} °C" if temp is not None else "-- °C"

            fCons = state["fuelCons"]
            self.instConsLabel.text = f"Inst: {fCons.metrics.current:.2f}" if fCons is not None and fCons.metrics.current is not None else "Inst: 0.00"
            self.aveConsLabel.text = f"Ave: {fCons.all_trip_average():.2f}" if fCons is not None and fCons.all_trip_average() is not None else "Ave: 0.00"

            gear = state["gear"]
            self.estGearLabel.text = f"{gear if gear != 0 else 'N'}"

            load = state["load"].metrics.current if state["load"].metrics else None
            self.engLoadLabel.text = f"{load:.0f} %" if load is not None else "-- %"
            throttle = state["throttle"].metrics.current if state["throttle"].metrics else None
            self.throttleLabel.text = f"{throttle:.0f} %" if throttle is not None else "-- %"

    def update_bg_rect(self, *args):
        self.bg_rect.pos = self.pos
        self.bg_rect.size = self.size

class MyApp(App):
    def build(self):
        self.analyser = Parser()
        self.vehicleState = VehicleState()
        self.topbar = TopBar(analyser=self.analyser, vehicleState=self.vehicleState)

        self.analyser.start_parsing(mode="serial")
        # self.analyser.start_parsing(mode="csv", csv_path="", sample_rate=64)

        self.screenManager = ScreenManager()
        self.screenManager.transition = NoTransition()

        self.realTimeScreen = RealTimeScreen(self.screenManager, self.analyser, self.vehicleState, name='realTime')
        self.screenManager.add_widget(self.realTimeScreen)

        self.vehicleStateScreen = VehicleStateScreen(self.screenManager, self.analyser, self.vehicleState, name='vehicleState')
        self.screenManager.add_widget(self.vehicleStateScreen)

        print("Available screens in ScreenManager:")
        print([screen.name for screen in self.screenManager.screens])
        return self.screenManager



if __name__ == "__main__":
    MyApp().run()