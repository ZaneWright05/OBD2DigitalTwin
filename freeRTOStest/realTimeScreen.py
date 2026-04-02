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

from tachometer import Tachometer
from topbar import TopBar
from dataParser import Analyser, read_csv, read_from_com

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
        self.spacing = 4
        self.bind(minimum_width=self.setter('width'))
        self.image = Image(
            source=os.path.join(imgDir, imageName),
            size_hint=(None, 1),
            size=(50, 50),
            fit_mode='contain'
        )

        self.label = Label(
            text=label_text,
            size_hint=(None, 1),
            width=100,
            color=(0, 0, 0, 1),
            font_size=18,
            halign='left',
            valign='middle'
        )
        self.label.bind(size=lambda w, _: setattr(w, 'text_size', w.size))
        self.add_widget(self.image)
        self.add_widget(self.label)

class RealTimeScreen(BoxLayout):
    def __init__(self, analyser, **kwargs):
        super().__init__(**kwargs)

        self.orientation = 'vertical'
        self.analyser = analyser

        with self.canvas.before:
            Color(0.7, 0.7, 0.7, 1)
            self.bg_rect = Rectangle(pos=self.pos, size=Window.size)

        self.bind(size=self.update_bg_rect, pos=self.update_bg_rect)

        self.topbar = TopBar()
        self.add_widget(self.topbar)

        self.content = Widget()

        gearBox = BoxLayout(orientation='vertical', size_hint=(None, None), size=(185, 390), pos=(610,0))
        self.estGearLabel = Label(text="Gear: -", size_hint=(1, 0.1),
                                   color=(0, 0, 0, 1), 
                                   font_size=18)
        
        gears = GridLayout(cols=2, size_hint=(1, 0.6), spacing=8, padding=8)
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

        metricBox = BoxLayout(orientation='vertical', size_hint=(None, None), size=(200, 100))
        metricBox.pos = (300,200)
        self.accMet = MetricWidget("acceleration.png", "-- m/s²")
        metricBox.add_widget(self.accMet)
        self.voltMet = MetricWidget("voltage.png", "-- V")
        metricBox.add_widget(self.voltMet)
        self.tempMet = MetricWidget("temperature.png", "-- °C")
        metricBox.add_widget(self.tempMet)

        self.speedLabel = Label(text="-- km/h", pos=(300, 300), size_hint=(None, None), color=(0, 0, 0, 1), font_size=18)
        self.content.add_widget(self.speedLabel)

        self.content.add_widget(metricBox)
        self.content.add_widget(gearBox)
        self.add_widget(self.content)
        Clock.schedule_interval(self.refresh, 0.1) # refresh every 0.1s

    def set_event_text(self, msg):
        self.topbar.eventLabel.text = msg

    def refresh(self, dt):
        state = self.analyser.get_most_recent()
        if state is None:
            return
        self.topbar.timeLabel.text = state["time"]
        self.topbar.distLabel.text = f"{state['distance']} km"

        speed = state["speed"].metrics if state["speed"].metrics else None
        self.speedLabel.text = f"{int(speed.current) if speed is not None else '--'} km/h"
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

        volt = state["latestData"].get("0x42")
        self.voltMet.label.text = f"{volt['value']:.2f} V" if volt is not None else "-- V"

        temp = state["latestData"].get("0x05")
        self.tempMet.label.text = f"{int(temp['value'])} °C" if temp is not None else "-- °C"

    def update_bg_rect(self, *args):
        self.bg_rect.pos = self.pos
        self.bg_rect.size = self.size


class MyApp(App):
    def build(self):
        self.analyser = Analyser()
        # self.worker = Thread(target=read_from_com, args=(self.analyser,), daemon=True)
        self.worker = Thread(target=read_csv, args=("", self.analyser, 16), daemon=True)
        self.worker.start()
        return RealTimeScreen(self.analyser)

    def on_stop(self):
        print("App is stopping, saving historic metrics...")
        self.analyser.save_HistoricMetrics()
        print("Historic Metrics saved.")


if __name__ == "__main__":
    MyApp().run()