from kivy.config import Config
import os

if os.name != 'nt': 
    Window.show_cursor = False
    Config.set("graphics", "fullscreen", "auto")
    Config.set("graphics", "resizable", "0")
    Config.set("graphics", "borderless", "1")
else:
    Config.set('graphics', 'width', '800')
    Config.set('graphics', 'height', '480')
    Config.set('graphics', 'resizable', '0')
    Config.set('graphics', 'borderless', '1')

from kivy.core.window import Window
from kivy.app import App
from kivy.clock import Clock
from threading import Thread
from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.button import Button
from kivy.uix.gridlayout import GridLayout

from kivy.uix.widget import Widget
from kivy.graphics import Color, Rectangle, Line

from dataParser import Analyser, read_csv, read_from_com
from helpers import pid
from metricAnalyser import Metrics, MetricAnalyser, Event

from kivy.uix.image import Image
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

class TopBar(Widget):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.size_hint = (None, None)
        self.size = (800, 50)
        with self.canvas:
            Color(1, 1, 1, 1)  # White color
            self.rect = Rectangle(pos=self.pos, size=self.size)
            Color(0, 0, 0, 1)
            self.border = Line(rectangle=(self.x, self.y, self.width, self.height), width=1)
        self.bind(pos=self.update_rect, size=self.update_rect)

        self.layout = BoxLayout(orientation='horizontal', size_hint=(1, 1), pos=(0, 0))
        self.add_widget(self.layout)

    def update_rect(self, *args):
        self.rect.pos = self.pos
        self.rect.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)
        self.layout.size = self.size
        self.layout.pos = self.pos

def bind_widget_to_parent(widget, parent, pos_func):
    def update_pos(*args):
        widget.pos = pos_func(parent, widget)
    parent.bind(size=update_pos)
    update_pos()


class RealTimeScreen(FloatLayout):
    def __init__(self, analyser, **var_args):
        super().__init__(**var_args)

        with self.canvas.before:
            Color(179/255, 179/255, 179/255, 1)
            self.bg_rect = Rectangle(pos=self.pos, size=self.size)
            
        self.bind(pos=self._update_bg_rect, size=self._update_bg_rect)

        self.topbar = TopBar()
        self.add_widget(self.topbar)
        bind_widget_to_parent(self.topbar, self, lambda parent, widget: (0, parent.height - widget.height))
        
        self.connectionImg = Image(source=os.path.join(imgDir, "disconnected.png"), size_hint=(None, None), size=(50, 50))
        self.topbar.layout.add_widget(self.connectionImg)
        
        self.signalImg = Image(source=os.path.join(imgDir, "lowConnection.png"), size_hint=(None, None), size=(50, 50))
        self.topbar.layout.add_widget(self.signalImg)
        
        self.infoImg = Image(source=os.path.join(imgDir, "info.png"), size_hint=(None, None), size=(50, 50))
        self.topbar.layout.add_widget(self.infoImg)

        gearBox = BoxLayout(orientation='vertical', size_hint=(None, None), size=(185, 390), pos=(610, self.height - 440))
        self.estGearLabel = Label(text="Gear: -", size_hint=(1, 0.1),
                                   color=(0, 0, 0, 1), 
                                   font_size=18,
                                   halign='center',
                                valign='middle')
        gears = GridLayout(cols=2, size_hint=(1, 0.6), spacing=8, padding=8)
        for i in range(1, 7):
            btn = Button(text=f"{i}",
            size_hint=(None, None),  
            size=convert_dimensions(17.5, 17.5),
            color=(0, 0, 0, 1),
            background_normal='', background_color=(1, 1, 1, 1))

            btn.bind(on_press=lambda instance, x=i: analyser.store_gear(int(x)))
            gears.add_widget(btn)

        

        gearBox.add_widget(self.estGearLabel)
        gearBox.add_widget(gears)
        self.add_widget(gearBox)
        bind_widget_to_parent(gearBox, self, lambda parent, widget: (610, parent.height - 440))

        # gearBox = BoxLayout(orientation='vertical', size_hint=(0.5, 1))
        # self.estGearLabel = Label(text="Gear: -", size_hint=(1, 0.1))
        # self.estGRatioLabel = Label(text="Ratio: -", size_hint=(1, 0.1))
        # gears = GridLayout(cols=2, size_hint=(1, 0.6), spacing=8, padding=8)
        # for i in range(1, 7):
        #     btn = Button(text=f"{i}",
        #     size_hint=(None, None),   # disable auto scaling
        #     size=(convert_dimensions(17.5, 17.5)))  # set fixed size in mm
        #     btn.bind(on_press=lambda instance, x=i: analyser.store_gear(int(x)))
        #     gears.add_widget(btn)
        # gearBox.add_widget(self.estGearLabel)
        # gearBox.add_widget(self.estGRatioLabel)
        # gearBox.add_widget(gears)


        self.analyser = analyser 
        Clock.schedule_interval(self.refresh, 0.1) # refresh every 0.1s

    def _update_bg_rect(self, *args):
        self.bg_rect.pos = self.pos
        self.bg_rect.size = self.size

    def refresh(self, dt):
        state = self.analyser.get_most_recent()
        

class MyApp(App):
    def build(self):
        self.analyser = Analyser()
        self.worker = Thread(target=read_from_com, args=(self.analyser,), daemon=True)
        # self.worker = Thread(target=read_csv, args=("", self.analyser, 16), daemon=True)
        self.worker.start()
        return RealTimeScreen(self.analyser)

    def on_stop(self):
        print("App is stopping, saving historic metrics...")
        self.analyser.save_HistoricMetrics()
        print("Historic Metrics saved.")

if __name__ == "__main__":
    MyApp().run()