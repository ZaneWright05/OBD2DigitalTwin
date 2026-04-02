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
from kivy.graphics import PushMatrix, PopMatrix, Rotate
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.image import Image
from kivy.clock import Clock
from threading import Thread
from kivy.uix.label import Label
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.button import Button
from kivy.uix.gridlayout import GridLayout

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

class TopBar(BoxLayout):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.orientation = 'horizontal'
        self.size_hint = (1, None)
        self.height = 50
        self.pos_hint = {'top': 1}
        self.padding = (10, 0, 0, 0)
        self.spacing = 8

        with self.canvas.before:
            Color(1, 1, 1, 1)
            self.rect = Rectangle(pos=self.pos, size=self.size)

        with self.canvas.after:
            Color(0, 0, 0, 1)
            self.border = Line(rectangle=(self.x, self.y, self.width, self.height), width=1)

        self.bind(pos=self.update_rect, size=self.update_rect)

        self.leftImgs = BoxLayout(
            orientation='horizontal',
            size_hint=(None, 1),
            spacing=8
        )
        self.leftImgs.bind(minimum_width=self.leftImgs.setter('width'))

        self.connectionImg = Image(
            source=os.path.join(imgDir, "disconnected.png"),
            size_hint=(None, None),
            size=(50, 50)
        )
        self.signalImg = Image(
            source=os.path.join(imgDir, "lowConnection.png"),
            size_hint=(None, None),
            size=(50, 50)
        )
        self.infoImg = Image(
            source=os.path.join(imgDir, "info.png"),
            size_hint=(None, None),
            size=(50, 50)
        )

        self.leftImgs.add_widget(self.connectionImg)
        self.leftImgs.add_widget(self.signalImg)
        self.leftImgs.add_widget(self.infoImg)

        self.rightContent = BoxLayout(
            orientation='horizontal',
            size_hint=(None, 1),
            spacing=4
        )
        self.rightContent.bind(minimum_width=self.rightContent.setter('width'))

        self.roadImg = Image(
            source=os.path.join(imgDir, "road.png"),
            size_hint=(None, 1),
            size=(50, 50),
            fit_mode='contain'
        )
        self.timerImg = Image(
            source=os.path.join(imgDir, "timer.png"),
            size_hint=(None, 1),
            size=(35, 35),
            fit_mode='contain'
        )

        self.distLabel = Label(
            text="00.00 km",
            size_hint=(None, 1),
            width=80,
            color=(0, 0, 0, 1),
            font_size=18,
            halign='left',
            valign='middle'
        )
        self.distLabel.bind(size=lambda w, _: setattr(w, 'text_size', w.size))

        self.timeLabel = Label(
            text="00:00:00",
            size_hint=(None, 1),
            width=80,
            color=(0, 0, 0, 1),
            font_size=18,
            halign='left',
            valign='middle'
        )
        self.timeLabel.bind(size=lambda w, _: setattr(w, 'text_size', w.size))

        self.rightContent.add_widget(self.roadImg)
        self.rightContent.add_widget(self.distLabel)
        self.rightContent.add_widget(self.timerImg)
        self.rightContent.add_widget(self.timeLabel)

        self.eventLabel = Label(
            text="[45:20] Temperature coolant spike detected",
            size_hint=(None, None),          # important: not (None, 1)
            pos_hint={"center_y": 0.5},
            color=(1, 1, 1, 1),
            font_size=16,
            halign="left",
            valign="middle",
            shorten=True,
            shorten_from="right",
            max_lines=1
        )

        # self.eventLabel.shorten = True
        # self.eventLabel.shorten_from = "right"
        # self.eventLabel.max_lines = 1

        with self.eventLabel.canvas.before:
            Color(142/255, 140/255, 140/255, 1)
            self.eventBg = RoundedRectangle(
                pos=self.eventLabel.pos,
                size=self.eventLabel.size,
                radius=[8, 8, 8, 8]
            )

        self.eventLabel.bind(text=self.update_event_pill, pos=self.update_event_bg, size=self.update_event_bg)
        self.bind(height=self.update_event_pill)

        self.update_event_pill()
        self.update_event_bg()



        self.add_widget(self.leftImgs)
        # self.add_widget(Widget(size_hint=(1, 1)))
        self.add_widget(self.eventLabel)
        self.add_widget(Widget(size_hint=(1, 1)))
        self.add_widget(self.rightContent)

    def update_rect(self, *args):
        self.rect.pos = self.pos
        self.rect.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)

    def update_event_pill(self, *args):
        fixed_w = 340
        h = self.height - 6
        if self.eventLabel.size != (fixed_w, h):
            self.eventLabel.size = (fixed_w, h)
        self.eventLabel.text_size = (fixed_w - 20, h)

    def update_event_bg(self, *args):
        self.eventBg.pos = self.eventLabel.pos
        self.eventBg.size = self.eventLabel.size

class Tachometer(Widget):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.size_hint = (None, None)
        self.size = (250, 250)

        self.min = 0
        self.max = 7000

        self.displayedValue = 0
        self.targetValue = 0

        self.needleMin = 0
        self.needleMax = 90

        self.dial = Image(
            source=os.path.join(imgDir, "tachometer.png"),
            size_hint=(None, None),
            size=(250, 250),
            pos=self.pos
        )
        
        minLabel = Label(text=str(self.min), pos=(self.x,self.y -22.5), size_hint=(None, None), color=(0, 0, 0, 1))
        maxLabel = Label(text=str(self.max), pos=(self.dial.width - 10, self.dial.height - 40), size_hint=(None, None), color=(0, 0, 0, 1))

        # self.rpmBox = Widget(size_hint=(None, None), size=(90, 36))
        # with self.rpmBox.canvas.before:
        #     Color(0.7, 0.7, 0.7, 1)
        #     self.rpmRect = RoundedRectangle(pos=self.rpmBox.pos, size=self.rpmBox.size, radius=[6, 6, 6, 6])

        self.rpmLabel = Label(
            text="0 RPM",
            size_hint=(None, None),
            font_size=28,
            color=(0, 0, 0, 1),
            halign="center",
            valign="middle"
        )
        self.rpmLabel.bind(size=lambda inst, _: setattr(inst, "text_size", inst.size))

        self.add_widget(self.dial)

        self.add_widget(minLabel)
        self.add_widget(maxLabel)
        self.needle = Image(
            source=os.path.join(imgDir, "needleLong2Col.png"),
            size_hint=(None, None),
            size=(254,6),
            pos=self.pos
        )

        # self.needle.pos = (
        #     self.dial.right - self.needle.width,
        #     self.dial.y
        # )

        with self.needle.canvas.before:
            PushMatrix()
            self.needle_rot = Rotate(angle=0, origin=(0, 0))
        with self.needle.canvas.after:
            PopMatrix()

        self.add_widget(self.needle)

        # self.add_widget(self.rpmBox)
        self.add_widget(self.rpmLabel)

        self.bind(pos=self._sync_visuals, size=self._sync_visuals)
        self._sync_visuals()

        Clock.schedule_interval(self._tick, 1.0/40.0)

    def _sync_visuals(self, *args):
        self.dial.pos = self.pos
        self.dial.size = (250, 250)

        dial_right = self.dial.x + self.dial.width
        dial_bottom = self.dial.y
        self.needle.pos = (
            dial_right - self.needle.width,
            dial_bottom
        )
        self.needle_rot.origin = (dial_right, dial_bottom)

        dial_center_x = self.dial.x + self.dial.width / 2
        dial_center_y = self.dial.y + self.dial.height / 2

        # self.rpmBox.pos = (
        #     dial_center_x - self.rpmBox.width / 2,
        #     dial_center_y - self.rpmBox.height / 2
        # )
        # self.rpmRect.pos = self.rpmBox.pos
        # self.rpmRect.size = self.rpmBox.size

        self.rpmLabel.pos = (dial_center_x, dial_center_y - self.rpmLabel.height / 2)


       

    def set_target_rpm(self, rpm):
        try:
            rpm = float(rpm)
        except Exception:
            rpm = 0.0
        self.targetValue = max(self.min, min(self.max, rpm))

    def _tick(self, dt):
        alpha = 0.35
        self.displayedValue += (self.targetValue - self.displayedValue) * alpha
        self._apply_angle(self.displayedValue)

    def _apply_angle(self, rpm):
        n = (rpm - self.min) / (self.max - self.min)
        n = max(0.0, min(1.0, n))
        self.needle_rot.angle = -(self.needleMin + n * (self.needleMax - self.needleMin))

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

        self.tach = Tachometer()
        self.tach.pos = (20, 20)
        self.content.add_widget(self.tach)

        self.speedLabel = Label(text="-- km/h", pos=(300, 300), size_hint=(None, None), color=(0, 0, 0, 1), font_size=18)
        self.content.add_widget(self.speedLabel)

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

        speed = state["speed"].metrics.current if state["speed"].metrics else None
        self.speedLabel.text = f"{int(speed) if speed is not None else '--'} km/h"

        rpm = state["rpm"].metrics.current if state["rpm"].metrics else None
        self.tach.set_target_rpm(rpm)
        self.tach.rpmLabel.text = f"{int(rpm) if rpm is not None else '0'} RPM"


    def update_bg_rect(self, *args):
        self.bg_rect.pos = self.pos
        self.bg_rect.size = self.size


class MyApp(App):
    def build(self):
        self.analyser = Analyser()
        # self.worker = Thread(target=read_from_com, args=(self.analyser,), daemon=True)
        self.worker = Thread(target=read_csv, args=("", self.analyser, 128), daemon=True)
        self.worker.start()
        return RealTimeScreen(self.analyser)

    def on_stop(self):
        print("App is stopping, saving historic metrics...")
        self.analyser.save_HistoricMetrics()
        print("Historic Metrics saved.")


if __name__ == "__main__":
    MyApp().run()