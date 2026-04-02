import os
from kivy.uix.widget import Widget
from kivy.graphics import PushMatrix, PopMatrix, Rotate
from kivy.uix.image import Image
from kivy.clock import Clock
from kivy.uix.label import Label

imgDir = os.path.join(os.path.dirname(__file__), "assets")

class Tachometer(Widget):
    def __init__(self, newSize=200, **kwargs):
        super().__init__(**kwargs)
        self.size_hint = (None, None)

        self.orignalSize = 250
        self.newSize = newSize
        self.size = (self.newSize, self.newSize)

        self.min = 0
        self.max = 7000

        self.displayedValue = 0
        self.targetValue = 0

        self.needleMin = 0
        self.needleMax = 90

        self.dial = Image(
            source=os.path.join(imgDir, "tachometer.png"),
            size_hint=(None, None),
            size=(self.newSize, self.newSize),
            pos=self.pos
        )
        
        self.minLabel = Label(text=str(self.min), size_hint=(None, None), color=(0, 0, 0, 1))
        self.maxLabel = Label(text=str(self.max), size_hint=(None, None), color=(0, 0, 0, 1))

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

        self.add_widget(self.minLabel)
        self.add_widget(self.maxLabel)
        self.needle = Image(
            source=os.path.join(imgDir, "needleLong2Col.png"),
            size_hint=(None, None),
            size=(254 *(self.newSize/self.orignalSize),6 *(self.newSize/self.orignalSize)),
            pos=self.pos
        )


        with self.needle.canvas.before:
            PushMatrix()
            self.needle_rot = Rotate(angle=0, origin=(0, 0))
        with self.needle.canvas.after:
            PopMatrix()

        self.add_widget(self.needle)

        self.add_widget(self.rpmLabel)

        self.bind(pos=self._sync_visuals, size=self._sync_visuals)
        self._sync_visuals()

        Clock.schedule_interval(self._tick, 1.0/40.0)

    def _sync_visuals(self, *args):
        self.dial.pos = self.pos
        self.dial.size = (self.newSize, self.newSize)

        dial_right = self.dial.x + self.dial.width
        dial_bottom = self.dial.y
        self.needle.pos = (
            dial_right - self.needle.width,
            dial_bottom
        )
        self.needle_rot.origin = (dial_right, dial_bottom)

        dial_center_x = self.dial.x + self.dial.width / 2
        dial_center_y = self.dial.y + self.dial.height / 2

        self.rpmLabel.pos = (dial_center_x, dial_center_y - self.rpmLabel.height / 2)

        self.minLabel.texture_update()
        self.maxLabel.texture_update()
        self.minLabel.size = self.minLabel.texture_size
        self.maxLabel.size = self.maxLabel.texture_size
        self.minLabel.pos = (self.x + (self.dial.width * 0.1), self.y - self.minLabel.height / 2)
        self.maxLabel.pos = (self.x + (self.dial.width * 1.025), self.y + self.dial.height - self.maxLabel.height)

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