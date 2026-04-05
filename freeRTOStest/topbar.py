import os
from kivy.uix.widget import Widget
from kivy.graphics import Color, Rectangle, Line, RoundedRectangle
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.image import Image
from kivy.uix.label import Label

imgDir = os.path.join(os.path.dirname(__file__), "assets")

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

        self.eventLabelHidden = False

        self.eventLabel = Label(
            text="[45:20] Temperature coolant spike detected",
            size_hint=(None, None),   
            pos_hint={"center_y": 0.5},
            color=(1, 1, 1, 1),
            font_size=14,
            halign="left",
            valign="middle",
            shorten=True,
            shorten_from="right",
            max_lines=1
        )

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