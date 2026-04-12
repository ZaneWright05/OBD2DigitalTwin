from time import time
import os
from kivy.uix.widget import Widget
from kivy.graphics import Color, Rectangle, Line, RoundedRectangle
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.image import Image
from kivy.uix.label import Label
from kivy.uix.button import Button

# from model.dataParser import Parser
# from model.vehicleState import VehicleState

imgDir = os.path.join(os.path.dirname(__file__), "assets")

class TopBar(BoxLayout):
    def __init__(self, analyser, vehicleState, **kwargs):
        super().__init__(**kwargs)

        self.orientation = 'horizontal'
        self.size_hint = (1, None)
        self.height = 50
        self.pos_hint = {'top': 1}
        self.padding = (10, 0, 0, 0)
        self.spacing = 8
        self.analyser = analyser
        self.vehicleState = vehicleState


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

        self.connectionState = False
        self.connectionImg = Image(
            source=os.path.join(imgDir, "disconnected.png"),
            size_hint=(None, None),
            size=(50, 50)
        )
        self.currentSignalLevel = "low"
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

        self.tripButton = Button(
            text="Start Trip",
            size_hint=(None, 1),
            width=90,
        )
        self._last_trip_btn_press_s = 0.0
        self.tripButton.bind(on_press=lambda _: self.stop_start_button_handler())

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


        self.rightContent.add_widget(self.tripButton)
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

    def set_signal(self, level):
        if level == "low" and self.currentSignalLevel != "low":
            self.signalImg.source = os.path.join(imgDir, "lowConnection.png")
        elif level == "medium" and self.currentSignalLevel != "medium":
            self.signalImg.source = os.path.join(imgDir, "medConnection.png")
        elif level == "high" and self.currentSignalLevel != "high":
            self.signalImg.source = os.path.join(imgDir, "highConnection.png")
        self.currentSignalLevel = level

    def set_connection(self, status: bool):
        self.connectionState = status
        if status:
            self.connectionImg.source = os.path.join(imgDir, "connected.png")
        else:
            self.connectionImg.source = os.path.join(imgDir, "disconnected.png")

    def update_rect(self, *args):
        self.rect.pos = self.pos
        self.rect.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)

    def update_event_pill(self, *args):
        fixed_w = 250
        h = self.height - 6
        if self.eventLabel.size != (fixed_w, h):
            self.eventLabel.size = (fixed_w, h)
        self.eventLabel.text_size = (fixed_w - 20, h)

    def update_event_bg(self, *args):
        self.eventBg.pos = self.eventLabel.pos
        self.eventBg.size = self.eventLabel.size

    def stop_start_button_handler(self):
        now = time()
        if (now - self._last_trip_btn_press_s) < 0.35:
            return
        self._last_trip_btn_press_s = now
        
        if self.analyser.connected:
            if self.analyser.running:
                if(self.analyser.stop_trip()):    
                    self.tripButton.text = "Start Trip"
                    self.set_event_text("Trip stopped, saving data...") # TODO: list as event so it dissapears
            else:
                if(self.analyser.start_trip()):
                    self.vehicleState.reset_state()
                    self.tripButton.text = "Stop Trip"
                    self.set_event_text("Trip started, collecting data...")
    
    def update_stop_start_btn(self):
        if self.analyser.running and self.tripButton.text != "Stop Trip":
            self.tripButton.text = "Stop Trip"
        elif not self.analyser.running and self.tripButton.text != "Start Trip":
            self.tripButton.text = "Start Trip"

    def set_event_text(self, msg):
        self.eventLabel.text = msg

    def update_topBar(self, state):
        self.update_stop_start_btn()

        self.set_connection(self.analyser.connected)
        if self.analyser.connected and self.analyser.running:
            freshness = state["freshness"]
            if freshness is not None:
                if freshness > 0.95:
                    self.set_signal("high")
                elif freshness > 0.33:
                    self.set_signal("medium")
                else:
                    self.set_signal("low")

            # TODO: 2 minute last shown timer
            event_msg = state.get("event_message", "")
            if event_msg:
                self.set_event_text(event_msg)

            self.timeLabel.text = state.get("time", "00:00:00")
            self.distLabel.text = f"{state.get('distance', '00.00')} km"
        else:
            self.set_signal("low")
            self.timeLabel.text = "00:00:00"
            self.distLabel.text = "00.00 km"