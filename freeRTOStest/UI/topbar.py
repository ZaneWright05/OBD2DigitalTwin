from time import time
import os
from kivy.uix.widget import Widget
from kivy.graphics import Color, Rectangle, Line, RoundedRectangle
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.image import Image
from kivy.uix.label import Label
from kivy.uix.button import Button
from CompPopup import ComparisonPopup
from model.helpers import ComparisonPoint
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

        self.tripBtnDisabled = False
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

        self.eventWidget = EventWidget()
        self.add_widget(self.leftImgs)
        self.add_widget(self.eventWidget)
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


    def stop_start_button_handler(self):
        now = time()
        if (now - self._last_trip_btn_press_s) < 0.35:
            return
        self._last_trip_btn_press_s = now
        
        if self.analyser.connected:
            if self.analyser.running:
                # build comparison points for popup before stopping trip and clearing data
                comparsionPoints = []
                metrics = self.analyser.metrics.values()   
                if metrics is not None:
                    for metric in metrics:
                        if metric.pid.name == "Speed":
                            comparsionPoints.append(ComparisonPoint(
                                pidName=metric.pid.name,
                                pidUnit=metric.pid.unit,
                                average=metric.metrics.average * 3.6, # convert to km/h
                            histAvg=0 if metric.historicMetrics is None else metric.historicMetrics.average * 3.6, 
                            min=metric.metrics.min * 3.6, 
                            histMin=metric.historicMetrics.min * 3.6 if metric.historicMetrics is not None else 0,
                            max=metric.metrics.max * 3.6, 
                            histMax=metric.historicMetrics.max * 3.6 if metric.historicMetrics is not None else 0,
                            hasRoc=False,
                            rocAvg=0,
                            histRocAvg=0
                            )
                        )
                    else:
                        comparsionPoints.append(ComparisonPoint(
                            pidName=metric.pid.name,
                            pidUnit=metric.pid.unit,
                            average=metric.metrics.average,
                            histAvg=0 if metric.historicMetrics is None else metric.historicMetrics.average,
                            min=metric.metrics.min,
                            histMin=metric.historicMetrics.min if metric.historicMetrics is not None else 0,
                            max=metric.metrics.max,
                            histMax=metric.historicMetrics.max if metric.historicMetrics is not None else 0,
                            hasRoc=True,
                            rocAvg=metric.single_trip_roc_average(),
                            histRocAvg=metric.historicMetrics.wAvgROC if metric.historicMetrics is not None else 0
                            )
                        )
                        speedMetric = self.analyser.speedMetric
                if speedMetric is not None:
                    comparsionPoints.append(ComparisonPoint(
                        pidName="Accel",
                            pidUnit="m/s²",
                            average=speedMetric.single_trip_roc_average(),
                            histAvg=speedMetric.historicMetrics.wAvgROC if speedMetric.historicMetrics is not None else 0,
                            min=speedMetric.metrics.minWAvgROC,
                            histMin=speedMetric.historicMetrics.minWAvgROC if speedMetric.historicMetrics is not None else 0,
                            max=speedMetric.metrics.maxWAvgROC,
                            histMax=speedMetric.historicMetrics.maxWAvgROC if speedMetric.historicMetrics is not None else 0,
                            hasRoc=False,
                            rocAvg=None,
                            histRocAvg=None
                            )
                        )

                if(self.analyser.stop_trip()):    
                    self.tripButton.text = "Start Trip"
                    self.eventWidget.set_event_text("Trip stopped, data saved") # TODO: list as event so it dissapears
                    self.show_comparison_popup(comparisonPoints=comparsionPoints)
            else:   
                if(self.analyser.start_trip()):
                    self.vehicleState.reset_state()
                    self.tripButton.text = "Stop Trip"
                    self.eventWidget.set_event_text("Trip started, collecting data...")
    
    def update_stop_start_btn(self):
        if self.analyser.running and self.tripButton.text != "Stop Trip":
            self.tripButton.text = "Stop Trip"
        elif not self.analyser.running and self.tripButton.text != "Start Trip":
            self.tripButton.text = "Start Trip"

    def show_comparison_popup(self, comparisonPoints: list[ComparisonPoint]):
        print("Showing popup")
        # Create and open the popup
        popup = ComparisonPopup(
            # title="Comparison Result",
                    #   content=content,
                    #   size_hint=(0.8, 0.4),
                      metrics=comparisonPoints,
                      auto_dismiss=False,
                      filePath=self.analyser.filePath)
        popup.open()

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

            speed = state["speed"]
            if speed is not None and speed.metrics.current > 0:
                self.tripButton.set_disabled(True)
                self.tripBtnDisabled = True
            elif self.tripBtnDisabled:
                self.tripButton.set_disabled(False)
                self.tripBtnDisabled = False

            event = state["event"]
            if event is None:
                if not self.eventLabelHidden:
                    self.eventLabelHidden = True
                    self.eventWidget.opacity = 0
                    self.eventWidget.disabled = True
                    self.eventWidget.set_event_text("")
            else:
                self.eventLabelHidden = False
                self.eventWidget.opacity = 1
                self.eventWidget.disabled = False
                if event.priority == 0:
                    self.eventWidget.set_bg_colour((142 / 255, 140 / 255, 140 / 255, 1)) # grey
                elif event.priority == 1: 
                    self.eventWidget.set_bg_colour((255/255,115/255,0/255, 1)) # orange
                else:
                    self.eventWidget.set_bg_colour((255/255,0/255,0/255, 1)) # red
                startTime_ms = event.timestamp * event.pid.period_ms
                startTime = f"{int(startTime_ms // 60000):02d}:{int((startTime_ms % 60000) // 1000):02d}"
                self.eventWidget.set_event_text(f"[{startTime}] {event.pid.name} - {event.type}")

            self.timeLabel.text = state.get("time", "00:00:00")
            self.distLabel.text = f"{state.get('distance', '00.00')} km"
        else:
            if self.tripBtnDisabled:
                self.tripButton.set_disabled(False)
                self.tripBtnDisabled = False
            self.set_signal("low")
            self.timeLabel.text = "00:00:00"
            self.distLabel.text = "00.00 km"

class EventWidget(BoxLayout):
    def __init__(self, width=250, bgCol=(142 / 255, 140 / 255, 140 / 255, 1), **kwargs):
        super().__init__(**kwargs)
        self.size_hint = (None, 0.8)
        self.width = width
        self.pos_hint = {"center_y": 0.5}
        self.bgCol = bgCol
        self.orientation = "horizontal"
        self.padding = (10, 0)
        self.spacing = 5

        with self.canvas.before:
            Color(*self.bgCol)
            self.background = RoundedRectangle(
                pos=self.pos,
                size=self.size,
                radius=[8, 8, 8, 8]
            )

        self.label = Label(
            text="",
            size_hint=(1, 1),
            color=(1, 1, 1, 1),
            font_size=14,
            halign="left",
            valign="middle",
            shorten=True,
            shorten_from="right",
            max_lines=1
        )
        self.label.bind(size=self.update_label_text_size)
        self.add_widget(self.label)

        self.bind(pos=self.update_background, size=self.update_background)

    def update(self, event):
        if event.priority == 0:
                    self.set_bg_colour((142 / 255, 140 / 255, 140 / 255, 1)) # grey
        elif event.priority == 1: 
            self.set_bg_colour((255/255,115/255,0/255, 1)) # orange
        else:
            self.set_bg_colour((255/255,0/255,0/255, 1)) # red
                # print(f"Event detected in UI: {event}")
        endStr = f"duration: {event.length * event.pid.period_ms / 1000:.1f} s" if event.ended else "detected"
        startTime_ms = event.timestamp * event.pid.period_ms
        startTime = f"{int(startTime_ms // 60000):02d}:{int((startTime_ms % 60000) // 1000):02d}"
        self.set_event_text(f"[{startTime}] {event.pid.name} - {event.type} {endStr}")

    def hide(self):
        self.opacity = 0
        self.disabled = True

    def show(self):
        self.opacity = 1
        self.disabled = False

    def update_background(self, *args):
        self.background.pos = self.pos
        self.background.size = self.size

    def update_label_text_size(self, *args):
        self.label.text_size = (self.label.width - 20, self.label.height)

    def set_event_text(self, text):
        self.label.text = text
        self.update_label_text_size()

    def set_bg_colour(self, bgCol):
        self.bgCol = bgCol
        self.canvas.before.clear()
        with self.canvas.before:
            Color(*self.bgCol)
            self.background = RoundedRectangle(
                pos=self.pos,
                size=self.size,
                radius=[8, 8, 8, 8]
            )