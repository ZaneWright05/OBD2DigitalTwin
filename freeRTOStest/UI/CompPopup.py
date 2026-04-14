from kivy.core.window import Window
from kivy.uix.modalview import ModalView
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.graphics import Color, Line, Rectangle

import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))
from model.helpers import ComparisonPoint
class ComparisonPopup(ModalView):
    def __init__(self, metrics, **kwargs):
        super().__init__(**kwargs)
        self.size_hint = (0.9, 0.9)
        self.auto_dismiss = False
        self.background = ''
        self.background_color = (0.7, 0.7, 0.7, 1)
        self.padding = (0, 0, 0, 0)
        
        popupX = Window.width * 0.05 
        popupY = Window.height * 0.05 
        popupWidth = Window.width * 0.9
        popupHeight = Window.height * 0.9

        layout = BoxLayout(orientation='vertical')

        title = Label(text="Trip vs Historical", color=(0, 0, 0, 1), font_size='20sp', size_hint=(1, None),height=54)
        layout.add_widget(title)

        header = GridLayout(cols=4, size_hint=(1, None), height=54)
        header.add_widget(Label(text="Metric", bold=True, color=(0, 0, 0, 1)))
        header.add_widget(Label(text="Trip", bold=True, color=(0, 0, 0, 1)))
        header.add_widget(Label(text="Hist", bold=True, color=(0, 0, 0, 1)))
        header.add_widget(Label(text="Difference", bold=True, color=(0, 0, 0, 1)))
        layout.add_widget(header)

        self.tablePos = 0
        self.metrics = metrics

        self.table = GridLayout(cols=4, size_hint=(1, None), height=108)
        for i in range(2):
            diff = ((self.metrics[i].average - self.metrics[i].histAvg) / self.metrics[i].histAvg) * 100 if self.metrics[i].histAvg != 0 else 0
            self.table.add_widget(Label(text=f"{self.metrics[i].pidName} ({self.metrics[i].pidUnit})", color=(0, 0, 0, 1)))
            self.table.add_widget(Label(text=f"{self.metrics[i].average:.2f}", color=(0, 0, 0, 1)))
            self.table.add_widget(Label(text=f"{self.metrics[i].histAvg:.2f}" if self.metrics[i].histAvg != 0 else "N/A", color=(0, 0, 0, 1)))
            self.table.add_widget(Label(text=f"{diff:.2f}%" if diff != 0 else "N/A", color=(0, 0, 0, 1)))
        layout.add_widget(self.table)

        self.row_backgrounds = []
        with self.table.canvas.before:
            for i in range(2):  # Number of rows in the table
                Color(142/255, 140/255, 140/255, 1 if i % 2 != 0 else 0.7)  # Alternate row colors
                rect = Rectangle(pos=(self.table.x, self.table.y), size=(self.table.width, self.table.height / 2))
                self.row_backgrounds.append(rect)
        self.table.bind(size=self.update_row_backgrounds, pos=self.update_row_backgrounds)

        # with self.table.canvas.before:
        #     Color(0.9, 0.9, 0.9, 1)  # Set the color (RGBA)
        #     self.table_background = Rectangle(rectangle=(popupX * 1.05, 0.995 * (popupY +( 5 * (popupHeight / 8))), popupWidth*0.995, (popupHeight/8 )* 0.9))

        self.insights = BoxLayout(orientation='vertical', size_hint=(1, None), height=162)
        self.insights.add_widget(Label(text="Further Insights:", bold=True, color=(0, 0, 0, 1)))
        self.insights.add_widget(Label(text="Trip Max: 0, Hist Max: 0", color=(0, 0, 0, 1)))
        self.insights.add_widget(Label(text="Trip Min: 0, Hist Min: 0", color=(0, 0, 0, 1)))
        self.insights.add_widget(Label(text="Trip ROC: 0, Hist ROC: 0", color=(0, 0, 0, 1)))
        self.update_insights(self.tablePos)
        layout.add_widget(self.insights)

        buttons = BoxLayout(size_hint=(1, None), height=54, spacing=0, padding=(0, 0, 0, 0))
        prev_button = Button(text="Prev", size_hint=(0.25, 1))
        prev_button.background_normal = ''
        prev_button.background_color = (1, 1, 1, 1)
        prev_button.color = (0, 0, 0, 1)
        prev_button.bind(on_press=lambda _: self.prev_metric())
        dismiss_button = Button(text="Dismiss", size_hint=(0.5, 1))
        dismiss_button.background_normal = ''
        dismiss_button.background_color = (1, 1, 1, 1)
        dismiss_button.color = (0, 0, 0, 1)
        next_button = Button(text="Next", size_hint=(0.25, 1))
        next_button.background_normal = ''
        next_button.background_color = (1, 1, 1, 1)
        next_button.color = (0, 0, 0, 1)
        next_button.bind(on_press=lambda _: self.next_metric())
        dismiss_button.bind(on_press=self.dismiss)
        buttons.add_widget(prev_button)
        buttons.add_widget(dismiss_button)
        buttons.add_widget(next_button)
        layout.add_widget(buttons)

        self.add_widget(layout)

        with self.canvas:
            Color(0, 0, 0, 1)
            # Draw lines relative to the ModalView's size
            Line(points=[popupX, popupY, popupX + popupWidth, popupY], width=1)
            Line(points=[popupX, popupY, popupX, popupY + popupHeight / 8], width=1)
            Line(points=[popupX + popupWidth, popupY, popupX + popupWidth, popupY + popupHeight / 8], width=1)
            Line(points=[popupX, popupY + popupHeight / 8, popupX + popupWidth, popupY + popupHeight / 8], width=1)
            Line(points=[popupX + popupWidth / 4, popupY, popupX + popupWidth / 4, popupY + popupHeight / 8], width=1)
            Line(points=[popupX, popupY + popupHeight / 2, popupX + popupWidth, popupY + popupHeight/2], width=1)
            Line(points=[popupX + 3 * (popupWidth / 4), popupY, popupX + 3 * (popupWidth / 4), popupY + popupHeight / 8], width=1)
            Line(points=[popupX, popupY + 7 * (popupHeight / 8), popupX + popupWidth, popupY + 7 * (popupHeight / 8)], width=1)
            Line(points=[popupX, popupY + 6 * (popupHeight / 8), popupX + popupWidth, popupY + 6 * (popupHeight / 8)], width=1)

    def update_row_backgrounds(self, *args):
        row_height = self.table.height / 2  
        for i, rect in enumerate(self.row_backgrounds):
            rect.pos = (self.table.x, self.table.y + i * row_height)
            rect.size = (self.table.width, row_height)

    def next_metric(self):
        self.tablePos += 1
        metric_1 = self.tablePos % len(self.metrics)
        metric_2 = (self.tablePos + 1) % len(self.metrics)
        self.update_table(metric_1, metric_2)
        self.update_insights(metric_1)

    def prev_metric(self):
        self.tablePos = (self.tablePos - 1) % len(self.metrics)
        metric_1 = self.tablePos % len(self.metrics)
        metric_2 = (self.tablePos + 1) % len(self.metrics)
        self.update_table(metric_1, metric_2)
        self.update_insights(metric_1)

    def update_insights(self, pos):
        self.insights.clear_widgets()
        self.insights.add_widget(Label(text=f"Further Insights ({self.metrics[pos].pidName}):", bold=True, color=(0, 0, 0, 1)))
        self.insights.add_widget(Label(text=f"Trip Max: {self.metrics[pos].max:.2f}, Hist Max: {self.metrics[pos].histMax:.2f}" if self.metrics[pos].histMax is not None else "N/A", color=(0, 0, 0, 1)))
        self.insights.add_widget(Label(text=f"Trip Min: {self.metrics[pos].min:.2f}, Hist Min: {self.metrics[pos].histMin:.2f}" if self.metrics[pos].histMin is not None else "N/A", color=(0, 0, 0, 1)))
        if self.metrics[pos].hasRoc:
            self.insights.add_widget(Label(text=f"Trip ROC: {self.metrics[pos].rocAvg:.2f}, Hist ROC: {self.metrics[pos].histRocAvg:.2f} " if self.metrics[pos].histRocAvg is not None else "N/A", color=(0, 0, 0, 1)))

    def update_table(self, one, two):
        self.table.clear_widgets()
        diff = ((self.metrics[one].average - self.metrics[one].histAvg) / self.metrics[one].histAvg) * 100 if self.metrics[one].histAvg != 0 else 0
        self.table.add_widget(Label(text=f"{self.metrics[one].pidName} ({self.metrics[one].pidUnit})", color=(0, 0, 0, 1)))
        self.table.add_widget(Label(text=f"{self.metrics[one].average:.2f}", color=(0, 0, 0, 1)))
        self.table.add_widget(Label(text=f"{self.metrics[one].histAvg:.2f}" if self.metrics[one].histAvg != 0 else "N/A", color=(0, 0, 0, 1)))
        self.table.add_widget(Label(text=f"{diff:.2f}%" if diff != 0 else "N/A", color=(0, 0, 0, 1)))

        diff = ((self.metrics[two].average - self.metrics[two].histAvg) / self.metrics[two].histAvg) * 100 if self.metrics[two].histAvg != 0 else 0
        self.table.add_widget(Label(text=f"{self.metrics[two].pidName} ({self.metrics[two].pidUnit})", color=(0, 0, 0, 1)))
        self.table.add_widget(Label(text=f"{self.metrics[two].average:.2f}", color=(0, 0, 0, 1)))
        self.table.add_widget(Label(text=f"{self.metrics[two].histAvg:.2f}" if self.metrics[two].histAvg != 0 else "N/A", color=(0, 0, 0, 1)))
        self.table.add_widget(Label(text=f"{diff:.2f}%" if diff != 0 else "N/A", color=(0, 0, 0, 1)))
