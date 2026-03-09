from threading import Thread

from kivy.app import App
from kivy.clock import Clock
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout

from dataParser import Analyser, read_csv 

class TestScreen(GridLayout):
    def __init__(self, analyser, **var_args):
        super(TestScreen, self).__init__(**var_args)
        self.cols = 1
        self.analyser = analyser 

        self.speed_label = Label(text="Speed: --")
        self.rpm_label = Label(text="RPM --")
        self.accel_label = Label(text="Accel: --")

        self.add_widget(self.speed_label)
        self.add_widget(self.rpm_label)
        self.add_widget(self.accel_label)

        Clock.schedule_interval(self.refresh, 0.1) # call refresh every 0.1s

    def refresh(self, dt):
        state = self.analyser.get_most_recent()

        speed = state["latestData"].get("0x0D")
        rpm = state["latestData"].get("0x0C")
        accel = state["accel"]

        if speed:
            self.speed_label.text = f"Speed: {speed['value']} {speed['unit']}"
        if rpm:
            self.rpm_label.text = f"RPM: {rpm['value']} {rpm['unit']}"
        if accel:
            self.accel_label.text = f"Accel: {accel:.2f} m/s^2"


class MyApp(App):
    def build(self):
        self.analyser = Analyser()

        self.worker = Thread(target=read_csv, 
                             args=('data_log2.csv', self.analyser, 128), 
                             daemon = True)

        self.worker.start()

        return TestScreen(self.analyser)

if __name__ == "__main__":
    MyApp().run()