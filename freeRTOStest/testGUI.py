from threading import Thread

from kivy.app import App
from kivy.clock import Clock
from kivy.core.window import Config
from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.gridlayout import GridLayout

from dataParser import Analyser, read_csv, read_from_com

width = 800 # 154.08 mm
height = 480 # 85.92 mm

Config.set('graphics', 'fullscreen', '1')
Config.set('graphics', 'width', str(width))
Config.set('graphics', 'height', str(height))
Config.set('graphics', 'show_taskbar', '0')

class TestScreen(BoxLayout):
    def __init__(self, analyser, **var_args):
        super(TestScreen, self).__init__(**var_args)
        self.orientation = 'horizontal'
        self.analyser = analyser 

        labels = GridLayout(cols=1, size_hint=(0.8, 1))
        self.speed_label = Label(text="Speed: --")
        self.rpm_label = Label(text="RPM --")
        self.accel_label = Label(text="Accel: --")
        self.throttle_label = Label(text="Throttle: --")
        self.fuelCons_label = Label(text="Fuel Cons: --")

        labels.add_widget(self.speed_label)
        labels.add_widget(self.rpm_label)
        labels.add_widget(self.accel_label)
        labels.add_widget(self.throttle_label)
        labels.add_widget(self.fuelCons_label)

        gearBox = BoxLayout(orientation='vertical', size_hint=(0.2, 1))
        self.estGearLabel = Label(text="Gear: -")
        self.estGRatioLabel = Label(text="Ratio: -")
        gears = GridLayout(cols=2, size_hint=(1, 1), spacing=8, padding=8)
        for i in range(1, 7):
            btn = Button(text=f"{i}")
            btn.bind(on_press=lambda instance, x=i: analyser.store_gear(int(x)))
            gears.add_widget(btn)
        gearBox.add_widget(self.estGearLabel)
        gearBox.add_widget(self.estGRatioLabel)
        gearBox.add_widget(gears)

        self.add_widget(labels)
        self.add_widget(gearBox)

        Clock.schedule_interval(self.refresh, 0.1) # call refresh every 0.1s

    def refresh(self, dt):
        state = self.analyser.get_most_recent()

        speed = state["latestData"].get("0x0D")
        rpm = state["latestData"].get("0x0C")
        accel = state["accel"]
        fuelCons = state["fuelCons"]
        throttle = state["latestData"].get("0x11")
        gear = state["gear"]

        if speed:
            self.speed_label.text = f"Speed: {speed['value']} {speed['unit']}"
        if rpm:
            self.rpm_label.text = f"RPM: {rpm['value']} {rpm['unit']}"
        if accel:
            self.accel_label.text = f"Accel: {accel:.2f} m/s^2"
        if fuelCons is not None:
            self.fuelCons_label.text = f"Fuel Cons: {fuelCons:.2f} l/km"
        if throttle:
            self.throttle_label.text = f"Throttle: {throttle['value']:.2f} {throttle['unit']}"
        if gear is not None:
            self.estGearLabel.text = f"Gear: {gear}"
            if gear == 0:
                self.estGearLabel.text = f"Gear: --"
        if state["ratio"] is not None:
            self.estGRatioLabel.text = f"Ratio: {state['ratio']:.2f}"

class MyApp(App):
    def build(self):
        self.analyser = Analyser()

        self.worker = Thread(target=read_from_com, 
                             args=(self.analyser,), 
                             daemon = True)

        
        # self.worker = Thread(target=read_csv, 
        #                      args=("",self.analyser,128), 
        #                      daemon = True)

        self.worker.start()

        return TestScreen(self.analyser)

if __name__ == "__main__":
    MyApp().run()