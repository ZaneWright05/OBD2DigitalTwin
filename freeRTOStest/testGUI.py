from kivy.config import Config
import os


from kivy.core.window import Window
if os.name != 'nt':  # Windows
    Window.show_cursor = False
    Config.set("graphics", "fullscreen", "auto")
    Config.set("graphics", "resizable", "0")
    Config.set("graphics", "borderless", "1")
else:
    Config.set('graphics', 'width', '800')
    Config.set('graphics', 'height', '480')
    Config.set('graphics', 'resizable', '0')
    Config.set('graphics', 'borderless', '1')

from kivy.app import App
from kivy.clock import Clock
from threading import Thread
from kivy.uix.label import Label
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.gridlayout import GridLayout

from dataParser import Analyser, read_csv, read_from_com
from helpers import pid
from metricAnalyser import Metrics, MetricAnalyser, Event

screen_width_px = 800 # 154.08 mm
screen_width_mm = 154.08
screen_height_px = 480 # 85.92 mm
screen_height_mm = 85.92
# Config.set('graphics', 'fullscreen', '1')
# Config.set('graphics', 'width', str(width))
# Config.set('graphics', 'height', str(height))
# Config.set('graphics', 'show_taskbar', '0')

def convert_dimensions(width_mm, height_mm):
    if os.name == 'nt':  # Windows
            # Windows typically has around 96 DPI, which is about 3.78 pixels per mm
        px_per_mm_x = 3.78
        px_per_mm_y = 3.78
    else:
        px_per_mm_x = screen_width_px / screen_width_mm
        px_per_mm_y = screen_height_px / screen_height_mm
    width_px = int(width_mm * px_per_mm_x)
    height_px = int(height_mm * px_per_mm_y)
    # print(f"Converting {width_mm}mm x {height_mm}mm to {width_px}px x {height_px}px")
    return width_px, height_px

class TestScreen(BoxLayout):
    def __init__(self, analyser, **var_args):
        super(TestScreen, self).__init__(**var_args)
        self.orientation = 'horizontal'
        self.analyser = analyser 

        labels = GridLayout(cols=1, size_hint=(0.5, 1))

        speed_box = BoxLayout(orientation='vertical', size_hint=(0.5, 0.4))
        self.speed_label = Label(text="Speed: --")
        self.speed_sub_label = Label(text="", font_size=10)
        speed_box.add_widget(self.speed_label)
        speed_box.add_widget(self.speed_sub_label)

        rpm_box = BoxLayout(orientation='vertical', size_hint=(0.5, 0.4))
        self.rpm_label = Label(text="RPM --")
        self.rpm_sub_label = Label(text="", font_size=10)
        rpm_box.add_widget(self.rpm_label)
        rpm_box.add_widget(self.rpm_sub_label)
        self.accel_label = Label(text="Accel: --")
        self.throttle_label = Label(text="Throttle: --")
        self.fuelCons_label = Label(text="Fuel Cons: --")

        # labels.add_widget(self.speed_label)
        labels.add_widget(speed_box)
        labels.add_widget(rpm_box)
        # labels.add_widget(self.rpm_label)
        # labels.add_widget(self.rpm_sub_label)
        
        labels.add_widget(self.accel_label)
        labels.add_widget(self.throttle_label)
        labels.add_widget(self.fuelCons_label)

        gearBox = BoxLayout(orientation='vertical', size_hint=(0.5, 1))
        self.estGearLabel = Label(text="Gear: -", size_hint=(1, 0.1))
        self.estGRatioLabel = Label(text="Ratio: -", size_hint=(1, 0.1))
        gears = GridLayout(cols=2, size_hint=(1, 0.6), spacing=8, padding=8)
        for i in range(1, 7):
            btn = Button(text=f"{i}",
            size_hint=(None, None),   # disable auto scaling
            size=(convert_dimensions(17.5, 17.5)))  # set fixed size in mm
            btn.bind(on_press=lambda instance, x=i: analyser.store_gear(int(x)))
            gears.add_widget(btn)
        gearBox.add_widget(self.estGearLabel)
        gearBox.add_widget(self.estGRatioLabel)
        gearBox.add_widget(gears)

        eventBox = BoxLayout(orientation='vertical', size_hint=(0.5, 1))
        self.eventLabel = Label(text="Current Event: --", size_hint=(1, 0.1))
       # self.eventVisibleTimer = 2 # seconds to show event after it ends
        gearBox.add_widget(self.eventLabel)

        self.add_widget(labels)
        self.add_widget(gearBox)

        Clock.schedule_interval(self.refresh, 0.1) # call refresh every 0.1s

    def refresh(self, dt):
        state = self.analyser.get_most_recent()
        # speed = state["latestData"].get("0x0D")
        speed = state["speed"]
        rpm = state["rpm"]
        fuelCons = state["fuelCons"]
        throttle = state["latestData"].get("0x11")
        gear = state["gear"]
        event = state["event"]
        if event:
            self.eventLabel.text = f"RPM Event: {float(rpm.pid.period_ms * event.timestamp)/1000:.2f}s, Type: {event.type}, Duration: {event.length} units (Priority {event.priority})"
        else:
            
            self.eventLabel.text = "Current Event: --"

        if speed:
            self.speed_sub_label.text = (f"AvgROC: {(speed.metrics.wAvgROC):.2f} m/s^2\n"+
                                       f"Avg: {speed.metrics.average:.2f} {speed.pid.unit}\n"+
                                       f"Max: {speed.metrics.max:.2f} {speed.pid.unit}\n")
            print(f"AvgROC: {(speed.metrics.wAvgROC):.2f} m/s^2\n")
                                    #    f"Min: {speed.metrics.min:.2f} {speed.pid.unit}")
            
            self.speed_label.text = f"Speed: {int(speed.metrics.current)} {speed.pid.unit}"

        if rpm.metrics:
            self.rpm_sub_label.text = (f"AvgROC: {rpm.metrics.wAvgROC:.2f} {rpm.pid.unit}\n"+
                                       f"Avg: {rpm.metrics.average:.2f} {rpm.pid.unit}\n"+
                                       f"Max: {rpm.metrics.max:.2f} {rpm.pid.unit}\n"+
                                       f"Min: {rpm.metrics.min:.2f} {rpm.pid.unit}")
            
            self.rpm_label.text = f"RPM: {int(rpm.metrics.current)} {rpm.pid.unit}"
        if fuelCons is not None:
            self.fuelCons_label.text = f"Fuel Cons: {fuelCons:.2f} l/km"
        if throttle:
            self.throttle_label.text = f"Throttle: {throttle['value']:.2f} {throttle['unit']}"
        if gear is not None:
            if gear == 0:
                self.estGearLabel.text = f"Gear: N"
            else:   
                self.estGearLabel.text = f"Gear: {gear}"
        if state["ratio"] is not None:
            self.estGRatioLabel.text = f"Ratio: {state['ratio']:.2f}"

class MyApp(App):
    def build(self):
        self.analyser = Analyser()

        # self.worker = Thread(target=read_from_com, 
        #                      args=(self.analyser,), 
        #                      daemon = True)

        
        self.worker = Thread(target=read_csv, 
                             args=("",self.analyser,64), 
                             daemon = True)

        self.worker.start()

        return TestScreen(self.analyser)

    def on_stop(self):
        print("App is stopping, saving historic metrics...")
        self.analyser.save_HistoricMetrics()
        print("Historic Metrics saved.")

if __name__ == "__main__":
    MyApp().run()