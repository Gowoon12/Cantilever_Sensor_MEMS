import sys
from PyQt5 import QtWidgets, uic
from adc_plot_widget_multi import ADCPlotWidget   # 파일명 맞게 수정하세요

BASE_PORT = 7001


class SingleSensorTable(QtWidgets.QWidget):
    def __init__(self, index, port, channel, parent=None):
        super().__init__(parent)
        uic.loadUi('sensorplot.ui', self)

        self.sensor_id = str(index)
        self.port = port
        self.channel = channel
        self.groupBox.setTitle(f"Sensor {index} (Port {port}, Ch {channel})")

        # Plot 위젯
        self.plot = ADCPlotWidget(port=port, channel=channel,
                                  parent=self, sensor_id=self.sensor_id)
        self.verticalLayout.addWidget(self.plot)
        self.plot.start()

        self._is_recording = False
        self.plot.metricsUpdated.connect(self.update_metrics)

    def update_metrics(self, freq: float, peak_height: float):
        self.label_2.setText(f"{freq:.2f}")
        self.label_4.setText(f"{peak_height:.2f}")

    def f_pushButton_record(self):
        if not self._is_recording:
            self.plot.start_recording(prefix=f"sensor{self.sensor_id}")
            self._is_recording = True
            self.changestate("Recording")
            window.add_log(f"Started recording {self.groupBox.title()}.",
                           type='sensor data process')
        else:
            self.plot.stop_recording()
            self._is_recording = False
            self.changestate("Paused")
            window.add_log(f"Paused recording {self.groupBox.title()}.",
                           type='sensor data process')

    def f_pushButton_save(self):
        saved_path = self.plot.save_record()
        if saved_path:
            self._is_recording = False
            self.changestate("Ok")
            window.add_log(f"Saved {self.groupBox.title()} → {saved_path}.",
                           type='sensor data process')
        else:
            window.add_log(f"No data to save for {self.groupBox.title()}.",
                           type='sensor data process')

    def f_pushButton_clear(self):
        self.plot.clear()
        self.changestate("Cleared")
        window.add_log(f"{self.groupBox.title()} data cleared.",
                       type='sensor data process')

    def changestate(self, state):
        self.label_state.setText(state)


class MyMainWindow(QtWidgets.QMainWindow):
    def __init__(self, sensor_count: int):
        super().__init__()
        uic.loadUi('main.ui', self)
        self.sensorlayout = self.sensorgridLayout
        self.logindex = 0
        self.add_log("Welcome to the Multi-Sensor Monitoring System!", type='info')

        self.sensortables = []
        for i in range(sensor_count):
            port = BASE_PORT + (i // 3)   # 7006, 7007, 7008
            channel = (i % 3) + 1         # 1, 2, 3
            table = SingleSensorTable(i + 1, port, channel, self)
            self.sensortables.append(table)
            self.sensorlayout.addWidget(table, i // 3, i % 3)

        self.add_log(f"Created {sensor_count} sensors.", type='info')

    def add_log(self, text, type='info'):
        self.logindex += 1
        color = {'info': 'black', 'sensor setup': 'orange',
                 'sensor data process': 'green', 'error': 'red'}.get(type, 'black')
        self.textBrowser.append(f"{self.logindex} <font color='{color}'>[{type}] {text}</font>")
        self.textBrowser.verticalScrollBar().setValue(
            self.textBrowser.verticalScrollBar().maximum())


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    sensor_count = 9   # 3포트 × 3채널
    window = MyMainWindow(sensor_count)
    window.show()
    sys.exit(app.exec_())
