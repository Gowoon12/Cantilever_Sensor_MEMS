import csv
import datetime
import os
import time, socket, collections, sys
from typing import Optional
from scipy.signal import savgol_filter, find_peaks
import numpy as np
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

VREF, ADC_MAX = 3.3, 4095


# ======================== UDP 수신 쓰레드 ========================
class _UdpWorker(QtCore.QThread):
    """UDP 패킷을 받아서 (t, v1, v2, v3) 신호 송출"""
    dataReady = QtCore.pyqtSignal(float, float, float, float)

    def __init__(self, iface: str, port: int, parent=None):
        super().__init__(parent)
        self.iface, self.port = iface, port
        self._stop = False

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.iface, self.port))
        sock.setblocking(False)
        while not self._stop:
            try:
                pkt, _ = sock.recvfrom(64)
            except BlockingIOError:
                self.msleep(1)
                continue
            now = time.perf_counter()
            try:
                segs = pkt.decode(errors="ignore").split()
                adc1 = int(segs[1].split(":")[1])
                adc2 = int(segs[2].split(":")[1])
                adc3 = int(segs[3].split(":")[1])
            except Exception as e:
                print("[ERROR] parsing failed:", e, "segs=", segs)
                continue

            v1 = adc1 * VREF / ADC_MAX
            v2 = adc2 * VREF / ADC_MAX
            v3 = adc3 * VREF / ADC_MAX
            self.dataReady.emit(now, v1, v2, v3)

    def stop(self):
        self._stop = True
        self.wait()


# ======================== Worker 공유 풀 ========================
_WORKER_POOL = {}  # {(iface, port): _UdpWorker}

def get_udp_worker(iface: str, port: int, parent=None) -> _UdpWorker:
    key = (iface, port)
    if key not in _WORKER_POOL:
        worker = _UdpWorker(iface, port, parent)
        _WORKER_POOL[key] = worker
        worker.start()
    return _WORKER_POOL[key]


# ======================== 단일 채널 위젯 ========================
class ADCPlotWidget(QtWidgets.QWidget):
    fpsChanged = QtCore.pyqtSignal(float)
    metricsUpdated = QtCore.pyqtSignal(float, float)  # (frequency, peak_height)

    def __init__(self,
                 iface: str = "0.0.0.0",
                 port: int = 7001,
                 channel: int = 1,
                 win_sec: float = 10.0,
                 history_sec: float = 90.0,
                 sensor_id: str = "1",
                 parent=None):
        super().__init__(parent)
        self.iface, self.port, self.channel = iface, port, channel
        self.win_sec = win_sec
        self.history_sec = history_sec
        self.sensor_id = sensor_id

        # --------- Plot 설정 ---------
        self._plot = pg.PlotWidget()
        self._plot.setLabel("left", "Voltage (V)")
        self._plot.setLabel("bottom", "Time", units="s")
        self._plot.setYRange(-0.5, 2.0)
        self._plot.showGrid(x=True, y=True)
        self._plot.getPlotItem().invertX(True)

        self._curve = self._plot.plot(pen=pg.mkPen("b", width=1.5),
                                      name=f"ADC{channel}")
        self._filt_curve = self._plot.plot(pen=pg.mkPen("y", width=2))
        self._peak_scatter = self._plot.plot(pen=None, symbol="o", symbolSize=8,
                                             symbolBrush=pg.mkBrush("r"))
        # valley (파란 점)
        self._valley_scatter = self._plot.plot(
            pen=None,
            symbol="o", symbolSize=8,
            symbolBrush=pg.mkBrush("b")
        )
        self._fps_text = pg.TextItem(anchor=(0, 1))
        self._plot.addItem(self._fps_text)

        lay = QtWidgets.QVBoxLayout(self)
        lay.addWidget(self._plot)

        # --------- 데이터 버퍼 (플롯용) ---------
        self._tbuf = collections.deque()
        self._vbuf = collections.deque()
        self._fps = collections.deque()

        # --------- 타이머 ---------
        self._timer = QtCore.QTimer(self, timeout=self._refresh, interval=30)

        # --------- UDP 쓰레드 (공유) ---------
        self._worker = get_udp_worker(self.iface, self.port, self)
        self._worker.dataReady.connect(self._on_data)

        # --------- 기록 ---------
        self._recording = False
        self._rec_file = None
        self._rec_writer = None
        self._rec_path = None
        self._rec_t0 = None

        # --------- 자동 저장 (10분마다 파일 롤링) ---------
        self._autosave_timer = QtCore.QTimer(self)
        self._autosave_timer.timeout.connect(self._auto_save_record)
        self._autosave_timer.start(30 * 60 * 1000)

    # ---------- 데이터 기록 ----------
    def start_recording(self, dir_path="data", prefix="sensor"):
        if self._recording:
            return
        # 날짜별 폴더
        date_str = datetime.datetime.now().strftime("%Y-%m-%d")
        base_dir = os.path.join(dir_path, date_str, f"Sensor{self.sensor_id}")
        os.makedirs(base_dir, exist_ok=True)

        # 파일명 (로컬 시간)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._rec_path = os.path.join(base_dir, f"{prefix}_{stamp}.csv")
        # 파일 열기
        self._rec_file = open(self._rec_path, "w", newline="")
        self._rec_writer = csv.writer(self._rec_file)
        self._rec_writer.writerow(["utc_iso", "relative_s", "voltage_V"])

        self._recording = True
        self._rec_t0 = None

    def stop_recording(self):
        self._recording = False
        if self._rec_file:
            self._rec_file.close()
            self._rec_file = None
            self._rec_writer = None

    def save_record(self, path=None):
        """현재 파일 닫고 경로 반환"""
        if self._rec_file:
            self._rec_file.close()
            saved_path = self._rec_path
            self._rec_file = None
            self._rec_writer = None
            self._recording = False
            return saved_path
        return None

    def _auto_save_record(self):
        """주기마다 자동 파일 분할 (새 파일 생성)"""
        if self._recording and self._rec_file:
            # 현재 파일 닫기
            self._rec_file.close()
            print(f"[AutoSave] 저장 완료: {self._rec_path}")

            # 새 파일명 강제 생성
            date_str = datetime.datetime.utcnow().strftime("%Y-%m-%d")
            base_dir = os.path.join("data", date_str, f"Sensor{self.sensor_id}")
            os.makedirs(base_dir, exist_ok=True)

            stamp = datetime.datetime.utcnow().strftime("%Y%m%d_%H%M%S")
            self._rec_path = os.path.join(base_dir, f"sensor{self.sensor_id}_{stamp}.csv")

            # 새 파일 열기
            self._rec_file = open(self._rec_path, "w", newline="")
            self._rec_writer = csv.writer(self._rec_file)
            self._rec_writer.writerow(["utc_iso", "relative_s", "voltage_V"])
            self._rec_t0 = None


    # ---------- 실행 제어 ----------
    def start(self):
        self._timer.start()

    def stop(self):
        self._timer.stop()

    def clear(self):
        self._tbuf.clear()
        self._vbuf.clear()
        self._fps.clear()
        self._curve.clear()
        self._filt_curve.clear()
        self._peak_scatter.clear()

    # ---------- 데이터 수신 ----------
    def _on_data(self, t, v1, v2, v3):
        if self.channel == 1: v = v1
        elif self.channel == 2: v = v2
        else: v = v3

        # 플롯 버퍼
        self._tbuf.append(t)
        self._vbuf.append(v)
        self._fps.append(t)

        while self._tbuf and (t - self._tbuf[0] > self.history_sec):
            self._tbuf.popleft()
            self._vbuf.popleft()
        while self._fps and (t - self._fps[0] > 1.0):
            self._fps.popleft()

        # ---- 파일 기록 ----
        if self._recording and self._rec_writer:
            if self._rec_t0 is None:
                self._rec_t0 = t
            utc_iso = datetime.datetime.utcnow().isoformat(timespec="milliseconds")
            self._rec_writer.writerow([utc_iso, t - self._rec_t0, v])
            self._rec_file.flush()

    # ---------- 신호 처리 ----------
    def _calc_filtered(self, ys):
        if len(ys) < 7:
            return ys
        return savgol_filter(ys, window_length=7, polyorder=2)

    def _find_peaks(self, xs, ys):
        y = np.asarray(ys)
        if y.size < 3:
            return np.array([], dtype=int)

        # ---- 동적 파라미터 ----
        amp_range = np.max(y) - np.min(y)
        threshold = np.mean(y) + 0.2 * np.std(y)      # 동적 height
        prominence = 0.1 * amp_range                  # 동적 prominence

        dt = (xs[-1] - xs[0]) / max(len(xs)-1, 1)
        min_dist_samples = max(1, int(0.05 / max(dt, 1e-3)))  # 최소 50ms

        idx, _ = find_peaks(
            y,
            height=threshold,
            prominence=prominence,
            distance=min_dist_samples
        )
        return idx

    # ---------- 화면 갱신 ----------
    def _refresh(self):
        if not self._tbuf:
            return
        t_latest = self._tbuf[-1]
        xs, ys = [], []
        for tt, v in zip(reversed(self._tbuf), reversed(self._vbuf)):
            dt = t_latest - tt
            if dt <= self.win_sec:
                xs.append(dt)
                ys.append(v)
            else:
                break
        xs.reverse(); ys.reverse()

        if len(xs) < 3:
            return

        # ----- 원본 플롯 -----
        self._curve.setData(xs, ys)
        self._plot.setXRange(0, self.win_sec, padding=0)

        # ----- FPS -----
        if self._fps:
            fps = len(self._fps) / max(self._fps[-1] - self._fps[0], 1e-3)
            self._fps_text.setText(f"{fps:4.1f} fps")
            self._fps_text.setPos(0, VREF)
            self.fpsChanged.emit(fps)

        # ----- 필터링 -----
        filt_ys = self._calc_filtered(ys)
        self._filt_curve.setData(xs, filt_ys)

        # ----- 피크 & 밸리 탐지 -----
        peaks_idx = self._find_peaks(xs, filt_ys)
        valleys_idx = self._find_peaks(xs, -np.array(filt_ys))  # 🔵 반전으로 valley 탐지

        self._peak_scatter.setData([xs[i] for i in peaks_idx],
                                   [filt_ys[i] for i in peaks_idx])

        # (밸리 시각화도 원하면 __init__()에 self._valley_scatter 추가 후 아래 라인 활성화)
        self._valley_scatter.setData([xs[i] for i in valleys_idx],
                                     [filt_ys[i] for i in valleys_idx])

        # ----- 주파수 & 진폭 계산 -----
        freq, amplitude = 0.0, 0.0

        # 주파수: 피크 간 평균 간격의 역수
        if len(peaks_idx) > 1:
            intervals = np.diff([xs[i] for i in peaks_idx])
            if len(intervals) > 0:
                freq = - 1.0 / np.mean(intervals)   # ✅ 양수로 수정

        # 진폭: 직전 valley ~ peak 차이
        if len(peaks_idx) > 0 and len(valleys_idx) > 0:
            last_peak = peaks_idx[-1]
            prev_valleys = [v for v in valleys_idx if v < last_peak]
            if prev_valleys:
                last_valley = prev_valleys[-1]
                amplitude = filt_ys[last_peak] - filt_ys[last_valley]

        # ----- 메트릭 업데이트 -----
        self.metricsUpdated.emit(freq, amplitude)


    def closeEvent(self, e):
        self.stop()
        self.stop_recording()
        super().closeEvent(e)
