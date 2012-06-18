"""
Graphical monitor of Flo
"""
import sys
import serial
from PySide import QtCore, QtGui

SERIAL_TIMEOUT = 0
TIMER_INTERVAL = 100

class MonitorWindow(QtGui.QWidget):
    def __init__(self, serials, parent=None):
        QtGui.QWidget.__init__(self, parent)
        
        layout = QtGui.QGridLayout()
        
        timer = QtCore.QTimer(self)
        
        self.serialWidgets = []
        i = 0
        for s in serials:
            w = SerialWidget(s, 7)
            self.serialWidgets += [w]
            self.connect(timer, QtCore.SIGNAL("timeout()"),
                             w.readSerial)
            layout.addWidget(self.createLabel(s[0]),0,i)
            layout.addWidget(w, 1, i)
            i += 1
    
        timer.start(TIMER_INTERVAL)
        self.setLayout(layout)
        
        self.setWindowTitle(self.tr("Wall Monitor"))

    def createLabel(self, text):
        label = QtGui.QLabel(text)
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setMargin(2)
        label.setFrameStyle(QtGui.QFrame.Box | QtGui.QFrame.Sunken)
        return label

class SerialWidget(QtGui.QWidget):
    num_extras = 5
    restart_color = QtGui.QColor(0, 0, 100)
    time_color = QtGui.QColor(0, 100, 100)
    send_len_color = QtGui.QColor(255, 0, 0)
    send_crc_color = QtGui.QColor(0, 255, 0)
    sonar_read_color = QtGui.QColor(0, 128, 128)
    recv_len_color = QtGui.QColor(127,34,255)
    white = QtGui.QColor(255,255,255)
    
    text_width = 200
    text_height = 20
    
    def __init__(self, ser_tup, num_devices, parent = None):
        QtGui.QWidget.__init__(self, parent)
        
        (self.port, self.ser) = ser_tup
        self.num_devices = num_devices
        
        self.serial_read_counter = 0
        self.line_counter = 0
        self.line_exception_counter = 0
        
        self.restarts = []
        self.times = []
        self.send_lens = []
        self.send_crcs = []
        self.sonar_reads = []
        self.recv_lens_arr = [[] for i in range(num_devices)]
        

        self.setBackgroundRole(QtGui.QPalette.Base)
        self.setSizePolicy(QtGui.QSizePolicy.Expanding,
                           QtGui.QSizePolicy.Expanding)
    
    @property
    def num_slots(self):
        return self.num_devices + self.num_extras
    
    def minimumSizeHint(self):
        return QtCore.QSize(50, 50)

    def sizeHint(self):
        return QtCore.QSize(1000, 900)
    
    def readSerial(self):
        newlines = self.ser.readlines()
        if newlines:
            self.serial_read_counter += 1
            self.line_counter += len(newlines)
            for line in newlines:
                raw = line.replace("\n", "").replace("\r", "")
                if len(raw) > 0:
                    cmd = "self.%s" % raw
                    print raw
                    try:
                        eval(cmd)
                    except Exception, e:
                        self.line_exception_counter += 1
                    
            self.update()
    
    def pixel(self, val, min_val, max_val, min_to, max_to):
        if val < min_val:
            return min_to
        elif val > max_val:
            return max_to
        else:
            ratio = (1.0 * val - min_val) / (max_val - min_val)
            return (max_to-min_to) * ratio + min_to
    
    @property
    def slot_height(self):
        return self.height()/self.num_slots
    
    def get_lines(self, vals, slot, min_val, max_val):
        sy = (slot+1)*self.slot_height
        screen_vals = [(serial_read_counter, self.pixel(val, min_val, max_val, 0, self.slot_height)) 
                       for (serial_read_counter, val) in vals]
        return [QtCore.QLine(sx, sy-sh, sx, sy) for (sx,sh) in screen_vals]
    
    def series(self, painter, color, slot, arr, min_val, max_val, label):
        lines = self.get_lines(arr, slot, min_val, max_val)
        painter.setPen(QtGui.QPen(color, 0))
        painter.drawLines(lines)
        painter.fillRect(QtCore.QRect(0, (slot+1)*self.slot_height, self.text_width, self.text_height), self.white)
        painter.drawText(QtCore.QPoint(0, (slot+1)*self.slot_height+self.text_height), label)
        
    def paintEvent(self, event):
        painter = QtGui.QPainter()
        painter.begin(self)

        if len(self.restarts):
            self.series(painter, self.restart_color, 0, self.restarts, 0, 100, 
                        "main %d" % self.restarts[len(self.restarts)-1][1])
            
        if len(self.times):
            self.series(painter, self.time_color, 1, self.times, 0, 100,
                        "T %d" % self.times[len(self.times)-1][1])
        
        if len(self.send_crcs):
            self.series(painter, self.send_crc_color, 2, self.send_crcs, 0, 30,
                        "Scrc %d" % self.send_crcs[len(self.send_crcs)-1][1])
        
        if len(self.send_lens):
            self.series(painter, self.send_len_color, 3, self.send_lens, 0, 30,
                        "Slen %d" % self.send_lens[len(self.send_lens)-1][1])
                        
        if len(self.sonar_reads):
            self.series(painter, self.sonar_read_color, 4, self.sonar_reads, 0, 30,
                        "sonar %d" % self.sonar_reads[len(self.sonar_reads)-1][1])
        
        for i in range(self.num_devices):
            recv_lens = self.recv_lens_arr[i]
            if len(recv_lens):
                self.series(painter, self.recv_len_color, i+self.num_extras, recv_lens, -1, 30, 
                            "R%d %d" % (i, recv_lens[len(recv_lens)-1][1]))
        

        painter.end()
    
    """ functions """
    def main(self, global_ticks):
        self.restarts += [(self.serial_read_counter, global_ticks)]
        
    def ticks(self, global_secs):
        self.times += [(self.serial_read_counter, global_secs)]
    
    def send_message(self, message_length, crc, message):
        self.send_lens += [(self.serial_read_counter, message_length)]
        self.send_crcs += [(self.serial_read_counter, crc)]
    
    def read_message(self, channel, message_length, message):
        self.recv_lens_arr[channel] += [(self.serial_read_counter, message_length)]
        
    def sonar_read(self, value):
        self.sonar_reads += [(self.serial_read_counter, value)]

if __name__ == "__main__":
    ports = ['/dev/ttyUSB0']
    speed = 115200
    serials = []
    for p in ports:
        serials += [(p, serial.Serial(p, speed, timeout=SERIAL_TIMEOUT))]
    app = QtGui.QApplication(sys.argv)
    window = MonitorWindow(serials)
    window.show()
    sys.exit(app.exec_())
    
