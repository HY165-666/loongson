import tkinter as tk
from tkinter import ttk
import threading
import time
import math
import socket
import json

# ============================================================
# TCP 服务端（持久运行，断开后自动重连）
# ============================================================
class TcpServer:
    def __init__(self, host='0.0.0.0', port=12345, callback=None):
        self.host = host
        self.port = port
        self.callback = callback
        self.running = True
        self.server_sock = None
        self.client_sock = None
        self.buffer = ""

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_sock.bind((self.host, self.port))
            self.server_sock.listen(1)
            print(f"[TCP] 服务端已启动，监听 {self.host}:{self.port}")
        except Exception as e:
            print(f"[TCP] 服务端启动失败: {e}")
            return

        while self.running:
            try:
                print("[TCP] 等待 C++ 程序连接...")
                self.client_sock, addr = self.server_sock.accept()
                print(f"[TCP] C++ 程序已连接: {addr}")
                self._handle_client()
            except Exception as e:
                if self.running:
                    print(f"[TCP] 连接异常: {e}")
                break

        if self.server_sock:
            self.server_sock.close()
        print("[TCP] 服务端已关闭")

    def _handle_client(self):
        """处理单个客户端，断开后自动返回重新等待"""
        while self.running:
            try:
                data = self.client_sock.recv(4096).decode('utf-8')
                if not data:
                    print("[TCP] 客户端断开连接，等待新连接...")
                    break
                self.buffer += data
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    if line:
                        try:
                            j = json.loads(line)
                            if self.callback:
                                self.callback(j)
                        except json.JSONDecodeError:
                            print(f"[TCP] JSON 解析错误: {line}")
            except Exception as e:
                if self.running:
                    print(f"[TCP] 接收异常: {e}")
                break
        if self.client_sock:
            self.client_sock.close()
            self.client_sock = None

    def stop(self):
        self.running = False
        if self.client_sock:
            self.client_sock.close()
        if self.server_sock:
            self.server_sock.close()


# ============================================================
# 主界面类
# ============================================================
class StreetLightMonitor:
    def __init__(self, root):
        self.root = root
        self.root.title("基于龙芯1C+2K异构景区智能路灯集群监控系统")
        self.root.geometry("1280x720")
        self.root.configure(bg='#f0f0f0')

        # 数据存储
        self.lamp_data = {
            'lamp1': {'duty': 0, 'status': 0, 'power': 0.0},
            'lamp2': {'duty': 0, 'status': 0, 'power': 0.0},
            'lamp3': {'duty': 0, 'status': 0, 'power': 0.0},
        }
        self.env_data = {'light': 0.0, 'temp': 0.0, 'hum': 0.0}

        # 能耗累计
        self.smart_energy = 0.0
        self.traditional_energy = 0.0
        self.last_update_time = time.time()
        self.traditional_power_per_lamp = 1.5

        # 创建界面
        self.create_header()
        self.create_main_panel()

        # 启动 TCP 服务端
        self.receiver = TcpServer(callback=self.update_data)
        self.receiver.start()

        # 启动能耗累计定时器
        self.update_energy()

    # ---------- 界面创建函数 ----------
    def create_header(self):
        header = tk.Frame(self.root, bg='#003366', height=80)
        header.pack(side=tk.TOP, fill=tk.X)
        header.pack_propagate(False)
        title = tk.Label(header,
                         text="基于龙芯1C+2K异构景区智能路灯集群监控系统",
                         font=('Microsoft YaHei', 20, 'bold'),
                         fg='white', bg='#003366')
        title.pack(expand=True)
        logo = tk.Label(header, text="【龙芯 LoongArch】", font=('Arial', 12),
                        fg='#ffcc00', bg='#003366')
        logo.place(x=20, y=50)

    def create_main_panel(self):
        main_frame = tk.Frame(self.root, bg='#f0f0f0')
        main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=10)

        left_frame = tk.Frame(main_frame, bg='#f0f0f0')
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.cards = {}
        for i, lamp_id in enumerate(['lamp1', 'lamp2', 'lamp3'], 1):
            card = tk.Frame(left_frame, bg='white', relief=tk.RAISED, bd=2)
            card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
            self.cards[lamp_id] = self.create_card(card, lamp_id, i)

        right_frame = tk.Frame(main_frame, bg='#f0f0f0')
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        chart_frame = tk.LabelFrame(right_frame, text="系统总功率对比（实时）",
                                    font=('Microsoft YaHei', 12), bg='white')
        chart_frame.pack(fill=tk.BOTH, expand=True, pady=(0,5))
        self.chart_canvas = tk.Canvas(chart_frame, bg='white', height=200)
        self.chart_canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        energy_frame = tk.LabelFrame(right_frame, text="当日能耗统计",
                                     font=('Microsoft YaHei', 12), bg='white')
        energy_frame.pack(fill=tk.BOTH, expand=True)
        self.energy_labels = {}
        stats = ['智能系统实际耗电', '传统恒亮理论耗电', '今日累计节电', '综合节能率']
        for idx, text in enumerate(stats):
            lbl = tk.Label(energy_frame, text=f"{text}: 0.0 Wh",
                           font=('Microsoft YaHei', 11),
                           anchor='w', bg='white', padx=10)
            lbl.grid(row=idx//2, column=idx%2, sticky='w', padx=10, pady=5)
            self.energy_labels[text] = lbl

    def create_card(self, parent, lamp_id, num):
        card_data = {}
        title = tk.Label(parent, text=f"路灯 {num}",
                         font=('Microsoft YaHei', 14, 'bold'), bg='white')
        title.pack(pady=(10,5))

        power_frame = tk.Frame(parent, bg='white')
        power_frame.pack(anchor='w', padx=10, pady=2)
        tk.Label(power_frame, text="实时供电:", font=('Microsoft YaHei', 10), bg='white').pack(side=tk.LEFT)
        power_val = tk.Label(power_frame, text="0.0 W",
                             font=('Microsoft YaHei', 10, 'bold'), fg='blue', bg='white')
        power_val.pack(side=tk.LEFT)
        card_data['power'] = power_val

        duty_frame = tk.Frame(parent, bg='white')
        duty_frame.pack(anchor='w', padx=10, pady=2)
        tk.Label(duty_frame, text="当前亮度:", font=('Microsoft YaHei', 10), bg='white').pack(side=tk.LEFT)
        duty_progress = ttk.Progressbar(duty_frame, length=100, mode='determinate')
        duty_progress.pack(side=tk.LEFT, padx=5)
        duty_text = tk.Label(duty_frame, text="0%", font=('Microsoft YaHei', 9), bg='white')
        duty_text.pack(side=tk.LEFT)
        card_data['duty_progress'] = duty_progress
        card_data['duty_text'] = duty_text

        status_frame = tk.Frame(parent, bg='white')
        status_frame.pack(anchor='w', padx=10, pady=2)
        tk.Label(status_frame, text="设备状态:", font=('Microsoft YaHei', 10), bg='white').pack(side=tk.LEFT)
        status_val = tk.Label(status_frame, text="正常",
                              font=('Microsoft YaHei', 10, 'bold'), bg='white')
        status_val.pack(side=tk.LEFT)
        card_data['status'] = status_val

        env_frame = tk.Frame(parent, bg='white')
        env_frame.pack(anchor='w', padx=10, pady=2)
        env_label = tk.Label(env_frame, text="环境: --",
                             font=('Microsoft YaHei', 9), bg='white')
        env_label.pack()
        card_data['env'] = env_label
        return card_data

    # ---------- 数据更新 ----------
    def update_data(self, data):
        for lamp_id in ['lamp1', 'lamp2', 'lamp3']:
            self.lamp_data[lamp_id]['duty']   = data[lamp_id]['duty']
            self.lamp_data[lamp_id]['status'] = data[lamp_id]['status']
            self.lamp_data[lamp_id]['power']  = data[lamp_id]['power']
        self.env_data = data['env']
        self.root.after(0, self._refresh_ui)

    def _refresh_ui(self):
        for lamp_id, card in self.cards.items():
            d = self.lamp_data[lamp_id]
            card['power'].config(text=f"{d['power']:.1f} W")
            duty = d['duty']
            card['duty_progress']['value'] = duty
            card['duty_text'].config(text=f"{duty}%")
            status_map = {0: '休眠', 1: '正常', 2: '故障'}
            status_text = status_map.get(d['status'], '未知')
            color = {'休眠': 'gray', '正常': 'green', '故障': 'red'}.get(status_text, 'black')
            card['status'].config(text=status_text, fg=color)
            env_text = f"环境: 光照 {self.env_data['light']:.1f} Lux | 温度 {self.env_data['temp']:.1f}°C | 湿度 {self.env_data['hum']:.0f}%"
            card['env'].config(text=env_text)
        self.draw_chart()

    def draw_chart(self):
        self.chart_canvas.delete("all")
        width = self.chart_canvas.winfo_width()
        height = self.chart_canvas.winfo_height()
        if width < 10 or height < 10:
            return
        smart_total = sum(self.lamp_data[lid]['power'] for lid in self.lamp_data)
        traditional_total = self.traditional_power_per_lamp * 3
        max_power = max(smart_total, traditional_total, 10.0)
        bar_width = width // 4
        spacing = bar_width // 2
        start_x = (width - (2 * (bar_width + spacing))) // 2
        bottom_y = height - 20
        data = [
            ('传统恒亮', traditional_total, '#e74c3c'),
            ('智慧系统', smart_total, '#2ecc71')
        ]
        for i, (label, value, color) in enumerate(data):
            x = start_x + i * (bar_width + spacing)
            bar_height = (value / max_power) * (height - 40)
            y0 = bottom_y - bar_height
            self.chart_canvas.create_rectangle(x, y0, x + bar_width, bottom_y,
                                               fill=color, outline='')
            self.chart_canvas.create_text(x + bar_width/2, y0 - 10,
                                          text=f"{value:.1f}W", font=('Arial', 10, 'bold'))
            self.chart_canvas.create_text(x + bar_width/2, bottom_y + 15,
                                          text=label, font=('Arial', 9))
        if smart_total < traditional_total:
            self.chart_canvas.create_text(width//2, 20,
                                          text=f"⚡ 当前节能 {(traditional_total - smart_total):.1f} W",
                                          fill='#2980b9', font=('Arial', 12, 'bold'))

    def update_energy(self):
        now = time.time()
        delta = now - self.last_update_time
        self.last_update_time = now
        total_power = sum(self.lamp_data[lid]['power'] for lid in self.lamp_data)
        traditional_power = self.traditional_power_per_lamp * 3
        if delta > 0:
            hours = delta / 3600.0
            self.smart_energy += total_power * hours
            self.traditional_energy += traditional_power * hours
        saved = self.traditional_energy - self.smart_energy
        if self.traditional_energy > 0:
            saving_rate = (saved / self.traditional_energy) * 100
        else:
            saving_rate = 0.0
        self.energy_labels['智能系统实际耗电'].config(text=f"智能系统实际耗电: {self.smart_energy:.1f} Wh")
        self.energy_labels['传统恒亮理论耗电'].config(text=f"传统恒亮理论耗电: {self.traditional_energy:.1f} Wh")
        self.energy_labels['今日累计节电'].config(text=f"今日累计节电: {saved:.1f} Wh")
        self.energy_labels['综合节能率'].config(text=f"综合节能率: {saving_rate:.1f} %")
        self.root.after(1000, self.update_energy)

    def on_closing(self):
        self.receiver.stop()
        self.root.destroy()


# ============================================================
# 启动程序
# ============================================================
if __name__ == "__main__":
    root = tk.Tk()
    app = StreetLightMonitor(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()