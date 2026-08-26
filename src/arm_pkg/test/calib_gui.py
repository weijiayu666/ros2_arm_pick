"""
机械臂偏置校准工具
- 串口连接 / 断开
- 实时显示6轴当前角度
- 拖动滑块微调偏置
- 保存/清除/读取校准数据
波特率: 115200
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import re

# ── 常量 ──────────────────────────────────────────
BAUD = 115200
SERVO_NUM = 6
AXIS_NAMES = ["1轴(底盘)", "2轴", "3轴", "4轴", "5轴", "6轴(夹爪)"]
DEFAULT_ANGLES = [0, 0, 0, -90, 0, 0]  # 归位角度(度), 4轴=-90

# ── 串口通信线程 ──────────────────────────────────
class SerialThread(threading.Thread):
    def __init__(self, port, callback):
        super().__init__(daemon=True)
        self.port = port
        self.callback = callback
        self.ser = None
        self.running = True
        self.lock = threading.Lock()

    def run(self):
        try:
            self.ser = serial.Serial(self.port, BAUD, timeout=0.05)
        except Exception as e:
            self.callback("error", str(e))
            return
        buf = ""
        while self.running:
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting).decode("utf-8", errors="replace")
                    buf += data
                    while "!" in buf:
                        idx = buf.index("!")
                        line = buf[:idx+1]
                        buf = buf[idx+1:]
                        self.callback("rx", line.strip())
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.callback("error", str(e))
                break

    def send(self, cmd):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.write((cmd + "\n").encode())  # \n 帮助触发串口传输
                self.callback("tx", cmd)

    def stop(self):
        self.running = False
        if self.ser:
            self.ser.close()


# ── 主窗口 ────────────────────────────────────────
class CalibApp:
    def __init__(self, root):
        self.root = root
        self.root.title("机械臂偏置校准")
        self.root.resizable(False, False)
        self.serial = None

        # 角度显示值
        self.angles = [tk.StringVar(value="?") for _ in range(SERVO_NUM)]
        # 偏置值 (0.1° 单位)
        self.offsets = [tk.IntVar(value=0) for _ in range(SERVO_NUM)]
        # 舵机 PWM 值
        self.pwms = [tk.StringVar(value="?") for _ in range(SERVO_NUM)]

        self._build_ui()
        self._refresh_ports()

    # ── UI 构建 ───────────────────────────────────
    def _build_ui(self):
        main = ttk.Frame(self.root, padding=10)
        main.grid(row=0, column=0, sticky="nsew")

        # ── 顶部：串口连接 ──
        row = 0
        top = ttk.LabelFrame(main, text="串口", padding=5)
        top.grid(row=row, column=0, columnspan=6, sticky="ew", pady=(0,8))

        ttk.Label(top, text="端口:").pack(side=tk.LEFT)
        self.cb_port = ttk.Combobox(top, width=10, state="readonly")
        self.cb_port.pack(side=tk.LEFT, padx=4)

        ttk.Button(top, text="刷新", width=4, command=self._refresh_ports).pack(side=tk.LEFT, padx=2)

        self.btn_conn = ttk.Button(top, text="连接", command=self._toggle_connect)
        self.btn_conn.pack(side=tk.LEFT, padx=4)

        self.lb_status = ttk.Label(top, text="未连接", foreground="gray")
        self.lb_status.pack(side=tk.LEFT, padx=10)

        ttk.Button(top, text="读取当前角度", command=self._read_angles).pack(side=tk.RIGHT, padx=4)
        ttk.Button(top, text="读取偏置", command=self._read_offsets).pack(side=tk.RIGHT, padx=4)

        # ── 中部：6轴控制 ──
        row += 1
        cols = ["轴", "归位角°", "当前角°", "当前PWM", "偏置°", "微调", "目标角°", "", "控制"]
        for j, c in enumerate(cols):
            ttk.Label(main, text=c, font=("", 9, "bold")).grid(
                row=row, column=j, padx=6, pady=2)

        self.scale_widgets = []
        self.target_vars = []  # 目标角度输入变量(度)
        for i in range(SERVO_NUM):
            r = row + 1 + i
            bg = "#f0f0f0" if i % 2 == 0 else "#ffffff"

            ttk.Label(main, text=AXIS_NAMES[i], background=bg, width=10).grid(
                row=r, column=0, padx=2, pady=1, sticky="w")

            ttk.Label(main, text=f"{DEFAULT_ANGLES[i]:+.1f}°", background=bg).grid(
                row=r, column=1, padx=6)

            ttk.Label(main, textvariable=self.angles[i], foreground="blue",
                      font=("", 10, "bold"), background=bg).grid(row=r, column=2, padx=6)

            ttk.Label(main, textvariable=self.pwms[i], foreground="gray",
                      background=bg).grid(row=r, column=3, padx=6)

            # 偏置调节滑条
            frame = ttk.Frame(main)
            frame.grid(row=r, column=4, padx=2)
            scale = ttk.Scale(frame, from_=100, to=-100, variable=self.offsets[i],
                              length=120, command=lambda v, idx=i: self._on_offset_drag(idx))
            scale.pack(side=tk.LEFT)
            lb = ttk.Label(frame, textvariable=self.offsets[i], width=5)
            lb.pack(side=tk.LEFT)
            ttk.Label(frame, text="0.1°", foreground="gray").pack(side=tk.LEFT)
            self.scale_widgets.append((scale, lb))

            # 微调按钮
            btn_frame = ttk.Frame(main)
            btn_frame.grid(row=r, column=5, padx=2)
            ttk.Button(btn_frame, text="−", width=2,
                       command=lambda idx=i: self._nudge(idx, -5)).pack(side=tk.LEFT, padx=1)
            ttk.Button(btn_frame, text="+", width=2,
                       command=lambda idx=i: self._nudge(idx, +5)).pack(side=tk.LEFT, padx=1)
            ttk.Button(btn_frame, text="应用", width=4,
                       command=lambda idx=i: self._apply_one(idx)).pack(side=tk.LEFT, padx=2)

            # 目标角度输入 + 移动按钮
            tgt_var = tk.StringVar(value="")
            self.target_vars.append(tgt_var)
            ttk.Entry(main, textvariable=tgt_var, width=6,
                      font=("", 9)).grid(row=r, column=6, padx=2)
            ttk.Label(main, text="°", background=bg, foreground="gray").grid(
                row=r, column=7, padx=0)
            ttk.Button(main, text="移动", width=4,
                       command=lambda idx=i: self._move_servo(idx)).grid(
                row=r, column=8, padx=2)

        # ── 底部：整体操作 ──
        row += SERVO_NUM + 1
        bot = ttk.Frame(main)
        bot.grid(row=row, column=0, columnspan=9, pady=(10, 0), sticky="ew")

        ttk.Button(bot, text="应用全部偏置", command=self._apply_all).pack(side=tk.LEFT, padx=4)
        ttk.Button(bot, text="保存到Flash", command=self._save_calib).pack(side=tk.LEFT, padx=4)
        ttk.Button(bot, text="清除全部偏置", command=self._clear_all).pack(side=tk.LEFT, padx=4)
        ttk.Button(bot, text="全部归位", command=self._home_all).pack(side=tk.LEFT, padx=4)

        # ── 日志区 ──
        row += 1
        self.log = tk.Text(main, height=6, width=75, state=tk.DISABLED, font=("Consolas", 9))
        self.log.grid(row=row, column=0, columnspan=9, pady=(8, 0), sticky="ew")

    # ── 串口操作 ───────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cb_port["values"] = ports
        if ports:
            self.cb_port.set(ports[0])

    def _toggle_connect(self):
        if self.serial:
            self.serial.stop()
            self.serial = None
            self.btn_conn.config(text="连接")
            self.lb_status.config(text="已断开", foreground="gray")
            self._log("串口已断开")
        else:
            port = self.cb_port.get()
            if not port:
                messagebox.showwarning("提示", "请选择串口")
                return
            self._log(f"连接 {port} ...")
            self.serial = SerialThread(port, self._on_serial)
            self.serial.start()
            self.btn_conn.config(text="断开")
            self.lb_status.config(text=f"已连接 {port}", foreground="green")
            self.root.after(500, self._read_angles)

    def _on_serial(self, event, data):
        self.root.after(0, lambda: self._handle_serial(event, data))

    def _handle_serial(self, event, data):
        if event == "tx":
            self._log(f"→ {data}")
        elif event == "rx":
            self._log(f"← {data}")
            self._parse_rx(data)
        elif event == "error":
            self._log(f"⚠ {data}")

    def _send(self, cmd):
        if self.serial:
            self.serial.send(cmd)

    # ── 解析响应 ───────────────────────────────────
    def _parse_rx(self, data):
        # #AA+0.0,+2.5,-1.0,...!
        m = re.match(r"#AA([-+0-9.,]+)!", data)
        if m:
            parts = m.group(1).split(",")
            for i, p in enumerate(parts[:SERVO_NUM]):
                try:
                    val = float(p)
                    self.angles[i].set(f"{val:+.1f}°")
                except:
                    pass
            return

        # #OF+0.0,-2.5,...!
        m = re.match(r"#OF([-+0-9.,]+)!", data)
        if m:
            parts = m.group(1).split(",")
            for i, p in enumerate(parts[:SERVO_NUM]):
                try:
                    val = float(p)
                    self.offsets[i].set(int(val * 10))  # 度→0.1°
                except:
                    pass
            self._log("偏置数据已同步")
            return

        # #A00,+45.0!
        m = re.match(r"#A(\d+),([-+0-9.]+)!", data)
        if m:
            sid = int(m.group(1))
            if sid < SERVO_NUM:
                try:
                    self.angles[sid].set(f"{float(m.group(2)):+.1f}°")
                except:
                    pass
            return

    # ── 命令 ───────────────────────────────────────
    def _move_servo(self, idx):
        """移动指定舵机到目标角度(度)"""
        val = self.target_vars[idx].get().strip()
        if not val:
            return
        try:
            angle_deg = float(val)
        except ValueError:
            self._log(f"轴{idx+1} 角度格式错误: {val}")
            return
        # 固件角度单位是 0.1°，且约定范围 ±135.0°
        if angle_deg < -135.0 or angle_deg > 135.0:
            self._log(f"轴{idx+1} 角度超出范围 ±135.0°")
            return
        angle_deci = int(angle_deg * 10)
        self._send(f"$SA:{idx},{angle_deci}!")
        self._log(f"轴{idx+1} → {angle_deg:+.1f}°")

    def _read_angles(self):
        self._send("$GAA!")
    def _read_offsets(self):
        self._send("$ZCRD!")
    def _save_calib(self):
        self._send("$ZCS!")
    def _clear_all(self):
        if messagebox.askyesno("确认", "清除全部偏置？"):
            for i in range(SERVO_NUM):
                self.offsets[i].set(0)
            self._send("$ZCR!")
    def _home_all(self):
        self._send("$ZCS!")

    def _on_offset_drag(self, idx):
        """滑条拖动时实时显示"""
        off = self.offsets[idx].get()
        self._log(f"轴{idx+1} 偏置 → {off/10:+.1f}°")

    def _nudge(self, idx, delta):
        """微调按钮"""
        cur = self.offsets[idx].get()
        self.offsets[idx].set(cur + delta)
        self._log(f"轴{idx+1} 微调 {delta/10:+.1f}° → {self.offsets[idx].get()/10:+.1f}°")

    def _apply_one(self, idx):
        off = self.offsets[idx].get()
        self._send(f"$ZCSET:{idx},{off}!")
        self._log(f"轴{idx+1} 偏置已发送: {off/10:+.1f}°")

    def _apply_all(self):
        for i in range(SERVO_NUM):
            off = self.offsets[i].get()
            self._send(f"$ZCSET:{i},{off}!")
        self._log("全部偏置已发送")

    # ── 日志 ───────────────────────────────────────
    def _log(self, msg):
        self.log.config(state=tk.NORMAL)
        t = time.strftime("%H:%M:%S")
        self.log.insert(tk.END, f"[{t}] {msg}\n")
        self.log.see(tk.END)
        self.log.config(state=tk.DISABLED)


# ── 入口 ──────────────────────────────────────────
if __name__ == "__main__":
    root = tk.Tk()
    app = CalibApp(root)
    root.mainloop()
