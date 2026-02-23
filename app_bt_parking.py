import math
import time
import threading
import random
import queue
import re
from dataclasses import dataclass
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    import serial.tools.list_ports as list_ports
except Exception:
    print("SERIAL:", serial)
    print("LIST_PORTS:", list_ports)
    serial = None
    list_ports = None

# =========================
# THEME / SIZING
# =========================
BG = "#1e1f26"
FG = "#eaeaea"
OK = "#67d667"
WARN = "#e6b84a"
BAD = "#e05b5b"
ACCENT = "#8ab4ff"

CAR_LEN = 120
CAR_WID = 60
PX_PER_CM = 2.0
MAX_CM = 300
REFRESH_MS = 50

BAUD_OPTIONS = [9600, 19200, 38400, 57600, 115200]

SENSORS = [
    {"key": "F", "name": "Frente",    "offset": ( CAR_LEN*0.35,  0.0 ),        "angle": 0.0},
    {"key": "R", "name": "Derecha",   "offset": ( 0.0,           CAR_WID*0.7 ), "angle":  math.pi/2},
    {"key": "B", "name": "Atrás",     "offset": (-CAR_LEN*0.35,  0.0 ),        "angle":  math.pi},
    {"key": "L", "name": "Izquierda", "offset": ( 0.0,          -CAR_WID*0.7 ), "angle": -math.pi/2},
]

def clamp(v, a, b):
    return max(a, min(b, v))

def rotate_point(px, py, ang):
    ca, sa = math.cos(ang), math.sin(ang)
    return px*ca - py*sa, px*sa + py*ca

def world_from_car(local_pt, car_pos, car_ang):
    lx, ly = local_pt
    rx, ry = rotate_point(lx, ly, car_ang)
    return car_pos[0] + rx, car_pos[1] + ry

# =========================
# SERIAL WORKER
# =========================
class SerialWorker:
    def __init__(self):
        self.ser = None
        self.running = True
        self.thread = None
        self.lock = threading.Lock()

        self.mock = True
        self.cm = {s["key"]: MAX_CM for s in SENSORS}
        self.estado = None
        self.last_raw = ""

        self.events = queue.Queue()

    def available_ports(self):
        if list_ports is None:
            print("list_ports es None")
            return []
        ports = [p.device for p in list_ports.comports()]
        print("Puertos detectados:", ports)
        return ports

    def connect(self, port: str, baud: int) -> bool:
        self.disconnect()
        if serial is None:
            raise RuntimeError("pyserial no instalado. Instalá con: pip install pyserial")

        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0.2)
            self.mock = False

            if self.thread is None or not self.thread.is_alive():
                self.thread = threading.Thread(target=self._loop, daemon=True)
                self.thread.start()
            return True
        except Exception:
            self.ser = None
            self.mock = True
            return False

    def disconnect(self):
        with self.lock:
            if self.ser:
                try:
                    self.ser.close()
                except:
                    pass
            self.ser = None
        self.mock = True

    def send(self, msg: str):
        msg = (msg or "").strip()
        if not msg or self.mock:
            return
        try:
            with self.lock:
                if self.ser and self.ser.writable():
                    self.ser.write((msg + "\n").encode("utf-8"))
        except:
            pass

    def _parse_line(self, line: str) -> dict:
        line_u = line.strip().upper()
        parsed = {}
        if not line_u:
            return parsed

        for k, v in re.findall(r"([A-Z]+)\s*:\s*(\d+)", line_u):
            try:
                parsed[k] = int(v)
            except:
                pass

        # EDU-CIAA -> UI
        if "I" in parsed and "L" not in parsed:
            parsed["L"] = parsed["I"]
        if "D" in parsed and "R" not in parsed:
            parsed["R"] = parsed["D"]
        if "A" in parsed and "B" not in parsed:
            parsed["B"] = parsed["A"]
        if "ESTADO" in parsed and "E" not in parsed:
            parsed["E"] = parsed["ESTADO"]

        return parsed

    def _mock_step(self):
        for s in SENSORS:
            k = s["key"]
            cur = self.cm.get(k, MAX_CM)
            cur += random.randint(-7, 7)
            self.cm[k] = clamp(cur, 20, MAX_CM)
        self.estado = random.randint(0, 5)

    def _post_update(self):
        self.events.put(("data", dict(self.cm), self.estado, self.last_raw))

    def _loop(self):
        buf = ""
        while self.running:
            if self.ser is None or self.mock:
                self._mock_step()
                self.last_raw = "(MOCK) I/D/F/A..."
                self._post_update()
                time.sleep(0.05)
                continue

            try:
                with self.lock:
                    chunk = self.ser.read(256) if self.ser else b""

                if not chunk:
                    continue

                text = chunk.decode("utf-8", errors="ignore")
                buf += text

                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip("\r")
                    if not line.strip():
                        continue

                    self.last_raw = line[:250]
                    parsed = self._parse_line(line)

                    if parsed:
                        for s in SENSORS:
                            k = s["key"]
                            if k in parsed:
                                self.cm[k] = clamp(parsed[k], 0, MAX_CM)
                        if "E" in parsed:
                            self.estado = parsed["E"]

                    self._post_update()

            except Exception:
                self.disconnect()

# =========================
# PARK EVAL
# =========================
@dataclass
class ParkingResult:
    suitable: bool
    gap_cm: float
    reason: str

def evaluate_parking(cm_left, cm_right, car_len_cm=CAR_LEN/PX_PER_CM, margin_cm=20):
    if cm_left is None or cm_right is None:
        return ParkingResult(False, 0, "lectura inválida")
    if cm_left < 40 or cm_right < 40:
        return ParkingResult(False, cm_left + cm_right, "muy cerca de un lateral")
    gap = cm_left + cm_right
    need = car_len_cm + margin_cm
    if gap >= need:
        return ParkingResult(True, gap, f"espacio suficiente (necesita ~{int(need)} cm)")
    else:
        return ParkingResult(False, gap, f"espacio insuficiente (requiere ~{int(need)} cm)")

# =========================
# SCROLLABLE PANEL (fix cut UI)
# =========================
class ScrollFrame(tk.Frame):
    def __init__(self, parent, bg=BG):
        super().__init__(parent, bg=bg)
        self.canvas = tk.Canvas(self, bg=bg, highlightthickness=0)
        self.vsb = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.vsb.set)

        self.inner = tk.Frame(self.canvas, bg=bg)
        self.inner_id = self.canvas.create_window((0, 0), window=self.inner, anchor="nw")

        self.canvas.pack(side="left", fill="both", expand=True)
        self.vsb.pack(side="right", fill="y")

        self.inner.bind("<Configure>", self._on_configure)
        self.canvas.bind("<Configure>", self._on_canvas_configure)

        # wheel scroll
        self.canvas.bind_all("<MouseWheel>", self._on_mousewheel)

    def _on_configure(self, event):
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        # make inner frame width follow canvas width
        self.canvas.itemconfig(self.inner_id, width=event.width)

    def _on_mousewheel(self, event):
        # Windows: event.delta multiples of 120
        self.canvas.yview_scroll(int(-1*(event.delta/120)), "units")

# =========================
# APP
# =========================
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("EDU-CIAA + HC-05 · Visualizador + Control")
        self.configure(bg=BG)

        # ---- responsive: use screen size
        sw = self.winfo_screenwidth()
        sh = self.winfo_screenheight()

        # target size but clamp to screen
        target_w, target_h = 1080, 650
        w = min(target_w, max(820, sw - 80))
        h = min(target_h, max(560, sh - 80))

        # center
        x = (sw - w) // 2
        y = (sh - h) // 2
        self.geometry(f"{w}x{h}+{x}+{y}")

        # allow resize (fix cut UI)
        self.minsize(820, 560)
        self.resizable(True, True)

        # layout constants derived from current window size
        self.PANEL_W = 360
        self.canvas_w = w - self.PANEL_W - 30
        self.canvas_h = h - 20

        self.car_x = self.canvas_w * 0.5
        self.car_y = self.canvas_h * 0.6
        self.car_ang = 0.0

        self.worker = SerialWorker()
        self.cm = {s["key"]: MAX_CM for s in SENSORS}
        self.estado = None
        self.mode_parking = False

        self._build_ui()
        self.after(1000,self.refresh_ports)
        self.after(REFRESH_MS, self._tick)
        self.after(30, self._poll_events)
        self.bind("<Configure>", self._on_resize)
        

    def _build_ui(self):
        # Use grid so it adapts
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=0)

        self.canvas = tk.Canvas(self, bg=BG, highlightthickness=0)
        self.canvas.grid(row=0, column=0, padx=(10, 5), pady=10, sticky="nsew")

        # scrollable panel at right
        self.panel_sf = ScrollFrame(self, bg=BG)
        self.panel_sf.grid(row=0, column=1, padx=(5, 10), pady=10, sticky="ns")
        self.panel_sf.configure(width=self.PANEL_W)

        panel = self.panel_sf.inner

        tk.Label(panel, text="Controles", fg=FG, bg=BG, font=("Segoe UI", 14, "bold")).pack(pady=(6, 10), anchor="w")

        # COM + refresh
        row1 = tk.Frame(panel, bg=BG)
        row1.pack(fill="x")
        tk.Label(row1, text="COM:", fg=FG, bg=BG).pack(side="left")
        self.cb_ports = ttk.Combobox(row1, values=self.worker.available_ports(), state="readonly")
        self.cb_ports.pack(side="left", fill="x", expand=True, padx=(6, 6))
        ttk.Button(row1, text="Refrescar", command=self.refresh_ports).pack(side="left")

        # baud
        row2 = tk.Frame(panel, bg=BG)
        row2.pack(fill="x", pady=(8, 0))
        tk.Label(row2, text="Baud:", fg=FG, bg=BG).pack(side="left")
        self.cb_baud = ttk.Combobox(row2, values=BAUD_OPTIONS, state="readonly", width=10)
        self.cb_baud.pack(side="left", padx=(6, 0))
        self.cb_baud.set("9600")

        self.lbl_conn = tk.Label(panel, text="Estado: MOCK (sin puerto)", fg=ACCENT, bg=BG, anchor="w")
        self.lbl_conn.pack(fill="x", pady=8)

        btn_row = tk.Frame(panel, bg=BG)
        btn_row.pack(fill="x", pady=(0, 10))
        ttk.Button(btn_row, text="Conectar", command=self.connect_port).pack(side="left", expand=True, fill="x", padx=(0, 5))
        ttk.Button(btn_row, text="Desconectar", command=self.disconnect_port).pack(side="left", expand=True, fill="x", padx=(5, 0))

        ttk.Separator(panel, orient="horizontal").pack(fill="x", pady=10)

        ttk.Button(panel, text="START", command=lambda: self.send_cmd("START")).pack(fill="x", pady=4)
        ttk.Button(panel, text="STOP",  command=lambda: self.send_cmd("STOP")).pack(fill="x", pady=4)
        ttk.Button(panel, text="PARK",  command=lambda: self.send_cmd("PARK")).pack(fill="x", pady=4)

        ttk.Separator(panel, orient="horizontal").pack(fill="x", pady=10)

        self.lbl_read = tk.Label(panel, text="F: --  R: --  B: --  L: --", fg=ACCENT, bg=BG, font=("Consolas", 11))
        self.lbl_read.pack(fill="x", pady=(0, 2))

        self.lbl_estado = tk.Label(panel, text="Estado (firmware): —", fg=FG, bg=BG, anchor="w")
        self.lbl_estado.pack(fill="x", pady=(0, 8))

        self.lbl_park = tk.Label(panel, text="Estacionamiento: —", fg=FG, bg=BG, wraplength=self.PANEL_W-30, justify="left")
        self.lbl_park.pack(fill="x", pady=(0, 10))

        tk.Label(panel, text="RAW (lo que llega por serie):", fg=FG, bg=BG).pack(anchor="w")
        self.txt_raw = tk.Text(panel, height=9, bg="#15161c", fg="#cfd3ff", insertbackground=FG, relief="flat")
        self.txt_raw.pack(fill="both", expand=True, pady=(4, 0))
        self.txt_raw.configure(state="disabled")

        style = ttk.Style()
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("TButton", font=("Segoe UI", 10))
        style.configure("TCombobox", font=("Segoe UI", 10))

        self.refresh_ports()

    def _on_resize(self, event):
        # Keep panel width stable and update wraplength
        try:
            self.lbl_park.config(wraplength=max(240, self.PANEL_W - 30))
        except:
            pass

    def refresh_ports(self):
        ports = self.worker.available_ports()
        print("Refrescando COM:", ports)

        self.cb_ports["values"] = ports

        if ports:
            self.cb_ports.current(0)
        else:
            self.cb_ports.set("")

    def connect_port(self):
        port = self.cb_ports.get().strip()
        if not port:
            messagebox.showinfo("Conectar", "Elegí un COM. Cerrá el monitor serie si está abierto.")
            return

        try:
            baud = int(self.cb_baud.get())
        except:
            baud = 9600
            self.cb_baud.set("9600")

        ok = self.worker.connect(port, baud)
        if ok:
            self.lbl_conn.config(text=f"Estado: CONECTADO a {port} @ {baud}", fg=OK)
        else:
            self.lbl_conn.config(text=f"Estado: error al conectar {port}. Modo MOCK.", fg=BAD)

    def disconnect_port(self):
        self.worker.disconnect()
        self.lbl_conn.config(text="Estado: MOCK (sin puerto)", fg=ACCENT)

    def send_cmd(self, cmd: str):
        self.worker.send(cmd)
        self.mode_parking = (cmd == "PARK")

    def _poll_events(self):
        try:
            while True:
                evt = self.worker.events.get_nowait()
                if evt and evt[0] == "data":
                    _, cm_dict, est, raw = evt
                    self.cm.update(cm_dict)
                    self.estado = est
                    self._append_raw(raw)
        except queue.Empty:
            pass
        self.after(30, self._poll_events)

    def _append_raw(self, line: str):
        self.txt_raw.configure(state="normal")
        self.txt_raw.insert("end", line + "\n")

        content = self.txt_raw.get("1.0", "end")
        if len(content) > 1400:
            self.txt_raw.delete("1.0", "end")
            self.txt_raw.insert("end", content[-1400:])

        self.txt_raw.see("end")
        self.txt_raw.configure(state="disabled")

    def _tick(self):
        try:
            self.draw_scene()
        except Exception:
            pass
        self.after(REFRESH_MS, self._tick)

    def draw_scene(self):
        self.canvas.delete("all")

        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        if cw < 50 or ch < 50:
            return

        f = self.cm.get("F", MAX_CM)
        r = self.cm.get("R", MAX_CM)
        b = self.cm.get("B", MAX_CM)
        l = self.cm.get("L", MAX_CM)

        self.lbl_read.config(text=f"F:{int(f):3d}  R:{int(r):3d}  B:{int(b):3d}  L:{int(l):3d}")
        self.lbl_estado.config(text=f"Estado (firmware): {self.estado if self.estado is not None else '—'}")

        pr = evaluate_parking(l, r)
        self.lbl_park.config(text=f"Estacionamiento: {'APTO' if pr.suitable else 'NO APTO'} | gap≈{int(pr.gap_cm)}cm · {pr.reason}")

        # grid
        g = 50
        for x in range(0, cw + 1, g):
            self.canvas.create_line(x, 0, x, ch, fill="#2b2d39")
        for y in range(0, ch + 1, g):
            self.canvas.create_line(0, y, cw, y, fill="#2b2d39")

        # car centered on canvas
        car_x = cw * 0.5
        car_y = ch * 0.6
        Lpx, Wpx = CAR_LEN, CAR_WID

        local = [( Lpx/2,  Wpx/2), ( Lpx/2, -Wpx/2), (-Lpx/2, -Wpx/2), (-Lpx/2,  Wpx/2)]
        pts = [world_from_car(p, (car_x, car_y), self.car_ang) for p in local]
        flat = [c for p in pts for c in p]
        self.canvas.create_polygon(*flat, outline="#d2d2d2", width=2, fill="")

        hx = car_x + math.cos(self.car_ang) * (Lpx/2)
        hy = car_y + math.sin(self.car_ang) * (Lpx/2)
        self.canvas.create_line(car_x, car_y, hx, hy, fill="#d2d2d2", width=2)

        # sensor rays
        for sd in SENSORS:
            origin = world_from_car(sd["offset"], (car_x, car_y), self.car_ang)
            ang = self.car_ang + sd["angle"]
            dirv = (math.cos(ang), math.sin(ang))

            cm = clamp(self.cm.get(sd["key"], MAX_CM), 0, MAX_CM)
            dist_px = cm * PX_PER_CM
            hit = (origin[0] + dirv[0]*dist_px, origin[1] + dirv[1]*dist_px)

            if cm < MAX_CM * 0.25:
                col = BAD
            elif cm < MAX_CM * 0.6:
                col = WARN
            else:
                col = OK

            self.canvas.create_line(origin[0], origin[1], hit[0], hit[1], fill=col, width=3)
            self.canvas.create_oval(hit[0]-3, hit[1]-3, hit[0]+3, hit[1]+3, outline="#ffffff", width=2)
            self.canvas.create_oval(origin[0]-4, origin[1]-4, origin[0]+4, origin[1]+4, outline=col, width=2)

def main():
    App().mainloop()

if __name__ == "__main__":
    main()