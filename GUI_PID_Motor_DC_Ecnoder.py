import tkinter as tk
from tkinter import ttk, messagebox
import serial.tools.list_ports
import serial
from threading import Thread
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
import numpy as np
from collections import deque

class MotorControlGUI:
    def _init_(self, root):
        self.root = root
        self.root.title("Motor DC Control with Real-Time Graph")

        self.serial_connection = None
        self.is_running = False
        self.rpm_data = deque(maxlen=100)  # Buffer data RPM
        self.time_data = deque(maxlen=100)  # Buffer waktu
        self.setpoint_data = deque(maxlen=100)  # Buffer data setpoint
        self.start_time = None

        # Frame utama untuk tata letak kiri-kanan
        self.main_frame = tk.Frame(root)
        self.main_frame.pack(fill="both", expand=True)

        # Frame untuk pengaturan di sebelah kiri
        self.left_frame = tk.Frame(self.main_frame)
        self.left_frame.pack(side="left", fill="y", padx=10, pady=5)

        # Frame untuk koneksi serial
        self.connection_frame = tk.LabelFrame(self.left_frame, text="Serial Connection")
        self.connection_frame.pack(fill="x", padx=10, pady=5)

        self.com_port_label = tk.Label(self.connection_frame, text="COM Port:")
        self.com_port_label.pack(anchor="w", padx=5)
        self.com_port_combo = ttk.Combobox(self.connection_frame, values=self.get_available_ports())
        self.com_port_combo.pack(fill="x", padx=5)
        self.refresh_button = tk.Button(self.connection_frame, text="Refresh", command=self.refresh_ports)
        self.refresh_button.pack(fill="x", padx=5, pady=2)
        self.connect_button = tk.Button(self.connection_frame, text="Connect", command=self.connect_serial)
        self.connect_button.pack(fill="x", padx=5, pady=2)
        self.disconnect_button = tk.Button(self.connection_frame, text="Disconnect", command=self.disconnect_serial, state="disabled")
        self.disconnect_button.pack(fill="x", padx=5, pady=2)

        # Frame untuk pengaturan parameter
        self.param_frame = tk.LabelFrame(self.left_frame, text="Motor Parameters")
        self.param_frame.pack(fill="x", padx=10, pady=5)

        self.kp_label = tk.Label(self.param_frame, text="Kp:")
        self.kp_label.pack(anchor="w", padx=5, pady=2)
        self.kp_entry = tk.Entry(self.param_frame, width=10)
        self.kp_entry.pack(fill="x", padx=5, pady=2)

        self.ki_label = tk.Label(self.param_frame, text="Ki:")
        self.ki_label.pack(anchor="w", padx=5, pady=2)
        self.ki_entry = tk.Entry(self.param_frame, width=10)
        self.ki_entry.pack(fill="x", padx=5, pady=2)

        self.kd_label = tk.Label(self.param_frame, text="Kd:")
        self.kd_label.pack(anchor="w", padx=5, pady=2)
        self.kd_entry = tk.Entry(self.param_frame, width=10)
        self.kd_entry.pack(fill="x", padx=5, pady=2)

        self.setpoint_label = tk.Label(self.param_frame, text="Setpoint:")
        self.setpoint_label.pack(anchor="w", padx=5, pady=2)
        self.setpoint_entry = tk.Entry(self.param_frame, width=10)
        self.setpoint_entry.pack(fill="x", padx=5, pady=2)

        self.apply_button = tk.Button(self.param_frame, text="Apply", command=self.apply_parameters)
        self.apply_button.pack(fill="x", padx=5, pady=5)

        # Frame untuk kontrol arah motor
        self.control_frame = tk.LabelFrame(self.left_frame, text="Motor Control")
        self.control_frame.pack(fill="x", padx=10, pady=5)

        self.cw_button = tk.Button(self.control_frame, text="Searah Jarum Jam", command=self.set_cw)
        self.cw_button.pack(fill="x", padx=5, pady=2)

        self.ccw_button = tk.Button(self.control_frame, text="Berlawanan Jarum Jam", command=self.set_ccw)
        self.ccw_button.pack(fill="x", padx=5, pady=2)

        self.stop_button = tk.Button(self.control_frame, text="Stop Motor", command=self.stop_motor)
        self.stop_button.pack(fill="x", padx=5, pady=2)

        # Frame untuk grafik di sebelah kanan
        self.right_frame = tk.Frame(self.main_frame)
        self.right_frame.pack(side="right", fill="both", expand=True, padx=10, pady=5)

        # Grafik untuk RPM dan Setpoint
        self.fig, self.ax = plt.subplots()
        self.line_rpm, = self.ax.plot([], [], label="RPM", color="red")
        self.line_setpoint, = self.ax.plot([], [], label="Setpoint", color="blue", linestyle="--")
        self.ax.legend()
        self.ax.set_title("Step Response")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("RPM")

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.right_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=5)

        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=100)

        # Frame untuk menampilkan hasil analisis step response
        self.analysis_frame = tk.LabelFrame(self.right_frame, text="Step Response Analysis")
        self.analysis_frame.pack(fill="x", padx=10, pady=5)

        self.rise_time_label = tk.Label(self.analysis_frame, text="Rise Time: N/A")
        self.rise_time_label.pack(side="left", padx=5, pady=2)

        self.settling_time_label = tk.Label(self.analysis_frame, text="Settling Time: N/A")
        self.settling_time_label.pack(side="left", padx=5, pady=2)

        self.overshoot_label = tk.Label(self.analysis_frame, text="Overshoot: N/A")
        self.overshoot_label.pack(side="left", padx=5, pady=2)

        self.ss_error_label = tk.Label(self.analysis_frame, text="Steady-State Error: N/A")
        self.ss_error_label.pack(side="left", padx=5, pady=2)

    def get_available_ports(self):
        return [port.device for port in serial.tools.list_ports.comports()]

    def refresh_ports(self):
        self.com_port_combo["values"] = self.get_available_ports()

    def connect_serial(self):
        port = self.com_port_combo.get()
        if not port:
            messagebox.showerror("Error", "Please select a COM port!")
            return

        try:
            self.serial_connection = serial.Serial(port, baudrate=9600, timeout=1)
            self.is_running = True
            self.start_time = time.time()
            self.read_thread = Thread(target=self.read_serial)
            self.read_thread.start()
            self.connect_button["state"] = "disabled"
            self.disconnect_button["state"] = "normal"
        except Exception as e:
            messagebox.showerror("Error", f"Failed to connect: {e}")

    def disconnect_serial(self):
        self.is_running = False
        if self.serial_connection:
            self.serial_connection.close()
            self.serial_connection = None
        self.connect_button["state"] = "normal"
        self.disconnect_button["state"] = "disabled"

    def apply_parameters(self):
        if not self.serial_connection:
            messagebox.showerror("Error", "Serial connection is not established!")
            return

        try:
            kp = float(self.kp_entry.get())
            ki = float(self.ki_entry.get())
            kd = float(self.kd_entry.get())
            setpoint = float(self.setpoint_entry.get())

            self.serial_connection.write(f"Kp:{kp}\n".encode())
            self.serial_connection.write(f"Ki:{ki}\n".encode())
            self.serial_connection.write(f"Kd:{kd}\n".encode())
            self.serial_connection.write(f"SETPOINT:{setpoint}\n".encode())

            self.current_setpoint = setpoint
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numbers!")

    def set_cw(self):
        if self.serial_connection:
            self.serial_connection.write(b"CW\n")

    def set_ccw(self):
        if self.serial_connection:
            self.serial_connection.write(b"CCW\n")

    def stop_motor(self):
        if self.serial_connection:
            self.serial_connection.write(b"STOP\n")

    def read_serial(self):
        while self.is_running:
            if self.serial_connection and self.serial_connection.in_waiting > 0:
                try:
                    line = self.serial_connection.readline().decode().strip()
                    if line.startswith("SETPOINT:"):
                        setpoint, rpm = map(float, line.replace("SETPOINT:", "").split(", RPM:"))
                        self.rpm_data.append(rpm)
                        self.setpoint_data.append(setpoint)
                        self.time_data.append(time.time() - self.start_time)
                except Exception as e:
                    print(f"Error reading serial: {e}")

    def update_plot(self, frame):
        self.line_rpm.set_data(self.time_data, self.rpm_data)
        self.line_setpoint.set_data(self.time_data, self.setpoint_data)

        self.ax.relim()
        self.ax.autoscale_view()

        self.calculate_step_response_metrics()

    def calculate_step_response_metrics(self):
        if len(self.rpm_data) < 2:
            return

        # Menghitung waktu naik
        if len(self.time_data) > 0 and self.rpm_data[0] < self.setpoint_data[0]:
            rise_time = self.time_data[-1] - self.time_data[0]
            self.rise_time_label.config(text=f"Rise Time: {rise_time:.2f}s")
        else:
            self.rise_time_label.config(text="Rise Time: N/A")

        # Settling time and overshoot calculations
        final_value = self.setpoint_data[-1] if self.setpoint_data else 0
        max_value = max(self.rpm_data)
        overshoot = ((max_value - final_value) / final_value) * 100 if final_value != 0 else 0
        self.overshoot_label.config(text=f"Overshoot: {overshoot:.2f}%")

        if len(self.rpm_data) > 1:
            settling_time = next(
                (t for t, v in zip(self.time_data, self.rpm_data) if abs(v - final_value) < 0.02 * final_value),
                "N/A"
            )
            self.settling_time_label.config(text=f"Settling Time: {settling_time if settling_time != 'N/A' else 'N/A'}")

        ss_error = abs(final_value - self.rpm_data[-1])
        self.ss_error_label.config(text=f"Steady-State Error: {ss_error:.2f}")

if _name_ == "_main_":
    root = tk.Tk()
    app = MotorControlGUI(root)
    root.mainloop()