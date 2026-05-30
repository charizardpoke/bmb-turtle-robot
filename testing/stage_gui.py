# RUNNING COMMANDS - LAPTOP:
# python3 stage_gui.py

import tkinter as tk
import subprocess


SCRIPT_PATH = "./auto_script.sh"


class StageGUI:
    def __init__(self):
        self.process = None

        self.root = tk.Tk()
        self.root.title("BMB Turtle Auto Script GUI")
        self.root.geometry("350x260")

        tk.Label(
            self.root,
            text="BMB Turtle Auto Control",
            font=("Arial", 16)
        ).pack(pady=15)

        self.status_label = tk.Label(
            self.root,
            text="Status: Not loaded",
            font=("Arial", 11)
        )
        self.status_label.pack(pady=10)

        tk.Button(
            self.root,
            text="Load auto_script.sh",
            width=22,
            height=2,
            command=self.load_script
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="Run All",
            width=22,
            height=2,
            bg="green",
            fg="white",
            command=self.run_all
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="STOP",
            width=22,
            height=2,
            bg="red",
            fg="white",
            command=self.stop_robot
        ).pack(pady=15)

    def load_script(self):
        command = f"source {SCRIPT_PATH} && echo loaded"

        result = subprocess.run(
            ["bash", "-lc", command],
            capture_output=True,
            text=True
        )

        if result.returncode == 0:
            self.status_label.config(text="Status: auto_script.sh loaded")
        else:
            self.status_label.config(text="Status: Load failed")
            print(result.stderr)

    def run_all(self):
        self.status_label.config(text="Status: Running all stages")

        command = f"source {SCRIPT_PATH} && run_all"

        self.process = subprocess.Popen(
            ["bash", "-lc", command]
        )

    def stop_robot(self):
        self.status_label.config(text="Status: STOP sent")

        command = f"source {SCRIPT_PATH} && stop_now"

        subprocess.Popen(
            ["bash", "-lc", command]
        )

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    gui = StageGUI()
    gui.run()