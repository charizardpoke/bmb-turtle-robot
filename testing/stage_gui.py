# RUNNING COMMANDS:
# cd ~/ros2_ws/src/testing
# python3 stage_gui.py

import tkinter as tk
import subprocess


SCRIPT_PATH = "./auto_script.sh"


class StageGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("BMB Turtle Auto Script GUI")
        self.root.geometry("350x400")

        tk.Label(
            self.root,
            text="Auto Script Stage Control",
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
            text="Stage 1",
            width=22,
            height=2,
            command=lambda: self.run_stage("stage_1")
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="Stage 2",
            width=22,
            height=2,
            command=lambda: self.run_stage("stage_2")
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="Stage 3",
            width=22,
            height=2,
            command=lambda: self.run_stage("stage_3")
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="Stage 4",
            width=22,
            height=2,
            command=lambda: self.run_stage("stage_4")
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="Stage 5",
            width=22,
            height=2,
            command=lambda: self.run_stage("stage_5")
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="STOP",
            width=22,
            height=2,
            bg="red",
            fg="white",
            command=lambda: self.run_stage("stop_now")
        ).pack(pady=15)

    def load_script(self):
        command = f"source {SCRIPT_PATH} && echo loaded"

        try:
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

        except Exception as error:
            self.status_label.config(text="Status: Error loading script")
            print(error)

    def run_stage(self, stage_name):
        self.status_label.config(text=f"Status: Running {stage_name}")

        command = f"source {SCRIPT_PATH} && {stage_name}"

        try:
            subprocess.Popen(["bash", "-lc", command])
        except Exception as error:
            self.status_label.config(text=f"Status: Error running {stage_name}")
            print(error)

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    gui = StageGUI()
    gui.run()