# RUNNING COMMANDS - LAPTOP:
# python3 stage_gui.py

import tkinter as tk
import subprocess


SCRIPT_PATH = "./auto_script.sh"


class StageGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Wall-E Auto Script GUI")
        self.root.geometry("350x250")

        tk.Label(
            self.root,
            text="Auto Script Control",
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
            command=lambda: self.run_command("run_all")
        ).pack(pady=5)

        tk.Button(
            self.root,
            text="STOP",
            width=22,
            height=2,
            bg="red",
            fg="white",
            command=lambda: self.run_command("stop_now")
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

    def run_command(self, command_name):
        self.status_label.config(text=f"Status: Running {command_name}")

        command = f"source {SCRIPT_PATH} && {command_name}"

        try:
            subprocess.Popen(["bash", "-lc", command])
        except Exception as error:
            self.status_label.config(text=f"Status: Error running {command_name}")
            print(error)

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    gui = StageGUI()
    gui.run()