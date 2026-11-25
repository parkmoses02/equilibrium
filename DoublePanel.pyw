"""Windows launcher (no terminal)."""
import tkinter as tk
from tkinter import messagebox

if __name__ == "__main__":
    root = tk.Tk()
    try:
        from panel.double_console import Console
        Console(root)
    except ImportError as exc:
        root.withdraw()
        messagebox.showerror("Equilibrium", f"{exc}\nPython 환경에 pyserial을 설치하세요: python -m pip install pyserial")
        root.destroy()
    else:
        root.mainloop()
