import socket
import tkinter as tk
from tkinter import messagebox

# Set up the "serial" server (this simulates a serial port)
def start_serial_server():
    # Localhost and a free port
    host = '192.168.0.145'
    port = 3333
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"Serial server started on {host}:{port}")
    
    conn, addr = server_socket.accept()
    print(f"Connection from {addr}")
    
    return conn

# This will run on the client side (Tkinter GUI)
class SerialPortEmulator:
    def __init__(self, master, conn):
        self.master = master
        self.conn = conn
        
        self.master.title("Serial Port Communication")
        
        self.label = tk.Label(master, text="Type to send characters to serial port:")
        self.label.pack()
        
        # Entry widget to type into
        self.entry = tk.Entry(master, width=40)
        self.entry.pack()
        self.entry.focus()
        
        # Bind the key press event to a handler function
        self.entry.bind('<KeyPress>', self.on_key_press)
        
        # Button to quit
        self.quit_button = tk.Button(master, text="Quit", command=self.master.quit)
        self.quit_button.pack()
        
    def on_key_press(self, event):
        char = event.char  # The character typed
        if char:  # If a valid character is typed
            try:
                # Send the character to the "serial" connection
                self.conn.send(char.encode())
                print(f"Sent: {char}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to send character: {e}")

# Main application function
def main():
    # Create a socket connection (for simulating serial)
    conn = start_serial_server()
    
    # Tkinter GUI setup
    root = tk.Tk()
    app = SerialPortEmulator(root, conn)
    
    # Start the Tkinter main loop
    root.mainloop()

if __name__ == "__main__":
    main()
