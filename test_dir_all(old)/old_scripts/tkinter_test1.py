
#import tkinter as tk

# def on_button_click(row, col):
#     print(f"Button at Row {row}, Column {col} clicked!")

# def update_text():
#     # Update the text in the label with the text from the entry box
#     new_text = text_entry.get()
#     label.config(text=new_text)

# # Keyboard commands to send to the drone
# def keyboard_keydown(e):

#     if e.keysym == 'space':
#         print(">> Stop")
#     else:
#         print(f"{e.keysym}")

# # Create the main window
# root = tk.Tk()
# root.title("RoboCamp Interface extension")

# # Define the number of rows and columns
# rows = 2
# cols = 3
# # Create and place buttons in a 3x2 grid
# for row in range(rows):
#     for col in range(cols):
#         # Create a button
#         if (row == 1 and col == 2):
#             button = tk.Button(root, text=f"Update", command=update_text)

#         else:
#             button = tk.Button(root, text=f"Button {row+1},{col+1}",
#                 command=lambda r=row, c=col: on_button_click(r, c))
#         button.grid(row=row, column=col, padx=10, pady=10, sticky="ew")

# # Create a label to display text
# label = tk.Label(root, text="Default Text", font=("Arial", 16))
# label.grid(row=rows, column=0, columnspan=cols, pady=10)
# #label.pack(pady=10)

# # Create an entry box for user input
# text_entry = tk.Entry(root, font=("Arial", 14))
# text_entry.grid(row=rows+1, column=0, columnspan=cols, pady=10)
# #text_entry.pack(pady=10)

# root.bind("<KeyPress>", lambda e: keyboard_keydown(e))


# # Start the Tkinter event loop
# root.mainloop()


# import tkinter as tk

# root = tk.Tk()
# root.title("Grid Options Demo")

# # Label spanning multiple columns
# label = tk.Label(root, text="This is a label spanning 3 columns", bg="lightblue",)
# label.grid(row=0, column=0, columnspan=3, pady=10)

# # Buttons with various grid options
# button1 = tk.Button(root, text="Button 1")
# button1.grid(row=1, column=0, padx=5, pady=5, sticky="w")

# button2 = tk.Button(root, text="Button 2")
# button2.grid(row=1, column=1, padx=5, pady=5, ipadx=10, ipady=10)  # Larger internal padding

# button3 = tk.Button(root, text="Button 3")
# button3.grid(row=1, column=2, padx=5, pady=5, sticky="e")

# # Textbox spanning two rows
# entry = tk.Entry(root)
# entry.grid(row=2, column=0, rowspan=2, columnspan=3, sticky="nsew", padx=5, pady=5)

# # Configure column/row weights to stretch widgets
# root.grid_columnconfigure(0, weight=1)  # Column 0 will expand
# root.grid_columnconfigure(1, weight=1)  # Column 1 will expand
# root.grid_columnconfigure(2, weight=1)  # Column 2 will expand
# root.grid_rowconfigure(2, weight=1)     # Row 2 will expand

# root.mainloop()


# def on_click():
#     pass

# import tkinter as tk
# from multiprocessing import Queue, Value

# root = tk.Tk()
# root.title("Server setup Demo")

# default_server = tk.StringVar(value="192.168.0.145")
# default_drone = tk.StringVar(value="127.0.0.1")

# label_s = tk.Label(root, text="Server:",)
# label_s.grid(row=0, column=0, pady=10)

# entry_s = tk.Entry(root, textvariable=default_server, width=15, bg="lightgray")
# entry_s.grid(row=0, column=1, padx=5, pady=5)

# label_ss = tk.Label(root, bg="lightgray", width=2)
# label_ss.grid(row=0, column=2, padx=5, pady=5)

# button_s = tk.Button(root, text="START", width=5, command=on_click)
# button_s.grid(row=0, column=3, padx=5, pady=5)

# label_d = tk.Label(root, text="Drone: ",)
# label_d.grid(row=1, column=0, pady=10)

# entry_d = tk.Entry(root, textvariable=default_drone, width=15, bg="lightgray")
# entry_d.grid(row=1, column=1, padx=5, pady=5)

# label_dd = tk.Label(root, bg="lightgray", width=2)
# label_dd.grid(row=1, column=2, padx=5, pady=5)

# button_d = tk.Button(root, text="PING", width=5, command=lambda: print("PING"))
# button_d.grid(row=1, column=3, padx=5, pady=5, sticky="w")

# root.mainloop()



# import tkinter as tk

# # Define multi-page button names
# extra_button_names = [
#     [  # Page 1
#         ["A", "B", "C"],
#         ["D", "E", "F"],
#         ["G", "H", "I"],
#     ],
#     [  # Page 2
#         ["1", "2"],
#         ["3", "4"],
#     ],
# ]

# class App:
#     def __init__(self, root):
#         self.root = root
#         self.root.title("Multi-Page Button Matrix")
        
#         # Initialize variables
#         self.current_page = 0
#         self.extra_buttons = []  # Store button references
        
#         # Create frames
#         self.button_frame = tk.Frame(root)
#         self.button_frame.pack(pady=20)
        
#         self.nav_frame = tk.Frame(root)
#         self.nav_frame.pack(pady=10)
        
#         # Navigation buttons
#         self.prev_button = tk.Button(self.nav_frame, text="Previous", command=self.prev_page)
#         self.prev_button.grid(row=0, column=0, padx=5)
        
#         self.next_button = tk.Button(self.nav_frame, text="Next", command=self.next_page)
#         self.next_button.grid(row=0, column=1, padx=5)
        
#         # Render the initial page
#         self.render_page()
    
#     def render_page(self):
#         """Render buttons for the current page."""
#         # Clear existing buttons
#         for btn in self.extra_buttons:
#             btn.destroy()
#         self.extra_buttons.clear()
        
#         # Create new buttons for the current page
#         page = extra_button_names[self.current_page]
#         for row_index, row in enumerate(page):
#             for col_index, button_name in enumerate(row):
#                 btn = tk.Button(self.button_frame, text=button_name, font=("Arial", 14))
#                 btn.grid(row=row_index, column=col_index, padx=5, pady=5)
#                 self.extra_buttons.append(btn)
    
#     def prev_page(self):
#         """Go to the previous page."""
#         if self.current_page > 0:
#             self.current_page -= 1
#             self.render_page()
    
#     def next_page(self):
#         """Go to the next page."""
#         if self.current_page < len(extra_button_names) - 1:
#             self.current_page += 1
#             self.render_page()

# # Main application
# if __name__ == "__main__":
#     root = tk.Tk()
#     app = App(root)
#     root.mainloop()


import tkinter as tk

def on_enter(event, help_text):
    """Update the help label with text for the hovered button."""
    help_label.config(text=help_text)

def on_leave(event):
    """Clear the help label when the mouse leaves the button."""
    help_label.config(text="")

# Create main Tkinter window
root = tk.Tk()
root.title("Hover Help Example")

# Frame for buttons
button_frame = tk.Frame(root)
button_frame.pack(pady=20)

# Help label
help_label = tk.Label(root, text="", font=("Arial", 12), fg="blue")
help_label.pack(pady=10)

# Button info (button_name: help_text)
buttons_info = {
    "Button 1": "This is the first button.",
    "Button 2": "This is the second button.",
    "Button 3": "Click here for more options.",
}

# Create buttons and bind hover events
for button_name, help_text in buttons_info.items():
    button = tk.Button(button_frame, text=button_name, font=("Arial", 14))
    button.pack(side="left", padx=10)
    
    # Bind hover events
    button.bind("<Enter>", lambda event, text=help_text: on_enter(event, text))
    button.bind("<Leave>", on_leave)

# Run the main loop
root.mainloop()


    # server_window = tk.Toplevel(root)
    # server_window.title("Server status")

    # default_server = tk.StringVar(value=host)
    # default_drone = tk.StringVar(value=jetson_address[0])

    # label_s = tk.Label(server_window, text="Server:",)
    # label_s.grid(row=0, column=0, pady=10)
    # entry_s = tk.Entry(server_window, textvariable=default_server, width=14, bg="lightgray",
    #                    state="readonly")
    # entry_s.grid(row=0, column=1, padx=5, pady=5)

    # label_ss = tk.Label(server_window, bg="green", width=2)
    # label_ss.grid(row=0, column=2, padx=5, pady=5)
    # btn_server_toggle = tk.Button(server_window, text="STOP", width=8,
    #                               command=server_toggle,)
    # btn_server_toggle.grid(row=0, column=3, padx=5, pady=5, sticky="we")

    # label_d = tk.Label(server_window, text="Drone: ",)
    # label_d.grid(row=1, column=0, pady=10)
    # entry_d = tk.Entry(server_window, textvariable=default_drone, width=14, bg="lightgray",
    #                    state="readonly")
    # entry_d.grid(row=1, column=1, padx=5, pady=5)

    # label_dd = tk.Label(server_window, bg="lightgray", width=2)
    # label_dd.grid(row=1, column=2, padx=5, pady=5)
    # button_d = tk.Button(server_window, text="PING", width=8,
    #                     command=lambda: on_button_click("btn_ping"))
    # button_d.grid(row=1, column=3, padx=5, pady=5, sticky="ew")