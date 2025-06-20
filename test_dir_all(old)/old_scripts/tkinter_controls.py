import sys
import tkinter as tk

def keyboard_keydown(e):    # Keyboard commands
    if   e.keysym == 'space':
        #server_socket.sendto(b'stop', jetson_adress)
        print(">> Stop")
    elif e.keysym == 'Escape':
        print("K-- Keyboard closed")
        sys.exit()

def on_button_click(btn_name):  # Button click event handler

    if   btn_name == "btn_rtl":
        #server_socket.sendto(b'rtl', jetson_adress)
        print(">> Set RTL")     #set_return(the_connection)
    elif btn_name == "btn_land":
        #server_socket.sendto(b'land', jetson_adress)
        print(">> Set LAND") #mode_send(the_connection, 9)
    elif btn_name == "btn_loiter":
        #server_socket.sendto(b'loiter', jetson_adress)
        print(">> Set LOITER") #mode_send(the_connection, ?)
    elif btn_name == "btn_guided":
        #server_socket.sendto(b'guided', jetson_adress)
        print(">> Set GUIDED") #mode_send(the_connection, 4)
    elif btn_name == "btn_arm":
        #server_socket.sendto(b'arm', jetson_adress)
        print(">> Try ARMING")  #arm(the_connection)
    elif btn_name == "btn_takeoff":
        #server_socket.sendto(b'takeoff', jetson_adress)
        print("Try to takeoff...")
    elif btn_name == "btn_ping":
        #server_socket.sendto(b'ping', jetson_adress)
        print(">> Sent PING")
    elif btn_name == "btn_fence":
        #server_socket.sendto(b'fence', jetson_adress)
        print(">> Set FENCE")

    elif btn_name == "btn_nmea":
        #forward_nmea.value = 1 - forward_nmea.value
        print(f"Toggle NMEA forwarding")# to {forward_nmea.value}")
    elif btn_name == "btn_distp":
        #server_socket.sendto(b'lin+', jetson_adress)
        print(">> DIST+")
    elif btn_name == "btn_distm":
        #server_socket.sendto(b'lin-', jetson_adress)
        print(">> DIST-")
    elif btn_name == "btn_showpos":
        #server_socket.sendto(b'poss', jetson_adress)
        print(">> DISP POS")
    elif btn_name == "btn_repos":
        #server_socket.sendto(b'posr', jetson_adress)
        print(">> REPOS")
    elif btn_name == "btn_point1":
        #server_socket.sendto(b'p1', jetson_adress)
        print("goto point 1")
    elif btn_name == "btn_point2":
        #server_socket.sendto(b'p2', jetson_adress)
        print("goto point 2")
    elif btn_name == "btn_point3":
        #server_socket.sendto(b'p3', jetson_adress)
        print("goto point 3")
    elif btn_name == "btn_point4":
        #server_socket.sendto(b'p4', jetson_adress)
        print("goto point 4")


def toggle_matrix():
    """Show or hide the additional button matrix."""
    global more_buttons_visible
    if more_buttons_visible:
        # Hide the extra buttons
        for row in more_buttons:
            for button in row:
                button.grid_remove()  # Hide each individual button

        btn_more.config(text="more")  # Update the button text
        more_buttons_visible = False
    else:
        # Show the extra buttons
        for row_index, row in enumerate(more_buttons):
            for col_index, button in enumerate(row):
                button.grid(row=4 + row_index, column=col_index, padx=10, pady=10, sticky="ew")
        btn_more.config(text="hide")  # Update the button text
        more_buttons_visible = True

button_matrix = [   # Button matrix layout
    ["btn_rtl", "btn_land", "btn_loiter"],
    ["btn_guided", "btn_arm", "btn_takeoff"],
    ["btn_ping", "btn_fence", "btn_more"],
]

button_matrix_2 = [ # Button matrix layout, more buttons
    ["btn_nmea", "btn_distp", "btn_distm"],
    ["btn_showpos", "btn_point1", "btn_point2"],
    ["btn_repos", "btn_point3", "btn_point4"],
]

button_text = {     # Actual display text for the buttons
    "btn_arm": "ARM",
    "btn_rtl": "RTL",
    "btn_land": "LAND",
    "btn_loiter": "LOITER",
    "btn_guided": "GUIDED",
    "btn_takeoff": "TAKEOFF",
    "btn_ping": "PING",
    "btn_fence": " FENCE ",
    "btn_more": "more",
    "btn_nmea": "NMEA",
    "btn_distp": "DIST+",
    "btn_distm": "DIST-",
    "btn_point1": "POINT 1",
    "btn_point2": "POINT 2",
    "btn_point3": "POINT 3",
    "btn_point4": "POINT 4",
    "btn_showpos": "DISP P",
    "btn_repos": "REPOS",
}


root = tk.Tk()
root.title("Button matrix")

# Button initialization
for row_index, row in enumerate(button_matrix):
    for col_index, button_name in enumerate(row):

        display_text = button_text.get(button_name, button_name)

        if button_name == "btn_more":
            btn_more = tk.Button(root, text=display_text, 
                                 command=toggle_matrix)
            btn_more.grid(row=row_index, column=col_index, padx=10, pady=10, sticky="ew")

        else:
            button = tk.Button(root, text=display_text, 
                command=lambda name=button_name: on_button_click(name))
            button.grid(row=row_index, column=col_index, padx=10, pady=10, sticky="ew")

# More buttons initialization
more_buttons = []
for row_index, row in enumerate(button_matrix_2):

    button_row = []
    for col_index, button_name in enumerate(row):
        display_text = button_text.get(button_name, button_name)
    
        button = tk.Button(root, text=display_text, 
                           command=lambda name=button_name: on_button_click(name))
        button_row.append(button)
    more_buttons.append(button_row)   
more_buttons_visible = False


root.bind("<KeyPress>", lambda e: keyboard_keydown(e))

root.mainloop()