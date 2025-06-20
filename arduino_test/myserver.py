import socket
import re
import sys

#UDP_IP = "192.168.0.102"
#UDP_PORT = 3333

# Read the content of the socat output log file 
with open('/tmp/socat_output.log', 'r') as file:
    log_content = file.read()

# Find all matches of the pattern 'PTY is /dev/pts/number'
matches = re.findall(r'PTY is (/dev/pts/\d+)', log_content)

# Get the first match
if matches:
    VIRTUAL_SERIAL_PORT = matches[0]
    print(f"Found open pts for writing: {matches[0]}")
else:
    print("No open socat pts found, aborting...")
    sys.exit()

#VIRTUAL_SERIAL_PORT = "/dev/pts/2"     # not reliable, pts changes per process

# fake_points = [ b"$GNGGA,091950.25,4547.7617991,N,01558.4370742,E,2,12,0.66,123.341,M,41.375,M,,0000*4F",
#                 b"$GNGGA,091946.75,4547.7618141,N,01558.4371248,E,2,12,0.67,123.252,M,41.375,M,,0000*4B",
#                 b"$GNGGA,091943.25,4547.7618385,N,01558.4372001,E,2,12,0.66,123.135,M,41.375,M,,0000*4E",
#                 b"$GNGGA,091939.75,4547.7618390,N,01558.4372229,E,2,12,0.66,123.009,M,41.375,M,,0000*44"]

# i = 0
# help_p = 0

# def help_follow_thread():
#     while True:
#         if help_p == 1:
#             i = i + 1
#             if i > 3:
#                 i = 0
            


#def start_udp_server(host='192.168.1.100', port=3333):
def start_udp_server(host='192.168.46.173', port=3333): # esp32:172  # Studenti
#def start_udp_server(host='192.168.0.108', port=3333): # esp32: # Astro
#def start_udp_server(host='192.168.43.221', port=3333): # esp32:98

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    server_socket.bind((host, port))
    print(f'UDP server listening on {host}:{port}')

    while True:

        data, client_adress = server_socket.recvfrom(1024)

        #print(data.decode('utf-8'))

        with open(VIRTUAL_SERIAL_PORT, 'wb') as f:
            #f.write(data.decode('utf-8'))
            print(data)
            #f.write(bytes(data))
            #f.write(b'\n')
            if (data == b'help\n'):
                print("Got help message: ", client_adress)
                server_socket.sendto(b'foll', client_adress)
            elif (data == b'face\n'):
                print("Face tracking request: ", client_adress)
                server_socket.sendto(b'face', client_adress)
            #     i = i + 1
            #     if i > 3:
            #         i = 0
            #     f.write(fake_points[i])
            # else:
            #     f.write(data)


if __name__ == '__main__':
    start_udp_server()

# #def start_udp_server(host='192.168.0.229', port=3333): # Jetson
# def start_udp_server(host='192.168.0.145', port=3333):  # sim/Putanec
#     server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#     server_socket.bind((host, port))
#     print(f'UDP server listening on {host}:{port}')

#     while True:
#         data, client_adress = server_socket.recvfrom(1024)

#         print(data)

#         if(data == b'search\n'):
#             print("Face search request: ", client_adress)
#             server_socket.sendto(b'search1', client_adress)

#         elif (data == b'track\n'):
#             print("Face tracking request: ", client_adress)
#             server_socket.sendto(b'track1', client_adress)

#         elif (data == b'gps\n'):
#             print("GPS location request: ", client_adress)
#             server_socket.sendto(b'gps1', client_adress)

#         elif (data == b'off\n'):
#             print("OFF request")