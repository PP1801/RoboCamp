import socket

def start_udp_server(host='192.168.0.103', port=3333):
#def start_udp_server(host='192.168.43.221', port=3333):
    # Create a socket object
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Bind the socket to the address and port
    server_socket.bind((host, port))
    print(f'UDP server listening on {host}:{port}')
    
    while True:
        # Receive data from the client
        data, client_address = server_socket.recvfrom(1024)  # Buffer size is 1024 bytes
        #print(f'Connected by {client_address}')
        
        # Print the received data
        #print('Received data:', data.decode('utf-8'))

        print(data.decode('utf-8'))

if __name__ == '__main__':
    start_udp_server()
