import socket
import select
import pickle

# some globals
HEADER_LENGTH = 10
listening = False


def receive_message(client_socket):
    try:
        message_header = client_socket.recv(HEADER_LENGTH)

        if not len(message_header):
            return False

        message_length = int(message_header.decode('utf-8').strip())
        return {'header': message_header, 'data': client_socket.recv(message_length)}

    except:
        return False


def send_message(client_socket, message_tuple):
    message = pickle.dumps(message_tuple)
    message_header = f"{len(message):<{HEADER_LENGTH}}".encode('utf-8')
    try:
        client_socket.sendall(message_header + message)
    except Exception as ex:
        print(__name__, "send_message caught exception", ex)


def stop_listening():
    global listening
    listening = False


def bind_and_listen(ip, port, connect_callback, disconnect_callback, message_callback):
    global listening
    # Create a socket socket.AF_INET - address family, IPv4, some otehr possible are AF_INET6, AF_BLUETOOTH,
    # AF_UNIX socket.SOCK_STREAM - TCP, conection-based, socket.SOCK_DGRAM - UDP, connectionless, datagrams,
    # socket.SOCK_RAW - raw IP packets
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # SO_ - socket option
    # SOL_ - socket option level
    # Sets REUSEADDR (as a socket option) to 1 on socket
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Bind, so server informs operating system that it's going to use given IP and port For a server using 0.0.0.0
    # means to listen on all available interfaces, useful to connect locally to 127.0.0.1 and remotely to LAN
    # interface IP
    server_socket.bind((ip, port))

    # This makes server listen to new connections
    server_socket.listen()

    # List of sockets for select.select()
    sockets_list = [server_socket]

    clients = {}

    print(__name__, f'Listening for connections on {ip}:{port}...')
    listening = True

    while listening:
        # Calls Unix select() system call or Windows select() WinSock call with three parameters:
        #   - rlist - sockets to be monitored for incoming data
        #   - wlist - sockets for data to be send to (checks if for example buffers are not full and socket is ready to
        #             send some data)
        #   - xlist - sockets to be monitored for exceptions (we want to monitor all sockets for errors, so we can use
        #             rlist)
        # Returns lists:
        #   - reading - sockets we received some data on (that way we don't have to check sockets manually)
        #   - writing - sockets ready for data to be send thru them
        #   - errors  - sockets with some exceptions
        # This is a blocking call, code execution will "wait" here and "get" notified in case any action should be taken
        read_sockets, _, exception_sockets = select.select(sockets_list, [], sockets_list)

        # Iterate over notified sockets
        for notified_socket in read_sockets:

            if notified_socket == server_socket:

                client_socket, client_address = server_socket.accept()

                # Add accepted socket to select.select() list
                sockets_list.append(client_socket)

                user = '{}:{}'.format(*client_address)

                clients[client_socket] = user
                print(__name__, 'Accepted new connection from ', user)

                if connect_callback:
                    connect_callback(client_socket)

            # Else existing socket is sending a message
            else:
                # Receive message
                message = receive_message(notified_socket)

                # If False, client disconnected, cleanup
                if message is False:
                    if disconnect_callback:
                        disconnect_callback(notified_socket, -1)

                    print(__name__, 'Closed connection from: ', clients[notified_socket])
                else:
                    if message_callback:
                        message_callback(notified_socket, message)

                if message is False or pickle.loads(message['data'])[0] == 'disconnect':
                    print(__name__, 'Closing socket')
                    sockets_list.remove(notified_socket)
                    del clients[notified_socket]
                    notified_socket.close()
                    print(__name__, 'Socket closed')

        # It's not really necessary to have this, but will handle some socket exceptions just in case
        for notified_socket in exception_sockets:
            print(__name__, "Disconnecting a client")
            if disconnect_callback:
                disconnect_callback(notified_socket, 0)
