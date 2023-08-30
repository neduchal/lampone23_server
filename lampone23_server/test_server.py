import socket

UDP_IP = "192.168.100.22"
UDP_PORT = 9999
MESSAGE = b"FFFF"

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
msg = sock.recvfrom(2048)

print(msg[0])