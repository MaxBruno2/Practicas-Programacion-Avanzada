import socket
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Int32, Bool


SERVER_IP = "3.149.222.108"
SERVER_PORT = 4062
BUFFER_SIZE = 1024


class TcpToRos(Node):

    def __init__(self):
        super().__init__('tcp_to_ros')

        # Publishers
        self.pub_pot = self.create_publisher(Int32, 'mixer/pot', 10)
        self.pub_eje_x = self.create_publisher(Int32, 'mixer/eje_x', 10)
        self.pub_eje_y = self.create_publisher(Int32, 'mixer/eje_y', 10)

        self.pub_boton = self.create_publisher(Bool, 'mixer/boton', 10)
        self.pub_atras = self.create_publisher(Bool, 'mixer/atras', 10)
        self.pub_pausa = self.create_publisher(Bool, 'mixer/pausa', 10)
        self.pub_adelante = self.create_publisher(Bool, 'mixer/adelante', 10)

        self.socket = None
        self.connect_to_server()

        # Timer para leer del socket sin bloquear ROS
        self.create_timer(0.01, self.read_socket)

    def connect_to_server(self):
        while rclpy.ok():
            try:
                self.get_logger().info('Conectando al servidor TCP...')
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((SERVER_IP, SERVER_PORT))
                self.socket.sendall(b'<name>ROS2\n')
                self.socket.setblocking(False)
                self.get_logger().info('Conectado al servidor TCP')
                return
            except Exception as e:
                self.get_logger().error(f'Error TCP: {e}')
                self.destroy_socket()
                time.sleep(1.0)

    def destroy_socket(self):
        if self.socket:
            try:
                self.socket.close()
            except Exception:
                pass
            self.socket = None

    def read_socket(self):
        if not self.socket:
            self.connect_to_server()
            return

        try:
            data = self.socket.recv(BUFFER_SIZE).decode('utf-8')
            if not data:
                raise ConnectionError('Servidor desconectado')

            for line in data.splitlines():
                self.handle_message(line.strip())

        except BlockingIOError:
            pass  # no hay datos
        except Exception as e:
            self.get_logger().warn(f'Desconexión TCP: {e}')
            self.destroy_socket()

    def handle_message(self, msg: str):
        try:
            if msg.startswith('<pot>'):
                self.pub_pot.publish(Int32(data=int(msg[5:])))

            elif msg.startswith('<ejeX>'):
                self.pub_eje_x.publish(Int32(data=int(msg[6:])))

            elif msg.startswith('<ejeY>'):
                self.pub_eje_y.publish(Int32(data=int(msg[6:])))

            elif msg.startswith('<boton>'):
                self.pub_boton.publish(Bool(data=bool(int(msg[7:]))))

            elif msg.startswith('<atras>'):
                self.pub_atras.publish(Bool(data=bool(int(msg[7:]))))

            elif msg.startswith('<pausa>'):
                self.pub_pausa.publish(Bool(data=bool(int(msg[7:]))))

            elif msg.startswith('<adelante>'):
                self.pub_adelante.publish(Bool(data=bool(int(msg[10:]))))

        except Exception as e:
            self.get_logger().warn(f'Mensaje inválido: {msg} ({e})')


def main():
    rclpy.init()
    node = TcpToRos()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
