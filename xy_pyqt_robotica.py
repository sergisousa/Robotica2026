import os
import socket
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLineEdit
from PyQt5.QtCore import QObject, QTimer
import time
from queue import Queue, Empty
import threading

## ----------- versão deste script para robótica
##
##   [Rede UDP]
##       ↓  raw_packet_queue  (bytes)
##   [parser_worker_thread]   — acumula até lds_end, faz parse
##       ↓  gui_data_queue    (dicts) — buffer ilimitado, NADA se perde
##   [saver_dispatcher_thread] — grava TODOS os frames p/ disco (async)
##       ↓  render_queue      (maxsize=1) — sempre sobrescreve com o mais recente
##   [GUI / QTimer 10 ms]    — só lê o frame mais recente, sem fazer I/O
##

# ============================
# CONFIGURAÇÕES DE REDE
# ============================
UDP_IP          = "0.0.0.0"
UDP_PORT        = 0
# UDP_IP_ROBOT    = "192.168.1.80"   # <--- CONFIRMAR IP DO ROBOT
UDP_IP_ROBOT    = "192.168.0.100"   # <--- CONFIRMAR IP DO ROBOT
UDP_PORT_ROBOT  = 4224
MSG_START       = b"s2 b;"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)
sock.bind((UDP_IP, UDP_PORT))
my_ip, my_port = sock.getsockname()
print(f"Socket vinculado. A escutar na porta: {my_port}")

try:
    print(f"A enviar comando para {UDP_IP_ROBOT}:{UDP_PORT_ROBOT}...")
    sock.sendto(MSG_START, (UDP_IP_ROBOT, UDP_PORT_ROBOT))
    print("Comando enviado.")
except Exception as e:
    print(f"Erro ao enviar comando: {e}")

# ============================
# CLASSE PARA ENVIAR COMANDOS
# ============================
class UdpSender(QObject):
    def __init__(self, shared_socket, ip, port):
        super().__init__()
        self.addr = (ip, port)
        self.sock = shared_socket

    def send(self, msg):
        try:
            self.sock.sendto(msg.encode(), self.addr)
        except Exception as e:
            print(f"Erro ao enviar via socket partilhado: {e}")

udp_send = UdpSender(sock, UDP_IP_ROBOT, UDP_PORT_ROBOT)

# ============================
# SISTEMA DE FICHEIROS (Thread separada — só I/O)
# ============================
save_queue = Queue()

def file_writer():
    """Escreve em disco de forma assíncrona. Nunca bloqueia outras threads."""
    while True:
        fname, lines = save_queue.get()
        try:
            dirpath = os.path.dirname(fname)
            if dirpath:
                os.makedirs(dirpath, exist_ok=True)
            with open(fname, "a") as f:
                f.writelines(lines)
        except Exception as e:
            print(f"Erro a gravar ficheiro {fname}: {e}")
        save_queue.task_done()

threading.Thread(target=file_writer, daemon=True).start()

# ============================
# PARSING DE DADOS (STRING -> DICT/NUMPY)
# ============================
def parse_frame_data(frame_str):
    x, y, t = None, None, None
    last_node_ind = 0
    target_node_ind = 0
    gi, gx, gy, gt = [], [], [], []
    conns = np.zeros((163, 6))

    for token in frame_str.split(";"):
        # print(token)
        token = token.strip()
        if not token:
            continue
        parts = token.split()
        if len(parts) != 2:
            continue
        k = parts[0]
        try:
            v = float(parts[1])
        except ValueError:
            continue

        # coordenadas robot enviadas com: odo_x VAL; odo_y VAL; ...
        if k == "xe":
            x = float(v); continue
        if k == "ye":
            y = float(v); continue
        if k == "te":
            t = float(v); continue
        
        # pontos do grafo enviados com: graph_x VAL; graph_y VAL; ...
        if k == "graph_ind":
            gi.append(float(v)); continue
        if k == "graph_x":
            gx.append(float(v)); continue
        if k == "graph_y":
            gy.append(float(v)); continue
        if k == "graph_t":
            gt.append(float(v)); continue
        
        # indice do ultimo node
        if k == "last_node_ind":
            last_node_ind = int(v); continue
        if k == "target_node_ind":
            target_node_ind = int(v); continue
        
        # 6 connections enviadas com: graph_conn_I_J BOOL;
        # é preciso que o label tenha 16 caracteres: "graph_conn_000_0"
        if k.startswith("graph_conn") and len(k) == 16:
            try:
                row, col = int(k[11:14]), int(k[15])
                conns[row, col] = int(v)
            except ValueError:
                pass
            continue

    return {
        "x":       x,
        "y":       y,
        "t":       t,
        "gx":      gx,
        "gy":      gy,
        "gt":      gt,
        "graph_connections": conns,
        "last": last_node_ind,
        "target": target_node_ind
    }

# ============================
# QUEUES DO PIPELINE
# ============================
raw_packet_queue = Queue()          # bytes: Rede → Parser
gui_data_queue   = Queue()          # dicts: Parser → Saver/Dispatcher
render_queue     = Queue(maxsize=1) # dicts: Saver → GUI (tamanho 1 = sempre o mais recente)

# ============================
# THREAD DE REDE
# ============================
def network_listener_thread():
    print(">>> Thread de Rede Iniciada.")
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            if data:
                raw_packet_queue.put(data)
        except OSError as e:
            print(f"Erro Socket: {e}")
            time.sleep(1)

# ============================
# THREAD DE PARSING
# ============================
def parser_worker_thread():
    print(">>> Thread de Parsing Iniciada.")
    buffer = ""
    LDS_START = "lds_start"
    LDS_END   = "lds_end"

    while True:
        try:
            # 1. BLOQUEIA até chegar pelo menos UM pacote. (Zero CPU gasto enquanto espera)
            data = raw_packet_queue.get()
            buffer += data.decode("utf-8", errors="ignore")

            # 2. DRENA IMEDIATAMENTE tudo o resto que já esteja na fila (Batching)
            while True:
                try:
                    extra_data = raw_packet_queue.get_nowait()
                    buffer += extra_data.decode("utf-8", errors="ignore")
                except Empty:
                    break

            # Proteção contra lag extremo
            if len(buffer) > 200000:
                print("!!! Lag detetado: a truncar buffer para manter tempo real.")
                last_start = buffer.rfind(LDS_START)
                buffer = buffer[last_start:] if last_start != -1 else ""

            # 3. EXTRAI TODOS os frames possíveis do buffer atual
            while True:
                start_idx = buffer.find(LDS_START)
                
                if start_idx == -1:
                    buffer = ""  # Só há lixo no buffer, limpa tudo e sai do loop
                    break
                
                # Corta imediatamente qualquer lixo que esteja ANTES do lds_start
                if start_idx > 0:
                    buffer = buffer[start_idx:]
                    # Como cortámos o início, o lds_start passa a estar no índice 0
                    start_idx = 0

                end_idx = buffer.find(LDS_END, len(LDS_START))
                if end_idx == -1:
                    break  # O frame ainda não acabou de chegar. Sai e vai ler mais pacotes.

                # Verifica frame aninhado/corrompido (um start dentro de outro start)
                next_start = buffer.find(LDS_START, len(LDS_START))
                if next_start != -1 and next_start < end_idx:
                    buffer = buffer[next_start:]
                    continue

                # Corta o frame perfeito
                end_cut = end_idx + len(LDS_END)
                lap_str = buffer[:end_cut]
                buffer = buffer[end_cut:]  # O que sobra (se sobrar) fica para a próxima ronda

                if lap_str.strip():
                    processed = parse_frame_data(lap_str)
                    if processed is not None:
                        gui_data_queue.put(processed)

        except Exception as e:
            print(f"Erro no parser: {e}")

# ============================
# THREAD SAVER / DISPATCHER  ← NOVO
# ============================
# Esta thread é a única responsável por gravar para disco.
# Vê TODOS os frames → nenhum se perde.
# Depois de gravar, mete o frame mais recente na render_queue (size=1).

# Flags de gravação (modificáveis pela GUI ou consola)
saveEvo              = 1

# Contadores (acedidos apenas pelo saver_thread → sem race conditions)
_fileCount            = 0
_calib_points_counter = 0

def saver_dispatcher_thread():
    global saveEvo, saveTimes, saveCalib, saveDelta2A, fileTimes, calib_file, num_calib_points, delta2A_file, last_delta2A_time
    global _fileCount, _calib_points_counter

    print(">>> Thread de Gravação/Dispatcher Iniciada.")

    while True:
        try:
            # Bloqueia até haver dados — lê TODOS os frames sem saltar nenhum
            data = gui_data_queue.get(timeout=0.5)  # ← timeout em vez de bloquear para sempre
        
            # ── Gravar Evolução ────────────────────────────────────────────
            if saveEvo:
                print("Entrou aqui")
                lines_to_save = []
                lines_to_save.append(f"x:{data['x']}\n")
                lines_to_save.append(f"y:{data['y']}\n")
                lines_to_save.append(f"t:{data['t']}\n")
                lines_to_save.append(f"last:{data['last']}\n")
                lines_to_save.append(f"target:{data['target']}\n")

                for i in range(len(data["gx"])):
                    lines_to_save.append(f"graph_point-{i}:{data['gx'][i]},{data['gy'][i]}\n")
                
                for i in range(len(data["gt"])):
                    lines_to_save.append(f"graph_theta-{i}:{data['gt'][i]}\n")

                for i in range(len(data["graph_connections"])):
                    m = data["graph_connections"][i]
                    m0, m1, m2, m3, m4, m5 = m
                    lines_to_save.append(f"connections-{i}:{m0},{m1},{m2},{m3},{m4},{m5}\n")

                print("Num linhas = ", len(lines_to_save))

                save_queue.put((f"data_evolution/data_{_fileCount:06d}.txt", lines_to_save))
                _fileCount += 1

            # ── Enviar para render_queue (SEMPRE o frame mais recente) ─────
            # Se a GUI ainda não consumiu o anterior, descartamos o antigo.
            try:
                render_queue.get_nowait()   # drena o frame antigo, se existir
            except Empty:
                pass
            render_queue.put_nowait(data)   # coloca o frame novo

        except Empty:
            continue  # sem dados, volta ao início sem congelar
        except Exception as e:
            print(f"Erro no saver/dispatcher: {e}")
            continue

threading.Thread(target=network_listener_thread,  daemon=True).start()
threading.Thread(target=parser_worker_thread,      daemon=True).start()
threading.Thread(target=saver_dispatcher_thread,   daemon=True).start()

# ============================
# JANELA PYQTGRAPH
# ============================
def on_send():
    msg = input_cmd.text()
    if msg:
        print("Enviando comando:", msg)
        udp_send.send(msg)
        input_cmd.clear()

app = pg.mkQApp("Lidar Viewer")

main_win = QWidget()
main_win.setWindowTitle("Lidar Viewer v4.2")
main_win.resize(1000, 800)
main_layout = QVBoxLayout(main_win)
main_layout.setContentsMargins(0, 0, 0, 0)

win = pg.GraphicsLayoutWidget()
main_layout.addWidget(win)

input_cmd = QLineEdit(main_win)
input_cmd.setPlaceholderText("Comando para o robot...")
input_cmd.setFixedHeight(50)
input_cmd.setStyleSheet("""
    QLineEdit {
        border: 2px solid #555;
        border-radius: 10px;
        padding: 5px;
        background: #222;
        color: white;
        font-size: 14px;
    }
    QLineEdit:focus { border: 2px solid #0078d7; }
""")
input_cmd.returnPressed.connect(on_send)
main_layout.addWidget(input_cmd)
main_win.show()

plot = win.addPlot()
plot.setRange(xRange=(-2, 2), yRange=(-2, 2))
plot.showGrid(x=True, y=True)
plot.setLabel("bottom", "X")
plot.setLabel("left",   "Y")
plot.setAspectLocked(True)

scatter = pg.ScatterPlotItem(size=5, pen=None, brush="w")
plot.addItem(scatter)

pose_text = pg.TextItem(
    html='<div style="background-color: rgba(0,0,0,150); padding: 5px; color: yellow; border: 1px solid white;"><b>Pose:</b> N/A</div>',
    anchor=(0, 0)
)
pose_text.setPos(-1.9, 1.9)
plot.addItem(pose_text)

MAX_LINES    = 50
line_items   = []
text_items_i = []

for _ in range(MAX_LINES):
    line = pg.PlotDataItem(pen=pg.mkPen("r", width=2))
    line.hide()
    plot.addItem(line)
    line_items.append(line)

    txt_i = pg.TextItem(color="w", anchor=(0, 1))
    txt_i.hide()
    plot.addItem(txt_i)
    text_items_i.append(txt_i)

# ============================
# FUNÇÃO UPDATE (GUI) — SEM I/O, SEM LOOPS LONGOS
# ============================
# A GUI nunca faz I/O — só lê da render_queue e redesenha.
# Se não houver frame novo, simplesmente não apaga o ecrã (frame anterior mantém-se visível).

def update():
    # Tenta ler o frame mais recente. Se não houver nenhum novo, sai sem apagar nada.
    try:
        data = render_queue.get_nowait()
    except Empty:
        return  # Nenhum frame novo → pontos e linhas do frame anterior ficam no ecrã

timer = QTimer()
timer.timeout.connect(update)
timer.start(10)   # 100 Hz — ajustar se necessário

if __name__ == "__main__":
    try:
        app.exec()
    except KeyboardInterrupt:
        print("A fechar...")