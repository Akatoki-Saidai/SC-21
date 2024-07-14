import bluetooth

def receive_data(server_addr):
    # Bluetoothソケットを作成
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    # 任意のポートを使用
    port = 1

    # サーバーに接続
    sock.connect((server_addr, port))
    print("Connected to server at", server_addr)

    try:
        # データを受信
        data = sock.recv(1024)
        print("Received data: ", data.decode("utf-8"))
    except Exception as e:
        print("Error:", e)

    # ソケットを閉じる
    sock.close()

# サーバーのBluetoothアドレス (変更する必要があります)
server_address = "XX:XX:XX:XX:XX:XX"

while True:
    receive_data(server_address)
