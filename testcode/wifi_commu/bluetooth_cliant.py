import bluetooth
import time

def send_data(data, addr, timeout=60):
    # Bluetoothソケットを作成
    server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    # 任意のポートを使用
    port = 1
    # サーバーソケットをバインド
    server_sock.bind(("", port))
    server_sock.listen(1)

    # タイムアウトを設定
    server_sock.settimeout(timeout)

    try:
        # クライアントからの接続を待機
        print("Waiting for connection on RFCOMM channel %d" % port)
        client_sock, client_info = server_sock.accept()
        print("Accepted connection from ", client_info)

        try:
            # データを送信
            client_sock.send(data)
            print("Sent data: ", data)
        except Exception as e:
            print("Error sending data:", e)
        finally:
            # クライアントソケットを閉じる
            client_sock.close()
    except bluetooth.BluetoothError as e:
        print("Bluetooth error: No connection within the timeout period")
    except Exception as e:
        print("Unexpected error occured in bluetooth connection:", e)
    finally:
        # サーバーソケットを閉じる
        server_sock.close()

# 送信したいデータ
data_to_send = "Hello, Bluetooth!"
# クライアントのアドレス (省略可能、未使用)
client_addr = None
# タイムアウト時間 (秒)
timeout_duration = 60

send_data(data_to_send, client_addr, timeout_duration)
